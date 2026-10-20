"""
Run one episode: Codex app-server as the policy, the robot behind
dynamic tools, the model's task_complete claim judged later by a person.

Run inside the ai_worker container, in a shell with the ROS environment loaded,
while the physical SG2 bringup and Cyclo are running:
    python3 run_codex.py
    python3 run_codex.py --task "your task" --run-id first_try

The first Ctrl+C stops new actions and lets a running robot tool finish.
The summary for judging is runs/<run_id>/result.json
"""

import argparse
from datetime import datetime, timezone
from pathlib import Path
import sys

import yaml

from codex_host import CodexHost, HostConfig, write_json
from rollout import RobotRollout
from transport import AppServerTransport
from workspace import build_instructions, prepare_workspace, write_run_record


HERE = Path(__file__).resolve().parent
# End reasons where the host itself failed rather than the episode ending
HOST_FAILURES = frozenset({'host_error', 'transport_closed'})


def host_config(cfg, developer, prompt, **overrides):
    """HostConfig from config.yaml; overrides are for checks (poll_s and so on)."""
    return HostConfig(
        developer_instructions=developer,
        initial_prompt=prompt,
        model=cfg['model']['id'],
        effort=cfg['model']['reasoning_effort'],
        codex=cfg['codex']['bin'],
        codex_home=Path(cfg['codex']['home']),
        permission_profile=cfg['codex']['permission_profile'],
        token_limit=cfg['episode'].get('max_total_tokens'),
        **overrides,
    )


def estimate_cost(total, prices):
    """USD estimate from tokenUsage.total; ignores any long-prompt surcharge"""
    if not total or not prices:
        return None
    cached = total.get('cachedInputTokens', 0)
    written = total.get('cacheWriteInputTokens', 0)
    uncached = total.get('inputTokens', 0) - cached - written
    dollars = (uncached * prices['input'] + cached * prices['cached_input']
               + written * prices['cache_write']
               + total.get('outputTokens', 0) * prices['output']) / 1e6
    return round(dollars, 3)


def summarize(record, outcome, cfg):
    """result.json: what a person needs to judge the episode"""
    rollout = outcome.get('rollout') or {}
    total = (outcome.get('token_usage') or {}).get('total') or {}
    cached = total.get('cachedInputTokens', 0)
    written = total.get('cacheWriteInputTokens', 0)
    return {
        'run_id': record['run_id'],
        'task': record['task'],
        'model': record['model'],
        'effort': record['effort'],
        'end_reason': outcome.get('end_reason'),
        'end_detail': outcome.get('end_detail'),
        'finish_reason': rollout.get('finish_reason'),
        'claim': rollout.get('claim'),
        'actions_executed': rollout.get('actions_executed'),
        'observations': rollout.get('observations'),
        'robot_tool_calls': rollout.get('tool_calls'),
        'rejections': rollout.get('rejections'),
        'turns': outcome.get('turns'),
        'continuations': outcome.get('continuations'),
        'elapsed_s': outcome.get('elapsed_s'),
        'tokens': {
            'total': total.get('totalTokens', 0),
            'uncached_input': total.get('inputTokens', 0) - cached - written,
            'cached_input': cached,
            'cache_write': written,
            'output': total.get('outputTokens', 0),
            'reasoning': total.get('reasoningOutputTokens', 0),
        },
        'cost_usd_estimate': estimate_cost(total, cfg['model'].get('price_per_million')),
        'verdict': None,        # filled in by a person: "success" or "failure"
        'verdict_note': None,
    }


def run_episode(cfg, config_path, task, run_dir, guard,
                transport_factory=AppServerTransport, host_overrides=None):
    """Prepare the workspace, run one Codex thread on guard and write result.json."""
    run_dir = Path(run_dir).resolve()
    agent_dir = prepare_workspace(
        run_dir,
        cfg['codex']['home'],
        template_dir=HERE / 'agent_template',
        )
    developer, prompt = build_instructions(agent_dir, task)
    record = write_run_record(run_dir, run_dir.name, task, cfg, config_path, developer, prompt)
    rollout = RobotRollout(guard, cfg, run_dir, task,
                           max_actions=cfg['episode']['max_actions'],
                           max_seconds=cfg['episode']['max_seconds'])
    host = CodexHost(host_config(cfg, developer, prompt, **(host_overrides or {})),
                     rollout, run_dir, transport_factory=transport_factory)
    result = summarize(record, host.run(), cfg)
    write_json(run_dir / 'result.json', result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--config', default=str(HERE / 'config.yaml'))
    parser.add_argument('--task', required=True)
    parser.add_argument('--run-id', default=None, help='folder name under runs/ (default: UTC time)')
    args = parser.parse_args()

    import rclpy
    from observation_guard import ObservationGuard
    from robot_io import RobotIO
    from robot_tools import RobotTools

    with open(args.config, encoding='utf-8') as stream:
        cfg = yaml.safe_load(stream)
    run_id = args.run_id or datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
    run_dir = HERE / cfg['run']['output_root'] / run_id

    rclpy.init()
    io = None
    try:
        io = RobotIO(cfg)
        io.start()
        io.wait_ready()
        guard = ObservationGuard(RobotTools(io, cfg), cfg)
        result = run_episode(cfg, args.config, args.task, run_dir, guard)
    finally:
        if io is not None:
            io.close()
        rclpy.try_shutdown()

    claim = result['claim']
    print(f'\nend: {result["end_reason"]} / {result["finish_reason"]}, '
          f'actions {result["actions_executed"]}, {result["elapsed_s"]} s, '
          f'tokens {result["tokens"]["total"]}, ~${result["cost_usd_estimate"]}')
    print(f'claim: {claim["reason"] if claim else None}')
    print(f'record: {run_dir}  (fill in verdict in result.json)')
    return 1 if result['end_reason'] in HOST_FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
