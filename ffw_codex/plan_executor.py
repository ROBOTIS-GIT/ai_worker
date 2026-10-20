"""Proposed bounded sequential plans; no ROS or model calls here."""
from copy import deepcopy
import math

from codex_host import UncertainExecution, write_json

PLAN_TOOL = 'robot_execute_plan'
ALLOWED = {'robot_move', 'robot_gripper', 'robot_head', 'robot_lift'}


def max_steps(cfg):
    value = cfg.get('plan', {}).get('max_steps', 5)
    if type(value) is not int or not 1 <= value <= 10:
        raise ValueError('plan.max_steps must be an integer in [1, 10]')
    return value


def build_plan_spec(specs, cfg):
    variants = []
    for spec in specs:
        if spec['name'] not in ALLOWED:
            continue
        properties = deepcopy(spec['inputSchema']['properties'])
        properties.pop('request_id')
        reason = properties.pop('reason')
        variants.append({
            'type': 'object',
            'properties': {
                'tool': {'type': 'string', 'enum': [spec['name']]},
                'args': {'type': 'object', 'properties': properties,
                         'required': list(properties), 'additionalProperties': False},
                'reason': reason,
            },
            'required': ['tool', 'args', 'reason'],
            'additionalProperties': False,
        })
    return {
        'type': 'function', 'name': PLAN_TOOL,
        'description': (
            'Execute a short preplanned sequence, strictly one action at a time. '
            'The host captures observations and checks state after each action. '
            'It does not visually interpret intermediate images. Continue only '
            'after arrived with a valid observation. Head, lift, and every '
            'gripper command must be the final step. Use only when intermediate '
            'visual reasoning is unnecessary. Plan completion is not task completion. '
            'The first response line is a JSON packet, followed by camera data: URLs; '
            'pass each data: line to image().'
        ),
        'inputSchema': {
            'type': 'object',
            'properties': {
                'request_id': {'type': 'string'},
                'reason': {'type': 'string'},
                'steps': {'type': 'array', 'minItems': 1,
                          'maxItems': max_steps(cfg), 'items': {'anyOf': variants}},
            },
            'required': ['request_id', 'reason', 'steps'],
            'additionalProperties': False,
        },
    }


def validate_plan(arguments, specs, cfg):
    """Check all static arguments before the first command is sent."""
    if not isinstance(arguments, dict) or set(arguments) != {'request_id', 'reason', 'steps'}:
        raise ValueError('Expected request_id, reason, steps.')
    for key in ('request_id', 'reason'):
        if not isinstance(arguments[key], str) or not arguments[key].strip():
            raise ValueError(f'{key} must be a nonempty string.')
    steps = arguments['steps']
    if not isinstance(steps, list) or not 1 <= len(steps) <= max_steps(cfg):
        raise ValueError('Invalid number of steps.')
    by_name = {spec['name']: spec for spec in specs}
    for i, step in enumerate(steps, 1):
        if not isinstance(step, dict) or set(step) != {'tool', 'args', 'reason'}:
            raise ValueError(f'Step {i}: expected tool, args, reason.')
        name = step['tool']
        if not isinstance(name, str) or name not in ALLOWED:
            raise ValueError(f'Step {i}: unsupported tool.')
        if not isinstance(step['reason'], str) or not step['reason'].strip():
            raise ValueError(f'Step {i}: reason is required.')
        props = by_name[name]['inputSchema']['properties']
        expected = set(props) - {'request_id', 'reason'}
        args = step['args']
        if not isinstance(args, dict) or set(args) != expected:
            raise ValueError(f'Step {i}: invalid argument keys.')
        for key, value in args.items():
            schema = props[key]
            if schema['type'] == 'number':
                if type(value) not in (int, float):
                    raise ValueError(f'Step {i}: {key} must be a number.')
                try:
                    finite = math.isfinite(value)
                except OverflowError:
                    finite = False
                if not finite:
                    raise ValueError(f'Step {i}: {key} must be finite.')
                if value < schema.get('minimum', -math.inf) or value > schema.get('maximum', math.inf):
                    raise ValueError(f'Step {i}: {key} is out of range.')
            elif schema['type'] == 'string':
                if not isinstance(value, str) or value not in schema['enum']:
                    raise ValueError(f'Step {i}: invalid {key}.')
            else:
                raise ValueError(f'Step {i}: unsupported schema for {key}.')
        checkpoint = name in {'robot_head', 'robot_lift', 'robot_gripper'}
        if checkpoint and i != len(steps):
            raise ValueError(f'Step {i}: this action must be the final step.')
    return deepcopy(steps)


def execute_plan(rollout, index, arguments):
    try:
        steps = validate_plan(arguments, rollout.specs, rollout.cfg)
    except ValueError as exc:
        return rollout._reject(index, PLAN_TOOL, 'invalid_plan', str(exc))

    with rollout.lock:
        source = dict(rollout.context or {})
    if not source.get('movement_allowed'):
        return rollout._reject(index, PLAN_TOOL, 'observe_required', 'Call robot_observe.')
    if arguments['request_id'] != source.get('request_id'):
        return rollout._reject(index, PLAN_TOOL, 'stale_request_id', 'Plan observation is stale.')

    summary = {
        'plan_id': f'plan_{index:04d}',
        'source_observation_id': source.get('observation_id'),
        'source_request_id': arguments['request_id'],
        'steps': [], 'remaining_steps': steps,
        'running_step': None, 'stop_reason': None,
    }
    journal = rollout.private_dir / f'plan_{index:04d}_state.json'
    last = None
    images = ()
    seen = None     # last step packet that returned an observation, and its images
    any_sent = False
    status = 'plan_interrupted'

    def save_journal():
        write_json(journal, {'arguments': arguments, 'status': status, **summary})

    for step_index, step in enumerate(steps, 1):
        rollout.tick()
        with rollout.lock:
            finished = rollout.finish_reason
            context = dict(rollout.context or {})
        if finished is not None:
            summary['stop_reason'] = finished
            break
        if not context.get('movement_allowed'):
            summary['stop_reason'] = 'observe_required'
            break

        summary['running_step'] = step_index
        save_journal()
        try:
            response = rollout._act(
                index, step['tool'],
                {**step['args'], 'reason': step['reason'],
                 'request_id': context['request_id']},
                internal=True, step_index=step_index,
            )
        except Exception as exc:
            # Never retry a step whose execution state is uncertain.
            rollout.halt('uncertain_execution')
            status = 'plan_uncertain'
            summary['stop_reason'] = f'{type(exc).__name__}: {exc}'
            save_journal()
            raise UncertainExecution(summary['stop_reason']) from exc

        last, images = response['packet'], response['images']
        if last['observation'] is not None:
            seen = (last, images)
        any_sent = any_sent or last['action_executed']
        summary['steps'].append({
            'step_index': step_index, 'tool': step['tool'],
            'args': step['args'], 'reason': step['reason'], 'packet': last,
        })
        summary['remaining_steps'] = steps[step_index:]
        summary['running_step'] = None
        can_continue = (
            last['status'] == 'arrived'
            and last['observation'] is not None
            and last['action_context']['movement_allowed']
        )
        if not can_continue:
            summary['stop_reason'] = last['rejection'] or last['status']
            save_journal()
            break
        save_journal()
    else:
        status = 'plan_completed'

    reason = summary['stop_reason'] or 'All planned actions arrived; inspect the final observation.'
    save_journal()
    rollout._record_private(index, PLAN_TOOL, arguments, {
        'status': status, 'reason': reason, 'command_sent': any_sent,
        'data': summary, 'images': [],
    })
    observation = last['observation'] if last else None
    note = last.get('observation_note') if last else None
    if (observation is None and seen is not None
            and last['action_context']['movement_allowed']):
        # A rejected step ran nothing; the earlier step's observation is still current.
        observation, images = seen[0]['observation'], seen[1]
        note = (f'Step {len(summary["steps"])} was rejected without running; this is the '
                'observation after the last step that ran, and it is still current.')
    return rollout._reply(
        index, PLAN_TOOL, action_executed=any_sent,
        status=status, reason=reason, result=summary,
        observation=observation, observation_note=note, images=images,
    )
