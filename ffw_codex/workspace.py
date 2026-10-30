#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Jiho Park

import hashlib
from pathlib import Path
import shutil

from codex_host import utc_now, write_json


HERE = Path(__file__).resolve().parent
# Instruction file names
AGENTS_FILES = ('AGENTS.md', 'AGENTS.override.md')
CAMERAS = ('head', 'wrist_left', 'wrist_right')
INITIAL_PROMPT = 'Task: {task}\n\nThe episode has not started. Call robot_start.'


def sha256(data):
    if isinstance(data, str):
        data = data.encode('utf-8')
    return hashlib.sha256(data).hexdigest()


def stray_agents_files(agent_dir, codex_home):
    """Find additional AGENTS files that could affect the run"""
    agent_dir = Path(agent_dir).resolve()
    candidates = [Path(codex_home) / name for name in AGENTS_FILES]
    candidates.append(agent_dir / 'AGENTS.override.md')
    for directory in agent_dir.parents:
        candidates += [directory / name for name in AGENTS_FILES]
        if (directory / '.git').exists():
            break
    return [str(path) for path in candidates if path.exists()]


def prepare_workspace(run_dir, codex_home, template_dir):
    """Create the agent workspace; fail if it already exists"""
    run_dir = Path(run_dir).resolve()
    agent_dir = run_dir / 'agent'
    public_dir = run_dir / 'public'
    # Create permission-profile directories before Codex starts
    (public_dir / 'observations').mkdir(parents=True, exist_ok=True)
    shutil.copytree(template_dir, agent_dir)
    (agent_dir / 'notes').mkdir(exist_ok=True)
    (agent_dir / 'scratch').mkdir()

    stray = stray_agents_files(agent_dir, codex_home)
    if stray:
        raise RuntimeError(f'Codex would also load {stray}; move or remove them')

    write_json(agent_dir / 'workspace.json', {
        'python': 'python3',
        'run_dir': str(run_dir),
        'agent_dir': str(agent_dir),
        'observations_dir': str(public_dir / 'observations'),
        'observation_files': [f'{camera}.jpg' for camera in CAMERAS] + ['observation.json'],
        'history_path': str(public_dir / 'history.jsonl'),
        'notes_path': str(agent_dir / 'notes' / 'NOTES.md'),
        'scratch_dir': str(agent_dir / 'scratch'),
        'robot_contract': str(agent_dir / 'context' / 'robot_contract.md'),
        'task_context': str(agent_dir / 'context' / 'task_context.md'),
        'readable': [str(public_dir), str(agent_dir)],
        'writable': [str(agent_dir / 'notes'), str(agent_dir / 'scratch')],
    })
    return agent_dir


def build_instructions(agent_dir, task):
    """Build developer instructions and the initial prompt"""
    developer = (Path(agent_dir) / 'context' / 'task_context.md').read_text(encoding='utf-8')
    return developer, INITIAL_PROMPT.format(task=task)


def write_run_record(run_dir, run_id, task, cfg, config_path, developer, prompt):
    """Record run settings and input hashes in run.json"""
    run_dir = Path(run_dir).resolve()
    agent_dir = run_dir / 'agent'
    record = {
        'run_id': run_id,
        'created_at': utc_now(),
        'task': task,
        'model': cfg['model']['id'],
        'effort': cfg['model']['reasoning_effort'],
        'episode': dict(cfg['episode']),
        'config_path': str(Path(config_path).resolve()),
        'config_sha256': sha256(Path(config_path).read_bytes()),
        'agent_files_sha256': {
            str(path.relative_to(agent_dir)): sha256(path.read_bytes())
            for path in sorted(agent_dir.rglob('*'))
            if path.is_file()
        },
        'instructions': {
            'loaded_by_codex': 'agent/AGENTS.md',
            'developer_instructions_source': 'agent/context/task_context.md',
            'developer_instructions_sha256': sha256(developer),
            'initial_prompt': prompt,
            'initial_prompt_sha256': sha256(prompt),
        },
    }
    write_json(run_dir / 'run.json', record)
    return record
