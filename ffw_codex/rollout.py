"""
Robot rollout: the Rollout that CodexHost calls for dynamic robot tools.

It turns one tool call into one ObservationGuard dispatch, saves what the model
may see under public/, keeps full results under rollout/, and answers with a
single text packet plus the new camera images. Robot control itself stays in
RobotTools and ObservationGuard.
"""

import base64
from copy import deepcopy
import json
from pathlib import Path
import threading
import time
from functools import partial

from codex_host import UncertainExecution, utc_now, write_json
from plan_executor import PLAN_TOOL, build_plan_spec, execute_plan


ACTION_TOOLS = {
    # dynamic tool name -> ObservationGuard/RobotTools tool name
    'robot_move': 'move',
    'robot_gripper': 'set_gripper',
    'robot_head': 'move_head',
    'robot_lift': 'move_lift',
    'robot_base': 'move_base',
}
ACTION_ARGUMENTS = {
    'robot_move': ('arm', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'),
    'robot_gripper': ('arm', 'value'),
    'robot_head': ('head_joint1', 'head_joint2'),
    'robot_lift': ('position_m',),
    'robot_base': ('vx', 'vy', 'wz', 'duration_s'),
}
RETURN_FORMAT = (
    ' Returns a string. The first line is a JSON packet; action_executed says '
    'whether a robot command ran. If packet.observation is not null, each '
    'following line is one camera image as a data:image/jpeg;base64 URL in the '
    'order head, wrist_left, wrist_right: pass each line that starts with '
    '"data:" to image(). The same files are listed in observation.images and '
    'can be opened with view_image.'
)


def number(description, low=None, high=None):
    schema = {'type': 'number', 'description': description}
    if low is not None:
        schema['minimum'] = low
    if high is not None:
        schema['maximum'] = high
    return schema


def action_schema(properties):
    """Common request_id and reason, then the action's own arguments."""
    properties = {
        'request_id': {'type': 'string',
                       'description': 'action_context.request_id of the latest observation.'},
        'reason': {'type': 'string',
                   'description': 'The visible evidence and the purpose of this action.'},
        **properties,
    }
    return {'type': 'object', 'properties': properties,
            'required': list(properties), 'additionalProperties': False}


def build_tool_specs(cfg):
    limits = cfg['limits']
    arms = list(cfg['frames']['ee'])
    base = cfg['frames']['base']
    arm = {'type': 'string', 'enum': arms}
    no_arguments = {'type': 'object', 'properties': {}, 'required': [],
                    'additionalProperties': False}
    head1, head2 = limits['head_joint1_rad'], limits['head_joint2_rad']
    lift = limits['lift_joint_m']

    def spec(name, description, schema):
        return {'type': 'function', 'name': name, 'description': description + RETURN_FORMAT,
                'inputSchema': schema}

    specs = [
        spec('robot_start',
             'Start the episode: capture the first synchronized observation of the three '
             'cameras and the robot state. Call once, before any other robot tool.',
             no_arguments),
        spec('robot_observe',
             'Capture a new synchronized observation. Required when movement is not '
             'allowed, for example after an expired or changed observation.',
             no_arguments),
        spec('robot_move',
             f'Move one end effector to an absolute pose in {base} '
             '(meters, radians). RPY specifies end-effector orientation '
             'as Rz(yaw) * Ry(pitch) * Rx(roll), not wrist joint angles '
             'or rotation increments. A target may change both position '
             'and orientation. '
             f'At most {limits["max_step_m"]} m and '
             f'{limits["max_rotation_rad"]} rad from the current measured '
             'pose. The rotation limit applies to the total relative '
             'quaternion rotation, not separately to each Euler angle. '
             'The gripper is unchanged. status stopped: the hand came '
             'to rest before the target, for example against an object.',
             action_schema({
                 'arm': arm,
                 **{
                     axis: number(f'Absolute {base} {axis} in meters.')
                     for axis in 'xyz'
                 },
                 **{
                     angle: number(
                         f'Absolute end-effector {angle} in radians. '
                         'Not an individual wrist joint angle or '
                         'a rotation increment. Zero requests zero '
                         'for this angle; it does not preserve the '
                         'current value. To preserve orientation, '
                         'reuse the last issued move target rpy for that arm, '
                         'not measured drift. Before its first move, use the '
                         'measured rpy as the initial reference. In a plan, '
                         'use the previous target rpy for that arm.'
                     )
                     for angle in ('roll', 'pitch', 'yaw')
                 },
             })),
        spec('robot_gripper',
             'Set one gripper: 0 is open, 1 is closed. status stopped: the fingers came to '
             'rest before the target, for example on an object. Neither a closed nor a '
             'stopped gripper proves a grasp. Opening while holding an object releases it; '
             'do not open unless the latest images clearly show the intended support '
             'directly below the object and the object is low enough to be supported.',
             action_schema({'arm': arm, 'value': number('0 open .. 1 closed.', 0.0, 1.0)})),
        spec('robot_head',
             'Move both head joints to absolute angles in radians. head_joint1 tilts about '
             'the local Y axis, head_joint2 pans about the local Z axis.',
             action_schema({
                 'head_joint1': number('Absolute head_joint1 in radians.', *head1),
                 'head_joint2': number('Absolute head_joint2 in radians.', *head2),
             })),
        spec('task_complete',
             'Declare that the task in the first message is done. Call it only when a new '
             'observation shows the evidence, with that observation\'s request_id. This ends '
             'the episode: no further robot tools run. The host records a final observation. '
             'There is no way to give up; if the task is not done, keep working.',
             {'type': 'object',
              'properties': {
                  'request_id': {'type': 'string',
                                 'description': 'action_context.request_id of the latest '
                                                'observation.'},
                  'reason': {'type': 'string',
                             'description': 'The visible evidence that the task is done.'},
              },
              'required': ['request_id', 'reason'], 'additionalProperties': False}),
        spec('robot_lift',
             'Move the lift joint to an absolute displacement in meters. 0 is the highest '
             'position; negative values lower the torso, which carries both arm bases and '
             f'the head. The arm controller keeps each end effector at its last commanded '
             f'{base} pose, so the arms re-bend to compensate and the hands stay where they '
             'are; they may lag by about 1 cm while the lift is moving. Lowering the lift '
             'moves the region the arms can reach downward, and raising it moves that region '
             'upward. The head camera moves with the torso, so its view changes.',
             action_schema({'position_m': number('Absolute lift_joint in meters.', *lift)})),
    ]

    if 'base' in cfg:
        motion = cfg['base']
        lin, ang = motion['max_lin_mps'], motion['max_ang_rps']
        deadband_lin, deadband_ang = motion['deadband_mps'], motion['deadband_rps']
        specs.append(
            spec('robot_base',
                 'Drive the mobile base at a body velocity in base_link for duration_s seconds, '
                 'then stop: vx forward and vy left in m/s, wz counterclockwise in rad/s. '
                 f'Each non-zero axis must be at least {deadband_lin} m/s ({deadband_ang} rad/s '
                 f'for wz); hypot(vx, vy) at most {lin} m/s, |wz| at most {ang} rad/s, '
                 f'hypot(vx, vy) * duration_s at most {motion["max_step_m"]} m and '
                 f'|wz| * duration_s at most {motion["max_step_rad"]} rad. Both hands must be near '
                 'the body. arrived means the velocity profile ran and the base came to rest, not '
                 'that a distance was reached; result.motion.odom_measured gives the measured dx, '
                 'dy in meters and dyaw in radians, from wheel rotation: a blocked base with '
                 'slipping wheels still reports motion, so confirm in the new images that the '
                 'scene moved and the path is clear. The first command after a direction change '
                 'may travel substantially less because of steering alignment and the velocity '
                 'ramps.',
                 action_schema({
                     'vx': number('Forward velocity in m/s; 0 or a magnitude of at least '
                                  f'{deadband_lin}.', -lin, lin),
                     'vy': number('Leftward velocity in m/s; 0 or a magnitude of at least '
                                  f'{deadband_lin}.', -lin, lin),
                     'wz': number('Counterclockwise yaw rate in rad/s; 0 or a magnitude of at '
                                  f'least {deadband_ang}.', -ang, ang),
                     'duration_s': number('Command time in seconds, including speeding up and '
                                          'slowing down.', 0.0, motion['max_duration_s']),
                 })))

    return specs + [build_plan_spec(specs, cfg)]


def rounded(value, digits=4):
    return round(float(value), digits)


class RobotRollout:
    """Serves the robot dynamic tools for one episode.

    handle_tool_call runs in CodexHost's worker thread; tick, finished, halt,
    next_call and outcome run in its main thread. Shared state is guarded by a
    lock that is never held while the robot moves.
    """

    def __init__(self, guard, cfg, run_dir, task, max_actions, max_seconds):
        self.guard = guard
        self.cfg = cfg
        self.run_dir = Path(run_dir).resolve()
        self.public_dir = self.run_dir / 'public'
        self.private_dir = self.run_dir / 'rollout'
        for directory in (self.public_dir / 'observations', self.private_dir):
            directory.mkdir(parents=True, exist_ok=True)
        self.history_path = self.public_dir / 'history.jsonl'
        self.task = task
        self.max_actions = max_actions
        self.max_seconds = max_seconds
        self.specs = build_tool_specs(cfg)
        self.lock = threading.Lock()
        self.started_at = None
        self.context = None
        self.call_count = 0
        self.observation_count = 0
        self.actions_executed = 0
        self.rejections = {}
        self.finish_reason = None
        self.claim = None   # the model's task_complete call, judged later by a person

    # Rollout interface -----------------------------

    def tool_specs(self):
        return deepcopy(self.specs)

    def next_call(self):
        with self.lock:
            return self._next_call()

    def tick(self):
        with self.lock:
            if self.started_at is not None and self._seconds_remaining() <= 0:
                self._finish('budget_seconds')

    def finished(self):
        with self.lock:
            return self.finish_reason is not None

    def halt(self, reason):
        with self.lock:
            self._finish(f'halted:{reason}')

    def outcome(self):
        with self.lock:
            return {
                'finish_reason': self.finish_reason,
                'claim': self.claim,
                'actions_executed': self.actions_executed,
                'observations': self.observation_count,
                'tool_calls': self.call_count,
                'rejections': dict(self.rejections),
                'elapsed_s': (None if self.started_at is None
                              else round(time.monotonic() - self.started_at, 1)),
            }

    def handle_tool_call(self, tool, arguments):
        with self.lock:
            self.call_count += 1
            index = self.call_count
            finish_reason = self.finish_reason
        # Replies take the lock themselves, so they are built outside it.
        if finish_reason is not None:
            return self._reject(index, tool, 'episode_finished',
                                f'The episode has finished ({finish_reason}).')
        if not isinstance(arguments, dict):
            return self._reject(index, tool, 'invalid_arguments', 'Arguments must be an object.')
        if tool == 'robot_start':
            if self.started_at is not None:
                return self._reject(index, tool, 'already_started',
                                    'robot_start was already called; use robot_observe.')
            with self.lock:
                self.started_at = time.monotonic()
            return self._observe(index, tool)
        if self.started_at is None:
            return self._reject(index, tool, 'not_started', 'Call robot_start first.')
        if tool == 'robot_observe':
            return self._observe(index, tool)
        if tool == 'task_complete':
            return self._complete(index, tool, arguments)
        if tool == PLAN_TOOL:
            return execute_plan(self, index, arguments)
        if tool in ACTION_TOOLS:
            return self._act(index, tool, arguments)
        return self._reject(index, tool, 'unknown_tool', f'Unknown robot tool {tool!r}.')

    # Observation ---------------------------------------------------------

    def _observe(self, index, tool):
        result = self.guard.dispatch('observe', {})
        self._record_private(index, tool, {}, result)
        if result.get('status') != 'ok':
            # Nothing moved; the model may simply observe again.
            with self.lock:
                self.context = None
            return self._reply(index, tool, action_executed=False, status='observation_failed',
                               reason=result.get('reason', 'observation failed'))
        observation = self._save_observation(result['data'], result['images'])
        return self._reply(index, tool, action_executed=False, status='observed',
                           reason='New synchronized observation.', observation=observation,
                           images=result['images'])

    def _save_observation(self, data, images):
        """Write the public copy of one snapshot and make it the current observation."""
        with self.lock:
            self.observation_count += 1
            number_ = self.observation_count
            self.context = dict(data.get('action_context') or {})
            context = dict(self.context)
        directory = self.public_dir / 'observations' / f'{number_:03d}'
        directory.mkdir(parents=True, exist_ok=False)
        image_files = []
        for image in images:
            path = directory / f'{image["camera"]}.jpg'
            path.write_bytes(image['jpeg'])
            image_files.append({'camera': image['camera'], 'path': str(path),
                                'width': image.get('width'), 'height': image.get('height')})
        observation = {
            'observation_number': number_,
            'observation_id': context.get('observation_id'),
            'stamp_s': data.get('stamp_s'),
            'frame': data.get('frame'),
            'arms': {
                arm: {
                    'xyz': [rounded(state['ee'][axis]) for axis in 'xyz'],
                    'rpy': [rounded(state['ee'][angle]) for angle in ('roll', 'pitch', 'yaw')],
                    'gripper': rounded(state['gripper'].get('value', 0.0), 3),
                    'gripper_rad': rounded(state['gripper']['position_rad']),
                }
                for arm, state in (data.get('arms') or {}).items()
            },
            'body': {name: rounded(joint['position'])
                     for name, joint in (data.get('body') or {}).items()},
            'base': self._base_pose(data.get('base')),
            'images': image_files,
            'path': str(directory / 'observation.json'),
        }
        write_json(directory / 'observation.json', observation)
        return observation

    # Completion ----------------------------------------------------------

    def _complete(self, index, tool, arguments):
        """The model declares the task done; record a final observation and finish."""
        if set(arguments) != {'request_id', 'reason'}:
            return self._reject(index, tool, 'invalid_arguments',
                                'task_complete takes request_id and reason.')
        if not isinstance(arguments['reason'], str) or not arguments['reason'].strip():
            return self._reject(index, tool, 'invalid_arguments',
                                'reason must describe the visible evidence.')
        with self.lock:
            context = dict(self.context or {})
        if not context.get('request_id'):
            return self._reject(index, tool, 'observe_required',
                                'No current observation. Call robot_observe first.')
        if arguments['request_id'] != context['request_id']:
            return self._reject(index, tool, 'stale_request_id',
                                'request_id does not match the latest observation.')

        # Final state for the person who judges the claim; the model does not need it.
        result = self.guard.dispatch('observe', {})
        self._record_private(index, tool, arguments, result)
        final = None
        if result.get('status') == 'ok':
            final = self._save_observation(result['data'], result['images'])['path']
        with self.lock:
            self.claim = {'at': utc_now(), 'reason': arguments['reason'],
                          'request_id': arguments['request_id'],
                          'final_observation': final}
            self._finish('task_complete')
        return self._reply(index, tool, action_executed=False, status='completed',
                           reason='The episode has ended on your task_complete call.',
                           result={'final_observation': final})


    # Actions -------------------------------------------------------------

    def _act(self, index, tool, arguments, *, internal=False, step_index=None):
        reply = partial(self._reply, internal=internal)
        reject = partial(self._reject, internal=internal)

        expected = {'request_id', 'reason', *ACTION_ARGUMENTS[tool]}
        missing = sorted(expected - set(arguments))
        extra = sorted(set(arguments) - expected)
        if missing or extra:
            return reject(index, tool, 'invalid_arguments',
                                f'Missing {missing}, unexpected {extra}.')
        if not isinstance(arguments['reason'], str) or not arguments['reason'].strip():
            return reject(index, tool, 'invalid_arguments',
                                'reason must describe the visible evidence and purpose.')
        with self.lock:
            context = dict(self.context or {})

            if self.started_at is not None and self._seconds_remaining() <= 0:
                self._finish('budget_seconds')

            if self.finish_reason is not None:
                blocked = (
                    'episode_finished',
                    f'Episode ended: {self.finish_reason}',
                )
            elif self.actions_executed >= self.max_actions:
                self._finish('budget_actions')
                blocked = (
                    'budget_actions',
                    'The action budget is used up.',
                )
            elif not context.get('movement_allowed'):
                blocked = (
                    'observe_required',
                    'No valid observation for movement. Call robot_observe.',
                )
            elif arguments['request_id'] != context.get('request_id'):
                blocked = (
                    'stale_request_id',
                    'request_id does not match the latest observation; use '
                    'action_context.request_id from the latest packet.',
                )
            else:
                blocked = None
        if blocked:
            return reject(index, tool, *blocked)

        guard_args = {key: arguments[key] for key in ACTION_ARGUMENTS[tool]}
        guard_args.update(observation_id=context['observation_id'],
                          request_id=arguments['request_id'])
        result = self.guard.dispatch(ACTION_TOOLS[tool], guard_args)
        self._record_private(index, tool, arguments, result, step_index=step_index)
        status = result.get('status')
        data = result.get('data') or {}
        sent = result.get('command_sent')

        if status == 'rejected' and sent is False:
            if data.get('invalidate_observation'):
                # Nothing ran, but the robot was not ready (base moving, odom, cmd_vel).
                with self.lock:
                    self.context = None
                return reply(
                    index, tool, action_executed=False, status='rejected',
                    rejection='not_ready', reason=result.get('reason'),
                    result={'requested': data.get('requested')},
                    observation_note='The observation is no longer valid. Call robot_observe.')
            if 'action_context' in data:
                # Argument validation : nothing ran, the observation stays valid
                with self.lock:
                    self.context = dict(data['action_context'])
                return reply(
                    index, tool, action_executed=False, status='rejected',
                    rejection='invalid_action', reason=result.get('reason'),
                    result={'requested': data.get('requested'),
                            'current_pose': self._pose(data.get('current_pose'))},
                    observation_note='The previous observation is still current.')
            # Guard: expired or changed observation, or run halted
            with self.lock:
                self.context = None
                halted = data.get('run_halted')
            code = 'run_halted' if halted else 'observation_rejected'
            return reject(index, tool, code, result.get('reason'))

        if status in ('arrived', 'stopped') and sent is True:
            # stopped: blocked short of the target and at rest; it ran and is measured
            with self.lock:
                self.actions_executed += 1
                if self.actions_executed >= self.max_actions:
                    self._finish('budget_actions')
            action_result = {'arrival': self._arrival(data.get('arrival')),
                             'target': data.get('target')}
            if data.get('motion') is not None:
                action_result['motion'] = self._motion(data['motion'])
            if data.get('post_observation') is None or data.get('post_observation_error'):
                with self.lock:
                    self.context = None
                return reply(
                    index, tool, action_executed=True, status=f'{status}_without_observation',
                    reason=data.get('post_observation_error') or 'post-action observation missing',
                    result=action_result,
                    observation_note='No new observation. Call robot_observe before moving.')
            observation = self._save_observation(data['post_observation'], result['images'])
            return reply(index, tool, action_executed=True, status=status,
                               reason=result.get('reason'), result=action_result,
                               observation=observation, images=result['images'])

        with self.lock:
            self._finish('uncertain_execution')
        raise UncertainExecution(f'{tool}: status={status} command_sent={sent} '
                                 f'reason={result.get("reason")}')

    @staticmethod
    def _pose(pose):
        if not pose:
            return None
        return {key: rounded(pose[key]) for key in ('x', 'y', 'z') if key in pose}

    @staticmethod
    def _base_pose(base):
        if not base:
            return None
        return {'frame': base.get('frame'),
                'xy': [rounded(base['x']), rounded(base['y'])],
                'yaw': rounded(base['yaw'])}

    @staticmethod
    def _motion(motion):
        """Base result for the model: what was commanded and what odom measured."""
        measured = motion.get('odom_measured')
        return {
            'commanded': {key: rounded(value) for key, value in motion['commanded'].items()},
            'duration_s': rounded(motion['duration_s'], 2),
            'rest_confirmed': bool(motion.get('rest_confirmed')),
            'odom_measured': None if not measured else {
                'frame': measured['frame'],
                'dx_m': rounded(measured['dx']),
                'dy_m': rounded(measured['dy']),
                'dyaw_rad': rounded(measured['dyaw']),
            },
        }

    @staticmethod
    def _arrival(arrival):
        if not arrival:
            return None
        result = {'arrived': arrival.get('arrived'), 'stopped': bool(arrival.get('stopped'))}
        if 'error_rad' in arrival:
            result['error_rad'] = rounded(arrival['error_rad'])
        if 'position_error_m' in arrival:
            result['position_error_m'] = rounded(arrival['position_error_m'])
        if 'orientation_error_deg' in arrival:
            result['orientation_error_deg'] = round(arrival['orientation_error_deg'], 2)
        return result

    # Replies -------------------------------------------------------------

    def _reject(self, index, tool, code, reason, *, internal=False):
        return self._reply(index, tool, action_executed=False, status='rejected',
                           rejection=code, reason=reason, internal=internal)

    def _reply(self, index, tool, action_executed, status, reason, rejection=None,
               result=None, observation=None, images=(), observation_note=None, *, internal=False):
        """One JSON packet line, then one data: URL line per camera image."""
        with self.lock:
            if rejection is not None:
                self.rejections[rejection] = self.rejections.get(rejection, 0) + 1
            context = self.context or {}
            packet = {
                'action_executed': action_executed,
                'tool': tool,
                'status': status,
                'rejection': rejection,
                'reason': reason,
                'result': result,
                'observation': observation,
                'observation_note': observation_note,
                'action_context': {
                    'movement_allowed': bool(context.get('movement_allowed')),
                    'request_id': context.get('request_id'),
                    'max_age_s': context.get('max_age_s'),
                },
                'progress': {
                    'call_index': index,
                    'actions_executed': self.actions_executed,
                    'actions_remaining': max(0, self.max_actions - self.actions_executed),
                    'seconds_remaining': (None if self.started_at is None
                                          else round(self._seconds_remaining())),
                    'rollout_finished': self.finish_reason is not None,
                },
                'next_call': self._next_call(),
                'history_path': str(self.history_path),
            }
            if internal:
                return {
                    "packet": packet,
                    "images": images,
                }
            if index == 1 or (tool == 'robot_start' and observation is not None):
                packet['task'] = self.task
                packet['frame'] = self.cfg['frames']['base']
        with self.history_path.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps({'at': utc_now(), **packet}, ensure_ascii=False) + '\n')
        items = [{'type': 'inputText', 'text': json.dumps(packet, separators=(',', ':'))}]
        for image in images:
            encoded = base64.b64encode(image['jpeg']).decode('ascii')
            items.append({'type': 'inputImage', 'imageUrl': 'data:image/jpeg;base64,' + encoded})
        return {'success': True, 'contentItems': items}

    def _record_private(
        self, index, tool, arguments, result, *, step_index=None
    ):
        """Save a result without JPEG bytes."""
        record = {
            key: value
            for key, value in result.items()
            if key != 'images'
        }
        record['images'] = [
            {
                key: value
                for key, value in image.items()
                if key != 'jpeg'
            }
            for image in result.get('images', [])
        ]

        filename = (
            f'{index:04d}_{tool}.json'
            if step_index is None
            else f'plan_{index:04d}_step_{step_index:02d}.json'
        )

        write_json(
            self.private_dir / filename,
            {
                'at': utc_now(),
                'tool': tool,
                'arguments': arguments,
                'result': record,
            },
        )

    # Internal state (call with the lock held) ----------------------------

    def _seconds_remaining(self):
        return self.max_seconds - (time.monotonic() - self.started_at)

    def _finish(self, reason):
        if self.finish_reason is None:
            self.finish_reason = reason

    def _next_call(self):
        if self.finish_reason is not None:
            return None
        if self.started_at is None:
            return {'tool': 'robot_start'}
        context = self.context or {}
        if context.get('movement_allowed'):
            return {
                'tool': (
                    'robot_move | robot_gripper | robot_head | robot_lift | '
                    + ('robot_base | ' if 'base' in self.cfg else '')
                    + 'robot_execute_plan'
                ),
                'request_id': context.get('request_id'),
            }
        return {'tool': 'robot_observe'}
