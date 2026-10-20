"""Run-local observation binding in front of RobotTools.dispatch.

This does not cancel Cyclo commands or provide cross-process command ownership.
"""

from copy import deepcopy
import math
import time
from uuid import uuid4

from transforms import quat_angle, rpy_to_quat, wrap_angle


ACTION_TOOLS = frozenset({'move', 'set_gripper', 'move_head', 'move_lift', 'move_base'})
CAMERAS = frozenset({'head', 'wrist_left', 'wrist_right'})
DEFAULTS = {
    'max_age_s': 60.0,
    'max_clock_age_s': 1.0,
    'max_image_age_s': 1.0,
    'max_state_lag_sim_s': 1.0,
    'position_m': 0.01,
    'orientation_deg': 5.0,
    'gripper_rad': 0.03,
    'head_rad': 0.03,
    'lift_m': 0.01,
    'base_position_m': 0.05,
    'base_yaw_deg': 3.0,
}


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('measurement must be a number')
    result = float(value)
    if not math.isfinite(result):
        raise ValueError('measurement must be finite')
    return result


class ObservationGuard:
    def __init__(self, tools, cfg):
        self.tools = tools
        self.cfg = cfg
        supplied = cfg.get('observation_guard', {})
        if not isinstance(supplied, dict) or set(supplied) - set(DEFAULTS):
            raise ValueError('invalid observation_guard configuration')
        self.limits = {key: finite(supplied.get(key, default))
                       for key, default in DEFAULTS.items()}
        if any(value <= 0 for value in self.limits.values()):
            raise ValueError('observation_guard limits must be positive')
        self._baseline = None
        self._observed_at = None
        self._observation_id = None
        self._request_id = None
        self._used_requests = set()
        self._halted = False

    def _invalidate(self):
        self._baseline = None
        self._observed_at = None
        self._observation_id = None
        self._request_id = None

    def _context(self):
        allowed = not self._halted and self._request_id is not None
        return {
            'movement_allowed': allowed,
            'observation_id': self._observation_id if allowed else None,
            'request_id': self._request_id if allowed else None,
            'max_age_s': self.limits['max_age_s'],
            'run_halted': self._halted,
        }

    def _reject(self, reason, code='observation_rejected'):
        return {
            'status': 'rejected',
            'reason': reason,
            'command_sent': False,
            'data': {'code': code, 'requires_observe': not self._halted,
                     'run_halted': self._halted},
            'images': [],
        }

    def _state_problem(self, current):
        previous = self._baseline
        if current['frame'] != previous['frame'] or current['frame'] != self.cfg['frames']['base']:
            return 'coordinate frame changed'
        clock = current['clock']
        if clock['rewinds'] != previous['clock']['rewinds']:
            return 'simulation clock was reset'
        for key in ('received_age_s', 'advanced_age_s'):
            age = finite(clock[key])
            if not 0 <= age <= self.limits['max_clock_age_s']:
                return 'simulation clock is stale or stopped'
        sim_s = finite(clock['sim_s'])

        def fresh(sample):
            lag = sim_s - finite(sample['stamp_s'])
            return abs(lag) <= self.limits['max_state_lag_sim_s']

        for arm in self.cfg['frames']['ee']:
            before = previous['arms'][arm]
            after = current['arms'][arm]
            a, b = before['ee'], after['ee']
            if a['frame'] != current['frame'] or b['frame'] != current['frame']:
                return f'{arm} end-effector frame mismatch'
            if not fresh(b) or not fresh(after['gripper']):
                return f'{arm} measurement is stale'
            distance = math.dist(
                [finite(a[key]) for key in ('x', 'y', 'z')],
                [finite(b[key]) for key in ('x', 'y', 'z')],
            )
            if distance > self.limits['position_m']:
                return f'{arm} position changed by {distance:.4f} m'
            qa = rpy_to_quat(*(finite(a[key]) for key in ('roll', 'pitch', 'yaw')))
            qb = rpy_to_quat(*(finite(b[key]) for key in ('roll', 'pitch', 'yaw')))
            angle = quat_angle(qa, qb)
            if angle > self.limits['orientation_deg']:
                return f'{arm} orientation changed by {angle:.2f} degrees'
            gap = abs(finite(after['gripper']['position_rad'])
                      - finite(before['gripper']['position_rad']))
            if gap > self.limits['gripper_rad']:
                return f'{arm} gripper position changed'

        for joint in ('head_joint1', 'head_joint2', 'lift_joint'):
            a, b = previous['body'][joint], current['body'][joint]
            expected_unit = 'm' if joint == 'lift_joint' else 'rad'
            if a['unit'] != expected_unit or b['unit'] != expected_unit:
                return f'{joint} unit mismatch'
            if not fresh(b):
                return f'{joint} measurement is stale'
            limit = self.limits['lift_m' if joint == 'lift_joint' else 'head_rad']
            if abs(finite(b['position']) - finite(a['position'])) > limit:
                return f'{joint} position changed'

        a, b = previous['base'], current['base']
        if a['frame'] != b['frame']:
            return 'base odometry frame changed'
        if not fresh(b):
            return 'base measurement is stale'
        distance = math.hypot(finite(b['x']) - finite(a['x']), finite(b['y']) - finite(a['y']))
        if distance > self.limits['base_position_m']:
            return f'base position changed by {distance:.4f} m'
        turn = abs(math.degrees(wrap_angle(finite(b['yaw']) - finite(a['yaw']))))
        if turn > self.limits['base_yaw_deg']:
            return f'base heading changed by {turn:.2f} degrees'
        return None

    def _base_moving(self, state):
        """Any action needs the base at rest, however little it has moved so far."""
        base, rest = state['base'], self.cfg['base']
        speed = math.hypot(finite(base['vx']), finite(base['vy']))
        turn = abs(finite(base['wz']))
        if speed >= float(rest['rest_speed_mps']) or turn >= float(rest['rest_speed_rps']):
            return f'the base is moving ({speed:.3f} m/s, {turn:.3f} rad/s)'
        return None

    def _register(self, data, images):
        self._invalidate()
        if self._halted:
            data['action_context'] = self._context()
            return
        try:
            if len(images) != 3 or {image['camera'] for image in images} != CAMERAS:
                raise ValueError('three camera images are required')
            ages = [finite(image['receive_age_s']) for image in images]
            if min(ages) < 0 or max(ages) > self.limits['max_image_age_s']:
                raise ValueError('camera images are stale')
            self._baseline = deepcopy(data)
            problem = self._state_problem(data)
            if problem:
                raise ValueError(problem)
            self._observed_at = time.monotonic() - max(ages)
            self._observation_id = 'obs_' + uuid4().hex
            self._request_id = 'req_' + uuid4().hex
            data['action_context'] = self._context()
        except Exception:
            self._invalidate()
            raise

    def dispatch(self, name, args):
        if name not in ACTION_TOOLS:
            result = self.tools.dispatch(name, args)
            if name == 'observe' and result.get('status') == 'ok':
                self._register(result['data'], result['images'])
            return result

        if self._halted:
            return self._reject('Movement blocked for this run after an uncertain action.',
                                'run_halted')
        if not isinstance(args, dict):
            return self._reject('Action arguments must be an object.')
        observation_id = args.get('observation_id')
        request_id = args.get('request_id')
        if not isinstance(observation_id, str) or not isinstance(request_id, str):
            return self._reject('Copy observation_id and request_id from action_context.')
        if request_id in self._used_requests:
            return self._reject('This request_id was already processed.', 'duplicate_request')
        if self._request_id is None or observation_id != self._observation_id or request_id != self._request_id:
            return self._reject('Action does not match the current observation. Call observe.')

        try:
            if time.monotonic() - self._observed_at > self.limits['max_age_s']:
                raise ValueError('observation is too old')
            current = self.tools.dispatch('get_state', {})
            if current['status'] != 'ok':
                raise ValueError(current['reason'])
            problem = self._state_problem(current['data']) or self._base_moving(current['data'])
            if problem:
                raise ValueError(problem)
            if time.monotonic() - self._observed_at > self.limits['max_age_s']:
                raise ValueError('observation expired during the state check')
        except Exception as exc:
            self._invalidate()
            return self._reject(f'Pre-action check failed: {exc}. Call observe.')

        # Claim the request before RobotTools can publish anything
        self._used_requests.add(request_id)
        self._request_id = None
        clean_args = {key: value for key, value in args.items()
                      if key not in ('observation_id', 'request_id')}
        try:
            result = self.tools.dispatch(name, clean_args)
            status, sent = result['status'], result['command_sent']
            if not isinstance(result['data'], dict) or not isinstance(result['images'], list):
                raise ValueError('invalid action result')

            if status == 'rejected' and sent is False:
                if result['data'].get('invalidate_observation'):
                    # Nothing moved, but the robot was not ready; observe again.
                    self._invalidate()
                else:
                    # A known validation rejection did not move anything.
                    # Preserve the observation age and issue a new attempt token.
                    self._request_id = 'req_' + uuid4().hex
            elif status in ('arrived', 'stopped') and sent is True:
                self._invalidate()
                snapshot = result['data'].get('post_observation')
                if snapshot is not None and not result['data'].get('post_observation_error'):
                    self._register(snapshot, result['images'])
            else:
                # A timeout is not a Cyclo cancellation or a completion ACK.
                self._halted = True
                self._invalidate()
            result['data']['action_context'] = self._context()
            result['data']['accepted_request'] = {
                'observation_id': observation_id, 'request_id': request_id,
            }
            return result
        except Exception as exc:
            self._halted = True
            self._invalidate()
            return {
                'status': 'error',
                'reason': f'Action execution or result processing failed: {exc}',
                'command_sent': None,
                'data': {'action_context': self._context(),
                         'accepted_request': {'observation_id': observation_id,
                                              'request_id': request_id}},
                'images': [],
            }
