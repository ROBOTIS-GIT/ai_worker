"""
Validation and execution tools reached by the model through RobotRollout and ObservationGuard.
"""

import math

from transforms import base_delta, quat_angle, rpy_to_quat, quat_to_rpy


POSE_KEYS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
OBSERVATION_CAMERAS = ('head', 'wrist_left', 'wrist_right')


def _finite_float(args, key):
    """Read one required argument as a finite float

    Returns:
        (value, None) on success
        (None, reason) on failure
    """
    if key not in args:
        return None, f'missing required argument {key}'

    raw = args[key]

    # bool passes as a number in Python (True == 1) reject it explicitly
    if isinstance(raw, bool):
        return None, f'{key} is not a number ({raw!r})'

    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None, f'{key} is not a number ({raw!r})'

    if not math.isfinite(value):
        return None, f'{key} is not finite'
    return value, None

def _arrival_status(arrival):
    """arrived, stopped (blocked short of the target and still) or timeout"""
    if arrival['arrived']:
        return 'arrived'
    return 'stopped' if arrival.get('stopped') else 'timeout'

def validate_move(args, cfg, current_pose, arm_base_z):
    """Validate one absolute E-E target against robot-only limits

    Returns:
        (True, 'ok', clean) when accepted
        (False, reason, None) when rejected
    """
    if not isinstance(args, dict):
        return False, f'args must be a dict, not {type(args).__name__}', None

    arm = args.get('arm')
    if arm not in cfg['frames']['ee']:
        return False, f'unknown arm {arm!r}', None

    clean = {'arm': arm}

    for key in POSE_KEYS:
        value, error = _finite_float(args, key)
        if error is not None:
            return False, error, None
        clean[key] = value

    # Reach is measured from the lift-carried arm base, so the bound moves
    # with the lift and says nothing about the environment.
    reach = cfg['reach']
    z_min = float(reach['z_min'])
    z_from_arm_base = clean['z'] - float(arm_base_z)

    if z_from_arm_base < z_min:
        return (
            False,
            (
                f'z {clean["z"]:.3f} is {-z_from_arm_base:.3f} m below '
                f'{reach["frame"]}; the arm reaches {-z_min:.3f} m below it'
            ),
            None,
        )

    clean['z_from_arm_base'] = z_from_arm_base

    step_m = math.dist(
        (current_pose['x'], current_pose['y'], current_pose['z']),
        (clean['x'], clean['y'], clean['z']),
    )
    max_step_m = float(cfg['limits']['max_step_m'])

    if step_m > max_step_m:
        return (False, f'step {step_m:.3f} m exceeds {max_step_m} m', None)

    clean['step_m'] = step_m

    # Rotation is limited against the current measured pose, not a stored
    # reference pose, so no prior orientation is handed to the model.
    target_quat = rpy_to_quat(clean['roll'], clean['pitch'], clean['yaw'])
    rotation_deg = quat_angle(target_quat, current_pose['quat'])
    max_rotation_deg = math.degrees(float(cfg['limits']['max_rotation_rad']))

    if rotation_deg > max_rotation_deg:
        return (
            False,
            (
                f'rotation {rotation_deg:.1f} deg from the current pose '
                f'exceeds {max_rotation_deg:.1f} deg'
            ),
            None,
        )

    clean['rotation_deg'] = rotation_deg

    return True, 'ok', clean


def validate_gripper(args, cfg):
    """Validate one normalized gripper command"""
    if not isinstance(args, dict):
        return False, f'arguments are not an object ({type(args).__name__})', None

    arm = args.get('arm')
    if arm not in cfg['frames']['ee']:
        return False, f'unknown arm {arm!r}', None

    value, error = _finite_float(args, 'value')
    if error is not None:
        return False, error, None

    if not 0.0 <= value <= 1.0:
        return False, f'value {value} outside [0, 1]', None

    return True, 'ok', {'arm': arm, 'value': value}


BASE_ARGS = ('vx', 'vy', 'wz', 'duration_s')


def validate_base(args, base_cfg):
    """Validate one body-velocity profile against the base limits

    Linear speed and step are checked on hypot(vx, vy), not per axis. Every
    non-zero axis must reach the controller deadband, or the controller
    zeroes it. duration_s includes the ramps.

    Returns:
        (True, 'ok', clean) when accepted
        (False, reason, None) when rejected
    """
    if not isinstance(args, dict):
        return False, f'args must be a dict, not {type(args).__name__}', None

    clean = {}
    for key in BASE_ARGS:
        # Numbers only: float() would also accept '0.1' and b'0.1'.
        if key in args and not isinstance(args[key], (int, float)):
            return False, f'{key} is not a number ({args[key]!r})', None
        value, error = _finite_float(args, key)
        if error is not None:
            return False, error, None
        clean[key] = value

    eps = 1e-9
    duration_s = clean['duration_s']
    max_duration_s = float(base_cfg['max_duration_s'])
    if not 0.0 < duration_s <= max_duration_s + eps:
        return False, f'duration_s {duration_s} outside (0, {max_duration_s}]', None

    if clean['vx'] == 0.0 and clean['vy'] == 0.0 and clean['wz'] == 0.0:
        return False, 'vx, vy and wz are all zero', None

    for key in ('vx', 'vy', 'wz'):
        deadband = float(base_cfg['deadband_rps' if key == 'wz' else 'deadband_mps'])
        if clean[key] != 0.0 and abs(clean[key]) < deadband - eps:
            unit = 'rad/s' if key == 'wz' else 'm/s'
            return (False, f'{key}={clean[key]} is below the controller deadband '
                           f'{deadband} {unit}; use 0 or at least {deadband}', None)

    speed = math.hypot(clean['vx'], clean['vy'])
    max_lin = float(base_cfg['max_lin_mps'])
    max_ang = float(base_cfg['max_ang_rps'])
    if speed > max_lin + eps:
        return False, f'hypot(vx, vy) {speed:.3f} m/s exceeds {max_lin} m/s', None
    if abs(clean['wz']) > max_ang + eps:
        return False, f'|wz| {abs(clean["wz"]):.3f} rad/s exceeds {max_ang} rad/s', None

    max_step_m = float(base_cfg['max_step_m'])
    max_step_rad = float(base_cfg['max_step_rad'])
    if speed * duration_s > max_step_m + eps:
        return (False, f'hypot(vx, vy) * duration_s = {speed * duration_s:.3f} m '
                       f'exceeds {max_step_m} m', None)
    if abs(clean['wz']) * duration_s > max_step_rad + eps:
        return (False, f'|wz| * duration_s = {abs(clean["wz"]) * duration_s:.3f} rad '
                       f'exceeds {max_step_rad} rad', None)

    return True, 'ok', clean


def _same_direction(command, stalled):
    """True when command pushes the way a stalled command did"""
    lin_a = (command['vx'], command['vy'])
    lin_b = (stalled['vx'], stalled['vy'])
    norm_a, norm_b = math.hypot(*lin_a), math.hypot(*lin_b)
    if norm_a > 0.0 and norm_b > 0.0:
        cosine = (lin_a[0] * lin_b[0] + lin_a[1] * lin_b[1]) / (norm_a * norm_b)
        if cosine > 0.5:
            return True
    return command['wz'] * stalled['wz'] > 0.0


class RobotTools:
    """Model-facing tools built on top of RobotIO"""

    def __init__(self, io, cfg):
        self.io = io
        self.cfg = cfg
        # Command of the last base stall; blocks a same-direction retry until
        # another action or a new observation.
        self._base_stall = None
        # Last command that moved a negligible amount; a second one in the
        # same direction counts as a stall.
        self._base_negligible = None

    def _model_arms(self, raw_arms):
        """Convert raw RobotIO arm state to model-facing units"""
        lower_rad, upper_rad = (
            float(value)
            for value in self.cfg['limits']['gripper_rad']
        )
        gripper_span = upper_rad - lower_rad

        if gripper_span <= 0.0:
            raise ValueError(
                'limits.gripper_rad upper bound must be greater '
                'than the lower bound'
            )

        arms = {}

        for arm, measured in raw_arms.items():
            pose = measured['ee']
            gripper = measured['gripper']

            roll, pitch, yaw = quat_to_rpy(pose['quat'])
            position_rad = float(gripper['position_rad'])
            normalized = (position_rad - lower_rad) / gripper_span

            gripper_data = {
                'joint': gripper['joint'],
                'value': normalized,
                'position_rad': position_rad,
                'stamp_s': float(gripper['stamp_s']),
            }

            # Latest-state observations provide age_s, while snapshots do not
            if 'age_s' in gripper:
                gripper_data['age_s'] = float(gripper['age_s'])

            arms[arm] = {
                'ee': {
                    'frame': pose['frame'],
                    'x': float(pose['x']),
                    'y': float(pose['y']),
                    'z': float(pose['z']),
                    'roll': float(roll),
                    'pitch': float(pitch),
                    'yaw': float(yaw),
                    'stamp_s': float(pose['stamp_s']),
                },
                'gripper': gripper_data,
            }
        return arms

    @staticmethod
    def _model_base(raw_base):
        """Odom pose and velocity without the wall-clock receive time"""
        base = {
            key: (raw_base[key] if key in ('frame', 'child_frame') else float(raw_base[key]))
            for key in ('frame', 'child_frame', 'x', 'y', 'yaw', 'vx', 'vy', 'wz', 'stamp_s')
        }
        if 'age_s' in raw_base:
            base['age_s'] = float(raw_base['age_s'])
        return base

    def get_state(self):
        """Return the latest robot state in model-facing units"""
        raw = self.io.get_state()

        return {
            'status': 'ok',
            'reason': 'state observed',
            'command_sent': False,
            'data': {
                'frame': raw['frame'],
                'arms': self._model_arms(raw['arms']),
                'clock': raw['clock'],
                'body': raw['body'],
                **({'base': self._model_base(raw['base'])} if 'base' in raw else {}),
            },
            'images': [],
        }

    @staticmethod
    def _image_metadata(image):
        """Return image metadata without JPEG bytes"""
        return {
            key: value
            for key, value in image.items()
            if key != 'jpeg'
        }

    def _snapshot_data(self, snapshot):
        """Return JSON-safe metadata and model-facing snapshot state"""
        return {
            'stamp_s': float(snapshot['stamp_s']),
            'frame': snapshot['frame'],
            'arms': self._model_arms(snapshot['arms']),
            'clock': snapshot['clock'],
            'body': snapshot['body'],
            **({'base': self._model_base(snapshot['base'])} if 'base' in snapshot else {}),
            'max_offset_s': float(snapshot['max_offset_s']),
            'cameras': {
                camera: self._image_metadata(image)
                for camera, image in snapshot['images'].items()
            },
        }

    def observe(self):
        """Return all cameras and both arms from one synchronized snapshot"""
        missing = [
            camera
            for camera in OBSERVATION_CAMERAS
            if camera not in self.cfg['cameras']
        ]

        if missing:
            raise KeyError(
                f'missing configured cameras: {", ".join(missing)}'
            )

        snapshot = self.io.get_snapshot(OBSERVATION_CAMERAS)
        images = [
            snapshot['images'][camera]
            for camera in OBSERVATION_CAMERAS
        ]

        return {
            'status': 'ok',
            'reason': 'synchronized observation captured',
            'command_sent': False,
            'data': self._snapshot_data(snapshot),
            'images': images,
        }

    def _capture_after(self, cameras, stamp_s):
        """Capture a synchronized observation after an action
        
        Returns:
            (snapshot, None) on success
            (None, reason) on failure
        """
        cameras = tuple(cameras)

        # Capture camera frames and robot state from one post-action moment
        try:
            snapshot = self.io.get_snapshot(
                cameras,
                after_stamp_s=stamp_s,
            )
        except Exception as exc:
            return None, f'{type(exc).__name__}: {exc}'

        return snapshot, None

    def move(self, arm, x, y, z, roll, pitch, yaw):
        """Validate, publish and wait for one absolute arm movement"""
        args = {
            'arm': arm,
            'x': x,
            'y': y,
            'z': z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
        }

        if arm not in self.cfg['frames']['ee']:
            return {
                'status': 'rejected',
                'reason': f'unknown arm {arm!r}',
                'command_sent': False,
                'data': {'requested': args},
                'images': [],
            }

        try:
            current_pose = self.io.get_ee_pose(arm)
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'failed to read current pose: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': False,
                'data': {'requested': args},
                'images': [],
            }

        try:
            arm_base = self.io.get_arm_base_z()
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'failed to read arm base height: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': False,
                'data': {'requested': args},
                'images': [],
            }

        try:
            ok, reason, target = validate_move(args, self.cfg, current_pose, arm_base['z'])
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'move validation failed: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': False,
                'data': {'requested': args},
                'images': [],
            }

        if not ok:
            return {
                'status': 'rejected',
                'reason': reason,
                'command_sent': False,
                'data': {
                    'requested': args,
                    'current_pose': current_pose,
                },
                'images': [],
            }

        try:
            sent = self.io.send_pose(arm, target)
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'failed to publish move command: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': False,
                'data': {
                    'target': target,
                    'current_pose': current_pose,
                },
                'images': [],
            }

        try:
            arrival = self.io.wait_until_arrived(arm, target, sent['quat'])
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'failed while waiting for movement: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': True,
                'data': {
                    'target': target,
                    'command': sent,
                },
                'images': [],
            }

        cameras = OBSERVATION_CAMERAS
        snapshot, observation_error = self._capture_after(
            cameras,
            arrival['pose']['stamp_s'],
        )

        status = _arrival_status(arrival)
        reason = {
            'arrived': 'move target reached',
            'stopped': 'move stopped before its target',
            'timeout': 'move did not settle before timeout',
        }[status]

        if observation_error is not None:
            reason += (
                '; post observation unavailable: '
                f'{observation_error}'
            )

        return {
            'status': status,
            'reason': reason,
            'command_sent': True,
            'data': {
                'target': target,
                'command': sent,
                'arrival': arrival,
                'post_observation': (
                    None
                    if snapshot is None
                    else self._snapshot_data(snapshot)
                ),
                'post_observation_error': observation_error,
            },
            'images': (
                []
                if snapshot is None
                else [
                    snapshot['images'][camera]
                    for camera in cameras
                ]
            ),
        }
    
    def set_gripper(self, arm, value):
        """Validate, publish and wait for one gripper movement"""
        args = {
            'arm': arm,
            'value': value,
        }

        ok, reason, clean = validate_gripper(args, self.cfg)

        if not ok:
            return {
                'status': 'rejected',
                'reason': reason,
                'command_sent': False,
                'data': {'requested': args},
                'images': [],
            }

        try:
            target_rad = self.io.send_gripper(
                clean['arm'],
                clean['value'],
            )
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'failed to publish gripper command: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': False,
                'data': {'target': clean},
                'images': [],
            }

        try:
            arrival = self.io.wait_until_gripper_arrived(
                clean['arm'],
                target_rad,
            )
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'failed while waiting for gripper: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': True,
                'data': {
                    'target': clean,
                    'target_rad': target_rad,
                },
                'images': [],
            }

        cameras = OBSERVATION_CAMERAS
        snapshot, observation_error = self._capture_after(
            cameras,
            arrival['gripper']['stamp_s'],
        )

        status = _arrival_status(arrival)
        reason = {
            'arrived': 'gripper target reached',
            'stopped': 'gripper stopped before its target',
            'timeout': 'gripper did not settle before timeout',
        }[status]

        if observation_error is not None:
            reason += (
                '; post observation unavailable: '
                f'{observation_error}'
            )

        return {
            'status': status,
            'reason': reason,
            'command_sent': True,
            'data': {
                'target': clean,
                'target_rad': target_rad,
                'arrival': arrival,
                'post_observation': (
                    None
                    if snapshot is None
                    else self._snapshot_data(snapshot)
                ),
                'post_observation_error': observation_error,
            },
            'images': (
                []
                if snapshot is None
                else [
                    snapshot['images'][camera]
                    for camera in cameras
                ]
            ),
        }

    def move_head(self, head_joint1, head_joint2):
        return self._move_body(
            'head',
            {
                'head_joint1': head_joint1,
                'head_joint2': head_joint2,
            },
        )
    
    def move_lift(self, position_m):
        return self._move_body(
            'lift',
            {'lift_joint': position_m},
        )

    def _move_body(self, group, requested):
        """Validate, send, wait and capture one head/lift movement"""
        targets = {}

        for joint in self.io.body_groups[group]:
            value, error = _finite_float(requested, joint)

            if error is None:
                lower, upper = self.io.body_limits[joint]
                if not lower <= value <= upper:
                    error = (
                        f'{joint}={value} is outside '
                        f'[{lower}, {upper}]'
                    )

            if error is not None:
                return {
                    'status': 'rejected',
                    'reason': error,
                    'command_sent': False,
                    'data': {'requested': requested},
                    'images': [],
                }

            targets[joint] = value

        try:
            self.io.get_body_state(group=group)
            sent_at = self.io.send_body(group, targets)
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'failed before sending {group} command: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': False,
                'data': {'target': targets},
                'images': [],
            }

        try:
            arrival = self.io.wait_until_body_arrived(
                group,
                targets,
                after_received=sent_at,
            )
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'failed while waiting for {group}: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': True,
                'data': {'target': targets},
                'images': [],
            }

        # Require images newer than every measured joint in the result.
        after_stamp_s = max(
            state['stamp_s']
            for state in arrival['joints'].values()
        )

        snapshot, observation_error = self._capture_after(
            OBSERVATION_CAMERAS,
            after_stamp_s,
        )

        status = _arrival_status(arrival)
        reason = {
            'arrived': f'{group} reached its joint targets',
            'stopped': f'{group} stopped before its joint targets',
            'timeout': f'{group} did not settle before timeout',
        }[status]

        if observation_error is not None:
            reason += f'; post-action observation failed: {observation_error}'

        return {
            'status': status,
            'reason': reason,
            'command_sent': True,
            'data': {
                'target': targets,
                'arrival': arrival,
                'post_observation': (
                    None
                    if snapshot is None
                    else self._snapshot_data(snapshot)
                ),
                'post_observation_error': observation_error,
            },
            'images': (
                []
                if snapshot is None
                else [
                    snapshot['images'][camera]
                    for camera in OBSERVATION_CAMERAS
                ]
            ),
        }

    
    def _posture_problem(self):
        """None when both hands are tucked in and the lift is in range for base motion."""
        posture = self.cfg['base']['allowed_posture']
        ee_x_max = float(posture['ee_x_max_m'])
        ee_abs_y_max = float(posture['ee_abs_y_max_m'])
        lift_low, lift_high = map(float, posture['lift_m'])

        for arm in self.cfg['frames']['ee']:
            pose = self.io.get_ee_pose(arm)
            if pose['x'] > ee_x_max or abs(pose['y']) > ee_abs_y_max:
                return (f'{arm} hand at x {pose["x"]:.3f}, y {pose["y"]:.3f} is outside the '
                        f'base-motion posture (x <= {ee_x_max}, |y| <= {ee_abs_y_max}); '
                        'bring it closer to the body first')
        lift = self.io.get_body_state(group='lift')['lift_joint']['position']
        if not lift_low <= lift <= lift_high:
            return (f'lift_joint {lift:.3f} m is outside the base-motion range '
                    f'[{lift_low}, {lift_high}]')
        return None

    def _base_rest_problem(self, state):
        base = self.cfg['base']
        if (math.hypot(state['vx'], state['vy']) >= float(base['rest_speed_mps'])
                or abs(state['wz']) >= float(base['rest_speed_rps'])):
            return 'the base is still moving'
        return None

    def move_base(self, vx, vy, wz, duration_s):
        """Run one bounded base-velocity profile, confirm rest, then observe

        Success means the profile ran and the base came to rest, not that a
        distance was reached. data.motion reports the odom-measured net
        displacement in the start base_link frame.

        Rejections:
            data.invalidate_observation False: the request itself is invalid
                or the posture does not allow base motion; the observation stays.
            data.invalidate_observation True: the robot is not ready (odom,
                cmd_vel, base still moving); a new observation is needed.
        """
        requested = {'vx': vx, 'vy': vy, 'wz': wz, 'duration_s': duration_s}
        base_cfg = self.cfg['base']

        def rejected(reason, invalidate):
            return {
                'status': 'rejected',
                'reason': reason,
                'command_sent': False,
                'data': {'requested': requested, 'invalidate_observation': invalidate},
                'images': [],
            }

        ok, reason, command = validate_base(requested, base_cfg)
        if not ok:
            return rejected(reason, False)

        if self._base_stall is not None and _same_direction(command, self._base_stall):
            return rejected(
                'the base made no progress in this direction; inspect the new observation '
                'and identify the cause before attempting that direction again', False)

        try:
            problem = self._posture_problem()
        except Exception as exc:
            return rejected(f'failed to read posture: {type(exc).__name__}: {exc}', True)
        if problem:
            return rejected(problem, False)

        try:
            problem = (self._base_rest_problem(self.io.get_base_state())
                       or self.io.base_command_problem())
        except Exception as exc:
            problem = f'{type(exc).__name__}: {exc}'
        if problem:
            return rejected(f'base not ready: {problem}', True)

        try:
            run = self.io.send_base(**command)
        except Exception as exc:
            # send_base raises only before its first publish.
            return rejected(f'base not ready: {type(exc).__name__}: {exc}', True)

        motion = {
            'commanded': run['commanded'],
            'duration_s': run['duration_s'],
            'outcome': run['outcome'],
            'detail': run['detail'],
            'elapsed_s': run['elapsed_s'],
            'path_m': run['path_m'],
            'path_rad': run['path_rad'],
            'rest_confirmed': False,
            'odom_measured': None,
        }

        def failed(status, reason):
            return {
                'status': status,
                'reason': reason,
                'command_sent': True,
                'data': {'target': command, 'motion': motion,
                         'arrival': {'arrived': False, 'stopped': False}},
                'images': [],
            }

        if run['outcome'] not in ('completed', 'stalled'):
            return failed('error', f'base motion ended with {run["outcome"]}: {run["detail"]}')
        if run['stop_error'] is not None:
            return failed('error', f'zero velocity could not be published: {run["stop_error"]}')

        rest = self.io.wait_until_base_stopped(run['stopped_at'])
        if not rest['rested']:
            return failed('timeout', f'base rest not confirmed: {rest["reason"]}')
        motion['rest_confirmed'] = True
        motion['odom_measured'] = {'frame': 'start base_link',
                                   **base_delta(run['start'], rest['state'])}

        # A short command can end before the wheels finish steering, so one
        # negligible move does not block the direction. A stall seen while
        # moving, or a second negligible move in the same direction, does.
        measured = motion['odom_measured']
        no_linear = (command['vx'] != 0.0 or command['vy'] != 0.0) and (
            math.hypot(measured['dx'], measured['dy']) < float(base_cfg['progress_min_m']))
        no_angular = command['wz'] != 0.0 and (
            abs(measured['dyaw']) < float(base_cfg['progress_min_rad']))
        if run['outcome'] == 'stalled':
            status = 'stopped'
            reason = f'base made no progress while moving and stopped ({run["detail"]})'
            self._base_stall = dict(command)
            self._base_negligible = None
        elif no_linear or no_angular:
            status = 'stopped'
            repeated = (self._base_negligible is not None
                        and _same_direction(command, self._base_negligible))
            if repeated:
                reason = 'base moved a negligible amount again in this direction'
                self._base_stall = dict(command)
                self._base_negligible = None
            else:
                reason = 'base moved a negligible amount'
                self._base_negligible = dict(command)
        else:
            status = 'arrived'
            reason = 'base profile ran and the base came to rest'
            self._base_stall = None
            self._base_negligible = None

        snapshot, observation_error = self._capture_after(
            OBSERVATION_CAMERAS,
            rest['state']['stamp_s'],
        )
        if observation_error is not None:
            reason += f'; post-action observation failed: {observation_error}'

        return {
            'status': status,
            'reason': reason,
            'command_sent': True,
            'data': {
                'target': command,
                'motion': motion,
                'arrival': {'arrived': status == 'arrived', 'stopped': status == 'stopped'},
                'post_observation': (
                    None
                    if snapshot is None
                    else self._snapshot_data(snapshot)
                ),
                'post_observation_error': observation_error,
            },
            'images': (
                []
                if snapshot is None
                else [
                    snapshot['images'][camera]
                    for camera in OBSERVATION_CAMERAS
                ]
            ),
        }

    def dispatch(self, name, args):
        """Validate and execute one model-requested tool call."""
        tool_specs = {
            'get_state': {
                'required': set(),
                'allowed': set(),
            },
            'observe': {
                'required': set(),
                'allowed': set(),
            },
            'move': {
                'required': {
                    'arm',
                    'x',
                    'y',
                    'z',
                    'roll',
                    'pitch',
                    'yaw',
                },
                'allowed': {
                    'arm',
                    'x',
                    'y',
                    'z',
                    'roll',
                    'pitch',
                    'yaw',
                },
            },
            'set_gripper': {
                'required': {'arm', 'value'},
                'allowed': {'arm', 'value'},
            },
            'move_head': {
                'required': {'head_joint1', 'head_joint2'},
                'allowed': {'head_joint1', 'head_joint2'},
            },
            'move_lift': {
                'required': {'position_m'},
                'allowed': {'position_m'},
            },
            'move_base': {
                'required': set(BASE_ARGS),
                'allowed': set(BASE_ARGS),
            },
        }

        # only allowed tools are accepted
        if not isinstance(name, str) or name not in tool_specs:
            return {
                'status': 'rejected',
                'reason': f'unknown tool {name!r}',
                'command_sent': False,
                'data': {
                    'tool': repr(name),
                },
                'images': [],
            }

        if not isinstance(args, dict):
            return {
                'status': 'rejected',
                'reason': (
                    f'{name} arguments must be an object, '
                    f'not {type(args).__name__}'
                ),
                'command_sent': False,
                'data': {
                    'tool': name,
                },
                'images': [],
            }

        spec = tool_specs[name]
        missing = sorted(spec['required'] - set(args))
        unexpected = sorted(
            (
                key
                for key in args
                if key not in spec['allowed']
            ),
            key=repr,
        )

        if missing:
            return {
                'status': 'rejected',
                'reason': (
                    f'{name} missing required arguments: '
                    f'{", ".join(missing)}'
                ),
                'command_sent': False,
                'data': {
                    'tool': name,
                    'missing': missing,
                },
                'images': [],
            }

        if unexpected:
            return {
                'status': 'rejected',
                'reason': (
                    f'{name} received unexpected arguments: '
                    f'{", ".join(map(str, unexpected))}'
                ),
                'command_sent': False,
                'data': {
                    'tool': name,
                    'unexpected': unexpected,
                },
                'images': [],
            }

        handlers = {
            'get_state': self.get_state,
            'observe': self.observe,
            'move': self.move,
            'set_gripper': self.set_gripper,
            'move_head': self.move_head,
            'move_lift': self.move_lift,
            'move_base': self.move_base,
        }

        try:
            result = handlers[name](**args)
        except Exception as exc:
            return {
                'status': 'error',
                'reason': (
                    f'{name} failed unexpectedly: '
                    f'{type(exc).__name__}: {exc}'
                ),
                'command_sent': (
                    False
                    if name in ('get_state', 'observe')
                    else None
                ),
                'data': {
                    'tool': name,
                },
                'images': [],
            }

        required_result_keys = {
            'status',
            'reason',
            'command_sent',
            'data',
            'images',
        }

        if not isinstance(result, dict):
            return {
                'status': 'error',
                'reason': (
                    f'{name} returned {type(result).__name__}, '
                    'expected an object'
                ),
                'command_sent': None,
                'data': {
                    'tool': name,
                },
                'images': [],
            }

        missing_result_keys = sorted(
            required_result_keys - set(result)
        )

        if missing_result_keys:
            return {
                'status': 'error',
                'reason': (
                    f'{name} result is missing fields: '
                    f'{", ".join(missing_result_keys)}'
                ),
                'command_sent': result.get('command_sent'),
                'data': {
                    'tool': name,
                    'missing': missing_result_keys,
                },
                'images': [],
            }

        # A new observation or another completed action lifts the stall block.
        if name == 'observe' and result['status'] == 'ok':
            self._base_stall = None
        elif name not in ('get_state', 'move_base') and result['status'] in ('arrived', 'stopped'):
            self._base_stall = None

        return result
