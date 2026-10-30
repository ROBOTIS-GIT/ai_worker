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

from collections import deque
import math
import threading
import time

from builtin_interfaces.msg import Duration
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from robotis_interfaces.msg import MoveL
from sensor_msgs.msg import Image, JointState
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from stall import StallDetector
from transforms import quat_angle, rpy_to_quat, wrap_angle, yaw_from_quat


BASE_KEYS = ('x', 'y', 'yaw', 'vx', 'vy', 'wz')
BASE_LIMITS = (
    'publish_hz', 'deadband_mps', 'deadband_rps', 'max_lin_mps', 'max_ang_rps',
    'max_duration_s', 'max_step_m', 'max_step_rad', 'ramp_s', 'path_margin',
    'steer_grace_s', 'progress_window_s', 'progress_min_m', 'progress_min_rad',
    'odom_max_age_s', 'rest_window_s', 'rest_position_m',
    'rest_yaw_rad', 'rest_speed_mps', 'rest_speed_rps', 'rest_timeout_s',
)
ZERO_REPEATS = 2


def stamp_seconds(stamp):
    """Convert a builtin_interfaces/Time stamp to seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


class RobotIO(Node):
    """Background observation of cameras, joints, TF and clock."""

    def __init__(self, cfg):
        super().__init__(
            'poc_codex_io',
            parameter_overrides=[
                Parameter('use_sim_time', value=False)
            ]
        )
        self.cfg = cfg
        io_cfg = cfg['io']
        self.cameras = dict(cfg['cameras'])
        self.jpeg_quality = int(cfg['run']['jpeg_quality'])
        self.ready_timeout_s = float(io_cfg['ready_timeout_s'])
        self.lookup_timeout_s = float(io_cfg['lookup_timeout_s'])
        self.image_timeout_s = float(io_cfg['image_timeout_s'])
        self.snapshot_tolerance_s = float(io_cfg['snapshot_tolerance_s'])
        self._image_history = int(io_cfg['image_history'])
        self._joint_history = int(io_cfg['joint_history'])

        self._cache_lock = threading.Lock()
        self._joint_cache = {}
        self._image_cache = {}

        self._bridge = CvBridge()

        # Serve TF callbacks on this node's executor.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        # BEST_EFFORT supports reliable and best-effort publishers.
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, qos_profile_sensor_data)
        for camera, topic in self.cameras.items():
            self.create_subscription(
                Image, topic, self._image_callback(camera), qos_profile_sensor_data)

        self.pose_pub = {
            arm: self.create_publisher(MoveL, topic, 10)
            for arm, topic in cfg['cyclo']['goal_topic'].items()
        }
        self.gripper_pub = {
            arm: self.create_publisher(JointTrajectory, topic, 10)
            for arm, topic in cfg['cyclo']['gripper_topic'].items()
        }

        self.body_groups = {
            'head': ('head_joint1', 'head_joint2'),
            'lift': ('lift_joint',),
        }
        self.body_units = {
            'head_joint1': 'rad',
            'head_joint2': 'rad',
            'lift_joint': 'm',
        }

        limit_keys = {
            'head_joint1': 'head_joint1_rad',
            'head_joint2': 'head_joint2_rad',
            'lift_joint': 'lift_joint_m',
        }

        self.body_limits = {}
        for joint, key in limit_keys.items():
            lower, upper = map(float, cfg['limits'][key])
            if (
                not math.isfinite(lower)
                or not math.isfinite(upper)
                or lower > upper
            ):
                raise ValueError(f'invalid limits. {key}')
            self.body_limits[joint] = (lower, upper)

        motion_cfg = cfg.get('body_motion', {})
        self.body_move_time_s = float(motion_cfg.get('move_time_s', 3.0))

        tolerance = cfg['tolerance']
        self.body_tolerances = {
            'head_joint1': float(tolerance.get('head_joint1_rad', 0.01)),
            'head_joint2': float(tolerance.get('head_joint2_rad', 0.01)),
            'lift_joint': float(tolerance.get('lift_joint_m', 0.005)),
        }

        for value in (self.body_move_time_s, *self.body_tolerances.values()):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    'body motion duration and tolerances must be positive'
                )

        self.body_pub = {
            'head': self.create_publisher(
                JointTrajectory,
                motion_cfg.get(
                    'head_topic',
                    '/leader/joystick_controller_left/joint_trajectory',
                ),
                10,
            ),
            'lift': self.create_publisher(
                JointTrajectory,
                motion_cfg.get(
                    'lift_topic',
                    '/leader/joystick_controller_right/joint_trajectory',
                ),
                10,
            )
        }

        # Mobile base I/O
        base_cfg = cfg['base']
        self.base_cfg = {}
        for key in BASE_LIMITS:
            value = float(base_cfg[key])
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'base.{key} must be finite and positive')
            self.base_cfg[key] = value
        self.cmd_vel_topic = str(base_cfg['cmd_vel_topic'])
        self.odom_topic = str(base_cfg['odom_topic'])
        self._odom_cache = deque(maxlen=int(io_cfg['odom_history']))
        self._base_lock = threading.Lock()      # one base command at a time
        self._base_cancel = threading.Event()   # set by close()
        self._base_used = False

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, qos_profile_sensor_data)

        self._io_executor = SingleThreadedExecutor()
        self._io_executor.add_node(self)
        self._spin_thread = None
        self._spin_error = None
        self._closing = False
        self._closed = False

    # Callbacks on the spin thread

    def _on_joint_state(self, msg):
        received = time.monotonic()
        stamp_s = stamp_seconds(msg.header.stamp)
        with self._cache_lock:
            for name, position in zip(msg.name, msg.position):
                history = self._joint_cache.get(name)
                if history is None:
                    history = self._joint_cache[name] = deque(maxlen=self._joint_history)
                history.append({'position': position, 'stamp_s': stamp_s, 'received': received})

    def _on_odom(self, msg):
        received = time.monotonic()
        pose, twist = msg.pose.pose, msg.twist.twist
        q = pose.orientation
        sample = {
            'frame': msg.header.frame_id,
            'child_frame': msg.child_frame_id,
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': yaw_from_quat(q.x, q.y, q.z, q.w),
            'vx': twist.linear.x,
            'vy': twist.linear.y,
            'wz': twist.angular.z,
            'stamp_s': stamp_seconds(msg.header.stamp),
            'received': received,
        }
        with self._cache_lock:
            self._odom_cache.append(sample)

    def _image_callback(self, camera):
        def on_image(msg):
            # Cache raw images; encode JPEGs when requested.
            entry = {
                'msg': msg,
                'stamp_s': stamp_seconds(msg.header.stamp),
                'received': time.monotonic(),
            }
            with self._cache_lock:
                history = self._image_cache.get(camera)
                if history is None:
                    history = self._image_cache[camera] = deque(maxlen=self._image_history)
                history.append(entry)
        return on_image

    # thread lifecycle

    def _spin(self):
        try:
            self._io_executor.spin()
        except ExternalShutdownException:
            pass  # ROS shutdown
        except Exception as exc:
            if not self._closing:
                self._spin_error = exc

    def start(self):
        """Start the background spin thread. Call once, after rclpy.init()."""
        if self._closing:
            raise RuntimeError('RobotIO is closing or already closed')
        if self._spin_thread is not None:
            raise RuntimeError('RobotIO.start() was already called')
        self._spin_thread = threading.Thread(
            target=self._spin, name='robot_io_spin', daemon=True)
        self._spin_thread.start()

    def _check_running(self):
        if self._closing:
            raise RuntimeError('RobotIO is closing or already closed')
        if self._spin_error is not None:
            raise RuntimeError(
                f'ROS spin thread failed: {self._spin_error!r}'
            ) from self._spin_error
        if self._spin_thread is None:
            raise RuntimeError('RobotIO.start() has not been called')
        if not self._spin_thread.is_alive():
            raise RuntimeError('ROS spin thread stopped (rclpy shut down?)')

    def close(self):
        """Stop callbacks before destroying the node; failed shutdown can be retried."""
        if self._closed:
            return
        self._base_cancel.set()
        if self._base_lock.acquire(timeout=2.0):
            self._base_lock.release()
        if self._base_used:
            self.stop_base()

        self._closing = True
        stopped = self._io_executor.shutdown(timeout_sec=2.0)
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
            if self._spin_thread.is_alive():
                raise RuntimeError('RobotIO spin thread did not stop; node was not destroyed')
        if not stopped:
            raise RuntimeError('RobotIO executor shutdown timed out; node was not destroyed')
        self.destroy_node()
        self._closed = True

    # observation

    def _missing_observations(self):
        missing = []
        required_joints = (
            tuple(self.cfg['cyclo']['gripper_joint'].values())
            + tuple(self.body_units)
        )

        with self._cache_lock:
            missing += [
                f'joint {joint}'
                for joint in required_joints
                if not self._joint_cache.get(joint)
            ]
            missing += [
                f'camera {camera}: {topic}'
                for camera, topic in self.cameras.items()
                if not self._image_cache.get(camera)
            ]

            if not self._odom_cache:
                missing.append(self.odom_topic)

        base = self.cfg['frames']['base']
        for ee in self.cfg['frames']['ee'].values():
            if not self.tf_buffer.can_transform(base, ee, Time()):
                missing.append(f'tf {base} -> {ee}')
        return missing

    def wait_ready(self, timeout_s=None):
        """Wait for cameras, gripper/head/lift joint states and both hand transforms."""
        if timeout_s is None:
            timeout_s = self.ready_timeout_s
        deadline = time.monotonic() + timeout_s
        while True:
            self._check_running()
            missing = self._missing_observations()
            if not missing:
                return
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f'observations not ready after {timeout_s:.1f} s, '
                    f'missing: {", ".join(missing)}'
                )
            time.sleep(0.1)

    def get_ee_pose(self, arm, timeout_s=None):
        """Return the latest base_link pose in meters and quaternion (x, y, z, w)."""
        if timeout_s is None:
            timeout_s = self.lookup_timeout_s
        base = self.cfg['frames']['base']
        ee = self.cfg['frames']['ee'][arm]
        deadline = time.monotonic() + timeout_s
        while True:
            self._check_running()
            try:
                tf = self.tf_buffer.lookup_transform(base, ee, Time())
            except TransformException as exc:
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        f'no transform {base} -> {ee} within {timeout_s:.1f} s ({exc})') from exc
                time.sleep(0.05)
                continue
            self._require_fresh_stamp(stamp_seconds(tf.header.stamp))
            t, r = tf.transform.translation, tf.transform.rotation
            return {
                'frame': base,
                'x': t.x,
                'y': t.y,
                'z': t.z,
                'quat': [r.x, r.y, r.z, r.w],
                'stamp_s': stamp_seconds(tf.header.stamp),
            }

    def get_arm_base_z(self, timeout_s=None):
        """Return the lift-carried arm base height in base_link, in meters."""
        if timeout_s is None:
            timeout_s = self.lookup_timeout_s
        base = self.cfg['frames']['base']
        arm_base = self.cfg['reach']['frame']
        deadline = time.monotonic() + timeout_s
        while True:
            self._check_running()
            try:
                tf = self.tf_buffer.lookup_transform(base, arm_base, Time())
            except TransformException as exc:
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        f'no transform {base} -> {arm_base} within '
                        f'{timeout_s:.1f} s ({exc})'
                    ) from exc
                time.sleep(0.05)
                continue
            self._require_fresh_stamp(stamp_seconds(tf.header.stamp))
            return {
                'frame': base,
                'link': arm_base,
                'z': tf.transform.translation.z,
                'stamp_s': stamp_seconds(tf.header.stamp),
            }

    def get_gripper(self, arm, timeout_s=None):
        """Gripper joint position in rad from /joint_states, with its stamp and age."""
        if timeout_s is None:
            timeout_s = self.lookup_timeout_s
        joint = self.cfg['cyclo']['gripper_joint'][arm]
        deadline = time.monotonic() + timeout_s
        while True:
            self._check_running()
            with self._cache_lock:
                history = self._joint_cache.get(joint)
                entry = history[-1] if history else None
            if entry is not None:
                self._require_fresh_stamp(entry['stamp_s'])

                return {
                    'joint': joint,
                    'position_rad': entry['position'],
                    'stamp_s': entry['stamp_s'],
                    'age_s': time.monotonic() - entry['received']
                }
            if time.monotonic() >= deadline:
                raise RuntimeError(f'no /joint_states value for {joint} within {timeout_s:.1f} s')
            time.sleep(0.05)

    def get_body_state(self, group=None, timeout_s=None, after_received=None):
        """Read recent head/lift samples from the joint-state cache."""
        if timeout_s is None:
            timeout_s = self.lookup_timeout_s

        names = (
            tuple(self.body_units)
            if group is None
            else self.body_groups[group]
        )

        deadline = time.monotonic() + float(timeout_s)

        while True:
            self._check_running()
            now = time.monotonic()

            with self._cache_lock:
                samples = {}
                for joint in names:
                    history = self._joint_cache.get(joint)
                    if history:
                        samples[joint] = dict(history[-1])

            ready = all(
                joint in samples
                and math.isfinite(samples[joint]['position'])
                and now - samples[joint]['received'] <= self.lookup_timeout_s
                and (
                    after_received is None
                    or samples[joint]['received'] > after_received
                )
                for joint in names
            )

            if ready:
                for joint in names:
                    self._require_fresh_stamp(samples[joint]['stamp_s'])
                return {
                    joint: {
                        'position': float(samples[joint]['position']),
                        'unit': self.body_units[joint],
                        'stamp_s': float(samples[joint]['stamp_s']),
                        'age_s': now - samples[joint]['received'],
                    }
                    for joint in names
                }

            if now >= deadline:
                raise TimeoutError(
                    f'no recent joint-state samples for {names}'
                )

            time.sleep(min(0.02, max(0.0, deadline - now)))

    def get_base_state(self, timeout_s=None, after_received=None):
        """Latest odom sample that is fresh in wall time. Pose is in the odom frame."""
        if timeout_s is None:
            timeout_s = self.lookup_timeout_s
        deadline = time.monotonic() + float(timeout_s)
        max_age_s = self.base_cfg['odom_max_age_s']

        while True:
            self._check_running()
            now = time.monotonic()
            with self._cache_lock:
                sample = dict(self._odom_cache[-1]) if self._odom_cache else None

            if (
                sample is not None
                and now - sample['received'] <= max_age_s
                and all(math.isfinite(sample[key]) for key in BASE_KEYS)
                and (after_received is None or sample['received'] > after_received)
            ):
                sample['age_s'] = now - sample['received']
                return sample

            if now >= deadline:
                raise TimeoutError(f'no fresh {self.odom_topic} sample')
            time.sleep(min(0.02, max(0.0, deadline - now)))

    def get_runtime_clock(self):
        """ROS timestamps use system time on the physical robot."""
        self._check_running()

        if self.get_parameter('use_sim_time').value:
            raise RuntimeError(
                'Physical RobotIO requires use_sim_time=False'
            )

        return {
            'kind': 'system',
            'now_s': self.get_clock().now().nanoseconds * 1e-9,
        }

    def _require_fresh_stamp(self, stamp_s):
        """Reject stale data or timestamps from a different clock."""
        stamp_s = float(stamp_s)
        limits = self.cfg['observation_guard']

        now_s = self.get_runtime_clock()['now_s']
        age_s = now_s - stamp_s

        if (
            not math.isfinite(stamp_s)
            or stamp_s <= 0.0
            or not (
                -float(limits['max_future_stamp_s'])
                <= age_s
                <= float(limits['max_state_lag_s'])
            )
        ):
            raise RuntimeError(
                f'stale or incompatible timestamp: age={age_s:.3f} s'
            )

    def get_state(self):
        """Return robot state with independent source timestamps."""
        arms = {}
        for arm in self.cfg['frames']['ee']:
            arms[arm] = {'ee': self.get_ee_pose(arm), 'gripper': self.get_gripper(arm)}
        return {
            'frame': self.cfg['frames']['base'],
            'arms': arms,
            'body': self.get_body_state(),
            'base': self.get_base_state(),
            'clock': self.get_runtime_clock(),
        }

    @staticmethod
    def _nearest(history, stamp_s):
        """Entry of history whose stamp is closest to stamp_s, or None if empty."""
        return min(history, key=lambda entry: abs(entry['stamp_s'] - stamp_s), default=None)

    def _state_at(self, stamp_s, joints):
        """Returns (problem, arms). problem is None when every value was found."""
        base = self.cfg['frames']['base']
        tol = self.snapshot_tolerance_s
        arms = {}
        for arm, ee in self.cfg['frames']['ee'].items():
            try:
                tf = self.tf_buffer.lookup_transform(base, ee, Time(seconds=stamp_s))
            except TransformException as exc:
                return (
                    f'tf {base} -> {ee} at {stamp_s:.3f} '
                    f'not available yet ({type(exc).__name__})',
                    None,
                )
            joint = self.cfg['cyclo']['gripper_joint'][arm]
            sample = self._nearest(joints[arm], stamp_s)
            if sample is None or abs(sample['stamp_s'] - stamp_s) > tol:
                return f'no {joint} sample within {tol} s of {stamp_s:.3f}', None
            t, r = tf.transform.translation, tf.transform.rotation
            arms[arm] = {
                'ee': {
                    'frame': base,
                    'x': t.x,
                    'y': t.y,
                    'z': t.z,
                    'quat': [r.x, r.y, r.z, r.w],
                    'stamp_s': stamp_s,
                },
                'gripper': {
                    'joint': joint,
                    'position_rad': sample['position'],
                    'stamp_s': sample['stamp_s'],
                },
            }
        return None, arms

    def _try_snapshot(self, cameras, after_stamp_s):
        tol = self.snapshot_tolerance_s

        with self._cache_lock:
            frames = {
                camera: list(self._image_cache.get(camera, ()))
                for camera in cameras
            }
            joints = {
                arm: list(self._joint_cache.get(joint, ()))
                for arm, joint
                in self.cfg['cyclo']['gripper_joint'].items()
            }
            body_histories = {
                joint: list(self._joint_cache.get(joint, ()))
                for joint in self.body_units
            }
            odom_history = list(self._odom_cache)

        waiting = [
            camera for camera in cameras
            if not frames[camera]
        ]
        if waiting:
            return f'no frame yet from {waiting}', None, None, None

        problem = 'no compatible camera/state timestamps'

        candidates = sorted(
            {frame['stamp_s'] for frame in frames[cameras[0]]},
            reverse=True,
        )

        for stamp_s in candidates:
            if after_stamp_s is not None and stamp_s <= after_stamp_s:
                continue

            chosen = {}

            for camera in cameras:
                frame = self._nearest(frames[camera], stamp_s)

                if abs(frame['stamp_s'] - stamp_s) > tol:
                    problem = f'{camera}: timestamp mismatch'
                    break

                # Every image must be newer than the completed motion.
                if (
                    after_stamp_s is not None
                    and frame['stamp_s'] <= after_stamp_s
                ):
                    problem = (f'{camera}: image predates completed motion')
                    break

                chosen[camera] = frame

            if len(chosen) != len(cameras):
                continue

            problem, arms = self._state_at(stamp_s, joints)
            if problem is not None:
                continue

            problem, body = self._body_state_at(stamp_s, body_histories)
            if problem is not None:
                continue

            problem, base = self._base_state_at(stamp_s, odom_history)
            if problem is not None:
                continue

            return None, stamp_s, chosen, {
                'arms': arms,
                'body': body,
                'base': base,
            }

        return problem, None, None, None

    def get_snapshot(self, cameras, after_stamp_s=None, timeout_s=None):
        """Return the latest timestamp-matched camera and robot state.

        Match within snapshot_tolerance_s; images must exceed after_stamp_s if set.
        """
        cameras = list(cameras)
        unknown = [c for c in cameras if c not in self.cameras]
        if not cameras or unknown:
            raise KeyError(f'cameras must be some of {sorted(self.cameras)}, got {cameras}')
        if timeout_s is None:
            timeout_s = self.image_timeout_s
        deadline = time.monotonic() + timeout_s
        while True:
            self._check_running()
            problem, stamp_s, frames, state = self._try_snapshot(
                cameras,
                after_stamp_s,
            )
            if problem is None:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError(f'no consistent snapshot within {timeout_s:.1f} s: {problem}')
            time.sleep(0.02)

        arms = state['arms']
        body = state['body']
        base = state['base']

        # Encode matched frames outside the cache lock.
        now = time.monotonic()
        images = {
            camera: self._encode_frame(camera, frame, now)
            for camera, frame in frames.items()
        }

        encoded_at = time.monotonic()
        for camera, frame in frames.items():
            images[camera]['receive_age_s'] = (
                encoded_at - frame['received']
            )

        offsets = [
            abs(image['stamp_s'] - stamp_s)
            for image in images.values()
        ]
        offsets += [
            abs(arm['gripper']['stamp_s'] - stamp_s)
            for arm in arms.values()
        ]
        offsets += [
            abs(joint['stamp_s'] - stamp_s)
            for joint in body.values()
        ]
        offsets.append(abs(base['stamp_s'] - stamp_s))

        return {
            'stamp_s': stamp_s,
            'frame': self.cfg['frames']['base'],
            'arms': arms,
            'body': body,
            'base': base,
            'images': images,
            'max_offset_s': max(offsets),
            'clock': self.get_runtime_clock(),
        }

    def _encode_frame(self, camera, frame, now):
        msg = frame['msg']
        image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ok, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            raise RuntimeError(f'{camera}: JPEG encoding failed')
        return {
            'camera': camera,
            'jpeg': buffer.tobytes(),
            'topic': self.cameras[camera],
            'frame_id': msg.header.frame_id,
            'stamp_s': frame['stamp_s'],
            'receive_age_s': now - frame['received'],
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding,
        }

    # Cyclo command publication

    def _wait_for_subscriber(self, publisher, timeout_s=5.0):
        """Wait in wall time until Cyclo subscribes to the publisher"""
        deadline = time.monotonic() + timeout_s

        while True:
            self._check_running()

            if publisher.get_subscription_count() > 0:
                return True
            if time.monotonic() >= deadline:
                return False

            time.sleep(0.05)

    @staticmethod
    def _duration(seconds):
        """Convert non-negative seconds to builtin_interfaces/Duration"""
        seconds = float(seconds)

        if not math.isfinite(seconds) or seconds < 0.0:
            raise ValueError(
                f'duration must be finite and non-negative, got {seconds!r}'
            )

        sec = int(seconds)
        nanosec = int(round((seconds - sec) * 1e9))

        # Handle nanosecond rounding overflow.
        if nanosec == 1_000_000_000:
            sec += 1
            nanosec = 0

        return Duration(sec=sec, nanosec=nanosec)

    def send_pose(self, arm, target, move_time_s=None):
        """Publish a MoveL goal expressed in SG2 base_link coordinates"""
        if move_time_s is None:
            move_time_s = self.cfg['cyclo']['move_time_s']

        publisher = self.pose_pub[arm]

        if not self._wait_for_subscriber(publisher):
            raise RuntimeError(
                f'nothing subscribes to {publisher.topic_name}; '
                'is Cyclo running?'
            )

        msg = MoveL()
        msg.pose.header.frame_id = self.cfg['frames']['base']
        msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Apply the SG2 solver offset once.
        msg.pose.pose.position.x = float(
            target['x'] + self.cfg['cyclo']['x_offset_m']
        )
        msg.pose.pose.position.y = float(target['y'])
        msg.pose.pose.position.z = float(target['z'])

        qx, qy, qz, qw = rpy_to_quat(
            target['roll'],
            target['pitch'],
            target['yaw'],
        )
        msg.pose.pose.orientation.x = float(qx)
        msg.pose.pose.orientation.y = float(qy)
        msg.pose.pose.orientation.z = float(qz)
        msg.pose.pose.orientation.w = float(qw)

        msg.time_from_start = self._duration(move_time_s)
        publisher.publish(msg)

        return {
            'quat': [qx, qy, qz, qw],
            'sent_x': msg.pose.pose.position.x,
        }

    def send_gripper(self, arm, value, move_time_s=1.0):
        """Publish a normalized 0-open/1-closed gripper command"""
        lo, hi = self.cfg['limits']['gripper_rad']
        target_rad = float(lo + value * (hi - lo))
        publisher = self.gripper_pub[arm]

        if not self._wait_for_subscriber(publisher):
            raise RuntimeError(
                f'nothing subscribes to {publisher.topic_name}; '
                'is Cyclo running?'
            )

        trajectory = JointTrajectory()
        trajectory.joint_names = [
            self.cfg['cyclo']['gripper_joint'][arm]
        ]

        point = JointTrajectoryPoint()
        point.positions = [target_rad]
        point.time_from_start = self._duration(move_time_s)
        trajectory.points = [point]

        publisher.publish(trajectory)
        return target_rad

    def send_body(self, group, targets):
        """Publish one absolute trajectory for the head or lift."""
        names = self.body_groups[group]

        if set(targets) != set(names):
            raise ValueError(
                f'{group} requires exactly these joints: {names}'
            )

        positions = []
        for joint in names:
            raw = targets[joint]
            if isinstance(raw, bool):
                raise ValueError(f'{joint} must be numeric')

            value = float(raw)
            lower, upper = self.body_limits[joint]

            if not math.isfinite(value) or not lower <= value <= upper:
                raise ValueError(
                    f'{joint}={value} is outside [{lower}, {upper}]'
                )

            positions.append(value)

        publisher = self.body_pub[group]
        if not self._wait_for_subscriber(publisher):
            raise RuntimeError(
                f'nothing subscribes to {publisher.topic_name}'
            )

        trajectory = JointTrajectory()
        trajectory.joint_names = list(names)

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = self._duration(self.body_move_time_s)
        trajectory.points = [point]

        # A zero stamp starts the trajectory on receipt.
        sent_at = time.monotonic()
        publisher.publish(trajectory)
        return sent_at

    # mobile base

    def base_command_problem(self):
        """Check cmd_vel peers; graph discovery may lag and provides no lock."""
        if self.cmd_vel_pub.get_subscription_count() < 1:
            return f'nothing subscribes to {self.cmd_vel_topic}'
        publishers = self.count_publishers(self.cmd_vel_topic)
        if publishers > 1:
            return f'{publishers - 1} other publisher(s) on {self.cmd_vel_topic}'
        return None

    def _publish_twist(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_vel_pub.publish(msg)

    def stop_base(self):
        """Publish zero velocity ZERO_REPEATS times. Returns the last error or None."""
        error = None
        for _ in range(ZERO_REPEATS):
            try:
                self._publish_twist(0.0, 0.0, 0.0)
            except Exception as exc:
                error = f'{type(exc).__name__}: {exc}'
            time.sleep(0.02)
        return error

    @staticmethod
    def _profile_scale(elapsed_s, duration_s, ramp_s, floor):
        """Command scale in [floor, 1]: ramp up, hold, ramp down, all inside duration_s."""
        ramp_s = min(ramp_s, duration_s / 2.0)
        edge = min(elapsed_s, duration_s - elapsed_s)
        if edge >= ramp_s:
            return 1.0
        return floor + (1.0 - floor) * max(0.0, edge) / ramp_s

    def send_base(self, vx, vy, wz, duration_s):
        """Run a base velocity profile, then publish zero.

        Raises only before publication; later failures return command_sent=True.
        Confirm rest with wait_until_base_stopped().
        """
        cfg = self.base_cfg
        command = {'vx': float(vx), 'vy': float(vy), 'wz': float(wz)}
        duration_s = float(duration_s)

        if not all(math.isfinite(v) for v in (*command.values(), duration_s)):
            raise ValueError('base command must be finite')
        if not 0.0 < duration_s <= cfg['max_duration_s']:
            raise ValueError(f'duration_s must be in (0, {cfg["max_duration_s"]}]')
        if (math.hypot(command['vx'], command['vy']) > cfg['max_lin_mps']
                or abs(command['wz']) > cfg['max_ang_rps']):
            raise ValueError('base command exceeds the speed limits')

        if not self._base_lock.acquire(blocking=False):
            raise RuntimeError('another base command is running')
        try:
            return self._run_base(command, duration_s)
        finally:
            self._base_lock.release()

    def _run_base(self, command, duration_s):
        cfg = self.base_cfg

        # Pre-publication failures send no command.
        if self._base_cancel.is_set():
            raise RuntimeError('RobotIO is closing')
        problem = self.base_command_problem()
        if problem:
            raise RuntimeError(problem)
        start = self.get_base_state()

        # Keep active axes at or above the deadband.
        moving = [
            (abs(value), cfg['deadband_rps' if axis == 'wz' else 'deadband_mps'])
            for axis, value in command.items() if value != 0.0
        ]
        if not moving:
            raise ValueError('base command is all zero')
        floor = min(1.0, max((deadband + 1e-6) / size for size, deadband in moving))
        moves_linear = command['vx'] != 0.0 or command['vy'] != 0.0
        moves_angular = command['wz'] != 0.0

        period_s = 1.0 / cfg['publish_hz']
        path_m = path_rad = 0.0
        previous = start
        window = deque()     # (monotonic, path_m, path_rad)
        outcome, detail = 'completed', None
        self._base_used = True
        started = time.monotonic()
        tick = 0

        try:
            while True:
                now = time.monotonic()
                elapsed_s = now - started
                if elapsed_s >= duration_s:
                    break
                if self._base_cancel.is_set():
                    outcome, detail = 'cancelled', 'RobotIO is closing'
                    break

                scale = self._profile_scale(elapsed_s, duration_s, cfg['ramp_s'], floor)
                self._publish_twist(*(command[axis] * scale for axis in ('vx', 'vy', 'wz')))

                problem = self.base_command_problem()
                if problem:
                    outcome, detail = 'ownership', problem
                    break

                with self._cache_lock:
                    latest = dict(self._odom_cache[-1]) if self._odom_cache else None
                if latest is None or now - latest['received'] > cfg['odom_max_age_s']:
                    outcome, detail = 'odom_lost', f'no fresh {self.odom_topic}'
                    break
                if latest['received'] > previous['received']:
                    path_m += math.hypot(latest['x'] - previous['x'], latest['y'] - previous['y'])
                    path_rad += abs(wrap_angle(latest['yaw'] - previous['yaw']))
                    previous = latest

                if (path_m > cfg['max_step_m'] * cfg['path_margin']
                        or path_rad > cfg['max_step_rad'] * cfg['path_margin']):
                    outcome, detail = 'path_limit', f'odom path {path_m:.3f} m, {path_rad:.3f} rad'
                    break

                # Check progress after the steering grace period.
                window.append((now, path_m, path_rad))
                while len(window) > 1 and now - window[1][0] >= cfg['progress_window_s']:
                    window.popleft()
                if (elapsed_s >= cfg['steer_grace_s']
                        and now - window[0][0] >= cfg['progress_window_s']):
                    gained_m = path_m - window[0][1]
                    gained_rad = path_rad - window[0][2]
                    if ((moves_linear and gained_m < cfg['progress_min_m'])
                            or (moves_angular and gained_rad < cfg['progress_min_rad'])):
                        outcome = 'stalled'
                        detail = (
                            f'{gained_m:.3f} m, {gained_rad:.3f} rad '
                            f'in {cfg["progress_window_s"]} s'
                        )
                        break

                tick += 1
                time.sleep(max(0.0, started + tick * period_s - time.monotonic()))
        except Exception as exc:
            outcome, detail = 'error', f'{type(exc).__name__}: {exc}'
        finally:
            stop_error = self.stop_base()
            stopped_at = time.monotonic()

        return {
            'command_sent': True,
            'outcome': outcome,
            'detail': detail,
            'stop_error': stop_error,
            'stopped_at': stopped_at,
            'commanded': command,
            'duration_s': duration_s,
            'elapsed_s': stopped_at - started,
            'start': start,
            'path_m': path_m,
            'path_rad': path_rad,
        }

    def wait_until_base_stopped(self, after_received, timeout_s=None):
        """Confirm base rest from fresh odometry; return rested=False on failure."""
        cfg = self.base_cfg
        if timeout_s is None:
            timeout_s = cfg['rest_timeout_s']
        started = time.monotonic()
        deadline = started + float(timeout_s)

        def moved(old, new):
            return (math.hypot(new[0] - old[0], new[1] - old[1]) >= cfg['rest_position_m']
                    or abs(wrap_angle(new[2] - old[2])) >= cfg['rest_yaw_rad'])

        detector = StallDetector(cfg['rest_window_s'], 0.0, started, moved)
        sample = None

        def result(rested, reason):
            return {
                'rested': rested,
                'reason': reason,
                'state': sample,
                'elapsed_s': time.monotonic() - started,
            }

        try:
            while True:
                now = time.monotonic()
                if now >= deadline:
                    return result(False, f'base did not come to rest within {timeout_s} s')
                try:
                    sample = self.get_base_state(
                        timeout_s=min(cfg['odom_max_age_s'], deadline - now),
                        after_received=after_received)
                except TimeoutError:
                    return result(False, f'no fresh {self.odom_topic} while confirming rest')
                after_received = sample['received']

                slow = (math.hypot(sample['vx'], sample['vy']) < cfg['rest_speed_mps']
                        and abs(sample['wz']) < cfg['rest_speed_rps'])
                still = detector.update(time.monotonic(), sample['stamp_s'],
                                        (sample['x'], sample['y'], sample['yaw']))
                if slow and still:
                    return result(True, 'base at rest')
        except Exception as exc:
            return result(False, f'{type(exc).__name__}: {exc}')

    def wait_until_arrived(self, arm, target, target_quat, timeout_s=None):
        """Wait for hand rest after the trajectory time.

        Report arrived inside tolerance or stopped outside it.
        """
        tolerance = self.cfg['tolerance']
        if timeout_s is None:
            timeout_s = float(tolerance['timeout_s'])
        timeout_s = float(timeout_s)
        if not math.isfinite(timeout_s) or timeout_s < 0.0:
            raise ValueError(
                f'timeout_s must be finite and non-negative, got {timeout_s!r}'
            )

        started = time.monotonic()
        deadline = started + timeout_s
        stall_s = float(tolerance['stall_s'])
        rest = StallDetector(
            stall_s,
            float(self.cfg['cyclo']['move_time_s']) + stall_s,
            started,
            lambda old, new: (
                math.dist(old[0], new[0]) > float(tolerance['stall_position_m'])
                or quat_angle(old[1], new[1]) > float(tolerance['stall_orientation_deg'])
            ),
        )

        while True:
            remaining = max(0.0, deadline - time.monotonic())

            pose = self.get_ee_pose(
                arm, timeout_s=min(self.lookup_timeout_s, remaining)
            )
            position_error = math.dist(
                (pose['x'], pose['y'], pose['z']),
                (target['x'], target['y'], target['z']),
            )
            orientation_error = quat_angle(
                pose['quat'],
                target_quat,
            )
            inside = (
                position_error <= float(tolerance['position_m'])
                and orientation_error <= float(tolerance['orientation_deg'])
            )

            now = time.monotonic()
            at_rest = rest.update(
                now,
                pose['stamp_s'],
                ((pose['x'], pose['y'], pose['z']), pose['quat']),
            )

            if at_rest or now >= deadline:
                return {
                    'arrived': at_rest and inside,
                    'stopped': at_rest and not inside,
                    'pose': pose,
                    'position_error_m': position_error,
                    'orientation_error_deg': orientation_error,
                    'elapsed_s': now - started,
                }
            time.sleep(min(0.05, max(0.0, deadline - now)))

    def wait_until_gripper_arrived(self, arm, target_rad, timeout_s=None, move_time_s=1.0):
        """Wait for gripper rest after move_time_s.

        Report arrived inside tolerance or stopped outside it.
        """
        tolerance = self.cfg['tolerance']
        if timeout_s is None:
            timeout_s = float(tolerance['timeout_s'])
        timeout_s = float(timeout_s)
        target_rad = float(target_rad)

        if not math.isfinite(target_rad):
            raise ValueError(
                f'target_rad must be finite, got {target_rad!r}'
            )
        if not math.isfinite(timeout_s) or timeout_s < 0.0:
            raise ValueError(
                f'timeout_s must be finite and non-negative, got {timeout_s!r}'
            )

        started = time.monotonic()
        deadline = started + timeout_s
        stall_s = float(tolerance['stall_s'])
        rest = StallDetector(
            stall_s,
            float(move_time_s) + stall_s,
            started,
            lambda old, new: abs(old - new) > float(tolerance['stall_gripper_rad']),
        )

        while True:
            remaining = max(0.0, deadline - time.monotonic())

            measured = self.get_gripper(
                arm,
                timeout_s=min(self.lookup_timeout_s, remaining),
            )

            error_rad = abs(
                measured['position_rad'] - target_rad
            )
            inside = error_rad <= tolerance['gripper_rad']
            now = time.monotonic()
            at_rest = rest.update(now, measured['stamp_s'], measured['position_rad'])

            if at_rest or now >= deadline:
                return {
                    'arrived': at_rest and inside,
                    'stopped': at_rest and not inside,
                    'gripper': measured,
                    'error_rad': error_rad,
                    'elapsed_s': now - started,
                }

            time.sleep(min(0.05, max(0.0, deadline - now)))

    def wait_until_body_arrived(
        self,
        group,
        targets,
        after_received,
        timeout_s=None,
        settle_s=None,
    ):
        """Wait for fresh measured positions to settle inside tolerance"""
        tolerance = self.cfg['tolerance']

        if timeout_s is None:
            timeout_s = float(tolerance['timeout_s'])
        if settle_s is None:
            settle_s = float(tolerance['settle_s'])

        timeout_s = float(timeout_s)
        settle_s = float(settle_s)

        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError('timeout_s must be finite and positive')
        if not math.isfinite(settle_s) or settle_s < 0.0:
            raise ValueError('settle_s must be finite and non-negative')

        started = time.monotonic()
        deadline = started + timeout_s
        inside_since = None
        previous_stamps = None

        while True:
            now = time.monotonic()
            remaining = max(0.0, deadline - now)

            measured = self.get_body_state(
                group=group,
                timeout_s=min(self.lookup_timeout_s, remaining),
                after_received=after_received,
            )

            errors = {
                joint: abs(measured[joint]['position'] - targets[joint])
                for joint in self.body_groups[group]
            }

            inside = all(
                errors[joint] <= self.body_tolerances[joint]
                for joint in errors
            )

            stamps = {
                joint: measured[joint]['stamp_s']
                for joint in errors
            }

            advanced = (
                previous_stamps is None
                or all(
                    stamps[joint] > previous_stamps[joint]
                    for joint in stamps
                )
            )

            now = time.monotonic()
            arrived = False

            if not inside:
                inside_since = None

            if advanced:
                previous_stamps = stamps

                if inside:
                    if inside_since is None:
                        inside_since = now
                    arrived = now - inside_since >= settle_s

            result = {
                'arrived': arrived,
                'joints': measured,
                'errors': errors,
                'elapsed_s': now - started,
            }

            if now >= deadline:
                result['arrived'] = False
                return result

            if arrived:
                return result

            time.sleep(min(0.02, max(0.0, deadline - now)))

    def _body_state_at(self, stamp_s, histories):
        """Match head/lift samples to the selected camera timestamp."""
        body = {}

        for joint, unit in self.body_units.items():
            sample = self._nearest(histories[joint], stamp_s)

            if (
                sample is None
                or not math.isfinite(sample['position'])
                or abs(sample['stamp_s'] - stamp_s)
                > self.snapshot_tolerance_s
            ):
                return (
                    f'no matching {joint} sample at {stamp_s:.3f}',
                    None,
                )

            body[joint] = {
                'position': float(sample['position']),
                'unit': unit,
                'stamp_s': float(sample['stamp_s']),
            }

        return None, body

    def _base_state_at(self, stamp_s, history):
        """Match an odom sample to the selected camera timestamp."""
        sample = self._nearest(history, stamp_s)
        if (
            sample is None
            or abs(sample['stamp_s'] - stamp_s) > self.snapshot_tolerance_s
            or not all(math.isfinite(sample[key]) for key in BASE_KEYS)
        ):
            return f'no matching {self.odom_topic} sample at {stamp_s:.3f}', None
        return None, {
            key: sample[key]
            for key in ('frame', 'child_frame', *BASE_KEYS, 'stamp_s')
        }
