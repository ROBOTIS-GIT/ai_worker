#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
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
# Author: Woojin Wie

import math

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
import tf2_ros
from tf2_ros import TransformException
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker, MarkerArray


class HeadEefTracker(Node):
    """Node to control head to point camera at center of two end effectors."""

    def __init__(self):
        super().__init__('head_eef_tracker')

        # Declare parameters
        self.declare_parameter('update_rate', 100.0)  # Hz
        self.declare_parameter('target_frame', 'arm_base_link')
        self.declare_parameter('eef_l_link', 'end_effector_l_link')
        self.declare_parameter('eef_r_link', 'end_effector_r_link')
        self.declare_parameter('camera_link', 'zedm_camera_link')
        self.declare_parameter('head_joint1_name', 'head_joint1')
        self.declare_parameter('head_joint2_name', 'head_joint2')
        self.declare_parameter(
            'joint_trajectory_topic',
            '/leader/joystick_controller_left/joint_trajectory'
        )
        self.declare_parameter('robot_description_topic', '/robot_description')
        self.declare_parameter('visualization_topic', '~/head_target_visualization')
        self.declare_parameter('enable_visualization', True)

        # Load parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.target_frame = self.get_parameter('target_frame').value
        self.eef_l_link = self.get_parameter('eef_l_link').value
        self.eef_r_link = self.get_parameter('eef_r_link').value
        self.camera_link = self.get_parameter('camera_link').value
        self.head_joint1_name = self.get_parameter('head_joint1_name').value
        self.head_joint2_name = self.get_parameter('head_joint2_name').value
        self.joint_trajectory_topic = self.get_parameter('joint_trajectory_topic').value
        self.robot_description_topic = self.get_parameter('robot_description_topic').value
        self.visualization_topic = self.get_parameter('visualization_topic').value
        self.enable_visualization = self.get_parameter('enable_visualization').value

        # URDF data (will be populated from /robot_description)
        self.urdf_robot = None
        self.head_joint1_pos = None
        self.head_joint1_limit_lower = None
        self.head_joint1_limit_upper = None
        self.head_joint2_limit_lower = None
        self.head_joint2_limit_upper = None
        self.head_joint1_axis = None
        self.head_joint2_axis = None
        self.urdf_loaded = False

        # Debug logging counter
        self.debug_counter = 0
        self.debug_log_interval = 10  # Log every N updates
        self.enable_debug_logging = False
        self.enable_visualization = True

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriber for robot_description
        # Use TRANSIENT_LOCAL durability to receive the last published message
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.robot_description_sub = self.create_subscription(
            String,
            self.robot_description_topic,
            self.robot_description_callback,
            qos_profile
        )

        # Publisher for joint trajectory
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            self.joint_trajectory_topic,
            10
        )

        # Publisher for visualization markers
        if self.enable_visualization:
            self.marker_pub = self.create_publisher(
                MarkerArray,
                self.visualization_topic,
                10
            )
        else:
            self.marker_pub = None

        # Joint names (will be set from parameters)
        self.joint_names = [self.head_joint1_name, self.head_joint2_name]

        # Marker ID counter
        self.marker_id_counter = 0

        # Timer for periodic updates (will be created after URDF is loaded)
        self.timer = None

        self.get_logger().info('Head EEF Tracker initialized')
        self.get_logger().info(f'  Update rate: {self.update_rate} Hz')
        self.get_logger().info(f'  Target frame: {self.target_frame}')
        self.get_logger().info(f'  EEF L link: {self.eef_l_link}')
        self.get_logger().info(f'  EEF R link: {self.eef_r_link}')
        self.get_logger().info(f'  Camera link: {self.camera_link}')
        self.get_logger().info(f'  Publishing to: {self.joint_trajectory_topic}')
        if self.enable_visualization:
            self.get_logger().info(f'  Visualization: {self.visualization_topic}')
        self.get_logger().info(
            f'  Waiting for robot_description on: '
            f'{self.robot_description_topic}'
        )

    def robot_description_callback(self, msg):
        """Handle robot_description topic callback."""
        if self.urdf_loaded:
            return  # Already loaded

        try:
            # Parse URDF
            self.urdf_robot = URDF.from_xml_string(msg.data)

            # Extract head joint information
            self.parse_urdf()

            if self.urdf_loaded:
                self.get_logger().info('=' * 60)
                self.get_logger().info('URDF loaded successfully')
                self.get_logger().info(f'  head_joint1 ({self.head_joint1_name}) - PITCH:')
                self.get_logger().info(f'    Position (xyz): {self.head_joint1_pos}')
                self.get_logger().info(f'    Axis: {self.head_joint1_axis}')
                self.get_logger().info(
                    f'    Limits: [{self.head_joint1_limit_lower:.4f}, '
                    f'{self.head_joint1_limit_upper:.4f}]'
                )
                self.get_logger().info(f'  head_joint2 ({self.head_joint2_name}) - YAW:')
                self.get_logger().info(f'    Axis: {self.head_joint2_axis}')
                self.get_logger().info(
                    f'    Limits: [{self.head_joint2_limit_lower:.4f}, '
                    f'{self.head_joint2_limit_upper:.4f}]'
                )
                self.get_logger().info('=' * 60)

                # Create timer now that URDF is loaded
                if self.timer is None:
                    timer_period = 1.0 / self.update_rate
                    self.timer = self.create_timer(timer_period, self.timer_callback)
            else:
                self.get_logger().warn('Failed to parse URDF, will retry on next message')

        except Exception as e:
            self.get_logger().error(f'Error parsing robot_description: {e}')

    def parse_urdf(self):
        """Parse URDF to extract head joint information."""
        try:
            head_joint1 = self.urdf_robot.joint_map.get(self.head_joint1_name)
            head_joint2 = self.urdf_robot.joint_map.get(self.head_joint2_name)

            if head_joint1 is None or head_joint2 is None:
                self.get_logger().error(
                    f'Could not find joints: {self.head_joint1_name}, '
                    f'{self.head_joint2_name}'
                )
                return

            # Extract head_joint1 origin position
            if head_joint1.origin is not None and head_joint1.origin.xyz is not None:
                xyz = head_joint1.origin.xyz
                # Handle both list and tuple
                if isinstance(xyz, (list, tuple)) and len(xyz) >= 3:
                    self.head_joint1_pos = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
                else:
                    self.head_joint1_pos = (0.0, 0.0, 0.0)
            else:
                self.head_joint1_pos = (0.0, 0.0, 0.0)

            # Extract joint limits
            if head_joint1.limit is not None:
                self.head_joint1_limit_lower = head_joint1.limit.lower
                self.head_joint1_limit_upper = head_joint1.limit.upper
            else:
                self.get_logger().warn(f'No limits found for {self.head_joint1_name}')
                return

            if head_joint2.limit is not None:
                self.head_joint2_limit_lower = head_joint2.limit.lower
                self.head_joint2_limit_upper = head_joint2.limit.upper
            else:
                self.get_logger().warn(f'No limits found for {self.head_joint2_name}')
                return

            # Extract joint axes for debugging
            if head_joint1.axis is not None:
                self.head_joint1_axis = tuple(head_joint1.axis)
            else:
                self.head_joint1_axis = None

            if head_joint2.axis is not None:
                self.head_joint2_axis = tuple(head_joint2.axis)
            else:
                self.head_joint2_axis = None

            self.urdf_loaded = True

        except Exception as e:
            self.get_logger().error(f'Error parsing URDF: {e}')

    def get_transform(self, target_frame, source_frame):
        """Get transform from source_frame to target_frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
            return transform
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {source_frame} to {target_frame}: {ex}'
            )
            return None

    def calculate_head_angles(self, target_point, head_joint1_pos, debug=False):
        """
        Calculate head_joint1 (pitch) and head_joint2 (yaw) angles to point camera at target.

        Args:
            target_point: Point in arm_base_link frame (x, y, z)
            head_joint1_pos: Position of head_joint1 origin in arm_base_link frame (x, y, z)
            debug: If True, log detailed calculation steps

        Returns
        -------
            (head_joint1_angle, head_joint2_angle) or (None, None) if invalid

        """
        # Calculate vector from head_joint1 origin to target
        dx = target_point[0] - head_joint1_pos[0]
        dy = target_point[1] - head_joint1_pos[1]
        dz = target_point[2] - head_joint1_pos[2]

        if debug:
            self.get_logger().info('  Vector calculation:')
            self.get_logger().info(
                f'    Target: ({target_point[0]:.4f}, {target_point[1]:.4f}, '
                f'{target_point[2]:.4f})'
            )
            self.get_logger().info(
                f'    head_joint1_pos: ({head_joint1_pos[0]:.4f}, '
                f'{head_joint1_pos[1]:.4f}, {head_joint1_pos[2]:.4f})'
            )
            self.get_logger().info(f'    Vector (dx, dy, dz): ({dx:.4f}, {dy:.4f}, {dz:.4f})')

        # Calculate distance in XY plane (for yaw/joint2)
        # head_joint2 rotates around Z axis, so it affects the XY plane
        r_xy = math.sqrt(dx**2 + dy**2)

        if debug:
            self.get_logger().info(f'    Distance in XY plane (r_xy): {r_xy:.4f}')

        # Calculate yaw angle (head_joint2 rotates around Z axis) - YAW
        # This is the angle in the XY plane from X axis
        # When dy is essentially 0, yaw should be 0 (straight ahead/back), not ±180 degrees
        if abs(dy) < 1e-4:  # If dy is very small, target is centered in Y direction
            yaw = 0.0
            if debug:
                self.get_logger().info(f'    dy≈0 ({dy:.6f}), setting yaw to 0 (centered)')
        elif r_xy > 1e-6:  # Avoid division by zero
            yaw = math.atan2(dy, dx)
        else:
            yaw = 0.0

        if debug:
            self.get_logger().info(
                f'    Raw yaw (head_joint2): {math.degrees(yaw):.2f}° '
                f'({yaw:.4f} rad)'
            )

        # Calculate pitch angle (head_joint1 rotates around Y axis) - PITCH
        # This rotates in the XZ plane
        # We need the angle from the horizontal (XZ plane) to the target
        # Note: Joint convention may be inverted - if looking up when should look down,
        # we need to negate the angle
        if r_xy > 1e-6:  # Avoid division by zero
            # Pitch is the angle from the XZ plane (horizontal) to the target
            # Positive dz means target is above, negative means below
            # Invert sign to match joint rotation convention
            pitch_raw = math.atan2(dz, r_xy)
            pitch = -pitch_raw
        else:
            # If r_xy is very small, target is directly above/below
            if abs(dz) > 1e-6:
                pitch_raw = math.copysign(math.pi / 2, dz)
                pitch = -pitch_raw  # ±90 degrees, inverted
            else:
                pitch_raw = 0.0
                pitch = 0.0

        if debug:
            self.get_logger().info(
                f'    Raw pitch (head_joint1): {math.degrees(pitch_raw):.2f}° '
                f'({pitch_raw:.4f} rad)'
            )
            self.get_logger().info(
                f'    Inverted pitch: {math.degrees(pitch):.2f}° '
                f'({pitch:.4f} rad)'
            )

        # Clamp angles to joint limits
        head_joint1_angle = max(
            self.head_joint1_limit_lower,
            min(self.head_joint1_limit_upper, pitch)
        )
        head_joint2_angle = max(
            self.head_joint2_limit_lower,
            min(self.head_joint2_limit_upper, yaw)
        )

        if debug:
            self.get_logger().info('  After clamping:')
            self.get_logger().info(
                f'    head_joint1 (pitch): {math.degrees(head_joint1_angle):.2f}° '
                f'({head_joint1_angle:.4f} rad)'
            )
            self.get_logger().info(
                f'    head_joint2 (yaw): {math.degrees(head_joint2_angle):.2f}° '
                f'({head_joint2_angle:.4f} rad)'
            )
            if head_joint1_angle != pitch:
                self.get_logger().warn(
                    f'    WARNING: head_joint1 was clamped! Raw: {pitch:.4f}, '
                    f'Clamped: {head_joint1_angle:.4f}'
                )
            if head_joint2_angle != yaw:
                self.get_logger().warn(
                    f'    WARNING: head_joint2 was clamped! Raw: {yaw:.4f}, '
                    f'Clamped: {head_joint2_angle:.4f}'
                )

        return (head_joint1_angle, head_joint2_angle)

    def create_visualization_markers(self, head_joint1_pos, target_point, pos_l, pos_r):
        """Create visualization markers showing head targeting."""
        if not self.enable_visualization or self.marker_pub is None:
            return

        marker_array = MarkerArray()
        now = self.get_clock().now()

        # Marker 1: Arrow from head_joint1 to target (center point)
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = now.to_msg()
        marker.ns = 'head_target'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Arrow start and end points
        start_point = Point()
        start_point.x = float(head_joint1_pos[0])
        start_point.y = float(head_joint1_pos[1])
        start_point.z = float(head_joint1_pos[2])

        end_point = Point()
        end_point.x = float(target_point[0])
        end_point.y = float(target_point[1])
        end_point.z = float(target_point[2])

        marker.points = [start_point, end_point]

        # Arrow color (green)
        marker.color = ColorRGBA()
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Arrow scale
        marker.scale.x = 0.02  # shaft diameter
        marker.scale.y = 0.04  # head diameter
        marker.scale.z = 0.05  # head length

        marker.lifetime.sec = 1  # 1 second lifetime
        marker_array.markers.append(marker)

        # Marker 2: Sphere at target point (center of end effectors)
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = now.to_msg()
        marker.ns = 'head_target'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(target_point[0])
        marker.pose.position.y = float(target_point[1])
        marker.pose.position.z = float(target_point[2])
        marker.pose.orientation.w = 1.0

        # Sphere color (yellow)
        marker.color = ColorRGBA()
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        # Sphere scale
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.lifetime.sec = 1
        marker_array.markers.append(marker)

        # Marker 3: Sphere at left end effector
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = now.to_msg()
        marker.ns = 'eef_positions'
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(pos_l[0])
        marker.pose.position.y = float(pos_l[1])
        marker.pose.position.z = float(pos_l[2])
        marker.pose.orientation.w = 1.0

        # Left EEF color (red)
        marker.color = ColorRGBA()
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.6

        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        marker.lifetime.sec = 1
        marker_array.markers.append(marker)

        # Marker 4: Sphere at right end effector
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = now.to_msg()
        marker.ns = 'eef_positions'
        marker.id = 3
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(pos_r[0])
        marker.pose.position.y = float(pos_r[1])
        marker.pose.position.z = float(pos_r[2])
        marker.pose.orientation.w = 1.0

        # Right EEF color (blue)
        marker.color = ColorRGBA()
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.6

        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        marker.lifetime.sec = 1
        marker_array.markers.append(marker)

        # Publish markers
        self.marker_pub.publish(marker_array)

    def timer_callback(self):
        """Periodic callback to update head position."""
        # Check if URDF is loaded
        if not self.urdf_loaded:
            return

        # Get transforms for both end effectors
        transform_l = self.get_transform(self.target_frame, self.eef_l_link)
        transform_r = self.get_transform(self.target_frame, self.eef_r_link)

        if transform_l is None or transform_r is None:
            if not self.enable_debug_logging:
                return
            self.debug_counter += 1
            if self.debug_counter % self.debug_log_interval == 0:
                self.get_logger().warn(
                    f'[Update {self.debug_counter}] Failed to get transforms '
                    f'for end effectors'
                )
            return

        # Extract positions
        pos_l = (
            transform_l.transform.translation.x,
            transform_l.transform.translation.y,
            transform_l.transform.translation.z
        )
        pos_r = (
            transform_r.transform.translation.x,
            transform_r.transform.translation.y,
            transform_r.transform.translation.z
        )

        # Calculate center point
        center_x = (pos_l[0] + pos_r[0]) / 2.0
        center_y = (pos_l[1] + pos_r[1]) / 2.0
        center_z = (pos_l[2] + pos_r[2]) / 2.0

        center_point = (center_x, center_y, center_z)

        # Use head_joint1 position from URDF
        if self.head_joint1_pos is None:
            self.get_logger().warn('head_joint1 position not available from URDF')
            return

        # Debug logging
        self.debug_counter += 1
        should_debug = (
            (self.debug_counter % self.debug_log_interval == 0) and
            self.enable_debug_logging
        )

        if should_debug:
            self.get_logger().info('-' * 60)
            self.get_logger().info(f'[Update {self.debug_counter}] Head EEF Tracking:')
            self.get_logger().info(
                f'  End Effector Positions (in {self.target_frame}):'
            )
            self.get_logger().info(
                f'    {self.eef_l_link}: ({pos_l[0]:.4f}, {pos_l[1]:.4f}, '
                f'{pos_l[2]:.4f})'
            )
            self.get_logger().info(
                f'    {self.eef_r_link}: ({pos_r[0]:.4f}, {pos_r[1]:.4f}, '
                f'{pos_r[2]:.4f})'
            )
            self.get_logger().info(
                f'  Center Point: ({center_x:.4f}, {center_y:.4f}, '
                f'{center_z:.4f})'
            )

        # Calculate required head angles
        head_joint1_angle, head_joint2_angle = self.calculate_head_angles(
            center_point, self.head_joint1_pos, debug=should_debug
        )

        if head_joint1_angle is None or head_joint2_angle is None:
            self.get_logger().warn('Failed to calculate head angles')
            return

        # Create and publish joint trajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.frame_id = ''
        trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [head_joint1_angle, head_joint2_angle]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 0

        trajectory_msg.points = [point]

        self.joint_trajectory_pub.publish(trajectory_msg)

        # Publish visualization markers
        self.create_visualization_markers(
            self.head_joint1_pos, center_point, pos_l, pos_r
        )

        if should_debug:
            self.get_logger().info('  Published Joint Commands:')
            self.get_logger().info(
                f'    {self.head_joint1_name} (PITCH): '
                f'{math.degrees(head_joint1_angle):.2f}° '
                f'({head_joint1_angle:.4f} rad)'
            )
            self.get_logger().info(
                f'    {self.head_joint2_name} (YAW): '
                f'{math.degrees(head_joint2_angle):.2f}° '
                f'({head_joint2_angle:.4f} rad)'
            )
            self.get_logger().info('-' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = HeadEefTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
