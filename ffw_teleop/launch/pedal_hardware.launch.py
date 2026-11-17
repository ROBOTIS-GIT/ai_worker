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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch only the DYNAMIXEL pedal, independently of the leader bringup."""
    namespace = LaunchConfiguration('namespace')
    description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', PathJoinSubstitution([
            FindPackageShare('ffw_description'), 'urdf', 'common', 'pedal',
            'pedal.urdf.xacro']),
        ' port_name:=', LaunchConfiguration('port_name'),
        ' baud_rate:=', LaunchConfiguration('baud_rate'),
        ' use_mock_hardware:=', LaunchConfiguration('use_mock_hardware'),
    ]), value_type=str)
    controllers = PathJoinSubstitution([
        FindPackageShare('ffw_teleop'), 'config', 'pedal_controllers.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'port_name', description='Pedal serial device (prefer /dev/serial/by-id)', default_value='/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_2DFEE81F503059384C2E3120FF050A3D-if00'),
        DeclareLaunchArgument('baud_rate', default_value='1000000'),
        DeclareLaunchArgument('namespace', default_value='pedal'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        DeclareLaunchArgument('long_press_seconds', default_value='1.0'),
        DeclareLaunchArgument('desired_position', default_value='0.5'),
        DeclareLaunchArgument('pressed_center', default_value='0.0'),
        DeclareLaunchArgument('pressed_window', default_value='0.1'),
        DeclareLaunchArgument('source_topic', default_value='/source'),
        DeclareLaunchArgument('set_source_service', default_value='/set_source'),
        DeclareLaunchArgument('set_enabled_service', default_value='/set_enabled'),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=namespace, parameters=[{'robot_description': description}], output='screen'),
        Node(
            package='controller_manager', executable='ros2_control_node', namespace=namespace,
            parameters=[controllers], output='screen'),
        Node(
            package='controller_manager', executable='spawner', namespace=namespace,
            arguments=['joint_state_broadcaster', 'position_controller'], output='screen'),
        Node(
            package='ffw_teleop', executable='pedal_input', namespace=namespace,
            parameters=[{
                name: ParameterValue(LaunchConfiguration(name), value_type=float)
                for name in ('long_press_seconds', 'desired_position',
                             'pressed_center', 'pressed_window')
            }, {
                name: LaunchConfiguration(name)
                for name in ('source_topic', 'set_source_service', 'set_enabled_service')
            }], output='screen'),
    ])
