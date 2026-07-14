from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ffw_teleop',
            executable='keyboard_control',
            name='keyboard_joint_controller',
            output='screen',
        ),
    ])
