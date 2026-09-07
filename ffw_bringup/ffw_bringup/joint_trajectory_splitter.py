#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


def _select_joints(message, indices):
    selected = JointTrajectory()
    selected.header = message.header
    selected.joint_names = [message.joint_names[index] for index in indices]

    for point in message.points:
        selected_point = JointTrajectoryPoint()
        for field in ('positions', 'velocities', 'accelerations', 'effort'):
            values = getattr(point, field)
            setattr(
                selected_point,
                field,
                [values[index] for index in indices] if values else [],
            )
        selected_point.time_from_start = point.time_from_start
        selected.points.append(selected_point)

    return selected


def split_trajectory(message):
    joint_count = len(message.joint_names)
    if not joint_count:
        raise ValueError('trajectory has no joint names')

    for point in message.points:
        for field in ('positions', 'velocities', 'accelerations', 'effort'):
            values = getattr(point, field)
            if values and len(values) != joint_count:
                raise ValueError(
                    f'{field} has {len(values)} values for {joint_count} joints'
                )

    arm_indices = []
    gripper_indices = []
    for index, name in enumerate(message.joint_names):
        if name.startswith('gripper_'):
            gripper_indices.append(index)
        else:
            arm_indices.append(index)
    return (
        _select_joints(message, arm_indices),
        _select_joints(message, gripper_indices),
    )


# Split /leader/joint_trajectory_command_broadcaster_{left,right}/joint_trajectory
# into modular arm and gripper controller topics.
class JointTrajectorySplitter(Node):

    def __init__(self):
        super().__init__('joint_trajectory_splitter')

        self._trajectory_publishers = {}
        for side, suffix in (('left', 'l'), ('right', 'r')):
            self._trajectory_publishers[side] = (
                self.create_publisher(
                    JointTrajectory, f'/arm_{suffix}_controller/joint_trajectory', 10
                ),
                self.create_publisher(
                    JointTrajectory, f'/gripper_{suffix}_controller/joint_trajectory', 10
                ),
            )
            self.create_subscription(
                JointTrajectory,
                f'/leader/joint_trajectory_command_broadcaster_{side}/joint_trajectory',
                lambda message, side=side: self._split(side, message),
                10,
            )

    def _split(self, side, message):
        try:
            arm, gripper = split_trajectory(message)
        except ValueError as error:
            self.get_logger().warning(f'Ignored invalid {side} trajectory: {error}')
            return

        arm_publisher, gripper_publisher = self._trajectory_publishers[side]
        if arm.joint_names:
            arm_publisher.publish(arm)
        if gripper.joint_names:
            gripper_publisher.publish(gripper)


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectorySplitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
