import unittest

from ffw_bringup.joint_trajectory_splitter import split_trajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class TestJointTrajectorySplitter(unittest.TestCase):

    def test_splits_every_point_field_by_joint_name(self):
        message = JointTrajectory()
        message.joint_names = ['arm_l_joint1', 'gripper_l_joint1', 'arm_l_joint2']
        point = JointTrajectoryPoint()
        point.positions = [1.0, 2.0, 3.0]
        point.velocities = [4.0, 5.0, 6.0]
        point.time_from_start.sec = 1
        message.points = [point]

        arm, gripper = split_trajectory(message)

        self.assertEqual(arm.joint_names, ['arm_l_joint1', 'arm_l_joint2'])
        self.assertEqual(list(arm.points[0].positions), [1.0, 3.0])
        self.assertEqual(list(arm.points[0].velocities), [4.0, 6.0])
        self.assertEqual(gripper.joint_names, ['gripper_l_joint1'])
        self.assertEqual(list(gripper.points[0].positions), [2.0])
        self.assertEqual(gripper.points[0].time_from_start.sec, 1)


if __name__ == '__main__':
    unittest.main()
