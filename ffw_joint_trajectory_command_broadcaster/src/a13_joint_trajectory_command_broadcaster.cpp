// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "a13_joint_trajectory_command_broadcaster/a13_joint_trajectory_command_broadcaster.hpp"

#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace a13_joint_trajectory_command_broadcaster
{
controller_interface::CallbackReturn A13JointTrajectoryCommandBroadcaster::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  if (!param_listener_) {
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();
  if (!params_.enable_teleoperation) {
    RCLCPP_ERROR(get_node()->get_logger(), "A13 requires enable_teleoperation=true");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (const auto & side : {"l", "r"}) {
    std::vector<std::string> expected;
    for (int joint = 1; joint <= 7; ++joint) {
      expected.push_back(std::string("arm_") + side + "_joint" + std::to_string(joint));
    }
    const auto & configured = std::string(side) == "l" ? params_.left_joints : params_.right_joints;
    if (configured != expected) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "A13 requires arm_%s_joint1 through joint7 in order, without gripper or finger joints",
        side);
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  return a1_joint_trajectory_command_broadcaster::A1JointTrajectoryCommandBroadcaster::on_configure(
    previous_state);
}
}  // namespace a13_joint_trajectory_command_broadcaster

PLUGINLIB_EXPORT_CLASS(
  a13_joint_trajectory_command_broadcaster::A13JointTrajectoryCommandBroadcaster,
  controller_interface::ControllerInterface)
