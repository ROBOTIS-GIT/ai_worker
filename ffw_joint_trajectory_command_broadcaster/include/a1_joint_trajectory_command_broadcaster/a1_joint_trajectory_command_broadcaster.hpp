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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "a1_joint_trajectory_command_broadcaster/visibility_control.h"
#include \
  "leader_joint_trajectory_command_broadcaster/leader_joint_trajectory_command_broadcaster.hpp"
// *INDENT-OFF*
#include <ffw_joint_trajectory_command_broadcaster/a1_joint_trajectory_command_broadcaster_parameters.hpp>  // NOLINT
// *INDENT-ON*

namespace a1_joint_trajectory_command_broadcaster
{
// A1 owns its private converted input views; the common base remains unchanged.
class A1JointTrajectoryCommandBroadcaster
  : public leader_joint_trajectory_command_broadcaster::LeaderJointTrajectoryCommandBroadcaster
{
public:
  A1_JOINT_TRAJECTORY_COMMAND_BROADCASTER_PUBLIC
  A1JointTrajectoryCommandBroadcaster() = default;

  A1_JOINT_TRAJECTORY_COMMAND_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  A1_JOINT_TRAJECTORY_COMMAND_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  A1_JOINT_TRAJECTORY_COMMAND_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  A1_JOINT_TRAJECTORY_COMMAND_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  A1_JOINT_TRAJECTORY_COMMAND_BROADCASTER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  A1_JOINT_TRAJECTORY_COMMAND_BROADCASTER_PUBLIC
  double scale_command_position(
    const std::string & joint_name, double position) const;

private:
  std::shared_ptr<ParamListener> scaling_param_listener_;
  Params scaling_params_;

  struct InputConversion
  {
    std::string joint_name;
    bool is_position = false;
    double sign = 1.0;
    double offset = 0.0;
    double last_raw_position = 0.0;
  };
  std::vector<InputConversion> input_conversions_;
  // Own the cached handles until after their loaned views have been destroyed.
  std::vector<hardware_interface::StateInterface::SharedPtr> cached_state_handles_;
  std::vector<hardware_interface::LoanedStateInterface> cached_state_interfaces_;
};
}  // namespace a1_joint_trajectory_command_broadcaster
