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

#include "a13_joint_trajectory_command_broadcaster/visibility_control.h"
#include "a1_joint_trajectory_command_broadcaster/a1_joint_trajectory_command_broadcaster.hpp"

namespace a13_joint_trajectory_command_broadcaster
{
class A13JointTrajectoryCommandBroadcaster
  : public a1_joint_trajectory_command_broadcaster::A1JointTrajectoryCommandBroadcaster
{
public:
  A13_JOINT_TRAJECTORY_COMMAND_BROADCASTER_PUBLIC
  A13JointTrajectoryCommandBroadcaster() = default;

  A13_JOINT_TRAJECTORY_COMMAND_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
};
}  // namespace a13_joint_trajectory_command_broadcaster
