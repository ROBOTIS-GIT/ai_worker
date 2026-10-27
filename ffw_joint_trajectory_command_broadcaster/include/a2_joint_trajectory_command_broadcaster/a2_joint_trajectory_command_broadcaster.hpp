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

#include "a2_joint_trajectory_command_broadcaster/visibility_control.h"
#include \
  "leader_joint_trajectory_command_broadcaster/leader_joint_trajectory_command_broadcaster.hpp"

namespace a2_joint_trajectory_command_broadcaster
{
// A2 currently uses the common leader conversion without additional correction.
// Lifecycle, communication, synchronization, and publishing remain owned by the base.
class A2JointTrajectoryCommandBroadcaster
  : public leader_joint_trajectory_command_broadcaster::LeaderJointTrajectoryCommandBroadcaster
{
public:
  A2_JOINT_TRAJECTORY_COMMAND_BROADCASTER_PUBLIC
  A2JointTrajectoryCommandBroadcaster() = default;
};
}  // namespace a2_joint_trajectory_command_broadcaster
