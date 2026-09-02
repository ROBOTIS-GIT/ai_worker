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

#ifndef LEADER_JOYSTICK_CONTROLLER__LEADER_JOYSTICK_CONTROLLER_HPP_
#define LEADER_JOYSTICK_CONTROLLER__LEADER_JOYSTICK_CONTROLLER_HPP_

#include <cstdint>
#include <string>

#include "leader_joystick_controller/visibility_control.h"
#include \
  <ffw_joystick_controller/leader_joystick_controller_parameters.hpp>
#include "joystick_controller/joystick_controller.hpp"
#include "robotis_interfaces/msg/teleoperation_command.hpp"

namespace leader_joystick_controller
{

class LeaderJoystickController : public joystick_controller::JoystickController
{
public:
  LEADER_JOYSTICK_CONTROLLER_PUBLIC
  LeaderJoystickController() = default;

  LEADER_JOYSTICK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  LEADER_JOYSTICK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  void handle_tact_switches(
    bool left_tact_pressed, bool right_tact_pressed,
    const rclcpp::Time & current_time) override;

private:
  void publish_teleoperation_toggle(const std::string & target_arm);

  rclcpp::Time both_tact_press_start_time_;
  bool both_tact_long_press_triggered_ = false;
  rclcpp::Publisher<robotis_interfaces::msg::TeleoperationCommand>::SharedPtr
    teleoperation_command_pub_;
  uint64_t teleoperation_request_id_ = 0;
  std::shared_ptr<ParamListener> leader_param_listener_;
  Params leader_params_;
};

}  // namespace leader_joystick_controller

#endif  // LEADER_JOYSTICK_CONTROLLER__LEADER_JOYSTICK_CONTROLLER_HPP_
