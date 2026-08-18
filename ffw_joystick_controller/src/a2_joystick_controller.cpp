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

#include "a2_joystick_controller/a2_joystick_controller.hpp"

#include <string>

#include "std_msgs/msg/string.hpp"

namespace a2_joystick_controller
{
namespace
{
constexpr char kArmControlMode[] = "arm_control";
constexpr char kSwerveMode[] = "swerve";
}  // namespace

controller_interface::CallbackReturn A2JoystickController::on_init()
{
  const auto result = joystick_controller::JoystickController::on_init();
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result;
  }

  try {
    a2_param_listener_ = std::make_shared<ParamListener>(get_node());
    a2_params_ = a2_param_listener_->get_params();
  } catch (const std::exception & error) {
    fprintf(stderr, "Exception thrown during A2 joystick init: %s\n", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn A2JoystickController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  const auto result = joystick_controller::JoystickController::on_configure(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result;
  }

  if (!a2_param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "A2 parameter listener is not initialized");
    return controller_interface::CallbackReturn::ERROR;
  }
  a2_params_ = a2_param_listener_->get_params();

  if (a2_params_.teleoperation_toggle_enabled) {
    teleoperation_command_pub_ =
      get_node()->create_publisher<robotis_interfaces::msg::TeleoperationCommand>(
      a2_params_.teleoperation_command_topic, 10);
  }
  both_tact_press_start_time_ = rclcpp::Time(0);
  both_tact_long_press_triggered_ = false;
  teleoperation_request_id_ = 0;

  RCLCPP_INFO(
    get_node()->get_logger(),
    "A2 joystick controller configured (mode switch hold: %.2f s)",
    a2_params_.a2_mode_switch_long_press_duration);
  return controller_interface::CallbackReturn::SUCCESS;
}

void A2JoystickController::publish_teleoperation_toggle(const std::string & target_arm)
{
  if (!teleoperation_command_pub_) {
    return;
  }

  robotis_interfaces::msg::TeleoperationCommand command;
  command.request_id = ++teleoperation_request_id_;
  command.target_arm = target_arm;
  command.command = robotis_interfaces::msg::TeleoperationCommand::COMMAND_TOGGLE;
  teleoperation_command_pub_->publish(command);
}

void A2JoystickController::handle_tact_switches(
  bool left_tact_pressed, bool right_tact_pressed, const rclcpp::Time & current_time)
{
  if (!a2_params_.teleoperation_toggle_enabled) {
    joystick_controller::JoystickController::handle_tact_switches(
      left_tact_pressed, right_tact_pressed, current_time);
    return;
  }

  // left=bit1, right=bit0: 00=none, 01=right, 10=left, 11=both
  const uint8_t current_state =
    (left_tact_pressed ? 2 : 0) | (right_tact_pressed ? 1 : 0);
  const uint8_t prev_state =
    (prev_left_tact_switch_ ? 2 : 0) | (prev_right_tact_switch_ ? 1 : 0);

  if (current_state == 3) {
    if (!both_pressed_flag_ || (prev_state != 3 && !both_tact_long_press_triggered_)) {
      both_tact_press_start_time_ = current_time;
    } else if (!both_tact_long_press_triggered_) {
      const auto press_duration = current_time - both_tact_press_start_time_;
      if (press_duration.seconds() >= a2_params_.a2_mode_switch_long_press_duration) {
        both_tact_long_press_triggered_ = true;
      }
    }
  }

  // Account for the release update being the first update past the threshold.
  if (both_pressed_flag_ && prev_state == 3 && current_state != 3 &&
    !both_tact_long_press_triggered_)
  {
    const auto press_duration = current_time - both_tact_press_start_time_;
    if (press_duration.seconds() >= a2_params_.a2_mode_switch_long_press_duration) {
      both_tact_long_press_triggered_ = true;
    }
  }

  // A gesture that has included both buttons must not emit an individual action.
  if (current_state == 3) {
    both_pressed_flag_ = true;
  }

  if (left_tact_pressed && !prev_left_tact_switch_) {
    left_tact_press_start_time_ = current_time;
    left_tact_long_press_triggered_ = false;
  }
  if (right_tact_pressed && !prev_right_tact_switch_) {
    right_tact_press_start_time_ = current_time;
    right_tact_long_press_triggered_ = false;
  }

  if (left_tact_pressed && !both_pressed_flag_ && !left_tact_long_press_triggered_) {
    const auto press_duration = current_time - left_tact_press_start_time_;
    if (press_duration.seconds() >= params_.long_press_duration) {
      std_msgs::msg::String trigger_msg;
      trigger_msg.data = "left_long_time";
      tact_trigger_pub_->publish(trigger_msg);
      RCLCPP_INFO(get_node()->get_logger(), "Left tact switch long press triggered!");
      left_tact_long_press_triggered_ = true;
    }
  }
  if (right_tact_pressed && !both_pressed_flag_ && !right_tact_long_press_triggered_) {
    const auto press_duration = current_time - right_tact_press_start_time_;
    if (press_duration.seconds() >= params_.long_press_duration) {
      std_msgs::msg::String trigger_msg;
      trigger_msg.data = "right_long_time";
      tact_trigger_pub_->publish(trigger_msg);
      RCLCPP_INFO(get_node()->get_logger(), "Right tact switch long press triggered!");
      right_tact_long_press_triggered_ = true;
    }
  }

  // Execute exactly one action after every participating button has been released.
  if (current_state == 0 && prev_state != 0) {
    if (both_pressed_flag_) {
      if (both_tact_long_press_triggered_) {
        std_msgs::msg::String mode_msg;
        current_mode_ = current_mode_ == kArmControlMode ? kSwerveMode : kArmControlMode;
        mode_msg.data = current_mode_;
        mode_pub_->publish(mode_msg);
        RCLCPP_INFO(
          get_node()->get_logger(), "Mode switched to: %s", current_mode_.c_str());
      } else {
        publish_teleoperation_toggle(
          robotis_interfaces::msg::TeleoperationCommand::TARGET_BOTH);
        RCLCPP_INFO(get_node()->get_logger(), "Both-arm teleoperation toggled");
      }
    } else if (prev_state == 1 && !right_tact_long_press_triggered_) {
      std_msgs::msg::String trigger_msg;
      trigger_msg.data = "right";
      tact_trigger_pub_->publish(trigger_msg);
      publish_teleoperation_toggle(
        robotis_interfaces::msg::TeleoperationCommand::TARGET_RIGHT);
      RCLCPP_INFO(get_node()->get_logger(), "Right tact switch triggered!");
    } else if (prev_state == 2 && !left_tact_long_press_triggered_) {
      std_msgs::msg::String trigger_msg;
      trigger_msg.data = "left";
      tact_trigger_pub_->publish(trigger_msg);
      publish_teleoperation_toggle(
        robotis_interfaces::msg::TeleoperationCommand::TARGET_LEFT);
      RCLCPP_INFO(get_node()->get_logger(), "Left tact switch triggered!");
    }

    both_pressed_flag_ = false;
    both_tact_long_press_triggered_ = false;
    left_tact_long_press_triggered_ = false;
    right_tact_long_press_triggered_ = false;
  }

  prev_left_tact_switch_ = left_tact_pressed;
  prev_right_tact_switch_ = right_tact_pressed;
  prev_tact_switch_ = (current_state == 3);
}

}  // namespace a2_joystick_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  a2_joystick_controller::A2JoystickController,
  controller_interface::ControllerInterface)
