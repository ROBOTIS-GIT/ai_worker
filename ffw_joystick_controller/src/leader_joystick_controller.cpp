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

#include "leader_joystick_controller/leader_joystick_controller.hpp"

#include <string>

#include "std_msgs/msg/string.hpp"

namespace leader_joystick_controller
{
namespace
{
constexpr char kHeadControlMode[] = "head_control";
constexpr char kSwerveMode[] = "swerve";
}  // namespace

controller_interface::CallbackReturn LeaderJoystickController::on_init()
{
  const auto result = joystick_controller::JoystickController::on_init();
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result;
  }

  try {
    leader_param_listener_ = std::make_shared<ParamListener>(get_node());
    leader_params_ = leader_param_listener_->get_params();
  } catch (const std::exception & error) {
    fprintf(stderr, "Exception thrown during leader joystick init: %s\n", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeaderJoystickController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  const auto result = joystick_controller::JoystickController::on_configure(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result;
  }

  if (!leader_param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Leader parameter listener is not initialized");
    return controller_interface::CallbackReturn::ERROR;
  }
  leader_params_ = leader_param_listener_->get_params();

  if (leader_params_.teleoperation_toggle_enabled) {
    current_mode_ = kHeadControlMode;
    teleoperation_command_pub_ =
      get_node()->create_publisher<robotis_interfaces::msg::TeleoperationCommand>(
      leader_params_.teleoperation_command_topic, 10);
    const auto source_state_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    leader_action_enabled_pub_ = get_node()->create_publisher<std_msgs::msg::Bool>(
      leader_params_.command_source_state_topic, source_state_qos);
    leader_action_enabled_ = false;
    std_msgs::msg::Bool source_state;
    source_state.data = leader_action_enabled_;
    leader_action_enabled_pub_->publish(source_state);
  }
  both_tact_press_start_time_ = rclcpp::Time(0);
  both_tact_long_press_triggered_ = false;
  teleoperation_request_id_ = 0;

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Leader joystick controller configured (teleoperation toggle hold: %.2f s)",
    leader_params_.teleoperation_toggle_long_press_duration);
  return controller_interface::CallbackReturn::SUCCESS;
}

void LeaderJoystickController::publish_teleoperation_toggle(const std::string & target_arm)
{
  if (!teleoperation_command_pub_ || !leader_action_enabled_) {
    return;
  }

  robotis_interfaces::msg::TeleoperationCommand command;
  command.request_id = ++teleoperation_request_id_;
  command.target_arm = target_arm;
  command.command = robotis_interfaces::msg::TeleoperationCommand::COMMAND_TOGGLE;
  teleoperation_command_pub_->publish(command);
}

void LeaderJoystickController::toggle_leader_action_output()
{
  if (!leader_action_enabled_pub_) {
    return;
  }
  leader_action_enabled_ = !leader_action_enabled_;
  std_msgs::msg::Bool source_state;
  source_state.data = leader_action_enabled_;
  leader_action_enabled_pub_->publish(source_state);
  // TODO(teleoperation): Notify the external model runtime to stop or resume here once
  // its standard ROS topic/service interface is selected.
}

void LeaderJoystickController::handle_tact_switches(
  bool left_tact_pressed, bool right_tact_pressed, const rclcpp::Time & current_time)
{
  if (!leader_params_.teleoperation_toggle_enabled) {
    joystick_controller::JoystickController::handle_tact_switches(
      left_tact_pressed, right_tact_pressed, current_time);
    return;
  }

  // left=bit1, right=bit0: 00=none, 01=right, 10=left, 11=both
  const uint8_t current_state =
    (left_tact_pressed ? 2 : 0) | (right_tact_pressed ? 1 : 0);
  const uint8_t prev_state =
    (prev_left_tact_switch_ ? 2 : 0) | (prev_right_tact_switch_ ? 1 : 0);

  const bool individual_long_press_triggered =
    left_tact_long_press_triggered_ || right_tact_long_press_triggered_;
  if (current_state == 3 && !individual_long_press_triggered) {
    if (!both_pressed_flag_ || (prev_state != 3 && !both_tact_long_press_triggered_)) {
      both_tact_press_start_time_ = current_time;
    } else if (!both_tact_long_press_triggered_) {
      const auto press_duration = current_time - both_tact_press_start_time_;
      if (press_duration.seconds() >=
        leader_params_.teleoperation_toggle_long_press_duration)
      {
        both_tact_long_press_triggered_ = true;
        toggle_leader_action_output();
        RCLCPP_INFO(
          get_node()->get_logger(), "Leader action output %s; both arms remain stopped",
          leader_action_enabled_ ? "enabled" : "disabled");
      }
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
    if (press_duration.seconds() >=
      leader_params_.teleoperation_toggle_long_press_duration)
    {
      publish_teleoperation_toggle(
        robotis_interfaces::msg::TeleoperationCommand::TARGET_LEFT);
      if (leader_action_enabled_) {
        RCLCPP_INFO(get_node()->get_logger(), "Left-arm teleoperation toggled");
      } else {
        RCLCPP_INFO(
          get_node()->get_logger(),
          "Left-arm teleoperation ignored while model control is selected");
      }
      left_tact_long_press_triggered_ = true;
    }
  }
  if (right_tact_pressed && !both_pressed_flag_ && !right_tact_long_press_triggered_) {
    const auto press_duration = current_time - right_tact_press_start_time_;
    if (press_duration.seconds() >=
      leader_params_.teleoperation_toggle_long_press_duration)
    {
      publish_teleoperation_toggle(
        robotis_interfaces::msg::TeleoperationCommand::TARGET_RIGHT);
      if (leader_action_enabled_) {
        RCLCPP_INFO(get_node()->get_logger(), "Right-arm teleoperation toggled");
      } else {
        RCLCPP_INFO(
          get_node()->get_logger(),
          "Right-arm teleoperation ignored while model control is selected");
      }
      right_tact_long_press_triggered_ = true;
    }
  }

  // Execute exactly one action after every participating button has been released.
  if (current_state == 0 && prev_state != 0) {
    if (both_pressed_flag_) {
      if (
        !both_tact_long_press_triggered_ &&
        !left_tact_long_press_triggered_ && !right_tact_long_press_triggered_)
      {
        std_msgs::msg::String mode_msg;
        current_mode_ = current_mode_ == kHeadControlMode ? kSwerveMode : kHeadControlMode;
        mode_msg.data = current_mode_;
        mode_pub_->publish(mode_msg);
        RCLCPP_INFO(
          get_node()->get_logger(), "Mode switched to: %s", current_mode_.c_str());
      }
    } else if (prev_state == 1 && !right_tact_long_press_triggered_) {
      std_msgs::msg::String trigger_msg;
      trigger_msg.data = "right";
      tact_trigger_pub_->publish(trigger_msg);
      RCLCPP_INFO(get_node()->get_logger(), "Right tact switch triggered!");
    } else if (prev_state == 2 && !left_tact_long_press_triggered_) {
      std_msgs::msg::String trigger_msg;
      trigger_msg.data = "left";
      tact_trigger_pub_->publish(trigger_msg);
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

}  // namespace leader_joystick_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  leader_joystick_controller::LeaderJoystickController,
  controller_interface::ControllerInterface)
