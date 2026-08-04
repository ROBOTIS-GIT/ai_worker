// Copyright 2024 ROBOTIS CO., LTD.
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

#include <joystick_controller/joystick_controller.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/helpers.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace joystick_controller
{

// Constants for better maintainability
namespace constants
{
constexpr size_t TACT_SWITCH_INTERFACE_INDEX = 2;
constexpr double TACT_SWITCH_THRESHOLD = 0.5;
constexpr double DEFAULT_JOG_SCALE = 0.1;

  // cmd_vel scaling factors
constexpr double LINEAR_X_SCALE = 3.0;
constexpr double LINEAR_Y_SCALE = 3.0;
constexpr double ANGULAR_Z_SCALE = 2.0;
constexpr double RADIANS_TO_DEGREES = 180.0 / 3.14159265358979323846;

  // Sensor names
const char LEFT_JOYSTICK_NAME[] = "sensorxel_l_joy";
const char RIGHT_JOYSTICK_NAME[] = "sensorxel_r_joy";

}  // namespace constants

int64_t steady_time_nanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

JoystickController::JoystickController()
: controller_interface::ControllerInterface()
{
}

// Helper methods for better code organization
double JoystickController::normalize_joystick_value(double raw_adc, bool is_tact_switch) const
{
  if (is_tact_switch) {
    return raw_adc;
  }

  // const double deadzone = std::clamp(params_.deadzone, 0.0, std::nextafter(1.0, 0.0));
  const double deadzone = std::clamp(params_.deadzone, 0.0, 1.0);

  double normalized_value;
  if (raw_adc < params_.joystick_calibration_center) {
    normalized_value = -(params_.joystick_calibration_center - raw_adc) /
      (params_.joystick_calibration_center - params_.joystick_calibration_min);
  } else {
    normalized_value = (raw_adc - params_.joystick_calibration_center) /
      (params_.joystick_calibration_max - params_.joystick_calibration_center);
  }

  // Apply deadzone
  if (std::abs(normalized_value) <= deadzone) {
    return 0.0;
  }

  // Normalize after deadzone, normalized_value = [-1,1]
  if (normalized_value > 0) {
    normalized_value = (normalized_value - deadzone) / (1.0 - deadzone);
  } else {
    normalized_value = (normalized_value + deadzone) / (1.0 - deadzone);
  }

  return normalized_value;
}

std::vector<double> JoystickController::read_and_normalize_sensor_values(size_t sensor_idx) const
{
  std::vector<double> normalized_values(state_interface_types_.size(), 0.0);

  for (size_t j = 0; j < state_interface_types_.size(); ++j) {
    if (j >= joint_state_interface_.size() || sensor_idx >= joint_state_interface_[j].size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid interface access: j=%zu, i=%zu", j,
          sensor_idx);
      continue;
    }

    auto opt_value = joint_state_interface_[j][sensor_idx].get().get_optional();
    if (!opt_value.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "No value for state interface [%zu][%zu]", j,
          sensor_idx);
      continue;
    }

    double raw_adc = opt_value.value();
    bool is_tact_switch = (j == constants::TACT_SWITCH_INTERFACE_INDEX);
    double normalized_value = normalize_joystick_value(raw_adc, is_tact_switch);

    // Apply reverse if needed
    const auto & interface_name = state_interface_types_[j];
    const auto & reverse_interfaces =
      sensor_reverse_interfaces_.at(sensorxel_joy_names_[sensor_idx]);
    if (std::find(reverse_interfaces.begin(), reverse_interfaces.end(),
        interface_name) != reverse_interfaces.end())
    {
      normalized_value = -normalized_value;
    }

    normalized_values[j] = normalized_value;
  }

  return normalized_values;
}

void JoystickController::update_joystick_values(
  const std::string & sensor_name,
  const std::vector<double> & normalized_values,
  JoystickValues & joystick_values,
  bool & left_tact_pressed,
  bool & right_tact_pressed) const
{
  if (sensor_name == constants::LEFT_JOYSTICK_NAME) {
    joystick_values.left_x = normalized_values[0];
    joystick_values.left_y = normalized_values[1];
    if (normalized_values.size() > constants::TACT_SWITCH_INTERFACE_INDEX) {
      left_tact_pressed = (normalized_values[constants::TACT_SWITCH_INTERFACE_INDEX] >
        constants::TACT_SWITCH_THRESHOLD);
    }
  } else if (sensor_name == constants::RIGHT_JOYSTICK_NAME) {
    joystick_values.right_x = normalized_values[0];
    joystick_values.right_y = normalized_values[1];
    if (normalized_values.size() > constants::TACT_SWITCH_INTERFACE_INDEX) {
      right_tact_pressed = (normalized_values[constants::TACT_SWITCH_INTERFACE_INDEX] >
        constants::TACT_SWITCH_THRESHOLD);
    }
  }
}

void JoystickController::update_last_active_positions(
  const std::vector<std::string> & controlled_joints)
{
  // This method should be called with the correct sensor context
  // For now, we'll use the first sensor as default
  if (sensorxel_joy_names_.empty()) {
    return;
  }

  const std::string & sensor_name = sensorxel_joy_names_[0];
  auto & last_active_positions = sensor_last_active_positions_[sensor_name];

  for (size_t i = 0; i < controlled_joints.size(); ++i) {
    const auto & joint_name = controlled_joints[i];
    auto it = std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(),
        joint_name);
    if (it != current_joint_states_.name.end()) {
      size_t index = std::distance(current_joint_states_.name.begin(), it);
      if (i < last_active_positions.size()) {
        last_active_positions[i] = current_joint_states_.position[index];
      }
    }
  }
}

std::vector<double> JoystickController::calculate_joint_positions(
  const std::vector<std::string> & controlled_joints,
  const std::string & sensor_name,
  const JoystickValues & joystick_values) const
{
  std::vector<double> positions;

  for (size_t i = 0; i < controlled_joints.size(); ++i) {
    const auto & joint_name = controlled_joints[i];
    auto it = std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(),
        joint_name);
    if (it != current_joint_states_.name.end()) {
      size_t index = std::distance(current_joint_states_.name.begin(), it);
      double current_position = current_joint_states_.position[index];

      // Use right joystick X-axis for lift control; left joystick drives cmd_vel only.
      double sensorxel_joy_value = (sensor_name ==
        constants::RIGHT_JOYSTICK_NAME) ? joystick_values.right_x : 0.0;

      double new_position = current_position + sensorxel_joy_value *
        sensor_jog_scale_.at(sensor_name);
      positions.push_back(new_position);
    }
  }

  return positions;
}

void JoystickController::publish_joint_trajectory(
  const std::vector<std::string> & controlled_joints,
  const std::vector<double> & positions,
  const std::string & sensor_name)
{
  auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
  trajectory_msg.header.stamp = rclcpp::Time(0);
  trajectory_msg.joint_names = controlled_joints;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration(0, 0);
  point.positions = positions;
  point.velocities.resize(positions.size(), 0.0);
  point.accelerations.resize(positions.size(), 0.0);

  trajectory_msg.points.push_back(point);

  auto joint_trajectory_publisher = sensor_joint_trajectory_publisher_[sensor_name];
  if (joint_trajectory_publisher) {
    joint_trajectory_publisher->publish(trajectory_msg);
  } else {
    RCLCPP_WARN(get_node()->get_logger(),
        "Joint trajectory publisher not found for sensor: %s", sensor_name.c_str());
  }
}

void JoystickController::publish_joint_state(
  const std::vector<std::string> & controlled_joints,
  const std::vector<double> & positions,
  const std::string & sensor_name,
  const rclcpp::Time & time)
{
  auto joint_state_msg = sensor_msgs::msg::JointState();
  joint_state_msg.header.stamp = time;
  joint_state_msg.header.frame_id = "";
  joint_state_msg.name = controlled_joints;
  joint_state_msg.position = positions;
  // Set velocity and effort to NaN (not available from joystick)
  joint_state_msg.velocity.resize(positions.size(), std::numeric_limits<double>::quiet_NaN());
  joint_state_msg.effort.resize(positions.size(), std::numeric_limits<double>::quiet_NaN());

  auto joint_state_publisher = sensor_joint_state_stamped_publisher_[sensor_name];
  if (joint_state_publisher) {
    joint_state_publisher->publish(joint_state_msg);
  } else {
    RCLCPP_WARN(get_node()->get_logger(),
        "Joint state publisher not found for sensor: %s", sensor_name.c_str());
  }
}

geometry_msgs::msg::Twist JoystickController::create_manual_cmd_vel(
  const JoystickValues & joystick_values) const
{
  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = -joystick_values.left_x / constants::LINEAR_X_SCALE;
  twist_msg.linear.y = joystick_values.left_y / constants::LINEAR_Y_SCALE;
  twist_msg.angular.z = -joystick_values.right_y / constants::ANGULAR_Z_SCALE;
  return twist_msg;
}

void JoystickController::publish_zero_cmd_vel()
{
  if (cmd_vel_pub_) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
  }
}

bool JoystickController::read_fresh_odometry(OdometrySnapshot & snapshot) const
{
  if (!latest_odometry_available_ || !latest_odometry_.valid) {
    return false;
  }

  const int64_t age_ns = steady_time_nanoseconds() - latest_odometry_.received_steady_time_ns;
  const int64_t stale_timeout_ns = static_cast<int64_t>(
    params_.random_base_odom_stale_timeout * 1e9);
  if (age_ns < 0 || age_ns > stale_timeout_ns) {
    return false;
  }

  snapshot = latest_odometry_;
  return true;
}

bool JoystickController::process_odometry_snapshot(const OdometrySnapshot & snapshot)
{
  if (snapshot.sequence == 0 || snapshot.sequence == last_processed_odometry_sequence_) {
    return false;
  }

  last_processed_odometry_sequence_ = snapshot.sequence;
  bool discontinuity_detected = !snapshot.valid;
  if (snapshot.valid && previous_odometry_available_) {
    const bool stamp_went_backwards =
      previous_odometry_stamp_ns_ != 0 &&
      (snapshot.source_stamp_ns == 0 ||
      snapshot.source_stamp_ns < previous_odometry_stamp_ns_);
    const double position_jump = std::hypot(
      snapshot.pose.x - previous_odometry_pose_.x,
      snapshot.pose.y - previous_odometry_pose_.y);
    const double yaw_jump = std::abs(RandomBaseController::normalize_angle(
        snapshot.pose.yaw - previous_odometry_pose_.yaw));
    discontinuity_detected =
      stamp_went_backwards || position_jump > params_.random_base_odom_jump_position ||
      yaw_jump > params_.random_base_odom_jump_yaw;
  }

  const bool motion_was_active = random_base_controller_.is_active();
  const bool anchor_was_set = random_base_controller_.has_anchor();
  if (discontinuity_detected) {
    reset_random_base_runtime(true);
    if (anchor_was_set) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Odometry discontinuity detected; random base anchor was reset.");
    }
  }

  if (snapshot.valid) {
    latest_odometry_ = snapshot;
    latest_odometry_available_ = true;
    previous_odometry_pose_ = snapshot.pose;
    previous_odometry_stamp_ns_ = snapshot.source_stamp_ns;
    previous_odometry_available_ = true;
    random_base_stationary_window_.observe(
      snapshot.twist, snapshot.received_steady_time_ns,
      params_.random_base_settle_linear_velocity,
      params_.random_base_settle_angular_velocity,
      static_cast<int64_t>(params_.random_base_odom_stale_timeout * 1e9));
  } else {
    latest_odometry_available_ = false;
    previous_odometry_available_ = false;
    previous_odometry_stamp_ns_ = 0;
    random_base_stationary_window_.reset();
  }

  return discontinuity_detected && motion_was_active;
}

void JoystickController::reset_random_base_runtime(bool reset_anchor)
{
  if (reset_anchor) {
    random_base_controller_.reset();
    random_base_stationary_window_.reset();
    last_random_base_auto_failed_ = false;
  } else {
    random_base_controller_.cancel();
  }
}

bool JoystickController::request_random_base_move(const rclcpp::Time & current_time)
{
  if (!params_.enable_random_base_reposition || middle_pedal_held_.load()) {
    return false;
  }

  if (random_base_controller_.is_active()) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Random base move request ignored because a move is already active.");
    return false;
  }

  const auto * buffered_odometry = odometry_buffer_.readFromRT();
  if (buffered_odometry != nullptr) {
    process_odometry_snapshot(*buffered_odometry);
  }

  OdometrySnapshot snapshot;
  if (!read_fresh_odometry(snapshot)) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Random base move request ignored because fresh valid odometry is unavailable.");
    return false;
  }

  if (std::hypot(snapshot.twist.linear_x, snapshot.twist.linear_y) >
    params_.random_base_settle_linear_velocity ||
    std::abs(snapshot.twist.angular_z) > params_.random_base_settle_angular_velocity)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Random base move request ignored because the base is still moving.");
    return false;
  }

  double anchor_distance = 0.0;
  double anchor_yaw_error = 0.0;
  if (random_base_controller_.has_anchor()) {
    const auto & anchor = random_base_controller_.anchor();
    anchor_distance = std::hypot(snapshot.pose.x - anchor.x, snapshot.pose.y - anchor.y);
    anchor_yaw_error = std::abs(RandomBaseController::normalize_angle(
        snapshot.pose.yaw - anchor.yaw));
  }

  RandomBaseStartOptions start_options;
  start_options.allow_reanchor = true;
  start_options.previous_auto_failed = last_random_base_auto_failed_;
  start_options.reanchor_stationary = random_base_stationary_window_.satisfies(
    static_cast<int64_t>(params_.random_base_reanchor_stationary_duration * 1e9));
  const auto start_status = random_base_controller_.start_with_policy(
    snapshot.pose, current_time.seconds(), start_options);

  const bool started =
    start_status == RandomBaseStartStatus::STARTED_WITH_NEW_ANCHOR ||
    start_status == RandomBaseStartStatus::STARTED_WITH_EXISTING_ANCHOR ||
    start_status == RandomBaseStartStatus::STARTED_WITH_REANCHORED_POSE;
  if (!started) {
    switch (start_status) {
      case RandomBaseStartStatus::OUTSIDE_ANCHOR_ENVELOPE:
        if (anchor_distance <= params_.random_base_anchor_position_limit) {
          RCLCPP_WARN(
            get_node()->get_logger(),
            "Random base move rejected: yaw difference %.2f deg exceeds the %.2f deg "
            "anchor reuse limit.",
            anchor_yaw_error * constants::RADIANS_TO_DEGREES,
            params_.random_base_anchor_yaw_limit * constants::RADIANS_TO_DEGREES);
        } else {
          RCLCPP_WARN(
            get_node()->get_logger(),
            "Random base move rejected: anchor distance %.3f m is in the blocked gap "
            "above %.3f m and below %.3f m; use an explicit anchor reset.",
            anchor_distance, params_.random_base_anchor_position_limit,
            params_.random_base_reanchor_distance);
        }
        break;
      case RandomBaseStartStatus::REANCHOR_REQUIRES_STATIONARY_ODOMETRY:
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Automatic re-anchor at %.3f m rejected: odometry must remain stationary for "
          "%.3f s continuously.",
          anchor_distance, params_.random_base_reanchor_stationary_duration);
        break;
      case RandomBaseStartStatus::REANCHOR_BLOCKED_AFTER_AUTO_FAILURE:
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Automatic re-anchor at %.3f m blocked because the previous AUTO move failed; "
          "call the reset_random_anchor service or restart the controller.",
          anchor_distance);
        break;
      case RandomBaseStartStatus::BUSY:
        RCLCPP_INFO(
          get_node()->get_logger(),
          "Random base move request ignored because a move is already active.");
        break;
      case RandomBaseStartStatus::INVALID_INPUT:
      default:
        RCLCPP_WARN(
          get_node()->get_logger(), "Random base move request contained invalid input.");
        break;
    }
    return false;
  }

  const auto & anchor = random_base_controller_.anchor();
  const auto & target = random_base_controller_.target();
  const char * anchor_mode =
    start_status == RandomBaseStartStatus::STARTED_WITH_NEW_ANCHOR ? "new anchor" :
    start_status == RandomBaseStartStatus::STARTED_WITH_REANCHORED_POSE ?
    "automatically re-anchored pose" : "existing anchor";
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Random base move started with %s: anchor=(%.4f, %.4f, %.2f deg), "
    "target=(%.4f, %.4f, %.2f deg).",
    anchor_mode,
    anchor.x, anchor.y, anchor.yaw * constants::RADIANS_TO_DEGREES,
    target.x, target.y, target.yaw * constants::RADIANS_TO_DEGREES);
  return true;
}

bool JoystickController::request_random_base_return(const rclcpp::Time & current_time)
{
  if (!params_.enable_random_base_reposition || middle_pedal_held_.load()) {
    return false;
  }

  if (random_base_controller_.is_active()) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Return-to-anchor request ignored because a random base move is already active.");
    return false;
  }

  const auto * buffered_odometry = odometry_buffer_.readFromRT();
  if (buffered_odometry != nullptr) {
    process_odometry_snapshot(*buffered_odometry);
  }

  OdometrySnapshot snapshot;
  if (!read_fresh_odometry(snapshot)) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Return-to-anchor request ignored because fresh valid odometry is unavailable.");
    return false;
  }
  if (!random_base_controller_.has_anchor()) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Return-to-anchor request ignored because no random base anchor is available.");
    return false;
  }
  if (last_random_base_auto_failed_) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Return-to-anchor request blocked because the previous AUTO move failed; "
      "reset the anchor or restart the controller.");
    return false;
  }
  if (!random_base_stationary_window_.satisfies(
      static_cast<int64_t>(params_.random_base_reanchor_stationary_duration * 1e9)))
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Return-to-anchor request ignored because odometry has not remained stationary for "
      "%.3f s.",
      params_.random_base_reanchor_stationary_duration);
    return false;
  }

  const auto start_status = random_base_controller_.start_return_to_anchor(
    snapshot.pose, current_time.seconds());
  if (start_status != RandomBaseStartStatus::STARTED_RETURN_TO_ANCHOR) {
    if (start_status == RandomBaseStartStatus::OUTSIDE_ANCHOR_ENVELOPE) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Return-to-anchor request rejected because the base is outside the anchor envelope.");
    } else {
      RCLCPP_WARN(
        get_node()->get_logger(), "Return-to-anchor request contained invalid input.");
    }
    return false;
  }

  const auto & anchor = random_base_controller_.anchor();
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Random base return started: anchor=(%.4f, %.4f, %.2f deg).",
    anchor.x, anchor.y, anchor.yaw * constants::RADIANS_TO_DEGREES);
  return true;
}

void JoystickController::publish_cmd_vel(
  const JoystickValues & joystick_values, const rclcpp::Time & current_time)
{
  const auto manual_twist = create_manual_cmd_vel(joystick_values);
  if (!params_.enable_random_base_reposition) {
    cmd_vel_pub_->publish(manual_twist);
    return;
  }

  bool force_zero = false;
  const int64_t current_time_ns = current_time.nanoseconds();
  if (last_update_time_ns_ != 0 && current_time_ns < last_update_time_ns_) {
    const bool motion_was_active = random_base_controller_.is_active();
    const bool anchor_was_set = random_base_controller_.has_anchor();
    reset_random_base_runtime(true);
    force_zero = motion_was_active;
    if (anchor_was_set) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Controller time moved backwards; random base anchor was reset.");
    }
  }
  last_update_time_ns_ = current_time_ns;

  if (reset_random_anchor_requested_.exchange(false)) {
    const bool motion_was_active = random_base_controller_.is_active();
    reset_random_base_runtime(true);
    force_zero = force_zero || motion_was_active;
    RCLCPP_INFO(get_node()->get_logger(), "Random base anchor reset completed.");
  }

  const auto * odometry = odometry_buffer_.readFromRT();
  if (odometry != nullptr) {
    force_zero = process_odometry_snapshot(*odometry) || force_zero;
  }

  const bool manual_mobile_requested =
    manual_twist.linear.x != 0.0 || manual_twist.linear.y != 0.0 ||
    manual_twist.angular.z != 0.0;
  if (random_base_controller_.is_active() &&
    (middle_pedal_held_.load() || manual_mobile_requested))
  {
    random_base_controller_.cancel();
    last_random_base_auto_failed_ = true;
    force_zero = true;
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Random base movement cancelled by manual override.");
  }

  if (force_zero) {
    publish_zero_cmd_vel();
    return;
  }

  if (!random_base_controller_.is_active()) {
    cmd_vel_pub_->publish(manual_twist);
    return;
  }

  OdometrySnapshot snapshot;
  if (!read_fresh_odometry(snapshot)) {
    random_base_controller_.cancel();
    last_random_base_auto_failed_ = true;
    publish_zero_cmd_vel();
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Random base movement cancelled because odometry became stale or unavailable.");
    return;
  }

  const auto result = random_base_controller_.update(
    snapshot.pose, snapshot.twist, current_time.seconds());
  geometry_msgs::msg::Twist auto_twist;
  auto_twist.linear.x = result.command.linear_x;
  auto_twist.linear.y = result.command.linear_y;
  auto_twist.angular.z = result.command.angular_z;

  switch (result.status) {
    case RandomBaseStepStatus::ACTIVE:
      cmd_vel_pub_->publish(auto_twist);
      break;
    case RandomBaseStepStatus::SUCCEEDED:
      last_random_base_auto_failed_ = false;
      publish_zero_cmd_vel();
      RCLCPP_INFO(get_node()->get_logger(), "Random base movement completed.");
      break;
    case RandomBaseStepStatus::TIMED_OUT:
      last_random_base_auto_failed_ = true;
      publish_zero_cmd_vel();
      RCLCPP_WARN(get_node()->get_logger(), "Random base movement timed out.");
      break;
    case RandomBaseStepStatus::INVALID_INPUT:
      last_random_base_auto_failed_ = true;
      publish_zero_cmd_vel();
      RCLCPP_ERROR(get_node()->get_logger(), "Random base movement received invalid input.");
      break;
    case RandomBaseStepStatus::OUTSIDE_ANCHOR_ENVELOPE:
      last_random_base_auto_failed_ = true;
      publish_zero_cmd_vel();
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Random base movement left the anchor envelope and was cancelled.");
      break;
    case RandomBaseStepStatus::IDLE:
    default:
      publish_zero_cmd_vel();
      break;
  }
}

void JoystickController::publish_joystick_values()
{
  auto sensorxel_joy_msg = std_msgs::msg::Float64MultiArray();
  for (const auto & sensorxel_joy_value : sensorxel_joy_values_) {
    sensorxel_joy_msg.data.insert(
      sensorxel_joy_msg.data.end(),
      sensorxel_joy_value.begin(),
      sensorxel_joy_value.end());
  }

  if (sensorxel_joy_publisher_.count("common") > 0) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Publishing joystick values to common topic");
    sensorxel_joy_publisher_["common"]->publish(sensorxel_joy_msg);
  }
}

void JoystickController::handle_tact_switches(
  bool left_tact_pressed, bool right_tact_pressed, const rclcpp::Time & current_time)
{
  // Create current state as bit pattern: left=bit1, right=bit0
  // 00 = neither pressed, 01 = right only, 10 = left only, 11 = both pressed
  uint8_t current_state = (left_tact_pressed ? 2 : 0) | (right_tact_pressed ? 1 : 0);
  uint8_t prev_state = (prev_left_tact_switch_ ? 2 : 0) | (prev_right_tact_switch_ ? 1 : 0);

  // Handle left tact switch press start
  if (left_tact_pressed && !prev_left_tact_switch_) {
    left_tact_press_start_time_ = current_time;
    left_tact_long_press_triggered_ = false;
  }

  // Handle right tact switch press start
  if (right_tact_pressed && !prev_right_tact_switch_) {
    right_tact_press_start_time_ = current_time;
    right_tact_long_press_triggered_ = false;
  }

  // Check for long press on left tact switch
  if (left_tact_pressed && !left_tact_long_press_triggered_) {
    auto press_duration = current_time - left_tact_press_start_time_;
    if (press_duration.seconds() >= params_.long_press_duration) {
      std_msgs::msg::String trigger_msg;
      trigger_msg.data = "left_long_time";
      tact_trigger_pub_->publish(trigger_msg);
      RCLCPP_INFO(get_node()->get_logger(), "Left tact switch long press triggered!");
      if (!middle_pedal_held_.load() && !right_tact_pressed) {
        request_random_base_return(current_time);
      }
      left_tact_long_press_triggered_ = true;
    }
  }

  // Check for long press on right tact switch
  if (right_tact_pressed && !right_tact_long_press_triggered_) {
    auto press_duration = current_time - right_tact_press_start_time_;
    if (press_duration.seconds() >= params_.long_press_duration) {
      const bool middle_pedal_held = middle_pedal_held_.load();
      std_msgs::msg::String trigger_msg;
      trigger_msg.data = middle_pedal_held ? "right_long_time_middle" : "right_long_time";
      tact_trigger_pub_->publish(trigger_msg);
      RCLCPP_INFO(
        get_node()->get_logger(), "Right tact switch long press triggered! (middle: %s)",
        middle_pedal_held ? "held" : "not held");
      if (!middle_pedal_held && !left_tact_pressed) {
        request_random_base_move(current_time);
      }
      right_tact_long_press_triggered_ = true;
    }
  }

  // Only trigger actions when reaching 00 state (no buttons pressed)
  if (current_state == 0 && prev_state != 0) {
    switch (prev_state) {
      case 1:  // 01 -> 00 (right button only was pressed)
        if (!right_tact_long_press_triggered_) {
          if (middle_pedal_held_.load()) {
            std_msgs::msg::String trigger_msg;
            trigger_msg.data = "right";
            tact_trigger_pub_->publish(trigger_msg);
            RCLCPP_INFO(get_node()->get_logger(), "Right tact trigger (middle pedal held)");
          } else {
            std_msgs::msg::UInt8 enable_msg;
            enable_msg.data = 2;
            right_enable_pub_->publish(enable_msg);
            RCLCPP_INFO(get_node()->get_logger(), "Right toggle pub");
          }
        }
        break;

      case 2:  // 10 -> 00 (left button only was pressed)
        if (!left_tact_long_press_triggered_) {
          if (middle_pedal_held_.load()) {
            std_msgs::msg::String trigger_msg;
            trigger_msg.data = "left";
            tact_trigger_pub_->publish(trigger_msg);
            RCLCPP_INFO(get_node()->get_logger(), "Left tact trigger (middle pedal held)");
          } else {
            std_msgs::msg::UInt8 enable_msg;
            enable_msg.data = 2;
            left_enable_pub_->publish(enable_msg);
            RCLCPP_INFO(get_node()->get_logger(), "Left toggle pub");
          }
        }
        break;

      case 3:  // 11 -> 00 (both buttons were pressed)
        if ((!left_tact_long_press_triggered_) && (!right_tact_long_press_triggered_)) {
          std_msgs::msg::UInt8 enable_msg;
          enable_msg.data = 2;
          left_enable_pub_->publish(enable_msg);
          right_enable_pub_->publish(enable_msg);
          RCLCPP_INFO(get_node()->get_logger(), "both toggle pub");
        }
        break;
    }

    // Reset long press flags when buttons are released
    if (prev_state == 1 || prev_state == 3) {  // Right button was pressed
      right_tact_long_press_triggered_ = false;
    }
    if (prev_state == 2 || prev_state == 3) {  // Left button was pressed
      left_tact_long_press_triggered_ = false;
    }
  }

  // Update previous state
  prev_left_tact_switch_ = left_tact_pressed;
  prev_right_tact_switch_ = right_tact_pressed;
}

controller_interface::InterfaceConfiguration
JoystickController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration
JoystickController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & sensorxel_joy_name : sensorxel_joy_names_) {
    for (const auto & interface_type : state_interface_types_) {
      config.names.push_back(sensorxel_joy_name + "/" + interface_type);
    }
  }

  return config;
}

void JoystickController::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Store current joint states
  current_joint_states_ = *msg;

  // initialize last_active_positions_ by sensor
  if (!has_joint_states_) {
    for (const auto & sensor_name : sensorxel_joy_names_) {
      const auto & controlled_joints = sensor_controlled_joints_[sensor_name];
      auto & last_active_positions = sensor_last_active_positions_[sensor_name];
      last_active_positions.resize(controlled_joints.size());
      for (size_t i = 0; i < controlled_joints.size(); ++i) {
        const auto & joint_name = controlled_joints[i];
        auto it = std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(),
            joint_name);
        if (it != current_joint_states_.name.end()) {
          size_t index = std::distance(current_joint_states_.name.begin(), it);
          last_active_positions[i] = current_joint_states_.position[index];
        }
      }
    }
  }

  has_joint_states_ = true;
}

void JoystickController::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  OdometrySnapshot snapshot;
  snapshot.sequence = odometry_sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
  snapshot.received_steady_time_ns = steady_time_nanoseconds();
  snapshot.source_stamp_ns =
    static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
    static_cast<int64_t>(msg->header.stamp.nanosec);

  const auto & position = msg->pose.pose.position;
  const auto & orientation = msg->pose.pose.orientation;
  const auto & twist = msg->twist.twist;
  const double quaternion_norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;

  snapshot.valid =
    msg->header.frame_id == random_base_odom_frame_ &&
    msg->child_frame_id == random_base_child_frame_ &&
    std::isfinite(position.x) && std::isfinite(position.y) &&
    std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
    std::isfinite(orientation.z) && std::isfinite(orientation.w) &&
    std::isfinite(quaternion_norm_squared) &&
    quaternion_norm_squared > std::numeric_limits<double>::epsilon() &&
    std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
    std::isfinite(twist.angular.z);

  if (snapshot.valid) {
    const double inverse_norm = 1.0 / std::sqrt(quaternion_norm_squared);
    const double qx = orientation.x * inverse_norm;
    const double qy = orientation.y * inverse_norm;
    const double qz = orientation.z * inverse_norm;
    const double qw = orientation.w * inverse_norm;
    const double sin_yaw = 2.0 * (qw * qz + qx * qy);
    const double cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz);

    snapshot.pose.x = position.x;
    snapshot.pose.y = position.y;
    snapshot.pose.yaw = std::atan2(sin_yaw, cos_yaw);
    snapshot.twist.linear_x = twist.linear.x;
    snapshot.twist.linear_y = twist.linear.y;
    snapshot.twist.angular_z = twist.angular.z;
  }

  odometry_buffer_.writeFromNonRT(snapshot);
}

controller_interface::return_type JoystickController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!has_joint_states_) {
    return controller_interface::return_type::OK;
  }

  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }

  const bool joystick_update_enabled = params_.enable_joystick_update;
  JoystickValues joystick_values;
  bool left_tact_switch_pressed = false;
  bool right_tact_switch_pressed = false;

  // Process each sensor
  for (size_t sensor_idx = 0; sensor_idx < sensorxel_joy_names_.size(); ++sensor_idx) {
    const auto & sensor_name = sensorxel_joy_names_[sensor_idx];
    RCLCPP_DEBUG(get_node()->get_logger(), "Processing sensor: %s", sensor_name.c_str());

    const auto & controlled_joints = sensor_controlled_joints_[sensor_name];
    auto & last_active_positions = sensor_last_active_positions_[sensor_name];

    // Read all joystick interfaces so tact-switch features still work even when motion is disabled.
    std::vector<double> normalized_values = read_and_normalize_sensor_values(sensor_idx);
    if (!joystick_update_enabled) {
      for (size_t j = 0; j < normalized_values.size(); ++j) {
        if (j != constants::TACT_SWITCH_INTERFACE_INDEX) {
          normalized_values[j] = 0.0;
        }
      }
    }

    // Motion activity only depends on joystick axes, not the tact switch state.
    bool any_sensorxel_joy_active = false;
    for (size_t j = 0; j < normalized_values.size(); ++j) {
      if (j == constants::TACT_SWITCH_INTERFACE_INDEX) {
        continue;
      }
      if (std::abs(normalized_values[j]) > 0.0) {
        any_sensorxel_joy_active = true;
        break;
      }
    }

    // Update joystick values
    update_joystick_values(sensor_name, normalized_values, joystick_values,
                          left_tact_switch_pressed, right_tact_switch_pressed);

    // Update last active positions when joystick becomes inactive
    if (was_active_ && !any_sensorxel_joy_active && !current_joint_states_.name.empty() &&
      !controlled_joints.empty())
    {
      for (size_t i = 0; i < controlled_joints.size(); ++i) {
        const auto & joint_name = controlled_joints[i];
        auto it = std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(),
            joint_name);
        if (it != current_joint_states_.name.end()) {
          size_t index = std::distance(current_joint_states_.name.begin(), it);
          if (i < last_active_positions.size()) {
            last_active_positions[i] = current_joint_states_.position[index];
          }
        }
      }
    }

    // Publish joint trajectory
    if (
      joystick_update_enabled && !current_joint_states_.name.empty() && !controlled_joints.empty())
    {
      std::vector<double> positions;

      if (any_sensorxel_joy_active) {
        positions = calculate_joint_positions(
          controlled_joints, sensor_name, joystick_values);
        for (size_t i = 0; i < positions.size() && i < last_active_positions.size(); ++i) {
          last_active_positions[i] = positions[i];
        }
      } else {
        positions = last_active_positions;
      }

      publish_joint_trajectory(controlled_joints, positions, sensor_name);
    }

    // Publish joint state with timestamp from update() function
    if (!current_joint_states_.name.empty() && !controlled_joints.empty()) {
      publish_joint_state(controlled_joints, last_active_positions, sensor_name, time);
    }

    was_active_ = any_sensorxel_joy_active;
    sensorxel_joy_values_[sensor_idx] = normalized_values;
  }

  // Publish cmd_vel
  publish_cmd_vel(joystick_values, time);

  // Publish joystick values
  publish_joystick_values();

  handle_tact_switches(left_tact_switch_pressed, right_tact_switch_pressed, time);

  RCLCPP_DEBUG(get_node()->get_logger(), "Joystick controller update completed");

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn JoystickController::on_init()
{
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  if (!param_listener_) {
    RCLCPP_ERROR(logger, "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  if (params_.deadzone < 0.0 || params_.deadzone > 1.0) {
    RCLCPP_ERROR(
      logger,
      "Invalid deadzone %.3f. 'deadzone' must be in the range [0.0, 1.0).",
      params_.deadzone);
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!params_.enable_joystick_update) {
    RCLCPP_INFO(logger, "Joystick updates are disabled by parameter.");
  }

  RandomBaseControllerConfig random_base_config;
  random_base_config.radius = params_.random_base_radius;
  random_base_config.yaw_range = params_.random_base_yaw_range;
  random_base_config.linear_gain = params_.random_base_linear_gain;
  random_base_config.angular_gain = params_.random_base_angular_gain;
  random_base_config.min_linear_speed = params_.random_base_min_linear_speed;
  random_base_config.max_linear_speed = params_.random_base_max_linear_speed;
  random_base_config.min_angular_speed = params_.random_base_min_angular_speed;
  random_base_config.max_angular_speed = params_.random_base_max_angular_speed;
  random_base_config.max_linear_acceleration =
    params_.random_base_max_linear_acceleration;
  random_base_config.max_angular_acceleration =
    params_.random_base_max_angular_acceleration;
  random_base_config.position_tolerance = params_.random_base_position_tolerance;
  random_base_config.yaw_tolerance = params_.random_base_yaw_tolerance;
  random_base_config.settle_linear_velocity =
    params_.random_base_settle_linear_velocity;
  random_base_config.settle_angular_velocity =
    params_.random_base_settle_angular_velocity;
  random_base_config.settle_duration = params_.random_base_settle_duration;
  random_base_config.motion_timeout = params_.random_base_motion_timeout;
  random_base_config.anchor_position_limit = params_.random_base_anchor_position_limit;
  random_base_config.anchor_yaw_limit = params_.random_base_anchor_yaw_limit;
  random_base_config.reanchor_distance = params_.random_base_reanchor_distance;

  const bool random_base_io_config_valid =
    !params_.random_base_odom_topic.empty() && !params_.random_base_odom_frame.empty() &&
    !params_.random_base_child_frame.empty() &&
    std::isfinite(params_.random_base_odom_stale_timeout) &&
    params_.random_base_odom_stale_timeout > 0.0 &&
    std::isfinite(params_.random_base_odom_jump_position) &&
    params_.random_base_odom_jump_position > 0.0 &&
    std::isfinite(params_.random_base_odom_jump_yaw) &&
    params_.random_base_odom_jump_yaw > 0.0 &&
    std::isfinite(params_.random_base_reanchor_stationary_duration) &&
    params_.random_base_reanchor_stationary_duration >= 0.0;
  if (!random_base_controller_.set_config(random_base_config) || !random_base_io_config_valid) {
    RCLCPP_ERROR(logger, "Invalid random base reposition parameters.");
    return controller_interface::CallbackReturn::ERROR;
  }

  random_base_odom_frame_ = params_.random_base_odom_frame;
  random_base_child_frame_ = params_.random_base_child_frame;
  odometry_buffer_.initRT(OdometrySnapshot{});
  odometry_sequence_.store(0, std::memory_order_relaxed);
  reset_random_anchor_requested_.store(false);
  latest_odometry_ = OdometrySnapshot{};
  previous_odometry_pose_ = PlanarPose{};
  last_processed_odometry_sequence_ = 0;
  previous_odometry_stamp_ns_ = 0;
  last_update_time_ns_ = 0;
  latest_odometry_available_ = false;
  previous_odometry_available_ = false;
  random_base_stationary_window_.reset();
  last_random_base_auto_failed_ = false;

  // Get sensorxel_joy sensor names from parameters
  sensorxel_joy_names_ = params_.joystick_sensors;
  n_sensorxel_joys_ = sensorxel_joy_names_.size();

  if (sensorxel_joy_names_.empty()) {
    RCLCPP_WARN(logger, "'joystick_sensors' parameter is empty.");
  }

  // Initialize the sensorxel_joy values vector with the correct size
  sensorxel_joy_values_.resize(n_sensorxel_joys_);
  for (auto & vec : sensorxel_joy_values_) {
    vec.resize(state_interface_types_.size(), 0.0);
  }
  // read parameters by sensor name
  for (const auto & sensor_name : sensorxel_joy_names_) {
    // controlled_joints
    std::string joints_param = sensor_name + "_controlled_joints";
    if (get_node()->has_parameter(joints_param)) {
      sensor_controlled_joints_[sensor_name] =
        get_node()->get_parameter(joints_param).as_string_array();
    }
    // reverse_interfaces
    std::string reverse_param = sensor_name + "_reverse_interfaces";
    if (get_node()->has_parameter(reverse_param)) {
      sensor_reverse_interfaces_[sensor_name] =
        get_node()->get_parameter(reverse_param).as_string_array();
    } else {
      // If parameter does not exist, initialize as empty vector
      sensor_reverse_interfaces_[sensor_name] = std::vector<std::string>();
    }
    // joint_trajectory_topic
    std::string topic_param = sensor_name + "_joint_trajectory_topic";
    if (get_node()->has_parameter(topic_param)) {
      RCLCPP_WARN(get_node()->get_logger(), "parameter: %s, value: %s", topic_param.c_str(),
          get_node()->get_parameter(topic_param).as_string().c_str());
      sensor_joint_trajectory_topic_[sensor_name] =
        get_node()->get_parameter(topic_param).as_string();
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "parameter: %s not found", topic_param.c_str());
    }
    // joint_states_stamped_topic
    std::string joint_state_topic_param = sensor_name + "_joint_states_stamped_topic";
    if (get_node()->has_parameter(joint_state_topic_param)) {
      sensor_joint_state_stamped_topic_[sensor_name] =
        get_node()->get_parameter(joint_state_topic_param).as_string();
    } else {
      sensor_joint_state_stamped_topic_[sensor_name] = "";
    }
    // jog_scale
    std::string jog_scale_param = sensor_name + "_jog_scale";
    if (get_node()->has_parameter(jog_scale_param)) {
      sensor_jog_scale_[sensor_name] = get_node()->get_parameter(jog_scale_param).as_double();
    } else {
      // fallback: default jog scale
      sensor_jog_scale_[sensor_name] = constants::DEFAULT_JOG_SCALE;
      RCLCPP_WARN(get_node()->get_logger(), "parameter: %s not found, using default %.1f",
          jog_scale_param.c_str(), constants::DEFAULT_JOG_SCALE);
    }
  }

  // Create publisher for sensorxel_joy values (common topic)
  sensorxel_joy_publisher_["common"] =
    get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/sensorxel_joy_values", rclcpp::SystemDefaultsQoS());

  // Create publisher for joint trajectory
  for (const auto & sensor_name : sensorxel_joy_names_) {
    RCLCPP_WARN(get_node()->get_logger(),
        "Creating joint trajectory publisher for sensor: %s, topic: %s", sensor_name.c_str(),
        sensor_joint_trajectory_topic_[sensor_name].c_str());
    sensor_joint_trajectory_publisher_[sensor_name] =
      get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      sensor_joint_trajectory_topic_[sensor_name], rclcpp::SystemDefaultsQoS());

    // Create joint state publisher for this sensor with timestamp from update() function
    std::string joint_state_topic_name = sensor_joint_state_stamped_topic_[sensor_name];
    if (joint_state_topic_name.empty()) {
      joint_state_topic_name = sensor_joint_trajectory_topic_[sensor_name];
      // Replace "joint_trajectory" with "joint_states_stamped" in topic name
      size_t pos = joint_state_topic_name.find("joint_trajectory");
      if (pos != std::string::npos) {
        joint_state_topic_name.replace(pos, std::string("joint_trajectory").length(),
            "joint_states_stamped");
      } else {
        // Fallback: append "_joint_states_stamped" if pattern not found
        joint_state_topic_name += "_joint_states_stamped";
      }
    }
    sensor_joint_state_stamped_publisher_[sensor_name] =
      get_node()->create_publisher<sensor_msgs::msg::JointState>(
      joint_state_topic_name, rclcpp::SystemDefaultsQoS());
    RCLCPP_INFO(get_node()->get_logger(),
        "Created joint state stamped publisher for sensor: %s, topic: %s",
        sensor_name.c_str(), joint_state_topic_name.c_str());
  }

  // Create subscriber for joint states
  RCLCPP_WARN(get_node()->get_logger(), "Creating joint states subscriber for topic: %s",
      params_.joint_states_topic.c_str());
  joint_states_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    params_.joint_states_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&JoystickController::joint_states_callback, this, std::placeholders::_1));

  if (params_.enable_random_base_reposition) {
    odometry_subscriber_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(
      params_.random_base_odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&JoystickController::odometry_callback, this, std::placeholders::_1));
    reset_random_anchor_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
      "/leader/joystick_controller/reset_random_anchor",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        reset_random_anchor_requested_.store(true);
        response->success = true;
        response->message = "Random base anchor reset queued.";
      });
    RCLCPP_INFO(
      logger, "Random base reposition enabled with odometry topic '%s'.",
      params_.random_base_odom_topic.c_str());
  } else {
    odometry_subscriber_.reset();
    reset_random_anchor_service_.reset();
  }

  // Create publisher for right tact switch trigger
  tact_trigger_pub_ = get_node()->create_publisher<std_msgs::msg::String>(
    "/leader/joystick_controller/tact_trigger", 10);

  // Create publishers for left/right enable (0=disable, 1=enable, 2=toggle)
  left_enable_pub_ = get_node()->create_publisher<std_msgs::msg::UInt8>(
    "/leader/left_command", 1);
  right_enable_pub_ = get_node()->create_publisher<std_msgs::msg::UInt8>(
    "/leader/right_command", 1);

  prev_right_tact_switch_ = false;
  prev_left_tact_switch_ = false;

  // Initialize long press variables
  left_tact_long_press_triggered_ = false;
  right_tact_long_press_triggered_ = false;
  left_tact_press_start_time_ = rclcpp::Time(0);
  right_tact_press_start_time_ = rclcpp::Time(0);

  // Create publisher for cmd_vel
  cmd_vel_pub_ = get_node()->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Subscribe to foot switch middle pedal state
  middle_pedal_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "/leader/foot_switch/middle_pedal", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      middle_pedal_held_.store(msg->data);
    });

  RCLCPP_INFO(get_node()->get_logger(), "JoystickController configured successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
  reset_random_base_runtime(true);
  reset_random_anchor_requested_.store(false);
  last_processed_odometry_sequence_ = 0;
  previous_odometry_stamp_ns_ = 0;
  last_update_time_ns_ = 0;
  latest_odometry_available_ = false;
  previous_odometry_available_ = false;

  // Initialize state interface vector
  joint_state_interface_.resize(state_interface_types_.size());

  // Order all sensorxel_joy sensors in the storage
  for (size_t i = 0; i < state_interface_types_.size(); ++i) {
    const auto & interface = state_interface_types_[i];
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    ordered_interfaces;
    if (!controller_interface::get_ordered_interfaces(
        state_interfaces_, sensorxel_joy_names_, interface, ordered_interfaces))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' state interfaces, got %zu.",
        n_sensorxel_joys_, interface.c_str(), ordered_interfaces.size());
      return CallbackReturn::ERROR;
    }
    joint_state_interface_[i] = ordered_interfaces;
  }

  RCLCPP_INFO(get_node()->get_logger(), "JoystickController activated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  publish_zero_cmd_vel();
  reset_random_base_runtime(true);
  RCLCPP_INFO(get_node()->get_logger(), "JoystickController deactivated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  publish_zero_cmd_vel();
  reset_random_base_runtime(true);
  odometry_subscriber_.reset();
  reset_random_anchor_service_.reset();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  publish_zero_cmd_vel();
  reset_random_base_runtime(true);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  publish_zero_cmd_vel();
  reset_random_base_runtime(true);
  return CallbackReturn::SUCCESS;
}
}  // namespace joystick_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joystick_controller::JoystickController, controller_interface::ControllerInterface)
