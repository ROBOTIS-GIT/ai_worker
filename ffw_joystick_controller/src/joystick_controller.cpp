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

#include <chrono>
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

  // Sensor names
const char LEFT_JOYSTICK_NAME[] = "sensorxel_l_joy";
const char RIGHT_JOYSTICK_NAME[] = "sensorxel_r_joy";

  // Mode names
const char SWERVE_MODE[] = "swerve";
const char ARM_CONTROL_MODE[] = "arm_control";
}  // namespace constants

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

  double normalized_value;
  if (raw_adc < params_.joystick_calibration_center) {
    normalized_value = -(params_.joystick_calibration_center - raw_adc) /
      (params_.joystick_calibration_center - params_.joystick_calibration_min);
  } else {
    normalized_value = (raw_adc - params_.joystick_calibration_center) /
      (params_.joystick_calibration_max - params_.joystick_calibration_center);
  }

  // Apply deadzone
  if (std::abs(normalized_value) < params_.deadzone) {
    return 0.0;
  }

  // Normalize after deadzone
  if (normalized_value > 0) {
    normalized_value = (normalized_value - params_.deadzone) / (1.0 - params_.deadzone);
  } else {
    normalized_value = (normalized_value + params_.deadzone) / (1.0 - params_.deadzone);
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

std::vector<double> JoystickController::calculate_joint_positions(
  const std::vector<std::string> & controlled_joints,
  const std::vector<double> & normalized_values,
  const std::string & sensor_name,
  bool swerve_mode,
  const JoystickValues & joystick_values,
  const std::vector<double> & command_positions) const
{
  std::vector<double> positions = command_positions;

  for (size_t i = 0; i < controlled_joints.size(); ++i) {
    double sensorxel_joy_value = 0.0;
    bool has_motion_intent = false;

    if (swerve_mode) {
      // In swerve mode only the right joystick X-axis controls the lift joint.
      if (sensor_name == constants::RIGHT_JOYSTICK_NAME && i == 0) {
        sensorxel_joy_value = joystick_values.right_x;
        has_motion_intent = std::abs(sensorxel_joy_value) > 0.0;
      }
    } else {
      // In head-control mode X controls the first joint and Y controls the second joint.
      const size_t axis_index = i == 0 ? 0 : 1;
      if (i < 2 && axis_index < normalized_values.size()) {
        sensorxel_joy_value = normalized_values[axis_index];
        has_motion_intent = std::abs(sensorxel_joy_value) > 0.0;
      }
    }

    // Keep the previous command unless this axis is intentionally operated.
    if (!has_motion_intent || i >= positions.size()) {
      continue;
    }

    const auto & joint_name = controlled_joints[i];
    auto it = std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(),
        joint_name);
    if (it != current_joint_states_.name.end()) {
      size_t index = std::distance(current_joint_states_.name.begin(), it);
      if (index < current_joint_states_.position.size()) {
        const double current_position = current_joint_states_.position[index];
        positions[i] = current_position + sensorxel_joy_value *
          sensor_jog_scale_.at(sensor_name);
      }
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

void JoystickController::publish_cmd_vel(bool swerve_mode, const JoystickValues & joystick_values)
{
  geometry_msgs::msg::Twist twist_msg;

  if (swerve_mode) {
    twist_msg.linear.x = -joystick_values.left_x / constants::LINEAR_X_SCALE;
    twist_msg.linear.y = joystick_values.left_y / constants::LINEAR_Y_SCALE;
    twist_msg.angular.z = -joystick_values.right_y / constants::ANGULAR_Z_SCALE;
  } else {
    // Zero twist when not in swerve mode
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;
  }

  cmd_vel_pub_->publish(twist_msg);
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
      left_tact_long_press_triggered_ = true;
    }
  }

  // Check for long press on right tact switch
  if (right_tact_pressed && !right_tact_long_press_triggered_) {
    auto press_duration = current_time - right_tact_press_start_time_;
    if (press_duration.seconds() >= params_.long_press_duration) {
      std_msgs::msg::String trigger_msg;
      trigger_msg.data = "right_long_time";
      tact_trigger_pub_->publish(trigger_msg);
      RCLCPP_INFO(get_node()->get_logger(), "Right tact switch long press triggered!");
      right_tact_long_press_triggered_ = true;
    }
  }

  // Set flag when both buttons are pressed
  if (current_state == 3) {
    both_pressed_flag_ = true;
  }

  // Only trigger actions when reaching 00 state (no buttons pressed)
  if (current_state == 0 && prev_state != 0) {
    if (both_pressed_flag_) {
      // Mode change - both buttons were pressed at some point
      std_msgs::msg::String mode_msg;
      if (current_mode_ == constants::ARM_CONTROL_MODE) {
        current_mode_ = constants::SWERVE_MODE;
      } else {
        current_mode_ = constants::ARM_CONTROL_MODE;
      }
      mode_msg.data = current_mode_;
      mode_pub_->publish(mode_msg);
      RCLCPP_INFO(get_node()->get_logger(), "Mode switched to: %s", current_mode_.c_str());

      // Reset flag after mode change
      both_pressed_flag_ = false;
    } else {
      // Individual button trigger - only if both were never pressed and not long press
      switch (prev_state) {
        case 1:  // 01 -> 00 (right button only was pressed)
          if (!right_tact_long_press_triggered_) {
            std_msgs::msg::String trigger_msg;
            trigger_msg.data = "right";
            tact_trigger_pub_->publish(trigger_msg);
            RCLCPP_INFO(get_node()->get_logger(), "Right tact switch triggered!");
          }
          break;

        case 2:  // 10 -> 00 (left button only was pressed)
          if (!left_tact_long_press_triggered_) {
            std_msgs::msg::String trigger_msg;
            trigger_msg.data = "left";
            tact_trigger_pub_->publish(trigger_msg);
            RCLCPP_INFO(get_node()->get_logger(), "Left tact switch triggered!");
          }
          break;
      }
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
  prev_tact_switch_ = (current_state == 3);
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
  // Always retain the latest feedback, but do not overwrite an initialized command with it.
  current_joint_states_ = *msg;
  has_joint_states_ = true;

  // Retry initialization until one message contains every controlled joint for the sensor.
  for (const auto & sensor_name : sensorxel_joy_names_) {
    if (sensor_command_initialized_[sensor_name]) {
      continue;
    }

    const auto & controlled_joints = sensor_controlled_joints_[sensor_name];
    std::vector<double> command_positions;
    command_positions.reserve(controlled_joints.size());
    bool complete = !controlled_joints.empty();

    for (const auto & joint_name : controlled_joints) {
      auto it = std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(),
          joint_name);
      if (it == current_joint_states_.name.end()) {
        complete = false;
        break;
      }

      const size_t index = std::distance(current_joint_states_.name.begin(), it);
      if (index >= current_joint_states_.position.size()) {
        complete = false;
        break;
      }
      command_positions.push_back(current_joint_states_.position[index]);
    }

    if (complete) {
      sensor_command_positions_[sensor_name] = std::move(command_positions);
      sensor_command_initialized_[sensor_name] = true;
    }
  }
}

controller_interface::return_type JoystickController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!has_joint_states_) {
    return controller_interface::return_type::OK;
  }

  bool swerve_mode = (current_mode_ == constants::SWERVE_MODE);
  JoystickValues joystick_values;
  bool left_tact_switch_pressed = false;
  bool right_tact_switch_pressed = false;

  // Process each sensor
  for (size_t sensor_idx = 0; sensor_idx < sensorxel_joy_names_.size(); ++sensor_idx) {
    const auto & sensor_name = sensorxel_joy_names_[sensor_idx];
    RCLCPP_DEBUG(get_node()->get_logger(), "Processing sensor: %s", sensor_name.c_str());

    const auto & controlled_joints = sensor_controlled_joints_[sensor_name];
    auto & command_positions = sensor_command_positions_[sensor_name];

    // Read and normalize sensor values
    std::vector<double> normalized_values = read_and_normalize_sensor_values(sensor_idx);

    // Update joystick values
    update_joystick_values(sensor_name, normalized_values, joystick_values,
                          left_tact_switch_pressed, right_tact_switch_pressed);

    // Publish joint trajectory
    if (sensor_command_initialized_[sensor_name] && !controlled_joints.empty()) {
      command_positions = calculate_joint_positions(
        controlled_joints, normalized_values, sensor_name, swerve_mode, joystick_values,
        command_positions);
      publish_joint_trajectory(controlled_joints, command_positions, sensor_name);
    }

    sensorxel_joy_values_[sensor_idx] = normalized_values;
  }

  // Publish cmd_vel
  publish_cmd_vel(swerve_mode, joystick_values);

  // Publish joystick values
  publish_joystick_values();

  // Handle all tact switch functionality (mode switching and individual triggers)
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
    sensor_command_positions_[sensor_name].clear();
    sensor_command_initialized_[sensor_name] = false;
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
  }

  // Create subscriber for joint states
  RCLCPP_WARN(get_node()->get_logger(), "Creating joint states subscriber for topic: %s",
      params_.joint_states_topic.c_str());
  joint_states_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    params_.joint_states_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&JoystickController::joint_states_callback, this, std::placeholders::_1));

  // Create publisher for mode
  mode_pub_ = get_node()->create_publisher<std_msgs::msg::String>(
    "/leader/joystick_controller_right/joystick_mode", 10);
  prev_tact_switch_ = false;

  // Create publisher for right tact switch trigger
  tact_trigger_pub_ = get_node()->create_publisher<std_msgs::msg::String>(
    "/leader/joystick_controller/tact_trigger", 10);
  prev_right_tact_switch_ = false;
  prev_left_tact_switch_ = false;
  both_pressed_flag_ = false;

  // Initialize long press variables
  left_tact_long_press_triggered_ = false;
  right_tact_long_press_triggered_ = false;
  left_tact_press_start_time_ = rclcpp::Time(0);
  right_tact_press_start_time_ = rclcpp::Time(0);

  // Create publisher for cmd_vel
  cmd_vel_pub_ = get_node()->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  RCLCPP_INFO(get_node()->get_logger(), "JoystickController configured successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  // Re-seed commands from the next complete joint-state message after every activation.
  for (const auto & sensor_name : sensorxel_joy_names_) {
    sensor_command_positions_[sensor_name].clear();
    sensor_command_initialized_[sensor_name] = false;
  }

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
  RCLCPP_INFO(get_node()->get_logger(), "JoystickController deactivated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JoystickController::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}
}  // namespace joystick_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joystick_controller::JoystickController, controller_interface::ControllerInterface)
