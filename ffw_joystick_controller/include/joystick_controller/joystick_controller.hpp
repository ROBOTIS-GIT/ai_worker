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

#ifndef JOYSTICK_CONTROLLER__JOYSTICK_CONTROLLER_HPP_
#define JOYSTICK_CONTROLLER__JOYSTICK_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <map>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joystick_controller/visibility_control.h"
#include "ffw_joystick_controller/joystick_controller_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace joystick_controller
{

class JoystickController : public controller_interface::ControllerInterface
{
public:
  JOYSTICK_CONTROLLER_PUBLIC
  JoystickController();

  JOYSTICK_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  JOYSTICK_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  JOYSTICK_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  JOYSTICK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  JOYSTICK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  JOYSTICK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  JOYSTICK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  JOYSTICK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  JOYSTICK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  JOYSTICK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  std::vector<std::string> sensorxel_joy_names_;
  std::vector<std::string> state_interface_types_ = {"JOYSTICK X VALUE", "JOYSTICK Y VALUE", "JOYSTICK TACT SWITCH"};
  size_t n_sensorxel_joys_ = 0;
  std::vector<std::vector<double>> sensorxel_joy_values_;
  std::vector<std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>>
  joint_state_interface_;
  sensor_msgs::msg::JointState current_joint_states_;
  bool was_active_ = false;  // Track previous sensorxel_joy state
  bool has_joint_states_ = false;  // Track if joint states have been received

  std::map<std::string, std::vector<std::string>> sensor_controlled_joints_;
  std::map<std::string, std::vector<std::string>> sensor_reverse_interfaces_;
  std::map<std::string, std::string> sensor_joint_trajectory_topic_;
  std::map<std::string, std::vector<double>> sensor_last_active_positions_;
  std::map<std::string, rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr> sensor_joint_trajectory_publisher_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> sensorxel_joy_publisher_;

  // Add per-sensor jog scale
  std::map<std::string, double> sensor_jog_scale_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  std::string current_mode_ = "swerve";
  bool prev_tact_switch_ = false;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

}  // namespace joystick_controller

#endif  // JOYSTICK_CONTROLLER__JOYSTICK_CONTROLLER_HPP_
