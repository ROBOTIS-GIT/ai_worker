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

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "ffw_joystick_controller/joystick_controller_parameters.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joystick_controller/random_base_controller.hpp"
#include "joystick_controller/visibility_control.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace joystick_controller
{

// Structure to hold joystick values for better organization
struct JoystickValues
{
  double left_x = 0.0;
  double left_y = 0.0;
  double right_x = 0.0;
  double right_y = 0.0;
};

struct OdometrySnapshot
{
  PlanarPose pose;
  PlanarTwist twist;
  int64_t received_steady_time_ns = 0;
  int64_t source_stamp_ns = 0;
  uint64_t sequence = 0;
  bool valid = false;
};

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
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Helper methods for better code organization
  double normalize_joystick_value(double raw_adc, bool is_tact_switch) const;
  std::vector<double> read_and_normalize_sensor_values(size_t sensor_idx) const;
  void update_joystick_values(
    const std::string & sensor_name,
    const std::vector<double> & normalized_values,
    JoystickValues & joystick_values,
    bool & left_tact_pressed,
    bool & right_tact_pressed) const;
  void update_last_active_positions(const std::vector<std::string> & controlled_joints);
  std::vector<double> calculate_joint_positions(
    const std::vector<std::string> & controlled_joints,
    const std::string & sensor_name,
    const JoystickValues & joystick_values) const;
  void publish_joint_trajectory(
    const std::vector<std::string> & controlled_joints,
    const std::vector<double> & positions,
    const std::string & sensor_name);
  void publish_joint_state(
    const std::vector<std::string> & controlled_joints,
    const std::vector<double> & positions,
    const std::string & sensor_name,
    const rclcpp::Time & time);
  geometry_msgs::msg::Twist create_manual_cmd_vel(const JoystickValues & joystick_values) const;
  void publish_cmd_vel(
    const JoystickValues & joystick_values, const rclcpp::Time & current_time);
  void publish_zero_cmd_vel();
  bool request_random_base_move(const rclcpp::Time & current_time);
  bool request_random_base_return(const rclcpp::Time & current_time);
  bool read_fresh_odometry(OdometrySnapshot & snapshot) const;
  bool process_odometry_snapshot(const OdometrySnapshot & snapshot);
  void reset_random_base_runtime(bool reset_anchor);
  void publish_joystick_values();
  void handle_tact_switches(
    bool left_tact_pressed, bool right_tact_pressed, const rclcpp::Time & current_time);
  std::vector<std::string> sensorxel_joy_names_;
  std::vector<std::string> state_interface_types_ = {"JOYSTICK X VALUE", "JOYSTICK Y VALUE",
    "JOYSTICK TACT SWITCH"};
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
  std::map<std::string, std::string> sensor_joint_state_stamped_topic_;
  std::map<std::string, std::vector<double>> sensor_last_active_positions_;
  std::map<std::string,
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr>
  sensor_joint_trajectory_publisher_;
  std::map<std::string,
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr>
  sensor_joint_state_stamped_publisher_;
  std::map<std::string,
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> sensorxel_joy_publisher_;

  // Add per-sensor jog scale
  std::map<std::string, double> sensor_jog_scale_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  bool prev_right_tact_switch_ = false;
  bool prev_left_tact_switch_ = false;

  // Long press functionality
  rclcpp::Time left_tact_press_start_time_;
  rclcpp::Time right_tact_press_start_time_;
  bool left_tact_long_press_triggered_ = false;
  bool right_tact_long_press_triggered_ = false;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tact_trigger_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr left_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr right_enable_pub_;

  std::atomic_bool middle_pedal_held_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr middle_pedal_sub_;

  RandomBaseController random_base_controller_;
  StationaryOdometryWindow random_base_stationary_window_;
  realtime_tools::RealtimeBuffer<OdometrySnapshot> odometry_buffer_;
  std::atomic<uint64_t> odometry_sequence_{0};
  std::atomic_bool reset_random_anchor_requested_{false};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_random_anchor_service_;
  std::string random_base_odom_frame_ = "odom";
  std::string random_base_child_frame_ = "base_link";
  OdometrySnapshot latest_odometry_;
  PlanarPose previous_odometry_pose_;
  uint64_t last_processed_odometry_sequence_ = 0;
  int64_t previous_odometry_stamp_ns_ = 0;
  int64_t last_update_time_ns_ = 0;
  bool latest_odometry_available_ = false;
  bool previous_odometry_available_ = false;
  bool last_random_base_auto_failed_ = false;
};

}  // namespace joystick_controller

#endif  // JOYSTICK_CONTROLLER__JOYSTICK_CONTROLLER_HPP_
