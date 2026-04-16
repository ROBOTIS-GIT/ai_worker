// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace ffw_gripper_trigger
{

class GripperTrigger : public rclcpp::Node
{
public:
  GripperTrigger()
  : rclcpp::Node("gripper_trigger")
  {
    gripper_threshold_ = declare_parameter<double>("gripper_threshold", -2.7);
    trajectory_duration_sec_ = declare_parameter<double>("trajectory_duration_sec", 2.0);

    leader_joint_states_topic_ = declare_parameter<std::string>(
      "leader_joint_states_topic", "/leader/joint_states");

    left_enable_topic_ = declare_parameter<std::string>(
      "left_enable_topic", "/leader/left_enable");
    right_enable_topic_ = declare_parameter<std::string>(
      "right_enable_topic", "/leader/right_enable");

    left_trajectory_topic_ = declare_parameter<std::string>(
      "left_trajectory_topic",
      "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory");
    right_trajectory_topic_ = declare_parameter<std::string>(
      "right_trajectory_topic",
      "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory");

    param_node_name_ = declare_parameter<std::string>(
      "param_node_name", "/leader/joint_trajectory_command_broadcaster");

    left_recovery_positions_ = declare_parameter<std::vector<double>>(
      "left_recovery_positions",
      std::vector<double>{0.75, 0.0, 0.0, -2.3, 0.0, 0.0, 0.0, 0.0});
    right_recovery_positions_ = declare_parameter<std::vector<double>>(
      "right_recovery_positions",
      std::vector<double>{0.75, 0.0, 0.0, -2.3, 0.0, 0.0, 0.0, 0.0});

    // Fetch joint names from broadcaster's parameters
    param_client_ = create_client<rcl_interfaces::srv::GetParameters>(
      param_node_name_ + "/get_parameters");

    RCLCPP_INFO(get_logger(),
      "Waiting for parameter service '%s/get_parameters'...",
      param_node_name_.c_str());
    while (rclcpp::ok() &&
      !param_client_->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_WARN(get_logger(), "Still waiting...");
    }
    if (!rclcpp::ok()) {
      return;
    }

    fetch_joint_params();

    // Publishers
    auto qos = rclcpp::SystemDefaultsQoS();
    left_enable_pub_ = create_publisher<std_msgs::msg::UInt8>(left_enable_topic_, qos);
    right_enable_pub_ = create_publisher<std_msgs::msg::UInt8>(right_enable_topic_, qos);
    left_traj_pub_ =
      create_publisher<trajectory_msgs::msg::JointTrajectory>(left_trajectory_topic_, qos);
    right_traj_pub_ =
      create_publisher<trajectory_msgs::msg::JointTrajectory>(right_trajectory_topic_, qos);

    // Subscriber
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      leader_joint_states_topic_, rclcpp::SystemDefaultsQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        const size_t n = msg->position.size();
        if (n < 2) {
          return;
        }
        check_and_trigger(msg->position[n - 2], left_joints_, left_recovery_positions_,
          left_enable_pub_, left_traj_pub_, "left");
        check_and_trigger(msg->position[n - 1], right_joints_, right_recovery_positions_,
          right_enable_pub_, right_traj_pub_, "right");
      });

    RCLCPP_INFO(get_logger(),
      "gripper_trigger started (threshold=%.2f, left_joints=%zu, right_joints=%zu)",
      gripper_threshold_, left_joints_.size(), right_joints_.size());
  }

private:
  void fetch_joint_params()
  {
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"left_joints", "right_joints"};

    auto future = param_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
        get_node_base_interface(), future, std::chrono::seconds(5)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to get parameters from %s", param_node_name_.c_str());
      return;
    }

    auto result = future.get();
    for (size_t i = 0; i < result->values.size(); ++i) {
      const auto & val = result->values[i];
      if (val.type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY) {
        if (request->names[i] == "left_joints") {
          left_joints_ = val.string_array_value;
          RCLCPP_INFO(get_logger(), "Loaded left_joints: %zu joints", left_joints_.size());
        } else if (request->names[i] == "right_joints") {
          right_joints_ = val.string_array_value;
          RCLCPP_INFO(get_logger(), "Loaded right_joints: %zu joints", right_joints_.size());
        }
      }
    }
  }

  void check_and_trigger(
    double gripper_pos,
    const std::vector<std::string> & joints,
    const std::vector<double> & recovery_positions,
    const rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr & enable_pub,
    const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr & traj_pub,
    const char * tag)
  {
    if (joints.empty() || std::isnan(gripper_pos) || gripper_pos >= gripper_threshold_) {
      return;
    }

    RCLCPP_WARN(get_logger(),
      "[%s] Gripper below threshold (%.2f < %.2f)", tag, gripper_pos, gripper_threshold_);

    std_msgs::msg::UInt8 enable_msg;
    enable_msg.data = 0;
    enable_pub->publish(enable_msg);

    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.header.stamp = rclcpp::Time(0, 0);
    traj_msg.joint_names = joints;
    traj_msg.points.resize(1);
    traj_msg.points[0].positions = recovery_positions;
    const int32_t sec = static_cast<int32_t>(trajectory_duration_sec_);
    const uint32_t nanosec =
      static_cast<uint32_t>((trajectory_duration_sec_ - sec) * 1e9);
    traj_msg.points[0].time_from_start = rclcpp::Duration(sec, nanosec);
    traj_pub->publish(traj_msg);
  }

  // Params
  double gripper_threshold_;
  double trajectory_duration_sec_;
  std::string leader_joint_states_topic_;
  std::string left_enable_topic_;
  std::string right_enable_topic_;
  std::string left_trajectory_topic_;
  std::string right_trajectory_topic_;
  std::string param_node_name_;
  std::vector<std::string> left_joints_;
  std::vector<std::string> right_joints_;
  std::vector<double> left_recovery_positions_;
  std::vector<double> right_recovery_positions_;

  // ROS interfaces
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr param_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr left_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr right_enable_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_traj_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_traj_pub_;
};

}  // namespace ffw_gripper_trigger

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ffw_gripper_trigger::GripperTrigger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
