// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <algorithm>
#include <chrono>
#include <thread>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "dynamixel_interfaces/srv/set_data_to_dxl.hpp"

namespace ffw_leader_feedback
{

class LeaderFeedback : public rclcpp::Node
{
public:
  LeaderFeedback()
  : rclcpp::Node("leader_feedback"),
    left_enabled_(false),
    right_enabled_(false)
  {
    // Fetch initial enable state from broadcaster's parameters
    auto param_node_name = declare_parameter<std::string>(
      "param_node_name", "/leader/joint_trajectory_command_broadcaster");

    auto param_client = create_client<rcl_interfaces::srv::GetParameters>(
      param_node_name + "/get_parameters");

    RCLCPP_INFO(get_logger(),
      "Waiting for parameter service '%s/get_parameters'...",
      param_node_name.c_str());
    while (rclcpp::ok() &&
      !param_client->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_WARN(get_logger(), "Still waiting...");
    }

    if (rclcpp::ok()) {
      auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
      request->names = {"left_enabled_init", "right_enabled_init"};
      auto future = param_client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(
          get_node_base_interface(), future, std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS)
      {
        auto result = future.get();
        if (result->values.size() >= 2) {
          left_enabled_ = result->values[0].bool_value;
          right_enabled_ = result->values[1].bool_value;
          RCLCPP_INFO(get_logger(),
            "Loaded initial enable state: left=%s, right=%s",
            left_enabled_ ? "true" : "false", right_enabled_ ? "true" : "false");
        }
      } else {
        RCLCPP_WARN(get_logger(), "Failed to get enable params, using defaults (false)");
      }
    }

    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/leader/leader_position_controller/commands");
    left_enable_topic_ = declare_parameter<std::string>(
      "left_enable_topic", "/leader/left_command");
    right_enable_topic_ = declare_parameter<std::string>(
      "right_enable_topic", "/leader/right_command");
    left_follower_topic_ = declare_parameter<std::string>(
      "left_follower_joint_states_topic", "/robot/arm_left_follower/joint_states");
    right_follower_topic_ = declare_parameter<std::string>(
      "right_follower_joint_states_topic", "/robot/arm_right_follower/joint_states");
    torque_service_name_ = declare_parameter<std::string>(
      "torque_service", "/leader/dynamixel_hardware_interface/set_dxl_data");

    // Fetch command_joints from leader_position_controller's parameters
    auto leader_controller_name = declare_parameter<std::string>(
      "leader_position_controller_name", "/leader/leader_position_controller");
    auto joints_client = create_client<rcl_interfaces::srv::GetParameters>(
      leader_controller_name + "/get_parameters");
    RCLCPP_INFO(get_logger(),
      "Waiting for parameter service '%s/get_parameters'...",
      leader_controller_name.c_str());
    while (rclcpp::ok() &&
      !joints_client->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_WARN(get_logger(), "Still waiting...");
    }
    if (rclcpp::ok()) {
      auto req = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
      req->names = {"joints"};
      auto fut = joints_client->async_send_request(req);
      if (rclcpp::spin_until_future_complete(
          get_node_base_interface(), fut, std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS)
      {
        auto res = fut.get();
        if (!res->values.empty() &&
          res->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY)
        {
          command_joints_ = res->values[0].string_array_value;
          RCLCPP_INFO(get_logger(),
            "Loaded command_joints from %s: %zu joints",
            leader_controller_name.c_str(), command_joints_.size());
        }
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to get joints from %s",
          leader_controller_name.c_str());
      }
    }

    const std::vector<int64_t> default_right = {1, 2, 3, 4, 5, 6, 7};
    // const std::vector<int64_t> default_left = {31, 32, 33, 34, 35, 36, 37};
    const std::vector<int64_t> default_left = {91, 92, 93, 94, 95, 96, 97};
    auto right_ids_param =
      declare_parameter<std::vector<int64_t>>("right_ids", default_right);
    auto left_ids_param =
      declare_parameter<std::vector<int64_t>>("left_ids", default_left);
    for (auto id : right_ids_param) {right_ids_.push_back(static_cast<uint8_t>(id));}
    for (auto id : left_ids_param) {left_ids_.push_back(static_cast<uint8_t>(id));}

    // Torque service client: wait until available
    dxl_data_client_ = create_client<dynamixel_interfaces::srv::SetDataToDxl>(
      torque_service_name_);

    RCLCPP_INFO(get_logger(),
      "Waiting for torque service '%s' to become available...",
      torque_service_name_.c_str());
    while (rclcpp::ok() &&
      !dxl_data_client_->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_WARN(get_logger(),
        "Torque service '%s' not available yet, retrying...",
        torque_service_name_.c_str());
    }
    if (!rclcpp::ok()) {
      return;
    }
    RCLCPP_INFO(get_logger(), "Torque service available.");

    // Send initial torque command based on initial enable state (both false)
    send_torque_for_group(left_enabled_, left_ids_, "left");
    send_torque_for_group(right_enabled_, right_ids_, "right");

    // QoS
    auto be_qos = rclcpp::QoS(5).best_effort();

    // Enable subscribers:
    //   1 = torque OFF, 2 = toggle, else = torque ON
    //   (only call torque service when state actually changes)
    auto make_cb = [this](bool & enabled, std::vector<uint8_t> & ids, const char * tag) {
      return [this, &enabled, &ids, tag](std_msgs::msg::UInt8::SharedPtr msg) {
        bool new_state;
        switch (msg->data) {
          case 1:  new_state = true; break;
          case 2:  new_state = !enabled; break;
          default: new_state = false; break;
        }
        if (new_state != enabled) {
          enabled = new_state;
          send_torque_for_group(enabled, ids, tag);
        }
      };
    };
    left_enable_sub_ = create_subscription<std_msgs::msg::UInt8>(
      left_enable_topic_, rclcpp::SystemDefaultsQoS(),
      make_cb(left_enabled_, left_ids_, "left"));
    right_enable_sub_ = create_subscription<std_msgs::msg::UInt8>(
      right_enable_topic_, rclcpp::SystemDefaultsQoS(),
      make_cb(right_enabled_, right_ids_, "right"));

    // Follower subscribers — publish leader command directly on message arrival
    left_follower_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      left_follower_topic_, be_qos,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        update_positions_from_msg(msg);
        if (!left_enabled_ || !right_enabled_) {  // at least one side torque ON
          publish_position_command();
        }
      });

    right_follower_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      right_follower_topic_, be_qos,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        update_positions_from_msg(msg);
        if (!left_enabled_ || !right_enabled_) {  // at least one side torque ON
          publish_position_command();
        }
      });

    // Position command publisher
    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      command_topic_, rclcpp::QoS(5).reliable());

    RCLCPP_INFO(get_logger(),
      "leader_feedback started: cmd=%s follower_l=%s follower_r=%s",
      command_topic_.c_str(), left_follower_topic_.c_str(),
      right_follower_topic_.c_str());
  }

private:
  void update_positions_from_msg(const sensor_msgs::msg::JointState::SharedPtr & msg)
  {
    const size_t n = std::min(msg->name.size(), msg->position.size());
    for (size_t i = 0; i < n; ++i) {
      follower_positions_[msg->name[i]] = msg->position[i];
    }
  }

  void send_torque_for_group(
    bool group_enabled, const std::vector<uint8_t> & ids, const char * tag)
  {
    if (ids.empty()) {
      return;
    }
    if (!dxl_data_client_ || !dxl_data_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(),
        "[%s] set_dxl_data service not ready; skipping torque update", tag);
      return;
    }
    const uint32_t torque_value = group_enabled ? 0u : 1u;
    for (uint8_t id : ids) {
      auto req = std::make_shared<dynamixel_interfaces::srv::SetDataToDxl::Request>();
      req->id = id;
      req->item_name = "Torque Enable";
      req->item_data = torque_value;
      dxl_data_client_->async_send_request(req);
      // Throttle to avoid overflowing service queue
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    RCLCPP_INFO(get_logger(),
      "[%s] Torque Enable %u (motor_num %zu)", tag, torque_value, ids.size());
  }

  void publish_position_command()
  {
    if (command_joints_.empty() || !command_pub_) {
      return;
    }
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(command_joints_.size(),
      std::numeric_limits<double>::quiet_NaN());
    for (size_t i = 0; i < command_joints_.size(); ++i) {
      auto it = follower_positions_.find(command_joints_[i]);
      if (it != follower_positions_.end()) {
        msg.data[i] = it->second;
      }
    }
    command_pub_->publish(msg);
  }

  // State
  bool left_enabled_;
  bool right_enabled_;
  std::unordered_map<std::string, double> follower_positions_;

  // Params (cached)
  std::string command_topic_;
  std::string left_enable_topic_;
  std::string right_enable_topic_;
  std::string left_follower_topic_;
  std::string right_follower_topic_;
  std::string torque_service_name_;
  std::vector<std::string> command_joints_;
  std::vector<uint8_t> right_ids_;
  std::vector<uint8_t> left_ids_;

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr left_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr right_enable_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_follower_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_follower_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Client<dynamixel_interfaces::srv::SetDataToDxl>::SharedPtr dxl_data_client_;
};

}  // namespace ffw_leader_feedback

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ffw_leader_feedback::LeaderFeedback>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
