// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
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
    right_enabled_(false),
    already_left_t_set_(false),
    already_right_t_set_(false)
  {
    // Parameters
    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/leader/leader_position_controller/commands");
    left_enable_topic_ = declare_parameter<std::string>(
      "left_enable_topic", "/leader/left_enable");
    right_enable_topic_ = declare_parameter<std::string>(
      "right_enable_topic", "/leader/right_enable");
    left_follower_topic_ = declare_parameter<std::string>(
      "left_follower_joint_states_topic", "/robot/arm_left_follower/joint_states");
    right_follower_topic_ = declare_parameter<std::string>(
      "right_follower_joint_states_topic", "/robot/arm_right_follower/joint_states");
    torque_service_name_ = declare_parameter<std::string>(
      "torque_service", "/leader/dynamixel_hardware_interface/set_dxl_data");

    command_joints_ = declare_parameter<std::vector<std::string>>(
      "command_joints",
      std::vector<std::string>{
      "arm_r_joint1", "arm_r_joint2", "arm_r_joint3", "arm_r_joint4",
      "arm_r_joint5", "arm_r_joint6", "arm_r_joint7",
      "arm_l_joint1", "arm_l_joint2", "arm_l_joint3", "arm_l_joint4",
      "arm_l_joint5", "arm_l_joint6", "arm_l_joint7"});

    const std::vector<int64_t> default_right = {1, 2, 3, 4, 5, 6, 7};
    // const std::vector<int64_t> default_left = {31, 32, 33, 34, 35, 36, 37};
    const std::vector<int64_t> default_left = {91, 92, 93, 94, 95, 96, 97};
    auto right_ids_param =
      declare_parameter<std::vector<int64_t>>("right_ids", default_right);
    auto left_ids_param =
      declare_parameter<std::vector<int64_t>>("left_ids", default_left);
    for (auto id : right_ids_param) {right_ids_.push_back(static_cast<uint8_t>(id));}
    for (auto id : left_ids_param) {left_ids_.push_back(static_cast<uint8_t>(id));}

    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 50.0);

    // QoS: Best Effort for follower state streams (avoid backpressure)
    auto be_qos = rclcpp::QoS(5).best_effort();

    // Enable subscribers: small bool stream, reliable is fine
    auto enable_qos = rclcpp::QoS(5);
    left_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      left_enable_topic_, enable_qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data != left_enabled_) {
          already_left_t_set_ = false;
        }
        left_enabled_ = msg->data;
      });

    right_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      right_enable_topic_, enable_qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data != right_enabled_) {
          already_right_t_set_ = false;
        }
        right_enabled_ = msg->data;
      });

    // Follower joint state subscribers (best effort)
    left_follower_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      left_follower_topic_, be_qos,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        update_positions_from_msg(msg);
      });

    right_follower_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      right_follower_topic_, be_qos,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        update_positions_from_msg(msg);
      });

    // Position command publisher
    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      command_topic_, rclcpp::QoS(5).reliable());

    // Torque service client
    dxl_data_client_ = create_client<dynamixel_interfaces::srv::SetDataToDxl>(
      torque_service_name_);

    const auto period_s = 1.0 / std::max(1.0, update_rate_hz_);
    const auto period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(period_s));
    timer_ = create_wall_timer(period, [this]() {on_timer();});

    RCLCPP_INFO(get_logger(),
      "leader_feedback started: cmd=%s follower_l=%s follower_r=%s rate=%.1fHz",
      command_topic_.c_str(), left_follower_topic_.c_str(),
      right_follower_topic_.c_str(), update_rate_hz_);
  }

private:
  void update_positions_from_msg(const sensor_msgs::msg::JointState::SharedPtr & msg)
  {
    const size_t n = std::min(msg->name.size(), msg->position.size());
    for (size_t i = 0; i < n; ++i) {
      follower_positions_[msg->name[i]] = msg->position[i];
    }
  }

  void send_torque_enable(
    const std::vector<uint8_t> & ids, uint32_t value, const char * tag)
  {
    if (!dxl_data_client_ || !dxl_data_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[%s] set_dxl_data service not ready", tag);
      return;
    }
    for (uint8_t id : ids) {
      auto req = std::make_shared<dynamixel_interfaces::srv::SetDataToDxl::Request>();
      req->id = id;
      req->item_name = "Torque Enable";
      req->item_data = value;
      dxl_data_client_->async_send_request(req);
    }
    RCLCPP_INFO(get_logger(),
      "[%s] Torque Enable -> %u for %zu IDs", tag, value, ids.size());
  }

  void handle_group_torque(
    bool group_enabled, bool & already_set,
    const std::vector<uint8_t> & ids, const char * tag)
  {
    if (already_set || ids.empty()) {
      return;
    }
    const uint32_t torque_value = group_enabled ? 0u : 1u;
    if (!dxl_data_client_ || !dxl_data_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[%s] set_dxl_data service not ready", tag);
      return;
    }
    send_torque_enable(ids, torque_value, tag);
    already_set = true;
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

  void on_timer()
  {
    handle_group_torque(left_enabled_, already_left_t_set_, left_ids_, "left");
    handle_group_torque(right_enabled_, already_right_t_set_, right_ids_, "right");
    publish_position_command();
  }

  // State
  bool left_enabled_;
  bool right_enabled_;
  bool already_left_t_set_;
  bool already_right_t_set_;
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
  double update_rate_hz_;

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_enable_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_follower_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_follower_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Client<dynamixel_interfaces::srv::SetDataToDxl>::SharedPtr dxl_data_client_;
  rclcpp::TimerBase::SharedPtr timer_;
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
