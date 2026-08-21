// Copyright 2021 ros2_control development team
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

#include "a2_joint_trajectory_command_broadcaster/a2_joint_trajectory_command_broadcaster.hpp"

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <functional>
#include <cmath>
#include <algorithm>
#include <iterator>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "urdf/model.h"

namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

namespace a2_joint_trajectory_command_broadcaster
{
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
using hardware_interface::HW_IF_POSITION;

namespace
{
constexpr uint8_t kLeftArm = 1;
constexpr uint8_t kRightArm = 2;
constexpr uint8_t kBothArms = kLeftArm | kRightArm;

uint8_t arms_from_name(const std::string & name)
{
  if (name == "left") {
    return kLeftArm;
  }
  if (name == "right") {
    return kRightArm;
  }
  if (name == "both") {
    return kBothArms;
  }
  return 0;
}

std::string arms_to_name(const uint8_t arms)
{
  switch (arms & kBothArms) {
    case kLeftArm:
      return "left";
    case kRightArm:
      return "right";
    case kBothArms:
      return "both";
    default:
      return "none";
  }
}

bool initial_pose_state_busy(const uint8_t state)
{
  return state == robotis_interfaces::msg::ControlModeStatus::INITIAL_POSE_WAITING ||
         state == robotis_interfaces::msg::ControlModeStatus::INITIAL_POSE_MOVING;
}

bool preset_state_busy(const uint8_t state)
{
  return state == robotis_interfaces::msg::ControlModeStatus::PRESET_WAITING ||
         state == robotis_interfaces::msg::ControlModeStatus::PRESET_MOVING;
}
}  // namespace

A2JointTrajectoryCommandBroadcaster::A2JointTrajectoryCommandBroadcaster() {}

controller_interface::CallbackReturn A2JointTrajectoryCommandBroadcaster::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
A2JointTrajectoryCommandBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration A2JointTrajectoryCommandBroadcaster::
state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : params_.left_joints) {
    state_interfaces_config.names.push_back(joint + "/" + HW_IF_POSITION);
  }
  for (const auto & joint : params_.right_joints) {
    state_interfaces_config.names.push_back(joint + "/" + HW_IF_POSITION);
  }
  return state_interfaces_config;
}

controller_interface::CallbackReturn A2JointTrajectoryCommandBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  // Map interface if needed
  map_interface_to_joint_state_.clear();
  map_interface_to_joint_state_[HW_IF_POSITION] = params_.map_interface_to_joint_state.position;

  try {
    // Create publishers for left and right groups
    std::vector<std::string> groups = {"left", "right"};

    for (const auto & group_name : groups) {
      // Get joints for this group
      std::vector<std::string> group_joints;
      if (group_name == "left" && !params_.left_joints.empty()) {
        group_joints = params_.left_joints;
      } else if (group_name == "right" && !params_.right_joints.empty()) {
        group_joints = params_.right_joints;
      }

      if (group_joints.empty()) {
        continue;  // Skip empty groups
      }

      group_joint_names_[group_name] = group_joints;

      // Get offsets for this group
      if (group_name == "left" && !params_.left_offsets.empty()) {
        group_joint_offsets_[group_name] = params_.left_offsets;
      } else if (group_name == "right" && !params_.right_offsets.empty()) {
        group_joint_offsets_[group_name] = params_.right_offsets;
      } else {
        // Initialize empty offsets if not provided
        group_joint_offsets_[group_name] = std::vector<double>();
      }

      // Get reverse joints for this group
      if (group_name == "left" && !params_.left_reverse_joints.empty()) {
        group_reverse_joints_[group_name] = params_.left_reverse_joints;
      } else if (group_name == "right" && !params_.right_reverse_joints.empty()) {
        group_reverse_joints_[group_name] = params_.right_reverse_joints;
      } else {
        // Initialize empty reverse joints if not provided
        group_reverse_joints_[group_name] = std::vector<std::string>();
      }

      // Create topic name with group-specific namespace
      std::string topic_name;
      topic_name = "joint_trajectory_command_broadcaster_" + group_name + "/" +
        params_.output_topic_suffix;
      group_topic_names_[group_name] = topic_name;

      // Create publisher for this group
      joint_trajectory_publishers_[group_name] =
        get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        topic_name, rclcpp::SystemDefaultsQoS());

      realtime_joint_trajectory_publishers_[group_name] =
        std::make_shared<realtime_tools::RealtimePublisher<trajectory_msgs::msg::JointTrajectory>>(
        joint_trajectory_publishers_[group_name]);

      RCLCPP_INFO(
        get_node()->get_logger(),
        "Created joint trajectory publisher for group '%s' on topic: %s with %zu joints",
        group_name.c_str(), topic_name.c_str(), group_joints.size());
    }

    // Store the groups for later use
    trajectory_groups_ = groups;

    // Create subscriber for follower joint states
    joint_states_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      params_.follower_joint_states_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&A2JointTrajectoryCommandBroadcaster::joint_states_callback, this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Subscribed to follower joint states topic: %s",
      params_.follower_joint_states_topic.c_str());

    if (params_.enable_teleoperation) {
      requested_control_mode_ =
        static_cast<uint16_t>(params_.default_control_mode);
      left_preset_id_ = static_cast<uint16_t>(params_.default_left_preset_id);
      right_preset_id_ = static_cast<uint16_t>(params_.default_right_preset_id);
      requested_arms_ = 0;
      active_arms_ = 0;
      initial_pose_available_arms_ = 0;
      initial_pose_busy_arms_ = 0;
      preset_busy_arms_ = 0;
      requested_arms_rt_.store(0, std::memory_order_relaxed);
      previous_requested_arms_rt_ = 0;
      unsynced_arms_rt_ = 0;
      left_initial_pose_trigger_ = TriggerHoldState{};
      right_initial_pose_trigger_ = TriggerHoldState{};
      auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
      control_command_publisher_ =
        get_node()->create_publisher<robotis_interfaces::msg::ControlModeCommand>(
        params_.control_command_topic, command_qos);
      teleoperation_command_subscriber_ =
        get_node()->create_subscription<robotis_interfaces::msg::TeleoperationCommand>(
        params_.joystick_command_topic, rclcpp::SystemDefaultsQoS(),
        std::bind(
          &A2JointTrajectoryCommandBroadcaster::teleoperation_command_callback,
          this, std::placeholders::_1));
      control_status_subscriber_ =
        get_node()->create_subscription<robotis_interfaces::msg::ControlModeStatus>(
        params_.control_status_topic, command_qos,
        std::bind(
          &A2JointTrajectoryCommandBroadcaster::control_status_callback,
          this, std::placeholders::_1));
      set_control_mode_service_ =
        get_node()->create_service<robotis_interfaces::srv::SetControlMode>(
        params_.set_mode_service,
        std::bind(
          &A2JointTrajectoryCommandBroadcaster::set_control_mode_callback,
          this, std::placeholders::_1, std::placeholders::_2));
      set_teleoperation_service_ =
        get_node()->create_service<robotis_interfaces::srv::SetTeleoperation>(
        params_.set_teleoperation_service,
        std::bind(
          &A2JointTrajectoryCommandBroadcaster::set_teleoperation_callback,
          this, std::placeholders::_1, std::placeholders::_2));
      set_preset_service_ =
        get_node()->create_service<robotis_interfaces::srv::SetPreset>(
        params_.set_preset_service,
        std::bind(
          &A2JointTrajectoryCommandBroadcaster::set_preset_callback,
          this, std::placeholders::_1, std::placeholders::_2));
      RCLCPP_INFO(
        get_node()->get_logger(),
        "Cyclo teleoperation enabled with default control mode %u",
        static_cast<unsigned int>(requested_control_mode_));
    }
  } catch (const std::exception & e) {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  const std::string & urdf = get_robot_description();
  is_model_loaded_ = !urdf.empty() && model_.initString(urdf);
  if (!is_model_loaded_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to parse robot description. Will proceed without URDF-based filtering.");
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn A2JointTrajectoryCommandBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!init_joint_data()) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "None of requested interfaces exist. Controller will not run.");
    return CallbackReturn::ERROR;
  }

  // Check offsets for each group
  for (const auto & group_name : trajectory_groups_) {
    const auto & group_joints = group_joint_names_[group_name];
    const size_t num_joints = group_joints.size();

    if (group_joint_offsets_[group_name].empty()) {
      // If no offsets provided, use zeros
      group_joint_offsets_[group_name].assign(num_joints, 0.0);
    } else if (group_joint_offsets_[group_name].size() != num_joints) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "The number of provided offsets (%zu) for group '%s' does not match the number of "
        "joints (%zu).",
        group_joint_offsets_[group_name].size(), group_name.c_str(), num_joints);
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Group '%s' configured with %zu joints and %zu offsets",
      group_name.c_str(), num_joints, group_joint_offsets_[group_name].size());
  }

  // No need to init JointState or DynamicJointState messages, only JointTrajectory
  // will be published. We'll construct it on-the-fly in update()

  if (params_.enable_teleoperation) {
    publish_control_command();
  }

  return CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn A2JointTrajectoryCommandBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_.clear();
  name_if_value_mapping_.clear();
  group_joint_names_.clear();
  group_joint_offsets_.clear();
  group_topic_names_.clear();
  group_reverse_joints_.clear();

  return CallbackReturn::SUCCESS;
}

template<typename T>
bool has_any_key(
  const std::unordered_map<std::string, T> & map, const std::vector<std::string> & keys)
{
  for (const auto & key_item : map) {
    const auto & key = key_item.first;
    if (std::find(keys.cbegin(), keys.cend(), key) != keys.cend()) {
      return true;
    }
  }
  return false;
}

bool A2JointTrajectoryCommandBroadcaster::init_joint_data()
{
  joint_names_.clear();
  if (state_interfaces_.empty()) {
    return false;
  }

  // Initialize mapping
  for (auto si = state_interfaces_.crbegin(); si != state_interfaces_.crend(); si++) {
    if (name_if_value_mapping_.count(si->get_prefix_name()) == 0) {
      name_if_value_mapping_[si->get_prefix_name()] = {};
    }
    std::string interface_name = si->get_interface_name();
    if (map_interface_to_joint_state_.count(interface_name) > 0) {
      interface_name = map_interface_to_joint_state_[interface_name];
    }
    name_if_value_mapping_[si->get_prefix_name()][interface_name] = kUninitializedValue;
  }

  // Filter out joints without position interface (since we want positions)
  for (const auto & name_ifv : name_if_value_mapping_) {
    const auto & interfaces_and_values = name_ifv.second;
    if (has_any_key(interfaces_and_values, {HW_IF_POSITION})) {
      if (
        !params_.use_urdf_to_filter || !is_model_loaded_ ||
        model_.getJoint(name_ifv.first))
      {
        joint_names_.push_back(name_ifv.first);
      }
    }
  }

  return true;
}

double get_value(
  const std::unordered_map<std::string, std::unordered_map<std::string, double>> & map,
  const std::string & name, const std::string & interface_name)
{
  const auto & interfaces_and_values = map.at(name);
  const auto interface_and_value = interfaces_and_values.find(interface_name);
  if (interface_and_value != interfaces_and_values.cend()) {
    return interface_and_value->second;
  } else {
    return kUninitializedValue;
  }
}

void A2JointTrajectoryCommandBroadcaster::joint_states_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Update follower joint positions
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (i < msg->position.size()) {
      follower_joint_positions_[msg->name[i]] = msg->position[i];
    }
  }

  // Debug logging (only log occasionally to avoid spam)
  static int callback_count = 0;
  if (++callback_count % 100 == 0) {
    RCLCPP_DEBUG(get_node()->get_logger(),
      "Received follower joint states for %zu joints", msg->name.size());
  }
}

double A2JointTrajectoryCommandBroadcaster::calculate_mean_error() const
{
  // Check if we have received any follower joint states
  if (follower_joint_positions_.empty()) {
    return std::numeric_limits<double>::max();  // Return max error if no follower data
  }

  double total_error = 0.0;
  int valid_joints = 0;

  // Calculate mean error across all joints in all groups
  for (const auto & group_pair : group_joint_names_) {
    const auto & group_name = group_pair.first;
    const auto & group_joints = group_pair.second;
    // Safely get group offsets and reverse joints
    std::vector<double> group_offsets;
    std::vector<std::string> group_reverse_joints;

    auto offsets_it = group_joint_offsets_.find(group_name);
    if (offsets_it != group_joint_offsets_.end()) {
      group_offsets = offsets_it->second;
    }

    auto reverse_it = group_reverse_joints_.find(group_name);
    if (reverse_it != group_reverse_joints_.end()) {
      group_reverse_joints = reverse_it->second;
    }

    for (size_t i = 0; i < group_joints.size(); ++i) {
      const auto & joint_name = group_joints[i];
      auto follower_it = follower_joint_positions_.find(joint_name);
      if (follower_it == follower_joint_positions_.end()) {
        continue;  // Skip joints not available in follower
      }

      double leader_pos = get_value(name_if_value_mapping_, joint_name, HW_IF_POSITION);
      if (std::isnan(leader_pos)) {
        continue;  // Skip joints without valid leader position
      }

      // Apply reverse and offset to leader position for comparison
      if (std::find(group_reverse_joints.begin(), group_reverse_joints.end(), joint_name) !=
        group_reverse_joints.end())
      {
        leader_pos = -leader_pos;
      }

      // Apply group offset
      if (i < group_offsets.size()) {
        leader_pos += group_offsets[i];
      }

      total_error += std::abs(leader_pos - follower_it->second);
      valid_joints++;
    }
  }

  return valid_joints > 0 ? total_error / valid_joints : std::numeric_limits<double>::max();
}

double A2JointTrajectoryCommandBroadcaster::calculate_group_mean_error(
  const std::string & group_name) const
{
  if (follower_joint_positions_.empty()) {
    return std::numeric_limits<double>::max();
  }

  const auto joints_it = group_joint_names_.find(group_name);
  if (joints_it == group_joint_names_.end()) {
    return std::numeric_limits<double>::max();
  }

  const auto offsets_it = group_joint_offsets_.find(group_name);
  const auto reverse_it = group_reverse_joints_.find(group_name);
  const std::vector<double> empty_offsets;
  const std::vector<std::string> empty_reverse_joints;
  const auto & offsets = offsets_it == group_joint_offsets_.end() ?
    empty_offsets : offsets_it->second;
  const auto & reverse_joints = reverse_it == group_reverse_joints_.end() ?
    empty_reverse_joints : reverse_it->second;

  double total_error = 0.0;
  size_t valid_joints = 0;
  for (size_t i = 0; i < joints_it->second.size(); ++i) {
    const auto & joint_name = joints_it->second[i];
    // Gripper commands remain immediate in MoveJ and must not prolong arm synchronization.
    if (joint_name.find("gripper") != std::string::npos) {
      continue;
    }
    const auto follower_it = follower_joint_positions_.find(joint_name);
    if (follower_it == follower_joint_positions_.end()) {
      continue;
    }

    double leader_position = get_value(name_if_value_mapping_, joint_name, HW_IF_POSITION);
    if (std::isnan(leader_position)) {
      continue;
    }
    if (
      std::find(reverse_joints.begin(), reverse_joints.end(), joint_name) !=
      reverse_joints.end())
    {
      leader_position = -leader_position;
    }
    if (i < offsets.size()) {
      leader_position += offsets[i];
    }
    total_error += std::abs(leader_position - follower_it->second);
    ++valid_joints;
  }
  return valid_joints > 0 ?
         total_error / static_cast<double>(valid_joints) :
         std::numeric_limits<double>::max();
}

bool A2JointTrajectoryCommandBroadcaster::check_trigger_active() const
{
  // Check if gripper trigger joints are above threshold
  double gripper_r_pos = get_value(name_if_value_mapping_, "gripper_r_joint1", HW_IF_POSITION);
  double gripper_l_pos = get_value(name_if_value_mapping_, "gripper_l_joint1", HW_IF_POSITION);

  // Return true if both grippers are above threshold
  return (!std::isnan(gripper_r_pos) &&
         gripper_r_pos * params_.trigger_sign >=
         params_.trigger_threshold * params_.trigger_sign) &&
         (!std::isnan(gripper_l_pos) &&
         gripper_l_pos * params_.trigger_sign >= params_.trigger_threshold * params_.trigger_sign);
}

void A2JointTrajectoryCommandBroadcaster::update_trigger_state(const rclcpp::Time & current_time)
{
  bool current_trigger_active = check_trigger_active();

  if (current_trigger_active && !trigger_counting_) {
    // Start trigger counting (only if mode hasn't changed in this trigger session)
    if (!mode_changed_in_this_trigger_) {
      trigger_counting_ = true;
      trigger_start_time_ = current_time;
      RCLCPP_INFO(get_node()->get_logger(), "Trigger activated - counting started");
    }
  } else if (!current_trigger_active) {
    // Trigger released - reset all states
    if (trigger_counting_) {
      trigger_counting_ = false;
      RCLCPP_INFO(get_node()->get_logger(), "Trigger released - counting stopped");
    }
    // Reset for next trigger session when trigger is completely released
    mode_changed_in_this_trigger_ = false;
  }

  // Check if trigger has been held for specified duration and mode hasn't changed in this session
  if (trigger_counting_ && !mode_changed_in_this_trigger_ &&
    (current_time - trigger_start_time_) >=
    rclcpp::Duration::from_seconds(params_.trigger_duration))
  {
    // Toggle auto mode state
    if (auto_mode_ == AutoMode::STOPPED) {
      auto_mode_ = AutoMode::ACTIVE;
      // Reset sync state when starting auto mode
      joints_synced_ = false;
      first_publish_ = true;
      RCLCPP_INFO(get_node()->get_logger(),
          "Auto mode ACTIVATED - follower will slowly follow leader");
    } else {
      auto_mode_ = AutoMode::STOPPED;
      RCLCPP_INFO(get_node()->get_logger(), "Auto mode STOPPED - follower paused");
    }

    // Mark that mode has changed in this trigger session
    mode_changed_in_this_trigger_ = true;
    trigger_counting_ = false;  // Stop counting
  }
}

bool A2JointTrajectoryCommandBroadcaster::check_arm_trigger_active(const uint8_t arm) const
{
  const char * joint_name = arm == kLeftArm ? "gripper_l_joint1" : "gripper_r_joint1";
  const double position = get_value(name_if_value_mapping_, joint_name, HW_IF_POSITION);
  return !std::isnan(position) &&
         position * params_.trigger_sign >=
         params_.trigger_threshold * params_.trigger_sign;
}

void A2JointTrajectoryCommandBroadcaster::request_final_initial_pose(
  const uint8_t target_arms)
{
  uint8_t accepted_arms = 0;
  uint8_t unavailable_arms = 0;
  uint8_t busy_arms = 0;
  {
    std::lock_guard<std::mutex> lock(teleoperation_mutex_);
    unavailable_arms = target_arms & static_cast<uint8_t>(~initial_pose_available_arms_);
    busy_arms = target_arms & (initial_pose_busy_arms_ | preset_busy_arms_);
    accepted_arms = target_arms & initial_pose_available_arms_ &
      static_cast<uint8_t>(~(initial_pose_busy_arms_ | preset_busy_arms_));
    if (accepted_arms != 0) {
      requested_arms_ &= static_cast<uint8_t>(~accepted_arms);
      initial_pose_busy_arms_ |= accepted_arms;
      ++transition_id_;
    }
  }

  if (unavailable_arms != 0) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Ignoring %s trigger: current control mode has no enabled initial pose for that arm",
      arms_to_name(unavailable_arms).c_str());
  }
  if (busy_arms != 0) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Ignoring %s trigger: another pose movement is already in progress",
      arms_to_name(busy_arms).c_str());
  }
  if (accepted_arms == 0) {
    return;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "%s trigger held; moving the selected arm to the current mode's final initial pose",
    arms_to_name(accepted_arms).c_str());
  publish_control_command("none", arms_to_name(accepted_arms));
}

void A2JointTrajectoryCommandBroadcaster::update_initial_pose_trigger_state(
  const rclcpp::Time & current_time)
{
  uint8_t triggered_arms = 0;
  auto update_arm =
    [this, &current_time, &triggered_arms](
    const uint8_t arm, TriggerHoldState & state)
    {
      const bool active = check_arm_trigger_active(arm);
      if (!active) {
        state = TriggerHoldState{};
        return;
      }
      if (state.triggered) {
        return;
      }
      if (!state.counting) {
        state.start_time = current_time;
        state.counting = true;
        return;
      }
      if (
        (current_time - state.start_time) >=
        rclcpp::Duration::from_seconds(params_.trigger_duration))
      {
        state.counting = false;
        state.triggered = true;
        triggered_arms |= arm;
      }
    };

  update_arm(kLeftArm, left_initial_pose_trigger_);
  update_arm(kRightArm, right_initial_pose_trigger_);
  if (triggered_arms != 0) {
    request_final_initial_pose(triggered_arms);
  }
}

bool A2JointTrajectoryCommandBroadcaster::check_joints_synced() const
{
  double mean_error = calculate_mean_error();
  return mean_error <= params_.sync_threshold;
}

void A2JointTrajectoryCommandBroadcaster::publish_control_command(
  const std::string & preset_target_arm,
  const std::string & initial_pose_target_arm)
{
  if (!control_command_publisher_) {
    return;
  }

  robotis_interfaces::msg::ControlModeCommand command;
  uint8_t requested_arms = 0;
  {
    std::lock_guard<std::mutex> lock(teleoperation_mutex_);
    requested_arms = requested_arms_;
    command.transition_id = transition_id_;
    command.control_mode = requested_control_mode_;
    command.enabled_arms = arms_to_name(requested_arms);
    command.preset_target_arm = preset_target_arm;
    command.initial_pose_target_arm = initial_pose_target_arm;
    command.left_preset_id = left_preset_id_;
    command.right_preset_id = right_preset_id_;
  }
  requested_arms_rt_.store(requested_arms, std::memory_order_release);
  control_command_publisher_->publish(command);
}

void A2JointTrajectoryCommandBroadcaster::teleoperation_command_callback(
  const robotis_interfaces::msg::TeleoperationCommand::SharedPtr msg)
{
  if (!msg || !params_.enable_teleoperation) {
    return;
  }
  const uint8_t target_arms = arms_from_name(msg->target_arm);
  if (target_arms == 0) {
    RCLCPP_WARN(
      get_node()->get_logger(), "Ignoring teleoperation command with invalid target_arm: %s",
      msg->target_arm.c_str());
    return;
  }

  bool changed = false;
  uint8_t blocked_arms = 0;
  {
    std::lock_guard<std::mutex> lock(teleoperation_mutex_);
    const uint8_t previous_arms = requested_arms_;
    switch (msg->command) {
      case robotis_interfaces::msg::TeleoperationCommand::COMMAND_ENABLE: {
          blocked_arms = target_arms & initial_pose_busy_arms_;
          requested_arms_ |= target_arms & static_cast<uint8_t>(~blocked_arms);
          break;
        }
      case robotis_interfaces::msg::TeleoperationCommand::COMMAND_DISABLE:
        requested_arms_ &= static_cast<uint8_t>(~target_arms);
        break;
      case robotis_interfaces::msg::TeleoperationCommand::COMMAND_TOGGLE: {
          const uint8_t disable_arms = target_arms & requested_arms_;
          const uint8_t requested_enable_arms = target_arms &
            static_cast<uint8_t>(~requested_arms_);
          blocked_arms = requested_enable_arms & initial_pose_busy_arms_;
          const uint8_t enable_arms = requested_enable_arms &
            static_cast<uint8_t>(~blocked_arms);
          requested_arms_ &= static_cast<uint8_t>(~disable_arms);
          requested_arms_ |= enable_arms;
          break;
        }
      default:
        RCLCPP_WARN(
          get_node()->get_logger(), "Ignoring unknown teleoperation command: %u",
          static_cast<unsigned int>(msg->command));
        return;
    }
    changed = previous_arms != requested_arms_;
    if (changed) {
      ++transition_id_;
    }
  }
  if (blocked_arms != 0) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Ignoring teleoperation enable for %s while initial pose movement is in progress",
      arms_to_name(blocked_arms).c_str());
  }
  if (!changed) {
    return;
  }
  publish_control_command();
}

void A2JointTrajectoryCommandBroadcaster::set_control_mode_callback(
  const std::shared_ptr<robotis_interfaces::srv::SetControlMode::Request> request,
  std::shared_ptr<robotis_interfaces::srv::SetControlMode::Response> response)
{
  if (!params_.enable_teleoperation || request->control_mode == 0) {
    std::lock_guard<std::mutex> lock(teleoperation_mutex_);
    response->accepted = false;
    response->transition_id = transition_id_;
    response->message = "Teleoperation is disabled or control mode zero was requested";
    return;
  }
  {
    std::lock_guard<std::mutex> lock(teleoperation_mutex_);
    if (
      requested_arms_ != 0 || active_arms_ != 0 ||
      initial_pose_busy_arms_ != 0 || preset_busy_arms_ != 0)
    {
      response->accepted = false;
      response->transition_id = transition_id_;
      response->message =
        "Control mode can only be changed while both arms are stopped and no initial pose "
        "movement is in progress";
      return;
    }
    requested_control_mode_ = request->control_mode;
    initial_pose_available_arms_ = 0;
    response->accepted = true;
    response->transition_id = ++transition_id_;
    response->message = "Control mode request accepted";
  }
  publish_control_command();
}

void A2JointTrajectoryCommandBroadcaster::set_teleoperation_callback(
  const std::shared_ptr<robotis_interfaces::srv::SetTeleoperation::Request> request,
  std::shared_ptr<robotis_interfaces::srv::SetTeleoperation::Response> response)
{
  const uint8_t target_arms = arms_from_name(request->target_arm);
  if (!params_.enable_teleoperation || target_arms == 0) {
    std::lock_guard<std::mutex> lock(teleoperation_mutex_);
    response->accepted = false;
    response->transition_id = transition_id_;
    response->message = "Teleoperation is disabled or target_arm is invalid";
    return;
  }
  {
    std::lock_guard<std::mutex> lock(teleoperation_mutex_);
    if (request->enabled) {
      const uint8_t blocked_arms = target_arms & initial_pose_busy_arms_;
      if (blocked_arms != 0) {
        response->accepted = false;
        response->transition_id = transition_id_;
        response->message =
          "Teleoperation cannot be enabled while initial pose movement is in progress";
        return;
      }
      requested_arms_ |= target_arms;
    } else {
      requested_arms_ &= static_cast<uint8_t>(~target_arms);
    }
    response->accepted = true;
    response->transition_id = ++transition_id_;
    response->message = "Arm state request accepted";
  }
  publish_control_command();
}

void A2JointTrajectoryCommandBroadcaster::set_preset_callback(
  const std::shared_ptr<robotis_interfaces::srv::SetPreset::Request> request,
  std::shared_ptr<robotis_interfaces::srv::SetPreset::Response> response)
{
  const uint8_t target_arms = arms_from_name(request->target_arm);
  if (!params_.enable_teleoperation || target_arms == 0) {
    std::lock_guard<std::mutex> lock(teleoperation_mutex_);
    response->accepted = false;
    response->transition_id = transition_id_;
    response->message = "Teleoperation is disabled or target_arm is invalid";
    return;
  }
  if (
    ((target_arms & kLeftArm) != 0 &&
    request->left_preset_id == 0) ||
    ((target_arms & kRightArm) != 0 &&
    request->right_preset_id == 0))
  {
    std::lock_guard<std::mutex> lock(teleoperation_mutex_);
    response->accepted = false;
    response->transition_id = transition_id_;
    response->message = "Preset ID zero is invalid";
    return;
  }
  {
    std::lock_guard<std::mutex> lock(teleoperation_mutex_);
    if ((target_arms & initial_pose_busy_arms_) != 0) {
      response->accepted = false;
      response->transition_id = transition_id_;
      response->message =
        "Preset cannot be started while initial pose movement is in progress";
      return;
    }
    // Disable the selected arms in the same command that starts their preset motion.
    requested_arms_ &= static_cast<uint8_t>(~target_arms);
    if ((target_arms & kLeftArm) != 0) {
      left_preset_id_ = request->left_preset_id;
    }
    if ((target_arms & kRightArm) != 0) {
      right_preset_id_ = request->right_preset_id;
    }
    response->accepted = true;
    response->transition_id = ++transition_id_;
    response->message = "Preset motion accepted; selected arms teleoperation disabled";
  }
  publish_control_command(request->target_arm);
}

void A2JointTrajectoryCommandBroadcaster::control_status_callback(
  const robotis_interfaces::msg::ControlModeStatus::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(teleoperation_mutex_);
    if (msg->transition_id != transition_id_) {
      return;
    }
    requested_arms_ = arms_from_name(msg->requested_arms);
    active_arms_ = arms_from_name(msg->active_arms);
    initial_pose_available_arms_ = arms_from_name(msg->initial_pose_available_arms);
    initial_pose_busy_arms_ = 0;
    if (initial_pose_state_busy(msg->left_initial_pose_state)) {
      initial_pose_busy_arms_ |= kLeftArm;
    }
    if (initial_pose_state_busy(msg->right_initial_pose_state)) {
      initial_pose_busy_arms_ |= kRightArm;
    }
    preset_busy_arms_ = 0;
    if (preset_state_busy(msg->left_preset_state)) {
      preset_busy_arms_ |= kLeftArm;
    }
    if (preset_state_busy(msg->right_preset_state)) {
      preset_busy_arms_ |= kRightArm;
    }
  }
  if (msg->state == robotis_interfaces::msg::ControlModeStatus::STATE_ERROR) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Cyclo teleoperation transition failed: %s",
      msg->message.c_str());
  } else {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Cyclo teleoperation state=%u mode=%u arms=%s presets=(%u,%u): %s",
      static_cast<unsigned int>(msg->state),
      static_cast<unsigned int>(msg->active_control_mode),
      msg->active_arms.c_str(),
      static_cast<unsigned int>(msg->left_preset_id),
      static_cast<unsigned int>(msg->right_preset_id), msg->message.c_str());
  }
}

controller_interface::return_type A2JointTrajectoryCommandBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Update stored values
  for (const auto & state_interface : state_interfaces_) {
    std::string interface_name = state_interface.get_interface_name();
    if (map_interface_to_joint_state_.count(interface_name) > 0) {
      interface_name = map_interface_to_joint_state_[interface_name];
    }
    auto value = state_interface.get_optional();
    if (value) {
      name_if_value_mapping_[state_interface.get_prefix_name()][interface_name] = *value;
    }
  }

  double mean_error = 0.0;
  uint8_t requested_arms = 0;
  uint8_t newly_enabled_arms = 0;
  if (params_.enable_teleoperation) {
    update_initial_pose_trigger_state(time);
    requested_arms = requested_arms_rt_.load(std::memory_order_acquire);
    newly_enabled_arms = requested_arms &
      static_cast<uint8_t>(~previous_requested_arms_rt_);
    unsynced_arms_rt_ |= newly_enabled_arms;
    unsynced_arms_rt_ &= requested_arms;
    previous_requested_arms_rt_ = requested_arms;
  } else {
    // Keep the legacy gripper-trigger behavior for existing leader models.
    update_trigger_state(time);
    if (auto_mode_ == AutoMode::STOPPED) {
      return controller_interface::return_type::OK;
    }

    mean_error = calculate_mean_error();
    const bool current_synced = check_joints_synced();
    if (first_publish_) {
      joints_synced_ = false;
      first_publish_ = false;
      RCLCPP_INFO(
        get_node()->get_logger(),
        "First publish - using adaptive time_from_start based on error");
    } else {
      if (!joints_synced_ && current_synced) {
        joints_synced_ = true;
        RCLCPP_INFO(
          get_node()->get_logger(),
          "Joints synced for the first time - switching to immediate time_from_start permanently");
      }
    }
  }

  // Publish JointTrajectory messages for each group with current positions
  for (const auto & group_name : trajectory_groups_) {
    const auto & group_joints = group_joint_names_[group_name];
    // Safely get group offsets and reverse joints
    std::vector<double> group_offsets;
    std::vector<std::string> group_reverse_joints;

    auto offsets_it = group_joint_offsets_.find(group_name);
    if (offsets_it != group_joint_offsets_.end()) {
      group_offsets = offsets_it->second;
    }

    auto reverse_it = group_reverse_joints_.find(group_name);
    if (reverse_it != group_reverse_joints_.end()) {
      group_reverse_joints = reverse_it->second;
    }

    if (group_joints.empty()) {
      continue;  // Skip empty groups
    }

    auto & realtime_publisher = realtime_joint_trajectory_publishers_[group_name];
    if (realtime_publisher) {
      trajectory_msgs::msg::JointTrajectory traj_msg;
      traj_msg.header.stamp = rclcpp::Time(0, 0);
      traj_msg.joint_names = group_joints;

      const size_t num_joints = group_joints.size();
      traj_msg.points.clear();
      traj_msg.points.resize(1);
      traj_msg.points[0].positions.resize(num_joints, kUninitializedValue);

      for (size_t i = 0; i < num_joints; ++i) {
        double pos_value =
          get_value(name_if_value_mapping_, group_joints[i], HW_IF_POSITION);

        // Check if the current joint is in the reverse_joints parameter
        if (
          std::find(
            group_reverse_joints.begin(),
            group_reverse_joints.end(),
            group_joints[i]) != group_reverse_joints.end())
        {
          pos_value = -pos_value;
        }

        // Apply offset
        if (i < group_offsets.size()) {
          pos_value += group_offsets[i];
        }

        traj_msg.points[0].positions[i] = pos_value;
      }

      // Teleoperation uses a per-arm adaptive duration. Cyclo teleoperation consumes this
      // duration for its MoveJ slow start and still publishes immediate follower commands.
      if (params_.enable_teleoperation) {
        const uint8_t group_arm = arms_from_name(group_name);
        if (
          group_arm == 0 ||
          (requested_arms & group_arm) == 0 ||
          (unsynced_arms_rt_ & group_arm) == 0)
        {
          traj_msg.points[0].time_from_start = rclcpp::Duration(0, 0);
        } else {
          double group_error = calculate_group_mean_error(group_name);
          if (group_error <= params_.sync_threshold) {
            unsynced_arms_rt_ &= static_cast<uint8_t>(~group_arm);
            traj_msg.points[0].time_from_start = rclcpp::Duration(0, 0);
            RCLCPP_INFO(
              get_node()->get_logger(),
              "%s MoveJ slow start synchronized at mean error %.4f rad",
              group_name.c_str(), group_error);
          } else {
            if (group_error < params_.min_error) {
              group_error = params_.min_error;
            }
            const double error_ratio = std::min(group_error / params_.max_error, 1.0);
            const double adaptive_delay =
              params_.min_delay + (params_.max_delay - params_.min_delay) * error_ratio;
            traj_msg.points[0].time_from_start =
              rclcpp::Duration::from_seconds(adaptive_delay);
            if ((newly_enabled_arms & group_arm) != 0) {
              RCLCPP_INFO(
                get_node()->get_logger(),
                "%s MoveJ slow start began: mean error %.4f rad, time_from_start %.3f s",
                group_name.c_str(), group_error, adaptive_delay);
            }
          }
        }
      } else if (joints_synced_) {
        traj_msg.points[0].time_from_start = rclcpp::Duration(0, 0);  // immediate when synced
      } else {
        // Adaptive timing based on mean error using parameters
        if(mean_error < params_.min_error) {
          mean_error = 0;
        }

        double error_ratio = std::min(mean_error / params_.max_error, 1.0);
        // Corrected logic: small error -> small delay, large error -> large delay
        double adaptive_delay = params_.min_delay + (params_.max_delay - params_.min_delay) *
          error_ratio;


        // Convert to nanoseconds
        int32_t delay_ns = static_cast<int32_t>(adaptive_delay * 1e9);
        traj_msg.points[0].time_from_start = rclcpp::Duration(0, delay_ns);
      }

      realtime_publisher->try_publish(traj_msg);
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace a2_joint_trajectory_command_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  a2_joint_trajectory_command_broadcaster::A2JointTrajectoryCommandBroadcaster,
  controller_interface::ControllerInterface)
