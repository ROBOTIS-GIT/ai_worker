
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

#include "joint_trajectory_command_broadcaster/joint_trajectory_command_broadcaster.hpp"

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

namespace joint_trajectory_command_broadcaster
{
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

JointTrajectoryCommandBroadcaster::JointTrajectoryCommandBroadcaster() {}

controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_init()
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
JointTrajectoryCommandBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration JointTrajectoryCommandBroadcaster::
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

controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_configure(
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
      topic_name = "joint_trajectory_command_broadcaster_" + group_name + "/joint_trajectory";
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

      // Create joint state publisher for this group with timestamp from update() function
      std::string joint_state_topic_name;
      if (group_name == "left" && !params_.left_joint_states_stamped_topic.empty()) {
        joint_state_topic_name = params_.left_joint_states_stamped_topic;
      } else if (group_name == "right" && !params_.right_joint_states_stamped_topic.empty()) {
        joint_state_topic_name = params_.right_joint_states_stamped_topic;
      } else {
        joint_state_topic_name =
          "joint_trajectory_command_broadcaster_" + group_name + "/joint_states_stamped";
      }
      joint_state_stamped_publishers_[group_name] =
        get_node()->create_publisher<sensor_msgs::msg::JointState>(
        joint_state_topic_name, rclcpp::SystemDefaultsQoS());

      realtime_joint_state_stamped_publishers_[group_name] =
        std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
        joint_state_stamped_publishers_[group_name]);

      RCLCPP_INFO(
        get_node()->get_logger(),
        "Created joint state stamped publisher for group '%s' on topic: %s",
        group_name.c_str(), joint_state_topic_name.c_str());
    }

    // Store the groups for later use
    trajectory_groups_ = groups;

    // Create subscriber for follower joint states
    joint_states_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      params_.follower_joint_states_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&JointTrajectoryCommandBroadcaster::joint_states_callback, this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Subscribed to follower joint states topic: %s",
      params_.follower_joint_states_topic.c_str());
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

controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_activate(
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

  // Pre-allocate joint state message size for each group (real-time safety)
  // Must be done after init_joint_data() so group_joint_names_ is populated
  for (const auto & group_name : trajectory_groups_) {
    const auto & group_joints = group_joint_names_[group_name];
    if (group_joints.empty()) {
      continue;  // Skip empty groups
    }

    auto it = realtime_joint_state_stamped_publishers_.find(group_name);
    if (it != realtime_joint_state_stamped_publishers_.end() && it->second) {
      auto & msg = it->second->msg_;
      const size_t num_joints = group_joints.size();
      msg.name.resize(num_joints);
      msg.position.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
      msg.velocity.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
      msg.effort.resize(num_joints, std::numeric_limits<double>::quiet_NaN());

      // Set joint names once (done in on_activate, not in update loop)
      for (size_t i = 0; i < num_joints; ++i) {
        msg.name[i] = group_joints[i];
      }

      RCLCPP_INFO(
        get_node()->get_logger(),
        "Pre-allocated joint state message for group '%s' with %zu joints",
        group_name.c_str(), num_joints);
    }
  }

  return CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn JointTrajectoryCommandBroadcaster::on_deactivate(
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

bool JointTrajectoryCommandBroadcaster::init_joint_data()
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

void JointTrajectoryCommandBroadcaster::joint_states_callback(
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

double JointTrajectoryCommandBroadcaster::calculate_mean_error() const
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

bool JointTrajectoryCommandBroadcaster::check_trigger_active() const
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

void JointTrajectoryCommandBroadcaster::update_trigger_state(const rclcpp::Time & current_time)
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

bool JointTrajectoryCommandBroadcaster::check_joints_synced() const
{
  double mean_error = calculate_mean_error();
  return mean_error <= params_.sync_threshold;
}

controller_interface::return_type JointTrajectoryCommandBroadcaster::update(
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

  // Update trigger state for auto mode control
  update_trigger_state(time);

  // Skip publishing if auto mode is STOPPED
  if (auto_mode_ == AutoMode::STOPPED) {
    return controller_interface::return_type::OK;
  }

  // Calculate mean error and check if joints are synced
  double mean_error = calculate_mean_error();
  bool current_synced = check_joints_synced();

  // Update sync status and handle first publish
  if (first_publish_) {
    joints_synced_ = false;
    first_publish_ = false;
    RCLCPP_INFO(get_node()->get_logger(),
        "First publish - using adaptive time_from_start based on error");
  } else {
    // Once synced, stay synced permanently
    if (!joints_synced_ && current_synced) {
      joints_synced_ = true;
      RCLCPP_INFO(get_node()->get_logger(),
          "Joints synced for the first time - switching to immediate time_from_start permanently");
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

      // Set time_from_start based on sync status and mean error
      if (joints_synced_) {
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

    // Publish joint state for this group with timestamp from update() function
    // Real-time safe: message size pre-allocated in on_activate(), only copy values here
    // Apply same transformations (reverse, offsets) as joint_trajectory
    auto joint_state_pub_it = realtime_joint_state_stamped_publishers_.find(group_name);
    if (joint_state_pub_it != realtime_joint_state_stamped_publishers_.end() &&
      joint_state_pub_it->second)
    {
      auto & realtime_joint_state_publisher = joint_state_pub_it->second;
      if (realtime_joint_state_publisher->trylock()) {
        auto & msg = realtime_joint_state_publisher->msg_;

        // Set timestamp from update() function argument (actual sensor read time)
        msg.header.stamp = time;
        msg.header.frame_id = "";

        // Copy values from name_if_value_mapping_ to pre-allocated message
        // Apply same reverse and offset transformations as joint_trajectory
        // No string operations, no dynamic allocation - only value copying
        for (size_t i = 0; i < group_joints.size() && i < msg.name.size(); ++i) {
          const std::string & joint_name = group_joints[i];

          // Get position and apply reverse/offset (same as joint_trajectory)
          double pos_value = get_value(name_if_value_mapping_, joint_name, HW_IF_POSITION);

          // Check if the current joint is in the reverse_joints parameter
          if (
            std::find(
              group_reverse_joints.begin(),
              group_reverse_joints.end(),
              joint_name) != group_reverse_joints.end())
          {
            pos_value = -pos_value;
          }

          // Apply group offset
          if (i < group_offsets.size()) {
            pos_value += group_offsets[i];
          }

          msg.position[i] = std::isnan(pos_value) ? std::numeric_limits<double>::quiet_NaN() :
            pos_value;

          // Get velocity if available (no reverse/offset for velocity)
          if (name_if_value_mapping_.count(joint_name) > 0) {
            const auto & interfaces = name_if_value_mapping_.at(joint_name);
            auto vel_it = interfaces.find(HW_IF_VELOCITY);
            if (vel_it != interfaces.end() && !std::isnan(vel_it->second)) {
              // Apply reverse sign to velocity if joint is reversed
              double vel_value = vel_it->second;
              if (
                std::find(
                  group_reverse_joints.begin(),
                  group_reverse_joints.end(),
                  joint_name) != group_reverse_joints.end())
              {
                vel_value = -vel_value;
              }
              msg.velocity[i] = vel_value;
            } else {
              msg.velocity[i] = std::numeric_limits<double>::quiet_NaN();
            }

            // Get effort if available (no reverse/offset for effort)
            auto eff_it = interfaces.find(HW_IF_EFFORT);
            if (eff_it != interfaces.end() && !std::isnan(eff_it->second)) {
              msg.effort[i] = eff_it->second;
            } else {
              msg.effort[i] = std::numeric_limits<double>::quiet_NaN();
            }
          } else {
            msg.velocity[i] = std::numeric_limits<double>::quiet_NaN();
            msg.effort[i] = std::numeric_limits<double>::quiet_NaN();
          }
        }

        realtime_joint_state_publisher->unlockAndPublish();
      }
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace joint_trajectory_command_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_trajectory_command_broadcaster::JointTrajectoryCommandBroadcaster,
  controller_interface::ControllerInterface)
