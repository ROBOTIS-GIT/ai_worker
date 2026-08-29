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

#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <thread>
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
#include "std_msgs/msg/string.hpp"
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
using hardware_interface::HW_IF_POSITION;

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
    }

    // Store the groups for later use
    trajectory_groups_ = groups;

    // Initialize group runtimes with mode from parameters
    for (const auto & group_name : trajectory_groups_) {
      bool init_enabled = (group_name == "left") ?
        params_.left_enabled_init : params_.right_enabled_init;
      group_runtime_[group_name].mode = init_enabled ? Mode::TELEOP : Mode::IDLE;
    }
    RCLCPP_INFO(get_node()->get_logger(),
      "Initial mode: left=%s, right=%s",
      params_.left_enabled_init ? "TELEOP" : "IDLE",
      params_.right_enabled_init ? "TELEOP" : "IDLE");

    // Load save poses per group from parameters (left_save_pose_<id> / right_save_pose_<id>)
    for (int64_t id : params_.save_pose_ids) {
      for (const auto & group_name : trajectory_groups_) {
        std::string pname = group_name + "_save_pose_" + std::to_string(id);
        if (!get_node()->has_parameter(pname)) {
          get_node()->declare_parameter(pname, std::vector<double>{});
        }
        auto values = get_node()->get_parameter(pname).as_double_array();
        if (!values.empty()) {
          group_save_poses_[group_name][static_cast<uint8_t>(id)] = values;
          RCLCPP_INFO(get_node()->get_logger(),
            "Loaded %s (%zu values)", pname.c_str(), values.size());
        }
      }
    }

    // One-shot follower subscriptions: init last_target once, then ignore
    for (const auto & group_name : trajectory_groups_) {
      group_last_target_initialized_[group_name] = false;
    }

    auto make_follower_cb = [this](const std::string & group_name) {
      return [this, group_name](sensor_msgs::msg::JointState::SharedPtr msg) {
        if (group_last_target_initialized_[group_name]) {
          return;
        }
        const auto & joints = group_joint_names_[group_name];
        if (joints.empty()) {
          return;
        }
        std::vector<double> positions(joints.size(), kUninitializedValue);
        const size_t n = std::min(msg->name.size(), msg->position.size());
        for (size_t i = 0; i < joints.size(); ++i) {
          for (size_t j = 0; j < n; ++j) {
            if (msg->name[j] == joints[i]) {
              positions[i] = msg->position[j];
              break;
            }
          }
        }
        group_last_target_[group_name] = positions;
        group_last_target_initialized_[group_name] = true;
        RCLCPP_INFO(get_node()->get_logger(),
          "[%s] last_target initialized from follower joint_states",
          group_name.c_str());
      };
    };

    left_follower_js_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      params_.left_follower_joint_states_topic, rclcpp::SystemDefaultsQoS(),
      make_follower_cb("left"));
    right_follower_js_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      params_.right_follower_joint_states_topic, rclcpp::SystemDefaultsQoS(),
      make_follower_cb("right"));

    // Enable topic subscriptions
    //   0=disable, 1=enable, 2=toggle, 3+=disable + trigger save pose <N>
    left_enable_sub_ = get_node()->create_subscription<std_msgs::msg::UInt8>(
      "/leader/left_command", rclcpp::SystemDefaultsQoS(),
      [this](std_msgs::msg::UInt8::SharedPtr msg) {
        handle_enable_msg("left", msg->data);
      });

    right_enable_sub_ = get_node()->create_subscription<std_msgs::msg::UInt8>(
      "/leader/right_command", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        handle_enable_msg("right", msg->data);
      });

    // Block until follower joint_states init last_target for every group
    while (rclcpp::ok()) {
      bool all_init = true;
      for (const auto & g : trajectory_groups_) {
        if (!group_last_target_initialized_[g]) {
          all_init = false;
          break;
        }
      }
      if (all_init) {
        break;
      }
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Waiting for follower joint_states to initialize last_target...");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(get_node()->get_logger(), "Controller configured successfully.");
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

  // Subscribe to follower's robot_description once to extract joint limits for TELEOP clamping
  bool follower_urdf_loaded = false;
  auto urdf_sub = get_node()->create_subscription<std_msgs::msg::String>(
    params_.follower_robot_description_topic,
    rclcpp::QoS(1).transient_local().reliable(),
    [this, &follower_urdf_loaded](std_msgs::msg::String::SharedPtr msg) {
      if (follower_urdf_loaded) {
        return;
      }
      urdf::Model follower_model;
      if (!follower_model.initString(msg->data)) {
        RCLCPP_ERROR(get_node()->get_logger(),
          "Failed to parse follower robot_description");
        return;
      }
      for (const auto & group_name : trajectory_groups_) {
        const auto & joints = group_joint_names_[group_name];
        std::vector<double> lowers, uppers;
        for (const auto & jn : joints) {
          auto j = follower_model.getJoint(jn);
          if (j && j->limits) {
            lowers.push_back(j->limits->lower);
            uppers.push_back(j->limits->upper);
          } else {
            lowers.push_back(-std::numeric_limits<double>::infinity());
            uppers.push_back(std::numeric_limits<double>::infinity());
            RCLCPP_WARN(get_node()->get_logger(),
              "[%s] No limit for joint '%s' in follower URDF",
              group_name.c_str(), jn.c_str());
          }
        }
        group_lower_limits_[group_name] = lowers;
        group_upper_limits_[group_name] = uppers;
        RCLCPP_INFO(get_node()->get_logger(),
          "[%s] Loaded follower joint limits (%zu joints)",
          group_name.c_str(), lowers.size());
      }
      follower_urdf_loaded = true;
    });

  RCLCPP_INFO(get_node()->get_logger(),
    "Waiting for follower robot_description on '%s'...",
    params_.follower_robot_description_topic.c_str());
  while (rclcpp::ok() && !follower_urdf_loaded) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Still waiting for follower robot_description...");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!rclcpp::ok()) {
    return CallbackReturn::ERROR;
  }
  // urdf_sub goes out of scope here, effectively unsubscribing

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
  group_lower_limits_.clear();
  group_upper_limits_.clear();
  group_last_target_.clear();
  group_runtime_.clear();

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

controller_interface::return_type JointTrajectoryCommandBroadcaster::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
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

    const size_t num_joints = group_joints.size();
    auto & last_target = group_last_target_[group_name];
    auto & rt = group_runtime_[group_name];

    // Update last_target based on mode
    switch (rt.mode) {
      case Mode::IDLE: {
        // No update: hold last_target
        break;
      }

      case Mode::TELEOP: {
        // Leader tracking, with optional blend on entry
        double blend_alpha = 1.0;
        if (rt.blend.active) {
          double elapsed = (get_node()->now() - rt.blend.start_time).seconds();
          double t = std::clamp(elapsed / params_.leader_blend_duration, 0.0, 1.0);
          blend_alpha = 3.0 * t * t - 2.0 * t * t * t;  // cubic smoothstep
          if (t >= 1.0) {
            rt.blend.active = false;
          }
        }

        const auto & lowers = group_lower_limits_[group_name];
        const auto & uppers = group_upper_limits_[group_name];
        const bool clamp_enabled =
          lowers.size() == num_joints && uppers.size() == num_joints;

        last_target.resize(num_joints);
        for (size_t i = 0; i < num_joints; ++i) {
          double leader_val =
            get_value(name_if_value_mapping_, group_joints[i], HW_IF_POSITION);
          if (std::find(group_reverse_joints.begin(), group_reverse_joints.end(),
              group_joints[i]) != group_reverse_joints.end())
          {
            leader_val = -leader_val;
          }
          if (i < group_offsets.size()) {
            leader_val += group_offsets[i];
          }

          if (rt.blend.active && i < rt.blend.start_pos.size()) {
            leader_val = (1.0 - blend_alpha) * rt.blend.start_pos[i] +
              blend_alpha * leader_val;
          }

          // Clamp to follower joint limits
          if (clamp_enabled) {
            leader_val = std::clamp(leader_val, lowers[i], uppers[i]);
          }
          last_target[i] = leader_val;
        }
        break;
      }

      case Mode::SAVE_POSE: {
        // Cubic interp to target; hold at target after completion
        if (rt.interp.active) {
          double elapsed = (get_node()->now() - rt.interp.start_time).seconds();
          double t = std::clamp(elapsed / rt.interp.duration_sec, 0.0, 1.0);
          double s = 3.0 * t * t - 2.0 * t * t * t;
          if (t >= 1.0) {
            rt.interp.active = false;
          }

          last_target.resize(num_joints);
          for (size_t i = 0; i < num_joints; ++i) {
            if (i < rt.interp.start_pos.size() && i < rt.interp.target_pos.size()) {
              last_target[i] = rt.interp.start_pos[i] +
                s * (rt.interp.target_pos[i] - rt.interp.start_pos[i]);
            }
          }
        }
        // else: hold last_target (which should equal target after completion)
        break;
      }
    }

    // last_target is valid only after first enable/interp
    bool valid = last_target.size() == num_joints &&
                 !std::isnan(last_target[0]);

    // Publish trajectory (always when last_target valid)
    auto & realtime_publisher = realtime_joint_trajectory_publishers_[group_name];
    if (valid && realtime_publisher) {
      trajectory_msgs::msg::JointTrajectory traj_msg;
      traj_msg.header.stamp = rclcpp::Time(0, 0);
      traj_msg.joint_names = group_joints;
      traj_msg.points.resize(1);
      traj_msg.points[0].positions = last_target;
      traj_msg.points[0].time_from_start = rclcpp::Duration(0, 0);
      realtime_publisher->try_publish(traj_msg);
    }
  }

  return controller_interface::return_type::OK;
}

void JointTrajectoryCommandBroadcaster::handle_enable_msg(
  const std::string & group_name, uint8_t data)
{
  auto & rt = group_runtime_[group_name];

  switch (data) {
    case 0: // stop
      rt.mode = Mode::IDLE;
      return;

    case 1: // teleop
      if (rt.mode != Mode::TELEOP) {
        start_teleop_blend(group_name);
      }
      rt.mode = Mode::TELEOP;
      return;

    case 2:  // toggle
      if (rt.mode == Mode::TELEOP) {
        rt.mode = Mode::IDLE;
      } else {
        start_teleop_blend(group_name);
        rt.mode = Mode::TELEOP;
      }
      return;

    default: {  // 3, 4, ... : save pose N
      auto gp_it = group_save_poses_.find(group_name);
      if (gp_it == group_save_poses_.end()) {
        return;
      }
      auto pose_it = gp_it->second.find(data);
      if (pose_it == gp_it->second.end()) {
        RCLCPP_WARN(get_node()->get_logger(),
          "[%s] No save pose defined for id=%u", group_name.c_str(), data);
        return;
      }
      start_save_pose_interp(group_name, pose_it->second);
      rt.mode = Mode::SAVE_POSE;
      RCLCPP_INFO(get_node()->get_logger(),
        "[%s] Start interpolation to save pose %u", group_name.c_str(), data);
      return;
    }
  }
}

void JointTrajectoryCommandBroadcaster::start_teleop_blend(
  const std::string & group_name)
{
  auto last_it = group_last_target_.find(group_name);
  if (last_it == group_last_target_.end() || last_it->second.empty()) {
    return;
  }
  auto & bs = group_runtime_[group_name].blend;
  bs.start_pos = last_it->second;   // snapshot
  bs.start_time = get_node()->now();
  bs.active = true;
  RCLCPP_INFO(get_node()->get_logger(),
    "[%s] Start teleop blend (%.1fs)",
    group_name.c_str(), params_.leader_blend_duration);
}

void JointTrajectoryCommandBroadcaster::start_save_pose_interp(
  const std::string & group_name, const std::vector<double> & target)
{
  auto last_it = group_last_target_.find(group_name);
  if (last_it == group_last_target_.end() || last_it->second.empty()) {
    RCLCPP_WARN(get_node()->get_logger(),
      "[%s] last_target not initialized; skip interpolation", group_name.c_str());
    return;
  }
  if (target.size() != last_it->second.size()) {
    RCLCPP_WARN(get_node()->get_logger(),
      "[%s] save pose size (%zu) mismatch joints (%zu)",
      group_name.c_str(), target.size(), last_it->second.size());
    return;
  }
  auto & st = group_runtime_[group_name].interp;
  st.start_time = get_node()->now();
  st.start_pos = last_it->second;
  st.target_pos = target;
  st.duration_sec = params_.save_pose_duration;
  st.active = true;
}

}  // namespace joint_trajectory_command_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_trajectory_command_broadcaster::JointTrajectoryCommandBroadcaster,
  controller_interface::ControllerInterface)
