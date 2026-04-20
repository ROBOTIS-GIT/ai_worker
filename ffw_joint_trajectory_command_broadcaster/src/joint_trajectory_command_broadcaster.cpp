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

    // Load initial enable state from parameters (bool → state: true=1, false=0)
    left_state_ = params_.left_enabled_init ? 1u : 0u;
    right_state_ = params_.right_enabled_init ? 1u : 0u;
    RCLCPP_INFO(get_node()->get_logger(),
      "Initial enable state: left=%u, right=%u", left_state_, right_state_);

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
      "/leader/left_enable", rclcpp::SystemDefaultsQoS(),
      [this](std_msgs::msg::UInt8::SharedPtr msg) {
        handle_enable_msg("left", msg->data, left_state_);
      });

    right_enable_sub_ = get_node()->create_subscription<std_msgs::msg::UInt8>(
      "/leader/right_enable", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        handle_enable_msg("right", msg->data, right_state_);
      });

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
  group_last_target_.clear();
  group_interp_state_.clear();
  group_leader_blend_.clear();

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
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Wait until every group's last_target has been initialized from follower
  bool all_initialized = true;
  for (const auto & group_name : trajectory_groups_) {
    if (!group_last_target_initialized_[group_name]) {
      all_initialized = false;
      break;
    }
  }
  if (!all_initialized) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Waiting for follower joint_states to initialize last_target...");
    return controller_interface::return_type::OK;
  }

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

    // Per-group state (0=disabled, 1=leader tracking, 3+=save pose)
    uint8_t group_state = 0;
    if (group_name == "left") {
      group_state = left_state_;
    } else if (group_name == "right") {
      group_state = right_state_;
    }
    const bool leader_tracking = (group_state == 1);

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

    const size_t num_joints = group_joints.size();
    auto & last_target = group_last_target_[group_name];

    // Compute interpolation progress (only relevant if not leader tracking)
    auto & interp = group_interp_state_[group_name];
    bool use_interp = !leader_tracking && interp.active;
    double interp_s = 0.0;
    if (use_interp) {
      double elapsed = (get_node()->now() - interp.start_time).seconds();
      double t = std::clamp(elapsed / interp.duration_sec, 0.0, 1.0);
      interp_s = 3.0 * t * t - 2.0 * t * t * t;  // cubic smoothstep
      if (t >= 1.0) {
        interp.active = false;
      }
    }

    // Compute leader-blend factor (only when leader_tracking)
    auto & blend = group_leader_blend_[group_name];
    double blend_alpha = 1.0;  // 1.0 = full leader tracking
    if (leader_tracking && blend.active) {
      double elapsed = (get_node()->now() - blend.start_time).seconds();
      double t = std::clamp(elapsed / params_.leader_blend_duration, 0.0, 1.0);
      blend_alpha = 3.0 * t * t - 2.0 * t * t * t;  // cubic smoothstep
      if (t >= 1.0) {
        blend.active = false;
      }
    }

    // Update last_target:
    //   leader_tracking (state==1) → leader value (blended during first N seconds)
    //   interp active                → interpolated value
    //   else                         → hold previous last_target
    if (leader_tracking || use_interp) {
      last_target.resize(num_joints);
      for (size_t i = 0; i < num_joints; ++i) {
        double pos_value;
        if (use_interp && i < interp.start_pos.size() && i < interp.target_pos.size()) {
          pos_value = interp.start_pos[i] +
            interp_s * (interp.target_pos[i] - interp.start_pos[i]);
        } else {
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

          if (leader_tracking && blend.active && i < blend.start_pos.size()) {
            pos_value = (1.0 - blend_alpha) * blend.start_pos[i] +
              blend_alpha * leader_val;
          } else {
            pos_value = leader_val;
          }
        }
        last_target[i] = pos_value;
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
          double pos_value = (i < last_target.size()) ? last_target[i] :
            std::numeric_limits<double>::quiet_NaN();

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

void JointTrajectoryCommandBroadcaster::handle_enable_msg(
  const std::string & group_name, uint8_t data, uint8_t & state)
{
  // Ignore all enable messages until last_target is initialized from follower
  auto init_it = group_last_target_initialized_.find(group_name);
  if (init_it == group_last_target_initialized_.end() || !init_it->second) {
    RCLCPP_WARN(get_node()->get_logger(),
      "[%s] enable ignored: waiting for follower init", group_name.c_str());
    return;
  }

  if (data == 2) {
    // toggle between 1 and 0 only
    uint8_t new_state = (state == 1) ? 0u : 1u;
    if (new_state == 1 && state != 1) {
      start_leader_blend(group_name);
    }
    state = new_state;
    return;
  }

  if (data == 0 || data == 1) {
    if (data == 1 && state != 1) {
      start_leader_blend(group_name);
    }
    state = data;
    return;
  }

  // 3+ : save pose N
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
  state = data;
  start_interpolation(group_name, pose_it->second);
  RCLCPP_INFO(get_node()->get_logger(),
    "[%s] Start interpolation to save pose %u", group_name.c_str(), data);
}

void JointTrajectoryCommandBroadcaster::start_leader_blend(
  const std::string & group_name)
{
  auto last_it = group_last_target_.find(group_name);
  if (last_it == group_last_target_.end() || last_it->second.empty()) {
    return;
  }
  auto & bs = group_leader_blend_[group_name];
  bs.start_pos = last_it->second;   // snapshot
  bs.start_time = get_node()->now();
  bs.active = true;
  RCLCPP_INFO(get_node()->get_logger(),
    "[%s] Start leader blend (%.1fs)",
    group_name.c_str(), params_.leader_blend_duration);
}

void JointTrajectoryCommandBroadcaster::start_interpolation(
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
  auto & st = group_interp_state_[group_name];
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
