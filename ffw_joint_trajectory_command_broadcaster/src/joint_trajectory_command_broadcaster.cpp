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
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iterator>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
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

    // Declare follower parameters supplied by leader_initializer.
    for (const auto & [name, value] :
      get_node()->get_node_parameters_interface()->get_parameter_overrides())
    {
      if (name.rfind("follower_current_position.", 0) == 0 ||
        name.rfind("follower_joint_limits.", 0) == 0)
      {
        auto_declare<double>(name, value.get<double>());
      } else if (name.rfind("follower_end_tool.", 0) == 0) {
        auto_declare<std::string>(name, value.get<std::string>());
      }
    }
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
      const auto & group_joints =
        group_name == "left" ? params_.left_joints : params_.right_joints;
      if (group_joints.empty()) {
        continue;  // Skip empty groups
      }

      group_joint_names_[group_name] = group_joints;
      group_joint_offsets_[group_name] =
        group_name == "left" ? params_.left_offsets : params_.right_offsets;
      // Reverse joints are currently unused in leader controller configs (default: []).
      group_reverse_joints_[group_name] =
        group_name == "left" ? params_.left_reverse_joints : params_.right_reverse_joints;

      const auto & topic_name = group_name == "left" ?
        params_.dynamixel_l_joint_trajectory_topic : params_.dynamixel_r_joint_trajectory_topic;
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

    // Read follower positions, limits, and end tools.
    std::map<std::string, double> follower_current_position;
    if (!get_node()->get_parameters("follower_current_position", follower_current_position)) {
      throw std::runtime_error("Missing follower_current_position parameter");
    }
    std::map<std::string, double> follower_joint_limits;
    get_node()->get_parameters("follower_joint_limits", follower_joint_limits);
    std::map<std::string, std::string> follower_end_tool;
    get_node()->get_parameters("follower_end_tool", follower_end_tool);
    for (const auto & group_name : trajectory_groups_) {
      const auto & joints = group_joint_names_[group_name];
      std::vector<double> positions, lowers, uppers;
      for (const auto & joint : joints) {
        if ((joint == "gripper_l_joint1" || joint == "gripper_r_joint1") &&
          follower_end_tool[group_name] != "gripper")
        {
          // Use defaults when the follower end tool is not a gripper.
          positions.push_back(0.0);
          lowers.push_back(0.0);
          uppers.push_back(1.1);
          continue;
        }
        const auto it = follower_current_position.find(joint);
        if (it != follower_current_position.end()) {
          if (!std::isfinite(it->second)) {
            throw std::runtime_error("Follower position must be finite: " + joint);
          }
          positions.push_back(it->second);
        } else if (joint == "gripper_l_joint1" || joint == "gripper_r_joint1") {
          // Use zero if the follower has no gripper joint state.
          positions.push_back(0.0);
        } else {
          throw std::runtime_error("Missing follower position: " + joint);
        }

        const auto lower_it = follower_joint_limits.find(joint + ".lower");
        const auto upper_it = follower_joint_limits.find(joint + ".upper");
        // Missing bounds are unlimited.
        const double lower = lower_it != follower_joint_limits.end() ? lower_it->second :
          -std::numeric_limits<double>::infinity();
        const double upper = upper_it != follower_joint_limits.end() ? upper_it->second :
          std::numeric_limits<double>::infinity();
        if ((lower_it != follower_joint_limits.end() && !std::isfinite(lower)) ||
          (upper_it != follower_joint_limits.end() && !std::isfinite(upper)) || lower > upper)
        {
          throw std::runtime_error("Invalid follower joint limits: " + joint);
        }
        lowers.push_back(lower);
        uppers.push_back(upper);
      }
      group_last_target_[group_name] = positions;
      group_lower_limits_[group_name] = lowers;
      group_upper_limits_[group_name] = uppers;
      for (auto & [pose_id, pose] : group_save_poses_[group_name]) {
        if (pose.size() != joints.size()) {
          throw std::runtime_error(
            group_name + ": save pose size mismatch: " + std::to_string(pose_id));
        }

        for (size_t i = 0; i < pose.size(); ++i) {
          if (!std::isfinite(pose[i])) {
            throw std::runtime_error(
              group_name + ": invalid save pose value: " + joints[i]);
          }

          pose[i] = std::clamp(pose[i], lowers[i], uppers[i]);
        }
      }
      RCLCPP_INFO(get_node()->get_logger(),
        "[%s] Loaded follower positions and joint limits",
        group_name.c_str());
    }

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

          if (!std::isfinite(leader_val)) {
            RCLCPP_FATAL(
              get_node()->get_logger(),
              "[%s] Non-finite leader target for joint '%s': %g; exiting leader node",
              group_name.c_str(), group_joints[i].c_str(), leader_val);
            std::exit(EXIT_FAILURE);
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

    const bool valid = !last_target.empty() && last_target.size() == num_joints;
    if (valid) {
      for (size_t i = 0; i < num_joints; ++i) {
        if (!std::isfinite(last_target[i])) {
          RCLCPP_FATAL(
            get_node()->get_logger(),
            "[%s] Non-finite target for joint '%s': %g; exiting leader node",
            group_name.c_str(), group_joints[i].c_str(), last_target[i]);
          std::exit(EXIT_FAILURE);
        }
      }
    }

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
