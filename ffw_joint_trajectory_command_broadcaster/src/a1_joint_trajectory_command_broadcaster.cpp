// Copyright 2026 ROBOTIS CO., LTD.
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

#include "a1_joint_trajectory_command_broadcaster/a1_joint_trajectory_command_broadcaster.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iterator>
#include <limits>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace a1_joint_trajectory_command_broadcaster
{
namespace
{
using Base = leader_joint_trajectory_command_broadcaster::LeaderJointTrajectoryCommandBroadcaster;

// The base reads the cached views only during this call. Restore the real loans on every exit.
class ScopedInputView
{
public:
  ScopedInputView(
    std::vector<hardware_interface::LoanedStateInterface> & original,
    std::vector<hardware_interface::LoanedStateInterface> & cached)
  : original_(original), cached_(cached)
  {
    original_.swap(cached_);
  }

  ~ScopedInputView() {original_.swap(cached_);}

  ScopedInputView(const ScopedInputView &) = delete;
  ScopedInputView & operator=(const ScopedInputView &) = delete;

private:
  std::vector<hardware_interface::LoanedStateInterface> & original_;
  std::vector<hardware_interface::LoanedStateInterface> & cached_;
};

struct ScalingRange
{
  double direction;
  double lower;
  double upper;
};

// Existing corrected ROS zero is the pivot for every joint. Bounds match the SG2 follower URDF.
constexpr ScalingRange kJoint6Range{-1.0, -1.57, 1.57};       // Both arms: upward is negative.
constexpr ScalingRange kLeftJoint7Range{1.0, -1.5804, 1.8201};   // Left: outward is positive.
constexpr ScalingRange kRightJoint7Range{-1.0, -1.8201, 1.5804};  // Right: outward is negative.

double scale_position(
  double position, double gain, const ScalingRange & range)
{
  if (gain == 1.0) {
    return position;
  }
  if (position * range.direction > 0.0) {
    position *= gain;
  }
  return std::clamp(position, range.lower, range.upper);
}
}  // namespace

controller_interface::CallbackReturn A1JointTrajectoryCommandBroadcaster::on_init()
{
  const auto result = Base::on_init();
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result;
  }
  try {
    scaling_param_listener_ = std::make_shared<ParamListener>(get_node());
    scaling_params_ = scaling_param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "A1 scaling initialization failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn A1JointTrajectoryCommandBroadcaster::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  if (!scaling_param_listener_) {
    return controller_interface::CallbackReturn::ERROR;
  }
  scaling_params_ = scaling_param_listener_->get_params();
  const auto & joint6 = scaling_params_.joint6_scaling;
  const auto & joint7 = scaling_params_.joint7_scaling;
  if (!std::isfinite(joint6.gain) || joint6.gain <= 0.0 ||
    !std::isfinite(joint7.gain) || joint7.gain <= 0.0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "A1 scaling gains must be finite and positive; use 1.0 to disable scaling");
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto result = Base::on_configure(previous_state);
  if (result == controller_interface::CallbackReturn::SUCCESS) {
    if (!configure_physical_pose_reference()) {
      return controller_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
      get_node()->get_logger(),
      "A1 MoveJ swaps joint1/2: joint6 %s (gain %.3f), joint7 %s (gain %.3f); "
      "RelativePose uses physical joint states",
      joint6.gain != 1.0 ? "enabled" : "disabled", joint6.gain,
      joint7.gain != 1.0 ? "enabled" : "disabled", joint7.gain);
  }
  return result;
}

bool A1JointTrajectoryCommandBroadcaster::configure_physical_pose_reference()
{
  physical_state_subscription_.reset();
  if (!params_.enable_teleoperation || !params_.leader_pose_reference_enabled) {
    return true;
  }
  if (scaling_params_.leader_joint_states_topic.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "A1 physical JointState topic must not be empty");
    return false;
  }
  using Trajectory = trajectory_msgs::msg::JointTrajectory;
  struct PhysicalGroup
  {
    rclcpp::Publisher<Trajectory>::SharedPtr publisher;
    std::vector<std::string> names;
    std::vector<double> offsets;
    std::vector<std::string> reverse;
  };
  std::vector<PhysicalGroup> groups;
  decltype(joint_trajectory_publishers_) physical_publishers;
  try {
    for (const auto & name : {"left", "right"}) {
      auto publisher = get_node()->create_publisher<Trajectory>(
        std::string("~/physical_") + name + "_joint_trajectory", rclcpp::QoS(1).reliable());
      physical_publishers.emplace(name, publisher);
      groups.push_back({publisher, group_joint_names_.at(name),
          group_joint_offsets_.at(name), group_reverse_joints_.at(name)});
    }
    // The existing protected hook selects its input topics from these publishers.
    // Reconfigure only this A1 instance; restore the MoveJ publishers before activation.
    // The realtime publishers remain untouched, so commands cannot enter the FK topics.
    joint_trajectory_publishers_.swap(physical_publishers);
    const bool configured = configure_pose_reference();
    joint_trajectory_publishers_.swap(physical_publishers);
    if (!configured) {return false;}
    physical_state_subscription_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      scaling_params_.leader_joint_states_topic, rclcpp::SensorDataQoS().keep_last(1),
      [groups = std::move(groups)](const sensor_msgs::msg::JointState::SharedPtr state) {
        for (const auto & group : groups) {
          Trajectory sample;
          sample.header = state->header;
          sample.joint_names = group.names;
          sample.points.resize(1);
          auto & positions = sample.points.front().positions;
          positions.reserve(group.names.size());
          for (size_t i = 0; i < group.names.size(); ++i) {
            const auto & name = group.names[i];
            const auto found = std::find(state->name.begin(), state->name.end(), name);
            const auto index = static_cast<size_t>(std::distance(state->name.begin(), found));
            double value = found != state->name.end() && index < state->position.size() ?
            state->position[index] : std::numeric_limits<double>::quiet_NaN();
            // Arm FK receives the original DHI positions by name in every command mode.
            // The shared bridge also forwards the gripper, which retains its command offset.
            if (name == "gripper_l_joint1" || name == "gripper_r_joint1") {
              const double sign = std::find(group.reverse.begin(), group.reverse.end(), name) ==
              group.reverse.end() ? 1.0 : -1.0;
              value = value * sign + group.offsets[i];
            }
            positions.push_back(value);
          }
          group.publisher->publish(sample);
        }
      });
    return true;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "A1 physical FK input configuration failed: %s",
      error.what());
    return false;
  }
}

controller_interface::CallbackReturn A1JointTrajectoryCommandBroadcaster::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  const auto result = Base::on_activate(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result;
  }
  cached_state_interfaces_.clear();
  cached_state_handles_.clear();
  input_conversions_.clear();
  cached_state_interfaces_.reserve(state_interfaces_.size());
  cached_state_handles_.reserve(state_interfaces_.size());
  input_conversions_.reserve(state_interfaces_.size());
  for (const auto & source : state_interfaces_) {
    InputConversion conversion;
    conversion.joint_name = source.get_prefix_name();
    conversion.joint_reference_source_index = input_conversions_.size();
    conversion.last_raw_position = std::numeric_limits<double>::quiet_NaN();
    const auto mapping = map_interface_to_joint_state_.find(source.get_interface_name());
    const auto & mapped_name = mapping == map_interface_to_joint_state_.end() ?
      source.get_interface_name() : mapping->second;
    conversion.is_position = mapped_name == "position";
    if (conversion.is_position) {
      for (const auto & group : group_joint_names_) {
        const auto joint = std::find(group.second.begin(), group.second.end(),
            conversion.joint_name);
        if (joint == group.second.end()) {
          continue;
        }
        const auto index = static_cast<size_t>(std::distance(group.second.begin(), joint));
        const auto & reverse = group_reverse_joints_.at(group.first);
        conversion.sign = std::find(reverse.begin(), reverse.end(), conversion.joint_name) ==
          reverse.end() ? 1.0 : -1.0;
        conversion.offset = group_joint_offsets_.at(group.first).at(index);
        break;
      }
    }
    hardware_interface::InterfaceInfo info;
    info.name = source.get_interface_name();
    info.data_type = "double";
    auto handle = std::make_shared<hardware_interface::StateInterface>(
      hardware_interface::InterfaceDescription(source.get_prefix_name(), info));
    cached_state_interfaces_.emplace_back(handle);
    cached_state_handles_.push_back(std::move(handle));
    input_conversions_.push_back(std::move(conversion));
  }
  // Hardware and JointState names follow the physical A1 chain. Only joint-mode
  // follower references exchange the first two joints, independently of loan order.
  for (auto & conversion : input_conversions_) {
    if (!conversion.is_position) {
      continue;
    }
    std::string source_name = conversion.joint_name;
    if (source_name == "arm_l_joint1" || source_name == "arm_r_joint1") {
      source_name.back() = '2';
    } else if (source_name == "arm_l_joint2" || source_name == "arm_r_joint2") {
      source_name.back() = '1';
    } else {
      continue;
    }
    const auto source = std::find_if(
      input_conversions_.begin(), input_conversions_.end(),
      [&source_name](const InputConversion & candidate) {
        return candidate.is_position && candidate.joint_name == source_name;
      });
    if (source == input_conversions_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "A1 joint mapping requires %s", source_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    conversion.joint_reference_source_index =
      static_cast<size_t>(std::distance(input_conversions_.begin(), source));
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn A1JointTrajectoryCommandBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  const auto result = Base::on_deactivate(previous_state);
  cached_state_interfaces_.clear();
  cached_state_handles_.clear();
  input_conversions_.clear();
  return result;
}

controller_interface::return_type A1JointTrajectoryCommandBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (cached_state_handles_.size() != state_interfaces_.size()) {
    return controller_interface::return_type::ERROR;
  }
  // The common pose bridge resolves reference_type from YAML. Do not assume a
  // numeric mode ID: only joint references receive A1 mapping and scaling.
  const bool joint_reference = params_.enable_teleoperation &&
    !pose_reference_mode_rt_.load(std::memory_order_acquire);
  // Read every physical source before constructing either swapped destination.
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    const auto measurement = state_interfaces_[i].get_optional();
    if (measurement) {
      input_conversions_[i].last_raw_position = *measurement;
    }
  }
  for (size_t i = 0; i < input_conversions_.size(); ++i) {
    const auto & conversion = input_conversions_[i];
    const auto source_index = joint_reference ? conversion.joint_reference_source_index : i;
    double cached_position = input_conversions_[source_index].last_raw_position;
    if (conversion.is_position && joint_reference && std::isfinite(cached_position)) {
      const double corrected = cached_position * conversion.sign + conversion.offset;
      const double target = scale_command_position(conversion.joint_name, corrected);
      if (target != corrected) {
        // The unmodified base will apply its reverse/offset again to this private input.
        cached_position = (target - conversion.offset) * conversion.sign;
      }
    }
    if (!cached_state_handles_[i]->set_value(cached_position)) {
      return controller_interface::return_type::ERROR;
    }
  }
  ScopedInputView input_view(state_interfaces_, cached_state_interfaces_);
  return Base::update(time, period);
}

double A1JointTrajectoryCommandBroadcaster::scale_command_position(
  const std::string & joint_name, double position) const
{
  // Only transform MoveJ references; preserve non-finite measurements for feedback validation.
  if (!std::isfinite(position)) {
    return position;
  }
  const auto & joint6 = scaling_params_.joint6_scaling;
  const auto & joint7 = scaling_params_.joint7_scaling;
  if (joint_name == "arm_l_joint6" || joint_name == "arm_r_joint6") {
    return scale_position(position, joint6.gain, kJoint6Range);
  }
  if (joint_name == "arm_l_joint7") {
    return scale_position(position, joint7.gain, kLeftJoint7Range);
  }
  if (joint_name == "arm_r_joint7") {
    return scale_position(position, joint7.gain, kRightJoint7Range);
  }
  return position;
}
}  // namespace a1_joint_trajectory_command_broadcaster

PLUGINLIB_EXPORT_CLASS(
  a1_joint_trajectory_command_broadcaster::A1JointTrajectoryCommandBroadcaster,
  controller_interface::ControllerInterface)
