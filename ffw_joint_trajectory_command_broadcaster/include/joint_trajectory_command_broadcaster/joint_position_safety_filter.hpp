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

#ifndef JOINT_TRAJECTORY_COMMAND_BROADCASTER__JOINT_POSITION_SAFETY_FILTER_HPP_
#define JOINT_TRAJECTORY_COMMAND_BROADCASTER__JOINT_POSITION_SAFETY_FILTER_HPP_

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "urdf/model.h"

namespace joint_trajectory_command_broadcaster
{

enum class JointSampleStatus
{
  ACCEPTED,
  MISSING,
  NON_FINITE,
  OUT_OF_RANGE,
  EXCESSIVE_JUMP,
  UNKNOWN_JOINT,
};

struct JointSampleResult
{
  JointSampleStatus status = JointSampleStatus::UNKNOWN_JOINT;
  double candidate = std::numeric_limits<double>::quiet_NaN();
  double retained = std::numeric_limits<double>::quiet_NaN();
};

class JointPositionSafetyFilter
{
public:
  void configure(
    const urdf::Model & model, const std::vector<std::string> & joint_names,
    const double max_position_jump, const double position_limit_margin)
  {
    states_.clear();
    max_position_jump_ = max_position_jump;
    position_limit_margin_ = position_limit_margin;

    for (const auto & joint_name : joint_names) {
      JointState state;
      const auto joint = model.getJoint(joint_name);
      if (
        joint && joint->limits &&
        joint->type != urdf::Joint::CONTINUOUS)
      {
        state.has_position_limits = true;
        state.lower = joint->limits->lower;
        state.upper = joint->limits->upper;
      }
      states_.emplace(joint_name, state);
    }
  }

  void reset()
  {
    states_.clear();
  }

  void begin_cycle()
  {
    for (auto & item : states_) {
      item.second.current_sample_valid = false;
    }
  }

  JointSampleResult mark_missing(const std::string & joint_name)
  {
    auto state_it = states_.find(joint_name);
    if (state_it == states_.end()) {
      return {JointSampleStatus::UNKNOWN_JOINT};
    }
    state_it->second.current_sample_valid = false;
    return {
      JointSampleStatus::MISSING,
      std::numeric_limits<double>::quiet_NaN(),
      state_it->second.last_valid};
  }

  JointSampleResult update(const std::string & joint_name, const double candidate)
  {
    auto state_it = states_.find(joint_name);
    if (state_it == states_.end()) {
      return {
        JointSampleStatus::UNKNOWN_JOINT, candidate,
        std::numeric_limits<double>::quiet_NaN()};
    }

    auto & state = state_it->second;
    state.current_sample_valid = false;

    if (!std::isfinite(candidate)) {
      return {JointSampleStatus::NON_FINITE, candidate, state.last_valid};
    }

    if (
      state.has_position_limits &&
      (candidate < state.lower - position_limit_margin_ ||
      candidate > state.upper + position_limit_margin_))
    {
      return {JointSampleStatus::OUT_OF_RANGE, candidate, state.last_valid};
    }

    if (state.initialized && std::abs(candidate - state.last_valid) > max_position_jump_) {
      return {JointSampleStatus::EXCESSIVE_JUMP, candidate, state.last_valid};
    }

    state.last_valid = candidate;
    state.initialized = true;
    state.current_sample_valid = true;
    return {JointSampleStatus::ACCEPTED, candidate, state.last_valid};
  }

  bool is_initialized(const std::string & joint_name) const
  {
    const auto state_it = states_.find(joint_name);
    return state_it != states_.end() && state_it->second.initialized;
  }

  bool is_current_sample_valid(const std::string & joint_name) const
  {
    const auto state_it = states_.find(joint_name);
    return state_it != states_.end() && state_it->second.current_sample_valid;
  }

  bool group_initialized(const std::vector<std::string> & joint_names) const
  {
    for (const auto & joint_name : joint_names) {
      if (!is_initialized(joint_name)) {
        return false;
      }
    }
    return !joint_names.empty();
  }

  double last_valid(const std::string & joint_name) const
  {
    const auto state_it = states_.find(joint_name);
    return state_it == states_.end() ?
           std::numeric_limits<double>::quiet_NaN() : state_it->second.last_valid;
  }

private:
  struct JointState
  {
    double last_valid = std::numeric_limits<double>::quiet_NaN();
    double lower = 0.0;
    double upper = 0.0;
    bool initialized = false;
    bool current_sample_valid = false;
    bool has_position_limits = false;
  };

  std::unordered_map<std::string, JointState> states_;
  double max_position_jump_ = 1.0;
  double position_limit_margin_ = 0.05;
};

}  // namespace joint_trajectory_command_broadcaster

#endif  // JOINT_TRAJECTORY_COMMAND_BROADCASTER__JOINT_POSITION_SAFETY_FILTER_HPP_
