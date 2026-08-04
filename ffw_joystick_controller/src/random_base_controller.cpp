// Copyright 2024 ROBOTIS CO., LTD.
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

#include "joystick_controller/random_base_controller.hpp"

#include <algorithm>
#include <cmath>

namespace joystick_controller
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
}  // namespace

RandomBaseController::RandomBaseController(uint32_t seed)
: random_engine_(seed)
{
}

bool RandomBaseController::is_config_valid(const RandomBaseControllerConfig & config)
{
  const bool values_are_finite =
    std::isfinite(config.radius) && std::isfinite(config.yaw_range) &&
    std::isfinite(config.linear_gain) && std::isfinite(config.angular_gain) &&
    std::isfinite(config.min_linear_speed) && std::isfinite(config.max_linear_speed) &&
    std::isfinite(config.min_angular_speed) && std::isfinite(config.max_angular_speed) &&
    std::isfinite(config.max_linear_acceleration) &&
    std::isfinite(config.max_angular_acceleration) &&
    std::isfinite(config.position_tolerance) && std::isfinite(config.yaw_tolerance) &&
    std::isfinite(config.settle_linear_velocity) &&
    std::isfinite(config.settle_angular_velocity) &&
    std::isfinite(config.settle_duration) && std::isfinite(config.motion_timeout) &&
    std::isfinite(config.anchor_position_limit) && std::isfinite(config.anchor_yaw_limit) &&
    std::isfinite(config.reanchor_distance);

  return values_are_finite && config.radius > 0.0 && config.yaw_range >= 0.0 &&
         config.yaw_range <= kPi && config.linear_gain > 0.0 && config.angular_gain > 0.0 &&
         config.min_linear_speed > 0.0 &&
         config.max_linear_speed >= config.min_linear_speed &&
         config.min_angular_speed > 0.0 &&
         config.max_angular_speed >= config.min_angular_speed &&
         config.max_linear_acceleration > 0.0 && config.max_angular_acceleration > 0.0 &&
         config.position_tolerance > 0.0 && config.yaw_tolerance > 0.0 &&
         config.settle_linear_velocity >= 0.0 && config.settle_angular_velocity >= 0.0 &&
         config.settle_duration >= 0.0 && config.motion_timeout > config.settle_duration &&
         config.anchor_position_limit >= config.radius + config.position_tolerance &&
         config.anchor_yaw_limit >= config.yaw_range + config.yaw_tolerance &&
         config.anchor_yaw_limit <= kPi &&
         config.reanchor_distance > config.anchor_position_limit;
}

double RandomBaseController::normalize_angle(double angle)
{
  return std::remainder(angle, kTwoPi);
}

bool RandomBaseController::set_config(const RandomBaseControllerConfig & config)
{
  if (!is_config_valid(config)) {
    return false;
  }

  config_ = config;
  reset();
  return true;
}

void RandomBaseController::reseed(uint32_t seed)
{
  random_engine_.seed(seed);
}

bool RandomBaseController::start(
  const PlanarPose & current_pose, double current_time_seconds)
{
  const auto status = start_with_policy(
    current_pose, current_time_seconds, RandomBaseStartOptions{});
  return status == RandomBaseStartStatus::STARTED_WITH_NEW_ANCHOR ||
         status == RandomBaseStartStatus::STARTED_WITH_EXISTING_ANCHOR;
}

RandomBaseStartStatus RandomBaseController::start_with_policy(
  const PlanarPose & current_pose,
  double current_time_seconds,
  const RandomBaseStartOptions & options)
{
  RandomBaseStartStatus status = RandomBaseStartStatus::STARTED_WITH_EXISTING_ANCHOR;
  if (active_ || !is_pose_finite(current_pose) || !std::isfinite(current_time_seconds)) {
    return active_ ? RandomBaseStartStatus::BUSY : RandomBaseStartStatus::INVALID_INPUT;
  }

  if (!has_anchor_) {
    anchor_ = current_pose;
    anchor_.yaw = normalize_angle(anchor_.yaw);
    has_anchor_ = true;
    status = RandomBaseStartStatus::STARTED_WITH_NEW_ANCHOR;
  } else if (!is_within_anchor_envelope(current_pose)) {
    const double anchor_distance = std::hypot(
      current_pose.x - anchor_.x, current_pose.y - anchor_.y);
    if (!options.allow_reanchor || anchor_distance < config_.reanchor_distance) {
      return RandomBaseStartStatus::OUTSIDE_ANCHOR_ENVELOPE;
    }
    if (options.previous_auto_failed) {
      return RandomBaseStartStatus::REANCHOR_BLOCKED_AFTER_AUTO_FAILURE;
    }
    if (!options.reanchor_stationary) {
      return RandomBaseStartStatus::REANCHOR_REQUIRES_STATIONARY_ODOMETRY;
    }

    anchor_ = current_pose;
    anchor_.yaw = normalize_angle(anchor_.yaw);
    status = RandomBaseStartStatus::STARTED_WITH_REANCHORED_POSE;
  }

  target_ = sample_target();
  motion_start_time_ = current_time_seconds;
  settle_start_time_ = 0.0;
  last_command_time_ = current_time_seconds;
  last_command_ = PlanarTwist{};
  settling_ = false;
  active_ = true;
  return status;
}

RandomBaseStartStatus RandomBaseController::start_return_to_anchor(
  const PlanarPose & current_pose,
  double current_time_seconds)
{
  if (active_ || !is_pose_finite(current_pose) || !std::isfinite(current_time_seconds)) {
    return active_ ? RandomBaseStartStatus::BUSY : RandomBaseStartStatus::INVALID_INPUT;
  }
  if (!has_anchor_) {
    return RandomBaseStartStatus::ANCHOR_UNAVAILABLE;
  }
  if (!is_within_anchor_envelope(current_pose)) {
    return RandomBaseStartStatus::OUTSIDE_ANCHOR_ENVELOPE;
  }

  target_ = anchor_;
  motion_start_time_ = current_time_seconds;
  settle_start_time_ = 0.0;
  last_command_time_ = current_time_seconds;
  last_command_ = PlanarTwist{};
  settling_ = false;
  active_ = true;
  return RandomBaseStartStatus::STARTED_RETURN_TO_ANCHOR;
}

void StationaryOdometryWindow::observe(
  const PlanarTwist & twist,
  int64_t sample_time_ns,
  double linear_velocity_limit,
  double angular_velocity_limit,
  int64_t maximum_sample_gap_ns)
{
  const bool inputs_are_valid =
    sample_time_ns >= 0 && std::isfinite(twist.linear_x) &&
    std::isfinite(twist.linear_y) && std::isfinite(twist.angular_z) &&
    std::isfinite(linear_velocity_limit) && linear_velocity_limit >= 0.0 &&
    std::isfinite(angular_velocity_limit) && angular_velocity_limit >= 0.0 &&
    maximum_sample_gap_ns >= 0;
  if (!inputs_are_valid) {
    reset();
    return;
  }

  if (has_sample_ &&
    (sample_time_ns < last_sample_time_ns_ ||
    sample_time_ns - last_sample_time_ns_ > maximum_sample_gap_ns))
  {
    reset();
  }

  const bool low_velocity =
    std::hypot(twist.linear_x, twist.linear_y) <= linear_velocity_limit &&
    std::abs(twist.angular_z) <= angular_velocity_limit;
  if (low_velocity) {
    if (!stationary_) {
      stationary_since_ns_ = sample_time_ns;
      stationary_ = true;
    }
  } else {
    stationary_ = false;
    stationary_since_ns_ = 0;
  }

  last_sample_time_ns_ = sample_time_ns;
  has_sample_ = true;
}

void StationaryOdometryWindow::reset()
{
  stationary_since_ns_ = 0;
  last_sample_time_ns_ = 0;
  has_sample_ = false;
  stationary_ = false;
}

bool StationaryOdometryWindow::satisfies(int64_t required_duration_ns) const
{
  return required_duration_ns >= 0 && has_sample_ && stationary_ &&
         last_sample_time_ns_ >= stationary_since_ns_ &&
         last_sample_time_ns_ - stationary_since_ns_ >= required_duration_ns;
}

RandomBaseStepResult RandomBaseController::update(
  const PlanarPose & current_pose,
  const PlanarTwist & current_twist,
  double current_time_seconds)
{
  RandomBaseStepResult result;
  if (!active_) {
    return result;
  }

  if (!is_pose_finite(current_pose) || !is_twist_finite(current_twist) ||
    !std::isfinite(current_time_seconds) || current_time_seconds < motion_start_time_ ||
    current_time_seconds < last_command_time_)
  {
    cancel();
    result.status = RandomBaseStepStatus::INVALID_INPUT;
    return result;
  }

  if (!is_within_anchor_envelope(current_pose)) {
    cancel();
    result.status = RandomBaseStepStatus::OUTSIDE_ANCHOR_ENVELOPE;
    return result;
  }

  if ((current_time_seconds - motion_start_time_) >= config_.motion_timeout) {
    cancel();
    result.status = RandomBaseStepStatus::TIMED_OUT;
    return result;
  }

  const double error_x = target_.x - current_pose.x;
  const double error_y = target_.y - current_pose.y;
  const double position_error = std::hypot(error_x, error_y);
  const double yaw_error = normalize_angle(target_.yaw - current_pose.yaw);
  const double linear_velocity = std::hypot(current_twist.linear_x, current_twist.linear_y);
  const bool pose_is_settled =
    position_error <= config_.position_tolerance &&
    std::abs(yaw_error) <= config_.yaw_tolerance;
  const bool velocity_is_settled =
    linear_velocity <= config_.settle_linear_velocity &&
    std::abs(current_twist.angular_z) <= config_.settle_angular_velocity;

  if (pose_is_settled && velocity_is_settled) {
    last_command_ = PlanarTwist{};
    last_command_time_ = current_time_seconds;
    if (!settling_) {
      settling_ = true;
      settle_start_time_ = current_time_seconds;
    }

    if ((current_time_seconds - settle_start_time_) >= config_.settle_duration) {
      cancel();
      result.status = RandomBaseStepStatus::SUCCEEDED;
      return result;
    }

    result.status = RandomBaseStepStatus::ACTIVE;
    return result;
  }

  settling_ = false;
  result.status = RandomBaseStepStatus::ACTIVE;
  PlanarTwist desired_command = calculate_command(current_pose, position_error, yaw_error);
  if (position_error <= config_.position_tolerance) {
    desired_command.linear_x = 0.0;
    desired_command.linear_y = 0.0;
    last_command_.linear_x = 0.0;
    last_command_.linear_y = 0.0;
  }
  if (std::abs(yaw_error) <= config_.yaw_tolerance) {
    desired_command.angular_z = 0.0;
    last_command_.angular_z = 0.0;
  }
  result.command = limit_command_rate(desired_command, current_time_seconds);
  return result;
}

void RandomBaseController::cancel()
{
  active_ = false;
  settling_ = false;
  settle_start_time_ = 0.0;
  last_command_time_ = 0.0;
  last_command_ = PlanarTwist{};
}

void RandomBaseController::reset()
{
  cancel();
  has_anchor_ = false;
  motion_start_time_ = 0.0;
  anchor_ = PlanarPose{};
  target_ = PlanarPose{};
}

bool RandomBaseController::is_active() const
{
  return active_;
}

bool RandomBaseController::has_anchor() const
{
  return has_anchor_;
}

const PlanarPose & RandomBaseController::anchor() const
{
  return anchor_;
}

const PlanarPose & RandomBaseController::target() const
{
  return target_;
}

bool RandomBaseController::is_pose_finite(const PlanarPose & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

bool RandomBaseController::is_twist_finite(const PlanarTwist & twist)
{
  return std::isfinite(twist.linear_x) && std::isfinite(twist.linear_y) &&
         std::isfinite(twist.angular_z);
}

double RandomBaseController::clamp_with_minimum(
  double value, double minimum, double maximum)
{
  if (value == 0.0) {
    return 0.0;
  }

  return std::copysign(std::clamp(std::abs(value), minimum, maximum), value);
}

bool RandomBaseController::is_within_anchor_envelope(const PlanarPose & pose) const
{
  if (!has_anchor_ || !is_pose_finite(pose)) {
    return false;
  }

  const double anchor_distance = std::hypot(pose.x - anchor_.x, pose.y - anchor_.y);
  const double anchor_yaw_error = std::abs(normalize_angle(pose.yaw - anchor_.yaw));
  return anchor_distance <= config_.anchor_position_limit &&
         anchor_yaw_error <= config_.anchor_yaw_limit;
}

PlanarPose RandomBaseController::sample_target()
{
  const double radius = config_.radius * std::sqrt(unit_distribution_(random_engine_));
  const double angle = kTwoPi * unit_distribution_(random_engine_);
  const double local_x = radius * std::cos(angle);
  const double local_y = radius * std::sin(angle);
  const double anchor_cos = std::cos(anchor_.yaw);
  const double anchor_sin = std::sin(anchor_.yaw);
  const double yaw_delta = config_.yaw_range * (2.0 * unit_distribution_(random_engine_) - 1.0);

  PlanarPose target;
  target.x = anchor_.x + anchor_cos * local_x - anchor_sin * local_y;
  target.y = anchor_.y + anchor_sin * local_x + anchor_cos * local_y;
  target.yaw = normalize_angle(anchor_.yaw + yaw_delta);
  return target;
}

PlanarTwist RandomBaseController::calculate_command(
  const PlanarPose & current_pose, double position_error, double yaw_error) const
{
  PlanarTwist command;
  if (position_error > config_.position_tolerance) {
    const double error_x = target_.x - current_pose.x;
    const double error_y = target_.y - current_pose.y;
    const double current_cos = std::cos(current_pose.yaw);
    const double current_sin = std::sin(current_pose.yaw);
    const double body_error_x = current_cos * error_x + current_sin * error_y;
    const double body_error_y = -current_sin * error_x + current_cos * error_y;
    const double speed = clamp_with_minimum(
      config_.linear_gain * position_error,
      config_.min_linear_speed, config_.max_linear_speed);
    command.linear_x = speed * body_error_x / position_error;
    command.linear_y = speed * body_error_y / position_error;
  }

  if (std::abs(yaw_error) > config_.yaw_tolerance) {
    command.angular_z = clamp_with_minimum(
      config_.angular_gain * yaw_error,
      config_.min_angular_speed,
      config_.max_angular_speed);
  }

  return command;
}

PlanarTwist RandomBaseController::limit_command_rate(
  const PlanarTwist & desired_command, double current_time_seconds)
{
  const double time_step = current_time_seconds - last_command_time_;
  if (time_step <= 0.0) {
    return last_command_;
  }

  PlanarTwist command = desired_command;
  const double linear_delta_x = desired_command.linear_x - last_command_.linear_x;
  const double linear_delta_y = desired_command.linear_y - last_command_.linear_y;
  const double linear_delta = std::hypot(linear_delta_x, linear_delta_y);
  const double maximum_linear_delta = config_.max_linear_acceleration * time_step;
  if (linear_delta > maximum_linear_delta && linear_delta > 0.0) {
    const double scale = maximum_linear_delta / linear_delta;
    command.linear_x = last_command_.linear_x + scale * linear_delta_x;
    command.linear_y = last_command_.linear_y + scale * linear_delta_y;
  }

  const double angular_delta = desired_command.angular_z - last_command_.angular_z;
  const double maximum_angular_delta = config_.max_angular_acceleration * time_step;
  command.angular_z = last_command_.angular_z + std::clamp(
    angular_delta, -maximum_angular_delta, maximum_angular_delta);

  last_command_ = command;
  last_command_time_ = current_time_seconds;
  return command;
}

}  // namespace joystick_controller
