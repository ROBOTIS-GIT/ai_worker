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

#ifndef JOYSTICK_CONTROLLER__RANDOM_BASE_CONTROLLER_HPP_
#define JOYSTICK_CONTROLLER__RANDOM_BASE_CONTROLLER_HPP_

#include <cstdint>
#include <random>

namespace joystick_controller
{

struct PlanarPose
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct PlanarTwist
{
  double linear_x = 0.0;
  double linear_y = 0.0;
  double angular_z = 0.0;
};

struct RandomBaseControllerConfig
{
  double radius = 0.03;
  double yaw_range = 0.087;
  double linear_gain = 2.0;
  double angular_gain = 2.0;
  double min_linear_speed = 0.04;
  double max_linear_speed = 0.15;
  double min_angular_speed = 0.06;
  double max_angular_speed = 0.30;
  double max_linear_acceleration = 0.20;
  double max_angular_acceleration = 0.40;
  double position_tolerance = 0.004;
  double yaw_tolerance = 0.009;
  double settle_linear_velocity = 0.01;
  double settle_angular_velocity = 0.02;
  double settle_duration = 0.2;
  double motion_timeout = 5.0;
  double anchor_position_limit = 0.05;
  double anchor_yaw_limit = 0.105;
  double reanchor_distance = 0.20;
};

struct RandomBaseStartOptions
{
  bool allow_reanchor = false;
  bool reanchor_stationary = false;
  bool previous_auto_failed = false;
};

enum class RandomBaseStartStatus
{
  STARTED_WITH_NEW_ANCHOR,
  STARTED_WITH_EXISTING_ANCHOR,
  STARTED_WITH_REANCHORED_POSE,
  STARTED_RETURN_TO_ANCHOR,
  BUSY,
  INVALID_INPUT,
  ANCHOR_UNAVAILABLE,
  OUTSIDE_ANCHOR_ENVELOPE,
  REANCHOR_REQUIRES_STATIONARY_ODOMETRY,
  REANCHOR_BLOCKED_AFTER_AUTO_FAILURE,
};

enum class RandomBaseStepStatus
{
  IDLE,
  ACTIVE,
  SUCCEEDED,
  TIMED_OUT,
  INVALID_INPUT,
  OUTSIDE_ANCHOR_ENVELOPE,
};

struct RandomBaseStepResult
{
  RandomBaseStepStatus status = RandomBaseStepStatus::IDLE;
  PlanarTwist command;
};

class StationaryOdometryWindow
{
public:
  void observe(
    const PlanarTwist & twist,
    int64_t sample_time_ns,
    double linear_velocity_limit,
    double angular_velocity_limit,
    int64_t maximum_sample_gap_ns);
  void reset();
  bool satisfies(int64_t required_duration_ns) const;

private:
  int64_t stationary_since_ns_ = 0;
  int64_t last_sample_time_ns_ = 0;
  bool has_sample_ = false;
  bool stationary_ = false;
};

class RandomBaseController
{
public:
  explicit RandomBaseController(uint32_t seed = std::random_device{}());

  static bool is_config_valid(const RandomBaseControllerConfig & config);
  static double normalize_angle(double angle);

  bool set_config(const RandomBaseControllerConfig & config);
  void reseed(uint32_t seed);

  bool start(const PlanarPose & current_pose, double current_time_seconds);
  RandomBaseStartStatus start_with_policy(
    const PlanarPose & current_pose,
    double current_time_seconds,
    const RandomBaseStartOptions & options);
  RandomBaseStartStatus start_return_to_anchor(
    const PlanarPose & current_pose,
    double current_time_seconds);
  RandomBaseStepResult update(
    const PlanarPose & current_pose,
    const PlanarTwist & current_twist,
    double current_time_seconds);

  void cancel();
  void reset();

  bool is_active() const;
  bool has_anchor() const;
  const PlanarPose & anchor() const;
  const PlanarPose & target() const;

private:
  static bool is_pose_finite(const PlanarPose & pose);
  static bool is_twist_finite(const PlanarTwist & twist);
  static double clamp_with_minimum(double value, double minimum, double maximum);
  bool is_within_anchor_envelope(const PlanarPose & pose) const;

  PlanarPose sample_target();
  PlanarTwist calculate_command(
    const PlanarPose & current_pose,
    double position_error,
    double yaw_error) const;
  PlanarTwist limit_command_rate(
    const PlanarTwist & desired_command,
    double current_time_seconds);

  RandomBaseControllerConfig config_;
  std::mt19937 random_engine_;
  std::uniform_real_distribution<double> unit_distribution_{0.0, 1.0};

  bool has_anchor_ = false;
  bool active_ = false;
  bool settling_ = false;
  double motion_start_time_ = 0.0;
  double settle_start_time_ = 0.0;
  double last_command_time_ = 0.0;
  PlanarTwist last_command_;
  PlanarPose anchor_;
  PlanarPose target_;
};

}  // namespace joystick_controller

#endif  // JOYSTICK_CONTROLLER__RANDOM_BASE_CONTROLLER_HPP_
