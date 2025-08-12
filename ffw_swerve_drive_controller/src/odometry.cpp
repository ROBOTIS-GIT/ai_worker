// Copyright 2025 ROBOTIS .co., Ltd.
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

/*
 * Author: Geonhee Lee
 */

 #include "ffw_swerve_drive_controller/odometry.hpp"

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <string>

#include "tf2/transform_datatypes.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/logging.hpp"

namespace ffw_swerve_drive_controller
{

Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0, 0, RCL_ROS_TIME),
  position_x_odom_(0.0),
  position_y_odom_(0.0),
  orientation_yaw_odom_(0.0),
  velocity_in_base_frame_linear_x_(0.0),
  velocity_in_base_frame_linear_y_(0.0),
  velocity_in_base_frame_angular_z_(0.0),
  num_modules_(0),
  wheel_radius_(0.0),
  solver_method_(OdomSolverMethod::SVD),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_x_accumulator_(velocity_rolling_window_size),
  linear_y_accumulator_(velocity_rolling_window_size),
  angular_z_accumulator_(velocity_rolling_window_size)
{
  base_frame_offset_.fill(0.0);
}

void Odometry::init(const rclcpp::Time & time, const std::array<double, 3> & base_frame_offset)
{
  position_x_odom_ = 0.0;
  position_y_odom_ = 0.0;
  orientation_yaw_odom_ = 0.0;
  timestamp_ = time;
  base_frame_offset_ = base_frame_offset;

  linear_x_accumulator_ = rcpputils::RollingMeanAccumulator<double>(velocity_rolling_window_size_);
  linear_y_accumulator_ = rcpputils::RollingMeanAccumulator<double>(velocity_rolling_window_size_);
  angular_z_accumulator_ = rcpputils::RollingMeanAccumulator<double>(velocity_rolling_window_size_);
}

void Odometry::setModuleParams(
  const std::vector<double> & module_x_offsets,
  const std::vector<double> & module_y_offsets,
  const double wheel_radius)
{
  if (module_x_offsets.size() != module_y_offsets.size()) {
    throw std::runtime_error("Odometry: Module X and Y offset vectors must have the same size.");
  }
  num_modules_ = module_x_offsets.size();
  if (num_modules_ != 3 && num_modules_ != 4) {
    throw std::runtime_error("Odometry: Number of modules must be 3 or 4.");
  }
  if (wheel_radius <= 0.0) {
    throw std::runtime_error("Odometry: Wheel radius must be positive.");
  }
  module_x_offsets_ = module_x_offsets;
  module_y_offsets_ = module_y_offsets;
  wheel_radius_ = wheel_radius;
}

// ***** set solver method *****
void Odometry::set_solver_method(OdomSolverMethod method)
{
  solver_method_ = method;
  auto logger = rclcpp::get_logger("OdometryClass");   // 로거 사용
  switch (method) {
    case OdomSolverMethod::PSEUDO_INVERSE:
      RCLCPP_INFO(logger, "Odometry solver set to: Normal Equation (Manual Pseudo-inverse)");
      break;
    case OdomSolverMethod::QR_DECOMPOSITION:
      RCLCPP_INFO(logger, "Odometry solver set to: QR Decomposition (Eigen)");
      break;
    case OdomSolverMethod::SVD:
      RCLCPP_INFO(logger, "Odometry solver set to: SVD (Eigen)");
      break;
    default:
      RCLCPP_WARN(logger, "Unknown odometry solver method specified. Defaulting to SVD.");
      solver_method_ = OdomSolverMethod::SVD;
      break;
  }
}
// ***********************************

double Odometry::normalize_angle(double angle_rad)
{
  double remainder = std::fmod(angle_rad + M_PI, 2.0 * M_PI);
  if (remainder < 0.0) {
    remainder += 2.0 * M_PI;
  }
  return remainder - M_PI;
}

bool Odometry::update(
  const std::vector<double> & steering_positions,
  const std::vector<double> & wheel_velocities,
  const double dt)
{
  auto logger = rclcpp::get_logger("swerve_odometry_update");

  if (num_modules_ == 0 || wheel_radius_ == 0.0) { /* ... */ return false;}
  if (steering_positions.size() != num_modules_ || wheel_velocities.size() != num_modules_) {
    return false;
  }
  if (dt < 0.00001) { /* ... */ return false;}

  RCLCPP_DEBUG(
    logger, "Odometry Update ---- dt: %.4f ---- Solver: %d", dt,
    static_cast<int>(solver_method_));

  size_t num_equations = 2 * num_modules_;
  Eigen::MatrixXd A_eigen(num_equations, 3);
  Eigen::VectorXd b_eigen(num_equations);

  for (size_t i = 0; i < num_modules_; ++i) {
    const double theta_s = steering_positions[i];
    const double omega_w = wheel_velocities[i];
    const double v_w = omega_w * wheel_radius_;
    const double lx = module_x_offsets_[i];
    const double ly = module_y_offsets_[i];

    const double vx_module = v_w * std::cos(theta_s);
    const double vy_module = v_w * std::sin(theta_s);

    size_t row1 = i * 2;
    size_t row2 = row1 + 1;

    A_eigen(row1, 0) = 1.0; A_eigen(row1, 1) = 0.0; A_eigen(row1, 2) = -ly;
    b_eigen(row1) = vx_module;

    A_eigen(row2, 0) = 0.0; A_eigen(row2, 1) = 1.0; A_eigen(row2, 2) = lx;
    b_eigen(row2) = vy_module;
  }

  Eigen::Vector3d robot_twist_eigen;

  // solver method selection
  if (solver_method_ == OdomSolverMethod::PSEUDO_INVERSE) {
    RCLCPP_DEBUG_ONCE(logger, "Using Normal Equation solver for odometry.");
    // least squares solution using normal equations
    std::vector<std::vector<double>> A_manual(num_equations, std::vector<double>(3));
    std::vector<double> b_manual(num_equations);
    for (size_t r = 0; r < num_equations; ++r) {
      b_manual[r] = b_eigen(r);
      for (size_t c = 0; c < 3; ++c) {
        A_manual[r][c] = A_eigen(r, c);
      }
    }

    std::vector<std::vector<double>> AT(3, std::vector<double>(num_equations));
    for (size_t i = 0; i < num_equations; ++i) {
      for (size_t j = 0; j < 3; ++j) {
        AT[j][i] = A_manual[i][j];
      }
    }
    std::vector<std::vector<double>> ATA(3, std::vector<double>(3, 0.0));
    for (size_t i = 0; i < 3; ++i) {
      for (size_t j = 0; j < 3; ++j) {
        for (size_t k = 0; k < num_equations; ++k) {
          ATA[i][j] += AT[i][k] * A_manual[k][j];
        }
      }
    }
    std::vector<double> ATb(3, 0.0);
    for (size_t i = 0; i < 3; ++i) {
      for (size_t j = 0; j < num_equations; ++j) {
        ATb[i] += AT[i][j] * b_manual[j];
      }
    }
    double det = ATA[0][0] * (ATA[1][1] * ATA[2][2] - ATA[2][1] * ATA[1][2]) -
      ATA[0][1] * (ATA[1][0] * ATA[2][2] - ATA[1][2] * ATA[2][0]) +
      ATA[0][2] * (ATA[1][0] * ATA[2][1] - ATA[1][1] * ATA[2][0]);
    if (std::abs(det) < 1e-9) {
      // when the matrix is singular or near-singular
      RCLCPP_WARN(
        logger,
        "Normal Equation: Matrix ATA is singular (det: %e). Setting twist to zero.",
        det);
      robot_twist_eigen.setZero();
    } else {
      double invDet = 1.0 / det;
      std::vector<std::vector<double>> invATA(3, std::vector<double>(3));
      // ... 3 x 3 inverse matrix calculation
      invATA[0][0] = (ATA[1][1] * ATA[2][2] - ATA[2][1] * ATA[1][2]) * invDet;
      invATA[0][1] = (ATA[0][2] * ATA[2][1] - ATA[0][1] * ATA[2][2]) * invDet;
      invATA[0][2] = (ATA[0][1] * ATA[1][2] - ATA[0][2] * ATA[1][1]) * invDet;
      invATA[1][0] = (ATA[1][2] * ATA[2][0] - ATA[1][0] * ATA[2][2]) * invDet;
      invATA[1][1] = (ATA[0][0] * ATA[2][2] - ATA[0][2] * ATA[2][0]) * invDet;
      invATA[1][2] = (ATA[1][0] * ATA[0][2] - ATA[0][0] * ATA[1][2]) * invDet;
      invATA[2][0] = (ATA[1][0] * ATA[2][1] - ATA[2][0] * ATA[1][1]) * invDet;
      invATA[2][1] = (ATA[2][0] * ATA[0][1] - ATA[0][0] * ATA[2][1]) * invDet;
      invATA[2][2] = (ATA[0][0] * ATA[1][1] - ATA[1][0] * ATA[0][1]) * invDet;

      robot_twist_eigen(0) = invATA[0][0] * ATb[0] + invATA[0][1] * ATb[1] + invATA[0][2] * ATb[2];
      robot_twist_eigen(1) = invATA[1][0] * ATb[0] + invATA[1][1] * ATb[1] + invATA[1][2] * ATb[2];
      robot_twist_eigen(2) = invATA[2][0] * ATb[0] + invATA[2][1] * ATb[1] + invATA[2][2] * ATb[2];
    }
  } else if (solver_method_ == OdomSolverMethod::QR_DECOMPOSITION) {
    RCLCPP_DEBUG_ONCE(logger, "Using QR Decomposition solver for odometry.");
    // Eigen ColPivHouseholderQR for rank deficient matrices
    robot_twist_eigen = A_eigen.colPivHouseholderQr().solve(b_eigen);
  } else {
    // OdomSolverMethod::SVD
    RCLCPP_DEBUG_ONCE(logger, "Using SVD solver for odometry.");
    // Eigen SVD for rank deficient matrices
    robot_twist_eigen = A_eigen.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_eigen);
  }
  // ****************************************

  Eigen::Vector3d robot_filtered_twist =
    updateFromVelocity(robot_twist_eigen(0), robot_twist_eigen(1), robot_twist_eigen(2));
  velocity_in_base_frame_linear_x_ = robot_filtered_twist(0);
  velocity_in_base_frame_linear_y_ = robot_filtered_twist(1);
  velocity_in_base_frame_angular_z_ = robot_filtered_twist(2);

  // --- Integration (Update pose based on calculated twist) ---
  // The integration logic remains the same as Mecanum, using the calculated twist
  // NOTE: The position is expressed in the odometry frame, while the twist is in the base frame.

  // Integrate orientation (yaw) first
  orientation_yaw_odom_ += velocity_in_base_frame_angular_z_ * dt;
  // Normalize yaw angle
  orientation_yaw_odom_ = normalize_angle(orientation_yaw_odom_);   // Use the helper function

  // Convert base frame velocity to odometry frame velocity
  tf2::Quaternion orientation_q;
  orientation_q.setRPY(0.0, 0.0, orientation_yaw_odom_);   // Current orientation in odom frame

  tf2::Matrix3x3 rotation_matrix(orientation_q);
  tf2::Vector3 velocity_in_base_frame(
    velocity_in_base_frame_linear_x_,
    velocity_in_base_frame_linear_y_,
    0.0);
  tf2::Vector3 velocity_in_odom_frame = rotation_matrix * velocity_in_base_frame;

  // Integrate position in the odometry frame
  position_x_odom_ += velocity_in_odom_frame.x() * dt;
  position_y_odom_ += velocity_in_odom_frame.y() * dt;
  return true;
}

// Update odometry using target velocities
bool Odometry::update(double target_vx, double target_vy, double target_w, const double dt)
{
  velocity_in_base_frame_linear_x_ = target_vx;
  velocity_in_base_frame_linear_y_ = target_vy;
  velocity_in_base_frame_angular_z_ = target_w;

  // Integrate orientation (yaw) first
  orientation_yaw_odom_ += velocity_in_base_frame_angular_z_ * dt;
  // Normalize yaw angle
  orientation_yaw_odom_ = normalize_angle(orientation_yaw_odom_);

  // Convert base frame velocity to odometry frame velocity
  tf2::Quaternion orientation_q;
  // Current orientation in odom frame
  orientation_q.setRPY(0.0, 0.0, orientation_yaw_odom_);

  tf2::Matrix3x3 rotation_matrix(orientation_q);
  tf2::Vector3 velocity_in_base_frame(
    velocity_in_base_frame_linear_x_,
    velocity_in_base_frame_linear_y_,
    0.0);
  tf2::Vector3 velocity_in_odom_frame = rotation_matrix * velocity_in_base_frame;

  // Integrate position in the odometry frame
  position_x_odom_ += velocity_in_odom_frame.x() * dt;
  position_y_odom_ += velocity_in_odom_frame.y() * dt;
  return true;
}

Eigen::Vector3d Odometry::updateFromVelocity(double linear_x, double linear_y, double angular_z)
{
  Eigen::Vector3d filtered_twist;

  if (velocity_rolling_window_size_ <= 1) {
    // not use rolling mean
    filtered_twist(0) = linear_x;
    filtered_twist(1) = linear_y;
    filtered_twist(2) = angular_z;
    return filtered_twist;
  }

  // RollingMeanAccumulator에 '속도'를 직접 누적
  linear_x_accumulator_.accumulate(linear_x);
  linear_y_accumulator_.accumulate(linear_y);
  angular_z_accumulator_.accumulate(angular_z);

  filtered_twist(0) = linear_x_accumulator_.getRollingMean();
  filtered_twist(1) = linear_y_accumulator_.getRollingMean();
  filtered_twist(2) = angular_z_accumulator_.getRollingMean();

  return filtered_twist;
}

void Odometry::resetAccumulators()
{
  linear_x_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  linear_y_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_z_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  RCLCPP_INFO(
    rclcpp::get_logger(
      "OdometryClass"), "Setting velocity rolling window size to %zu",
    velocity_rolling_window_size);

  velocity_rolling_window_size_ = velocity_rolling_window_size;

  resetAccumulators();
}
}  // namespace ffw_swerve_drive_controller
