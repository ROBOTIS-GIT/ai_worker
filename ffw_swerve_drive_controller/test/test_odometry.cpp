// Copyright 2026 ROBOTIS .co., Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include <gtest/gtest.h>

#include <stdexcept>

#include "ffw_swerve_drive_controller/odometry.hpp"

namespace ffw_swerve_drive_controller
{
namespace
{

TEST(OdometryTest, UsesEachWheelRadiusForFeedbackVelocity)
{
  Odometry odometry;
  odometry.init(rclcpp::Time(0, 0, RCL_ROS_TIME));
  odometry.setModuleParams(
    {-1.0, 0.0, 1.0}, {0.0, 0.0, 0.0}, {0.1, 0.2, 0.3});

  ASSERT_TRUE(odometry.update({0.0, 0.0, 0.0}, {10.0, 10.0, 10.0}, 1.0));

  // The wheel linear speeds are 1, 2, and 3 m/s, so least-squares forward speed is 2 m/s.
  EXPECT_NEAR(odometry.getVx(), 2.0, 1e-9);
  EXPECT_NEAR(odometry.getVy(), 0.0, 1e-9);
  EXPECT_NEAR(odometry.getWz(), 0.0, 1e-9);
}

TEST(OdometryTest, RejectsMismatchedWheelRadiusCount)
{
  Odometry odometry;

  EXPECT_THROW(
    odometry.setModuleParams({-1.0, 0.0, 1.0}, {0.0, 0.0, 0.0}, {0.1, 0.2}),
    std::runtime_error);
}

TEST(OdometryTest, RejectsNonPositiveWheelRadius)
{
  Odometry odometry;

  EXPECT_THROW(
    odometry.setModuleParams({-1.0, 0.0, 1.0}, {0.0, 0.0, 0.0}, {0.1, 0.0, 0.3}),
    std::runtime_error);
}

}  // namespace
}  // namespace ffw_swerve_drive_controller
