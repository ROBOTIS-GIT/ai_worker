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

#ifndef FFW_SWERVE_DRIVE_CONTROLLER__MARKER_VISUALIZE_HPP_
#define FFW_SWERVE_DRIVE_CONTROLLER__MARKER_VISUALIZE_HPP_

#include <string>
#include <vector>
#include <cmath>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


namespace ffw_swerve_drive_controller
{

struct Point2D { double x, y; };

class MarkerVisualize
{
public:
  // Default constructor explicitly initializing rclcpp::Duration member
  MarkerVisualize()
  : marker_lifetime_(0, 0) {}

  void init(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_interface,
    const std::string & base_frame_id,
    const std::string & marker_topic_name,
    size_t num_modules,
    double visualization_update_period_sec);

  void publish_markers(
    const rclcpp::Time & stamp,
    double robot_vx,
    double robot_vy,
    double robot_wz,
    const std::vector<double> & module_x_offsets,
    const std::vector<double> & module_y_offsets,
    const std::vector<double> & actual_robot_frame_steering_angles,
    const std::vector<double> & final_wheel_linear_vels);

private:
  static constexpr double ROBOT_ARROW_SCALE_FACTOR = 1.0;
  static constexpr double MODULE_ARROW_SCALE_FACTOR = 1.0;
  static constexpr double ARROW_SHAFT_DIAMETER = 0.035;
  static constexpr double ARROW_HEAD_DIAMETER = 0.07;
  static constexpr double ARROW_HEAD_LENGTH = 0.07;
  static constexpr double LINE_WIDTH = 0.025;
  static constexpr double SPHERE_RADIUS = 0.08;
  static constexpr double ROBOT_TEXT_SCALE = 0.12;
  static constexpr double MODULE_INFO_TEXT_SCALE = 0.08;
  static constexpr double ICR_TEXT_SCALE = 0.09;
  static constexpr double ICR_DIST_TEXT_SCALE = 0.06;
  static constexpr double STEERING_CUBE_LENGTH = 0.15;
  static constexpr double STEERING_CUBE_WIDTH = 0.04;
  static constexpr double STEERING_CUBE_THICKNESS = 0.05;

  visualization_msgs::msg::Marker create_arrow_marker(
    int id, const rclcpp::Time & stamp, const std::string & ns,
    const geometry_msgs::msg::Point & start_point,
    const geometry_msgs::msg::Point & end_point,
    const std_msgs::msg::ColorRGBA & color,
    double scale_x = ARROW_SHAFT_DIAMETER,
    double scale_y = ARROW_HEAD_DIAMETER,
    double scale_z = ARROW_HEAD_LENGTH);

  visualization_msgs::msg::Marker create_line_strip_marker(
    int id, const rclcpp::Time & stamp, const std::string & ns,
    const std::vector<geometry_msgs::msg::Point> & points,
    const std_msgs::msg::ColorRGBA & color,
    double scale_x = LINE_WIDTH);

  visualization_msgs::msg::Marker create_sphere_marker(
    int id, const rclcpp::Time & stamp, const std::string & ns,
    const geometry_msgs::msg::Point & position,
    double radius,
    const std_msgs::msg::ColorRGBA & color);

  visualization_msgs::msg::Marker create_text_marker(
    int id, const rclcpp::Time & stamp, const std::string & ns,
    const std::string & text,
    const geometry_msgs::msg::Point & position,
    const std_msgs::msg::ColorRGBA & color,
    double scale_z);


  visualization_msgs::msg::Marker create_cube_marker(
    int id, const rclcpp::Time & stamp, const std::string & ns,
    const geometry_msgs::msg::Pose & pose,
    const geometry_msgs::msg::Vector3 & scale,
    const std_msgs::msg::ColorRGBA & color);

  void add_delete_marker_action(
    visualization_msgs::msg::MarkerArray & marker_array,
    int id, const std::string & ns);
  void add_delete_all_markers_in_ns_action(
    visualization_msgs::msg::MarkerArray & marker_array,
    const std::string & ns);

  std::optional<Point2D> get_line_intersection(
    Point2D p1, Point2D p2, Point2D p3, Point2D p4);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_interface_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_ = nullptr;
  std::string base_frame_id_;
  size_t num_modules_ = 0;
  rclcpp::Duration marker_lifetime_;

  // Fixed ID allocation
  const int ROBOT_VEL_ARROW_ID = 0;
  const int ROBOT_VEL_TEXT_ID = 1;
  const int TARGET_ICR_POINT_ID = 100;
  const int TARGET_ICR_TEXT_ID = 101;
  const int MODULE_VEL_ARROW_BASE_ID = 200;
  const int MODULE_INFO_TEXT_BASE_ID = 400;
  const int MODULE_ICR_DIST_TEXT_BASE_ID = 500;
  const int MODULE_LINE_TO_ICR_BASE_ID = 600;
  const int MODULE_STEERING_CUBE_BASE_ID = 700;
  const int WHEEL_PERPENDICULAR_LINE_BASE_ID = 800;
  const int INTERSECTION_POINT_BASE_ID = 900;
  const int AVG_INTERSECTION_POINT_ID = 999;
  const int AVG_INTERSECTION_TEXT_ID = 1000;
};

}  // namespace ffw_swerve_drive_controller

#endif  // FFW_SWERVE_DRIVE_CONTROLLER__MARKER_VISUALIZE_HPP_
