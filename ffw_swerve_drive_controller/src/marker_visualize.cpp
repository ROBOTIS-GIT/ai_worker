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

#include <iomanip>
#include <optional>

#include "ffw_swerve_drive_controller/marker_visualize.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace ffw_swerve_drive_controller
{

void MarkerVisualize::init(
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_interface_weak_ptr,
  const std::string & base_frame_id,
  const std::string & marker_topic_name,
  size_t num_modules,
  double visualization_update_period_sec)
{
  node_interface_ = node_interface_weak_ptr;
  base_frame_id_ = base_frame_id;
  num_modules_ = num_modules;

  if (visualization_update_period_sec <= 0.0) {
    marker_lifetime_ = rclcpp::Duration::from_seconds(0.25);
  } else {
    marker_lifetime_ = rclcpp::Duration::from_seconds(visualization_update_period_sec * 1.2 + 0.05);
  }

  if (auto node = node_interface_.lock()) {
    marker_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_name, rclcpp::SystemDefaultsQoS());
    RCLCPP_INFO(
      node->get_logger(), "MarkerVisualize initialized, publishing to: %s. Marker lifetime: %.2f s",
      marker_topic_name.c_str(), marker_lifetime_.seconds());
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("MarkerVisualize"),
      "MarkerVisualize: Failed to lock lifecycle node in init. Publisher not created.");
  }
}

void MarkerVisualize::add_delete_marker_action(
  visualization_msgs::msg::MarkerArray & marker_array,
  int id, const std::string & ns)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = base_frame_id_;
  marker.ns = ns;
  marker.id = id;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  marker_array.markers.push_back(marker);
}

std::optional<Point2D> MarkerVisualize::get_line_intersection(
  Point2D p1, Point2D p2, Point2D p3, Point2D p4)
{
  double x1 = p1.x, y1 = p1.y;
  double x2 = p2.x, y2 = p2.y;
  double x3 = p3.x, y3 = p3.y;
  double x4 = p4.x, y4 = p4.y;
  double den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
  if (std::abs(den) < 1e-9) {return std::nullopt;}
  Point2D intersection_point;
  intersection_point.x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / den;
  intersection_point.y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / den;
  return intersection_point;
}


// DELETEALL to delete all markers in a specific namespace
void MarkerVisualize::add_delete_all_markers_in_ns_action(
  visualization_msgs::msg::MarkerArray & marker_array,
  const std::string & ns)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = base_frame_id_;
  marker.ns = ns;
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(marker);
}

void MarkerVisualize::publish_markers(
  const rclcpp::Time & stamp,
  double robot_vx,
  double robot_vy,
  double robot_wz,
  const std::vector<double> & module_x_offsets,
  const std::vector<double> & module_y_offsets,
  const std::vector<double> & actual_robot_frame_steering_angles,
  const std::vector<double> & final_wheel_linear_vels)
{
  auto node = node_interface_.lock();
  if (!node || !marker_publisher_) {return;}
  if (module_x_offsets.size() != num_modules_ ||
    module_y_offsets.size() != num_modules_ ||
    actual_robot_frame_steering_angles.size() != num_modules_ ||
    final_wheel_linear_vels.size() != num_modules_)
  {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 1000,
      "MarkerVisualize: Mismatch in module data sizes. Expected %zu.", num_modules_);
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;

  // ***** excute delete action *****
  add_delete_all_markers_in_ns_action(marker_array, "icr_visualization");
  add_delete_all_markers_in_ns_action(marker_array, "icr_visualization_text");
  add_delete_all_markers_in_ns_action(marker_array, "icr_to_robot_visualization_lines");
  add_delete_all_markers_in_ns_action(marker_array, "actual_icr_intersections");
  add_delete_all_markers_in_ns_action(marker_array, "actual_icr_average");
  add_delete_all_markers_in_ns_action(marker_array, "actual_icr_average_text");

  // --- 1. visualize robot velocity vector and text ---
  geometry_msgs::msg::Point robot_vel_start, robot_vel_end, robot_text_pos;
  robot_vel_start.x = 0.0; robot_vel_start.y = 0.0; robot_vel_start.z = 0.18;
  robot_vel_end.x = robot_vx * ROBOT_ARROW_SCALE_FACTOR;
  robot_vel_end.y = robot_vy * ROBOT_ARROW_SCALE_FACTOR;
  robot_vel_end.z = 0.18;
  robot_text_pos.x = 0.0;
  robot_text_pos.y = -2.0;
  robot_text_pos.z = 0.3;

  std_msgs::msg::ColorRGBA robot_vel_color;
  robot_vel_color.r = 1.0f; robot_vel_color.g = 0.0f; robot_vel_color.b = 0.0f;
  robot_vel_color.a = 1.0f;
  std_msgs::msg::ColorRGBA text_color_white;
  text_color_white.r = 1.0f; text_color_white.g = 1.0f; text_color_white.b = 1.0f;
  text_color_white.a = 1.0f;
  std_msgs::msg::ColorRGBA text_color_blue;
  text_color_blue.r = 0.0f; text_color_blue.g = 0.0f; text_color_blue.b = 1.0f;
  text_color_blue.a = 1.0f;
  std_msgs::msg::ColorRGBA text_color_yellow;
  text_color_yellow.r = 1.0f; text_color_yellow.g = 1.0f; text_color_yellow.b = 0.0f;
  text_color_yellow.a = 1.0f;

  marker_array.markers.push_back(
    create_arrow_marker(
      ROBOT_VEL_ARROW_ID, stamp, "robot_velocity_info", robot_vel_start, robot_vel_end,
      robot_vel_color));

  std::ostringstream robot_vel_ss;
  robot_vel_ss << std::fixed << std::setprecision(2)
               << "[cmd_vel]"
               << "\n vx: " << robot_vx
               << "\n vy: " << robot_vy
               << "\n wz: " << robot_wz;
  marker_array.markers.push_back(
    create_text_marker(
      ROBOT_VEL_TEXT_ID, stamp, "robot_velocity_info_text", robot_vel_ss.str(), robot_text_pos,
      text_color_blue, MarkerVisualize::ROBOT_TEXT_SCALE));


  // --- 2.visualize each module's ICR and vector ---
  std_msgs::msg::ColorRGBA module_vel_color;
  module_vel_color.r = 0.0f; module_vel_color.g = 1.0f; module_vel_color.b = 0.0f;
  module_vel_color.a = 1.0f;
  std_msgs::msg::ColorRGBA steering_cube_color;
  steering_cube_color.r = 1.0f; steering_cube_color.g = 1.0f; steering_cube_color.b = 0.0f;
  steering_cube_color.a = 0.7f;
  std_msgs::msg::ColorRGBA wheel_perpendicular_line_color;
  wheel_perpendicular_line_color.r = 0.6f; wheel_perpendicular_line_color.g = 0.6f;
  wheel_perpendicular_line_color.b = 0.6f; wheel_perpendicular_line_color.a = 1.0f;

  std::vector<Point2D> line_start_points(num_modules_);
  std::vector<Point2D> line_end_points(num_modules_);

  std::vector<geometry_msgs::msg::Point> module_text_offsets_storage;
  module_text_offsets_storage.reserve(num_modules_);
  if (num_modules_ == 4) {
    geometry_msgs::msg::Point p;
    p.x = -0.4; p.y = 0.4; p.z = 0.2; module_text_offsets_storage.push_back(p);  // FL
    p.x = 0.4; p.y = 0.4; p.z = 0.2; module_text_offsets_storage.push_back(p);  // FR
    p.x = -0.4; p.y = -0.4; p.z = 0.2; module_text_offsets_storage.push_back(p);  // RL
    p.x = 0.4; p.y = -0.4; p.z = 0.2; module_text_offsets_storage.push_back(p);  // RR
  } else if (num_modules_ == 3) {
    geometry_msgs::msg::Point p;
    p.x = -0.4; p.y = 0.4; p.z = 0.2; module_text_offsets_storage.push_back(p);  // FL
    p.x = 0.4; p.y = 0.4; p.z = 0.2; module_text_offsets_storage.push_back(p);  // FR
    p.x = 0.0; p.y = -0.5; p.z = 0.2; module_text_offsets_storage.push_back(p);  // RC
  }

  for (size_t i = 0; i < num_modules_; ++i) {
    geometry_msgs::msg::Point module_origin, module_info_text_pos;
    module_origin.x = module_x_offsets[i];
    module_origin.y = module_y_offsets[i];
    module_origin.z = MarkerVisualize::STEERING_CUBE_THICKNESS / 2.0;

    if (i < module_text_offsets_storage.size()) {
      module_info_text_pos = module_text_offsets_storage[i];
    } else {
      module_info_text_pos.x = module_x_offsets[i] + (module_y_offsets[i] > 0 ? 0.15 : -0.15);
      module_info_text_pos.y = module_y_offsets[i] + (module_x_offsets[i] > 0 ? -0.15 : 0.15);
      module_info_text_pos.z = 0.2;
    }

    geometry_msgs::msg::Point module_vel_end;
    double robot_frame_steering_angle = actual_robot_frame_steering_angles[i];
    double wheel_lin_vel = final_wheel_linear_vels[i];
    double MODULE_ARROW_SCALE_FACTOR = 0.5;
    module_vel_end.x = module_origin.x + wheel_lin_vel * std::cos(robot_frame_steering_angle) *
      MODULE_ARROW_SCALE_FACTOR;
    module_vel_end.y = module_origin.y + wheel_lin_vel * std::sin(robot_frame_steering_angle) *
      MODULE_ARROW_SCALE_FACTOR;
    module_vel_end.z = module_origin.z;
    marker_array.markers.push_back(
      create_arrow_marker(
        MODULE_VEL_ARROW_BASE_ID + i, stamp, "module_velocities", module_origin, module_vel_end,
        module_vel_color,
        MarkerVisualize::ARROW_SHAFT_DIAMETER * 0.9, MarkerVisualize::ARROW_HEAD_DIAMETER * 0.9,
        MarkerVisualize::ARROW_HEAD_LENGTH * 0.9));

    std::ostringstream module_info_ss;
    module_info_ss << std::fixed << std::setprecision(1) << "Steering:" <<
      (robot_frame_steering_angle * 180.0 / M_PI) << ",Wheel:" << std::fixed << std::setprecision(
      2) << wheel_lin_vel;
    marker_array.markers.push_back(
      create_text_marker(
        MODULE_INFO_TEXT_BASE_ID + i, stamp, "module_info_text", module_info_ss.str(),
        module_info_text_pos, text_color_white, MarkerVisualize::MODULE_INFO_TEXT_SCALE));

    geometry_msgs::msg::Pose steering_cube_pose;
    steering_cube_pose.position = module_origin;
    tf2::Quaternion q_steer;
    q_steer.setRPY(0, 0, robot_frame_steering_angle);
    steering_cube_pose.orientation = tf2::toMsg(q_steer);
    geometry_msgs::msg::Vector3 steering_cube_scale;
    steering_cube_scale.x = MarkerVisualize::STEERING_CUBE_LENGTH;
    steering_cube_scale.y = MarkerVisualize::STEERING_CUBE_WIDTH;
    steering_cube_scale.z = MarkerVisualize::STEERING_CUBE_THICKNESS;
    marker_array.markers.push_back(
      create_cube_marker(
        MODULE_STEERING_CUBE_BASE_ID + i, stamp, "steering_directions",
        steering_cube_pose, steering_cube_scale, steering_cube_color));

    double perpendicular_to_steering_angle = robot_frame_steering_angle + M_PI / 2.0;
    line_start_points[i] = {module_origin.x, module_origin.y};
    double far_point_dist = 15.0;
    line_end_points[i] = {
      module_origin.x + far_point_dist * std::cos(perpendicular_to_steering_angle),
      module_origin.y + far_point_dist * std::sin(perpendicular_to_steering_angle)
    };

    double viz_line_thickness = LINE_WIDTH * 1.0;
    double viz_line_half_length = MarkerVisualize::STEERING_CUBE_WIDTH * 20.0;
    geometry_msgs::msg::Point line_p1_viz, line_p2_viz;
    line_p1_viz.x = module_origin.x - viz_line_half_length * std::cos(
      perpendicular_to_steering_angle);
    line_p1_viz.y = module_origin.y - viz_line_half_length * std::sin(
      perpendicular_to_steering_angle);
    line_p1_viz.z = module_origin.z + 0.01;
    line_p2_viz.x = module_origin.x + viz_line_half_length * std::cos(
      perpendicular_to_steering_angle);
    line_p2_viz.y = module_origin.y + viz_line_half_length * std::sin(
      perpendicular_to_steering_angle);
    line_p2_viz.z = module_origin.z + 0.01;
    std::vector<geometry_msgs::msg::Point> line_points_viz = {line_p1_viz, line_p2_viz};
    marker_array.markers.push_back(
      create_line_strip_marker(
        WHEEL_PERPENDICULAR_LINE_BASE_ID + i, stamp, "wheel_perpendicular_lines", line_points_viz,
        wheel_perpendicular_line_color, viz_line_thickness));
  }

  // --- 3. visualize target ICR point ---
  bool target_icr_valid = (std::abs(robot_wz) > 1e-2);
  if (target_icr_valid) {
    geometry_msgs::msg::Point target_icr_point;
    target_icr_point.x = -robot_vy / robot_wz;
    target_icr_point.y = robot_vx / robot_wz;
    target_icr_point.z = 0.0;
    std_msgs::msg::ColorRGBA target_icr_color = text_color_blue;
    marker_array.markers.push_back(
      create_sphere_marker(
        TARGET_ICR_POINT_ID, stamp, "icr_visualization", target_icr_point,
        MarkerVisualize::SPHERE_RADIUS * 0.9, text_color_blue));

    std_msgs::msg::ColorRGBA line_to_icr_color;
    line_to_icr_color.r = 0.7; line_to_icr_color.g = 0.7; line_to_icr_color.b = 0.0;
    line_to_icr_color.a = 0.3;
    for (size_t i = 0; i < num_modules_; ++i) {
      geometry_msgs::msg::Point module_origin;
      module_origin.x = module_x_offsets[i];
      module_origin.y = module_y_offsets[i];
      module_origin.z = 0.0;
      std::vector<geometry_msgs::msg::Point> points = {module_origin, target_icr_point};

      // if length between module_origin and target_icr_point is 5m, continue
      double icr_to_robot_dist = std::hypot(
        module_origin.x - target_icr_point.x,
        module_origin.y - target_icr_point.y);
      if (icr_to_robot_dist > 5.0) {
        continue;
      }
      marker_array.markers.push_back(
        create_line_strip_marker(
          MODULE_LINE_TO_ICR_BASE_ID + i, stamp, "icr_to_robot_visualization_lines", points,
          target_icr_color));
    }

    geometry_msgs::msg::Point target_icr_text_pos = target_icr_point;
    target_icr_text_pos.x += -0.1;
    target_icr_text_pos.z += MarkerVisualize::SPHERE_RADIUS + 0.09;
    std::ostringstream target_icr_ss;
    target_icr_ss << std::fixed << std::setprecision(1) << "ICR(" << target_icr_point.x << "," <<
      target_icr_point.y << ")";
    marker_array.markers.push_back(
      create_text_marker(
        TARGET_ICR_TEXT_ID, stamp, "icr_visualization_text", target_icr_ss.str(),
        target_icr_text_pos, text_color_blue, MarkerVisualize::ICR_TEXT_SCALE * 0.9));
  } else {
    add_delete_marker_action(marker_array, TARGET_ICR_POINT_ID, "icr_visualization");
    add_delete_marker_action(marker_array, TARGET_ICR_TEXT_ID, "icr_visualization_text");
    for (size_t i = 0; i < num_modules_; ++i) {
      add_delete_marker_action(
        marker_array, MODULE_LINE_TO_ICR_BASE_ID + i,
        "icr_to_robot_visualization_lines");
    }
  }

  // --- 4. visualize actual ICR points based on steering angles ---
  std::vector<Point2D> intersection_points_2d;
  if (num_modules_ >= 2) {
    for (size_t i = 0; i < num_modules_; ++i) {
      for (size_t j = i + 1; j < num_modules_; ++j) {
        std::optional<Point2D> intersection = get_line_intersection(
          line_start_points[i], line_end_points[i],
          line_start_points[j], line_end_points[j]
        );
        if (intersection) {
          intersection_points_2d.push_back(intersection.value());
        }
      }
    }
  }

  std_msgs::msg::ColorRGBA intersection_color;
  intersection_color.r = 0.0f; intersection_color.g = 1.0f; intersection_color.b = 1.0f;
  intersection_color.a = 0.8f;
  double intersection_radius = 0.045;

  int max_possible_intersections = 0;
  if (num_modules_ >= 2) {max_possible_intersections = num_modules_ * (num_modules_ - 1) / 2;}
  for (int k = 0; k < max_possible_intersections; ++k) {
    add_delete_marker_action(
      marker_array, INTERSECTION_POINT_BASE_ID + k,
      "actual_icr_intersections");
  }
  add_delete_marker_action(marker_array, AVG_INTERSECTION_POINT_ID, "actual_icr_average");
  add_delete_marker_action(marker_array, AVG_INTERSECTION_TEXT_ID, "actual_icr_average_text");

  Point2D avg_intersection = {0.0, 0.0};
  if (!intersection_points_2d.empty()) {
    for (size_t k = 0; k < intersection_points_2d.size(); ++k) {
      const auto & pt = intersection_points_2d[k];
      geometry_msgs::msg::Point p_marker;
      p_marker.x = pt.x; p_marker.y = pt.y; p_marker.z = 0.0;
      marker_array.markers.push_back(
        create_sphere_marker(
          INTERSECTION_POINT_BASE_ID + k,
          stamp, "actual_icr_intersections", p_marker, intersection_radius, intersection_color));
      avg_intersection.x += pt.x;
      avg_intersection.y += pt.y;
    }
    avg_intersection.x /= intersection_points_2d.size();
    avg_intersection.y /= intersection_points_2d.size();

    geometry_msgs::msg::Point avg_icr_marker_pos;
    avg_icr_marker_pos.x = avg_intersection.x;
    avg_icr_marker_pos.y = avg_intersection.y;
    avg_icr_marker_pos.z = 0.02;
    std_msgs::msg::ColorRGBA avg_icr_color = intersection_color;
    marker_array.markers.push_back(
      create_sphere_marker(
        AVG_INTERSECTION_POINT_ID, stamp, "actual_icr_average", avg_icr_marker_pos,
        MarkerVisualize::SPHERE_RADIUS * 1.3, avg_icr_color));

    geometry_msgs::msg::Point avg_icr_text_pos = avg_icr_marker_pos;
    avg_icr_text_pos.z += MarkerVisualize::SPHERE_RADIUS * 1.3 + 0.06;
    avg_icr_text_pos.x += 0.1;
    std::ostringstream avg_icr_ss;
    avg_icr_ss << std::fixed << std::setprecision(2) << "Intersection(" << avg_intersection.x <<
      "," << avg_intersection.y << ")";
  }

  if (marker_publisher_) {
    marker_publisher_->publish(marker_array);
  }
}

// create marker functions
visualization_msgs::msg::Marker MarkerVisualize::create_arrow_marker(
  int id, const rclcpp::Time & stamp, const std::string & ns,
  const geometry_msgs::msg::Point & start_point,
  const geometry_msgs::msg::Point & end_point,
  const std_msgs::msg::ColorRGBA & color,
  double scale_x, double scale_y, double scale_z)
{
  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id = base_frame_id_;
  arrow.header.stamp = stamp;
  arrow.ns = ns;
  arrow.id = id;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = visualization_msgs::msg::Marker::ADD;
  arrow.points.resize(2);
  arrow.points[0] = start_point;
  arrow.points[1] = end_point;
  arrow.scale.x = scale_x;
  arrow.scale.y = scale_y;
  if (start_point.x == end_point.x && start_point.y == end_point.y &&
    start_point.z == end_point.z)
  {
    arrow.scale.z = 0.0001;
  } else {
    arrow.scale.z = scale_z;
  }
  arrow.color = color;
  arrow.lifetime = marker_lifetime_;
  return arrow;
}

visualization_msgs::msg::Marker MarkerVisualize::create_line_strip_marker(
  int id, const rclcpp::Time & stamp, const std::string & ns,
  const std::vector<geometry_msgs::msg::Point> & points,
  const std_msgs::msg::ColorRGBA & color,
  double scale_x)
{
  visualization_msgs::msg::Marker line_strip;
  line_strip.header.frame_id = base_frame_id_; line_strip.header.stamp = stamp; line_strip.ns = ns;
  line_strip.id = id;
  line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strip.action = visualization_msgs::msg::Marker::ADD;
  line_strip.pose.orientation.w = 1.0; line_strip.points = points; line_strip.scale.x = scale_x;
  line_strip.color = color; line_strip.lifetime = marker_lifetime_;
  return line_strip;
}

visualization_msgs::msg::Marker MarkerVisualize::create_sphere_marker(
  int id, const rclcpp::Time & stamp, const std::string & ns,
  const geometry_msgs::msg::Point & position,
  double radius,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::Marker sphere;
  sphere.header.frame_id = base_frame_id_; sphere.header.stamp = stamp; sphere.ns = ns;
  sphere.id = id;
  sphere.type = visualization_msgs::msg::Marker::SPHERE;
  sphere.action = visualization_msgs::msg::Marker::ADD;
  sphere.pose.position = position; sphere.pose.orientation.w = 1.0;
  sphere.scale.x = radius * 2.0; sphere.scale.y = radius * 2.0; sphere.scale.z = radius * 2.0;
  sphere.color = color; sphere.lifetime = marker_lifetime_;
  return sphere;
}

visualization_msgs::msg::Marker MarkerVisualize::create_text_marker(
  int id, const rclcpp::Time & stamp, const std::string & ns,
  const std::string & text,
  const geometry_msgs::msg::Point & position,
  const std_msgs::msg::ColorRGBA & color,
  double scale_z)
{
  visualization_msgs::msg::Marker text_marker;
  text_marker.header.frame_id = base_frame_id_; text_marker.header.stamp = stamp;
  text_marker.ns = ns; text_marker.id = id;
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::msg::Marker::ADD;
  text_marker.pose.position = position; text_marker.pose.orientation.w = 1.0;
  text_marker.text = text; text_marker.scale.z = scale_z;
  text_marker.color = color; text_marker.lifetime = marker_lifetime_;
  return text_marker;
}

visualization_msgs::msg::Marker MarkerVisualize::create_cube_marker(
  int id, const rclcpp::Time & stamp, const std::string & ns,
  const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::Marker cube;
  cube.header.frame_id = base_frame_id_; cube.header.stamp = stamp; cube.ns = ns; cube.id = id;
  cube.type = visualization_msgs::msg::Marker::CUBE;
  cube.action = visualization_msgs::msg::Marker::ADD;
  cube.pose = pose; cube.scale = scale;
  cube.color = color; cube.lifetime = marker_lifetime_;
  return cube;
}

}  // namespace ffw_swerve_drive_controller
