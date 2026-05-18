// Copyright 2025 ROBOTIS CO., LTD.
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
//
// Author: Woojin Wie

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "urdf/model.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace ffw_head_eef_tracker
{

using Vec3 = std::array<double, 3>;

class HeadEefTracker : public rclcpp::Node
{
public:
  HeadEefTracker()
  : rclcpp::Node("head_eef_tracker"),
    urdf_loaded_(false),
    debug_counter_(0),
    debug_log_interval_(10),
    enable_debug_logging_(false)
  {
    // Declare parameters
    update_rate_ = declare_parameter<double>("update_rate", 100.0);
    target_frame_ = declare_parameter<std::string>("target_frame", "arm_base_link");
    eef_l_link_ = declare_parameter<std::string>("eef_l_link", "end_effector_l_link");
    eef_r_link_ = declare_parameter<std::string>("eef_r_link", "end_effector_r_link");
    camera_link_ = declare_parameter<std::string>("camera_link", "zedm_camera_link");
    head_joint1_name_ = declare_parameter<std::string>("head_joint1_name", "head_joint1");
    head_joint2_name_ = declare_parameter<std::string>("head_joint2_name", "head_joint2");
    joint_trajectory_topic_ = declare_parameter<std::string>(
      "joint_trajectory_topic",
      "/leader/joystick_controller_left/joint_trajectory");
    joint_states_topic_ = declare_parameter<std::string>(
      "joint_states_topic", "/robot/head_leader/joint_states");
    robot_description_topic_ = declare_parameter<std::string>(
      "robot_description_topic", "/robot_description");
    visualization_topic_ = declare_parameter<std::string>(
      "visualization_topic", "~/head_target_visualization");
    enable_visualization_ = declare_parameter<bool>("enable_visualization", true);

    head_joint1_pos_ = {{0.0, 0.0, 0.0}};
    head_joint1_limit_lower_ = 0.0;
    head_joint1_limit_upper_ = 0.0;
    head_joint2_limit_lower_ = 0.0;
    head_joint2_limit_upper_ = 0.0;
    head_joint1_axis_ = {{0.0, 0.0, 0.0}};
    head_joint2_axis_ = {{0.0, 0.0, 0.0}};
    head_joint1_axis_valid_ = false;
    head_joint2_axis_valid_ = false;

    // TF2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriber for robot_description (TRANSIENT_LOCAL + RELIABLE, depth 1)
    auto qos_profile = rclcpp::QoS(1).transient_local().reliable();
    robot_description_sub_ = create_subscription<std_msgs::msg::String>(
      robot_description_topic_, qos_profile,
      std::bind(&HeadEefTracker::robot_description_callback, this, std::placeholders::_1));

    // Publishers
    joint_trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      joint_trajectory_topic_, 10);
    joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_states_topic_, 10);

    if (enable_visualization_) {
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        visualization_topic_, 10);
    }

    joint_names_ = {head_joint1_name_, head_joint2_name_};

    RCLCPP_INFO(get_logger(), "Head EEF Tracker initialized");
    RCLCPP_INFO(get_logger(), "  Update rate: %.2f Hz", update_rate_);
    RCLCPP_INFO(get_logger(), "  Target frame: %s", target_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  EEF L link: %s", eef_l_link_.c_str());
    RCLCPP_INFO(get_logger(), "  EEF R link: %s", eef_r_link_.c_str());
    RCLCPP_INFO(get_logger(), "  Camera link: %s", camera_link_.c_str());
    RCLCPP_INFO(get_logger(), "  Publishing to: %s", joint_trajectory_topic_.c_str());
    if (enable_visualization_) {
      RCLCPP_INFO(get_logger(), "  Visualization: %s", visualization_topic_.c_str());
    }
    RCLCPP_INFO(get_logger(),
      "  Waiting for robot_description on: %s", robot_description_topic_.c_str());
  }

private:
  void robot_description_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (urdf_loaded_) {
      return;
    }

    try {
      urdf::Model model;
      if (!model.initString(msg->data)) {
        RCLCPP_WARN(get_logger(), "Failed to parse URDF, will retry on next message");
        return;
      }

      parse_urdf(model);

      if (urdf_loaded_) {
        RCLCPP_INFO(get_logger(), "============================================================");
        RCLCPP_INFO(get_logger(), "URDF loaded successfully");
        RCLCPP_INFO(get_logger(), "  head_joint1 (%s) - PITCH:", head_joint1_name_.c_str());
        RCLCPP_INFO(get_logger(),
          "    Position (xyz): (%.4f, %.4f, %.4f)",
          head_joint1_pos_[0], head_joint1_pos_[1], head_joint1_pos_[2]);
        if (head_joint1_axis_valid_) {
          RCLCPP_INFO(get_logger(),
            "    Axis: (%.4f, %.4f, %.4f)",
            head_joint1_axis_[0], head_joint1_axis_[1], head_joint1_axis_[2]);
        } else {
          RCLCPP_INFO(get_logger(), "    Axis: (none)");
        }
        RCLCPP_INFO(get_logger(),
          "    Limits: [%.4f, %.4f]", head_joint1_limit_lower_, head_joint1_limit_upper_);
        RCLCPP_INFO(get_logger(), "  head_joint2 (%s) - YAW:", head_joint2_name_.c_str());
        if (head_joint2_axis_valid_) {
          RCLCPP_INFO(get_logger(),
            "    Axis: (%.4f, %.4f, %.4f)",
            head_joint2_axis_[0], head_joint2_axis_[1], head_joint2_axis_[2]);
        } else {
          RCLCPP_INFO(get_logger(), "    Axis: (none)");
        }
        RCLCPP_INFO(get_logger(),
          "    Limits: [%.4f, %.4f]", head_joint2_limit_lower_, head_joint2_limit_upper_);
        RCLCPP_INFO(get_logger(), "============================================================");

        if (!timer_) {
          auto period = std::chrono::duration<double>(1.0 / update_rate_);
          timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&HeadEefTracker::timer_callback, this));
        }
      } else {
        RCLCPP_WARN(get_logger(), "Failed to parse URDF, will retry on next message");
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Error parsing robot_description: %s", e.what());
    }
  }

  void parse_urdf(const urdf::Model & model)
  {
    auto head_joint1 = model.getJoint(head_joint1_name_);
    auto head_joint2 = model.getJoint(head_joint2_name_);

    if (!head_joint1 || !head_joint2) {
      RCLCPP_ERROR(get_logger(),
        "Could not find joints: %s, %s",
        head_joint1_name_.c_str(), head_joint2_name_.c_str());
      return;
    }

    // Extract head_joint1 origin position
    head_joint1_pos_ = {{
      head_joint1->parent_to_joint_origin_transform.position.x,
      head_joint1->parent_to_joint_origin_transform.position.y,
      head_joint1->parent_to_joint_origin_transform.position.z
    }};

    // Extract joint limits
    if (head_joint1->limits) {
      head_joint1_limit_lower_ = head_joint1->limits->lower;
      head_joint1_limit_upper_ = head_joint1->limits->upper;
    } else {
      RCLCPP_WARN(get_logger(), "No limits found for %s", head_joint1_name_.c_str());
      return;
    }

    if (head_joint2->limits) {
      head_joint2_limit_lower_ = head_joint2->limits->lower;
      head_joint2_limit_upper_ = head_joint2->limits->upper;
    } else {
      RCLCPP_WARN(get_logger(), "No limits found for %s", head_joint2_name_.c_str());
      return;
    }

    // Joint axes (urdf::Joint::axis defaults to (1,0,0); treat all-zero as invalid)
    head_joint1_axis_ = {{head_joint1->axis.x, head_joint1->axis.y, head_joint1->axis.z}};
    head_joint1_axis_valid_ =
      !(head_joint1_axis_[0] == 0.0 && head_joint1_axis_[1] == 0.0 &&
      head_joint1_axis_[2] == 0.0);
    head_joint2_axis_ = {{head_joint2->axis.x, head_joint2->axis.y, head_joint2->axis.z}};
    head_joint2_axis_valid_ =
      !(head_joint2_axis_[0] == 0.0 && head_joint2_axis_[1] == 0.0 &&
      head_joint2_axis_[2] == 0.0);

    urdf_loaded_ = true;
  }

  bool get_transform(
    const std::string & target_frame, const std::string & source_frame,
    geometry_msgs::msg::TransformStamped & out)
  {
    try {
      out = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(),
        "Could not transform %s to %s: %s",
        source_frame.c_str(), target_frame.c_str(), ex.what());
      return false;
    }
  }

  std::pair<double, double> calculate_head_angles(
    const Vec3 & target_point, const Vec3 & head_joint1_pos, bool debug)
  {
    const double dx = target_point[0] - head_joint1_pos[0];
    const double dy = target_point[1] - head_joint1_pos[1];
    const double dz = target_point[2] - head_joint1_pos[2];

    if (debug) {
      RCLCPP_INFO(get_logger(), "  Vector calculation:");
      RCLCPP_INFO(get_logger(),
        "    Target: (%.4f, %.4f, %.4f)",
        target_point[0], target_point[1], target_point[2]);
      RCLCPP_INFO(get_logger(),
        "    head_joint1_pos: (%.4f, %.4f, %.4f)",
        head_joint1_pos[0], head_joint1_pos[1], head_joint1_pos[2]);
      RCLCPP_INFO(get_logger(),
        "    Vector (dx, dy, dz): (%.4f, %.4f, %.4f)", dx, dy, dz);
    }

    const double r_xy = std::sqrt(dx * dx + dy * dy);

    if (debug) {
      RCLCPP_INFO(get_logger(), "    Distance in XY plane (r_xy): %.4f", r_xy);
    }

    // Calculate yaw (head_joint2 around Z axis)
    double yaw;
    if (std::abs(dy) < 1e-4) {
      yaw = 0.0;
      if (debug) {
        RCLCPP_INFO(get_logger(),
          "    dy~0 (%.6f), setting yaw to 0 (centered)", dy);
      }
    } else if (r_xy > 1e-6) {
      yaw = std::atan2(dy, dx);
    } else {
      yaw = 0.0;
    }

    if (debug) {
      RCLCPP_INFO(get_logger(),
        "    Raw yaw (head_joint2): %.2f deg (%.4f rad)",
        yaw * 180.0 / M_PI, yaw);
    }

    // Calculate pitch (head_joint1 around Y axis), inverted to match joint convention
    double pitch_raw;
    double pitch;
    if (r_xy > 1e-6) {
      pitch_raw = std::atan2(dz, r_xy);
      pitch = -pitch_raw;
    } else {
      if (std::abs(dz) > 1e-6) {
        pitch_raw = std::copysign(M_PI / 2.0, dz);
        pitch = -pitch_raw;
      } else {
        pitch_raw = 0.0;
        pitch = 0.0;
      }
    }

    if (debug) {
      RCLCPP_INFO(get_logger(),
        "    Raw pitch (head_joint1): %.2f deg (%.4f rad)",
        pitch_raw * 180.0 / M_PI, pitch_raw);
      RCLCPP_INFO(get_logger(),
        "    Inverted pitch: %.2f deg (%.4f rad)",
        pitch * 180.0 / M_PI, pitch);
    }

    const double head_joint1_angle = std::max(
      head_joint1_limit_lower_, std::min(head_joint1_limit_upper_, pitch));
    const double head_joint2_angle = std::max(
      head_joint2_limit_lower_, std::min(head_joint2_limit_upper_, yaw));

    if (debug) {
      RCLCPP_INFO(get_logger(), "  After clamping:");
      RCLCPP_INFO(get_logger(),
        "    head_joint1 (pitch): %.2f deg (%.4f rad)",
        head_joint1_angle * 180.0 / M_PI, head_joint1_angle);
      RCLCPP_INFO(get_logger(),
        "    head_joint2 (yaw): %.2f deg (%.4f rad)",
        head_joint2_angle * 180.0 / M_PI, head_joint2_angle);
      if (head_joint1_angle != pitch) {
        RCLCPP_WARN(get_logger(),
          "    WARNING: head_joint1 was clamped! Raw: %.4f, Clamped: %.4f",
          pitch, head_joint1_angle);
      }
      if (head_joint2_angle != yaw) {
        RCLCPP_WARN(get_logger(),
          "    WARNING: head_joint2 was clamped! Raw: %.4f, Clamped: %.4f",
          yaw, head_joint2_angle);
      }
    }

    return {head_joint1_angle, head_joint2_angle};
  }

  void create_visualization_markers(
    const Vec3 & head_joint1_pos, const Vec3 & target_point,
    const Vec3 & pos_l, const Vec3 & pos_r)
  {
    if (!enable_visualization_ || !marker_pub_) {
      return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    const auto now = this->now();

    // Marker 1: Arrow from head_joint1 to target
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = target_frame_;
      marker.header.stamp = now;
      marker.ns = "head_target";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point start_point;
      start_point.x = head_joint1_pos[0];
      start_point.y = head_joint1_pos[1];
      start_point.z = head_joint1_pos[2];

      geometry_msgs::msg::Point end_point;
      end_point.x = target_point[0];
      end_point.y = target_point[1];
      end_point.z = target_point[2];

      marker.points = {start_point, end_point};

      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;

      marker.scale.x = 0.02;
      marker.scale.y = 0.04;
      marker.scale.z = 0.05;

      marker.lifetime.sec = 1;
      marker_array.markers.push_back(marker);
    }

    // Marker 2: Sphere at target point
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = target_frame_;
      marker.header.stamp = now;
      marker.ns = "head_target";
      marker.id = 1;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = target_point[0];
      marker.pose.position.y = target_point[1];
      marker.pose.position.z = target_point[2];
      marker.pose.orientation.w = 1.0;

      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.8f;

      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;

      marker.lifetime.sec = 1;
      marker_array.markers.push_back(marker);
    }

    // Marker 3: Sphere at left end effector
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = target_frame_;
      marker.header.stamp = now;
      marker.ns = "eef_positions";
      marker.id = 2;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = pos_l[0];
      marker.pose.position.y = pos_l[1];
      marker.pose.position.z = pos_l[2];
      marker.pose.orientation.w = 1.0;

      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.6f;

      marker.scale.x = 0.03;
      marker.scale.y = 0.03;
      marker.scale.z = 0.03;

      marker.lifetime.sec = 1;
      marker_array.markers.push_back(marker);
    }

    // Marker 4: Sphere at right end effector
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = target_frame_;
      marker.header.stamp = now;
      marker.ns = "eef_positions";
      marker.id = 3;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = pos_r[0];
      marker.pose.position.y = pos_r[1];
      marker.pose.position.z = pos_r[2];
      marker.pose.orientation.w = 1.0;

      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 0.6f;

      marker.scale.x = 0.03;
      marker.scale.y = 0.03;
      marker.scale.z = 0.03;

      marker.lifetime.sec = 1;
      marker_array.markers.push_back(marker);
    }

    marker_pub_->publish(marker_array);
  }

  void timer_callback()
  {
    if (!urdf_loaded_) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform_l;
    geometry_msgs::msg::TransformStamped transform_r;
    const bool ok_l = get_transform(target_frame_, eef_l_link_, transform_l);
    const bool ok_r = get_transform(target_frame_, eef_r_link_, transform_r);

    if (!ok_l || !ok_r) {
      if (!enable_debug_logging_) {
        return;
      }
      debug_counter_++;
      if (debug_counter_ % debug_log_interval_ == 0) {
        RCLCPP_WARN(get_logger(),
          "[Update %d] Failed to get transforms for end effectors", debug_counter_);
      }
      return;
    }

    const Vec3 pos_l = {{
      transform_l.transform.translation.x,
      transform_l.transform.translation.y,
      transform_l.transform.translation.z
    }};
    const Vec3 pos_r = {{
      transform_r.transform.translation.x,
      transform_r.transform.translation.y,
      transform_r.transform.translation.z
    }};

    const Vec3 center_point = {{
      (pos_l[0] + pos_r[0]) / 2.0,
      (pos_l[1] + pos_r[1]) / 2.0,
      (pos_l[2] + pos_r[2]) / 2.0
    }};

    debug_counter_++;
    const bool should_debug =
      (debug_counter_ % debug_log_interval_ == 0) && enable_debug_logging_;

    if (should_debug) {
      RCLCPP_INFO(get_logger(), "------------------------------------------------------------");
      RCLCPP_INFO(get_logger(), "[Update %d] Head EEF Tracking:", debug_counter_);
      RCLCPP_INFO(get_logger(),
        "  End Effector Positions (in %s):", target_frame_.c_str());
      RCLCPP_INFO(get_logger(),
        "    %s: (%.4f, %.4f, %.4f)",
        eef_l_link_.c_str(), pos_l[0], pos_l[1], pos_l[2]);
      RCLCPP_INFO(get_logger(),
        "    %s: (%.4f, %.4f, %.4f)",
        eef_r_link_.c_str(), pos_r[0], pos_r[1], pos_r[2]);
      RCLCPP_INFO(get_logger(),
        "  Center Point: (%.4f, %.4f, %.4f)",
        center_point[0], center_point[1], center_point[2]);
    }

    const auto [head_joint1_angle, head_joint2_angle] = calculate_head_angles(
      center_point, head_joint1_pos_, should_debug);

    // Publish joint trajectory
    trajectory_msgs::msg::JointTrajectory trajectory_msg;
    trajectory_msg.header.frame_id = "";
    trajectory_msg.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {head_joint1_angle, head_joint2_angle};
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 0;

    trajectory_msg.points = {point};
    joint_trajectory_pub_->publish(trajectory_msg);

    // Publish joint state
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name = joint_names_;
    joint_state_msg.position = {head_joint1_angle, head_joint2_angle};
    joint_states_pub_->publish(joint_state_msg);

    create_visualization_markers(head_joint1_pos_, center_point, pos_l, pos_r);

    if (should_debug) {
      RCLCPP_INFO(get_logger(), "  Published Joint Commands:");
      RCLCPP_INFO(get_logger(),
        "    %s (PITCH): %.2f deg (%.4f rad)",
        head_joint1_name_.c_str(),
        head_joint1_angle * 180.0 / M_PI, head_joint1_angle);
      RCLCPP_INFO(get_logger(),
        "    %s (YAW): %.2f deg (%.4f rad)",
        head_joint2_name_.c_str(),
        head_joint2_angle * 180.0 / M_PI, head_joint2_angle);
      RCLCPP_INFO(get_logger(), "------------------------------------------------------------");
    }
  }

  // Parameters
  double update_rate_;
  std::string target_frame_;
  std::string eef_l_link_;
  std::string eef_r_link_;
  std::string camera_link_;
  std::string head_joint1_name_;
  std::string head_joint2_name_;
  std::string joint_trajectory_topic_;
  std::string joint_states_topic_;
  std::string robot_description_topic_;
  std::string visualization_topic_;
  bool enable_visualization_;

  // URDF data
  bool urdf_loaded_;
  Vec3 head_joint1_pos_;
  double head_joint1_limit_lower_;
  double head_joint1_limit_upper_;
  double head_joint2_limit_lower_;
  double head_joint2_limit_upper_;
  Vec3 head_joint1_axis_;
  Vec3 head_joint2_axis_;
  bool head_joint1_axis_valid_;
  bool head_joint2_axis_valid_;

  // Debug
  int debug_counter_;
  int debug_log_interval_;
  bool enable_debug_logging_;

  // Joint names
  std::vector<std::string> joint_names_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ffw_head_eef_tracker

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ffw_head_eef_tracker::HeadEefTracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
