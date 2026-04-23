// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace collision_avoidance
{

class EnvironmentCollisionAvoidance : public rclcpp::Node
{
public:
  EnvironmentCollisionAvoidance()
  : rclcpp::Node("environment_collision_avoidance"),
    env_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Parameters
    pointcloud_topic_ = declare_parameter<std::string>(
      "pointcloud_topic", "/zedm/zed_node/point_cloud/cloud_registered");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    // left_eef_frame_ = declare_parameter<std::string>("left_eef_frame", "arm_l_link7");
    // right_eef_frame_ = declare_parameter<std::string>("right_eef_frame", "arm_r_link7");
    left_eef_frame_ = declare_parameter<std::string>("left_eef_frame", "end_effector_l_link");
    right_eef_frame_ = declare_parameter<std::string>("right_eef_frame", "end_effector_r_link");
    left_command_topic_ = declare_parameter<std::string>(
      "left_command_topic", "/leader/left_command");
    right_command_topic_ = declare_parameter<std::string>(
      "right_command_topic", "/leader/right_command");

    voxel_leaf_ = declare_parameter<double>("voxel_leaf_size", 0.02);
    stop_distance_ = declare_parameter<double>("stop_distance", 0.10);
    resume_distance_ = declare_parameter<double>("resume_distance", 0.15);
    check_rate_hz_ = declare_parameter<double>("check_rate_hz", 20.0);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.1);

    auto qos = rclcpp::SystemDefaultsQoS();
    left_pub_ = create_publisher<std_msgs::msg::UInt8>(left_command_topic_, qos);
    right_pub_ = create_publisher<std_msgs::msg::UInt8>(right_command_topic_, qos);

    // Latched voxel map for RViz (set display Durability to Transient Local)
    auto map_qos = rclcpp::QoS(1).transient_local().reliable();
    env_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/environment_voxel_map", map_qos);

    // Subscribe once to capture environment
    auto cloud_qos = rclcpp::QoS(1).best_effort();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, cloud_qos,
      std::bind(&EnvironmentCollisionAvoidance::on_cloud, this,
        std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "Waiting for pointcloud on '%s'...", pointcloud_topic_.c_str());

    // Block until environment captured (spin_some so the subscription callback fires)
    while (rclcpp::ok() && !env_ready_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Still waiting for pointcloud...");
      rclcpp::spin_some(get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // Free the subscription after capture
    cloud_sub_.reset();

    // Periodic collision check
    const auto period = std::chrono::duration<double>(1.0 / check_rate_hz_);
    check_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&EnvironmentCollisionAvoidance::check_collision, this));

    RCLCPP_INFO(get_logger(),
      "EnvironmentCollisionAvoidance started (%zu points, stop=%.2fm, resume=%.2fm)",
      env_cloud_->size(), stop_distance_, resume_distance_);
  }

private:
  // Manual PointCloud2 → pcl::PointCloud<pcl::PointXYZ>
  static void pc2_to_pcl(
    const sensor_msgs::msg::PointCloud2 & msg,
    pcl::PointCloud<pcl::PointXYZ> & out)
  {
    out.clear();
    out.points.reserve(static_cast<size_t>(msg.width) * msg.height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
        out.points.emplace_back(x, y, z);
      }
    }
    out.width = static_cast<uint32_t>(out.points.size());
    out.height = 1;
    out.is_dense = true;
  }

  void on_cloud(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (env_ready_) {
      return;
    }

    // Lookup transform from source to base_frame
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    try {
      auto tf_msg = tf_buffer_.lookupTransform(
        base_frame_, msg->header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
      transform = tf2::transformToEigen(tf_msg);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TF transform failed: %s", ex.what());
      return;
    }

    // Convert PointCloud2 → PCL (in source frame)
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
    pc2_to_pcl(*msg, *raw);

    // Transform into base_frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*raw, *transformed, transform.cast<float>().matrix());

    // Voxel downsample
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(transformed);
    const float leaf = static_cast<float>(voxel_leaf_);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*env_cloud_);

    if (env_cloud_->empty()) {
      RCLCPP_WARN(get_logger(), "Downsampled cloud is empty; retrying...");
      return;
    }

    kdtree_.setInputCloud(env_cloud_);
    env_ready_ = true;
    RCLCPP_INFO(get_logger(),
      "Captured environment: %zu points (after voxel filter)", env_cloud_->size());

    publish_voxel_map();
  }

  void publish_voxel_map()
  {
    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = now();
    out.header.frame_id = base_frame_;
    out.height = 1;
    out.width = static_cast<uint32_t>(env_cloud_->size());
    out.is_dense = true;
    out.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(env_cloud_->size());

    sensor_msgs::PointCloud2Iterator<float> it_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(out, "z");
    for (const auto & p : env_cloud_->points) {
      *it_x = p.x; *it_y = p.y; *it_z = p.z;
      ++it_x; ++it_y; ++it_z;
    }

    env_map_pub_->publish(out);
    RCLCPP_INFO_ONCE(get_logger(),
      "Published voxel map on '~/environment_voxel_map' (frame=%s, %u points)",
      base_frame_.c_str(), out.width);
  }

  bool query_distance(const std::string & eef_frame, float & dist_out)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
        base_frame_, eef_frame, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF lookup failed for '%s': %s", eef_frame.c_str(), ex.what());
      return false;
    }

    pcl::PointXYZ query;
    query.x = static_cast<float>(tf.transform.translation.x);
    query.y = static_cast<float>(tf.transform.translation.y);
    query.z = static_cast<float>(tf.transform.translation.z);

    std::vector<int> indices(1);
    std::vector<float> sq_dists(1);
    if (kdtree_.nearestKSearch(query, 1, indices, sq_dists) <= 0) {
      return false;
    }
    dist_out = std::sqrt(sq_dists[0]);
    return true;
  }

  void check_side(
    const std::string & eef_frame,
    const rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr & pub,
    bool & blocked_flag,
    const char * tag)
  {
    (void)pub;  // unused for now; will be used when publishing is re-enabled
    float dist = 0.0f;
    if (!query_distance(eef_frame, dist)) {
      return;
    }

    if (!blocked_flag && dist < stop_distance_) {
      // std_msgs::msg::UInt8 msg;
      // msg.data = 0;
      // pub->publish(msg);
      blocked_flag = true;
      RCLCPP_WARN(get_logger(),
        "[%s] EEF too close to environment (%.3f m < %.3f m). Disabling.",
        tag, dist, stop_distance_);
    } else if (blocked_flag && dist > resume_distance_) {
      // std_msgs::msg::UInt8 msg;
      // msg.data = 1;
      // pub->publish(msg);
      blocked_flag = false;
      RCLCPP_INFO(get_logger(),
        "[%s] EEF clear (%.3f m > %.3f m). Enabling.",
        tag, dist, resume_distance_);
    } else if (blocked_flag) {
      RCLCPP_WARN(get_logger(),
        "[%s] EEF still close: %.3f m", tag, dist);
    }
  }

  void check_collision()
  {
    check_side(left_eef_frame_, left_pub_, left_blocked_, "left");
    check_side(right_eef_frame_, right_pub_, right_blocked_, "right");
  }

  // Parameters
  std::string pointcloud_topic_;
  std::string base_frame_;
  std::string left_eef_frame_;
  std::string right_eef_frame_;
  std::string left_command_topic_;
  std::string right_command_topic_;
  double voxel_leaf_;
  double stop_distance_;
  double resume_distance_;
  double check_rate_hz_;
  double tf_timeout_sec_;

  // PCL state
  pcl::PointCloud<pcl::PointXYZ>::Ptr env_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
  bool env_ready_ = false;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr left_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr env_map_pub_;
  rclcpp::TimerBase::SharedPtr check_timer_;

  // Edge-triggered enable state (to avoid spamming the topic)
  bool left_blocked_ = false;
  bool right_blocked_ = false;
};

}  // namespace collision_avoidance

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<collision_avoidance::EnvironmentCollisionAvoidance>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
