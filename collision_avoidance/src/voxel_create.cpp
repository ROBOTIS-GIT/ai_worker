// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace collision_avoidance
{

constexpr uint8_t SRC_ZED      = 0x01;
constexpr uint8_t SRC_RS_LEFT  = 0x02;
constexpr uint8_t SRC_RS_RIGHT = 0x04;

struct VoxelKey
{
  int32_t x;
  int32_t y;
  int32_t z;
  bool operator==(const VoxelKey & o) const noexcept
  {
    return x == o.x && y == o.y && z == o.z;
  }
};

struct VoxelKeyHash
{
  size_t operator()(const VoxelKey & k) const noexcept
  {
    // Standard spatial hash
    return static_cast<size_t>(
      static_cast<int64_t>(k.x) * 73856093 ^
      static_cast<int64_t>(k.y) * 19349663 ^
      static_cast<int64_t>(k.z) * 83492791);
  }
};

class VoxelCreate : public rclcpp::Node
{
public:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<Image, CameraInfo>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  VoxelCreate()
  : rclcpp::Node("voxel_create"),
    env_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    zed_pc_topic_ = declare_parameter<std::string>(
      "zed_pointcloud_topic", "/zedm/zed_node/point_cloud/cloud_registered");

    rs_left_depth_topic_ = declare_parameter<std::string>(
      "rs_left_depth_topic", "/camera_left/camera_l/depth/image_rect_raw");
    rs_left_info_topic_ = declare_parameter<std::string>(
      "rs_left_info_topic", "/camera_left/camera_l/depth/camera_info");

    rs_right_depth_topic_ = declare_parameter<std::string>(
      "rs_right_depth_topic", "/camera_right/camera_r/depth/image_rect_raw");
    rs_right_info_topic_ = declare_parameter<std::string>(
      "rs_right_info_topic", "/camera_right/camera_r/depth/camera_info");

    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    output_topic_ = declare_parameter<std::string>("output_topic", "~/voxel_map");
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.1);

    voxel_leaf_ = declare_parameter<double>("voxel_leaf_size", 0.02);
    accumulate_frames_ = declare_parameter<int>("accumulate_frames", 45);
    depth_stride_ = declare_parameter<int>("depth_stride", 2);
    depth_min_m_ = declare_parameter<double>("depth_min_m", 0.1);
    depth_max_m_ = declare_parameter<double>("depth_max_m", 3.0);
    min_cameras_ = declare_parameter<int>("min_cameras", 2);

    auto map_qos = rclcpp::QoS(1).transient_local().reliable();
    voxel_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, map_qos);
    rs_left_debug_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/rs_left_cloud", map_qos);
    rs_right_debug_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/rs_right_cloud", map_qos);

    zed_pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      zed_pc_topic_, rclcpp::QoS(1).best_effort(),
      std::bind(&VoxelCreate::on_zed_pc, this, std::placeholders::_1));

    rs_left_depth_sub_.subscribe(this, rs_left_depth_topic_, rmw_qos_profile_default);
    rs_left_info_sub_.subscribe(this, rs_left_info_topic_, rmw_qos_profile_default);
    rs_left_sync_ = std::make_shared<Sync>(
      SyncPolicy(10), rs_left_depth_sub_, rs_left_info_sub_);
    rs_left_sync_->registerCallback(
      std::bind(&VoxelCreate::on_rs_depth, this,
        std::placeholders::_1, std::placeholders::_2, std::string("rs_left")));

    rs_right_depth_sub_.subscribe(this, rs_right_depth_topic_, rmw_qos_profile_default);
    rs_right_info_sub_.subscribe(this, rs_right_info_topic_, rmw_qos_profile_default);
    rs_right_sync_ = std::make_shared<Sync>(
      SyncPolicy(10), rs_right_depth_sub_, rs_right_info_sub_);
    rs_right_sync_->registerCallback(
      std::bind(&VoxelCreate::on_rs_depth, this,
        std::placeholders::_1, std::placeholders::_2, std::string("rs_right")));

    RCLCPP_INFO(get_logger(),
      "VoxelCreate (intersection mode, min_cameras=%d) started. accumulate=%d frames",
      min_cameras_, accumulate_frames_);
  }

private:
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

  bool lookup_transform(const std::string & source_frame, Eigen::Isometry3d & out)
  {
    try {
      auto tf_msg = tf_buffer_.lookupTransform(
        base_frame_, source_frame, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
      out = tf2::transformToEigen(tf_msg);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TF '%s' -> '%s' failed: %s",
        source_frame.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
  }

  void stamp_voxels(
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    uint8_t source_bit)
  {
    const float inv_leaf = 1.0f / static_cast<float>(voxel_leaf_);
    for (const auto & p : cloud.points) {
      const VoxelKey key{
        static_cast<int32_t>(std::floor(p.x * inv_leaf)),
        static_cast<int32_t>(std::floor(p.y * inv_leaf)),
        static_cast<int32_t>(std::floor(p.z * inv_leaf))
      };
      voxel_sources_[key] |= source_bit;
    }
  }

  void on_zed_pc(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (done_) {
      return;
    }

    Eigen::Isometry3d transform;
    if (!lookup_transform(msg->header.frame_id, transform)) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
    pc2_to_pcl(*msg, *raw);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*raw, *transformed, transform.cast<float>().matrix());

    stamp_voxels(*transformed, SRC_ZED);
    add_frame("zed", transformed->size());
  }

  void on_rs_depth(
    const Image::ConstSharedPtr & depth,
    const CameraInfo::ConstSharedPtr & info,
    std::string source)
  {
    if (done_) {
      return;
    }

    Eigen::Isometry3d transform;
    if (!lookup_transform(depth->header.frame_id, transform)) {
      return;
    }

    const float fx = static_cast<float>(info->k[0]);
    const float fy = static_cast<float>(info->k[4]);
    const float cx = static_cast<float>(info->k[2]);
    const float cy = static_cast<float>(info->k[5]);

    const int W = static_cast<int>(depth->width);
    const int H = static_cast<int>(depth->height);
    const int stride = std::max(1, depth_stride_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(static_cast<size_t>((W / stride) * (H / stride)));

    const float Zmin = static_cast<float>(depth_min_m_);
    const float Zmax = static_cast<float>(depth_max_m_);

    if (depth->encoding == "16UC1") {
      const uint16_t * data = reinterpret_cast<const uint16_t *>(depth->data.data());
      for (int v = 0; v < H; v += stride) {
        const int row = v * W;
        for (int u = 0; u < W; u += stride) {
          const uint16_t mm = data[row + u];
          if (mm == 0) {
            continue;
          }
          const float Z = mm * 0.001f;
          if (Z < Zmin || Z > Zmax) {
            continue;
          }
          const float X = (u - cx) * Z / fx;
          const float Y = (v - cy) * Z / fy;
          cloud->points.emplace_back(X, Y, Z);
        }
      }
    } else if (depth->encoding == "32FC1") {
      const float * data = reinterpret_cast<const float *>(depth->data.data());
      for (int v = 0; v < H; v += stride) {
        const int row = v * W;
        for (int u = 0; u < W; u += stride) {
          const float Z = data[row + u];
          if (!std::isfinite(Z) || Z < Zmin || Z > Zmax) {
            continue;
          }
          const float X = (u - cx) * Z / fx;
          const float Y = (v - cy) * Z / fy;
          cloud->points.emplace_back(X, Y, Z);
        }
      }
    } else {
      RCLCPP_WARN_ONCE(get_logger(),
        "Unsupported depth encoding '%s' on %s",
        depth->encoding.c_str(), source.c_str());
      return;
    }

    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed, transform.cast<float>().matrix());

    if (source == "rs_left" && !rs_left_debug_published_) {
      publish_debug_cloud(*transformed, rs_left_debug_pub_);
      rs_left_debug_published_ = true;
      RCLCPP_INFO(get_logger(), "Published rs_left debug cloud (%zu pts)", transformed->size());
    } else if (source == "rs_right" && !rs_right_debug_published_) {
      publish_debug_cloud(*transformed, rs_right_debug_pub_);
      rs_right_debug_published_ = true;
      RCLCPP_INFO(get_logger(), "Published rs_right debug cloud (%zu pts)", transformed->size());
    }

    const uint8_t src_bit = (source == "rs_left") ? SRC_RS_LEFT : SRC_RS_RIGHT;
    stamp_voxels(*transformed, src_bit);
    add_frame(source, transformed->size());
  }

  void publish_debug_cloud(
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub)
  {
    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = now();
    out.header.frame_id = base_frame_;
    out.height = 1;
    out.width = static_cast<uint32_t>(cloud.size());
    out.is_dense = true;
    out.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(cloud.size());

    sensor_msgs::PointCloud2Iterator<float> it_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(out, "z");
    for (const auto & p : cloud.points) {
      *it_x = p.x; *it_y = p.y; *it_z = p.z;
      ++it_x; ++it_y; ++it_z;
    }

    pub->publish(out);
  }

  void add_frame(const std::string & source, size_t added)
  {
    ++frames_seen_;
    RCLCPP_INFO(get_logger(),
      "[%s] frame %d/%d (+%zu pts, voxels %zu)",
      source.c_str(), frames_seen_, accumulate_frames_,
      added, voxel_sources_.size());

    if (frames_seen_ < accumulate_frames_) {
      return;
    }

    finalize();
  }

  void finalize()
  {
    if (done_) {
      return;
    }

    const float leaf = static_cast<float>(voxel_leaf_);
    const float half = leaf * 0.5f;

    env_cloud_->clear();
    env_cloud_->points.reserve(voxel_sources_.size());

    size_t kept = 0, dropped = 0;
    for (const auto & kv : voxel_sources_) {
      const int bits = __builtin_popcount(kv.second);
      if (bits >= min_cameras_) {
        const VoxelKey & k = kv.first;
        env_cloud_->points.emplace_back(
          k.x * leaf + half,
          k.y * leaf + half,
          k.z * leaf + half);
        ++kept;
      } else {
        ++dropped;
      }
    }
    env_cloud_->width = static_cast<uint32_t>(env_cloud_->points.size());
    env_cloud_->height = 1;
    env_cloud_->is_dense = true;

    RCLCPP_INFO(get_logger(),
      "Intersection result: %zu kept, %zu dropped (threshold=%d of 3 cameras)",
      kept, dropped, min_cameras_);

    if (env_cloud_->empty()) {
      RCLCPP_WARN(get_logger(),
        "Intersection cloud empty. Lower min_cameras or check overlap. Resetting...");
      voxel_sources_.clear();
      frames_seen_ = 0;
      return;
    }

    publish_voxel_map();

    done_ = true;
    zed_pc_sub_.reset();
    rs_left_sync_.reset();
    rs_right_sync_.reset();
    rs_left_depth_sub_.unsubscribe();
    rs_left_info_sub_.unsubscribe();
    rs_right_depth_sub_.unsubscribe();
    rs_right_info_sub_.unsubscribe();
    voxel_sources_.clear();
    RCLCPP_INFO(get_logger(),
      "Voxel map captured and published (%zu pts). Subscriptions released.",
      env_cloud_->size());
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

    voxel_pub_->publish(out);
  }

  // Parameters
  std::string zed_pc_topic_;
  std::string rs_left_depth_topic_, rs_left_info_topic_;
  std::string rs_right_depth_topic_, rs_right_info_topic_;
  std::string base_frame_;
  std::string output_topic_;
  double tf_timeout_sec_;
  double voxel_leaf_;
  int accumulate_frames_;
  int depth_stride_;
  double depth_min_m_;
  double depth_max_m_;
  int min_cameras_;

  // State
  std::unordered_map<VoxelKey, uint8_t, VoxelKeyHash> voxel_sources_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr env_cloud_;
  int frames_seen_ = 0;
  bool done_ = false;
  bool rs_left_debug_published_ = false;
  bool rs_right_debug_published_ = false;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr zed_pc_sub_;
  message_filters::Subscriber<Image> rs_left_depth_sub_;
  message_filters::Subscriber<CameraInfo> rs_left_info_sub_;
  std::shared_ptr<Sync> rs_left_sync_;
  message_filters::Subscriber<Image> rs_right_depth_sub_;
  message_filters::Subscriber<CameraInfo> rs_right_info_sub_;
  std::shared_ptr<Sync> rs_right_sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rs_left_debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rs_right_debug_pub_;
};

}  // namespace collision_avoidance

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<collision_avoidance::VoxelCreate>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
