#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <yolo_msgs/msg/detection.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

using moveit::planning_interface::MoveGroupInterface;
using std::placeholders::_1;
using std::placeholders::_2;
using Trigger = std_srvs::srv::Trigger;

namespace
{
geometry_msgs::msg::Quaternion quaternionFromRpy(const std::vector<double> & rpy)
{
  tf2::Quaternion q;
  q.setRPY(rpy.at(0), rpy.at(1), rpy.at(2));
  q.normalize();
  return tf2::toMsg(q);
}

rclcpp::Duration secondsToDuration(double seconds)
{
  return rclcpp::Duration::from_seconds(std::max(0.0, seconds));
}

bool sameClass(const std::string & lhs, const std::string & rhs)
{
  auto normalize = [](std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return value;
  };
  return normalize(lhs) == normalize(rhs);
}
}  // namespace

class MoveItBottleBasketPickPlace
{
public:
  explicit MoveItBottleBasketPickPlace(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declareParameters();
    readParameters();

    detections_sub_ = node_->create_subscription<yolo_msgs::msg::DetectionArray>(
      detections_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MoveItBottleBasketPickPlace::detectionsCallback, this, _1));
    depth_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MoveItBottleBasketPickPlace::depthCallback, this, _1));
    camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MoveItBottleBasketPickPlace::cameraInfoCallback, this, _1));
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MoveItBottleBasketPickPlace::jointStateCallback, this, _1));

    left_gripper_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      left_gripper_topic_, 10);
    right_gripper_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      right_gripper_topic_, 10);

    capture_srv_ = node_->create_service<Trigger>(
      "~/capture", std::bind(&MoveItBottleBasketPickPlace::captureCallback, this, _1, _2));
    execute_srv_ = node_->create_service<Trigger>(
      "~/execute", std::bind(&MoveItBottleBasketPickPlace::executeCallback, this, _1, _2));
    cancel_srv_ = node_->create_service<Trigger>(
      "~/cancel", std::bind(&MoveItBottleBasketPickPlace::cancelCallback, this, _1, _2));

    RCLCPP_INFO(node_->get_logger(), "MoveIt bottle basket pick/place ready");
    RCLCPP_INFO(node_->get_logger(), "  MoveIt groups: arm_l, arm_r");
    RCLCPP_INFO(node_->get_logger(), "  cartesian path: %s", use_cartesian_path_ ? "true" : "false");
    RCLCPP_INFO(node_->get_logger(), "  left gripper topic: %s", left_gripper_topic_.c_str());
    RCLCPP_INFO(node_->get_logger(), "  right gripper topic: %s", right_gripper_topic_.c_str());
  }

private:
  void declareParameters()
  {
    node_->declare_parameter<std::string>("detections_topic", "/yolo/detections");
    node_->declare_parameter<std::string>(
      "depth_topic", "/camera_head/camera_head/aligned_depth_to_color/image_raw");
    node_->declare_parameter<std::string>(
      "camera_info_topic", "/camera_head/camera_head/aligned_depth_to_color/camera_info");
    node_->declare_parameter<std::string>("joint_states_topic", "/joint_states");
    node_->declare_parameter<std::string>("target_frame", "base_link");
    node_->declare_parameter<std::string>("projection_frame", "");
    node_->declare_parameter<std::string>("bottle_class_name", "bottle");
    node_->declare_parameter<std::string>("basket_class_name", "basket");
    node_->declare_parameter<double>("min_score", 0.15);
    node_->declare_parameter<int>("depth_window", 7);
    node_->declare_parameter<bool>("execute_motion", false);
    node_->declare_parameter<double>("arm_center_deadband_ratio", 0.15);
    node_->declare_parameter<double>("safe_z", 0.98);
    node_->declare_parameter<double>("above_z_offset", 0.04);
    node_->declare_parameter<double>("grasp_z_offset", -0.15);
    node_->declare_parameter<std::vector<double>>("grasp_position_offset", {0.05, 0.0, 0.0});
    node_->declare_parameter<double>("basket_place_z_offset", -0.05);
    node_->declare_parameter<std::vector<double>>(
      "tool_orientation_rpy", {0.0, -1.5707963267948966, 0.0});
    node_->declare_parameter<bool>("use_cartesian_path", true);
    node_->declare_parameter<double>("cartesian_eef_step", 0.01);
    node_->declare_parameter<double>("cartesian_min_fraction", 0.90);
    node_->declare_parameter<double>("max_velocity_scaling", 0.35);
    node_->declare_parameter<double>("max_acceleration_scaling", 0.35);
    node_->declare_parameter<std::string>(
      "arm_joint_trajectory_topic",
      "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory");
    node_->declare_parameter<std::string>(
      "right_arm_joint_trajectory_topic",
      "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory");
    node_->declare_parameter<std::string>("gripper_joint", "gripper_l_joint1");
    node_->declare_parameter<std::string>("right_gripper_joint", "gripper_r_joint1");
    node_->declare_parameter<double>("gripper_open_position", 0.25);
    node_->declare_parameter<double>("gripper_closed_position", 0.55);
    node_->declare_parameter<double>("gripper_duration", 1.0);
    node_->declare_parameter<double>("gripper_settle_time", 0.1);
    node_->declare_parameter<std::vector<double>>(
      "home_joint_positions",
      {1.1383335868601994, 0.16853415484402948, 0.08995359213960642,
        -1.6951566286377817, 0.27226960562479596, -0.9465500356997781,
        -0.11196998211609252, 0.0});
    node_->declare_parameter<std::vector<double>>(
      "right_home_joint_positions",
      {1.1433190244208276, -0.13817811315876127, -0.04756538864936023,
        -1.7302943760602871, -0.11737349872306227, -0.925337957617297,
        0.19234304058867288, 0.0015339807878856412});
    node_->declare_parameter<bool>("return_home_after_place", true);
  }

  void readParameters()
  {
    detections_topic_ = node_->get_parameter("detections_topic").as_string();
    depth_topic_ = node_->get_parameter("depth_topic").as_string();
    camera_info_topic_ = node_->get_parameter("camera_info_topic").as_string();
    joint_states_topic_ = node_->get_parameter("joint_states_topic").as_string();
    target_frame_ = node_->get_parameter("target_frame").as_string();
    projection_frame_ = node_->get_parameter("projection_frame").as_string();
    bottle_class_name_ = node_->get_parameter("bottle_class_name").as_string();
    basket_class_name_ = node_->get_parameter("basket_class_name").as_string();
    min_score_ = node_->get_parameter("min_score").as_double();
    depth_window_ = node_->get_parameter("depth_window").as_int();
    execute_motion_ = node_->get_parameter("execute_motion").as_bool();
    arm_center_deadband_ratio_ = node_->get_parameter("arm_center_deadband_ratio").as_double();
    safe_z_ = node_->get_parameter("safe_z").as_double();
    above_z_offset_ = node_->get_parameter("above_z_offset").as_double();
    grasp_z_offset_ = node_->get_parameter("grasp_z_offset").as_double();
    grasp_position_offset_ = node_->get_parameter("grasp_position_offset").as_double_array();
    basket_place_z_offset_ = node_->get_parameter("basket_place_z_offset").as_double();
    tool_orientation_ = quaternionFromRpy(node_->get_parameter("tool_orientation_rpy").as_double_array());
    use_cartesian_path_ = node_->get_parameter("use_cartesian_path").as_bool();
    cartesian_eef_step_ = node_->get_parameter("cartesian_eef_step").as_double();
    cartesian_min_fraction_ = node_->get_parameter("cartesian_min_fraction").as_double();
    max_velocity_scaling_ = node_->get_parameter("max_velocity_scaling").as_double();
    max_acceleration_scaling_ = node_->get_parameter("max_acceleration_scaling").as_double();
    left_gripper_topic_ = node_->get_parameter("arm_joint_trajectory_topic").as_string();
    right_gripper_topic_ = node_->get_parameter("right_arm_joint_trajectory_topic").as_string();
    left_gripper_joint_ = node_->get_parameter("gripper_joint").as_string();
    right_gripper_joint_ = node_->get_parameter("right_gripper_joint").as_string();
    gripper_open_position_ = node_->get_parameter("gripper_open_position").as_double();
    gripper_closed_position_ = node_->get_parameter("gripper_closed_position").as_double();
    gripper_duration_ = node_->get_parameter("gripper_duration").as_double();
    gripper_settle_time_ = node_->get_parameter("gripper_settle_time").as_double();
    left_home_joint_positions_ = node_->get_parameter("home_joint_positions").as_double_array();
    right_home_joint_positions_ = node_->get_parameter("right_home_joint_positions").as_double_array();
    return_home_after_place_ = node_->get_parameter("return_home_after_place").as_bool();

    while (grasp_position_offset_.size() < 3) {
      grasp_position_offset_.push_back(0.0);
    }
  }

  bool ensureMoveGroups()
  {
    std::lock_guard<std::mutex> lock(move_group_mutex_);
    if (!left_group_) {
      RCLCPP_INFO(node_->get_logger(), "Initializing MoveIt group arm_l");
      left_group_ = std::make_unique<MoveGroupInterface>(node_, "arm_l");
      configureMoveGroup(*left_group_);
    }
    if (!right_group_) {
      RCLCPP_INFO(node_->get_logger(), "Initializing MoveIt group arm_r");
      right_group_ = std::make_unique<MoveGroupInterface>(node_, "arm_r");
      configureMoveGroup(*right_group_);
    }
    return true;
  }

  void configureMoveGroup(MoveGroupInterface & group)
  {
    group.setMaxVelocityScalingFactor(max_velocity_scaling_);
    group.setMaxAccelerationScalingFactor(max_acceleration_scaling_);
    group.setPlanningTime(5.0);
    group.setNumPlanningAttempts(5);
  }

  void detectionsCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_detections_ = *msg;
    have_detections_ = true;
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_depth_ = *msg;
    have_depth_ = true;
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_camera_info_ = *msg;
    have_camera_info_ = true;
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      current_joint_positions_[msg->name[i]] = msg->position[i];
    }
  }

  void captureCallback(
    const Trigger::Request::SharedPtr,
    const Trigger::Response::SharedPtr response)
  {
    const auto poses = processBottleAndBasket();
    if (!poses) {
      response->success = false;
      response->message = capture_failure_reason_.empty() ?
        "failed to capture bottle and basket poses" : capture_failure_reason_;
      return;
    }

    {
      std::lock_guard<std::mutex> lock(capture_mutex_);
      captured_bottle_pose_ = poses->bottle;
      captured_basket_pose_ = poses->basket;
      captured_arm_ = poses->arm;
      have_capture_ = true;
    }

    const auto & b = poses->bottle.pose.position;
    const auto & k = poses->basket.pose.position;
    response->success = true;
    response->message =
      "bottle captured for " + poses->arm + " arm at (" +
      std::to_string(b.x) + ", " + std::to_string(b.y) + ", " + std::to_string(b.z) +
      "); basket at (" + std::to_string(k.x) + ", " + std::to_string(k.y) + ", " +
      std::to_string(k.z) + ")";
  }

  void executeCallback(
    const Trigger::Request::SharedPtr,
    const Trigger::Response::SharedPtr response)
  {
    if (!execute_motion_) {
      response->success = false;
      response->message = "set execute_motion:=true to enable motion execution";
      return;
    }
    if (executing_.load()) {
      response->success = false;
      response->message = "pick/place is already running";
      return;
    }

    geometry_msgs::msg::PoseStamped bottle;
    geometry_msgs::msg::PoseStamped basket;
    std::string arm;
    {
      std::lock_guard<std::mutex> lock(capture_mutex_);
      if (!have_capture_) {
        response->success = false;
        response->message = "capture bottle and basket first";
        return;
      }
      bottle = captured_bottle_pose_;
      basket = captured_basket_pose_;
      arm = captured_arm_;
    }

    cancel_requested_.store(false);
    executing_.store(true);
    std::thread([this, bottle, basket, arm]() {
      executeSequence(bottle, basket, arm);
      executing_.store(false);
    }).detach();

    response->success = true;
    response->message = "MoveIt bottle to basket pick/place started";
  }

  void cancelCallback(
    const Trigger::Request::SharedPtr,
    const Trigger::Response::SharedPtr response)
  {
    cancel_requested_.store(true);
    {
      std::lock_guard<std::mutex> lock(move_group_mutex_);
      if (left_group_) {
        left_group_->stop();
      }
      if (right_group_) {
        right_group_->stop();
      }
    }
    response->success = true;
    response->message = executing_.load() ? "cancel requested" : "no execution is running";
  }

  struct CapturedPoses
  {
    geometry_msgs::msg::PoseStamped bottle;
    geometry_msgs::msg::PoseStamped basket;
    std::string arm;
  };

  std::optional<CapturedPoses> processBottleAndBasket()
  {
    yolo_msgs::msg::DetectionArray detections;
    sensor_msgs::msg::Image depth;
    sensor_msgs::msg::CameraInfo camera_info;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!have_detections_) {
        capture_failure_reason_ = "no YOLO detections received yet";
        return std::nullopt;
      }
      if (!have_depth_) {
        capture_failure_reason_ = "no depth image received yet";
        return std::nullopt;
      }
      if (!have_camera_info_) {
        capture_failure_reason_ = "no camera info received yet";
        return std::nullopt;
      }
      detections = latest_detections_;
      depth = latest_depth_;
      camera_info = latest_camera_info_;
    }

    const auto bottle_detection = selectDetection(detections, bottle_class_name_);
    if (!bottle_detection) {
      capture_failure_reason_ = "no matching bottle detection";
      return std::nullopt;
    }
    const auto basket_detection = selectDetection(detections, basket_class_name_);
    if (!basket_detection) {
      capture_failure_reason_ = "no matching basket detection";
      return std::nullopt;
    }

    auto bottle_pose = detectionToPose(*bottle_detection, depth, camera_info);
    if (!bottle_pose) {
      return std::nullopt;
    }
    auto basket_pose = detectionToPose(*basket_detection, depth, camera_info);
    if (!basket_pose) {
      return std::nullopt;
    }

    basket_pose->pose.position.z += basket_place_z_offset_;
    basket_pose->pose.orientation = tool_orientation_;
    const auto arm = selectArm(*bottle_detection, *bottle_pose, camera_info);

    RCLCPP_INFO(
      node_->get_logger(), "Captured bottle for %s arm: %.3f %.3f %.3f",
      arm.c_str(), bottle_pose->pose.position.x, bottle_pose->pose.position.y,
      bottle_pose->pose.position.z);
    RCLCPP_INFO(
      node_->get_logger(), "Captured basket: %.3f %.3f %.3f",
      basket_pose->pose.position.x, basket_pose->pose.position.y, basket_pose->pose.position.z);

    return CapturedPoses{*bottle_pose, *basket_pose, arm};
  }

  std::optional<yolo_msgs::msg::Detection> selectDetection(
    const yolo_msgs::msg::DetectionArray & detections,
    const std::string & class_name) const
  {
    std::optional<yolo_msgs::msg::Detection> best;
    for (const auto & detection : detections.detections) {
      if (detection.score < min_score_) {
        continue;
      }
      if (!sameClass(detection.class_name, class_name)) {
        continue;
      }
      if (!best || detection.score > best->score) {
        best = detection;
      }
    }
    return best;
  }

  std::optional<geometry_msgs::msg::PoseStamped> detectionToPose(
    const yolo_msgs::msg::Detection & detection,
    const sensor_msgs::msg::Image & depth,
    const sensor_msgs::msg::CameraInfo & camera_info)
  {
    if (!detection.bbox3d.frame_id.empty()) {
      geometry_msgs::msg::PointStamped source;
      source.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
      source.header.frame_id = detection.bbox3d.frame_id;
      source.point = detection.bbox3d.center.position;
      return transformPoint(source);
    }

    const int u = static_cast<int>(std::lround(detection.bbox.center.position.x));
    const int v = static_cast<int>(std::lround(detection.bbox.center.position.y));
    const int region_width = std::max(static_cast<int>(std::lround(detection.bbox.size.x)), depth_window_);
    const int region_height = std::max(static_cast<int>(std::lround(detection.bbox.size.y)), depth_window_);
    const auto sampled_depth = sampleDepth(depth, u, v, region_width, region_height);
    if (!sampled_depth) {
      capture_failure_reason_ = "no valid depth in bottle/basket bbox";
      return std::nullopt;
    }

    const double x = (static_cast<double>(u) - camera_info.k[2]) * *sampled_depth / camera_info.k[0];
    const double y = (static_cast<double>(v) - camera_info.k[5]) * *sampled_depth / camera_info.k[4];
    const double z = *sampled_depth;

    geometry_msgs::msg::PointStamped source;
    source.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    source.header.frame_id = projection_frame_.empty() ? camera_info.header.frame_id : projection_frame_;
    source.point.x = x;
    source.point.y = y;
    source.point.z = z;
    return transformPoint(source);
  }

  std::optional<double> sampleDepth(
    const sensor_msgs::msg::Image & image,
    int u,
    int v,
    int region_width,
    int region_height) const
  {
    if (image.height == 0 || image.width == 0 || image.data.empty()) {
      return std::nullopt;
    }
    const int half_width = std::max(0, region_width / 2);
    const int half_height = std::max(0, region_height / 2);
    const int u_min = std::max(0, u - half_width);
    const int u_max = std::min(static_cast<int>(image.width), u + half_width + 1);
    const int v_min = std::max(0, v - half_height);
    const int v_max = std::min(static_cast<int>(image.height), v + half_height + 1);

    std::vector<double> values;
    for (int yy = v_min; yy < v_max; ++yy) {
      for (int xx = u_min; xx < u_max; ++xx) {
        const std::size_t offset = static_cast<std::size_t>(yy) * image.step;
        double value = 0.0;
        if (image.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
          image.encoding == "16UC1")
        {
          const auto index = offset + static_cast<std::size_t>(xx) * sizeof(std::uint16_t);
          if (index + sizeof(std::uint16_t) > image.data.size()) {
            continue;
          }
          std::uint16_t raw = 0;
          std::memcpy(&raw, &image.data[index], sizeof(raw));
          value = static_cast<double>(raw) * 0.001;
        } else if (image.encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
          image.encoding == "32FC1")
        {
          const auto index = offset + static_cast<std::size_t>(xx) * sizeof(float);
          if (index + sizeof(float) > image.data.size()) {
            continue;
          }
          float raw = 0.0f;
          std::memcpy(&raw, &image.data[index], sizeof(raw));
          value = static_cast<double>(raw);
        } else {
          return std::nullopt;
        }
        if (std::isfinite(value) && value > 0.0) {
          values.push_back(value);
        }
      }
    }
    if (values.empty()) {
      return std::nullopt;
    }
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
  }

  std::optional<geometry_msgs::msg::PoseStamped> transformPoint(
    const geometry_msgs::msg::PointStamped & source)
  {
    try {
      const auto target = tf_buffer_.transform(
        source, target_frame_, tf2::durationFromSec(0.5));
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = target.header.stamp;
      pose.header.frame_id = target_frame_;
      pose.pose.position.x = target.point.x;
      pose.pose.position.y = target.point.y;
      pose.pose.position.z = target.point.z;
      pose.pose.orientation = tool_orientation_;
      return pose;
    } catch (const tf2::TransformException & exc) {
      capture_failure_reason_ =
        "cannot transform " + source.header.frame_id + " -> " + target_frame_ + ": " + exc.what();
      RCLCPP_WARN(node_->get_logger(), "%s", capture_failure_reason_.c_str());
      return std::nullopt;
    }
  }

  std::string selectArm(
    const yolo_msgs::msg::Detection & detection,
    const geometry_msgs::msg::PoseStamped & pose,
    const sensor_msgs::msg::CameraInfo & camera_info) const
  {
    if (camera_info.width == 0) {
      return pose.pose.position.y < 0.0 ? "right" : "left";
    }
    const double center_x = detection.bbox.center.position.x;
    const double image_center_x = static_cast<double>(camera_info.width) * 0.5;
    const double deadband_half_width =
      static_cast<double>(camera_info.width) * arm_center_deadband_ratio_ * 0.5;
    if (center_x > image_center_x + deadband_half_width) {
      return "right";
    }
    if (center_x < image_center_x - deadband_half_width) {
      return "left";
    }
    return pose.pose.position.y < 0.0 ? "right" : "left";
  }

  geometry_msgs::msg::PoseStamped makeGraspPose(
    const geometry_msgs::msg::PoseStamped & object_pose) const
  {
    auto pose = object_pose;
    pose.pose.position.x += grasp_position_offset_[0];
    pose.pose.position.y += grasp_position_offset_[1];
    pose.pose.position.z += grasp_z_offset_ + grasp_position_offset_[2];
    pose.pose.orientation = tool_orientation_;
    return pose;
  }

  geometry_msgs::msg::PoseStamped makeAbovePose(
    const geometry_msgs::msg::PoseStamped & pose) const
  {
    auto above = pose;
    above.pose.position.z = std::max(pose.pose.position.z + above_z_offset_, safe_z_);
    above.pose.orientation = tool_orientation_;
    return above;
  }

  void executeSequence(
    const geometry_msgs::msg::PoseStamped & bottle_pose,
    const geometry_msgs::msg::PoseStamped & basket_pose,
    const std::string & arm)
  {
    const auto grasp_pose = makeGraspPose(bottle_pose);
    const auto above_grasp_pose = makeAbovePose(grasp_pose);
    const auto place_pose = basket_pose;
    const auto above_place_pose = makeAbovePose(place_pose);

    bool success = false;
    const auto step = [&](const std::string & name, const std::function<bool()> & fn) {
      if (cancel_requested_.load()) {
        RCLCPP_WARN(node_->get_logger(), "Cancelled before %s", name.c_str());
        return false;
      }
      RCLCPP_INFO(node_->get_logger(), "Executing %s", name.c_str());
      const bool ok = fn();
      if (!ok) {
        RCLCPP_ERROR(node_->get_logger(), "Pick/place stopped at %s", name.c_str());
      }
      return ok;
    };

    success =
      step("open gripper", [&]() { return moveGripperForArm(arm, gripper_open_position_); }) &&
      step("move above bottle", [&]() { return moveArmToPose(arm, above_grasp_pose); }) &&
      step("move down to bottle", [&]() { return moveArmToPose(arm, grasp_pose); }) &&
      step("close gripper", [&]() { return moveGripperForArm(arm, gripper_closed_position_); }) &&
      step("lift bottle", [&]() { return moveArmToPose(arm, above_grasp_pose); }) &&
      step("move above basket", [&]() { return moveArmToPose(arm, above_place_pose); }) &&
      step("move down to basket", [&]() { return moveArmToPose(arm, place_pose); }) &&
      step("open gripper at basket", [&]() { return moveGripperForArm(arm, gripper_open_position_); }) &&
      step("retreat above basket", [&]() { return moveArmToPose(arm, above_place_pose); });

    if (return_home_after_place_ && !cancel_requested_.load()) {
      if (!moveHome(arm)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to return home");
      }
    }
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "MoveIt bottle to basket pick/place finished");
    }
  }

  bool moveArmToPose(const std::string & arm, const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!ensureMoveGroups()) {
      return false;
    }
    auto & group = *(arm == "right" ? right_group_ : left_group_);
    group.setStartStateToCurrentState();

    RCLCPP_INFO(
      node_->get_logger(), "MoveIt target %s in %s: %.3f %.3f %.3f",
      arm.c_str(), pose.header.frame_id.c_str(),
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    if (use_cartesian_path_) {
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(pose.pose);
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double fraction = group.computeCartesianPath(
        waypoints, cartesian_eef_step_, trajectory);
      RCLCPP_INFO(node_->get_logger(), "Cartesian path fraction: %.3f", fraction);
      if (fraction >= cartesian_min_fraction_) {
        return group.execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
      }
      RCLCPP_WARN(node_->get_logger(), "Cartesian path failed enough; falling back to normal plan");
    }

    group.setPoseTarget(pose);
    MoveGroupInterface::Plan plan;
    const auto plan_result = group.plan(plan);
    group.clearPoseTargets();
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      return false;
    }
    return group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool moveHome(const std::string & arm)
  {
    if (!ensureMoveGroups()) {
      return false;
    }
    auto & group = *(arm == "right" ? right_group_ : left_group_);
    const auto & home = arm == "right" ? right_home_joint_positions_ : left_home_joint_positions_;
    if (home.size() < 7) {
      RCLCPP_ERROR(node_->get_logger(), "home joint positions for %s arm need at least 7 values", arm.c_str());
      return false;
    }

    std::vector<double> arm_home(home.begin(), home.begin() + 7);
    group.setStartStateToCurrentState();
    group.setJointValueTarget(arm_home);
    MoveGroupInterface::Plan plan;
    const auto plan_result = group.plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt home plan failed");
      return false;
    }
    if (group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      return false;
    }
    return moveGripperForArm(arm, gripper_open_position_);
  }

  bool moveGripperForArm(const std::string & arm, double position)
  {
    if (arm == "right") {
      return publishGripper(right_gripper_pub_, right_gripper_joint_, position);
    }
    return publishGripper(left_gripper_pub_, left_gripper_joint_, position);
  }

  bool publishGripper(
    const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr & publisher,
    const std::string & joint_name,
    double position)
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names.push_back(joint_name);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.push_back(position);
    point.time_from_start = secondsToDuration(gripper_duration_);
    trajectory.points.push_back(point);

    RCLCPP_INFO(node_->get_logger(), "Gripper target %s: %.3f", joint_name.c_str(), position);
    publisher->publish(trajectory);
    rclcpp::sleep_for(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(gripper_duration_ + gripper_settle_time_)));
    return !cancel_requested_.load();
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::mutex move_group_mutex_;
  std::unique_ptr<MoveGroupInterface> left_group_;
  std::unique_ptr<MoveGroupInterface> right_group_;

  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_gripper_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_gripper_pub_;
  rclcpp::Service<Trigger>::SharedPtr capture_srv_;
  rclcpp::Service<Trigger>::SharedPtr execute_srv_;
  rclcpp::Service<Trigger>::SharedPtr cancel_srv_;

  std::mutex data_mutex_;
  bool have_detections_{false};
  bool have_depth_{false};
  bool have_camera_info_{false};
  yolo_msgs::msg::DetectionArray latest_detections_;
  sensor_msgs::msg::Image latest_depth_;
  sensor_msgs::msg::CameraInfo latest_camera_info_;

  std::mutex joint_mutex_;
  std::map<std::string, double> current_joint_positions_;

  std::mutex capture_mutex_;
  bool have_capture_{false};
  geometry_msgs::msg::PoseStamped captured_bottle_pose_;
  geometry_msgs::msg::PoseStamped captured_basket_pose_;
  std::string captured_arm_{"left"};
  std::string capture_failure_reason_;

  std::atomic_bool executing_{false};
  std::atomic_bool cancel_requested_{false};

  std::string detections_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string joint_states_topic_;
  std::string target_frame_;
  std::string projection_frame_;
  std::string bottle_class_name_;
  std::string basket_class_name_;
  double min_score_{0.15};
  int depth_window_{7};
  bool execute_motion_{false};
  double arm_center_deadband_ratio_{0.15};
  double safe_z_{0.98};
  double above_z_offset_{0.04};
  double grasp_z_offset_{-0.15};
  std::vector<double> grasp_position_offset_{0.05, 0.0, 0.0};
  double basket_place_z_offset_{-0.05};
  geometry_msgs::msg::Quaternion tool_orientation_;
  bool use_cartesian_path_{true};
  double cartesian_eef_step_{0.01};
  double cartesian_min_fraction_{0.90};
  double max_velocity_scaling_{0.35};
  double max_acceleration_scaling_{0.35};
  std::string left_gripper_topic_;
  std::string right_gripper_topic_;
  std::string left_gripper_joint_;
  std::string right_gripper_joint_;
  double gripper_open_position_{0.25};
  double gripper_closed_position_{0.55};
  double gripper_duration_{1.0};
  double gripper_settle_time_{0.1};
  std::vector<double> left_home_joint_positions_;
  std::vector<double> right_home_joint_positions_;
  bool return_home_after_place_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "moveit_bottle_basket_pick_place",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false));
  auto app = std::make_shared<MoveItBottleBasketPickPlace>(node);
  (void)app;

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
