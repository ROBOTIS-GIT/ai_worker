// Copyright 2026 ROBOTIS CO., LTD.
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
#include \
  "leader_joint_trajectory_command_broadcaster/leader_joint_trajectory_command_broadcaster.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/parameter_client.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include "cyclo_teleoperation/core/leader_pose_reference.hpp"

namespace leader_joint_trajectory_command_broadcaster
{
namespace
{
using Clock = std::chrono::steady_clock;
using Pose = geometry_msgs::msg::PoseStamped;
using Trajectory = trajectory_msgs::msg::JointTrajectory;
using Status = robotis_interfaces::msg::ControlModeStatus;

Eigen::Affine3d poseValue(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond rotation(pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  Eigen::Affine3d value = Eigen::Affine3d::Identity();
  value.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  if (!value.translation().allFinite() || !rotation.coeffs().allFinite() ||
    rotation.norm() < 1e-9)
  {
    throw std::runtime_error("Invalid follower EEF state");
  }
  value.linear() = rotation.normalized().toRotationMatrix();
  return value;
}

std::string resolveModel(std::string path)
{
  const std::string prefix = "package://";
  if (path.rfind(prefix, 0) != 0) {return path;}
  const auto slash = path.find('/', prefix.size());
  if (slash == std::string::npos) {throw std::runtime_error("Invalid package URDF URI");}
  return ament_index_cpp::get_package_share_directory(
    path.substr(prefix.size(), slash - prefix.size())) + path.substr(slash);
}
}  // namespace

// One reusable bridge for every subclass of Leader JTCB. All FK and metadata work
// runs in ROS callbacks, never in ros2_control's real-time update() thread.
class LeaderPoseReferenceBridge : public std::enable_shared_from_this<LeaderPoseReferenceBridge>
{
public:
  LeaderPoseReferenceBridge(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, const Params & params,
    const std::array<std::string, 2> & raw_topics, std::atomic<bool> & pose_mode_rt)
  : node_(std::move(node)), params_(params), buffer_(node_->get_clock()),
    listener_(buffer_, node_, false), pose_mode_rt_(pose_mode_rt)
  {
    for (const double gain : {params_.cartesian_scaling.left_translation_gain,
        params_.cartesian_scaling.right_translation_gain})
    {
      if (!std::isfinite(gain) || gain <= 0.0) {
        throw std::invalid_argument("Cartesian translation gains must be finite and positive");
      }
    }
    parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(
      node_, params_.follower_control_node);
    const auto command_qos = rclcpp::QoS(1).reliable();
    const auto state_qos = rclcpp::SensorDataQoS().keep_last(1);
    const auto status_qos = rclcpp::QoS(1).reliable().transient_local();
    const std::array<std::string, 2> state_topics{{
      params_.follower_left_eef_state_topic, params_.follower_right_eef_state_topic}};
    const std::array<std::string, 2> pose_topics{{
      params_.left_eef_action_topic, params_.right_eef_action_topic}};
    const std::array<std::string, 2> gripper_topics{{
      params_.left_gripper_action_topic, params_.right_gripper_action_topic}};
    for (size_t i = 0; i < arms_.size(); ++i) {
      auto & arm = arms_[i];
      arm.pose_publisher = node_->create_publisher<Pose>(pose_topics[i], command_qos);
      arm.gripper_publisher = node_->create_publisher<Trajectory>(gripper_topics[i], command_qos);
      arm.raw_subscription = node_->create_subscription<Trajectory>(raw_topics[i], command_qos,
          [this, i](const Trajectory::SharedPtr message) {
            std::lock_guard<std::mutex> lock(mutex_);
            arms_[i].raw = message;
            arms_[i].raw_received = Clock::now();
          });
      arm.state_subscription = node_->create_subscription<Pose>(state_topics[i], state_qos,
          [this, i](const Pose::SharedPtr message) {
            std::lock_guard<std::mutex> lock(mutex_);
            arms_[i].state = message;
            arms_[i].state_history.push_back(message);
            while (arms_[i].state_history.size() > 128) {arms_[i].state_history.pop_front();}
            arms_[i].state_received = Clock::now();
          });
    }
    source_subscription_ = node_->create_subscription<std_msgs::msg::String>(
      params_.command_source_state_topic, status_qos,
      [this](const std_msgs::msg::String::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool teleop = message->data == "teleop";
        if (teleop != teleop_) {
          resetAnchors(); metadata_ready_ = false;
          metadata_requested_ = Clock::time_point{};
          pose_mode_rt_.store(false, std::memory_order_release);
        }
        teleop_ = teleop;
        });
    status_subscription_ = node_->create_subscription<Status>(params_.control_status_topic,
      status_qos, [this](const Status::SharedPtr message) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (status_.active_control_mode != message->active_control_mode ||
          message->state == Status::STATE_ERROR || message->state == Status::STATE_LOADING)
          {
            resetAnchors();
            metadata_ready_ = false;
            metadata_requested_ = Clock::time_point{};
            pose_mode_rt_.store(false, std::memory_order_release);
          }
          for (size_t i = 0; i < arms_.size(); ++i) {
            const std::string side = i == 0 ? "left" : "right";
            const bool was_active = status_.active_arms == "both" || status_.active_arms == side;
            const bool active = message->active_arms == "both" || message->active_arms == side;
            if (was_active && !active) {arms_[i].anchored = false; arms_[i].hold_valid = false;}
          }
          status_ = *message;
      });
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(10), [this]() {update();});
  }

  void reset(const bool enabled)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = enabled;
    pose_mode_rt_.store(false, std::memory_order_release);
    resetAnchors();
    metadata_ready_ = false;
    metadata_requested_ = Clock::time_point{};
  }

private:
  struct Arm
  {
    Trajectory::SharedPtr raw;
    Pose::SharedPtr state;
    std::deque<Pose::SharedPtr> state_history;
    Clock::time_point raw_received{}, state_received{};
    Eigen::Affine3d leader_anchor = Eigen::Affine3d::Identity();
    Eigen::Affine3d follower_anchor = Eigen::Affine3d::Identity();
    Eigen::Affine3d hold = Eigen::Affine3d::Identity();
    bool anchored = false, hold_valid = false;
    rclcpp::Publisher<Pose>::SharedPtr pose_publisher;
    rclcpp::Publisher<Trajectory>::SharedPtr gripper_publisher;
    rclcpp::Subscription<Pose>::SharedPtr state_subscription;
    rclcpp::Subscription<Trajectory>::SharedPtr raw_subscription;
  };

  void resetAnchors()
  {
    for (auto & arm : arms_) {arm.anchored = false; arm.hold_valid = false;}
  }

  void requestMetadata()
  {
    if (pending_ &&
      std::chrono::duration<double>(Clock::now() - metadata_requested_).count() > 2.0)
    {
      ++metadata_generation_;
      parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(node_,
          params_.follower_control_node);
      pending_ = false;
    }
    if (pending_ || !parameters_->service_is_ready() ||
      std::chrono::duration<double>(Clock::now() - metadata_requested_).count() < 1.0)
    {
      return;
    }
    metadata_requested_ = Clock::now();
    pending_ = true;
    const auto mode = status_.active_control_mode;
    const auto generation = ++metadata_generation_;
    const std::string prefix = "control_modes." + std::to_string(mode);
    const auto weak = weak_from_this();
    parameters_->get_parameters({prefix + ".reference_type", prefix + ".pose_anchor_frame",
        "follower_base_frame", "follower_left_eef", "follower_right_eef",
        "input_reference_urdf_path", "left_gripper_joint", "right_gripper_joint"},
      [weak, mode, generation](const std::shared_future<std::vector<rclcpp::Parameter>> future) {
        const auto self = weak.lock();
        if (!self) {return;}
        std::lock_guard<std::mutex> lock(self->mutex_);
        if (generation != self->metadata_generation_) {return;}
        self->pending_ = false;
        if (!self->teleop_ || self->status_.active_control_mode != mode) {return;}
        try {
          const auto values = future.get();
          if (values.size() != 8) {throw std::runtime_error("Incomplete control metadata");}
          auto text = [&values](const size_t i) {
            return values[i].get_type() == rclcpp::ParameterType::PARAMETER_STRING ?
                   values[i].as_string() : std::string{};
          };
          self->pose_mode_ = text(0) == "absolute_eef_pose";
          if (text(0) != "absolute_eef_pose" && text(0) != "absolute_joint_position") {
            throw std::runtime_error("Control mode must declare its reference_type");
          }
          self->anchor_frame_ = text(1);
          self->base_frame_ = text(2);
          self->eefs_ = {{self->params_.leader_left_eef.empty() ? text(3) :
            self->params_.leader_left_eef, self->params_.leader_right_eef.empty() ? text(4) :
            self->params_.leader_right_eef}};
          self->grippers_ = {{text(6), text(7)}};
          if (self->pose_mode_) {
            const bool follower_model = self->params_.leader_use_follower_reference_model &&
            !text(5).empty();
            const auto path = resolveModel(follower_model ? text(5) :
                self->params_.leader_urdf_path);
            // Follower-sized reference models contain lift/base joints that are
            // not raw leader inputs. FK must be relative to their arm frame.
            const auto root = follower_model && self->params_.leader_base_frame.empty() ?
            self->anchor_frame_ : self->params_.leader_base_frame;
            if (path.empty()) {
              throw std::runtime_error("leader_urdf_path is required for EEF teleop");
            }
            if (path != self->loaded_model_ || root != self->loaded_root_) {
              self->reference_.configure(path, root);
              self->loaded_model_ = path;
              self->loaded_root_ = root;
            }
            RCLCPP_INFO(self->node_->get_logger(),
              "Leader reference model=%s root=%s; Cartesian translation gains: left=%.3f right=%.3f",
              path.c_str(), root.empty() ? "<URDF root>" : root.c_str(),
              self->params_.cartesian_scaling.left_translation_gain,
              self->params_.cartesian_scaling.right_translation_gain);
          }
          self->metadata_ready_ = true;
          self->pose_mode_rt_.store(self->pose_mode_, std::memory_order_release);
          self->resetAnchors();
        } catch (const std::exception & error) {
          RCLCPP_WARN_THROTTLE(self->node_->get_logger(), *self->node_->get_clock(), 2000,
            "Leader reference not ready: %s", error.what());
        }
      });
  }

  void update()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_ || !teleop_ ||
      status_.active_control_mode == 0 ||
      (status_.state != Status::STATE_ACTIVE && status_.state != Status::STATE_HOLDING))
    {
      return;
    }
    if (!metadata_ready_) {requestMetadata(); return;}
    if (!pose_mode_) {return;}
    for (size_t i = 0; i < arms_.size(); ++i) {
      auto & arm = arms_[i];
      const std::string side = i == 0 ? "left" : "right";
      const bool requested = status_.requested_arms == "both" || status_.requested_arms == side;
      const auto preset = i == 0 ? status_.left_preset_state : status_.right_preset_state;
      const auto initial = i ==
        0 ? status_.left_initial_pose_state : status_.right_initial_pose_state;
      if (!requested &&
        (preset != Status::PRESET_UNUSED || initial != Status::INITIAL_POSE_UNUSED))
      {
        arm.anchored = false; arm.hold_valid = false; continue;
      }
      if (!arm.raw || !arm.state || arm.raw->points.empty() ||
        std::chrono::duration<double>(Clock::now() - arm.raw_received).count() > 0.5 ||
        std::chrono::duration<double>(Clock::now() - arm.state_received).count() > 0.5)
      {
        arm.anchored = false; arm.hold_valid = false; continue;
      }
      try {
        auto measured = arm.state;
        if (measured->header.frame_id != base_frame_) {
          throw std::runtime_error("Follower EEF state uses an unexpected reference frame");
        }
        Eigen::Affine3d base_from_arm = Eigen::Affine3d::Identity();
        if (!anchor_frame_.empty() && anchor_frame_ != base_frame_) {
          const auto latest = buffer_.lookupTransform(base_frame_, anchor_frame_,
              tf2::TimePointZero);
          const auto latest_stamp = rclcpp::Time(latest.header.stamp);
          if (latest_stamp.nanoseconds() != 0 &&
            rclcpp::Time(measured->header.stamp) > latest_stamp)
          {
            // robot_state_publisher may run slower than JointState/EEF feedback.
            // Select a measured pose covered by TF instead of repeatedly re-anchoring.
            const auto found = std::find_if(arm.state_history.rbegin(), arm.state_history.rend(),
                [&latest_stamp](const Pose::SharedPtr & value) {
                  return rclcpp::Time(value->header.stamp) <= latest_stamp;
                });
            if (found == arm.state_history.rend()) {continue;}
            measured = *found;
          }
          // The physical leader may use wall time while the follower uses /clock.
          // Reception freshness is steady-clock based; historical TF age must be
          // compared within the follower's timestamp domain, never to leader now().
          const double age = (rclcpp::Time(arm.state->header.stamp) -
            rclcpp::Time(measured->header.stamp)).seconds();
          if (age < 0.0 || age > params_.follower_state_timeout) {continue;}
          const auto transform = buffer_.lookupTransform(base_frame_, anchor_frame_,
            rclcpp::Time(measured->header.stamp, node_->get_clock()->get_clock_type()),
            rclcpp::Duration(0, 0));
          geometry_msgs::msg::Pose pose;
          pose.position.x = transform.transform.translation.x;
          pose.position.y = transform.transform.translation.y;
          pose.position.z = transform.transform.translation.z;
          pose.orientation = transform.transform.rotation;
          base_from_arm = poseValue(pose);
        }
        const auto follower = base_from_arm.inverse() * poseValue(measured->pose);
        const auto & mapping = i == 0 ? params_.leader_left_model_joints :
          params_.leader_right_model_joints;
        const auto leader = reference_.computePose(arm.raw->joint_names,
          arm.raw->points.front().positions, mapping, eefs_[i]);
        Eigen::Affine3d goal;
        if (requested) {
          if (!arm.anchored) {
            arm.leader_anchor = leader;
            arm.follower_anchor = follower;
            arm.anchored = true;
            RCLCPP_INFO(node_->get_logger(), "%s relative reference anchored", side.c_str());
          }
          const double translation_gain = i == 0 ?
            params_.cartesian_scaling.left_translation_gain :
            params_.cartesian_scaling.right_translation_gain;
          // Scale the anchored translation before publishing /action/*/pose.
          // The measured FK, orientation delta and paused-arm hold stay unscaled.
          goal = cyclo_teleoperation::LeaderPoseReference::absoluteGoal(
            arm.leader_anchor, leader, arm.follower_anchor, base_from_arm, translation_gain);
          arm.hold = base_from_arm.inverse() * goal;
          arm.hold_valid = true;
        } else {
          if (arm.anchored || !arm.hold_valid) {arm.hold = follower; arm.hold_valid = true;}
          arm.anchored = false;
          goal = base_from_arm * arm.hold;
        }
        Pose action;
        action.header = measured->header;  // Same measured lift sample for reconstruction.
        action.pose.position.x = goal.translation().x();
        action.pose.position.y = goal.translation().y();
        action.pose.position.z = goal.translation().z();
        const Eigen::Quaterniond rotation(goal.linear());
        action.pose.orientation.x = rotation.x(); action.pose.orientation.y = rotation.y();
        action.pose.orientation.z = rotation.z(); action.pose.orientation.w = rotation.w();
        arm.pose_publisher->publish(action);
        if (!grippers_[i].empty()) {
          const auto found = std::find(arm.raw->joint_names.begin(), arm.raw->joint_names.end(),
              grippers_[i]);
          if (found != arm.raw->joint_names.end()) {
            const auto index = static_cast<size_t>(std::distance(arm.raw->joint_names.begin(),
                found));
            Trajectory gripper;
            gripper.joint_names = {grippers_[i]};
            gripper.points.resize(1);
            gripper.points[0].positions = {arm.raw->points.front().positions.at(index)};
            arm.gripper_publisher->publish(gripper);
          }
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
          "Leader %s EEF action paused: %s", side.c_str(), error.what());
      }
    }
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  Params params_;
  std::mutex mutex_;
  std::array<Arm, 2> arms_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::atomic<bool> & pose_mode_rt_;
  cyclo_teleoperation::LeaderPoseReference reference_;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr source_subscription_;
  rclcpp::Subscription<Status>::SharedPtr status_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  Status status_;
  Clock::time_point metadata_requested_{};
  uint64_t metadata_generation_ = 0;
  bool enabled_ = false, teleop_ = false, metadata_ready_ = false, pending_ = false,
    pose_mode_ = false;
  std::string anchor_frame_, base_frame_, loaded_model_, loaded_root_;
  std::array<std::string, 2> eefs_, grippers_;
};

bool LeaderJointTrajectoryCommandBroadcaster::configure_pose_reference()
{
  pose_reference_bridge_.reset();
  if (!params_.enable_teleoperation || !params_.leader_pose_reference_enabled) {return true;}
  try {
    pose_reference_bridge_ = std::make_shared<LeaderPoseReferenceBridge>(get_node(), params_,
      std::array<std::string, 2>{{joint_trajectory_publishers_.at("left")->get_topic_name(),
          joint_trajectory_publishers_.at("right")->get_topic_name()}}, pose_reference_mode_rt_);
    return true;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Leader reference configuration failed: %s",
        error.what());
    return false;
  }
}

void LeaderJointTrajectoryCommandBroadcaster::reset_pose_reference(const bool active)
{
  if (pose_reference_bridge_) {pose_reference_bridge_->reset(active);}
}
}  // namespace leader_joint_trajectory_command_broadcaster
