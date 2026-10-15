#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "urdf/model.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace
{
constexpr double kGripperOpen = 0.0;
constexpr double kGripperClosed = 1.1;
constexpr int kJointCount = 20;
using Trajectory = trajectory_msgs::msg::JointTrajectory;

struct HandPose
{
  std::vector<double> open;
  std::vector<double> close;
};
}

class GripperToHand : public rclcpp::Node
{
public:
  GripperToHand()
  : Node("gripper_to_hand")
  {
    const auto presets = declare_parameter<std::vector<std::string>>(
      "hand_preset", std::vector<std::string>{"fist", "fist"});
    if (presets.size() != 2 || presets[0].empty() || presets[1].empty()) {
      throw std::invalid_argument("hand_preset must contain two preset names: [left, right]");
    }
    for (const std::string side : {"left", "right"}) {
      const auto end_tool = declare_parameter<std::string>(
        "follower_end_tool." + side, "");
      if (end_tool != "hand") {
        continue;
      }
      const size_t hand = side == "left" ? 0 : 1;
      preset_names_[hand] = presets[hand];
      const std::string prefix = "presets." + preset_names_[hand] + "." + side;
      auto & configured_pose = configured_poses_[hand];
      configured_pose.open = declare_parameter<std::vector<double>>(
        prefix + ".open", std::vector<double>{});
      configured_pose.close = declare_parameter<std::vector<double>>(
        prefix + ".close", std::vector<double>{});
      const auto valid = [](const std::vector<double> & values) {
          return values.size() == kJointCount &&
                 std::all_of(values.begin(), values.end(),
                   [](double value) {return std::isfinite(value);});
        };
      if (!valid(configured_pose.open) || !valid(configured_pose.close)) {
        throw std::invalid_argument(
                "Unknown or invalid hand preset '" + preset_names_[hand] + "' for " + side +
                ": " + prefix + ".open and .close must each contain 20 finite positions");
      }
      const std::string suffix = side == "left" ? "l" : "r";
      const std::string gripper_joint = "gripper_" + suffix + "_joint1";
      auto publisher = create_publisher<Trajectory>(
        "/leader/joint_trajectory_command_broadcaster_" + side + "_hand/joint_trajectory", 1);
      Trajectory output;
      for (int joint = 1; joint <= kJointCount; ++joint) {
        output.joint_names.push_back("finger_" + suffix + "_joint" + std::to_string(joint));
      }
      hand_joint_names_[hand] = output.joint_names;
      output.points.resize(1);

      std::function<void(Trajectory::ConstSharedPtr)> callback =
        [this, hand, gripper_joint, publisher, output]
        (Trajectory::ConstSharedPtr input) mutable {
          if (poses_[hand].close.empty()) {
            return;
          }
          // This adapter consumes live single-point gripper commands.
          const auto joint = std::find(
            input->joint_names.begin(), input->joint_names.end(), gripper_joint);
          if (input->points.size() != 1 || joint == input->joint_names.end() ||
            input->points[0].positions.size() != input->joint_names.size())
          {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000, "Invalid trajectory for %s", gripper_joint.c_str());
            return;
          }
          const auto index = static_cast<size_t>(joint - input->joint_names.begin());
          const double value = input->points[0].positions[index];
          if (!std::isfinite(value)) {
            return;
          }
          const double ratio = std::clamp(
            (value - gripper_limits_[hand].first) /
            (gripper_limits_[hand].second - gripper_limits_[hand].first), 0.0, 1.0);
          output.header = input->header;
          output.points[0].time_from_start = input->points[0].time_from_start;
          const auto & pose = poses_[hand];
          auto & positions = output.points[0].positions;
          positions.resize(kJointCount);
          for (size_t i = 0; i < positions.size(); ++i) {
            positions[i] = pose.open[i] + ratio * (pose.close[i] - pose.open[i]);
          }
          publisher->publish(output);
        };
      subscriptions_.push_back(create_subscription<Trajectory>(
        "/gripper_" + suffix + "_controller/joint_trajectory", 1, callback));
    }
    // TODO: Use joint limits from leader_initializer to improve maintainability.
    hand_description_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        poses_ = {};
        urdf::Model model;
        if (!model.initString(message->data)) {
          RCLCPP_ERROR(get_logger(), "Invalid follower URDF; hand commands disabled");
          return;
        }
        std::array<HandPose, 2> poses;
        std::array<std::pair<double, double>, 2> gripper_limits;
        for (size_t hand = 0; hand < hand_joint_names_.size(); ++hand) {
          if (hand_joint_names_[hand].empty()) {
            continue;
          }
          const std::string gripper_name = hand == 0 ? "gripper_l_joint1" : "gripper_r_joint1";
          const auto gripper = model.getJoint(gripper_name);
          auto & range = gripper_limits[hand];
          range = {kGripperOpen, kGripperClosed};
          if (gripper && gripper->limits) {
            range = {gripper->limits->lower, gripper->limits->upper};
          }
          if (!std::isfinite(range.first) || !std::isfinite(range.second) ||
            range.first >= range.second)
          {
            RCLCPP_ERROR(get_logger(), "Invalid gripper limits: %s", gripper_name.c_str());
            return;
          }
          RCLCPP_INFO(get_logger(), "%s input range: [%.3f, %.3f] rad (%s)",
            gripper_name.c_str(), range.first, range.second,
            gripper && gripper->limits ? "follower URDF" : "fallback");
          for (size_t i = 0; i < hand_joint_names_[hand].size(); ++i) {
            const auto & name = hand_joint_names_[hand][i];
            const auto joint = model.getJoint(name);
            if (!joint || !joint->limits || joint->type != urdf::Joint::REVOLUTE ||
              !std::isfinite(joint->limits->lower) || !std::isfinite(joint->limits->upper) ||
              joint->limits->lower >= joint->limits->upper)
            {
              RCLCPP_ERROR(get_logger(), "Missing or invalid hand limits: %s", name.c_str());
              return;
            }
            const double open = configured_poses_[hand].open[i];
            const double close = configured_poses_[hand].close[i];
            if (open < joint->limits->lower || open > joint->limits->upper ||
              close < joint->limits->lower || close > joint->limits->upper)
            {
              RCLCPP_ERROR(get_logger(), "Hand preset '%s' exceeds limits for %s",
                preset_names_[hand].c_str(), name.c_str());
              return;
            }
            poses[hand].open.push_back(open);
            poses[hand].close.push_back(close);
          }
          RCLCPP_INFO(get_logger(), "%s hand preset: %s",
            hand == 0 ? "left" : "right", preset_names_[hand].c_str());
        }
        gripper_limits_ = gripper_limits;
        poses_ = std::move(poses);
        RCLCPP_INFO(get_logger(), "Validated hand mapping against follower URDF");
      });
  }

private:
  std::array<std::vector<std::string>, 2> hand_joint_names_;
  std::array<std::pair<double, double>, 2> gripper_limits_;
  std::array<std::string, 2> preset_names_;
  std::array<HandPose, 2> configured_poses_;
  std::array<HandPose, 2> poses_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr hand_description_sub_;
  std::vector<rclcpp::Subscription<Trajectory>::SharedPtr> subscriptions_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperToHand>());
  rclcpp::shutdown();
  return 0;
}
