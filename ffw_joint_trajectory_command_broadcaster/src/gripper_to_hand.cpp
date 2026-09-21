#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
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
}

class GripperToHand : public rclcpp::Node
{
public:
  GripperToHand()
  : Node("gripper_to_hand")
  {
    for (const std::string side : {"left", "right"}) {
      const auto end_tool = declare_parameter<std::string>(
        "follower_end_tool." + side, "");
      if (end_tool != "hand") {
        continue;
      }
      const size_t hand = side == "left" ? 0 : 1;
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
          if (close_positions_[hand].empty()) {
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
          output.points[0].positions = close_positions_[hand];
          for (auto & position : output.points[0].positions) {
            position *= ratio;
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
        close_positions_ = {};
        urdf::Model model;
        if (!model.initString(message->data)) {
          RCLCPP_ERROR(get_logger(), "Invalid follower URDF; hand commands disabled");
          return;
        }
        std::array<std::vector<double>, 2> positions;
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
              joint->limits->lower > 0.0 || joint->limits->upper < 0.0 ||
              joint->limits->lower >= joint->limits->upper)
            {
              RCLCPP_ERROR(get_logger(), "Missing or invalid hand limits: %s", name.c_str());
              return;
            }
            // Joints 5, 9, 13, 17 stay at zero; thumb signs are mirrored.
            const bool fixed = i >= 4 && (i - 4) % 4 == 0;
            const bool negative = hand == 0 ? (i == 0 || i == 2 || i == 3) : i == 1;
            const double close = fixed ? 0.0 :
              negative ? joint->limits->lower : joint->limits->upper;
            if (!fixed && (negative ? close >= 0.0 : close <= 0.0)) {
              RCLCPP_ERROR(get_logger(), "Hand limit has wrong direction: %s", name.c_str());
              return;
            }
            positions[hand].push_back(close);
          }
        }
        gripper_limits_ = gripper_limits;
        close_positions_ = std::move(positions);
        RCLCPP_INFO(get_logger(), "Loaded hand mapping limits from follower URDF");
      });
  }

private:
  std::array<std::vector<std::string>, 2> hand_joint_names_;
  std::array<std::pair<double, double>, 2> gripper_limits_;
  std::array<std::vector<double>, 2> close_positions_;
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
