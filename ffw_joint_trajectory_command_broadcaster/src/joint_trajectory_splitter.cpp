#include <memory>
#include <stdexcept>
#include <utility>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace ffw_trajectory_splitter
{
using Trajectory = trajectory_msgs::msg::JointTrajectory;
using Point = trajectory_msgs::msg::JointTrajectoryPoint;

static std::pair<Trajectory, Trajectory> split_trajectory(const Trajectory & message)
{
  if (message.joint_names.empty()) {
    throw std::invalid_argument("trajectory has no joint names");
  }

  const auto fields = {&Point::positions, &Point::velocities, &Point::accelerations,
    &Point::effort};
  for (const auto & point : message.points) {
    for (const auto field : fields) {
      const auto & values = point.*field;
      if (!values.empty() && values.size() != message.joint_names.size()) {
        throw std::invalid_argument("trajectory field length does not match joint names");
      }
    }
  }

  Trajectory arm, gripper;
  for (auto * output : {&arm, &gripper}) {
    output->header = message.header;
    output->points.resize(message.points.size());
    for (size_t p = 0; p < message.points.size(); ++p) {
      output->points[p].time_from_start = message.points[p].time_from_start;
    }
  }
  for (size_t i = 0; i < message.joint_names.size(); ++i) {
    const auto & name = message.joint_names[i];
    auto & output = name.rfind("gripper_", 0) == 0 ? gripper : arm;
    output.joint_names.push_back(name);
    for (size_t p = 0; p < message.points.size(); ++p) {
      for (const auto field : fields) {
        const auto & values = message.points[p].*field;
        if (!values.empty()) {
          (output.points[p].*field).push_back(values[i]);
        }
      }
    }
  }
  return {std::move(arm), std::move(gripper)};
}

class JointTrajectorySplitter : public rclcpp::Node
{
public:
  JointTrajectorySplitter()
  : Node("joint_trajectory_splitter")
  {
    for (const std::string side : {"left", "right"}) {
      const std::string suffix = side == "left" ? "l" : "r";
      auto arm = create_publisher<Trajectory>(
        "/arm_" + suffix + "_controller/joint_trajectory", 10);
      auto gripper = create_publisher<Trajectory>(
        "/gripper_" + suffix + "_controller/joint_trajectory", 10);
      subscriptions_.push_back(create_subscription<Trajectory>(
        "/leader/joint_trajectory_command_broadcaster_" + side + "/joint_trajectory", 10,
        [this, side, arm, gripper](Trajectory::ConstSharedPtr message) {
          try {
            auto outputs = split_trajectory(*message);
            if (!outputs.first.joint_names.empty()) {
              arm->publish(outputs.first);
            }
            if (!outputs.second.joint_names.empty()) {
              gripper->publish(outputs.second);
            }
          } catch (const std::invalid_argument & error) {
            RCLCPP_WARN(
              get_logger(), "Ignored invalid %s trajectory: %s", side.c_str(), error.what());
          }
        }));
    }
  }

private:
  std::vector<rclcpp::Subscription<Trajectory>::SharedPtr> subscriptions_;
};
}  // namespace ffw_trajectory_splitter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ffw_trajectory_splitter::JointTrajectorySplitter>());
  rclcpp::shutdown();
  return 0;
}
