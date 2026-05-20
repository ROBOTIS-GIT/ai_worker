#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include "ffw_calibration/msg/calibration_status.hpp"
#include "ffw_calibration/srv/apply_effort.hpp"
#include "ffw_calibration/srv/capture_joint.hpp"
#include "ffw_calibration/srv/get_calibration_config.hpp"
#include "ffw_calibration/srv/get_homing_offsets.hpp"
#include "ffw_calibration/srv/get_motion_poses.hpp"
#include "ffw_calibration/srv/get_safety_prep_poses.hpp"
#include "ffw_calibration/srv/get_test_joint_poses.hpp"
#include "ffw_calibration/srv/move_arm_trajectory.hpp"
#include "ffw_calibration/srv/move_to_pose.hpp"
#include "ffw_calibration/srv/move_to_zero_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace
{

using namespace std::chrono_literals;

struct JointSample
{
  double position{0.0};
  rclcpp::Time stamp;
};

struct OffsetUpdate
{
  bool ok{false};
  int old_value{0};
  int new_value{0};
  std::string message;
};

const std::vector<std::string> kJointOrder{
  "arm_r_joint2", "arm_r_joint4", "arm_r_joint6", "arm_r_joint7",
  "arm_r_joint1", "arm_r_joint3", "arm_r_joint5",
  "arm_l_joint2", "arm_l_joint4", "arm_l_joint6", "arm_l_joint7",
  "arm_l_joint1", "arm_l_joint3", "arm_l_joint5",
};

const std::map<std::string, std::string> kJointToGpio{
  {"arm_r_joint1", "dxl1"}, {"arm_r_joint2", "dxl2"},
  {"arm_r_joint3", "dxl3"}, {"arm_r_joint4", "dxl4"},
  {"arm_r_joint5", "dxl5"}, {"arm_r_joint6", "dxl6"},
  {"arm_r_joint7", "dxl7"}, {"arm_l_joint1", "dxl31"},
  {"arm_l_joint2", "dxl32"}, {"arm_l_joint3", "dxl33"},
  {"arm_l_joint4", "dxl34"}, {"arm_l_joint5", "dxl35"},
  {"arm_l_joint6", "dxl36"}, {"arm_l_joint7", "dxl37"},
};

// 이 GPIO(DXL ID)는 homing offset 갱신 시 old + delta_geom 이 아니라 old - delta_geom 과 같도록
// 기하학적 delta_pulse 부호만 반전한다. 그 외 GPIO는 old + delta_geom (YAML 은 항상 old + 기록 pulse).
const std::unordered_set<std::string> kHomingOffsetNegatePulseGpio{
  "dxl1", "dxl3", "dxl4", "dxl5", "dxl6", "dxl33", "dxl35",
};

const std::vector<std::string> kRightTrajectoryJoints{
  "arm_r_joint1", "arm_r_joint2", "arm_r_joint3", "arm_r_joint4",
  "arm_r_joint5", "arm_r_joint6", "arm_r_joint7", "gripper_r_joint1",
};

const std::vector<std::string> kLeftTrajectoryJoints{
  "arm_l_joint1", "arm_l_joint2", "arm_l_joint3", "arm_l_joint4",
  "arm_l_joint5", "arm_l_joint6", "arm_l_joint7", "gripper_l_joint1",
};

// 각 joint(1..7)가 켜졌을 때 사용할 effort 값. 호출 측에서 enable 마스크로
// joint별로 끄고 켤 수 있다. 끄면 0.
const std::vector<double> kJointEffortOn{30.0, 30.0, 30.0, 30.0, 30.0, 20.0, 1000.0};

// UI 캘리 순서(한 팔): joint2,4,6,7,1,3,5 → effort 배열 인덱스 (joint n → n-1).
constexpr std::array<int, 7> kSafetyCalibJointNums{{2, 4, 6, 7, 1, 3, 5}};
constexpr std::array<int, 7> kSafetyCalibEffortIdxOrder{{1, 3, 5, 6, 0, 2, 4}};

// duration_sec<=0 일 때 목표 단일 waypoint 의 time_from_start 가 0 이면 컨트롤러가 이상 동작할 수 있어 최소값 사용.
constexpr double kMinGoalTrajectorySec = 0.05;

constexpr int kTestJointCount = 7;
constexpr int kTestPosePerJoint = 4;
constexpr int kTestTrajAxes = 8;
constexpr int kTestPoseSlotCount = kTestJointCount * kTestPosePerJoint;
constexpr int kTestTrajPackedSize = kTestPoseSlotCount * kTestTrajAxes;

struct ArmTraj8Slot
{
  std::array<double, kTestTrajAxes> right{};
  std::array<double, kTestTrajAxes> left{};
  bool defined{false};
};

struct TestArmTraj8
{
  std::array<double, kTestTrajAxes> right{};
  std::array<double, kTestTrajAxes> left{};
  bool defined{false};
  double duration_sec{6.0};
};

struct Traj8PoseBlock
{
  double duration_sec{0.0};
  ArmTraj8Slot traj{};
};

struct RobotPosesData
{
  double test_default_duration_sec{6.0};
  std::array<std::array<TestArmTraj8, kTestPosePerJoint>, kTestJointCount> test_slots{};
  Traj8PoseBlock safety_before_joint1{};
  Traj8PoseBlock safety_before_joint3{};
  Traj8PoseBlock safety_before_joint5{};
  Traj8PoseBlock common_zero_pose{};
  double common_et_pose_duration_sec{10.0};
  Traj8PoseBlock common_lift_up{};
  Traj8PoseBlock common_lift_down{};
};

std::string trim(const std::string & value);

std::vector<double> parse_csv_doubles(const std::string & csv)
{
  std::vector<double> values;
  std::stringstream ss(csv);
  std::string token;
  while (std::getline(ss, token, ',')) {
    token = trim(token);
    if (token.empty()) {
      continue;
    }
    values.push_back(std::stod(token));
  }
  return values;
}

void apply_arm_traj8_values(
  const std::string & arm,
  const std::vector<double> & values,
  const std::filesystem::path & path,
  const std::string & context,
  ArmTraj8Slot & slot,
  bool & pose_has_right)
{
  if (static_cast<int>(values.size()) != kTestTrajAxes) {
    throw std::runtime_error(
      path.string() + ": " + context + " " + arm + " expects " +
      std::to_string(kTestTrajAxes) + " values, got " + std::to_string(values.size()));
  }
  if (arm == "right") {
    std::copy(values.begin(), values.end(), slot.right.begin());
    pose_has_right = true;
    return;
  }
  std::copy(values.begin(), values.end(), slot.left.begin());
  slot.defined = pose_has_right;
  pose_has_right = false;
}

RobotPosesData load_robot_poses(const std::filesystem::path & path)
{
  RobotPosesData data;
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("failed to open " + path.string());
  }

  enum class Section { None, Common, Safety, Test };
  Section section = Section::None;

  const std::regex section_re(R"(^\s*(common|safety|test):\s*(#.*)?$)");
  const std::regex default_duration_re(
    R"(^\s*default_duration_sec:\s*([0-9]+(?:\.[0-9]+)?)\s*(?:#.*)?$)");
  const std::regex duration_re(R"(^\s*duration_sec:\s*([0-9]+(?:\.[0-9]+)?)\s*(?:#.*)?$)");
  const std::regex joint_re(R"(^\s*joint([1-7]):\s*(#.*)?$)");
  const std::regex pose_re(R"(^\s*pose([1-4]):\s*(#.*)?$)");
  const std::regex common_block_re(R"(^\s*(zero_pose|et_pose|lift_up|lift_down):\s*(#.*)?$)");
  const std::regex safety_stage_re(R"(^\s*before_joint(1|3|5):\s*(#.*)?$)");
  const std::regex arm_array_re(R"(^\s*(right|left):\s*\[([^\]]*)\]\s*(?:#.*)?$)");

  std::string current_block;
  int joint_num = -1;
  int pose_index = -1;
  bool pose_has_right = false;
  Traj8PoseBlock * active_pose_block = nullptr;
  TestArmTraj8 * active_test_slot = nullptr;

  auto reset_pose_context = [&]() {
    current_block.clear();
    joint_num = -1;
    pose_index = -1;
    pose_has_right = false;
    active_pose_block = nullptr;
    active_test_slot = nullptr;
  };

  std::string line;
  while (std::getline(in, line)) {
    const auto trimmed = trim(line);
    if (trimmed.empty() || trimmed.front() == '#') {
      continue;
    }
    std::smatch match;

    if (std::regex_match(line, match, section_re)) {
      section = match[1].str() == "common" ? Section::Common :
        match[1].str() == "safety" ? Section::Safety : Section::Test;
      reset_pose_context();
      continue;
    }

    if (section == Section::Test && std::regex_match(line, match, default_duration_re)) {
      data.test_default_duration_sec = std::stod(match[1].str());
      continue;
    }

    if (section == Section::Common && std::regex_match(line, match, common_block_re)) {
      current_block = match[1].str();
      pose_has_right = false;
      active_pose_block = nullptr;
      active_test_slot = nullptr;
      if (current_block == "zero_pose") {
        active_pose_block = &data.common_zero_pose;
      } else if (current_block == "lift_up") {
        active_pose_block = &data.common_lift_up;
      } else if (current_block == "lift_down") {
        active_pose_block = &data.common_lift_down;
      }
      continue;
    }

    if (section == Section::Safety && std::regex_match(line, match, safety_stage_re)) {
      current_block = "before_joint" + match[1].str();
      pose_has_right = false;
      active_test_slot = nullptr;
      if (current_block == "before_joint1") {
        active_pose_block = &data.safety_before_joint1;
      } else if (current_block == "before_joint3") {
        active_pose_block = &data.safety_before_joint3;
      } else if (current_block == "before_joint5") {
        active_pose_block = &data.safety_before_joint5;
      } else {
        active_pose_block = nullptr;
      }
      continue;
    }

    if (section == Section::Test && std::regex_match(line, match, joint_re)) {
      joint_num = std::stoi(match[1].str());
      pose_index = -1;
      pose_has_right = false;
      active_pose_block = nullptr;
      active_test_slot = nullptr;
      continue;
    }

    if (section == Section::Test && joint_num >= 1 && joint_num <= kTestJointCount &&
      std::regex_match(line, match, pose_re))
    {
      pose_index = std::stoi(match[1].str());
      pose_has_right = false;
      active_pose_block = nullptr;
      active_test_slot = &data.test_slots[static_cast<std::size_t>(joint_num - 1)]
        [static_cast<std::size_t>(pose_index - 1)];
      active_test_slot->duration_sec = data.test_default_duration_sec;
      continue;
    }

    if (std::regex_match(line, match, duration_re)) {
      const double duration = std::stod(match[1].str());
      if (section == Section::Common && current_block == "et_pose") {
        data.common_et_pose_duration_sec = duration;
      } else if (active_pose_block != nullptr) {
        active_pose_block->duration_sec = duration;
      } else if (active_test_slot != nullptr) {
        active_test_slot->duration_sec = duration;
      }
      continue;
    }

    if (!std::regex_match(line, match, arm_array_re)) {
      continue;
    }

    if (active_pose_block != nullptr) {
      apply_arm_traj8_values(
        match[1].str(), parse_csv_doubles(match[2].str()), path, current_block,
        active_pose_block->traj, pose_has_right);
      continue;
    }

    if (active_test_slot != nullptr) {
      auto values = parse_csv_doubles(match[2].str());
      if (static_cast<int>(values.size()) != kTestTrajAxes) {
        throw std::runtime_error(
          path.string() + ": joint" + std::to_string(joint_num) + " pose" +
          std::to_string(pose_index) + " " + match[1].str() + " expects " +
          std::to_string(kTestTrajAxes) + " values, got " + std::to_string(values.size()));
      }
      if (match[1].str() == "right") {
        std::copy(values.begin(), values.end(), active_test_slot->right.begin());
        pose_has_right = true;
      } else {
        std::copy(values.begin(), values.end(), active_test_slot->left.begin());
        active_test_slot->defined = pose_has_right;
        pose_has_right = false;
      }
    }
  }

  if (!data.common_zero_pose.traj.defined) {
    throw std::runtime_error(path.string() + ": common.zero_pose needs right and left");
  }
  if (!data.safety_before_joint1.traj.defined || !data.safety_before_joint3.traj.defined ||
    !data.safety_before_joint5.traj.defined)
  {
    throw std::runtime_error(path.string() + ": safety before_joint1/3/5 each need right and left");
  }
  return data;
}

std::vector<double> traj8_slot_to_vector(
  const ArmTraj8Slot & slot, const std::string & arm, const std::size_t n)
{
  const auto & src = arm == "right" ? slot.right : slot.left;
  std::vector<double> out(n, 0.0);
  const std::size_t count = std::min(n, src.size());
  for (std::size_t i = 0; i < count; ++i) {
    out[i] = src[i];
  }
  return out;
}

/** homing_offsets.yaml 과 같은 robot config 디렉터리 아래 homing_offsets_backup */
std::filesystem::path homing_offsets_backup_dir(const std::filesystem::path & homing_offsets_path)
{
  return homing_offsets_path.parent_path() / "homing_offsets_backup";
}

std::string homing_offsets_backup_filename(const std::chrono::system_clock::time_point & now)
{
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm_local{};
  localtime_r(&t, &tm_local);
  std::ostringstream oss;
  oss << "homing_offsets_" << std::put_time(&tm_local, "%Y%m%d_%H%M") << ".yaml";
  return oss.str();
}

std::vector<double> build_effort_from_enable(const std::vector<bool> & enable)
{
  if (enable.empty()) {
    return kJointEffortOn;
  }
  std::vector<double> out(kJointEffortOn.size(), 0.0);
  for (std::size_t i = 0; i < kJointEffortOn.size(); ++i) {
    if (i < enable.size() && enable[i]) {
      out[i] = kJointEffortOn[i];
    }
  }
  return out;
}

std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::string arm_name_from_joint(const std::string & joint)
{
  return joint.rfind("arm_r_", 0) == 0 ? "right" : "left";
}

std::string target_key_from_joint(const std::string & joint)
{
  static const std::regex prefix("^arm_[rl]_");
  return std::regex_replace(joint, prefix, "");
}

int delta_rad_to_pulse(const double delta_rad)
{
  constexpr double zero = 0.0;
  constexpr double value_of_max_radian_position = 262144.0;
  constexpr double value_of_min_radian_position = -262144.0;
  constexpr double max_radian = 3.14159265;
  constexpr double min_radian = -3.14159265;

  if (delta_rad > 0.0) {
    return static_cast<int>(std::llround(
      delta_rad * (value_of_max_radian_position - zero) / max_radian));
  }
  if (delta_rad < 0.0) {
    return static_cast<int>(std::llround(
      delta_rad * (value_of_min_radian_position - zero) / min_radian));
  }
  return 0;
}

std::string read_file(const std::filesystem::path & path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("failed to open " + path.string());
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

void atomic_write_file(const std::filesystem::path & path, const std::string & text)
{
  // 임시 파일 + rename 으로 inode 가 바뀌면 owner/mode 가 새 파일 권한으로
  // 갈아치워져서 권한 문제가 생긴다. 같은 inode 를 truncate 후 덮어쓰기만 한다.
  std::ofstream out(path, std::ios::trunc);
  if (!out) {
    throw std::runtime_error("failed to open " + path.string() + " for writing");
  }
  out << text;
  out.flush();
  if (!out) {
    throw std::runtime_error("failed to flush " + path.string());
  }
  out.close();

  const int fd = ::open(path.c_str(), O_RDONLY);
  if (fd >= 0) {
    (void)::fsync(fd);
    (void)::close(fd);
  }
}

std::map<std::string, std::map<std::string, double>> load_targets(
  const std::filesystem::path & path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("failed to open " + path.string());
  }

  std::map<std::string, std::map<std::string, double>> targets;
  std::string current_arm;
  std::string line;
  const std::regex arm_re("^\\s{2}(right|left):\\s*(#.*)?$");
  const std::regex target_re("^\\s{4}(joint[1-7]):\\s*(-?[0-9]+(?:\\.[0-9]+)?)(?:\\s*#.*)?$");

  while (std::getline(in, line)) {
    std::smatch match;
    if (std::regex_match(line, match, arm_re)) {
      current_arm = match[1].str();
      continue;
    }
    if (!current_arm.empty() && std::regex_match(line, match, target_re)) {
      targets[current_arm][match[1].str()] = std::stod(match[2].str());
    }
  }
  return targets;
}

bool read_offset(const std::filesystem::path & path, const std::string & gpio, int & offset)
{
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  const std::regex re("^\\s+" + gpio + ":\\s*(-?\\d+).*$");
  std::string line;
  while (std::getline(in, line)) {
    std::smatch match;
    if (std::regex_match(line, match, re)) {
      offset = std::stoi(match[1].str());
      return true;
    }
  }
  return false;
}

OffsetUpdate add_offset_preserving_yaml(
  const std::filesystem::path & path,
  const std::string & gpio,
  const int delta_pulse)
{
  OffsetUpdate result;
  std::string text = read_file(path);
  const std::regex re(
    "(^|\\n)([ \\t]+" + gpio + ":[ \\t]*)(-?\\d+)([^\\n]*)");
  std::smatch match;
  if (!std::regex_search(text, match, re)) {
    result.message = gpio + " not found or commented out in " + path.string();
    return result;
  }

  result.old_value = std::stoi(match[3].str());
  result.new_value = result.old_value + delta_pulse;

  const std::string replacement =
    match[1].str() + match[2].str() + std::to_string(result.new_value) + match[4].str();
  text.replace(match.position(0), match.length(0), replacement);
  atomic_write_file(path, text);

  result.ok = true;
  result.message = "ok";
  return result;
}

double clamp_duration(const double requested, const double fallback)
{
  const double duration = requested > 0.0 ? requested : fallback;
  return std::clamp(duration, 0.1, 30.0);
}

/** move_to_zero_pose 전용: 0 이면 즉시 목표(시간 0) 궤적, 음수/NaN 이면 fallback */
double clamp_zero_pose_trajectory_duration(const double requested, const double fallback)
{
  if (requested == 0.0) {
    return 0.0;
  }
  if (requested < 0.0 || std::isnan(requested)) {
    return fallback;
  }
  return std::clamp(requested, 0.01, 30.0);
}

std::vector<double> clamped_effort(std::vector<double> target)
{
  if (target.empty()) {
    target = kJointEffortOn;
  }
  target.resize(7, 0.0);
  for (auto & value : target) {
    value = std::clamp(value, -1000.0, 1000.0);
  }
  return target;
}

std::vector<double> trajectory_all_zero(const std::size_t n)
{
  return std::vector<double>(n, 0.0);
}

}  // namespace

class CalibrationService : public rclcpp::Node
{
public:
  CalibrationService()
  : Node("ffw_calibration_service")
  {
    calibration_pose_path_ = declare_parameter<std::string>(
      "calibration_pose_path",
      "~/ros2_ws/src/ai_worker/ffw_calibration/config/common/calibration_pose.yaml");
    homing_offsets_path_ = declare_parameter<std::string>(
      "homing_offsets_path",
      "~/ros2_ws/src/ai_worker/ffw_calibration/config/ffw_bg2_rev4_follower/homing_offsets.yaml");
    robot_poses_path_ = declare_parameter<std::string>(
      "robot_poses_path",
      "~/ros2_ws/src/ai_worker/ffw_calibration/config/common/robot_poses.yaml");
    const auto joint_states_topic = declare_parameter<std::string>("joint_states_topic", "/joint_states");
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 1.0);
    default_zero_pose_duration_sec_ = declare_parameter<double>("default_zero_pose_duration_sec", 10.0);
    default_effort_duration_sec_ = declare_parameter<double>("default_effort_duration_sec", 3.0);
    effort_hz_ = declare_parameter<double>("effort_hz", 50.0);

    targets_ = load_targets(calibration_pose_path_);
    try {
      robot_poses_ = load_robot_poses(robot_poses_path_);
      if (robot_poses_.common_zero_pose.duration_sec > 0.0) {
        default_zero_pose_duration_sec_ = robot_poses_.common_zero_pose.duration_sec;
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to load robot poses from %s: %s",
        robot_poses_path_.c_str(), error.what());
      robot_poses_ = RobotPosesData{};
    }

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&CalibrationService::on_joint_state, this, std::placeholders::_1));
    service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    status_pub_ = create_publisher<ffw_calibration::msg::CalibrationStatus>(
      "/calibration/status", 10);
    arm_r_effort_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_r_effort_controller/commands", 10);
    arm_l_effort_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_l_effort_controller/commands", 10);
    arm_r_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory", 10);
    arm_l_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory", 10);

    get_config_srv_ = create_service<ffw_calibration::srv::GetCalibrationConfig>(
      "/calibration/get_config",
      std::bind(&CalibrationService::handle_get_config, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      service_callback_group_);
    get_offsets_srv_ = create_service<ffw_calibration::srv::GetHomingOffsets>(
      "/calibration/get_offsets",
      std::bind(&CalibrationService::handle_get_offsets, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      service_callback_group_);
    get_test_joint_poses_srv_ = create_service<ffw_calibration::srv::GetTestJointPoses>(
      "/calibration/get_test_joint_poses",
      std::bind(
        &CalibrationService::handle_get_test_joint_poses, this, std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(),
      service_callback_group_);
    get_safety_prep_poses_srv_ = create_service<ffw_calibration::srv::GetSafetyPrepPoses>(
      "/calibration/get_safety_prep_poses",
      std::bind(
        &CalibrationService::handle_get_safety_prep_poses, this, std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(),
      service_callback_group_);
    get_motion_poses_srv_ = create_service<ffw_calibration::srv::GetMotionPoses>(
      "/calibration/get_motion_poses",
      std::bind(
        &CalibrationService::handle_get_motion_poses, this, std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(),
      service_callback_group_);
    capture_joint_srv_ = create_service<ffw_calibration::srv::CaptureJoint>(
      "/calibration/capture_joint",
      std::bind(&CalibrationService::handle_capture_joint, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      service_callback_group_);
    move_zero_pose_srv_ = create_service<ffw_calibration::srv::MoveToZeroPose>(
      "/calibration/move_to_zero_pose",
      std::bind(&CalibrationService::handle_move_to_zero_pose, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      service_callback_group_);
    move_arm_trajectory_srv_ = create_service<ffw_calibration::srv::MoveArmTrajectory>(
      "/calibration/move_arm_trajectory",
      std::bind(
        &CalibrationService::handle_move_arm_trajectory, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      service_callback_group_);
    move_pose_srv_ = create_service<ffw_calibration::srv::MoveToPose>(
      "/calibration/move_to_pose",
      std::bind(&CalibrationService::handle_move_to_pose, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      service_callback_group_);
    apply_effort_srv_ = create_service<ffw_calibration::srv::ApplyEffort>(
      "/calibration/apply_effort",
      std::bind(&CalibrationService::handle_apply_effort, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      service_callback_group_);
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "/calibration/stop",
      std::bind(&CalibrationService::handle_stop, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      service_callback_group_);
    zero_effort_srv_ = create_service<std_srvs::srv::Trigger>(
      "/calibration/zero_effort",
      std::bind(
        &CalibrationService::handle_zero_effort, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      service_callback_group_);
    restore_backup_srv_ = create_service<std_srvs::srv::Trigger>(
      "/calibration/restore_backup",
      std::bind(&CalibrationService::handle_restore_backup, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      service_callback_group_);
    finalize_calibration_srv_ = create_service<std_srvs::srv::Trigger>(
      "/calibration/finalize_calibration",
      std::bind(
        &CalibrationService::handle_finalize_calibration, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      service_callback_group_);

    RCLCPP_INFO(get_logger(), "AI Worker calibration service ready");
  }

  ~CalibrationService() override
  {
    publish_zero_effort();
  }

private:
  bool try_acquire_busy(std::string & message)
  {
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      message = "calibration service is busy";
      return false;
    }
    return true;
  }

  void release_busy()
  {
    busy_.store(false);
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    const auto stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
      now() : rclcpp::Time(msg->header.stamp);
    const auto n = std::min(msg->name.size(), msg->position.size());
    for (std::size_t i = 0; i < n; ++i) {
      joint_samples_[msg->name[i]] = JointSample{msg->position[i], stamp};
    }
  }

  bool get_joint_sample(const std::string & joint, JointSample & sample, std::string & message)
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    const auto it = joint_samples_.find(joint);
    if (it == joint_samples_.end()) {
      message = "no joint state received for " + joint;
      return false;
    }
    sample = it->second;
    const auto age = (now() - sample.stamp).seconds();
    if (age > stale_timeout_sec_) {
      message = "stale joint state for " + joint;
      return false;
    }
    return true;
  }

  bool get_target(const std::string & joint, double & target, std::string & message) const
  {
    const auto arm = arm_name_from_joint(joint);
    const auto target_key = target_key_from_joint(joint);
    const auto arm_it = targets_.find(arm);
    if (arm_it == targets_.end()) {
      message = "missing target arm " + arm;
      return false;
    }
    const auto target_it = arm_it->second.find(target_key);
    if (target_it == arm_it->second.end()) {
      message = "missing target for " + arm + "." + target_key;
      return false;
    }
    target = target_it->second;
    return true;
  }

  void publish_status(
    const std::string & phase,
    const std::string & arm,
    const std::string & joint,
    const double progress,
    const std::string & message,
    const std::string & result = "")
  {
    ffw_calibration::msg::CalibrationStatus msg;
    msg.stamp = now();
    msg.phase = phase;
    msg.arm = arm;
    msg.joint = joint;
    msg.progress = std::clamp(progress, 0.0, 1.0);
    msg.message = message;
    msg.result = result;
    status_pub_->publish(msg);
  }

  void publish_status_done(
    const std::string & phase, const std::string & arm, const std::string & message)
  {
    publish_status(phase, arm, "", 1.0, message, "done");
  }

  void publish_status_failed(
    const std::string & phase, const std::string & arm, const std::string & message)
  {
    publish_status(phase, arm, "", 1.0, message, "failed");
  }

  void handle_get_config(
    const std::shared_ptr<ffw_calibration::srv::GetCalibrationConfig::Request>,
    const std::shared_ptr<ffw_calibration::srv::GetCalibrationConfig::Response> response)
  {
    response->success = true;
    response->message = "ok";
    for (const auto & joint : kJointOrder) {
      double target = 0.0;
      std::string message;
      response->joint_names.push_back(joint);
      response->gpio_keys.push_back(kJointToGpio.at(joint));
      response->target_rads.push_back(get_target(joint, target, message) ? target : 0.0);
    }
  }

  void handle_get_test_joint_poses(
    const std::shared_ptr<ffw_calibration::srv::GetTestJointPoses::Request>,
    const std::shared_ptr<ffw_calibration::srv::GetTestJointPoses::Response> response)
  {
    response->success = true;
    response->message = "ok";
    response->default_duration_sec = robot_poses_.test_default_duration_sec;
    response->pose_duration_sec.fill(robot_poses_.test_default_duration_sec);
    response->defined.fill(false);
    response->right_targets.fill(0.0);
    response->left_targets.fill(0.0);

    for (int joint_num = 1; joint_num <= kTestJointCount; ++joint_num) {
      for (int pose_index = 1; pose_index <= kTestPosePerJoint; ++pose_index) {
        const auto & slot = robot_poses_.test_slots[static_cast<std::size_t>(joint_num - 1)]
          [static_cast<std::size_t>(pose_index - 1)];
        const int slot_index = (joint_num - 1) * kTestPosePerJoint + (pose_index - 1);
        const int base = slot_index * kTestTrajAxes;
        response->pose_duration_sec[static_cast<std::size_t>(slot_index)] = slot.duration_sec;
        response->defined[static_cast<std::size_t>(slot_index)] = slot.defined;
        if (!slot.defined) {
          continue;
        }
        for (int axis = 0; axis < kTestTrajAxes; ++axis) {
          response->right_targets[static_cast<std::size_t>(base + axis)] =
            slot.right[static_cast<std::size_t>(axis)];
          response->left_targets[static_cast<std::size_t>(base + axis)] =
            slot.left[static_cast<std::size_t>(axis)];
        }
      }
    }
  }

  static void copy_traj8_to_response(
    const std::array<double, kTestTrajAxes> & src,
    std::array<double, kTestTrajAxes> & dst)
  {
    dst = src;
  }

  void handle_get_safety_prep_poses(
    const std::shared_ptr<ffw_calibration::srv::GetSafetyPrepPoses::Request>,
    const std::shared_ptr<ffw_calibration::srv::GetSafetyPrepPoses::Response> response)
  {
    if (!robot_poses_.safety_before_joint1.traj.defined) {
      response->success = false;
      response->message = "safety prep poses not loaded";
      return;
    }
    response->success = true;
    response->message = "ok";
    response->before_joint1_duration_sec = robot_poses_.safety_before_joint1.duration_sec;
    response->before_joint3_duration_sec = robot_poses_.safety_before_joint3.duration_sec;
    response->before_joint5_duration_sec = robot_poses_.safety_before_joint5.duration_sec;
    copy_traj8_to_response(
      robot_poses_.safety_before_joint1.traj.right, response->before_joint1_right);
    copy_traj8_to_response(
      robot_poses_.safety_before_joint1.traj.left, response->before_joint1_left);
    copy_traj8_to_response(
      robot_poses_.safety_before_joint3.traj.right, response->before_joint3_right);
    copy_traj8_to_response(
      robot_poses_.safety_before_joint3.traj.left, response->before_joint3_left);
    copy_traj8_to_response(
      robot_poses_.safety_before_joint5.traj.right, response->before_joint5_right);
    copy_traj8_to_response(
      robot_poses_.safety_before_joint5.traj.left, response->before_joint5_left);
  }

  void handle_get_motion_poses(
    const std::shared_ptr<ffw_calibration::srv::GetMotionPoses::Request>,
    const std::shared_ptr<ffw_calibration::srv::GetMotionPoses::Response> response)
  {
    if (!robot_poses_.common_zero_pose.traj.defined) {
      response->success = false;
      response->message = "motion poses not loaded";
      return;
    }
    response->success = true;
    response->message = "ok";
    response->zero_pose_duration_sec = robot_poses_.common_zero_pose.duration_sec;
    response->et_pose_duration_sec = robot_poses_.common_et_pose_duration_sec;
    copy_traj8_to_response(robot_poses_.common_zero_pose.traj.right, response->zero_pose_right);
    copy_traj8_to_response(robot_poses_.common_zero_pose.traj.left, response->zero_pose_left);

    response->lift_up_defined = robot_poses_.common_lift_up.traj.defined;
    response->lift_up_duration_sec = robot_poses_.common_lift_up.duration_sec;
    if (robot_poses_.common_lift_up.traj.defined) {
      copy_traj8_to_response(robot_poses_.common_lift_up.traj.right, response->lift_up_right);
      copy_traj8_to_response(robot_poses_.common_lift_up.traj.left, response->lift_up_left);
    }

    response->lift_down_defined = robot_poses_.common_lift_down.traj.defined;
    response->lift_down_duration_sec = robot_poses_.common_lift_down.duration_sec;
    if (robot_poses_.common_lift_down.traj.defined) {
      copy_traj8_to_response(robot_poses_.common_lift_down.traj.right, response->lift_down_right);
      copy_traj8_to_response(robot_poses_.common_lift_down.traj.left, response->lift_down_left);
    }
  }

  void handle_get_offsets(
    const std::shared_ptr<ffw_calibration::srv::GetHomingOffsets::Request>,
    const std::shared_ptr<ffw_calibration::srv::GetHomingOffsets::Response> response)
  {
    response->success = true;
    response->message = "ok";
    for (const auto & joint : kJointOrder) {
      const auto & gpio = kJointToGpio.at(joint);
      int offset = 0;
      if (!read_offset(homing_offsets_path_, gpio, offset)) {
        response->success = false;
        response->message = "failed to read " + gpio;
      }
      response->joint_names.push_back(joint);
      response->gpio_keys.push_back(gpio);
      response->offsets.push_back(offset);
    }
  }

  void handle_capture_joint(
    const std::shared_ptr<ffw_calibration::srv::CaptureJoint::Request> request,
    const std::shared_ptr<ffw_calibration::srv::CaptureJoint::Response> response)
  {
    std::string message;
    if (!try_acquire_busy(message)) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto release = std::unique_ptr<void, std::function<void(void *)>>(
      reinterpret_cast<void *>(1), [this](void *) { release_busy(); });

    const auto joint_it = kJointToGpio.find(request->joint);
    if (joint_it == kJointToGpio.end()) {
      response->success = false;
      response->message = "unknown joint " + request->joint;
      return;
    }

    JointSample sample;
    if (!get_joint_sample(request->joint, sample, message)) {
      response->success = false;
      response->message = message;
      return;
    }

    double target = 0.0;
    if (!get_target(request->joint, target, message)) {
      response->success = false;
      response->message = message;
      return;
    }

    const double delta_rad = target - sample.position;
    const int delta_pulse_geom = delta_rad_to_pulse(delta_rad);
    const int homing_offset_delta_pulse =
      kHomingOffsetNegatePulseGpio.count(joint_it->second) > 0 ?
      -delta_pulse_geom :
      delta_pulse_geom;

    try {
      create_backup_once();
      const auto updated = add_offset_preserving_yaml(
        homing_offsets_path_, joint_it->second, homing_offset_delta_pulse);
      response->success = updated.ok;
      response->message = updated.message;
      response->measured_rad = sample.position;
      response->target_rad = target;
      response->delta_rad = delta_rad;
      response->delta_pulse = homing_offset_delta_pulse;
      response->old_offset = updated.old_value;
      response->new_offset = updated.new_value;

      if (updated.ok) {
        RCLCPP_INFO(
          get_logger(),
          "Captured %s: measured=%+.4f target=%+.4f delta_rad=%+.4f delta_pulse=%+d offset[%s] %d -> %d",
          request->joint.c_str(), sample.position, target, delta_rad, homing_offset_delta_pulse,
          joint_it->second.c_str(), updated.old_value, updated.new_value);
        publish_status("capture", arm_name_from_joint(request->joint), request->joint, 1.0, "captured");
      } else {
        RCLCPP_WARN(get_logger(), "%s", updated.message.c_str());
      }
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "Capture failed: %s", error.what());
    }
  }

  void handle_move_to_zero_pose(
    const std::shared_ptr<ffw_calibration::srv::MoveToZeroPose::Request> request,
    const std::shared_ptr<ffw_calibration::srv::MoveToZeroPose::Response> response)
  {
    const auto duration =
      clamp_zero_pose_trajectory_duration(request->duration_sec, default_zero_pose_duration_sec_);
    const auto arm = trim(request->arm);
    if (arm != "right" && arm != "left" && arm != "both") {
      response->success = false;
      response->message = "arm must be right, left, or both";
      return;
    }

    if (!request->enable.empty() &&
        request->enable.size() != kJointEffortOn.size())
    {
      response->success = false;
      response->message = "enable must have 0 or " +
        std::to_string(kJointEffortOn.size()) + " entries";
      return;
    }
    std::string busy_message;
    if (!try_acquire_busy(busy_message)) {
      response->success = false;
      response->message = busy_message;
      return;
    }

    const std::string phase = "zero_pose";
    publish_status(phase, arm, "", 0.0, "task started");

    const std::vector<double> zero_right = trajectory_all_zero(kRightTrajectoryJoints.size());
    const std::vector<double> zero_left = trajectory_all_zero(kLeftTrajectoryJoints.size());

    std::thread([this, phase, arm, duration, zero_right, zero_left]() {
      const bool include_right = arm == "right" || arm == "both";
      const bool include_left = arm == "left" || arm == "both";

      std::string message;
      const bool ok = run_trajectory_only_to_targets(
        phase, arm, duration,
        include_right ? &zero_right : nullptr,
        include_left ? &zero_left : nullptr,
        message);

      if (ok) {
        publish_status_done(phase, arm, "zero pose sequence complete");
      } else {
        publish_status_failed(phase, arm, message.empty() ? "zero pose failed" : message);
      }
      release_busy();
    }).detach();

    response->success = true;
    response->message = "task started";
  }

  void handle_move_arm_trajectory(
    const std::shared_ptr<ffw_calibration::srv::MoveArmTrajectory::Request> request,
    const std::shared_ptr<ffw_calibration::srv::MoveArmTrajectory::Response> response)
  {
    const auto arm = trim(request->arm);
    if (arm != "right" && arm != "left" && arm != "both") {
      response->success = false;
      response->message = "arm must be right, left, or both";
      return;
    }

    const bool include_right = arm == "right" || arm == "both";
    const bool include_left = arm == "left" || arm == "both";
    std::vector<double> right_t;
    std::vector<double> left_t;

    if (request->use_mid_zero_target) {
      if (arm == "both") {
        response->success = false;
        response->message = "use_mid_zero_target requires arm right or left";
        return;
      }
      if (!robot_poses_.safety_before_joint1.traj.defined) {
        response->success = false;
        response->message = "safety prep poses not loaded";
        return;
      }
      if (arm == "right") {
        right_t = traj8_slot_to_vector(
          robot_poses_.safety_before_joint1.traj, "right", kRightTrajectoryJoints.size());
      } else {
        left_t = traj8_slot_to_vector(
          robot_poses_.safety_before_joint1.traj, "left", kLeftTrajectoryJoints.size());
      }
    } else {
      if (include_right) {
        if (request->right_targets.size() != kRightTrajectoryJoints.size()) {
          response->success = false;
          response->message = "right_targets must have " +
            std::to_string(kRightTrajectoryJoints.size()) + " entries";
          return;
        }
        right_t.assign(request->right_targets.begin(), request->right_targets.end());
      }
      if (include_left) {
        if (request->left_targets.size() != kLeftTrajectoryJoints.size()) {
          response->success = false;
          response->message = "left_targets must have " +
            std::to_string(kLeftTrajectoryJoints.size()) + " entries";
          return;
        }
        left_t.assign(request->left_targets.begin(), request->left_targets.end());
      }
    }

    const auto duration =
      clamp_zero_pose_trajectory_duration(request->duration_sec, default_zero_pose_duration_sec_);

    std::string busy_message;
    if (!try_acquire_busy(busy_message)) {
      response->success = false;
      response->message = busy_message;
      return;
    }

    const std::string phase = "move_trajectory";
    publish_status(phase, arm, "", 0.0, "task started");

    std::thread([this, phase, arm, duration, right_t, left_t, include_right, include_left]() {
      std::string message;
      const bool ok = run_trajectory_only_to_targets(
        phase, arm, duration,
        include_right && !right_t.empty() ? &right_t : nullptr,
        include_left && !left_t.empty() ? &left_t : nullptr,
        message);

      if (ok) {
        publish_status_done(phase, arm, "trajectory complete");
      } else {
        publish_status_failed(phase, arm, message.empty() ? "trajectory failed" : message);
      }
      release_busy();
    }).detach();

    response->success = true;
    response->message = "task started";
  }

  void handle_move_to_pose(
    const std::shared_ptr<ffw_calibration::srv::MoveToPose::Request> request,
    const std::shared_ptr<ffw_calibration::srv::MoveToPose::Response> response)
  {
    const auto duration = clamp_duration(request->duration_sec, default_zero_pose_duration_sec_);
    const bool include_right = !request->right_positions.empty();
    const bool include_left = !request->left_positions.empty();
    if (!include_right && !include_left) {
      response->success = false;
      response->message = "at least one of right_positions / left_positions must be provided";
      return;
    }

    std::vector<double> right_target;
    std::vector<double> left_target;
    if (include_right) {
      if (request->right_positions.size() != kRightTrajectoryJoints.size()) {
        response->success = false;
        response->message = "right_positions must have " +
          std::to_string(kRightTrajectoryJoints.size()) + " entries";
        return;
      }
      right_target.assign(request->right_positions.begin(), request->right_positions.end());
    }
    if (include_left) {
      if (request->left_positions.size() != kLeftTrajectoryJoints.size()) {
        response->success = false;
        response->message = "left_positions must have " +
          std::to_string(kLeftTrajectoryJoints.size()) + " entries";
        return;
      }
      left_target.assign(request->left_positions.begin(), request->left_positions.end());
    }

    const std::string arm =
      include_right && include_left ? "both"
      : include_right ? "right" : "left";

    if (!request->enable.empty() &&
        request->enable.size() != kJointEffortOn.size())
    {
      response->success = false;
      response->message = "enable must have 0 or " +
        std::to_string(kJointEffortOn.size()) + " entries";
      return;
    }
    const auto effort_target = build_effort_from_enable(
      std::vector<bool>(request->enable.begin(), request->enable.end()));

    std::string busy_message;
    if (!try_acquire_busy(busy_message)) {
      response->success = false;
      response->message = busy_message;
      return;
    }

    const std::string phase =
      request->label.empty() ? "move_to_pose" : "move_to_pose:" + request->label;
    publish_status(phase, arm, "", 0.0, "task started");

    std::thread(
      [this, phase, arm, duration, effort_target,
       right_target, left_target, include_right, include_left]() {
        std::string message;
        const bool ok = execute_move_sequence(
          phase, arm, duration,
          include_right ? &right_target : nullptr,
          include_left ? &left_target : nullptr,
          effort_target, message);

        if (ok) {
          publish_status_done(phase, arm, "move sequence complete");
        } else {
          publish_status_failed(phase, arm, message.empty() ? "move failed" : message);
        }
        release_busy();
      }).detach();

    response->success = true;
    response->message = "task started";
  }

  void handle_apply_effort(
    const std::shared_ptr<ffw_calibration::srv::ApplyEffort::Request> request,
    const std::shared_ptr<ffw_calibration::srv::ApplyEffort::Response> response)
  {
    const auto arm = trim(request->arm);
    if (arm != "right" && arm != "left") {
      response->success = false;
      response->message = "arm must be right or left";
      return;
    }

    const int safety_prep = request->safety_prep_joint_number;
    if (safety_prep != 0 && safety_prep != 1 && safety_prep != 3 && safety_prep != 5) {
      response->success = false;
      response->message = "safety_prep_joint_number must be 0, 1, 3, or 5";
      return;
    }
    if (safety_prep != 0) {
      if (request->effort_joint_2467_preset || request->effort_hold_2467_ramp_joint1 ||
        request->effort_hold_all_ramp_joint3)
      {
        response->success = false;
        response->message =
          "safety_prep_joint_number is mutually exclusive with effort_joint_2467_preset, "
          "effort_hold_2467_ramp_joint1, effort_hold_all_ramp_joint3";
        return;
      }
    } else if (request->effort_hold_2467_ramp_joint1 && !request->effort_joint_2467_preset) {
      response->success = false;
      response->message = "effort_hold_2467_ramp_joint1 requires effort_joint_2467_preset";
      return;
    } else if (request->effort_joint_2467_preset && request->effort_hold_all_ramp_joint3) {
      response->success = false;
      response->message =
        "effort_joint_2467_preset is mutually exclusive with effort_hold_all_ramp_joint3";
      return;
    }

    std::vector<double> target_effort;
    if (safety_prep == 0 && !request->effort_joint_2467_preset && !request->effort_hold_all_ramp_joint3) {
      target_effort = clamped_effort(request->target);
    } else if (safety_prep == 0 && request->effort_joint_2467_preset && !request->effort_hold_2467_ramp_joint1) {
      target_effort = build_effort_from_enable(
        std::vector<bool>{false, true, false, true, false, true, true});
    }

    const auto duration = clamp_duration(request->duration_sec, default_effort_duration_sec_);
    const bool instant_zero_traj = request->instant_zero_trajectory_first;
    const bool preset_2467 = request->effort_joint_2467_preset;
    const bool hold_2467_ramp_j1 = request->effort_hold_2467_ramp_joint1;
    const bool hold_all_ramp_j3 = request->effort_hold_all_ramp_joint3;

    std::string busy_message;
    if (!try_acquire_busy(busy_message)) {
      response->success = false;
      response->message = busy_message;
      return;
    }

    const std::string phase = "apply_effort";
    publish_status(phase, arm, "", 0.0, "task started");

    std::thread(
      [this, phase, arm, duration, safety_prep, instant_zero_traj, preset_2467,
       hold_2467_ramp_j1, hold_all_ramp_j3, target_effort]() {
        stop_requested_.store(false);
        std::string fail_message;

        if (instant_zero_traj) {
          const std::size_t n_traj =
            arm == "right" ? kRightTrajectoryJoints.size() : kLeftTrajectoryJoints.size();
          const std::vector<double> zeros(n_traj, 0.0);
          publish_arm_trajectory(arm, zeros, nullptr, 0.0);
          publish_status(phase, arm, "", 0.05, "instant zero trajectory");
        }

        const double ramp_prog_start = instant_zero_traj ? 0.1 : 0.0;
        if (safety_prep != 0) {
          ramp_effort_safety_prep_prev_joints(
            arm, safety_prep, duration, phase, ramp_prog_start, 0.99);
        } else if (preset_2467 && hold_2467_ramp_j1) {
          ramp_effort_joint1_only_hold_rest(arm, duration, phase, ramp_prog_start, 0.99);
        } else if (hold_all_ramp_j3) {
          ramp_effort(arm, target_effort, duration, phase, ramp_prog_start, 0.99);
        } else {
          ramp_effort(arm, target_effort, duration, phase, ramp_prog_start, 0.99);
        }

        if (stop_requested_.load()) {
          fail_message = "effort stopped";
        }

        if (fail_message.empty()) {
          publish_status_done(phase, arm, "effort ramp complete");
        } else {
          publish_status_failed(phase, arm, fail_message);
        }
        release_busy();
      }).detach();

    response->success = true;
    response->message = "task started";
  }

  void handle_zero_effort(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    stop_requested_.store(false);
    publish_zero_effort();
    publish_status("zero_effort", "", "", 1.0, "effort zeroed");
    response->success = true;
    response->message = "effort zeroed";
  }

  void handle_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    stop_requested_.store(true);
    publish_zero_effort();
    publish_status("stop", "", "", 1.0, "effort stopped");
    response->success = true;
    response->message = "effort stopped";
  }

  void handle_finalize_calibration(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::string message;
    if (!try_acquire_busy(message)) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto release = std::unique_ptr<void, std::function<void(void *)>>(
      reinterpret_cast<void *>(1), [this](void *) { release_busy(); });

    const std::string phase = "finalize_calibration";
    publish_status_done(
      phase, "both", "calibration finalized (no trajectory or effort commands)");
    response->success = true;
    response->message = "finalize_calibration complete";
  }

  void handle_restore_backup(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::string message;
    if (!try_acquire_busy(message)) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto release = std::unique_ptr<void, std::function<void(void *)>>(
      reinterpret_cast<void *>(1), [this](void *) { release_busy(); });

    if (backup_path_.empty() || !std::filesystem::exists(backup_path_)) {
      response->success = false;
      response->message = "no backup file available";
      return;
    }

    try {
      atomic_write_file(homing_offsets_path_, read_file(backup_path_));
      publish_status("restore", "", "", 1.0, "backup restored");
      response->success = true;
      response->message = "backup restored from " + backup_path_.string();
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
    }
  }

  bool run_trajectory_only_to_targets(
    const std::string & phase,
    const std::string & arm,
    const double target_duration,
    const std::vector<double> * right_target,
    const std::vector<double> * left_target,
    std::string & message)
  {
    stop_requested_.store(false);
    const bool include_right = arm == "right" || arm == "both";
    const bool include_left = arm == "left" || arm == "both";

    const bool has_traj_target =
      (include_right && right_target != nullptr) || (include_left && left_target != nullptr);
    const bool instant_traj = has_traj_target && target_duration <= 0.0;
    const double traj_end_sec =
      instant_traj ? kMinGoalTrajectorySec : target_duration;

    // 목표 pose 만 단일 waypoint 로 발행 (현재 pose 선행점 없음).
    if (include_right && right_target) {
      publish_arm_trajectory("right", *right_target, nullptr, traj_end_sec);
    }
    if (include_left && left_target) {
      publish_arm_trajectory("left", *left_target, nullptr, traj_end_sec);
    }
    publish_status(phase, arm, "", 0.1, "trajectory to zero");

    const double wait_sec =
      instant_traj ? (kMinGoalTrajectorySec + 0.05) : (traj_end_sec + 0.2);
    const auto wait = std::chrono::duration<double>(wait_sec);
    const auto wait_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(wait);
    const auto t0 = std::chrono::steady_clock::now();
    const auto t_end = t0 + wait_ns;
    while (std::chrono::steady_clock::now() < t_end) {
      if (stop_requested_.load()) {
        message = phase + " sequence stopped";
        return false;
      }
      const auto elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      const double ratio = std::clamp(elapsed / wait.count(), 0.0, 1.0);
      publish_status(phase, arm, "", 0.1 + 0.89 * ratio, "moving to zero");
      rclcpp::sleep_for(100ms);
    }
    publish_status(phase, arm, "", 0.99, "target pose reached");
    return true;
  }

  bool execute_move_sequence(
    const std::string & phase,
    const std::string & arm,
    const double target_duration,
    const std::vector<double> * right_target,
    const std::vector<double> * left_target,
    const std::vector<double> & effort_target,
    std::string & message)
  {
    std::vector<double> right_current;
    std::vector<double> left_current;

    if (right_target) {
      if (!get_current_trajectory_positions("right", right_current, message)) {
        return false;
      }
    }
    if (left_target) {
      if (!get_current_trajectory_positions("left", left_current, message)) {
        return false;
      }
    }

    // 1) Hold exactly the currently observed pose before changing effort.
    if (right_target) {
      publish_arm_trajectory("right", right_current, nullptr, 0.0);
    }
    if (left_target) {
      publish_arm_trajectory("left", left_current, nullptr, 0.0);
    }
    publish_status(phase, arm, "", 0.05, "holding current pose");
    rclcpp::sleep_for(100ms);

    // 2) Increase effort gradually. Progress 0.05 -> 0.5 under outer phase.
    if (right_target && left_target) {
      ramp_effort_both(effort_target, default_effort_duration_sec_, phase, 0.05, 0.5);
    } else if (right_target) {
      ramp_effort("right", effort_target, default_effort_duration_sec_, phase, 0.05, 0.5);
    } else if (left_target) {
      ramp_effort("left", effort_target, default_effort_duration_sec_, phase, 0.05, 0.5);
    }
    if (stop_requested_.load()) {
      message = phase + " sequence stopped";
      return false;
    }

    // 2.5) Effort 가 다 올라간 직후 1초 정도 정착 시간을 둔다.
    publish_status(phase, arm, "", 0.5, "settling after effort ramp");
    for (int i = 0; i < 10; ++i) {
      if (stop_requested_.load()) {
        message = phase + " sequence stopped";
        return false;
      }
      rclcpp::sleep_for(100ms);
    }

    // 3) Move from current pose to the target pose. 정착 후 새로 읽은
    //    현재 위치를 trajectory 의 시작점으로 명시해서 발행한다.
    if (right_target) {
      std::vector<double> right_now;
      if (!get_current_trajectory_positions("right", right_now, message)) {
        return false;
      }
      publish_arm_trajectory("right", right_now, right_target, target_duration);
    }
    if (left_target) {
      std::vector<double> left_now;
      if (!get_current_trajectory_positions("left", left_now, message)) {
        return false;
      }
      publish_arm_trajectory("left", left_now, left_target, target_duration);
    }
    publish_status(phase, arm, "", 0.5, "moving to target pose");

    // 4) Block until the trajectory is expected to complete, publishing
    //    progress periodically so the UI bar can sync to real time.
    const auto wait = std::chrono::duration<double>(target_duration + 0.2);
    const auto wait_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(wait);
    const auto start = std::chrono::steady_clock::now();
    const auto end = start + wait_ns;
    while (std::chrono::steady_clock::now() < end) {
      if (stop_requested_.load()) {
        message = phase + " sequence stopped";
        return false;
      }
      const auto elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      const double ratio = std::clamp(elapsed / wait.count(), 0.0, 1.0);
      // 0.5 -> 0.99 (1.0 은 done 메시지로 표기)
      const double progress = 0.5 + 0.49 * ratio;
      publish_status(phase, arm, "", progress, "moving to target pose");
      rclcpp::sleep_for(100ms);
    }
    publish_status(phase, arm, "", 0.99, "target pose reached");
    return true;
  }

  bool get_current_trajectory_positions(
    const std::string & arm,
    std::vector<double> & positions,
    std::string & message)
  {
    const auto & names = arm == "right" ? kRightTrajectoryJoints : kLeftTrajectoryJoints;
    positions.clear();
    positions.reserve(names.size());

    for (const auto & name : names) {
      JointSample sample;
      if (!get_joint_sample(name, sample, message)) {
        if (name.rfind("gripper_", 0) == 0) {
          positions.push_back(0.0);
          continue;
        }
        return false;
      }
      positions.push_back(sample.position);
    }
    return true;
  }

  static void fill_time(
    trajectory_msgs::msg::JointTrajectoryPoint & point, const double duration)
  {
    point.time_from_start.sec = static_cast<int32_t>(std::floor(duration));
    point.time_from_start.nanosec =
      static_cast<uint32_t>((duration - std::floor(duration)) * 1e9);
  }

  // to_positions == nullptr : 단일 waypoint (from_positions, time = end_time_sec).
  // to_positions != nullptr : 두 waypoint — from @0, *to_positions @end_time_sec.
  void publish_arm_trajectory(
    const std::string & arm,
    const std::vector<double> & from_positions,
    const std::vector<double> * to_positions,
    const double end_time_sec)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = arm == "right" ? kRightTrajectoryJoints : kLeftTrajectoryJoints;

    trajectory_msgs::msg::JointTrajectoryPoint p0;
    p0.positions = from_positions;
    p0.positions.resize(traj.joint_names.size(), 0.0);
    fill_time(p0, to_positions != nullptr ? 0.0 : end_time_sec);
    traj.points.push_back(p0);

    if (to_positions != nullptr) {
      trajectory_msgs::msg::JointTrajectoryPoint p1;
      p1.positions = *to_positions;
      p1.positions.resize(traj.joint_names.size(), 0.0);
      fill_time(p1, end_time_sec);
      traj.points.push_back(p1);
    }

    if (arm == "right") {
      arm_r_traj_pub_->publish(traj);
    } else {
      arm_l_traj_pub_->publish(traj);
    }
  }

  void ramp_effort_both(
    const std::vector<double> & target,
    const double duration,
    const std::string & outer_phase = "effort",
    const double prog_start = 0.0,
    const double prog_end = 1.0)
  {
    stop_requested_.store(false);
    const auto right_start = current_right_effort_;
    const auto left_start = current_left_effort_;
    const int steps = std::max(1, static_cast<int>(std::round(duration * effort_hz_)));
    const auto sleep_time = std::chrono::duration<double>(1.0 / effort_hz_);

    for (int i = 0; i <= steps; ++i) {
      if (stop_requested_.load()) {
        publish_zero_effort();
        publish_status(outer_phase, "both", "", prog_start, "effort stopped");
        return;
      }
      const double alpha = static_cast<double>(i) / static_cast<double>(steps);
      std::vector<double> right_command(7, 0.0);
      std::vector<double> left_command(7, 0.0);
      for (std::size_t j = 0; j < right_command.size(); ++j) {
        const double rs = j < right_start.size() ? right_start[j] : 0.0;
        const double ls = j < left_start.size() ? left_start[j] : 0.0;
        right_command[j] = rs + (target[j] - rs) * alpha;
        left_command[j] = ls + (target[j] - ls) * alpha;
      }
      publish_effort("right", right_command);
      publish_effort("left", left_command);
      const double progress = prog_start + (prog_end - prog_start) * alpha;
      publish_status(outer_phase, "both", "", progress, "applying effort");
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(sleep_time));
    }
  }

  void create_backup_once()
  {
    if (!backup_path_.empty()) {
      return;
    }
    const auto now = std::chrono::system_clock::now();
    const auto backup_dir = homing_offsets_backup_dir(homing_offsets_path_);
    std::filesystem::create_directories(backup_dir);

    backup_path_ = backup_dir / homing_offsets_backup_filename(now);
    int suffix = 2;
    while (std::filesystem::exists(backup_path_)) {
      const std::time_t t = std::chrono::system_clock::to_time_t(now);
      std::tm tm_local{};
      localtime_r(&t, &tm_local);
      std::ostringstream oss;
      oss << "homing_offsets_" << std::put_time(&tm_local, "%Y%m%d_%H%M")
          << "_" << suffix++ << ".yaml";
      backup_path_ = backup_dir / oss.str();
    }

    std::filesystem::copy_file(
      homing_offsets_path_, backup_path_, std::filesystem::copy_options::none);
    RCLCPP_INFO(get_logger(), "Created homing offset backup: %s", backup_path_.c_str());
  }

  void publish_effort(const std::string & arm, const std::vector<double> & effort)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = effort;
    if (arm == "right") {
      current_right_effort_ = effort;
      arm_r_effort_pub_->publish(msg);
    } else {
      current_left_effort_ = effort;
      arm_l_effort_pub_->publish(msg);
    }
  }

  void publish_zero_effort()
  {
    publish_effort("right", std::vector<double>(7, 0.0));
    publish_effort("left", std::vector<double>(7, 0.0));
  }

  void ramp_effort(
    const std::string & arm,
    const std::vector<double> & target,
    const double duration,
    const std::string & outer_phase = "effort",
    const double prog_start = 0.0,
    const double prog_end = 1.0)
  {
    stop_requested_.store(false);
    const auto start = arm == "right" ? current_right_effort_ : current_left_effort_;
    const int steps = std::max(1, static_cast<int>(std::round(duration * effort_hz_)));
    const auto sleep_time = std::chrono::duration<double>(1.0 / effort_hz_);

    for (int i = 0; i <= steps; ++i) {
      if (stop_requested_.load()) {
        publish_zero_effort();
        publish_status(outer_phase, arm, "", prog_start, "effort stopped");
        return;
      }
      const double alpha = static_cast<double>(i) / static_cast<double>(steps);
      std::vector<double> command(7, 0.0);
      for (std::size_t j = 0; j < command.size(); ++j) {
        const double s = j < start.size() ? start[j] : 0.0;
        command[j] = s + (target[j] - s) * alpha;
      }
      publish_effort(arm, command);
      const double progress = prog_start + (prog_end - prog_start) * alpha;
      publish_status(outer_phase, arm, "", progress, "applying effort");
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(sleep_time));
    }
  }

  /** SAFETY 준비: 캘리 순서상 prep 관절 이전 축만 ON 램프, prep 축 0, 이후 축 유지. */
  void ramp_effort_safety_prep_prev_joints(
    const std::string & arm,
    const int prep_joint_num,
    const double duration,
    const std::string & outer_phase = "apply_effort",
    const double prog_start = 0.0,
    const double prog_end = 0.99)
  {
    int prep_order_idx = -1;
    for (int i = 0; i < static_cast<int>(kSafetyCalibJointNums.size()); ++i) {
      if (kSafetyCalibJointNums[static_cast<std::size_t>(i)] == prep_joint_num) {
        prep_order_idx = i;
        break;
      }
    }
    if (prep_order_idx < 0) {
      return;
    }

    stop_requested_.store(false);
    const auto start = arm == "right" ? current_right_effort_ : current_left_effort_;
    const int steps = std::max(1, static_cast<int>(std::round(duration * effort_hz_)));
    const auto sleep_time = std::chrono::duration<double>(1.0 / effort_hz_);

    for (int i = 0; i <= steps; ++i) {
      if (stop_requested_.load()) {
        publish_zero_effort();
        publish_status(outer_phase, arm, "", prog_start, "effort stopped");
        return;
      }
      const double alpha = static_cast<double>(i) / static_cast<double>(steps);
      std::vector<double> command(7, 0.0);
      for (std::size_t j = 0; j < command.size(); ++j) {
        const double s = j < start.size() ? start[j] : 0.0;
        command[j] = s;
      }
      for (int o = 0; o < prep_order_idx; ++o) {
        const int idx = kSafetyCalibEffortIdxOrder[static_cast<std::size_t>(o)];
        const double goal = kJointEffortOn[static_cast<std::size_t>(idx)];
        const double s = idx < static_cast<int>(start.size()) ? start[static_cast<std::size_t>(idx)] : 0.0;
        command[static_cast<std::size_t>(idx)] = s + (goal - s) * alpha;
      }
      const int prep_idx = kSafetyCalibEffortIdxOrder[static_cast<std::size_t>(prep_order_idx)];
      command[static_cast<std::size_t>(prep_idx)] = 0.0;

      publish_effort(arm, command);
      const double progress = prog_start + (prog_end - prog_start) * alpha;
      publish_status(outer_phase, arm, "", progress, "applying effort");
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(sleep_time));
    }
  }

  /** 관절 1 만 목표 effort 로 램프, 나머지 축은 ramp 시작 시점 값 유지. */
  void ramp_effort_joint1_only_hold_rest(
    const std::string & arm,
    const double duration,
    const std::string & outer_phase = "apply_effort",
    const double prog_start = 0.0,
    const double prog_end = 0.99)
  {
    stop_requested_.store(false);
    const auto start = arm == "right" ? current_right_effort_ : current_left_effort_;
    const double j1_goal = kJointEffortOn[0];
    const int steps = std::max(1, static_cast<int>(std::round(duration * effort_hz_)));
    const auto sleep_time = std::chrono::duration<double>(1.0 / effort_hz_);

    for (int i = 0; i <= steps; ++i) {
      if (stop_requested_.load()) {
        publish_zero_effort();
        publish_status(outer_phase, arm, "", prog_start, "effort stopped");
        return;
      }
      const double alpha = static_cast<double>(i) / static_cast<double>(steps);
      std::vector<double> command(7, 0.0);
      for (std::size_t j = 0; j < command.size(); ++j) {
        const double s = j < start.size() ? start[j] : 0.0;
        command[j] = j == 0 ? s + (j1_goal - s) * alpha : s;
      }
      publish_effort(arm, command);
      const double progress = prog_start + (prog_end - prog_start) * alpha;
      publish_status(outer_phase, arm, "", progress, "applying effort");
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(sleep_time));
    }
  }

  std::filesystem::path calibration_pose_path_;
  std::filesystem::path homing_offsets_path_;
  std::filesystem::path robot_poses_path_;
  std::filesystem::path backup_path_;
  double stale_timeout_sec_{1.0};
  double default_zero_pose_duration_sec_{5.0};
  double default_effort_duration_sec_{10.0};
  double effort_hz_{50.0};

  std::map<std::string, std::map<std::string, double>> targets_;
  RobotPosesData robot_poses_;
  std::mutex joint_mutex_;
  std::unordered_map<std::string, JointSample> joint_samples_;
  std::atomic_bool busy_{false};
  std::atomic_bool stop_requested_{false};
  std::vector<double> current_right_effort_{std::vector<double>(7, 0.0)};
  std::vector<double> current_left_effort_{std::vector<double>(7, 0.0)};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Publisher<ffw_calibration::msg::CalibrationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_r_effort_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_l_effort_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_r_traj_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_l_traj_pub_;

  rclcpp::Service<ffw_calibration::srv::GetCalibrationConfig>::SharedPtr get_config_srv_;
  rclcpp::Service<ffw_calibration::srv::GetHomingOffsets>::SharedPtr get_offsets_srv_;
  rclcpp::Service<ffw_calibration::srv::GetTestJointPoses>::SharedPtr get_test_joint_poses_srv_;
  rclcpp::Service<ffw_calibration::srv::GetSafetyPrepPoses>::SharedPtr get_safety_prep_poses_srv_;
  rclcpp::Service<ffw_calibration::srv::GetMotionPoses>::SharedPtr get_motion_poses_srv_;
  rclcpp::Service<ffw_calibration::srv::CaptureJoint>::SharedPtr capture_joint_srv_;
  rclcpp::Service<ffw_calibration::srv::MoveToZeroPose>::SharedPtr move_zero_pose_srv_;
  rclcpp::Service<ffw_calibration::srv::MoveArmTrajectory>::SharedPtr move_arm_trajectory_srv_;
  rclcpp::Service<ffw_calibration::srv::MoveToPose>::SharedPtr move_pose_srv_;
  rclcpp::Service<ffw_calibration::srv::ApplyEffort>::SharedPtr apply_effort_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_effort_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restore_backup_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finalize_calibration_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CalibrationService>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
