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

#include "ffw_swerve_drive_controller/swerve_drive_controller.hpp"

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <functional>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"


namespace ffw_swerve_drive_controller
{

using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using rcl_interfaces::msg::ParameterDescriptor;
using rcl_interfaces::msg::ParameterType;

// Reset function
void SwerveDriveController::reset_controller_reference_msg(
  const std::shared_ptr<geometry_msgs::msg::Twist> & msg)
{
  msg->linear.x = std::numeric_limits<double>::quiet_NaN();
  msg->linear.y = std::numeric_limits<double>::quiet_NaN();
  msg->linear.z = std::numeric_limits<double>::quiet_NaN();
  msg->angular.x = std::numeric_limits<double>::quiet_NaN();
  msg->angular.y = std::numeric_limits<double>::quiet_NaN();
  msg->angular.z = std::numeric_limits<double>::quiet_NaN();
}

// --- Controller Implementation ---
SwerveDriveController::SwerveDriveController()
: controller_interface::ControllerInterface(),
  ref_timeout_(0, 0)
{}
// *****************************************

CallbackReturn SwerveDriveController::on_init()
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Initializing SwerveDriveController");
  try {
    // ***** declare parameters *****
    auto_declare<std::string>("ffw_type", "ffw_v2");
    auto_declare<std::vector<std::string>>("steering_joint_names", std::vector<std::string>{});
    auto_declare<std::vector<std::string>>("wheel_joint_names", std::vector<std::string>{});
    auto_declare<double>("wheel_radius", 0.05);
    auto_declare<std::vector<double>>("module_x_offsets", std::vector<double>{});
    auto_declare<std::vector<double>>("module_y_offsets", std::vector<double>{});
    auto_declare<std::vector<double>>("module_angle_offsets", std::vector<double>{});
    auto_declare<std::vector<double>>("module_steering_limit_lower", std::vector<double>{});
    auto_declare<std::vector<double>>("module_steering_limit_upper", std::vector<double>{});
    auto_declare<std::vector<double>>("module_wheel_speed_limit_lower", std::vector<double>{});
    auto_declare<std::vector<double>>("module_wheel_speed_limit_upper", std::vector<double>{});
    auto_declare<bool>("enabled_steering_flip", false);
    auto_declare<bool>("enabled_open_loop", false);
    auto_declare<bool>("enabled_steering_angular_velocity_limit", false);
    auto_declare<bool>("enabled_sync_steering_angular_velocity", false);
    auto_declare<bool>("enabled_steering_angular_limit", true);
    auto_declare<double>("steering_angular_velocity_limit", 0.05);
    auto_declare<double>("steering_alignment_angle_error_threshold", 0.1);
    auto_declare<double>("steering_alignment_start_angle_error_threshold", 0.1);
    auto_declare<double>("steering_alignment_start_speed_error_threshold", 0.1);
    auto_declare<double>("linear_vel_deadband", 0.1);
    auto_declare<double>("angular_vel_deadband", 0.1);

    auto_declare<std::string>("cmd_vel_topic", "/cmd_vel");
    auto_declare<bool>("use_stamped_cmd_vel", false);
    auto_declare<double>("cmd_vel_timeout", 500.0);

    // Odometry parameters
    auto_declare<std::string>("odom_solver_method", "svd");
    auto_declare<std::string>("odom_frame_id", "odom");
    auto_declare<std::string>("base_frame_id", "base_link");
    auto_declare<bool>("enable_odom_tf", true);
    auto_declare<std::vector<double>>(
      "pose_covariance_diagonal", {0.001, 0.001, 1e6, 1e6, 1e6,
        0.01});
    auto_declare<std::vector<double>>(
      "twist_covariance_diagonal", {0.001, 0.001, 1e6, 1e6, 1e6,
        0.01});
    auto_declare<int>("velocity_rolling_window_size", 1);
    auto_declare<std::string>("odom_source", "feedback");

    // Visualization parameters
    auto_declare<bool>("enable_visualization", true);
    auto_declare<std::string>("visualization_marker_topic", "~/swerve_visualization_markers");
    auto_declare<double>("visualization_update_time", 10.0);

    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Exception during parameter declaration: %s",
      e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "Parameter declaration successful");
  return CallbackReturn::SUCCESS;
}

// command_interface_configuration, state_interface_configuration
controller_interface::InterfaceConfiguration
SwerveDriveController::command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  std::vector<std::string> steering_names;
  std::vector<std::string> wheel_names;
  try {
    // Use get_node() which is available after on_init()
    steering_names = get_node()->get_parameter("steering_joint_names").as_string_array();
    wheel_names = get_node()->get_parameter("wheel_joint_names").as_string_array();
  } catch (const std::exception & e) {
    // Log error but don't crash, configuration might be incomplete
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Error reading joint names during command config: %s.",
      e.what());
    // It's safer to return an empty config here, let CM handle missing interfaces later
    return conf;
  }

  if (steering_names.empty() || wheel_names.empty()) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Joint names parameters are empty during command config.");
    return conf;
  }
  if (steering_names.size() != wheel_names.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Steering and wheel joint names parameters must have the same size!");
    return conf;
  }

  conf.names.reserve(steering_names.size() + wheel_names.size());
  for (const auto & joint_name : steering_names) {
    conf.names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  for (const auto & joint_name : wheel_names) {
    conf.names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  return conf;
}

controller_interface::InterfaceConfiguration SwerveDriveController::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  std::vector<std::string> steering_names;
  std::vector<std::string> wheel_names;
  try {
    steering_names = get_node()->get_parameter("steering_joint_names").as_string_array();
    wheel_names = get_node()->get_parameter("wheel_joint_names").as_string_array();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Error reading joint names during state config: %s.",
      e.what());
    return conf;
  }

  if (steering_names.empty() || wheel_names.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "Joint names parameters are empty during state config.");
    return conf;
  }
  if (steering_names.size() != wheel_names.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Steering and wheel joint names parameters must have the same size!");
    return conf;
  }

  conf.names.reserve(steering_names.size() + wheel_names.size());
  for (const auto & joint_name : steering_names) {
    conf.names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  for (const auto & joint_name : wheel_names) {
    conf.names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  return conf;
}

CallbackReturn SwerveDriveController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();
  RCLCPP_DEBUG(logger, "Configuring Swerve Drive Controller...");

  // update parameters if they have changed
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  // Get parameters
  try {
    steering_joint_names_ = get_node()->get_parameter("steering_joint_names").as_string_array();
    wheel_joint_names_ = get_node()->get_parameter("wheel_joint_names").as_string_array();
    wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
    module_x_offsets_ = get_node()->get_parameter("module_x_offsets").as_double_array();
    module_y_offsets_ = get_node()->get_parameter("module_y_offsets").as_double_array();
    module_angle_offsets_ = get_node()->get_parameter("module_angle_offsets").as_double_array();
    module_steering_limit_lower_ =
      get_node()->get_parameter("module_steering_limit_lower").as_double_array();
    module_steering_limit_upper_ =
      get_node()->get_parameter("module_steering_limit_upper").as_double_array();
    module_wheel_speed_limit_lower_ =
      get_node()->get_parameter("module_wheel_speed_limit_lower").as_double_array();
    module_wheel_speed_limit_upper_ =
      get_node()->get_parameter("module_wheel_speed_limit_upper").as_double_array();
    steering_angular_velocity_limit_ =
      get_node()->get_parameter("steering_angular_velocity_limit").as_double();
    enabled_steering_flip_ = get_node()->get_parameter("enabled_steering_flip").as_bool();
    enabled_open_loop_ = get_node()->get_parameter("enabled_open_loop").as_bool();
    enabled_steering_angular_velocity_limit_ =
      get_node()->get_parameter("enabled_steering_angular_velocity_limit").as_bool();
    steering_alignment_angle_error_threshold_ = get_node()->get_parameter(
      "steering_alignment_angle_error_threshold").as_double();
    steering_alignment_start_angle_error_threshold_ = get_node()->get_parameter(
      "steering_alignment_start_angle_error_threshold").as_double();
    steering_alignment_start_speed_error_threshold_ = get_node()->get_parameter(
      "steering_alignment_start_speed_error_threshold").as_double();

    linear_vel_deadband_ = get_node()->get_parameter(
      "linear_vel_deadband").as_double();
    angular_vel_deadband_ = get_node()->get_parameter(
      "angular_vel_deadband").as_double();

    cmd_vel_topic_ = get_node()->get_parameter("cmd_vel_topic").as_string();
    use_stamped_cmd_vel_ = get_node()->get_parameter("use_stamped_cmd_vel").as_bool();
    cmd_vel_timeout_ = get_node()->get_parameter("cmd_vel_timeout").as_double();
    odom_frame_id_ = get_node()->get_parameter("odom_frame_id").as_string();
    base_frame_id_ = get_node()->get_parameter("base_frame_id").as_string();
    enable_odom_tf_ = get_node()->get_parameter("enable_odom_tf").as_bool();
    pose_covariance_diagonal_ =
      get_node()->get_parameter("pose_covariance_diagonal").as_double_array();
    twist_covariance_diagonal_ =
      get_node()->get_parameter("twist_covariance_diagonal").as_double_array();
    odom_solver_method_str_ = get_node()->get_parameter("odom_solver_method").as_string();
    odom_source_ = get_node()->get_parameter("odom_source").as_string();
    enable_visualization_ = get_node()->get_parameter("enable_visualization").as_bool();
    visualization_marker_topic_ =
      get_node()->get_parameter("visualization_marker_topic").as_string();
    visualization_update_time_ = get_node()->get_parameter("visualization_update_time").as_double();
    velocity_rolling_window_size_ =
      get_node()->get_parameter("velocity_rolling_window_size").as_int();

    ref_timeout_ = rclcpp::Duration::from_seconds(cmd_vel_timeout_);

    enabled_speed_limits_ = params_.enabled_speed_limits;
    publish_limited_velocity_ = params_.publish_limited_velocity;
    limiter_linear_x_ = SpeedLimiter(
      params_.linear.x.has_velocity_limits, params_.linear.x.has_acceleration_limits,
      params_.linear.x.has_jerk_limits, params_.linear.x.min_velocity,
      params_.linear.x.max_velocity,
      params_.linear.x.min_acceleration, params_.linear.x.max_acceleration,
      params_.linear.x.min_jerk,
      params_.linear.x.max_jerk);
    limiter_linear_y_ = SpeedLimiter(
      params_.linear.y.has_velocity_limits, params_.linear.y.has_acceleration_limits,
      params_.linear.y.has_jerk_limits, params_.linear.y.min_velocity,
      params_.linear.y.max_velocity,
      params_.linear.y.min_acceleration, params_.linear.y.max_acceleration,
      params_.linear.y.min_jerk,
      params_.linear.y.max_jerk);
    limiter_angular_z_ = SpeedLimiter(
      params_.angular.z.has_velocity_limits, params_.angular.z.has_acceleration_limits,
      params_.angular.z.has_jerk_limits, params_.angular.z.min_velocity,
      params_.angular.z.max_velocity, params_.angular.z.min_acceleration,
      params_.angular.z.max_acceleration, params_.angular.z.min_jerk, params_.angular.z.max_jerk);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger, "Exception during parameter reading: %s", e.what());
    return CallbackReturn::ERROR;
  }

  num_modules_ = steering_joint_names_.size();

  // --- Parameter Validation ---
  if (steering_joint_names_.size() != num_modules_ || wheel_joint_names_.size() != num_modules_ ||
    module_x_offsets_.size() != num_modules_ || module_y_offsets_.size() != num_modules_ ||
    module_angle_offsets_.size() != num_modules_ ||
    module_steering_limit_lower_.size() != num_modules_ ||
    module_steering_limit_upper_.size() != num_modules_ ||
    module_wheel_speed_limit_lower_.size() != num_modules_ ||
    module_wheel_speed_limit_upper_.size() != num_modules_)
  {
    RCLCPP_FATAL(
      logger,
      "Parameter array lengths do not match expected number of modules (%ld).",
      num_modules_);
    RCLCPP_FATAL(
      logger,
      "steering_joint_names: %zu, wheel_joint_names: %zu,"
      " module_x_offsets: %zu, module_y_offsets: %zu, module_angle_offsets: %zu,"
      " module_steering_limit_lower: %zu, module_steering_limit_upper: %zu, "
      "module_wheel_speed_limit_lower: %zu, module_wheel_speed_limit_upper: %zu",
      steering_joint_names_.size(), wheel_joint_names_.size(),
      module_x_offsets_.size(), module_y_offsets_.size(),
      module_angle_offsets_.size(),
      module_steering_limit_lower_.size(),
      module_steering_limit_upper_.size(),
      module_wheel_speed_limit_lower_.size(), module_wheel_speed_limit_upper_.size());
    return CallbackReturn::ERROR;
  }
  if (wheel_radius_ <= 0.0) {
    RCLCPP_ERROR(logger, "'wheel_radius' must be positive.");
    return CallbackReturn::ERROR;
  }

  // --- Setup Subscriber ---
  cmd_vel_subscriber_ = get_node()->create_subscription<CmdVelMsg>(
    cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&SwerveDriveController::reference_callback, this, std::placeholders::_1)
  );

  // Initialize the buffer
  auto initial_cmd = std::make_shared<CmdVelMsg>();
  reset_controller_reference_msg(initial_cmd);
  cmd_vel_buffer_.initRT(initial_cmd);
  last_cmd_vel_time_ = get_node()->now();

  RCLCPP_DEBUG(
    logger, "Subscribed to %s (expecting geometry_msgs/msg/Twist)",
    cmd_vel_topic_.c_str());

  // Publisher
  try {
    odom_s_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SystemDefaultsQoS());
    rt_odom_state_publisher_ = std::make_unique<OdomStatePublisher>(odom_s_publisher_);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger, "Exception during publisher creation: %s", e.what());
    return CallbackReturn::ERROR;
  }


  // initialize odometry and set parameters
  try {
    odometry_.init(get_node()->now());
    odometry_.setModuleParams(module_x_offsets_, module_y_offsets_, wheel_radius_);
    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size_);
    OdomSolverMethod solver_method_enum = OdomSolverMethod::SVD;
    if (odom_solver_method_str_ == "pseudo_inverse") {
      solver_method_enum = OdomSolverMethod::PSEUDO_INVERSE;
    } else if (odom_solver_method_str_ == "qr") {
      solver_method_enum = OdomSolverMethod::QR_DECOMPOSITION;
    } else if (odom_solver_method_str_ == "svd") {
      solver_method_enum = OdomSolverMethod::SVD;
    } else {
      RCLCPP_WARN(
        logger,
        "Invalid 'odom_solver_method' parameter: %s. Using SVD by default.",
        odom_solver_method_str_.c_str());
    }
    // Set the solver method
    odometry_.set_solver_method(solver_method_enum);
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL(logger, "Error initializing odometry: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // --- Realtime Odom State Publisher ---
  rt_odom_state_publisher_->lock();
  rt_odom_state_publisher_->msg_.header.stamp = get_node()->now();
  rt_odom_state_publisher_->msg_.header.frame_id = odom_frame_id_;
  rt_odom_state_publisher_->msg_.child_frame_id = base_frame_id_;
  rt_odom_state_publisher_->msg_.pose.pose.position.z = 0;

  constexpr size_t NUM_DIMENSIONS = 6;
  auto & pose_covariance = rt_odom_state_publisher_->msg_.pose.covariance;
  auto & twist_covariance = rt_odom_state_publisher_->msg_.twist.covariance;
  for (size_t index = 0; index < NUM_DIMENSIONS; ++index) {
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    pose_covariance[diagonal_index] = pose_covariance_diagonal_[index];
    twist_covariance[diagonal_index] = twist_covariance_diagonal_[index];
  }

  rt_odom_state_publisher_->unlock();

  // --- TF State Publisher ---
  try {
    // Tf State publisher
    tf_odom_s_publisher_ =
      get_node()->create_publisher<TfStateMsg>("/tf", rclcpp::SystemDefaultsQoS());
    rt_tf_odom_state_publisher_ = std::make_unique<TfStatePublisher>(tf_odom_s_publisher_);
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  rt_tf_odom_state_publisher_->lock();
  rt_tf_odom_state_publisher_->msg_.transforms.resize(1);
  rt_tf_odom_state_publisher_->msg_.transforms[0].header.stamp = get_node()->now();
  rt_tf_odom_state_publisher_->msg_.transforms[0].header.frame_id = odom_frame_id_;
  rt_tf_odom_state_publisher_->msg_.transforms[0].child_frame_id = base_frame_id_;
  rt_tf_odom_state_publisher_->msg_.transforms[0].transform.translation.z = 0.0;
  rt_tf_odom_state_publisher_->unlock();

  RCLCPP_DEBUG(logger, "Subscribed to %s", cmd_vel_topic_.c_str());
  RCLCPP_DEBUG(logger, "Publishing odometry to ~/odometry");

  // ***** joint commander publisher *****
  commanded_joint_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "joint_commanders", rclcpp::SystemDefaultsQoS());
  rt_commanded_joint_state_publisher_ = std::make_unique<CommandedJointStatePublisher>(
    commanded_joint_state_publisher_);
  RCLCPP_DEBUG(logger, "Publishing joint commands to /joint_commanders");
  // ***** realtime joint commander publisher *****
  if (rt_commanded_joint_state_publisher_) {
    rt_commanded_joint_state_publisher_->lock();
    auto & msg = rt_commanded_joint_state_publisher_->msg_;
    msg.name.reserve(num_modules_ * 2);
    msg.position.resize(num_modules_ * 2, std::numeric_limits<double>::quiet_NaN());
    msg.velocity.resize(num_modules_ * 2, std::numeric_limits<double>::quiet_NaN());
    // enroll the joint names
    for (size_t i = 0; i < num_modules_; ++i) {
      msg.name.push_back(steering_joint_names_[i]);
    }
    for (size_t i = 0; i < num_modules_; ++i) {
      msg.name.push_back(wheel_joint_names_[i]);
    }
    rt_commanded_joint_state_publisher_->unlock();
  }

  // ***** Visualizer *****
  if (enable_visualization_) {
    visualizer_ = std::make_unique<MarkerVisualize>();
    // get_node() returns a shared pointer to the node.
    visualization_update_time_ = get_node()->get_parameter("visualization_update_time").as_double();
    if (visualization_update_time_ < 0.0) {
      RCLCPP_WARN(
        logger,
        "visualization_update_time (%.3f) is negative. Setting to 0.0 (publish every update).",
        visualization_update_time_);
      visualization_update_time_ = 0.0;
    }
    if (get_node()) {
      last_visualization_publish_time_ = get_node()->now() - rclcpp::Duration::from_seconds(
        visualization_update_time_ + 1.0);
    } else {
      // no initialization, set to 0
      last_visualization_publish_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    visualizer_->init(
      this->get_node(), base_frame_id_, visualization_marker_topic_, num_modules_,
      visualization_update_time_);
  }

  // ---publish_limited_velocity  ---
  if (publish_limited_velocity_) {
    limited_velocity_publisher_ =
      get_node()->create_publisher<Twist>("limited_cmd_vel", rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
  }
  const Twist empty_twist;
  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);
  previoud_steering_commands_.reserve(num_modules_);
  is_rotation_direction_ = Rotation::STOP;
  RCLCPP_DEBUG(logger, "Configuration successful");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveDriveController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Activating Swerve Drive Controller...");

  // Reset internal state variables
  target_vx_ = 0.0;
  target_vy_ = 0.0;
  target_wz_ = 0.0;
  last_cmd_vel_time_ = get_node()->now();

  // --- Get and organize hardware interface handles ---
  module_handles_.clear();
  module_handles_.reserve(num_modules_);

  RCLCPP_DEBUG(get_node()->get_logger(), "Attempting to claim %zu modules.", num_modules_);
  RCLCPP_DEBUG(
    get_node()->get_logger(), "Available command interfaces (%zu):", command_interfaces_.size());
  RCLCPP_DEBUG(
    get_node()->get_logger(), "Available state interfaces (%zu):",
    state_interfaces_.size());

  for (size_t i = 0; i < num_modules_; ++i) {
    // ***** find the steering and wheel joint names according to the module index *****
    const auto & steering_joint = steering_joint_names_[i];
    const auto & wheel_joint = wheel_joint_names_[i];
    RCLCPP_DEBUG(
      get_node()->get_logger(), "Processing module %zu:"
      " Expected Steering='%s', Expected Wheel='%s'",
      i, steering_joint.c_str(), wheel_joint.c_str());
    // **********************************

    // --- Find state interface for Steering Position
    const hardware_interface::LoanedStateInterface * steering_state_pos_ptr = nullptr;
    const std::string expected_steering_state_name = steering_joint;
    const std::string expected_steering_state_if_name = HW_IF_POSITION;

    // -- Find the state interface for wheel velocity state
    const hardware_interface::LoanedStateInterface * wheel_state_vel_ptr = nullptr;

    RCLCPP_DEBUG(
      get_node()->get_logger(), "Searching for State Interface: Joint='%s', Type='%s'",
      expected_steering_state_name.c_str(), expected_steering_state_if_name.c_str());
    bool state_found = false;
    const std::string expected_position_state_name = steering_joint_names_[i] + "/" +
      HW_IF_POSITION;
    const std::string expected_speed_state_name = wheel_joint_names_[i] + "/" + HW_IF_VELOCITY;

    // Find the state interface for steering joint state
    // w.r.t position and wheel joint state w.r.t joint velocity
    for (const auto & state_if : state_interfaces_) {
      if (state_if.get_name() == expected_position_state_name) {
        steering_state_pos_ptr = &state_if;
        state_found = true;
      }
      if (state_if.get_name() == expected_speed_state_name) {
        wheel_state_vel_ptr = &state_if;
        state_found = true;
      }
    }
    if (!state_found) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "State interface '%s' not found in state_interfaces_ list during update.",
        expected_position_state_name.c_str());
      return CallbackReturn::ERROR;
    }

    // --- Find command interface (Steering Position) ---
    hardware_interface::LoanedCommandInterface * steering_cmd_pos_ptr = nullptr;
    const std::string expected_steering_cmd_name = steering_joint_names_[i] + "/" + HW_IF_POSITION;
    const std::string expected_steering_cmd_if_name = HW_IF_POSITION;

    RCLCPP_DEBUG(
      get_node()->get_logger(), "  Searching for Command Interface: Joint='%s', Type='%s'",
      expected_steering_cmd_name.c_str(), expected_steering_cmd_if_name.c_str());

    bool cmd_steering_found = false;
    for (auto & cmd_if : command_interfaces_) {
      // compare the name of the command interface with the expected name
      if (cmd_if.get_name() == expected_steering_cmd_name) {
        steering_cmd_pos_ptr = &cmd_if;
        cmd_steering_found = true;
        break;
      }
    }
    if (!cmd_steering_found) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "Command interface '%s' not found in command_interfaces_ list during update.",
        expected_steering_cmd_name.c_str());
      return CallbackReturn::ERROR;
    }

    // --- Find command interface (Wheel Velocity) ---
    hardware_interface::LoanedCommandInterface * wheel_cmd_vel_ptr = nullptr;
    const std::string expected_wheel_cmd_name = wheel_joint_names_[i] + "/" + HW_IF_VELOCITY;
    const std::string expected_wheel_cmd_if_name = HW_IF_VELOCITY;

    // Find the command interface for wheel velocity
    bool cmd_wheel_found = false;
    RCLCPP_DEBUG(
      get_node()->get_logger(), "  Searching for Command Interface: Joint='%s', Type='%s'",
      expected_wheel_cmd_name.c_str(), expected_wheel_cmd_if_name.c_str());

    for (auto & cmd_if : command_interfaces_) {
      // compare the name of the command interface with the expected name
      if (cmd_if.get_name() == expected_wheel_cmd_name) {
        wheel_cmd_vel_ptr = &cmd_if;
        cmd_wheel_found = true;
        break;
      }
    }

    if (!cmd_wheel_found) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "Command interface '%s' not found in command_interfaces_ list during update.",
        expected_wheel_cmd_name.c_str());
      // return controller_interface::return_type::ERROR;
    }

    // --- Add found handles and params to module_handles_ vector ---
    try {
      module_handles_.emplace_back(
        ModuleHandles{
          std::cref(*steering_state_pos_ptr),
          std::ref(*steering_cmd_pos_ptr),
          std::cref(*wheel_state_vel_ptr),
          std::ref(*wheel_cmd_vel_ptr),
          module_x_offsets_[i],
          module_y_offsets_[i],
          module_angle_offsets_[i],
          module_steering_limit_lower_[i],
          module_steering_limit_upper_[i]
        });
      RCLCPP_DEBUG(
        get_node()->get_logger(), "Successfully processed interfaces for module %zu.",
        i);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Exception while adding module handles for module %zu: %s", i,
        e.what());
      module_handles_.clear();
      return CallbackReturn::ERROR;
    }
  }
  // End of module loop

  // ... (Final check and Activation successful log) ...
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveDriveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivating Swerve Drive Controller...");
  // Stop the robot
  for (size_t i = 0; i < num_modules_; ++i) {
    try {
      // Use module_handles if available and initialized correctly
      if (!module_handles_.empty() && i < module_handles_.size()) {
        // Optionally set steering to current pos
        auto get_steering_val = module_handles_[i].steering_state_pos.get().get_optional();
        if (!module_handles_[i].steering_cmd_pos.get().set_value(get_steering_val.value())) {
          RCLCPP_WARN(
            get_node()->get_logger(), "Failed to set value for interface %s",
            module_handles_[i].steering_cmd_pos.get().get_name().c_str());
        }
      } else {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Module handles not available during deactivation for index %ld.",
          i);
        // Fallback (less safe)
        for (auto & iface : command_interfaces_) {
          // Check if wheel_joint_names_ is still valid and index is within bounds
          if (i < wheel_joint_names_.size() && iface.get_name() == wheel_joint_names_[i] &&
            iface.get_interface_name() == HW_IF_VELOCITY)
          {
            if (!iface.set_value(0.0)) {
              RCLCPP_WARN(
                get_node()->get_logger(), "Failed to set value for interface %s",
                iface.get_name().c_str());
            }
            break;
          }
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Error setting command interface values during deactivation for module %zu: %s", i,
        e.what());
    } catch (...) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Unknown error setting command interface values during deactivation for module %zu",
        i);
    }
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "Deactivation successful");
  return CallbackReturn::SUCCESS;
}

// helper function to normalize angles to [0, 2*pi)
double SwerveDriveController::normalize_angle(double angle_rad)
{
  // Use fmod for potentially better performance and handling edge cases
  double remainder = std::fmod(angle_rad + M_PI, 2.0 * M_PI);
  if (remainder < 0.0) {
    remainder += 2.0 * M_PI;
  }
  return remainder - M_PI;
}

// helper function to calculate the shortest angular distance
double SwerveDriveController::normalize_angle_positive(double angle)
{
  // Use fmod and add 2*PI to handle negative results correctly
  return std::fmod(std::fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

double SwerveDriveController::shortest_angular_distance(double from, double to)
{
  // Ensure angles are normalized between 0 and 2*pi for correct subtraction
  double result = normalize_angle_positive(to) - normalize_angle_positive(from);
  // Adjust the result to be in [-pi, pi]
  if (result > M_PI) {
    result -= 2.0 * M_PI;
  } else if (result < -M_PI) {
    result += 2.0 * M_PI;
  }
  return result;
}

controller_interface::return_type SwerveDriveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  double time_gap = std::max(0.001, period.seconds());

  // 1. read the latest command velocity
  auto current_cmd_vel_ptr = cmd_vel_buffer_.readFromRT();
  RCLCPP_DEBUG(
    get_node()->get_logger(), "Reading command velocity: %s",
    current_cmd_vel_ptr ? "valid" : "invalid");

  // check if the command velocity is valid
  bool timeout = false;
  // Check if ref_timeout_ is valid (non-zero duration) before calculating difference
  if (ref_timeout_.seconds() > 0.0 && (time - last_cmd_vel_time_) > ref_timeout_) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(), 100000, "time: %.3f, last_cmd_vel_time_: %.3f, ref_timeout_: %.3f",
      time.seconds(), last_cmd_vel_time_.seconds(), ref_timeout_.seconds());
    timeout = true;
  }

  if (timeout) {
    target_vx_ = 0.0;
    target_vy_ = 0.0;
    target_wz_ = 0.0;

    // Reset accumulators in odometry if timeout occurs
    RCLCPP_DEBUG_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(), 1000, "Resetting odometry accumulators due to timeout.");
    odometry_.resetAccumulators();
  } else if (current_cmd_vel_ptr && *current_cmd_vel_ptr) {
    // Valid command pointer received
    const auto & current_cmd_vel = **current_cmd_vel_ptr;

    double new_vx = current_cmd_vel.linear.x;
    double new_vy = current_cmd_vel.linear.y;
    double new_wz = current_cmd_vel.angular.z;

    if (std::abs(new_vx) < linear_vel_deadband_)
    {
      new_vx = 0.0;
    }
    if (std::abs(new_vy) < linear_vel_deadband_)
    {
      new_vy = 0.0;
    }
    if (std::abs(new_wz) < angular_vel_deadband_)
    {
      new_wz = 0.0;
    }

    if (target_vx_ != new_vx ||
      target_vy_ != new_vy ||
      target_wz_ != new_wz)
    {
      RCLCPP_DEBUG(
        get_node()->get_logger(),
        "Received new command velocity (deadband applied): vx=%.2f, vy=%.2f, wz=%.2f",
        new_vx, new_vy, new_wz);
      is_rotation_direction_ = Rotation::STOP;
    }
    target_vx_ = new_vx;
    target_vy_ = new_vy;
    target_wz_ = new_wz;

    // last_cmd_vel_time_ is updated in the callback
  }

  if (std::isnan(target_vx_) || std::isnan(target_vy_) || std::isnan(target_wz_)) {
    RCLCPP_DEBUG_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Received NaN in target velocity (vx:%.2f, vy:%.2f, wz:%.2f). Setting targets to zero.",
      target_vx_, target_vy_, target_wz_);
    target_vx_ = 0.0;
    target_vy_ = 0.0;
    target_wz_ = 0.0;
  }

  // --- 1.1 command may be limited further by SpeedLimit without affecting the stored twist command
  if (enabled_speed_limits_) {
    // T-1 command for speed limit
    Twist previous_cmd;
    // T-2 command for speed limit
    Twist pprevious_cmd;

    // Keep the size of the history to 2 for previous_commands
    if (previous_commands_.size() >= 2) {
      previous_cmd = previous_commands_.back();
      pprevious_cmd = previous_commands_.front();
    } else if (previous_commands_.size() == 1) {
      previous_cmd = previous_commands_.back();
      pprevious_cmd = previous_cmd;
    } else {
      // when previous_commands_ is empty or not initialized
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Speed limiter: Not enough previous commands in history.");
    }

    // target_vx_, target_vy_, target_wz_ is the current command velocity (before limiting)
    // SpeedLimiter limits the target velocities based on the previous command and the time period
    limiter_linear_x_.limit(
      target_vx_, previous_cmd.linear.x, pprevious_cmd.linear.x,
      time_gap);
    limiter_linear_x_.limit(
      target_vy_, previous_cmd.linear.y, pprevious_cmd.linear.y,
      time_gap);
    limiter_angular_z_.limit(
      target_wz_, previous_cmd.angular.z, pprevious_cmd.angular.z,
      time_gap);

    // remove the oldest command from the history and maintain the size of 2
    if (previous_commands_.size() >= 2) {
      previous_commands_.pop();
    }
    // set the limited command and save it to the history
    Twist current_limited_cmd_obj;
    current_limited_cmd_obj.linear.x = target_vx_;
    current_limited_cmd_obj.linear.y = target_vy_;
    current_limited_cmd_obj.angular.z = target_wz_;
    previous_commands_.emplace(current_limited_cmd_obj);

    // if publish_limited_velocity_ is true, publish the limited velocity
    if (publish_limited_velocity_ && realtime_limited_velocity_publisher_ &&
      realtime_limited_velocity_publisher_->trylock())
    {
      auto & limited_velocity_msg = realtime_limited_velocity_publisher_->msg_;
      limited_velocity_msg.linear.x = target_vx_;
      limited_velocity_msg.linear.y = target_vy_;
      limited_velocity_msg.angular.z = target_wz_;
      realtime_limited_velocity_publisher_->unlockAndPublish();
    }
  }

  // --- 2. align the steering w.r.t offset and set the value ---
  double current_steering_positions;
  std::vector<double> current_wheel_velocities;
  std::vector<double> corrected_steering_positions;

  current_wheel_velocities.reserve(num_modules_);
  corrected_steering_positions.reserve(num_modules_);
  bool all_states_read = true;
  RCLCPP_DEBUG_THROTTLE(
    get_node()->get_logger(),
    *get_node()->get_clock(), 1000, "Number of modules: %zu", num_modules_);
  for (size_t i = 0; i < num_modules_; ++i) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Reading state for module %zu", i);
    try {
      // validate module_handles_ before accessing it
      RCLCPP_DEBUG(get_node()->get_logger(), "Module handles size: %zu", module_handles_.size());
      if (module_handles_.empty() || i >= module_handles_.size()) {
        RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(),
          *get_node()->get_clock(), 1000,
          "Module handles not ready for index %zu in state reading.",
          i);
        all_states_read = false;
        break;
      }
      auto cur_steering_get_val = module_handles_[i].steering_state_pos.get().get_optional();
      auto cur_wheel_get_val = module_handles_[i].wheel_state_vel.get().get_optional();

      if (enabled_open_loop_) {
        current_steering_positions = previoud_steering_commands_[i];
      } else {
        current_steering_positions = cur_steering_get_val.value();
      }

      corrected_steering_positions.push_back(
        current_steering_positions + module_handles_[i].angle_offset);
      current_wheel_velocities.push_back(cur_wheel_get_val.value());
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(), 1000,
        "Exception reading state for module %zu during odometry update: %s", i, e.what());
      all_states_read = false;
      break;
    }
  }

  // --- 3. update the odometry ---
  if (all_states_read) {
    const double dt = time_gap;

    // Compute and store orientation info
    if (odom_source_ == "command") {
      // update the odometry using just target velocities(command), it will be not good
      if (!odometry_.update(target_vx_, target_vy_, target_wz_, dt)) {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(),
          *get_node()->get_clock(), 1000, "Odometry update failed, dt might be too small (%.6f).",
          dt);
      }
    } else if (odom_source_ == "feedback") {
      // calcuate the odometry using the kinematics from the steering and wheel velocities
      if (!odometry_.update(corrected_steering_positions, current_wheel_velocities, dt)) {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(),
          *get_node()->get_clock(), 1000, "Odometry update failed, dt might be too small (%.6f).",
          dt);
      }
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid odometry source selected.");
    }

  } else {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(), 1000, "Skipping odometry update due to state reading errors.");
  }
  RCLCPP_DEBUG(
    get_node()->get_logger(), "Computed odometry: x=%.2f, y=%.2f, theta=%.2f ",
    odometry_.getX(), odometry_.getY(), odometry_.getYaw());


  // --- 4. calculate the wheel velocities and steering angles based on the inverse kinematics ---
  bool is_steering_aligned = false;

  std::vector<double> final_steering_commands(num_modules_);
  std::vector<double> final_wheel_velocity_commands(num_modules_);
  for (size_t i = 0; i < num_modules_; ++i) {
    // Check if module_handles_ is populated correctly
    if (module_handles_.empty() || i >= module_handles_.size()) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(), 1000, "Module handles not initialized correctly for index %zu",
        i);
      continue;
    }

    const double module_x = module_handles_[i].x_offset;
    const double module_y = module_handles_[i].y_offset;
    const double angle_offset = module_handles_[i].angle_offset;
    const double limit_lower = module_handles_[i].steering_limit_lower;
    const double limit_upper = module_handles_[i].steering_limit_upper;

    // 4.1.derivate the each wheel velocity and steering angle
    double wheel_vel_x = target_vx_ - target_wz_ * module_y;
    double wheel_vel_y = target_vy_ + target_wz_ * module_x;
    double target_steering_angle_robot = std::atan2(wheel_vel_y, wheel_vel_x + 1e-9);
    double target_wheel_speed = std::sqrt(wheel_vel_x * wheel_vel_x + wheel_vel_y * wheel_vel_y);
    double target_steering_joint_angle =
      normalize_angle(target_steering_angle_robot - angle_offset);

    // 4.2. read the current steering angle from the hardware interface
    double current_steering_angle = 0.0;
    double current_wheel_velocity = 0.0;
    try {
      // Use const reference from ModuleHandles
      if (enabled_open_loop_) {
        current_steering_angle = previoud_steering_commands_[i];
      } else {
        // Read the current steering angle from the hardware interface
        auto cur_steering_get_val = module_handles_[i].steering_state_pos.get().get_optional();
        auto cur_wheel_get_val = module_handles_[i].wheel_state_vel.get().get_optional();
        current_steering_angle = cur_steering_get_val.value();
        current_wheel_velocity = cur_wheel_get_val.value();
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(), 1000,
        "Exception reading state for module %zu steering: %s", i, e.what());
      continue;
    }

    // 4.3.check if fliped steering is allowed
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "target_steering_joint_angle: %.2f, current_steering_angle: %.2f",
      target_steering_joint_angle, current_steering_angle);
    double optimized_steering_angle = target_steering_joint_angle;
    double wheel_rotation_direction = 1.0;

    if (enabled_steering_flip_) {
      double angle_diff = shortest_angular_distance(
        current_steering_angle,
        target_steering_joint_angle);

      if (std::fabs(angle_diff) > M_PI * 0.5) {
        if (normalize_angle(target_steering_joint_angle + M_PI) > limit_lower &&
          normalize_angle(target_steering_joint_angle + M_PI) < limit_upper)
        {
          // Flip the steering angle by adding M_PI
          optimized_steering_angle = normalize_angle(target_steering_joint_angle + M_PI);
          wheel_rotation_direction = -1.0;
          RCLCPP_DEBUG(
            get_node()->get_logger(),
            "Module %zu: Flipping steering angle from %.2f to %.2f (angle diff %.2f rad).",
            i, target_steering_joint_angle, optimized_steering_angle, angle_diff);
        } else {
          RCLCPP_WARN(
            get_node()->get_logger(),
            "Module %zu: Flipping steering angle would result in out-of-bounds angle %.2f.",
            i, normalize_angle(target_steering_joint_angle + M_PI));
        }
      } else {
        if (target_steering_joint_angle < limit_lower ||
          target_steering_joint_angle > limit_upper)
        {
          if (normalize_angle(target_steering_joint_angle + M_PI) > limit_lower &&
            normalize_angle(target_steering_joint_angle + M_PI) < limit_upper)
          {
            // Flip the steering angle by adding M_PI
            optimized_steering_angle = normalize_angle(target_steering_joint_angle + M_PI);
            wheel_rotation_direction = -1.0;
          }
        }
      }
    }

    // 4.4. check if the target steering angle is within the limits
    bool is_no_limitation = false;
    double limited_steering_cmd = optimized_steering_angle;
    if (limit_lower <= -M_PI && limit_upper >= M_PI) {
      // Special case: limits cross -pi/pi boundary
      is_no_limitation = true;
    } else if (limited_steering_cmd < limit_lower || limited_steering_cmd > limit_upper) {
      // Normal case
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Module %zu: Target steering angle %.2f outside limits [%.2f, %.2f]. Stopping.",
        i, limited_steering_cmd, limit_lower, limit_upper);
      return controller_interface::return_type::ERROR;
    }

    // 4.5. limit the steering angular velocity
    if (enabled_steering_angular_velocity_limit_) {
      if (steering_angular_velocity_limit_ < std::numeric_limits<double>::max() &&
        steering_angular_velocity_limit_ > 0.0)
      {
        double max_allowed_steering_change_this_dt = steering_angular_velocity_limit_ *
          time_gap;

        if (is_in_alignment_mode_)
        {
            target_wheel_speed = 0.0;

            optimized_steering_angle = alignment_target_angle_;

            double current_error = shortest_angular_distance(current_steering_angle, alignment_target_angle_);
            const double ALIGNMENT_TOLERANCE =0.1;
            if (std::abs(current_error) < ALIGNMENT_TOLERANCE)
            {
                RCLCPP_INFO(get_node()->get_logger(), "Steering alignment complete. Resuming normal operation.");
                is_in_alignment_mode_ = false;
            }

        }
        else{
          if (is_no_limitation) {
            // If no limitation, just use the optimized steering angle

            double absolute_current_steering_angle = std::fmod(current_steering_angle, 2.0 * M_PI);

            double desired_steering_change_rad =
              shortest_angular_distance(
              absolute_current_steering_angle,
              limited_steering_cmd);
            double actual_steering_change_this_dt = std::clamp(
              desired_steering_change_rad,
              -max_allowed_steering_change_this_dt,
              max_allowed_steering_change_this_dt
            );

            // If the desired steering change is larger than the maximum allowed change,
            optimized_steering_angle = current_steering_angle + actual_steering_change_this_dt;

            if (optimized_steering_angle >= limit_upper || optimized_steering_angle <= limit_lower)
            {
              RCLCPP_WARN(
                get_node()->get_logger(),
                "Steering angle (%.2f) exceeds limits [%.2f, %.2f]. Stopping wheel and adjusting angle.",
                optimized_steering_angle, limit_lower, limit_upper);

              target_wheel_speed = 0.0;

              if (optimized_steering_angle > limit_upper) {
                optimized_steering_angle -= M_PI;
              } else {
                optimized_steering_angle += M_PI;
              }
              is_in_alignment_mode_ = true;
            }
          } else {
            double desired_steering_change_rad = shortest_angular_distance(
              current_steering_angle,
              limited_steering_cmd);

            double actual_steering_change_this_dt = std::clamp(
              desired_steering_change_rad,
              -max_allowed_steering_change_this_dt,
              max_allowed_steering_change_this_dt
            );

            optimized_steering_angle = normalize_angle(
              current_steering_angle + actual_steering_change_this_dt);
          }
        }

      }
    }

    // 4.6. calculate the final wheel velocity command
    double final_wheel_vel_cmd = wheel_rotation_direction * target_wheel_speed / wheel_radius_;

    // 4.7. save the commands in order to send the hardware interface
    // and stop the wheel when steering is not aligned
    try {
      // limit the wheel velocity command
      // until current steering angle is close to the target steering angle
      if (fabs(current_wheel_velocity) >= steering_alignment_start_speed_error_threshold_) {
        if (steering_alignment_angle_error_threshold_ <=
          fabs(shortest_angular_distance(current_steering_angle, limited_steering_cmd)))
        {
          // when current steering angle is not close to the target steering angle, stop the wheel
          final_wheel_vel_cmd = 0.0;
          is_steering_aligned = false;
        } else {
          is_steering_aligned = true;
        }
      } else {
        // when current wheel velocity is low, we can align the steering angle
        if (steering_alignment_start_angle_error_threshold_ <=
          fabs(shortest_angular_distance(current_steering_angle, limited_steering_cmd)))
        {
          // when current steering angle is not close to the target steering angle, stop the wheel
          final_wheel_vel_cmd = 0.0;
          is_steering_aligned = false;
        } else {
          is_steering_aligned = true;
        }
      }

      // Limit the wheel velocity command
      if (final_wheel_vel_cmd < module_wheel_speed_limit_lower_[i] ||
        final_wheel_vel_cmd > module_wheel_speed_limit_upper_[i])
      {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Module %zu: Wheel velocity command %.2f outside limits [%.2f, %.2f]. Clamping.",
          i, final_wheel_vel_cmd, module_wheel_speed_limit_lower_[i],
          module_wheel_speed_limit_upper_[i]);

        double clipped_wheel_vel_cmd;
        clipped_wheel_vel_cmd = std::clamp(
          final_wheel_vel_cmd,
          module_wheel_speed_limit_lower_[i],
          module_wheel_speed_limit_upper_[i]);

        if (enabled_wheel_saturation_scaling_) {
          wheel_saturation_scale_factor_ = std::min(
            wheel_saturation_scale_factor_,
            clipped_wheel_vel_cmd / final_wheel_vel_cmd);
          RCLCPP_WARN(
            get_node()->get_logger(),
            "min scale factor: %.2f, final_wheel_vel_cmd: %.2f, clipped_wheel_vel_cmd: %.2f",
            wheel_saturation_scale_factor_, final_wheel_vel_cmd, clipped_wheel_vel_cmd);
        } else {
          wheel_saturation_scale_factor_ = 1.0;
        }
      }

      RCLCPP_DEBUG(
        get_node()->get_logger(),
        "Calculated Module %zu: Steering command %.2f, Wheel command %.2f",
        i, optimized_steering_angle, final_wheel_vel_cmd);

      // joints commands
      final_steering_commands[i] = optimized_steering_angle;
      final_wheel_velocity_commands[i] = final_wheel_vel_cmd;
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(), 1000,
        "Exception writing commands for module %zu: %s", i, e.what());
      try {
        // Attempt to set safe values
        if (!module_handles_[i].steering_cmd_pos.get().set_value(current_steering_angle)) {
          RCLCPP_WARN(
            get_node()->get_logger(), "Failed to set value for interface %s",
            module_handles_[i].steering_cmd_pos.get().get_name().c_str());
        }
        if (!module_handles_[i].wheel_cmd_vel.get().set_value(0.0)) {
          RCLCPP_WARN(
            get_node()->get_logger(), "Failed to set value for interface %s",
            module_handles_[i].wheel_cmd_vel.get().get_name().c_str());
        }

        // joints commands
        final_steering_commands[i] = current_steering_angle;
        final_wheel_velocity_commands[i] = 0.0;
      } catch (...) {
      }
      continue;
    } catch (...) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Unknown exception writing commands for module %zu", i);
      continue;
    }
  }
  // End of module loop

  // --- 5. send the commands to the hardware interface ---
  if (target_vx_ == 0.00 && target_vy_ == 0.00 && target_wz_ == 0.00) {
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "Robot is halted. Commanding zero wheel velocity and holding current steering.");
    for (size_t i = 0; i < num_modules_; ++i) {
      if (module_handles_.empty() || i >= module_handles_.size()) {continue;}
      double current_steering_angle_for_hold = 0.0;
      try {
        auto cur_steering_get_val = module_handles_[i].steering_state_pos.get().get_optional();
        current_steering_angle_for_hold = cur_steering_get_val.value();

        if (!module_handles_[i].steering_cmd_pos.get().set_value(current_steering_angle_for_hold)) {
          RCLCPP_WARN(
            get_node()->get_logger(), "Failed to set value for interface %s",
            module_handles_[i].steering_cmd_pos.get().get_name().c_str());
        }

        if (!module_handles_[i].wheel_cmd_vel.get().set_value(0.0)) {
          RCLCPP_WARN(
            get_node()->get_logger(), "Failed to set value for interface %s",
            module_handles_[i].wheel_cmd_vel.get().get_name().c_str());
        }

        if (i < final_steering_commands.size()) {
          final_steering_commands[i] = current_steering_angle_for_hold;
        }
        if (i < final_wheel_velocity_commands.size()) {
          final_wheel_velocity_commands[i] = 0.0;
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(),
          *get_node()->get_clock(),
          1000,
          "Exception during halted command writing for module %zu: %s", i, e.what());
      }
    }
  } else {
    // Update the final commands
    for (size_t i = 0; i < num_modules_; ++i) {
      RCLCPP_DEBUG(
        get_node()->get_logger(), "Command Module %zu: Steering command %.2f, Wheel command %.2f",
        i, final_steering_commands[i],
        final_wheel_velocity_commands[i] * wheel_saturation_scale_factor_);

      // Set the steering and wheel commands
      if (!module_handles_[i].steering_cmd_pos.get().set_value(final_steering_commands[i])) {
        RCLCPP_WARN(
          get_node()->get_logger(), "Failed to set value for interface %s",
          module_handles_[i].steering_cmd_pos.get().get_name().c_str());
      }
      previoud_steering_commands_[i] = final_steering_commands[i];
      if (is_steering_aligned) {
        if (!module_handles_[i].wheel_cmd_vel.get().set_value(
            final_wheel_velocity_commands[i] *
            wheel_saturation_scale_factor_))
        {
          RCLCPP_WARN(
            get_node()->get_logger(), "Failed to set value for interface %s",
            module_handles_[i].wheel_cmd_vel.get().get_name().c_str());
        }
      } else {
        // If steering is not aligned, stop the wheel
        if (!module_handles_[i].wheel_cmd_vel.get().set_value(0.0)) {
          RCLCPP_WARN(
            get_node()->get_logger(), "Failed to set value for interface %s",
            module_handles_[i].wheel_cmd_vel.get().get_name().c_str());
        }
        RCLCPP_DEBUG(
          get_node()->get_logger(),
          "Steering not aligned, stopping wheel. final_steering_commands: %f",
          final_steering_commands[i]);
      }
    }
  }
  wheel_saturation_scale_factor_ = 1.0;

  // --- 6. publish odometry message and TF and joint commadns and marker visualization---
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getYaw());
  if (all_states_read && rt_odom_state_publisher_ && rt_odom_state_publisher_->trylock()) {
    rt_odom_state_publisher_->msg_.header.stamp = time;
    rt_odom_state_publisher_->msg_.pose.pose.position.x = odometry_.getX();
    rt_odom_state_publisher_->msg_.pose.pose.position.y = odometry_.getY();
    rt_odom_state_publisher_->msg_.pose.pose.orientation = tf2::toMsg(orientation);
    rt_odom_state_publisher_->msg_.twist.twist.linear.x = odometry_.getVx();
    rt_odom_state_publisher_->msg_.twist.twist.linear.y = odometry_.getVy();
    rt_odom_state_publisher_->msg_.twist.twist.angular.z = odometry_.getWz();
    rt_odom_state_publisher_->unlockAndPublish();
  }

  // Publish tf /odom frame
  if (enable_odom_tf_ && rt_tf_odom_state_publisher_->trylock()) {
    rt_tf_odom_state_publisher_->msg_.transforms.front().header.stamp = time;
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.translation.x = odometry_.getX();
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.translation.y = odometry_.getY();
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.rotation =
      tf2::toMsg(orientation);
    rt_tf_odom_state_publisher_->unlockAndPublish();
  }

  // Publish joint commands in order to compare the actual joint states
  if (rt_commanded_joint_state_publisher_ && rt_commanded_joint_state_publisher_->trylock()) {
    auto & msg = rt_commanded_joint_state_publisher_->msg_;
    msg.header.stamp = time;

    for (size_t i = 0; i < num_modules_; ++i) {
      msg.position[i] = final_steering_commands[i];
      msg.velocity[i] = std::numeric_limits<double>::quiet_NaN();

      msg.position[i + num_modules_] = std::numeric_limits<double>::quiet_NaN();
      msg.velocity[i + num_modules_] = final_wheel_velocity_commands[i];
    }
    rt_commanded_joint_state_publisher_->unlockAndPublish();
  }

  // publish visualization markers
  if (enable_visualization_ && visualizer_) {
    if ((time - last_visualization_publish_time_).seconds() >= visualization_update_time_) {
      std::vector<double> robot_frame_steering_angles_for_viz(num_modules_);
      std::vector<double> wheel_linear_vels_for_viz(num_modules_);

      for (size_t i = 0; i < num_modules_; ++i) {
        // set the steering angles and wheel velocities for visualization considering the offsets
        if (i < final_steering_commands.size() && i < module_angle_offsets_.size()) {
          robot_frame_steering_angles_for_viz[i] = (
            final_steering_commands[i] + module_angle_offsets_[i]);
        } else {
          robot_frame_steering_angles_for_viz[i] = 0.0;
        }
        if (i < final_wheel_velocity_commands.size()) {
          wheel_linear_vels_for_viz[i] = final_wheel_velocity_commands[i] * wheel_radius_;
        } else {
          wheel_linear_vels_for_viz[i] = 0.0;
        }
      }

      if (robot_frame_steering_angles_for_viz.size() == num_modules_) {
        visualizer_->publish_markers(
          time,
          target_vx_, target_vy_, target_wz_,
          module_x_offsets_, module_y_offsets_,
          robot_frame_steering_angles_for_viz,
          wheel_linear_vels_for_viz
        );
        last_visualization_publish_time_ = time;
      } else {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(),
          *get_node()->get_clock(), 1000,
          "command/steering vectors not ready or size mismatch for publishing.");
      }
    }
  }

  return controller_interface::return_type::OK;
}

// reference_callback (Corrected type and logic)
void SwerveDriveController::reference_callback(const std::shared_ptr<CmdVelMsg> msg)
{
  // Directly store the Twist message shared pointer
  last_cmd_vel_time_ = this->get_node()->now();
  // Only write if not halted (prevents buffer filling while stopped)
  cmd_vel_buffer_.writeFromNonRT(msg);

  RCLCPP_DEBUG(
    get_node()->get_logger(), "Received new command: vx=%.2f, vy=%.2f, wz=%.2f",
    msg->linear.x, msg->linear.y, msg->angular.z);
  // Timeout logic removed from here, handled in update
}

}  // namespace ffw_swerve_drive_controller

// Pluginlib export macro
#include "pluginlib/class_list_macros.hpp"

// Export the controller class as a plugin
PLUGINLIB_EXPORT_CLASS(
  ffw_swerve_drive_controller::SwerveDriveController,
  controller_interface::ControllerInterface)
