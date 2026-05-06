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
  try {
    // ***** declare parameters *****
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
    auto_declare<bool>("enabled_open_loop", false);
    auto_declare<bool>("enabled_steering_angular_velocity_limit", true);
    auto_declare<double>("steering_angular_velocity_limit", 1.0);
    auto_declare<double>("steering_alignment_angle_error_threshold", 0.1);
    auto_declare<double>("steering_alignment_start_angle_error_threshold", 0.1);
    auto_declare<double>("steering_alignment_start_speed_error_threshold", 0.1);
    auto_declare<double>("linear_vel_deadband", 0.1);
    auto_declare<double>("angular_vel_deadband", 0.1);

    auto_declare<std::string>("cmd_vel_topic", "/cmd_vel");
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
      get_node()->get_logger(), "Parameter declaration failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
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

  // update parameters if they have changed
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
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
    visualization_update_time_ =
      get_node()->get_parameter("visualization_update_time").as_double();
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
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception during TF publisher creation at configure: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  rt_tf_odom_state_publisher_->lock();
  rt_tf_odom_state_publisher_->msg_.transforms.resize(1);
  rt_tf_odom_state_publisher_->msg_.transforms[0].header.stamp = get_node()->now();
  rt_tf_odom_state_publisher_->msg_.transforms[0].header.frame_id = odom_frame_id_;
  rt_tf_odom_state_publisher_->msg_.transforms[0].child_frame_id = base_frame_id_;
  rt_tf_odom_state_publisher_->msg_.transforms[0].transform.translation.z = 0.0;
  rt_tf_odom_state_publisher_->unlock();

  // ***** joint commander publisher *****
  commanded_joint_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "joint_commanders", rclcpp::SystemDefaultsQoS());
  rt_commanded_joint_state_publisher_ = std::make_unique<CommandedJointStatePublisher>(
    commanded_joint_state_publisher_);
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
    visualization_update_time_ = std::max(0.0, visualization_update_time_);
    last_visualization_publish_time_ = get_node()->now() - rclcpp::Duration::from_seconds(
      visualization_update_time_ + 1.0);
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
  cmd_velocity_history_[0] = empty_twist;
  cmd_velocity_history_[1] = empty_twist;
  cmd_velocity_history_len_ = 2;
  previoud_steering_commands_.reserve(num_modules_);

  // Initialize 180° Rule smooth reversal state tracking
  reversal_phase_.resize(num_modules_, ReversalPhase::NORMAL);
  previous_wheel_rotation_direction_.resize(num_modules_, 1.0);
  wheel_speed_scale_.resize(num_modules_, 1.0);
  reversal_target_steering_angle_.resize(num_modules_, 0.0);

  // Pre-allocate vectors for update() loop to avoid real-time heap allocation
  current_wheel_velocities_.resize(num_modules_, 0.0);
  corrected_steering_positions_.resize(num_modules_, 0.0);
  final_steering_commands_.resize(num_modules_, 0.0);
  final_wheel_velocity_commands_.resize(num_modules_, 0.0);
  robot_frame_steering_angles_for_viz_.resize(num_modules_, 0.0);
  wheel_linear_vels_for_viz_.resize(num_modules_, 0.0);
  previoud_steering_commands_.resize(num_modules_, 0.0);

  RCLCPP_DEBUG(
    logger, "Configuration complete: %zu modules, wheel_radius=%.3f",
    num_modules_, wheel_radius_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveDriveController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset internal state variables
  target_vx_ = 0.0;
  target_vy_ = 0.0;
  target_wz_ = 0.0;
  last_cmd_vel_time_ = get_node()->now();

  // Reset 180° Rule smooth reversal state
  for (size_t i = 0; i < num_modules_; ++i) {
    reversal_phase_[i] = ReversalPhase::NORMAL;
    previous_wheel_rotation_direction_[i] = 1.0;
    wheel_speed_scale_[i] = 1.0;
    reversal_target_steering_angle_[i] = 0.0;
  }

  // --- Get and organize hardware interface handles ---
  module_handles_.clear();
  module_handles_.reserve(num_modules_);

  for (size_t i = 0; i < num_modules_; ++i) {
    const auto & steering_joint = steering_joint_names_[i];
    const auto & wheel_joint = wheel_joint_names_[i];

    // --- Find state interface for Steering Position
    const hardware_interface::LoanedStateInterface * steering_state_pos_ptr = nullptr;
    const std::string expected_steering_state_name = steering_joint;
    const std::string expected_steering_state_if_name = HW_IF_POSITION;

    // -- Find the state interface for wheel velocity state
    const hardware_interface::LoanedStateInterface * wheel_state_vel_ptr = nullptr;
    const std::string expected_position_state_name =
      steering_joint_names_[i] + "/" + HW_IF_POSITION;
    const std::string expected_speed_state_name = wheel_joint_names_[i] + "/" + HW_IF_VELOCITY;

    // Find state interfaces for steering position and wheel velocity
    for (const auto & state_if : state_interfaces_) {
      const auto & name = state_if.get_name();
      if (name == expected_position_state_name) {
        steering_state_pos_ptr = &state_if;
      } else if (name == expected_speed_state_name) {
        wheel_state_vel_ptr = &state_if;
      }
    }

    if (!steering_state_pos_ptr || !wheel_state_vel_ptr) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "State interface not found for module %zu (steering: %s, wheel: %s)",
        i, steering_state_pos_ptr ? "OK" : "MISSING", wheel_state_vel_ptr ? "OK" : "MISSING");
      return CallbackReturn::ERROR;
    }

    // --- Find command interfaces ---
    hardware_interface::LoanedCommandInterface * steering_cmd_pos_ptr = nullptr;
    hardware_interface::LoanedCommandInterface * wheel_cmd_vel_ptr = nullptr;
    const std::string expected_steering_cmd_name = steering_joint_names_[i] + "/" + HW_IF_POSITION;
    const std::string expected_wheel_cmd_name = wheel_joint_names_[i] + "/" + HW_IF_VELOCITY;

    for (auto & cmd_if : command_interfaces_) {
      const auto & name = cmd_if.get_name();
      if (name == expected_steering_cmd_name) {
        steering_cmd_pos_ptr = &cmd_if;
      } else if (name == expected_wheel_cmd_name) {
        wheel_cmd_vel_ptr = &cmd_if;
      }
    }

    if (!steering_cmd_pos_ptr || !wheel_cmd_vel_ptr) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Command interface not found for module %zu (steering: %s, wheel: %s)",
        i, steering_cmd_pos_ptr ? "OK" : "MISSING", wheel_cmd_vel_ptr ? "OK" : "MISSING");
      return CallbackReturn::ERROR;
    }

    // --- Add found handles and params to module_handles_ vector ---
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
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "Activation successful with %zu modules", num_modules_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveDriveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop the robot - hold current steering position
  if (!module_handles_.empty()) {
    for (size_t i = 0; i < std::min(num_modules_, module_handles_.size()); ++i) {
      try {
        auto steering_val = module_handles_[i].steering_state_pos.get().get_optional();
        module_handles_[i].steering_cmd_pos.get().set_value(steering_val.value());
        module_handles_[i].wheel_cmd_vel.get().set_value(0.0);
      } catch (const std::exception & e) {
        RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Deactivation error for module %zu: %s", i, e.what());
      }
    }
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "Deactivation complete");
  return CallbackReturn::SUCCESS;
}

// helper function to normalize angles to [-π, +π]
double SwerveDriveController::normalize_angle(double angle_rad)
{
  double remainder = std::fmod(angle_rad + M_PI, kTwoPi);
  return (remainder < 0.0 ? remainder + kTwoPi : remainder) - M_PI;
}

// helper function to normalize angles to [0, 2π)
double SwerveDriveController::normalize_angle_positive(double angle)
{
  double result = std::fmod(angle, kTwoPi);
  return result < 0.0 ? result + kTwoPi : result;
}

// helper function to calculate the shortest angular distance
double SwerveDriveController::shortest_angular_distance(double from, double to)
{
  double result = normalize_angle_positive(to) - normalize_angle_positive(from);
  if (result > M_PI) {
    return result - kTwoPi;
  } else if (result < -M_PI) {
    return result + kTwoPi;
  }
  return result;
}

controller_interface::return_type SwerveDriveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  double time_gap = std::max(0.001, period.seconds());

  // 1. read the latest command velocity
  auto current_cmd_vel_ptr = cmd_vel_buffer_.readFromRT();

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
    odometry_.resetAccumulators();
  } else if (current_cmd_vel_ptr && *current_cmd_vel_ptr) {
    // Valid command pointer received
    const auto & current_cmd_vel = **current_cmd_vel_ptr;

    double new_vx = current_cmd_vel.linear.x;
    double new_vy = current_cmd_vel.linear.y;
    double new_wz = current_cmd_vel.angular.z;

    if (std::abs(new_vx) < linear_vel_deadband_) {
      new_vx = 0.0;
    }
    if (std::abs(new_vy) < linear_vel_deadband_) {
      new_vy = 0.0;
    }
    if (std::abs(new_wz) < angular_vel_deadband_) {
      new_wz = 0.0;
    }

    target_vx_ = new_vx;
    target_vy_ = new_vy;
    target_wz_ = new_wz;
  }

  if (std::isnan(target_vx_) || std::isnan(target_vy_) || std::isnan(target_wz_)) {
    target_vx_ = 0.0;
    target_vy_ = 0.0;
    target_wz_ = 0.0;
  }

  // --- 1.1 command may be limited further by SpeedLimit without affecting
  //     the stored twist command
  if (enabled_speed_limits_) {
    Twist previous_cmd;
    Twist pprevious_cmd;

    if (cmd_velocity_history_len_ >= 2) {
      pprevious_cmd = cmd_velocity_history_[0];
      previous_cmd = cmd_velocity_history_[1];
    } else if (cmd_velocity_history_len_ == 1) {
      previous_cmd = cmd_velocity_history_[0];
      pprevious_cmd = previous_cmd;
    } else {
      previous_cmd = Twist();
      pprevious_cmd = Twist();
    }

    // target_vx_, target_vy_, target_wz_ is the current command velocity (before limiting)
    // SpeedLimiter limits the target velocities based on the previous command and the time period
    limiter_linear_x_.limit(
      target_vx_, previous_cmd.linear.x, pprevious_cmd.linear.x,
      time_gap);
    limiter_linear_y_.limit(
      target_vy_, previous_cmd.linear.y, pprevious_cmd.linear.y,
      time_gap);
    limiter_angular_z_.limit(
      target_wz_, previous_cmd.angular.z, pprevious_cmd.angular.z,
      time_gap);

    Twist current_limited_cmd_obj;
    current_limited_cmd_obj.linear.x = target_vx_;
    current_limited_cmd_obj.linear.y = target_vy_;
    current_limited_cmd_obj.angular.z = target_wz_;

    if (cmd_velocity_history_len_ >= 2) {
      cmd_velocity_history_[0] = cmd_velocity_history_[1];
      cmd_velocity_history_[1] = current_limited_cmd_obj;
    } else if (cmd_velocity_history_len_ == 1) {
      cmd_velocity_history_[1] = current_limited_cmd_obj;
      cmd_velocity_history_len_ = 2;
    } else {
      cmd_velocity_history_[0] = current_limited_cmd_obj;
      cmd_velocity_history_len_ = 1;
    }

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
  // Early validation: check module_handles_ once instead of per iteration
  const size_t handle_count = module_handles_.size();
  if (handle_count != num_modules_) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Module handles count (%zu) doesn't match num_modules (%zu)", handle_count, num_modules_);
    return controller_interface::return_type::ERROR;
  }

  bool all_states_read = true;
  for (size_t i = 0; i < num_modules_; ++i) {
    try {
      const auto & handle = module_handles_[i];
      auto cur_steering_get_val = handle.steering_state_pos.get().get_optional();
      auto cur_wheel_get_val = handle.wheel_state_vel.get().get_optional();

      double current_steering_pos = enabled_open_loop_ ?
        previoud_steering_commands_[i] : cur_steering_get_val.value();

      corrected_steering_positions_[i] = current_steering_pos + handle.angle_offset;
      current_wheel_velocities_[i] = cur_wheel_get_val.value();
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Exception reading state for module %zu: %s", i, e.what());
      all_states_read = false;
      break;
    }
  }

  // --- 3. update the odometry ---
  if (all_states_read) {
    bool odom_ok = (odom_source_ == "command") ?
      odometry_.update(target_vx_, target_vy_, target_wz_, time_gap) :
      odometry_.update(corrected_steering_positions_, current_wheel_velocities_, time_gap);

    if (!odom_ok) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "Odometry update failed (dt=%.6f)", time_gap);
    }
  }

  // --- 4. calculate the wheel velocities and steering angles based on the inverse kinematics ---
  // All modules must pass alignment gating before any wheel torque is applied from §4.6.
  bool all_steering_aligned = true;

  const bool cmd_velocity_all_zero =
    (target_vx_ == 0.0 && target_vy_ == 0.0 && target_wz_ == 0.0);
  if (cmd_velocity_all_zero) {
    for (size_t i = 0; i < num_modules_; ++i) {
      reversal_phase_[i] = ReversalPhase::NORMAL;
      wheel_speed_scale_[i] = 1.0;
    }
  }

  for (size_t i = 0; i < num_modules_; ++i) {
    const auto & handle = module_handles_[i];
    const double module_x = handle.x_offset;
    const double module_y = handle.y_offset;
    const double angle_offset = handle.angle_offset;

    // 4.1. Compute wheel velocity vector and target steering angle
    const double wheel_vel_x = target_vx_ - target_wz_ * module_y;
    const double wheel_vel_y = target_vy_ + target_wz_ * module_x;
    const double target_steering_angle_robot = std::atan2(wheel_vel_y, wheel_vel_x + kEpsilon);
    const double target_wheel_speed = std::hypot(wheel_vel_x, wheel_vel_y);
    const double target_steering_joint_angle =
      normalize_angle(target_steering_angle_robot - angle_offset);

    // 4.2. Current joint state: reuse §2 buffers when the full read succeeded
    //     (no duplicate HW read).
    double current_steering_angle = 0.0;
    double current_wheel_velocity = 0.0;
    if (all_states_read) {
      current_wheel_velocity = current_wheel_velocities_[i];
      current_steering_angle = enabled_open_loop_ ?
        previoud_steering_commands_[i] :
        (corrected_steering_positions_[i] - angle_offset);
    } else {
      try {
        auto cur_steering_get_val = handle.steering_state_pos.get().get_optional();
        auto cur_wheel_get_val = handle.wheel_state_vel.get().get_optional();
        current_steering_angle = cur_steering_get_val.value();
        current_wheel_velocity = cur_wheel_get_val.value();
      } catch (const std::exception & e) {
        RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(),
          *get_node()->get_clock(), 1000,
          "Exception reading state for module %zu steering: %s", i, e.what());
        final_steering_commands_[i] = previoud_steering_commands_[i];
        final_wheel_velocity_commands_[i] = 0.0;
        all_steering_aligned = false;
        continue;
      }
    }

    // 4.3. Apply 180° Rule (Steering Flip Optimization)
    // Instead of turning 270°, turn -90° and reverse the drive motor direction
    double optimized_steering_angle = target_steering_joint_angle;
    double wheel_rotation_direction = 1.0;

    // Calculate the shortest angular distance from current to target
    const double angle_diff = shortest_angular_distance(
      current_steering_angle, target_steering_joint_angle);

    // If rotation would be more than 90°, flip the steering and reverse motor
    if (std::fabs(angle_diff) > kPiHalf) {
      optimized_steering_angle = normalize_angle(target_steering_joint_angle + M_PI);
      wheel_rotation_direction = -1.0;
    }

    // 4.3.1. Handle mechanical steering limit at ±π (±180°)
    // If the shortest path would cross this boundary, flip the steering instead
    {
      const double angle_diff_after_opt = shortest_angular_distance(
        current_steering_angle, optimized_steering_angle);

      // Check if path crosses ±π boundary
      const bool crosses_boundary =
        (current_steering_angle > 0 && optimized_steering_angle < 0 && angle_diff_after_opt > 0) ||
        (current_steering_angle < 0 && optimized_steering_angle > 0 && angle_diff_after_opt < 0);

      if (crosses_boundary) {
        optimized_steering_angle = normalize_angle(optimized_steering_angle + M_PI);
        wheel_rotation_direction *= -1.0;
      }
    }

    // 4.3.2. Smooth direction reversal sequence: DECEL → STEERING → ACCEL
    // When cmd is all zeros, IK uses atan2(0, ε)→0 and hypot→0 (degenerate). That is not a real
    // direction change vs in-place spin — do not run reversal FSM or it captures reversal_target≈0
    // and steers wheels "forward" on the next non-zero command.
    bool direction_changed = false;
    if (!cmd_velocity_all_zero) {
      direction_changed =
        (wheel_rotation_direction != previous_wheel_rotation_direction_[i]);

      // Start reversal sequence when direction changes
      if (direction_changed && reversal_phase_[i] == ReversalPhase::NORMAL) {
        reversal_phase_[i] = ReversalPhase::DECELERATING;
        reversal_target_steering_angle_[i] = optimized_steering_angle;
      }
    }

    // 4.4. Wrap-around steering angle to [-π, +π] range (-180° ~ +180°)
    double limited_steering_cmd = normalize_angle(optimized_steering_angle);

    // Determine steering command based on reversal phase
    double steering_target_for_this_cycle = limited_steering_cmd;

    if (!cmd_velocity_all_zero) {
      switch (reversal_phase_[i]) {
        case ReversalPhase::DECELERATING:
          steering_target_for_this_cycle = current_steering_angle;
          wheel_speed_scale_[i] -= kReversalDecelRate * time_gap;
          if (wheel_speed_scale_[i] <= kReversalThreshold) {
            wheel_speed_scale_[i] = 0.0;
            // Re-latch steering target from current IK (avoids stale snapshot from DECEL start).
            reversal_target_steering_angle_[i] = limited_steering_cmd;
            reversal_phase_[i] = ReversalPhase::STEERING;
          }
          break;

        case ReversalPhase::STEERING:
          // Track live IK every frame so reversal target cannot stay stale across STEERING.
          reversal_target_steering_angle_[i] = limited_steering_cmd;
          steering_target_for_this_cycle = limited_steering_cmd;
          wheel_speed_scale_[i] = 0.0;
          {
            const double steering_error = std::fabs(
              shortest_angular_distance(
                current_steering_angle, limited_steering_cmd));
            if (steering_error < kSteeringTolerance) {
              previous_wheel_rotation_direction_[i] = wheel_rotation_direction;
              reversal_phase_[i] = ReversalPhase::ACCELERATING;
            }
          }
          break;

        case ReversalPhase::ACCELERATING:
          wheel_speed_scale_[i] += kReversalAccelRate * time_gap;
          if (wheel_speed_scale_[i] >= 1.0) {
            wheel_speed_scale_[i] = 1.0;
            reversal_phase_[i] = ReversalPhase::NORMAL;
          }
          break;

        default:
          wheel_speed_scale_[i] = 1.0;
          break;
      }
    }

    wheel_speed_scale_[i] = std::clamp(wheel_speed_scale_[i], 0.0, 1.0);

    // 4.5. Apply steering angular velocity limit for smooth control
    if (enabled_steering_angular_velocity_limit_) {
      double effective_steering_vel_limit = steering_angular_velocity_limit_;
      if (effective_steering_vel_limit <= 0.0 ||
        effective_steering_vel_limit >= std::numeric_limits<double>::max())
      {
        effective_steering_vel_limit = 1.0;  // Default: 1.0 rad/s
      }

      const double max_change = effective_steering_vel_limit * time_gap;
      const double desired_change = shortest_angular_distance(
        current_steering_angle, steering_target_for_this_cycle);

      // If we can reach target in this cycle, use target directly (avoid normalize jump)
      if (std::abs(desired_change) <= max_change) {
        optimized_steering_angle = steering_target_for_this_cycle;
      } else {
        // Move max_change toward target direction
        const double direction = (desired_change > 0) ? 1.0 : -1.0;
        const double new_angle = current_steering_angle + direction * max_change;

        // Normalize but check for ±π boundary jump
        const double normalized = normalize_angle(new_angle);
        const double jump_check = std::abs(normalized - current_steering_angle);

        // If normalize caused a large jump (crossing ±π boundary), use raw value clamped
        if (jump_check > M_PI) {
          // Clamp to ±π instead of jumping
          optimized_steering_angle = (new_angle > 0) ? M_PI : -M_PI;
        } else {
          optimized_steering_angle = normalized;
        }
      }
    } else {
      // No velocity limit - directly use target angle
      optimized_steering_angle = steering_target_for_this_cycle;
    }

    // 4.6. Calculate the final wheel velocity command
    // Use previous direction during DECEL phase, new direction after STEERING phase
    double effective_direction = (reversal_phase_[i] == ReversalPhase::DECELERATING) ?
      previous_wheel_rotation_direction_[i] : wheel_rotation_direction;
    double final_wheel_vel_cmd = effective_direction * target_wheel_speed *
      wheel_speed_scale_[i] / wheel_radius_;

    // 4.7. save the commands in order to send the hardware interface
    // and stop the wheel when steering is not aligned
    try {
      // limit the wheel velocity command
      // until current steering angle is close to the target steering angle
      const double align_err =
        std::fabs(shortest_angular_distance(current_steering_angle, limited_steering_cmd));
      bool module_steering_aligned = true;
      if (std::fabs(current_wheel_velocity) >= steering_alignment_start_speed_error_threshold_) {
        if (steering_alignment_angle_error_threshold_ <= align_err) {
          final_wheel_vel_cmd = 0.0;
          module_steering_aligned = false;
        }
      } else if (steering_alignment_start_angle_error_threshold_ <= align_err) {
        final_wheel_vel_cmd = 0.0;
        module_steering_aligned = false;
      }
      all_steering_aligned = all_steering_aligned && module_steering_aligned;

      // Limit the wheel velocity command
      if (final_wheel_vel_cmd < module_wheel_speed_limit_lower_[i] ||
        final_wheel_vel_cmd > module_wheel_speed_limit_upper_[i])
      {
        const double clipped_wheel_vel_cmd = std::clamp(
          final_wheel_vel_cmd,
          module_wheel_speed_limit_lower_[i],
          module_wheel_speed_limit_upper_[i]);

        if (enabled_wheel_saturation_scaling_) {
          wheel_saturation_scale_factor_ = std::min(
            wheel_saturation_scale_factor_,
            clipped_wheel_vel_cmd / final_wheel_vel_cmd);
        }
      }

      // joints commands
      final_steering_commands_[i] = optimized_steering_angle;
      final_wheel_velocity_commands_[i] = final_wheel_vel_cmd;
    } catch (...) {
      // Fallback to safe values on any exception
      all_steering_aligned = false;
      module_handles_[i].steering_cmd_pos.get().set_value(current_steering_angle);
      module_handles_[i].wheel_cmd_vel.get().set_value(0.0);
      final_steering_commands_[i] = current_steering_angle;
      final_wheel_velocity_commands_[i] = 0.0;
    }
  }
  // End of module loop

  // --- 5. send the commands to the hardware interface ---
  if (target_vx_ == 0.0 && target_vy_ == 0.0 && target_wz_ == 0.0) {
    // Robot halted - hold steering, stop wheels
    for (size_t i = 0; i < num_modules_; ++i) {
      auto & handle = module_handles_[i];
      auto steering_val = handle.steering_state_pos.get().get_optional();
      const double hold_angle = steering_val.value_or(0.0);
      handle.steering_cmd_pos.get().set_value(hold_angle);
      handle.wheel_cmd_vel.get().set_value(0.0);
      final_steering_commands_[i] = hold_angle;
      final_wheel_velocity_commands_[i] = 0.0;
      previoud_steering_commands_[i] = hold_angle;
    }
  } else {
    // Update the final commands
    for (size_t i = 0; i < num_modules_; ++i) {
      auto & handle = module_handles_[i];

      handle.steering_cmd_pos.get().set_value(final_steering_commands_[i]);
      previoud_steering_commands_[i] = final_steering_commands_[i];

      const double wheel_vel = all_steering_aligned ?
        final_wheel_velocity_commands_[i] * wheel_saturation_scale_factor_ : 0.0;
      handle.wheel_cmd_vel.get().set_value(wheel_vel);
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
    auto & tf_front = rt_tf_odom_state_publisher_->msg_.transforms.front();
    tf_front.header.stamp = time;
    tf_front.transform.translation.x = odometry_.getX();
    tf_front.transform.translation.y = odometry_.getY();
    tf_front.transform.rotation = tf2::toMsg(orientation);
    rt_tf_odom_state_publisher_->unlockAndPublish();
  }

  // Publish joint commands in order to compare the actual joint states
  if (rt_commanded_joint_state_publisher_ && rt_commanded_joint_state_publisher_->trylock()) {
    auto & msg = rt_commanded_joint_state_publisher_->msg_;
    msg.header.stamp = time;

    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    for (size_t i = 0; i < num_modules_; ++i) {
      msg.position[i] = final_steering_commands_[i];
      msg.velocity[i] = kNaN;
      msg.position[i + num_modules_] = kNaN;
      msg.velocity[i + num_modules_] = final_wheel_velocity_commands_[i];
    }
    rt_commanded_joint_state_publisher_->unlockAndPublish();
  }

  // publish visualization markers
  if (enable_visualization_ && visualizer_ &&
    (time - last_visualization_publish_time_).seconds() >= visualization_update_time_)
  {
    for (size_t i = 0; i < num_modules_; ++i) {
      robot_frame_steering_angles_for_viz_[i] =
        final_steering_commands_[i] + module_angle_offsets_[i];
      wheel_linear_vels_for_viz_[i] = final_wheel_velocity_commands_[i] * wheel_radius_;
    }

    visualizer_->publish_markers(
      time, target_vx_, target_vy_, target_wz_,
      module_x_offsets_, module_y_offsets_,
      robot_frame_steering_angles_for_viz_, wheel_linear_vels_for_viz_);
    last_visualization_publish_time_ = time;
  }
  return controller_interface::return_type::OK;
}

void SwerveDriveController::reference_callback(const std::shared_ptr<CmdVelMsg> msg)
{
  last_cmd_vel_time_ = this->get_node()->now();
  cmd_vel_buffer_.writeFromNonRT(msg);
}

}   // namespace ffw_swerve_drive_controller

// Pluginlib export macro
 #include "pluginlib/class_list_macros.hpp"

// Export the controller class as a plugin
PLUGINLIB_EXPORT_CLASS(
  ffw_swerve_drive_controller::SwerveDriveController,
  controller_interface::ControllerInterface)
