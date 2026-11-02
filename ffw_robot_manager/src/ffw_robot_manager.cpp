// Copyright 2025 ROBOTIS CO., LTD.
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
//
// Author: Woojin Wie

#include "ffw_robot_manager/ffw_robot_manager.hpp"
#include <limits>
#include "ffw_robot_manager/topic_watchdog.hpp"
#include "ffw_robot_manager/robot_type.hpp"
#include "dynamixel_interfaces/msg/dynamixel_state.hpp"
#include "sensor_msgs/msg/battery_state.hpp"


namespace ffw_robot_manager
{
FfwRobotManager::FfwRobotManager() {}

controller_interface::CallbackReturn FfwRobotManager::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration FfwRobotManager::command_interface_configuration()
const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration FfwRobotManager::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::ALL;
  return config;
}

controller_interface::CallbackReturn FfwRobotManager::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  gpio_names_.clear();
  gpio_interface_indices_.clear();
  torque_disabled_ = false;
  led_error_set_ = false;

  // Create service client for Dynamixel torque control
  torque_client_ =
    get_node()->create_client<std_srvs::srv::SetBool>(
      "dynamixel_hardware_interface/set_dxl_torque");

  // Create service client for LED control
  led_client_ =
    get_node()->create_client<dynamixel_interfaces::srv::SetDataToDxl>("ffw_sensor/set_dxl_data");

  // Initialize LED to normal state
  reset_led_state();

  source_.clear();
  joystick_mode_.clear();
  last_mode_led_.clear();
  const auto mode_qos = rclcpp::QoS(1).reliable().transient_local();
  source_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
    "/source", mode_qos, [this](const std_msgs::msg::String & msg) {
      source_ = msg.data;
      update_mode_led();
    });
  joystick_mode_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
    "/reference/joystick/mode", mode_qos, [this](const std_msgs::msg::String & msg) {
      joystick_mode_ = msg.data;
      update_mode_led();
    });

  // Setup topic watchdogs
  setup_watchdogs();

  // Setup battery monitoring
  setup_battery_monitoring();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FfwRobotManager::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Discover all GPIO devices and their error interfaces
  gpio_names_.clear();
  gpio_interface_indices_.clear();
  std::unordered_set<std::string> found_gpios;
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    const auto & si = state_interfaces_[i];
    const std::string & prefix = si.get_prefix_name();
    const std::string & interface = si.get_interface_name();
    // Heuristic: GPIOs are named like dxl1, dxl2, ...
    if (prefix.rfind("dxl", 0) == 0) {
      found_gpios.insert(prefix);
      if (interface == "Error Code" || interface == "Hardware Error Status") {
        gpio_interface_indices_[prefix][interface] = i;
      }

      // Find battery voltage interfaces dynamically
      if (battery_monitoring_enabled_ && interface == "Present Input Voltage" && robot_type_) {
        for (auto & battery_config : battery_configurations_) {
          if (prefix == battery_config.interface_name) {
            battery_config.voltage_index = i;
            RCLCPP_INFO(get_node()->get_logger(),
                        "Found %s battery voltage interface at index %zu for %s",
                        battery_config.name.c_str(), i, prefix.c_str());
          }
        }
      }
    }
  }
  gpio_names_.assign(found_gpios.begin(), found_gpios.end());
  std::sort(gpio_names_.begin(), gpio_names_.end());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FfwRobotManager::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  gpio_names_.clear();
  gpio_interface_indices_.clear();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FfwRobotManager::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  for (const auto & gpio : gpio_names_) {
    bool has_error = false;
    std::string error_details;

    // Check Error Code
    auto error_code_it = gpio_interface_indices_[gpio].find("Error Code");
    if (error_code_it != gpio_interface_indices_[gpio].end()) {
      auto opt = state_interfaces_[error_code_it->second].get_optional();
      if (opt.has_value() && opt.value() != 0) {
        has_error = true;
        auto error_info =
          dynamixel_hardware_interface::get_error_code_info(static_cast<int>(opt.value()));
        if (error_info) {
          error_details += "Error Code: " + std::string(error_info->label) + " (" +
            error_info->description + ")";
        } else {
          error_details += "Error Code: Unknown error (" +
            std::to_string(static_cast<int>(opt.value())) + ")";
        }
      }
    }

    // Check Hardware Error Status
    auto hw_error_it = gpio_interface_indices_[gpio].find("Hardware Error Status");
    if (hw_error_it != gpio_interface_indices_[gpio].end()) {
      auto opt = state_interfaces_[hw_error_it->second].get_optional();
      if (opt.has_value() && opt.value() != 0) {
        has_error = true;
        if (!error_details.empty()) {error_details += "; ";}
        error_details += "Hardware Error Status: ";

        int status_value = static_cast<int>(opt.value());
        bool first_bit = true;
        for (int bit = 0; bit < 8; ++bit) {
          if (status_value & (1 << bit)) {
            auto bit_info = dynamixel_hardware_interface::get_hardware_error_status_bit_info(bit);
            if (bit_info) {
              if (!first_bit) {error_details += ", ";}
              error_details += bit_info->label;
              first_bit = false;
            }
          }
        }
      }
    }

    // Log and disable torque if there are errors
    if (has_error) {
      // RCLCPP_WARN(get_node()->get_logger(),
      // "GPIO '%s' has errors: %s", gpio.c_str(), error_details.c_str());

      // Disable torque for all Dynamixels if not already disabled and parameter is enabled
      if (!torque_disabled_ && params_.disable_torque_on_error) {
        disable_all_torque();
      }

      // Set LED to error state if not already set and parameter is enabled
      if (!led_error_set_ && params_.set_led_error_on_error) {
        set_led_error_state();
      }
    }
  }

  // Update battery states if monitoring is enabled
  if (battery_monitoring_enabled_) {
    update_battery_states();
  }

  return controller_interface::return_type::OK;
}

void FfwRobotManager::disable_all_torque()
{
  RCLCPP_WARN_STREAM(get_node()->get_logger(), "Disabling torque for all Dynamixels.");

  if (!torque_client_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Torque service client not available");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;  // Disable torque

  auto future = torque_client_->async_send_request(request);
}

void FfwRobotManager::set_led_error_state()
{
  const std::lock_guard<std::mutex> lock(led_mutex_);
  RCLCPP_WARN_STREAM(get_node()->get_logger(), "Setting LED to error state (red color).");

  // Set all LED colors to red (255) and mode to RGB_BLINK
  LedValues led_values(255, 0, 0, 255, 0, 0, LedMode::RGB_BLINK, LedMode::RGB_BLINK);

  if (set_led_values(led_values)) {
    led_error_set_ = true;
  }
}

void FfwRobotManager::reset_led_state()
{
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Resetting LED to normal state.");

  // Reset LED colors to normal (blue) and mode to RGB_BREATHE
  LedValues led_values(0, 255, 255, 0, 255, 255, LedMode::RGB_BREATHE, LedMode::RGB_BREATHE);

  if (set_led_values(led_values)) {
    led_error_set_ = false;
  }
}

void FfwRobotManager::update_mode_led()
{
  const std::lock_guard<std::mutex> lock(led_mutex_);
  if (led_error_set_ || (dxl_state_watchdog_ && dxl_state_watchdog_->is_timed_out())) {
    return;
  }
  const std::string mode = source_ == "model_action" ? "model_action" :
    source_ == "teleop" && (joystick_mode_ == "head_control" || joystick_mode_ == "swerve") ?
    joystick_mode_ : "";
  if (mode.empty() || mode == last_mode_led_ || !led_client_->service_is_ready()) {
    return;
  }
  bool sent = false;
  if (mode == "model_action") {
    sent = set_led_color(0, 255, 255, LedMode::RGB_BREATHE);
  } else if (mode == "head_control") {
    sent = set_led_color(255, 255, 255, LedMode::RGB_BREATHE);
  } else if (mode == "swerve") {
    sent = set_led_color(255, 255, 0, LedMode::RGB_BREATHE);
  }
  if (sent) {last_mode_led_ = mode;}
}

bool FfwRobotManager::set_led_values(const std::vector<uint32_t> & values)
{
  if (!led_client_) {
    RCLCPP_ERROR(get_node()->get_logger(), "LED service client not available");
    return false;
  }

  static const std::vector<std::string> led_items = {
    "Neopixel_Head_Left_Red",
    "Neopixel_Head_Left_Green",
    "Neopixel_Head_Left_Blue",
    "Neopixel_Head_Right_Red",
    "Neopixel_Head_Right_Green",
    "Neopixel_Head_Right_Blue",
    "Neopixel_Head_Left_Mode",
    "Neopixel_Head_Right_Mode"
  };

  if (values.size() != led_items.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "LED values size mismatch: expected %zu, got %zu",
        led_items.size(), values.size());
    return false;
  }

  // Send all LED requests asynchronously without blocking
  for (size_t i = 0; i < led_items.size(); ++i) {
    auto request = std::make_shared<dynamixel_interfaces::srv::SetDataToDxl::Request>();
    request->id = 91;  // LED is on ID 91
    request->item_name = led_items[i];
    request->item_data = values[i];

    // Send request asynchronously - don't wait for response
    led_client_->async_send_request(request);
  }

  return true;
}

bool FfwRobotManager::set_led_values(const LedValues & led_values)
{
  // Convert struct to vector and call the original function
  std::vector<uint32_t> values = {
    led_values.left_red,
    led_values.left_green,
    led_values.left_blue,
    led_values.right_red,
    led_values.right_green,
    led_values.right_blue,
    static_cast<uint32_t>(led_values.left_mode),
    static_cast<uint32_t>(led_values.right_mode)
  };

  return set_led_values(values);
}

bool FfwRobotManager::set_led_values(
  uint32_t left_red, uint32_t left_green, uint32_t left_blue,
  uint32_t right_red, uint32_t right_green, uint32_t right_blue,
  uint32_t left_mode, uint32_t right_mode)
{
  // Create LedValues struct and call the struct-based function
  LedValues led_values(left_red, left_green, left_blue, right_red, right_green, right_blue,
    left_mode, right_mode);
  return set_led_values(led_values);
}

bool FfwRobotManager::set_led_values(
  uint32_t left_red, uint32_t left_green, uint32_t left_blue,
  uint32_t right_red, uint32_t right_green, uint32_t right_blue,
  LedMode left_mode, LedMode right_mode)
{
  // Create LedValues struct and call the struct-based function
  LedValues led_values(left_red, left_green, left_blue, right_red, right_green, right_blue,
    left_mode, right_mode);
  return set_led_values(led_values);
}

bool FfwRobotManager::set_led_color(uint32_t red, uint32_t green, uint32_t blue, uint32_t mode)
{
  // Create LedValues struct with same color for both heads
  LedValues led_values(red, green, blue, mode);
  return set_led_values(led_values);
}

bool FfwRobotManager::set_led_color(uint32_t red, uint32_t green, uint32_t blue, LedMode mode)
{
  // Create LedValues struct with same color for both heads
  LedValues led_values(red, green, blue, mode);
  return set_led_values(led_values);
}

void FfwRobotManager::setup_watchdogs()
{
  // Watchdog for /ffw_follower/dxl_state
  dxl_state_watchdog_ = std::make_unique<TopicWatchdog<dynamixel_interfaces::msg::DynamixelState>>(
    get_node().get(),
    "/ffw_follower/dxl_state",
    std::chrono::milliseconds(500),
    [this]() {
      const std::lock_guard<std::mutex> lock(led_mutex_);
      last_mode_led_.clear();  // Restore the selected mode after watchdog recovery.
      // Set LED to red blinking on timeout.
      set_led_color(255, 0, 0, LedMode::RGB_BLINK);
    }
  );
}

void FfwRobotManager::setup_battery_monitoring()
{
  // Create robot type configuration
  robot_type_ = create_robot_type(params_.ffw_type);
  if (!robot_type_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Unknown robot type: %s", params_.ffw_type.c_str());
    battery_monitoring_enabled_ = false;
    return;
  }

  // Check if battery monitoring is enabled for this robot type
  if (robot_type_->is_battery_monitoring_enabled()) {
    battery_monitoring_enabled_ = true;

    // Get battery configurations from robot type
    battery_configurations_ = robot_type_->get_battery_configurations();

    // Create battery state publishers dynamically
    battery_publishers_.clear();
    for (const auto & battery_config : battery_configurations_) {
      auto publisher = get_node()->create_publisher<sensor_msgs::msg::BatteryState>(
        battery_config.topic_name, 10);
      battery_publishers_.push_back(publisher);

      RCLCPP_INFO(get_node()->get_logger(), "Created battery publisher for %s at %s",
                  battery_config.name.c_str(), battery_config.topic_name.c_str());
    }

    RCLCPP_INFO(get_node()->get_logger(),
                "Battery monitoring enabled for %s with %zu batteries and model: %s",
                robot_type_->get_type_name().c_str(),
                battery_configurations_.size(),
                robot_type_->get_battery_model()->get_model_name().c_str());
  } else {
    battery_monitoring_enabled_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "Battery monitoring disabled for robot type: %s",
                robot_type_->get_type_name().c_str());
  }
}

void FfwRobotManager::update_battery_states()
{
  if (!battery_monitoring_enabled_) {
    return;
  }

  // Update all batteries dynamically
  for (size_t i = 0; i < battery_configurations_.size() && i < battery_publishers_.size(); ++i) {
    const auto & battery_config = battery_configurations_[i];
    auto & publisher = battery_publishers_[i];

    // Check if voltage index is valid (allow 0; ignore unset sentinel SIZE_MAX)
    if (battery_config.voltage_index != std::numeric_limits<size_t>::max() &&
      battery_config.voltage_index < state_interfaces_.size())
    {
      auto voltage_opt = state_interfaces_[battery_config.voltage_index].get_optional();
      if (voltage_opt.has_value()) {
        double voltage = voltage_opt.value();
        double soc_fraction = robot_type_->get_battery_model()->voltage_to_soc(voltage);
        // Prefer explicit frame_id from BatteryInfo when provided; otherwise derive
        const std::string frame_id = !battery_config.frame_id.empty() ?
          battery_config.frame_id :
          (!battery_config.name.empty() ?
          std::string("battery_") + battery_config.name :
          std::string("battery_") + battery_config.interface_name);
        auto battery_state = create_battery_state(voltage, soc_fraction, frame_id);
        publisher->publish(battery_state);
      }
    }
  }
}


sensor_msgs::msg::BatteryState FfwRobotManager::create_battery_state(
  double voltage, double soc, const std::string & frame_id)
{
  sensor_msgs::msg::BatteryState battery_state;
  battery_state.header.stamp = get_node()->now();
  battery_state.header.frame_id = frame_id;
  battery_state.voltage = static_cast<float>(voltage);
  battery_state.percentage = static_cast<float>(soc);
  battery_state.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  battery_state.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;

  // Set power supply technology based on robot type's battery model
  if (robot_type_ && robot_type_->get_battery_model()) {
    battery_state.power_supply_technology =
      robot_type_->get_battery_model()->get_power_supply_technology();
  } else {
    battery_state.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  }

  battery_state.present = true;

  return battery_state;
}

}  // namespace ffw_robot_manager

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ffw_robot_manager::FfwRobotManager,
  controller_interface::ControllerInterface)
