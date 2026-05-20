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

#ifndef FFW_ROBOT_MANAGER__ROBOT_TYPE_HPP_
#define FFW_ROBOT_MANAGER__ROBOT_TYPE_HPP_

#include <string>
#include <memory>
#include <vector>
#include "ffw_robot_manager/battery_model.hpp"

namespace ffw_robot_manager
{

/**
 * @brief Battery information structure
 */
struct BatteryInfo
{
  std::string name;            // Battery name (e.g., "left", "right", "main")
  std::string interface_name;  // Dynamixel interface name (e.g., "dxl1", "dxl61")
  std::string topic_name;      // ROS topic name (e.g., "/ai_worker/hardware/battery/left")
  std::string frame_id;        // TF frame id for this battery (e.g., "battery_left")
  size_t voltage_index;        // State interface index for voltage reading
};

/**
 * @brief Robot type configuration class
 *
 * This class defines the configuration for different robot types,
 * including battery monitoring settings and battery models.
 */
class RobotType
{
public:
  virtual ~RobotType() = default;

  /**
   * @brief Get the robot type name
   * @return Robot type name string
   */
  virtual std::string get_type_name() const = 0;

  /**
   * @brief Check if battery monitoring is enabled for this robot type
   * @return True if battery monitoring is enabled
   */
  virtual bool is_battery_monitoring_enabled() const = 0;

  /**
   * @brief Get the battery model for this robot type
   * @return Shared pointer to battery model, nullptr if not supported
   */
  virtual std::shared_ptr<BatteryModel> get_battery_model() const = 0;

  /**
   * @brief Get the battery configurations for this robot type
   * @return Vector of battery information
   */
  virtual std::vector<BatteryInfo> get_battery_configurations() const = 0;
};

/**
 * @brief FFW SG2 Rev1 robot type implementation
 */
class FfwSg2Rev1RobotType : public RobotType
{
public:
  FfwSg2Rev1RobotType();
  ~FfwSg2Rev1RobotType() override = default;

  std::string get_type_name() const override;
  bool is_battery_monitoring_enabled() const override;
  std::shared_ptr<BatteryModel> get_battery_model() const override;
  std::vector<BatteryInfo> get_battery_configurations() const override;

private:
  std::shared_ptr<BatteryModel> battery_model_;
};

class FfwF2RobotType : public RobotType
{
public:
  FfwF2RobotType();
  ~FfwF2RobotType() override = default;

  std::string get_type_name() const override;
  bool is_battery_monitoring_enabled() const override;
  std::shared_ptr<BatteryModel> get_battery_model() const override;
  std::vector<BatteryInfo> get_battery_configurations() const override;

private:
  std::shared_ptr<BatteryModel> battery_model_;
};

class FfwSh5Rev1RobotType : public RobotType
{
public:
  FfwSh5Rev1RobotType();
  ~FfwSh5Rev1RobotType() override = default;

  std::string get_type_name() const override;
  bool is_battery_monitoring_enabled() const override;
  std::shared_ptr<BatteryModel> get_battery_model() const override;
  std::vector<BatteryInfo> get_battery_configurations() const override;

private:
  std::shared_ptr<BatteryModel> battery_model_;
};

/**
 * @brief Factory function to create robot types
 * @param type_name Name of the robot type
 * @return Shared pointer to robot type, nullptr if type not found
 */
std::shared_ptr<RobotType> create_robot_type(const std::string & type_name);

}  // namespace ffw_robot_manager

#endif  // FFW_ROBOT_MANAGER__ROBOT_TYPE_HPP_
