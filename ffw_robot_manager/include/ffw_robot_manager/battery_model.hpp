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

#ifndef FFW_ROBOT_MANAGER__BATTERY_MODEL_HPP_
#define FFW_ROBOT_MANAGER__BATTERY_MODEL_HPP_

#include <cstdint>
#include <string>
#include <memory>
#include <utility>

namespace ffw_robot_manager
{

/**
 * @brief Abstract base class for battery models
 *
 * This class defines the interface for different battery models
 * to convert voltage readings to State of Charge (SOC) percentages.
 */
class BatteryModel
{
public:
  virtual ~BatteryModel() = default;

  /**
   * @brief Convert voltage in Volts to SOC fraction
   * @param voltage_v Voltage in Volts
   * @return SOC fraction (0.0 to 1.0)
   */
  virtual double voltage_to_soc(double voltage_v) const = 0;

  /**
   * @brief Get the battery model name
   * @return Model name string
   */
  virtual std::string get_model_name() const = 0;

  /**
   * @brief Get the voltage range for this battery model
   * @return Pair of (min_voltage_v, max_voltage_v)
   */
  virtual std::pair<double, double> get_voltage_range() const = 0;

  /**
   * @brief Check if a voltage is within the valid range for this model
   * @param voltage_v Voltage in Volts
   * @return True if voltage is within valid range
   */
  virtual bool is_voltage_valid(double voltage_v) const = 0;

  /**
   * @brief Get the power supply technology for this battery model
   * @return Power supply technology constant (e.g., POWER_SUPPLY_TECHNOLOGY_LIPO)
   */
  virtual uint8_t get_power_supply_technology() const = 0;
};

/**
 * @brief Factory function to create battery models
 * @param model_name Name of the battery model
 * @return Shared pointer to battery model, nullptr if model not found
 */
std::shared_ptr<BatteryModel> create_battery_model(const std::string & model_name);

}  // namespace ffw_robot_manager

#endif  // FFW_ROBOT_MANAGER__BATTERY_MODEL_HPP_
