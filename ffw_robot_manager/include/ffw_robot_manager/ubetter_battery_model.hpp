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

#ifndef FFW_ROBOT_MANAGER__UBETTER_BATTERY_MODEL_HPP_
#define FFW_ROBOT_MANAGER__UBETTER_BATTERY_MODEL_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include "ffw_robot_manager/battery_model.hpp"

namespace ffw_robot_manager
{

/**
 * @brief Ubetter battery model implementation
 *
 * This class implements the voltage-to-SOC conversion for Ubetter batteries
 * using the provided lookup table with linear interpolation.
 */
class UbetterBatteryModel : public BatteryModel
{
public:
  UbetterBatteryModel();
  ~UbetterBatteryModel() override = default;

  double voltage_to_soc(double voltage_v) const override;
  std::string get_model_name() const override;
  std::pair<double, double> get_voltage_range() const override;
  bool is_voltage_valid(double voltage_v) const override;
  uint8_t get_power_supply_technology() const override;

private:
  static constexpr size_t BATTERY_DATA_NUMBER = 32;
  static const uint16_t battery_percent_data[BATTERY_DATA_NUMBER][2];
};

}  // namespace ffw_robot_manager

#endif  // FFW_ROBOT_MANAGER__UBETTER_BATTERY_MODEL_HPP_
