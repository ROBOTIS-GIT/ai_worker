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

#include <limits>
#include "ffw_robot_manager/robot_type.hpp"
#include "ffw_robot_manager/ubetter_battery_model.hpp"

namespace ffw_robot_manager
{

FfwSg2Rev1RobotType::FfwSg2Rev1RobotType()
{
  // Hard-code ubetter battery model for ffw_sg2_rev1
  battery_model_ = std::make_shared<UbetterBatteryModel>();
}

std::string FfwSg2Rev1RobotType::get_type_name() const
{
  return "ffw_sg2_rev1";
}

bool FfwSg2Rev1RobotType::is_battery_monitoring_enabled() const
{
  return true;
}

std::shared_ptr<BatteryModel> FfwSg2Rev1RobotType::get_battery_model() const
{
  return battery_model_;
}

std::vector<BatteryInfo> FfwSg2Rev1RobotType::get_battery_configurations() const
{
  std::vector<BatteryInfo> batteries;

  // Left battery
  BatteryInfo left_battery;
  left_battery.name = "left";
  left_battery.interface_name = "dxl1";
  left_battery.topic_name = "ai_worker/battery/left/state";
  left_battery.frame_id = "battery_left";
  left_battery.voltage_index = std::numeric_limits<size_t>::max();
  batteries.push_back(left_battery);

  // Right battery
  BatteryInfo right_battery;
  right_battery.name = "right";
  right_battery.interface_name = "dxl61";
  right_battery.topic_name = "ai_worker/battery/right/state";
  right_battery.frame_id = "battery_right";
  right_battery.voltage_index = std::numeric_limits<size_t>::max();
  batteries.push_back(right_battery);

  return batteries;
}

FfwF2RobotType::FfwF2RobotType()
{
  // Hard-code ubetter battery model for ffw_f2
  battery_model_ = std::make_shared<UbetterBatteryModel>();
}

std::string FfwF2RobotType::get_type_name() const
{
  return "ffw_f2";
}

bool FfwF2RobotType::is_battery_monitoring_enabled() const
{
  return true;
}

std::shared_ptr<BatteryModel> FfwF2RobotType::get_battery_model() const
{
  return battery_model_;
}

std::vector<BatteryInfo> FfwF2RobotType::get_battery_configurations() const
{
  std::vector<BatteryInfo> batteries;

  // Left battery
  BatteryInfo left_battery;
  left_battery.name = "left";
  left_battery.interface_name = "dxl1";
  left_battery.topic_name = "ai_worker/battery/left/state";
  left_battery.frame_id = "battery_left";
  left_battery.voltage_index = std::numeric_limits<size_t>::max();
  batteries.push_back(left_battery);

  // Right battery
  BatteryInfo right_battery;
  right_battery.name = "right";
  right_battery.interface_name = "dxl61";
  right_battery.topic_name = "ai_worker/battery/right/state";
  right_battery.frame_id = "battery_right";
  right_battery.voltage_index = std::numeric_limits<size_t>::max();
  batteries.push_back(right_battery);

  return batteries;
}

FfwSh5Rev1RobotType::FfwSh5Rev1RobotType()
{
  // SH5 uses the same battery model and monitored interfaces as SG2.
  battery_model_ = std::make_shared<UbetterBatteryModel>();
}

std::string FfwSh5Rev1RobotType::get_type_name() const
{
  return "ffw_sh5_rev1";
}

bool FfwSh5Rev1RobotType::is_battery_monitoring_enabled() const
{
  return true;
}

std::shared_ptr<BatteryModel> FfwSh5Rev1RobotType::get_battery_model() const
{
  return battery_model_;
}

std::vector<BatteryInfo> FfwSh5Rev1RobotType::get_battery_configurations() const
{
  std::vector<BatteryInfo> batteries;

  // Left battery
  BatteryInfo left_battery;
  left_battery.name = "left";
  left_battery.interface_name = "dxl1";
  left_battery.topic_name = "ai_worker/battery/left/state";
  left_battery.frame_id = "battery_left";
  left_battery.voltage_index = std::numeric_limits<size_t>::max();
  batteries.push_back(left_battery);

  // Right battery
  BatteryInfo right_battery;
  right_battery.name = "right";
  right_battery.interface_name = "dxl61";
  right_battery.topic_name = "ai_worker/battery/right/state";
  right_battery.frame_id = "battery_right";
  right_battery.voltage_index = std::numeric_limits<size_t>::max();
  batteries.push_back(right_battery);

  return batteries;
}

std::shared_ptr<RobotType> create_robot_type(const std::string & type_name)
{
  if (type_name == "ffw_sg2_rev1") {
    return std::make_shared<FfwSg2Rev1RobotType>();
  } else if (type_name == "ffw_sh5_rev1") {
    return std::make_shared<FfwSh5Rev1RobotType>();
  } else if (type_name == "ffw_f2") {
    return std::make_shared<FfwF2RobotType>();
  }

  return nullptr;  // Type not found
}

}  // namespace ffw_robot_manager
