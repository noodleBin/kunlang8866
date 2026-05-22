/******************************************************************************
 * Copyright 2023 The century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 */

#pragma once
#include <string>
#include <unordered_map>
/**
 * @namespace century::common
 * @brief century::common
 */
namespace century {
namespace common {

enum DomainType {
  AARCH64 = 3,
  X86 = 4,
};

enum module_type {
  E_SYSTEM = 0,       ///< system status, e.g. cpu, disk, mem
  E_PERCEPTION,       ///< perception
  E_LOCALIZATION,     ///< localization
  E_PLANNING,         ///< planning
  E_CONTROL,          ///< control
  E_PREDICTION,       ///< prediction
  E_CHASSIS,          ///< chassis
  E_TRAFFIC_LIGHT,    ///< traffic light
  E_MEMS,             ///< tracker mems
  E_RADAR,            ///< tracker radar
  E_RS,               ///< tracker rs
  E_ULTRASONIC,       ///< tracker ultrasonic
  E_CAMERA,           ///< tracker camera
  E_RADAR_CUBTEK,     ///< tracker radar cubtek
  E_MCLOUD,           ///< tracker mcloud
  E_MONITOR,          ///< tracker monitor
  E_UDAS_ULTRASONIC,  ///< udas ultrasonic eol
  E_IMU_RAW,          ///< imu raw
  E_LIDAR,            ///< lidar pointcloud
};

static std::unordered_map<module_type, std::string> module_enum_to_string = {
    {E_SYSTEM, "SYSTEM"},
    {E_PERCEPTION, "PERCEPTION"},
    {E_LOCALIZATION, "LOCALIZATION"},
    {E_PLANNING, "PLANNING"},
    {E_CONTROL, "CONTROL"},
    {E_PREDICTION, "PREDICTION"},
    {E_CHASSIS, "CHASSIS"},
    {E_TRAFFIC_LIGHT, "TRAFFIC_LIGHT"},
    {E_MEMS, "MEMS"},
    {E_RADAR, "RADAR"},
    {E_RS, "RS"},
    {E_ULTRASONIC, "ULTRASONIC"},
    {E_CAMERA, "CAMERA"},
    {E_RADAR_CUBTEK, "RADAR_CUBTEK"},
    {E_MCLOUD, "MCLOUD"},
    {E_MONITOR, "MONITOR"},
    {E_UDAS_ULTRASONIC, "UDAS_ULTRASONIC"},
    {E_IMU_RAW, "IMU_RAW"},
    {E_LIDAR, "LIDAR"},
};

// upstream --> downstream
enum ModuleCategory : uint32_t {
  MC_UNKNOWN = 0,
  MC_SYSTEM,
  MC_CHASSIS,
  MC_LOCALIZATION,
  MC_PERCEPTION,
  MC_PLANNING,
  MC_MAX,
};

#if 0
static ModuleCategory GetModuleCategory(const int module_type) {
  ModuleCategory res = ModuleCategory::MC_UNKNOWN;
  // if (module_type < 0 && module_type >= common::module_type::E_max) {
  //   return res;
  // }

  switch (module_type) {
    case common::module_type::E_CHASSIS:
      res = ModuleCategory::MC_CHASSIS;
      break;
    case common::module_type::E_LOCALIZATION:
    case common::module_type::E_IMU_RAW:
      res = ModuleCategory::MC_LOCALIZATION;
      break;
    case common::module_type::E_PERCEPTION:
    case common::module_type::E_TRAFFIC_LIGHT:
    case common::module_type::E_CAMERA:
    case common::module_type::E_MEMS:
    case common::module_type::E_RS:
    case common::module_type::E_RADAR:
    case common::module_type::E_RADAR_CUBTEK:
    case common::module_type::E_ULTRASONIC:
    case common::module_type::E_UDAS_ULTRASONIC:
    case common::module_type::E_PREDICTION:
      res = ModuleCategory::MC_PERCEPTION;
      break;
    case common::module_type::E_PLANNING:
    case common::module_type::E_CONTROL:
      res = ModuleCategory::MC_PLANNING;
      break;
    case common::module_type::E_MCLOUD:
    case common::module_type::E_MONITOR:
      res = ModuleCategory::MC_SYSTEM;
      break;
    default:
      res = ModuleCategory::MC_PLANNING;
      break;
  }

  return res;
}

#endif

}  // namespace common
}  // namespace century
