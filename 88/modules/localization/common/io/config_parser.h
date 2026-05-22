/******************************************************************************
 * Copyright 2024 The Century Authors. All Rights Reserved.
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
#pragma once
#include <string>
#include <vector>

#include "third_party/mmath/eigen_types.h"
#include "third_party/mmath/se3.h"
#include "yaml-cpp/yaml.h"

#include "cyber/common/log.h"
#include "modules/localization/common/io/message_parser.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_config.h"

namespace century {
namespace common {
namespace io {

constexpr double sync_thresh = 0.02;  // default:20ms

struct NdtMapConfig {
  std::string ndt_map_version = "";
  std::string coordinate_type = "";
  unsigned int map_node_size_x;
  unsigned int map_node_size_y;
  double map_min_x;
  double map_min_y;
  double map_max_x;
  double map_max_y;
  double map_resolution;
  double map_ground_height_offset;
  bool map_is_compression;
};

class ConfigParser {
 public:
  ConfigParser() {}
  ~ConfigParser() {}

  bool LoadInsConfig(const std::string& file_path, Eigen::Vector3d* gravity_acc,
                     Eigen::Vector3d* gyro_noise, Eigen::Vector3d* acc_noise,
                     mmath::SE3* Tx_veh_imu, mmath::SE3* Tx_imu_gnss);

  bool LoadLidarConfig(const std::string& file_path, mmath::SE3* Tx_veh_lidar1,
                       mmath::SE3* Tx_imu_lidar1, mmath::SE3* Tx_veh_lidar2,
                       mmath::SE3* Tx_imu_lidar2);
  bool LoadNdtMapConfig(const std::string& ndt_map_path,
                        NdtMapConfig* ndt_map_config);
};

}  // namespace io
}  // namespace common
}  // namespace century
