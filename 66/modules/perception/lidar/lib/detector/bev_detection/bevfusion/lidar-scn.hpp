/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
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

#ifndef __LIDAR_SCN_HPP__
#define __LIDAR_SCN_HPP__

#include <vector>
#include <string>
#include "modules/perception/lidar/lib/detector/bev_detection/bevfusion/lidar-voxelization.hpp"

namespace bevfusion {
namespace lidar {

// use model accuracy during SCN model inference.
enum class Precision : int { NonePrecision = 0, Float16 = 1, Int8 = 2 };

struct SCNParameter {
  VoxelizationParameter voxelization;
  std::string model;
  CoordinateOrder order = CoordinateOrder::XYZ;
  Precision precision = Precision::Float16;
};

class SCN {
 public:
  virtual ~SCN() = default;
  // points and voxels must be of half-float device pointer
  virtual const nvtype::half* forward(const nvtype::half* points, unsigned int num_points, void* stream = nullptr) = 0;
  virtual std::vector<int64_t> shape() = 0;
};

std::shared_ptr<SCN> create_scn(const SCNParameter& param);

};  // namespace lidar
};  // namespace bevfusion

#endif  // __LIDAR_SCN_HPP__