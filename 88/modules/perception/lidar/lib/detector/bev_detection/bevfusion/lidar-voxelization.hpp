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

#ifndef __LIDAR_VOXELIZATION_HPP__
#define __LIDAR_VOXELIZATION_HPP__

#include <memory>
#include <vector>

#include "modules/perception/lidar/lib/detector/dnn_common/common/dtype.hpp"
// #include "common/dtype.hpp"

namespace bevfusion {
namespace lidar {

enum class CoordinateOrder : int {
  NoneOrder = 0,
  XYZ = 1,  // BEVFusion
  ZYX = 2   // CenterPoint
};

struct VoxelizationParameter {
  nvtype::Float3 min_range;
  nvtype::Float3 max_range;
  nvtype::Float3 voxel_size;
  nvtype::Int3 grid_size;
  int num_feature;
  int max_voxels;
  int max_points_per_voxel;
  int max_points;

  static nvtype::Int3 compute_grid_size(const nvtype::Float3& max_range, const nvtype::Float3& min_range,
                                        const nvtype::Float3& voxel_size);
};

class Voxelization {
 public:
  virtual ~Voxelization() = default;
  // points and voxels must be of half-float device pointer
  virtual void forward(const nvtype::half* points, int num_points, void* stream = nullptr,
                       CoordinateOrder output_order = CoordinateOrder::XYZ) = 0;

  virtual unsigned int num_voxels() = 0;
  virtual unsigned int voxel_dim() = 0;
  virtual unsigned int indices_dim() = 0;
  virtual std::vector<int> grid_size() = 0;
  virtual const void* indices() = 0;
  virtual const void* features() = 0;
  virtual CoordinateOrder order() = 0;
};

std::shared_ptr<Voxelization> create_voxelization(VoxelizationParameter parameter);

};  // namespace lidar
};  // namespace bevfusion

#endif  // __LIDAR_VOXELIZATION_HPP__