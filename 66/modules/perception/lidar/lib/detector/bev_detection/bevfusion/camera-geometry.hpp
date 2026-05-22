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

#ifndef __CAMERA_GEOMETRY_HPP__
#define __CAMERA_GEOMETRY_HPP__

#include <memory>

// #include "common/dtype.hpp"
#include "modules/perception/lidar/lib/detector/dnn_common/common/dtype.hpp"

namespace bevfusion {
namespace camera {

struct GeometryParameter {
  nvtype::Float3 xbound;
  nvtype::Float3 ybound;
  nvtype::Float3 zbound;
  nvtype::Float3 dbound;
  nvtype::Int3 geometry_dim;  // w(x 360), h(y 360), c(z 80)
  unsigned int feat_width;
  unsigned int feat_height;
  unsigned int image_width;
  unsigned int image_height;
  unsigned int num_camera;
};

class Geometry {
 public:
  virtual ~Geometry() = default;
  virtual nvtype::Int3* intervals() = 0;
  virtual unsigned int num_intervals() = 0;

  virtual unsigned int* indices() = 0;
  virtual unsigned int num_indices() = 0;

  // You can call this function if you need to update the matrix
  // All matrix pointers must be on the host
  // img_aug_matrix is num_camera x 4 x 4 matrix on host
  // lidar2image    is num_camera x 4 x 4 matrix on host
  virtual void update(const float* camera2lidar, const float* camera_intrinsics, const float* img_aug_matrix,
                      void* stream = nullptr) = 0;

  // Consider releasing excess memory if you don't need to update the matrix
  // After freeing the memory, the update function call will raise a logical exception.
  virtual void free_excess_memory() = 0;
};

std::shared_ptr<Geometry> create_geometry(GeometryParameter param);

};  // namespace camera
};  // namespace bevfusion

#endif  // __CAMERA_GEOMETRY_HPP__