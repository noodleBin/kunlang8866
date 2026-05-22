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

#ifndef __CAMERA_DEPTH_HPP__
#define __CAMERA_DEPTH_HPP__

#include <memory>

// #include "common/dtype.hpp"
#include "modules/perception/lidar/lib/detector/dnn_common/common/dtype.hpp"

namespace bevfusion {
namespace camera {

class Depth {
 public:
  virtual ~Depth() = default;
  // points must be of half-float type
  // return value will is a num_camera x image_height x image_width Tensor
  virtual nvtype::half* forward(const nvtype::half* points, int num_points, int points_dim, void* stream = nullptr) = 0;

  // You can call this function if you need to update the matrix
  // All matrix pointers must be on the host
  // img_aug_matrix is num_camera x 4 x 4 matrix on host
  // lidar2image    is num_camera x 4 x 4 matrix on host
  virtual void update(const float* img_aug_matrix, const float* lidar2image, void* stream = nullptr) = 0;
};

std::shared_ptr<Depth> create_depth(unsigned int image_width, unsigned int image_height, unsigned int num_camera);

};  // namespace camera
};  // namespace bevfusion

#endif  // __CAMERA_DEPTH_HPP__