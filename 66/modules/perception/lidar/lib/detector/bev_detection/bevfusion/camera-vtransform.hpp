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

#ifndef __CAMERA_VTRANSFORM_HPP__
#define __CAMERA_VTRANSFORM_HPP__

#include <memory>
#include <string>
#include <vector>

// #include "common/dtype.hpp"
#include "modules/perception/lidar/lib/detector/dnn_common/common/dtype.hpp"


namespace bevfusion {
namespace camera {

class VTransform {
 public:
  virtual ~VTransform() = default;
  virtual nvtype::half* forward(const nvtype::half* camera_bev, void* stream = nullptr) = 0;

  virtual std::vector<int> feat_shape() = 0;
  virtual void print() = 0;
};

std::shared_ptr<VTransform> create_vtransform(const std::string& model);

};  // namespace camera
};  // namespace bevfusion

#endif  // __CAMERA_VTRANSFORM_HPP__