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

#ifndef __TRANSFUSION_HPP__
#define __TRANSFUSION_HPP__

#include <memory>
#include <string>
#include <vector>

// #include "common/dtype.hpp"
#include "modules/perception/lidar/lib/detector/dnn_common/common/dtype.hpp"

namespace bevfusion {
namespace fuser {

class Transfusion {
 public:
  virtual ~Transfusion() = default;
  virtual nvtype::half* forward(const nvtype::half* camera_bev,
                                const nvtype::half* lidar_bev,
                                void* stream) = 0;
  virtual void print() = 0;
  bool use_camera_ = false;
};

std::shared_ptr<Transfusion> create_transfusion(const std::string& model,
                                                const bool& use_camera);

};  // namespace fuser
};  // namespace bevfusion

#endif  // __TRANSFUSION_HPP__
