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

#ifndef __HEAD_TRANSBBOX_HPP__
#define __HEAD_TRANSBBOX_HPP__

#include <memory>
#include <string>
#include <vector>

#include "modules/perception/lidar/lib/detector/dnn_common/common/dtype.hpp"
// #include "common/dtype.hpp"

namespace bevfusion {
namespace head {
namespace transbbox {

struct TransBBoxParameter {
  std::string model;
  float out_size_factor = 8;
  nvtype::Float2 voxel_size{0.075, 0.075};
  nvtype::Float2 pc_range{-54.0f, -54.0f};
  nvtype::Float3 post_center_range_start{-61.2, -61.2, -10.0};
  nvtype::Float3 post_center_range_end{61.2, 61.2, 10.0};
  float confidence_threshold = 0.0f;
  bool sorted_bboxes = true;
};

struct Position {
  float x, y, z;
};

struct Size {
  float w, l, h;  // x, y, z
};

struct Velocity {
  float vx, vy;
};

struct BoundingBox {
  Position position;
  Size size;
  Velocity velocity;
  float z_rotation;
  float score;
  int id;
};

class TransBBox {
 public:
  virtual ~TransBBox() = default;
  virtual std::vector<BoundingBox> forward(const nvtype::half* transfusion_feature, float confidence_threshold, void* stream,
                                           bool sorted_by_conf = false) = 0;
  virtual void print() = 0;
};

std::shared_ptr<TransBBox> create_transbbox(const TransBBoxParameter& param);

};  // namespace transbbox
};  // namespace head
};  // namespace bevfusion

#endif  // __HEAD_TRANSBBOX_HPP__