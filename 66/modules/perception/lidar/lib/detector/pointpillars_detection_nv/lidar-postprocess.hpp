/*
 * SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES.
 * All rights reserved. SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef __LIDAR_POSTPROCESS_HPP__
#define __LIDAR_POSTPROCESS_HPP__

#include <memory>
#include <vector>

#include "modules/perception/lidar/lib/detector/dnn_common/common/dtype.hpp"

namespace pointpillar {
namespace lidar {

struct BoundingBox {
  float x;
  float y;
  float z;
  float w;
  float l;
  float h;
  float rt;
  int id;
  float score;
  BoundingBox(){};
  BoundingBox(float x_, float y_, float z_, float w_, float l_, float h_,
              float rt_, int id_, float score_)
      : x(x_),
        y(y_),
        z(z_),
        w(w_),
        l(l_),
        h(h_),
        rt(rt_),
        id(id_),
        score(score_) {}
};

struct PostProcessParameter {
  nvtype::Float3 min_range;
  nvtype::Float3 max_range;
  nvtype::Int2 feature_size;
  int num_classes;
  int num_anchors;
  int len_per_anchor;
  int num_box_values;
  float dir_offset;
  std::vector<float> score_thresh_per_class;
  std::vector<float> anchor_vector;
  std::vector<float> anchor_bottom_heights_vector;
  std::vector<float> nms_iou_thresh_per_class;
  
};

class PostProcess {
 public:
  virtual void forward(const float* cls, const float* box, const float* dir,
                       void* stream) = 0;

  virtual std::vector<BoundingBox> bndBoxVec() = 0;
};

std::shared_ptr<PostProcess> create_postprocess(
    const PostProcessParameter& param);

};  // namespace lidar
};  // namespace pointpillar

#endif  // __LIDAR_POSTPROCESS_HPP__
