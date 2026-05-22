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

#ifndef __BEVFUSION_HPP__
#define __BEVFUSION_HPP__

#include "modules/perception/lidar/lib/detector/bev_detection/bevfusion/camera-backbone.hpp"
#include "modules/perception/lidar/lib/detector/bev_detection/bevfusion/camera-bevpool.hpp"
#include "modules/perception/lidar/lib/detector/bev_detection/bevfusion/camera-depth.hpp"
#include "modules/perception/lidar/lib/detector/bev_detection/bevfusion/camera-geometry.hpp"
#include "modules/perception/lidar/lib/detector/bev_detection/bevfusion/camera-normalization.hpp"
#include "modules/perception/lidar/lib/detector/bev_detection/bevfusion/camera-vtransform.hpp"
#include "modules/perception/lidar/lib/detector/bev_detection/bevfusion/head-transbbox.hpp"
#include "modules/perception/lidar/lib/detector/bev_detection/bevfusion/lidar-scn.hpp"
#include "modules/perception/lidar/lib/detector/bev_detection/bevfusion/transfusion.hpp"

namespace bevfusion {

struct CoreParameter {
  std::string camera_model;
  std::string camera_vtransform;
  camera::GeometryParameter geometry;
  camera::NormalizationParameter normalize;
  lidar::SCNParameter lidar_scn;
  std::string transfusion;
  head::transbbox::TransBBoxParameter transbbox;
  bool use_camera = false;
};

class Core {
 public:
  virtual ~Core() = default;
  virtual std::vector<head::transbbox::BoundingBox> forward(
      const unsigned char **camera_images,
      const nvtype::half *lidar_points_device, int num_points, void *stream,
      float preprocess_time_ms = 0.0f) = 0;

  virtual std::vector<head::transbbox::BoundingBox> forward_no_normalize(
      const nvtype::half *camera_normed_images_device,
      const nvtype::half *lidar_points_device, int num_points, void *stream,
      float preprocess_time_ms = 0.0f) = 0;

  virtual void print() = 0;
  virtual void set_timer(bool enable) = 0;
  virtual void set_debug(bool enable) = 0;

  virtual void update(const float *camera2lidar, const float *camera_intrinsics,
                      const float *lidar2image, const float *img_aug_matrix,
                      void *stream = nullptr) = 0;

  // Consider releasing excess memory if you don't need to update the matrix
  // After freeing the memory, the update function call will raise a logical
  // exception.
  virtual void free_excess_memory() = 0;
};

std::shared_ptr<Core> create_core(const CoreParameter &param);

};  // namespace bevfusion

#endif  // __BEVFUSION_HPP__