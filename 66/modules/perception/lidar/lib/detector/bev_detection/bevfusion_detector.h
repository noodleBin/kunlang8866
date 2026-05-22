
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

#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "modules/perception/base/object.h"
#include "modules/perception/base/point_cloud.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/lidar/lib/interface/base_lidar_detector.h"

#include "modules/perception/lidar/lib/detector/dnn_common/proto/bev_fusion_detection_config.pb.h"
#include "modules/perception/lidar/lib/detector/proto/object_subtype_mapping_config.pb.h"

#include "modules/perception/lidar/lib/detector/bev_detection/bevfusion/bevfusion.hpp"
#include "modules/perception/lidar/lib/detector/dnn_common/common/tensor.hpp"
#include "modules/perception/lidar/lib/detector/dnn_common/common/check.hpp"

namespace century {
namespace perception {
namespace lidar {

using Object = base::Object;

class BevFusionDetector : public BaseLidarDetector {
 public:
  BevFusionDetector();
  virtual ~BevFusionDetector();

  bool Init(const LidarDetectorInitOptions& options =
                LidarDetectorInitOptions()) override;

  bool Detect(const LidarDetectorOptions& options, LidarFrame* frame) override;

  std::string Name() const override { return "BevFusionDetector"; }

 private:
  void CreateBevFusionCore() noexcept;
  nv::Tensor GetLidarInputTensor(LidarFrame* frame) noexcept;
  void GetObjects(
    const std::vector<bevfusion::head::transbbox::BoundingBox>& bboxes,
    LidarFrame* frame) noexcept;

 private:
  BevFusionDetectionConfig bev_config_;
  // ObjectSubTypeMapping object_subtype_mapping_config_;
  std::unordered_map<int, base::ObjectSubType> object_subtype_mapping_;
  std::shared_ptr<bevfusion::Core> bevfusion_ptr_;
  cudaStream_t stream_;

};

}  // namespace lidar
}  // namespace perception
}  // namespace century