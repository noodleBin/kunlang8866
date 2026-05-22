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

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "modules/perception/lidar/lib/detector/pointpillars_detection_nv/proto/pointpillars_detection_config.pb.h"

#include "modules/perception/base/object.h"
#include "modules/perception/base/point_cloud.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/lidar/lib/detector/pointpillars_detection_nv/pointpillar.hpp"
#include "modules/perception/lidar/lib/interface/base_lidar_detector.h"

namespace century {
namespace perception {
namespace lidar {

using Object = base::Object;

class PointPillarsDetector : public BaseLidarDetector {
 public:
  PointPillarsDetector();
  virtual ~PointPillarsDetector();

  bool Init(const LidarDetectorInitOptions& options =
                LidarDetectorInitOptions()) override;

  bool Detect(const LidarDetectorOptions& options, LidarFrame* frame) override;

  std::string Name() const override { return "PointPillarsDetector"; }

 private:
  std::shared_ptr<pointpillar::lidar::Core> CreateCore() noexcept;
  void GetObjects(const std::vector<pointpillar::lidar::BoundingBox>& bboxes,
                  LidarFrame* frame) noexcept;
  base::ObjectSubType GetObjectSubType(const int label);

  void GetBoxCorner(std::vector<float>& box_corner,
                    std::vector<float>& box_rectangular,
                    std::vector<std::shared_ptr<Object>>* objects) noexcept;

  void GetBoxIndices(const std::vector<float>& box_corner,
                     const std::vector<float>& box_rectangular,
                     std::vector<std::shared_ptr<Object>>* objects) noexcept;

 private:
  uint32_t mem_size_{0};
  std::shared_ptr<pointpillar::lidar::Core> pointpillar_ptr_;
  std::vector<float> input_raw_;

  cudaStream_t stream_;
  PointpillarsDetectionConfig pointpillars_detection_config_;

  std::shared_ptr<base::AttributePointCloud<base::PointF>> original_cloud_;
};

}  // namespace lidar
}  // namespace perception
}  // namespace century