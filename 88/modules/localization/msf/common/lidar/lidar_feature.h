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

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "third_party/mmath/eigen_types.h"
#include "third_party/mmath/se3.h"
#include "third_party/mmath/thread_pool.h"

#include "cyber/common/log.h"
#include "modules/localization/msf/common/io/pcl_point_types.h"

namespace century {
namespace loc {
enum FeatureType { LINE = 0, PLANE = 1 };
class LidarFeature {
 public:
  struct Parameter {
    int sliding_window_radius = 10;
    int buckets_number = 6;
    float line_curvature_threshold = -0.5;
    float plane_curvature_threshold = -0.866;
    float line_distance_threshold = 0.5;
    float plane_distance_threshold = 0.2;
  };

  LidarFeature() = default;

  const loc::PointCloudXYZI& GetFeatureCloud(
      loc::FeatureType feature_type) const;

  void GetFeatureCloud(loc::FeatureType feature_type,
                       loc::PointCloudXYZI* points) const;

  bool ComputeFeatures(const loc::PointCloudXYZIRT& point_cloud);

  void Clear();

 private:
  bool ComputeCurvature(const loc::PointCloudXYZIRT& pcd);
  void RemoveBadPoints(const loc::PointCloudXYZIRT& pcd);
  void ComputeFeature(const loc::PointCloudXYZIRT& pcd);
  void ComputeOnRing(const loc::PointCloudXYZIRT& pcd,
                     const std::vector<std::pair<int, int>>& buckets_range);
  void RemoveSurroundedPoints(const loc::PointCloudXYZIRT& pcd, int point_index,
                              float distance);

  Parameter param_;
  loc::PointCloudXYZI empty_points_;
  int ring_size_;
  std::vector<float> curvatures_;
  std::vector<bool> is_bad_points_;

  loc::PointCloudXYZI line_feature_points_;
  loc::PointCloudXYZI plane_feature_points_;
};

}  // namespace loc
}  // namespace century
