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

#include "modules/localization/msf/common/io/pcl_point_types.h"
#include "modules/localization/msf/common/lidar/lidar_feature.h"

namespace century {
namespace loc {

class LidarFeatureMultiThread {
 public:
  typedef std::vector<loc::PointCloudXYZIRT,
                      Eigen::aligned_allocator<loc::PointCloudXYZIRT>>
      PointCloudXYZIRTVector;

  struct Parameter {
    int thread_num = 2;
    int ring_number = 32;
    int sum_lidar = 1;
    std::size_t space_num = 10000;
  };

  LidarFeatureMultiThread();

  const std::vector<LidarFeature>& GetLidarFeatures() const {
    return lidar_features_;
  }

  void AddPointCloud(const loc::PointCloudXYZIRT& points);
  void AddPointCloud(const PointCloudXYZIRTVector& points);
  void GetFeaturePointCloud(loc::PointCloudXYZI* line_feature,
                            loc::PointCloudXYZI* plane_feature);
  void Clear();

 private:
  void SeperatePointCloud(const loc::PointCloudXYZIRT& point_cloud,
                          PointCloudXYZIRTVector* vec_points);
  void LayoutByRing(const loc::PointCloudXYZIRT& point_cloud);

  Parameter param_;
  std::vector<std::pair<int, int>> ring_hash_;
  std::vector<LidarFeature> lidar_features_;
  std::unique_ptr<mmath::ThreadPool> thread_pool_;

  loc::PointCloudXYZI line_feature_points_;
  loc::PointCloudXYZI plane_feature_points_;
  std::vector<loc::PointCloudXYZIRT> lidar_rings_;
};

}  // namespace loc
}  // namespace century
