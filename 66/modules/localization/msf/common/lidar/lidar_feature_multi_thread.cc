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
#include "modules/localization/msf/common/lidar/lidar_feature_multi_thread.h"

namespace century {
namespace loc {

LidarFeatureMultiThread::LidarFeatureMultiThread() {
  int lidar_index = 0;
  for (int i = 0; i < param_.sum_lidar; ++i) {
    for (int j = 0; j < param_.ring_number; ++j) {
      ring_hash_.emplace_back(std::make_pair(lidar_index, j));
    }
    ++lidar_index;
  }

  line_feature_points_.reserve(param_.space_num);
  plane_feature_points_.reserve(param_.space_num);
  lidar_features_.resize(param_.ring_number);

  thread_pool_ = std::make_unique<mmath::ThreadPool>(param_.thread_num);
}

void LidarFeatureMultiThread::AddPointCloud(
    const loc::PointCloudXYZIRT& points) {
  PointCloudXYZIRTVector vec_points;
  SeperatePointCloud(points, &vec_points);
  AddPointCloud(vec_points);
}

void LidarFeatureMultiThread::AddPointCloud(
    const PointCloudXYZIRTVector& points) {
  std::vector<std::future<bool>> results;
  for (unsigned i = 0; i < points.size(); ++i) {
    Clear();
    LayoutByRing(points[i]);
    for (int j = 0; j < param_.ring_number; ++j) {
      results.emplace_back(thread_pool_->enqueue(&LidarFeature::ComputeFeatures,
                                                 &lidar_features_[j],
                                                 lidar_rings_[j]));
    }
  }
  for (auto& result : results) {
    result.wait();
  }

  for (const auto& lidar_features : lidar_features_) {
    line_feature_points_ +=
        lidar_features.GetFeatureCloud(loc::FeatureType::LINE);
    plane_feature_points_ +=
        lidar_features.GetFeatureCloud(loc::FeatureType::PLANE);
  }
}

void LidarFeatureMultiThread::GetFeaturePointCloud(
    loc::PointCloudXYZI* line_feature, loc::PointCloudXYZI* plane_feature) {
  line_feature->clear();
  plane_feature->clear();
  *line_feature = line_feature_points_;
  *plane_feature = plane_feature_points_;
}

void LidarFeatureMultiThread::Clear() {
  line_feature_points_.clear();
  plane_feature_points_.clear();
  lidar_rings_.clear();
}

void LidarFeatureMultiThread::SeperatePointCloud(
    const loc::PointCloudXYZIRT& points, PointCloudXYZIRTVector* vec_points) {
  vec_points->clear();
  vec_points->resize(param_.sum_lidar);

  for (unsigned i = 0; i < points.size(); ++i) {
    loc::PointXYZIRT p = points[i];
    if (p.ring >= ring_hash_.size()) {
      continue;
    }
    p.ring = ring_hash_[p.ring].second;
    (*vec_points)[ring_hash_[p.ring].first].emplace_back(p);
  }
}

void LidarFeatureMultiThread::LayoutByRing(
    const loc::PointCloudXYZIRT& point_cloud) {
  lidar_rings_.resize(param_.ring_number);
  const int points_size = point_cloud.size();

  for (int i = 0; i < points_size; ++i) {
    unsigned ring = point_cloud[i].ring;
    lidar_rings_[ring].emplace_back(point_cloud[i]);
  }
}

}  // namespace loc
}  // namespace century
