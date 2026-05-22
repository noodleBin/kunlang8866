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
#include "modules/localization/msf/common/lidar/lidar_feature.h"

namespace century {
namespace loc {

const loc::PointCloudXYZI& LidarFeature::GetFeatureCloud(
    loc::FeatureType feature_type) const {
  switch (feature_type) {
    case LINE:
      return line_feature_points_;
    case PLANE:
      return plane_feature_points_;
    default:
      AERROR << "GetFeatureCloud: feature type not supported!";
      break;
  }
  return empty_points_;
}

void LidarFeature::GetFeatureCloud(loc::FeatureType feature_type,
                                   loc::PointCloudXYZI* point_cloud) const {
  switch (feature_type) {
    case LINE:
      *point_cloud = line_feature_points_;
      break;
    case PLANE:
      *point_cloud = plane_feature_points_;
      break;
    default:
      AERROR << "GetFeatureCloud: feature type not supported!";
      break;
  }
}

void LidarFeature::Clear() {
  curvatures_.clear();
  is_bad_points_.clear();

  line_feature_points_.clear();
  plane_feature_points_.clear();
}
bool LidarFeature::ComputeFeatures(const loc::PointCloudXYZIRT& pcd) {
  Clear();

  ring_size_ = pcd.size();
  curvatures_.resize(ring_size_, 0);
  is_bad_points_.resize(ring_size_, false);
  if (ComputeCurvature(pcd)) {
    RemoveBadPoints(pcd);
    ComputeFeature(pcd);
  }

  return true;
}

bool LidarFeature::ComputeCurvature(const loc::PointCloudXYZIRT& pcd) {
  const int window_radius = param_.sliding_window_radius;
  if (ring_size_ <= 2 * window_radius) {
    return false;
  }

  std::vector<Eigen::Vector3d> temp_p;
  for (int i = window_radius; i < ring_size_ - window_radius; ++i) {
    temp_p.emplace_back(Eigen::Vector3d(pcd[i].x, pcd[i].y, pcd[i].z));
  }

  auto findKthSmallest = [](std::vector<float>& nums, int k) {
    int n = nums.size();
    for (int i = 0; i < n; ++i) {
      int minIndex = i;
      for (int j = i + 1; j < n; ++j) {
        if (nums[j] < nums[minIndex]) {
          minIndex = j;
        }
      }
      std::swap(nums[i], nums[minIndex]);
      if (i == k - 1) {
        return nums[i];
      }
    }
    return nums[k];
  };

  std::vector<float> temp_curvature(window_radius);
  int mid_index = window_radius / 2;
  for (int i = window_radius; i < ring_size_ - window_radius; ++i) {
    int p_idx = i - window_radius;
    Eigen::Vector3d p = temp_p[p_idx];
    for (int j = 1; j <= window_radius; ++j) {
      int p1_idx = i - j - window_radius;
      int p2_idx = i + j - window_radius;
      Eigen::Vector3d p1 = temp_p[p1_idx];
      Eigen::Vector3d p2 = temp_p[p2_idx];
      auto p1_p = p1 - p;
      auto p2_p = p2 - p;
      temp_curvature[j - 1] =
          (p1_p).dot(p2_p) / ((p1_p).norm() * (p2_p).norm());
    }

    curvatures_[i] = findKthSmallest(temp_curvature, mid_index);
  }
  return true;
}

void LidarFeature::RemoveBadPoints(const loc::PointCloudXYZIRT& pcd) {
  const int window_radius = param_.sliding_window_radius;

  for (int i = window_radius; i < ring_size_ - window_radius; ++i) {
    Eigen::Vector3d p(pcd[i].x, pcd[i].y, pcd[i].z);
    Eigen::Vector3d pi(pcd[i + 1].x, pcd[i + 1].y, pcd[i + 1].z);

    float theta = acos(p.dot(pi) / (p.norm() * pi.norm())) * 180.0 / M_PI;

    if (theta > 1) {
      for (int j = 0; j <= window_radius; ++j) {
        is_bad_points_[i + j] = true;
        is_bad_points_[i - j] = true;
      }
    }
  }
}

void LidarFeature::ComputeFeature(const loc::PointCloudXYZIRT& pcd) {
  const int window_radius = param_.sliding_window_radius;
  const int buckets_number = param_.buckets_number;

  if (buckets_number < 2) {
    AERROR << "Buckets number too small!";
    return;
  }

  std::vector<std::pair<int, int>> buckets_range;
  int each_bucket_number = (ring_size_ - 2 * window_radius) / buckets_number;
  int beg = window_radius;
  int end = beg + each_bucket_number;
  buckets_range.emplace_back(std::make_pair(beg, end));

  for (int i = 0; i < buckets_number - 2; ++i) {
    beg += each_bucket_number;
    end += each_bucket_number;
    buckets_range.emplace_back(std::make_pair(beg, end));
  }

  beg = end;
  end = ring_size_ - window_radius;
  buckets_range.emplace_back(std::make_pair(beg, end));
  ComputeOnRing(pcd, buckets_range);
}

void LidarFeature::ComputeOnRing(
    const loc::PointCloudXYZIRT& pcd,
    const std::vector<std::pair<int, int>>& buckets_range) {
  const float line_threshold = param_.line_curvature_threshold;
  const float plane_threshold = param_.plane_curvature_threshold;

  std::vector<int> points_index(ring_size_);
  for (int i = 0; i < ring_size_; ++i) {
    points_index[i] = i;
  }

  for (int i = 0; i < param_.buckets_number; ++i) {
    int beg_idx = buckets_range[i].first;
    int end_idx = buckets_range[i].second;

    sort(points_index.begin() + beg_idx, points_index.begin() + end_idx,
         [&](int a, int b) { return curvatures_[a] > curvatures_[b]; });

    for (int j = beg_idx; j < end_idx; ++j) {
      int index = points_index[j];

      if (curvatures_[index] > line_threshold &&
          false == is_bad_points_[index]) {
        pcl::PointXYZI p;
        loc::PointXYZIRT q = pcd[index];
        p.x = q.x;
        p.y = q.y;
        p.z = q.z;
        p.intensity = q.intensity;

        line_feature_points_.emplace_back(p);
        is_bad_points_[index] = true;
        RemoveSurroundedPoints(pcd, index, param_.line_distance_threshold);
      } else if (curvatures_[index] < line_threshold) {
        break;
      }
    }

    for (int j = end_idx - 1; j >= beg_idx; --j) {
      int index = points_index[j];

      if (curvatures_[index] < plane_threshold &&
          false == is_bad_points_[index]) {
        pcl::PointXYZI p;
        loc::PointXYZIRT q = pcd[index];
        p.x = q.x;
        p.y = q.y;
        p.z = q.z;
        p.intensity = q.intensity;

        plane_feature_points_.emplace_back(p);
        is_bad_points_[index] = true;
        RemoveSurroundedPoints(pcd, index, param_.plane_distance_threshold);
      }
    }
  }
}

void LidarFeature::RemoveSurroundedPoints(const loc::PointCloudXYZIRT& pcd,
                                          int point_index, float distance) {
  const int window_radius = param_.sliding_window_radius;
  const int px = point_index;

  Eigen::Vector3d p(pcd[px].x, pcd[px].y, pcd[px].z);

  for (int i = 1; i <= window_radius; ++i) {
    Eigen::Vector3d pi(pcd[px - i].x, pcd[px - i].y,
                       pcd[px - i].z);
    if ((p - pi).norm() < distance) {
      is_bad_points_[px - i] = true;
    } else {
      break;
    }
  }

  for (int j = 1; j <= window_radius; ++j) {
    Eigen::Vector3d pj(pcd[px + j].x, pcd[px + j].y,
                       pcd[px + j].z);
    if ((p - pj).norm() < distance) {
      is_bad_points_[px + j] = true;
    } else {
      break;
    }
  }
}

}  // namespace loc
}  // namespace century
