
/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License")
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
 *
 *******************************************************************************/
#pragma once

#include "modules/perception/base/point_cloud.h"

namespace century {
namespace perception {
namespace lidar {

using PointCloudTypePtr = base::PointXYZIRTFCloudPtr;
using PointXYZIRTFCloud = base::PointXYZIRTFCloud;

class RansacPlane {
 public:
  RansacPlane() {
    m_thres = 0.15;
    m_inliersNumMax = INT_MIN;
    srand((unsigned)time(NULL));
  };

  template <typename PType, typename PIndices>
  int DoRansacPlaneNoAVX(const PType& cloud, const PIndices& indices,
                         Eigen::Vector4f& vec);

 private:
  Eigen::Vector4f PlaneFitting(const std::vector<Eigen::Vector3f>& pts);

  template <class PointT>
  Eigen::Vector4f PlaneFitting(const std::shared_ptr<PointT>& cloud,
                               const std::vector<int>& indices,
                               float* filterData);

  int OperatorUnit(float* x_ptr, float* y_ptr, float* z_ptr, float* vec1_ptr,
                   float* vec2_ptr, float* vec3_ptr, float* vec4_ptr,
                   float* thres_ptr, size_t point_size);

  void GetGroundIndex(float* x_ptr, float* y_ptr, float* z_ptr, float* vec1_ptr,
                      float* vec2_ptr, float* vec3_ptr, float* vec4_ptr,
                      float* thres_ptr, size_t point_size, float* filter_data);

  int GetCandidate(const size_t& indicesSz, std::vector<int>& candidateIdxs);

 public:
  int m_inliersNumMax;
  std::vector<int> m_inliers;

 private:
  double m_thres;
  const int m_ransacIter = 80;
};

}  // namespace lidar
}  // namespace perception
}  // namespace century
