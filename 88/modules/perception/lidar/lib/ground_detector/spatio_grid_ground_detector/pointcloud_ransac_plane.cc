
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

#include "modules/perception/lidar/lib/ground_detector/spatio_grid_ground_detector/pointcloud_ransac_plane.h"

namespace century {
namespace perception {
namespace lidar {
template <typename PType, typename PIndices>
int RansacPlane::DoRansacPlaneNoAVX(const PType& cloud, const PIndices& indices,
                                    Eigen::Vector4f& vec) {
  if (cloud == nullptr || indices.size() == 0) {
    return -1;
  }

  const std::size_t indicesSz = indices.size();
  if (indicesSz < 3) {
    vec = Eigen::Vector4f(0, 0, 1, 0);
    return -1;
  }

  m_inliersNumMax = INT_MIN;

  m_inliers.clear();

  Eigen::Vector4f vecMax;
  std::vector<int> candidateIdxs;

  for (int iter = 0; iter < m_ransacIter; ++iter) {
    candidateIdxs.clear();
    int flag = GetCandidate(indicesSz, candidateIdxs);
    if (flag != 0) {
      continue;
    }

    std::vector<Eigen::Vector3f> candidate(3);
    for (size_t i = 0; i < 3; ++i) {
      auto& pt = cloud->at(indices.at(candidateIdxs[i]));
      candidate[i](0) = pt.x;
      candidate[i](1) = pt.y;
      candidate[i](2) = pt.z;
    }

    Eigen::Vector4f vecTemp = PlaneFitting(candidate);

    int inlinerNum = 0;
    std::vector<int> inliersIdxTmp(indicesSz);

    for (size_t i = 0; i < indicesSz; ++i) {
      double err = fabs(cloud->at(indices.at(i)).x * vecTemp(0) +
                        cloud->at(indices.at(i)).y * vecTemp(1) +
                        cloud->at(indices.at(i)).z * vecTemp(2) + vecTemp(3));

      if (err < m_thres) {
        inliersIdxTmp.at(inlinerNum) = indices.at(i);
        ++inlinerNum;
      }

      if (inlinerNum + indicesSz - 1 - i < m_inliersNumMax) {
        break;
      }
    }

    if (inlinerNum > m_inliersNumMax) {
      m_inliersNumMax = inlinerNum;
      inliersIdxTmp.resize(m_inliersNumMax);
      m_inliers = inliersIdxTmp;
      vecMax = vecTemp;
    }
  }

  if (m_inliersNumMax < 3) {
    vec = Eigen::Vector4f(0, 0, 1, 0);
    return -2;
  }

  std::vector<Eigen::Vector3f> inliner_pts(m_inliersNumMax);
  for (int i = 0; i < m_inliersNumMax; ++i) {
    auto& pt = cloud->at(m_inliers.at(i));
    inliner_pts[i](0) = pt.x;
    inliner_pts[i](1) = pt.y;
    inliner_pts[i](2) = pt.z;
  }
  vec = PlaneFitting(inliner_pts);

  return 0;
}

template int RansacPlane::DoRansacPlaneNoAVX(const PointCloudTypePtr& cloud,
                                             const std::vector<int>& indices,
                                             Eigen::Vector4f& vec);

Eigen::Vector4f RansacPlane::PlaneFitting(
    const std::vector<Eigen::Vector3f>& pts) {
  if (pts.size() == 0) {
    return Eigen::Vector4f(0, 0, 1, 0);
  }
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  for (const auto& pt : pts) center += pt;
  center /= pts.size();

  Eigen::MatrixXf A(pts.size(), 3);
  for (int i = 0; i < pts.size(); i++) {
    A(i, 0) = pts[i](0) - center(0);
    A(i, 1) = pts[i](1) - center(1);
    A(i, 2) = pts[i](2) - center(2);
  }

  Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinV);
  const float a = svd.matrixV()(0, 2);
  const float b = svd.matrixV()(1, 2);
  const float c = svd.matrixV()(2, 2);
  const float d = -(a * center(0) + b * center(1) + c * center(2));
  return Eigen::Vector4f(a, b, c, d);
};

template <class PointT>
Eigen::Vector4f RansacPlane::PlaneFitting(const std::shared_ptr<PointT>& cloud,
                                          const std::vector<int>& indices,
                                          float* filterData) {
  std::vector<Eigen::Vector3f> filterPts;
  size_t sz = indices.size();
  filterPts.resize(m_inliersNumMax);
  m_inliers.resize(m_inliersNumMax);
  int count = 0;
  for (size_t i = 0; i < sz; ++i) {
    if (*(filterData + i) != 0) {
      auto& pt = cloud->at(indices.at(i));
      filterPts[count][0] = pt.x;
      filterPts[count][1] = pt.y;
      filterPts[count][2] = pt.z;
      m_inliers.at(count) = indices.at(i);
      ++count;
    }
  }

  if (count != m_inliersNumMax) {
  }
  return PlaneFitting(filterPts);
};

template Eigen::Vector4f RansacPlane::PlaneFitting(
    const std::shared_ptr<base::PointFCloud>& cloud,
    const std::vector<int>& indices, float* filterData);

int RansacPlane::GetCandidate(const size_t& indicesSz,
                              std::vector<int>& candidateIdxs) {
  int count = 0;
  while (++count < 100) {
    int idx = rand() % indicesSz;
    bool flag = true;
    for (int i = 0; i < candidateIdxs.size(); ++i) {
      if (idx == candidateIdxs[i]) {
        flag = false;
        break;
      }
    }

    if (flag) {
      candidateIdxs.push_back(idx);
    }

    if (candidateIdxs.size() == 3) {
      return 0;
    }
  }
  return -1;
}

}  // namespace lidar
}  // namespace perception
}  // namespace century