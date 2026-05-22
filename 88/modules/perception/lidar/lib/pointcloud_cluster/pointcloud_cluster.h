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
#include <map>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include "modules/perception/lidar/common/cloud_mask.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/lidar/common/lidar_log.h"

namespace century {
namespace perception {
namespace lidar {
class EuclideanCluster {
 public:
  EuclideanCluster() = default;
  ~EuclideanCluster() = default;

 public:
  bool Init() noexcept;
  bool Cluster(LidarFrame* frame) noexcept;
  bool ClusterDbscan(LidarFrame* frame) noexcept;
  std::string Name() const noexcept { return "EuclideanCluster"; }
};
}  // namespace lidar
}  // namespace perception
}  // namespace century