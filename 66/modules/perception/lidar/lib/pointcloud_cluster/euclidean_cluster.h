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

#include "modules/perception/lidar/lib/pointcloud_cluster/proto/pointcloud_cluster_config.pb.h"

#include "modules/perception/lidar/lib/interface/base_pointcloud_cluster.h"
#include "modules/perception/lidar/lib/interface/base_roi_filter.h"
#include "modules/perception/lidar/lib/map_manager/map_manager.h"

namespace century {
namespace perception {
namespace lidar {

class EuclideanCluster : public BasePointCloudCluster {
 public:
  EuclideanCluster() = default;
  ~EuclideanCluster() override = default;

  bool Init(const ClusterInitOptions& options = ClusterInitOptions()) override;
  bool Cluster(const ClusterOptions& options, LidarFrame* frame) override;
  std::string Name() const override { return "EuclideanCluster"; }

 private:
  bool use_map_manager_;
  float leaf_size_{0.1f};
  int min_cluster_size_{3};
  int max_cluster_size_{25000};
  double cluster_tolerance_{0.9};
  PointCloudClusterConfig cluster_config_;

  MapManager map_manager_;
  lidar::BaseROIFilter* roi_filter_;
  std::shared_ptr<base::AttributePointCloud<base::PointF>> roi_cloud_;
};

}  // namespace lidar
}  // namespace perception
}  // namespace century