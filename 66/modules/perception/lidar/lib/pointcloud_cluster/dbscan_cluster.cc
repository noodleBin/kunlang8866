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
#include "modules/perception/lidar/lib/pointcloud_cluster/dbscan_cluster.h"

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/convex_hull.h>
#include "cyber/common/file.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace century {
namespace perception {
namespace lidar {

using century::cyber::common::GetAbsolutePath;

bool DBSCANCluster::Init(const ClusterInitOptions& options) {
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(
      config_manager->GetModelConfig("PointCloudPreprocessor", &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;

  config_file = GetAbsolutePath(work_root, "conf/perception/lidar");
  config_file =
      GetAbsolutePath(config_file, "pointcloud_cluster_config.pb.txt");
  ACHECK(
      century::cyber::common::GetProtoFromFile(config_file, &cluster_config_))
      << ", work_root: " << work_root << ", config_file: " << config_file;

  min_cluster_size_ = cluster_config_.dbscan_min_size();
  max_cluster_size_ = cluster_config_.dbscan_max_size();
  cluster_tolerance_ = cluster_config_.dbscan_tolerance();
  dbscan_max_neighbor_ = cluster_config_.dbscan_max_neighbor();

  AERROR << "cluster_config_: " << cluster_config_.euclidean_leaf_size();

  return true;
}

bool DBSCANCluster::Cluster(const ClusterOptions& options,
                            LidarFrame* frame) {

  if (frame == nullptr) {
    AERROR << "Input null frame ptr.";
    return false;
  }
  if (frame->cloud == nullptr) {
    AERROR << "Input null frame cloud.";
    return false;
  }
  if (frame->cloud->size() == 0) {
    AERROR << "Input none points.";
    return false;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
  cloud_filtered->reserve(frame->cloud->size());
  for (size_t i = 0; i < frame->cloud->size(); ++i) {
    // int idx = indices.indices.at(i);

    // get point
    auto pt = frame->cloud->at(i);
    pcl::PointXYZ pcl_pt;
    pcl_pt.x = pt.x;
    pcl_pt.y = pt.y;
    pcl_pt.z = pt.z;
    cloud_filtered->push_back(pcl_pt);
  }

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);

  // DBSCAN clustering implementation
  std::vector<bool> processed(cloud_filtered->points.size(), false);

  for (size_t i = 0; i < cloud_filtered->points.size(); i++) {
    if (processed[i]) continue;

    std::vector<int> seed_queue;
    std::vector<int> cluster_indices_temp;

    seed_queue.push_back(i);
    processed[i] = true;

    size_t queue_idx = 0;
    while (queue_idx < seed_queue.size()) {
      std::vector<int> neighbor_indices;
      std::vector<float> neighbor_distances;
      neighbor_indices.reserve(dbscan_max_neighbor_);

      if (tree->radiusSearch(cloud_filtered->points[seed_queue[queue_idx]],
                             cluster_tolerance_, neighbor_indices,
                             neighbor_distances) > 0) {
        for (const auto& neighbor_idx : neighbor_indices) {
          if (!processed[neighbor_idx]) {
            seed_queue.push_back(neighbor_idx);
            processed[neighbor_idx] = true;
          }
        }
      }
      cluster_indices_temp.push_back(seed_queue[queue_idx]);
      queue_idx++;
    }

    if (cluster_indices_temp.size() >= min_cluster_size_ &&
        cluster_indices_temp.size() <= max_cluster_size_) {
      pcl::PointIndices indices;
      indices.indices = std::move(cluster_indices_temp);
      cluster_indices.push_back(indices);
    }
  }
  for (pcl::PointIndices cluster : cluster_indices) {
    if (cluster.indices.size() < 3) {
      continue;
    }
    // get cluster point index in cloud
    std::vector<int> point_idx;
    for (auto idx : cluster.indices) {
      point_idx.push_back(idx);
    }
    base::Object object;
    object.confidence = 1.0;
    object.lidar_supplement.is_in_roi = true;
    object.lidar_supplement.num_points_in_roi = cluster.indices.size();
    object.lidar_supplement.cloud.CopyPointCloud(*frame->cloud, point_idx);

    // classification
    object.type = base::ObjectType::UNKNOWN;
    object.lidar_supplement.raw_probs.push_back(std::vector<float>(
        static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
    object.lidar_supplement.raw_probs.back()[static_cast<int>(object.type)] =
        1.0f;
    object.lidar_supplement.raw_classification_methods.push_back(Name());
    // copy to type
    object.type_probs.assign(object.lidar_supplement.raw_probs.back().begin(),
                             object.lidar_supplement.raw_probs.back().end());
    // copy to background objects
    std::shared_ptr<base::Object> obj(new base::Object);
    *obj = object;
    frame->segmented_objects.push_back(std::move(obj));
  }
  return true;
}

PERCEPTION_REGISTER_POINTCLOUDCLUSTERER(DBSCANCluster);

}  // namespace lidar
}  // namespace perception
}  // namespace century