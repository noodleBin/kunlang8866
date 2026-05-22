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
#include "modules/perception/lidar/lib/pointcloud_cluster/euclidean_cluster.h"

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

#include "cyber/common/file.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/common/point_cloud_processing/common.h"

namespace century {
namespace perception {
namespace lidar {

using century::cyber::common::GetAbsolutePath;

bool EuclideanCluster::Init(const ClusterInitOptions& options) {
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

  min_cluster_size_ = cluster_config_.euclidean_min_size();
  max_cluster_size_ = cluster_config_.euclidean_max_size();
  cluster_tolerance_ = cluster_config_.euclidean_tolerance();
  leaf_size_ = cluster_config_.euclidean_leaf_size();
  use_map_manager_ = cluster_config_.use_map_manager();

  use_map_manager_ = use_map_manager_ && options.enable_hdmap_input;

  if (use_map_manager_) {
    lidar::MapManagerInitOptions map_manager_init_options;
    if (!map_manager_.Init(map_manager_init_options)) {
      AINFO << "Failed to init map manager.";
      use_map_manager_ = false;
    }
  }

  roi_cloud_ = base::PointFCloudPool::Instance().Get();
  roi_filter_ =
      lidar::BaseROIFilterRegisterer::GetInstanceByName("HdmapROIFilter");
  CHECK_NOTNULL(roi_filter_);
  lidar::ROIFilterInitOptions roi_filter_init_options;
  ACHECK(roi_filter_->Init(roi_filter_init_options))
      << "Failed to init roi filter.";

  AERROR << "cluster_config_: " << cluster_config_.euclidean_leaf_size();
  return true;
}

bool EuclideanCluster::Cluster(const ClusterOptions& options,
                               LidarFrame* frame) {
  // check input
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

  if (use_map_manager_) {
    MapManagerOptions map_manager_options;
    if (!map_manager_.Update(map_manager_options, frame)) {
      AERROR << "Failed to update map structure.";
      return false;
    }

    common::TransformPointCloud(*frame->cloud,
      frame->lidar2world_pose,
      frame->world_cloud.get());

    lidar::ROIFilterOptions roi_filter_options;
    if (frame->hdmap_struct != nullptr &&
        roi_filter_->Filter(roi_filter_options,
                            frame)) {
      roi_cloud_->CopyPointCloud(*frame->cloud,
                                 frame->roi_indices);
      frame->cloud = roi_cloud_;
    }
  }

  std::map<int, int>
      index_mapper_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl_cloud->reserve(frame->cloud->size());
  for (size_t i = 0; i < frame->cloud->size(); ++i) {
    index_mapper_[i] = i;
    auto pt = frame->cloud->at(i);
    pcl::PointXYZ pcl_pt;
    pcl_pt.x = pt.x;
    pcl_pt.y = pt.y;
    pcl_pt.z = pt.z;
    pcl_cloud->push_back(pcl_pt);
  }

  // kd tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(pcl_cloud);
  voxel_grid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid.filter(*downsampled_cloud);

  pcl_cloud = downsampled_cloud;
  tree->setInputCloud(pcl_cloud);

  for (size_t i = 0; i < pcl_cloud->size(); ++i) {
    index_mapper_[i] = i;
    auto pt = pcl_cloud->at(i);
    base::PointF pcl_ptf;
    pcl_ptf.x = pt.x;
    pcl_ptf.y = pt.y;
    pcl_ptf.z = pt.z;
    (*frame->cloud)[i] = pcl_ptf;
  }

  frame->cloud->resize(pcl_cloud->size());

  // do euclidean cluster
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(pcl_cloud);
  ec.extract(cluster_indices);

  frame->segmented_objects.clear();

  // get object from cluster_indices
  for (pcl::PointIndices cluster : cluster_indices) {
    if (cluster.indices.size() < min_cluster_size_) {
      continue;
    }
    // get cluster point index in cloud
    std::vector<int> point_idx;
    for (auto idx : cluster.indices) {
      point_idx.push_back(index_mapper_.at(idx));
    }
    base::Object object;
    object.confidence = 1.0;
    object.lidar_supplement.is_in_roi = true;
    object.lidar_supplement.num_points_in_roi = cluster.indices.size();
    object.lidar_supplement.cloud.CopyPointCloud(*frame->cloud, point_idx);
    // object.lidar_supplement.cloud_world.CopyPointCloud(*frame->world_cloud,
    //                                                    point_idx);

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

PERCEPTION_REGISTER_POINTCLOUDCLUSTERER(EuclideanCluster);

}  // namespace lidar
}  // namespace perception
}  // namespace century 