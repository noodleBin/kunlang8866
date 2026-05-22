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
#include "modules/perception/lidar/lib/pointcloud_cluster/pointcloud_cluster.h"

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/convex_hull.h>

// #include "modules/perception/common/geometry/convex_hull_2d.h"

static std::mutex g_mutex_;

namespace century {
namespace perception {
namespace lidar {

using PointFCloud =
    century::perception::base::AttributePointCloud<base::PointF>;
using PointFCloudPtr = std::shared_ptr<PointFCloud>;

bool EuclideanCluster::Init() noexcept { return true; }

bool EuclideanCluster::ClusterDbscan(LidarFrame* frame) noexcept {
  float cluster_tolerance_ = 2;
  float min_cluster_size_ = 3;
  float max_cluster_size_ = 25000; 

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
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
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
          neighbor_indices.reserve(50);
          
          if (tree->radiusSearch(cloud_filtered->points[seed_queue[queue_idx]], 
                                cluster_tolerance_, 
                                neighbor_indices,
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


bool EuclideanCluster::Cluster(LidarFrame* frame) noexcept {
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

  //   CloudMask mask;
  //   mask.Set(frame->cloud->size(), 0);
  //   mask.AddIndicesOfIndices(frame->roi_indices, frame->non_ground_indices,
  //   1);
  // base::PointIndices indices = frame->non_ground_indices;

  // if (indices.indices.empty()) {
  //   AERROR << "Indices are empty. Cannot create point cloud.";
  //   return false;
  // }

  std::map<int, int>
      index_mapper_;  // key: index-in-pcl_cloud, value: index-in-cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl_cloud->reserve(frame->cloud->size());
  for (size_t i = 0; i < frame->cloud->size(); ++i) {
    // int idx = indices.indices.at(i);
    index_mapper_[i] = i;
    // get point
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
  // Apply VoxelGrid filter for uniform downsampling
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(pcl_cloud);
  voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);  // Set the voxel grid leaf size
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid.filter(*downsampled_cloud);

  // Update pcl_cloud with the downsampled result
  pcl_cloud = downsampled_cloud;
  tree->setInputCloud(pcl_cloud);

  for (size_t i = 0; i < pcl_cloud->size(); ++i) {
    // int idx = indices.indices.at(i);
    index_mapper_[i] = i;
    // get point
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
  ec.setClusterTolerance(0.9);
  ec.setMinClusterSize(3);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(pcl_cloud);
  ec.extract(cluster_indices);

  frame->segmented_objects.clear();

  // get object from cluster_indices
  for (pcl::PointIndices cluster : cluster_indices) {
    if (cluster.indices.size() < 3) {
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

}  // namespace lidar
}  // namespace perception
}  // namespace century