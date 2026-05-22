/******************************************************************************
 * Copyright 2018 The Century Authors. All Rights Reserved.
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

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Dense"

#include "modules/perception/base/hdmap_struct.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/base/point_cloud.h"
#include "modules/perception/base/sensor_meta.h"

namespace century {
namespace perception {
namespace lidar {

struct LidarFrame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // point cloud
  std::shared_ptr<base::AttributePointCloud<base::PointF>> cloud;
  // non ground point cloud
  std::shared_ptr<base::AttributePointCloud<base::PointF>> raw_cloud;

  std::shared_ptr<base::AttributePointCloud<base::PointF>> ego_cloud;

  // world point cloud
  std::shared_ptr<base::AttributePointCloud<base::PointD>> world_cloud;
  // vehicle point cloud
  std::shared_ptr<base::AttributePointCloud<base::PointXYZIRTF>> vehicle_cloud;
  // timestamp
  double timestamp = 0.0;
    // parsing ground height
  float parsing_ground_height = 10.0f;
  // lidar-origin ground z-value
  float original_ground_z = 10.0f;
  
  // lidar to world pose
  Eigen::Affine3d lidar2world_pose = Eigen::Affine3d::Identity();
  // lidar to world pose
  Eigen::Affine3d novatel2world_pose = Eigen::Affine3d::Identity();
  // vehicle to imu
  Eigen::Affine3d vehicle2imu_pose = Eigen::Affine3d::Identity();
  // loc data vector
  std::vector<std::pair<double, Eigen::Affine3d>> localizetion_poses;
  // hdmap struct
  std::shared_ptr<base::HdmapStruct> hdmap_struct = nullptr;
  // segmented objects
  std::vector<std::shared_ptr<base::Object>> segmented_objects;
  // tracked objects
  std::vector<std::shared_ptr<base::Object>> tracked_objects;
  // point cloud roi indices
  base::PointIndices roi_indices;
  // point cloud non ground indices
  base::PointIndices non_ground_indices;
  // secondary segmentor indices
  base::PointIndices secondary_indices;
  // sensor info
  base::SensorInfo sensor_info;
  // reserve string
  std::string reserve;

  void Reset() {
    if (cloud) {
      cloud->clear();
    }
    if (world_cloud) {
      world_cloud->clear();
    }
    if (vehicle_cloud) {
      vehicle_cloud->clear();
    }
    timestamp = 0.0;
    lidar2world_pose = Eigen::Affine3d::Identity();
    novatel2world_pose = Eigen::Affine3d::Identity();
    vehicle2imu_pose = Eigen::Affine3d::Identity();
    if (hdmap_struct) {
      hdmap_struct->road_boundary.clear();
      hdmap_struct->road_polygons.clear();
      hdmap_struct->junction_polygons.clear();
      hdmap_struct->hole_polygons.clear();
    }
    segmented_objects.clear();
    tracked_objects.clear();
    roi_indices.indices.clear();
    non_ground_indices.indices.clear();
    secondary_indices.indices.clear();
    localizetion_poses.clear();
  }

  void FilterPointCloud(base::PointCloud<base::PointF> *filtered_cloud,
                        const std::vector<uint32_t> &indices) {
    if (cloud && filtered_cloud) {
      filtered_cloud->CopyPointCloudExclude(*cloud, indices);
    }
  }
};  // struct LidarFrame

}  // namespace lidar
}  // namespace perception
}  // namespace century
