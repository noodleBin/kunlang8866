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
#include "modules/perception/lidar/lib/pointcloud_cluster/bev_occupancy/bev_occupancy_cluster.h"
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/convex_hull.h>
#include "cyber/common/file.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/common/point_cloud_processing/common.h"
#include "modules/perception/base/singleton.h"

namespace century {
namespace perception {
namespace lidar {
namespace {
const float kMostConfidence = 1.0f;
const float kMostPossibility = 1.0f;
}

using century::cyber::common::GetAbsolutePath;

bool BevOccupancyCluster::Init(const ClusterInitOptions& options) {
  (void)options;
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
  bev_resolution_ = cluster_config_.bev_resolution();
  min_cluster_point_size_ = cluster_config_.bev_cluster_points_min_size();
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

  return true;
}

bool BevOccupancyCluster::Cluster(const ClusterOptions& options,
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

  if (use_map_manager_ && options.enable_hdmap_input) {
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

  
  OccupancyMap occ_map(bev_resolution_, frame->cloud);
  auto grid_map = occ_map.GetGridMap();

  IslandSegment segment(grid_map);
  std::vector<std::vector<int>> islands = segment.FindIslandsIndices();

  std::vector<std::shared_ptr<base::Object>> segmented_objects;
  segmented_objects.reserve(islands.size());

  // frame->segmented_objects.clear();
  auto id = frame->segmented_objects.size();
  for (auto island : islands) {
    if (island.size() < min_cluster_point_size_) {
      continue;
    }
    base::Object object;
    object.id = id++;
    object.confidence = kMostConfidence;
    object.lidar_supplement.is_in_roi = true;
    object.lidar_supplement.num_points_in_roi = island.size();
    object.lidar_supplement.cloud.CopyPointCloud(*frame->cloud, island);
  
    // classification
    object.type = base::ObjectType::UNKNOWN;
    object.sub_type = base::ObjectSubType::UNKNOWN;
    object.lidar_supplement.raw_probs.push_back(std::vector<float>(
        static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
    object.lidar_supplement.raw_probs.back()[static_cast<int>(object.type)] =
        kMostPossibility;
    object.lidar_supplement.raw_classification_methods.push_back(Name());
    // copy to type
    object.type_probs.assign(object.lidar_supplement.raw_probs.back().begin(),
                             object.lidar_supplement.raw_probs.back().end());
    // copy to background objects
    std::shared_ptr<base::Object> obj(new base::Object);
    *obj = object;
    segmented_objects.push_back(std::move(obj));
  }

  segmented_objects.resize(segmented_objects.size());

  {
    auto* mutex = base::Singleton<std::mutex>::GetInstance();
    std::lock_guard<std::mutex> lock(*mutex);
    // frame->segmented_objects.swap(segmented_objects);
    frame->segmented_objects.insert(
      frame->segmented_objects.end(),
      segmented_objects.begin(), segmented_objects.end());
  }



  return true;
}

PERCEPTION_REGISTER_POINTCLOUDCLUSTERER(BevOccupancyCluster);

}  // namespace lidar
}  // namespace perception
}  // namespace century