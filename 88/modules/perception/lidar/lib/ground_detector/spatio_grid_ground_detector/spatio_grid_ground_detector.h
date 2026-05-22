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

#pragma once

#include <string>
#include <vector>

#include <pcl/pcl_base.h>

#include "modules/perception/common/i_lib/pc/i_ground.h"
#include "modules/perception/lidar/lib/ground_detector/spatio_grid_ground_detector/pointcloud_ransac_plane.h"
#include "modules/perception/lidar/lib/interface/base_ground_detector.h"
#include "modules/perception/lidar/lib/scene_manager/ground_service/ground_service.h"
#include "modules/perception/lidar/lib/scene_manager/scene_manager.h"
#include "modules/perception/lidar/lib/ground_detector/spatio_grid_ground_detector/proto/spatio_grid_ground_detector_config.pb.h"


namespace century {
namespace perception {
namespace lidar {
using IndicesPtr = pcl::IndicesPtr;
// using PointCloudTypePtr = base::PointFCloudPtr;
using PointXYZIRTFCloud = base::PointXYZIRTFCloud;


enum class PortType {
  QINGDAO_PORT = 0,
  DONGJIAZHEN_PORT = 1,
};

class SpatioGridGroundDetector : public BaseGroundDetector {
 public:
  SpatioGridGroundDetector() = default;
  ~SpatioGridGroundDetector() {}

  bool Init(const GroundDetectorInitOptions& options =
                GroundDetectorInitOptions()) override;

  bool Detect(const GroundDetectorOptions& options, LidarFrame* frame) override;

  std::string Name() const override { return "SpatioGridGroundDetector"; }

 private:
  bool RemoveGroundPointsWithoutDownSampling(
      const PointCloudTypePtr inCloud, std::vector<int>& non_ground_indices);

  void FilterPointCloud(LidarFrame* frame) noexcept;

  void FindGroundCandidates(const PointCloudTypePtr& inCloud,
                            IndicesPtr& outIndices,
                            double cell_size = 0.4) noexcept;

  void FilterClosePoints(const std::vector<std::vector<int>>& indices,
                         const int size, IndicesPtr& outIndices) noexcept;

  void CalVecToTransform(const Eigen::Vector4f& vec,
                         Eigen::Affine3f& pos) noexcept;

 private:
  float guass_ground_height_ = 0.0f;
  float original_intensity_threshold_ = 3.0f;
  float dust_intensity_threshold_ = 6.0f;
  float range_around_forward_x_ = 20.0f;
  float range_around_forward_y_ = 10.0f;
  float range_around_backward_x_ = -20.0f;
  float range_around_backward_y_ = -10.0f;
  std::vector<int> origin_points_indices_;
  std::map<int, int> local2origin_indices_;
  PortType port_type_ = PortType::QINGDAO_PORT;

  SpatioGridGroundDetectorConfig config_params_;

};  // class SpatioGridGroundDetector

}  // namespace lidar
}  // namespace perception
}  // namespace century