/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <functional>
#include <limits>
#include <list>
#include <string>
#include <utility>
#include <vector>

#include "modules/perception/common/i_lib/pc/i_ground.h"
#include "modules/perception/lidar/lib/interface/base_ground_detector.h"
#include "modules/perception/lidar/lib/scene_manager/ground_service/ground_service.h"
#include "modules/perception/lidar/lib/scene_manager/scene_manager.h"

namespace century {
namespace perception {
namespace lidar {

class SpatioTemporalGroundDetectorV2 : public BaseGroundDetector {
 public:
  SpatioTemporalGroundDetectorV2() = default;
  ~SpatioTemporalGroundDetectorV2() {
    if (nullptr != pfdetector_) {
      delete pfdetector_;
    }
    if (nullptr != param_) {
      delete param_;
    }
  }

  bool Init(const GroundDetectorInitOptions& options =
                GroundDetectorInitOptions()) override;

  bool Detect(const GroundDetectorOptions& options, LidarFrame* frame) override;

  std::string Name() const override { return "SpatioTemporalGroundDetectorV2"; }

 private:
  bool ValidateInput(LidarFrame* frame);
  void UpdateCloudCenter(LidarFrame* frame);
  size_t GetNumPoints(LidarFrame* frame);
  void ReallocateBuffers(size_t num_points);
  void CopyPointDataWithRoi(LidarFrame* frame, size_t num_points,
                            unsigned int* data_id,
                            unsigned int* valid_point_num);
  void CopyPointDataWithoutRoi(LidarFrame* frame, size_t num_points,
                               unsigned int* data_id,
                               unsigned int* valid_point_num);
  bool PreparePointData(LidarFrame* frame, size_t num_points,
                        unsigned int* data_id, unsigned int* valid_point_num);
  bool RunGroundDetector(LidarFrame* frame, unsigned int valid_point_num);
  float GetPointHeight(LidarFrame* frame, size_t pc_index, float pc[3],
                       int count,
                       const std::function<bool(int, int)>& valid_index_func);
  float ComputeAdaptiveThreshold(const Eigen::Vector3d& pc_novatel);
  void ClassifyAndUpdatePoints(
      LidarFrame* frame, size_t num_points, size_t* ground_z_value_count,
      float* ground_z_value,
      const std::function<bool(int, int)>& valid_index_func);
  void UpdateGroundHeight(LidarFrame* frame, size_t ground_z_value_count,
                          float ground_z_value);
  void OutputDebugInfo(LidarFrame* frame);
  void UpdateGroundService();

  common::PlaneFitGroundDetectorParam* param_ = nullptr;
  common::PlaneFitGroundDetector* pfdetector_ = nullptr;
  std::vector<float> data_;
  std::vector<float> ground_height_signed_;
  std::vector<int> point_indices_temp_;
  std::vector<std::pair<int, int>> point_attribute_;

  bool use_roi_ = true;
  bool use_ground_service_ = false;
  bool use_semantic_ground_ = false;
  float ground_thres_ = 0.25f;
  float near_range_dist_ = 15.0f;
  float near_range_ground_thres_ = 0.10f;
  float middle_range_dist_ = 15.0f;
  float middle_range_ground_thres_ = 0.10f;
  int grid_size_ = 256;

  float ori_sample_z_lower_ = -3.0f;
  float ori_sample_z_upper_ = -1.0f;
  float parsing_height_buffer_ = 0.2f;
  bool debug_output_ = false;
  bool single_ground_detect_ = true;

  size_t default_point_size_ = 320000;
  Eigen::Vector3d cloud_center_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  GroundServiceContent ground_service_content_;

  std::list<float> origin_ground_z_array_;
  const size_t ground_z_average_frame = 5;
};  // class SpatioTemporalGroundDetectorV2

}  // namespace lidar
}  // namespace perception
}  // namespace century
