/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/perception/onboard/proto/lidar_component_config.pb.h"

#include "cyber/cyber.h"
#include "modules/perception/lidar/app/lidar_obstacle_detection.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/lidar/lib/interface/base_pointcloud_preprocessor.h"
#include "modules/perception/onboard/component/lidar_inner_component_messages.h"
#include "modules/perception/onboard/transform_wrapper/transform_wrapper.h"
#include "modules/perception/common/fps_counter.h"

namespace century {
namespace perception {
namespace onboard {

struct LidarPoseInfo {
    Eigen::Affine3d lidar2vehicle;
    Eigen::Affine3d imu2vehicle;
    Eigen::Affine3d lidar2imu;
    Eigen::Affine3d novatel2world;
};

class LidarSegmentationComponent
    : public cyber::Component<PointCloudInType> {
 public:
  LidarSegmentationComponent() = default;
  virtual ~LidarSegmentationComponent() = default;

  bool Init() override;
  bool Proc(const std::shared_ptr<PointCloudInType>& message) override;

 private:
  bool InitAlgorithmPlugin();
  bool InternalProc(
      const std::shared_ptr<const PointCloudInType>& in_message,
      const std::shared_ptr<LidarFrameMessage>& out_message);

  void DebugPCD(
      const std::string& base_path,
      const std::shared_ptr<PointCloudInType>& in_message) noexcept;

  void PublishSegmentedPointCloud(
      const std::shared_ptr<LidarFrameMessage>& out_message) noexcept;

  bool QueryLidarPoseInfo(LidarPoseInfo* pose_info) noexcept;

 private:
  static std::atomic<uint32_t> seq_num_;
  std::string sensor_name_;
  std::string detector_name_;
  bool enable_hdmap_ = true;
  bool is_query_lidar_pose_ = false;

  float lidar_query_tf_offset_ = 20.0f;
  std::string lidar2novatel_tf2_child_frame_id_;
  std::string lidar2imu_tf2_child_frame_id_;
  TransformWrapper lidar2vehicle_trans_;

  base::SensorInfo sensor_info_;

  lidar::PointCloudPreprocessorInitOptions preprocessor_init_options_;
  // algorithm plugin
  std::shared_ptr<lidar::BasePointCloudPreprocessor> ground_preprocessor_;
  std::shared_ptr<century::cyber::Writer<LidarFrameMessage>> writer_;
  std::shared_ptr<century::cyber::Writer<drivers::PointCloud>> seg_pointcloud_writer_;

  LidarPoseInfo pose_info_;
  std::unique_ptr<FpsCounter> fps_counter_;
};

CYBER_REGISTER_COMPONENT(LidarSegmentationComponent);
}  // namespace onboard
}  // namespace perception
}  // namespace century