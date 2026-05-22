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
#include <thread>
#include <unordered_map>

#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/perception/onboard/proto/visualizer_component_config.pb.h"
#include "modules/perception/proto/perception_obstacle_debug.pb.h"

#include "cyber/cyber.h"
#include "modules/perception/common/visualization/lidar_detector_lite_viz.h"
#include "modules/perception/common/visualization/lidar_detector_viz.h"
#include "modules/perception/lib/thread/concurrent_queue.h"
#include "modules/perception/lib/thread/thread_pool.h"
#include "modules/perception/lidar/app/lidar_obstacle_detector.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/lidar/lib/interface/base_pointcloud_cluster.h"
#include "modules/perception/lidar/lib/interface/base_roi_filter.h"
#include "modules/perception/lidar/lib/map_manager/map_manager.h"
#include "modules/perception/lidar/lib/object_builder/object_builder.h"
#include "modules/perception/lidar/lib/pointcloud_cluster/pointcloud_cluster.h"
#include "modules/perception/onboard/component/lidar_inner_component_messages.h"
#include "modules/perception/onboard/inner_component_messages/inner_component_messages.h"
#include "modules/perception/onboard/transform_wrapper/transform_wrapper.h"

namespace century {
namespace perception {
namespace onboard {

using LidarFrameMessageReaderPtr =
    std::shared_ptr<cyber::Reader<LidarFrameMessage>>;
using LidarFrameMessagePtr = std::shared_ptr<LidarFrameMessage>;
using LocalizationReaderPtr =
    std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>>;
using LidarMsgsVecPtr = std::shared_ptr<std::vector<LidarFrameMessagePtr>>;

enum class CameraName {
  FRONT_LEFT = 1,
  FRONT_MIDDLE = 2,
  FRONT_RIGHT = 3,
  BACK_LEFT = 4,
  BACK_MIDDLE = 5,
  BACK_RIGHT = 6,
};

class LidarVisualizerComponent
    : public cyber::Component<PerceptionObstacleDebugMsg> {
 public:
  LidarVisualizerComponent() = default;
  virtual ~LidarVisualizerComponent() = default;

  bool Init() override;
  bool Proc(
      const std::shared_ptr<PerceptionObstacleDebugMsg>& message) override;

 private:
  bool InitAlgorithmPlugin();
  bool InternalProc(
      const std::shared_ptr<const PerceptionObstacleDebugMsg>& in_message,
      const std::shared_ptr<LidarFrameMessage>& out_message);
  bool UpdateLidarPose(
      LidarFrameMessagePtr merged_msg,
      const std::shared_ptr<century::localization::LocalizationEstimate>&
          localization_msg) noexcept;

  bool UpdateLidarPose(LidarFrameMessagePtr merged_msg,
                       const Affine3dProto& affine3d) noexcept;

  bool ConvertObjectFromPb(const PerceptionObstacle& pb_msg,
                           const base::ObjectPtr& object_ptr);

  bool AppendObstacleVisuals(
      const PerceptionObstacleDebugMsg& message,
      const std::shared_ptr<LidarFrameMessage>& merged_message,
      common::visualizer::PolygonVector* out_polygons,
      std::vector<common::visualizer::BBox>* out_boxes);

  void BuildHdMapVisuals(lidar::LidarFrame* frame,
                         common::visualizer::MapInfo* maps_info);

  void DoVisualize() noexcept;
  void DoVisualizePangoLin() noexcept;
  void OnReceiveImage(
      const std::shared_ptr<drivers::CompressedImage>& compressed_image);

  void OnFrontLeftReceiveImage(
      const std::shared_ptr<drivers::CompressedImage>& compressed_image);
  void OnFrontMidReceiveImage(
      const std::shared_ptr<drivers::CompressedImage>& compressed_image);
  void OnFrontRightReceiveImage(
      const std::shared_ptr<drivers::CompressedImage>& compressed_image);
  void OnRearLeftReceiveImage(
      const std::shared_ptr<drivers::CompressedImage>& compressed_image);
  void OnRearMidReceiveImage(
      const std::shared_ptr<drivers::CompressedImage>& compressed_image);
  void OnRearRightReceiveImage(
      const std::shared_ptr<drivers::CompressedImage>& compressed_image);
  bool KeyHandler(const int key);

  void ShowImages(
      const std::shared_ptr<drivers::CompressedImage>& compressed_image,
      const std::string& label) noexcept;

 private:
  std::atomic<uint32_t> camera_index_{1};
  LocalizationReaderPtr localization_reader_;
  /// Queue for storing merged LiDAR frame messages.
  std::shared_ptr<lib::FixedSizeConQueue<LidarFrameMessagePtr>>
      merged_msg_queue_;

  std::unique_ptr<common::LidarVisualizer> visualizer_;
  std::unique_ptr<common::visualizer::PangolinViewer> pango_viewer_;

  std::shared_ptr<std::thread> visualize_thread_;
  std::shared_ptr<std::thread> panolin_thread_;
  lidar::MapManager map_manager_;
  lidar::BaseROIFilter* roi_filter_;
  std::shared_ptr<base::AttributePointCloud<base::PointF>> roi_cloud_;

  bool use_map_manager_{true};
  bool use_viz_pcl_{true};
  bool use_viz_pangolin_{false};
  bool seg_viz_{true};
  bool raw_viz_{true};
  bool aeb_dis_viz_{false};
  bool grid_viz_{false};
  float vehicle_length_{0.0f};
  float vehicle_width_{0.0f};
  float vehicle_height_{0.0f};
  bool freespace_viz_{true};
};

CYBER_REGISTER_COMPONENT(LidarVisualizerComponent);

}  // namespace onboard
}  // namespace perception
}  // namespace century
