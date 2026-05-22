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
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/perception/onboard/proto/lidar_component_config.pb.h"
#include "modules/perception/proto/perception_obstacle_debug.pb.h"

#include "cyber/cyber.h"
#include "modules/perception/common/visualization/lidar_detector_viz.h"
#include "modules/perception/lib/thread/concurrent_queue.h"
#include "modules/perception/lib/thread/thread_pool.h"
#include "modules/perception/lidar/app/lidar_obstacle_detector.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/lidar/lib/interface/base_pointcloud_cluster.h"
#include "modules/perception/lidar/lib/interface/base_roi_filter.h"
#include "modules/perception/lidar/lib/map_manager/map_manager.h"
#include "modules/perception/lidar/lib/object_builder/object_builder.h"
#include "modules/perception/lidar/lib/object_filter_bank/object_filter_bank.h"
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
/**
 * @class LidarDetectorComponent
 * @brief Component for processing LiDAR data, including detection, clustering,
 * synchronization, and visualization.
 */
class LidarDetectorComponent : public cyber::Component<> {
 public:
  /**
   * @brief Default constructor.
   */
  LidarDetectorComponent() = default;

  /**
   * @brief Destructor.
   */
  virtual ~LidarDetectorComponent();

  /**
   * @brief Initialize the component.
   * @return true if initialization is successful, false otherwise.
   */
  bool Init() override;

 private:
  /**
   * @brief Process LiDAR frames around the vehicle.
   * This function processes LiDAR frames that are within a certain distance
   * from the vehicle, typically used for obstacle detection and clustering.
   */
  void ProcessLidarFrameAroundVehicle() noexcept;

  /**
   * @brief Initialize the algorithm plugin.
   * @return true if successful, false otherwise.
   */
  bool InitAlgorithmPlugin() noexcept;

  /**
   * @brief Start receiving messages from the input channels.
   */
  void StartReceiveMsg() noexcept;

  /**
   * @brief Process a single LiDAR frame message.
   */
  void ProcessLidarFrameMessage() noexcept;

  /**
   * @brief Synchronize and merge point cloud messages based on their
   * timestamps.
   * @param tolerance Time tolerance for synchronization in seconds.
   * @return A merged point cloud message if successful, nullptr otherwise.
   */
  LidarFrameMessagePtr SyncAndMergePointCloudMessages(
      double tolerance) noexcept;

  /**
   * @brief Task for synchronizing point cloud messages.
   */
  void SyncPointCloudMsgTask() noexcept;

  /**
   * @brief Convert a LiDAR frame message to a sensor frame message.
   * @param in_message Input LiDAR frame message.
   * @param out_message Output sensor frame message.
   * @return true if conversion is successful, false otherwise.
   */
  bool InternalProc(
      const std::shared_ptr<const LidarFrameMessage>& in_message,
      const std::shared_ptr<SensorFrameMessage>& out_message) noexcept;

  /**
   * @brief Perform obstacle detection on a LiDAR frame.
   * @param frame LiDAR frame message.
   */
  void Detect(LidarFrameMessagePtr frame) noexcept;

  /**
   * @brief Perform clustering on a LiDAR frame.
   * @param frame LiDAR frame message.
   */
  void Cluster(LidarFrameMessagePtr frame) noexcept;

  /**
   * @brief Filter obstacles using HDMap information.
   * @param merged_msg Merged LiDAR frame message.
   * @param world_pose Localization estimate in world coordinates.
   * @return true if filtering is successful, false otherwise.
   */
  bool UpdateLidarPose(
      LidarFrameMessagePtr merged_msg,
      const std::shared_ptr<century::localization::LocalizationEstimate>&
          world_pose) noexcept;

  /**
   * @brief Dump the world point cloud for debugging or visualization.
   * @param ts Timestamp of the point cloud.
   * @param vehicle_cloud Point cloud in the vehicle's frame.
   * @param map_vehicle_pose Transformation from map to vehicle frame.
   */
  void DumpWorldPcl(const double ts, const base::PointFCloudPtr& vehicle_cloud,
                    const Eigen::Affine3d& map_vehicle_pose) const noexcept;

  void DumpWorldPcl(const LidarFrameMessagePtr& raw_cloud);

  /**
   * @brief For perception visualization, publish the perception obstacles
   *
   * @param merged_msg
   */
  void PublishPerceptionObstacles(
      const std::shared_ptr<century::cyber::Writer<PerceptionObstacleDebugMsg>>&
          writer,
      const LidarFrameMessagePtr& merged_msg, double tm) noexcept;

  void FilterPointCloudInBoundingBox(
      const LidarFrameMessagePtr& merged_msg) noexcept;

  /**
   * @brief Remove high-trailer objects from the LiDAR frame.
   *
   * @param merged_msg
   */
  void RemoveHighTrailerObjects(
      const LidarFrameMessagePtr& merged_msg) noexcept;

 private:
  /// Flag to enable or disable visualization debugging.
  bool use_viz_debug_ = false;

  /// Flag to enable or disable HDMap-based filtering.
  bool enable_hdmap_ = true;

  /// Flag to enable or disable the map manager.
  bool use_map_manager_ = false;

  // Flag to enable or disable the obj filter bank.
  bool use_filter_bank_ = false;

  // Flag to determine whether to use pose query with offset.
  bool use_pose_query_ = true;
  double pose_query_offset_ = 0.0;
  bool use_point_interpolation_ = false;

  /// List of input channel names for receiving LiDAR data.
  std::vector<std::string> input_channels_;

  /// Name of the obstacle detector algorithm.
  std::string detector_name_;

  /// Name of the clustering algorithm.
  std::string cluster_name_;

  /// Name of the sensor providing the LiDAR data.
  std::string sensor_name_;

  /// Name of the output channel for publishing processed data.
  std::string output_channel_name_;

  /// Name of the output channel for publishing LiDAR frames around the vehicle.
  std::string around_ego_output_channel_name_;

  std::string debug_output_channel_name_;

  std::string debug_around_output_channel_name_;

  /// Readers for receiving LiDAR frame messages.
  std::vector<LidarFrameMessageReaderPtr> lidar_readers_;

  /// Pointer to the obstacle detection algorithm.
  std::unique_ptr<lidar::BaseLidarObstacleDetection> detector_;

  /// Queue for storing synchronized point cloud messages.
  std::shared_ptr<lib::FixedSizeConQueue<LidarMsgsVecPtr>>
      pointcloud_msg_queue_;

  /// Queue for storing merged LiDAR frame messages.
  std::shared_ptr<lib::FixedSizeConQueue<LidarFrameMessagePtr>>
      merged_msg_queue_;

  /// Queue for storing LiDAR frame messages around the vehicle.
  std::shared_ptr<lib::FixedSizeConQueue<LidarFrameMessagePtr>>
      around_vehicle_queue_;
  /// Thread pool for parallel processing.
  std::unique_ptr<lib::ThreadPool> thread_pool_;

  /// Writer for publishing perception obstacles.
  std::shared_ptr<century::cyber::Writer<PerceptionObstacles>>
      perception_writer_;

  std::shared_ptr<century::cyber::Writer<PerceptionObstacleDebugMsg>>
      perception_debug_writer_;

  std::shared_ptr<century::cyber::Writer<PerceptionObstacleDebugMsg>>
      perception_debug_around_writer_;

  std::shared_ptr<century::cyber::Writer<PerceptionObstacles>>
      around_ego_perception_writer_;

  /// Reader for receiving localization estimates.
  LocalizationReaderPtr localization_reader_;

  /// Transform wrapper for LiDAR to vehicle coordinate transformation.
  TransformWrapper lidar2vehicle_trans_;

  /// Object builder for constructing detected objects.
  lidar::ObjectBuilder builder_;

  /// Processor for clustering point cloud data.
  std::unique_ptr<lidar::BasePointCloudCluster> cluster_processor_;

  /// Visualizer for LiDAR data.
  std::unique_ptr<common::LidarVisualizer> visualizer_;

  // Tool for obj filter include cone reconstruct
  lidar::ObjectFilterBank filter_bank_;
};

CYBER_REGISTER_COMPONENT(LidarDetectorComponent);
}  // namespace onboard
}  // namespace perception
}  // namespace century
