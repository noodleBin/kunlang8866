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

#include <array>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/proto/sensor_image.pb.h"
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
#include "modules/perception/lidar/lib/freespace/freespace_mask.h"
#include "modules/perception/lidar/lib/interface/base_pointcloud_cluster.h"
#include "modules/perception/lidar/lib/interface/base_roi_filter.h"
#include "modules/perception/lidar/lib/map_manager/map_manager.h"
#include "modules/perception/lidar/lib/object_builder/object_builder.h"
#include "modules/perception/lidar/lib/object_filter_bank/object_filter_bank.h"
#include "modules/perception/lidar/lib/pointcloud_cluster/pointcloud_cluster.h"
#include "modules/perception/onboard/component/lidar_inner_component_messages.h"
#include "modules/perception/onboard/inner_component_messages/inner_component_messages.h"
#include "modules/perception/onboard/msg_buffer/time_window_synchronizer.h"
#include "modules/perception/onboard/transform_wrapper/transform_wrapper.h"
#if defined(__x86_64__)
#include "modules/perception/camera/lib/nvjpeg_decoder/nvjpeg_decoder.h"
#elif defined(__aarch64__)
#include "modules/perception/camera/lib/nvjpeg_decoder/nvjpeg_opencv.h"
#endif

namespace century {
namespace perception {
namespace onboard {

using LidarFrameMessageReaderPtr =
    std::shared_ptr<cyber::Reader<LidarFrameMessage>>;
using LidarFrameMessagePtr = std::shared_ptr<LidarFrameMessage>;
using LocalizationReaderPtr =
    std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>>;
using LidarMsgsVecPtr = std::shared_ptr<std::vector<LidarFrameMessagePtr>>;
using CompressedImageMsgType =
    std::shared_ptr<century::drivers::CompressedImage>;
using RawImageMsgType = std::shared_ptr<century::drivers::Image>;
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

  bool LoadComponentConfig(
      const LidarDetectionComponentConfig& comp_config) noexcept;

  bool InitIoResources(
      const LidarDetectionComponentConfig& comp_config) noexcept;

  bool InitCameraResources() noexcept;

  void StartBackgroundTasks() noexcept;

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

  void UpdateFreespaceMotionState(
      const std::shared_ptr<century::localization::LocalizationEstimate>&
          localization_msg,
      LidarFrameMessagePtr merged_msg) noexcept;

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
  bool BuildFreespaceMaskPolygon(
      const LidarFrameMessagePtr& merged_msg,
      PerceptionObstacleDebugMsg* debug_msg) noexcept;

  void FillFreespaceIntoPerceptionObstacles(
      const PerceptionObstacleDebugMsg& debug_msg,
      const Eigen::Affine3d& lidar2world_pose,
      PerceptionObstacles* perception_obstacles) const noexcept;

  /**
   * @brief Remove high-trailer objects from the LiDAR frame.
   *
   * @param merged_msg
   */
  void RemoveHighTrailerObjects(
      const LidarFrameMessagePtr& merged_msg) noexcept;

  void OnReceiveCompressedImage(
      const std::shared_ptr<drivers::CompressedImage>& compressed_image,
      int index) noexcept;

  void OnReceiveRawImage(const std::shared_ptr<drivers::Image>& raw_image,
                         int index) noexcept;

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
  bool enable_freespace_mask_ = true;
  bool freespace_use_hdmap_road_ = false;
  double freespace_mask_resolution_ = 0.4;
  double freespace_mask_x_min_ = -50.0;
  double freespace_mask_x_max_ = 50.0;
  double freespace_mask_y_min_ = -50.0;
  double freespace_mask_y_max_ = 50.0;
  double freespace_obstacle_inflate_ = 0.0;
  int freespace_min_points_per_cell_ = 3;
  bool freespace_show_ray_source_ = false;
  bool use_camera_ = false;

  std::string image_format_ = "compressed";
  bool freespace_use_cuda_ = true;
  bool freespace_enable_temporal_filter_ = false;
  double freespace_temporal_expand_alpha_ = 0.35;
  double freespace_temporal_max_expand_ = 1.2;
  double freespace_temporal_source_shift_reset_ = 2.0;
  lidar::FreespaceTemporalFilterState freespace_temporal_state_;

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

  std::string post_detection_frame_output_channel_name_;

  /// Readers for receiving LiDAR frame messages.
  std::vector<LidarFrameMessageReaderPtr> lidar_readers_;

  std::shared_ptr<century::cyber::Writer<LidarFrameMessage>>
      post_detection_frame_writer_;

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

  std::vector<std::shared_ptr<cyber::Reader<drivers::CompressedImage>>>
      compressed_image_readers_;
  std::vector<std::shared_ptr<cyber::Reader<drivers::Image>>>
      raw_image_readers_;

  std::vector<std::unique_ptr<century::perception::camera::NvJpegDecoder>>
      jpeg_decoders_;
  std::unique_ptr<TimeWindowSynchronizer> time_window_synchronizer_ptr_;
  std::vector<std::string> camera_channels_;
  std::vector<std::vector<float>> dmapx_;
  std::vector<std::vector<float>> dmapy_;
  std::vector<cudaStream_t> camera_streams_;
  int image_width_ = 1920;
  int image_height_ = 1536;
};

CYBER_REGISTER_COMPONENT(LidarDetectorComponent);
}  // namespace onboard
}  // namespace perception
}  // namespace century
