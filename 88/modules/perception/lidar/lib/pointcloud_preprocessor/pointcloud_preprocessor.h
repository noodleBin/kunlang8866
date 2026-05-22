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

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <pcl/pcl_base.h>

#include "modules/perception/lidar/lib/pointcloud_preprocessor/proto/pointcloud_preprocessor_config.pb.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector/spatio_temporal_ground_detector.h"
#include "modules/perception/lidar/lib/interface/base_pointcloud_preprocessor.h"
#include "modules/perception/lidar/lib/roi_filter/ego_service_filter/ego_service_filter.h"

namespace century {
namespace perception {
namespace lidar {

using PointCloudTypePtr = base::PointXYZIRTFCloudPtr;
using IndicesPtr = pcl::IndicesPtr;

using PointCloudTypePtr = base::PointXYZIRTFCloudPtr;
using PointXYZIRTFCloud = base::PointXYZIRTFCloud;

class PointCloudPreprocessor : public BasePointCloudPreprocessor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PointCloudPreprocessor() : BasePointCloudPreprocessor() {}

  virtual ~PointCloudPreprocessor() = default;

  bool Init(const PointCloudPreprocessorInitOptions& options =
                PointCloudPreprocessorInitOptions()) override;

  bool Preprocess(
      const PointCloudPreprocessorOptions& options,
      const std::shared_ptr<century::drivers::PointCloud const>& message,
      LidarFrame* frame) const override {
    return true;
  }

  bool Preprocess(const PointCloudPreprocessorOptions& options,
                  LidarFrame* frame) const override {
    return true;
  }

  bool Preprocess(const PointCloudPreprocessorOptions& options,
                  const std::shared_ptr<PointCloudInType const>& message,
                  LidarFrame* frame) override;

  std::string Name() const override { return "PointCloudPreprocessor"; }

 private:
  bool TransformCloud(const base::PointFCloudPtr& local_cloud,
                      const Eigen::Affine3d& pose,
                      base::PointDCloudPtr world_cloud) const;
  /**
   * @brief Initialize point clouds in the LidarFrame.
   * @param frame LidarFrame to initialize point clouds for.
   * @param message Input point cloud message.
   * @note This function sets up the point cloud pointers in the LidarFrame.
   */
  void InitializeClouds(
      LidarFrame* frame,
      const std::shared_ptr<PointCloudInType const>& message) noexcept;

  /**
   * @brief Reserve memory for the point cloud in the LidarFrame.
   * @param frame LidarFrame to reserve memory for.
   * @param point_size Number of points to reserve memory for.
   * @note This function should be called after InitializeClouds.
   *
   */
  void ReserveCloudMemory(LidarFrame* frame, size_t point_size) noexcept;

  /**
   * @brief Filter points that are within the ego vehicle's bounding box.
   * @param point Input point to evaluate.
   * @return true if the point is within the ego vehicle's bounding box, false
   * otherwise.
   * @note This function uses the ego_filter_ to determine if the point is
   *       within the ego vehicle's bounding box.
   */
  inline bool FilterEgoPoints(const base::PointXYZIRTF& point) noexcept;

  /**
   * @brief Filter points based on intensity.
   * @param point Input point to evaluate.
   * @return true if the point's intensity is below the threshold, false
   * otherwise.
   * @note This function uses the intensity thresholds defined in the class
   *       to filter points.
   */
  inline bool FilterByIntensity(const base::PointXYZIRTF& point) noexcept;

  /**
   * @brief Filter points based on region of interest (ROI).
   * @param point Input point to evaluate.
   * @return true if the point is outside the ROI, false otherwise.
   * @note This function uses the ego_filter_ to determine if the point is
   *       within the ROI.
   */
  inline bool FilterByRoi(const base::PointXYZIRTF& point) noexcept;

  /**
   * @brief Filter points based on range criteria.
   * @param point Input point to evaluate.
   * @return true if the point is outside the defined range, false otherwise.
   * @note This function uses the range criteria defined in the class to filter
   *       points.
   */
  inline bool FilterByRange(const base::PointXYZIRTF& point) noexcept;

  /**
   * @brief Convert points from the input message to the LidarFrame.
   * @param message Input point cloud message.
   * @param point_size Number of points in the input message.
   * @param frame LidarFrame to store the converted points.
   * @note This function assumes that the point cloud in the LidarFrame
   *       has already been initialized and has enough reserved memory.
   */
  void ProcessPoints(const std::shared_ptr<PointCloudInType const>& message,
                     size_t point_size, LidarFrame* frame) noexcept;

  /**
   * @brief Determine if a point should be filtered out based on intensity and
   * position.
   * @param pt Input point to evaluate.
   * @return true if the point should be filtered out, false otherwise.
   * @note This function uses intensity thresholds and spatial criteria to
   *       decide whether to filter the point.
   */
  bool ShouldFilterPoint(const base::PointXYZIRTF& pt) noexcept;

  /**
   * @brief Transform a point from the sensor frame to the IMU frame.
   * @param pt Input point in the sensor frame.
   * @return Transformed point in the IMU frame.
   * @note This function uses the sensor to IMU extrinsics defined in the
   *       PointCloudPreprocessorOptions.
   */
  inline void TransformPoint(base::PointXYZIRTF* pt) noexcept;

  /**
   * @brief Add a transformed point to the LidarFrame.
   * @param frame LidarFrame to add the point to.
   * @param point Transformed point to add.
   * @param timestamp Timestamp of the point.
   * @param index Index of the point in the original point cloud.
   * @note This function adds the point to both the vehicle and raw clouds
   *       in the LidarFrame.
   */
  void AddPointToCloud(LidarFrame* frame, const base::PointXYZIRTF& point,
                       double timestamp, size_t index) noexcept;

  /**
   * @brief Finalize the point clouds in the LidarFrame after processing.
   * @param frame LidarFrame to finalize the point clouds for.
   * @note This function resizes the clouds to their actual size,
   *       copies the raw cloud to the final cloud, and transforms
   *       the cloud to the world frame.
   */
  void FinalizeClouds(LidarFrame* frame) noexcept;

  /**
   * @brief Filter the point cloud in the LidarFrame based on non-ground
   * indices.
   * @param frame LidarFrame containing the point cloud to filter.
   * @note This function updates the cloud in the LidarFrame to only include
   *       points that are not classified as ground points.
   */
  void FilterPointCloud(LidarFrame* frame) noexcept;

  void SavePointcloudPcd(
      const std::shared_ptr<century::drivers::PointXYZIRTCloud const>&
          message) noexcept;

  struct CloudCandidate {
    base::PointF point;
    double timestamp = 0.0;
    int origin_index = 0;
    size_t cloud_index = 0;
  };

  void FilterOutlierCandidates(const std::vector<CloudCandidate>& candidates,
                               std::vector<uint8_t>* keep) const noexcept;

 private:
  bool use_det_ = true;
  float max_intensity_threshold_ = 80.f;
  float original_intensity_threshold_ = 3.0f;
  float dust_intensity_threshold_ = 6.0f;
  float range_around_forward_x_ = 20.0f;
  float range_around_forward_y_ = 10.0f;
  float range_around_backward_x_ = -20.0f;
  float range_around_backward_y_ = -10.0f;
  bool is_intensity_filter_ = true;
  bool dump_pcd_ = false;
  float height_threshold_ = 0.25f;
  bool filter_naninf_points_ = true;
  bool filter_nearby_box_points_ = true;
  float box_forward_x_ = 0.0f;
  float box_backward_x_ = 0.0f;
  float box_forward_y_ = 0.0f;
  float box_backward_y_ = 0.0f;
  bool filter_high_z_points_ = true;
  float z_threshold_ = 5.0f;
  bool filter_outlier_points_ = false;
  float outlier_search_radius_ = 0.5f;
  int outlier_min_neighbors_ = 2;

  std::vector<int> origin_points_indices_;
  std::string topic_name_;

  PointCloudPreprocessorConfig config_;
  PointCloudPreprocessorOptions options_;

  BaseGroundDetector* ground_detector_{nullptr};

  BaseROIFilter* ego_filter_;
};  // class PointCloudPreprocessor

}  // namespace lidar
}  // namespace perception
}  // namespace century
