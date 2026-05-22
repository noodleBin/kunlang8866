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

#pragma once

#include <memory>
#include <string>

#include <pcl/pcl_base.h>

#include "modules/perception/lidar/lib/pointcloud_segmentation/proto/pointcloud_segmentation_config.pb.h"

#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/lib/interface/base_pointcloud_preprocessor.h"

namespace century {
namespace perception {
namespace lidar {

using PointCloudTypePtr = base::PointXYZIRTFCloudPtr;
using IndicesPtr = pcl::IndicesPtr;

class PointcloudSegmentation : public BasePointCloudPreprocessor {
 public:
  PointcloudSegmentation() : BasePointCloudPreprocessor() {}
  ~PointcloudSegmentation() = default;

  /**
   * @brief Initialize the point cloud segmentation module.
   *
   * @param options Initialization options for the preprocessor.
   * @return true if initialization is successful, false otherwise.
   */
  bool Init(const PointCloudPreprocessorInitOptions& options =
                PointCloudPreprocessorInitOptions()) override;

  /**
   * @brief Preprocess the input point cloud message and update the LidarFrame.
   *
   * @param options Preprocessing options.
   * @param message Input point cloud message (PointXYZIRTCloud format).
   * @param frame Output LidarFrame to store processed data.
   * @return true if preprocessing is successful, false otherwise.
   */
  bool Preprocess(
      const PointCloudPreprocessorOptions& options,
      const std::shared_ptr<century::drivers::PointXYZIRTCloud const>& message,
      LidarFrame* frame) override;

  /**
   * @brief Preprocess the input point cloud message and update the LidarFrame.
   *
   * @param options Preprocessing options.
   * @param message Input point cloud message (generic PointCloud format).
   * @param frame Output LidarFrame to store processed data.
   * @return true if preprocessing is successful, false otherwise.
   */
  bool Preprocess(
      const PointCloudPreprocessorOptions& options,
      const std::shared_ptr<century::drivers::PointCloud const>& message,
      LidarFrame* frame) const {
    return true;
  }

  /**
   * @brief Preprocess the LidarFrame without an input point cloud message.
   *
   * @param options Preprocessing options.
   * @param frame Output LidarFrame to store processed data.
   * @return true if preprocessing is successful, false otherwise.
   */
  bool Preprocess(const PointCloudPreprocessorOptions& options,
                  LidarFrame* frame) const override {
    return true;
  }

  /**
   * @brief Get the name of the point cloud segmentation module.
   *
   * @return The name of the module as a string.
   */
  std::string Name() const override { return "PointcloudSegmentation"; }

 private:
  /**
   * @brief Transform the input point cloud from local coordinates to vehicle
   * coordinates.
   *
   * @param local_cloud Input point cloud in local coordinates.
   * @param pose Transformation matrix (Affine3d) from local to vehicle
   * coordinates.
   * @param vehicle Output point cloud in vehicle coordinates.
   * @return true if transformation is successful, false otherwise.
   */
  bool TransformCloud(
      const std::shared_ptr<century::drivers::PointXYZIRTCloud const>&
          local_cloud,
      LidarFrame* frame);

  /**
   * @brief Apply filtering to the input point cloud.
   *
   * @param incloud Input point cloud.
   * @param outcloud Output filtered point cloud.
   */
  void FilterPointCloud(const PointCloudTypePtr& incloud,
                        PointCloudTypePtr& outcloud) noexcept;

  /**
   * @brief Find ground point candidates in the input point cloud.
   *
   * @param inCloud Input point cloud.
   * @param outIndices Output indices of ground point candidates.
   * @param cell_size Size of the grid cell for ground candidate detection.
   */
  void FindGroundCandidates(const PointCloudTypePtr& inCloud,
                            IndicesPtr& outIndices, double cell_size) noexcept;

  /**
   * @brief Filter out points that are too close to the sensor.
   *
   * @param indices Input indices of points.
   * @param size Threshold size for filtering.
   * @param outIndices Output indices after filtering.
   */
  void FilterClosePoints(const std::vector<std::vector<int>>& indices,
                         const int size, IndicesPtr& outIndices) noexcept;

  /**
   * @brief Filter the input point cloud and build additional data structures.
   *
   * @param inCloud Input point cloud.
   * @param outCloud Output filtered point cloud.
   * @param projectionDis Output projection distances.
   * @param indicesImg Output indices for image representation.
   */
  void FilterAndBuild(const PointCloudTypePtr& inCloud,
                      PointCloudTypePtr& outCloud,
                      std::vector<float>& projectionDis,
                      std::vector<int>& indicesImg) noexcept;

  /**
   * @brief Remove ground points from the input point cloud.
   *
   * @param inCloud Input point cloud.
   * @param outCloud Output point cloud with ground points removed.
   * @return true if ground points are successfully removed, false otherwise.
   */
  bool RemoveGroundPoints(const PointCloudTypePtr& inCloud,
                          PointCloudTypePtr& outCloud) noexcept;

  /**
   * @brief Get indices of points in the input point cloud.
   *
   * @param inCloud Input point cloud.
   * @param indices Output indices of points.
   * @return true if indices are successfully retrieved, false otherwise.
   */
  bool GetPointCloudIndices(PointCloudTypePtr& inCloud,
                            std::vector<int>& indices) noexcept;

  /**
   * @brief Extract obstacle points from the input point cloud.
   *
   * @param tr_x Translation in the x-axis.
   * @param tr_y Translation in the y-axis.
   * @param inCloud Input point cloud.
   * @param indices Indices of obstacle points.
   * @param outCloud Output point cloud containing only obstacle points.
   */
  void GetObstaclePointCloud(double tr_x, double tr_y,
                             const PointCloudTypePtr& inCloud,
                             const std::vector<int>& indices,
                             PointCloudTypePtr outCloud) noexcept;

  /**
   * @brief Filter noise from the point cloud.
   *
   * @return true if noise filtering is successful, false otherwise.
   */
  bool FilterNoise() noexcept;

 private:
  std::vector<float> projection_dis_;
  std::vector<int> indices_image_;
  std::vector<int> obj_img_indices_;
  std::vector<int> obj_indices_;

  //   Eigen::Matrix4f lidar2imu_matrix_;
  PointCloudTypePtr out_cloud_;
  PointCloudSegmentationConfig config_;
};
}  // namespace lidar
}  // namespace perception
}  // namespace century