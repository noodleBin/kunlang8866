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

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "cyber/common/log.h"
#include "modules/perception/base/object.h"
#include "modules/perception/lidar/common/lidar_frame.h"
// #include "modules/perception/lidar/common/lidar_frame_message.h"
#include "modules/perception/lidar/lib/map_manager/map_manager.h"
#include "modules/perception/onboard/component/lidar_inner_component_messages.h"

namespace century {
namespace perception {
namespace common {

// Visualization function for Lidar Detector

using onboard::LidarFrameMessage;
using LidarFrameMessagePtr = std::shared_ptr<LidarFrameMessage>;

class LidarVisualizer {
 public:
  LidarVisualizer(bool use_map_manager = false);
  ~LidarVisualizer() = default;

  /**
   * @brief Visualize a lidar frame
   * @param in_message The lidar frame message to visualize
   * @return true if visualization was successful, false otherwise
   */
  bool Visualize(const std::shared_ptr<LidarFrameMessage>& in_message) noexcept;

  /**
   * @brief Spin the visualizer once to update display
   */
  void SpinOnce();

  /**
   * @brief Save current visualization to a file
   * @param filename The path to save the image
   * @return true if save was successful, false otherwise
   */
  bool SaveVisualization(const std::string& filename);

  /**
   * @brief Set camera position
   * @param pos_x X position of camera
   * @param pos_y Y position of camera
   * @param pos_z Z position of camera
   * @param view_x X view direction
   * @param view_y Y view direction
   * @param view_z Z view direction
   */
  void SetCameraPosition(float pos_x, float pos_y, float pos_z, float view_x,
                         float view_y, float view_z);

  /**
   * @brief Set the background color of the visualizer
   * @param r Red component (0-255)
   * @param g Green component (0-255)
   * @param b Blue component (0-255)
   */
  void SetBackgroundColor(int r, int g, int b);

 private:
  // Initialize visualizer
  bool Initialize();

  // Display point cloud
  void DisplayPointCloud(const base::PointFCloudPtr& cloud,
                         const Eigen::Vector3d& location,
                         const Eigen::Matrix3d& rotation,
                         const std::string& name, uint8_t r, uint8_t g,
                         uint8_t b);

  // Display segmented objects
  void DisplaySegmentedObjects(const std::vector<base::ObjectPtr>& objects,
                               const Eigen::Vector3d& location,
                               const Eigen::Matrix3d& rotation);

  // Display road polygons
  void DisplayRoadPolygons(
      const std::shared_ptr<LidarFrameMessage>& in_message);

  void SavePointCloudToPCD(
      const base::PointFCloudPtr& cloud,
      const std::string& folder_path = "/century/data/pcd_output/");

  void SavePointCloudToBIN(
      const base::PointFCloudPtr& cloud,
      const std::string& folder_path = "/century/data/bin_output/");

  void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);
  
  void drawEgo(const Eigen::Matrix3d& rotation);
  void drawPose(const Eigen::Matrix3d& rotation);

  void help();

  // PCL visualizer instance
  pcl::visualization::PCLVisualizer::Ptr viewer_;

  // Flags
  bool use_map_manager_;
  bool is_initialized_;

  // Store IDs of visualization elements for later removal
  std::vector<std::string> cube_ids_;

  std::unordered_map<base::ObjectSubType, std::string> subtype_to_name_;

  std::atomic<uint8_t> show_control_flag_{30};
};

}  // namespace common
}  // namespace perception
}  // namespace century