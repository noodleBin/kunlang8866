/******************************************************************************
 * Copyright 2018 The Century Authors. All Rights Reserved.
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
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/measure.pb.h"
#include "modules/localization/proto/pose.pb.h"

namespace century {
namespace localization {
namespace msf {

/**
 * @class LocationExporter
 * @brief Export info about localziation in rosbag.
 */
class LocationExporter {
 public:
  explicit LocationExporter(const std::string &loc_file_folder);
  ~LocationExporter();

  void GnssLocCallback(const std::string &msg);
  void LidarLocCallback(const std::string &msg);
  void FusionLocCallback(const std::string &msg);
  void InsLocCallback(const std::string &msg);
  void OdometryLocCallback(const std::string &msg);
  bool WritePCD(const std::string &pcd_folder);

 private:
  std::string gnss_loc_file_;
  std::string lidar_loc_file_;
  std::string fusion_loc_file_;
  std::string ins_loc_file_;
  std::string odometry_loc_file_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr fusion_poses_ = nullptr;
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_poses_ = nullptr;
  pcl::PointCloud<pcl::PointXYZI>::Ptr ins_poses_ = nullptr;

  FILE *gnss_loc_file_handle_;
  FILE *lidar_loc_file_handle_;
  FILE *fusion_loc_file_handle_;
  FILE *ins_loc_file_handle_;
  FILE *odometry_loc_file_handle_;
};

}  // namespace msf
}  // namespace localization
}  // namespace century
