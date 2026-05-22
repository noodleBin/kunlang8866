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

#include "modules/localization/msf/local_tool/data_extraction/location_exporter.h"



#include "cyber/common/log.h"

namespace century {
namespace localization {
namespace msf {

LocationExporter::LocationExporter(const std::string &loc_file_folder) {
  gnss_loc_file_ = loc_file_folder + "/gnss_loc.txt";
  lidar_loc_file_ = loc_file_folder + "/lidar_loc.txt";
  fusion_loc_file_ = loc_file_folder + "/fusion_loc.txt";
  ins_loc_file_ = loc_file_folder + "/ins_loc.txt";
  odometry_loc_file_ = loc_file_folder + "/odometry_loc.txt";

  if ((gnss_loc_file_handle_ = fopen(gnss_loc_file_.c_str(), "a")) == nullptr) {
    AERROR << "Cannot open gnss localization file!";
  }

  if ((lidar_loc_file_handle_ = fopen(lidar_loc_file_.c_str(), "a")) ==
      nullptr) {
    AERROR << "Cannot open lidar localization file!";
  }

  if ((fusion_loc_file_handle_ = fopen(fusion_loc_file_.c_str(), "a")) ==
      nullptr) {
    AERROR << "Cannot open fusion localization file!";
  }

  if ((ins_loc_file_handle_ = fopen(ins_loc_file_.c_str(), "a")) == nullptr) {
    AERROR << "Cannot open ins localization file!";
  }

  if ((odometry_loc_file_handle_ = fopen(odometry_loc_file_.c_str(), "a")) ==
      nullptr) {
    AERROR << "Cannot open odometry localization file!";
  }

  fusion_poses_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  lidar_poses_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  ins_poses_.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

LocationExporter::~LocationExporter() {
  if (gnss_loc_file_handle_ != nullptr) {
    fclose(gnss_loc_file_handle_);
  }

  if (lidar_loc_file_handle_ != nullptr) {
    fclose(lidar_loc_file_handle_);
  }

  if (fusion_loc_file_handle_ != nullptr) {
    fclose(fusion_loc_file_handle_);
  }

  if (ins_loc_file_handle_ != nullptr) {
    fclose(ins_loc_file_handle_);
  }

  if (odometry_loc_file_handle_ != nullptr) {
    fclose(odometry_loc_file_handle_);
  }
}

void LocationExporter::GnssLocCallback(const std::string &msg_string) {
  AINFO << "GNSS location callback.";
  LocalizationEstimate msg;
  msg.ParseFromString(msg_string);

  static unsigned int index = 1;

  double timestamp = msg.measurement_time();
  double x = msg.pose().position().x();
  double y = msg.pose().position().y();
  double z = msg.pose().position().z();

  double qx = msg.pose().orientation().qx();
  double qy = msg.pose().orientation().qy();
  double qz = msg.pose().orientation().qz();
  double qw = msg.pose().orientation().qw();

  double std_x = msg.uncertainty().position_std_dev().x();
  double std_y = msg.uncertainty().position_std_dev().y();
  double std_z = msg.uncertainty().position_std_dev().z();

  fprintf(gnss_loc_file_handle_,
          "%u %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", index, timestamp,
          x, y, z, qx, qy, qz, qw, std_x, std_y, std_z);

  ++index;
}

void LocationExporter::LidarLocCallback(const std::string &msg_string) {
  AINFO << "Lidar location callback.";
  localization::PoseWithCov msg;
  msg.ParseFromString(msg_string);
  static unsigned int index = 1;

  double timestamp = msg.header().timestamp_sec();
  double x = msg.position().x();
  double y = msg.position().y();
  double z = msg.position().z();

  double qx = msg.orientation().qx();
  double qy = msg.orientation().qy();
  double qz = msg.orientation().qz();
  double qw = msg.orientation().qw();

  // century::common::math::EulerAnglesZXY<double> euler(
  //     qw, qx, qy, qz);
  // double roll = euler.roll();
  // double pitch = euler.pitch();
  // double yaw = euler.yaw();

  // double std_x = msg.uncertainty().position_std_dev().x();
  // double std_y = msg.uncertainty().position_std_dev().y();
  // double std_z = msg.uncertainty().position_std_dev().z();

  fprintf(lidar_loc_file_handle_,
          "%u %lf %lf %lf %lf %lf %lf %lf %lf\n", index, timestamp,
          x, y, z, qx, qy, qz, qw);

  ++index;
  pcl::PointXYZI this_pose;
  this_pose.x = x;
  this_pose.y = y;
  this_pose.z = z;
  this_pose.intensity = lidar_poses_->size();
  lidar_poses_->push_back(this_pose);
}

void LocationExporter::FusionLocCallback(const std::string &msg_string) {
  AINFO << "Fusion location callback.";
  LocalizationEstimate msg;
  msg.ParseFromString(msg_string);
  static unsigned int index = 1;

  double timestamp = msg.header().timestamp_sec();
  double x = msg.pose().position().x();
  double y = msg.pose().position().y();
  double z = msg.pose().position().z();

  double qx = msg.pose().orientation().qx();
  double qy = msg.pose().orientation().qy();
  double qz = msg.pose().orientation().qz();
  double qw = msg.pose().orientation().qw();

  // century::common::math::EulerAnglesZXY<double> euler(
  //     qw, qx, qy, qz);
  // double roll = euler.roll();
  // double pitch = euler.pitch();
  // double yaw = euler.yaw();

  double std_x = msg.uncertainty().position_std_dev().x();
  double std_y = msg.uncertainty().position_std_dev().y();
  double std_z = msg.uncertainty().position_std_dev().z();

  fprintf(fusion_loc_file_handle_,
          "%u %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", index, timestamp,
          x, y, z, qx, qy, qz, qw, std_x, std_y, std_z);

  ++index;
  pcl::PointXYZI this_pose;
  this_pose.x = x;
  this_pose.y = y;
  this_pose.z = z;
  this_pose.intensity = fusion_poses_->size();
  fusion_poses_->push_back(this_pose);
}

void LocationExporter::InsLocCallback(const std::string &msg_string) {
  AINFO << "Ins location callback.";
  LocalizationEstimate msg;
  msg.ParseFromString(msg_string);
  static unsigned int index = 1;

  double timestamp = msg.header().timestamp_sec();

  double x = msg.pose().position().x();
  double y = msg.pose().position().y();
  double z = msg.pose().position().z();

  double qx = msg.pose().orientation().qx();
  double qy = msg.pose().orientation().qy();
  double qz = msg.pose().orientation().qz();
  double qw = msg.pose().orientation().qw();

  // century::common::math::EulerAnglesZXY<double> euler(
  //     qw, qx, qy, qz);
  // double roll = euler.roll();
  // double pitch = euler.pitch();
  // double yaw = euler.yaw();

  double std_x = msg.uncertainty().position_std_dev().x();
  double std_y = msg.uncertainty().position_std_dev().y();
  double std_z = msg.uncertainty().position_std_dev().z();

  fprintf(ins_loc_file_handle_,
          "%u %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", index, timestamp,
          x, y, z, qx, qy, qz, qw, std_x, std_y, std_z);

  ++index;

  pcl::PointXYZI this_pose;
  this_pose.x = x;
  this_pose.y = y;
  this_pose.z = z;
  this_pose.intensity = ins_poses_->size();
  ins_poses_->push_back(this_pose);
}

void LocationExporter::OdometryLocCallback(const std::string &msg_string) {
  AINFO << "Odometry location callback.";
  Gps msg;
  msg.ParseFromString(msg_string);
  static unsigned int index = 1;

  double timestamp = msg.header().timestamp_sec();
  double x = msg.localization().position().x();
  double y = msg.localization().position().y();
  double z = msg.localization().position().z();

  double qx = msg.localization().orientation().qx();
  double qy = msg.localization().orientation().qy();
  double qz = msg.localization().orientation().qz();
  double qw = msg.localization().orientation().qw();

  // century::common::math::EulerAnglesZXY<double> euler(
  //     qw, qx, qy, qz);
  // double roll = euler.roll();
  // double pitch = euler.pitch();
  // double yaw = euler.yaw();

  double std_x = 0;
  double std_y = 0;
  double std_z = 0;

  fprintf(odometry_loc_file_handle_,
          "%u %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", index, timestamp,
          x, y, z, qx, qy, qz, qw, std_x, std_y, std_z);

  ++index;
}

bool LocationExporter::WritePCD(const std::string &pcd_folder) {
  if (!fusion_poses_->empty()) {
    AINFO << "Write fusion_trajectory.pcd";
    pcl::io::savePCDFileBinary(pcd_folder + "/fusion_trajectory.pcd",
                               *fusion_poses_);
  }
  if (!lidar_poses_->empty()) {
    AINFO << "Write lidar_trajectory.pcd";
    pcl::io::savePCDFileBinary(pcd_folder + "/lidar_trajectory.pcd",
                               *lidar_poses_);
  }

  if (!ins_poses_->empty()) {
    AINFO << "Write ins_trajectory.pcd";
    pcl::io::savePCDFileBinary(pcd_folder + "/ins_trajectory.pcd", *ins_poses_);
  }
  return true;
}

}  // namespace msf
}  // namespace localization
}  // namespace century
