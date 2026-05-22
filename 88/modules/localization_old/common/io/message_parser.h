/******************************************************************************
 * Copyright 2024 The Century Authors. All Rights Reserved.
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

#include "third_party/mmath/coord_convert.h"
#include "third_party/mmath/se3.h"

#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/localization.pb.h"

#include "cyber/cyber.h"
#include "modules/localization/msf/common/io/pcl_point_types.h"

namespace century {
namespace common {
namespace io {

class MessageParser {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  MessageParser() {}
  ~MessageParser() {}

  bool CheckInsStatus(const std::shared_ptr<drivers::gnss::Insx>& msg,
                      double* first_ins_ok_time, double ins_pos_thresh,
                      double ins_atti_thresh);
  void ParseInsxMsg(const std::shared_ptr<drivers::gnss::Insx>& msg,
                    const Eigen::Vector3d& mercator_origin, double* timestamp,
                    mmath::SE3* Tx_Mp_ins, Eigen::Vector3d* linear_vel,
                    Eigen::Vector6d* omega_acc);

  void ParseLidarMsg(const drivers::PointXYZIRTCloud& lidar_msg,
                     const std::shared_ptr<loc::PointCloudXYZIRT>& pcl_cloud);

  void ParseLidarMsg(const drivers::PointXYZIRTCloud& lidar_msg,
                     const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pcl_cloud);

  void ParseFusionMsg(
      const std::shared_ptr<localization::LocalizationEstimate>& msg,
      double* timestamp, mmath::SE3* pose, Eigen::Vector3d* vel);
  void GetLidarPoseMsg(const double& timestamp, const std::string& frame_id,
                       const mmath::SE3& pose, const double fitness_score,
                       const bool is_converged,
                       localization::PoseWithCov* pose_msg);

  void ParseCorrImuMsg(const std::shared_ptr<localization::CorrectedImu>& msg,
                       double* timestamp, Eigen::Vector3d* linear_acc,
                       Eigen::Vector3d* ang_vel);

  void GetLocMsg(
      const double& timestamp, const mmath::SE3& pose,
      const Eigen::Vector6d& omega_vel, const Eigen::Vector3d& acc,
      const localization::LocalizationEstimate::StatusType loc_status,
      localization::LocalizationEstimate* loc_msg);

  double GetMessageStamp(
      const std::shared_ptr<drivers::PointXYZIRTCloud>& lidar_msg,
      const double& sync_thresh);

 private:
  int sequece_num_;
  std::string module_name_;
};

}  // namespace io
}  // namespace common
}  // namespace century
