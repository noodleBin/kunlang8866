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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pcl/common/io.h"
#include "pcl/io/pcd_io.h"
#include "third_party/mmath/linear_interpolater.h"
#include "third_party/mmath/rigid_body_kinematic.h"
#include "third_party/mmath/se3.h"
#include "third_party/mmath/thread_pool.h"
#include "third_party/mmath/voxel_filter.h"

#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/lidar_config.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/pose.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/time/clock.h"
#include "cyber/transport/shm/shm_queue.h"
#include "modules/drivers/lidar/robosense/rs_driver/src/rs_driver/msg/point_cloud_msg.hpp"
#include "modules/localization/common/io/config_parser.h"
#include "modules/localization/common/io/message_parser.h"
#include "modules/localization/common/loc_time.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/localization/msf/common/io/pcl_point_types.h"
#include "modules/localization/msf/common/lidar/lidar_feature_multi_thread.h"
#include "modules/localization/ndt/ndt_locator/lidar_locator_ndt.h"

#define SAVE_SHOW_DATA_LIDAR 0

namespace {
constexpr int LIDAR_FRAME_INTERVAL_NUM = 2;
constexpr int LIDAR_FRAME_INTERVAL_MS = 100;
constexpr int MIN_SIZE_WIDTH_HEIGHT = 1;
constexpr int PCD_SIZE_MIN = 3000;
// constexpr int POINT_TIMESTAMP_LOW = 40000;
// constexpr int POINT_TIMESTAMP_HIGH = 65000;
constexpr double MAX_HEIGHT_LIDAR = 100.0;
constexpr int LIDAR_QUEUE_SIZE = 2;
constexpr int LIDAR_BUFFER_SIZE = 2;
constexpr double UTM_offset_x = 250932.851957;
constexpr double UTM_offset_y = 3987498.593868;
constexpr double UTM_offset_z = 0.0;

}  // namespace

namespace century {
namespace loc {
#if SAVE_SHOW_DATA_LIDAR
struct LidarPoseData {
  double timestamp;
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};
#endif

struct LidarLocComponentConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // config
  LidarConfig lidar_cfg;
  std::string loc_config_path = "";
  std::string ndt_map_path = "";

  // topic
  std::string left_lidar_topic = "";
  std::string right_lidar_topic = "";
  std::string left_lidar_trans_topic = "";
  std::string right_lidar_trans_topic = "";
  std::string lidar_loc_topic = "";
  std::string msf_loc_topic = "";

  // params
  std::string lidar_calib_file = "";
  mmath::SE3 Tx_veh_lidar1;
  mmath::SE3 Tx_imu_lidar1;
  mmath::SE3 Tx_veh_lidar2;
  mmath::SE3 Tx_imu_lidar2;

  TimeUs interpolater_max_history_us = 60 * kSecond;
  TimeUs interpolater_max_predict_us = 1 * kSecond;

  double sync_time_thresh;
  double lidar_height;
  unsigned int resolution_id = 0;
  int zone_id;

  // debug
  bool is_sim = false;
  bool show_pcd = false;
};

class LidarLocComponent final : public cyber::Component<drivers::gnss::Imu> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef mmath::LinearInterpolater<mmath::SE3> SE3LinearInterpolater;
  typedef mmath::LinearInterpolater<Eigen::Vector3d> Vector3dLinearInterpolater;
  typedef mmath::LinearInterpolater<Eigen::Vector6d> Vector6dLinearInterpolater;
  typedef localization::ndt::LidarFrame LidarFrame;
  LidarLocComponent();
  ~LidarLocComponent();

  bool Init() override;
  bool Proc(const std::shared_ptr<drivers::gnss::Imu>& message) override;

 private:
  bool InitConfig();
  bool InitParams();
  bool InitReaderWriter();

  void FusionPoseCallback(
      const std::shared_ptr<localization::LocalizationEstimate>& msg);
  void LeftLidarFeatureExtract(
      const double timestamp,
      const std::shared_ptr<loc::PointCloudXYZIRT>& pcd);
  void RightLidarFeatureExtract(
      const double timestamp,
      const std::shared_ptr<loc::PointCloudXYZIRT>& pcd);
  void LeftLidarCallback(
      const std::shared_ptr<drivers::PointCloudPacked>& cloud_msg);
  void RightLidarCallback(
      const std::shared_ptr<drivers::PointCloudPacked>& cloud_msg);
  void ShmLeftLidarCallback(const std::shared_ptr<loc::CloudFrame>& cloud_msg);
  void ShmRightLidarCallback(const std::shared_ptr<loc::CloudFrame>& cloud_msg);
  void LidarMatcherTimerCallback();
  void LidarFrameTransfer(const std::shared_ptr<loc::PointCloudXYZIRT>& msg1,
                          std::shared_ptr<loc::PointCloudXYZI>& msg2);
  void LidarFrameTransfer(const std::shared_ptr<loc::PointCloudXYZI>& msg,
                          LidarFrame* lidar_frame);
  bool ReadShmLeftLidarData();
  bool ReadShmRightLidarData();
  bool CheckInputPcd(const std::shared_ptr<loc::PointCloudXYZIRT>& pcd);
  bool GetInitGuessPose(const TimeUs time, mmath::SE3* guess_pose);
  bool LoadZoneIdFromFolder(const std::string& folder_path, int* zone_id);
  bool IsInMapRange(const Eigen::Vector3d& pose);
  void PublishLidarPose(const double timestamp, const mmath::SE3& Tx_Mp_L,
                        const double fitness_score, const bool is_converged);

  // params
  common::io::NdtMapConfig ndt_map_config_;
  LidarLocComponentConfig config_;
  std::unique_ptr<common::io::ConfigParser> config_parser_ = nullptr;
  std::unique_ptr<common::io::MessageParser> message_parser_ = nullptr;
  std::unique_ptr<LidarFeatureMultiThread> left_feature_extractor_ = nullptr;
  std::unique_ptr<LidarFeatureMultiThread> right_feature_extractor_ = nullptr;

  std::unique_ptr<Vector6dLinearInterpolater> raw_imu_interpolater_ = nullptr;
  std::unique_ptr<SE3LinearInterpolater> Tx_Mp_L1_interpolater_ = nullptr;
  std::unique_ptr<SE3LinearInterpolater> Tx_Mp_L2_interpolater_ = nullptr;
  std::unique_ptr<SE3LinearInterpolater> Tx_Mp_imu_interpolater_ = nullptr;
  std::unique_ptr<Vector6dLinearInterpolater> Vel6_Mp_L1_interpolater_ =
      nullptr;
  std::unique_ptr<Vector6dLinearInterpolater> Vel6_Mp_L2_interpolater_ =
      nullptr;
  std::unique_ptr<Vector3dLinearInterpolater> angular_vel_Mp_imu_interpolater_ =
      nullptr;
  std::unique_ptr<Vector3dLinearInterpolater> linear_vel_Mp_imu_interpolater_ =
      nullptr;
  std::unique_ptr<Vector3dLinearInterpolater> vel3_V_Mp_V_interpolater_ =
      nullptr;
  std::deque<std::pair<double, std::shared_ptr<loc::PointCloudXYZI>>>
      LeftLidar_line_buffer_;
  std::deque<std::pair<double, std::shared_ptr<loc::PointCloudXYZI>>>
      LeftLidar_plane_buffer_;
  std::deque<std::pair<double, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>>
      LeftLidar_common_buffer_;
  std::deque<std::pair<double, std::shared_ptr<loc::PointCloudXYZI>>>
      RightLidar_line_buffer_;
  std::deque<std::pair<double, std::shared_ptr<loc::PointCloudXYZI>>>
      RightLidar_plane_buffer_;
  std::deque<std::pair<double, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>>
      RightLidar_common_buffer_;

#if SAVE_SHOW_DATA_LIDAR
  std::deque<std::pair<double, std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>>>
      LeftLidar_common_buffer_show_;
  std::deque<std::pair<double, std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>>>
      RightLidar_common_buffer_show_;
#endif

  bool get_fuison_ = false;
  bool working_ = false;
  mmath::SE3 Tx_Mp_L1_;
  mmath::SE3 Tx_Mp_L2_;
  mmath::SE3 Tx_pre_cur_;

  std::unique_ptr<localization::ndt::LidarLocatorNdt> lidar_locator_ = nullptr;
  std::unique_ptr<mmath::ThreadPool> left_lidar_loc_thread_ = nullptr;
  std::unique_ptr<mmath::ThreadPool> right_lidar_loc_thread_ = nullptr;
  std::unique_ptr<cyber::Timer> lidar_matcher_timer_ = nullptr;
  // listener
  std::mutex cloud_list_mutex_;
  std::mutex interpolater_mutex_;
  std::mutex left_feature_extract_mutex_;
  std::mutex right_feature_extract_mutex_;
  std::mutex left_lidar_buffer_mutex;
  std::mutex right_lidar_buffer_mutex;
  std::shared_ptr<cyber::Reader<drivers::PointCloudPacked>>
      left_lidar_listener_ = nullptr;
  std::shared_ptr<cyber::Reader<drivers::PointCloudPacked>>
      right_lidar_listener_ = nullptr;

  std::unique_ptr<cyber::transport::shm_queue<PointCloudShm>> shm_cloud_left_ =
      nullptr;
  std::unique_ptr<cyber::transport::shm_queue<PointCloudShm>> shm_cloud_right_ =
      nullptr;

  std::shared_ptr<cyber::Reader<loc::CloudFrame>> shm_left_lidar_listener_ =
      nullptr;
  std::shared_ptr<cyber::Reader<loc::CloudFrame>> shm_right_lidar_listener_ =
      nullptr;
  std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>>
      fusion_pose_listener_ = nullptr;

  // talker
  std::shared_ptr<cyber::Writer<loc::CloudFrame>> left_lidar_trans_talker_ =
      nullptr;
  std::shared_ptr<cyber::Writer<loc::CloudFrame>> right_lidar_trans_talker_ =
      nullptr;
  std::shared_ptr<cyber::Writer<localization::PoseWithCov>> lidar_pose_talker_ =
      nullptr;

  // timer
  century::RuntimeCounter timer_;

#if SAVE_SHOW_DATA_LIDAR
  std::unique_ptr<std::ofstream> lidar_pose_file_;
#endif
};

CYBER_REGISTER_COMPONENT(LidarLocComponent);

}  // namespace loc
}  // namespace century
