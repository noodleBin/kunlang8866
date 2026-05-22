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

#include <deque>
#include <memory>
#include <optional>
#include <string>

#include <Eigen/Core>

#include "third_party/mmath/rigid_body_kinematic.h"
#include "third_party/mmath/se3.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/msf_config.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/localization/proto/msf_reset.pb.h"

#include "cyber/common/log.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/time/clock.h"
#include "cyber/time/rate.h"
#include "cyber/timer/timer.h"
#include "modules/localization/common/io/config_parser.h"
#include "modules/localization/common/io/message_parser.h"
#include "modules/localization/msf/common/filter/error_state_kalman_filter.h"
#include "modules/localization/msf/msf_task.h"

namespace {
constexpr unsigned int SELECT_INTERVAL_NUM = 125;
constexpr unsigned int WAITING_ESKF_INTERVAL_MS = 500;
constexpr double INS_TIMESTAMP_OFFSET = -0.01;
constexpr double NDT_FITNESS_SCORE_THRESH = 0.35;
constexpr double PRINT_DATA_INTERVAL = 1.0;
}  // namespace

namespace century {
namespace loc {

struct MsfLocConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // config
  msf::MsfConfig msf_cfg;
  std::string loc_config_path = "";
  std::string ins_calib_file = "";
  YAML::Node eskf_node;

  // topic
  std::string ins_loc_topic = "";
  std::string ins_pva_topic = "";
  std::string ins_corrimu_topic = "";
  std::string lidar_loc_topic = "";
  std::string msf_reset_topic = "";
  std::string chassis_topic = "";
  std::string msf_loc_topic = "";

  // params
  mmath::SE3 Tx_veh_ins;
  mmath::SE3 Tx_ins_veh;
  Eigen::Vector3d mercator_origin;
  int msf_loc_rate = 50;
  double sync_time_thresh = 0.2;
  double non_recv_time_thresh = 1.0;
  double max_history_second = 1.0;
  bool use_ins_pose = false;
  bool use_ins_velocity = false;
  bool use_chassis_velocity = false;
};

class MsfLocComponent final
    : public cyber::Component<localization::CorrectedImu> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MsfLocComponent() = default;
  ~MsfLocComponent() = default;

  bool Init() override;
  bool Proc(const std::shared_ptr<localization::CorrectedImu>& msg) override;

 private:
  bool InitConfig();
  bool InitParams();
  bool InitReaderWriter();
  void InsLocCallback(
      const std::shared_ptr<localization::LocalizationEstimate>& msg);
  void MsfResetCallback(
      const std::shared_ptr<localization::MsfReset>& msg);
  void InsPvaCallback(const std::shared_ptr<drivers::gnss::Insx>& msg);
  void LidarLocCallback(const std::shared_ptr<localization::PoseWithCov>& msg);
  void ChassisCallback(const std::shared_ptr<canbus::Chassis>& msg);
  void ProcessMsfLocStatus();
  void PublishLocalizationEstimate();
  bool FixHeading(const Eigen::Matrix4d& pose, mmath::SE3* pose_fixed);
  bool FixENUVel(const Eigen::Vector3d& velocity, Eigen::Vector3d* velocity_fixed);

  std::unique_ptr<LidarLocData> lidar_loc_data_ = nullptr;
  std::unique_ptr<InsLocData> ins_loc_data_ = nullptr;
  std::unique_ptr<ChassisData> chassis_data_ = nullptr;
  std::unique_ptr<TaskPool> task_pool_ = nullptr;

  // params
  MsfLocConfig config_;
  std::unique_ptr<ErrorStateKalmanFilter> eskf_ = nullptr;
  localization::LocalizationEstimate::StatusType ins_loc_status_ =
      localization::LocalizationEstimate::UNINITIALIZED;
  localization::LocalizationEstimate::StatusType lidar_loc_status_ =
      localization::LocalizationEstimate::UNINITIALIZED;
  localization::LocalizationEstimate::StatusType msf_loc_status_ =
      localization::LocalizationEstimate::UNINITIALIZED;

  std::unique_ptr<common::io::ConfigParser> config_parser_ = nullptr;
  std::unique_ptr<common::io::MessageParser> message_parser_ = nullptr;

  double last_ins_loc_time_ = cyber::Clock::NowInSeconds();
  double last_lidar_loc_time = cyber::Clock::NowInSeconds();

  // listener
  std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>>
      ins_loc_listener_ = nullptr;
  std::shared_ptr<cyber::Reader<localization::PoseWithCov>>
      lidar_loc_listener_ = nullptr;
  std::shared_ptr<cyber::Reader<localization::MsfReset>>
      msf_reset_listener_ = nullptr;
  std::shared_ptr<cyber::Reader<canbus::Chassis>> chassis_listener_ = nullptr;
  std::shared_ptr<cyber::Reader<drivers::gnss::Insx>> ins_pva_listener_ =
      nullptr;

  // talker
  std::shared_ptr<cyber::Writer<localization::LocalizationEstimate>>
      msf_loc_talker_ = nullptr;

  // timer
  std::shared_ptr<cyber::Timer> timer_ = nullptr;
  std::optional<double> gamma_;
  bool eskf_initialized_ = false;
  bool ins_loc_enabled_ = false;
  bool ins_pva_enabled_ = false;
  Eigen::Matrix4d odom_pose_ = Eigen::Matrix4d::Identity();
  Eigen::Vector3d odom_vel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acc_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_vel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d ang_vel_ = Eigen::Vector3d::Zero();

  std::optional<Eigen::Matrix4d> init_pose_;
  std::optional<Eigen::Vector3d> init_vel_;

  bool print_debug_data_ = true;
  std::ofstream data_compare_csv_file_;
  std::string data_compare_csv_path_ = "/century/data/log/a_data_compare_" +
                                   cyber::Clock::Now().ToStringSimple() + ".csv";
};

CYBER_REGISTER_COMPONENT(MsfLocComponent);

}  // namespace loc
}  // namespace century