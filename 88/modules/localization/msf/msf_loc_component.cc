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

#include "modules/localization/msf/msf_loc_component.h"

#include "msf_loc_component.h"

namespace century {
namespace loc {
MsfLocComponent::MsfLocComponent() {}
MsfLocComponent::~MsfLocComponent() {
#if SAVE_SHOW_DATA_FUSION
  if (fusion_pose_file_ && fusion_pose_file_->is_open()) {
      fusion_pose_file_->close();
  }
#endif
}

bool MsfLocComponent::Init() {
  // init config
  if (!InitConfig()) {
    AERROR << "InitConfig failed!";
    return false;
  }

  // init params
  if (!InitParams()) {
    AERROR << "InitParams failed!";
    return false;
  }

  // init pub sub
  if (!InitReaderWriter()) {
    AERROR << "InitReaderWriter failed!";
    return false;
  }

  data_compare_csv_file_.open(data_compare_csv_path_.c_str(), std::ios::app);
  data_compare_csv_file_ << "timestamp_s" << "," << "ins_delta_x" << ","
                         << "ins_delta_y"
                         << ","
                         << "ins_delta_z"
                         << "," << "ins_delta_roll" << "," << "ins_delta_pitch"
                         << ","
                         << "ins_delta_yaw" << "," << "fusion_delta_x" << ","
                         << "fusion_delta_y"
                         << ","
                         << "fusion_delta_z"
                    << "," << "fusion_delta_roll" << "," << "fusion_delta_pitch"
                         << ","
                         << "fusion_delta_yaw" << std::endl;

  AINFO << "Msf Loc Init success";
  return true;
}

bool MsfLocComponent::Proc(
    const std::shared_ptr<localization::CorrectedImu>& msg) {
  if (nullptr == msg) {
    return false;
  }

  auto imu_ptr =
      std::allocate_shared<ImuData>(Eigen::aligned_allocator<ImuData>());
  message_parser_->ParseCorrImuMsg(msg, &(imu_ptr->timestamp),
                                   &(imu_ptr->linear_acc), &(imu_ptr->ang_vel));
  if (!eskf_initialized_ && ins_loc_enabled_ && ins_pva_enabled_) {
    eskf_.reset();
    eskf_ = std::make_unique<ErrorStateKalmanFilter>(config_.eskf_node);
    if (init_pose_.has_value() && init_vel_.has_value()) {
      eskf_->Init(init_pose_.value(), init_vel_.value(), *imu_ptr);
      eskf_initialized_ = true;
      AINFO << "eskf initialized!";
    }
  } else if (eskf_initialized_) {
    auto add_input_task = std::make_shared<Task>(imu_ptr->timestamp);
    add_input_task->SetWorkItem([=]() { eskf_->AddNewNode(*imu_ptr); });
    task_pool_->Commit(std::move(add_input_task), Task::IMU_Input);
  }
  return true;
}

void MsfLocComponent::InsLocCallback(
    const std::shared_ptr<localization::LocalizationEstimate>& msg) {
  if (nullptr == msg) {
    return;
  }

  // static unsigned int cnt = 0;
  // if (cnt++ % SELECT_INTERVAL_NUM != 0) {
  //   return;
  // }

  AINFO << "Ins Loc msg:" << std::fixed << msg->status_type() << " "
         << msg->header().timestamp_sec() << " " << msg->pose().position().x()
         << " " << msg->pose().position().y() << " "
         << msg->pose().position().z() << " " << msg->pose().euler_angles().x()
         << " " << msg->pose().euler_angles().y() << " "
         << msg->pose().euler_angles().z() << " "
         << msg->pose().linear_velocity().x() << " "
         << msg->pose().linear_velocity().y() << " "
         << msg->pose().linear_velocity().z();

  ins_loc_status_ = msg->status_type();
  if (localization::LocalizationEstimate::NORMAL != ins_loc_status_ &&
      localization::LocalizationEstimate::POOR_ACCURACY != ins_loc_status_) {
    return;
  }

  double pose_x = msg->pose().position().x();
  double pose_y = msg->pose().position().y();
  double pose_z = msg->pose().position().z();
  double quat_w = msg->pose().orientation().qw();
  double quat_x = msg->pose().orientation().qx();
  double quat_y = msg->pose().orientation().qy();
  double quat_z = msg->pose().orientation().qz();

  ins_loc_data_->timestamp = msg->header().timestamp_sec();
  ins_loc_data_->map_veh_pose =
      mmath::SE3({quat_w, quat_x, quat_y, quat_z}, {pose_x, pose_y, pose_z});

  if (localization::LocalizationEstimate::NORMAL == ins_loc_status_) {
    ins_loc_data_->noise_vec << 1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4, 1e-2, 1e-2,
        1e-2;
  } else if (localization::LocalizationEstimate::POOR_ACCURACY ==
             ins_loc_status_) {
    ins_loc_data_->noise_vec << 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3, 1e-1, 1e-1,
        1e-1;
  } else {
    AWARN << "Ins loc status is invalid, eskf not process Gnss";
    return;
  }
  ins_loc_data_->linear_vel << msg->pose().linear_velocity().x(),
      msg->pose().linear_velocity().y(), msg->pose().linear_velocity().z();

  if (!eskf_initialized_) {
    // init_pose_ = (ins_loc_data_->map_veh_pose).getTransformationMatrix();
    // init_vel_ = ins_loc_data_->linear_vel;
    ins_loc_enabled_ = true;
    return;
  }

  // if (config_.use_ins_pose) {
  //   KalmanFilter::Measurement meas;
  //   meas.time = ins_loc_data_->timestamp + INS_TIMESTAMP_OFFSET;
  //   meas.T_nb = ins_loc_data_->map_veh_pose.getTransformationMatrix();
  //   meas.measurement_type = KalmanFilter::POSE;

  //   auto add_ins_meas_pose_task = std::make_shared<Task>(meas.time);
  //   AINFO << "add_ins_meas_pose_task time: "
  //         << std::to_string(add_ins_meas_pose_task->GetTimeStamp());
  //   add_ins_meas_pose_task->SetWorkItem([=]() { eskf_->AddMeasurement(meas);
  //   }); task_pool_->Commit(std::move(add_ins_meas_pose_task),
  //   Task::Ins_Meas_Pose);
  // }
}

void MsfLocComponent::MsfResetCallback(
    const std::shared_ptr<localization::MsfReset>& msg) {
  if (nullptr == msg) {
    return;
  }
  static bool reset = false;
  double timestamp = msg->header().timestamp_sec();
  AINFO << "msf reset timestamp: " << std::to_string(timestamp)
        << "-->reset status: " << reset;
  if (msg->is_reset() && msg->is_reset() != reset) {
    AINFO << "reset eskf timestamp: " << std::to_string(timestamp);
    eskf_initialized_ = false;
    init_pose_.reset();
    init_vel_.reset();
    reset = msg->is_reset();
  }
}

void MsfLocComponent::InsPvaCallback(
    const std::shared_ptr<drivers::gnss::Insx>& msg) {
  if (nullptr == msg) {
    return;
  }

  static unsigned int cnt = 0;
  if (cnt++ % SELECT_INTERVAL_NUM != 0) {
    return;
  }

  // insx
  double timestamp;
  mmath::SE3 Tx_Mp_ins;
  Eigen::Vector3d vel_ins;  // enu coordinates
  Eigen::Vector6d omega_acc;

  // parse llh
  double lon_deg, lat_deg, height;
  timestamp = msg->header().timestamp_sec();
  lon_deg = msg->position().lon();
  lat_deg = msg->position().lat();
  height = msg->position().height();

  char zone[20] = {0};
  Eigen::Vector3d pos;
  double gamma = 0.0;
  mmath::Wgs84toUtm(lon_deg, lat_deg, &pos[0], &pos[1], nullptr, &gamma, zone);
  gamma_ = gamma;
  pos[0] -= config_.mercator_origin[0];
  pos[1] -= config_.mercator_origin[1];
  pos[2] = height;

  AINFO << "Ins pva msg:" << std::fixed << msg->pos_type() << " "
      << msg->header().timestamp_sec() << " " << pos[0]
      << " " << pos[1] << " "
      << pos[2] << " " << msg->euler_angles().x()
      << " " << msg->euler_angles().y() << " "
      << msg->euler_angles().z() << " "
      << msg->linear_velocity().x() << " "
      << msg->linear_velocity().y() << " "
      << msg->linear_velocity().z();

  mmath::SO3 rot = mmath::SO3::fromEulerYPR(
      Eigen::Vector3d(msg->euler_angles().z(), msg->euler_angles().y(),
                      msg->euler_angles().x()) *
      mmath::kDegToRad);
  Tx_Mp_ins = mmath::SE3(rot, pos);

  // vel
  vel_ins << msg->linear_velocity().x(), msg->linear_velocity().y(),
      msg->linear_velocity().z();

  if (!eskf_initialized_) {
    init_pose_ = Tx_Mp_ins.getTransformationMatrix();
    init_vel_ = vel_ins;
    ins_pva_enabled_ = true;
    AINFO << "eskf init-----";
    return;
  }

  if (eskf_initialized_ && config_.use_ins_velocity) {
    KalmanFilter::Measurement meas;
    meas.time = timestamp + INS_TIMESTAMP_OFFSET;
    meas.v_enu = vel_ins;
    meas.measurement_type = KalmanFilter::ENU_VEL;

    auto add_ins_meas_velocity_task = std::make_shared<Task>(meas.time);
    AINFO << "add_ins_meas_velocity_task time: "
          << std::to_string(add_ins_meas_velocity_task->GetTimeStamp());
    add_ins_meas_velocity_task->SetWorkItem(
        [=]() { eskf_->AddMeasurement(meas); });
    task_pool_->Commit(std::move(add_ins_meas_velocity_task),
                       Task::Ins_Meas_Velocity);
  }

  if (eskf_initialized_ && config_.use_ins_pose) {
    KalmanFilter::Measurement meas;
    meas.time = timestamp + INS_TIMESTAMP_OFFSET;
    meas.T_nb = Tx_Mp_ins.getTransformationMatrix();
    meas.measurement_type = KalmanFilter::POSE;

    auto add_ins_meas_pose_task = std::make_shared<Task>(meas.time);
    AINFO << "add_ins_meas_pose_task time: "
          << std::to_string(add_ins_meas_pose_task->GetTimeStamp());
    add_ins_meas_pose_task->SetWorkItem([=]() { eskf_->AddMeasurement(meas); });
    task_pool_->Commit(std::move(add_ins_meas_pose_task), Task::Ins_Meas_Pose);
  }
}

void loc::MsfLocComponent::LidarLocCallback(
    const std::shared_ptr<localization::PoseWithCov>& msg) {
  if (nullptr == msg) {
    return;
  }

  double pose_x = msg->position().x();
  double pose_y = msg->position().y();
  double pose_z = msg->position().z();
  double quat_w = msg->orientation().qw();
  double quat_x = msg->orientation().qx();
  double quat_y = msg->orientation().qy();
  double quat_z = msg->orientation().qz();

  lidar_loc_data_->timestamp = msg->header().timestamp_sec();
  lidar_loc_data_->lidar_loc_pose =
      mmath::SE3({quat_w, quat_x, quat_y, quat_z}, {pose_x, pose_y, pose_z});
  lidar_loc_data_->lidar_loc_pose = lidar_loc_data_->lidar_loc_pose * config_.Tx_veh_ins;
  lidar_loc_data_->fitness_score = msg->fitness_score();
  lidar_loc_data_->is_converged = msg->is_converged();

  if (!lidar_loc_data_->is_converged) {
    lidar_loc_status_ = localization::LocalizationEstimate::LOST;
    return;
  } else if (lidar_loc_data_->fitness_score > NDT_FITNESS_SCORE_THRESH) {
    lidar_loc_status_ = localization::LocalizationEstimate::POOR_ACCURACY;
    return;
  } else {
    lidar_loc_status_ = localization::LocalizationEstimate::NORMAL;
  }

  KalmanFilter::Measurement meas;
  meas.time = lidar_loc_data_->timestamp;
  meas.T_nb = lidar_loc_data_->lidar_loc_pose.getTransformationMatrix();
  meas.measurement_type = KalmanFilter::POSE;

  auto add_lidar_pos_meas_task = std::make_shared<Task>(meas.time);
  AINFO << "add_lidar_pos_meas_task time: "
        << std::to_string(add_lidar_pos_meas_task->GetTimeStamp());
  add_lidar_pos_meas_task->SetWorkItem([=]() { eskf_->AddMeasurement(meas); });
  task_pool_->Commit(std::move(add_lidar_pos_meas_task), Task::Lidar_Meas);
}

void MsfLocComponent::ChassisCallback(
    const std::shared_ptr<canbus::Chassis>& msg) {
  if (nullptr == msg) {
    return;
  }

  static unsigned int cnt = 0;
  if (++cnt % SELECT_INTERVAL_NUM != 0) {
    return;
  }

  // AINFO << "chassis data: " <<  msg->wheel_speed_0() << "," << msg->wheel_speed_1() << "," << msg->wheel_speed_2() << "," << msg->wheel_speed_3();
  chassis_data_->wheel_speed_3 = msg->wheel_speed_3();

  if (std::abs(chassis_data_->wheel_speed_3) > 0.001) return;

  Eigen::Vector3d chassis_vel;
  chassis_vel.x() = 0.0;
  chassis_vel.y() = 0.0;
  chassis_vel.z() = 0.0;

  if (eskf_initialized_ && config_.use_chassis_velocity) {
    KalmanFilter::Measurement meas;
    meas.time = msg->header().timestamp_sec();
    meas.v_b = chassis_vel;
    meas.measurement_type = KalmanFilter::BODY_VEL;

    auto add_chassis_task = std::make_shared<Task>(meas.time);
    ADEBUG << "add_chassis_task time: "
           << std::to_string(add_chassis_task->GetTimeStamp());
    add_chassis_task->SetWorkItem([=]() { eskf_->AddMeasurement(meas); });
    task_pool_->Commit(std::move(add_chassis_task), Task::Chassis_Meas);
  }
}

void MsfLocComponent::ProcessMsfLocStatus() {
  if (localization::LocalizationEstimate::LOST == ins_loc_status_ ||
      localization::LocalizationEstimate::LOST == lidar_loc_status_) {
    msf_loc_status_ = localization::LocalizationEstimate::LOST;
  }

  if (localization::LocalizationEstimate::POOR_ACCURACY == ins_loc_status_ ||
      localization::LocalizationEstimate::POOR_ACCURACY == lidar_loc_status_) {
    msf_loc_status_ = localization::LocalizationEstimate::POOR_ACCURACY;
  }

  if (localization::LocalizationEstimate::NORMAL == ins_loc_status_ ||
      localization::LocalizationEstimate::NORMAL == lidar_loc_status_) {
    msf_loc_status_ = localization::LocalizationEstimate::NORMAL;
  }

  if (localization::LocalizationEstimate::LOST == msf_loc_status_) {
    AWARN << "[msf] LOC_LOST!";
  } else if (localization::LocalizationEstimate::POOR_ACCURACY ==
             msf_loc_status_) {
    AWARN << "[msf] LOC_POOR_ACCURACY!";
  }

  return;
}

void MsfLocComponent::PublishLocalizationEstimate() {
  if (cyber_likely(eskf_initialized_)) {
    auto timestamp = eskf_->GetLatestTimeStamp();
    eskf_->GetOdometry(odom_pose_, linear_vel_, linear_acc_, ang_vel_);

    mmath::SE3 Tx_Mp_ins, Tx_Mp_veh;
    if (!FixHeading(odom_pose_, &Tx_Mp_ins)) return;
    Tx_Mp_veh = Tx_Mp_ins * config_.Tx_ins_veh;

    // const Eigen::Quaterniond& q = config_.Tx_ins_veh.getSO3().getQuaternion();
    // AINFO << "q-----:" << q.w() << "," << q.x() << "," << q.y() << "," << q.z();
    // AINFO << "Msf Tx_Mp_ins---->" << std::fixed << msf_loc_status_ << " "
    //     << timestamp << " " << Tx_Mp_ins.getTranslation().x() << " "
    //     << Tx_Mp_ins.getTranslation().y() << " "
    //     << Tx_Mp_ins.getTranslation().z() << " "
    //     << Tx_Mp_ins.getSO3().getEulerYPR().z() * mmath::kRadToDeg << " "
    //     << Tx_Mp_ins.getSO3().getEulerYPR().y() * mmath::kRadToDeg << " "
    //     << Tx_Mp_ins.getSO3().getEulerYPR().x() * mmath::kRadToDeg << " "
    //     << linear_vel_.x() << " " << linear_vel_.y() << " "
    //     << linear_vel_.z();

    ProcessMsfLocStatus();

    Eigen::Vector6d omega_enuvel_veh;
    Eigen::Vector6d acc_veh_vrf;
    localization::LocalizationEstimate loc_msg;

    // omega, vel
    Eigen::Vector6d omega_enuvel_ins;
    omega_enuvel_ins.topRows(3) = ang_vel_;
    omega_enuvel_ins.bottomRows(3) = linear_vel_;
    mmath::RigidBodyKinematic::calcBodyVelocity(
        omega_enuvel_ins, config_.Tx_ins_veh, &omega_enuvel_veh);

    // acc
    Eigen::Vector6d acc_ins_vrf;
    acc_ins_vrf.topRows(3).setZero();
    acc_ins_vrf.bottomRows(3) = linear_acc_;
    mmath::RigidBodyKinematic::calcBodyAcceleration(
        omega_enuvel_ins, acc_ins_vrf, config_.Tx_ins_veh, &acc_veh_vrf);

    message_parser_->GetLocMsg(timestamp, Tx_Mp_veh, omega_enuvel_veh,
                               acc_veh_vrf.tail<3>(), msf_loc_status_,
                               &loc_msg);
    msf_loc_talker_->Write(loc_msg);

    AINFO << "Msf Loc msg---->" << std::fixed << msf_loc_status_ << " "
           << timestamp << " " << Tx_Mp_veh.getTranslation().x() << " "
           << Tx_Mp_veh.getTranslation().y() << " "
           << Tx_Mp_veh.getTranslation().z() << " "
           << Tx_Mp_veh.getSO3().getEulerYPR().z() * mmath::kRadToDeg << " "
           << Tx_Mp_veh.getSO3().getEulerYPR().y() * mmath::kRadToDeg << " "
           << Tx_Mp_veh.getSO3().getEulerYPR().x() * mmath::kRadToDeg << " "
           << linear_vel_.x() << " " << linear_vel_.y() << " "
           << linear_vel_.z();

    if (print_debug_data_ && data_compare_csv_file_.is_open()) {
      static double start = timestamp;
      static double end = timestamp;
      static mmath::SE3 map_fusion_pose_pre = Tx_Mp_veh;
      static mmath::SE3 map_veh_pose_pre = ins_loc_data_->map_veh_pose;
      end = timestamp;
      if (end - start > PRINT_DATA_INTERVAL) {
        auto diff_fusion_pre_to_curr =
            map_fusion_pose_pre.inverseCompose(Tx_Mp_veh);
        auto diff_ins_pre_to_curr =
            map_veh_pose_pre.inverseCompose(ins_loc_data_->map_veh_pose);

        data_compare_csv_file_
            << std::fixed << std::setprecision(8) << ins_loc_data_->timestamp
            << "," << diff_ins_pre_to_curr.getTranslation().x() << ","
            << diff_ins_pre_to_curr.getTranslation().y() << ","
            << diff_ins_pre_to_curr.getTranslation().z() << ","
            << diff_ins_pre_to_curr.getSO3().getEulerYPR().z() *
                   mmath::kRadToDeg
            << ","
            << diff_ins_pre_to_curr.getSO3().getEulerYPR().y() *
                   mmath::kRadToDeg
            << ","
            << diff_ins_pre_to_curr.getSO3().getEulerYPR().x() *
                   mmath::kRadToDeg
            << "," << diff_fusion_pre_to_curr.getTranslation().x() << ","
            << diff_fusion_pre_to_curr.getTranslation().y() << ","
            << diff_fusion_pre_to_curr.getTranslation().z() << ","
            << diff_fusion_pre_to_curr.getSO3().getEulerYPR().z() *
                   mmath::kRadToDeg
            << ","
            << diff_fusion_pre_to_curr.getSO3().getEulerYPR().y() *
                   mmath::kRadToDeg
            << ","
            << diff_fusion_pre_to_curr.getSO3().getEulerYPR().x() *
                   mmath::kRadToDeg
            << std::endl;

        map_fusion_pose_pre = Tx_Mp_veh;
        map_veh_pose_pre = ins_loc_data_->map_veh_pose;
        start = end;
      }
    }

#if SAVE_SHOW_DATA_FUSION
    if (!fusion_pose_file_ || !fusion_pose_file_->is_open()) {
        return;
    }

    FusionPoseData fusion_pose_data;
    fusion_pose_data.timestamp = timestamp;
    fusion_pose_data.x = Tx_Mp_veh.getTranslation().x();
    fusion_pose_data.y = Tx_Mp_veh.getTranslation().y();
    fusion_pose_data.z = Tx_Mp_veh.getTranslation().z();
    fusion_pose_data.roll = Tx_Mp_veh.getSO3().getEulerYPR().z();
    fusion_pose_data.pitch = Tx_Mp_veh.getSO3().getEulerYPR().y();
    fusion_pose_data.yaw = Tx_Mp_veh.getSO3().getEulerYPR().x();
    fusion_pose_file_->write(reinterpret_cast<const char*>(&fusion_pose_data), sizeof(FusionPoseData));
    fusion_pose_file_->flush();
#endif
  } else {
    AINFO_EVERY(10) << "eskf not initialized, waiting for init...";
    cyber::SleepFor(std::chrono::milliseconds(WAITING_ESKF_INTERVAL_MS));
  }
}

bool MsfLocComponent::FixHeading(const Eigen::Matrix4d& pose,
                                 mmath::SE3* pose_fixed) {
  mmath::SE3 T(pose), T_fixed;

  if (gamma_.has_value()) {
    double roll = T.getSO3().getEulerYPR().z();
    double pitch = T.getSO3().getEulerYPR().y();
    double yaw = T.getSO3().getEulerYPR().x();
    T_fixed = mmath::SE3(mmath::SO3::fromEulerYPR(Eigen::Vector3d(
                             yaw /*+ gamma_.value()*/, pitch, roll)),
                         T.getTranslation());
    *pose_fixed = T_fixed;
  } else {
    AWARN << "gamma_ is not initialized, waiting for init...";
    return false;
  }

  return true;
}

bool MsfLocComponent::InitConfig() {
  // set config
  AINFO << "config_file_path_: " << config_file_path_;
  if (!cyber::common::GetProtoFromFile(config_file_path_, &config_.msf_cfg)) {
    AERROR << "Load ins config failed!";
    return false;
  }

  config_.loc_config_path = config_.msf_cfg.loc_config_path();
  YAML::Node config_node = YAML::LoadFile(config_.loc_config_path);
  double mercator_origin_x =
      config_node["mercator_origin_x"].as<std::double_t>();
  double mercator_origin_y =
      config_node["mercator_origin_y"].as<std::double_t>();
  config_.mercator_origin << mercator_origin_x, mercator_origin_y, 0;
  config_.ins_loc_topic = config_node["loc_topic"].as<std::string>();
  config_.ins_pva_topic = config_node["ins_pva_topic"].as<std::string>();
  config_.ins_calib_file = config_node["ins_calib_file"].as<std::string>();
  config_.ins_corrimu_topic =
      config_node["ins_corrimu_topic"].as<std::string>();
  config_.msf_loc_topic = config_node["msf_loc_topic"].as<std::string>();
  config_.lidar_loc_topic = config_node["lidar_loc_topic"].as<std::string>();
  config_.chassis_topic = config_node["chassis_topic"].as<std::string>();
  config_.msf_reset_topic = config_node["msf_reset_topic"].as<std::string>();
  config_.use_ins_pose = config_node["use_ins_pose"].as<bool>();
  config_.use_ins_velocity = config_node["use_ins_velocity"].as<bool>();
  config_.use_chassis_velocity = config_node["use_chassis_velocity"].as<bool>();
  config_.eskf_node = config_node["error_state_kalman_filter"];
  config_.msf_loc_rate = config_.msf_cfg.msf_loc_rate();
  config_.sync_time_thresh = config_.msf_cfg.sync_time_thresh();
  config_.non_recv_time_thresh = config_.msf_cfg.non_recv_time_thresh();

  AINFO << "loc_config_path: " << config_.loc_config_path;
  AINFO << "ins_loc_topic: " << config_.ins_loc_topic;
  AINFO << "ins_corrimu_topic: " << config_.ins_corrimu_topic;
  AINFO << "msf_loc_topic: " << config_.msf_loc_topic;
  AINFO << "lidar_loc_topic: " << config_.lidar_loc_topic;
  AINFO << "msf_loc_rate: " << config_.msf_loc_rate;
  AINFO << "sync_time_thresh: " << config_.sync_time_thresh;
  AINFO << "non_recv_time_thresh:" << config_.non_recv_time_thresh;
  return true;
}

bool MsfLocComponent::InitParams() {
  lidar_loc_data_ = std::make_unique<LidarLocData>();
  ins_loc_data_ = std::make_unique<InsLocData>();
  chassis_data_ = std::make_unique<ChassisData>();
  task_pool_ = std::make_unique<TaskPool>(config_.max_history_second);
  // params
  message_parser_ = std::make_unique<common::io::MessageParser>();
  config_parser_ = std::make_unique<common::io::ConfigParser>();
  // eskf_ = std::make_unique<ErrorStateKalmanFilter>(config_.eskf_node);

  // ins
  if (!config_parser_->LoadInsConfig(config_.ins_calib_file, nullptr, nullptr,
                                     nullptr, &config_.Tx_veh_ins, nullptr)) {
    AERROR << "LoadInsConfig failed!";
    return false;
  }
  config_.Tx_ins_veh = config_.Tx_veh_ins.inverse();

  return true;
}

bool MsfLocComponent::InitReaderWriter() {
  // listener
  ins_loc_listener_ = node_->CreateReader<localization::LocalizationEstimate>(
      config_.ins_loc_topic,
      std::bind(&MsfLocComponent::InsLocCallback, this, std::placeholders::_1));
  ins_pva_listener_ = node_->CreateReader<drivers::gnss::Insx>(
      config_.ins_pva_topic,
      std::bind(&MsfLocComponent::InsPvaCallback, this, std::placeholders::_1));
  lidar_loc_listener_ = node_->CreateReader<localization::PoseWithCov>(
      config_.lidar_loc_topic, std::bind(&MsfLocComponent::LidarLocCallback,
                                         this, std::placeholders::_1));

  msf_reset_listener_ = node_->CreateReader<localization::MsfReset>(
      config_.msf_reset_topic, std::bind(&MsfLocComponent::MsfResetCallback,
                                         this, std::placeholders::_1));
  chassis_listener_ = node_->CreateReader<canbus::Chassis>(
      config_.chassis_topic, std::bind(&MsfLocComponent::ChassisCallback, this,
                                       std::placeholders::_1));
  // talker
  msf_loc_talker_ = node_->CreateWriter<localization::LocalizationEstimate>(
      config_.msf_loc_topic);

  // timer
  timer_.reset(new cyber::Timer(
      static_cast<std::uint16_t>(1000 / config_.msf_loc_rate),
      [this]() { this->PublishLocalizationEstimate(); }, false));
  timer_->Start();

#if SAVE_SHOW_DATA_FUSION
  fusion_pose_file_ = std::make_unique<std::ofstream>("/century/data/bag/match_data/fusion_pose.bin", std::ios::binary | std::ios::app);
  if (!fusion_pose_file_->is_open()) {
      std::cout << "error opening fusion pose file: " << std::endl;
  }
#endif

  AINFO << "InitReaderWriter success!";
  return true;
}
}  // namespace loc
}  // namespace century
