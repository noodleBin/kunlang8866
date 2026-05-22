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
#include "modules/localization/ins/ins_loc_component.h"

namespace century {
namespace ins {

InsLocComponent::InsLocComponent() {}
InsLocComponent::~InsLocComponent() {}

bool InsLocComponent::Init() {
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

  return true;
}

bool InsLocComponent::InitConfig() {
  // set config
  AINFO << "config_file_path_: " << config_file_path_;

  if (!cyber::common::GetProtoFromFile(config_file_path_, &config_.ins_cfg)) {
    AERROR << "Load ins config failed!";
    return false;
  }

  config_.loc_config_path = config_.ins_cfg.loc_config_path();
  YAML::Node config_node = YAML::LoadFile(config_.loc_config_path);
  double mercator_origin_x =
      config_node["mercator_origin_x"].as<std::double_t>();
  double mercator_origin_y =
      config_node["mercator_origin_y"].as<std::double_t>();
  config_.mercator_origin << mercator_origin_x, mercator_origin_y, 0;
  config_.ins_loc_topic = config_node["loc_topic"].as<std::string>();
  config_.ins_calib_file = config_node["ins_calib_file"].as<std::string>();
  config_.ins_pva_topic = config_node["ins_pva_topic"].as<std::string>();

  AINFO << "loc_config_path: " << config_.loc_config_path;
  AINFO << "ins_loc_topic: " << config_.ins_loc_topic;
  AINFO << "ins_pva_topic: " << config_.ins_pva_topic;

  config_.ins_pos_thresh = config_.ins_cfg.ins_pos_thresh();
  config_.ins_atti_thresh = config_.ins_cfg.ins_atti_thresh();
  config_.ins_z_vel_thresh = config_.ins_cfg.ins_z_vel_thresh();
  config_.sync_time_thresh = config_.ins_cfg.sync_time_thresh();

  AINFO << "ins_pos_thresh: " << config_.ins_pos_thresh;
  AINFO << "ins_atti_thresh: " << config_.ins_atti_thresh;
  AINFO << "ins_z_vel_thresh: " << config_.ins_z_vel_thresh;
  AINFO << "sync_time_thresh: " << config_.sync_time_thresh;

  return true;
}

bool InsLocComponent::InitParams() {
  // params
  message_parser_ = std::make_unique<common::io::MessageParser>();
  config_parser_ = std::make_unique<common::io::ConfigParser>();

  // ins
  if (!config_parser_->LoadInsConfig(config_.ins_calib_file, nullptr, nullptr,
                                     nullptr, &config_.Tx_veh_imu, nullptr)) {
    AERROR << "LoadInsConfig failed!";
    return false;
  }
  config_.Tx_ins_veh = config_.Tx_veh_imu.inverse();

  return true;
}

bool InsLocComponent::InitReaderWriter() {
  tf2_broadcaster_ = std::make_unique<transform::TransformBroadcaster>(node_);

  // talker
  ins_loc_talker_ = node_->CreateWriter<localization::LocalizationEstimate>(
      config_.ins_loc_topic);

  return true;
}

bool InsLocComponent::Proc(const std::shared_ptr<drivers::gnss::Insx> &msg) {
  if (nullptr == msg) {
    return false;
  }

  // insx
  double timestamp;
  mmath::SE3 Tx_Mp_ins;
  Eigen::Vector3d vel_ins;
  Eigen::Vector6d omega_acc;
  message_parser_->ParseInsxMsg(msg, config_.mercator_origin, &timestamp,
                                &Tx_Mp_ins, &vel_ins, &omega_acc);

  // ins std
  Eigen::Vector3d std_llh, std_ypr;
  std_llh << msg->position_std().x(), msg->position_std().y(),
      msg->position_std().z();
  std_ypr << msg->euler_angles_std().z(), msg->euler_angles_std().y(),
      msg->euler_angles_std().x();

  // time sync
  timestamp = msg->header().timestamp_sec();
  double measuretime = msg->measurement_time();
  double sync_diff = timestamp - measuretime;

  // ins status
  static int ins_status = -1;
  if (ins_status != msg->ins_status()) {
    AWARN << std::fixed << "[ins] ins_status changed: " << ins_status << "->"
          << msg->ins_status() << ", " << timestamp
          << "\n pos_type: " << msg->pos_type() << ", std_llh->"
          << std_llh.transpose() << ", std_ypr->" << std_ypr.transpose();
    ins_status = msg->ins_status();
  }

  // init
  mmath::SE3 Tx_Mp_veh;
  Eigen::Vector6d omega_vel_veh_vrf;
  Eigen::Vector6d acc_veh_vrf;
  localization::LocalizationEstimate loc_msg;
  if (!ins_initialized_) {
    ins_loc_status_ = localization::LocalizationEstimate::UNINITIALIZED;
    message_parser_->GetLocMsg(timestamp, Tx_Mp_veh, omega_vel_veh_vrf,
                               acc_veh_vrf.bottomRows(3), ins_loc_status_,
                               &loc_msg);
    ins_loc_talker_->Write(loc_msg);

    ins_initialized_ = message_parser_->CheckInsStatus(msg, &first_ins_ok_time_,
                                                       config_.ins_pos_thresh,
                                                       config_.ins_atti_thresh);
    return true;
  }

  // loc
  Tx_Mp_veh = Tx_Mp_ins.compose(config_.Tx_ins_veh);
  ADEBUG << std::fixed << "## msg timestamp(s): " << timestamp;
  ADEBUG << std::fixed << "ins trans:" << Tx_Mp_veh.getTranslation().transpose()
         << ", ypr:"
         << Tx_Mp_veh.getSO3().getEulerYPR().transpose() * mmath::kRadToDeg;

  // omega, vel
  Eigen::Vector6d omega_vel_ins_vrf;
  omega_vel_ins_vrf.topRows(3) = omega_acc.topRows(3);
  omega_vel_ins_vrf.bottomRows(3) = Tx_Mp_ins.getSO3().inverseRotate(vel_ins);
  mmath::RigidBodyKinematic::calcBodyFrameVelocity(
      omega_vel_ins_vrf, config_.Tx_ins_veh, &omega_vel_veh_vrf);

  // acc
  Eigen::Vector6d acc_ins_vrf;
  acc_ins_vrf.topRows(3).setZero();
  acc_ins_vrf.bottomRows(3) =
      Tx_Mp_ins.getSO3().inverseRotate(omega_acc.bottomRows(3));
  mmath::RigidBodyKinematic::calcBodyFrameAcceleration(
      omega_vel_ins_vrf, acc_ins_vrf, config_.Tx_ins_veh, &acc_veh_vrf);

  // check
  if ((std_llh(0) > 2 * config_.ins_pos_thresh) ||
      (std_llh(1) > 2 * config_.ins_pos_thresh) ||
      (std_ypr(0) > 2 * config_.ins_atti_thresh) ||
      (std::fabs(omega_vel_veh_vrf(5)) > config_.ins_z_vel_thresh) ||
      (std::fabs(sync_diff) > 2 * config_.sync_time_thresh)) {
    ins_loc_status_ = localization::LocalizationEstimate::LOST;
    AERROR << std::fixed << "[ins] LOC_LOST!"
           << "\n pos_type: " << msg->pos_type() << ", std_llh->"
           << std_llh.transpose() << ", std_ypr->" << std_ypr.transpose()
           << "\n z axis velocity: " << omega_vel_veh_vrf(5) 
           << ", z axis reference thresh velocity: " << config_.ins_z_vel_thresh
           << "\n sync_diff time: " << sync_diff << ", timestamp->" << timestamp
           << ", measuretime->" << measuretime;
  } else if ((std_llh(0) > config_.ins_pos_thresh) ||
             (std_llh(1) > config_.ins_pos_thresh) ||
             (std_ypr(0) > config_.ins_atti_thresh) ||
             (std::fabs(sync_diff) > config_.sync_time_thresh)) {
    ins_loc_status_ = localization::LocalizationEstimate::POOR_ACCURACY;
    AWARN << std::fixed << "[ins] LOC_POOR_ACCURACY!"
          << "\n pos_type: " << msg->pos_type() << ", std_llh->"
          << std_llh.transpose() << ", std_ypr->" << std_ypr.transpose()
          << "\n sync_diff time: " << sync_diff << ", timestamp->" << timestamp
          << ", measuretime->" << measuretime;
  } else {
    ins_loc_status_ = localization::LocalizationEstimate::NORMAL;
  }

  message_parser_->GetLocMsg(timestamp, Tx_Mp_veh, omega_vel_veh_vrf,
                             acc_veh_vrf.bottomRows(3), ins_loc_status_,
                             &loc_msg);
  ins_loc_talker_->Write(loc_msg);
  return true;
}

}  // namespace ins
}  // namespace century
