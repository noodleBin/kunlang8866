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

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "third_party/mmath/linear_interpolater.h"
#include "third_party/mmath/rigid_body_kinematic.h"
#include "third_party/mmath/se3.h"

#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/localization/proto/ins_config.pb.h"
#include "modules/localization/proto/localization.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/time/clock.h"
#include "modules/localization/common/io/config_parser.h"
#include "modules/localization/common/io/message_parser.h"
#include "modules/transform/transform_broadcaster.h"

namespace century {
namespace ins {

struct InsLocComponentConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // config
  InsConfig ins_cfg;
  std::string loc_config_path = "";
  std::string ins_calib_file = "";
  Eigen::Vector3d mercator_origin;

  // topic
  std::string ins_pva_topic = "";
  std::string ins_loc_topic = "";

  // params
  mmath::SE3 Tx_veh_imu;
  mmath::SE3 Tx_ins_veh;
  double ins_pos_thresh = 0.3;
  double ins_atti_thresh = 0.5;
  double ins_z_vel_thresh = 0.25;
  double sync_time_thresh = 0.02;
};

class InsLocComponent final : public cyber::Component<drivers::gnss::Insx> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using SE3LinearInterpolater = mmath::LinearInterpolater<mmath::SE3>;
  using Vector3dLinearInterpolater = mmath::LinearInterpolater<Eigen::Vector3d>;
  using Vector6dLinearInterpolater = mmath::LinearInterpolater<Eigen::Vector6d>;
  // using LocType = localization::LocalizationEstimate;

  InsLocComponent();
  ~InsLocComponent();

  bool Init() override;
  bool Proc(const std::shared_ptr<drivers::gnss::Insx>& msg) override;

 private:
  bool InitConfig();
  bool InitParams();
  bool InitReaderWriter();

  // bool CheckInsStatus(const std::shared_ptr<drivers::gnss::Insx>&
  // insx_msg);
  bool PublishPoseBroadcastTF(const localization::LocalizationEstimate& loc);

  // params
  InsLocComponentConfig config_;
  localization::LocalizationEstimate::StatusType ins_loc_status_;
  double first_ins_ok_time_ = 0;
  bool ins_initialized_ = false;

  std::unique_ptr<common::io::MessageParser> message_parser_ = nullptr;
  std::unique_ptr<common::io::ConfigParser> config_parser_ = nullptr;

  std::unique_ptr<Vector6dLinearInterpolater> raw_imu_interpolater_ = nullptr;
  std::unique_ptr<Vector3dLinearInterpolater> angular_vel_Mp_imu_interpolater_ =
      nullptr;
  std::unique_ptr<Vector3dLinearInterpolater> linear_vel_Mp_imu_interpolater_ =
      nullptr;
  std::unique_ptr<SE3LinearInterpolater> Tx_Mp_ins_interpolater_ = nullptr;
  std::unique_ptr<Vector6dLinearInterpolater> Vel6_Mp_ins_interpolater_ =
      nullptr;

  // listener
  std::unique_ptr<transform::TransformBroadcaster> tf2_broadcaster_ = nullptr;
  std::shared_ptr<cyber::Reader<drivers::gnss::Insx>> insx_listener_ = nullptr;

  // talker
  std::shared_ptr<cyber::Writer<localization::LocalizationEstimate>>
      ins_loc_talker_ = nullptr;
};

CYBER_REGISTER_COMPONENT(InsLocComponent);

}  // namespace ins
}  // namespace century
