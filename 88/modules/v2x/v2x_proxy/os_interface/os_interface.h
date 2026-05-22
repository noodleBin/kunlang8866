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

/**
 * @file os_interface.h
 * @brief define v2x proxy module and century os interface
 */

#pragma once

#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/v2x/proto/v2x_obstacles.pb.h"
#include "modules/v2x/proto/v2x_obu_rsi.pb.h"
#include "modules/v2x/proto/v2x_obu_traffic_light.pb.h"
#include "modules/v2x/proto/v2x_traffic_light.pb.h"

#include "cyber/cyber.h"
#include "modules/v2x/common/v2x_proxy_gflags.h"

namespace century {
namespace v2x {
class OsInterFace {
 public:
  OsInterFace();

  ~OsInterFace();

  bool InitFlag() { return init_flag_; }

  void GetLocalizationFromOs(
      const std::shared_ptr<::century::localization::LocalizationEstimate> &msg);

  void GetPlanningAdcFromOs(
      const std::shared_ptr<::century::planning::ADCTrajectory> &msg);

  void SendV2xObuTrafficLightToOs(
      const std::shared_ptr<::century::v2x::obu::ObuTrafficLight> &msg);

  void SendV2xTrafficLightToOs(
      const std::shared_ptr<::century::v2x::IntersectionTrafficLightData> &msg);

  void SendV2xObstacles2Sys(
      const std::shared_ptr<century::v2x::V2XObstacles> &msg);

  void SendV2xTrafficLight4Hmi2Sys(
      const std::shared_ptr<::century::perception::TrafficLightDetection> &msg);

 private:
  template <typename MessageT>
  void SendMsgToOs(::century::cyber::Writer<MessageT> *writer,
                   const std::shared_ptr<MessageT> &msg) {
    if (nullptr == writer) {
      return;
    }
    writer->Write(msg);
  }

  bool InitReaders();

  bool InitWriters();

  std::unique_ptr<::century::cyber::Node> node_ = nullptr;
  std::shared_ptr<
      ::century::cyber::Reader<::century::localization::LocalizationEstimate>>
      localization_reader_ = nullptr;
  std::shared_ptr<::century::cyber::Reader<::century::planning::ADCTrajectory>>
      planning_reader_ = nullptr;
  std::shared_ptr<::century::cyber::Writer<::century::v2x::obu::ObuTrafficLight>>
      v2x_obu_traffic_light_writer_ = nullptr;
  std::shared_ptr<
      ::century::cyber::Writer<::century::v2x::IntersectionTrafficLightData>>
      v2x_traffic_light_writer_ = nullptr;
  std::shared_ptr<
      ::century::cyber::Writer<::century::perception::TrafficLightDetection>>
      v2x_traffic_light_hmi_writer_ = nullptr;
  std::shared_ptr<::century::cyber::Writer<::century::v2x::V2XObstacles>>
      v2x_obstacles_internal_writer_ = nullptr;
  ::century::localization::LocalizationEstimate current_localization_;
  ::century::planning::ADCTrajectory adc_trajectory_msg_;
  bool flag_planning_new_ = false;

  mutable std::mutex mutex_localization_;
  mutable std::mutex mutex_planning_;
  mutable std::condition_variable cond_planning_;

  bool init_flag_ = false;
};
}  // namespace v2x
}  // namespace century
