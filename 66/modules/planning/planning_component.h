/******************************************************************************
 * Copyright 2022 The Century Authors. All Rights Reserved.
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
#include <string>
#include <vector>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/mcloud/proto/mcloud_info.pb.h"
#include "modules/mcloud/proto/super_traffic_light.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/learning_data.pb.h"
#include "modules/planning/proto/pad_msg.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/storytelling/proto/story.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/timer_component.h"
#include "cyber/message/raw_message.h"
#include "cyber/time/clock.h"
#include "modules/planning/common/message_process.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/open_space_planning.h"
#include "modules/planning/planning_base.h"
#include "modules/planning/aeb_planner/aeb_planner.h"
#include "modules/planning/proto/lane_borrow_response.pb.h"
#include "modules/planning/proto/pass_stacker_response.pb.h"
#include "modules/planning/proto/stackers_info.pb.h"
#include "modules/planning/proto/v2x_info.pb.h"
#include "modules/planning/proto/blocking_area_response.pb.h"
#include "modules/planning/proto/barrier.pb.h"
#include "modules/fas_aeb_backend/proto/fas_aeb_backend.pb.h"
#include "modules/dreamview/proto/background_music.pb.h"
#include "modules/planning/proto/top_bull_info.pb.h"

namespace century {
namespace planning {
struct TrajStatus {
  double stamp;
  bool has_traj;
  bool has_traj_for_recovery;
};

class TriggerMsg {
 public:
  TriggerMsg() { type_name_ = "TriggerMsg"; }

  ~TriggerMsg() = default;

  std::string GetTypeName() const { return type_name_; }

  TriggerMsg* New() const { return new TriggerMsg; }

  std::string DebugString() const {
    std::stringstream ss;
    ss << "seq_num_:" << seq_num_;
    ss << ",  trigger_type_ : " << trigger_type_;
    ss << ",  timestamp_ : " << timestamp_;
    return ss.str();
  }

 public:
  uint64_t timestamp_ = 0.0;
  uint32_t seq_num_ = 0;
  uint32_t trigger_type_ = 0;
  std::string type_name_ = "";
};

////////////////// PlanningTriggerComponent //////////////////////////////////
class PlanningTriggerComponent final
    : public cyber::Component<prediction::PredictionObstacles> {
 public:
  PlanningTriggerComponent() { shutdown_.store(false); }
  ~PlanningTriggerComponent() {
    shutdown_.store(true);
    if (trigger_thread_->Joinable()) {
      trigger_thread_->Join();
    }
  }
  bool Init() override;
  bool Proc(
      const std::shared_ptr<prediction::PredictionObstacles>& msg) override;

 private:
  void TriggerProc();

  std::mutex mutex_;

  std::shared_ptr<prediction::PredictionObstacles> received_msg_ = nullptr;
  std::atomic<bool> receive_flag_ = {false};
  std::atomic<uint64_t> received_time_ = {0};
  std::shared_ptr<century::cyber::Writer<TriggerMsg>> trigger_writer_ = nullptr;
  std::shared_ptr<century::cyber::Thread> trigger_thread_ = nullptr;
  std::atomic<bool> shutdown_ = {false};
};
CYBER_REGISTER_COMPONENT(PlanningTriggerComponent)

////////////////// PlanningComponent  ////////////////////////////////////
class PlanningComponent final : public cyber::Component<TriggerMsg> {
 public:
  PlanningComponent() = default;

  ~PlanningComponent() = default;

 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<TriggerMsg>& trigger_msg) override;

 private:
  void CheckRerouting();
  void SetChassisInfoForPlan();
  void ProcessInputData();
  bool CheckInput();
  void ProcessDataForOnlineTraining();
  bool PublishLearningData();
  void CalVehicleInPlayStreet();
  void CalVehicleInCommonJunction();
  bool IsAdcInTypeJunction(const century::hdmap::Junction_Type& junction_type);
  void IsAdcInCommonJunction();
  bool IsJunctionContainAdc(const century::common::VehicleState& vehicle_state,
                            const hdmap::JunctionInfo& junction_info);
  void CalPrpValidTrace(ADCTrajectory* const adc_trajectory);
  bool IsAdcTrajectoryValid(const ADCTrajectory& adc_trajectory);
  void CollectLaneDrivingTrajInfo(ADCTrajectory* const adc_trajectory);
  bool CheckIsEnableRescue(ADCTrajectory* const adc_trajectory);
  void PublishPlanningAebResult(AebResult* const aeb_result);
  void PublishTopBullInfo();
  void PublishPlanningTrajectory(ADCTrajectory* const adc_trajectory);
  void DetectTrafficLightState(const ADCTrajectory& adc_trajectory);
  bool EnablePullover();
  bool EnablePulloverThread();
  bool EnableRescue(ADCTrajectory* const adc_trajectory);
  bool EnableRescueThread();

  bool CheckInIgnoreJunction(century::hdmap::Junction::Type& ingore_junction);
  bool CheckIsTurnSceneToRescue();
  bool CheckIsOperationToRescueCondition(ADCTrajectory* const adc_trajectory);
  bool CheckIsStopByStacker(ADCTrajectory* adc_trajectory);
  bool CheckIsStopByPlanningErrorOrFailed(ADCTrajectory* adc_trajectory);
  bool CheckIsStopByNormalObstacle(ADCTrajectory* adc_trajectory);
  bool CheckDrivingModeEnableRescue();
  bool SameRefHeadingUturnFinished();
  bool EnableDeadEnd();
  bool CheckRescueLaneSafe();
  bool CheckRescueLaneSafeThread();
  bool CheckIsYield();
  bool IsCanPlanTebOnPublicRoad(const ReferenceLineInfo& reference_line_info);
  void CalculateTEBTrajectory(const LocalView& local_view,
                              bool* last_TEB_thread_finished,
                              bool* last_calculation_result);
  void RescueTrajectoryUpdate(ADCTrajectory* const adc_trajectory);
  void CalculatesEnableuseTEBTraces(bool* last_calculation_result,
                                    ADCTrajectory* const adc_trajectory_pb);

  void TakeOverRecord(const LocalView& local_view);
  bool IsVehicleUseAndCollisionElectricFence(ADCTrajectory* const adc_trajectory_pb);
  void CalcuDisplayTypeManual(ADCTrajectory* const ptr_trajectory_pb);
  void GenerateBackgroundMusic(ADCTrajectory* const ptr_trajectory_pb);
  void CheckEndPointInTurnLane(const std::shared_ptr<century::routing::RoutingResponse>& new_routing);
  void HandleRoutingUpdate(const std::shared_ptr<century::routing::RoutingResponse>& new_routing);
  void AdjustRoutingEndIfNeeded(const std::shared_ptr<century::routing::RoutingResponse>& new_routing);
  void HandleTrafficLightAndCloudUpdate();

 private:
  std::shared_ptr<cyber::Reader<planning::BorrowResponse>> get_request_reader_;
  std::shared_ptr<cyber::Reader<planning::PassStackerResponse>> pass_stacker_reponse_reader_;
  std::shared_ptr<cyber::Reader<planning::StackersInfo>> stackers_info_reader_;
  std::shared_ptr<cyber::Reader<planning::V2xInfo>> v2x_info_reader_;
  std::shared_ptr<cyber::Reader<planning::BlockingAreaResponse>> blocking_area_reponse_reader_;
  std::shared_ptr<cyber::Reader<planning::TemporaryParkingRequest>>
      temporary_parking_request_reader_;
  std::shared_ptr<cyber::Reader<planning::TemporaryParkingRequest>>
      multi_path_temp_stop_request_reader_;
  std::shared_ptr<cyber::Reader<planning::Barrier>> barrier_reader_;
  std::shared_ptr<cyber::Reader<prediction::PredictionObstacles>>
      prediction_obstacles_reader_;
  std::shared_ptr<cyber::Reader<perception::PerceptionObstacles>>
      perception_aeb_obstacles_reader_;
  std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>>
      localization_estimate_reader_;
  std::shared_ptr<cyber::Reader<canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<fas_aeb_backend::FasAebInfo>> fas_aeb_reader_;
  std::shared_ptr<cyber::Reader<dreamview::BackgroundMusic>> background_music_reader_;

  std::shared_ptr<cyber::Reader<perception::TrafficLightDetection>>
      traffic_light_reader_;
  std::shared_ptr<cyber::Reader<mcloud::SuperTrafficLight>>
      super_traffic_light_reader_;
  std::shared_ptr<cyber::Reader<mcloud::McloudInfo>> cloud_info_reader_;
  std::shared_ptr<cyber::Reader<routing::RoutingResponse>> routing_reader_;
  std::shared_ptr<cyber::Reader<planning::PadMessage>> pad_msg_reader_;
  std::shared_ptr<cyber::Reader<relative_map::MapMsg>> relative_map_reader_;
  std::shared_ptr<cyber::Reader<storytelling::Stories>> story_telling_reader_;

  std::shared_ptr<cyber::Writer<ADCTrajectory>> planning_writer_;
  std::shared_ptr<cyber::Writer<AebResult>> planning_aeb_writer_;
  std::shared_ptr<cyber::Writer<TopBullInfo>> top_bull_info_writer_;
  std::shared_ptr<cyber::Writer<routing::RoutingRequest>> rerouting_writer_;
  std::shared_ptr<cyber::Writer<PlanningLearningData>>
      planning_learning_data_writer_;
  std::shared_ptr<cyber::Writer<perception::TrafficLightDetection>>
      traffic_light_report_writer_;

  LocalView local_view_;

  std::unique_ptr<PlanningBase> planning_base_;
  std::unique_ptr<PlanningBase> planning_base_openspace_;
  std::shared_ptr<AebPlanner> aeb_planner_;

  std::shared_ptr<DependencyInjector> injector_;
  std::shared_ptr<DependencyInjector> injector_open_space_;

  PlanningConfig config_;
  MessageProcess message_process_;

  std::deque<TrajStatus> traj_status_buffer_;
  const hdmap::HDMap* hdmap_ = nullptr;
  std::mutex frame_teb_mutex_;
  ADCTrajectory last_teb_trajectory_;
  ADCTrajectory last_use_trajectory_;
  std::atomic<uint64_t> teb_trajectory_update_time_ = {0};
  bool is_new_routing_ =false;
  size_t rescue_stop_count_ = 0;
  size_t error_rescue_stop_count_ = 0;
};

CYBER_REGISTER_COMPONENT(PlanningComponent)

}  // namespace planning
}  // namespace century
