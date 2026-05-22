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
#include "modules/planning/planning_component.h"

#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/on_lane_planning.h"

namespace century {
namespace planning {
constexpr double kDistanceToDestination = 6.0;           // m
constexpr double kDistanceToDestinationThread = 8.0;     // m
constexpr double kMinTrajctoryLength = 0.5;              // m
constexpr double kCheckTrajectoryTime = 600;             // ms
constexpr double kCheckTrajectoryTimePublicRoad = 3000;  // ms
constexpr int kMinTrajSize = 5;                          // s
constexpr double kInvalidTrajectoryPercent = 0.8;
constexpr double KStopSpeedThreshold = 0.05;       // m/s
constexpr double KLowSpeedThreshold = 0.5;         // m/s
constexpr double kMaxDistanceToTrafficLine = 500;  // m
constexpr double kDistanceToTrafficLine = 50;      // m
constexpr double kDistanceToOther = 30;            // m
constexpr double kTinyTrajectoryLength = 0.01;     // m
constexpr double kDistanceReach = 5;
constexpr double kSingleLaneWidthThr = 4.5;
constexpr double kMaxSteeringPercentageWhenCruise = 50;
constexpr uint32_t kNewRoutingReportTimes = 3;
constexpr int kMinPRPTrajectorySize = 6;
constexpr double kPi_6 = 0.5267;
constexpr double kTurnWheelAngleThres = 3.5;

using century::common::ErrorCode;
using century::common::PointENU;
using century::cyber::Clock;
using century::cyber::ComponentBase;
using century::hdmap::HDMapUtil;
using century::hdmap::JunctionInfoConstPtr;
using century::hdmap::PathOverlap;
using century::mcloud::McloudInfo;
using century::mcloud::SuperTrafficLight;
using century::perception::TrafficLightDetection;
using century::relative_map::MapMsg;
using century::routing::RoutingRequest;
using century::routing::RoutingResponse;
using century::storytelling::Stories;
using century::common::Status;

bool PlanningTriggerComponent::Init() {
  AINFO << "PlanningTriggerComponent init";
  trigger_writer_ =
      node_->CreateWriter<TriggerMsg>("/century/prediction/trigger");

  trigger_thread_.reset(new century::cyber::Thread(
      "trigger_thread",
      std::bind(&PlanningTriggerComponent::TriggerProc, this)));
  return true;
}

bool PlanningTriggerComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>& msg) {
  AINFO << "trigger Proc msg: " << msg->header().ShortDebugString();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    received_msg_ = msg;
    receive_flag_.store(true);
    received_time_ = century::cyber::Time::Now().ToMillisecond();
  }
  return true;
}

void PlanningTriggerComponent::TriggerProc() {
  constexpr uint32_t kInterval = 100;  // ms
  constexpr uint32_t kShortSleepTime = 5;
  constexpr uint32_t kLongSleepTime = 80;

  static uint64_t last_send_time = 0.0;
  static uint32_t seq = 0;
  static int64_t total_delta =
      0;  // >0 faster than expectedm;   <0 slower than expected
  static int64_t cur_delta = 0;

  while (!shutdown_.load()) {
    uint32_t sleep_duration = kShortSleepTime;
    auto cur_time = century::cyber::Time::Now().ToMillisecond();
    bool trigger_flag = nullptr != received_msg_ &&
                        (cur_time - received_time_ >= kInterval) &&
                        (cur_time - last_send_time >= kInterval);
    if (receive_flag_ || trigger_flag) {
      std::lock_guard<std::mutex> lock(mutex_);

      if (last_send_time != 0) {
        cur_delta = kInterval - (cur_time - last_send_time);
        total_delta += cur_delta;
      }
      AINFO << "trigger status: received_time_:" << received_time_
            << ", last_send_time:" << last_send_time
            << ", cur_time:" << cur_time << ", total_delta:" << total_delta
            << ", cur_delta:" << cur_delta;

      if (receive_flag_.load() || total_delta < kInterval) {
        auto trigger_msg = std::make_shared<TriggerMsg>();
        trigger_msg->seq_num_ = ++seq;
        trigger_msg->timestamp_ = cur_time;
        trigger_msg->trigger_type_ = receive_flag_ ? 1 : 0;
        AINFO << "trigger msg:" << trigger_msg->DebugString();
        trigger_writer_->Write(trigger_msg);
        if (receive_flag_.load()) {
          receive_flag_.store(false);
          sleep_duration = kLongSleepTime;
        }
      } else {
        if (total_delta >= kInterval) {
          total_delta -= kInterval;
        }
      }
      last_send_time = cur_time;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_duration));
  }
}

bool PlanningComponent::Init() {
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
  injector_ = std::make_shared<DependencyInjector>();
  injector_->set_need_to_rescue(false);

  planning_base_ = std::make_unique<OnLanePlanning>(injector_);

  injector_open_space_ = std::make_shared<DependencyInjector>();
  injector_open_space_->set_need_to_rescue_thread(false);
  planning_base_openspace_ =
      std::make_unique<OpenSpacePlanning>(injector_open_space_);
  ACHECK(ComponentBase::GetProtoConfig(&config_))
      << "failed to load planning config file "
      << ComponentBase::ConfigFilePath();

  if (FLAGS_planning_offline_learning ||
      config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    if (!message_process_.Init(config_, injector_)) {
      AERROR << "failed to init MessageProcess";
      return false;
    }
  }

  planning_base_->Init(config_);
  planning_base_openspace_->Init(config_);

  aeb_planner_ = std::make_shared<AebPlanner>(config_.aeb_config());
  aeb_planner_->init();

  fas_aeb_reader_ =
      node_->CreateReader<fas_aeb_backend::FasAebInfo>("/century/fas_aeb_info");
  background_music_reader_ = node_->CreateReader<dreamview::BackgroundMusic>(
      "/century/background_music");

  prediction_obstacles_reader_ =
      node_->CreateReader<prediction::PredictionObstacles>(
          config_.topic_config().prediction_topic());
  perception_aeb_obstacles_reader_ =
      node_->CreateReader<perception::PerceptionObstacles>(
          config_.topic_config().perception_ego_obstacles_topic());
  get_request_reader_ =
      node_->CreateReader<planning::BorrowResponse>("/century/borrow_response");
  pass_stacker_reponse_reader_ =
      node_->CreateReader<planning::PassStackerResponse>(
          "/century/pass_stacker_response");
  stackers_info_reader_ =
      node_->CreateReader<planning::StackersInfo>("/century/stackers_info");
  v2x_info_reader_ =
      node_->CreateReader<planning::V2xInfo>("/century/v2x_info");
  blocking_area_reponse_reader_ =
      node_->CreateReader<planning::BlockingAreaResponse>(
          "/century/blocking_area_response");
  temporary_parking_request_reader_ =
      node_->CreateReader<planning::TemporaryParkingRequest>(
          "/century/temporary_parking_request");
  multi_path_temp_stop_request_reader_ =
      node_->CreateReader<planning::TemporaryParkingRequest>(
          "/century/multi_path_temp_stop_request");
  barrier_reader_ = node_->CreateReader<planning::Barrier>("/century/barrier");
  localization_estimate_reader_ =
      node_->CreateReader<localization::LocalizationEstimate>(
          config_.topic_config().localization_topic());
  chassis_reader_ = node_->CreateReader<canbus::Chassis>(
      config_.topic_config().chassis_topic());

  routing_reader_ = node_->CreateReader<RoutingResponse>(
      config_.topic_config().routing_response_topic());

  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      config_.topic_config().traffic_light_detection_topic());

  super_traffic_light_reader_ = node_->CreateReader<SuperTrafficLight>(
      config_.topic_config().super_traffic_light_detection_topic());

  cloud_info_reader_ = node_->CreateReader<mcloud::McloudInfo>(
      config_.topic_config().mcloud_topic());

  pad_msg_reader_ = node_->CreateReader<PadMessage>(
      config_.topic_config().planning_pad_topic());

  story_telling_reader_ = node_->CreateReader<Stories>(
      config_.topic_config().story_telling_topic());

  if (FLAGS_use_navigation_mode) {
    relative_map_reader_ = node_->CreateReader<MapMsg>(
        config_.topic_config().relative_map_topic());
  }
  planning_writer_ = node_->CreateWriter<ADCTrajectory>(
      config_.topic_config().planning_trajectory_topic());

  planning_aeb_writer_ = node_->CreateWriter<AebResult>(
      config_.topic_config().planning_aeb_topic());

  top_bull_info_writer_ =
      node_->CreateWriter<TopBullInfo>("/century/top_bull_info");

  rerouting_writer_ = node_->CreateWriter<RoutingRequest>(
      config_.topic_config().routing_request_topic());

  planning_learning_data_writer_ = node_->CreateWriter<PlanningLearningData>(
      config_.topic_config().planning_learning_data_topic());

  traffic_light_report_writer_ = node_->CreateWriter<TrafficLightDetection>(
      config_.topic_config().traffic_light_report_topic());

  return true;
}

bool PlanningComponent::Proc(const std::shared_ptr<TriggerMsg>& trigger_msg) {
  ADEBUG << __func__;
  AINFO << "Proc  received  trigger_msg: " << trigger_msg->DebugString();
  AINFO << "---------------------- Proc ----------------------";
  // check and process possible rerouting request

  CheckRerouting();

  // process fused input data
  ProcessInputData();
  AebResult aeb_result_pb;
  injector_->SetIsVehCollisionElectricFence(false);
  if (FLAGS_enable_electric_fence_drivable_area) {
    ADCTrajectory estop_trajectory;
    if (IsVehicleUseAndCollisionElectricFence(&estop_trajectory)) {
      CalcuDisplayTypeManual(&estop_trajectory);
      PublishPlanningTrajectory(&estop_trajectory);
      PublishPlanningAebResult(&aeb_result_pb);
      return false;
    }
  }

  ADCTrajectory adc_trajectory_pb;
  double aed_process_time = 0.0;
  const auto start_aeb_process_time = Clock::NowInSeconds();
  ADEBUG << "FLAGS_enable_aeb_planning: " << FLAGS_enable_aeb_planning;
  ADEBUG << "injector_ != nullptr: " << (injector_ != nullptr);
  aeb_planner_->setSeqNum(trigger_msg->seq_num_);
  if (FLAGS_enable_aeb_planning) {
    ADEBUG << "aeb_send_seq: " << trigger_msg->seq_num_;
    bool is_aeb_ready = aeb_planner_->GetAebResultFromPerceptionObstacles(
        start_aeb_process_time, local_view_, &adc_trajectory_pb,
        &aeb_result_pb);
    ADEBUG << "is_aeb_ready: " << is_aeb_ready;
  }
  aeb_planner_->UpdateMapInfo();
  AINFO << "aeb_result_pb_warning_level: " << aeb_result_pb.warning_level()
        << ", enable_auto_aeb: " << aeb_result_pb.ready_to_enable_aeb();
  aed_process_time = Clock::NowInSeconds() - start_aeb_process_time;
  AINFO << "AEB process time: " << aed_process_time;
  if (!aeb_result_pb.has_warning_level()) {
    ADEBUG << "No Warning Level !!!";
    aeb_result_pb.set_warning_level(
        ::century::planning::AebWarningLevel::WARNING_LEVEL_NONE);
  }
  // aeb debug info output
  ADEBUG << "FLAGS_enable_record_debug: " << FLAGS_enable_record_debug;
  if (FLAGS_enable_record_debug) {
    auto* aeb_info = aeb_result_pb.mutable_aeb_debug();
    aeb_info->set_time(aed_process_time);
    aeb_planner_->RecordDebugData(local_view_, aeb_result_pb,
                                 aeb_result_pb.mutable_aeb_debug());
    ADEBUG << "aeb_debug_info: " << aeb_info->DebugString();
  }
  PublishPlanningAebResult(&aeb_result_pb);

  if (!CheckInput()) {
    AERROR << "Input check failed";
    return false;
  }

  ProcessDataForOnlineTraining();
  ADEBUG << "trigger_msg->seq_num: " << trigger_msg->seq_num_;

  // publish learning data frame for RL test
  if (config_.learning_mode() == PlanningConfig::RL_TEST) {
    return PublishLearningData();
  }

  planning_base_->RunOnce(local_view_, &adc_trajectory_pb);
  if (local_view_.cloud_info != nullptr && is_new_routing_) {
    // AINFO<<"immediately stop =
    // "<<local_view_.cloud_info->immediately_parking();
    adc_trajectory_pb.set_has_reached_station(true);
    adc_trajectory_pb.set_has_reached_destination(true);
  }

  if (!injector_->is_in_play_street || SameRefHeadingUturnFinished()) {
    // AINFO << "Reset enable uturn flag";
    injector_->is_need_to_uturn_ = false;
  }
  if (!injector_->is_need_to_uturn_ && !injector_->pullover_finished) {
    injector_->is_need_to_uturn_ = EnableDeadEnd();
    injector_open_space_->is_need_to_uturn_ = EnableDeadEnd();
  }
  CalcuDisplayTypeManual(&adc_trajectory_pb);
  CollectLaneDrivingTrajInfo(&adc_trajectory_pb);

  CheckIsEnableRescue(&adc_trajectory_pb);
  if (FLAGS_enable_TEB_thread) {
    RescueTrajectoryUpdate(&adc_trajectory_pb);
  }
  const auto* frame = injector_->frame_history()->Latest();
  if (frame != nullptr) {
    AINFO << "is_on_open_space_trajectory: "
          << frame->open_space_info().is_on_open_space_trajectory();
  } else {
    AWARN << "Frame history is empty after planning run.";
  }

  TakeOverRecord(local_view_);

  PublishPlanningTrajectory(&adc_trajectory_pb);
  DetectTrafficLightState(adc_trajectory_pb);
  PublishTopBullInfo();

  // record in history
  {
    std::lock_guard<std::mutex> frame_teb_lock(frame_teb_mutex_);
    injector_->history()->Add(adc_trajectory_pb);
    injector_open_space_->history()->Add(adc_trajectory_pb);
  }

  return true;
}

void PlanningComponent::PublishTopBullInfo() {
  TopBullInfo top_bull_info;
  // get path info
  const DiscretizedPath& last_path =
      injector_->last_path_data().discretized_path();
  for (size_t i = 0; i < last_path.size(); ++i) {
    auto point = top_bull_info.add_path_info();
    point->set_x(last_path[i].x());
    point->set_y(last_path[i].y());
    point->set_theta(last_path[i].theta());
  }

  // get turn type
  auto adc_lane_turn = injector_->get_adc_lane_turn();
  if (hdmap::Lane_LaneTurn::Lane_LaneTurn_LEFT_TURN == adc_lane_turn) {
    top_bull_info.set_turn_type(LEFT_TURN);
  } else if (hdmap::Lane_LaneTurn::Lane_LaneTurn_RIGHT_TURN == adc_lane_turn) {
    top_bull_info.set_turn_type(RIGHT_TURN);
  } else {
    top_bull_info.set_turn_type(NO_TURN);
  }

  auto top_bull = injector_->planning_context()->planning_status().top_bull();

  // get random number
  top_bull_info.set_random_number(top_bull.random_number());
  top_bull_info.set_is_in_top_bull(top_bull.is_in_top_bull());
  if (TopBullStatus::WAITING == top_bull.action_type()) {
    top_bull_info.set_top_bull_type(TB_WAITING);
  } else if (TopBullStatus::BORROW == top_bull.action_type()) {
    top_bull_info.set_top_bull_type(TB_BORROW);
  } else if (TopBullStatus::REVERSE == top_bull.action_type()) {
    top_bull_info.set_top_bull_type(TB_REVERSE);
  } else {
    top_bull_info.set_top_bull_type(TB_NONE);
  }

  top_bull_info.set_top_bull_msg(top_bull.top_bull_msg());

  // publish
  common::util::FillHeader(node_->Name(), &top_bull_info);
  top_bull_info_writer_->Write(top_bull_info);
}

void PlanningComponent::SetChassisInfoForPlan() {
  if (!FLAGS_enable_canbus_info_conver) {
    return;
  }

  if (nullptr == local_view_.chassis) {
    AERROR << "No chasis input";
    return;
  }

  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  // set speedd_mps
  const auto& front_drive_wheel_speed =
      local_view_.chassis->front_drive_wheel_speed();
  const auto& back_drive_wheel_speed =
      local_view_.chassis->back_drive_wheel_speed();
  AINFO << "front_drive_wheel_speed = " << front_drive_wheel_speed;
  AINFO << "back_drive_wheel_speed = " << back_drive_wheel_speed;
  auto speed_mps = (front_drive_wheel_speed + back_drive_wheel_speed) * 0.5;
  AINFO << "speed_mps = " << speed_mps;
  local_view_.chassis->set_speed_mps(speed_mps);

  // set throttle_percentage (Forward gear is positive)
  const auto& front_motor_torque = local_view_.chassis->front_motor_torque();
  const auto& back_motor_torque = local_view_.chassis->back_motor_torque();
  AINFO << "front_motor_torque = " << front_motor_torque;
  AINFO << "back_motor_torque = " << back_motor_torque;
  auto average_motor_torque = (front_motor_torque + back_motor_torque) * 0.5;
  AINFO << "average_motor_torque = " << average_motor_torque;
  auto throttle_percentage =
      std::fabs(average_motor_torque) * 100.0 / vehicle_param.max_torque();
  AINFO << "throttle_percentage = " << throttle_percentage;
  local_view_.chassis->set_throttle_percentage(throttle_percentage);

  // set brake_percentage
  auto brake_percentage =
      (local_view_.chassis->y2_12_brake_proportional_electromagnet() +
       local_view_.chassis->y3_34_brake_proportional_electromagnet()) *
      0.5;
  AINFO << "local_view_.chassis->y2_12_brake_proportional_electromagnet() = "
        << local_view_.chassis->y2_12_brake_proportional_electromagnet();
  AINFO << "local_view_.chassis->y3_34_brake_proportional_electromagnet() = "
        << local_view_.chassis->y3_34_brake_proportional_electromagnet();
  AINFO << "brake_percentage = " << brake_percentage;
  local_view_.chassis->set_brake_percentage(brake_percentage);

  // set steering_percentage
  auto average_steering_front =
      (local_view_.chassis->bridge_1_left_wheel_angle() +
       local_view_.chassis->bridge_1_right_wheel_angle()) *
      0.5;

  double steering_percentage =
      std::fabs(average_steering_front * M_PI_2 / 90.0) * 100.0 /
      vehicle_param.max_steer_angle();
  AINFO << "steering_percentage = " << steering_percentage;

  local_view_.chassis->set_steering_percentage(steering_percentage);

  return;
}
void PlanningComponent::ProcessInputData() {
  prediction_obstacles_reader_->Observe();
  local_view_.prediction_obstacles =
      prediction_obstacles_reader_->GetLatestObserved();
  perception_aeb_obstacles_reader_->Observe();
  local_view_.perception_aeb_obstacles =
      perception_aeb_obstacles_reader_->GetLatestObserved();

  fas_aeb_reader_->Observe();
  local_view_.fas_aeb_result = fas_aeb_reader_->GetLatestObserved();

  background_music_reader_->Observe();
  local_view_.background_music = background_music_reader_->GetLatestObserved();

  localization_estimate_reader_->Observe();
  local_view_.localization_estimate =
      localization_estimate_reader_->GetLatestObserved();

  chassis_reader_->Observe();
  local_view_.chassis = chassis_reader_->GetLatestObserved();
  // SetChassisInfoForPlan();

  routing_reader_->Observe();
  HandleRoutingUpdate(routing_reader_->GetLatestObserved());

  HandleTrafficLightAndCloudUpdate();

  pad_msg_reader_->Observe();
  local_view_.pad_msg = pad_msg_reader_->GetLatestObserved();

  story_telling_reader_->Observe();
  local_view_.stories = story_telling_reader_->GetLatestObserved();

  if (FLAGS_use_navigation_mode) {
    relative_map_reader_->Observe();
    local_view_.relative_map = relative_map_reader_->GetLatestObserved();
  }
  get_request_reader_->Observe();
  local_view_.borrow_response = get_request_reader_->GetLatestObserved();
  pass_stacker_reponse_reader_->Observe();
  local_view_.pass_stacker_response =
      pass_stacker_reponse_reader_->GetLatestObserved();
  stackers_info_reader_->Observe();
  local_view_.stackers_info = stackers_info_reader_->GetLatestObserved();
  v2x_info_reader_->Observe();
  local_view_.v2x_info = v2x_info_reader_->GetLatestObserved();
  blocking_area_reponse_reader_->Observe();
  local_view_.blocking_area_response =
      blocking_area_reponse_reader_->GetLatestObserved();
  temporary_parking_request_reader_->Observe();
  local_view_.temporary_parking_request =
      temporary_parking_request_reader_->GetLatestObserved();
  multi_path_temp_stop_request_reader_->Observe();
  local_view_.multi_path_temp_stop_request =
      multi_path_temp_stop_request_reader_->GetLatestObserved();
  barrier_reader_->Observe();
  local_view_.barrier = barrier_reader_->GetLatestObserved();
}

void PlanningComponent::CheckEndPointInTurnLane(
    const std::shared_ptr<century::routing::RoutingResponse>& new_routing) {
  auto routing_requet = new_routing->mutable_routing_request();
  bool is_static_wating_request =
      (routing_requet->task_type() == routing::YARD_WAITINGAREA_STATIC);
  if (!is_static_wating_request) {
    return;
  }
  auto routing_end = routing_requet->mutable_waypoint()->rbegin();
  double routing_end_x = routing_end->pose().x();
  double routing_end_y = routing_end->pose().y();
  auto end_point_lane_id =
      routing_requet->waypoint().at(routing_requet->waypoint().size() - 1).id();
  century::hdmap::Id target_id;
  target_id.set_id(end_point_lane_id);
  // ues routing end id search lane.
  auto end_point_lane = hdmap_->GetLaneById(target_id);
  // check is turn lane
  if (!end_point_lane) {
    return;
  }
  bool need_get_predecessor_straight_lane = false;
  if (end_point_lane->lane().has_turn()) {
    if (end_point_lane->lane().turn() != hdmap::Lane::NO_TURN) {
      need_get_predecessor_straight_lane = true;
    }
  }
  if (!need_get_predecessor_straight_lane) {
    return;
  }
  // get all lane id
  std::vector<std::string> all_lane_ids;
  for (int road_index = 0; road_index < new_routing->road_size();
       ++road_index) {
    auto road_segment = new_routing->mutable_road(road_index);
    for (int passage_index = 0; passage_index < road_segment->passage_size();
         ++passage_index) {
      auto passage = road_segment->passage(passage_index);
      for (int lane_index = 0; lane_index < passage.segment_size();
           ++lane_index) {
        if (passage.segment(lane_index).id().empty()) {
          AERROR << "Current lane_index" << lane_index
                 << " | Failed to get lane id!";
          break;
        }
        all_lane_ids.emplace_back(passage.segment(lane_index).id());
      }
    }
  }

  auto predecessor_id = end_point_lane->lane().predecessor_id();
  if (predecessor_id.empty()) {
    // maby no predecessor lane.no consider
    return;
  }
  for (auto id : predecessor_id) {
    auto pre_lane = hdmap_->GetLaneById(id);
    if (pre_lane != nullptr) {
      // check predecessor id in routing lane.
      bool is_in_routing_lane =
          std::find(all_lane_ids.begin(), all_lane_ids.end(), id.id()) !=
          all_lane_ids.end();
      if (!is_in_routing_lane) {
        continue;
      }
      // and in straight lane
      if (pre_lane->lane().has_turn()) {
        if (pre_lane->lane().turn() != hdmap::Lane::NO_TURN) {
          continue;
        }
      }
      const auto& point = pre_lane->GetSmoothPoint(pre_lane->lane().length());
      routing_end->mutable_pose()->set_x(point.x());
      routing_end->mutable_pose()->set_y(point.y());
      routing_end->set_s(pre_lane->lane().length());
      routing_end->set_id(id.id());
      AINFO << "point -> " << point.x() << "," << point.y();
      break;
    }
  }

  // if adc near last rouitng end ,use adc pose
  double distance_adc_and_routing_end =
      std::sqrt(std::pow(routing_end_x -
                             injector_->vehicle_state()->pose().position().x(),
                         2) +
                std::pow(routing_end_y -
                             injector_->vehicle_state()->pose().position().y(),
                         2));
  AINFO << "distance_adc_and_routing_end = " << distance_adc_and_routing_end;
  double distance_new_routing_end_to_last_routing_end =
      std::sqrt(std::pow(routing_end->pose().x() - routing_end_x, 2) +
                std::pow(routing_end->pose().y() - routing_end_y, 2));
  if (distance_adc_and_routing_end <
      distance_new_routing_end_to_last_routing_end) {
    routing_end->mutable_pose()->set_x(
        injector_->vehicle_state()->pose().position().x());
    routing_end->mutable_pose()->set_y(
        injector_->vehicle_state()->pose().position().y());
  }
}

void PlanningComponent::HandleRoutingUpdate(
    const std::shared_ptr<century::routing::RoutingResponse>& new_routing) {
  if (new_routing == nullptr) {
    local_view_.routing = nullptr;
    return;
  }
  if (local_view_.routing != nullptr &&
      !hdmap::PncMap::IsNewRouting(*local_view_.routing, *new_routing)) {
    return;
  }
  if (FLAGS_enable_change_routing_end) {
    AdjustRoutingEndIfNeeded(new_routing);
    CheckEndPointInTurnLane(new_routing);
  }
  local_view_.routing = new_routing;
}

void PlanningComponent::AdjustRoutingEndIfNeeded(
    const std::shared_ptr<century::routing::RoutingResponse>& new_routing) {
  double change_routing_end_distance = FLAGS_change_routing_end_distance;
  auto routing_requet = new_routing->mutable_routing_request();
  auto task_type = routing_requet->task_type();

  bool is_static_wating_request =
      task_type == routing::LOADING_OPERATIONAREA_SAMEDIRECTION_1 ||
      task_type == routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_0 ||
      task_type == routing::LOADING_OPERATIONAREA_SAMEDIRECTION_3_1 ||
      task_type == routing::YARD_WAITINGAREA_STATIC ||
      task_type == routing::LOADING_OPERATIONAREA_SAMEDIRECTION_3_0 ||
      task_type == routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_1 ||
      task_type == routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_1;

  if (is_static_wating_request && task_type != routing::YARD_WAITINGAREA_STATIC) {
    const auto& vehicle_param =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
    change_routing_end_distance = vehicle_param.length() / 4;
  }

  if (!is_static_wating_request || routing_requet->rerouting_info().is_rerouting() ||
      routing_requet->rerouting_info().huaman_shaped().is_part_rerouting() ||
      routing_requet->rerouting_info().dead_road().is_rerouting() ||
      routing_requet->waypoint_size() <= 1) {
    return;
  }

  auto routing_end = routing_requet->mutable_waypoint()->rbegin();
  auto end_point_lane_id =
      routing_requet->waypoint().at(routing_requet->waypoint().size() - 1).id();
  century::hdmap::Id target_id;
  target_id.set_id(end_point_lane_id);
  // ues routing end id search lane.
  auto end_point_lane = hdmap_->GetLaneById(target_id);
  if (end_point_lane == nullptr) {
    return;
  }

  bool need_adjust =
      ((task_type == routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_1 ||
        task_type == routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_0 ||
        task_type == routing::LOADING_OPERATIONAREA_SAMEDIRECTION_3_1 ||
        task_type == routing::LOADING_OPERATIONAREA_SAMEDIRECTION_3_0) &&
       routing_requet->is_early_stop()) ||
      task_type == routing::YARD_WAITINGAREA_STATIC ||
      task_type == routing::LOADING_OPERATIONAREA_SAMEDIRECTION_1;

  if (!need_adjust) {
    return;
  }
  double s = 0.0;
  double l = 0.0;
  end_point_lane->GetProjection(
      {routing_end->pose().x(), routing_end->pose().y()}, &s, &l);
  double distance_adc_and_routing_end = std::sqrt(
      std::pow(
          routing_end->pose().x() -
              injector_->vehicle_state()->pose().position().x(),
          2) +
      std::pow(
          routing_end->pose().y() -
              injector_->vehicle_state()->pose().position().y(),
          2));
  if (distance_adc_and_routing_end < change_routing_end_distance) {
    routing_end->mutable_pose()->set_x(
        injector_->vehicle_state()->pose().position().x());
    routing_end->mutable_pose()->set_y(
        injector_->vehicle_state()->pose().position().y());
    return;
  }

  if (s > change_routing_end_distance) {
    // no change lane id
    double accumulate_s = s - change_routing_end_distance;
    const auto& point = end_point_lane->GetSmoothPoint(accumulate_s);
    routing_end->mutable_pose()->set_x(point.x());
    routing_end->mutable_pose()->set_y(point.y());
    routing_end->set_s(accumulate_s);
    AINFO << "point -> " << point.x() << "," << point.y();
    return;
  }

  // search predecessor lane
  double remaining_s = change_routing_end_distance - s;
  auto routing_segment = new_routing->road();
  auto iter = routing_segment.begin();
  std::vector<std::string> passage_id;
  while (iter != routing_segment.end()) {
    for (auto passage : iter->passage()) {
      for (auto segment : passage.segment()) {
        passage_id.push_back(segment.id());
      }
    }
    ++iter;
  }

  if (passage_id.empty()) {
    return;
  }
  bool get_end_point = false;
  while (!get_end_point) {
    auto predecessor_id = end_point_lane->lane().predecessor_id();
    if (predecessor_id.empty()) {
      // maby no predecessor lane.no consider
      break;
    }
    bool has_successor_lane_in_routing = false;
    for (auto iter_lane_id : passage_id) {
      for (auto id : predecessor_id) {
        if (iter_lane_id != id.id()) {
          continue;
        }
        has_successor_lane_in_routing = true;
        end_point_lane = hdmap_->GetLaneById(id);
        if (end_point_lane != nullptr) {
          if (end_point_lane->lane().length() > remaining_s) {
            remaining_s = end_point_lane->lane().length() - remaining_s;
            const auto& point = end_point_lane->GetSmoothPoint(remaining_s);
            routing_end->mutable_pose()->set_x(point.x());
            routing_end->mutable_pose()->set_y(point.y());
            routing_end->set_s(remaining_s);
            routing_end->set_id(iter_lane_id);
            get_end_point = true;
          } else {
          }
        }
      }
    }
    if (!has_successor_lane_in_routing) {
      has_successor_lane_in_routing = false;
      break;
    }
  }
}

void PlanningComponent::HandleTrafficLightAndCloudUpdate() {
  traffic_light_reader_->Observe();
  local_view_.traffic_light = traffic_light_reader_->GetLatestObserved();

  super_traffic_light_reader_->Observe();
  local_view_.super_traffic_light =
      super_traffic_light_reader_->GetLatestObserved();

  cloud_info_reader_->Observe();
  const auto& new_cloud_info = cloud_info_reader_->GetLatestObserved();
  if (nullptr != new_cloud_info) {
    bool is_last_cloud_info_null = false;
    if (nullptr == local_view_.cloud_info) {
      local_view_.cloud_info = new_cloud_info;
      is_last_cloud_info_null = true;
    }
    if (local_view_.cloud_info->has_header() && new_cloud_info->has_header()) {
      bool is_different = local_view_.cloud_info->header().sequence_num() !=
                          new_cloud_info->header().sequence_num();
      is_new_routing_ = false;
      if (is_different ||
          (is_last_cloud_info_null && new_cloud_info->immediately_parking())) {
        is_new_routing_ = true;
        local_view_.cloud_info = new_cloud_info;
        if (new_cloud_info->immediately_parking()) {
          auto* destination = injector_->planning_context()
                                  ->mutable_planning_status()
                                  ->mutable_destination();
          destination->set_arrived_station_immediately(true);
          planning::BorrowResponse borrow_response;
          borrow_response.set_response_type(planning::ResponseType::UNTREATED);
          borrow_response.set_block_obs_id("");
          borrow_response.set_has_response(false);
          injector_->set_borrow_response(borrow_response);
          injector_->borrow_response().clear_block_obs_id();
          injector_->set_use_reverse_trajectory(false);
          injector_->set_use_reverse_type(
              ReverseTrajectoryType::FORWARD_DRIVING);
          AINFO << "cloud_info arrived_station_immediately is true.";
        } else {
          auto* destination = injector_->planning_context()
                                  ->mutable_planning_status()
                                  ->mutable_destination();
          destination->set_arrived_station_immediately(false);
          AINFO << "cloud_info arrived_station_immediately is false.";
        }
      }
    }
  } else {
    local_view_.cloud_info = nullptr;
  }
}

void PlanningComponent::ProcessDataForOnlineTraining() {
  if (config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    // data process for online training
    message_process_.OnChassis(*local_view_.chassis);
    message_process_.OnPrediction(*local_view_.prediction_obstacles);
    message_process_.OnRoutingResponse(*local_view_.routing);
    message_process_.OnStoryTelling(*local_view_.stories);
    message_process_.OnTrafficLightDetection(*local_view_.traffic_light);
    message_process_.OnLocalization(*local_view_.localization_estimate);
  }
}

bool PlanningComponent::PublishLearningData() {
  PlanningLearningData planning_learning_data;
  LearningDataFrame* learning_data_frame =
      injector_->learning_based_data()->GetLatestLearningDataFrame();
  if (learning_data_frame) {
    planning_learning_data.mutable_learning_data_frame()->CopyFrom(
        *learning_data_frame);
    common::util::FillHeader(node_->Name(), &planning_learning_data);
    planning_learning_data_writer_->Write(planning_learning_data);
  } else {
    AERROR << "fail to generate learning data frame";
    return false;
  }
  return true;
}

bool PlanningComponent::CheckIsEnableRescue(
    ADCTrajectory* const adc_trajectory) {
  if (!FLAGS_enable_rescue) {
    AERROR << "disable rescue from config.";
    return false;
  }

  OpenspaceCommon::IsRoutingReverseDriving(local_view_);
  AINFO << "is_reverse_routing: " << OpenspaceCommon::is_reverse_routing();
  // check adc is near destination.
  Status status = injector_->vehicle_state()->Update(
      *local_view_.localization_estimate, *local_view_.chassis);
  if (!status.ok()) {
    AERROR << "update vehicle state failed: " << status;
    return false;
  }
  if (nullptr == local_view_.routing ||
      !local_view_.routing->has_routing_request() ||
      local_view_.routing->routing_request().waypoint().empty()) {
    AERROR << "routing or routing_request or waypoints is empty.";
    return false;
  }
   const auto& point_1 = local_view_.localization_estimate->pose();
   const auto& point_2 =
       local_view_.routing->routing_request().waypoint().rbegin()->pose();
   const auto& distance = std::hypot(point_1.position().x() - point_2.x(),
                                     point_1.position().y() - point_2.y());
   injector_->is_in_near_goal_ = distance < kDistanceToDestination;
   injector_->is_reach_goal_ = distance < kDistanceReach;
   if (!FLAGS_enable_TEB_thread && !injector_->pullover_finished) {
     if (!injector_->need_to_rescue() && !injector_->is_need_to_uturn_) {
       if (FLAGS_enable_always_teb || FLAGS_enable_rescue_replan_reason2 ||
           EnableRescue(adc_trajectory)) {
         injector_->set_need_to_rescue(true);
         adc_trajectory->set_trajectory_scenario(ADCTrajectory::OPENSPACE);
         return true;
       }
     }
  } else if (FLAGS_enable_TEB_thread) {
    std::lock_guard<std::mutex> frame_teb_lock(frame_teb_mutex_);
    injector_open_space_->is_in_near_goal_ =
        distance < kDistanceToDestinationThread;
    injector_open_space_->is_reach_goal_ = distance < kDistanceReach;

    if (!injector_open_space_->need_to_rescue_thread() &&
        !injector_open_space_->is_need_to_uturn_) {
      if (FLAGS_enable_always_teb || FLAGS_enable_rescue_replan_reason2 ||
          EnableRescueThread()) {
        injector_open_space_->set_need_to_rescue_thread(true);
        adc_trajectory->set_trajectory_scenario(ADCTrajectory::OPENSPACE);
        return true;
      }
    }
  }

  return false;
}

void PlanningComponent::PublishPlanningTrajectory(
    ADCTrajectory* const adc_trajectory) {
  auto start_time = adc_trajectory->header().timestamp_sec();
#ifdef TIMEDELAY_TRACE_ENABLE
  uint64_t lastseq = adc_trajectory->header().sequence_num();
#endif
  common::util::FillHeader(node_->Name(), adc_trajectory);
  // modify trajectory relative time due to the timestamp change in header
  const double dt = start_time - adc_trajectory->header().timestamp_sec();
  for (auto& p : *(adc_trajectory->mutable_trajectory_point())) {
    p.set_relative_time(p.relative_time() + dt);
  }

  planning_writer_->Write(*adc_trajectory);
}

void PlanningComponent::PublishPlanningAebResult(AebResult* const aeb_result) {
  AINFO << __func__;
#ifdef TIMEDELAY_TRACE_ENABLE
  uint64_t lastseq = aeb_result->header().sequence_num();
#endif
  common::util::FillHeader(node_->Name(), aeb_result);
  planning_aeb_writer_->Write(*aeb_result);
}

void PlanningComponent::DetectTrafficLightState(
    const ADCTrajectory& adc_trajectory) {
  perception::TrafficLightDetection traffic_light_detection;
  const auto& signal =
      adc_trajectory.debug().planning_data().signal_light().signal();
  for (const auto& item : signal) {
    auto* traffic_light = traffic_light_detection.add_traffic_light();
    traffic_light->set_id(item.light_id());
    traffic_light->set_color(item.color());
  }
  traffic_light_report_writer_->Write(traffic_light_detection);
}

void PlanningComponent::CheckRerouting() {
  auto* rerouting = injector_->planning_context()
                        ->mutable_planning_status()
                        ->mutable_rerouting();
  if (rerouting->is_new_routing()) {
    auto keep_report_times = rerouting->keep_report_new_routing_times();
    if ((++keep_report_times) < kNewRoutingReportTimes) {
      rerouting->set_keep_report_new_routing_times(keep_report_times);
    } else {
      rerouting->set_is_new_routing(false);
      rerouting->set_keep_report_new_routing_times(0);
    }
  } else {
    rerouting->set_keep_report_new_routing_times(0);
  }
  if (!rerouting->need_rerouting()) {
    return;
  }
  common::util::FillHeader(node_->Name(), rerouting->mutable_routing_request());
  rerouting->set_need_rerouting(false);
  rerouting_writer_->Write(rerouting->routing_request());
}

bool PlanningComponent::IsVehicleUseAndCollisionElectricFence(
    ADCTrajectory* const adc_trajectory_pb) {
  if (injector_ == nullptr || local_view_.localization_estimate == nullptr ||
      local_view_.chassis == nullptr) {
    AERROR << "localization_estimate or chassis is nullptr.";
    return false;
  }

  common::Status status = injector_->vehicle_state()->Update(
      *local_view_.localization_estimate, *local_view_.chassis, false);
  bool is_auto_state = false;
  if (local_view_.chassis.get()->driving_mode() ==
      canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    is_auto_state = true;
  } else {
    injector_->SetIsVehCollisionElectricFence(true);
  }
  double lat_velocity = 0.0, lon_velocity = 0.0;
  if (status.ok() &&
      util::IsVehicleStateValid(injector_->vehicle_state()->vehicle_state())) {
    aeb_planner_->CalculateLateralSpeed(
        local_view_,
        injector_->vehicle_state()->vehicle_state().linear_velocity(),
        lon_velocity, lat_velocity);
  }
  if ((FLAGS_enable_not_auto_electric_fence_drivable_area && !is_auto_state &&
       (!status.ok() ||
        !util::IsVehicleStateValid(
            injector_->vehicle_state()->vehicle_state()) ||
        util::IsVehicleCollisionWithElectricFence(
            injector_->vehicle_state()->vehicle_state(), lat_velocity,
            lon_velocity, false))) ||
      (is_auto_state && status.ok() &&
       util::IsVehicleStateValid(injector_->vehicle_state()->vehicle_state()) &&
       util::IsVehicleCollisionWithElectricFence(
           injector_->vehicle_state()->vehicle_state(), lat_velocity,
           lon_velocity, true))) {
    AERROR << "adc is not within the drivable area.";
    injector_->SetIsVehCollisionElectricFence(true);

    ADCTrajectory estop_trajectory;
    std::string msg = "adc is not within the drivable area.";
    EStop* estop = estop_trajectory.mutable_estop();
    estop->set_is_estop(true);
    estop->set_reason(msg);
    adc_trajectory_pb->CopyFrom(estop_trajectory);
    return true;
  }
  return false;
}

bool PlanningComponent::CheckInput() {
  ADCTrajectory trajectory_pb;
  auto* not_ready = trajectory_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();

  std::string input_error_reason;
  input_error_reason.clear();

  if (local_view_.localization_estimate == nullptr) {
    input_error_reason += " localization not ready;";
  }
  if (local_view_.chassis == nullptr) {
    input_error_reason += " chassis not ready;";
  }
  if (local_view_.prediction_obstacles == nullptr) {
    input_error_reason += " prediction obstacles not ready;";
  } else {
    // const double delay =
    //     Clock::NowInSeconds() -
    //     local_view_.prediction_obstacles->header().timestamp_sec();
    // if (std::abs(delay) > FLAGS_prediction_expire_time_sec) {
    //   input_error_reason += " prediction obstacles msg is expired, delay " +
    //                         std::to_string(delay) + "seconds;";
    // }
  }
  if (HDMapUtil::BaseMapPtr() == nullptr) {
    input_error_reason += " map not ready;";
  }

  if (FLAGS_use_navigation_mode) {
    if (local_view_.relative_map == nullptr ||
        !local_view_.relative_map->has_header()) {
      input_error_reason += " relative map not ready;";
    }
  } else {
    if (local_view_.routing == nullptr || !local_view_.routing->has_header()) {
      input_error_reason += " routing not ready;";
    }
  }

  if (!input_error_reason.empty()) {
    not_ready->set_reason(input_error_reason);
  }

  if (not_ready->has_reason()) {
    AERROR << not_ready->reason() << " skip the planning cycle.";
    CalcuDisplayTypeManual(&trajectory_pb);
    common::util::FillHeader(node_->Name(), &trajectory_pb);
    planning_writer_->Write(trajectory_pb);
    return false;
  }
  return true;
}

void PlanningComponent::CalVehicleInPlayStreet() {
  const auto& reference_line_info =
      injector_->frame_history()->Latest()->reference_line_info().front();
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  double check_s = reference_line_info.AdcSlBoundary().end_s();
  auto lane_type = util::GetLaneTypeAt(reference_line, check_s);
  if (hdmap::Lane::PLAY_STREET != lane_type) {
    injector_->is_in_play_street = false;
    injector_open_space_->is_in_play_street = false;
    if (!FLAGS_enable_public_road_teb) {
      traj_status_buffer_.clear();
      return;
    }
  } else {
    injector_->is_in_play_street = true;
    injector_open_space_->is_in_play_street = true;
  }
  return;
}

void PlanningComponent::CalVehicleInCommonJunction() {
  // common junction
  injector_->is_in_common_junction_ = false;
  injector_open_space_->is_in_common_junction_ = false;
  const auto& point = common::util::PointFactory::ToPointENU(
      injector_->frame_history()->Latest()->vehicle_state());
  std::vector<JunctionInfoConstPtr> junctions;
  const hdmap::HDMap* base_map_ptr = HDMapUtil::BaseMapPtr();
  if (base_map_ptr->GetJunctions(point, FLAGS_openspace_junction_search_radius,
                                 &junctions) != 0) {
    AERROR << "Fail to get junctions from map.";
    return;
  }

  if (junctions.size() <= 0) {
    injector_->is_in_common_junction_ = false;
    injector_open_space_->is_in_common_junction_ = false;
  } else {
    injector_->is_in_common_junction_ = true;
    injector_open_space_->is_in_common_junction_ = true;
  }
  return;
}

bool PlanningComponent::IsAdcInTypeJunction(
    const century::hdmap::Junction_Type& junction_type) {
  const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  common::PointENU adc_point_enu;
  adc_point_enu.set_x(injector_->vehicle_state()->vehicle_state().x());
  adc_point_enu.set_y(injector_->vehicle_state()->vehicle_state().y());
  std::vector<JunctionInfoConstPtr> junctions;
  if (0 == base_map_ptr->GetJunctions(adc_point_enu,
                                      FLAGS_openspace_junction_search_radius,
                                      &junctions)) {
    for (const auto& ptr_junction : junctions) {
      if (junction_type == ptr_junction->junction().type() &&
          IsJunctionContainAdc(injector_->vehicle_state()->vehicle_state(),
                               *ptr_junction)) {
        // in targe junction
        AINFO << "IN Targe JUNCTION";
        return true;
      }
    }
  } else {
    ADEBUG << "Fail to get junctions from base_map.";
  }
  return false;
}

void PlanningComponent::IsAdcInCommonJunction() {
  injector_->is_in_common_junction_ = false;
  injector_open_space_->is_in_common_junction_ = false;
  const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  common::PointENU adc_point_enu;
  adc_point_enu.set_x(injector_->vehicle_state()->vehicle_state().x());
  adc_point_enu.set_y(injector_->vehicle_state()->vehicle_state().y());
  std::vector<JunctionInfoConstPtr> junctions;
  if (0 == base_map_ptr->GetJunctions(adc_point_enu,
                                      FLAGS_openspace_junction_search_radius,
                                      &junctions)) {
    for (const auto& ptr_junction : junctions) {
      if (century::hdmap::Junction::COMMON_JUNCTION ==
              ptr_junction->junction().type() &&
          IsJunctionContainAdc(injector_->vehicle_state()->vehicle_state(),
                               *ptr_junction)) {
        // in commonjunction
        AINFO << "IN COMMON JUNCTION";
        injector_->is_in_common_junction_ = true;
        injector_open_space_->is_in_common_junction_ = true;
        return;
      }
    }
  } else {
    ADEBUG << "Fail to get junctions from base_map.";
  }
  return;
}

bool PlanningComponent::IsJunctionContainAdc(
    const century::common::VehicleState& vehicle_state,
    const hdmap::JunctionInfo& junction_info) {
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  // Compute the ADC bounding box.
  Vec2d ego_center_map_frame((vehicle_param.front_edge_to_center() -
                              vehicle_param.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param.left_edge_to_center() -
                              vehicle_param.right_edge_to_center()) *
                                 0.5);
  ego_center_map_frame.SelfRotate(vehicle_state.heading());
  ego_center_map_frame.set_x(vehicle_state.x() + ego_center_map_frame.x());
  ego_center_map_frame.set_y(vehicle_state.y() + ego_center_map_frame.y());
  century::common::math::Box2d adc_box(
      ego_center_map_frame, vehicle_state.heading(), vehicle_param.length(),
      vehicle_param.width());
  // Check whether Junction's polygon contain ADC bounding box.
  const auto& polygon = junction_info.polygon();
  return polygon.Contains(century::common::math::Polygon2d(adc_box));
}

void PlanningComponent::CalPrpValidTrace(ADCTrajectory* const adc_trajectory) {
  TrajStatus traj_status;
  traj_status.stamp = Clock::NowInSeconds();
  traj_status.has_traj = IsAdcTrajectoryValid(*adc_trajectory);
  traj_status_buffer_.emplace_back(traj_status);
  const double k_check_time = kCheckTrajectoryTimePublicRoad;
  while (true) {
    const auto& time_diff =
        (traj_status.stamp - traj_status_buffer_.front().stamp) * 1000;
    ADEBUG << "time_diff: " << time_diff << " ms.";
    if (time_diff < k_check_time) {
      break;
    }
    traj_status_buffer_.pop_front();
  }
  return;
}

bool PlanningComponent::IsAdcTrajectoryValid(
    const ADCTrajectory& adc_trajectory) {
  if (ErrorCode::OK != adc_trajectory.header().status().error_code()) {
    return false;
  }
  auto pick_path_label = adc_trajectory.debug()
                             .planning_data()
                             .valid_path_info()
                             .pick_path_label();
  bool use_fallback_path = util::CompareTwoStringIsEqual(
      pick_path_label, FLAGS_path_label_is_fallback);
  bool use_self_path =
      util::CompareTwoStringIsEqual(pick_path_label, FLAGS_path_label_is_self);
  bool adc_has_stop =
      (adc_trajectory.decision().main_decision().has_stop() &&
       adc_trajectory.total_path_length() < kMinTrajctoryLength);
  if (util::IsLaneBorrow(injector_->planning_context())) {
    if (use_fallback_path || use_self_path) {
      AINFO << "[Check TEB] path is not borrow in lane borrow, not valid!";
      return false;
    }
    std::string path_stop_tag = "PathDecider/nearest-stop";
    auto stop_decision = adc_trajectory.decision().main_decision().stop().tag();
    if (adc_has_stop &&
        util::CompareTwoStringIsEqual(stop_decision, path_stop_tag)) {
      AINFO << "[Check TEB] path nearest stop in lane borrow";
      return false;
    }
  } else if (adc_has_stop && injector_->is_single_lane_) {
    std::string path_stop_tag = "PathDecider/blocking_obstacle";
    auto stop_decision = adc_trajectory.decision().main_decision().stop().tag();
    if (util::CompareTwoStringIsEqual(stop_decision, path_stop_tag) ||
        use_fallback_path) {
      AINFO << "[Check TEB] path is blocked / fallback: " << use_fallback_path;
      return false;
    }
  }
  return true;
}

void PlanningComponent::CollectLaneDrivingTrajInfo(
    ADCTrajectory* const adc_trajectory) {
  if (adc_trajectory == nullptr) {
    traj_status_buffer_.clear();
    AINFO << "CollectLaneDrivingTrajInfo 11";
    return;
  }
  if (injector_->frame_history() == nullptr ||
      injector_->frame_history()->Latest() == nullptr) {
    AINFO << "CollectLaneDrivingTrajInfo 12";
    traj_status_buffer_.clear();
    return;
  }

  if (injector_->frame_history()->Latest()->reference_line_info().empty()) {
    AINFO << "CollectLaneDrivingTrajInfo 14";
    traj_status_buffer_.clear();
    return;
  }

  if (adc_trajectory->trajectory_scenario() != ADCTrajectory::LANEFOLLOW) {
    traj_status_buffer_.clear();
    AINFO << "CollectLaneDrivingTrajInfo 17";
    return;
  }

  if (FLAGS_enable_TEB_thread) {
    if (&last_use_trajectory_ == nullptr) {
      traj_status_buffer_.clear();
      AINFO << "CollectLaneDrivingTrajInfo 18";
      return;
    }
    if (injector_open_space_->frame_teb_history() == nullptr ||
        injector_open_space_->frame_teb_history()->Latest() == nullptr) {
      AINFO << "CollectLaneDrivingTrajInfo 19";
      traj_status_buffer_.clear();
      return;
    }
  }
  CalPrpValidTrace(adc_trajectory);
}

bool PlanningComponent::EnablePullover() {
  if (FLAGS_enable_use_pullover_mode && injector_->is_in_near_goal_ &&
      !injector_->frame_history()->Latest()->reference_line_info().empty()) {
    const double adc_heading = injector_->vehicle_state()->heading();
    const auto& routing_end_point = injector_->frame_history()
                                        ->Latest()
                                        ->local_view()
                                        .routing->routing_request()
                                        .waypoint()
                                        .rbegin()
                                        ->pose();
    const ReferenceLine& reference_line = injector_->frame_history()
                                              ->Latest()
                                              ->reference_line_info()
                                              .front()
                                              .reference_line();
    common::SLPoint routing_end_position_sl;
    reference_line.XYToSL(routing_end_point, &routing_end_position_sl);
    const double routing_end_heading =
        reference_line.GetReferencePoint(routing_end_position_sl.s()).heading();
    const bool near_angle = std::fabs(century::common::math::AngleDiff(
                                adc_heading, routing_end_heading)) < M_PI_2;

    common::math::Vec2d adc_init_position = {injector_->vehicle_state()->x(),
                                             injector_->vehicle_state()->y()};
    common::SLPoint adc_position_sl;
    reference_line.XYToSL(adc_init_position, &adc_position_sl);
    double current_road_left_line_l = 0.0;
    double current_road_right_line_l = 0.0;
    if (!reference_line.GetLaneWidth(adc_position_sl.s(),
                                     &current_road_left_line_l,
                                     &current_road_right_line_l)) {
      AERROR << "reference_line current GetLaneWidth failed.";
      return false;
    }
    bool near_goal = std::fabs(routing_end_position_sl.s() -
                               adc_position_sl.s()) < kDistanceToDestination;

    if (near_goal && near_angle &&
        routing_end_position_sl.l() < current_road_left_line_l) {
      injector_->pullover_using_ = true;
      return true;
    }
  }
  return false;
}

bool PlanningComponent::EnablePulloverThread() {
  if (FLAGS_enable_use_pullover_mode &&
      injector_open_space_->is_in_near_goal_ &&
      injector_open_space_->frame_teb_history()->Latest() != nullptr &&
      !injector_open_space_->frame_teb_history()
           ->Latest()
           ->reference_line_info()
           .empty()) {
    const double adc_heading = injector_open_space_->vehicle_state()->heading();
    const auto& routing_end_point = injector_open_space_->frame_teb_history()
                                        ->Latest()
                                        ->local_view()
                                        .routing->routing_request()
                                        .waypoint()
                                        .rbegin()
                                        ->pose();
    const ReferenceLine& reference_line =
        injector_open_space_->frame_teb_history()
            ->Latest()
            ->reference_line_info()
            .front()
            .reference_line();
    common::SLPoint routing_end_position_sl;
    reference_line.XYToSL(routing_end_point, &routing_end_position_sl);
    const double routing_end_heading =
        reference_line.GetReferencePoint(routing_end_position_sl.s()).heading();
    const bool near_angle = std::fabs(century::common::math::AngleDiff(
                                adc_heading, routing_end_heading)) < M_PI_2;

    common::math::Vec2d adc_init_position = {
        injector_open_space_->vehicle_state()->x(),
        injector_open_space_->vehicle_state()->y()};
    common::SLPoint adc_position_sl;
    reference_line.XYToSL(adc_init_position, &adc_position_sl);
    double current_road_left_line_l = 0.0;
    double current_road_right_line_l = 0.0;

    if (!reference_line.GetLaneWidth(adc_position_sl.s(),
                                     &current_road_left_line_l,
                                     &current_road_right_line_l)) {
      AERROR << "reference_line current GetLaneWidth failed.";
      return false;
    }
    bool near_goal = std::fabs(routing_end_position_sl.s() -
                               adc_position_sl.s()) < kDistanceToDestination;
    if (near_goal && near_angle &&
        routing_end_position_sl.l() < current_road_left_line_l) {
      injector_open_space_->pullover_using_ = true;
      AERROR << "pullover_using_test: "
             << "pullover_using_: true";
      return true;
    }
  }
  return false;
}

bool PlanningComponent::EnableRescue(ADCTrajectory* const adc_trajectory) {
  AINFO << __func__;
  // check driving mode
  if (!CheckDrivingModeEnableRescue()) {
    return false;
  }
  AINFO << "stop_reason_code: "
        << planning::StopReasonCode_Name(
               adc_trajectory->decision().main_decision().stop().reason_code());
  if (CheckIsStopByStacker(adc_trajectory)) {
    AINFO << "do not need rescue because of stacker.";
    return false;
  }

  century::hdmap::Junction::Type ingore_junction =
      century::hdmap::Junction::UNKNOWN;
  if (CheckInIgnoreJunction(ingore_junction)) {
    AINFO << "in ignore junction: ["
          << hdmap::Junction_Type_Name(ingore_junction) << "], disable rescue.";
    return false;
  }

  if (CheckIsOperationToRescueCondition(adc_trajectory)) {
    AINFO << "Operation condition, failed to call igv, need rescue.";
    return true;
  }

  // only trigger rescue when turn scene
  if (!CheckIsTurnSceneToRescue()) {
    AINFO << "No turn scene, do not need rescue.";
    return false;
  }

  injector_->pullover_using_ = false;
  if (EnablePullover()) {
    AINFO << "TEB_ODD: Enable_Rescue_PullOver";
    return true;
  }

  // check keep time
  const double k_check_time = kCheckTrajectoryTimePublicRoad;
  if (traj_status_buffer_.size() < kMinTrajSize ||
      Clock::NowInSeconds() - traj_status_buffer_.front().stamp <
          (0.5 * k_check_time / 1000)) {
    AINFO << " no use rescue. 1";
    return false;
  }

  // check adc is stopping.
  bool is_lowspeed =
      std::fabs(local_view_.chassis->speed_mps()) < KLowSpeedThreshold;
  AINFO << "is_low_speed: " << is_lowspeed;
  if (!is_lowspeed) {
    return false;
  }

  if (injector_->is_in_near_goal_ || injector_->is_reach_goal_) {
    return false;
  }

  bool special_area_stop = false;
  if (special_area_stop) {
    // TODO
  }

  if (CheckIsStopByPlanningErrorOrFailed(adc_trajectory)) {
    AINFO << " need rescue because of planning error";
    return true;
  }

  if (CheckIsStopByNormalObstacle(adc_trajectory)) {
    AINFO << " need rescue because of stop nromal obstacle";
    return true;
  }

  constexpr bool enable_check_lane_safe = false;
  if (enable_check_lane_safe && CheckRescueLaneSafe()) {
    AINFO << "TEB_ODD: Enable_Rescue_PublicRoad";
    return true;
  }
  return false;
}

bool PlanningComponent::EnableRescueThread() {
  // check driving mode
  if (!CheckDrivingModeEnableRescue()) {
    AINFO << "driving mode != COMPLETE_AUTO_DRIVE, disable rescue.";
    return false;
  }

  injector_open_space_->pullover_using_ = false;
  if (EnablePulloverThread()) {
    AINFO << "TEB_ODD: Enable_Rescue_PullOver_Thread";
    injector_open_space_->set_enable_rescue_pullover(true);
    return true;
  }

  // check keep time
  const double k_check_time = kCheckTrajectoryTimePublicRoad;
  if (traj_status_buffer_.size() < kMinTrajSize ||
      Clock::NowInSeconds() - traj_status_buffer_.front().stamp <
          (0.5 * k_check_time / 1000)) {
    AINFO << " no use rescue. 1";
    return false;
  }

  // check adc is stopping.
  bool is_lowspeed = std::fabs(local_view_.chassis->speed_mps()) < KLowSpeedThreshold;
  if (!is_lowspeed) {
    return false;
  }

  if (injector_open_space_->is_in_near_goal_ ||
      injector_open_space_->is_reach_goal_) {
    return false;
  }

  // check prp is valid
  int valid_traj_num = 0;
  for (const auto status : traj_status_buffer_) {
    valid_traj_num += status.has_traj;
  }

  bool invalid_traj_too_much = (traj_status_buffer_.size() - valid_traj_num) /
                                   traj_status_buffer_.size() >
                               kInvalidTrajectoryPercent;
  bool prp_latest_trace_is_invalid = !traj_status_buffer_.back().has_traj;
  bool is_prp_invalid =
      (invalid_traj_too_much && prp_latest_trace_is_invalid) ||
      (injector_open_space_->is_adc_out_lane_);
  if (!is_prp_invalid) {
    // return false;
  }

  constexpr bool enable_thread_check_lane_safe = false;
  if (enable_thread_check_lane_safe && CheckRescueLaneSafeThread()) {
    return true;
  }

  return false;
}

bool PlanningComponent::CheckIsStopByStacker(ADCTrajectory* adc_trajectory) {
  ADEBUG << __func__;
  if (adc_trajectory->decision().main_decision().stop().reason_code() ==
      StopReasonCode::STOP_REASON_STACKER) {
    const auto& object_decision = adc_trajectory->decision().object_decision();
    ADEBUG << "object_decision_size: " << object_decision.decision().size();
    for (auto decision : object_decision.decision()) {
      for (const auto& object_decision_type : decision.object_decision()) {
        ADEBUG << "id: " << std::to_string(decision.perception_id())
               << ", object_decision_has_stop: "
               << object_decision_type.has_stop();
        if (object_decision_type.has_stop()) {
          AINFO << "object_decision_type_stop_reason_code: "
                << object_decision_type.stop().reason_code()
                << ", stop_obstacle_id: "
                << std::to_string(decision.perception_id());
          if (object_decision_type.stop().reason_code() ==
              StopReasonCode::STOP_REASON_STACKER) {
            AINFO << "do not rescue because of stacker.";
            return true;
          }
        }
      }
    }
  }
  return false;
}

bool PlanningComponent::CheckInIgnoreJunction(
    century::hdmap::Junction::Type& ingore_junction) {
  if (!config_.rescue_config().enable_check_ignore_junction()) {
    AINFO << "ignore_junction trigger rescue is disabled in config.";
    return false;
  }

  common::PointENU adc_point_enu;
  adc_point_enu.set_x(injector_->vehicle_state()->vehicle_state().x());
  adc_point_enu.set_y(injector_->vehicle_state()->vehicle_state().y());
  const std::vector<std::pair<hdmap::Junction::Type, const char*>>
      junction_checks = {
          {hdmap::Junction::BLOCKING_AREA_J1, "is_in_J1"},
          {hdmap::Junction::BLOCKING_AREA_J2J3, "is_in_J2_J3"},
          {hdmap::Junction::J4_1, "is_in_J4_1"},
          // if need more junctions, just added here
      };
  std::string debug_msg = "";
  for (const auto& check : junction_checks) {
    bool is_in = util::IsAdcInJunction(
        adc_point_enu, injector_->vehicle_state()->vehicle_state(),
        check.first);
    if (!debug_msg.empty()) {
      debug_msg += ", ";
    }
    debug_msg += check.second + std::string(": ") + (is_in ? "true" : "false");
    // if in ignore junction, set type and return true immediately
    if (is_in) {
      AINFO << debug_msg;
      ingore_junction = check.first;
      return true;
    }
  }
  AINFO << debug_msg;
  return false;
}

// true -> turn scene; false -> normal scene, do not need rescue
bool PlanningComponent::CheckIsTurnSceneToRescue() {
  ADEBUG << __func__;
  if (!config_.rescue_config().enable_check_turn_scene()) {
    AINFO
        << "check turn scene trigger rescue is disabled in config.";
    return false;
  }

  const century::planning::Frame* frame = injector_->frame_history()->Latest();
  if (nullptr == frame) {
    AWARN << "frame history is empty, skip turn-scene rescue check.";
    return false;
  }
  if (frame->reference_line_info().empty()) {
    AERROR << "reference line info is empty, return false.";
    return false;
  }
  const auto& reference_line_info = frame->reference_line_info().front();
  double distance_to_turn_lane = std::numeric_limits<double>::max();
  if (util::GetRemainDisToTurn(
          reference_line_info, config_.rescue_config().distance_to_turn_lane(),
          config_.rescue_config().check_turn_step(),
          config_.rescue_config().turn_scene_kappa_threshold(),
          distance_to_turn_lane)) {
    AINFO << "distance_to_turn_lane: " << distance_to_turn_lane;
    if (std::fabs(distance_to_turn_lane) <
        config_.rescue_config().distance_to_turn_lane()) {
      AINFO << "Turn scene, return true.";
      return true;
    }
  }
  return false;
}

bool PlanningComponent::CheckIsOperationToRescueCondition(
    ADCTrajectory* const adc_trajectory) {
  ADEBUG << __func__;
  if (!config_.rescue_config().enable_operation_rescue()) {
    AERROR << "disable operation trigger rescue in config.";
    return false;
  }

  const century::planning::Frame* frame = injector_->frame_history()->Latest();
  if (nullptr == frame) {
    AWARN << "frame history is empty, skip operation rescue check.";
    return false;
  }
  if (nullptr == local_view_.routing ||
      !local_view_.routing->has_routing_request() ||
      local_view_.routing->routing_request().waypoint().empty()) {
    AERROR << "routing or routing_request or waypoints is empty.";
    return false;
  }
  const auto& routing_request = local_view_.routing->routing_request();
  bool is_adc_ahead_operation_point = false;
  const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                            injector_->vehicle_state()->y()};
  AINFO << "task_type: " << routing_request.task_type();

  common::PointENU adc_point_enu;
  adc_point_enu.set_x(injector_->vehicle_state()->vehicle_state().x());
  adc_point_enu.set_y(injector_->vehicle_state()->vehicle_state().y());
  bool is_in_T4_T5_T8_junction = util::IsAdcInJunction(
      adc_point_enu, injector_->vehicle_state()->vehicle_state(),
      century::hdmap::Junction::T4_T5_T8);

  bool is_operation_condition =
      (routing::YARD_OPERATIONAREA_DYNAMIC == routing_request.task_type() ||
       routing::YARD_OPERATIONAREA_STATIC == routing_request.task_type() ||
       routing::LOADING_OPERATIONAREA_SAMEDIRECTION_1 ==
           routing_request.task_type() ||
       routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_1 ==
           routing_request.task_type());

  if (!frame->reference_line_info().empty()) {
    size_t waypoint_num = routing_request.waypoint().size();
    const auto& end_waypoint = routing_request.waypoint().at(waypoint_num - 1)
                                   .pose();
    common::math::Vec2d target_point{end_waypoint.x(), end_waypoint.y()};
    ADEBUG << "adc_position, x: " << adc_position.x()
          << ", y: " << adc_position.y()
          << ", target_point, x: " << target_point.x()
          << ", y: " << target_point.y();
    is_adc_ahead_operation_point = util::CompareTwoPointsWithReference(
        frame->reference_line_info().front(), adc_position, target_point);
  }
  AINFO << "is_operation_condition: " << is_operation_condition
        << ", reference_line_size: " << frame->reference_line_info().size()
        << ", is_adc_ahead_operation_point: " << is_adc_ahead_operation_point
        << ", is_in_T4_T5_T8_junction: " << is_in_T4_T5_T8_junction
        << ", speed_mps: " << local_view_.chassis->speed_mps();
  // reference is empty && operation condition && adc is stop
  if ((frame->reference_line_info().empty() || is_adc_ahead_operation_point) &&
      is_operation_condition && !is_in_T4_T5_T8_junction &&
      local_view_.chassis->speed_mps() < 0.2) {
    adc_trajectory->clear_trajectory_point();
    // need rescue to arrive at the operation area
    AINFO << "prp disable, need rescue to arrive at the operation area.";
    injector_->set_openspace_reason(OpenspaceReason::OPERATION);
    return true;
  }
  return false;
}

bool PlanningComponent::CheckIsStopByPlanningErrorOrFailed(
    ADCTrajectory* adc_trajectory) {
  bool is_stop_by_planning_error = false;
  const auto& traj_error_status = adc_trajectory->header().status();
  if (traj_error_status.error_code() != ErrorCode::OK &&
      local_view_.chassis->speed_mps() < 0.2) {
    if (traj_error_status.msg() ==
        "ADC deviation from the direction of the road.") {
      error_rescue_stop_count_++;
      if (error_rescue_stop_count_ >=
          config_.rescue_config().planning_error_stop_count_threshold_for_rescue()) {
        injector_->set_openspace_reason(OpenspaceReason::PLANNING_ERROR);
        is_stop_by_planning_error = true;
        AINFO << "planning error, need rescue, is_stop_by_planning_error: "
              << is_stop_by_planning_error;
      }
    }
  } else if (traj_error_status.msg().find("failed optimization status:") !=
                 std::string::npos &&
             local_view_.chassis->speed_mps() < 0.2) {
    error_rescue_stop_count_++;
    if (error_rescue_stop_count_ >=
        config_.rescue_config()
            .planning_error_stop_count_threshold_for_rescue()) {
      injector_->set_openspace_reason(OpenspaceReason::PLANNING_FAILED);
      is_stop_by_planning_error = true;
      AINFO << "failed optimization status, need rescue, "
               "is_stop_by_planning_error: "
            << is_stop_by_planning_error;
    }
  } else {
    error_rescue_stop_count_ = 0;
  }
  AINFO << "error_rescue_stop_count_: " << error_rescue_stop_count_
         << ", is_stop_by_planning_error: " << is_stop_by_planning_error;
  return is_stop_by_planning_error;
}

bool PlanningComponent::CheckIsStopByNormalObstacle(
    ADCTrajectory* adc_trajectory) {
  if (nullptr == injector_->frame_history()->Latest() ||
      injector_->frame_history()->Latest()->reference_line_info().empty()) {
    AERROR << "warning, frame history or reference line is empty.";
    return false;
  }
  bool is_stop_by_obstacle = false;
  const auto& reference_line_info =
      injector_->frame_history()->Latest()->reference_line_info().front();
  if (StopReasonCode::STOP_REASON_OBSTACLE ==
      adc_trajectory->decision().main_decision().stop().reason_code()) {
    const auto& object_decision =
        adc_trajectory->decision().object_decision();
    ADEBUG << "object_decision_size: " << object_decision.decision().size();
    for (auto decision : object_decision.decision()) {
      for (const auto& object_decision_type : decision.object_decision()) {
        ADEBUG << "id: " << std::to_string(decision.perception_id())
               << ", object_decision_has_stop: "
               << object_decision_type.has_stop();
        if (object_decision_type.has_stop()) {
          ADEBUG << "object_decision_type_stop_reason_code: "
                 << object_decision_type.stop().reason_code()
                 << ", stop_obstacle_id: "
                 << std::to_string(decision.perception_id());
          util::DealWithStopObstacle(
              reference_line_info, decision, object_decision_type,
              injector_->vehicle_state()->vehicle_state(),
              config_.rescue_config().stop_count_threshold_for_rescue(),
              config_.rescue_config().reference_end_s_threshold(),
              rescue_stop_count_, is_stop_by_obstacle);
          if (is_stop_by_obstacle) {
            injector_->set_openspace_reason(OpenspaceReason::OBSTACLE_STOPPING);
          }
        }
      }
    }
  } else if (StopReasonCode::STOP_REASON_HEAD_VEHICLE ==
             adc_trajectory->decision().main_decision().stop().reason_code()) {
    ADEBUG << "need_request_borrow: "
                        << injector_->need_request_borrow();
    ADEBUG << "has_response: " << injector_->borrow_response().has_response()
           << ", borrow_response_type: "
           << injector_->borrow_response().response_type();
    if (injector_->borrow_response().has_response() &&
        century::planning::ResponseType::ACCEPT ==
            injector_->borrow_response().response_type()) {
      rescue_stop_count_++;
      if (rescue_stop_count_ >= config_.rescue_config()
                                    .accept_borrow_obstacle_count_threshold() &&
          local_view_.chassis->speed_mps() < 0.2) {
        injector_->set_openspace_reason(OpenspaceReason::OBSTACLE_STOPPING);
        is_stop_by_obstacle = true;
        if (util::IsReachReferenceLineEnd(
                reference_line_info,
                injector_->vehicle_state()->vehicle_state(),
                config_.rescue_config().reference_end_s_threshold())) {
          is_stop_by_obstacle = false;
        }
        AINFO << "need rescue because of accept borrow obstacle!!! "
                 "is_stop_by_obstacle: "
              << is_stop_by_obstacle;
      }
    } else {
      rescue_stop_count_ = 0;
    }
  } else {
    rescue_stop_count_ = 0;
  }
  AINFO << "rescue_stop_count: " << rescue_stop_count_
        << ", is_stop_by_obstacle: " << is_stop_by_obstacle;
  return is_stop_by_obstacle;
}

bool PlanningComponent::CheckRescueLaneSafe() {
  if (injector_->frame_history() == nullptr ||
      injector_->frame_history()->Latest() == nullptr ||
      injector_->frame_history()->Latest()->reference_line_info().empty()) {
    AERROR << " injector_->frame_history() == nullptr is "
           << (injector_->frame_history() == nullptr);
    return false;
  } else if (CheckIsYield()) {
    // waiting yield
    traj_status_buffer_.clear();
    return false;
  }

  const auto& reference_line_info =
      injector_->frame_history()->Latest()->reference_line_info().front();
  const auto adc_end_s = reference_line_info.AdcSlBoundary().end_s();

  if (reference_line_info.IsAdcInGateArea()) {
    AINFO << "[Check TEB] adc is in gate area, can not rescue";
    return false;
  }

  double k_dist = kDistanceToTrafficLine;
  for (const auto& overlap : reference_line_info.FirstEncounteredOverlaps()) {
    if (ReferenceLineInfo::SIGNAL == overlap.first ||
        ReferenceLineInfo::STOP_SIGN == overlap.first ||
        ReferenceLineInfo::YIELD_SIGN == overlap.first) {
      k_dist = ReferenceLineInfo::SIGNAL == overlap.first
                   ? kDistanceToTrafficLine
                   : kDistanceToOther;
      if (std::fabs(overlap.second.start_s - adc_end_s) < k_dist) {
        return false;
      }
    }
  }

  // to add city_drive condition
  // not use right lane at current
  if (FLAGS_enable_public_road_teb) {
    return IsCanPlanTebOnPublicRoad(reference_line_info);
  }
  return true;
}

bool PlanningComponent::CheckRescueLaneSafeThread() {
  if (injector_open_space_->frame_teb_history() == nullptr ||
      injector_open_space_->frame_teb_history()->Latest() == nullptr ||
      injector_open_space_->frame_teb_history()
          ->Latest()
          ->reference_line_info()
          .empty()) {
    return false;
  } else if (CheckIsYield()) {
    // waiting yield
    traj_status_buffer_.clear();

    return false;
  }

  const auto& reference_line_info = injector_open_space_->frame_teb_history()
                                        ->Latest()
                                        ->reference_line_info()
                                        .front();
  const auto adc_end_s = reference_line_info.AdcSlBoundary().end_s();

  if (reference_line_info.IsAdcInGateArea()) {
    AINFO << "[Check TEB thread] adc is in gate area, can not rescue";
    return false;
  }

  double k_dist = kDistanceToTrafficLine;
  for (const auto& overlap : reference_line_info.FirstEncounteredOverlaps()) {
    if (ReferenceLineInfo::SIGNAL == overlap.first ||
        ReferenceLineInfo::STOP_SIGN == overlap.first ||
        ReferenceLineInfo::YIELD_SIGN == overlap.first) {
      k_dist = ReferenceLineInfo::SIGNAL == overlap.first
                   ? kDistanceToTrafficLine
                   : kDistanceToOther;
      if (std::fabs(overlap.second.start_s - adc_end_s) < k_dist) {
        return false;
      }
    }
  }

  // to add city_drive condition
  // not use right lane at current
  if (FLAGS_enable_public_road_teb) {
    return IsCanPlanTebOnPublicRoad(reference_line_info);
  }

  return false;
}

bool PlanningComponent::IsCanPlanTebOnPublicRoad(
    const ReferenceLineInfo& reference_line_info) {
  auto is_turn_right = FLAGS_enable_TEB_thread
                           ? injector_open_space_->is_turn_right_
                           : injector_->is_turn_right_;
  const auto adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  if (reference_line_info.IsAdcInCommonJunction() && is_turn_right) {
    AINFO << " is_turn_right: " << is_turn_right;
    return false;
  }
  if ((reference_line_info.IsAdcInCommonJunction() && !is_turn_right)) {
    return false;
  }
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  reference_line.GetLaneFromS(adc_end_s, &lanes);
  if (lanes.empty()) {
    AINFO << "adc_end_s[" << adc_end_s << "] can't find a lane";
    return false;
  }
  const auto& lane = lanes[0];
  AINFO << "adc_end_s[" << adc_end_s << "] ,lane[" << lane->lane().id().id()
        << "]"
        << "front size: " << lane->lane().left_neighbor_forward_lane_id().size()
        << ", reverse size: "
        << lane->lane().left_neighbor_reverse_lane_id().size();
  if (lane->lane().left_neighbor_forward_lane_id().empty()) {
    AERROR << "left neighbor forward lane is empty, return false!";
    return false;
  }

  if (!lane->lane().left_neighbor_reverse_lane_id().empty()) {
    AERROR << "left neighbor reverse lane is empty, return false!";
    return false;
  }

  if (lane->lane().left_neighbor_forward_lane_id().empty() &&
      lane->lane().left_neighbor_reverse_lane_id().empty()) {
    double left, right;
    if (!reference_line.GetLaneWidth(adc_end_s, &left, &right)) {
      return false;
    }
    if (left + right < kSingleLaneWidthThr) {
      AERROR << " lane width " << left + right << " not enter rescue";
      return false;
    }
  }
  return true;
}

bool PlanningComponent::CheckIsYield() {
  const auto& reference_line_info =
      FLAGS_enable_TEB_thread
          ? injector_open_space_->frame_teb_history()
                ->Latest()
                ->reference_line_info()
                .front()
          : injector_->frame_history()->Latest()->reference_line_info().front();
  for (const auto* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (obstacle->IsVirtual() && obstacle->HasLongitudinalDecision() &&
        obstacle->LongitudinalDecision().has_stop()) {
      const auto& stop_code =
          obstacle->LongitudinalDecision().stop().reason_code();
      if (stop_code == StopReasonCode::STOP_REASON_CROSSWALK) {
        AINFO << "stop_code =STOP_REASON_CROSSWALK , no enter teb";
        return true;
      }
    }
  }
  return false;
}

bool PlanningComponent::SameRefHeadingUturnFinished() {
  // check referline existence
  if (injector_->frame_history() == nullptr ||
      injector_->frame_history()->Latest() == nullptr ||
      injector_->frame_history()->Latest()->reference_line_info().empty()) {
    AINFO << " no use.";
    return false;
  }
  // check adc deviation
  const auto& reference_line_info =
      injector_->frame_history()->Latest()->reference_line_info().front();
  const auto& adc_sl = reference_line_info.AdcSlBoundary();
  double adc_s = (adc_sl.start_s() + adc_sl.end_s()) * 0.5;
  double adc_heading = injector_->vehicle_state()->heading();
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  double ref_heading = reference_line.GetReferencePoint(adc_s).heading();
  bool near_ref_angle = std::fabs(century::common::math::AngleDiff(
                            adc_heading, ref_heading)) < kPi_6;
  if (near_ref_angle) {
    return true;
  }
  return false;
}

bool PlanningComponent::EnableDeadEnd() {
  if (!FLAGS_enable_use_deadend_mode) {
    return false;
  }
  // check in manual mode.
  if (local_view_.chassis->driving_mode() !=
      canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "local_view_.chassis->driving_mode() !=COMPLETE_AUTO_DRIVE ";
    return false;
  }
  // check adc is slowly.
  bool is_lowspeed =
      std::fabs(local_view_.chassis->speed_mps()) < KLowSpeedThreshold;

  // check adc is near destination.
  const auto& point_1 = local_view_.localization_estimate->pose();
  const auto& point_2 =
      local_view_.routing->routing_request().waypoint().rbegin()->pose();
  bool is_near_goal = false;
  const auto& distance = std::sqrt((point_1.position().x() - point_2.x()) *
                                       (point_1.position().x() - point_2.x()) +
                                   (point_1.position().y() - point_2.y()) *
                                       (point_1.position().y() - point_2.y()));
  bool near_goal_dist = distance < kDistanceToDestination;
  if (near_goal_dist) {
    is_near_goal = true;
  }

  // check referline existence
  if (injector_->frame_history() == nullptr ||
      injector_->frame_history()->Latest() == nullptr ||
      injector_->frame_history()->Latest()->reference_line_info().empty()) {
    AINFO << " no use rescue.";
    return false;
  }
  // check adc deviation
  const auto& reference_line_info =
      injector_->frame_history()->Latest()->reference_line_info().front();
  const auto& adc_sl = reference_line_info.AdcSlBoundary();
  double adc_s = (adc_sl.start_s() + adc_sl.end_s()) * 0.5;
  double adc_heading = injector_->vehicle_state()->heading();
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  double ref_heading = reference_line.GetReferencePoint(adc_s).heading();
  bool near_ref_angle = std::fabs(century::common::math::AngleDiff(
                            adc_heading, ref_heading)) < M_PI_2;
  ADEBUG << "adc_s" << adc_s;
  ADEBUG << "ref_heading " << ref_heading << "adc_heading " << adc_heading;
  injector_->is_adc_deviate_lane_direction_ = false;
  injector_open_space_->is_adc_deviate_lane_direction_ = false;
  if (!near_ref_angle) {
    AINFO << "ADC contrary from the direction of the ref.";
    injector_->is_adc_deviate_lane_direction_ = true;
    injector_open_space_->is_adc_deviate_lane_direction_ = true;
  }

  // check path dead
  hdmap::LaneInfoConstPtr car_lane;
  PointENU car_pose;
  car_pose.set_x(point_1.position().x());
  car_pose.set_y(point_1.position().y());
  double vehicle_lane_s = 0.0;
  double vehicle_lane_l = 0.0;
  hdmap_->GetNearestLane(car_pose, &car_lane, &vehicle_lane_s, &vehicle_lane_l);
  if (car_lane == nullptr) {
    return false;
  }

  // check adc near is near traffic line.
  bool near_traffic_line = false;
  double near_traffic_line_dist = kMaxDistanceToTrafficLine;
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  const auto adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  for (const auto& overlap : first_encountered_overlaps) {
    if (overlap.first == ReferenceLineInfo::SIGNAL) {
      near_traffic_line_dist = std::fabs(overlap.second.start_s - adc_end_s);
    }
  }
  if (near_traffic_line_dist < kDistanceToTrafficLine) {
    near_traffic_line = true;
  }

  if ((injector_->is_adc_deviate_lane_direction_ &&
       injector_->is_in_play_street) &&
      !is_near_goal && !near_traffic_line && is_lowspeed) {
    AINFO << "TEB_ODD: Enable_Uturn_PlayStreet";
    return true;
  } else {
    return false;
  }
}

bool PlanningComponent::CheckDrivingModeEnableRescue() {
  auto is_teb_overtime = FLAGS_enable_TEB_thread
                             ? injector_open_space_->is_teb_overtime_
                             : injector_->is_teb_overtime_;
  if (!FLAGS_enable_use_rescue_mode ||
      local_view_.chassis->driving_mode() !=
          canbus::Chassis::COMPLETE_AUTO_DRIVE ||
      is_teb_overtime) {
    AINFO << "No_Enable_Rescue "
          << " conf:" << FLAGS_enable_use_rescue_mode << " | no_auto_driving: "
          << (local_view_.chassis->driving_mode() !=
              canbus::Chassis::COMPLETE_AUTO_DRIVE)
          << " | teb_overtime: " << is_teb_overtime;
    if (local_view_.chassis->driving_mode() !=
        canbus::Chassis::COMPLETE_AUTO_DRIVE) {
      injector_->is_teb_overtime_ = false;
      injector_open_space_->is_teb_overtime_ = false;
    }
    traj_status_buffer_.clear();
    return false;
  }
  return true;
}
void PlanningComponent::CalculateTEBTrajectory(const LocalView& local_view,
                                               bool* last_TEB_thread_finished,
                                               bool* last_calculation_result) {
  ADCTrajectory ptr_trajectory_pb;

  planning_base_openspace_->RunOnce(local_view, &ptr_trajectory_pb);

  {
    std::lock_guard<std::mutex> frame_teb_lock(frame_teb_mutex_);
    // adc_trajectory_pb_thread_temp=planning_base_openspace_->GetTrajectory();
    *last_calculation_result = planning_base_openspace_->calculation_result_;
    *last_TEB_thread_finished = planning_base_openspace_->finish_status_;
    if (last_calculation_result && last_TEB_thread_finished) {
      last_teb_trajectory_.CopyFrom(ptr_trajectory_pb);
      teb_trajectory_update_time_ = century::cyber::Time::Now().ToMillisecond();
    }
  }
}
void PlanningComponent::RescueTrajectoryUpdate(
    ADCTrajectory* const adc_trajectory_pb) {
  static bool last_TEB_thread_finished = true;
  static bool last_calculation_result = false;

  if (FLAGS_enable_TEB_thread && last_TEB_thread_finished &&
      local_view_.routing->has_header() &&
      canbus::Chassis::COMPLETE_AUTO_DRIVE ==
          local_view_.chassis.get()->driving_mode() &&
      !injector_->planning_context()
           ->mutable_planning_status()
           ->mutable_destination()
           ->has_reached_station()) {
    last_TEB_thread_finished = false;
    auto task_future_ = cyber::Async(
        &PlanningComponent::CalculateTEBTrajectory, this, local_view_,
        &last_TEB_thread_finished, &last_calculation_result);
  }

  {
    std::lock_guard<std::mutex> frame_teb_lock(frame_teb_mutex_);
    if (injector_open_space_->pullover_finished) {
      last_teb_trajectory_.set_has_reached_destination(true);
      last_teb_trajectory_.set_has_reached_station(true);
    }
    CalculatesEnableuseTEBTraces(&last_calculation_result, adc_trajectory_pb);
    // bool enable_use_teb_result = FLAGS_enable_TEB_thread &&
    //                              last_teb_trajectory_.has_debug() &&
    //                              !injector_open_space_->is_in_near_goal_ &&
    //                              !injector_open_space_->is_reach_goal_ &&
    //                              injector_open_space_->need_to_rescue_thread();
    bool enable_use_teb_result =
        FLAGS_enable_TEB_thread && last_teb_trajectory_.has_debug() &&
        // injector_open_space_->need_to_rescue_thread() && !teb_result_timeout;
        injector_open_space_->need_to_rescue_thread();

    // enable_use_teb_result = true;
    AERROR << " enable_use_teb_result: " << enable_use_teb_result
           << " last_teb_trajectory_.has_debug(): "
           << last_teb_trajectory_.has_debug() << " need_to_rescue_thread: "
           << injector_open_space_->need_to_rescue_thread()
           << " last_calculation_result: " << last_calculation_result
           << " last_TEB_thread_finished: " << last_TEB_thread_finished
           << " time_error: "
           << (century::cyber::Time::Now().ToMillisecond() -
               teb_trajectory_update_time_);
    injector_->exit_from_teb_.store(enable_use_teb_result);
    injector_open_space_->exit_from_teb_.store(enable_use_teb_result);
    if (enable_use_teb_result) {
      adc_trajectory_pb->CopyFrom(last_teb_trajectory_);
    } else {
      injector_open_space_->set_enable_rescue_pullover(false);
    }
    planning_base_openspace_->last_use_teb_trajectory_ = enable_use_teb_result;
  }
  last_use_trajectory_ = *adc_trajectory_pb;
}

void PlanningComponent::CalculatesEnableuseTEBTraces(
    bool* last_calculation_result, ADCTrajectory* const adc_trajectory_pb) {
  if (!local_view_.routing->has_header() ||
      canbus::Chassis::COMPLETE_AUTO_DRIVE !=
          local_view_.chassis.get()->driving_mode()) {
    *last_calculation_result = false;
    last_teb_trajectory_.Clear();
  }

  bool teb_planning_timeout_flag =
      (century::cyber::Time::Now().ToMillisecond() -
       teb_trajectory_update_time_) > config_.max_update_timeout_threshold() &&
      (last_teb_trajectory_.has_trajectory_scenario() &&
       last_use_trajectory_.trajectory_scenario() != ADCTrajectory::LANEFOLLOW);
  last_teb_trajectory_.mutable_debug()
      ->mutable_planning_data()
      ->mutable_open_space()
      ->set_teb_planning_timeout(teb_planning_timeout_flag);
  bool adc_has_trajectory =
      (adc_trajectory_pb != nullptr &&
       !adc_trajectory_pb->decision().main_decision().has_stop() &&
       adc_trajectory_pb->total_path_length() > 0.5 &&
       adc_trajectory_pb->header().status().error_code() == ErrorCode::OK &&
       std::fabs(injector_open_space_->vehicle_state()->steering_percentage()) <
           kMaxSteeringPercentageWhenCruise) &&
      !injector_open_space_->is_in_near_goal_ &&
      !injector_open_space_->is_reach_goal_;
  AERROR << "adc_has_trajectory: " << adc_has_trajectory << " !has_stop: "
         << !adc_trajectory_pb->decision().main_decision().has_stop()
         << " total_path_length: " << adc_trajectory_pb->total_path_length()
         << "adc_trajectory_pb_error_code: "
         << adc_trajectory_pb->header().status().error_code();
  static uint32_t adc_has_trajectory_num = 0;
  adc_has_trajectory_num = adc_has_trajectory ? adc_has_trajectory_num++ : 0;
  // if (injector_open_space_->is_off_lane_depart_ && !adc_has_trajectory) {
  //   AERROR << "set_need_to_rescue_thread1";
  //   injector_open_space_->set_need_to_rescue_thread(true);
  // }

  if (adc_has_trajectory_num > kMinPRPTrajectorySize) {
    injector_open_space_->set_need_to_rescue_thread(false);
  }
  if (!*last_calculation_result && !injector_open_space_->first_into_rescue() &&
      !injector_open_space_->first_into_pullover()) {
    injector_open_space_->set_need_to_rescue_thread(false);
    AERROR << "last_calculation_result failure,can not use teb thread";
  }
}

void PlanningComponent::TakeOverRecord(const LocalView& local_view) {
  const auto& history_frame = injector_->use_thread_in_play_street()
                                  ? injector_->frame_teb_history()->Latest()
                                  : injector_->frame_history()->Latest();
  if (nullptr == history_frame) {
    return;
  }
  ADCTrajectory_RemotePub remote_status =
      history_frame->current_frame_planned_trajectory().remote_status();
  if (local_view.chassis->driving_mode() !=
          canbus::Chassis::COMPLETE_AUTO_DRIVE &&
      remote_status.is_enabled()) {
    ADCTrajectory_RemoteScenario remote_scenario =
        remote_status.remote_scenario();
    if (history_frame->open_space_info().is_on_open_space_trajectory()) {
      // or use history_frame->fault_report_
      switch (remote_scenario) {
        case ADCTrajectory::REMOTE_PUB_TEB_STOP_LONG:
          AERROR << "ADC_TakeOver: OpenSpace STOP_LONG_TakeOver";
          break;
        case ADCTrajectory::REMOTE_PUB_TEB_MORE_FAILED:
          AERROR << "ADC_TakeOver: OpenSpace MORE_FAILED_TakeOver";
          break;
        case ADCTrajectory::REMOTE_PUB_UTURN:
          AERROR << "ADC_TakeOver: OpenSpace UTURN_TakeOver";
          break;
        default:
          AERROR << "ADC_TakeOver: OpenSpace UnExpected_TakeOver";
          break;
      }
    } else {
      switch (remote_scenario) {
        case ADCTrajectory::REMOTE_PUB_TRAFFIC_LIGHT_UNKNOWN:
          AERROR << "ADC_TakeOver: Public TRAFFIC_LIGHT_TakeOver";
          break;
        case ADCTrajectory::REMOTE_PUB_AVOID_OBS:
          AERROR << "ADC_TakeOver: Public AVOID_OBS_TakeOver";
          break;
        case ADCTrajectory::REMOTE_PUB_PARK_OUT:
          AERROR << "ADC_TakeOver: Public PARK_OUT_TakeOver";
          break;
        case ADCTrajectory::REMOTE_PUB_PARK_IN:
          AERROR << "ADC_TakeOver: Public PARK_IN_TakeOver";
          break;
        case ADCTrajectory::REMOTE_PUB_STOP_LONG:
          AERROR << "ADC_TakeOver: Public STOP_LONG_TakeOver";
          break;
        default:
          AERROR << "ADC_TakeOver: Public UnExpected_TakeOver";
          break;
      }
    }
  }
}

void PlanningComponent::CalcuDisplayTypeManual(
    ADCTrajectory* ptr_trajectory_pb) {
  if (nullptr == local_view_.chassis) {
    AINFO << "chassis is nullptr";
    return;
  }

  // set manual display calculation
  auto vehicle_gear = injector_->vehicle_state()->vehicle_state().gear();
  if (canbus::Chassis::COMPLETE_MANUAL ==
          local_view_.chassis.get()->driving_mode() ||
      canbus::Chassis::COMPLETE_REMOTE ==
          local_view_.chassis.get()->driving_mode()) {
    ptr_trajectory_pb->set_display_type(DEFAULT_DIS);
    double wheel_angle =
        (local_view_.chassis.get()->bridge_1_left_wheel_angle() +
         local_view_.chassis.get()->bridge_1_right_wheel_angle()) /
        2;
    if (wheel_angle > kTurnWheelAngleThres) {
      ptr_trajectory_pb->set_display_type(TURN_LEFT);
      if (canbus::Chassis::GEAR_REVERSE ==
          local_view_.chassis.get()->gear_location()) {
        ptr_trajectory_pb->set_display_type(TURN_RIGHT);
      }
    } else if (wheel_angle < -kTurnWheelAngleThres) {
      ptr_trajectory_pb->set_display_type(TURN_RIGHT);
      if (canbus::Chassis::GEAR_REVERSE ==
          local_view_.chassis.get()->gear_location()) {
        ptr_trajectory_pb->set_display_type(TURN_LEFT);
      }
    }
    if (local_view_.chassis.get()->front_drive_wheel_speed() < 0.01 &&
        canbus::Chassis::GEAR_PARKING == vehicle_gear) {
      ptr_trajectory_pb->set_display_type(VEHICLE_STOPED);
    }
  }

  if (nullptr == local_view_.routing) {
    AINFO << "routing is nullptr";
    return;
  }

  // set wait load/unload display
  static bool wait_laod_unload_flage_ = false;
  if (routing::RAILWAY_OPERATIONAREA_DYNAMIC ==
          local_view_.routing->routing_request().task_type() ||
      routing::YARD_OPERATIONAREA_DYNAMIC ==
          local_view_.routing->routing_request().task_type()) {
    if (injector_->planning_context()
            ->mutable_planning_status()
            ->mutable_destination()
            ->has_reached_destination()) {
      wait_laod_unload_flage_ = true;
    }
  } else {
    wait_laod_unload_flage_ = false;
  }

  if (wait_laod_unload_flage_) {
    if (local_view_.routing->routing_request().is_loading()) {
      ptr_trajectory_pb->set_display_type(WAITING_UNLOAD);
    } else {
      ptr_trajectory_pb->set_display_type(WAITING_LOAD);
    }
  }

  // set background music play
  if (nullptr == local_view_.background_music ||
      local_view_.background_music->background_music_switch()) {
    ptr_trajectory_pb->set_background_music_enable(true);
    if (canbus::Chassis::COMPLETE_AUTO_DRIVE ==
            local_view_.chassis->driving_mode() &&
        DEFAULT_DIS == ptr_trajectory_pb->display_type()) {
      if (injector_->planning_context()
              ->mutable_planning_status()
              ->mutable_destination()
              ->has_reached_destination()) {
        ptr_trajectory_pb->set_display_type(DEFAULT_DIS);
      } else {
        GenerateBackgroundMusic(ptr_trajectory_pb);
      }
    }
  } else {
    ptr_trajectory_pb->set_background_music_enable(false);
    if (BACKGROUND_MUSIC == ptr_trajectory_pb->display_type()) {
      ptr_trajectory_pb->set_display_type(DEFAULT_DIS);
    }
  }
}

void PlanningComponent::GenerateBackgroundMusic(ADCTrajectory* const ptr_trajectory_pb) {
  static DisplayType background_type = DEFAULT_DIS;
  if (DEFAULT_DIS == background_type) {
    std::srand(static_cast<unsigned int>(std::time(0)));
    int randomNum = 100 + (std::rand() % 10);
    background_type = static_cast<DisplayType>(randomNum);
    ptr_trajectory_pb->set_display_type(background_type);
  } else {
    ptr_trajectory_pb->set_display_type(background_type);
  }
}

}  // namespace planning
}  // namespace century
