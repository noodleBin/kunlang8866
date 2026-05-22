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

#include "modules/planning/scenarios/rescue_teb/stage_teb.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "cyber/common/log.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/scenarios/util/util.h"
namespace century {
namespace planning {
namespace scenario {
namespace rescue {

namespace {
constexpr int kCheckTime = 3;
constexpr int kRescueCountDecrease = 2;
constexpr int kSelectBackCount = 15;
constexpr double kStopSpeed = 0.051;
constexpr double kReachDist = 0.6;
constexpr double kDistanceToDestination = 10.0;
constexpr double kDistanceNearEnd = 5.0;
constexpr double kAdcDistanceSidePassThreshold = 10.0;
constexpr double kDynamicSpeed = 2.0;
constexpr double kExitLThreshold = 1.5;
constexpr double kRoadWidthBuffer = 0.3;
constexpr int kRescueStopMinCheckTime = 1000;
constexpr double kVehicleStopPercentageThr = 0.9;
constexpr double kPlayStreetRoadWidthBuffer = 0.1;
constexpr double kArriveEndThr = 0.25;
constexpr double kRoutingEndLThr = 2.0;
}  // namespace

using century::common::TrajectoryPoint;
using century::cyber::Clock;

RescueStageTeb::RescueStageTeb(
    const ScenarioConfig::StageConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Stage(config, injector) {
  AERROR << "----------RescueStageTeb: RescueStageTeb----------------";
  ResetInitPostion();
  rescue_status_ = injector_->planning_context()
                       ->mutable_planning_status()
                       ->mutable_rescue();

  rescue_status_->mutable_adc_init_position()->set_x(
      injector_->vehicle_state()->x());
  rescue_status_->mutable_adc_init_position()->set_y(
      injector_->vehicle_state()->y());
  rescue_status_->mutable_adc_init_position()->set_z(0.0);
  rescue_status_->set_adc_init_heading(injector_->vehicle_state()->heading());

  auto* teb_common = injector_->planning_context()
                         ->mutable_planning_status()
                         ->mutable_teb_common();
  teb_common->Clear();
  AINFO << "injector_->vehicle_state()->x()" << injector_->vehicle_state()->x();
  teb_common->mutable_adc_init_position()->set_x(
      injector_->vehicle_state()->x());
  teb_common->mutable_adc_init_position()->set_y(
      injector_->vehicle_state()->y());
  teb_common->mutable_adc_init_position()->set_z(0.0);
  teb_common->set_adc_init_heading(injector_->vehicle_state()->heading());
  openspace_success_time_ = Clock::NowInSeconds();
}

int RescueStageTeb::GetFallBackCount() {
  auto* previous_frame = injector_->use_thread_in_play_street()
                             ? injector_->frame_teb_history()->Latest()
                             : injector_->frame_history()->Latest();
  if (nullptr == previous_frame) {
    return 0;
  }

  bool is_first_plan_success =
      previous_frame->open_space_info().open_space_provider_success();

  const auto adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();
  AINFO << "-RescueStageTeb-GetFallBackCount--count " << count_
        << " ,fallback_flag "
        << previous_frame->open_space_info().fallback_flag() << " ,adc_speed "
        << adc_speed;
  if (std::fabs(adc_speed) < kStopSpeed && is_first_plan_success) {
    ++count_;
  } else {
    count_ = 0;
  }

  return count_;
}

bool RescueStageTeb::CheckReferenceLineBlock() {
  const auto& reference_line_info = frame_->reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();
  const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary();

  const double adc_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width();

  auto* path_decision = &reference_line_info.path_decision();

  for (const auto* obstacle : path_decision->obstacles().Items()) {
    if (obstacle == nullptr) {
      continue;
    }

    const auto& perception_obstacle = obstacle->Perception();
    const auto& obs_id = perception_obstacle.id();

    const auto& obs_sl = obstacle->PerceptionSLBoundary();
    const double obs_speed = obstacle->speed();
    ADEBUG << "obs_speed " << obs_speed << "obs_sl " << obs_sl.start_s()
           << "obs_id " << obs_id;
  }

  for (const auto& obstacle : frame_->GetObstacleList()->Items()) {
    ADEBUG << "start_s" << obstacle->PerceptionSLBoundary().start_s();
    // Obstacle is virtual.
    if (obstacle->IsVirtual() || !obstacle->IsStatic() ||
        obstacle->speed() > kDynamicSpeed) {
      continue;
    }

    // Obstacle is behind ADC.
    if (obstacle->PerceptionSLBoundary().start_s() <= adc_sl_boundary.end_s()) {
      continue;
    }

    // Obstacle is far away.
    if (obstacle->PerceptionSLBoundary().start_s() >
        adc_sl_boundary.end_s() + kAdcDistanceSidePassThreshold) {
      continue;
    }
    const double driving_width =
        reference_line.GetDrivingWidth(obstacle->PerceptionSLBoundary());

    if (driving_width < adc_width + 2 * FLAGS_static_obstacle_nudge_l_buffer) {
      ADEBUG << "It is blocking our path.";
      return true;
    }
  }
  return false;
}

bool RescueStageTeb::FirstInProcessLogic() {
  const auto& history_frame = injector_->use_thread_in_play_street()
                                  ? injector_->frame_teb_history()->Latest()
                                  : injector_->frame_history()->Latest();
  if (history_frame) {
    if (!history_frame->open_space_info().is_on_open_space_trajectory()) {
      openspace_success_time_ = Clock::NowInSeconds();
      injector_->is_teb_overtime_ = false;
      if (injector_->is_in_near_goal_) {
        AINFO << "first enter pullover scenario, stop first";
        return true;
      }
      return false;
    }
    return false;
  }
  return false;
}

bool RescueStageTeb::InGateOrReachGoalOrOverTime() {
  injector_->is_teb_overtime_ =
      Clock::NowInSeconds() - openspace_success_time_ > FLAGS_rescue_max_time;
  const auto& reference_line_info = frame_->reference_line_info().front();
  bool reach_goal = CheckReachGoal();
  if (reference_line_info.IsAdcInGateArea() || reach_goal ||
      injector_->is_teb_overtime_) {
    AINFO << "IsAdcInGeteArea " << reference_line_info.IsAdcInGateArea()
          << " | Reach_Goal " << reach_goal << " | OverTime "
          << injector_->is_teb_overtime_;

    next_stage_ = ScenarioConfig::NO_STAGE;
    injector_->set_need_to_rescue(false);
    injector_->set_need_to_rescue_thread(false);
    openspace_success_time_ = Clock::NowInSeconds();
    return true;
  }
  return false;
}

bool RescueStageTeb::PullOverRunning() {
  if (injector_->pullover_using_ && injector_->is_in_near_goal_ &&
      !injector_->pullover_finished) {
    AINFO << "PullOver_Running";
    return true;
  }
  return false;
}

bool RescueStageTeb::Ready2Cruise() {
  const auto* common_status = injector_->planning_context()
                                  ->mutable_planning_status()
                                  ->mutable_teb_common();
  double pass_s = std::sqrt((common_status->adc_init_position().x() -
                             injector_->vehicle_state()->x()) *
                                (common_status->adc_init_position().x() -
                                 injector_->vehicle_state()->x()) +
                            (common_status->adc_init_position().y() -
                             injector_->vehicle_state()->y()) *
                                (common_status->adc_init_position().y() -
                                 injector_->vehicle_state()->y()));

  bool is_pullover_ready = injector_->is_in_near_goal_ &&
                           injector_->pullover_using_ &&
                           FLAGS_enable_use_pullover_mode;
  if (GetFallBackCount() > kSelectBackCount && !is_pullover_ready) {
    ResetInitPostion();
  }

  const bool is_ready_to_cruise =
      injector_->is_in_play_street
          ? scenario::util::CheckADCReadyToCruise(
                injector_->vehicle_state(), frame_,
                scenario_config_.front_obstacle_buffer(),
                scenario_config_.heading_buffer())
          : scenario::util::CheckRefReadyToCruise(
                injector_->vehicle_state(), frame_, rescue_status_,
                scenario_config_.front_obstacle_buffer_city(),
                scenario_config_.heading_buffer());
  ready_count_ = is_ready_to_cruise
                     ? ready_count_ + 1
                     : std::max(ready_count_ - kRescueCountDecrease, 0);

  AINFO << "ready_to_cruise" << is_ready_to_cruise << " ready_count"
        << ready_count_ << " pass_s" << pass_s << " adc_in_lane"
        << !injector_->teb_adc_is_out_lane_ << " destination_reached()"
        << frame_->mutable_open_space_info()->destination_reached();
  if (ready_count_ >= kCheckTime && (pass_s > FLAGS_rescue_min_dist) &&
      !injector_->teb_adc_is_out_lane_) {
    if (!FLAGS_enable_rescue_replan_reason2 && !is_pullover_ready) {
      // ResetInitPostion();
      return true;
    }
    return false;
  }
  return false;
}

Stage::StageStatus RescueStageTeb::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  CHECK_NOTNULL(frame);
  frame_ = frame;
  injector_->set_use_thread_in_play_street(FLAGS_enable_TEB_thread &&
                                           injector_->is_in_play_street);
  if (!injector_->use_thread_in_play_street()) {
    scenario_config_.CopyFrom(GetContext()->scenario_config);
  }

  if (injector_->planning_context()
          ->planning_status()
          .destination()
          .arrived_station_immediately() ) {
    next_stage_ = ScenarioConfig::NO_STAGE;
    injector_->set_need_to_rescue(false);
    return StageStatus::FINISHED;
  }
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);

  if (StopLongTimeReportAndExit(frame)) {
    if (injector_->is_in_near_goal_) {
      injector_->is_reach_goal_ = true;
      injector_->pullover_finished = true;
      next_stage_ = ScenarioConfig::NO_STAGE;
      injector_->set_need_to_rescue(false);
    }
    return StageStatus::FINISHED;
  }
  if (injector_->use_thread_in_play_street()) {
    CalculateFirstIntoTEB();
  }
  if (injector_->is_off_lane_depart_ && !injector_->is_in_play_street &&
      !FLAGS_enable_public_road_teb && FLAGS_enable_TEB_thread) {
    injector_->is_off_lane_depart_ = false;
  }

  injector_->teb_adc_is_out_lane_ = CalVehicleIsOutRoad(frame);

  if (FirstInProcessLogic()) {
    return StageStatus::ERROR;
  }
  if (injector_->first_into_rescue()) {
    openspace_success_time_ = Clock::NowInSeconds();
    injector_->is_teb_overtime_ = false;
  }
  if (InGateOrReachGoalOrOverTime()) {
    return Stage::FINISHED;
  }

  bool task_failed = false;
  if (!ExecuteTaskOnOpenSpace(frame)) {
    task_failed = true;
  }
  if (TaskFailedReportAndExit(task_failed, frame)) {
    return StageStatus::FINISHED;
  }

  if (PullOverRunning()) {
    return StageStatus::RUNNING;
  }

  if (Ready2Cruise()) {
    return FinishStage();
  }

  if (frame_->mutable_open_space_info()->destination_reached()) {
    injector_->use_teb_default_bound_ = false;
    return FinishStage();
  }

  return StageStatus::RUNNING;
}

Stage::StageStatus RescueStageTeb::FinishStage() {
  const auto vehicle_status = injector_->vehicle_state();

  if (rescue_status_->is_select_back_pose() && injector_->is_in_play_street ||
      FLAGS_enable_rescue_replan_reason2 ||
      (frame_->mutable_open_space_info()->destination_reached() &&
       is_back_traj_) ||
      injector_->teb_adc_is_out_lane_) {
    if (rescue_status_->is_select_back_pose()) {
      ResetInitPostion();
    }

    AINFO << "RescueStageTeb FinishStage_RUNNING: more_case, "
             "select_back_pose: "
          << rescue_status_->is_select_back_pose()
          << " | in_play_street: " << injector_->is_in_play_street
          << " | destination_reached"
          << frame_->mutable_open_space_info()->destination_reached()
          << " | back_traj_: " << is_back_traj_
          << " | adc_out_lane_: " << injector_->teb_adc_is_out_lane_;
    return StageStatus::RUNNING;
  }

  if (!injector_->is_in_play_street) {
    const auto& reference_line_info = frame_->reference_line_info().front();
    double curr_road_left_width = 0, curr_road_right_width = 0;
    reference_line_info.GetRoadWidthBasedAdc(&curr_road_left_width,
                                             &curr_road_right_width);
    const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary();
    if (adc_sl_boundary.start_l() < -curr_road_right_width + kRoadWidthBuffer ||
        adc_sl_boundary.end_l() > curr_road_left_width - kRoadWidthBuffer) {
      AINFO << "RescueStageTeb FinishStage_RUNNING: adc_out_lane, "
               "adc_sl_boundary.start_l(): "
            << adc_sl_boundary.start_l()
            << " adc_sl_boundary.end_l(): " << adc_sl_boundary.end_l();
      return StageStatus::RUNNING;
    }
  }

  if (std::fabs(vehicle_status->steering_percentage()) <
      scenario_config_.max_steering_percentage_when_cruise()) {
    // directly lane follow
    next_stage_ = ScenarioConfig::NO_STAGE;
    injector_->set_need_to_rescue(false);
    injector_->set_need_to_rescue_thread(false);
    AERROR << " directly lane follow ";
  } else {
    AINFO << "RescueStageTeb FinishStage_RUNNING: steering_too_large, "
             "steering_percentage: "
          << vehicle_status->steering_percentage();
    return StageStatus::RUNNING;
  }
  injector_->use_teb_default_bound_ = false;
  ResetInitPostion();
  AERROR << " FinishStage: directly lane follow ";
  return Stage::FINISHED;
}

void RescueStageTeb::ResetInitPostion() {
  auto* rescue_status = injector_->planning_context()
                            ->mutable_planning_status()
                            ->mutable_rescue();
  rescue_status->Clear();
  century::planning::RescueStatus pb_rescue_status;
  if (!century::cyber::common::GetProtoFromFile(FLAGS_rescue_status_config_file,
                                              &pb_rescue_status)) {
    AERROR << "----Failed to load txt rescue status from "
           << FLAGS_rescue_status_config_file;
  } else {
    ADEBUG << "----Loaded txt rescue status from "
           << FLAGS_rescue_status_config_file;
  }
  rescue_status->CopyFrom(pb_rescue_status);
  count_ = 0;
  ready_count_ = 0;
  AINFO << "----ResetInitPostion----";
}

bool RescueStageTeb::CheckReachGoal() {
  injector_->is_reach_goal_ = false;
  injector_->pullover_finished = false;

  const auto& end_point = frame_->local_view()
                              .routing->routing_request()
                              .waypoint()
                              .rbegin()
                              ->pose();
  double square = (injector_->vehicle_state()->x() - end_point.x()) *
                      (injector_->vehicle_state()->x() - end_point.x()) +
                  (injector_->vehicle_state()->y() - end_point.y()) *
                      (injector_->vehicle_state()->y() - end_point.y());
  const auto& distance = std::sqrt(square);
  if (distance > kDistanceToDestination) {
    return false;
  }

  // temp close origin exit function
  // if (!FLAGS_enable_use_pullover_mode) {
  //   return false;
  // }

  const auto& reference_line_info = frame_->reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();

  common::math::Vec2d adc_init_position = {injector_->vehicle_state()->x(),
                                           injector_->vehicle_state()->y()};
  common::SLPoint adc_sl;
  reference_line.XYToSL(adc_init_position, &adc_sl);

  common::math::Vec2d routing_end_point = {end_point.x(), end_point.y()};
  common::SLPoint routing_end_point_sl;
  reference_line.XYToSL(routing_end_point, &routing_end_point_sl);

  const auto& history_frame = injector_->use_thread_in_play_street()
                                  ? injector_->frame_teb_history()->Latest()
                                  : injector_->frame_history()->Latest();
  bool arrive_routing_end = false;
  if (injector_->pullover_end_trace_ && history_frame->open_space_info()
                                                .chosen_partitioned_trajectory()
                                                .first.size() > 0) {
    if (std::fabs(history_frame->open_space_info()
                      .chosen_partitioned_trajectory()
                      .first.back()
                      .path_point()
                      .s()) < kArriveEndThr) {
      arrive_routing_end = true;
    }
  }

  if (injector_->is_in_near_goal_ && injector_->pullover_end_trace_ &&
      history_frame->open_space_info()
              .chosen_partitioned_trajectory()
              .first.size() > 0) {
    AINFO << "distance: " << distance << " reached: "
          << history_frame->open_space_info().destination_reached()
          << " arrive_end_s: "
          << std::fabs(history_frame->open_space_info()
                           .chosen_partitioned_trajectory()
                           .first.back()
                           .path_point()
                           .s())
          << " stop_speed: "
          << injector_->vehicle_state()->vehicle_state().linear_velocity();
  }

  bool exceed_routing_end = false;
  if ((adc_sl.s() - kDistanceNearEnd > routing_end_point_sl.s()) &&
      std::fabs(routing_end_point_sl.l()) <= kRoutingEndLThr) {
    exceed_routing_end = true;
  }

  if (history_frame) {
    AERROR << "distance: " << distance << " destination_reached: "
           << history_frame->open_space_info().destination_reached()
           << " arrive_routing_end: " << arrive_routing_end
           << " linear_velocity: "
           << std::fabs(
                  injector_->vehicle_state()->vehicle_state().linear_velocity())
           << " exceed_routing_end: " << exceed_routing_end;
    if (distance < kDistanceNearEnd &&
            (history_frame->open_space_info().destination_reached() &&
             arrive_routing_end) &&
            (std::fabs(injector_->vehicle_state()
                           ->vehicle_state()
                           .linear_velocity()) < kStopSpeed) ||
        exceed_routing_end) {
      injector_->is_reach_goal_ = true;
      injector_->pullover_finished = true;
      AINFO << "Near Routing End Point, Exit!";
      injector_->set_need_to_rescue(false);
      injector_->set_need_to_rescue_thread(false);
      next_stage_ = ScenarioConfig::NO_STAGE;
      return true;
    }
  }

  // if (distance < kDistanceToDestination) {
  //   if ((!FLAGS_enable_use_pullover_mode && distance < kDistanceNearEnd) ||
  //       (distance < kDistanceNearEnd &&
  //        frame_->mutable_open_space_info()->destination_reached()) ||
  //       ((std::fabs(
  //           injector_->vehicle_state()->vehicle_state().linear_velocity()) <
  //             kStopSpeed &&
  //         distance < kReachDist))) {
  //     injector_->is_reach_goal_ = true;
  //     AINFO << " return rescue mode "
  //           << "destination->has_reached_destination() "
  //           << frame_->mutable_open_space_info()->destination_reached();
  //     injector_->set_need_to_rescue(false);
  //     next_stage_ = ScenarioConfig::NO_STAGE;
  //     return true;
  //   }
  // }
  return false;
}

bool RescueStageTeb::CheckReachTrajectoryEnd(
    const common::TrajectoryPoint& planning_init_point) {
  const auto& path_point = planning_init_point.path_point();
  double dist =
      std::sqrt((path_point.x() - injector_->vehicle_state()->x()) *
                    (path_point.x() - injector_->vehicle_state()->x()) +
                (path_point.y() - injector_->vehicle_state()->y()) *
                    (path_point.y() - injector_->vehicle_state()->y()));
  double adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();
  AINFO << "---dist " << dist << " --- adc_speed " << adc_speed;
  return (std::fabs(adc_speed) < kStopSpeed && dist < kReachDist);
}

void RescueStageTeb::GenerateStopTrajectory() {
  TrajGearPair fallback_trajectory_pair_candidate =
      frame_->open_space_info().fallback_trajectory();
  fallback_trajectory_pair_candidate.first.clear();

  double relative_time = 0.0;
  // TODO(Jinyun) Move to conf-
  static constexpr int stop_trajectory_length = 10;
  static constexpr double relative_stop_time = 0.1;
  static constexpr double vEpsilon = 0.00001;
  double standstill_acceleration =
      frame_->vehicle_state().linear_velocity() >= -vEpsilon
          ? -FLAGS_open_space_standstill_acceleration
          : FLAGS_open_space_standstill_acceleration;
  // trajectory_data->clear();
  for (size_t i = 0; i < stop_trajectory_length; ++i) {
    TrajectoryPoint point;
    point.mutable_path_point()->set_x(frame_->vehicle_state().x());
    point.mutable_path_point()->set_y(frame_->vehicle_state().y());
    point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(0.0);
    point.set_relative_time(relative_time);
    point.set_v(0.0);
    point.set_a(standstill_acceleration);
    fallback_trajectory_pair_candidate.first.AppendTrajectoryPoint(point);
    relative_time += relative_stop_time;
  }

  auto& trajectory = frame_->open_space_info().fallback_trajectory().first;
  auto& gear = frame_->open_space_info().fallback_trajectory().second;
  PublishableTrajectory publishable_trajectory(Clock::NowInSeconds(),
                                               trajectory);
  auto publishable_traj_and_gear =
      std::make_pair(std::move(publishable_trajectory), gear);

  *(frame_->mutable_open_space_info()->mutable_publishable_trajectory_data()) =
      std::move(publishable_traj_and_gear);
}

bool RescueStageTeb::CheckVehicleLongStop(Frame* frame) {
  CheckData check_data;
  check_data.stamp = Clock::NowInSeconds();
  check_data.has_speed = frame->vehicle_state().linear_velocity() >
                         FLAGS_rescue_vehicle_stop_threshold;
  vehicle_speed_deque_.push_back(check_data);
  const double k_check_time = injector_->is_in_play_street
                                  ? FLAGS_rescue_stop_check_time
                                  : FLAGS_rescue_stop_check_time * 2;
  while (true) {
    const auto& time_diff =
        (check_data.stamp - vehicle_speed_deque_.front().stamp) * 1000;
    ADEBUG << "time_diff" << time_diff;
    if (time_diff < k_check_time) {
      break;
    }
    vehicle_speed_deque_.pop_front();
  }

  bool long_time_stop = false;
  if ((check_data.stamp - vehicle_speed_deque_.front().stamp) * 1000 >
          std::max(FLAGS_rescue_stop_check_time - kRescueStopMinCheckTime,
                   kRescueStopMinCheckTime) &&
      vehicle_speed_deque_.size() > 2) {
    int has_speed_count = 0;
    for (size_t i = 0; i < vehicle_speed_deque_.size(); ++i) {
      has_speed_count += vehicle_speed_deque_[i].has_speed ? 1 : 0;
    }

    long_time_stop = ((vehicle_speed_deque_.size() - has_speed_count) /
                      vehicle_speed_deque_.size()) > kVehicleStopPercentageThr;
  }
  return long_time_stop;
}

bool RescueStageTeb::StopLongTimeReportAndExit(Frame* frame) {
  bool long_time_stop = CheckVehicleLongStop(frame);
  if (long_time_stop) {
    frame->fault_report_ = century::planning::Frame::FaultReport::TEB_STOP_LONG;
    AERROR << "Teb_Report: Vehicle is long time stop.";
    vehicle_speed_deque_.clear();
    vehicle_speed_deque_.shrink_to_fit();
    return true;
  } else {
    frame->fault_report_ = century::planning::Frame::FaultReport::TEB_NORMAL;
    return false;
  }
  return false;
}

bool RescueStageTeb::TaskFailedReportAndExit(const bool& task_failed,
                                             Frame* frame) {
  if (task_failed) {
    AERROR << "RescueStageTeb failed";
    error_cnt_++;
    if (error_cnt_ >= std::numeric_limits<int>::max() - 1) {
      error_cnt_ = 0;
    } else if (error_cnt_ > FLAGS_rescue_failed_report_threshold) {
      frame->fault_report_ =
          century::planning::Frame::FaultReport::TEB_MORE_FAILED;
      AERROR << "Teb_Report: Planning Failed too more.";
    } else if (error_cnt_ > FLAGS_rescue_warring_report_threshold) {
      frame->fault_report_ = century::planning::Frame::FaultReport::TEB_WARRING;
      AERROR << "Teb_Report: Planning Warring too more.";
    } else {
      frame->fault_report_ = century::planning::Frame::FaultReport::TEB_NORMAL;
    }
  } else {
    error_cnt_ = 0;
    frame->fault_report_ = century::planning::Frame::FaultReport::TEB_NORMAL;
  }

  if (frame->fault_report_ ==
      century::planning::Frame::FaultReport::TEB_MORE_FAILED) {
    error_cnt_ = 0;
    return true;
  }
  return false;
}

bool RescueStageTeb::CalVehicleIsOutRoad(Frame* frame) {
  const auto& reference_line_info = frame->reference_line_info().front();
  const auto& adc_sl = reference_line_info.AdcSlBoundary();
  double curr_road_left_width = 0, curr_road_right_width = 0;
  reference_line_info.GetRoadWidthBasedAdc(&curr_road_left_width,
                                           &curr_road_right_width);
  if (adc_sl.start_l() < -curr_road_right_width + kPlayStreetRoadWidthBuffer ||
      adc_sl.end_l() > curr_road_left_width - kPlayStreetRoadWidthBuffer) {
    AINFO << "ADC is outside of the road.";
    return true;
  }
  return false;
}

void RescueStageTeb::ClearDataThread() {
  vehicle_speed_deque_.clear();
  return;
}

void RescueStageTeb::CalculateFirstIntoTEB() {
  static bool last_need_to_pullover_thread = false;
  bool first_enable_pullover =
      (injector_->enable_rescue_pullover() && !last_need_to_pullover_thread);
  last_need_to_pullover_thread = injector_->enable_rescue_pullover();
  injector_->set_first_into_pullover(first_enable_pullover);

  static bool last_need_to_rescue_thread = false;
  bool first_enable_rescue =
      (injector_->exit_from_teb_.load() && !last_need_to_rescue_thread);
  static uint32_t use_prp_trac_count = 0;
  if (injector_->is_off_lane_depart_) {
    if (!injector_->exit_from_teb_.load()) {
      use_prp_trac_count++;
    }
    injector_->is_off_lane_depart_ =
        use_prp_trac_count >= 3 ? false : injector_->is_off_lane_depart_;

  } else {
    use_prp_trac_count = 0;
  }
  last_need_to_rescue_thread = injector_->exit_from_teb_.load();
  injector_->set_first_into_rescue(first_enable_rescue);
}
}  // namespace rescue
}  // namespace scenario
}  // namespace planning
}  // namespace century
