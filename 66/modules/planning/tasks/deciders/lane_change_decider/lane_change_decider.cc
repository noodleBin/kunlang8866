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

#include "modules/planning/tasks/deciders/lane_change_decider/lane_change_decider.h"

#include <limits>
#include <memory>

#include "cyber/time/clock.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"

namespace century {
namespace planning {

using century::common::ErrorCode;
using century::common::SLPoint;
using century::common::Status;
using century::cyber::Clock;

namespace {

// TODO(All): add comments
constexpr double kSafeTimeOnSameDirection = 3.0;
constexpr double kSafeTimeOnOppositeDirection = 5.0;
constexpr double kForwardMinSafeDistanceOnSameDirection = 50.0;
constexpr double kBackwardMinSafeDistanceOnSameDirection = 10.0;
constexpr double kForwardMinSafeDistanceOnOppositeDirection = 100.0;
constexpr double kBackwardMinSafeDistanceOnOppositeDirection = 1.0;
constexpr double kDistanceBuffer = 0.5;
// A time constant for safe lane changes
constexpr double kChangeLaneTime = 6.0;
constexpr double kMinSpeed = 0.5;
constexpr double kMinConsiderBackDisatnce = 1.0;
constexpr double kMinConsiderForwardDisatnce = 10.0;
constexpr double kDynamicObsSpeed = 0.5;
constexpr double kConsiderRoutingEndDisatnce = 2.0;
constexpr double kMiinEndStateL = 0.1;
constexpr double kTireStackerHeight = 6.0;
constexpr double kConsiderStacker = 50.0;
constexpr double kLareralConsiderStacker = 5.0;
constexpr double kOppositeDirectionBlockingDistance = 50.0;
constexpr double kOppositeDirectionLateralDistance = 1.0;
constexpr double kDistanceToRoutingEnd = 5.0;
constexpr double kLonSToRoutingEnd = 20.0;
constexpr double kLatSToRoutingEnd = 10.0;
constexpr double kMinConsiderFrontDisatnce = 1.0;
constexpr double kExpresswayLanceChangeDistance = 50.0;
constexpr double kHalfVehicleWidth = 1.6;
constexpr double kCutInBuffer = 0.4;
}  // namespace

LaneChangeDecider::LaneChangeDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {
  ACHECK(config_.has_lane_change_decider_config());
}

// added a dummy parameter to enable this task in ExecuteTaskOnReferenceLine
Status LaneChangeDecider::Process(
    Frame* frame, ReferenceLineInfo* const current_reference_line_info) {
  Status status = MakeLaneChangeDecider(frame, current_reference_line_info);
  return status;
}

Status LaneChangeDecider::MakeLaneChangeDecider(
    Frame* frame, ReferenceLineInfo* const current_reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);

  const auto& lane_change_decider_config = config_.lane_change_decider_config();
  std::list<ReferenceLineInfo>* reference_line_info =
      frame->mutable_reference_line_info();
  if (reference_line_info->empty()) {
    const std::string msg = "Reference lines empty.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  if (lane_change_decider_config.reckless_change_lane()) {
    PrioritizeChangeLane(true, reference_line_info);
    return Status::OK();
  }

  auto* prev_status = injector_->planning_context()
                          ->mutable_planning_status()
                          ->mutable_change_lane();
  double now = Clock::NowInSeconds();
  prev_status->set_is_clear_to_change_lane(false);
  if (current_reference_line_info->IsChangeLanePath()) {
    bool is_clear_to_lanechange =
        IsClearToChangeLane(current_reference_line_info,
                            injector_->is_adc_in_expressway_junction_) &&
        std::fabs(injector_->last_using_lateral_ < kMiinEndStateL);
    // bool is_clear_to_lanechange =
    //   IsClearToChangeLane(current_reference_line_info) ;
    //  bool is_clear_to_lanechange =
    //  current_reference_line_info->GetIsClearToChangeLane();
    // AINFO<<"is_clear_to_lanechange = "<<is_clear_to_lanechange;
    // is_clear_to_lanechange = true;
    prev_status->set_is_clear_to_change_lane(is_clear_to_lanechange);
  }

  if (!prev_status->has_status()) {
    UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED,
                 GetCurrentPathId(*reference_line_info));
    prev_status->set_last_succeed_timestamp(now);
    return Status::OK();
  }

  bool has_change_lane = reference_line_info->size() > 1;
  ADEBUG << "has_change_lane: " << has_change_lane;
  if (!has_change_lane) {
    return UpdateStatusBaseOnReferencePath(reference_line_info,
                                           current_reference_line_info);
  } else {  // has change lane in reference lines.
    return UpdateStatusBaseOnChangeLanePath(reference_line_info,
                                            current_reference_line_info);
  }
  return Status::OK();
}

common::Status LaneChangeDecider::UpdateStatusBaseOnReferencePath(
    std::list<ReferenceLineInfo>* reference_line_info,
    ReferenceLineInfo* const current_reference_line_info) {
  auto* prev_status = injector_->planning_context()
                          ->mutable_planning_status()
                          ->mutable_change_lane();
  double now = Clock::NowInSeconds();
  const auto& path_id = reference_line_info->front().Lanes().Id();
  if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FINISHED) {
    if (current_reference_line_info->IsAdcPostureStraight()) {
      prev_status->set_is_success_change_lane_path(false);
    }
  } else if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
    if (current_reference_line_info->IsAdcLocatedInLane()) {
      UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED, path_id);
    }
  } else if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FAILED) {
    prev_status->set_is_success_change_lane_path(false);
  } else {
    const std::string msg =
        absl::StrCat("Unknown state: ", prev_status->ShortDebugString());
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

common::Status LaneChangeDecider::UpdateStatusBaseOnChangeLanePath(
    std::list<ReferenceLineInfo>* reference_line_info,
    ReferenceLineInfo* const current_reference_line_info) {
  auto* prev_status = injector_->planning_context()
                          ->mutable_planning_status()
                          ->mutable_change_lane();
  double now = Clock::NowInSeconds();
  auto current_path_id = GetCurrentPathId(*reference_line_info);
  const auto& lane_change_decider_config = config_.lane_change_decider_config();
  if (current_path_id.empty()) {
    const std::string msg = "The vehicle is not on any reference line";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  bool adc_in_ref_lane = AdcInChangeLane(reference_line_info);
  // AINFO<<"adc_in_ref_lane = "<<adc_in_ref_lane;
  if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
    if (prev_status->path_id() == current_path_id ||
        !current_reference_line_info->IsAdcPostureStraight() ||
        !adc_in_ref_lane) {
      PrioritizeChangeLane(true, reference_line_info);
    } else {
      // RemoveChangeLane(reference_line_info);
      PrioritizeChangeLane(false, reference_line_info);
      ADEBUG << "removed change lane.";
      UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED,
                   current_path_id);
      prev_status->set_is_success_change_lane_path(false);
    }
    return Status::OK();
  } else if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FAILED) {
    // TODO(SHU): add an optimization_failure counter to enter
    // change_lane_failed status
    if (now - prev_status->timestamp() <
        lane_change_decider_config.change_lane_fail_freeze_time()) {
      // RemoveChangeLane(reference_line_info);
      PrioritizeChangeLane(false, reference_line_info);
      ADEBUG << "freezed after failed";
    } else {
      UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, current_path_id);
      ADEBUG << "change lane again after failed";
    }
    return Status::OK();
  } else if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FINISHED) {
    auto& overtake_status =
        injector_->planning_context()->planning_status().overtake();
    if (now - prev_status->timestamp() <
            lane_change_decider_config.change_lane_success_freeze_time() &&
        !overtake_status.urgency_lane_change()) {
      // RemoveChangeLane(reference_line_info);
      PrioritizeChangeLane(false, reference_line_info);
      ADEBUG << "freezed after completed lane change";
    } else {
      PrioritizeChangeLane(true, reference_line_info);
      UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, current_path_id);
      ADEBUG << "change lane again after success";
    }
  } else {
    const std::string msg =
        absl::StrCat("Unknown state: ", prev_status->ShortDebugString());
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

void LaneChangeDecider::UpdatePreparationDistance(
    const bool is_opt_succeed, const Frame* frame,
    const ReferenceLineInfo* const reference_line_info,
    PlanningContext* planning_context) {
  auto* lane_change_status =
      planning_context->mutable_planning_status()->mutable_change_lane();
  ADEBUG << "Current time: " << lane_change_status->timestamp();
  ADEBUG << "Lane Change Status: " << lane_change_status->status();
  // If lane change planning succeeded, update and return
  if (is_opt_succeed) {
    lane_change_status->set_last_succeed_timestamp(Clock::NowInSeconds());
    lane_change_status->set_is_current_opt_succeed(true);
    return;
  }
  // If path optimizer or speed optimizer failed, report the status
  lane_change_status->set_is_current_opt_succeed(false);
  // If the planner just succeed recently, let's be more patient and try again
  if (Clock::NowInSeconds() - lane_change_status->last_succeed_timestamp() <
      FLAGS_allowed_lane_change_failure_time) {
    return;
  }
  // Get ADC's current s and the lane-change start distance s
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  const common::TrajectoryPoint& planning_start_point =
      frame->PlanningStartPoint();
  auto adc_sl_info = reference_line.ToFrenetFrame(planning_start_point);
  if (!lane_change_status->exist_lane_change_start_position()) {
    return;
  }
  common::SLPoint point_sl;
  reference_line.XYToSL(lane_change_status->lane_change_start_position(),
                        &point_sl);
  ADEBUG << "Current ADC s: " << adc_sl_info.first[0];
  ADEBUG << "Change lane point s: " << point_sl.s();
  // If the remaining lane-change preparation distance is too small,
  // refresh the preparation distance
  if (adc_sl_info.first[0] + FLAGS_min_lane_change_prepare_length >
      point_sl.s()) {
    lane_change_status->set_exist_lane_change_start_position(false);
    ADEBUG << "Refresh the lane-change preparation distance";
  }
}

void LaneChangeDecider::UpdateStatus(ChangeLaneStatus::Status status_code,
                                     const std::string& path_id) {
  UpdateStatus(Clock::NowInSeconds(), status_code, path_id);
}

void LaneChangeDecider::UpdateStatus(double timestamp,
                                     ChangeLaneStatus::Status status_code,
                                     const std::string& path_id) {
  auto* lane_change_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_change_lane();
  lane_change_status->set_timestamp(timestamp);
  lane_change_status->set_path_id(path_id);
  lane_change_status->set_status(status_code);
}

bool LaneChangeDecider::AdcInChangeLane(
    std::list<ReferenceLineInfo>* reference_line_info) const {
  auto iter = reference_line_info->begin();
  while (iter != reference_line_info->end()) {
    AINFO << "iter->IsChangeLanePath(): " << iter->IsChangeLanePath();
    if (iter->IsChangeLanePath()) {
      const auto& adc_sl = iter->AdcSlBoundary();
      double center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
      double center_s = (adc_sl.start_s() + adc_sl.end_s()) * 0.5;
      const auto& reference_point =
          iter->reference_line().GetReferencePoint(center_s);
      double ref_heading = reference_point.heading();
      double adc_heading = iter->vehicle_state().heading();
      double diff_heading =
          std::fabs(common::math::NormalizeAngle(ref_heading - adc_heading));
      double diff_heading_between_adc_and_startpoint = std::fabs(
          common::math::NormalizeAngle(iter->InitPointHeading() - adc_heading));
      // AINFO << " iter->InitPointHeading() = "
      //       << iter->InitPointHeading();
      // AINFO << "center_l  = " << center_l;
      // AINFO << "diff_heading = " << diff_heading;
      // AINFO << "diff_heading_between_adc_and_startpoint = "
      //       << diff_heading_between_adc_and_startpoint;
      bool is_in_ref_line =
          std::fabs(center_l) < FLAGS_lateral_error &&
          diff_heading < FLAGS_same_heading &&
          diff_heading_between_adc_and_startpoint < FLAGS_same_heading;
      // AINFO << "is_in_ref_line = " << is_in_ref_line;
      if (is_in_ref_line) {
        return true;
      }
    }

    ++iter;
  }
  return false;
}
void LaneChangeDecider::PrioritizeChangeLane(
    const bool is_prioritize_change_lane,
    std::list<ReferenceLineInfo>* reference_line_info) const {
  if (reference_line_info->empty()) {
    AERROR << "Reference line info empty";
    return;
  }

  const auto& lane_change_decider_config = config_.lane_change_decider_config();

  // TODO(SHU): disable the reference line order change for now
  if (!lane_change_decider_config.enable_prioritize_change_lane()) {
    return;
  }
  auto iter = reference_line_info->begin();
  while (iter != reference_line_info->end()) {
    ADEBUG << "iter->IsChangeLanePath(): " << iter->IsChangeLanePath();
    /* is_prioritize_change_lane == true: prioritize change_lane_reference_line
       is_prioritize_change_lane == false: prioritize
       non_change_lane_reference_line */
    if ((is_prioritize_change_lane && iter->IsChangeLanePath()) ||
        (!is_prioritize_change_lane && !iter->IsChangeLanePath())) {
      ADEBUG << "is_prioritize_change_lane: " << is_prioritize_change_lane;
      ADEBUG << "iter->IsChangeLanePath(): " << iter->IsChangeLanePath();
      break;
    }
    ++iter;
  }
  reference_line_info->splice(reference_line_info->begin(),
                              *reference_line_info, iter);
  ADEBUG << "reference_line_info->IsChangeLanePath(): "
         << reference_line_info->begin()->IsChangeLanePath();
}

// disabled for now
void LaneChangeDecider::RemoveChangeLane(
    std::list<ReferenceLineInfo>* reference_line_info) const {
  const auto& lane_change_decider_config = config_.lane_change_decider_config();
  // TODO(SHU): fix core dump when removing change lane
  if (!lane_change_decider_config.enable_remove_change_lane()) {
    return;
  }
  ADEBUG << "removed change lane";
  auto iter = reference_line_info->begin();
  while (iter != reference_line_info->end()) {
    if (iter->IsChangeLanePath()) {
      iter = reference_line_info->erase(iter);
    } else {
      ++iter;
    }
  }
}

std::string LaneChangeDecider::GetCurrentPathId(
    const std::list<ReferenceLineInfo>& reference_line_info) const {
  for (const auto& info : reference_line_info) {
    if (!info.IsChangeLanePath()) {
      return info.Lanes().Id();
    }
  }
  return "";
}

bool LaneChangeDecider::IsPerceptionBlocked(
    const ReferenceLineInfo& reference_line_info,
    const double search_beam_length, const double search_beam_radius_intensity,
    const double search_range, const double is_block_angle_threshold) {
  const auto& vehicle_state = reference_line_info.vehicle_state();
  const common::math::Vec2d adv_pos(vehicle_state.x(), vehicle_state.y());
  const double adv_heading = vehicle_state.heading();

  for (auto* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    double left_most_angle =
        common::math::NormalizeAngle(adv_heading + 0.5 * search_range);
    double right_most_angle =
        common::math::NormalizeAngle(adv_heading - 0.5 * search_range);
    bool right_most_found = false;
    if (obstacle->IsVirtual()) {
      ADEBUG << "skip one virtual obstacle";
      continue;
    }
    const auto& obstacle_polygon = obstacle->PerceptionPolygon();
    for (double search_angle = 0.0; search_angle < search_range;
         search_angle += search_beam_radius_intensity) {
      common::math::Vec2d search_beam_end(search_beam_length, 0.0);
      const double beam_heading = common::math::NormalizeAngle(
          adv_heading - 0.5 * search_range + search_angle);
      search_beam_end.SelfRotate(beam_heading);
      search_beam_end += adv_pos;
      common::math::LineSegment2d search_beam(adv_pos, search_beam_end);

      if (!right_most_found && obstacle_polygon.HasOverlap(search_beam)) {
        right_most_found = true;
        right_most_angle = beam_heading;
      }

      if (right_most_found && !obstacle_polygon.HasOverlap(search_beam)) {
        left_most_angle = beam_heading;
        break;
      }
    }
    if (!right_most_found) {
      // obstacle is not in search range
      continue;
    }
    if (std::fabs(common::math::NormalizeAngle(
            left_most_angle - right_most_angle)) > is_block_angle_threshold) {
      return true;
    }
  }

  return false;
}

bool LaneChangeDecider::IsClearToChangeLane(
    ReferenceLineInfo* reference_line_info, const bool in_expressway_junction) {
  double ego_start_s = reference_line_info->AdcSlBoundary().start_s();
  double ego_end_s = reference_line_info->AdcSlBoundary().end_s();
  double ego_v =
      std::abs(reference_line_info->vehicle_state().linear_velocity());
  const auto& route_end_waypoint =
      reference_line_info->Lanes().RouteEndWaypoint();
  century::common::PointENU routing_end;
  common::SLPoint end_sl_point;
  auto* reference_line = reference_line_info->mutable_reference_line();
  if (route_end_waypoint.lane) {
    routing_end = route_end_waypoint.lane->GetSmoothPoint(route_end_waypoint.s);
    if (!reference_line->XYToSL(routing_end, &end_sl_point)) {
      return false;
    }
  } else {
    end_sl_point.set_s(FLAGS_look_forward_short_distance);
    end_sl_point.set_l(0.0);
  }
  if (end_sl_point.s() < FLAGS_look_forward_short_distance) {
    const auto& route_end_waypoint_ref = reference_line_info->get_routing_end();
    if (!reference_line->XYToSL(route_end_waypoint_ref.pose(), &end_sl_point)) {
      return false;
    }
  }
  bool is_near_destination = end_sl_point.s() - ego_end_s < kLonSToRoutingEnd &&
                             end_sl_point.s() - ego_end_s > 0.0 &&
                             std::fabs(end_sl_point.l()) < kLatSToRoutingEnd;
  if (end_sl_point.s() - ego_end_s < kDistanceToRoutingEnd &&
      end_sl_point.s() - ego_end_s > 0.0 && is_near_destination) {
    return true;
  }
  if (in_expressway_junction) {
    common::SLPoint routing_end_sl;
    const auto routing_end = reference_line_info->get_routing_end();
    if (reference_line_info->reference_line().XYToSL(routing_end.pose(),
                                                     &routing_end_sl)) {
      auto adc_sl_boundary = reference_line_info->AdcSlBoundary();
      if (routing_end_sl.s() - adc_sl_boundary.end_s() >
          FLAGS_expressway_lance_change_distance) {
        return false;
      }
    }
  }
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle->IsVirtual()) {  // || obstacle->IsStatic()) {
      continue;
    }
    if (obstacle->Perception().type() ==
            perception::PerceptionObstacle::PEDESTRIAN &&
        obstacle->speed() < 2.0) {
      continue;
    }
    double start_s = std::numeric_limits<double>::max();
    double end_s = -std::numeric_limits<double>::max();
    double start_l = std::numeric_limits<double>::max();
    double end_l = -std::numeric_limits<double>::max();

    for (const auto& p : obstacle->PerceptionPolygon().points()) {
      SLPoint sl_point;
      reference_line_info->reference_line().XYToSL(p, &sl_point);
      start_s = std::fmin(start_s, sl_point.s());
      end_s = std::fmax(end_s, sl_point.s());

      start_l = std::fmin(start_l, sl_point.l());
      end_l = std::fmax(end_l, sl_point.l());
    }
    if (start_s - kConsiderRoutingEndDisatnce > end_sl_point.s()) {
      continue;
    }
    bool same_direction = true;   
    if (obstacle->HasTrajectory()) {
      double obstacle_moving_direction = obstacle->SpeedHeading();
      const auto& vehicle_state = reference_line_info->vehicle_state();
      double vehicle_moving_direction = vehicle_state.heading();
      if (vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE) {
        vehicle_moving_direction =
            common::math::NormalizeAngle(vehicle_moving_direction + M_PI);
      }
      double heading_difference = std::abs(common::math::NormalizeAngle(
          obstacle_moving_direction - vehicle_moving_direction));
      same_direction = heading_difference < (M_PI / 2.0);
    }
    if (reference_line_info->IsChangeLanePath()) {
      auto adc_sl = reference_line_info->AdcSlBoundary();
      const double adc_front_s = adc_sl.end_s();
      bool is_dynamic = !obstacle->IsStatic() &&
                        (obstacle->HasTrajectory() ||
                         std::fabs(obstacle->LongitudinalSpeed()) > kDynamicObsSpeed ||
                         std::fabs(obstacle->speed()) > kDynamicObsSpeed);
      double obstacle_min_abs_l = std::fmin(std::fabs(start_l), std::fabs(end_l));
      double lateral_dist_to_adc = 0.0;
      if (obstacle_min_abs_l > kHalfVehicleWidth) {
        lateral_dist_to_adc = obstacle_min_abs_l - kHalfVehicleWidth;
      } else {
        lateral_dist_to_adc = 0.0;
      }
      double longitudinal_dist_to_adc_front = 0.0;
      if (end_s < adc_front_s) {
        longitudinal_dist_to_adc_front = adc_front_s - end_s;
      } else if (start_s > adc_front_s) {
        longitudinal_dist_to_adc_front = start_s - adc_front_s;
      } else {
        longitudinal_dist_to_adc_front = 0.0;
      }
      if (!same_direction && is_dynamic &&
          longitudinal_dist_to_adc_front < kOppositeDirectionBlockingDistance &&
          lateral_dist_to_adc < kOppositeDirectionLateralDistance) {
        return false;
      }
    }
    if (reference_line_info->IsChangeLanePath()) {
      bool is_stacker = obstacle->Perception().type() ==
                            perception::PerceptionObstacle::STACKER ||
                        obstacle->Perception().type() ==
                            perception::PerceptionObstacle::FORKLIFT_STACKER;
      bool is_tire_lifter =
          obstacle->Perception().type() ==
              perception::PerceptionObstacle::WHEELCRANE;
      auto adc_sl = reference_line_info->AdcSlBoundary();
      double adc_center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
      bool is_stacker_back = end_s < adc_sl.start_s();
      bool is_satcker_front_long =
          start_s - kConsiderStacker > adc_sl.end_s();
      bool is_right_lanechange = adc_center_l > 0.0;
      if ((is_stacker || is_tire_lifter) && !is_stacker_back &&
          !is_satcker_front_long) {
        if (is_right_lanechange && end_l > -kLareralConsiderStacker - 1.6 &&
            end_l < 0.0) {
          return false;
        }
        if (!is_right_lanechange && start_l < kLareralConsiderStacker + 1.6 &&
            start_l > 0.0) {
          return false;
        }
        if (start_l <= 0.0 && end_l >= 0.0) {
          return false;
        }
      }
      double adc_lateral_collision_safe_buffer = kCutInBuffer;
      const auto obs_type = obstacle->Perception().type();
      if ((perception::PerceptionObstacle::VEHICLE == obs_type &&
           obstacle->IsStatic()) ||
          perception::PerceptionObstacle::STACKER == obs_type ||
          perception::PerceptionObstacle::FORKLIFT_STACKER == obs_type) {
        adc_lateral_collision_safe_buffer =
            adc_lateral_collision_safe_buffer + FLAGS_car_type_lateral_buffer;
      }
      if (is_tire_lifter) {
        adc_lateral_collision_safe_buffer =
            adc_lateral_collision_safe_buffer + FLAGS_car_type_lateral_buffer;
      }
      if (end_l < -kHalfVehicleWidth - adc_lateral_collision_safe_buffer ||
          start_l > kHalfVehicleWidth + adc_lateral_collision_safe_buffer) {
        continue;
      }
    }
    double kForwardSafeDistance = 0.0;
    double kBackwardSafeDistance = 0.0;
    if (same_direction) {
      kForwardSafeDistance =
          std::fmax(kForwardMinSafeDistanceOnSameDirection,
                    (ego_v - obstacle->speed()) * kSafeTimeOnSameDirection);
      kBackwardSafeDistance =
          std::fmax(kBackwardMinSafeDistanceOnSameDirection,
                    (obstacle->speed() - ego_v) * kSafeTimeOnSameDirection);
    } else {
      kForwardSafeDistance =
          std::fmax(kForwardMinSafeDistanceOnOppositeDirection,
                    (ego_v + obstacle->speed()) * kSafeTimeOnOppositeDirection);
      kBackwardSafeDistance = kBackwardMinSafeDistanceOnOppositeDirection;
    }
    if (obstacle->speed() < kMinSpeed || obstacle->IsStatic()) {
      kBackwardSafeDistance = kMinConsiderBackDisatnce;
    }
    if (obstacle->LongitudinalSpeed() > kDynamicObsSpeed) {
      kForwardSafeDistance = kMinConsiderForwardDisatnce;
    }
    
    if (is_near_destination && obstacle->LongitudinalSpeed() < kDynamicObsSpeed) {
      kForwardSafeDistance = kMinConsiderFrontDisatnce;
    }
    if (HysteresisFilter(ego_start_s - end_s, kBackwardSafeDistance,
                         kDistanceBuffer, obstacle->IsLaneChangeBlocking()) &&
        HysteresisFilter(start_s - ego_end_s, kForwardSafeDistance,
                         kDistanceBuffer, obstacle->IsLaneChangeBlocking())) {
      reference_line_info->path_decision()
          ->Find(obstacle->Id())
          ->SetLaneChangeBlocking(true);
      return false;
    } else {
      reference_line_info->path_decision()
          ->Find(obstacle->Id())
          ->SetLaneChangeBlocking(false);
    }
  }
  return true;
}
// This function is also included in the reference_line_info
bool LaneChangeDecider::HysteresisFilter(const double obstacle_distance,
                                         const double safe_distance,
                                         const double distance_buffer,
                                         const bool is_obstacle_blocking) {
  if (is_obstacle_blocking) {
    return obstacle_distance < safe_distance + distance_buffer;
  } else {
    return obstacle_distance < safe_distance - distance_buffer;
  }
}

}  // namespace planning
}  // namespace century
