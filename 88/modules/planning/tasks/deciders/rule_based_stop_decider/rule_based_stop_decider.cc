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

#include "modules/planning/tasks/deciders/rule_based_stop_decider/rule_based_stop_decider.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/route_segments.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/common.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/planning/tasks/deciders/lane_change_decider/lane_change_decider.h"

namespace century {
namespace planning {

using century::common::SLPoint;
using century::common::Status;
using century::common::math::Vec2d;

namespace {
// TODO(ALL): temporarily copy the value from lane_follow_stage.cc, will extract
// as a common value for planning later
constexpr double kEpsilon = 1e-2;
constexpr double kStraightForwardLineCost = 10.0;
constexpr double kStopWallDistance = 8.0;
constexpr double kMinCount = 0.1;

std::unordered_set<std::string> CollectLaneIds(
    const hdmap::RouteSegments &route_segments) {
  std::unordered_set<std::string> lane_ids;
  for (const auto &lane_segment : route_segments) {
    if (lane_segment.lane == nullptr) {
      continue;
    }
    lane_ids.insert(lane_segment.lane->id().id());
  }
  return lane_ids;
}

int CountPassageLaneOverlap(
    const routing::Passage &passage,
    const std::unordered_set<std::string> &lane_ids) {
  int overlap_count = 0;
  for (const auto &segment : passage.segment()) {
    if (lane_ids.count(segment.id()) > 0) {
      ++overlap_count;
    }
  }
  return overlap_count;
}

bool LocateRoutingPassage(const routing::RoutingResponse &routing,
                          const ReferenceLineInfo &reference_line_info,
                          int *road_index, int *passage_index) {
  CHECK_NOTNULL(road_index);
  CHECK_NOTNULL(passage_index);

  const auto lane_ids = CollectLaneIds(reference_line_info.Lanes());
  if (lane_ids.empty()) {
    return false;
  }

  int best_overlap_count = 0;
  int best_road_index = -1;
  int best_passage_index = -1;
  for (int i = 0; i < routing.road_size(); ++i) {
    const auto &road = routing.road(i);
    for (int j = 0; j < road.passage_size(); ++j) {
      const int overlap_count =
          CountPassageLaneOverlap(road.passage(j), lane_ids);
      if (overlap_count <= best_overlap_count) {
        continue;
      }
      best_overlap_count = overlap_count;
      best_road_index = i;
      best_passage_index = j;
    }
  }

  if (0 == best_overlap_count) {
    return false;
  }

  *road_index = best_road_index;
  *passage_index = best_passage_index;
  return true;
}

int EstimateRemainingRequiredLaneChanges(
    const routing::RoutingResponse &routing,
    const ReferenceLineInfo &reference_line_info) {
  int road_index = -1;
  int passage_index = -1;
  if (!LocateRoutingPassage(routing, reference_line_info, &road_index,
                            &passage_index)) {
    return 1;
  }

  int remaining_lane_changes = 0;
  for (int i = road_index; i < routing.road_size(); ++i) {
    const auto &road = routing.road(i);
    const int start_passage_index = (i == road_index ? passage_index : 0);
    for (int j = start_passage_index; j < road.passage_size(); ++j) {
      const auto change_lane_type = road.passage(j).change_lane_type();
      if (change_lane_type == routing::LEFT ||
          change_lane_type == routing::RIGHT) {
        ++remaining_lane_changes;
      }
    }
  }
  return std::max(remaining_lane_changes, 1);
}

double ComputeLaneChangePerLaneDistance(
    const ReferenceLineInfo &reference_line_info) {
  return kStopWallDistance;
}

double ComputeLaneChangeSoftStopBuffer(
    const ReferenceLineInfo &reference_line_info) {
  const double adc_v =
      std::fabs(reference_line_info.vehicle_state().linear_velocity());
  const double deceleration =
      std::max(std::fabs(FLAGS_soft_deceleration_for_lane_change_stop),
               kEpsilon);
  const double stop_dec_distance = 0.5 * adc_v * adc_v / deceleration;
  return std::max(stop_dec_distance,
                  FLAGS_min_stop_distance_for_lane_change_wall);
}

double ComputeDynamicLaneChangeStopDistance(
    const routing::RoutingResponse &routing,
    const ReferenceLineInfo &reference_line_info) {
  const int remaining_lane_changes =
      EstimateRemainingRequiredLaneChanges(routing, reference_line_info);
  return remaining_lane_changes *
         ComputeLaneChangePerLaneDistance(reference_line_info);
}

double ComputeDynamicLaneChangeTriggerDistance(
    const routing::RoutingResponse &routing,
    const ReferenceLineInfo &reference_line_info,
    const RuleBasedStopDeciderConfig &config) {
  const double stop_distance =
      ComputeDynamicLaneChangeStopDistance(routing, reference_line_info);
  const double trigger_buffer =
      std::max(FLAGS_brake_buffer_for_lane_change,
               ComputeLaneChangeSoftStopBuffer(reference_line_info));
  return std::max(config.approach_distance_for_lane_change(),
                  stop_distance + trigger_buffer);
}

double ClampStopDistanceToStayAhead(const double desired_stop_distance,
                                    const double distance_to_routing_end,
                                    const double forward_buffer) {
  double need_lanechange_count = desired_stop_distance / kStopWallDistance;
  double min_stop_distance = kStopWallDistance;
  if (need_lanechange_count - 1 > kMinCount) {
    min_stop_distance = (need_lanechange_count - 1) * kStopWallDistance;
  }
  return std::max(
      std::min(desired_stop_distance,
               std::max(distance_to_routing_end - forward_buffer, 0.0)),
      min_stop_distance);
}

bool GetRoutingEndSL(const routing::RoutingResponse &routing,
                     const ReferenceLineInfo &reference_line_info,
                     common::SLPoint *sl_point) {
  CHECK_NOTNULL(sl_point);
  if (routing.routing_request().waypoint_size() < 2) {
    return false;
  }

  common::PointENU routing_end;
  routing_end.set_x(
      (*(routing.routing_request().waypoint().rbegin())).pose().x());
  routing_end.set_y(
      (*(routing.routing_request().waypoint().rbegin())).pose().y());
  const auto &route_end_waypoint = reference_line_info.Lanes().RouteEndWaypoint();
  if (route_end_waypoint.lane) {
    routing_end = route_end_waypoint.lane->GetSmoothPoint(route_end_waypoint.s);
  }

  return reference_line_info.reference_line().XYToSL(routing_end, sl_point);
}
}  // namespace

RuleBasedStopDecider::RuleBasedStopDecider(
    const TaskConfig &config,
    const std::shared_ptr<DependencyInjector> &injector)
    : Decider(config, injector) {
  ACHECK(config.has_rule_based_stop_decider_config());
  rule_based_stop_decider_config_ = config.rule_based_stop_decider_config();
}

century::common::Status RuleBasedStopDecider::Process(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  // 1. Rule_based stop for side pass onto reverse lane
  //// TODO(zongxingguo) : Don't use it for the time being. Otherwise, sidepass
  /// stop will occur in the park. The parking decision will be made by the
  /// speed planning
  // StopOnSidePass(frame, reference_line_info);

  // 2. Rule_based stop for urgent lane change
  if (FLAGS_enable_lane_change_urgency_checking) {
    // AINFO << "check urgency check";
    CheckLaneChangeUrgency(frame);
  }

  // 3. Rule_based stop at path end position
  AddPathEndStop(frame, reference_line_info);

  return Status::OK();
}

void RuleBasedStopDecider::CheckLaneChangeUrgency(Frame *const frame) {
  for (auto &reference_line_info : *frame->mutable_reference_line_info()) {
    // Check if the target lane is blocked or not
    if (reference_line_info.IsChangeLanePath()) {
      is_clear_to_change_lane_ =
          LaneChangeDecider::IsClearToChangeLane(&reference_line_info);
      is_change_lane_planning_succeed_ =
          reference_line_info.Cost() < kStraightForwardLineCost;
      AddLaneChangeStopWall(frame, &reference_line_info);
      // AINFO<<"CONTINUE";
      continue;
    }

    // If it's not in lane-change scenario || (target lane is not blocked &&
    // change lane planning succeed), skip
    // AINFO<<"is_clear_to_change_lane_ = "<<is_clear_to_change_lane_;
    if (frame->reference_line_info().size() <= 1 ||
        (is_clear_to_change_lane_ && is_change_lane_planning_succeed_)) {
      // AINFO<<"CONTINUE";
      continue;
    }
    // When the target lane is blocked in change-lane case, check the urgency
    // Get the end point of current routing

    const auto &routing = frame->local_view().routing;
    if (routing == nullptr || routing->routing_request().waypoint_size() < 2) {
      continue;
    }

    common::SLPoint sl_point;
    if (GetRoutingEndSL(*routing, reference_line_info, &sl_point) &&
        reference_line_info.reference_line().IsOnLane(sl_point)) {
      // AINFO << "ROUTING END IN LANE";
      // Check the distance from ADC to the end point of current routing
      double distance_to_passage_end =
          sl_point.s() - reference_line_info.AdcSlBoundary().end_s();
      const double soft_stop_buffer =
          ComputeLaneChangeSoftStopBuffer(reference_line_info);
      const double desired_stop_distance =
          ComputeDynamicLaneChangeStopDistance(*routing, reference_line_info);
      const double trigger_distance = ComputeDynamicLaneChangeTriggerDistance(
          *routing, reference_line_info, rule_based_stop_decider_config_);
      // If ADC is still far from the end of routing, no need to stop, skip
      if (distance_to_passage_end > trigger_distance) {
        // AINFO << "CONTINUE";
        continue;
      }
      const double stop_distance = ClampStopDistanceToStayAhead(
          desired_stop_distance, distance_to_passage_end, soft_stop_buffer);
      // In urgent case, set a temporary stop fence and wait to change lane
      // TODO(Jiaxuan Xu): replace the stop fence to more intelligent actions
      const std::string stop_wall_id = "lane_change_stop";
      std::vector<std::string> wait_for_obstacles;
      util::BuildStopDecision(stop_wall_id, sl_point.s(), stop_distance,
                              StopReasonCode::STOP_REASON_LANE_CHANGE_URGENCY,
                              wait_for_obstacles, "RuleBasedStopDecider", frame,
                              &reference_line_info);
    } else {
      // AINFO << "ROUTING END NO IN LANE";
    }
  }
}

void RuleBasedStopDecider::AddLaneChangeStopWall(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  auto *lane_change_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_change_lane();
  if (LaneChangeDecider::IsClearToChangeLane(reference_line_info) &&
      !lane_change_status->exist_lane_change_start_position()) {
    // AINFO << "RETURN";
    return;
  }
  // double remain_distane = reference_line_info->GetRemainDistanceForBack(
  //     injector_->planning_context());
  const auto &routing = frame->local_view().routing;
  if (routing->routing_request().waypoint_size() < 2) {
    AERROR << "routing_request has no end";
    return;
  }

  const auto &route_end_waypoint =
      reference_line_info->Lanes().RouteEndWaypoint();
  auto *reference_line = reference_line_info->mutable_reference_line();
  common::SLPoint sl_point;
  common::PointENU routing_end;
  routing_end.set_x(
      (*(routing->routing_request().waypoint().rbegin())).pose().x());
  routing_end.set_y(
      (*(routing->routing_request().waypoint().rbegin())).pose().y());
  // If can't get lane from the route's end waypoint, then skip
  if (route_end_waypoint.lane) {
    auto routing_end =
        route_end_waypoint.lane->GetSmoothPoint(route_end_waypoint.s);
  }
  if (!reference_line->XYToSL(routing_end, &sl_point)) {
    return;
  }

  double remain_distane =
      sl_point.s() - reference_line_info->AdcSlBoundary().end_s();
  // AINFO << "remain_distane = " << remain_distane;
  if (remain_distane > 0.0 &&
      remain_distane < FLAGS_overtake_mindis_to_stopsign_threshold) {
    double adc_end_s = reference_line_info->AdcSlBoundary().end_s();
    double check_s = adc_end_s + remain_distane;
    double limit_stop_s = check_s - FLAGS_limit_stop_wall_for_lane_change;
    double stop_s = (adc_end_s > limit_stop_s) ? adc_end_s : limit_stop_s;

    if (remain_distane < FLAGS_brake_buffer_for_lane_change +
                             FLAGS_limit_stop_wall_for_lane_change) {
      // do nothing
    } else if (remain_distane < FLAGS_preview_brake_distance_for_lane_change +
                                    0.5 * FLAGS_brake_buffer_for_lane_change) {
      stop_s = check_s - FLAGS_brake_buffer_for_lane_change;
    } else {
      stop_s = check_s - FLAGS_preview_brake_distance_for_lane_change;
    }

    double adc_v =
        std::fabs(reference_line_info->vehicle_state().linear_velocity());
    double stop_dec_dis = std::fabs(
        adc_v * adc_v / FLAGS_soft_deceleration_for_lane_change_stop * 0.5);
    double soft_stop_wall_s =
        adc_end_s +
        std::fmax(stop_dec_dis, FLAGS_min_stop_distance_for_lane_change_wall);
    stop_s = std::fmax(stop_s, soft_stop_wall_s);
    // AINFO << "route lane change wall";
    const std::string stop_wall_id = "route lane change wall";
    std::vector<std::string> wait_for_obstacles;
    util::BuildStopDecision(stop_wall_id, stop_s, 0.0,
                            StopReasonCode::STOP_REASON_LANE_CHANGE_URGENCY,
                            wait_for_obstacles, "RouteLaneChangeStop", frame,
                            reference_line_info);
  }
}

void RuleBasedStopDecider::AddPathEndStop(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  if (!reference_line_info->path_data().path_label().empty() &&
      reference_line_info->path_data().frenet_frame_path().back().s() -
              reference_line_info->path_data().frenet_frame_path().front().s() <
          FLAGS_short_path_length_threshold) {
    const std::string stop_wall_id =
        PATH_END_VO_ID_PREFIX + reference_line_info->path_data().path_label();
    std::vector<std::string> wait_for_obstacles;
    util::BuildStopDecision(
        stop_wall_id,
        reference_line_info->path_data().frenet_frame_path().back().s() -
            rule_based_stop_decider_config_.stop_wall_distance(),
        0.0, StopReasonCode::STOP_REASON_REFERENCE_END, wait_for_obstacles,
        "RuleBasedStopDecider", frame, reference_line_info);
  }
}

void RuleBasedStopDecider::StopOnSidePass(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  static bool check_clear;
  static common::PathPoint change_lane_stop_path_point;

  const PathData &path_data = reference_line_info->path_data();
  double stop_s_on_pathdata = 0.0;

  if (path_data.path_label().find("self") != std::string::npos) {
    check_clear = false;
    change_lane_stop_path_point.Clear();
    return;
  }

  if (check_clear &&
      CheckClearDone(*reference_line_info, change_lane_stop_path_point)) {
    check_clear = false;
  }

  if (!check_clear &&
      CheckSidePassStop(path_data, *reference_line_info, &stop_s_on_pathdata)) {
    if (!LaneChangeDecider::IsPerceptionBlocked(
            *reference_line_info,
            rule_based_stop_decider_config_.search_beam_length(),
            rule_based_stop_decider_config_.search_beam_radius_intensity(),
            rule_based_stop_decider_config_.search_range(),
            rule_based_stop_decider_config_.is_block_angle_threshold()) &&
        reference_line_info->GetIsClearToChangeLane()) {
      return;
    }
    if (!CheckADCStop(path_data, *reference_line_info, stop_s_on_pathdata)) {
      if (!BuildSidePassStopFence(path_data, stop_s_on_pathdata,
                                  &change_lane_stop_path_point, frame,
                                  reference_line_info)) {
        AERROR << "Set side pass stop fail";
      }
    } else {
      if (reference_line_info->GetIsClearToChangeLane()) {
        check_clear = true;
      }
    }
  }
}

// @brief Check if necessary to set stop fence used for nonscenario side pass
bool RuleBasedStopDecider::CheckSidePassStop(
    const PathData &path_data, const ReferenceLineInfo &reference_line_info,
    double *stop_s_on_pathdata) {
  const std::vector<std::tuple<double, PathData::PathPointType, double>>
      &path_point_decision_guide = path_data.path_point_decision_guide();
  PathData::PathPointType last_path_point_type =
      PathData::PathPointType::UNKNOWN;
  for (const auto &point_guide : path_point_decision_guide) {
    if (last_path_point_type == PathData::PathPointType::IN_LANE &&
        std::get<1>(point_guide) ==
            PathData::PathPointType::OUT_ON_REVERSE_LANE) {
      *stop_s_on_pathdata = std::get<0>(point_guide);
      // Approximate the stop fence s based on the vehicle position
      const auto &vehicle_config =
          common::VehicleConfigHelper::Instance()->GetConfig();
      const double ego_front_to_center =
          vehicle_config.vehicle_param().front_edge_to_center();
      common::PathPoint stop_pathpoint;
      if (!path_data.GetPathPointWithRefS(*stop_s_on_pathdata,
                                          &stop_pathpoint)) {
        AERROR << "Can't get stop point on path data";
        return false;
      }
      const double ego_theta = stop_pathpoint.theta();
      Vec2d shift_vec{ego_front_to_center * std::cos(ego_theta),
                      ego_front_to_center * std::sin(ego_theta)};
      const Vec2d stop_fence_pose =
          shift_vec + Vec2d(stop_pathpoint.x(), stop_pathpoint.y());
      double stop_l_on_pathdata = 0.0;
      const auto &nearby_path = reference_line_info.reference_line().map_path();
      nearby_path.GetNearestPoint(stop_fence_pose, stop_s_on_pathdata,
                                  &stop_l_on_pathdata);
      return true;
    }
    last_path_point_type = std::get<1>(point_guide);
  }
  return false;
}

// @brief Set stop fence for side pass
bool RuleBasedStopDecider::BuildSidePassStopFence(
    const PathData &path_data, const double stop_s_on_pathdata,
    common::PathPoint *stop_point, Frame *const frame,
    ReferenceLineInfo *const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!path_data.GetPathPointWithRefS(stop_s_on_pathdata, stop_point)) {
    AERROR << "Can't get stop point on path data";
    return false;
  }

  const std::string stop_wall_id = "Side_Pass_Stop";
  std::vector<std::string> wait_for_obstacles;

  const auto &nearby_path = reference_line_info->reference_line().map_path();
  double stop_point_s = 0.0;
  double stop_point_l = 0.0;
  nearby_path.GetNearestPoint({stop_point->x(), stop_point->y()}, &stop_point_s,
                              &stop_point_l);

  util::BuildStopDecision(stop_wall_id, stop_point_s, 0.0,
                          StopReasonCode::STOP_REASON_SIDEPASS_SAFETY,
                          wait_for_obstacles, "RuleBasedStopDecider", frame,
                          reference_line_info);
  return true;
}

// @brief Check if ADV stop at a stop fence
bool RuleBasedStopDecider::CheckADCStop(
    const PathData &path_data, const ReferenceLineInfo &reference_line_info,
    const double stop_s_on_pathdata) {
  common::PathPoint stop_point;
  if (!path_data.GetPathPointWithRefS(stop_s_on_pathdata, &stop_point)) {
    AERROR << "Can't get stop point on path data";
    return false;
  }

  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  if (adc_speed > rule_based_stop_decider_config_.max_adc_stop_speed()) {
    ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
    return false;
  }

  // check stop close enough to stop line of the stop_sign
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const auto &nearby_path = reference_line_info.reference_line().map_path();
  double stop_point_s = 0.0;
  double stop_point_l = 0.0;
  nearby_path.GetNearestPoint({stop_point.x(), stop_point.y()}, &stop_point_s,
                              &stop_point_l);

  const double distance_stop_line_to_adc_front_edge =
      stop_point_s - adc_front_edge_s;

  if (distance_stop_line_to_adc_front_edge >
      rule_based_stop_decider_config_.max_valid_stop_distance()) {
    ADEBUG << "not a valid stop. too far from stop line.";
    return false;
  }

  return true;
}

bool RuleBasedStopDecider::CheckClearDone(
    const ReferenceLineInfo &reference_line_info,
    const common::PathPoint &stop_point) {
  const double adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s();
  const double adc_start_l = reference_line_info.AdcSlBoundary().start_l();
  const double adc_end_l = reference_line_info.AdcSlBoundary().end_l();
  const auto &lane_width = reference_line_info.GetLaneWidthBaseOnAdcCenter();
  double lane_left_width = lane_width.first;
  double lane_right_width = lane_width.second;
  SLPoint stop_sl_point;
  reference_line_info.reference_line().XYToSL(stop_point, &stop_sl_point);
  // use distance to last stop point to determine if needed to check clear
  // again
  if (adc_back_edge_s > stop_sl_point.s()) {
    if (adc_start_l > -lane_right_width || adc_end_l < lane_left_width) {
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace century
