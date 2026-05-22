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

#include "modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "absl/strings/str_cat.h"

#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/point_factory.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/path_boundary.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/tasks/deciders/utils/path_decider_obstacle_utils.h"

namespace century {
namespace planning {

using century::common::ErrorCode;
using century::common::Status;
using century::common::VehicleConfigHelper;
using century::common::math::Box2d;
using century::common::math::Clamp;
using century::cyber::Clock;
using century::hdmap::HDMapUtil;
using century::hdmap::JunctionInfo;
using century::perception::PerceptionObstacle;

namespace {
constexpr double kEpsilon = 1e-2;
constexpr double kAdcEdgeBuffer = 0.0;
constexpr double kNearDestinationDistance = 20;
constexpr double kLateralOverlapBuffer = 0.3;
constexpr double kHalfReserveAngle = 30;
constexpr double kAdcAheadDistance = 1;
constexpr double kMinPathBoundSpace = 0.1;
constexpr int kMaxDisappearFrameNum = 30;
constexpr double kMaxLateralDistance = 20.0;
constexpr double kCubicCheckStep = 3.0;
constexpr double kLateralBuffer = 0.3;
constexpr double kMinReverseSpeed = -1.0;
constexpr int kLonStep = 3;
constexpr double kNormalRoadWidth = 3.0;
constexpr double kTrajectoryTime = 7.0;
constexpr double kLateralSafeBuffer = 0.8;
constexpr double kDefaultAdcBufferCoeff = 1.0;
constexpr double kBoundaryBuffer = 0.1;
constexpr double kCrossVehicleBuffer = 0.3;
constexpr double kAdcLengthRatio = 2.0;
constexpr double kNoConsierLengthFrontRoutingEnd = 5.0;
constexpr double kLateralDistanceToDestination = 5.0;
constexpr double kDestinationNoBorrowDistance = 40.0;
constexpr double kTiny = 0.001;
constexpr double kIgvlatBuffer = 0.3;
constexpr double kIgvlonBuffer = 1;
constexpr double kMinKappa = 0.01;
constexpr double kTireStackerHeight = 6.0;
constexpr double kExtendLateralBuffer = 0.2;
constexpr double kHeightBuffer = 1.0;
constexpr double kLowHeightBuffer = 0.5;
constexpr double kHeightofContainer = 2.68;
constexpr double kLaneWidth = 3.0;
constexpr double kLatBoundaryBuffer = 0.2;
constexpr double kMinHeadingDiff = 0.1;
constexpr double kLonBufferForWheelcrane = 8.0;
constexpr double kMinIgvDistance = 50.0;
constexpr double kConsiderLateralDistance = 20.0;
constexpr double kLaneBorrowReturnInitialRelaxDistance = 2.0;
constexpr double kLaneBorrowReturnInitialRelaxBuffer = 0.35;
}  // namespace

std::map<std::string, PathBoundsDecider::ObstacleEdge>
    PathBoundsDecider::track_obstacle_edge_;

bool IsLaneBorrowInfo(
    const PathBoundsDecider::LaneBorrowInfo& lane_borrow_info) {
  return lane_borrow_info == PathBoundsDecider::LaneBorrowInfo::LEFT_BORROW ||
         lane_borrow_info == PathBoundsDecider::LaneBorrowInfo::RIGHT_BORROW;
}

PathBoundsDecider::PathBoundsDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {}

Status PathBoundsDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // Skip the path boundary decision if reusing the path.
  if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable()) {
    return Status::OK();
  }
  if (FLAGS_enable_v2x) {
    CheckNeedConsiderIgv(reference_line_info->reference_line());
  }
  IsAdcNearTurn();
  std::vector<PathBoundary> candidate_path_boundaries;
  // const TaskConfig& config = Decider::config_;

  // 0. Initialization.
  InitPathBoundsDecider(*frame, *reference_line_info);
  ComputeRefLineEndState(reference_line_info);
  routing_end_remain_dis_ = std::numeric_limits<double>::max();
  routing_end_s_ = GetRoutingEndS(*frame, *reference_line_info);

  // 1. Generate the fallback path boundary.
  Status ret = GenerateFallbackPathBound(*reference_line_info,
                                         &candidate_path_boundaries);
  if (!ret.ok()) {
    ADEBUG << "Cannot generate a fallback path bound.";
    return Status(ErrorCode::PLANNING_ERROR, ret.error_message());
  }

  // 2. If pull-over is requested, generate pull-over path boundary.
  if (injector_->planning_context()
          ->planning_status()
          .pull_over()
          .plan_pull_over_path()) {
    AINFO << "plan_pull_over_path";
    Status ret = GeneratePullOverPathBound(*frame, *reference_line_info,
                                           &candidate_path_boundaries);
    if (!ret.ok()) {
      AWARN << "Cannot generate a pullover path bound, do regular planning.";
    } else {
      reference_line_info->SetCandidatePathBoundaries(
          std::move(candidate_path_boundaries));
      ADEBUG << "Completed pullover and fallback path boundaries generation.";
      // set debug info in planning_data
      RecordPullOverDebugInfo(reference_line_info);
      return Status::OK();
    }
  }

  if (FLAGS_enable_smarter_lane_change &&
      reference_line_info->IsChangeLanePath()) {
    // 3. Generate lane change path boundary.
    if (!reference_line_info->lane_change_path_reusable()) {
      // enable borrow boundary for lanchange state
      if (util::IsLaneBorrow(injector_->planning_context())) {
        GetCandidateRegularPathBounds(reference_line_info,
                                      &candidate_path_boundaries);
      }
      return GenerateLaneChangePathBound(reference_line_info,
                                         &candidate_path_boundaries);
    }
  } else {
    lane_change_safe_count_ =
        config_.path_bounds_decider_config().keep_hold_safe_check_times();

    // 4. Generate regular path boundaries.
    GetCandidateRegularPathBounds(reference_line_info,
                                  &candidate_path_boundaries);
  }

  // Success
  reference_line_info->SetCandidatePathBoundaries(
      std::move(candidate_path_boundaries));
  ADEBUG << "Completed regular and fallback path boundaries generation.";
  return Status::OK();
}

void PathBoundsDecider::IsAdcNearTurn() {
  if (reference_line_info_->GetRemainDistanceToTurnLane() <
          FLAGS_distance_to_turnlane &&
      reference_line_info_->GetRemainDistanceToTurnLane() >
          0.5 * FLAGS_distance_to_turnlane) {
    // AINFO << "near turn ,no enter diagonal state";
    // AINFO << "reference_line_info_->GetRemainDistanceToTurnLane() = "
    //       << reference_line_info_->GetRemainDistanceToTurnLane();
    reference_line_info_->SetIsNearTurn(true);
  }
  double path_step = 1.0;
  bool ref_line_has_turn = false;
  const auto& adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  double s = (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) * 0.5;
  for (size_t i = 0; i < FLAGS_distance_to_turnlane*0.5; i += path_step) {
    hdmap::LaneInfoConstPtr locate_lane =
        reference_line_info_->LocateLaneInfo(s + i);

    if (nullptr != locate_lane) {
      // AINFO<<"1";
      common::SLPoint sl;
      sl.set_l(0);
      sl.set_s(s + i);
      common::math::Vec2d xy;
      if (!reference_line_info_->reference_line().SLToXY(sl, &xy)) {
        //  AINFO<<"CONTINUE";
        continue;
      }
      double s_projection = s + i;
      const auto& reference_point =
          reference_line_info_->reference_line().GetNearestReferencePoint(
              s_projection);
      //  AINFO<<"s_projection = "<<s_projection<<"  reference_point kappa =
      //  "<<reference_point.kappa();
      double minddle_kappa = reference_point.kappa();
      double theta_diff_adc_and_ref = century::common::math::NormalizeAngle(
          reference_line_info_->vehicle_state().heading() -
          reference_point.heading());
      // AINFO << "minddle_kappa = " << minddle_kappa;
      if (std::fabs(minddle_kappa) > kMinKappa ||
          locate_lane->lane().turn() != hdmap::Lane::NO_TURN ||
          std::fabs(theta_diff_adc_and_ref) > kMinHeadingDiff) {
        // if (std::fabs(minddle_kappa) > kMinKappa) {
        ref_line_has_turn = true;
      }
    }
  }
  // The reference curvature for determining the presence of a turn within 7.5
  // meters ahead of adc_end_s is being evaluated here.
  if (ref_line_has_turn) {
    reference_line_info_->SetIsNearTurn(true);
  }
      AINFO<<"ref_line_has_turn = "<<ref_line_has_turn;
}

void PathBoundsDecider::CheckNeedConsiderIgv(
    const ReferenceLine& reference_line) {
  double adc_start_s = reference_line_info_->AdcSlBoundary().start_s();

  auto v2x_info = injector_->v2x_info();
  std::unordered_map<std::string, SLBoundary> igv_boundary_map;
  // SLBoundary response_stacker_sl;

  for (const auto& vehicle_info : v2x_info.vehicle_info()) {
    // if (!vehicle_info.has_driving_mode() ||
    //     vehicle_info.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    //       AINFO<<"CONTINUE";
    //   continue;
    // }
    common::math::Vec2d vehicle_point =
        Vec2d(vehicle_info.x(), vehicle_info.y());
    // const ReferenceLine &reference_line =
    // reference_line_info_->reference_line();
    double vehicle_heading = vehicle_info.heading();
    double vehicle_length = 15.0;
    double vehicle_width = 3.2;
    std::string vehicle_id = vehicle_info.id();
    // center to loc point distance
    // double shift_distance = 0.0;
    // Compute the ADC bounding box.
    Box2d vehicle_box({vehicle_point.x(), vehicle_point.y()}, vehicle_heading,
                      vehicle_length, vehicle_width);

    SLBoundary vehicle_sl;
    if (!reference_line.GetSLBoundary(vehicle_box, &vehicle_sl)) {
      AERROR << "Failed to get igv sl boundary : " << vehicle_id;
      continue;
    }
    AINFO << "vehicle_sl.start_s = " << vehicle_sl.start_s()
          << "  vehicle_sl.end_s = " << vehicle_sl.end_s()
          << "  vehicle_sl.start_l = " << vehicle_sl.start_l()
          << "  vehicle_sl.end_s = " << vehicle_sl.end_l();

    if (vehicle_sl.end_s() < adc_start_s - kMinIgvDistance) {
      continue;
    }
    if (vehicle_sl.start_s() >
        reference_line_info_->reference_line().Length()) {
      continue;
    }

    igv_boundary_map.emplace(vehicle_id, vehicle_sl);
  }
  reference_line_info_->SetIgvBoundaryMap(igv_boundary_map);
}

void PathBoundsDecider::GetCandidateRegularPathBounds(
    ReferenceLineInfo* reference_line_info,
    std::vector<PathBoundary>* const path_bounds) {
  std::vector<LaneBorrowInfo> lane_borrow_info_list;
  GetLaneBorrowInfoList(*reference_line_info, &lane_borrow_info_list);

  // Try every possible lane-borrow option:
  for (const auto& lane_borrow_info : lane_borrow_info_list) {
    PathBound regular_path_bound;
    std::string blocking_obstacle_id = "";
    std::string borrow_lane_type = "";
    Status ret = GenerateRegularPathBound(
        *reference_line_info, lane_borrow_info, &regular_path_bound,
        &blocking_obstacle_id, &borrow_lane_type);
    if (!ret.ok()) {
      continue;
    }
    if (regular_path_bound.empty()) {
      continue;
    }
    const auto lane_borrow_status = injector_->planning_context()
                                        ->planning_status()
                                        .lane_borrow()
                                        .lane_borrow_status();
    const bool is_return_status =
        LaneborrowStatus_Status::LaneborrowStatus_Status_RETURN ==
        lane_borrow_status;
    // disable this change when not extending lane bounds to include adc
    if (config_.path_bounds_decider_config()
            .is_extend_lane_bounds_to_include_adc()) {
      CHECK_LE(adc_frenet_l_, regular_path_bound[0].l_max);
      CHECK_GE(adc_frenet_l_, regular_path_bound[0].l_min);
    }
    AdjustRegularPathBoundForLaneBorrow(
        lane_borrow_info, is_return_status, reference_line_info_->AdcSlBoundary(),
        &regular_path_bound);

    // Update the path boundary into the reference_line_info.
    std::vector<std::pair<double, double>> regular_path_bound_pair;
    // AINFO<<"adc_frenet_l_ = "<<adc_frenet_l_;
    // make sure adc l in range
    for (size_t i = 0; i < regular_path_bound.size(); ++i) {
      regular_path_bound_pair.emplace_back(regular_path_bound[i].l_min,
                                           regular_path_bound[i].l_max);
      // AINFO <<"s = "<<s<< "regular_l_min = " << regular_path_bound[i].l_min
      //       << "   regular_l_max = " << regular_path_bound[i].l_max;
    }

    path_bounds->emplace_back(regular_path_bound[0].s,
                              kPathBoundsDeciderResolution,
                              regular_path_bound_pair);
    std::string path_label = "";
    switch (lane_borrow_info) {
      case LaneBorrowInfo::LEFT_BORROW:
        path_label = "left";
        break;
      case LaneBorrowInfo::RIGHT_BORROW:
        path_label = "right";
        break;
      default:
        path_label = "self";
        break;
    }
    RecordDebugInfo(regular_path_bound, path_label, reference_line_info);
    path_bounds->back().set_label(
        absl::StrCat("regular/", path_label, "/", borrow_lane_type));
    path_bounds->back().set_blocking_obstacle_id(blocking_obstacle_id);
  }
  UpdateNarrowAreaStatus();
}

void PathBoundsDecider::AdjustRegularPathBoundForLaneBorrow(
    const LaneBorrowInfo& lane_borrow_info, const bool is_return_status,
    const SLBoundary& adc_sl, PathBound* const regular_path_bound) const {
  if (!IsLaneBorrowInfo(lane_borrow_info) || regular_path_bound == nullptr) {
    return;
  }

  for (auto& bound_point : *regular_path_bound) {
    const bool adc_overlaps_bound_point =
        bound_point.s > adc_sl.start_s() && bound_point.s < adc_sl.end_s();
    if (adc_overlaps_bound_point) {
      bound_point.l_min = std::min(bound_point.l_min, adc_frenet_l_);
      bound_point.l_max = std::max(bound_point.l_max, adc_frenet_l_);
    }

    if (!is_return_status) {
      continue;
    }

    const double relative_s = bound_point.s - adc_frenet_s_;
    if (relative_s < -kEpsilon ||
        relative_s > kLaneBorrowReturnInitialRelaxDistance) {
      continue;
    }

    if (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW) {
      bound_point.l_min =
          std::min(bound_point.l_min,
                   adc_frenet_l_ - kLaneBorrowReturnInitialRelaxBuffer);
      continue;
    }
    bound_point.l_max =
        std::max(bound_point.l_max,
                 adc_frenet_l_ + kLaneBorrowReturnInitialRelaxBuffer);
  }
}

void PathBoundsDecider::ComputeRefLineEndState(
    ReferenceLineInfo* reference_line_info) {
  const auto& config = config_.path_bounds_decider_config();
  hdmap::Lane_LaneType lane_type = reference_line_info->GetLaneType();
  if (hdmap::Lane::PLAY_STREET == lane_type) {
    // check having reverse obstacle

    // TODO(zongxing guo): optimize the IsNeedKeepRight function strategy.
    injector_->is_need_to_keep_right_ = false;
    injector_->reverse_obstacle_start_l_ =
        std::numeric_limits<double>::lowest();
    is_need_to_keep_right_ = IsNeedKeepRight(*reference_line_info);
    injector_->is_need_to_keep_right_ = is_need_to_keep_right_;
    injector_->nearst_obs_s_ = nearst_obs_s_;
    if (!config.enable_keep_right()) {
      is_need_to_keep_right_ = false;
    }

    double end_state_l = 0.0;
    if (injector_->is_need_to_keep_right_) {
      end_state_l = GetEndStateLForKeepRight(*reference_line_info_);
    }
    reference_line_info_->set_end_state_l(end_state_l);
  } else {
    if (FLAGS_enable_use_radical_decision &&
        injector_->is_can_enter_mixed_flow_ &&
        config.enable_mixed_flow_keep_right()) {
      injector_->reverse_obstacle_start_l_ =
          std::numeric_limits<double>::lowest();
      is_need_to_keep_right_ =
          IsNeedKeepRightForMixedFlow(*reference_line_info);
      ADEBUG << "is_need_to_keep_right_ = " << is_need_to_keep_right_;
      injector_->is_need_to_keep_right_ = is_need_to_keep_right_;
      injector_->nearst_obs_s_ = nearst_obs_s_;
      if (!config.enable_keep_right()) {
        is_need_to_keep_right_ = false;
      }
      double end_state_l = 0.0;
      if (injector_->is_need_to_keep_right_) {
        end_state_l = GetEndStateLForKeepRight(*reference_line_info_);
      }

      ADEBUG << "end_state_l = " << end_state_l;
      reference_line_info_->set_end_state_l(end_state_l);
    } else {
      double end_state_l = 0;
      injector_->is_need_to_keep_right_ = false;
      reference_line_info_->set_end_state_l(end_state_l);
    }
  }

  if (config.enable_wide_road_keep_right()) {
    ComputeWideRoadKeepRightEndState();
  }
}

void PathBoundsDecider::ComputeWideRoadKeepRightEndState() {
  const auto& reference_line = reference_line_info_->reference_line();

  double consider_length = std::min(
      adc_frenet_s_ +
          config_.path_bounds_decider_config().keep_right_road_length(),
      reference_line.Length());
  ADEBUG << "consider_length = " << consider_length
         << "    reference_line.Length() = " << reference_line.Length();
  const auto& reference_points =
      reference_line_info_->reference_line().GetReferencePoints(
          adc_frenet_s_, consider_length);
  bool is_long_straight_road = true;

  for (const auto& ref_point : reference_points) {
    ADEBUG << "ref_point kappa = " << ref_point.kappa();
    if (std::fabs(ref_point.kappa()) >
        config_.path_bounds_decider_config().keep_right_road_min_kappa()) {
      is_long_straight_road = false;
      break;
    }
  }
  ADEBUG << "is_long_straight_road = " << is_long_straight_road;
  bool has_neighbor_lane = false;
  bool is_wide_road = true;

  for (int i = static_cast<int>(adc_frenet_s_);
       i < static_cast<int>(consider_length - 1.0); i += kLonStep) {
    const auto& locate_lane = LocateLaneInfo(i);
    if (locate_lane != nullptr) {
      if (!locate_lane->lane().left_neighbor_forward_lane_id().empty() ||
          !locate_lane->lane().right_neighbor_forward_lane_id().empty() ||
          !locate_lane->lane().left_neighbor_reverse_lane_id().empty() ||
          !locate_lane->lane().right_neighbor_reverse_lane_id().empty()) {
        if (config_.path_bounds_decider_config().consider_neighbor_lane()) {
          has_neighbor_lane = true;
        }
        break;
      }

      double curr_lane_left_width = 0.0;
      double curr_lane_right_width = 0.0;
      if (!reference_line.GetLaneWidth(i, &curr_lane_left_width,
                                       &curr_lane_right_width)) {
        curr_lane_left_width = kNormalRoadWidth * 0.5;
        curr_lane_right_width = kNormalRoadWidth * 0.5;
      }
      ADEBUG << "curr_lane_left_width = " << curr_lane_left_width
             << "   curr_lane_right_width = " << curr_lane_right_width;
      if (curr_lane_left_width + curr_lane_right_width <
          config_.path_bounds_decider_config().keep_right_road_min_width()) {
        is_wide_road = false;
        break;
      }
    }
  }
  ADEBUG << "has_neighbor_lane = " << has_neighbor_lane;
  ADEBUG << "is_wide_road = " << is_wide_road;
  double half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  if (is_wide_road && !has_neighbor_lane && is_long_straight_road &&
      !injector_->is_need_to_keep_right_) {
    injector_->is_need_to_keep_right_ = true;
    // in road offset
    double offset = config_.path_bounds_decider_config().keep_right_offset();
    if (config_.path_bounds_decider_config().consider_neighbor_lane()) {
      offset = std::min(
          std::max(config_.path_bounds_decider_config().keep_right_offset(),
                   -adc_lane_width_ * 0.5 + half_width + kLateralBuffer),
          adc_lane_width_ * 0.5 - half_width - kLateralBuffer);
    }
    reference_line_info_->set_end_state_l(offset);
  }
}

bool PathBoundsDecider::UpdateObstacleSL(Obstacle* obs,
                                         const ReferenceLine& ref_line) {
  SLBoundary perception_sl;
  if (perception::PerceptionObstacle::UNKNOWN == obs->Perception().type() ||
      perception::PerceptionObstacle::UNKNOWN_MOVABLE ==
          obs->Perception().type() ||
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE ==
          obs->Perception().type()) {
    century::hdmap::Polygon polygon;
    for (const auto& point : obs->PerceptionPolygon().points()) {
      century::common::PointENU* hdmap_point = polygon.add_point();
      hdmap_point->set_x(point.x());
      hdmap_point->set_y(point.y());
    }
    if (FLAGS_enable_trim_unknown_obstacle) {
      if (!ref_line.TrimOutsideRoadPartSL(polygon, &perception_sl)) {
        AERROR << "Failed to trim unknown obstacle: " << obs->Id();
        return false;
      }
    } else if (!ref_line.GetSLBoundary(polygon, &perception_sl)) {
      AERROR << "Failed to get sl boundary for obstacle: " << obs->Id();
      return false;
    }
  } else {
    if (!ref_line.GetSLBoundary(obs->PerceptionBoundingBox(), &perception_sl)) {
      AERROR << "Failed to get sl boundary for obstacle: " << obs->Id();
      return false;
    }
  }
  obs->SetPerceptionSlBoundary(perception_sl);
  return true;
}

double PathBoundsDecider::GetRoutingEndS(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  common::SLPoint destination_sl;
  const auto& routing = frame.local_view().routing;
  const auto& routing_end = *(routing->routing_request().waypoint().rbegin());
  double destination_s = std::numeric_limits<double>::max();
  if (reference_line.XYToSL(routing_end.pose(), &destination_sl)) {
    destination_s = destination_sl.s();
  }
  auto adc_sl = reference_line_info.AdcSlBoundary();
  double distance_between_start_and_end =
      destination_sl.s() - (adc_sl.end_s() + adc_sl.start_s()) * 0.5;
  // AINFO << "distance_between_start_and_end = "
  //       << distance_between_start_and_end;
  bool is_near_destinaton =
      (std::fabs(destination_sl.l()) < kLateralDistanceToDestination) &&
      (distance_between_start_and_end < kDestinationNoBorrowDistance) &&
      (distance_between_start_and_end > kTiny);
  // AINFO << "is_near_destinaton = " << is_near_destinaton;
  if (!is_near_destinaton) {
    destination_s = std::numeric_limits<double>::max();
  }
  return destination_s;
}
void PathBoundsDecider::InitPathBoundsDecider(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  common::TrajectoryPoint planning_start_point = frame.PlanningStartPoint();
  reference_line_info_->SetADCLocatedInMergeLane(false);
  if (FLAGS_use_front_axe_center_in_path_planning) {
    planning_start_point =
        InferFrontAxeCenterFromRearAxeCenter(planning_start_point);
  }
  ADEBUG << "Plan at the starting point: x = "
         << planning_start_point.path_point().x()
         << ", y = " << planning_start_point.path_point().y()
         << ", and angle = " << planning_start_point.path_point().theta();
  adc_theta_ = planning_start_point.path_point().theta();
  adc_v_ = planning_start_point.v();

  // Initialize some private variables.
  // ADC s/l info.
  auto adc_sl_info = reference_line.ToFrenetFrame(planning_start_point);
  adc_frenet_s_ = adc_sl_info.first[0];
  adc_frenet_l_ = adc_sl_info.second[0];
  adc_frenet_sd_ = adc_sl_info.first[1];
  adc_frenet_ld_ = adc_sl_info.second[1] * adc_frenet_sd_;
  double offset_to_map = 0.0;
  reference_line.GetOffsetToMap(adc_frenet_s_, &offset_to_map);
  adc_l_to_lane_center_ = adc_frenet_l_ + offset_to_map;

  // ADC's lane width.
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  if (!reference_line.GetLaneWidth(adc_frenet_s_, &lane_left_width,
                                   &lane_right_width)) {
    AWARN << "Failed to get lane width at planning start point.";
    adc_lane_width_ = kDefaultLaneWidth;
  } else {
    adc_lane_width_ = lane_left_width + lane_right_width;
  }
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  mutable_path_decider_status->set_adc_will_be_blocked(false);
  if (!util::IsLaneBorrow(injector_->planning_context()) &&
      !util::IsLaneChange(injector_->planning_context())) {
    mutable_path_decider_status->set_will_pass_merge_lane_area(false);
  }
  mutable_path_decider_status->set_merge_lane_remain_dis(
      std::numeric_limits<double>::max());
  min_lateral_diff_ = std::numeric_limits<double>::max();
  auto* merge_lane_lateral_l_info =
      injector_->planning_context()->mutable_merge_lane_lateral_l();
  merge_lane_lateral_l_info->clear();
  merge_lane_lateral_l_info->shrink_to_fit();
}

common::TrajectoryPoint PathBoundsDecider::InferFrontAxeCenterFromRearAxeCenter(
    const common::TrajectoryPoint& traj_point) {
  double front_to_rear_axe_distance =
      VehicleConfigHelper::GetConfig().vehicle_param().wheel_base() * 0.5;
  common::TrajectoryPoint ret = traj_point;
  ret.mutable_path_point()->set_x(
      traj_point.path_point().x() +
      front_to_rear_axe_distance * std::cos(traj_point.path_point().theta()));
  ret.mutable_path_point()->set_y(
      traj_point.path_point().y() +
      front_to_rear_axe_distance * std::sin(traj_point.path_point().theta()));
  return ret;
}

Status PathBoundsDecider::GenerateRegularPathBound(
    const ReferenceLineInfo& reference_line_info,
    const LaneBorrowInfo& lane_borrow_info, PathBound* const path_bound,
    std::string* const blocking_obstacle_id,
    std::string* const borrow_lane_type) {
  bounds_type_ = REGULAR_PATH_BOUNDS;
  ADEBUG << "--------------- GenerateRegularPathBound ---------------";
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundary(reference_line_info, lane_borrow_info, path_bound)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  // 2. Decide a rough boundary based on lane info and ADC's position
  if (!GetBoundaryFromLanesAndADC(
          reference_line_info, lane_borrow_info, 0.1, path_bound,
          borrow_lane_type, false,
          util::IsLaneChange(injector_->planning_context()))) {
    const std::string msg =
        "Failed to decide a rough boundary based on "
        "road information.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  // TODO(jiacheng): once ready, limit the path boundary based on the
  //                 actual road boundary to avoid getting off-road.

  // 3. Fine-tune the boundary based on static obstacles
  PathBound temp_path_bound = *path_bound;
  if (!GetBoundaryFromStaticObstacles(reference_line_info.path_decision(),
                                      lane_borrow_info, path_bound,
                                      blocking_obstacle_id)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // Append some extra path bound points to avoid zero-length path data.
  int counter = 0;
  while (!blocking_obstacle_id->empty() &&
         path_bound->size() < temp_path_bound.size() &&
         counter < kNumExtraTailBoundPoint) {
    path_bound->push_back(temp_path_bound[path_bound->size()]);
    counter++;
  }
  // PathBoundsDebugString(*path_bound);

  // 4. Adjust the boundary considering dynamic obstacles
  // TODO(all): may need to implement this in the future.

  ADEBUG << "Completed generating path boundaries.";
  bounds_type_ = DEFAULT_PATH_BOUNDS;
  return Status::OK();
}

Status PathBoundsDecider::GenerateLaneChangePathBound(
    ReferenceLineInfo* reference_line_info,
    std::vector<PathBoundary>* const path_bounds) {
  PathBound lanechange_path_bound;
  bounds_type_ = LANE_CHANGE_PATH_BOUNDS;
  ADEBUG << "--------------- GenerateLaneChangePathBound ---------------";
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundary(*reference_line_info, LaneBorrowInfo::NO_BORROW,
                        &lanechange_path_bound)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 2. Decide a rough boundary based on lane info and ADC's position
  std::string dummy_borrow_lane_type;
  if (!GetBoundaryFromLanesAndADC(
          *reference_line_info, LaneBorrowInfo::NO_BORROW, 0.1,
          &lanechange_path_bound, &dummy_borrow_lane_type, false, true)) {
    const std::string msg =
        "Failed to decide a rough boundary based on "
        "road information.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 3. Remove the S-length of target lane out of the path-bound.
  GetBoundaryFromLaneChangeForbiddenZone(*reference_line_info,
                                         &lanechange_path_bound);

  PathBound temp_path_bound = lanechange_path_bound;
  std::string blocking_obstacle_id;
  auto* path_decision = reference_line_info->path_decision();
  if (!GetBoundaryFromStaticObstacles(*path_decision, LaneBorrowInfo::NO_BORROW,
                                      &lanechange_path_bound,
                                      &blocking_obstacle_id)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // Append some extra path bound points to avoid zero-length path data.
  int counter = 0;
  while (!blocking_obstacle_id.empty() &&
         lanechange_path_bound.size() < temp_path_bound.size() &&
         counter < kNumExtraTailBoundPoint) {
    lanechange_path_bound.push_back(
        temp_path_bound[lanechange_path_bound.size()]);
    counter++;
  }

  if (lanechange_path_bound.empty()) {
    const std::string msg = "Failed to get a valid lane_change path boundary";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // disable this change when not extending lane bounds to include adc
  if (config_.path_bounds_decider_config()
          .is_extend_lane_bounds_to_include_adc()) {
    CHECK_LE(adc_frenet_l_, lanechange_path_bound[0].l_max);
    CHECK_GE(adc_frenet_l_, lanechange_path_bound[0].l_min);
  }
  // Update the fallback path boundary into the reference_line_info.
  std::vector<std::pair<double, double>> lanechange_path_bound_pair;
  for (size_t i = 0; i < lanechange_path_bound.size(); ++i) {
    lanechange_path_bound_pair.emplace_back(lanechange_path_bound[i].l_min,
                                            lanechange_path_bound[i].l_max);
  }
  path_bounds->emplace_back(lanechange_path_bound[0].s,
                            kPathBoundsDeciderResolution,
                            lanechange_path_bound_pair);
  path_bounds->back().set_label("regular/lanechange");
  reference_line_info->SetCandidatePathBoundaries(std::move(*path_bounds));
  RecordDebugInfo(lanechange_path_bound, "lanechange", reference_line_info);
  ADEBUG << "Completed lanechange and fallback path boundaries generation.";
  bounds_type_ = DEFAULT_PATH_BOUNDS;
  return Status::OK();
}

Status PathBoundsDecider::GeneratePullOverPathBound(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    std::vector<PathBoundary>* const path_bounds) {
  PathBound pull_over_path_bound;
  bounds_type_ = PULL_OVER_PATH_BOUNDS;
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundary(reference_line_info, LaneBorrowInfo::NO_BORROW,
                        &pull_over_path_bound)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 2. Decide a rough boundary based on road boundary
  if (!GetBoundaryFromRoads(reference_line_info, &pull_over_path_bound)) {
    const std::string msg =
        "Failed to decide a rough boundary based on road boundary.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  ConvertBoundarySAxisFromLaneCenterToRefLine(reference_line_info,
                                              &pull_over_path_bound);
  if (adc_frenet_l_ < pull_over_path_bound.front().l_min ||
      adc_frenet_l_ > pull_over_path_bound.front().l_max) {
    const std::string msg =
        "ADC is outside road boundary already. Cannot generate pull-over path";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 2. Update boundary by lane boundary for pull_over
  UpdatePullOverBoundaryByLaneBoundary(reference_line_info,
                                       &pull_over_path_bound);

  // 3. Fine-tune the boundary based on static obstacles
  std::string blocking_obstacle_id;
  if (!GetBoundaryFromStaticObstacles(
          reference_line_info.path_decision(), LaneBorrowInfo::NO_BORROW,
          &pull_over_path_bound, &blocking_obstacle_id)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  int curr_idx = -1;
  if (!GetPullOverPosition(frame, reference_line_info, pull_over_path_bound,
                           &curr_idx)) {
    const std::string msg = "Failed to find a proper pull-over position.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // Trim path-bound properly
  TrimCandidatePullOverPathBound(&pull_over_path_bound, curr_idx, path_bounds);

  bounds_type_ = DEFAULT_PATH_BOUNDS;
  return Status::OK();
}

Status PathBoundsDecider::GenerateFallbackPathBound(
    const ReferenceLineInfo& reference_line_info,
    std::vector<PathBoundary>* const path_bounds) {
  ADEBUG << "--------------- GenerateFallbackPathBound ---------------";
  PathBound fallback_path_bound;
  bounds_type_ = FALL_BACK_PATH_BOUNDS;
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundary(reference_line_info, LaneBorrowInfo::NO_BORROW,
                        &fallback_path_bound, true)) {
    const std::string msg = "Failed to initialize fallback path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  // 2. Decide a rough boundary based on lane info and ADC's position
  std::string dummy_borrow_lane_type;
  if (!GetBoundaryFromLanesAndADC(
          reference_line_info, LaneBorrowInfo::NO_BORROW, 0.5,
          &fallback_path_bound, &dummy_borrow_lane_type, true, false)) {
    const std::string msg =
        "Failed to decide a rough fallback boundary based on "
        "road information.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (fallback_path_bound.empty()) {
    const std::string msg = "Failed to get a valid fallback path boundary";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  CHECK_LE(adc_frenet_l_, fallback_path_bound[0].l_max);
  CHECK_GE(adc_frenet_l_, fallback_path_bound[0].l_min);

  // Update the fallback path boundary into the reference_line_info.
  std::vector<std::pair<double, double>> fallback_path_bound_pair;
  for (size_t i = 0; i < fallback_path_bound.size(); ++i) {
    fallback_path_bound_pair.emplace_back(fallback_path_bound[i].l_min,
                                          fallback_path_bound[i].l_max);
  }
  path_bounds->emplace_back(fallback_path_bound[0].s,
                            kPathBoundsDeciderResolution,
                            fallback_path_bound_pair);
  path_bounds->back().set_label("fallback");

  ADEBUG << "Completed generating fallback path boundaries.";
  bounds_type_ = DEFAULT_PATH_BOUNDS;
  return Status::OK();
}

int PathBoundsDecider::IsPointWithinPathBound(
    const ReferenceLineInfo& reference_line_info, const double x,
    const double y, const PathBound& path_bound) {
  common::SLPoint point_sl;
  reference_line_info.reference_line().XYToSL({x, y}, &point_sl);
  if (point_sl.s() > path_bound.back().s ||
      point_sl.s() < path_bound.front().s - kPathBoundsDeciderResolution * 2) {
    ADEBUG << "Longitudinally outside the boundary.";
    return -1;
  }
  int idx_after = 0;
  while (idx_after < static_cast<int>(path_bound.size()) &&
         path_bound[idx_after].s < point_sl.s()) {
    ++idx_after;
  }
  ADEBUG << "The idx_after = " << idx_after;
  ADEBUG << "The boundary is: "
         << "[" << path_bound[idx_after].l_min << ", "
         << path_bound[idx_after].l_max << "].";
  ADEBUG << "The point is at: " << point_sl.l();
  int idx_before = idx_after - 1;
  if (path_bound[idx_before].l_min <= point_sl.l() &&
      path_bound[idx_before].l_max >= point_sl.l() &&
      path_bound[idx_after].l_min <= point_sl.l() &&
      path_bound[idx_after].l_max >= point_sl.l()) {
    return idx_after;
  }
  ADEBUG << "Laterally outside the boundary.";
  return -1;
}

bool PathBoundsDecider::FindDestinationPullOverS(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    const PathBound& path_bound, double* pull_over_s) {
  // destination_s based on routing_end
  const auto& reference_line = reference_line_info_->reference_line();
  common::SLPoint destination_sl;
  const auto& routing = frame.local_view().routing;
  const auto& routing_end = *(routing->routing_request().waypoint().rbegin());
  reference_line.XYToSL(routing_end.pose(), &destination_sl);
  const double destination_s = destination_sl.s();
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();

  // Check if destination is some distance away from ADC.
  ADEBUG << "Destination s[" << destination_s << "] adc_end_s[" << adc_end_s
         << "]";
  if (destination_s - adc_end_s < config_.path_bounds_decider_config()
                                      .pull_over_destination_to_adc_buffer()) {
    AERROR << "Destination is too close to ADC. distance["
           << destination_s - adc_end_s << "]";
    return false;
  }

  // Check if destination is within path-bounds searching scope.
  const double destination_to_pathend_buffer =
      config_.path_bounds_decider_config()
          .pull_over_destination_to_pathend_buffer();
  if (destination_s + destination_to_pathend_buffer >= path_bound.back().s) {
    AERROR << "Destination is not within path_bounds search scope";
    return false;
  }

  *pull_over_s = destination_s;
  return true;
}

bool PathBoundsDecider::FindEmergencyPullOverS(
    const ReferenceLineInfo& reference_line_info, double* pull_over_s) {
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  const double min_turn_radius = common::VehicleConfigHelper::Instance()
                                     ->GetConfig()
                                     .vehicle_param()
                                     .min_turn_radius();
  const double adjust_factor =
      config_.path_bounds_decider_config()
          .pull_over_approach_lon_distance_adjust_factor();
  const double pull_over_distance = min_turn_radius * 2 * adjust_factor;
  *pull_over_s = adc_end_s + pull_over_distance;

  return true;
}

bool PathBoundsDecider::GetPullOverPosition(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    const PathBound& path_bound, int* const curr_idx) {
  const auto& config = config_.path_bounds_decider_config();
  auto* pull_over_status = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_pull_over();
  // If already found a pull-over position, simply check if it's valid.
  if (pull_over_status->has_position()) {
    *curr_idx = IsPointWithinPathBound(
        reference_line_info, pull_over_status->position().x(),
        pull_over_status->position().y(), path_bound);
  }

  // If haven't found a pull-over position, search for one.
  if (*curr_idx < 0) {
    auto pull_over_type = pull_over_status->pull_over_type();
    pull_over_status->Clear();
    pull_over_status->set_pull_over_type(pull_over_type);
    pull_over_status->set_plan_pull_over_path(true);

    // tuple: x, y, theta, index
    PullOverConfiguration pull_over_configuration;
    if (!SearchPullOverPosition(frame, reference_line_info, path_bound,
                                &pull_over_configuration)) {
      return false;
    }

    *curr_idx = pull_over_configuration.index;

    // If have found a pull-over position, update planning-context
    pull_over_status->mutable_position()->set_x(pull_over_configuration.x);
    pull_over_status->mutable_position()->set_y(pull_over_configuration.y);
    pull_over_status->mutable_position()->set_z(0.0);
    pull_over_status->set_theta(pull_over_configuration.theta);
    pull_over_status->set_length_front(config.obstacle_lon_start_safe_buffer());
    pull_over_status->set_length_back(config.obstacle_lon_end_safe_buffer());
    pull_over_status->set_width_left(
        VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5);
    pull_over_status->set_width_right(
        VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5);

    ADEBUG << "Pull Over: x[" << pull_over_status->position().x() << "] y["
           << pull_over_status->position().y() << "] theta["
           << pull_over_status->theta() << "]";
  }
  return true;
}

bool PathBoundsDecider::SearchPullOverPosition(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    const PathBound& path_bound,
    PullOverConfiguration* const pull_over_configuration) {
  CHECK_NOTNULL(pull_over_configuration);

  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();

  // search direction
  bool search_backward = false;  // search FORWARD by default

  double pull_over_s = 0.0;
  if (PullOverStatus::EMERGENCY_PULL_OVER ==
      pull_over_status.pull_over_type()) {
    if (!FindEmergencyPullOverS(reference_line_info, &pull_over_s)) {
      AERROR << "Failed to find emergency_pull_over s";
      return false;
    }
    search_backward = false;  // search FORWARD from target position
  } else if (PullOverStatus::PULL_OVER == pull_over_status.pull_over_type()) {
    if (!FindDestinationPullOverS(frame, reference_line_info, path_bound,
                                  &pull_over_s)) {
      AERROR << "Failed to find pull_over s upon destination arrival";
      return false;
    }
    search_backward = true;  // search BACKWARD from target position
  } else {
    return false;
  }

  int idx = 0;
  if (search_backward) {
    // 1. Locate the first point before destination.
    idx = static_cast<int>(path_bound.size()) - 1;
    while (idx >= 0 && path_bound[idx].s > pull_over_s) {
      --idx;
    }
  } else {
    // 1. Locate the first point after emergency_pull_over s.
    while (idx < static_cast<int>(path_bound.size()) &&
           path_bound[idx].s < pull_over_s) {
      ++idx;
    }
  }
  if (idx < 0 || idx >= static_cast<int>(path_bound.size())) {
    AERROR << "Failed to find path_bound index for pull over s";
    return false;
  }

  // Search for a feasible location for pull-over.
  return SearchFeasibleLocationForPullOver(reference_line_info, search_backward,
                                           idx, path_bound,
                                           pull_over_configuration);
}

bool PathBoundsDecider::SearchFeasibleLocationForPullOver(
    const ReferenceLineInfo& reference_line_info, bool search_backward,
    int first_point_idx, const PathBound& path_bound,
    PullOverConfiguration* const pull_over_configuration) {
  const auto& config = config_.path_bounds_decider_config();
  const double pull_over_space_length =
      kPulloverLonSearchCoeff *
          VehicleConfigHelper::GetConfig().vehicle_param().length() -
      config.obstacle_lon_start_safe_buffer() -
      config.obstacle_lon_end_safe_buffer();
  const double pull_over_space_width =
      (kPulloverLatSearchCoeff - 1.0) *
      VehicleConfigHelper::GetConfig().vehicle_param().width();

  // 2. Find a window that is close to road-edge.
  // (not in any intersection)
  bool has_a_feasible_window = false;
  int idx = first_point_idx;
  while ((search_backward && idx >= 0 &&
          path_bound[idx].s - path_bound.front().s > pull_over_space_length) ||
         (!search_backward && idx < static_cast<int>(path_bound.size()) &&
          path_bound.back().s - path_bound[idx].s > pull_over_space_length)) {
    int j = idx;
    bool is_feasible_window = true;

    // Check if the point of idx is within intersection.
    double pt_ref_line_s = path_bound[idx].s;
    double pt_ref_line_l = 0.0;
    common::SLPoint pt_sl;
    pt_sl.set_s(pt_ref_line_s);
    pt_sl.set_l(pt_ref_line_l);
    common::math::Vec2d pt_xy;
    reference_line_info.reference_line().SLToXY(pt_sl, &pt_xy);
    common::PointENU hdmap_point;
    hdmap_point.set_x(pt_xy.x());
    hdmap_point.set_y(pt_xy.y());
    ADEBUG << "Pull-over position might be around (" << pt_xy.x() << ", "
           << pt_xy.y() << ")";
    std::vector<std::shared_ptr<const JunctionInfo>> junctions;
    HDMapUtil::BaseMap().GetJunctions(hdmap_point, 1.0, &junctions);
    if (!junctions.empty()) {
      AWARN << "Point is in PNC-junction.";
      idx = search_backward ? idx - 1 : idx + 1;
      continue;
    }

    if (!GetValidPallOverLocationIndex(reference_line_info, path_bound,
                                       pull_over_space_length,
                                       pull_over_space_width, search_backward,
                                       idx, &is_feasible_window, &j)) {
      return false;
    }

    if (is_feasible_window) {
      has_a_feasible_window = true;
      // estimate pull over point to have the vehicle keep same safety distance
      // to front and back
      const auto& vehicle_param =
          VehicleConfigHelper::GetConfig().vehicle_param();
      const double back_clear_to_total_length_ratio =
          (0.5 * (kPulloverLonSearchCoeff - 1.0) * vehicle_param.length() +
           vehicle_param.back_edge_to_center()) /
          vehicle_param.length() / kPulloverLonSearchCoeff;
      int start_idx = search_backward ? j : idx;
      int end_idx = search_backward ? idx : j;
      auto pull_over_idx = static_cast<size_t>(
          back_clear_to_total_length_ratio * static_cast<double>(end_idx) +
          (1.0 - back_clear_to_total_length_ratio) *
              static_cast<double>(start_idx));
      SetFeasiblePullOverConfiguration(reference_line_info, path_bound,
                                       pull_over_space_width, pull_over_idx,
                                       pull_over_configuration);

      break;
    }

    idx = search_backward ? idx - 1 : idx + 1;
  }

  return has_a_feasible_window;
}

bool PathBoundsDecider::GetValidPallOverLocationIndex(
    const ReferenceLineInfo& reference_line_info, const PathBound& path_bound,
    double pull_over_space_length, double pull_over_space_width,
    bool search_backward, int idx, bool* const is_feasible_window,
    int* const curr_idx) {
  const double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  while (
      (search_backward && *curr_idx >= 0 &&
       path_bound[idx].s - path_bound[*curr_idx].s < pull_over_space_length) ||
      (!search_backward && *curr_idx < static_cast<int>(path_bound.size()) &&
       path_bound[*curr_idx].s - path_bound[idx].s < pull_over_space_length)) {
    double curr_s = path_bound[*curr_idx].s;
    double curr_right_bound = std::fabs(path_bound[*curr_idx].l_min);
    double curr_road_left_width = 0;
    double curr_road_right_width = 0;
    reference_line_info.reference_line().GetRoadWidth(
        curr_s, &curr_road_left_width, &curr_road_right_width);
    ADEBUG << "s[" << curr_s << "] curr_road_left_width["
           << curr_road_left_width << "] curr_road_right_width["
           << curr_road_right_width << "]";
    if (curr_road_right_width - (curr_right_bound + adc_half_width) >
        config_.path_bounds_decider_config().pull_over_road_edge_buffer()) {
      AERROR << "Not close enough to road-edge. Not feasible for pull-over.";
      *is_feasible_window = false;
      break;
    }
    const double right_bound = path_bound[*curr_idx].l_min;
    const double left_bound = path_bound[*curr_idx].l_max;
    ADEBUG << "left_bound[" << left_bound << "] right_bound[" << right_bound
           << "]";
    if (left_bound - right_bound < pull_over_space_width) {
      AERROR << "Not wide enough to fit ADC. Not feasible for pull-over.";
      *is_feasible_window = false;
      break;
    }

    *curr_idx = search_backward ? *curr_idx - 1 : *curr_idx + 1;
  }
  if (*curr_idx < 0) {
    return false;
  }
  return true;
}

void PathBoundsDecider::SetFeasiblePullOverConfiguration(
    const ReferenceLineInfo& reference_line_info, const PathBound& path_bound,
    double pull_over_space_width, int pull_over_idx,
    PullOverConfiguration* const pull_over_configuration) {
  const auto& pull_over_point = path_bound[pull_over_idx];
  const double pull_over_s = pull_over_point.s;
  const double pull_over_l =
      pull_over_point.l_min + pull_over_space_width * 0.5;
  common::SLPoint pull_over_sl_point;
  pull_over_sl_point.set_s(pull_over_s);
  pull_over_sl_point.set_l(pull_over_l);

  common::math::Vec2d pull_over_xy_point;
  const auto& reference_line = reference_line_info.reference_line();
  reference_line.SLToXY(pull_over_sl_point, &pull_over_xy_point);
  const double pull_over_x = pull_over_xy_point.x();
  const double pull_over_y = pull_over_xy_point.y();

  // set the pull over theta to be the nearest lane theta rather than
  // reference line theta in case of reference line theta not aligned with
  // the lane
  const auto& reference_point = reference_line.GetReferencePoint(pull_over_s);
  double pull_over_theta = reference_point.heading();
  hdmap::LaneInfoConstPtr lane;
  double s = 0.0;
  double l = 0.0;
  auto point = common::util::PointFactory::ToPointENU(pull_over_x, pull_over_y);
  if (0 == HDMapUtil::BaseMap().GetNearestLaneWithHeading(
               point, 5.0, pull_over_theta, M_PI_2, &lane, &s, &l)) {
    pull_over_theta = lane->Heading(s);
  }
  *pull_over_configuration = {pull_over_x, pull_over_y, pull_over_theta,
                              static_cast<int>(pull_over_idx)};
}

void PathBoundsDecider::TrimCandidatePullOverPathBound(
    PathBound* const pull_over_path_bound, const int curr_idx,
    std::vector<PathBoundary>* const path_bounds) {
  while (static_cast<int>(pull_over_path_bound->size()) - 1 >
         curr_idx + kNumExtraTailBoundPoint) {
    pull_over_path_bound->pop_back();
  }
  for (size_t idx = curr_idx + 1; idx < pull_over_path_bound->size(); ++idx) {
    (*pull_over_path_bound)[idx].l_min =
        (*pull_over_path_bound)[curr_idx].l_min;
    (*pull_over_path_bound)[idx].l_max =
        (*pull_over_path_bound)[curr_idx].l_max;
  }
  ACHECK(!pull_over_path_bound->empty());
  CHECK_LE(adc_frenet_l_, (*pull_over_path_bound)[0].l_max);
  CHECK_GE(adc_frenet_l_, (*pull_over_path_bound)[0].l_min);

  // Update the fallback path boundary into the reference_line_info.
  std::vector<std::pair<double, double>> pull_over_path_bound_pair;
  for (size_t i = 0; i < pull_over_path_bound->size(); ++i) {
    pull_over_path_bound_pair.emplace_back((*pull_over_path_bound)[i].l_min,
                                           (*pull_over_path_bound)[i].l_max);
  }
  path_bounds->emplace_back((*pull_over_path_bound)[0].s,
                            kPathBoundsDeciderResolution,
                            pull_over_path_bound_pair);
  path_bounds->back().set_label("regular/pullover");
}

void PathBoundsDecider::RecordPullOverDebugInfo(
    ReferenceLineInfo* reference_line_info) {
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  auto* pull_over_debug = reference_line_info->mutable_debug()
                              ->mutable_planning_data()
                              ->mutable_pull_over();
  pull_over_debug->mutable_position()->CopyFrom(pull_over_status.position());
  pull_over_debug->set_theta(pull_over_status.theta());
  pull_over_debug->set_length_front(pull_over_status.length_front());
  pull_over_debug->set_length_back(pull_over_status.length_back());
  pull_over_debug->set_width_left(pull_over_status.width_left());
  pull_over_debug->set_width_right(pull_over_status.width_right());
}

void PathBoundsDecider::RemoveRedundantPathBoundaries(
    std::vector<PathBoundary>* const candidate_path_boundaries) {
  // 1. Check to see if both "left" and "right" exist.
  bool is_left_exist = false;
  std::vector<std::pair<double, double>> left_boundary;
  bool is_right_exist = false;
  std::vector<std::pair<double, double>> right_boundary;
  for (const auto& path_boundary : *candidate_path_boundaries) {
    if (std::string::npos != path_boundary.label().find("left")) {
      is_left_exist = true;
      left_boundary = path_boundary.boundary();
    }
    if (std::string::npos != path_boundary.label().find("right")) {
      is_right_exist = true;
      right_boundary = path_boundary.boundary();
    }
  }
  // 2. Check if "left" is contained by "right", and vice versa.
  if (!is_left_exist || !is_right_exist) {
    return;
  }
  bool is_left_redundant = false;
  bool is_right_redundant = false;
  if (IsContained(left_boundary, right_boundary)) {
    is_left_redundant = true;
  }
  if (IsContained(right_boundary, left_boundary)) {
    is_right_redundant = true;
  }

  // 3. If one contains the other, then remove the redundant one.
  for (size_t i = 0; i < candidate_path_boundaries->size(); ++i) {
    const auto& path_boundary = (*candidate_path_boundaries)[i];
    if (std::string::npos != path_boundary.label().find("right") &&
        is_right_redundant) {
      (*candidate_path_boundaries)[i] = candidate_path_boundaries->back();
      candidate_path_boundaries->pop_back();
      break;
    }
    if (std::string::npos != path_boundary.label().find("left") &&
        is_left_redundant) {
      (*candidate_path_boundaries)[i] = candidate_path_boundaries->back();
      candidate_path_boundaries->pop_back();
      break;
    }
  }
}

bool PathBoundsDecider::IsContained(
    const std::vector<std::pair<double, double>>& lhs,
    const std::vector<std::pair<double, double>>& rhs) {
  if (lhs.size() > rhs.size()) {
    return false;
  }
  for (size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i].first < rhs[i].first) {
      return false;
    }
    if (lhs[i].second > rhs[i].second) {
      return false;
    }
  }
  return true;
}

bool PathBoundsDecider::InitPathBoundary(
    const ReferenceLineInfo& reference_line_info,
    const LaneBorrowInfo& lane_borrow_info, PathBound* const path_bound,
    bool is_fallback) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  path_bound->clear();
  const auto& reference_line = reference_line_info.reference_line();
  const auto& config = config_.path_bounds_decider_config();

  // Starting from ADC's current position, increment until the horizon, and
  // set lateral bounds to be infinite at every spot.
  hdmap::Lane_LaneType lane_type = reference_line_info.GetLaneType();
  double path_length =
      reference_line_info.GetCruiseSpeed() * FLAGS_trajectory_time_length;
  if (hdmap::Lane::PLAY_STREET != lane_type) {
    path_length = GetPublicRoadPathLength(lane_borrow_info);
  } else {
    if (is_fallback || injector_->is_need_to_keep_right_ ||
        util::IsLaneBorrow(injector_->planning_context())) {
      path_length =
          std::fmax(path_length + config.extend_path_length_for_speed_plan(),
                    config.play_street_min_path_length());
    } else {
      path_length =
          std::fmin(path_length + config.extend_path_length_for_speed_plan(),
                    config.play_street_max_path_length());
    }
  }
  const double dis_to_dest = reference_line_info.SDistanceToDestination();
  path_length =
      (path_length - config.extend_path_over_destination() > dis_to_dest)
          ? (dis_to_dest + config.extend_path_over_destination())
          : path_length;

  for (double curr_s = adc_frenet_s_;
       curr_s < std::fmin(adc_frenet_s_ + path_length, reference_line.Length());
       curr_s += kPathBoundsDeciderResolution) {
    path_bound->emplace_back(curr_s, std::numeric_limits<double>::lowest(),
                             std::numeric_limits<double>::max());
  }

  // Return.
  if (path_bound->empty()) {
    ADEBUG << "Empty path boundary in InitPathBoundary";
    return false;
  }
  return true;
}

double PathBoundsDecider::GetPublicRoadPathLength(
    const LaneBorrowInfo& lane_borrow_info) {
  const auto& config = config_.path_bounds_decider_config();
  double path_length = Clamp(
      reference_line_info_->GetCruiseSpeed() * FLAGS_trajectory_time_length,
      config.public_road_min_path_length(),
      config.public_road_max_path_length());

  if (FLAGS_enable_anchor_lane_change_path &&
      reference_line_info_->IsChangeLanePath()) {
    return FLAGS_default_cruise_speed *
           config.anchor_change_path_preparation_plan_time();
  }
  if (LaneBorrowInfo::NO_BORROW == lane_borrow_info) {
    return path_length;
  }

  double remain_dis = reference_line_info_->GetRemainDistanceForBack(
      injector_->planning_context());
  routing_end_remain_dis_ = remain_dis - config.routing_end_reserved_distance();

  if (remain_dis > config.lane_borrow_use_remain_path_threshold()) {
    return std::fmin(remain_dis + config.fixed_short_path_length(),
                     path_length);
  }

  return path_length;
}

bool PathBoundsDecider::GetBoundaryFromRoads(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  // Go through every point, update the boudnary based on the road boundary.
  double past_road_left_width = adc_lane_width_ * 0.5;
  double past_road_right_width = adc_lane_width_ * 0.5;
  int path_blocked_idx = -1;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    // 1. Get road boundary.
    double curr_s = (*path_bound)[i].s;
    double curr_road_left_width = 0.0;
    double curr_road_right_width = 0.0;
    double refline_offset_to_lane_center = 0.0;
    reference_line.GetOffsetToMap(curr_s, &refline_offset_to_lane_center);
    if (!reference_line.GetRoadWidth(curr_s, &curr_road_left_width,
                                     &curr_road_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_road_left_width = past_road_left_width;
      curr_road_right_width = past_road_right_width;
    } else {
      curr_road_left_width += refline_offset_to_lane_center;
      curr_road_right_width -= refline_offset_to_lane_center;
      past_road_left_width = curr_road_left_width;
      past_road_right_width = curr_road_right_width;
    }
    double curr_left_bound = curr_road_left_width;
    double curr_right_bound = -curr_road_right_width;
    ADEBUG << "At s = " << curr_s
           << ", left road bound = " << curr_road_left_width
           << ", right road bound = " << curr_road_right_width
           << ", offset from refline to lane-center = "
           << refline_offset_to_lane_center;

    // 2. Update into path_bound.
    if (!UpdatePathBoundaryWithBuffer(&(*path_bound)[i], curr_left_bound,
                                      curr_right_bound, false, false,
                                      LaneBorrowInfo::NO_BORROW, false)) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (-1 != path_blocked_idx) {
      AINFO << "[GetBoundaryFromRoads] path_blocked_idx: " << path_blocked_idx
            << ", curr_s: " << curr_s;
      break;
    }
  }

  TrimPathBounds(path_blocked_idx, path_bound);
  return true;
}

bool PathBoundsDecider::GetBoundaryFromADC(
    const ReferenceLineInfo& reference_line_info, double ADC_extra_buffer,
    PathBound* const path_bound) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());

  // Calculate the ADC's lateral boundary.

  double ADC_lat_decel_buffer =
      (adc_frenet_ld_ > 0 ? 1.0 : -1.0) * adc_frenet_ld_ * adc_frenet_ld_ /
      config_.path_bounds_decider_config().max_lateral_acceleration() * 0.5;
  double curr_left_bound_adc =
      GetBufferBetweenADCCenterAndEdge() + ADC_extra_buffer +
      std::fmax(adc_l_to_lane_center_,
                adc_l_to_lane_center_ + ADC_lat_decel_buffer);
  double curr_right_bound_adc =
      -GetBufferBetweenADCCenterAndEdge() - ADC_extra_buffer +
      std::fmin(adc_l_to_lane_center_,
                adc_l_to_lane_center_ + ADC_lat_decel_buffer);

  // Expand the boundary in case ADC falls outside.
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_left_bound = (*path_bound)[i].l_max;
    curr_left_bound = std::fmax(curr_left_bound_adc, curr_left_bound);
    double curr_right_bound = (*path_bound)[i].l_min;
    curr_right_bound = std::fmin(curr_right_bound_adc, curr_right_bound);
    UpdatePathBoundary(i, curr_left_bound, curr_right_bound, path_bound);
  }
  return true;
}

// TODO(jiacheng): this function is to be retired soon.
bool PathBoundsDecider::GetBoundaryFromLanesAndADC(
    const ReferenceLineInfo& reference_line_info,
    const LaneBorrowInfo& lane_borrow_info, double ADC_buffer,
    PathBound* const path_bound, std::string* const borrow_lane_type,
    bool is_fallback, bool is_lanechange) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());
  merge_lane_end_s_ = std::numeric_limits<double>::max();
  merge_lane_start_s_ = std::numeric_limits<double>::max();
  the_first_merge_lane_id_ = "";

  bool is_left_lane_boundary = true;
  bool is_right_lane_boundary = true;

  // Go through every point, update the boundary based on lane info and
  // ADC's position.
  double past_lane_left_width = adc_lane_width_ * 0.5;
  double past_lane_right_width = adc_lane_width_ * 0.5;
  int path_blocked_idx = -1;
  bool borrowing_reverse_lane = false;
  double solid_lane_free_check_length = 0.0;
  double solid_lane_begin_s = 0.0;

  // use different method to generate lane change path bound
  CubicPolynomialCurve1d left_path_res;
  CubicPolynomialCurve1d right_path_res;
  PathBoundPoint lane_change_start_point(0.0, 0.0, 0.0);
  PathBoundPoint lane_change_end_point(0.0, 0.0, 0.0);
  bool vaild_lane_change_path_bound =
      is_lanechange && GetValidLaneChangePathBound(
                           &left_path_res, &right_path_res,
                           &lane_change_start_point, &lane_change_end_point);

  auto use_lane_borrow_info = lane_borrow_info;
  GetFallbackLaneBorrowInfo(is_fallback, &use_lane_borrow_info);
  bool in_near_junction_lane_borrow_scenario =
      reference_line_info_->IsInNearJunctionLaneBorrowScenario(
          injector_->planning_context());
  double lane_change_buffer =
      is_lanechange ? GetLaneChangeAdcBuffer(reference_line_info) : 0.0;
  ADEBUG << "lane_change_buffer: " << lane_change_buffer;

  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_neighbor_lane_width = 0.0;
    double curr_s = (*path_bound)[i].s;
    // 1. Get the current lane width at current point.
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    GetCurrentLaneWidth(reference_line_info, curr_s, &curr_lane_left_width,
                        &curr_lane_right_width, &past_lane_left_width,
                        &past_lane_right_width, &is_left_lane_boundary,
                        &is_right_lane_boundary);

    // 2. Get the neighbor lane widths at the current point.
    GetNeighborLaneWidth(reference_line_info, use_lane_borrow_info,
                         in_near_junction_lane_borrow_scenario, is_lanechange,
                         curr_lane_left_width, curr_lane_right_width, curr_s,
                         &solid_lane_begin_s, &solid_lane_free_check_length,
                         &curr_neighbor_lane_width, &borrowing_reverse_lane);

    // 3. Calculate the proper boundary based on lane-width, ADC's position,
    //    and ADC's velocity.
    double curr_left_bound_lane = 0.0;
    double curr_right_bound_lane = 0.0;
    GetLateralDecisionLaneBound(
        use_lane_borrow_info, curr_s, curr_lane_left_width,
        curr_lane_right_width, curr_neighbor_lane_width,
        vaild_lane_change_path_bound, left_path_res, right_path_res,
        lane_change_start_point, lane_change_end_point, &curr_left_bound_lane,
        &curr_right_bound_lane);

    double curr_left_bound = 0.0;
    double curr_right_bound = 0.0;
    GetLaneBoundIncludeADC(reference_line_info, curr_s, ADC_buffer, is_fallback,
                           is_lanechange, lane_change_buffer,
                           curr_left_bound_lane, curr_right_bound_lane,
                           &curr_left_bound, &curr_right_bound);

    // 4. Update the boundary.
    if (!UpdatePathBoundaryWithBuffer(&(*path_bound)[i], curr_left_bound,
                                      curr_right_bound, is_left_lane_boundary,
                                      is_right_lane_boundary,
                                      use_lane_borrow_info, is_lanechange)) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (-1 != path_blocked_idx) {
      AINFO << "[GetBoundaryFromLanesAndADC] path_blocked_idx: "
            << path_blocked_idx << ", curr_s: " << curr_s;
      break;
    }
  }

  TrimPathBounds(path_blocked_idx, path_bound);

  if (LaneBorrowInfo::NO_BORROW == lane_borrow_info) {
    *borrow_lane_type = "";
  } else {
    *borrow_lane_type = borrowing_reverse_lane ? "reverse" : "forward";
  }

  return true;
}

void PathBoundsDecider::GetFallbackLaneBorrowInfo(
    const bool is_fallback, LaneBorrowInfo* const use_lane_borrow_info) {
  if (!FLAGS_enable_extend_path_bound_base_adc_posture || !is_fallback ||
      LaneBorrowInfo::NO_BORROW != *use_lane_borrow_info) {
    return;
  }
  auto heading_diff = reference_line_info_->GetAdcHeadingDiffWithRefLine();
  if (reference_line_info_->IsAdcLocatedInLane() &&
      std::fabs(heading_diff) < FLAGS_adc_posture_correct_check_heading_diff) {
    return;
  }

  const auto& adc_sl = reference_line_info_->AdcSlBoundary();
  double adc_center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;

  if (heading_diff > FLAGS_adc_posture_correct_check_heading_diff) {
    *use_lane_borrow_info = LaneBorrowInfo::LEFT_BORROW;
  } else if (heading_diff < -FLAGS_adc_posture_correct_check_heading_diff) {
    *use_lane_borrow_info = LaneBorrowInfo::RIGHT_BORROW;
  } else if (adc_center_l > 0.0) {
    *use_lane_borrow_info = LaneBorrowInfo::LEFT_BORROW;
  } else {
    *use_lane_borrow_info = LaneBorrowInfo::RIGHT_BORROW;
  }
  return;
}

void PathBoundsDecider::GetLaneBoundIncludeADC(
    const ReferenceLineInfo& reference_line_info, const double curr_s,
    const double ADC_buffer, const bool is_fallback, const bool is_lanechange,
    const double lane_change_buffer, const double curr_left_bound_lane,
    const double curr_right_bound_lane, double* const curr_left_bound,
    double* const curr_right_bound) {
  const auto& config = config_.path_bounds_decider_config();
  double ADC_speed_buffer = (adc_frenet_ld_ > 0.0 ? 1.0 : -1.0) *
                            adc_frenet_ld_ * adc_frenet_ld_ /
                            config.max_lateral_acceleration() * 0.5;

  if (config.is_extend_lane_bounds_to_include_adc() || is_fallback ||
      is_lanechange) {
    double curr_road_left_width = 0.0;
    double curr_road_right_width = 0.0;
    double past_road_left_width = adc_lane_width_ * 0.5;
    double past_road_right_width = adc_lane_width_ * 0.5;
    if (!reference_line_info_->reference_line().GetRoadWidth(
            curr_s, &curr_road_left_width, &curr_road_right_width)) {
      curr_road_left_width = past_road_left_width;
      curr_road_right_width = past_road_right_width;
    } else {
      past_road_left_width = curr_road_left_width;
      past_road_right_width = curr_road_right_width;
    }
    // extend path bounds to include ADC in fallback or change lane path
    // bounds.
    double curr_left_bound_adc =
        std::fmax(adc_l_to_lane_center_,
                  adc_l_to_lane_center_ + ADC_speed_buffer) +
        GetBufferBetweenADCCenterAndEdge() + ADC_buffer +
        std::fmax(lane_change_buffer, 0.0);
    *curr_left_bound = std::fmax(curr_left_bound_lane, curr_left_bound_adc);
    if (is_lanechange) {
      *curr_left_bound =
          std::min(*curr_left_bound,
                   curr_road_left_width - GetBufferBetweenADCCenterAndEdge());
    }

    double curr_right_bound_adc =
        std::fmin(adc_l_to_lane_center_,
                  adc_l_to_lane_center_ + ADC_speed_buffer) -
        GetBufferBetweenADCCenterAndEdge() - ADC_buffer +
        std::fmin(lane_change_buffer, 0.0);
    *curr_right_bound = std::fmin(curr_right_bound_lane, curr_right_bound_adc);
    if(is_lanechange){
      *curr_right_bound =
          std::max(*curr_right_bound,
                   -curr_road_right_width + GetBufferBetweenADCCenterAndEdge());
    }
  } else {
    *curr_left_bound = curr_left_bound_lane;
    *curr_right_bound = curr_right_bound_lane;
  }
  if (curr_s > merge_lane_end_s_ ||
      (curr_s > merge_lane_start_s_ && is_lanechange)) {
    *curr_left_bound = std::fmin(curr_left_bound_lane, *curr_left_bound);
    *curr_right_bound = std::fmax(curr_right_bound_lane, *curr_right_bound);
  }
}

bool PathBoundsDecider::GetValidLaneChangePathBound(
    CubicPolynomialCurve1d* const left_path_res,
    CubicPolynomialCurve1d* const right_path_res,
    PathBoundPoint* const lane_change_start_point,
    PathBoundPoint* const lane_change_end_point) {
  if (IsNeedLaneChangePassMergeLane()) {
    ADEBUG << "adc will_pass_merge_lane_area, not valid lane change path bound";
    return false;
  }
  const auto& config = config_.path_bounds_decider_config();
  bool vaild_lane_change_path_bound = false;
  if (PathBoundsDeciderConfig::DEFAULT !=
      config.lane_change_path_bound_method()) {
    vaild_lane_change_path_bound = ComupteLaneChangeKeyPoint(
        lane_change_start_point, lane_change_end_point);
  }
  if (vaild_lane_change_path_bound) {
    if (PathBoundsDeciderConfig::CUBIC ==
        config.lane_change_path_bound_method()) {
      double dl_limit =
          Clamp(lane_change_pre_adc_lateral_v_ * config.cubic_dl_bound_weight(),
                -config.cubic_max_dl_bound(), config.cubic_max_dl_bound());
      std::array<double, 3> left_start_point = {lane_change_start_point->l_max,
                                                dl_limit, 0.0};
      std::array<double, 3> left_end_point = {lane_change_end_point->l_max, 0.0,
                                              0.0};
      std::array<double, 3> right_start_point = {lane_change_start_point->l_min,
                                                 dl_limit, 0.0};
      std::array<double, 3> right_end_point = {lane_change_end_point->l_min,
                                               0.0, 0.0};
      double s_length = lane_change_end_point->s - lane_change_start_point->s;
      *left_path_res =
          CubicPolynomialCurve1d(left_start_point, left_end_point[0], s_length);
      ADEBUG << "left_start_point_L: " << left_start_point[0]
             << ", left_end_point_l: " << left_end_point[0];
      for (double s = 0.0; s < left_path_res->ParamLength();
           s += kCubicCheckStep) {
        const double left_l = left_path_res->Evaluate(0, s);
        if (std::fabs(left_l) > kMaxLateralDistance) {
          vaild_lane_change_path_bound = false;
          break;
        }
      }

      if (vaild_lane_change_path_bound) {
        *right_path_res = CubicPolynomialCurve1d(right_start_point,
                                                 right_end_point[0], s_length);
        ADEBUG << "right_start_point_L: " << right_start_point[0]
               << ", right_end_point_l: " << right_end_point[0];
        for (double s = 0.0; s < right_path_res->ParamLength();
             s += kCubicCheckStep) {
          const double right_l = right_path_res->Evaluate(0, s);
          if (std::fabs(right_l) > kMaxLateralDistance) {
            vaild_lane_change_path_bound = false;
            break;
          }
        }
      }
    }
  }

  return vaild_lane_change_path_bound;
}

void PathBoundsDecider::GetCurrentLaneWidth(
    const ReferenceLineInfo& reference_line_info, const double curr_s,
    double* const curr_lane_left_width, double* const curr_lane_right_width,
    double* const past_lane_left_width, double* const past_lane_right_width,
    bool* const is_left_lane_boundary, bool* const is_right_lane_boundary) {
  const auto& reference_line = reference_line_info.reference_line();
  // 1. Get the current lane width at current point.
  if (!reference_line.GetLaneWidth(curr_s, curr_lane_left_width,
                                   curr_lane_right_width)) {
    if (FLAGS_enable_auto_borrow && injector_->auto_state_count_ == 100) {
      reference_line.GetRoadWidth(curr_s, curr_lane_left_width,
                                  curr_lane_right_width);
    }
    AWARN << "Failed to get lane width at s = " << curr_s;
    *curr_lane_left_width = *past_lane_left_width;
    *curr_lane_right_width = *past_lane_right_width;
  } else {
    // check if lane boundary is also road boundary
    double curr_road_left_width = 0.0;
    double curr_road_right_width = 0.0;
    if (reference_line.GetRoadWidth(curr_s, &curr_road_left_width,
                                    &curr_road_right_width)) {
      *is_left_lane_boundary =
          (std::abs(curr_road_left_width - *curr_lane_left_width) >
           kBoundaryBuffer);
      *is_right_lane_boundary =
          (std::abs(curr_road_right_width - *curr_lane_right_width) >
           kBoundaryBuffer);
    }
    double offset_to_lane_center = 0.0;
    reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
    *curr_lane_left_width += offset_to_lane_center;
    *curr_lane_right_width -= offset_to_lane_center;
    *past_lane_left_width = *curr_lane_left_width;
    *past_lane_right_width = *curr_lane_right_width;
  }
}

void PathBoundsDecider::GetNeighborLaneWidth(
    const ReferenceLineInfo& reference_line_info,
    const LaneBorrowInfo& use_lane_borrow_info,
    const bool in_near_junction_lane_borrow_scenario, bool is_lanechange,
    const double curr_lane_left_width, const double curr_lane_right_width,
    const double curr_s, double* const solid_lane_begin_s,
    double* const solid_lane_free_check_length,
    double* const curr_neighbor_lane_width,
    bool* const borrowing_reverse_lane) {
  // if in lane borrow state, allow to pass solid line no more than 5m,
  // inorder to improve the success of passing obstacle
  bool check_boundary =
      CheckLaneBoundaryType(reference_line_info, curr_s, use_lane_borrow_info);
  bool check_free_length =
      ((*solid_lane_free_check_length -
        config_.path_bounds_decider_config()
            .in_borrow_free_solid_line_length()) < -kEpsilon) &&
      (use_lane_borrow_info != LaneBorrowInfo::NO_BORROW);
  if (!check_boundary && check_free_length) {
    *solid_lane_free_check_length = curr_s - *solid_lane_begin_s;
  } else {
    *solid_lane_begin_s = curr_s;
  }

  if ((check_boundary || check_free_length ||
       in_near_junction_lane_borrow_scenario ||
       reference_line_info.IsChangeLanePath()) &&
      curr_s < merge_lane_end_s_) {
    GetNeighborLaneWidthBaseOnBorrowInfo(
        reference_line_info, curr_s, use_lane_borrow_info, is_lanechange,
        curr_lane_left_width, curr_lane_right_width, curr_neighbor_lane_width,
        borrowing_reverse_lane);
  }
}

void PathBoundsDecider::GetNeighborLaneWidthBaseOnBorrowInfo(
    const ReferenceLineInfo& reference_line_info, const double curr_s,
    const LaneBorrowInfo& lane_borrow_info, bool is_lanechange,
    const double curr_lane_left_width, const double curr_lane_right_width,
    double* const curr_neighbor_lane_width,
    bool* const borrowing_reverse_lane) {
  hdmap::Id neighbor_lane_id;
  bool need_check_merge_lane = false;
  double distance_between_ref = 0.0;
  if (LaneBorrowInfo::LEFT_BORROW == lane_borrow_info) {
    // Borrowing left neighbor lane.
    need_check_merge_lane = GetLeftNeighborLaneInfo(
        reference_line_info, curr_s, curr_lane_left_width,
        curr_lane_right_width, &neighbor_lane_id, &distance_between_ref,
        curr_neighbor_lane_width, borrowing_reverse_lane);

  } else if (LaneBorrowInfo::RIGHT_BORROW == lane_borrow_info) {
    // Borrowing right neighbor lane.
    need_check_merge_lane = GetRightNeighborLaneInfo(
        reference_line_info, curr_s, curr_lane_left_width,
        curr_lane_right_width, &neighbor_lane_id, &distance_between_ref,
        curr_neighbor_lane_width, borrowing_reverse_lane);

  } else if (IsNeedLaneChangePassMergeLane() && is_lanechange) {
    need_check_merge_lane = true;
    GetLaneChangeNeighborLaneInfo(
        reference_line_info, curr_s, curr_lane_left_width,
        curr_lane_right_width, &neighbor_lane_id, &distance_between_ref,
        curr_neighbor_lane_width, borrowing_reverse_lane);
  }
  if (need_check_merge_lane || reference_line_info.IsChangeLanePath()) {
    GetMergeLaneWidth(reference_line_info, curr_s, neighbor_lane_id,
                      distance_between_ref, curr_neighbor_lane_width);
  }
}

bool PathBoundsDecider::GetLeftNeighborLaneInfo(
    const ReferenceLineInfo& reference_line_info, const double curr_s,
    const double curr_lane_left_width, const double curr_lane_right_width,
    hdmap::Id* neighbor_lane_id, double* const distance_between_ref,
    double* const curr_neighbor_lane_width,
    bool* const borrowing_reverse_lane) {
  bool need_check_merge_lane = true;
  if (reference_line_info.GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::LeftForward, curr_s, neighbor_lane_id,
          curr_neighbor_lane_width)) {
    ADEBUG << "Borrow left forward neighbor lane.";
  } else if (reference_line_info.GetNeighborLaneInfo(
                 ReferenceLineInfo::LaneType::LeftReverse, curr_s,
                 neighbor_lane_id, curr_neighbor_lane_width)) {
    *borrowing_reverse_lane = true;
    ADEBUG << "Borrow left reverse neighbor lane.";
  } else {
    ADEBUG << "There is no left neighbor lane.";
    need_check_merge_lane = false;
  }
  *distance_between_ref =
      curr_lane_left_width + *curr_neighbor_lane_width * 0.5;
  if (!(*borrowing_reverse_lane) && !reference_line_info.IsChangeLanePath()) {
    auto ptr_neighbor_lane =
        hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(*neighbor_lane_id);
    if (ptr_neighbor_lane &&
        !ptr_neighbor_lane->lane().left_neighbor_forward_lane_id().empty()) {
      auto left_boundary_type =
          ptr_neighbor_lane->lane().left_boundary().boundary_type(0).types(0);
      if (hdmap::LaneBoundaryType::DOTTED_WHITE == left_boundary_type ||
          hdmap::LaneBoundaryType::DOTTED_YELLOW == left_boundary_type) {
        auto adc_buffer_coeff =
            config_.path_bounds_decider_config().adc_buffer_coeff();
        *curr_neighbor_lane_width +=
            adc_buffer_coeff * GetBufferBetweenADCCenterAndEdge();
      }
    }
  }
  return need_check_merge_lane;
}

bool PathBoundsDecider::GetRightNeighborLaneInfo(
    const ReferenceLineInfo& reference_line_info, const double curr_s,
    const double curr_lane_left_width, const double curr_lane_right_width,
    hdmap::Id* neighbor_lane_id, double* const distance_between_ref,
    double* const curr_neighbor_lane_width,
    bool* const borrowing_reverse_lane) {
  bool need_check_merge_lane = true;
  if (reference_line_info.GetNeighborLaneInfo(
          ReferenceLineInfo::LaneType::RightForward, curr_s, neighbor_lane_id,
          curr_neighbor_lane_width)) {
    ADEBUG << "Borrow right forward neighbor lane.";
  } else if (reference_line_info.GetNeighborLaneInfo(
                 ReferenceLineInfo::LaneType::RightReverse, curr_s,
                 neighbor_lane_id, curr_neighbor_lane_width)) {
    *borrowing_reverse_lane = true;
    ADEBUG << "Borrow right reverse neighbor lane.";
  } else {
    ADEBUG << "There is no right neighbor lane.";
    need_check_merge_lane = false;
  }
  *distance_between_ref =
      curr_lane_right_width + *curr_neighbor_lane_width * 0.5;
  if (!(*borrowing_reverse_lane) && !reference_line_info.IsChangeLanePath()) {
    auto ptr_neighbor_lane =
        hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(*neighbor_lane_id);
    if (ptr_neighbor_lane &&
        !ptr_neighbor_lane->lane().right_neighbor_forward_lane_id().empty()) {
      auto right_boundary_type =
          ptr_neighbor_lane->lane().right_boundary().boundary_type(0).types(0);
      if (hdmap::LaneBoundaryType::DOTTED_WHITE == right_boundary_type ||
          hdmap::LaneBoundaryType::DOTTED_YELLOW == right_boundary_type) {
        auto adc_buffer_coeff =
            config_.path_bounds_decider_config().adc_buffer_coeff();
        *curr_neighbor_lane_width +=
            adc_buffer_coeff * GetBufferBetweenADCCenterAndEdge();
      }
    }
  }

  return need_check_merge_lane;
}

void PathBoundsDecider::GetLaneChangeNeighborLaneInfo(
    const ReferenceLineInfo& reference_line_info, const double curr_s,
    const double curr_lane_left_width, const double curr_lane_right_width,
    hdmap::Id* neighbor_lane_id, double* const distance_between_ref,
    double* const curr_neighbor_lane_width,
    bool* const borrowing_reverse_lane) {
  const auto& adc_sl = reference_line_info.AdcSlBoundary();
  double adc_center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
  bool turn_left = (adc_center_l < 0.0);

  if (turn_left) {
    GetRightNeighborLaneInfo(reference_line_info, curr_s, curr_lane_left_width,
                             curr_lane_right_width, neighbor_lane_id,
                             distance_between_ref, curr_neighbor_lane_width,
                             borrowing_reverse_lane);
  } else {
    GetLeftNeighborLaneInfo(reference_line_info, curr_s, curr_lane_left_width,
                            curr_lane_right_width, neighbor_lane_id,
                            distance_between_ref, curr_neighbor_lane_width,
                            borrowing_reverse_lane);
  }
  return;
}

void PathBoundsDecider::GetMergeLaneWidth(
    const ReferenceLineInfo& reference_line_info, const double curr_s,
    const hdmap::Id neighbor_lane_id, const double distance_between_ref,
    double* const curr_neighbor_lane_width) {
  auto curr_lane_info = reference_line_info.LocateLaneInfo(curr_s);
  if (nullptr == curr_lane_info) {
    return;
  }
  const auto& curr_lane = curr_lane_info->lane();
  if (1) {
    return;
  }
  if (the_first_merge_lane_id_.empty()) {
    the_first_merge_lane_id_ = curr_lane.id().id();
  } else if (!util::CompareTwoStringIsEqual(the_first_merge_lane_id_,
                                            curr_lane.id().id())) {
    ADEBUG << "not same merge lane, there have two";
    return;
  }
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  mutable_path_decider_status->set_will_pass_merge_lane_area(true);

  if (std::fabs(curr_s - adc_frenet_s_) <
      config_.path_bounds_decider_config().near_merge_lane_check_distance()) {
    reference_line_info_->SetADCLocatedInMergeLane(true);
  }

  const auto& start_position =
      curr_lane.central_curve().segment(0).start_position();
  common::SLPoint start_point_sl;
  common::math::Vec2d start_point = {start_position.x(), start_position.y()};
  if (!reference_line_info.reference_line().XYToSL(start_point,
                                                   &start_point_sl)) {
    AINFO << "can not convert merge lane start point to SL, merge lane id: "
          << curr_lane.id().id();
    return;
  }

  mutable_path_decider_status->set_merge_lane_remain_dis(start_point_sl.s() -
                                                         adc_frenet_s_);
  merge_lane_end_s_ = start_point_sl.s() + curr_lane_info->total_length();
  merge_lane_start_s_ = start_point_sl.s();

  auto ptr_neighbor_lane =
      hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(neighbor_lane_id);
  if (!ptr_neighbor_lane) {
    return;
  }

  double diff_s = curr_s - start_point_sl.s();
  const auto lane_point = curr_lane_info->GetSmoothPoint(diff_s);
  common::math::Vec2d project_point = {lane_point.x(), lane_point.y()};
  double accumulate_s = 0.0;
  double lateral_l = 0.0;
  if (ptr_neighbor_lane->GetProjection(project_point, &accumulate_s,
                                       &lateral_l)) {
    double diff_l = distance_between_ref - std::fabs(lateral_l);
    *curr_neighbor_lane_width -= diff_l;
    auto* merge_lane_lateral_l_info =
        injector_->planning_context()->mutable_merge_lane_lateral_l();
    merge_lane_lateral_l_info->emplace_back(-lateral_l);
    ADEBUG << "[merge lane] curr_s: " << curr_s << ", diff_l: " << diff_l
           << ", fixed neighbor_lane_width: " << *curr_neighbor_lane_width
           << ", lateral_l: " << lateral_l;
  }
}

void PathBoundsDecider::GetLateralDecisionLaneBound(
    const LaneBorrowInfo& lane_borrow_info, const double curr_s,
    const double curr_lane_left_width, const double curr_lane_right_width,
    const double curr_neighbor_lane_width,
    const bool vaild_lane_change_path_bound,
    const CubicPolynomialCurve1d& left_path_res,
    const CubicPolynomialCurve1d& right_path_res,
    const PathBoundPoint& lane_change_start_point,
    const PathBoundPoint& lane_change_end_point,
    double* const curr_left_bound_lane, double* const curr_right_bound_lane) {
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double half_width = veh_param.width() * 0.5;
  double left_width_fix_buffer = 0.0;
  const auto& config = config_.path_bounds_decider_config();
  if (!vaild_lane_change_path_bound &&
      util::IsAdcInJunction(injector_->planning_context()) &&
      CanExtendCurrentLaneLeftBound(curr_s)) {
    auto adc_buffer_coeff = config.adc_buffer_coeff();
    left_width_fix_buffer =
        adc_buffer_coeff * GetBufferBetweenADCCenterAndEdge();
  }
  *curr_left_bound_lane =
      curr_lane_left_width +
      ((LaneBorrowInfo::LEFT_BORROW == lane_borrow_info ||
        (reference_line_info_->IsRightLaneChange() && IsWillPassMergeLane()))
           ? curr_neighbor_lane_width
           : left_width_fix_buffer);
  double curr_road_left_width = 0.0;
  double curr_road_right_width = 0.0;
  double past_road_left_width = adc_lane_width_ * 0.5;
  double past_road_right_width = adc_lane_width_ * 0.5;
  if (!reference_line_info_->reference_line().GetRoadWidth(
          curr_s, &curr_road_left_width, &curr_road_right_width)) {
    // AWARN << "Failed to get lane width at s = " << curr_s;
    curr_road_left_width = past_road_left_width;
    curr_road_right_width = past_road_right_width;
  } else {
    past_road_left_width = curr_road_left_width;
    past_road_right_width = curr_road_right_width;
  }
  // AINFO << "curr_s = " << curr_s
  //       << "   curr_road_left_width = " << curr_road_left_width
  //       << "   curr_road_right_width = " << curr_road_right_width
  //       << "   *curr_left_bound_lane = " << *curr_left_bound_lane
  //       << "   *curr_right_bound_lane = " << *curr_right_bound_lane;
  // bool extend_both_side = false;
  if ((LaneBorrowInfo::LEFT_BORROW == lane_borrow_info) ||
      (LaneBorrowInfo::RIGHT_BORROW == lane_borrow_info)) {
    auto* mutable_path_decider_status = injector_->planning_context()
                                            ->mutable_planning_status()
                                            ->mutable_path_decider();
    auto block_id = mutable_path_decider_status->front_static_obstacle_id();

    if (block_id != "") {
      const auto* block_obstacle =
          reference_line_info_->path_decision()->Find(block_id);
      if (block_obstacle != nullptr) {
        bool is_stacker = block_obstacle->Perception().type() ==
                              perception::PerceptionObstacle::STACKER ||
                          block_obstacle->Perception().type() ==
                              perception::PerceptionObstacle::FORKLIFT_STACKER;
        bool is_tire_lifter =
            block_obstacle->Perception().type() ==
                perception::PerceptionObstacle::WHEELCRANE;
        if (is_stacker || is_tire_lifter) {
          // if extend may cause no get stacker borrow direction.
          //extend_both_side = true;
          // AINFO<<"EXTEND BOTH SIDE FOR STACKER";
        }
      }
    }
  }

  *curr_right_bound_lane =
      -curr_lane_right_width -
      ((LaneBorrowInfo::RIGHT_BORROW == lane_borrow_info ||
        (reference_line_info_->IsLeftLaneChange() && IsWillPassMergeLane()))
           ? curr_neighbor_lane_width
           : 0.0);
  if (LaneBorrowInfo::LEFT_BORROW == lane_borrow_info) {
    *curr_left_bound_lane = curr_road_left_width - half_width ;
    *curr_right_bound_lane = -curr_road_right_width + half_width;
    // if (extend_both_side) {
      //double right_width = -adc_lane_width_ * 0.5 + half_width;
      auto adc_center_l = (reference_line_info_->AdcSlBoundary().start_l() +
                           reference_line_info_->AdcSlBoundary().end_l()) *
                          0.5;
      *curr_right_bound_lane = std::min(*curr_right_bound_lane, adc_center_l);
      // *curr_right_bound_lane = -curr_road_right_width;
    // }
  }
  if (LaneBorrowInfo::RIGHT_BORROW == lane_borrow_info) {
    *curr_left_bound_lane = curr_road_left_width - half_width ;
    *curr_right_bound_lane = -curr_road_right_width + half_width;
    // if (extend_both_side) {
      //double left_width = adc_lane_width_ * 0.5 - half_width;
      auto adc_center_l = (reference_line_info_->AdcSlBoundary().start_l() +
                           reference_line_info_->AdcSlBoundary().end_l()) *
                          0.5;
      *curr_left_bound_lane = std::max(*curr_left_bound_lane, adc_center_l);
      // *curr_left_bound_lane = curr_road_left_width;
    // }
  }

  if (vaild_lane_change_path_bound && !IsNeedLaneChangePassMergeLane()) {
    double start_point_s = lane_change_start_point.s;
    double end_point_s = lane_change_end_point.s;
    if (PathBoundsDeciderConfig::CUBIC ==
        config.lane_change_path_bound_method()) {
      if (curr_s < start_point_s + kEpsilon) {
        *curr_left_bound_lane =
            std::min(*curr_left_bound_lane, left_path_res.Evaluate(0, 0.0));
        *curr_right_bound_lane =
            std::max(*curr_right_bound_lane, right_path_res.Evaluate(0, 0.0));
        ADEBUG << "[CUBIC] curr_s < start_point_s: " << start_point_s
               << ", curr_left_bound_lane: " << *curr_left_bound_lane
               << ", curr_right_bound_lane: " << *curr_right_bound_lane;
      } else if (curr_s < end_point_s - kEpsilon) {
        *curr_left_bound_lane =
            std::min(*curr_left_bound_lane,
                     left_path_res.Evaluate(0, curr_s - start_point_s));
        *curr_right_bound_lane =
            std::max(*curr_right_bound_lane,
                     right_path_res.Evaluate(0, curr_s - start_point_s));
        ADEBUG << "curr_s < end_point_s: " << end_point_s
               << ", curr_left_bound_lane: " << *curr_left_bound_lane
               << ", curr_right_bound_lane: " << *curr_right_bound_lane;
      }
    } else if (PathBoundsDeciderConfig::LINEAR_LINE ==
               config.lane_change_path_bound_method()) {
      if (curr_s < end_point_s - kEpsilon) {
        *curr_left_bound_lane = InterpolationLookUp(
            curr_s, start_point_s, end_point_s, lane_change_start_point.l_max,
            lane_change_end_point.l_max);
        *curr_right_bound_lane = InterpolationLookUp(
            curr_s, start_point_s, end_point_s, lane_change_start_point.l_min,
            lane_change_end_point.l_min);
      }
      ADEBUG << "[LINEAR] curr_s: " << curr_s
             << ", curr_left_bound_lane: " << *curr_left_bound_lane
             << ", curr_right_bound_lane: " << *curr_right_bound_lane
             << ", width: " << (*curr_left_bound_lane - *curr_right_bound_lane);
    } else if (PathBoundsDeciderConfig::PRE_FORWARD ==
               config.lane_change_path_bound_method()) {
      if (curr_s < start_point_s + kEpsilon) {
        *curr_left_bound_lane = lane_change_start_point.l_max;
        *curr_right_bound_lane = lane_change_start_point.l_min;
      }
      ADEBUG << "[PRE] curr_s: " << curr_s
             << ", curr_left_bound_lane: " << *curr_left_bound_lane
             << ", curr_right_bound_lane: " << *curr_right_bound_lane;
    }
  }
}

// is keep right
bool PathBoundsDecider::IsNeedKeepRight(
    const ReferenceLineInfo& reference_line_info) {
  const auto& reference_line = reference_line_info.reference_line();
  const auto& path_decision = reference_line_info.path_decision();
  const auto& indexed_obstacles = path_decision.obstacles();
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double half_length = veh_param.length() * 0.5;
  double half_width = veh_param.width() * 0.5;
  bool has_reverse_car = false;
  double nearst_obstacle_s = std::numeric_limits<double>::max();

  // The ADC is approaching destination.
  const auto* destination_obstacle =
      reference_line_info.path_decision().Find(FLAGS_destination_obstacle_id);
  if (nullptr != destination_obstacle) {
    const auto& obs_sl = destination_obstacle->PerceptionSLBoundary();
    double longitudinal_distance =
        obs_sl.start_s() - adc_frenet_s_ - half_length;
    if (reference_line.IsOnLane(obs_sl) && longitudinal_distance > 0.0 &&
        longitudinal_distance < kNearDestinationDistance) {
      return false;
    }
  }
  auto& reverse_id = injector_->reverse_obj_;
  for (const auto* obstacle : indexed_obstacles.Items()) {
    if (nullptr == obstacle || obstacle->IsVirtual() || obstacle->IsStatic()) {
      ADEBUG << "Obstacle pointer is invalid.";
      continue;
    }
    // skip back obstacle
    if (obstacle->PerceptionSLBoundary().end_s() <
        adc_frenet_s_ - half_length) {
      continue;
    }
    // skip no lateral overlap obstacle
    const auto& obs_sl = obstacle->PerceptionSLBoundary();
    bool no_lateral_overlap =
        adc_frenet_l_ + half_width + kLateralOverlapBuffer < obs_sl.start_l() ||
        adc_frenet_l_ - half_width - kLateralOverlapBuffer > obs_sl.end_l();
    if (no_lateral_overlap) {
      continue;
    }

    // skip obstacle whicle no vehicle type
    if (PerceptionObstacle::VEHICLE != obstacle->Perception().type()) {
      continue;
    }

    const double obs_theta = obstacle->Perception().theta();
    const double obs_center_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
    const auto& obs_reference_point =
        reference_line.GetNearestReferencePoint(obs_center_s);
    double theta_diff_obs_and_ref = century::common::math::NormalizeAngle(
        obs_theta - obs_reference_point.heading());
    double theta_diff_obs_and_adc =
        century::common::math::NormalizeAngle(obs_theta - adc_theta_);
    double buffer_radian = kHalfReserveAngle / 90.0 * M_PI_2;
    if (theta_diff_obs_and_ref < -M_PI_2 - buffer_radian ||
        theta_diff_obs_and_ref > M_PI_2 + buffer_radian ||
        theta_diff_obs_and_adc < -M_PI_2 - buffer_radian ||
        theta_diff_obs_and_adc > M_PI_2 + buffer_radian) {
      has_reverse_car = true;
    }
    if (obs_center_s < nearst_obstacle_s) {
      nearst_obstacle_s = obs_center_s;
      nearst_obs_s_ = nearst_obstacle_s;
      injector_->reverse_obstacle_start_l_ = obs_sl.start_l();
      reverse_id.first = obstacle->Id();
      reverse_id.second = obstacle->PerceptionId();
    }
  }
  if (has_reverse_car) {
    AINFO << "[dzq] reverse id:" << reverse_id.first << reverse_id.second;
    return true;
  }

  return false;
}
// keep right for mixed flow
bool PathBoundsDecider::IsNeedKeepRightForMixedFlow(
    const ReferenceLineInfo& reference_line_info) {
  const auto& reference_line = reference_line_info.reference_line();
  const auto& indexed_obstacles =
      reference_line_info.path_decision().obstacles();
  double half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  double nearst_obstacle_s = std::numeric_limits<double>::max();
  bool is_reverse_obs_online = false;
  bool is_reverse_left_obs_online = false;
  // The ADC is approaching destination.
  const auto* destination_obstacle =
      reference_line_info.path_decision().Find(FLAGS_destination_obstacle_id);
  if (nullptr != destination_obstacle) {
    const auto& obs_sl = destination_obstacle->PerceptionSLBoundary();
    double longitudinal_distance =
        obs_sl.start_s() - reference_line_info.AdcSlBoundary().end_s();
    if (reference_line.IsOnLane(obs_sl) && longitudinal_distance > 0.0 &&
        longitudinal_distance < kNearDestinationDistance) {
      return false;
    }
  }

  for (const auto* obstacle : indexed_obstacles.Items()) {
    if (!NeedConsiderReverseObstacle(reference_line_info, obstacle)) {
      continue;
    }
    is_reverse_obs_online = true;
    const auto& obs_sl = obstacle->PerceptionSLBoundary();
    const double obs_center_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
    bool no_lateral_overlap =
        adc_frenet_l_ + half_width + kLateralOverlapBuffer < obs_sl.start_l() ||
        adc_frenet_l_ - half_width - kLateralOverlapBuffer > obs_sl.end_l();
    bool is_right_obs = obs_sl.end_l() < adc_frenet_l_;

    if (injector_->is_need_to_keep_right_) {
      ADEBUG << "in keep right ";
      ADEBUG << "!is_right_obs = " << !is_right_obs;
      ADEBUG << "is_reverse_obs_online = " << is_reverse_obs_online;
      if (!is_right_obs && is_reverse_obs_online) {
        is_reverse_left_obs_online = true;
        if (obs_center_s < nearst_obstacle_s) {
          nearst_obstacle_s = obs_center_s;
          nearst_obs_s_ = nearst_obstacle_s;
          injector_->reverse_obstacle_start_l_ = obs_sl.start_l();
        }
        return true;
      }
    } else {
      // first check
      if (no_lateral_overlap || is_right_obs) {
        continue;
      }
    }
    is_reverse_left_obs_online = true;
    if (obs_center_s < nearst_obstacle_s) {
      nearst_obstacle_s = obs_center_s;
      nearst_obs_s_ = nearst_obstacle_s;
      injector_->reverse_obstacle_start_l_ = obs_sl.start_l();
    }
  }
  ADEBUG << "is_reverse_left_obs_online = " << is_reverse_left_obs_online;
  if (is_reverse_left_obs_online) {
    return true;
  }
  // stability check
  return false;
}

bool PathBoundsDecider::NeedConsiderReverseObstacle(
    const ReferenceLineInfo& reference_line_info,
    const Obstacle* const obstacle) {
  if (nullptr == obstacle || obstacle->IsVirtual() || obstacle->IsStatic()) {
    return false;
  }
  // skip back obstacle
  if (obstacle->PerceptionSLBoundary().end_s() <
      reference_line_info.AdcSlBoundary().start_s()) {
    return false;
  }
  const auto& obs_sl = obstacle->PerceptionSLBoundary();
  if (!reference_line_info.reference_line().IsOnLane(obs_sl)) {
    return false;
  }
  const double obs_center_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
  const auto& obs_reference_point =
      reference_line_info.reference_line().GetNearestReferencePoint(
          obs_center_s);
  const auto& velocity = obstacle->Perception().velocity();
  const double obstacle_longitudinal_speed =
      common::math::Vec2d::CreateUnitVec2d(obs_reference_point.heading())
          .InnerProd(Vec2d(velocity.x(), velocity.y()));

  if (obstacle_longitudinal_speed > kMinReverseSpeed) {
    return false;
  }

  // out length no consider
  double consider_length =
      std::fabs(obstacle_longitudinal_speed) * kTrajectoryTime +
      adc_v_ * kTrajectoryTime;
  if (obs_sl.start_s() - reference_line_info_->AdcSlBoundary().end_s() >
      consider_length) {
    return false;
  }
  return true;
}

// update boundaries with corresponding one-side lane boundary for pull over
// (1) use left lane boundary for normal PULL_OVER type
// (2) use left/right(which is opposite to pull over direction
//     (pull over at closer road side) lane boundary for EMERGENCY_PULL_OVER
void PathBoundsDecider::UpdatePullOverBoundaryByLaneBoundary(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  const auto pull_over_type = pull_over_status.pull_over_type();
  if (PullOverStatus::PULL_OVER != pull_over_type &&
      PullOverStatus::EMERGENCY_PULL_OVER != pull_over_type) {
    return;
  }

  for (size_t i = 0; i < path_bound->size(); ++i) {
    const double curr_s = (*path_bound)[i].s;
    double left_bound = 3.0;
    double right_bound = 3.0;
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                    &curr_lane_right_width)) {
      double offset_to_lane_center = 0.0;
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      left_bound = curr_lane_left_width + offset_to_lane_center;
      right_bound = curr_lane_right_width + offset_to_lane_center;
    }
    ADEBUG << "left_bound[" << left_bound << "] right_bound[" << right_bound
           << "]";
    if (PullOverStatus::PULL_OVER == pull_over_type) {
      (*path_bound)[i].l_max = left_bound;
    } else if (PullOverStatus::EMERGENCY_PULL_OVER == pull_over_type) {
      // TODO(all): use left/right lane boundary accordingly
      (*path_bound)[i].l_max = left_bound;
    }
  }
}

void PathBoundsDecider::ConvertBoundarySAxisFromLaneCenterToRefLine(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  for (size_t i = 0; i < path_bound->size(); ++i) {
    // 1. Get road boundary.
    double curr_s = (*path_bound)[i].s;
    double refline_offset_to_lane_center = 0.0;
    reference_line.GetOffsetToMap(curr_s, &refline_offset_to_lane_center);
    (*path_bound)[i].l_min -= refline_offset_to_lane_center;
    (*path_bound)[i].l_max -= refline_offset_to_lane_center;
  }
}

double PathBoundsDecider::GetLaneChangeAdcBuffer(
    const ReferenceLineInfo& reference_line_info) {
  const auto& config = config_.path_bounds_decider_config();
  double lane_change_buffer = 0.0;
  if (reference_line_info.IsChangeLanePath()) {
    const auto& adc_sl = reference_line_info.AdcSlBoundary();
    double adc_center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
    if (adc_center_l < 0.0) {
      lane_change_buffer = -config.min_include_adc_path_bound_buffer();
    } else {
      lane_change_buffer = config.min_include_adc_path_bound_buffer();
    }
  }
  return lane_change_buffer;
}

void PathBoundsDecider::GetBoundaryFromLaneChangeForbiddenZone(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  if (IsNeedLaneChangePassMergeLane()) {
    GenerateLaneChangePassMergeLanePathBound(reference_line_info, path_bound);
    AINFO << "generate lane change pass merge lane path bound";
    return;
  }

  if (!NeedStopChangingLane(reference_line_info)) {
    ADEBUG << "NO NeedStopChangingLane";
    return;
  }

  into_lane_change_ = false;
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double half_width = veh_param.width() * 0.5;
  double ADC_speed_buffer =
      adc_frenet_ld_ * adc_frenet_ld_ /
      config_.path_bounds_decider_config().max_lateral_acceleration() * 0.5;
  const auto& adc_sl = reference_line_info_->AdcSlBoundary();
  double adc_center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;
  bool turn_left = (adc_center_l < 0.0);

  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = (*path_bound)[i].s;
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    double offset_to_map = 0.0;
    reference_line.GetOffsetToMap(curr_s, &offset_to_map);
    if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                    &curr_lane_right_width)) {
      double offset_to_lane_center = 0.0;
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;
    }
    curr_lane_left_width -= offset_to_map;
    curr_lane_right_width += offset_to_map;

    double two_lane_middle_l =
        turn_left ? -curr_lane_right_width : curr_lane_left_width;
    if (turn_left) {
      adc_center_l = std::fmax(adc_center_l, adc_frenet_l_);
      if (adc_center_l < two_lane_middle_l - half_width) {
        (*path_bound)[i].l_max = std::fmin(two_lane_middle_l - half_width,
                                           adc_center_l + ADC_speed_buffer);
      } else if (adc_center_l < two_lane_middle_l) {
        (*path_bound)[i].l_max =
            std::fmin(two_lane_middle_l, adc_center_l + ADC_speed_buffer);
      } else {
        (*path_bound)[i].l_max =
            std::fmin((*path_bound)[i].l_max, adc_center_l);
      }
      (*path_bound)[i].l_min =
          std::fmin(adc_center_l - kLateralSafeBuffer, (*path_bound)[i].l_min);
    } else {
      adc_center_l = std::fmin(adc_center_l, adc_frenet_l_);
      if (adc_center_l > two_lane_middle_l + half_width) {
        (*path_bound)[i].l_min = std::fmax(two_lane_middle_l + half_width,
                                           adc_center_l - ADC_speed_buffer);
      } else if (adc_center_l > two_lane_middle_l) {
        (*path_bound)[i].l_min =
            std::fmax(two_lane_middle_l, adc_center_l - ADC_speed_buffer);
      } else {
        (*path_bound)[i].l_min =
            std::fmax((*path_bound)[i].l_min, adc_center_l);
      }
      (*path_bound)[i].l_max =
          std::fmax(adc_center_l + kLateralSafeBuffer, (*path_bound)[i].l_max);
    }
  }
}

void PathBoundsDecider::GenerateLaneChangePassMergeLanePathBound(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  double ADC_speed_buffer =
      adc_frenet_ld_ * adc_frenet_ld_ /
      config_.path_bounds_decider_config().max_lateral_acceleration() * 0.5;
  const auto& adc_sl = reference_line_info_->AdcSlBoundary();
  double adc_center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;

  for (size_t i = 0; i < path_bound->size(); ++i) {
    if ((i * kPathBoundsDeciderResolution) <
        path_decider_status.merge_lane_remain_dis()) {
      if (reference_line_info.IsLeftLaneChange()) {
        (*path_bound)[i].l_max =
            std::fmin(adc_center_l + ADC_speed_buffer, (*path_bound)[i].l_max);
      } else if (reference_line_info.IsRightLaneChange()) {
        (*path_bound)[i].l_min =
            std::fmax(adc_center_l - ADC_speed_buffer, (*path_bound)[i].l_min);
      }
    } else {
      break;
    }
  }
}

bool PathBoundsDecider::NeedStopChangingLane(
    const ReferenceLineInfo& reference_line_info) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  // If there is a pre-determined lane-change starting position, then use it;
  // otherwise, decide one.
  auto* lane_change_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_change_lane();
  auto& overtake_info =
      injector_->planning_context()->planning_status().overtake();
  const auto& config = config_.path_bounds_decider_config();
  if (lane_change_status->is_clear_to_change_lane() &&
      !overtake_info.stop_change_lane()) {
    ++lane_change_safe_count_;
    if (lane_change_safe_count_ > config.keep_hold_safe_check_times()) {
      ADEBUG << "Current position is clear to change lane. No need prep s.";
      lane_change_status->set_exist_lane_change_start_position(false);
      return false;
    }
  } else {
    lane_change_safe_count_ = 0;
    lane_change_status->set_is_success_change_lane_path(false);
  }
  double lane_change_start_s = 0.0;
  if (lane_change_status->exist_lane_change_start_position()) {
    common::SLPoint point_sl;
    reference_line.XYToSL(lane_change_status->lane_change_start_position(),
                          &point_sl);
    lane_change_start_s = point_sl.s();
  } else {
    // TODO(jiacheng): train ML model to learn this.
    if (injector_->planning_context()
            ->planning_status()
            .overtake()
            .urgency_lane_change()) {
      const auto& adc_v =
          reference_line_info_->vehicle_state().linear_velocity();
      lane_change_start_s =
          std::fabs(adc_v * adc_v / config.forbidden_zone_use_deceleration() *
                    0.5) +
          adc_frenet_s_;
    } else {
      lane_change_start_s = FLAGS_lane_change_prepare_length + adc_frenet_s_;
    }

    // Update the decided lane_change_start_s into planning-context.
    common::SLPoint lane_change_start_sl;
    lane_change_start_sl.set_s(lane_change_start_s);
    lane_change_start_sl.set_l(0.0);
    common::math::Vec2d lane_change_start_xy;
    reference_line.SLToXY(lane_change_start_sl, &lane_change_start_xy);
    lane_change_status->set_exist_lane_change_start_position(true);
    lane_change_status->mutable_lane_change_start_position()->set_x(
        lane_change_start_xy.x());
    lane_change_status->mutable_lane_change_start_position()->set_y(
        lane_change_start_xy.y());
  }

  // Remove the target lane out of the path-boundary, up to the decided S.
  if (lane_change_start_s < adc_frenet_s_) {
    // If already passed the decided S, then return.
    // lane_change_status->set_exist_lane_change_start_position(false);
    return false;
  }
  return true;
}

// Currently, it processes each obstacle based on its frenet-frame
// projection. Therefore, it might be overly conservative when processing
// obstacles whose headings differ from road-headings a lot.
// TODO(all): (future work) this can be improved in the future.
bool PathBoundsDecider::GetBoundaryFromStaticObstacles(
    const PathDecision& path_decision, const LaneBorrowInfo& lane_borrow_info,
    PathBound* const path_boundaries, std::string* const blocking_obstacle_id) {
  // Preprocessing.
  auto indexed_obstacles = path_decision.obstacles();
  std::vector<ObstacleEdge> obstacles_edges;
  // according obs boudnary build path_boundary
  SortObstaclesForSweepLine(indexed_obstacles, lane_borrow_info,
                            &obstacles_edges);
  ADEBUG << "There are " << obstacles_edges.size() << " obstacles.";
  ADEBUG << "Track LIST SIZE: " << track_obstacle_edge_.size();
  int path_blocked_idx = -1;
  UpdatePathBoundBaseOnObstacleEdge(obstacles_edges, path_boundaries,
                                    &path_blocked_idx, blocking_obstacle_id,
                                    lane_borrow_info);
  if (LaneBorrowInfo::NO_BORROW != lane_borrow_info) {
    if (path_blocked_idx != -1) {
      AINFO << "PATH is blocked , need rerouting";
      injector_->path_block_count_++;
      injector_->path_block_count_ = std::min(200,injector_->path_block_count_);
    }else{
      injector_->path_block_count_ = 0;
    }
  }

  TrimPathBounds(path_blocked_idx, path_boundaries);

  return true;
}

void PathBoundsDecider::UpdatePathBoundBaseOnObstacleEdge(
    const std::vector<ObstacleEdge>& obstacles_edges,
    PathBound* const path_boundaries, int* const path_blocked_idx,
    std::string* const blocking_obstacle_id,
    const LaneBorrowInfo& lane_borrow_info) {
  double center_line = adc_frenet_l_;
  size_t obs_idx = 0;
  std::multiset<double, std::greater<double>> right_bounds;
  right_bounds.insert(std::numeric_limits<double>::lowest());
  std::multiset<double> left_bounds;
  left_bounds.insert(std::numeric_limits<double>::max());
  // Maps obstacle ID's to the decided ADC pass direction, if ADC should
  // pass from left, then true; otherwise, false.
  std::unordered_map<std::string, bool> obs_id_to_direction;
  // Step through every path point.

  // for(auto edg:obstacles_edges){
  //   AINFO<<"edg = "<<edg.s<<"     "<<edg.l_min<<"     "<<edg.l_max;
  // }
  // update second point ,so first is init boundary
  for (size_t i = 1; i < path_boundaries->size(); ++i) {
    double curr_s = (*path_boundaries)[i].s;
    // Check and see if there is any obstacle change:
    if (obs_idx < obstacles_edges.size() &&
        obstacles_edges[obs_idx].s < curr_s) {
      while (obs_idx < obstacles_edges.size() &&
             obstacles_edges[obs_idx].s < curr_s) {
        ChooseCurrentPathBoundDirection(obstacles_edges[obs_idx],
                                        &(*path_boundaries)[i], &left_bounds,
                                        &right_bounds, &obs_id_to_direction,
                                        &center_line, lane_borrow_info);

        if (!UpdateAndCheckPathBoundary(*left_bounds.begin(),
                                        *right_bounds.begin(),
                                        &(*path_boundaries)[i], &center_line)) {
          ADEBUG << "[1] Path is blocked at s = " << curr_s;
          *path_blocked_idx = static_cast<int>(i);
          if (!obs_id_to_direction.empty()) {
            *blocking_obstacle_id = obstacles_edges[obs_idx].obstacle_id;
            ADEBUG << "[1] blocking_obstacle_id: " << *blocking_obstacle_id;
          }
          min_lateral_diff_ = 0.0;
          break;
        }
        // AINFO<<"(*path_boundaries)[i].MAX-MIN
        // ="<<(*path_boundaries)[i].l_max<<"   "
        // <<(*path_boundaries)[i].l_min;
        ++obs_idx;
      }
    } else {
      // If no obstacle change, update the bounds and center_line.
      if (!UpdateAndCheckPathBoundary(*left_bounds.begin(),
                                      *right_bounds.begin(),
                                      &(*path_boundaries)[i], &center_line) &&
          !injector_->is_adc_in_gate_junction_) {
        ADEBUG << "[2] Path is blocked at s = " << curr_s;
        *path_blocked_idx = static_cast<int>(i);
        if (!obs_id_to_direction.empty()) {
          *blocking_obstacle_id = obs_id_to_direction.begin()->first;
          ADEBUG << "[2] blocking_obstacle_id: " << *blocking_obstacle_id;
        }
        min_lateral_diff_ = 0.0;
        break;
      }
    }
    min_lateral_diff_ =
        std::fmin(min_lateral_diff_,
                  (*path_boundaries)[i].l_max - (*path_boundaries)[i].l_min);
    // Early exit if path is blocked.
    if (-1 != *path_blocked_idx) {
      if (REGULAR_PATH_BOUNDS == bounds_type_) {
        // cause no borrow
        // maybe self path is block.need check borrow block
        // auto* mutable_path_decider_status = injector_->planning_context()
        //                                         ->mutable_planning_status()
        //                                         ->mutable_path_decider();
        // mutable_path_decider_status->set_adc_will_be_blocked(true);
      }
      break;
    }
  }
}

void PathBoundsDecider::ChooseCurrentPathBoundDirection(
    const ObstacleEdge& obstacle_edge, PathBoundPoint* const path_bound_point,
    std::multiset<double>* const left_bounds,
    std::multiset<double, std::greater<double>>* const right_bounds,
    std::unordered_map<std::string, bool>* const direction,
    double* const center_line, const LaneBorrowInfo& lane_borrow_info) {
  if (obstacle_edge.is_start_s) {
    // same obstacle only add once.
    if (direction->find(obstacle_edge.obstacle_id) != direction->end()) {
      return;
    }

    // A new obstacle enters into our scope:
    //   - Decide which direction for the ADC to pass.
    //   - Update the left/right bound accordingly.
    //   - If boundaries blocked, then decide whether can side-pass.
    //   - If yes, then borrow neighbor lane to side-pass.
    // chenck borrow direction.
    if (DirectionDecision::LEFT == obstacle_edge.direction_decision) {
      // AINFO<<"DirectionDecision::LEFT";
      (*direction)[obstacle_edge.obstacle_id] = true;
      right_bounds->insert(obstacle_edge.l_max);
    } else if (DirectionDecision::RIGHT == obstacle_edge.direction_decision) {
      // AINFO<<"DirectionDecision::RIGHT";
      (*direction)[obstacle_edge.obstacle_id] = false;
      left_bounds->insert(obstacle_edge.l_min);
    } else {
      // AINFO<<"NO set direction";
      // AINFO<<"*center_line = "<<*center_line;
      if (obstacle_edge.l_min + obstacle_edge.l_max < (*center_line) * 2.0) {
        // Obstacle is to the right of center-line, should pass from left.
        // AINFO<<"Obstacle is to the right of center-line, should pass from left";
        (*direction)[obstacle_edge.obstacle_id] = true;
        right_bounds->insert(obstacle_edge.l_max);
      } else {
        // Obstacle is to the left of center-line, should pass from right.
        // AINFO<<"Obstacle is to the left of center-line, should pass from right.";
        (*direction)[obstacle_edge.obstacle_id] = false;
        left_bounds->insert(obstacle_edge.l_min);
      }
    }
    UpdatePathBoundaryAndCenterLineWithBuffer(
        path_bound_point, *(left_bounds->begin()), *(right_bounds->begin()),
        center_line, lane_borrow_info);
  } else if (direction->find(obstacle_edge.obstacle_id) != direction->end()) {
    // An existing obstacle exits our scope.
    if ((*direction)[obstacle_edge.obstacle_id]) {
      if (right_bounds->find(obstacle_edge.l_max) != right_bounds->end()) {
        right_bounds->erase(right_bounds->find(obstacle_edge.l_max));
      }
    } else {
      if (left_bounds->find(obstacle_edge.l_min) != left_bounds->end()) {
        left_bounds->erase(left_bounds->find(obstacle_edge.l_min));
      }
    }
    direction->erase(obstacle_edge.obstacle_id);
  }
}

bool PathBoundsDecider::UpdateAndCheckPathBoundary(
    double left_bound, double right_bound,
    PathBoundPoint* const path_bound_point, double* const center_line) {
  // Update the bounds and center_line.
  path_bound_point->l_min =
      std::fmax(path_bound_point->l_min,
                right_bound + GetBufferBetweenADCCenterAndEdge());
  path_bound_point->l_max = std::fmin(
      path_bound_point->l_max, left_bound - GetBufferBetweenADCCenterAndEdge());
  double diff_l = path_bound_point->l_max - path_bound_point->l_min;
  const auto& config = config_.path_bounds_decider_config();
  bool has_lat_decision = util::IsLaneBorrow(injector_->planning_context()) ||
                          util::IsLaneChange(injector_->planning_context());

  if (diff_l <= -config.fix_lateral_buffer_first_level()) {
    return false;
  } else if (!has_lat_decision && diff_l < kMinPathBoundSpace) {
    auto* mutable_path_decider_status = injector_->planning_context()
                                            ->mutable_planning_status()
                                            ->mutable_path_decider();
    if (mutable_path_decider_status->adc_enter_narrow_area()) {
      return false;
    }

    path_bound_point->l_max += config.fix_lateral_buffer_first_level() * 0.5;
    path_bound_point->l_min -= config.fix_lateral_buffer_first_level() * 0.5;
    if (path_bound_point->l_max - path_bound_point->l_min < kEpsilon) {
      return false;
    }
  }
  *center_line = (path_bound_point->l_min + path_bound_point->l_max) * 0.5;
  return true;
}

void PathBoundsDecider::SortObstaclesForSweepLine(
    const IndexedList<std::string, Obstacle>& indexed_obstacles,
    const LaneBorrowInfo& lane_borrow_info,
    std::vector<ObstacleEdge>* const obstacles_edges) {
  CHECK_NOTNULL(obstacles_edges);

  // Go through every obstacle (including obstacles that
  // disappear briefly or change from static to dynamic)
  // and preprocess it.
  auto obstacles_list = indexed_obstacles.Items();
  for (const auto& disappear_obs : injector_->GetDisappearObstacles()) {
    obstacles_list.emplace_back(&disappear_obs.second);
  }
  UpdateStartMovingObstacles(*reference_line_info_);
  for (const auto& moving_obs : injector_->GetStartMovingObstacles()) {
    obstacles_list.emplace_back(&moving_obs.second.second);
  }
  const auto& config = config_.path_bounds_decider_config();
  double obstacle_lat_buffer = config.min_safe_obstacle_lateral_buffer();
  double obstacle_lon_start_buffer = config.obstacle_lon_start_safe_buffer();
  double obstacle_lon_end_buffer = config.obstacle_lon_end_safe_buffer();
  if (util::IsNeedLoosenPathConstrains(injector_->planning_context())) {
    obstacle_lon_start_buffer = config.narrow_area_lon_start_safe_buffer();
    obstacle_lon_end_buffer = config.narrow_area_lon_end_safe_buffer();
  }

  double not_passable_s =
      reference_line_info_->reference_line().Length() + kEpsilon;
  bool keep_self_path_straight = false;
  for (const auto* obstacle : obstacles_list) {
    if (nullptr == obstacle) {
      AERROR << "Obstacle pointer is null.";
      continue;
    }
     AINFO<<"obstacle = "<<obstacle->Id();
    // obs buffer
    double lateral_buffer = obstacle_lat_buffer;
      const auto obs_type = obstacle->Perception().type();
  if ((perception::PerceptionObstacle::VEHICLE == obs_type &&
       obstacle->IsStatic()) ||
      perception::PerceptionObstacle::STACKER == obs_type ||
      perception::PerceptionObstacle::FORKLIFT_STACKER == obs_type) {
    lateral_buffer =
        lateral_buffer + FLAGS_car_type_lateral_buffer+kLatBoundaryBuffer;
  }
  bool is_tire_lifter = obstacle->Perception().type() ==
                        perception::PerceptionObstacle::WHEELCRANE;
  if (is_tire_lifter) {
    // todo tire_lifer collision
    lateral_buffer =
        lateral_buffer + FLAGS_car_type_lateral_buffer+kLatBoundaryBuffer;
   obstacle_lon_start_buffer = kLonBufferForWheelcrane;
  }

  if (FLAGS_allow_narrow_pass && injector_->is_adc_in_gate_junction_) {
    lateral_buffer = 0.0;
  }
  // use st boundary collision check to make decision
  if(injector_->enable_shrink_collision_buffer_){
    lateral_buffer = 0.0;
  }
    if (!FLAGS_enable_anchor_lane_change_path &&
        NeedUpdateOvertakeObstacleEdge(obstacle)) {
      UpdateOvertakenObstacleEdge(
          obstacle, obstacles_edges, lateral_buffer,
          config.narrow_area_lon_start_safe_buffer(), 0.0);
      continue;
    }
    if (CheckObsIsCanBeIgnored(lane_borrow_info, obstacle, &not_passable_s,
                               &keep_self_path_straight)) {
      auto* obs_info =
          reference_line_info_->path_decision()->Find(obstacle->Id());
      if (obs_info) {
        obs_info->SetCanPass(false);
      }
      continue;
    }
    DirectionDecision decision = DirectionDecision::NONE;
    // make borrow direction
    MakeObsDirectionDecision(lane_borrow_info,obstacle, &decision);

    // adc in turn or obs in turn ,need large lateral buffer.
    // adc use eight model.
    const auto& obs_sl = obstacle->PerceptionSLBoundary();
    double strat_s = obs_sl.start_s();
    double end_s = obs_sl.end_s();
    bool is_straight_lane = true;
    const auto& reference_points =
        reference_line_info_->reference_line().GetReferencePoints(strat_s,
                                                                  end_s);
    for (const auto& ref_point : reference_points) {
      // ADEBUG << "ref_point kappa = " << ref_point.kappa();
      if (std::fabs(ref_point.kappa()) >
          config_.path_bounds_decider_config().keep_right_road_min_kappa()) {
        is_straight_lane = false;
        break;
      }
    }
    if (!is_straight_lane || reference_line_info_->IsNearTurn()) {
      lateral_buffer = FLAGS_borrow_lateral_buffer_in_turn;
    }

    // obstacle_lon_start_buffer = 3.0;
    // obstacle_lon_end_buffer = 3.0;
    UpdateTrackObstaclesEdge(decision, obstacle, obstacles_edges,
                             lateral_buffer, obstacle_lon_start_buffer,
                             obstacle_lon_end_buffer);
  }
  if (keep_self_path_straight) {
    UpdateKeepSelfPathStraightCount();
  }
  TrimTrackObstaclesEdge();

  // Sort.
  std::sort(obstacles_edges->begin(), obstacles_edges->end(),
            [](const ObstacleEdge& lhs, const ObstacleEdge& rhs) {
              return lhs.s != rhs.s ? lhs.s < rhs.s
                                    : lhs.is_start_s > rhs.is_start_s;
            });
}

bool PathBoundsDecider::NeedUpdateOvertakeObstacleEdge(
    const Obstacle* obstacle) {
  if (!util::IsLaneChange(injector_->planning_context())) {
    return false;
  }
  if (!reference_line_info_->IsAdcOnReferenceLine()) {
    ADEBUG << "adc is still located in origin lane, not exclude overtaken obs";
    return false;
  }
  auto* mutable_overtake = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_overtake();
  bool is_overtake_obs =
      (config_.path_bounds_decider_config()
           .enable_exclude_overtake_obstacle() &&
       reference_line_info_->IsChangeLanePath() &&
       ((obstacle->Id() == mutable_overtake->overtake_obstacle_id()) ||
        (obstacle->PerceptionId() ==
         mutable_overtake->overtake_obstacle_perception_id())));
  if (!is_overtake_obs) {
    return false;
  }
  auto& overtake_status =
      injector_->planning_context()->planning_status().overtake();
  if (overtake_status.urgency_lane_change()) {
    return false;
  }
  if (obstacle->PerceptionSLBoundary().end_s() <
      reference_line_info_->AdcSlBoundary().start_s()) {
    return false;
  }
  return true;
}
bool PathBoundsDecider::IsHigherIGV(const Obstacle& obstacle) {
  if(!FLAGS_allow_skip_higher_obs){
    return false;
  }
  const auto obs_type = obstacle.Perception().type();
  bool is_consider_obs =
      perception::PerceptionObstacle::UNKNOWN == obs_type ||
      perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type ||
      perception::PerceptionObstacle::VEHICLE == obs_type;
  if (!is_consider_obs) {
    return false;
  }
  const auto& veh_param = VehicleConfigHelper::GetConfig().vehicle_param();
  const auto& routing = frame_->local_view().routing;
  bool is_igv_loaded = routing->routing_request().is_loading();
  // AINFO<<"is_igv_loaded = "<<is_igv_loaded;
  double height_buffer = kHeightBuffer;
  if (injector_->is_adc_in_gate_junction_) {
    height_buffer = kLowHeightBuffer;
  }
  double igv_max_height =
      veh_param.height() + kHeightofContainer + height_buffer;
  if (!is_igv_loaded) {
    igv_max_height = veh_param.height() + height_buffer;
  }
  double obs_low_height = obstacle.Perception().position().z();
  if(perception::PerceptionObstacle::VEHICLE == obs_type){
    obs_low_height = obstacle.Perception().position().z()-obstacle.Perception().height()*0.5;
  }
  AINFO << "obstacle = " << obstacle.Id();
  AINFO << "obs_low_height = " << obs_low_height;
  AINFO << "igv_max_height = " << igv_max_height;
  if (obs_low_height > igv_max_height) {
    AINFO << "obs_low_height > igv_max_height";
    return true;
  }
  return false;
}

bool PathBoundsDecider::CheckObsIsCanBeIgnored(
    const LaneBorrowInfo& lane_borrow_info, const Obstacle* obstacle,
    double* const not_passable_s, bool* const keep_self_path_straight) {
  if (injector_->is_adc_in_gate_junction_) {
    return true;
  }
  const auto& obs_sl = obstacle->PerceptionSLBoundary();
  auto igv_boundary_map = reference_line_info_->IgvBoundaryMap();
  // AINFO<<"igv_boundary_map = "<<igv_boundary_map.size();
  for (const auto& igv_boundary : igv_boundary_map) {
    const auto& vehicle_sl = igv_boundary.second;
    double consider_start_l = vehicle_sl.start_l() - kIgvlatBuffer;
    double consider_end_l = vehicle_sl.end_l() + kIgvlatBuffer;
    double consider_start_s = vehicle_sl.start_s() - kIgvlonBuffer;
    double consider_end_s = vehicle_sl.end_s() - kIgvlonBuffer;
    if (obs_sl.start_s() <= consider_end_s &&
        obs_sl.end_s() >= consider_start_s &&
        obs_sl.start_l() <= consider_end_l &&
        obs_sl.end_l() >= consider_start_l) {
      // (obs_sl.start_l() >= consider_start_l &&
      //     obs_sl.end_l() <= consider_end_l &&
      //     obs_sl.start_s() >= consider_start_s &&
      //     obs_sl.end_s() <= consider_end_s) {
      
      auto* obs_info =
          reference_line_info_->path_decision()->Find(obstacle->Id());
      if (obs_info) {
        obs_info->SetIsIgv(true);
        obs_info->SetIgvVehicleId(igv_boundary.first);
      }
      // if now need to borrow igv,no ignore.
      bool can_ignore_igv = true;
      auto planning_status =
          injector_->planning_context()->mutable_planning_status();
      if (planning_status != nullptr) {
        const auto& top_bull = planning_status->top_bull();
        if ((injector_->borrow_response().has_response() &&
             planning::ResponseType::ACCEPT ==
                 injector_->borrow_response().response_type() &&
             injector_->borrow_response().block_obs_id() ==
                 top_bull.blocking_igv_id())) {
          can_ignore_igv = false;
        }
      }

      if (((planning::ResponseType::ACCEPT ==
            injector_->borrow_response().response_type()) &&
           injector_->borrow_response().block_obs_id() == obstacle->Id()) ||
          !can_ignore_igv) {
        return false;
      } else {
        return true;
      }
    }
  }
  if(IsHigherIGV(*obstacle)){
    AINFO<<"obs "<< obstacle->Id() << " is higher igv,can skip";
          auto* obs_info =
          reference_line_info_->path_decision()->Find(obstacle->Id());
      if (obs_info) {
        obs_info->SetIsHigherObs(true);
      }
    return true;
  }
  if ((obstacle->Perception().type() == PerceptionObstacle::STACKER ||
       obstacle->Perception().type() == PerceptionObstacle::FORKLIFT_STACKER)) {
    if (injector_->pass_stacker_response().pass_stacker_response_type() !=
            planning::PassStackerResponseType::PASS &&
        (FLAGS_enable_use_pass_stacker ||
         FLAGS_enable_use_pass_stacker_with_perception)) {
      return true;
    }
  }
  //     std::fabs(injector_->vehicle_state()->linear_velocity());
  double diff_distance =
      obs_sl.start_s() - reference_line_info_->AdcSlBoundary().end_s();
  // low speed ,long length adc ,so
  // AINFO << "obstacle = " << obstacle->Id();
  double max_valid_perception_distance = FLAGS_max_valid_perception_distance;
  if (obstacle->Perception().type() == PerceptionObstacle::WHEELCRANE) {
    max_valid_perception_distance = FLAGS_wheelcrane_consider_distance;
  }
  if (diff_distance > max_valid_perception_distance ||
      (!IsWithinPathDeciderScopeObstacle(*obstacle) &&
       !obstacle->IsStaticToDynamic()) ||
      NoNeedPassStaticObstacle(obstacle, not_passable_s)) {
    // AINFO<<"NoNeedPassStaticObstacle(obstacle, not_passable_s) =
    // "<<NoNeedPassStaticObstacle(obstacle, not_passable_s);
    // AINFO<<"!obstacle->IsStaticToDynamic() =
    // "<<!obstacle->IsStaticToDynamic();
    // AINFO<<"!IsWithinPathDeciderScopeObstacle(*obstacle)  =
    // "<<!IsWithinPathDeciderScopeObstacle(*obstacle) ;
    auto* obs_info =
        reference_line_info_->path_decision()->Find(obstacle->Id());
    if (obs_info) {
      obs_info->SetNeedShift(false);
    }
    return true;
  }

  // must consider obs,may cause stop and wheel shake
  if (injector_->last_using_lateral_ < kEpsilon) {
    // AINFO << "no need borrow";
    // return true;
  }
  // obs block all lane
  // bool is_block_obs = IsObstacleBlockAdc(obstacle);
  // if (is_block_obs) {
  //   // AINFO << "is_block_obs";
  //   // return true;
  // }
  // no consider obs behind routing end 5m.
  if (obs_sl.start_s() - kNoConsierLengthFrontRoutingEnd > routing_end_s_ &&
      obstacle->IsStatic()) {
    // AINFO << "NO CONSIDER";
    return true;
  }
  // when waiting to laneborrow near or in junction, ignore front non unknown
  // static obstacle in path_bound_decider, but do not ignore it in end_state_l
  const auto& path_decider =
      injector_->planning_context()->planning_status().path_decider();
  if (FLAGS_enable_near_junction_laneborrow && !injector_->is_auxiliary_road_ &&
      !injector_->is_in_play_street) {
    bool waiting_to_laneborrow =
        (PerceptionObstacle::UNKNOWN != obstacle->Perception().type() ||
         reference_line_info_->IsObstacleBlockAdc(obstacle)) &&
        reference_line_info_->IsLocatedInSolidLine(obs_sl) &&
        !path_decider.is_in_path_lane_borrow_scenario() &&
        !path_decider.is_need_go_left_lane();
    if (waiting_to_laneborrow) {
      return true;
    }
  }

  if (IsNeedKeepSelfPathStraightBeforeLaneBorrow(lane_borrow_info, obstacle)) {
    *keep_self_path_straight = true;
    return true;
  }

  if (CheckObstacleCanCauseHeadShake(*obstacle)) {
    return true;
  }

  const auto& config = config_.path_bounds_decider_config();
  double check_ignore_distance =
      (util::IsMixedTraffic(injector_->planning_context()) &&
       !injector_->is_auxiliary_road_) ||
              injector_->is_in_play_street
          ? config.ignore_obs_edge_distance_threshold_mixed_traffic()
          : config.ignore_obs_edge_distance_threshold();
  if (obstacle->Perception().type() == PerceptionObstacle::WHEELCRANE) {
    check_ignore_distance = FLAGS_wheelcrane_consider_distance;
  }
  // skip far obs
  if (diff_distance > check_ignore_distance) {
    return true;
  }

  // cause path_boundary no consider obs in selfborrow.
  if (!FLAGS_enable_self_borrow ||
      ((!injector_->enable_self_borrow_) &&
       (!(injector_->borrow_response().response_type() ==
          planning::ResponseType::ACCEPT)))) {
    // AINFO << "no self borrow";
    // return true;
  }

  return false;
}

bool PathBoundsDecider::IsObstacleBlockAdc(const Obstacle* obstacle) {
  if (!obstacle || obstacle->IsVirtual()) {
    return false;
  }
  const auto& adc_sl = reference_line_info_->AdcSlBoundary();
  const auto& obs_sl = obstacle->PerceptionSLBoundary();
  if (obs_sl.end_s() < adc_sl.start_s() - 1.0) {
    return false;
  }
  double lateral_buffer = 1.5;
  const auto& lane_width = reference_line_info_->GetLaneWidthBaseOnAdcCenter();
  double curr_lane_left_width = lane_width.first;
  double curr_lane_right_width = lane_width.second;
  double get_left_neighbor_lane_width = 0.0;
  double get_right_neighbor_lane_width = 0.0;
  hdmap::Id get_neighbor_lane_id;
  double curr_road_left_width = 0.0;
  double curr_road_right_width = 0.0;
  double past_road_left_width = curr_lane_left_width;
  double past_road_right_width = curr_lane_right_width;
  if (!reference_line_info_->reference_line().GetRoadWidth(
          adc_sl.end_s(), &curr_road_left_width, &curr_road_right_width)) {
    // AWARN << "Failed to get lane width at s = " << curr_s;
    curr_road_left_width = past_road_left_width;
    curr_road_right_width = past_road_right_width;
  } else {
    past_road_left_width = curr_road_left_width;
    past_road_right_width = curr_road_right_width;
  }
  // AINFO<<"curr_road_left_width = "<<curr_road_left_width;
  // AINFO<<"curr_road_right_width = "<<curr_road_right_width;
  get_left_neighbor_lane_width = curr_road_left_width - curr_lane_left_width;
  get_right_neighbor_lane_width = curr_road_right_width - curr_lane_right_width;
  double left_space =
      get_left_neighbor_lane_width + curr_lane_left_width - obs_sl.end_l();
  double right_space =
      get_right_neighbor_lane_width + curr_lane_right_width + obs_sl.start_l();
  double remain_space = std::fmax(left_space, right_space);
  // AINFO << "remain_space = " << remain_space;
  const auto& veh_param = VehicleConfigHelper::GetConfig().vehicle_param();
  if (remain_space < veh_param.width() + lateral_buffer) {
    return true;
  }

  return false;
}
bool PathBoundsDecider::CheckObstacleCanCauseHeadShake(
    const Obstacle& obstacle) {
  if (util::IsLaneBorrow(injector_->planning_context())) {
    return false;
  }
  const auto& config = config_.path_bounds_decider_config();
  const auto& obs_sl = obstacle.PerceptionSLBoundary();
  const auto& adc_sl = reference_line_info_->AdcSlBoundary();
  if (obs_sl.start_l() >
          adc_sl.end_l() + config.check_head_obs_lat_buffer_threshold() ||
      obs_sl.end_l() <
          adc_sl.start_l() - config.check_head_obs_lat_buffer_threshold()) {
    return false;
  }
  double diff_dis = obs_sl.start_s() - adc_sl.end_s();
  double diff_v = std::fabs(injector_->vehicle_state()->linear_velocity()) -
                  obstacle.speed();
  if (diff_v * config.check_head_bang_ttc() < diff_dis) {
    return false;
  }
  double obs_center_l = (obs_sl.start_l() + obs_sl.end_l()) * 0.5;
  double adc_center_l = (adc_sl.start_l() + adc_sl.end_l()) * 0.5;

  double check_l = (adc_center_l > obs_center_l)
                       ? obs_sl.end_l() + GetBufferBetweenADCCenterAndEdge() +
                             kLateralOverlapBuffer
                       : obs_sl.start_l() - GetBufferBetweenADCCenterAndEdge() -
                             kLateralOverlapBuffer;
  double distance = std::fmax(kEpsilon, obs_sl.start_s() - adc_sl.end_s() +
                                            VehicleConfigHelper::GetConfig()
                                                .vehicle_param()
                                                .front_edge_to_center());
  if (reference_line_info_->CheckIsHeadbang(distance, check_l)) {
    AINFO << "[check head bang] obstacle: " << obstacle.Id()
          << " will cause head bang";
    return true;
  }
  return false;
}

bool PathBoundsDecider::NoNeedPassStaticObstacle(const Obstacle* const obstacle,
                                                 double* const not_passable_s) {
  const auto& adc_sl = reference_line_info_->AdcSlBoundary();
  const auto& obs_sl = obstacle->PerceptionSLBoundary();
  // Only focus on obstacles that are ahead of ADC.
  // if (obs_sl.end_s() < adc_sl.start_s()-15.0){
  if (obs_sl.end_s() < adc_sl.start_s()) {
    return true;
  }
  if (obs_sl.start_s() > *not_passable_s) {
    return true;
  }

  if (IsNeedIgnoreInCrowdTraffic(obstacle)) {
    return true;
  }

  if (FLAGS_enable_near_junction_laneborrow) {
    if (IsNeedBeIgnoredNearJunction(obs_sl)) {
      ADEBUG << "ignore obs: " << obstacle->Id() << " in path bounds.";
      return true;
    }
  } else if (reference_line_info_->HasNeighborLane(adc_sl.end_s()) &&
             PerceptionObstacle::VEHICLE == obstacle->Perception().type() &&
             reference_line_info_->IsLocatedInSolidLine(obs_sl)) {
    *not_passable_s = obs_sl.end_s();
    return true;
  }

  if (REGULAR_PATH_BOUNDS == bounds_type_) {
    if (CheckObstacleIsCrossing(*obstacle)) {
      return true;
    }
    if (PerceptionObstacle::PEDESTRIAN == obstacle->Perception().type() &&
        hdmap::Lane::PLAY_STREET != reference_line_info_->GetLaneType() &&
        !injector_->can_borrow_pedestrian_) {
      return true;
    }
  }
  return false;
}

bool PathBoundsDecider::CheckObstacleIsCrossing(const Obstacle& obstacle) {
  const auto& adc_sl = reference_line_info_->AdcSlBoundary();
  const auto& obs_sl = obstacle.PerceptionSLBoundary();
  double obstacle_heading = obstacle.PerceptionBoundingBox().heading();
  double ref_heading = reference_line_info_->reference_line()
                           .GetReferencePoint(obs_sl.start_s())
                           .heading();
  double heading_diff = std::fabs(
      century::common::math::AngleDiff(obstacle_heading, ref_heading));
  bool near_ref_angle = (heading_diff < M_PI_4 || heading_diff > M_PI_4 * 3.0);

  ADEBUG << "obstacle_Type: [" << obstacle.Id() << "]";
  ADEBUG << "obstacle_heading: " << obstacle_heading;
  ADEBUG << "ref_heading: " << ref_heading << ", DIFF: " << heading_diff;

  if (!near_ref_angle) {
    if (obstacle.speed() > FLAGS_borrow_slow_obstacle_velocity_threshold) {
      return true;
    }
    if (!injector_->is_in_play_street &&
        PerceptionObstacle::VEHICLE == obstacle.Perception().type()) {
      if (obs_sl.start_l() < adc_sl.end_l() - kCrossVehicleBuffer &&
          obs_sl.end_l() > adc_sl.start_l() + kCrossVehicleBuffer) {
        // AINFO << "VEHICLE:" << obstacle.Id()
        //       << " may crossing, heading diff: " << heading_diff;
        // TODO(zongxingguo): need to check more frame.
        // return true;
      }
    }
  }
  return false;
}

bool PathBoundsDecider::IsNeedBeIgnoredNearJunction(const SLBoundary& obs_sl) {
  const auto& adc_sl = reference_line_info_->AdcSlBoundary();
  const auto& path_decider =
      injector_->planning_context()->planning_status().path_decider();
  common::SLPoint obs_center_point;
  obs_center_point.set_s(0.5 * (obs_sl.start_s() + obs_sl.end_s()));
  obs_center_point.set_l(0.5 * (obs_sl.start_l() + obs_sl.end_l()));

  if (path_decider.is_need_goback_reference_lane() &&
      obs_sl.end_s() > adc_sl.end_s() &&
      reference_line_info_->reference_line().IsOnLane(obs_center_point)) {
    return true;
  }

  if ((path_decider.is_adc_near_junction() ||
       path_decider.is_obs_near_junction()) &&
      reference_line_info_->IsInLeftNeighborLine(obs_sl) &&
      path_decider.is_in_path_lane_borrow_scenario() &&
      !path_decider.is_need_goback_reference_lane()) {
    return true;
  }

  return false;
}

bool PathBoundsDecider::IsNeedIgnoreInCrowdTraffic(const Obstacle* obstacle) {
  if (injector_->planning_context()
          ->planning_status()
          .path_decider()
          .is_in_crowd_traffic()) {
    if (PerceptionObstacle::UNKNOWN != obstacle->Perception().type() ||
        reference_line_info_->IsObstacleBlockAdc(obstacle)) {
      return true;
    }
  }
  return false;
}

void PathBoundsDecider::UpdateTrackObstaclesEdge(
    const DirectionDecision& direction_decision, const Obstacle* obstacle,
    std::vector<ObstacleEdge>* const obstacles_edges, const double lat_buffer,
    const double lon_start_buffer, const double lon_end_buffer) {
  auto iter = track_obstacle_edge_.find(obstacle->Id());
  if (iter != track_obstacle_edge_.end() &&
      !util::IsNeedLoosenPathConstrains(injector_->planning_context())) {
    UpdateExistObstacleEdge(direction_decision, obstacle, obstacles_edges,
                            lat_buffer, lon_start_buffer, lon_end_buffer);
  } else {
    const auto obstacle_sl = obstacle->PerceptionSLBoundary();
    const auto& config = config_.path_bounds_decider_config();
    ObstacleEdge obstacle_start_edge(
        1, obstacle_sl.start_s() - lon_start_buffer,
        obstacle_sl.start_l() - lat_buffer, obstacle_sl.end_l() + lat_buffer,
        obstacle->Id());
    obstacle_start_edge.l_min_safe_limit =
        obstacle_start_edge.l_min +
        lat_buffer * config.safe_limit_edge_buffer_ratio();
    obstacle_start_edge.l_max_safe_limit =
        obstacle_start_edge.l_max -
        lat_buffer * config.safe_limit_edge_buffer_ratio();
    obstacle_start_edge.timestamp = Clock::NowInSeconds();
    obstacle_start_edge.lon_start_buffer = lon_start_buffer;
    obstacle_start_edge.origin_start_s = obstacle_sl.start_s();
    obstacle_start_edge.origin_start_l = obstacle_sl.start_l();
    obstacle_start_edge.origin_end_l = obstacle_sl.end_l();
    obstacle_start_edge.direction_decision = direction_decision;
    obstacles_edges->emplace_back(obstacle_start_edge);
    track_obstacle_edge_.insert({obstacle->Id(), obstacle_start_edge});

    ObstacleEdge obstacle_end_edge(0, 0.0, 0.0, 0.0, obstacle->Id());
    obstacle_end_edge.s = obstacle_sl.end_s() + lon_end_buffer;
    obstacle_end_edge.l_min = obstacle_start_edge.l_min;
    obstacle_end_edge.l_max = obstacle_start_edge.l_max;
    obstacle_end_edge.l_min_safe_limit = obstacle_start_edge.l_min_safe_limit;
    obstacle_end_edge.l_max_safe_limit = obstacle_start_edge.l_max_safe_limit;
    obstacle_end_edge.timestamp = obstacle_start_edge.timestamp;
    obstacle_end_edge.lon_start_buffer = obstacle_start_edge.lon_start_buffer;
    obstacle_end_edge.origin_start_s = obstacle_start_edge.origin_start_s;
    obstacle_end_edge.origin_start_l = obstacle_start_edge.origin_start_l;
    obstacle_end_edge.origin_end_l = obstacle_start_edge.origin_end_l;
    obstacle_end_edge.direction_decision = direction_decision;
    obstacles_edges->emplace_back(obstacle_end_edge);
  }
}

void PathBoundsDecider::UpdateExistObstacleEdge(
    const DirectionDecision& direction_decision, const Obstacle* obstacle,
    std::vector<ObstacleEdge>* const obstacles_edges, const double lat_buffer,
    const double lon_start_buffer, const double lon_end_buffer) {
  const auto obstacle_sl = obstacle->PerceptionSLBoundary();
  const auto& config = config_.path_bounds_decider_config();
  auto iter = track_obstacle_edge_.find(obstacle->Id());
  double time_diff = Clock::NowInSeconds() - iter->second.timestamp;
  bool check_time = time_diff < config.obstacle_track_keep_time();
  bool check_s_diff =
      std::fabs(obstacle_sl.start_s() - iter->second.origin_start_s) <
      (FLAGS_static_obstacle_speed_threshold * time_diff +
       iter->second.lon_start_buffer);
  // AINFO << "obstacle = " << obstacle->Id();
  // AINFO << "S_DIFF = " << obstacle_sl.start_s() - iter->second.origin_start_s;
  bool check_start_l_diff =
      std::fabs(obstacle_sl.start_l() - iter->second.origin_start_l) <
      config.check_obstacle_lateral_diff();
  bool check_end_l_diff =
      std::fabs(obstacle_sl.end_l() - iter->second.origin_end_l) <
      config.check_obstacle_lateral_diff();
  // only extend to update.
  if (obstacle_sl.start_l() + kExtendLateralBuffer <
      iter->second.origin_start_l) {
    // AINFO << "extend start l";
    check_start_l_diff = true;
  }
  if (obstacle_sl.end_l() > iter->second.origin_end_l + kExtendLateralBuffer) {
    check_end_l_diff = true;
    // AINFO << "extend end l";
  }
  // AINFO << "START_L_DIFF = "
  //       << (obstacle_sl.start_l() - iter->second.origin_start_l);

  // AINFO << "END_L_DIFF = " << (obstacle_sl.end_l() - iter->second.origin_end_l);
  // AINFO << "check_obstacle_lateral_diff = "
  //       << config.check_obstacle_lateral_diff();
  bool can_track =
      (check_time && check_s_diff && check_start_l_diff && check_end_l_diff);
  ADEBUG << "Obs_Id: " << obstacle->Id() << ", can_track: " << can_track
         << ", check_time: " << check_time << ", check_s_diff: " << check_s_diff
         << ", check_start_l_diff: " << check_start_l_diff
         << ", check_end_l_diff: " << check_end_l_diff;
  if (can_track) {
    if (obstacle_sl.start_l() < iter->second.l_min_safe_limit) {
      double start_l_over_diff =
          iter->second.l_min_safe_limit - obstacle_sl.start_l();
      iter->second.l_min -= start_l_over_diff;
      iter->second.l_min_safe_limit = obstacle_sl.start_l();
    }
    if (obstacle_sl.end_l() > iter->second.l_max_safe_limit) {
      double end_l_over_diff =
          obstacle_sl.end_l() - iter->second.l_max_safe_limit;
      iter->second.l_max += end_l_over_diff;
      iter->second.l_max_safe_limit = obstacle_sl.end_l();
    }
    iter->second.s = obstacle_sl.start_s() - iter->second.lon_start_buffer;
  } else {
    iter->second.l_min = obstacle_sl.start_l() - lat_buffer;
    iter->second.l_max = obstacle_sl.end_l() + lat_buffer;
    iter->second.l_min_safe_limit =
        iter->second.l_min + lat_buffer * config.safe_limit_edge_buffer_ratio();
    iter->second.l_max_safe_limit =
        iter->second.l_max - lat_buffer * config.safe_limit_edge_buffer_ratio();
    iter->second.lon_start_buffer = lon_start_buffer;
    iter->second.origin_start_s = obstacle_sl.start_s();
    iter->second.origin_start_l = obstacle_sl.start_l();
    iter->second.origin_end_l = obstacle_sl.end_l();
    iter->second.s = obstacle_sl.start_s() - lon_start_buffer;
  }
  iter->second.timestamp = Clock::NowInSeconds();
  iter->second.direction_decision = direction_decision;
  obstacles_edges->emplace_back(iter->second);

  ObstacleEdge obstacle_end_edge(0, 0.0, 0.0, 0.0, obstacle->Id());
  obstacle_end_edge.s = obstacle_sl.end_s() + lon_end_buffer;
  obstacle_end_edge.l_min = iter->second.l_min;
  obstacle_end_edge.l_max = iter->second.l_max;
  obstacle_end_edge.l_min_safe_limit = iter->second.l_min_safe_limit;
  obstacle_end_edge.l_max_safe_limit = iter->second.l_max_safe_limit;
  obstacle_end_edge.timestamp = iter->second.timestamp;
  obstacle_end_edge.lon_start_buffer = iter->second.lon_start_buffer;
  obstacle_end_edge.origin_start_s = iter->second.origin_start_s;
  obstacle_end_edge.origin_start_l = iter->second.origin_start_l;
  obstacle_end_edge.origin_end_l = iter->second.origin_end_l;
  obstacle_end_edge.direction_decision = direction_decision;
  obstacles_edges->emplace_back(obstacle_end_edge);
}

void PathBoundsDecider::TrimTrackObstaclesEdge() {
  auto iter = track_obstacle_edge_.begin();
  double current_time = Clock::NowInSeconds();
  while (iter != track_obstacle_edge_.end()) {
    double time_diff = current_time - iter->second.timestamp;
    if (time_diff >
        config_.path_bounds_decider_config().obstacle_track_keep_time()) {
      ADEBUG << "OBSTACLE ID: " << iter->second.obstacle_id << " over "
             << config_.path_bounds_decider_config().obstacle_track_keep_time()
             << "s, need to be erased";
      track_obstacle_edge_.erase(iter++);
    } else {
      ++iter;
    }
  }
}

void PathBoundsDecider::UpdateOvertakenObstacleEdge(
    const Obstacle* obstacle, std::vector<ObstacleEdge>* const obstacles_edges,
    const double lat_buffer, const double lon_start_buffer,
    const double lon_end_buffer) {
  const auto& config = config_.path_bounds_decider_config();
  const auto obstacle_sl = obstacle->PerceptionSLBoundary();
  double obs_v = obstacle->speed();
  const auto& adc_sl = reference_line_info_->AdcSlBoundary();
  const auto& adc_v = reference_line_info_->vehicle_state().linear_velocity();
  double time_to_collision = (obstacle_sl.start_s() - adc_sl.end_s()) /
                             std::max(kEpsilon, adc_v - obs_v);
  if (time_to_collision > FLAGS_look_forward_time_sec) {
    return;
  }
  double drive_dis =
      obs_v * time_to_collision * config.obstacle_start_edge_buffer_ratio();
  ObstacleEdge obstacle_start_edge(
      1, obstacle_sl.start_s() - lon_start_buffer + drive_dis,
      obstacle_sl.start_l() - lat_buffer, obstacle_sl.end_l() + lat_buffer,
      obstacle->Id());
  obstacles_edges->emplace_back(obstacle_start_edge);
  ObstacleEdge obstacle_end_edge(
      0, obstacle_sl.end_s() + lon_end_buffer + drive_dis,
      obstacle_sl.start_l() - lat_buffer, obstacle_sl.end_l() + lat_buffer,
      obstacle->Id());
  obstacles_edges->emplace_back(obstacle_end_edge);
  return;
}

void PathBoundsDecider::DecidePassDirections(
    double l_min, double l_max,
    const std::vector<ObstacleEdge>& new_entering_obstacles,
    std::vector<std::vector<bool>>* const pass_directions) {
  CHECK_NOTNULL(pass_directions);

  // Convert into lateral edges.
  std::vector<ObstacleEdge> lateral_edges;
  lateral_edges.emplace_back(true, std::numeric_limits<double>::lowest(), 0.0,
                             0.0, "l_min");
  lateral_edges.emplace_back(false, l_min, 0.0, 0.0, "l_min");
  lateral_edges.emplace_back(true, l_max, 0.0, 0.0, "l_max");
  lateral_edges.emplace_back(false, std::numeric_limits<double>::max(), 0.0,
                             0.0, "l_max");
  for (size_t i = 0; i < new_entering_obstacles.size(); ++i) {
    if (new_entering_obstacles[i].l_min < l_min ||
        new_entering_obstacles[i].l_max > l_max) {
      continue;
    }
    lateral_edges.emplace_back(true, new_entering_obstacles[i].l_min, 0.0, 0.0,
                               new_entering_obstacles[i].obstacle_id);
    lateral_edges.emplace_back(false, new_entering_obstacles[i].l_max, 0.0, 0.0,
                               new_entering_obstacles[i].obstacle_id);
  }
  // Sort the lateral edges for lateral sweep-line algorithm.
  std::sort(lateral_edges.begin(), lateral_edges.end(),
            [](const ObstacleEdge& lhs, const ObstacleEdge& rhs) {
              return lhs.s != rhs.s ? lhs.s < rhs.s
                                    : lhs.is_start_s > rhs.is_start_s;
            });

  // Go through the lateral edges and find any possible slot.
  std::vector<double> empty_slot;
  int num_obs = 0;
  for (size_t i = 0; i < lateral_edges.size(); ++i) {
    // Update obstacle overlapping info.
    if (lateral_edges[i].is_start_s) {
      ++num_obs;
    } else {
      --num_obs;
    }
    // If there is an empty slot within lane boundary.
    if (0 == num_obs && i != lateral_edges.size() - 1) {
      empty_slot.push_back((lateral_edges[i].s + lateral_edges[i + 1].s) * 0.5);
    }
  }
  // For each empty slot, update a corresponding pass direction
  for (size_t i = 0; i < empty_slot.size(); ++i) {
    double pass_position = empty_slot[i];
    std::vector<bool> pass_direction;
    for (size_t j = 0; j < new_entering_obstacles.size(); ++j) {
      if (new_entering_obstacles[j].l_min > pass_position) {
        pass_direction.push_back(false);
      } else {
        pass_direction.push_back(true);
      }
    }
    pass_directions->push_back(pass_direction);
  }
  // TODO(jiacheng): sort the decisions based on the feasibility.
  return;
}

double PathBoundsDecider::GetBufferBetweenADCCenterAndEdge() {
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  // TODO(all): currently it's a fixed number. But it can take into account
  // many factors such as: ADC length, possible turning angle, speed, etc.

  return (adc_half_width + kAdcEdgeBuffer);
}

bool PathBoundsDecider::UpdatePathBoundaryWithBuffer(
    PathBoundPoint* const path_bound_point, double left_bound,
    double right_bound, bool is_left_lane_bound, bool is_right_lane_bound,
    LaneBorrowInfo lane_borrow_info, bool is_lanechange) {
  // substract vehicle width when bound does not come from the lane boundary
  double left_adc_buffer_coeff =
      (is_left_lane_bound
           ? config_.path_bounds_decider_config().adc_buffer_coeff()
           : kDefaultAdcBufferCoeff);
  double right_adc_buffer_coeff =
      (is_right_lane_bound
           ? config_.path_bounds_decider_config().adc_buffer_coeff()
           : kDefaultAdcBufferCoeff);
  // cehck has left neighbor lane for extend left boundary
  // AINFO << "path_bound_point->s() = " << path_bound_point->s;
  const auto& locate_lane = LocateLaneInfo(path_bound_point->s);
  if (locate_lane != nullptr) {
    if (!locate_lane->lane().left_neighbor_forward_lane_id().empty()) {
      left_adc_buffer_coeff = 0.0;
    }
    if (!locate_lane->lane().right_neighbor_forward_lane_id().empty()) {
      right_adc_buffer_coeff = 0.0;
    }
  }
  // if is in borrow ,is cut half width
  // Update the right bound (l_min):
  double right_bound_new =
      right_bound + right_adc_buffer_coeff * GetBufferBetweenADCCenterAndEdge();
  double left_bound_new =
      left_bound - left_adc_buffer_coeff * GetBufferBetweenADCCenterAndEdge();
  // on for borrow boundary.for two line to one line.extend
  if (util::IsLaneBorrow(injector_->planning_context())) {
    if (LaneBorrowInfo::NO_BORROW != lane_borrow_info) {
      right_bound_new = right_bound;
      left_bound_new = left_bound;
    }
  }
  if(is_lanechange){
    right_bound_new = right_bound;
    left_bound_new = left_bound;
  }
  double new_l_min = std::fmax(path_bound_point->l_min, right_bound_new);
  // Update the left bound (l_max):
  double new_l_max = std::fmin(path_bound_point->l_max, left_bound_new);
  //  AINFO<<"new_l_max = "<<new_l_max;
  //  AINFO<<"new_l_min = "<<new_l_min;
  //  no extend in laneborrow
  if (injector_->enable_self_borrow_ &&
      !(injector_->borrow_response().response_type() ==
        planning::ResponseType::ACCEPT)) {
    if (locate_lane != nullptr) {
      if (!locate_lane->lane().left_neighbor_forward_lane_id().empty()) {
        new_l_max =
            std::fmin(path_bound_point->l_max,
                      left_bound + FLAGS_distance_self_borrow_extend_boudary);
      }
      if (!locate_lane->lane().right_neighbor_forward_lane_id().empty()) {
        new_l_min =
            std::fmax(path_bound_point->l_min,
                      right_bound - FLAGS_distance_self_borrow_extend_boudary);
      }
    }
  }

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_l_min > new_l_max) {
    if (injector_->is_adc_in_gate_junction_) {
      path_bound_point->l_min = -kLaneWidth;
      path_bound_point->l_max = kLaneWidth;
      // AINFO<<"adc in gate junction";
      return true;
    } else {
       AINFO << "adc no in gate junction ,and no space to pass";
      return false;
    }
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  path_bound_point->l_min = new_l_min;
  path_bound_point->l_max = new_l_max;
  return true;
}

bool PathBoundsDecider::UpdatePathBoundaryAndCenterLineWithBuffer(
    PathBoundPoint* const path_bound_point, double left_bound,
    double right_bound, double* const center_line,
    const LaneBorrowInfo& lane_borrow_info) {
  UpdatePathBoundaryWithBuffer(path_bound_point, left_bound, right_bound, false,
                               false, lane_borrow_info,
                               LANE_CHANGE_PATH_BOUNDS == bounds_type_);
  *center_line = (path_bound_point->l_min + path_bound_point->l_max) * 0.5;
  return true;
}

bool PathBoundsDecider::UpdatePathBoundary(size_t idx, double left_bound,
                                           double right_bound,
                                           PathBound* const path_boundaries) {
  // Update the right bound (l_min):
  double new_l_min = std::fmax((*path_boundaries)[idx].l_min, right_bound);
  // Update the left bound (l_max):
  double new_l_max = std::fmin((*path_boundaries)[idx].l_max, left_bound);

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_l_min > new_l_max) {
    ADEBUG << "Path is blocked at idx = " << idx;
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  (*path_boundaries)[idx].l_min = new_l_min;
  (*path_boundaries)[idx].l_max = new_l_max;
  return true;
}

void PathBoundsDecider::TrimPathBounds(const int path_blocked_idx,
                                       PathBound* const path_boundaries) {
  if (-1 != path_blocked_idx) {
    if (0 == path_blocked_idx) {
      ADEBUG << "Completely blocked. Cannot move at all.";
    }
    int range = static_cast<int>(path_boundaries->size()) - path_blocked_idx;
    for (int i = 0; i < range; ++i) {
      path_boundaries->pop_back();
    }
  }
}

void PathBoundsDecider::PathBoundsDebugString(
    const PathBound& path_boundaries) {
  for (size_t i = 0; i < path_boundaries.size(); ++i) {
    AWARN << "idx " << i << "; s = " << path_boundaries[i].s
          << "; l_min = " << path_boundaries[i].l_min
          << "; l_max = " << path_boundaries[i].l_max;
  }
}

bool PathBoundsDecider::CheckLaneBoundaryType(
    const ReferenceLineInfo& reference_line_info, const double check_s,
    const LaneBorrowInfo& lane_borrow_info) {
  if (LaneBorrowInfo::NO_BORROW == lane_borrow_info) {
    return false;
  }

  const ReferenceLine& reference_line = reference_line_info.reference_line();
  auto ref_point = reference_line.GetNearestReferencePoint(check_s);
  if (ref_point.lane_waypoints().empty()) {
    return false;
  }

  const auto& waypoint = ref_point.lane_waypoints().front();
  hdmap::LaneBoundaryType::Type lane_boundary_type =
      hdmap::LaneBoundaryType::UNKNOWN;
  if (LaneBorrowInfo::LEFT_BORROW == lane_borrow_info) {
    lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
  } else if (LaneBorrowInfo::RIGHT_BORROW == lane_borrow_info) {
    lane_boundary_type = hdmap::RightBoundaryType(waypoint);
  }
  if (hdmap::Lane::PLAY_STREET != waypoint.lane->lane().type() &&
      (hdmap::LaneBoundaryType::SOLID_YELLOW == lane_boundary_type ||
       hdmap::LaneBoundaryType::SOLID_WHITE == lane_boundary_type)) {
    return false;
  }
  return true;
}

void PathBoundsDecider::UpdateNarrowAreaStatus() {
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  const auto& config = config_.path_bounds_decider_config();
  double safe_enough_buffer = (FLAGS_obstacle_max_lat_buffer_public_road -
                               config.narrow_area_lat_safe_buffer()) *
                              2.0;
  double adc_speed =
      std::max(0.0, reference_line_info_->vehicle_state().linear_velocity());

  // 0. check is can EXIT narrow area
  if (mutable_path_decider_status->adc_enter_narrow_area()) {
    if (!mutable_path_decider_status->adc_will_be_blocked() &&
        (min_lateral_diff_ > safe_enough_buffer ||
         adc_speed > FLAGS_adc_speed_low_threshold_public_road)) {
      auto count = mutable_path_decider_status->adc_slow_down_block_times();
      mutable_path_decider_status->set_adc_slow_down_block_times(--count);
    } else {
      mutable_path_decider_status->set_adc_slow_down_block_times(0);
    }

    if (mutable_path_decider_status->adc_slow_down_block_times() <
        -config.adc_slow_down_block_count_threshold()) {
      mutable_path_decider_status->set_adc_enter_narrow_area(false);
      mutable_path_decider_status->set_adc_slow_down_block_times(0);
    }
  } else {
    // 1. check is can ENTER narrow area
    double adc_speed =
        std::max(0.0, reference_line_info_->vehicle_state().linear_velocity());
    const auto& config = config_.path_bounds_decider_config();
    auto count = mutable_path_decider_status->adc_slow_down_block_times();
    if (mutable_path_decider_status->adc_will_be_blocked() &&
        !mutable_path_decider_status->adc_located_in_junction()) {
      if (adc_speed < FLAGS_adc_speed_low_threshold_public_road) {
        mutable_path_decider_status->set_adc_slow_down_block_times(++count);
      } else {
        mutable_path_decider_status->set_adc_slow_down_block_times(0);
      }
    } else {
      mutable_path_decider_status->set_adc_slow_down_block_times(
          std::max(0, --count));
    }

    if (mutable_path_decider_status->adc_slow_down_block_times() >
        config.adc_slow_down_block_count_threshold()) {
      mutable_path_decider_status->set_adc_enter_narrow_area(true);
      mutable_path_decider_status->set_adc_slow_down_block_times(0);
    }
  }
  ADEBUG << "after update, narrow area status: "
         << mutable_path_decider_status->adc_enter_narrow_area();
}

void PathBoundsDecider::RecordDebugInfo(
    const PathBound& path_boundaries, const std::string& debug_name,
    ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  ACHECK(!path_boundaries.empty());
  CHECK_NOTNULL(reference_line_info);

  // Take the left and right path boundaries, and transform them into two
  // PathData so that they can be displayed in simulator.
  std::vector<common::FrenetFramePoint> frenet_frame_left_boundaries;
  std::vector<common::FrenetFramePoint> frenet_frame_right_boundaries;
  for (const PathBoundPoint& path_bound_point : path_boundaries) {
    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(path_bound_point.s);
    frenet_frame_point.set_dl(0.0);
    frenet_frame_point.set_ddl(0.0);

    frenet_frame_point.set_l(path_bound_point.l_min);
    frenet_frame_right_boundaries.push_back(frenet_frame_point);
    frenet_frame_point.set_l(path_bound_point.l_max);
    frenet_frame_left_boundaries.push_back(frenet_frame_point);
  }

  auto frenet_frame_left_path =
      FrenetFramePath(std::move(frenet_frame_left_boundaries));
  auto frenet_frame_right_path =
      FrenetFramePath(std::move(frenet_frame_right_boundaries));

  PathData left_path_data;
  left_path_data.SetReferenceLine(&(reference_line_info->reference_line()));
  left_path_data.SetFrenetPath(std::move(frenet_frame_left_path));
  PathData right_path_data;
  right_path_data.SetReferenceLine(&(reference_line_info->reference_line()));
  right_path_data.SetFrenetPath(std::move(frenet_frame_right_path));

  // Insert the transformed PathData into the simulator display.
  if (util::CompareTwoStringIsEqual(debug_name, "self")) {
    auto* ptr_display_path_1 = reference_line_info->mutable_debug()
                                   ->mutable_planning_data()
                                   ->add_self_path();
    ptr_display_path_1->set_name("planning_path_boundary_1");
    ptr_display_path_1->mutable_path_point()->CopyFrom(
        {left_path_data.discretized_path().begin(),
         left_path_data.discretized_path().end()});
    auto* ptr_display_path_2 = reference_line_info->mutable_debug()
                                   ->mutable_planning_data()
                                   ->add_self_path();
    ptr_display_path_2->set_name("planning_path_boundary_2");
    ptr_display_path_2->mutable_path_point()->CopyFrom(
        {right_path_data.discretized_path().begin(),
         right_path_data.discretized_path().end()});
  }
  if (util::CompareTwoStringIsEqual(debug_name, "lanechange")) {
    auto* ptr_display_path_1 = reference_line_info->mutable_debug()
                                   ->mutable_planning_data()
                                   ->add_lanechange_path();
    ptr_display_path_1->set_name("planning_path_boundary_1");
    ptr_display_path_1->mutable_path_point()->CopyFrom(
        {left_path_data.discretized_path().begin(),
         left_path_data.discretized_path().end()});
    auto* ptr_display_path_2 = reference_line_info->mutable_debug()
                                   ->mutable_planning_data()
                                   ->add_lanechange_path();
    ptr_display_path_2->set_name("planning_path_boundary_2");
    ptr_display_path_2->mutable_path_point()->CopyFrom(
        {right_path_data.discretized_path().begin(),
         right_path_data.discretized_path().end()});
  }
  if (util::CompareTwoStringIsEqual(debug_name, "left")) {
    auto* ptr_display_path_1 = reference_line_info->mutable_debug()
                                   ->mutable_planning_data()
                                   ->add_left_borrow_path();
    ptr_display_path_1->set_name("planning_path_boundary_1");
    ptr_display_path_1->mutable_path_point()->CopyFrom(
        {left_path_data.discretized_path().begin(),
         left_path_data.discretized_path().end()});
    auto* ptr_display_path_2 = reference_line_info->mutable_debug()
                                   ->mutable_planning_data()
                                   ->add_left_borrow_path();
    ptr_display_path_2->set_name("planning_path_boundary_2");
    ptr_display_path_2->mutable_path_point()->CopyFrom(
        {right_path_data.discretized_path().begin(),
         right_path_data.discretized_path().end()});
  }
  if (util::CompareTwoStringIsEqual(debug_name, "right")) {
    auto* ptr_display_path_1 = reference_line_info->mutable_debug()
                                   ->mutable_planning_data()
                                   ->add_right_borrow_path();
    ptr_display_path_1->set_name("planning_path_boundary_1");
    ptr_display_path_1->mutable_path_point()->CopyFrom(
        {left_path_data.discretized_path().begin(),
         left_path_data.discretized_path().end()});
    auto* ptr_display_path_2 = reference_line_info->mutable_debug()
                                   ->mutable_planning_data()
                                   ->add_right_borrow_path();
    ptr_display_path_2->set_name("planning_path_boundary_2");
    ptr_display_path_2->mutable_path_point()->CopyFrom(
        {right_path_data.discretized_path().begin(),
         right_path_data.discretized_path().end()});
  }
}

bool PathBoundsDecider::ComupteLaneChangeKeyPoint(
    PathBoundPoint* const start_point, PathBoundPoint* const end_point) {
  lane_change_direction_ = LaneChangeDirection::NO_CHANGE;
  double start_s = 0.0;
  double end_s = 0.0;
  if (!CheckIsCanIntoLaneChange(&start_s, &end_s)) {
    return false;
  }

  double target_lane_end_left_width = adc_lane_width_ * 0.5;
  double target_lane_end_right_width = adc_lane_width_ * 0.5;
  reference_line_info_->reference_line().GetLaneWidth(
      end_s, &target_lane_end_left_width, &target_lane_end_right_width);

  start_point->s = start_s;
  end_point->s = end_s;
  end_point->l_min = -target_lane_end_right_width;
  end_point->l_max = target_lane_end_left_width;

  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  start_point->l_min =
      lane_change_pre_adc_lateral_l_ - adc_half_width - kEpsilon;
  start_point->l_max =
      lane_change_pre_adc_lateral_l_ + adc_half_width + kEpsilon;
  ADEBUG << "start_lmin: " << start_point->l_min
         << ", start_lmax: " << start_point->l_max;
  ADEBUG << "end_lmin: " << end_point->l_min
         << ", end_lmax: " << end_point->l_max;

  return true;
}

bool PathBoundsDecider::CheckIsCanIntoLaneChange(double* const start_s,
                                                 double* const end_s) {
  const auto& target_lane_adc_sl = reference_line_info_->AdcSlBoundary();
  double target_adc_center_l =
      (target_lane_adc_sl.start_l() + target_lane_adc_sl.end_l()) * 0.5;
  if (reference_line_info_->IsChangeLanePath() &&
      reference_line_info_->GetIsClearToChangeLane()) {
    if (!into_lane_change_ ||
        VaildPathLable::LANE_CHANGE != injector_->last_path_label_) {
      into_lane_change_ = true;
      lane_change_pre_adc_lateral_l_ = target_adc_center_l;
      lane_change_pre_adc_lateral_v_ =
          reference_line_info_->GetAdcLateralVelocity();
      start_time_into_lane_change_ = Clock::NowInSeconds();
      AINFO << "[JJZ] begin into lane change, l: "
            << lane_change_pre_adc_lateral_l_
            << ", v: " << lane_change_pre_adc_lateral_v_;
    }
  } else {
    into_lane_change_ = false;
  }

  if (!into_lane_change_) {
    return false;
  }

  *start_s = GetLaneChangeStartSValue();
  *end_s = GetLaneChangeEndSValue(*start_s);

  if ((*end_s - *start_s) <
      VehicleConfigHelper::GetConfig().vehicle_param().length()) {
    into_lane_change_ = false;
    return false;
  }

  return true;
}

double PathBoundsDecider::GetLaneChangeStartSValue() {
  const auto& config = config_.path_bounds_decider_config();
  const auto& adc_v = reference_line_info_->vehicle_state().linear_velocity();
  double time_diff = Clock::NowInSeconds() - start_time_into_lane_change_;
  double pre_time = config.lane_change_preview_time();

  if (PathBoundsDeciderConfig::CUBIC ==
      config.lane_change_path_bound_method()) {
    pre_time =
        InterpolationLookUp(adc_v, 0.0, FLAGS_planning_upper_speed_limit,
                            config.min_lane_change_preview_time_for_cubic(),
                            config.max_lane_change_preview_time_for_cubic());
  }
  return (reference_line_info_->AdcSlBoundary().end_s() +
          (pre_time - time_diff) * adc_v);
}

double PathBoundsDecider::GetLaneChangeEndSValue(const double start_s) {
  double remain_lane_change_time = GetRemainLaneChangeTime(start_s);
  AINFO << "remain_lane_change_time: " << remain_lane_change_time;
  const auto& config = config_.path_bounds_decider_config();
  const auto& adc_v = reference_line_info_->vehicle_state().linear_velocity();
  bool urgency_lane_change = injector_->planning_context()
                                 ->planning_status()
                                 .overtake()
                                 .urgency_lane_change();
  double use_adc_v = (urgency_lane_change)
                         ? std::fmax(adc_v, config.lane_change_acceleration() *
                                                remain_lane_change_time * 0.5)
                         : (adc_v + config.lane_change_acceleration() *
                                        remain_lane_change_time * 0.5);
  double ratio =
      (FLAGS_enable_overtake_speed_up) ? FLAGS_overtake_speed_up_ratio : 1.0;
  double adc_max_v = std::fmin(FLAGS_planning_upper_speed_limit * ratio,
                               FLAGS_overtake_upper_speed_limit);
  return start_s + std::fmin(adc_max_v, use_adc_v) *
                       std::fmax(remain_lane_change_time, 0.0);
}

double PathBoundsDecider::GetRemainLaneChangeTime(const double start_s) {
  double origin_lane_start_left_width = adc_lane_width_ * 0.5;
  double origin_lane_start_right_width = adc_lane_width_ * 0.5;
  double target_lane_start_left_width = adc_lane_width_ * 0.5;
  double target_lane_start_right_width = adc_lane_width_ * 0.5;

  for (auto& reference_line_info : frame_->reference_line_info()) {
    if (reference_line_info.IsChangeLanePath()) {
      reference_line_info.reference_line().GetLaneWidth(
          start_s, &target_lane_start_left_width,
          &target_lane_start_right_width);
      continue;
    }
    reference_line_info.reference_line().GetLaneWidth(
        start_s, &origin_lane_start_left_width, &origin_lane_start_right_width);
  }

  const auto& target_lane_adc_sl = reference_line_info_->AdcSlBoundary();
  double target_adc_center_l =
      (target_lane_adc_sl.start_l() + target_lane_adc_sl.end_l()) * 0.5;
  bool turn_left = (target_adc_center_l < 0.0);
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  double change_time = FLAGS_lane_change_total_time;
  if (turn_left) {
    lane_change_direction_ = LaneChangeDirection::LEFT_CHANGE;
    double lateral_l_upper = (-target_lane_start_right_width + adc_half_width);
    double lateral_l_lower =
        -target_lane_start_right_width - origin_lane_start_left_width;
    return InterpolationLookUp(target_adc_center_l, lateral_l_lower,
                               lateral_l_upper, change_time, 0.0);
  } else {
    lane_change_direction_ = LaneChangeDirection::RIGHT_CHANGE;
    double lateral_l_lower = (target_lane_start_left_width - adc_half_width);
    double lateral_l_upper =
        target_lane_start_left_width + origin_lane_start_right_width;
    return InterpolationLookUp(target_adc_center_l, lateral_l_lower,
                               lateral_l_upper, 0.0, change_time);
  }
}

double PathBoundsDecider::GetEndStateLForKeepRight(
    const ReferenceLineInfo& reference_line_info) {
  double end_state_l = 0.0;
  // get static obstacle
  const auto& reference_line = reference_line_info.reference_line();
  const auto& path_decision = reference_line_info.path_decision();
  const auto& indexed_obstacles = path_decision.obstacles();
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double half_width = veh_param.width() * 0.5;
  double max_end_state_l = std::numeric_limits<double>::lowest();
  bool is_need_to_offset = false;
  for (const auto* obstacle : indexed_obstacles.Items()) {
    if (nullptr == obstacle) {
      AERROR << "Obstacle pointer is null.";
      continue;
    }
    const auto& obs_sl = obstacle->PerceptionSLBoundary();
    if (obs_sl.start_s() > injector_->nearst_obs_s_) {
      continue;
    }

    bool is_right_obs = obs_sl.end_l() < adc_frenet_l_;

    if (is_right_obs) {
      const double obs_center_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
      const auto& obs_reference_point =
          reference_line.GetNearestReferencePoint(obs_center_s);
      const auto& velocity = obstacle->Perception().velocity();
      const double obstacle_longitudinal_speed =
          common::math::Vec2d::CreateUnitVec2d(obs_reference_point.heading())
              .InnerProd(Vec2d(velocity.x(), velocity.y()));
      if (obstacle_longitudinal_speed > 0.0 && !obstacle->IsStatic()) {
        continue;
      }
      if (obstacle_longitudinal_speed < kMinReverseSpeed ||
          obstacle->IsStatic()) {
        double tem_end_state_l = obs_sl.end_l() + half_width + kLateralBuffer;
        if (tem_end_state_l > max_end_state_l) {
          max_end_state_l = tem_end_state_l;
          is_need_to_offset = true;
        }
      }
    } else {
      // no offset lef static obs
      continue;
    }
  }
  if (is_need_to_offset) {
    end_state_l = max_end_state_l;
  } else {
    double curr_lane_left_width = kNormalRoadWidth * 0.5;
    double curr_lane_right_width = kNormalRoadWidth * 0.5;
    reference_line.GetLaneWidth(adc_frenet_s_, &curr_lane_left_width,
                                &curr_lane_right_width);
    AINFO << "curr_lane_left_width = " << curr_lane_left_width
          << "   curr_lane_right_width = " << curr_lane_right_width;
    end_state_l = -curr_lane_right_width + half_width + kLateralBuffer;
  }
  return end_state_l;
}
hdmap::LaneInfoConstPtr PathBoundsDecider::LocateLaneInfo(
    const double s) const {
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  reference_line_info_->reference_line().GetLaneFromS(s, &lanes);
  if (lanes.empty()) {
    AWARN << "cannot get any lane using s";
    return nullptr;
  }

  return lanes.front();
}

void PathBoundsDecider::GetLaneBorrowInfoList(
    const ReferenceLineInfo& reference_line_info,
    std::vector<LaneBorrowInfo>* const lane_borrow_info_list) {
  lane_borrow_info_list->emplace_back(LaneBorrowInfo::NO_BORROW);

  if (reference_line_info.is_path_lane_borrow()) {
    const auto& path_decider_status =
        injector_->planning_context()->planning_status().path_decider();
    for (const auto& lane_borrow_direction :
         path_decider_status.decided_side_pass_direction()) {
      if (PathDeciderStatus::LEFT_BORROW == lane_borrow_direction) {
        lane_borrow_info_list->emplace_back(LaneBorrowInfo::LEFT_BORROW);
      } else if (PathDeciderStatus::RIGHT_BORROW == lane_borrow_direction) {
        lane_borrow_info_list->emplace_back(LaneBorrowInfo::RIGHT_BORROW);
      }
    }
  }

  const auto& indexed_obstacles =
      reference_line_info.path_decision().obstacles();
  injector_->ClearDisappearedObstacles();
  std::unordered_map<int32_t, int> perception_id;
  for (const auto* obstacle : indexed_obstacles.Items()) {
    if (perception_id.find(obstacle->PerceptionId()) == perception_id.end()) {
      perception_id.insert({obstacle->PerceptionId(), 1});
    }
  }
  for (auto& borrow_obs : injector_->GetBorrowObstacles()) {
    bool exist_obs =
        (nullptr != indexed_obstacles.Find(borrow_obs.second.Id())) ||
        (perception_id.find(borrow_obs.second.PerceptionId()) !=
         perception_id.end());
    ADEBUG << "Borrow obstacle ID: " << borrow_obs.second.Id()
           << ", Borrow obstacle count: " << borrow_obs.first
           << ", exist_obs: " << exist_obs;
    // If the obstacle disappears and the number of consecutive disappearances
    // is less than kMaxDisappearFrameNum cycles, update the relative position
    // of the obstacle and add it to the disappearing list
    if (!exist_obs && borrow_obs.first < kMaxDisappearFrameNum) {
      ++borrow_obs.first;
      UpdateObstacleSL(&borrow_obs.second,
                       reference_line_info.reference_line());
      injector_->AddDisappearedObstacle(borrow_obs);
      AWARN << "Borrow Obstacle disappear, add it to obstacles list.";
    }
  }
  ADEBUG << "DisappearObstacles size: "
         << injector_->GetDisappearObstacles().size();
}

void PathBoundsDecider::UpdateStartMovingObstacles(
    const ReferenceLineInfo& reference_line_info) {
  const auto& indexed_obstacles =
      reference_line_info.path_decision().obstacles();
  std::unordered_map<int32_t, const Obstacle*> indexed_obstacles_info;
  for (auto* obstacle : indexed_obstacles.Items()) {
    if (indexed_obstacles_info.find(obstacle->PerceptionId()) ==
        indexed_obstacles_info.end()) {
      indexed_obstacles_info.insert({obstacle->PerceptionId(), obstacle});
    }
  }
  auto iter = injector_->GetStartMovingObstacles().begin();
  // remove obs in the list when
  // 1. obs disappeared;
  // 2. obs keep moving over time threshold;
  // 3. obs move faster than adc;
  // 4. obs change to static.
  while (iter != injector_->GetStartMovingObstacles().end()) {
    auto indexed_obstacle_iter =
        indexed_obstacles_info.find(iter->second.second.PerceptionId());
    bool is_obs_disappear =
        (nullptr == indexed_obstacles.Find(iter->second.second.Id())) &&
        (indexed_obstacles_info.end() == indexed_obstacle_iter);
    if (is_obs_disappear) {
      ADEBUG << "Obs perception_id: " << iter->second.second.PerceptionId()
             << " disappeared, delete it.";
      injector_->GetStartMovingObstacles().erase(iter++);
      continue;
    } else {
      ++(iter->second.first);
      iter->second.second.SetPerceptionSlBoundary(
          indexed_obstacle_iter->second->PerceptionSLBoundary());
    }
    if (iter->second.first > config_.path_bounds_decider_config()
                                 .moving_obstacle_keep_time_threshold()) {
      ADEBUG << "Obs perception_id: " << iter->second.second.PerceptionId()
             << " keep moving count: " << iter->second.first;
      injector_->GetStartMovingObstacles().erase(iter++);
      continue;
    }
    if (indexed_obstacle_iter->second->speed() >
        reference_line_info.vehicle_state().linear_velocity()) {
      ADEBUG << "Obs perception_id: " << iter->second.second.PerceptionId()
             << " speed: " << indexed_obstacle_iter->second->speed()
             << ", adc_speed: "
             << reference_line_info.vehicle_state().linear_velocity();
      injector_->GetStartMovingObstacles().erase(iter++);
      continue;
    }
    if (indexed_obstacle_iter->second->IsStatic()) {
      ADEBUG << "Obs perception_id: " << iter->second.second.PerceptionId()
             << " change to static, delete it.";
      injector_->GetStartMovingObstacles().erase(iter++);
      continue;
    }

    ++iter;
  }

  // add new obs into the list.
  for (auto& borrow_obs : injector_->GetBorrowObstacles()) {
    auto indexed_obstacle_iter =
        indexed_obstacles_info.find(borrow_obs.second.PerceptionId());
    bool is_obs_change_from_static_to_dynamic = false;
    if (indexed_obstacle_iter != indexed_obstacles_info.end()) {
      is_obs_change_from_static_to_dynamic =
          !indexed_obstacle_iter->second->IsStatic();
    }
    if (is_obs_change_from_static_to_dynamic) {
      auto start_moving_obs = *indexed_obstacle_iter->second;
      start_moving_obs.SetStaticToDynamic(true);
      injector_->AddStartMovingObstacle(borrow_obs.second.PerceptionId(),
                                        {1, start_moving_obs});
    }
  }
  ADEBUG << "StartMovingObstacles size: "
         << injector_->GetStartMovingObstacles().size();
}

bool PathBoundsDecider::IsNeedKeepSelfPathStraightBeforeLaneBorrow(
    const LaneBorrowInfo& lane_borrow_info, const Obstacle* obstacle) {
  if (REGULAR_PATH_BOUNDS != bounds_type_ ||
      LaneBorrowInfo::NO_BORROW != lane_borrow_info) {
    return false;
  }

  const auto& path_decider =
      injector_->planning_context()->planning_status().path_decider();
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  if (path_decider.is_adc_near_junction() ||
      path_decider.is_obs_near_junction() ||
      injector_->adc_in_junction_info_.first || injector_->is_auxiliary_road_) {
    mutable_laneborrow->set_self_path_keep_straight_count(0);
    return false;
  }
  if (LaneborrowStatus_Status::LaneborrowStatus_Status_PREPARE !=
          mutable_laneborrow->lane_borrow_status() &&
      LaneborrowStatus_Status::LaneborrowStatus_Status_DEFAULT !=
          mutable_laneborrow->lane_borrow_status()) {
    mutable_laneborrow->set_self_path_keep_straight_count(0);
    return false;
  }

  const auto& block_obs_list = mutable_laneborrow->block_obstacle_id();
  if (block_obs_list.empty()) {
    auto count = mutable_laneborrow->self_path_keep_straight_count();
    mutable_laneborrow->set_self_path_keep_straight_count(std::max(0, --count));
    return false;
  }
  if (obstacle->Id().empty()) {
    return false;
  }

  const auto& count_limit = config_.path_bounds_decider_config()
                                .self_path_keep_straight_count_limit();
  if (mutable_laneborrow->self_path_keep_straight_count() >= count_limit) {
    ADEBUG << "keep self path straight overtime, generate path bound, obs: "
           << obstacle->Id();
    return false;
  }
  const auto& obs_sl = obstacle->PerceptionSLBoundary();
  const auto& lane_width = reference_line_info_->GetLaneWidthBaseOnAdcCenter();
  double curr_lane_left_width = lane_width.first;
  double obs_start_l_limit =
      config_.path_bounds_decider_config().self_path_obs_start_l_limit();
  bool is_need_right_self_borrow = obs_sl.start_l() > obs_start_l_limit &&
                                   obs_sl.start_l() < curr_lane_left_width;
  if ((std::find(block_obs_list.begin(), block_obs_list.end(),
                 obstacle->Id()) != block_obs_list.end()) ||
      is_need_right_self_borrow) {
    mutable_laneborrow->set_need_keep_self_path_straight(true);
    return true;
  }
  return false;
}

void PathBoundsDecider::UpdateKeepSelfPathStraightCount() {
  auto* mutable_laneborrow = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_borrow();
  double adc_velocity =
      std::fabs(injector_->vehicle_state()->linear_velocity());
  double speed_limit = config_.path_bounds_decider_config()
                           .self_path_keep_straight_speed_limit();
  if (adc_velocity < speed_limit) {
    auto keep_straight_count =
        mutable_laneborrow->self_path_keep_straight_count();
    mutable_laneborrow->set_self_path_keep_straight_count(
        ++keep_straight_count);
  }
}

bool PathBoundsDecider::CanExtendCurrentLaneLeftBound(const double curr_s) {
  auto curr_lane_info = reference_line_info_->LocateLaneInfo(curr_s);
  if (!curr_lane_info) {
    return false;
  }
  auto left_boundary_type =
      curr_lane_info->lane().left_boundary().boundary_type(0).types(0);
  if (hdmap::LaneBoundaryType::DOTTED_WHITE == left_boundary_type ||
      hdmap::LaneBoundaryType::DOTTED_YELLOW == left_boundary_type) {
    ADEBUG << "can extend left bound, curr_s: " << curr_s;
    return true;
  }
  return false;
}

bool PathBoundsDecider::IsWillPassMergeLane() {
  return injector_->planning_context()
      ->planning_status()
      .path_decider()
      .will_pass_merge_lane_area();
}

bool PathBoundsDecider::IsNeedLaneChangePassMergeLane() {
  return IsWillPassMergeLane() && !reference_line_info_->IsAdcLocatedInLane();
}

void PathBoundsDecider::MakeObsDirectionDecision(const LaneBorrowInfo& lane_borrow_info,
    const Obstacle* obstacle, DirectionDecision* const decision) {
        const auto& obs_sl = obstacle->PerceptionSLBoundary();
  const auto& adc_sl = reference_line_info_->AdcSlBoundary();
  if (perception::PerceptionObstacle::WHEELCRANE ==
      obstacle->Perception().type()) {
    bool is_wheelcrane_out_lat_range =
        adc_sl.start_l() - obs_sl.end_l() > kConsiderLateralDistance ||
        obs_sl.start_l() - adc_sl.end_l() > kConsiderLateralDistance;
    // AINFO<<"IS WHEELCRANE";
    if ((LaneBorrowInfo::LEFT_BORROW == lane_borrow_info) &&
        !is_wheelcrane_out_lat_range) {
      *decision = DirectionDecision::LEFT;
    } else if (LaneBorrowInfo::RIGHT_BORROW == lane_borrow_info &&
               !is_wheelcrane_out_lat_range) {
      *decision = DirectionDecision::RIGHT;
    } else {
    }
  }
  if (REGULAR_PATH_BOUNDS != bounds_type_) {
    return;
  }
  if (!injector_->is_auxiliary_road_) {
    return;
  }


  bool obs_out_of_range = obs_sl.end_s() < adc_sl.end_s();
  if (obs_out_of_range) {
    return;
  }

  // obs located in the entrance of curb area.
  double adc_to_curb_distance =
      reference_line_info_->GetRemainDistanceToCurbArea();
  double adc_to_obs_distance = obs_sl.start_s() - adc_sl.end_s();
  const double adc_length =
      common::VehicleConfigHelper::GetConfig().vehicle_param().length();
  bool obs_near_curb = (adc_to_curb_distance - adc_to_obs_distance) <
                       adc_length * kAdcLengthRatio;
  if (obs_near_curb && obs_sl.start_l() > 0.0) {
    *decision = DirectionDecision::RIGHT;
    return;
  }
}

}  // namespace planning
}  // namespace century
