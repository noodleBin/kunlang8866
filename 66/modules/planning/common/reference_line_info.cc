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

/**
 * @file
 **/

#include "modules/planning/common/reference_line_info.h"

#include <algorithm>

#include "absl/strings/str_cat.h"

#include "modules/planning/proto/planning_status.pb.h"
#include "modules/planning/proto/sl_boundary.pb.h"

#include "cyber/task/task.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"

namespace century {
namespace planning {

using century::canbus::Chassis;
using century::common::EngageAdvice;
using century::common::TrajectoryPoint;
using century::common::VehicleConfigHelper;
using century::common::VehicleSignal;
using century::common::math::Box2d;
using century::common::math::Polygon2d;
using century::common::math::Vec2d;
using century::common::util::PointFactory;
using century::cyber::Clock;
using century::hdmap::Lane;
using century::perception::PerceptionObstacle;

namespace {
constexpr double kEpsilon = 1e-2;
constexpr double kMinTurnSignalDistance = 10.0;
constexpr double kMinTurnSignalTime = 4.0;
constexpr double kInvalidLength = 0.1;
constexpr double kTurnSignalDistanceBuffer = 0.5;
constexpr size_t kTurnSignalCountDelayBufferPublicRoad = 1;
constexpr size_t kTurnSignalCountDelayBufferPlayStreet = 10;
constexpr int kTurnSignalDelayFrameNumber = 15;
constexpr double kCarStraightenAngle = M_PI / 18.0;
constexpr double kSafeTimeOnSameDirection = 3.0;
constexpr double kSafeTimeOnOppositeDirection = 5.0;
constexpr double kForwardMinSafeDistanceOnSameDirection = 10.0;
constexpr double kBackwardMinSafeDistanceOnSameDirection = 10.0;
constexpr double kForwardMinSafeDistanceOnOppositeDirection = 50.0;
constexpr double kBackwardMinSafeDistanceOnOppositeDirection = 1.0;
constexpr double kLaneChangeBackCheckLevelRatio = 0.8;
constexpr double kLaneChangeForwardCheckLevelRatio = 0.6;
constexpr double kAdcKeepStraightLateralErr = 0.2;
constexpr double kSameDirectionThr = M_PI * 0.3;
constexpr double kDistanceBuffer = 0.5;
constexpr double kDistanceUseVehiclePolygon = 15.0;
constexpr double kMinSafeReactionDistance = 1.0;
constexpr double kLateralCutInBuffer = 0.3;
constexpr int kObstackeMinSize = 3;
constexpr double kForwardDistance = 40.0;
constexpr double kBackwardDistance = -15.0;
constexpr double kLateralDistanceBuffer = 3.5;
constexpr double kMutableLateralDistanceBuffer = 1.75;
constexpr int kMaxCrowdedRoad = 5;
constexpr int kGeneralCrowdedRoad = 2;
constexpr double kLonBuffer = 0.2;
constexpr double kJunctionDistanceBuffer = 20.0;
constexpr double kDegrees = 90.0;
constexpr double kLowerBound = 0.2;
constexpr double kUpperBound = 0.3;
constexpr double kStepForCheckSolidLine = 2.0;
constexpr double kPathResolution = 0.5;
constexpr double kMinObstacleLength = 0.2;
constexpr double kMinCoeff = 2.0;
constexpr double kValidRouteEndLateralDistance = 15.0;
constexpr double kLateralBuffer = 0.2;
constexpr double kMaxOverlapRange = 500.0;
constexpr double kJunctionSearchRadius = 5.0;
constexpr int kMinTrajectorySize = 3;
constexpr double kMinLateralSpeed = 0.1;
constexpr double kConsiderLateralDistance = 4.0;
constexpr double kConsiderFrontDistance = 15.0;
constexpr double kConsiderRearDistance = 5.0;
constexpr double kRadius = 3.5;
constexpr double kMinBackDistance = 5.0;
constexpr double kConsiderTimeForSpeedDiff = 3.0;
constexpr double kLateralBufferForBicycle = 0.2;
constexpr double kSlowBreakingTargetSpeed = 3.0;
constexpr double kLateralBufferForLaneWidth = 0.5;
constexpr double kLonCollisionbuffer = 3.0;
}  // namespace

std::unordered_map<std::string, bool>
    ReferenceLineInfo::junction_right_of_way_map_;
static HysteresisInterval obstacles_interval_left(0.0, -kLowerBound,
                                                  kUpperBound, 100UL);
static HysteresisInterval obstacles_interval_right(0.0, -kLowerBound,
                                                   kUpperBound, 100UL);
size_t ReferenceLineInfo::turn_left_count_ = 0UL;
size_t ReferenceLineInfo::turn_right_count_ = 0UL;
int ReferenceLineInfo::turn_left_continue_count_ = kTurnSignalDelayFrameNumber;
int ReferenceLineInfo::turn_right_continue_count_ = kTurnSignalDelayFrameNumber;

ReferenceLineInfo::ReferenceLineInfo(const common::VehicleState& vehicle_state,
                                     const TrajectoryPoint& adc_planning_point,
                                     const ReferenceLine& reference_line,
                                     const hdmap::RouteSegments& segments)
    : vehicle_state_(vehicle_state),
      adc_planning_point_(adc_planning_point),
      reference_line_(reference_line),
      lanes_(segments) {}

bool ReferenceLineInfo::Init(const std::vector<const Obstacle*>& obstacles) {
  const auto& param = VehicleConfigHelper::GetConfig().vehicle_param();
  // stitching point
  const auto& path_point = adc_planning_point_.path_point();
  Vec2d position(path_point.x(), path_point.y());
  Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) * 0.5,
      (param.left_edge_to_center() - param.right_edge_to_center()) * 0.5);
  Vec2d center(position + vec_to_center.rotate(path_point.theta()));
  Box2d box(center, path_point.theta(), param.length(), param.width());
  // realtime vehicle position
  Vec2d vehicle_position(vehicle_state_.x(), vehicle_state_.y());
  Vec2d vehicle_center(vehicle_position +
                       vec_to_center.rotate(vehicle_state_.heading()));
  Box2d vehicle_box(vehicle_center, vehicle_state_.heading(), param.length(),
                    param.width());

  if (!reference_line_.GetSLBoundary(vehicle_box, &adc_sl_boundary_)) {
    AERROR << "Failed to get ADC boundary from box: "
           << vehicle_box.DebugString();
    return false;
  }

  GetAdcBasedLaneWidth();
  InitFirstOverlaps();

  if (adc_sl_boundary_.end_s() < 0 ||
      adc_sl_boundary_.start_s() > reference_line_.Length()) {
    AWARN << "Vehicle SL " << adc_sl_boundary_.ShortDebugString()
          << " is not on reference line:[0, " << reference_line_.Length()
          << "]";
  }
  static constexpr double kOutOfReferenceLineL = 18.0;  // in meters
  if (adc_sl_boundary_.start_l() > kOutOfReferenceLineL ||
      adc_sl_boundary_.end_l() < -kOutOfReferenceLineL) {
    AERROR << "Ego vehicle is too far away from reference line.";
    return false;
  }
  is_on_reference_line_ = reference_line_.IsOnLane(adc_sl_boundary_);
  is_adc_posture_straight_ = CheckAdcPostureStraight();

  const auto& turn_type = GetPathTurnType(adc_sl_boundary_.end_s());

  if (hdmap::Lane::LEFT_TURN == turn_type) {
    is_at_turn_ = true;
    is_at_left_turn_ = true;
  } else if (hdmap::Lane::RIGHT_TURN == turn_type) {
    is_at_turn_ = true;
    is_at_right_turn_ = true;
  } else {
    is_at_turn_ = false;
  }

  if (!AddObstacles(obstacles)) {
    AERROR << "Failed to add obstacles to reference line";
    return false;
  }

  if (IsChangeLanePath() || !is_adc_located_in_lane_) {
    CheckIsClearToChangeLane();
    AINFO << "is changing lane path: " << IsChangeLanePath()
          << ", is clear safe: " << is_clear_to_change_lane_;
  } else {
    is_clear_to_change_lane_ = true;
    ADEBUG << "not changing lane, and adc located in lane, so clear safe.";
  }

  const auto& map_path = reference_line_.map_path();
  for (const auto& speed_bump : map_path.speed_bump_overlaps()) {
    // -1 and + 1.0 are added to make sure it can be sampled.
    reference_line_.AddSpeedLimit(speed_bump.start_s - 1.0,
                                  speed_bump.end_s + 1.0,
                                  FLAGS_speed_bump_speed_limit);
  }

  SetCruiseSpeed(FLAGS_default_cruise_speed);

  // set lattice planning target speed limit;
  SetLatticeCruiseSpeed(FLAGS_default_cruise_speed);

  vehicle_signal_.Clear();

  return true;
}

const std::vector<PathData>& ReferenceLineInfo::GetCandidatePathData() const {
  return candidate_path_data_;
}

void ReferenceLineInfo::SetCandidatePathData(
    std::vector<PathData>&& candidate_path_data) {
  candidate_path_data_ = std::move(candidate_path_data);
}

const std::vector<PathBoundary>& ReferenceLineInfo::GetCandidatePathBoundaries()
    const {
  return candidate_path_boundaries_;
}

void ReferenceLineInfo::SetCandidatePathBoundaries(
    std::vector<PathBoundary>&& path_boundaries) {
  candidate_path_boundaries_ = std::move(path_boundaries);
}

double ReferenceLineInfo::GetCruiseSpeed() const {
  // in playstreet or emergency pull over ,the cruise_speed down
  if (hdmap::Lane::PLAY_STREET == GetLaneType()) {
    ADEBUG << "use low cruise";
    return std::min(FLAGS_play_street_speed_limit, cruise_speed_);
  }
  return cruise_speed_ > 0.0 ? cruise_speed_ : FLAGS_default_cruise_speed;
}

hdmap::LaneInfoConstPtr ReferenceLineInfo::LocateLaneInfo(
    const double s) const {
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  reference_line_.GetLaneFromS(s, &lanes);
  if (lanes.empty()) {
    AWARN << "cannot get any lane using s";
    return nullptr;
  }
  // for(auto lane:lanes) {
  //   AINFO << "lane id: " << lane->id().id(
  // );}

  return lanes.front();
}

bool ReferenceLineInfo::HasNeighborLane(const double check_s) {
  const auto& locate_lane = LocateLaneInfo(check_s);
  if (locate_lane != nullptr) {
    if (!locate_lane->lane().left_neighbor_forward_lane_id().empty() ||
        !locate_lane->lane().right_neighbor_forward_lane_id().empty() ||
        !locate_lane->lane().left_neighbor_reverse_lane_id().empty() ||
        !locate_lane->lane().right_neighbor_reverse_lane_id().empty()) {
      return true;
    }
  }
  return false;
}

double ReferenceLineInfo::GetLeftNeighborLaneWidth(const double s) {
  hdmap::Id neighbor_lane_id;
  double left_lane_width = 0.0;
  if (!GetNeighborLaneInfo(ReferenceLineInfo::LaneType::LeftForward, s,
                           &neighbor_lane_id, &left_lane_width) &&
      !GetNeighborLaneInfo(ReferenceLineInfo::LaneType::LeftReverse, s,
                           &neighbor_lane_id, &left_lane_width)) {
    left_lane_width = 0.0;
  }
  return left_lane_width;
}
double ReferenceLineInfo::GetRightNeighborLaneWidth(const double s) {
  hdmap::Id neighbor_lane_id;
  double right_lane_width = 0.0;
  if (!GetNeighborLaneInfo(ReferenceLineInfo::LaneType::RightForward, s,
                           &neighbor_lane_id, &right_lane_width) &&
      !GetNeighborLaneInfo(ReferenceLineInfo::LaneType::RightReverse, s,
                           &neighbor_lane_id, &right_lane_width)) {
    right_lane_width = 0.0;
  }
  return right_lane_width;
}

bool ReferenceLineInfo::GetNeighborLaneInfo(
    const ReferenceLineInfo::LaneType lane_type, const double s,
    hdmap::Id* ptr_lane_id, double* ptr_lane_width) const {
  auto ptr_lane_info = LocateLaneInfo(s);
  if (ptr_lane_info == nullptr) {
    return false;
  }

  switch (lane_type) {
    case LaneType::LeftForward: {
      if (ptr_lane_info->lane().left_neighbor_forward_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().left_neighbor_forward_lane_id(0);
      break;
    }
    case LaneType::LeftReverse: {
      if (ptr_lane_info->lane().left_neighbor_reverse_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().left_neighbor_reverse_lane_id(0);
      break;
    }
    case LaneType::RightForward: {
      if (ptr_lane_info->lane().right_neighbor_forward_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().right_neighbor_forward_lane_id(0);
      break;
    }
    case LaneType::RightReverse: {
      if (ptr_lane_info->lane().right_neighbor_reverse_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().right_neighbor_reverse_lane_id(0);
      break;
    }
    default:
      ACHECK(false);
  }
  auto ptr_neighbor_lane =
      hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(*ptr_lane_id);
  if (ptr_neighbor_lane == nullptr) {
    return false;
  }

  auto ref_point = reference_line_.GetReferencePoint(s);

  double neighbor_s = 0.0;
  double neighbor_l = 0.0;
  if (!ptr_neighbor_lane->GetProjection({ref_point.x(), ref_point.y()},
                                        &neighbor_s, &neighbor_l)) {
    return false;
  }

  *ptr_lane_width = ptr_neighbor_lane->GetWidth(neighbor_s);
  return true;
}

bool ReferenceLineInfo::GetFirstOverlap(
    const std::vector<hdmap::PathOverlap>& path_overlaps,
    hdmap::PathOverlap* path_overlap) {
  CHECK_NOTNULL(path_overlap);
  const double start_s = adc_sl_boundary_.end_s();
  double overlap_min_s = kMaxOverlapRange;

  auto overlap_min_s_iter = path_overlaps.end();
  for (auto iter = path_overlaps.begin(); iter != path_overlaps.end(); ++iter) {
    if (iter->end_s < start_s) {
      continue;
    }
    if (overlap_min_s > iter->start_s) {
      overlap_min_s_iter = iter;
      overlap_min_s = iter->start_s;
    }
  }

  // Ensure that the path_overlaps is not empty.
  if (overlap_min_s_iter != path_overlaps.end()) {
    *path_overlap = *overlap_min_s_iter;
  }

  return overlap_min_s < kMaxOverlapRange;
}

bool ReferenceLineInfo::GetFirstTrafficLightJunction(
    hdmap::PathOverlap* path_overlap_junction) {
  CHECK_NOTNULL(path_overlap_junction);
  const double adc_start_s = adc_sl_boundary_.start_s();
  const double adc_end_s = adc_sl_boundary_.end_s();
  double overlap_min_s = kMaxOverlapRange;

  common::PointENU adc_point_enu;
  adc_point_enu.set_x(vehicle_state_.x());
  adc_point_enu.set_y(vehicle_state_.y());
  std::vector<hdmap::JunctionInfoConstPtr> junctions;
  if (0 != hdmap::HDMapUtil::BaseMapPtr()->GetJunctions(
               adc_point_enu, reference_line_.Length(), &junctions)) {
    return false;
  }

  bool has_traffic_light_junction = false;
  for (const auto& ptr_junction : junctions) {
    if (hdmap::Junction::COMMON_JUNCTION != ptr_junction->junction().type() &&
        hdmap::Junction::IN_ROAD != ptr_junction->junction().type()) {
      continue;
    }

    // Filter out non traffic light intersection junctions
    const auto& overlap_signal_ids = ptr_junction->OverlapSignalIds();
    if (overlap_signal_ids.empty()) {
      ADEBUG << "This junction is not traffic light intersection, junction id: "
             << ptr_junction->id().id();
      continue;
    }

    SLBoundary junction_sl;
    if (!reference_line_.GetSLBoundary(ptr_junction->junction().polygon(),
                                       &junction_sl)) {
      AERROR << "Failed to get sl boundary for traffic light junction: "
             << ptr_junction->id().id();
      continue;
    }

    if (junction_sl.end_s() < adc_start_s) {
      continue;
    }

    if (junction_sl.start_l() > 0 || junction_sl.end_l() < 0) {
      continue;
    }

    if (junction_sl.end_s() < adc_start_s ||
        junction_sl.start_s() > adc_end_s) {
      is_adc_in_traffic_light_junction_ = false;
    } else {
      is_adc_in_traffic_light_junction_ = true;
    }

    if (overlap_min_s > junction_sl.start_s()) {
      path_overlap_junction->object_id = ptr_junction->id().id();
      path_overlap_junction->start_s = junction_sl.start_s();
      path_overlap_junction->end_s = junction_sl.end_s();
      overlap_min_s = junction_sl.start_s();
      has_traffic_light_junction = true;
    }
  }

  return has_traffic_light_junction;
}

bool ReferenceLineInfo::GetFirstGateArea(
    hdmap::PathOverlap* path_overlap_gate_area) {
  CHECK_NOTNULL(path_overlap_gate_area);
  const double adc_start_s = adc_sl_boundary_.start_s();
  const double adc_end_s = adc_sl_boundary_.end_s();
  double overlap_min_s = kMaxOverlapRange;

  common::PointENU adc_point_enu;
  adc_point_enu.set_x(vehicle_state_.x());
  adc_point_enu.set_y(vehicle_state_.y());
  std::vector<hdmap::JunctionInfoConstPtr> junctions;
  if (0 != hdmap::HDMapUtil::BaseMapPtr()->GetJunctions(
               adc_point_enu, reference_line_.Length(), &junctions)) {
    return false;
  }

  bool has_gate_area = false;
  for (const auto& ptr_junction : junctions) {
    if (hdmap::Junction::GATE_AREA != ptr_junction->junction().type()) {
      continue;
    }

    SLBoundary junction_sl;
    if (!reference_line_.GetSLBoundary(ptr_junction->junction().polygon(),
                                       &junction_sl)) {
      AERROR << "Failed to get sl boundary for gate area: "
             << ptr_junction->id().id();
      continue;
    }

    if (junction_sl.end_s() < adc_start_s) {
      continue;
    }

    if (junction_sl.start_l() > 0 || junction_sl.end_l() < 0) {
      continue;
    }

    if (overlap_min_s > junction_sl.start_s()) {
      path_overlap_gate_area->object_id = ptr_junction->id().id();
      path_overlap_gate_area->start_s = junction_sl.start_s();
      path_overlap_gate_area->end_s = junction_sl.end_s();
      overlap_min_s = junction_sl.start_s();
      has_gate_area = true;
    }
  }
  if (has_gate_area) {
    if (path_overlap_gate_area->end_s < adc_start_s ||
        path_overlap_gate_area->start_s > adc_end_s) {
      is_adc_in_gate_area_ = false;
    } else {
      is_adc_in_gate_area_ = true;
      ADEBUG << "Adc is in gate area, id = "
             << path_overlap_gate_area->object_id;
    }
  }

  return has_gate_area;
}

bool ReferenceLineInfo::GetFirstDriveArea(
    hdmap::PathOverlap* path_overlap_drive_area) {
  CHECK_NOTNULL(path_overlap_drive_area);
  const double adc_start_s = adc_sl_boundary_.start_s();
  const double adc_end_s = adc_sl_boundary_.end_s();
  double overlap_min_s = kMaxOverlapRange;

  common::PointENU adc_point_enu;
  adc_point_enu.set_x(vehicle_state_.x());
  adc_point_enu.set_y(vehicle_state_.y());
  std::vector<hdmap::JunctionInfoConstPtr> junctions;
  if (0 != hdmap::HDMapUtil::BaseMapPtr()->GetJunctions(
               adc_point_enu, reference_line_.Length(), &junctions)) {
    return false;
  }

  bool has_drive_area = false;
  for (const auto& ptr_junction : junctions) {
    if (hdmap::Junction::DRIVE_AREA != ptr_junction->junction().type()) {
      continue;
    }

    SLBoundary junction_sl;
    if (!reference_line_.GetSLBoundary(ptr_junction->junction().polygon(),
                                       &junction_sl)) {
      AERROR << "Failed to get sl boundary for gate area: "
             << ptr_junction->id().id();
      continue;
    }

    if (junction_sl.end_s() < adc_start_s) {
      continue;
    }

    if (junction_sl.start_l() > 0 || junction_sl.end_l() < 0) {
      continue;
    }

    if (overlap_min_s > junction_sl.start_s()) {
      path_overlap_drive_area->object_id = ptr_junction->id().id();
      path_overlap_drive_area->start_s = junction_sl.start_s();
      path_overlap_drive_area->end_s = junction_sl.end_s();
      overlap_min_s = junction_sl.start_s();
      has_drive_area = true;
    }
  }

  if (has_drive_area) {
    if (path_overlap_drive_area->end_s < adc_start_s ||
        path_overlap_drive_area->start_s > adc_end_s) {
      is_adc_in_drive_area_ = false;
    } else {
      is_adc_in_drive_area_ = true;
      AERROR << "Adc is in drive area, id = "
             << path_overlap_drive_area->object_id;
    }
  }

  return has_drive_area;
}

void ReferenceLineInfo::GetRoadWidthBasedAdc(
    double* const road_left_width, double* const road_right_width) const {
  CHECK_NOTNULL(road_left_width);
  CHECK_NOTNULL(road_right_width);

  *road_left_width = FLAGS_default_reference_line_width * 0.5;
  *road_right_width = FLAGS_default_reference_line_width * 0.5;
  double ego_start_s = adc_sl_boundary_.start_s();
  double ego_end_s = adc_sl_boundary_.end_s();
  if (!reference_line_.GetRoadWidth((ego_start_s + ego_end_s) * 0.5,
                                    road_left_width, road_right_width)) {
    AWARN << "Failed to get road width at s = "
          << (ego_start_s + ego_end_s) * 0.5;
  }
  return;
}

std::pair<double, double> ReferenceLineInfo::GetLaneWidthByS(const double s) const {
  double left_width = FLAGS_default_reference_line_width * 0.5;
  double right_width = FLAGS_default_reference_line_width * 0.5;
  if (!reference_line_.GetLaneWidth(s, &left_width, &right_width)) {
    AWARN << "Failed to get lane width at s = " << s;
  }
  return {left_width, right_width};
}

double ReferenceLineInfo::GetHeadingByS(const double s) const {
  auto point = reference_line_.GetReferencePoint(s);
  return point.heading();
}

void ReferenceLineInfo::GetAdcBasedLaneWidth() {
  double left_width = FLAGS_default_reference_line_width * 0.5;
  double right_width = FLAGS_default_reference_line_width * 0.5;
  double ego_start_s = adc_sl_boundary_.start_s();
  double ego_end_s = adc_sl_boundary_.end_s();
  if (!reference_line_.GetLaneWidth((ego_start_s + ego_end_s) * 0.5,
                                    &left_width, &right_width)) {
    AWARN << "Failed to get lane width at s = "
          << (ego_start_s + ego_end_s) * 0.5;
  }
  lane_width_base_on_adc_center_.first = left_width;
  lane_width_base_on_adc_center_.second = right_width;

  is_adc_located_in_lane_ = (adc_sl_boundary_.start_l() > -right_width &&
                             adc_sl_boundary_.end_l() < left_width);
  is_adc_cover_in_lane_ =
      (adc_sl_boundary_.start_l() > -right_width - kLateralBufferForLaneWidth &&
       adc_sl_boundary_.end_l() < left_width + kLateralBufferForLaneWidth);
  common::SLPoint adc_sl_point;
  adc_sl_point.set_s((ego_start_s + ego_end_s) * 0.5);
  adc_sl_point.set_l((adc_sl_boundary_.start_l() + adc_sl_boundary_.end_l()) *
                     0.5);
  is_adc_center_in_lane_ = reference_line_.IsOnLane(adc_sl_point);
  return;
}

bool ReferenceLineInfo::AdcIsOnRouteLane(
    PlanningContext* planning_context) const {
  if (!is_adc_located_in_lane_) {
    return false;
  }

  double s = (adc_sl_boundary_.start_s() + adc_sl_boundary_.end_s()) * 0.5;
  hdmap::LaneInfoConstPtr locate_lane = LocateLaneInfo(s);

  if (locate_lane != nullptr) {
    auto& lane = locate_lane->lane();
    std::string ego_lane_id = lane.id().id();
    auto* preferred_lane_ids = planning_context->mutable_planning_status()
                                   ->mutable_overtake()
                                   ->mutable_preferred_lane_ids();

    for (const auto& lane_id : *preferred_lane_ids) {
      if (ego_lane_id == lane_id) {
        return true;
      }
    }
  }
  return false;
}

bool ReferenceLineInfo::AdcIsOnAllRouteLane(
    PlanningContext* planning_context) const {
  common::PointENU adc_pose;
  adc_pose.set_x(vehicle_state_.x());
  adc_pose.set_y(vehicle_state_.y());
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  if (0 !=
      hdmap::HDMapUtil::BaseMapPtr()->GetLanes(adc_pose, kRadius, &lanes)) {
    return false;
  }

  if (lanes.empty()) {
    return false;
  }

  auto* all_route_lane_ids = planning_context->mutable_planning_status()
                                 ->mutable_overtake()
                                 ->mutable_all_route_lane_ids();

  for (size_t j = 0; j < lanes.size(); ++j) {
    double s = 0.0;
    double l = 0.0;
    lanes[j]->GetProjection({adc_pose.x(), adc_pose.y()}, &s, &l);

    // Filter lane that do not in
    if (s < 0 || s > lanes[j]->total_length()) {
      continue;
    }

    double left_width = 0.0;
    double right_width = 0.0;
    lanes[j]->GetWidth(s, &left_width, &right_width);
    if (l > left_width || l < -right_width) {
      continue;
    }

    for (const auto& lane_id : *all_route_lane_ids) {
      if (lanes[j]->id().id() == lane_id) {
        ADEBUG << "adc is on all route ids, lane id =" << lane_id;
        return true;
      }
    }
  }

  return false;
}

bool ReferenceLineInfo::AdcIsOnOverlapJunction() const {
  for (const auto& overlap : first_encounter_overlaps_) {
    if (ReferenceLineInfo::JUNCTION == overlap.first) {
      if (!(adc_sl_boundary_.start_s() < overlap.second.start_s ||
            adc_sl_boundary_.end_s() > overlap.second.end_s)) {
        ADEBUG << "adc is on overlap junction, junction id = "
               << overlap.second.object_id;
        return true;
      }
    }
  }
  return false;
}

bool ReferenceLineInfo::GetRoutePassageEndPoint(
    PlanningContext* planning_context, PassageType passage_type,
    common::PointENU* const end_point) {
  if (nullptr == planning_context || nullptr == end_point) {
    ADEBUG << "nullptr == planning_context or nullptr == end_point.";
    return false;
  }

  std::string start_lane_id = "";
  if (!GetStartLaneID(passage_type, &start_lane_id)) {
    ADEBUG << "Failed get start lane id.";
    return false;
  }

  // Find start passage index
  const auto& route_lane_index = planning_context->route_lane_index();
  const auto iter = route_lane_index.find(start_lane_id);
  if (iter == route_lane_index.end()) {
    ADEBUG << "Failed find passage index and lane index.";
    return false;
  }

  const auto& route_lane_info = planning_context->route_lane_info();

  for(auto iter_lane = route_lane_info[iter->second.first].begin();
       iter_lane != route_lane_info[iter->second.first].end(); ++iter_lane){
        // AINFO<<"PASSAGE ID = "<<iter_lane->lane_id;
       }
  const auto ptr_passage_end_lane = hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(
      hdmap::MakeMapId(route_lane_info[iter->second.first].rbegin()->lane_id));
  if (nullptr == ptr_passage_end_lane) {
    ADEBUG << "Failed get passage end lane ptr.";
    return false;
  } else {
    if(!ptr_passage_end_lane->lane().successor_id().empty()){
      auto end_lane_successor = ptr_passage_end_lane->lane().successor_id();
      for (auto iter_successor = end_lane_successor.begin();
           iter_successor != end_lane_successor.end(); ++iter_successor) {
        const auto iter_second = route_lane_index.find(iter_successor->id());
        if (iter_second == route_lane_index.end()) {
          continue;
        } else {
          const auto ptr_passage_end_lane_second =
              hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(
                  hdmap::MakeMapId(route_lane_info[iter_second->second.first]
                                       .rbegin()
                                       ->lane_id));
          if (nullptr == ptr_passage_end_lane_second) {
            ADEBUG << "Failed get passage end lane ptr.";
            return false;
          } else {
            GetEndPoint(ptr_passage_end_lane_second, end_point);
            ADEBUG
                << "Passage end lane id = "
                << route_lane_info[iter_second->second.first].rbegin()->lane_id
                << " passage end point = " << end_point->DebugString();
            return true;
          }
        }
      }

      // return first passage end lane  pint if can not find second passage
      // end lane
      GetEndPoint(ptr_passage_end_lane, end_point);
      ADEBUG << "Passage end lane id = "
             << route_lane_info[iter->second.first].rbegin()->lane_id
             << " passage end point = " << end_point->DebugString();
      return true;
    } else {
      GetEndPoint(ptr_passage_end_lane, end_point);
      ADEBUG << "Passage end lane id = "
             << route_lane_info[iter->second.first].rbegin()->lane_id
             << " passage end point = " << end_point->DebugString();
      return true;
    }
  }
}

void ReferenceLineInfo::GetEndPoint(hdmap::LaneInfoConstPtr lane_info_ptr,
                                    common::PointENU* const end_point) {
  const auto last_point_ptr = lane_info_ptr->lane()
                                  .central_curve()
                                  .segment()
                                  .rbegin()
                                  ->line_segment()
                                  .point()
                                  .rbegin();
  end_point->set_x(last_point_ptr->x());
  end_point->set_y(last_point_ptr->y());
  end_point->set_z(last_point_ptr->z());
}

bool ReferenceLineInfo::GetStartLaneID(PassageType passage_type,
                                       std::string* const start_lane_id) {
  double s = (adc_sl_boundary_.start_s() + adc_sl_boundary_.end_s()) * 0.5;
  hdmap::LaneInfoConstPtr locate_lane = LocateLaneInfo(s);
  if (nullptr == locate_lane) {
    AERROR << "Failed get adc locate lane.";
    return false;
  }
  auto& lane = locate_lane->lane();
  *start_lane_id = lane.id().id();
  ADEBUG << "Adc lane id = " << *start_lane_id;

  // If get left neighbor passage end point or get right neighbor passage end
  // point
  if (PassageType::LEFT == passage_type) {
    if (lane.left_neighbor_forward_lane_id().empty()) {
      AERROR << "Adc lane have no left neighbor forward lane.";
      return false;
    } else {
      *start_lane_id = lane.left_neighbor_forward_lane_id(0).id();
      ADEBUG << "Left neighbor forwoard lane id = " << start_lane_id;
    }
  } else if (PassageType::RIGHT == passage_type) {
    if (lane.right_neighbor_forward_lane_id().empty()) {
      AERROR << "Adc lane have no right neighbor forward lane.";
      return false;
    } else {
      *start_lane_id = lane.right_neighbor_forward_lane_id(0).id();
      ADEBUG << "Right neighbor forwoard lane id = " << start_lane_id;
    }
  } else {
    // nothing to do, start lane id defaults to the self lane id.
  }

  return true;
}

bool ReferenceLineInfo::CheckIsHeadbang(const double obstacle_distance,
                                        const double check_l) {
  const auto init_frenet_state =
      reference_line_.ToFrenetFrame(adc_planning_point_);
  ADEBUG << "init_frenet_state l=" << init_frenet_state.second[0]
         << " | dl=" << init_frenet_state.second[1]
         << " | ddl=" << init_frenet_state.second[2]
         << ", obstacle_distance: " << obstacle_distance;
  std::array<double, 3> end_d = {check_l, 0.0, 0.0};
  QuinticPolynomialCurve1d path_points = QuinticPolynomialCurve1d(
      init_frenet_state.second, end_d, obstacle_distance);
  ADEBUG << "QuinticPolynomialCurve1d OK!!!";
  std::vector<common::FrenetFramePoint> frenet_path_points;
  int i = 0;
  while (i * kPathResolution < obstacle_distance) {
    const double l = path_points.Evaluate(0, i * kPathResolution);
    const double dl = path_points.Evaluate(1, i * kPathResolution);
    const double ddl = path_points.Evaluate(2, i * kPathResolution);
    common::FrenetFramePoint frenet_point;
    frenet_point.set_s(
        i * kPathResolution + adc_sl_boundary_.start_s() +
        VehicleConfigHelper::GetConfig().vehicle_param().back_edge_to_center());
    frenet_point.set_l(l);
    frenet_point.set_dl(dl);
    frenet_point.set_ddl(ddl);
    frenet_path_points.emplace_back(frenet_point);
    i += 1;
  }
  // Get last frenet frame point
  {
    const double l = path_points.Evaluate(0, obstacle_distance);
    const double dl = path_points.Evaluate(1, obstacle_distance);
    const double ddl = path_points.Evaluate(2, obstacle_distance);
    common::FrenetFramePoint frenet_point;
    frenet_point.set_s(obstacle_distance +
                       (adc_sl_boundary_.start_s() + adc_sl_boundary_.end_s()) *
                           0.5);
    frenet_point.set_l(l);
    frenet_point.set_dl(dl);
    frenet_point.set_ddl(ddl);
    frenet_path_points.emplace_back(frenet_point);
  }
  PathData path_data;
  path_data.SetReferenceLine(&reference_line_);
  path_data.SetFrenetPath(FrenetFramePath(std::move(frenet_path_points)));

  // Get max kappa
  double max_kappa = 0.0;
  for (auto& path_point : path_data.discretized_path()) {
    if (std::fabs(path_point.kappa()) > std::fabs(max_kappa)) {
      max_kappa = path_point.kappa();
    }
  }
  // Get max centripetal acceleration
  double max_centripetal_acceleration =
      std::fabs(max_kappa) * std::pow(vehicle_state_.linear_velocity(), 2);
  ADEBUG << "max_kappa = " << max_kappa
         << " max_centripetal_acceleration = " << max_centripetal_acceleration;
  if (max_centripetal_acceleration >
      FLAGS_max_centripetal_acceleration_threshold) {
    ADEBUG << "CheckIsHeadbang is true";
    return true;
  } else {
    return false;
  }
}

bool ReferenceLineInfo::CheckAdcPostureStraight() {
  // check heading
  double heading_diff = GetAdcHeadingDiffWithRefLine();
  adc_longitudinal_velocity_ =
      vehicle_state_.linear_velocity() * std::cos(heading_diff);
  adc_lateral_velocity_ =
      vehicle_state_.linear_velocity() * std::sin(heading_diff);
  if (std::fabs(heading_diff) > kCarStraightenAngle) {
    return false;
  }

  // check center position
  double left_width = lane_width_base_on_adc_center_.first;
  double right_width = lane_width_base_on_adc_center_.second;
  double center_l =
      (adc_sl_boundary_.start_l() + adc_sl_boundary_.end_l()) * 0.5;

  if (center_l < -right_width * FLAGS_check_lane_width_ratio ||
      center_l > left_width * FLAGS_check_lane_width_ratio) {
    return false;
  }
  return true;
}

void ReferenceLineInfo::SetRiskShiftResult(const double shift_value,
                                           const double confidence) {
  risk_shift_result_.first = shift_value;
  risk_shift_result_.second = confidence;
}

void ReferenceLineInfo::InitFirstOverlaps() {
  const auto& map_path = reference_line_.map_path();
  // clear_zone
  hdmap::PathOverlap clear_area_overlap;
  if (GetFirstOverlap(map_path.clear_area_overlaps(), &clear_area_overlap)) {
    first_encounter_overlaps_.emplace_back(CLEAR_AREA, clear_area_overlap);
  }

  // crosswalk
  hdmap::PathOverlap crosswalk_overlap;
  if (GetFirstOverlap(map_path.crosswalk_overlaps(), &crosswalk_overlap)) {
    first_encounter_overlaps_.emplace_back(CROSSWALK, crosswalk_overlap);
  }

  // pnc_junction
  hdmap::PathOverlap pnc_junction_overlap;
  if (GetFirstOverlap(map_path.pnc_junction_overlaps(),
                      &pnc_junction_overlap)) {
    first_encounter_overlaps_.emplace_back(PNC_JUNCTION, pnc_junction_overlap);
  }

  // junction
  hdmap::PathOverlap junction_overlap;
  if (GetFirstTrafficLightJunction(&junction_overlap)) {
    first_encounter_overlaps_.emplace_back(JUNCTION, junction_overlap);
  }

  // gate_area
  hdmap::PathOverlap gate_area_overlap;
  if (GetFirstGateArea(&gate_area_overlap)) {
    first_encounter_overlaps_.emplace_back(GATE_AREA, gate_area_overlap);
    ADEBUG << "first_encounter_overlaps_  added gate area, id = "
           << gate_area_overlap.object_id;
  }

  // drive_area
  hdmap::PathOverlap drive_area_overlap;
  if (IsInPlayStreet(adc_sl_boundary_.start_s()) ||
      IsInPlayStreet(adc_sl_boundary_.end_s())) {
    if (GetFirstDriveArea(&drive_area_overlap)) {
      first_encounter_overlaps_.emplace_back(DRIVE_AREA, drive_area_overlap);
      ADEBUG << "first_encounter_overlaps_  added drive area, id = "
             << drive_area_overlap.object_id;
    }
  }

  // signal
  hdmap::PathOverlap signal_overlap;
  if (GetFirstOverlap(map_path.signal_overlaps(), &signal_overlap)) {
    first_encounter_overlaps_.emplace_back(SIGNAL, signal_overlap);
  }

  // stop_sign
  hdmap::PathOverlap stop_sign_overlap;
  if (GetFirstOverlap(map_path.stop_sign_overlaps(), &stop_sign_overlap)) {
    hdmap::StopSignInfoConstPtr stop_sign_ptr =
        hdmap::HDMapUtil::BaseMap().GetStopSignById(
            hdmap::MakeMapId(stop_sign_overlap.object_id));

    if (nullptr != stop_sign_ptr &&
        hdmap::StopSign::STOP == stop_sign_ptr->stop_sign().motion_type()) {
      first_encounter_overlaps_.emplace_back(STOP_SIGN, stop_sign_overlap);
    }
  }

  // yield_sign
  hdmap::PathOverlap yield_sign_overlap;
  if (GetFirstOverlap(map_path.yield_sign_overlaps(), &yield_sign_overlap)) {
    first_encounter_overlaps_.emplace_back(YIELD_SIGN, yield_sign_overlap);
  }

  // sort by start_s
  if (!first_encounter_overlaps_.empty()) {
    std::sort(first_encounter_overlaps_.begin(),
              first_encounter_overlaps_.end(),
              [](const std::pair<OverlapType, hdmap::PathOverlap>& a,
                 const std::pair<OverlapType, hdmap::PathOverlap>& b) {
                return a.second.start_s < b.second.start_s;
              });
  }
}

bool ReferenceLineInfo::GetJunctionRange(double* start_s, double* end_s) const {
  for (const auto& overlap : first_encounter_overlaps_) {
    ADEBUG << overlap.first << ", " << overlap.second.DebugString();
    if (ReferenceLineInfo::JUNCTION == overlap.first) {
      *start_s = overlap.second.start_s;
      *end_s = overlap.second.end_s;
      return true;
    }
  }
  return false;
}

bool ReferenceLineInfo::IsSRangeInCommonJunction(const double start_s,
                                                 const double end_s) const {
  double junction_start_s = 0.0, junction_end_s = 0.0;
  if (!GetJunctionRange(&junction_start_s, &junction_end_s)) {
    return false;
  }
  if ((start_s > junction_start_s && start_s < junction_end_s) ||
      (end_s > junction_start_s && end_s < junction_end_s)) {
    return true;
  }
  return false;
}

bool ReferenceLineInfo::IsObstacleOverlapWithJunction(
    const Obstacle& obs, const hdmap::JunctionInfo& junction_info) const {
  // Check whether Junction's polygon contain Obstacle bounding box.
  const auto& junction_polygon = junction_info.polygon();
  return junction_polygon.HasOverlap(obs.PerceptionPolygon());
}

bool ReferenceLineInfo::IsObstacleInCommonJunction(const Obstacle& obs) const {
  common::PointENU obs_point_enu;
  obs_point_enu.set_x(obs.Perception().position().x());
  obs_point_enu.set_y(obs.Perception().position().y());
  std::vector<hdmap::JunctionInfoConstPtr> junctions;
  if (0 == hdmap::HDMapUtil::BaseMapPtr()->GetJunctions(
               obs_point_enu, kJunctionSearchRadius, &junctions)) {
    for (const auto& ptr_junction : junctions) {
      if (hdmap::Junction::COMMON_JUNCTION == ptr_junction->junction().type() &&
          IsObstacleOverlapWithJunction(obs, *ptr_junction)) {
        return true;
      }
    }
  } else {
    ADEBUG << "Fail to get junctions from base_map.";
  }
  return false;
}

bool ReferenceLineInfo::IsObstacleInGateArea(const Obstacle& obs) const {
  for (const auto& overlap : first_encounter_overlaps_) {
    ADEBUG << "junction id: " << overlap.second.object_id;
    if (ReferenceLineInfo::GATE_AREA == overlap.first) {
      const auto junction_id = hdmap::MakeMapId(overlap.second.object_id);
      const auto ptr_junction =
          hdmap::HDMapUtil::BaseMapPtr()->GetJunctionById(junction_id);
      if (IsObstacleOverlapWithJunction(obs, *ptr_junction)) {
        return true;
      }
    }
  }
  return false;
}

bool ReferenceLineInfo::IsObstacleInDriveArea(const Obstacle& obs) const {
  for (const auto& overlap : first_encounter_overlaps_) {
    ADEBUG << "junction id: " << overlap.second.object_id;
    if (ReferenceLineInfo::DRIVE_AREA == overlap.first) {
      const auto junction_id = hdmap::MakeMapId(overlap.second.object_id);
      const auto ptr_junction =
          hdmap::HDMapUtil::BaseMapPtr()->GetJunctionById(junction_id);
      if (IsObstacleOverlapWithJunction(obs, *ptr_junction)) {
        return true;
      }
    }
  }
  return false;
}
bool ReferenceLineInfo::IsInPlayStreet(const double s) {
  auto ref_point = reference_line_.GetNearestReferencePoint(s);
  if (ref_point.lane_waypoints().empty()) {
    const std::string msg = "Fail to get reference point.";
    AERROR << msg;
    return false;
  }
  const auto& waypoint = ref_point.lane_waypoints().front();
  return hdmap::Lane::PLAY_STREET == waypoint.lane->lane().type();
}

bool WithinOverlap(const hdmap::PathOverlap& overlap, double s) {
  return overlap.start_s - kEpsilon <= s && s <= overlap.end_s + kEpsilon;
}

void ReferenceLineInfo::SetJunctionRightOfWay(const double junction_s,
                                              const bool is_protected) const {
  for (const auto& overlap : reference_line_.map_path().junction_overlaps()) {
    if (WithinOverlap(overlap, junction_s)) {
      junction_right_of_way_map_[overlap.object_id] = is_protected;
    }
  }
}

ADCTrajectory::RightOfWayStatus ReferenceLineInfo::GetRightOfWayStatus() const {
  for (const auto& overlap : reference_line_.map_path().junction_overlaps()) {
    if (overlap.end_s < adc_sl_boundary_.start_s()) {
      junction_right_of_way_map_.erase(overlap.object_id);
    } else if (WithinOverlap(overlap, adc_sl_boundary_.end_s())) {
      auto is_protected = junction_right_of_way_map_[overlap.object_id];
      if (is_protected) {
        return ADCTrajectory::PROTECTED;
      }
    }
  }
  return ADCTrajectory::UNPROTECTED;
}

const hdmap::RouteSegments& ReferenceLineInfo::Lanes() const { return lanes_; }

std::list<hdmap::Id> ReferenceLineInfo::TargetLaneId() const {
  std::list<hdmap::Id> lane_ids;
  for (const auto& lane_seg : lanes_) {
    lane_ids.push_back(lane_seg.lane->id());
  }
  return lane_ids;
}

const SLBoundary& ReferenceLineInfo::AdcSlBoundary() const {
  return adc_sl_boundary_;
}

bool ReferenceLineInfo::AdcSlBoundaryAt(
    const common::PathPoint& path_point,
    SLBoundary* const adc_sl_boundary) const {
  if (adc_sl_boundary == nullptr) {
    AERROR << "The output pointer adc_sl_boundary is null.";
    return false;
  }

  const auto& adc_box =
      common::VehicleConfigHelper::Instance()->GetBoundingBox(path_point);

  return reference_line_.GetSLBoundary(adc_box, adc_sl_boundary);
}

PathDecision* ReferenceLineInfo::path_decision() { return &path_decision_; }

const PathDecision& ReferenceLineInfo::path_decision() const {
  return path_decision_;
}

const ReferenceLine& ReferenceLineInfo::reference_line() const {
  return reference_line_;
}

ReferenceLine* ReferenceLineInfo::mutable_reference_line() {
  return &reference_line_;
}

void ReferenceLineInfo::SetTrajectory(const DiscretizedTrajectory& trajectory) {
  discretized_trajectory_ = trajectory;
}

bool ReferenceLineInfo::AddObstacleHelper(
    const std::shared_ptr<Obstacle>& obstacle) {
  return AddObstacle(obstacle.get()) != nullptr;
}

bool ReferenceLineInfo::GetObstacleSLBoundary(const Obstacle& obstacle,
                                              SLBoundary* const perception_sl,
                                              bool is_need_prebuild) const {
  if (!obstacle.HasTrajectory()) {
    return false;
  }
  const auto& trajectory = obstacle.Trajectory();
  if (trajectory.trajectory_point().size() < kMinTrajectorySize ||
      FLAGS_prediction_trajectory_relative_time_index >
          trajectory.trajectory_point().size()) {
    return false;
  }
  const auto& trajectory_point = trajectory.trajectory_point(
      FLAGS_prediction_trajectory_relative_time_index);
  // double trajectory_point_time = trajectory_point.relative_time();
  // AINFO << "trajectory_point_time = " << trajectory_point_time;
  Polygon2d obs_polygon;
  obs_polygon = obstacle.GetPolygon(trajectory_point);
  century::hdmap::Polygon polygon;
  for (const auto& point : obs_polygon.points()) {
    century::common::PointENU* hdmap_point = polygon.add_point();
    hdmap_point->set_x(point.x());
    hdmap_point->set_y(point.y());
  }
  if (!reference_line_.GetSLBoundary(polygon, perception_sl)) {
    AERROR << "Failed to get sl boundary for obstacle: " << obstacle.Id();
    return false;
  }

  return true;
}

bool ReferenceLineInfo::GetObstacleSLBoundary(
    const Obstacle& obstacle, SLBoundary* const perception_sl) const {
  bool is_unknown_obs =
      perception::PerceptionObstacle::UNKNOWN == obstacle.Perception().type() ||
      perception::PerceptionObstacle::UNKNOWN_MOVABLE ==
          obstacle.Perception().type() ||
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE ==
          obstacle.Perception().type();
  bool is_stacler =
      perception::PerceptionObstacle::STACKER == obstacle.Perception().type() ||
          PerceptionObstacle::FORKLIFT_STACKER == obstacle.Perception().type();
  if (FLAGS_enable_create_sl_boundary_using_polygon) {
    bool is_close_vehicle = false;
    if (perception::PerceptionObstacle::VEHICLE ==
        obstacle.Perception().type()) {
      const auto& obs_pose = obstacle.Perception().position();
      double dx = obs_pose.x() - vehicle_state_.x();
      double dy = obs_pose.y() - vehicle_state_.y();
      double obs_distance = std::sqrt(dx * dx + dy * dy);
      is_close_vehicle = obs_distance < kDistanceUseVehiclePolygon;
    }
    if (is_unknown_obs || is_close_vehicle ||is_stacler) {
      century::hdmap::Polygon polygon;
      for (const auto& point : obstacle.PerceptionPolygon().points()) {
        century::common::PointENU* hdmap_point = polygon.add_point();
        hdmap_point->set_x(point.x());
        hdmap_point->set_y(point.y());
      }
      bool need_trim = is_unknown_obs && obstacle.IsStatic();
      if (FLAGS_enable_trim_unknown_obstacle && need_trim) {
        // AINFO << "obstacle = " << obstacle.Id();
        // AINFO << "need trim";
        if (!reference_line_.TrimOutsideRoadPartSL(polygon, perception_sl)) {
          AERROR << "Failed to trim unknown obstacle: " << obstacle.Id();
          return false;
        }
      } else if (!reference_line_.GetSLBoundary(polygon, perception_sl)) {
        AERROR << "Failed to get sl boundary for obstacle: " << obstacle.Id();
        return false;
      }
    } else {
      if (!reference_line_.GetSLBoundary(obstacle.PerceptionBoundingBox(),
                                         perception_sl)) {
        AERROR << "Failed to get sl boundary for obstacle: " << obstacle.Id();
        return false;
      }
    }
  } else {
    if (is_unknown_obs) {
      century::hdmap::Polygon polygon;
      for (const auto& point : obstacle.PerceptionPolygon().points()) {
        century::common::PointENU* hdmap_point = polygon.add_point();
        hdmap_point->set_x(point.x());
        hdmap_point->set_y(point.y());
      }
      if (!reference_line_.GetSLBoundary(polygon, perception_sl)) {
        AERROR << "Failed to get sl boundary for obstacle: " << obstacle.Id();
        return false;
      }
    } else {
      if (!reference_line_.GetSLBoundary(obstacle.PerceptionBoundingBox(),
                                         perception_sl)) {
        AERROR << "Failed to get sl boundary for obstacle: " << obstacle.Id();
        return false;
      }
    }
  }
  return true;
}

void ReferenceLineInfo::AddObsLateralAndLonSpeed(
    const Obstacle& obstacle, const planning::SLBoundary& adc_sl) {
  auto* mutable_obs = path_decision_.Find(obstacle.Id());
  if (nullptr == mutable_obs) {
    AERROR << "failed to add obstacle " << mutable_obs->Id();
    return;
  }
  const auto& obs_sl = mutable_obs->PerceptionSLBoundary();
  const auto& velocity = mutable_obs->Perception().velocity();
  double obs_center_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
  ADEBUG << "mutable_obs->Id() = " << mutable_obs->Id();
  ADEBUG << "obs_center_s = " << obs_center_s;
  const auto& ref_point = reference_line_.GetReferencePoint(obs_center_s);
  double ref_heading = ref_point.heading();
  double obstacle_lon_speed = common::math::Vec2d::CreateUnitVec2d(ref_heading)
                                  .InnerProd(Vec2d(velocity.x(), velocity.y()));
  ADEBUG << mutable_obs->Id() << "_obstacle_lon_speed = " << obstacle_lon_speed;
  double obstacle_lateral_speed =
      common::math::Vec2d::CreateUnitVec2d(ref_heading)
          .CrossProd(Vec2d(velocity.x(), velocity.y()));
  ADEBUG << mutable_obs->Id()
         << "_obstacle_lateral_speed = " << obstacle_lateral_speed;
  mutable_obs->SetPerceptionLateralSpeed(obstacle_lateral_speed);
  mutable_obs->SetPerceptionLongitudinalSpeed(obstacle_lon_speed);

  // get lon acc
  const auto& acc = mutable_obs->Perception().acceleration();
  double obstacle_lon_acc = common::math::Vec2d::CreateUnitVec2d(ref_heading)
                                .InnerProd(Vec2d(acc.x(), acc.y()));
  if (std::isnan(obstacle_lon_acc)) {
    obstacle_lon_acc = 0.0;
  }
  mutable_obs->SetPerceptionLongitudianlAcc(obstacle_lon_acc);
}

bool ReferenceLineInfo::CheckIsNeedUsePreSlboundary(
    const Obstacle& obstacle, const SLBoundary& perception_sl) {
  auto* mutable_obs = path_decision_.Find(obstacle.Id());
  if (nullptr == mutable_obs) {
    AERROR << "failed to add obstacle " << mutable_obs->Id();
    return false;
  }
  // roi=======> left:4 right:4 front:15 rear:5
  if (perception_sl.end_l() <
          adc_sl_boundary_.start_l() - kConsiderLateralDistance ||
      perception_sl.start_l() >
          adc_sl_boundary_.end_l() + kConsiderLateralDistance) {
    return false;
  }
  // AINFO << "obs_sl.start_s() = " << perception_sl.start_s();
  // AINFO << "obs_sl.end_s() = " << perception_sl.end_s();
  // AINFO << "adc_sl_boundary_.end_s() = " << adc_sl_boundary_.end_s();
  // AINFO << "adc_sl_boundary_.start_s() = " << adc_sl_boundary_.start_s();
  if (perception_sl.start_s() >
          adc_sl_boundary_.end_s() + kConsiderFrontDistance ||
      perception_sl.end_s() <
          adc_sl_boundary_.start_s() - kConsiderRearDistance) {
    return false;
  }
  // overlap no consider
  if (!(perception_sl.end_l() < adc_sl_boundary_.start_l() ||
        perception_sl.start_l() > adc_sl_boundary_.end_l())) {
    return false;
  }

  // dynamic obs
  if (obstacle.IsStatic()) {
    return false;
  }

  // obs need to face path
  bool is_left_cross_obs = perception_sl.end_l() < adc_sl_boundary_.start_l() &&
                           mutable_obs->LateralSpeed() > kMinLateralSpeed;
  // AINFO << "obstacle.LateralSpeed() = " << mutable_obs->LateralSpeed();
  bool is_right_cross_obs =
      perception_sl.start_l() > adc_sl_boundary_.end_l() &&
      mutable_obs->LateralSpeed() < -kMinLateralSpeed;
  if (is_right_cross_obs || is_left_cross_obs) {
    return true;
  }
  return false;
}

void ReferenceLineInfo::PreBuildObstacleSLboudnary(const Obstacle& obstacle,
                                                   SLBoundary* perception_sl) {
  auto* mutable_obs = path_decision_.Find(obstacle.Id());
  if (nullptr == mutable_obs) {
    AERROR << "failed to add obstacle " << mutable_obs->Id();
    return;
  }

  // range obs
  bool is_need_pre_buil_sl_boundary =
      CheckIsNeedUsePreSlboundary(obstacle, *perception_sl);
  // AINFO << "is_need_pre_buil_sl_boundary = " <<
  // is_need_pre_buil_sl_boundary;
  if (is_need_pre_buil_sl_boundary) {
    // AINFO << "obstacle = " << obstacle.Id();
    // AINFO << "ori_sl_boundary start_s = " << perception_sl->start_s()
    //       << "  end_s = " << perception_sl->end_s()
    //       << "  start_l = " << perception_sl->start_l()
    //       << "  end_l = " << perception_sl->end_l();
    SLBoundary perception_sl_after;
    if (!GetObstacleSLBoundary(obstacle, &perception_sl_after, true)) {
      return;
    }
    // AINFO << "pre_build_sl_boundary start_s = " <<
    // perception_sl_after.start_s()
    //       << "  end_s = " << perception_sl_after.end_s()
    //       << "  start_l = " << perception_sl_after.start_l()
    //       << "  end_l = " << perception_sl_after.end_l();
    // get min sor get max s
    perception_sl->set_start_s(
        std::min(perception_sl_after.start_s(), perception_sl->start_s()));
    perception_sl->set_end_s(
        std::max(perception_sl_after.end_s(), perception_sl->end_s()));
    // left or right
    if (perception_sl->start_l() > adc_sl_boundary_.end_l()) {
      perception_sl->set_start_l(
          std::min(perception_sl_after.start_l(), perception_sl->start_l()));
      perception_sl->set_end_l(
          std::max(perception_sl_after.end_l(), perception_sl->end_l()));
    } else {
      perception_sl->set_start_l(
          std::max(perception_sl_after.start_l(), perception_sl->start_l()));
      perception_sl->set_end_l(
          std::min(perception_sl_after.end_l(), perception_sl->end_l()));
    }
    // AINFO << "after_sl_boundary start_s = " << perception_sl->start_s()
    //       << "  end_s = " << perception_sl->end_s()
    //       << "  start_l = " << perception_sl->start_l()
    //       << "  end_l = " << perception_sl->end_l();
  }
}

// AddObstacle is thread safe
Obstacle* ReferenceLineInfo::AddObstacle(const Obstacle* obstacle) {
  if (!obstacle) {
    AERROR << "The provided obstacle is empty";
    return nullptr;
  }
  auto* mutable_obstacle = path_decision_.AddObstacle(*obstacle);
  if (!mutable_obstacle) {
    AERROR << "failed to add obstacle " << obstacle->Id();
    return nullptr;
  }

  SLBoundary perception_sl;
  if (!GetObstacleSLBoundary(*obstacle, &perception_sl)) {
    return mutable_obstacle;
  }

  if (FLAGS_enable_pre_build_sl_boundary) {
    PreBuildObstacleSLboudnary(*obstacle, &perception_sl);
  }

  bool is_projection_front_adc = false;
  if ((perception_sl.end_l() > adc_sl_boundary_.end_l() &&
       perception_sl.start_l() < adc_sl_boundary_.start_l() &&
       perception_sl.end_s() > adc_sl_boundary_.end_s() &&
       perception_sl.start_s() < adc_sl_boundary_.start_s())) {
    is_projection_front_adc = true;
  }

  if (perception_sl.end_s() - perception_sl.start_s() >
          kMinCoeff * obstacle->Perception().length() &&
      (obstacle->Perception().length() > kMinObstacleLength ||
      obstacle->Perception().width()> kMinObstacleLength||
       obstacle->IsVirtual()) &&
      FLAGS_enable_intelligent_projection) {
    century::hdmap::Polygon polygon;
    for (const auto& point : obstacle->PerceptionPolygon().points()) {
      century::common::PointENU* hdmap_point = polygon.add_point();
      hdmap_point->set_x(point.x());
      hdmap_point->set_y(point.y());
    }
    const double adc_s =
        (adc_sl_boundary_.end_s() + adc_sl_boundary_.start_s()) * 0.5;
    if (!reference_line_.GetSLBoundaryUseOneProjection(
            adc_planning_point_, polygon, &perception_sl, adc_s,
            obstacle->Perception().length(), is_projection_front_adc)) {
      AERROR << "Failed to get sl boundary for obstacle: " << obstacle->Id();
      return mutable_obstacle;
    }
  }

  mutable_obstacle->SetPerceptionSlBoundary(perception_sl);
  AddObsLateralAndLonSpeed(*obstacle, adc_sl_boundary_);
  mutable_obstacle->CheckLaneBlocking(reference_line_);
  if (mutable_obstacle->IsLaneBlocking()) {
    ADEBUG << "obstacle [" << obstacle->Id() << "] is lane blocking.";
  } else {
    ADEBUG << "obstacle [" << obstacle->Id() << "] is NOT lane blocking.";
  }

  if (IsIrrelevantObstacle(*mutable_obstacle)) {
    ObjectDecisionType ignore;
    ignore.mutable_ignore();
    path_decision_.AddLateralDecision("reference_line_filter", obstacle->Id(),
                                      ignore);
    path_decision_.AddLongitudinalDecision("reference_line_filter",
                                           obstacle->Id(), ignore);
    ADEBUG << "NO build reference line st boundary. id:" << obstacle->Id();
  } else {
    ADEBUG << "build reference line st boundary. id:" << obstacle->Id();
    mutable_obstacle->BuildReferenceLineStBoundary(reference_line_,
                                                   adc_sl_boundary_.start_s());

    ADEBUG << "reference line st boundary: t["
           << mutable_obstacle->reference_line_st_boundary().min_t() << ", "
           << mutable_obstacle->reference_line_st_boundary().max_t() << "] s["
           << mutable_obstacle->reference_line_st_boundary().min_s() << ", "
           << mutable_obstacle->reference_line_st_boundary().max_s() << "]";
  }
  return mutable_obstacle;
}

bool ReferenceLineInfo::AddObstacles(
    const std::vector<const Obstacle*>& obstacles) {
  if (FLAGS_use_multi_thread_to_add_obstacles) {
    std::vector<std::future<Obstacle*>> results;
    for (const auto* obstacle : obstacles) {
      results.push_back(
          cyber::Async(&ReferenceLineInfo::AddObstacle, this, obstacle));
    }
    for (auto& result : results) {
      if (!result.get()) {
        AERROR << "Fail to add obstacles.";
        return false;
      }
    }
  } else {
    for (const auto* obstacle : obstacles) {
      if (!AddObstacle(obstacle)) {
        AERROR << "Failed to add obstacle " << obstacle->Id();
        return false;
      }
    }
  }

  return true;
}

bool ReferenceLineInfo::IsIrrelevantObstacle(const Obstacle& obstacle) {
  ADEBUG << "check is irrelevant obstacle: " << obstacle.Id();
  const auto& obstacle_boundary = obstacle.PerceptionSLBoundary();
  // obs center_l is in reference lane with hysteresis.
  bool is_obs_on_ref_lane = IsOnLane(obstacle);
  if (FLAGS_enable_use_boundary_check_obs_on_lane) {
    is_obs_on_ref_lane = reference_line_.IsOnLane(obstacle_boundary);
  }
  bool is_adc_on_ref_lane = IsAdcOnLane(adc_sl_boundary_);
  if (FLAGS_enable_use_boundary_check_adc_on_lane) {
    is_adc_on_ref_lane = is_on_reference_line_;
  }
  // AINFO << "obs is on ref lane = " << is_obs_on_ref_lane;

  if (obstacle.IsCautionLevelObstacle()) {
    return IsCautionObstacleIrrelevant(obstacle, is_obs_on_ref_lane,
                                       is_adc_on_ref_lane);
  }
  // if adc is on the road, and obstacle behind adc, ignore
  double half_veh_length =
      VehicleConfigHelper::GetConfig().vehicle_param().length() * 0.5;
  if (obstacle_boundary.start_s() - half_veh_length - kLonCollisionbuffer >
      reference_line_.Length()) {
    return true;
  }

  if (IsCautionBicycle(obstacle, is_adc_on_ref_lane, is_obs_on_ref_lane)) {
    // AINFO << "need consider obs_" << obstacle.Id();
    return false;
  }
  // AINFO << "obstacle_boundary.end_s() = " << obstacle_boundary.end_s();
  if (is_adc_on_ref_lane && !IsChangeLanePath() &&
      obstacle_boundary.end_s() < adc_sl_boundary_.start_s() + kLonBuffer &&
      (is_obs_on_ref_lane ||
       obstacle_boundary.end_s() < 0.0)) {  // if obstacle is far backward
    return true;
  }
  return false;
}

bool ReferenceLineInfo::IsCautionBicycle(const Obstacle& obstacle,
                                         bool is_adc_on_ref_lane,
                                         bool is_obs_on_ref_lane) {
  bool is_bicycle = PerceptionObstacle::BICYCLE == obstacle.Perception().type();
  const auto& obstacle_boundary = obstacle.PerceptionSLBoundary();
  double diff_l =
      std::max(adc_sl_boundary_.start_l() - obstacle_boundary.end_l(),
               obstacle_boundary.start_l() - adc_sl_boundary_.end_l());
  // AINFO << "diff_l = " << diff_l;
  // if adc in high speed ,we slow breaking to 3.0.
  // no use speed diff;
  bool adc_is_high_speed =
      vehicle_state_.linear_velocity() > kSlowBreakingTargetSpeed;
  // AINFO << "adc_sl_boundary_.end_s() = " << adc_sl_boundary_.end_s();
  // right turn consider right back obs.
  if (at_right_turn_ && adc_sl_boundary_.start_l() - obstacle_boundary.end_l() <
                            kLateralBufferForBicycle) {
    return false;
  }
  // left turn consider left back obs.
  if (at_left_turn_ && obstacle_boundary.start_l() - adc_sl_boundary_.end_l() <
                           kLateralBufferForBicycle) {
    // AINFO << "left turn and too close adc";
    return false;
  }
  double diff_speed =
      obstacle.LongitudinalSpeed() - vehicle_state_.linear_velocity();
  // use pseed diff to cal consider distance.
  double min_back_distance =
      std::max(kMinBackDistance, diff_speed * kConsiderTimeForSpeedDiff);
  // AINFO << "obstacle.LongitudinalSpeed()  = " <<
  // obstacle.LongitudinalSpeed(); AINFO << "vehicle_state_.linear_velocity() =
  // "
  //       << vehicle_state_.linear_velocity();
  // AINFO << "min_back_distance = " << min_back_distance;
  bool is_in_back_consider_distance =
      (obstacle_boundary.end_s() >
       adc_sl_boundary_.start_s() - min_back_distance);
  // AINFO << "is_adc_on_ref_lane = " << is_adc_on_ref_lane;
  // AINFO << "is_in_back_consider_distance = " << is_in_back_consider_distance;
  // AINFO << "is_obs_on_ref_lane = " << is_obs_on_ref_lane;
  // AINFO << "is_bicycle = " << is_bicycle;
  // AINFO << "adc_is_high_speed = " << adc_is_high_speed;
  // AINFO << "diff_l > 0.2 = " << (diff_l > kLateralBufferForBicycle);
  // AINFO << "is_at_turn = " << is_at_turn_;
  if (is_adc_on_ref_lane && !IsChangeLanePath() &&
      is_in_back_consider_distance && is_obs_on_ref_lane && is_bicycle &&
      adc_is_high_speed && diff_l > kLateralBufferForBicycle && is_at_turn_) {
    AINFO << "need consider obs_" << obstacle.Id();
    return true;
  }
  return false;
}

bool ReferenceLineInfo::IsCautionObstacleIrrelevant(const Obstacle& obstacle,
                                                    bool is_obs_on_ref_lane,
                                                    bool is_adc_on_ref_lane) {
  const auto& obstacle_boundary = obstacle.PerceptionSLBoundary();
  if (((is_adc_on_ref_lane && !IsChangeLanePath() &&
        obstacle_boundary.end_s() < adc_sl_boundary_.start_s() + kLonBuffer &&
        (is_obs_on_ref_lane || obstacle_boundary.end_s() < 0.0)) &&
       (PerceptionObstacle::VEHICLE ==
        obstacle.Perception().type()))) {  // if obstacle is far backward
    return true;
  } else if ((obstacle_boundary.end_s() < adc_sl_boundary_.start_s() &&
              !(obstacle_boundary.end_l() < adc_sl_boundary_.start_l() ||
                obstacle_boundary.start_l() > adc_sl_boundary_.end_l()) &&
              (PerceptionObstacle::PEDESTRIAN == obstacle.Perception().type() ||
               PerceptionObstacle::BICYCLE ==
                   obstacle.Perception().type()))) {  // adc rear; overlap in
    // lateral;pedstrian or bicycle;
    return true;
  } else {
    return false;
  }
}

bool ReferenceLineInfo::IsOnLane(const Obstacle& obs) const {
  const auto& sl_boundary = obs.PerceptionSLBoundary();
  if (sl_boundary.end_s() < 0.0 ||
      sl_boundary.start_s() > reference_line_.Length()) {
    return false;
  }
  double middle_l = (sl_boundary.start_l() + sl_boundary.end_l()) * 0.5;
  double middle_s = (sl_boundary.start_s() + sl_boundary.end_s()) * 0.5;
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  reference_line_.GetLaneWidth(middle_s, &lane_left_width, &lane_right_width);
  double hy_middle_l = middle_l;
  // near lane left
  if (middle_l > (lane_left_width - lane_right_width) * 0.5) {
    obstacles_interval_left.SetAnchorLimits(lane_left_width, -kLowerBound,
                                            kUpperBound);
    hy_middle_l = obstacles_interval_left.HyValue(obs, middle_l);
    obstacles_interval_right.SetValueLevel(obs.Id(),
                                           HysteresisInterval::NO_INIT_LEVEL);
  } else {
    // near lane right
    obstacles_interval_right.SetAnchorLimits(-lane_right_width, -kUpperBound,
                                             kLowerBound);
    hy_middle_l = obstacles_interval_right.HyValue(obs, middle_l);
    obstacles_interval_left.SetValueLevel(obs.Id(),
                                          HysteresisInterval::NO_INIT_LEVEL);
  }
  return hy_middle_l <= lane_left_width && hy_middle_l >= -lane_right_width;
}

bool ReferenceLineInfo::IsAdcOnLane(const SLBoundary& sl_boundary) const {
  if (sl_boundary.end_s() < 0.0 ||
      sl_boundary.start_s() > reference_line_.Length()) {
    return false;
  }
  double middle_l = (sl_boundary.start_l() + sl_boundary.end_l()) * 0.5;
  double middle_s = (sl_boundary.start_s() + sl_boundary.end_s()) * 0.5;
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  reference_line_.GetLaneWidth(middle_s, &lane_left_width, &lane_right_width);
  return middle_l <= lane_left_width && middle_l >= -lane_right_width;
}

const DiscretizedTrajectory& ReferenceLineInfo::trajectory() const {
  return discretized_trajectory_;
}

void ReferenceLineInfo::SetLatticeStopPoint(const StopPoint& stop_point) {
  planning_target_.mutable_stop_point()->CopyFrom(stop_point);
}

void ReferenceLineInfo::SetLatticeCruiseSpeed(double speed) {
  planning_target_.set_cruise_speed(speed);
}

bool ReferenceLineInfo::IsStartFrom(
    const ReferenceLineInfo& previous_reference_line_info) const {
  if (reference_line_.reference_points().empty()) {
    return false;
  }
  auto start_point = reference_line_.reference_points().front();
  const auto& prev_reference_line =
      previous_reference_line_info.reference_line();
  common::SLPoint sl_point;
  prev_reference_line.XYToSL(start_point, &sl_point);
  return previous_reference_line_info.reference_line_.IsOnLane(sl_point);
}

const PathData& ReferenceLineInfo::path_data() const { return path_data_; }

const PathData& ReferenceLineInfo::fallback_path_data() const {
  return fallback_path_data_;
}

const SpeedData& ReferenceLineInfo::speed_data() const { return speed_data_; }

PathData* ReferenceLineInfo::mutable_path_data() { return &path_data_; }

PathData* ReferenceLineInfo::mutable_fallback_path_data() {
  return &fallback_path_data_;
}

SpeedData* ReferenceLineInfo::mutable_speed_data() { return &speed_data_; }

const RSSInfo& ReferenceLineInfo::rss_info() const { return rss_info_; }

RSSInfo* ReferenceLineInfo::mutable_rss_info() { return &rss_info_; }

bool ReferenceLineInfo::CombinePathAndSpeedProfile(
    const double relative_time, const double start_s,
    DiscretizedTrajectory* ptr_discretized_trajectory) {
  ACHECK(ptr_discretized_trajectory != nullptr);
  // use varied resolution to reduce data load but also provide enough data
  // point for control module
  const double kDenseTimeResoltuion = FLAGS_trajectory_time_min_interval;
  const double kSparseTimeResolution = FLAGS_trajectory_time_max_interval;
  const double kDenseTimeSec = FLAGS_trajectory_time_high_density_period;

  if (path_data_.discretized_path().empty()) {
    AERROR << "path data is empty";
    return false;
  }

  if (speed_data_.empty()) {
    AERROR << "speed profile is empty";
    return false;
  }

  for (double cur_rel_time = 0.0; cur_rel_time < speed_data_.TotalTime();
       cur_rel_time += (cur_rel_time < kDenseTimeSec ? kDenseTimeResoltuion
                                                     : kSparseTimeResolution)) {
    common::SpeedPoint speed_point;
    if (!speed_data_.EvaluateByTime(cur_rel_time, &speed_point)) {
      AERROR << "Fail to get speed point with relative time " << cur_rel_time;
      return false;
    }
    if (speed_point.s() < 1e-6) {
      speed_point.set_s(0.0);
    }
    if (speed_point.s() > path_data_.discretized_path().Length()) {
      break;
    }
    common::PathPoint path_point =
        path_data_.GetPathPointWithPathS(speed_point.s());
    path_point.set_s(path_point.s() + start_s);

    common::TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(speed_point.v());
    trajectory_point.set_a(speed_point.a());
    trajectory_point.set_relative_time(speed_point.t() + relative_time);
    ptr_discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
  }
  return true;
}

// TODO(all): It is a brutal way to insert the planning init point, one
// elegant way would be bypassing trajectory stitching logics somehow, or use
// planing init point from trajectory stitching to compute the trajectory at
// the very start
bool ReferenceLineInfo::AdjustTrajectoryWhichStartsFromCurrentPos(
    const common::TrajectoryPoint& planning_start_point,
    const std::vector<common::TrajectoryPoint>& trajectory,
    DiscretizedTrajectory* adjusted_trajectory) {
  ACHECK(adjusted_trajectory != nullptr);
  // find insert index by check heading
  static constexpr double kMaxAngleDiff = M_PI_2;
  const double start_point_heading = planning_start_point.path_point().theta();
  const double start_point_x = planning_start_point.path_point().x();
  const double start_point_y = planning_start_point.path_point().y();
  const double start_point_relative_time = planning_start_point.relative_time();

  int insert_idx = -1;
  for (size_t i = 0; i < trajectory.size(); ++i) {
    // skip trajectory_points early than planning_start_point
    if (trajectory[i].relative_time() <= start_point_relative_time) {
      continue;
    }

    const double cur_point_x = trajectory[i].path_point().x();
    const double cur_point_y = trajectory[i].path_point().y();
    const double tracking_heading =
        std::atan2(cur_point_y - start_point_y, cur_point_x - start_point_x);
    if (std::fabs(common::math::AngleDiff(start_point_heading,
                                          tracking_heading)) < kMaxAngleDiff) {
      insert_idx = i;
      break;
    }
  }
  if (insert_idx == -1) {
    AERROR << "All points are behind of planning init point";
    return false;
  }

  DiscretizedTrajectory cut_trajectory(trajectory);
  cut_trajectory.erase(cut_trajectory.begin(),
                       cut_trajectory.begin() + insert_idx);
  cut_trajectory.insert(cut_trajectory.begin(), planning_start_point);

  // In class TrajectoryStitcher, the stitched point which is also the
  // planning init point is supposed have one planning_cycle_time ahead
  // respect to current timestamp as its relative time. So the relative
  // timelines of planning init point and the trajectory which start from
  // current position(relative time = 0) are the same. Therefore any conflicts
  // on the relative time including the one below should return false and
  // inspected its cause.
  if (cut_trajectory.size() > 1 && cut_trajectory.front().relative_time() >=
                                       cut_trajectory[1].relative_time()) {
    AERROR << "planning init point relative_time["
           << cut_trajectory.front().relative_time()
           << "] larger than its next point's relative_time["
           << cut_trajectory[1].relative_time() << "]";
    return false;
  }

  // In class TrajectoryStitcher, the planing_init_point is set to have s as
  // 0, so adjustment is needed to be done on the other points
  double accumulated_s = 0.0;
  for (size_t i = 1; i < cut_trajectory.size(); ++i) {
    const auto& pre_path_point = cut_trajectory[i - 1].path_point();
    auto* cur_path_point = cut_trajectory[i].mutable_path_point();
    accumulated_s += std::sqrt((cur_path_point->x() - pre_path_point.x()) *
                                   (cur_path_point->x() - pre_path_point.x()) +
                               (cur_path_point->y() - pre_path_point.y()) *
                                   (cur_path_point->y() - pre_path_point.y()));
    cur_path_point->set_s(accumulated_s);
  }

  // reevaluate relative_time to make delta t the same
  adjusted_trajectory->clear();
  // use varied resolution to reduce data load but also provide enough data
  // point for control module
  const double kDenseTimeResoltuion = FLAGS_trajectory_time_min_interval;
  const double kSparseTimeResolution = FLAGS_trajectory_time_max_interval;
  const double kDenseTimeSec = FLAGS_trajectory_time_high_density_period;
  for (double cur_rel_time = cut_trajectory.front().relative_time();
       cur_rel_time <= cut_trajectory.back().relative_time();
       cur_rel_time += (cur_rel_time < kDenseTimeSec ? kDenseTimeResoltuion
                                                     : kSparseTimeResolution)) {
    adjusted_trajectory->AppendTrajectoryPoint(
        cut_trajectory.Evaluate(cur_rel_time));
  }
  return true;
}

void ReferenceLineInfo::SetDrivable(bool drivable) { is_drivable_ = drivable; }

bool ReferenceLineInfo::IsDrivable() const { return is_drivable_; }

bool ReferenceLineInfo::TrafficLightRequest() const {
  return traffic_light_request_;
}

void ReferenceLineInfo::SetTrafficLightRequest(
    const bool traffic_light_request) {
  traffic_light_request_ = traffic_light_request;
}

void ReferenceLineInfo::SetNearTrafficLightStopLine(
    const bool near_traffic_light_stop_line) {
  is_near_traffic_light_stop_line_ = near_traffic_light_stop_line;
}

bool ReferenceLineInfo::IsNearTrafficLightStopLine() const {
  return is_near_traffic_light_stop_line_;
}

bool ReferenceLineInfo::IsChangeLanePath() const {
  return !Lanes().IsOnSegment();
}

bool ReferenceLineInfo::IsNeighborLanePath() const {
  return Lanes().IsNeighborSegment();
}

bool ReferenceLineInfo::IsMixedTraffic() const {
  if (hdmap::Lane::CITY_DRIVING != GetLaneType()) {
    return false;
  }

  // Do not enter mixed traffic scenes at intersections
  for (const auto& overlap : reference_line_.map_path().junction_overlaps()) {
    hdmap::JunctionInfoConstPtr junction_info_ptr =
        hdmap::HDMapUtil::BaseMapPtr()->GetJunctionById(
            hdmap::MakeMapId(overlap.object_id));
    if (hdmap::Junction::COMMON_JUNCTION ==
            junction_info_ptr->junction().type() ||
        hdmap::Junction::IN_ROAD == junction_info_ptr->junction().type()) {
      double adc_s =
          (adc_sl_boundary_.start_s() + adc_sl_boundary_.end_s()) * 0.5;
      if (overlap.start_s - kJunctionDistanceBuffer <= adc_s &&
          adc_s <= overlap.end_s + kJunctionDistanceBuffer) {
        return false;
      }
    }
  }

  return true;
}

bool ReferenceLineInfo::HasNonMotorizedVehicle() {
  int obstacle_counter = 0;
  const auto& adc_front_s = adc_sl_boundary_.end_s();
  hdmap::LaneInfoConstPtr locate_lane = LocateLaneInfo(adc_front_s);
  if (locate_lane != nullptr) {
    if (!locate_lane->lane().left_neighbor_forward_lane_id().empty() ||
        !locate_lane->lane().right_neighbor_forward_lane_id().empty()) {
      has_neighbor_forward_lane_ = true;
    }
  }
  for (const auto& obstacle : path_decision_.obstacles().Items()) {
    const auto& obstacle_start_s = obstacle->PerceptionSLBoundary().start_s();
    const auto& obstacle_start_l = obstacle->PerceptionSLBoundary().start_l();
    const auto& obstacle_end_l = obstacle->PerceptionSLBoundary().end_l();
    const auto& adc_front_s = adc_sl_boundary_.end_s();
    double distance = obstacle_start_s - adc_front_s;
    const auto& lane_width = GetLaneWidthBaseOnAdcCenter();
    double left_width_self = lane_width.first;
    double right_width_self = lane_width.second;
    if (has_neighbor_forward_lane_) {
      if (distance <= kForwardDistance && distance >= kBackwardDistance &&
          (perception::PerceptionObstacle::PEDESTRIAN ==
               obstacle->Perception().type() ||
           perception::PerceptionObstacle::BICYCLE ==
               obstacle->Perception().type()) &&
          !(obstacle_end_l <=
                -right_width_self - kMutableLateralDistanceBuffer ||
            obstacle_start_l >=
                left_width_self + kMutableLateralDistanceBuffer)) {
        ++obstacle_counter;
      }
    } else {
      if (distance <= kForwardDistance && distance >= kBackwardDistance &&
          (perception::PerceptionObstacle::PEDESTRIAN ==
               obstacle->Perception().type() ||
           perception::PerceptionObstacle::BICYCLE ==
               obstacle->Perception().type()) &&
          !(obstacle_end_l <= -right_width_self - kLateralDistanceBuffer ||
            obstacle_start_l >= left_width_self + kLateralDistanceBuffer)) {
        ++obstacle_counter;
      }
    }
  }
  if (obstacle_counter >= kObstackeMinSize) {
    return true;
  }
  return false;
}

bool ReferenceLineInfo::HasRetrogradeObstacleOnBicycleLane() {
  double adc_s = (adc_sl_boundary_.start_s() + adc_sl_boundary_.end_s()) * 0.5;
  hdmap::LaneInfoConstPtr locate_lane = LocateLaneInfo(adc_s);
  if (nullptr == locate_lane) {
    return false;
  }

  if (!locate_lane->lane().left_neighbor_forward_lane_id().empty() ||
      !locate_lane->lane().left_neighbor_reverse_lane_id().empty() ||
      !locate_lane->lane().right_neighbor_forward_lane_id().empty() ||
      !locate_lane->lane().right_neighbor_reverse_lane_id().empty()) {
    return false;
  }

  for (const auto& obstacle : path_decision_.obstacles().Items()) {
    if (obstacle->IsVirtual() || obstacle->IsStatic()) {
      continue;
    }

    if (!reference_line_.IsOnLane(obstacle->PerceptionSLBoundary())) {
      continue;
    }

    if (obstacle->PerceptionSLBoundary().end_s() < adc_sl_boundary_.start_s()) {
      continue;
    }

    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
    double obs_theta = obstacle->Perception().theta();
    if (obstacle->HasTrajectory()) {
      obs_theta =
          obstacle->Trajectory().trajectory_point(0).path_point().theta();
    }
    double obs_s = (obstacle_sl.start_s() + obstacle_sl.end_s()) * 0.5;
    ReferencePoint reference_point =
        reference_line_.GetNearestReferencePoint(obs_s);
    double theta_diff = century::common::math::NormalizeAngle(
        obs_theta - reference_point.heading());
    double buffer_radian = FLAGS_buffer_degrees / kDegrees * M_PI_2;
    if (theta_diff < -M_PI_2 - buffer_radian ||
        theta_diff > M_PI_2 + buffer_radian) {
      return true;
    }
  }

  return false;
}

std::string ReferenceLineInfo::PathSpeedDebugString() const {
  return absl::StrCat("path_data:", path_data_.DebugString(),
                      "speed_data:", speed_data_.DebugString());
}

bool ReferenceLineInfo::SetTurnSignalBasedTurnType(
    common::VehicleSignal* vehicle_signal) const {
  double route_s = 0.0;
  const double adc_s = adc_sl_boundary_.end_s();
  for (const auto& seg : Lanes()) {
    if (route_s > adc_s + FLAGS_turn_signal_distance) {
      break;
    }
    route_s += seg.end_s - seg.start_s;
    if (route_s < adc_s) {
      continue;
    }
    const auto& turn = seg.lane->lane().turn();
    if (turn == hdmap::Lane::LEFT_TURN) {
      vehicle_signal->set_turn_signal(VehicleSignal::TURN_LEFT);
      turn_left_count_ = 0UL;
      turn_right_count_ = 0UL;
      turn_left_continue_count_ = kTurnSignalDelayFrameNumber;
      turn_right_continue_count_ = kTurnSignalDelayFrameNumber;
      return true;
    } else if (turn == hdmap::Lane::RIGHT_TURN) {
      vehicle_signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
      turn_left_count_ = 0UL;
      turn_right_count_ = 0UL;
      turn_left_continue_count_ = kTurnSignalDelayFrameNumber;
      turn_right_continue_count_ = kTurnSignalDelayFrameNumber;
      return true;
    } else if (turn == hdmap::Lane::U_TURN) {
      // check left or right by geometry.
      auto start_xy =
          PointFactory::ToVec2d(seg.lane->GetSmoothPoint(seg.start_s));
      auto middle_xy = PointFactory::ToVec2d(
          seg.lane->GetSmoothPoint((seg.start_s + seg.end_s) * 0.5));
      auto end_xy = PointFactory::ToVec2d(seg.lane->GetSmoothPoint(seg.end_s));
      auto start_to_middle = middle_xy - start_xy;
      auto start_to_end = end_xy - start_xy;
      if (start_to_middle.CrossProd(start_to_end) < 0) {
        vehicle_signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
      } else {
        vehicle_signal->set_turn_signal(VehicleSignal::TURN_LEFT);
      }
      turn_left_count_ = 0UL;
      turn_right_count_ = 0UL;
      turn_left_continue_count_ = kTurnSignalDelayFrameNumber;
      turn_right_continue_count_ = kTurnSignalDelayFrameNumber;
      return true;
    }
  }
  return false;
}

void ReferenceLineInfo::SetTurnSignalBasedPath(
    const FrenetFramePath& frenet_path,
    common::VehicleSignal* vehicle_signal) const {
  double curr_lane_left_width = 0.0;
  double curr_lane_right_width = 0.0;
  double advance_turn_distance =
      std::fmax(vehicle_state_.linear_velocity() * kMinTurnSignalTime,
                kMinTurnSignalDistance);
  double half_veh_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  double adc_l = (adc_sl_boundary_.start_l() + adc_sl_boundary_.end_l()) * 0.5;
  bool is_turn = false;
  for (auto item = frenet_path.begin();
       (item->s() - adc_sl_boundary_.end_s() < advance_turn_distance) &&
       (item != frenet_path.end());
       ++item) {
    reference_line_.GetLaneWidth(item->s(), &curr_lane_left_width,
                                 &curr_lane_right_width);
    if (item->l() - adc_l > kTurnSignalDistanceBuffer) {
      if (item->l() + half_veh_width > curr_lane_left_width &&
          curr_lane_left_width > adc_sl_boundary_.start_l()) {
        turn_right_count_ = 0UL;
        turn_left_continue_count_ = kTurnSignalDelayFrameNumber;
        turn_right_continue_count_ = kTurnSignalDelayFrameNumber;
        ++turn_left_count_;
        is_turn = true;
        // AINFO << "turn_left_count_: " << turn_left_count_;
        break;
      }
    } else if (adc_l - item->l() > kTurnSignalDistanceBuffer) {
      if ((item->l() - half_veh_width < curr_lane_left_width &&
           curr_lane_left_width < adc_sl_boundary_.end_l()) ||
          (item->l() - half_veh_width < -curr_lane_right_width &&
           -curr_lane_right_width < adc_sl_boundary_.end_l())) {
        turn_left_count_ = 0UL;
        turn_left_continue_count_ = kTurnSignalDelayFrameNumber;
        turn_right_continue_count_ = kTurnSignalDelayFrameNumber;
        ++turn_right_count_;
        is_turn = true;
        // AINFO << "turn_right_count_: " << turn_right_count_;
        break;
      }
    }
  }
  if (!is_turn) {
    if (turn_left_continue_count_ <= 0) {
      turn_left_count_ = 0UL;
    }
    if (turn_right_continue_count_ <= 0) {
      turn_right_count_ = 0UL;
    }
  }

  size_t delay_buffer = kTurnSignalCountDelayBufferPublicRoad;
  if (hdmap::Lane::PLAY_STREET == GetLaneType()) {
    delay_buffer = kTurnSignalCountDelayBufferPlayStreet;
  }
  if (turn_left_count_ > delay_buffer && turn_left_continue_count_ > 0) {
    --turn_left_continue_count_;
    vehicle_signal->set_turn_signal(VehicleSignal::TURN_LEFT);
    ADEBUG << "set TURN_LEFT signal base on path.";
    return;
  } else {
    turn_left_continue_count_ = kTurnSignalDelayFrameNumber;
  }
  if (turn_right_count_ > delay_buffer && turn_right_continue_count_ > 0) {
    --turn_right_continue_count_;
    vehicle_signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
    ADEBUG << "set TURN_RIGHT signal base on path.";
    return;
  } else {
    turn_right_continue_count_ = kTurnSignalDelayFrameNumber;
  }
}

void ReferenceLineInfo::SetTurnSignalBasedOnLaneTurnType(
    common::VehicleSignal* vehicle_signal) const {
  CHECK_NOTNULL(vehicle_signal);
  if (vehicle_signal->has_turn_signal() &&
      vehicle_signal->turn_signal() != VehicleSignal::TURN_NONE) {
    return;
  }
  vehicle_signal->set_turn_signal(VehicleSignal::TURN_NONE);

  // Set turn signal based on GetNeedTurnRight().
  if (GetNeedTurnRight()) {
    vehicle_signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
    return;
  }

  // Set turn signal based on lane-change.
  if (IsChangeLanePath()) {
    if (Lanes().PreviousAction() == routing::ChangeLaneType::LEFT) {
      vehicle_signal->set_turn_signal(VehicleSignal::TURN_LEFT);
    } else if (Lanes().PreviousAction() == routing::ChangeLaneType::RIGHT) {
      vehicle_signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
    }
    turn_left_count_ = 0UL;
    turn_right_count_ = 0UL;
    turn_left_continue_count_ = kTurnSignalDelayFrameNumber;
    turn_right_continue_count_ = kTurnSignalDelayFrameNumber;
    return;
  }

  const auto& frenet_path = path_data_.frenet_frame_path();

  if (frenet_path.Length() < kInvalidLength) {
    AINFO << " frenet_path.Length()  " << frenet_path.Length()
          << "is too short, early return SetTurnSignalBasedOnLaneTurnType() ";
    turn_left_count_ = 0UL;
    turn_right_count_ = 0UL;
    turn_left_continue_count_ = kTurnSignalDelayFrameNumber;
    turn_right_continue_count_ = kTurnSignalDelayFrameNumber;
    return;
  }

  // Set turn signal based on lane's turn type.
  if (SetTurnSignalBasedTurnType(vehicle_signal)) {
    return;
  }
  // Set turn signal based on path.
  SetTurnSignalBasedPath(frenet_path, vehicle_signal);
}

ADCTrajectory::ADBehavior ReferenceLineInfo::GetTurnBehavior() const {
  ADCTrajectory::ADBehavior ad_behavior = ADCTrajectory::AD_NONE;
  // Set ad behavior based on lane's turn type.
  double route_s = 0.0;
  const double adc_s = adc_sl_boundary_.end_s();
  for (const auto& seg : Lanes()) {
    if (route_s > adc_s + FLAGS_turn_signal_distance) {
      break;
    }
    route_s += seg.end_s - seg.start_s;
    if (route_s < adc_s) {
      continue;
    }
    const auto& turn = seg.lane->lane().turn();
    if (turn == hdmap::Lane::LEFT_TURN) {
      ad_behavior = ADCTrajectory::AD_TURNING_LEFT;
    } else if (turn == hdmap::Lane::RIGHT_TURN) {
      ad_behavior = ADCTrajectory::AD_TURNING_RIGHT;
    } else if (turn == hdmap::Lane::U_TURN) {
      // check left or right by geometry.
      auto start_xy =
          PointFactory::ToVec2d(seg.lane->GetSmoothPoint(seg.start_s));
      auto middle_xy = PointFactory::ToVec2d(
          seg.lane->GetSmoothPoint((seg.start_s + seg.end_s) * 0.5));
      auto end_xy = PointFactory::ToVec2d(seg.lane->GetSmoothPoint(seg.end_s));
      auto start_to_middle = middle_xy - start_xy;
      auto start_to_end = end_xy - start_xy;
      if (start_to_middle.CrossProd(start_to_end) < 0) {
        ad_behavior = ADCTrajectory::AD_TURNING_RIGHT;
      } else {
        ad_behavior = ADCTrajectory::AD_TURNING_LEFT;
      }
    }
  }
  return ad_behavior;
}

void ReferenceLineInfo::SetTurnSignal(
    const VehicleSignal::TurnSignal& turn_signal) {
  vehicle_signal_.set_turn_signal(turn_signal);
}

void ReferenceLineInfo::SetEmergencyLight() {
  vehicle_signal_.set_emergency_light(true);
}

void ReferenceLineInfo::ExportVehicleSignal(
    common::VehicleSignal* vehicle_signal) const {
  CHECK_NOTNULL(vehicle_signal);
  *vehicle_signal = vehicle_signal_;
  SetTurnSignalBasedOnLaneTurnType(vehicle_signal);
}

bool ReferenceLineInfo::ReachedDestination() const {
  static constexpr double kDestinationDeltaS = 0.05;
  const double distance_destination = SDistanceToDestination();
  return distance_destination <= kDestinationDeltaS;
}

double ReferenceLineInfo::SDistanceToDestination() const {
  double res = std::numeric_limits<double>::max();
  const auto* dest_ptr = path_decision_.Find(FLAGS_destination_obstacle_id);
  if (!dest_ptr) {
    return res;
  }
  if (!dest_ptr->LongitudinalDecision().has_stop()) {
    return res;
  }
  if (!reference_line_.IsOnLane(dest_ptr->PerceptionBoundingBox().center())) {
    return res;
  }
  const double stop_s = dest_ptr->PerceptionSLBoundary().start_s() +
                        dest_ptr->LongitudinalDecision().stop().distance_s();
  return stop_s - adc_sl_boundary_.end_s();
}

void ReferenceLineInfo::ExportDecision(
    DecisionResult* decision_result, PlanningContext* planning_context) const {
  MakeDecision(decision_result, planning_context);
  ExportVehicleSignal(decision_result->mutable_vehicle_signal());
  auto* main_decision = decision_result->mutable_main_decision();
  if (main_decision->has_stop()) {
    main_decision->mutable_stop()->set_change_lane_type(
        Lanes().PreviousAction());
  } else if (main_decision->has_cruise()) {
    main_decision->mutable_cruise()->set_change_lane_type(
        Lanes().PreviousAction());
  }
}

void ReferenceLineInfo::MakeDecision(DecisionResult* decision_result,
                                     PlanningContext* planning_context) const {
  CHECK_NOTNULL(decision_result);
  decision_result->Clear();

  // cruise by default
  decision_result->mutable_main_decision()->mutable_cruise();

  // check stop decision
  int error_code = MakeMainStopDecision(decision_result);
  if (error_code < 0) {
    MakeEStopDecision(decision_result);
  }
  MakeMainMissionCompleteDecision(decision_result, planning_context);
  SetObjectDecisions(decision_result->mutable_object_decision());
}

void ReferenceLineInfo::MakeMainMissionCompleteDecision(
    DecisionResult* decision_result, PlanningContext* planning_context) const {
  if (!decision_result->main_decision().has_stop()) {
    return;
  }
  auto main_stop = decision_result->main_decision().stop();
  if (main_stop.reason_code() != STOP_REASON_DESTINATION &&
      main_stop.reason_code() != STOP_REASON_PULL_OVER) {
    return;
  }
  const auto& adc_pos = adc_planning_point_.path_point();
  if (common::util::DistanceXY(adc_pos, main_stop.stop_point()) >
      FLAGS_destination_check_distance) {
    return;
  }

  auto mission_complete =
      decision_result->mutable_main_decision()->mutable_mission_complete();
  if (ReachedDestination()) {
    planning_context->mutable_planning_status()
        ->mutable_destination()
        ->set_has_passed_destination(true);
  } else {
    mission_complete->mutable_stop_point()->CopyFrom(main_stop.stop_point());
    mission_complete->set_stop_heading(main_stop.stop_heading());
  }
}

int ReferenceLineInfo::MakeMainStopDecision(
    DecisionResult* decision_result) const {
  double min_stop_line_s = std::numeric_limits<double>::infinity();
  const Obstacle* stop_obstacle = nullptr;
  const ObjectStop* stop_decision = nullptr;

  for (const auto* obstacle : path_decision_.obstacles().Items()) {
    const auto& object_decision = obstacle->LongitudinalDecision();
    if (!object_decision.has_stop()) {
      continue;
    }

    century::common::PointENU stop_point = object_decision.stop().stop_point();
    common::SLPoint stop_line_sl;
    reference_line_.XYToSL(stop_point, &stop_line_sl);

    double stop_line_s = stop_line_sl.s();
    if (stop_line_s < 0 || stop_line_s > reference_line_.Length()) {
      AERROR << "Ignore object:" << obstacle->Id() << " fence route_s["
             << stop_line_s << "] not in range[0, " << reference_line_.Length()
             << "]";
      continue;
    }

    // check stop_line_s vs adc_s
    if (stop_line_s < min_stop_line_s) {
      min_stop_line_s = stop_line_s;
      stop_obstacle = obstacle;
      stop_decision = &(object_decision.stop());
    }
  }

  if (stop_obstacle != nullptr) {
    MainStop* main_stop =
        decision_result->mutable_main_decision()->mutable_stop();
    main_stop->set_reason_code(stop_decision->reason_code());
    main_stop->set_reason("stop by " + stop_obstacle->Id());
    main_stop->set_tag(stop_decision->tag());
    main_stop->mutable_stop_point()->set_x(stop_decision->stop_point().x());
    main_stop->mutable_stop_point()->set_y(stop_decision->stop_point().y());
    main_stop->set_stop_heading(stop_decision->stop_heading());

    ADEBUG << " main stop obstacle id:" << stop_obstacle->Id()
           << " stop_line_s:" << min_stop_line_s << " stop_point: ("
           << stop_decision->stop_point().x() << stop_decision->stop_point().y()
           << " ) stop_heading: " << stop_decision->stop_heading();

    return 1;
  }

  return 0;
}

void ReferenceLineInfo::SetObjectDecisions(
    ObjectDecisions* object_decisions) const {
  for (const auto obstacle : path_decision_.obstacles().Items()) {
    if (!obstacle->HasNonIgnoreDecision()) {
      continue;
    }
    auto* object_decision = object_decisions->add_decision();

    object_decision->set_id(obstacle->Id());
    object_decision->set_perception_id(obstacle->PerceptionId());
    if (obstacle->HasLateralDecision() && !obstacle->IsLateralIgnore()) {
      object_decision->add_object_decision()->CopyFrom(
          obstacle->LateralDecision());
    }
    if (obstacle->HasLongitudinalDecision() &&
        !obstacle->IsLongitudinalIgnore()) {
      object_decision->add_object_decision()->CopyFrom(
          obstacle->LongitudinalDecision());
    }
  }
}

void ReferenceLineInfo::ExportEngageAdvice(
    EngageAdvice* engage_advice, PlanningContext* planning_context) const {
  static EngageAdvice prev_advice;
  static constexpr double kMaxAngleDiff = M_PI / 6.0;

  bool engage = false;
  if (!IsDrivable()) {
    prev_advice.set_reason("Reference line not drivable");
  } else if (!is_on_reference_line_) {
    const auto& scenario_type =
        planning_context->planning_status().scenario().scenario_type();
    if (scenario_type == ScenarioConfig::PARK_AND_GO || IsChangeLanePath()) {
      // note: when is_on_reference_line_ is FALSE
      //   (1) always engage while in PARK_AND_GO scenario
      //   (2) engage when "ChangeLanePath" is picked as Drivable ref line
      //       where most likely ADC not OnLane yet
      engage = true;
    } else {
      prev_advice.set_reason("Not on reference line");
    }
  } else {
    // check heading
    auto ref_point =
        reference_line_.GetReferencePoint(adc_sl_boundary_.end_s());
    if (common::math::AngleDiff(vehicle_state_.heading(), ref_point.heading()) <
        kMaxAngleDiff) {
      engage = true;
    } else {
      prev_advice.set_reason("Vehicle heading is not aligned");
    }
  }

  if (engage) {
    if (vehicle_state_.driving_mode() !=
        Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE) {
      // READY_TO_ENGAGE when in non-AUTO mode
      prev_advice.set_advice(EngageAdvice::READY_TO_ENGAGE);
    } else {
      // KEEP_ENGAGED when in AUTO mode
      prev_advice.set_advice(EngageAdvice::KEEP_ENGAGED);
    }
    prev_advice.clear_reason();
  } else {
    if (prev_advice.advice() != EngageAdvice::DISALLOW_ENGAGE) {
      prev_advice.set_advice(EngageAdvice::PREPARE_DISENGAGE);
    }
  }
  engage_advice->CopyFrom(prev_advice);
}

bool ReferenceLineInfo::FindClosestPointInTurn(double target) {
  if (path_point_in_turn_vector_.empty()) {
    return false;
  }

  auto closest = std::min_element(
      path_point_in_turn_vector_.begin(), path_point_in_turn_vector_.end(),
      [target](auto& a, auto& b) {
        return std::abs(a.first - target) < std::abs(b.first - target);
      });

  return closest->second;
}

void ReferenceLineInfo::MakeEStopDecision(
    DecisionResult* decision_result) const {
  decision_result->Clear();

  MainEmergencyStop* main_estop =
      decision_result->mutable_main_decision()->mutable_estop();
  main_estop->set_reason_code(MainEmergencyStop::ESTOP_REASON_INTERNAL_ERR);
  main_estop->set_reason("estop reason to be added");
  main_estop->mutable_cruise_to_stop();

  // set object decisions
  ObjectDecisions* object_decisions =
      decision_result->mutable_object_decision();
  for (const auto obstacle : path_decision_.obstacles().Items()) {
    auto* object_decision = object_decisions->add_decision();
    object_decision->set_id(obstacle->Id());
    object_decision->set_perception_id(obstacle->PerceptionId());
    object_decision->add_object_decision()->mutable_avoid();
  }
}

hdmap::Lane::LaneTurn ReferenceLineInfo::GetPathTurnType(const double s) const {
  const double forward_buffer = 20.0;
  double route_s = 0.0;
  for (const auto& seg : Lanes()) {
    if (route_s > s + forward_buffer) {
      break;
    }
    route_s += seg.end_s - seg.start_s;
    if (route_s < s) {
      continue;
    }
    const auto& turn_type = seg.lane->lane().turn();
    if (turn_type == hdmap::Lane::LEFT_TURN ||
        turn_type == hdmap::Lane::RIGHT_TURN ||
        turn_type == hdmap::Lane::U_TURN) {
      return turn_type;
    }
  }

  return hdmap::Lane::NO_TURN;
}

bool ReferenceLineInfo::GetIntersectionRightofWayStatus(
    const hdmap::PathOverlap& pnc_junction_overlap) const {
  if (GetPathTurnType(pnc_junction_overlap.start_s) != hdmap::Lane::NO_TURN) {
    return false;
  }

  // TODO(all): iterate exits of intersection to check/compare speed-limit
  return true;
}

int ReferenceLineInfo::GetPnCJunction(
    const double s, hdmap::PathOverlap* pnc_junction_overlap) const {
  CHECK_NOTNULL(pnc_junction_overlap);
  const std::vector<hdmap::PathOverlap>& pnc_junction_overlaps =
      reference_line_.map_path().pnc_junction_overlaps();

  static constexpr double kError = 1.0;  // meter
  for (const auto& overlap : pnc_junction_overlaps) {
    if (s >= overlap.start_s - kError && s <= overlap.end_s + kError) {
      *pnc_junction_overlap = overlap;
      return 1;
    }
  }
  return 0;
}

void ReferenceLineInfo::SetBlockingObstacle(
    const std::string& blocking_obstacle_id) {
  blocking_obstacle_ = path_decision_.Find(blocking_obstacle_id);
}

std::vector<common::SLPoint> ReferenceLineInfo::GetAllStopDecisionSLPoint()
    const {
  std::vector<common::SLPoint> result;
  for (const auto* obstacle : path_decision_.obstacles().Items()) {
    const auto& object_decision = obstacle->LongitudinalDecision();
    if (!object_decision.has_stop()) {
      continue;
    }
    century::common::PointENU stop_point = object_decision.stop().stop_point();
    common::SLPoint stop_line_sl;
    reference_line_.XYToSL(stop_point, &stop_line_sl);
    if (stop_line_sl.s() <= 0 || stop_line_sl.s() >= reference_line_.Length()) {
      continue;
    }
    result.push_back(stop_line_sl);
  }

  // sort by s
  if (!result.empty()) {
    std::sort(result.begin(), result.end(),
              [](const common::SLPoint& a, const common::SLPoint& b) {
                return a.s() < b.s();
              });
  }

  return result;
}

bool ReferenceLineInfo::CheckToChangeLane() {
  bool has_target_back_obs = false;
  const Obstacle* target_back_obs = nullptr;
  double block_check_start_l = 0.0;
  double block_check_end_l = 0.0;
  double adc_center_l =
      (adc_sl_boundary_.start_l() + adc_sl_boundary_.end_l()) * 0.5;
  CheckLevel check_level =
      (std::fabs(adc_center_l) < kAdcKeepStraightLateralErr)
          ? CheckLevel::PRE_CHECK
          : CheckLevel::CHANGE_CHECK;
  for (const auto* obstacle : risk_obstacles_) {
    const auto& obs_sl = obstacle->PerceptionSLBoundary();
    if (!has_target_back_obs && obs_sl.end_s() < adc_sl_boundary_.start_s()) {
      target_back_obs = obstacle;
      has_target_back_obs = true;
      block_check_start_l = obs_sl.start_l();
      block_check_end_l = obs_sl.end_l();
      AINFO << "target_back_obs: " << obstacle->Id();
    }
    if (has_target_back_obs && target_back_obs && target_back_obs != obstacle) {
      if (obs_sl.end_s() < target_back_obs->PerceptionSLBoundary().end_s() &&
          obs_sl.start_l() <
              block_check_end_l + FLAGS_block_obstacle_lateral_buffer &&
          obs_sl.end_l() >
              block_check_start_l - FLAGS_block_obstacle_lateral_buffer) {
        ADEBUG << "obstacle: " << obstacle->Id()
               << " has no chance to overtake the back car";
        block_check_start_l = std::fmin(obs_sl.start_l(), block_check_start_l);
        block_check_end_l = std::fmax(obs_sl.end_l(), block_check_end_l);
        continue;
      }
      block_check_start_l = std::fmin(obs_sl.start_l(), block_check_start_l);
      block_check_end_l = std::fmax(obs_sl.end_l(), block_check_end_l);
    }

    // filter slow or unknown obs(mainly green plant) because of poor
    // perception for speed detection
    bool slow_unknown_obs =
        obstacle->speed() < FLAGS_static_unknown_obstacle_speed_threshold &&
        perception::PerceptionObstacle::UNKNOWN ==
            obstacle->Perception().type();
    if ((obstacle->IsStatic() || slow_unknown_obs) &&
        !IsObstacleBlockAdc(obstacle)) {
      continue;
    }

    if (!LaneChangeSafeCheck(check_level, obstacle)) {
      path_decision_.Find(obstacle->Id())->SetLaneChangeBlocking(true);
      AINFO << "Lane Change is blocked by obstacle: " << obstacle->Id();
      return false;
    } else {
      path_decision_.Find(obstacle->Id())->SetLaneChangeBlocking(false);
    }
  }
  return true;
}

double ReferenceLineInfo::GetObstacleCheckCuttinBuffer(
    const Obstacle* obstacle) {
  if (nullptr == obstacle) {
    return FLAGS_obstacle_cutin_check_lat_buffer;
  }
  return (perception::PerceptionObstacle::UNKNOWN ==
          obstacle->Perception().type())
             ? FLAGS_unknown_obstacle_cutin_check_lat_buffer
             : FLAGS_obstacle_cutin_check_lat_buffer;
}

void ReferenceLineInfo::CheckIsClearToChangeLane() {
  is_clear_to_change_lane_ = false;
  ComputeRemainLaneChangeTime();
  if (LaneChangePathWillPassOverlap()) {
    return;
  }
  for (const auto* obstacle : path_decision_.obstacles().Items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }

    const auto& obs_sl = obstacle->PerceptionSLBoundary();
    const auto& lane_width = path_decision_.Find(obstacle->Id())
                                 ->GetLaneWidthBaseOnCenter(reference_line_);
    double left_width = lane_width.first;
    double right_width = lane_width.second;
    double cutin_buffer = GetObstacleCheckCuttinBuffer(obstacle);
    if (obs_sl.end_l() < -right_width + cutin_buffer ||
        obs_sl.start_l() > left_width - cutin_buffer ||
        obs_sl.start_s() > FLAGS_front_safe_check_distance ||
        obs_sl.end_s() < FLAGS_back_safe_check_distance) {
      continue;
    }
    // AINFO<<"has  risk obs = "<<obstacle->Id();
    risk_obstacles_.emplace_back(obstacle);
  }
  if (risk_obstacles_.empty()) {
    is_clear_to_change_lane_ = true;
    return;
  }
  std::sort(risk_obstacles_.begin(), risk_obstacles_.end(),
            [](const Obstacle* a, const Obstacle* b) {
              return a->PerceptionSLBoundary().end_s() >
                     b->PerceptionSLBoundary().end_s();
            });

  if (CheckToChangeLane()) {
    is_clear_to_change_lane_ = true;
  }
  return;
}

const double ReferenceLineInfo::GetRemainLaneChangeTime() const {
  return IsChangeLanePath() ? remain_lane_change_time_
                            : FLAGS_lane_change_total_time;
}

void ReferenceLineInfo::ComputeRemainLaneChangeTime() {
  double adc_center_l =
      (adc_sl_boundary_.start_l() + adc_sl_boundary_.end_l()) * 0.5;
  bool left_change = (adc_center_l < 0.0);
  double remain_lateral_dis = std::fabs(adc_center_l);
  double left_width = lane_width_base_on_adc_center_.first;
  double right_width = lane_width_base_on_adc_center_.second;
  const auto& car_param = VehicleConfigHelper::GetConfig().vehicle_param();
  double half_car_width = car_param.width() * 0.5;
  double upper_check_lateral = left_width + right_width;
  double lower_check_lateral = left_change ? (right_width - half_car_width)
                                           : (left_width - half_car_width);
  remain_lane_change_time_ = std::fmax(
      0.0,
      common::math::lerp(0.0, lower_check_lateral, FLAGS_lane_change_total_time,
                         upper_check_lateral, remain_lateral_dis));
  return;
}

bool ReferenceLineInfo::LaneChangePathWillPassOverlap() const {
  if (FLAGS_allow_lane_change_pass_overlap) {
    AINFO << "allow lane change behavior in overlap area, so not check";
    return false;
  }
  double dis_to_overlap = std::numeric_limits<double>::max();
  for (const auto& overlap : first_encounter_overlaps_) {
    if (ReferenceLineInfo::OverlapType::CROSSWALK != overlap.first &&
        (!FLAGS_enable_overtake_cross_junction ||
         ReferenceLineInfo::OverlapType::JUNCTION != overlap.first)) {
      continue;
    }
    double dis_to_overlap = overlap.second.end_s - adc_sl_boundary_.end_s();
    dis_to_overlap = std::fmin(dis_to_overlap, dis_to_overlap);
    AINFO << "distance_to_overlap: " << dis_to_overlap;
  }
  if (dis_to_overlap < kEpsilon) {
    return false;
  }
  double ratio =
      (FLAGS_enable_overtake_speed_up) ? FLAGS_overtake_speed_up_ratio : 1.0;
  double adc_max_v = std::fmin(FLAGS_planning_upper_speed_limit * ratio,
                               FLAGS_overtake_upper_speed_limit);
  return dis_to_overlap < (adc_max_v * GetRemainLaneChangeTime());
}

bool ReferenceLineInfo::LaneChangePathWillPassSolidLine() {
  double ratio =
      (FLAGS_enable_overtake_speed_up) ? FLAGS_overtake_speed_up_ratio : 1.0;
  double adc_max_v = std::fmin(FLAGS_planning_upper_speed_limit * ratio,
                               FLAGS_overtake_upper_speed_limit);
  double remain_dis_to_solid_line = GetRemainDistanceToSolidLine();
  if (remain_dis_to_solid_line < (adc_max_v * GetRemainLaneChangeTime())) {
    return true;
  }
  return false;
}

bool ReferenceLineInfo::LaneChangeSafeCheck(const CheckLevel& check_level,
                                            const Obstacle* obstacle) const {
  if (!obstacle) {
    AERROR << "check lane change safe, but input obstacle is nullptr!";
    return false;
  }
  if (CheckLevel::NO_CHECK == check_level) {
    return true;
  }

  // Raw estimation on whether same direction with ADC or not based on
  // prediction trajectory
  bool same_direction = true;
  if (obstacle->HasTrajectory()) {
    double obstacle_moving_direction = obstacle->SpeedHeading();
    // obstacle->Trajectory().trajectory_point(0).path_point().theta();
    // 
    double vehicle_moving_direction = vehicle_state_.heading();
    // TODO(zongxingguo): only consider in reverse type ,no in backward type. 
    // reverse type no consider lanechange.
    // if (vehicle_state_.gear() == canbus::Chassis::GEAR_REVERSE) {
    //   vehicle_moving_direction =
    //       common::math::NormalizeAngle(vehicle_moving_direction + M_PI);
    // }
    double heading_difference = std::fabs(common::math::NormalizeAngle(
        obstacle_moving_direction - vehicle_moving_direction));
    same_direction = heading_difference < kSameDirectionThr;
  }

  double fix_forward_check_ratio = (CheckLevel::CHANGE_CHECK == check_level)
                                       ? kLaneChangeForwardCheckLevelRatio
                                       : 1.0;
  double fix_back_check_ratio = (CheckLevel::CHANGE_CHECK == check_level)
                                    ? kLaneChangeBackCheckLevelRatio
                                    : 1.0;
  double ego_v = std::fabs(vehicle_state_.linear_velocity());

  double kForwardSafeDistance = 0.0;
  double kBackwardSafeDistance = 0.0;

  // set a safe distance based on direction
  if (same_direction) {
    // Same direction, calculate the safe distance in front of ADC:
    // 1. ADC speed is higher, the ForwardSafeDistance is the maximum value in
    //    a constant and relative moving distance.
    // 2. ADC speed is lower, the ForwardSafeDistance is a constant.
    kForwardSafeDistance = std::fmax(
        kForwardMinSafeDistanceOnSameDirection * fix_forward_check_ratio,
        (ego_v - obstacle->speed()) *
            (kSafeTimeOnSameDirection + GetRemainLaneChangeTime()));
    // Same direction, calculate the safe distance behind ADC:
    // 1. ADC speed is higher, the BackwardSafeDistance is a constant.
    // 2. ADC speed is lower, the BackwardSafeDistance is the maximum value in
    //    a constant and relative moving distance.
    kBackwardSafeDistance = std::fmax(
        kBackwardMinSafeDistanceOnSameDirection * fix_back_check_ratio,
        (obstacle->speed() - ego_v) *
            (kSafeTimeOnSameDirection + GetRemainLaneChangeTime()));
  } else {
    // Opposite direction, calculate the safe distance in front of ADC:
    // the ForwardSafeDistance is the maximum value in a constant and relative
    // moving distance.
    kForwardSafeDistance = std::fmax(
        kForwardMinSafeDistanceOnOppositeDirection * fix_forward_check_ratio,
        (ego_v + obstacle->speed()) *
            (kSafeTimeOnOppositeDirection + GetRemainLaneChangeTime()));
    // Opposite direction, calculate the safe distance behind ADC:
    // the BackwardSafeDistance is a constant value.
    kBackwardSafeDistance =
        kBackwardMinSafeDistanceOnOppositeDirection * fix_back_check_ratio;
  }

  // Equation of motion: v^2 / 2a
  if (obstacle->speed() < FLAGS_static_unknown_obstacle_speed_threshold) {
    kForwardSafeDistance =
        std::fmax(kForwardSafeDistance,
                  std::fabs((ego_v * ego_v) /
                            (FLAGS_slowdown_profile_deceleration * 2.0)) +
                      kMinSafeReactionDistance);
    kBackwardSafeDistance =
        std::fmax(0.0, kMinSafeReactionDistance -
                           ego_v * LaneChangeLateralTTC(*obstacle));
  }

  double ego_start_s = adc_sl_boundary_.start_s();
  double ego_end_s = adc_sl_boundary_.end_s();
  const auto& obs_sl = obstacle->PerceptionSLBoundary();

  if (HysteresisFilter(ego_start_s - obs_sl.end_s(), kBackwardSafeDistance,
                       kDistanceBuffer, obstacle->IsLaneChangeBlocking()) &&
      HysteresisFilter(obs_sl.start_s() - ego_end_s, kForwardSafeDistance,
                       kDistanceBuffer, obstacle->IsLaneChangeBlocking())) {
    ADEBUG << "back dis: " << kBackwardSafeDistance
           << ", front_dis: " << kForwardSafeDistance
           << ", is static: " << obstacle->IsStatic()
           << ", same direction: " << same_direction
           << ", change_lane_time: " << GetRemainLaneChangeTime()
           << ", ego_v: " << ego_v << ", obs_v: " << obstacle->speed()
           << ", obs id: " << obstacle->Id() << ", obs_v: " << obstacle->speed()
           << ", obs sl: [s: " << obs_sl.start_s() << "~" << obs_sl.end_s()
           << "] [l:" << obs_sl.start_l() << "~" << obs_sl.end_l() << "]";
    return false;
  }

  return true;
}

double ReferenceLineInfo::LaneChangeLateralTTC(const Obstacle& obstacle) const {
  const auto& obs_sl = obstacle.PerceptionSLBoundary();
  double lateral_diff = std::fmax(adc_sl_boundary_.start_l() - obs_sl.end_l(),
                                  obs_sl.start_l() - adc_sl_boundary_.end_l());
  if (lateral_diff <= FLAGS_obstacle_max_lat_buffer_public_road) {
    return 0.0;
  }
  return (lateral_diff - FLAGS_obstacle_max_lat_buffer_public_road) /
         std::fmax(kEpsilon, std::fabs(adc_lateral_velocity_));
}

bool ReferenceLineInfo::IsObstacleBlockAdc(const Obstacle* obstacle) const {
  if (!obstacle || obstacle->IsVirtual()) {
    return false;
  }
  const auto& obs_sl = obstacle->PerceptionSLBoundary();
  if (obs_sl.end_s() < adc_sl_boundary_.start_s() - kMinSafeReactionDistance) {
    return false;
  }
  double check_speed =
      std::fmax(obstacle->speed(), vehicle_state_.linear_velocity());
  double lateral_buffer =
      common::math::lerp(FLAGS_obstacle_min_lat_buffer_public_road,
                         FLAGS_adc_speed_low_threshold_public_road,
                         FLAGS_obstacle_max_lat_buffer_public_road,
                         FLAGS_planning_upper_speed_limit, check_speed);
  if (perception::PerceptionObstacle::VEHICLE !=
      obstacle->Perception().type()) {
    lateral_buffer = FLAGS_obstacle_min_lat_buffer_public_road;
  }
  if (obstacle->speed() > FLAGS_static_obstacle_speed_threshold) {
    if ((obs_sl.end_l() > adc_sl_boundary_.start_l() - lateral_buffer) &&
        (obs_sl.start_l() < adc_sl_boundary_.end_l() + lateral_buffer)) {
      return true;
    }
    // need consider obstacles beside adc when not occupy the right of way
    if (!is_adc_located_in_lane_) {
      if (obs_sl.start_s() < adc_sl_boundary_.end_s() + kEpsilon &&
          obs_sl.end_s() > adc_sl_boundary_.start_s() - kEpsilon) {
        return true;
      }
    }
  } else {
    double left_space = lane_width_base_on_adc_center_.first - obs_sl.end_l();
    double right_space =
        lane_width_base_on_adc_center_.second + obs_sl.start_l();
    double remain_space = std::fmax(left_space, right_space);
    const auto& veh_param = VehicleConfigHelper::GetConfig().vehicle_param();
    if (remain_space < veh_param.width() + lateral_buffer) {
      return true;
    }
  }
  return false;
}

bool ReferenceLineInfo::HysteresisFilter(
    const double obstacle_distance, const double safe_distance,
    const double distance_buffer, const bool is_obstacle_blocking) const {
  if (is_obstacle_blocking) {
    return obstacle_distance < safe_distance + distance_buffer;
  } else {
    return obstacle_distance < safe_distance - distance_buffer;
  }
}

bool ReferenceLineInfo::IsOriginalRefLine(const bool& is_turn_left) const {
  double ego_start_l = adc_sl_boundary_.start_l();
  double ego_end_l = adc_sl_boundary_.end_l();
  double center_l = (ego_start_l + ego_end_l) * 0.5;
  if ((is_turn_left && center_l < 0.0) || (!is_turn_left && center_l > 0.0)) {
    return false;
  }
  return true;
}

bool ReferenceLineInfo::IsLocatedInSolidLine(
    const SLBoundary& sl_boundary) const {
  double start_s = sl_boundary.start_s();
  double end_s = sl_boundary.end_s();
  double center_s = (start_s + end_s) * 0.5;
  double center_l = (sl_boundary.start_l() + sl_boundary.end_l()) * 0.5;
  common::SLPoint center_point;
  center_point.set_s(center_s);
  center_point.set_l(center_l);
  if (!reference_line_.IsOnLane(center_point)) {
    ADEBUG << "center point not in lane";
    return false;
  }

  if (!IsLaneBoundaryPassable(true, true, start_s)) {
    ADEBUG << "lane boundary not passable at start_s: " << start_s;
    return true;
  }
  if (!IsLaneBoundaryPassable(true, true, center_s)) {
    ADEBUG << "lane boundary not passable at center_s: " << center_s;
    return true;
  }
  if (!IsLaneBoundaryPassable(true, true, end_s)) {
    ADEBUG << "lane boundary not passable at end_s: " << end_s;
    return true;
  }
  return false;
}

bool ReferenceLineInfo::IsInLeftNeighborSolidLine(
    const SLBoundary& sl_boundary) {
  double curr_lane_left_width = lane_width_base_on_adc_center_.first;
  hdmap::Id neighbor_lane_id;
  double left_lane_width = 0.0;
  if (!GetNeighborLaneInfo(ReferenceLineInfo::LaneType::LeftForward,
                           sl_boundary.start_s(), &neighbor_lane_id,
                           &left_lane_width)) {
    return false;
  }
  if ((sl_boundary.end_l() >
       curr_lane_left_width + left_lane_width + kLateralBuffer) ||
      (sl_boundary.start_l() < curr_lane_left_width - kLateralBuffer)) {
    return false;
  }
  if (!IsLaneBoundaryPassable(true, true, sl_boundary.start_s()) ||
      !IsLaneBoundaryPassable(true, true, sl_boundary.end_s())) {
    return true;
  }
  return false;
}

bool ReferenceLineInfo::IsInEitherSolidLine(const SLBoundary& sl_boundary) {
  hdmap::Id neighbor_lane_id;
  double left_lane_width = 0.0;
  if (!GetNeighborLaneInfo(ReferenceLineInfo::LaneType::LeftForward,
                           sl_boundary.start_s(), &neighbor_lane_id,
                           &left_lane_width)) {
    return IsLocatedInSolidLine(sl_boundary);
  }
  double curr_lane_left_width = lane_width_base_on_adc_center_.first;
  double curr_lane_right_width = lane_width_base_on_adc_center_.second;
  if ((sl_boundary.end_l() >
       curr_lane_left_width + left_lane_width + kLateralBuffer) ||
      (sl_boundary.start_l() < -curr_lane_right_width - kLateralBuffer)) {
    return false;
  }
  if (!IsLaneBoundaryPassable(true, true, sl_boundary.start_s()) ||
      !IsLaneBoundaryPassable(true, true, sl_boundary.end_s())) {
    return true;
  }
  return false;
}

bool ReferenceLineInfo::IsInLeftNeighborLine(const SLBoundary& sl_boundary) {
  double curr_lane_left_width = lane_width_base_on_adc_center_.first;
  hdmap::Id neighbor_lane_id;
  double left_lane_width = 0.0;
  if (!GetNeighborLaneInfo(ReferenceLineInfo::LaneType::LeftForward,
                           sl_boundary.start_s(), &neighbor_lane_id,
                           &left_lane_width)) {
    return false;
  }
  double center_l = (sl_boundary.start_l() + sl_boundary.end_l()) * 0.5;
  if (center_l > curr_lane_left_width &&
      center_l < curr_lane_left_width + left_lane_width) {
    return true;
  }
  return false;
}

bool ReferenceLineInfo::IsLaneBoundaryPassable(bool check_left,
                                               bool check_right,
                                               double check_s) const {
  auto ref_point = reference_line_.GetNearestReferencePoint(check_s);
  if (ref_point.lane_waypoints().empty()) {
    return false;
  }

  const auto& waypoint = ref_point.lane_waypoints().front();
  hdmap::LaneBoundaryType::Type lane_boundary_type =
      hdmap::LaneBoundaryType::UNKNOWN;

  if (check_left) {
    lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
    if (hdmap::LaneBoundaryType::DOTTED_WHITE == lane_boundary_type ||
        hdmap::LaneBoundaryType::DOTTED_YELLOW == lane_boundary_type) {
      ADEBUG << "left boundary type: " << static_cast<int>(lane_boundary_type)
             << " can passable at s: " << check_s;
      return true;
    }
  }
  if (check_right) {
    lane_boundary_type = hdmap::RightBoundaryType(waypoint);
    if (hdmap::LaneBoundaryType::DOTTED_WHITE == lane_boundary_type ||
        hdmap::LaneBoundaryType::DOTTED_YELLOW == lane_boundary_type) {
      ADEBUG << "right boundary type: " << static_cast<int>(lane_boundary_type)
             << " can passable at s: " << check_s;
      return true;
    }
  }
  double length = waypoint.lane->total_length();
  ADEBUG << "check_left: " << check_left << ", check_right: " << check_right
         << ", not passable at s: " << check_s
         << ", lane id: " << waypoint.lane->id().id() << ", length: " << length;
  return false;
}

double ReferenceLineInfo::GetRemainDistanceForBack(
    PlanningContext* planning_context) {
  if (has_set_remain_distance_for_back_) {
    return remain_distance_for_back_;
  }
  remain_distance_for_back_ = std::numeric_limits<double>::max();
  if (nullptr == planning_context) {
    AERROR << "nullptr == planning_context";
    return remain_distance_for_back_;
  }

  if (is_adc_center_in_lane_) {
    double left_remain_distance;
    double right_remain_distance;
    GetRemainDistanceToRoutingEndPoint(CheckDirection::LEFT, planning_context,
                                       &left_remain_distance);
    GetRemainDistanceToRoutingEndPoint(CheckDirection::RIGHT, planning_context,
                                       &right_remain_distance);
    remain_distance_for_back_ =
        std::fmin(left_remain_distance, right_remain_distance);
  } else {
    double adc_center_l =
        (adc_sl_boundary_.start_l() + adc_sl_boundary_.end_l()) * 0.5;
    if (adc_center_l > 0.0) {
      double left_remain_distance;
      GetRemainDistanceToRoutingEndPoint(CheckDirection::LEFT, planning_context,
                                         &left_remain_distance);
      remain_distance_for_back_ =
          std::fmin(left_remain_distance, remain_distance_for_back_);
    } else {
      double right_remain_distance;
      GetRemainDistanceToRoutingEndPoint(
          CheckDirection::RIGHT, planning_context, &right_remain_distance);
      remain_distance_for_back_ =
          std::fmin(right_remain_distance, remain_distance_for_back_);
    }
  }
  has_set_remain_distance_for_back_ = true;
  return remain_distance_for_back_;
}

double ReferenceLineInfo::GetRemainDistanceToSolidLine() {
  if (has_set_remain_distance_to_solid_line_) {
    return remain_distance_to_solid_line_;
  }
  remain_distance_to_solid_line_ = std::numeric_limits<double>::max();
  double check_s = adc_sl_boundary_.end_s();
  bool can_use = false;
  while (check_s < reference_line_.Length()) {
    if (!IsLaneBoundaryPassable(true, true, check_s)) {
      can_use = true;
      break;
    }
    check_s += kStepForCheckSolidLine;
  }
  if (can_use) {
    remain_distance_to_solid_line_ = std::fmin(
        check_s - adc_sl_boundary_.end_s(), remain_distance_to_solid_line_);
  }
  has_set_remain_distance_to_solid_line_ = true;
  return remain_distance_to_solid_line_;
}

double ReferenceLineInfo::GetRemainDistanceToJunction() {
  if (has_set_remain_distance_to_junction_) {
    return remain_distance_to_junction_;
  }
  remain_distance_to_junction_ = std::numeric_limits<double>::max();
  for (const auto& overlap : first_encounter_overlaps_) {
    if (ReferenceLineInfo::SIGNAL == overlap.first ||
        ReferenceLineInfo::JUNCTION == overlap.first) {
      remain_distance_to_junction_ =
          std::fmin(remain_distance_to_junction_,
                    overlap.second.start_s - adc_sl_boundary_.end_s());
    }
  }
  has_set_remain_distance_to_junction_ = true;
  return remain_distance_to_junction_;
}

double ReferenceLineInfo::GetRemainDistanceToTurnLane() {
  if (FLAGS_enable_self_borrow_near_turn) {
    remain_distance_to_turn_lane_ = std::numeric_limits<double>::max();
    return remain_distance_to_turn_lane_;
  }
  if (has_set_remain_distance_to_turn_lane_) {
    return remain_distance_to_turn_lane_;
  }
  remain_distance_to_turn_lane_ = std::numeric_limits<double>::max();
  double check_s = adc_sl_boundary_.start_s();
  has_set_remain_distance_to_turn_lane_ = false;

  while (check_s < adc_sl_boundary_.start_s() + FLAGS_distance_to_turnlane) {
    std::vector<hdmap::LaneInfoConstPtr> lanes;
    reference_line_.GetLaneFromS(check_s, &lanes);
    if (!lanes.empty() && lanes.front() != nullptr) {
      const auto& lane = lanes.front()->lane();
      if (lane.has_turn()) {
        // AINFO<<"lane id  = "<<lane.id().id();
        if (Lane::LEFT_TURN == lane.turn()) {
          AINFO << "LEFT_TURN";
          has_set_remain_distance_to_turn_lane_ = true;
          break;
        } else if (Lane::RIGHT_TURN == lane.turn()) {
          AINFO << "RIGHT_TURN";
          has_set_remain_distance_to_turn_lane_ = true;
          break;
        } else if (Lane::U_TURN == lane.turn()) {
          AINFO << "U_TURN";
          has_set_remain_distance_to_turn_lane_ = true;
          break;
        } else {
          // AINFO << "no turn";
        }
      }
    }
    check_s += kStepForCheckSolidLine;
  }
  if (has_set_remain_distance_to_turn_lane_) {
    remain_distance_to_turn_lane_ = check_s - adc_sl_boundary_.start_s();
  }
  // AINFO<<"remain_distance_to_turn_lane_ = "<<remain_distance_to_turn_lane_;
  return remain_distance_to_turn_lane_;
}

double ReferenceLineInfo::GetRemainDistanceToSignal() {
  if (has_set_remain_distance_to_signal_) {
    return remain_distance_to_signal_;
  }
  remain_distance_to_signal_ = std::numeric_limits<double>::max();
  for (const auto& overlap : first_encounter_overlaps_) {
    if (ReferenceLineInfo::SIGNAL == overlap.first) {
      remain_distance_to_signal_ =
          std::fmin(remain_distance_to_signal_,
                    overlap.second.start_s - adc_sl_boundary_.end_s());
    }
  }
  has_set_remain_distance_to_signal_ = true;
  return remain_distance_to_signal_;
}

bool ReferenceLineInfo::GetRemainDistanceToRoutingEndPoint(
    const CheckDirection& check_direction, PlanningContext* planning_context,
    double* const remain_distance) {
  *remain_distance = std::numeric_limits<double>::max();
  if (nullptr == planning_context) {
    AERROR << "nullptr == planning_context";
    return false;
  }

  if (CheckDirection::LEFT == check_direction &&
      has_set_remain_distance_to_left_routing_end_) {
    *remain_distance = remain_distance_to_left_routing_end_;
    return true;
  }

  if (CheckDirection::RIGHT == check_direction &&
      has_set_remain_distance_to_right_routing_end_) {
    *remain_distance = remain_distance_to_right_routing_end_;
    return true;
  }

  if (CheckDirection::BOTH == check_direction &&
      has_set_remain_distance_to_left_routing_end_ &&
      has_set_remain_distance_to_right_routing_end_) {
    *remain_distance = std::fmin(remain_distance_to_left_routing_end_,
                                 remain_distance_to_right_routing_end_);
    return true;
  }

  bool has_routing_end = false;

  if ((CheckDirection::LEFT == check_direction ||
       CheckDirection::BOTH == check_direction) &&
      !has_set_remain_distance_to_left_routing_end_) {
    has_routing_end = HasRoutingEnd(planning_context, PassageType::LEFT);
  }
  if ((CheckDirection::RIGHT == check_direction ||
       CheckDirection::BOTH == check_direction) &&
      !has_set_remain_distance_to_right_routing_end_) {
    has_routing_end = HasRoutingEnd(planning_context, PassageType::RIGHT);
  }

  switch (check_direction) {
    case CheckDirection::LEFT:
      *remain_distance = remain_distance_to_left_routing_end_;
      break;
    case CheckDirection::RIGHT:
      *remain_distance = remain_distance_to_right_routing_end_;
      break;
    case CheckDirection::BOTH:
      *remain_distance = std::fmin(remain_distance_to_left_routing_end_,
                                   remain_distance_to_right_routing_end_);
      break;
    default:
      *remain_distance = std::numeric_limits<double>::max();
  }
  return has_routing_end;
}

bool ReferenceLineInfo::IsADCLocatedInMergeLane() {
  if (adc_located_in_merge_lane_) {
    return true;
  }
  auto locate_lane = LocateLaneInfo(adc_sl_boundary_.start_s());
  if (locate_lane && locate_lane->lane().is_merge()) {
    adc_located_in_merge_lane_ = true;
    return true;
  }
  return false;
}

bool ReferenceLineInfo::IsADCLocatedInAuxiliaryRoad() {
  if (adc_located_in_auxiliary_road_) {
    return true;
  }
  auto locate_lane = LocateLaneInfo(adc_sl_boundary_.end_s());
  if (locate_lane && locate_lane->lane().is_auxiliary_road()) {
    adc_located_in_auxiliary_road_ = true;
    return true;
  }
  return false;
}

bool ReferenceLineInfo::IsADCInCurbArea() {
  if (!is_adc_located_in_lane_) {
    return false;
  }
  if (IsLeftLaneBoundaryCurb(adc_sl_boundary_.end_s()) ||
      IsLeftLaneBoundaryCurb(adc_sl_boundary_.start_s())) {
    return true;
  }
  return false;
}

bool ReferenceLineInfo::IsLeftLaneBoundaryCurb(const double s) {
  auto ref_point = reference_line_.GetNearestReferencePoint(s);
  if (ref_point.lane_waypoints().empty()) {
    return false;
  }

  const auto& waypoint = ref_point.lane_waypoints().front();
  hdmap::LaneBoundaryType::Type lane_boundary_type =
      hdmap::LeftBoundaryType(waypoint);
  return hdmap::LaneBoundaryType::CURB == lane_boundary_type;
}

double ReferenceLineInfo::GetRemainDistanceToCurbArea() {
  if (has_set_remain_distance_to_curb_area_) {
    return remain_distance_to_curb_area_;
  }
  remain_distance_to_curb_area_ = std::numeric_limits<double>::max();
  double check_s = adc_sl_boundary_.end_s();
  bool can_use = false;
  while (check_s < reference_line_.Length()) {
    if (IsLeftLaneBoundaryCurb(check_s)) {
      can_use = true;
      break;
    }
    check_s += kStepForCheckSolidLine;
  }
  if (can_use) {
    remain_distance_to_curb_area_ = std::fmin(
        check_s - adc_sl_boundary_.end_s(), remain_distance_to_curb_area_);
  }
  has_set_remain_distance_to_curb_area_ = true;
  return remain_distance_to_curb_area_;
}

double ReferenceLineInfo::GetAdcHeadingDiffWithRefLine() {
  if (has_set_adc_heading_diff_with_ref_) {
    return heading_diff_between_adc_and_ref_;
  }

  auto ref_point = reference_line_.GetReferencePoint(adc_sl_boundary_.end_s());
  heading_diff_between_adc_and_ref_ =
      common::math::AngleDiff(ref_point.heading(), vehicle_state_.heading());
  has_set_adc_heading_diff_with_ref_ = true;
  return heading_diff_between_adc_and_ref_;
}

bool ReferenceLineInfo::HasLeftNeighborRoutingLane(
    const std::string& direction, PlanningContext* planning_context) {
  double distance_to_routing_end = std::numeric_limits<double>::max();
  if (direction.empty() || (0 != std::strcmp(direction.c_str(), "goleft") &&
                            0 != std::strcmp(direction.c_str(), "goback"))) {
    return false;
  }

  if (!GetRemainDistanceToRoutingEndPoint(
          CheckDirection::LEFT, planning_context, &distance_to_routing_end)) {
    AINFO << "reference line: "
          << LocateLaneInfo(adc_sl_boundary_.end_s())->lane().id().id()
          << " has NO left neighbor routing line.";
    return false;
  }

  if (0 == std::strcmp(direction.c_str(), "goleft")) {
    if (distance_to_routing_end < FLAGS_routing_lane_length_threshold) {
      AINFO << "[goleft] reference line: "
            << LocateLaneInfo(adc_sl_boundary_.end_s())->lane().id().id()
            << " has SHORTER left routing line."
            << " left line length: " << distance_to_routing_end
            << ", length limit: " << FLAGS_routing_lane_length_threshold;
      return false;
    }
  }

  if (0 == std::strcmp(direction.c_str(), "goback")) {
    if (IsAdcCenterInLane()) {
      if (distance_to_routing_end < FLAGS_routing_lane_length_threshold) {
        AINFO << "[goback] reference line: "
              << LocateLaneInfo(adc_sl_boundary_.end_s())->lane().id().id()
              << " has SHORTER left routing line."
              << " adc center in ref lane."
              << " left line length: " << distance_to_routing_end
              << ", length limit: " << FLAGS_routing_lane_length_threshold;
        return false;
      }
    } else if (distance_to_routing_end < reference_line().Length() + kEpsilon) {
      AINFO << "[goback] reference line: "
            << LocateLaneInfo(adc_sl_boundary_.end_s())->lane().id().id()
            << " has SHORTER left routing line."
            << " adc center NOT in ref lane."
            << " left line length: " << distance_to_routing_end
            << ", reference line length: " << reference_line().Length();
      return false;
    }
  }
  return true;
}

bool ReferenceLineInfo::HasRoutingEnd(PlanningContext* planning_context,
                                      const PassageType& passage_type) {
  common::PointENU routing_end;
  if (GetRoutePassageEndPoint(planning_context, passage_type, &routing_end)) {
    common::SLPoint sl_point;
    common::math::Vec2d xy_point = {routing_end.x(), routing_end.y()};
    if (reference_line_.XYToSL(xy_point, &sl_point) ) {
      // AINFO<<"sl_point.s() = "<<sl_point.s();
      //  AINFO<<"sl_point.l() = "<<sl_point.l();
      if (PassageType::LEFT == passage_type) {
        remain_distance_to_left_routing_end_ =
            sl_point.s() - adc_sl_boundary_.end_s();
        has_set_remain_distance_to_left_routing_end_ = true;
// AINFO<<"remain_distance_to_left_routing_end_ = "<<remain_distance_to_left_routing_end_;
      } else if (PassageType::RIGHT == passage_type) {
        remain_distance_to_right_routing_end_ =
            sl_point.s() - adc_sl_boundary_.end_s();
        has_set_remain_distance_to_right_routing_end_ = true;
      }
    }else{
      // AINFO<<"reference_line_.XYToSL(xy_point, &sl_point) failed";
    }
    return true;
  }
  return false;
}

bool ReferenceLineInfo::IsInNearJunctionLaneBorrowScenario(
    PlanningContext* planning_context) {
  const auto& path_decider_status =
      planning_context->planning_status().path_decider();
  double distance_to_routing_end = std::numeric_limits<double>::max();
  bool has_left_routing_lane = GetRemainDistanceToRoutingEndPoint(
      CheckDirection::LEFT, planning_context, &distance_to_routing_end);
  return FLAGS_enable_near_junction_laneborrow &&
         path_decider_status.is_in_path_lane_borrow_scenario() &&
         (has_left_routing_lane ||
          distance_to_routing_end < reference_line_.Length()) &&
         (path_decider_status.is_adc_near_junction() ||
          path_decider_status.is_obs_near_junction());
}

const bool ReferenceLineInfo::IsLeftLaneChange() const {
  double adc_center_l =
      (adc_sl_boundary_.start_l() + adc_sl_boundary_.end_l()) * 0.5;
  return IsChangeLanePath() && (adc_center_l < 0.0);
}

const bool ReferenceLineInfo::IsRightLaneChange() const {
  double adc_center_l =
      (adc_sl_boundary_.start_l() + adc_sl_boundary_.end_l()) * 0.5;
  return IsChangeLanePath() && (adc_center_l > 0.0);
}

const double ReferenceLineInfo::GetMaxKappaFabs() const {
  double max_kappa_fabs = 0;

  if (path_data_.discretized_path().empty()) {
    return max_kappa_fabs;
  }
  for (uint32_t i = 0; i < path_data_.discretized_path().size(); i++) {
   max_kappa_fabs= std::fmax(std::fabs(path_data_.discretized_path()[i].kappa()),max_kappa_fabs);
  }
  return max_kappa_fabs;
}

}  // namespace planning
}  // namespace century
