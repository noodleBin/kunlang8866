/******************************************************************************
 * Copyright 2017 The Century Authors. All Rights Reserved.
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

#include "modules/map/pnc_map/pnc_map.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "absl/strings/str_cat.h"
#include "google/protobuf/text_format.h"

#include "modules/map/proto/map_id.pb.h"

#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/routing/common/routing_gflags.h"

namespace century {
namespace hdmap {

using century::common::PointENU;
using century::common::VehicleState;
using century::common::util::PointFactory;
using century::routing::RoutingResponse;

namespace {

// Maximum lateral error used in trajectory approximation.
const double kTrajectoryApproximationMaxError = 2.0;
// Maximum search radius when querying nearby lanes in HD Map.
const double kMaxDistance = 18.0;  // meters.
// Maximum heading deviation from the ADC when querying nearby lanes in HD Map.
const double kMaxHeading = M_PI * 0.5;
// the buffer of Maximum heading deviation from the ADC when querying nearby
// lanes in HD Map.
const double kHeadingBuffer = M_PI / 10.0;

// TurnAroundChangeGetNearestPointFromRouting For Generate ReferenceLine
const double kTurnAroundSelectRange = 8.0;  // param value
const double kRouteEpsilon = 1e-3;
const double kStopSpeedForRelaxedRouteLaneSearch = 0.5;
const double kOppositeRouteHeadingThreshold = M_PI_2;
}  // namespace

PncMap::PncMap(const HDMap *hdmap) : hdmap_(hdmap) {}

const hdmap::HDMap *PncMap::hdmap() const { return hdmap_; }

LaneWaypoint PncMap::ToLaneWaypoint(
    const routing::LaneWaypoint &waypoint) const {
  auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(waypoint.id()));
  ACHECK(lane) << "Invalid lane id: " << waypoint.id();
  return LaneWaypoint(lane, waypoint.s());
}

double PncMap::LookForwardDistance(double velocity) {
  auto forward_distance = velocity * FLAGS_look_forward_time_sec;

  return forward_distance > FLAGS_look_forward_short_distance
             ? FLAGS_look_forward_long_distance
             : FLAGS_look_forward_short_distance;
}

LaneSegment PncMap::ToLaneSegment(const routing::LaneSegment &segment) const {
  auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(segment.id()));
  ACHECK(lane) << "Invalid lane id: " << segment.id();
  return LaneSegment(lane, segment.start_s(), segment.end_s());
}

void PncMap::UpdateNextRoutingWaypointIndex(int cur_index) {
  if (cur_index < 0) {
    next_routing_waypoint_index_ = 0;
    return;
  }
  if (cur_index >= static_cast<int>(route_indices_.size())) {
    next_routing_waypoint_index_ = routing_waypoint_index_.size() - 1;
    return;
  }
  // Search backwards when the car is driven backward on the route.
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index >
             cur_index) {
    --next_routing_waypoint_index_;
  }
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index ==
             cur_index &&
         adc_waypoint_.s <
             routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
    --next_routing_waypoint_index_;
  }
  // Search forwards
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index <
             cur_index) {
    ++next_routing_waypoint_index_;
  }
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         cur_index ==
             routing_waypoint_index_[next_routing_waypoint_index_].index &&
         adc_waypoint_.s >=
             routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
    ++next_routing_waypoint_index_;
  }
  if (next_routing_waypoint_index_ >= routing_waypoint_index_.size()) {
    next_routing_waypoint_index_ = routing_waypoint_index_.size() - 1;
  }
}

std::vector<routing::LaneWaypoint> PncMap::FutureRouteWaypoints() const {
  const auto &waypoints = routing_.routing_request().waypoint();
  return std::vector<routing::LaneWaypoint>(
      waypoints.begin() + next_routing_waypoint_index_, waypoints.end());
}

void PncMap::UpdateRoutingRange(int adc_index) {
  // Track routing range.
  range_lane_ids_.clear();
  range_start_ = std::max(0, adc_index - 1);
  range_end_ = range_start_;
  while (range_end_ < static_cast<int>(route_indices_.size())) {
    const auto &lane_id = route_indices_[range_end_].segment.lane->id().id();
    if (range_lane_ids_.count(lane_id) != 0) {
      break;
    }
    range_lane_ids_.insert(lane_id);
    ++range_end_;
  }
}

bool PncMap::UpdateVehicleState(const VehicleState &vehicle_state) {
  const auto enable_relaxed_route_lane_search = [&]() -> bool {
    if (std::fabs(vehicle_state.linear_velocity()) >
        kStopSpeedForRelaxedRouteLaneSearch) {
      return false;
    }
    if (route_indices_.empty() || nullptr==route_indices_.front().segment.lane) {
      return false;
    }

    const auto& first_segment = route_indices_.front().segment;
    const double routing_s = std::min(
        std::max(first_segment.start_s, 0.0), first_segment.lane->total_length());
    const double routing_heading = first_segment.lane->Heading(routing_s);
    double vehicle_heading =
        common::math::NormalizeAngle(vehicle_state.heading());
    if (vehicle_state.linear_velocity() < 0.0) {
      vehicle_heading = common::math::NormalizeAngle(vehicle_heading + M_PI);
    }
    const double angle_diff = std::fabs(
        common::math::AngleDiff(vehicle_heading, routing_heading));
    return angle_diff > kOppositeRouteHeadingThreshold;
  }();
  const double max_route_lane_distance =
      enable_relaxed_route_lane_search ? std::numeric_limits<double>::infinity()
                                       : kMaxDistance;
  const auto find_nearest_waypoint_on_route =
      [&](LaneWaypoint *waypoint, const double max_lateral_distance,
          const bool allow_out_of_lane_range) -> bool {
    CHECK_NOTNULL(waypoint);
    waypoint->lane = nullptr;
    const auto point = PointFactory::ToPointENU(vehicle_state);
    double best_segment_gap = std::numeric_limits<double>::infinity();
    double best_lane_distance = std::numeric_limits<double>::infinity();

    for (const auto &route_index : route_indices_) {
      const auto &segment = route_index.segment;
      if (segment.lane == nullptr) {
        continue;
      }

      double s = 0.0;
      double l = 0.0;
      if (!segment.lane->GetProjection({point.x(), point.y()}, &s, &l)) {
        continue;
      }
      if (!allow_out_of_lane_range &&
          (s < -kRouteEpsilon ||
           s > segment.lane->total_length() + kRouteEpsilon)) {
        continue;
      }

      double candidate_s = std::min(
          std::max(s, 0.0), segment.lane->total_length());
      double lane_distance = std::fabs(l);
      if (allow_out_of_lane_range) {
        common::PointENU nearest_point =
            segment.lane->GetNearestPoint({point.x(), point.y()}, &lane_distance);
        double nearest_l = 0.0;
        if (!segment.lane->GetProjection({nearest_point.x(), nearest_point.y()},
                                         &candidate_s, &nearest_l)) {
          continue;
        }
      }

      double segment_gap = 0.0;
      if (candidate_s < segment.start_s) {
        segment_gap = segment.start_s - candidate_s;
      } else if (candidate_s > segment.end_s) {
        segment_gap = candidate_s - segment.end_s;
      }
      if (lane_distance < best_lane_distance ||
          (std::fabs(lane_distance - best_lane_distance) < kRouteEpsilon &&
           segment_gap < best_segment_gap)) {
        best_segment_gap = segment_gap;
        best_lane_distance = lane_distance;
        waypoint->lane = segment.lane;
        waypoint->s =
            std::min(std::max(candidate_s, segment.start_s), segment.end_s);
      }
    }
    if (nullptr == waypoint->lane ||
        best_lane_distance > max_lateral_distance) {
      waypoint->lane = nullptr;
      return false;
    }
    return true;
  };

  if (!ValidateRouting(routing_)) {
    AERROR << "The routing is invalid when updating vehicle state.";
    return false;
  }
  if (!adc_state_.has_x() ||
      (common::util::DistanceXY(adc_state_, vehicle_state) >
       FLAGS_replan_lateral_distance_threshold +
           FLAGS_replan_longitudinal_distance_threshold)) {
    // Position is reset, but not replan.
    next_routing_waypoint_index_ = 0;
    adc_route_index_ = -1;
    stop_for_destination_ = false;
  }

  adc_state_ = vehicle_state;
  bool found_waypoint = false;
  if (enable_relaxed_route_lane_search) {
    found_waypoint = find_nearest_waypoint_on_route(
        &adc_waypoint_, max_route_lane_distance, true);
  }
  if (!found_waypoint && !GetNearestPointFromRouting(vehicle_state,
                                                     &adc_waypoint_)) {
    AERROR << "First Get Nearest Point From Routing With Heading Filed";
    AERROR << "First Failed to get waypoint from routing with point: "
           << "(" << vehicle_state.x() << ", " << vehicle_state.y() << ", "
           << vehicle_state.z() << ").";

    if (FLAGS_enable_generate_reference_line_with_dis) {
      if (!TurnAroundChangeGetNearestPointFromRouting(vehicle_state,
                                                      &adc_waypoint_)) {
        AERROR << "Second Get Nearest Point From Routing With Dis Filed";
        AERROR << "Second Failed to get waypoint from routing with point: "
           << "(" << vehicle_state.x() << ", " << vehicle_state.y() << ", "
           << vehicle_state.z() << ").";
      } else {
        AINFO << "Second Get Nearest Point From Routing With Dis Success";
        found_waypoint = true;
      }
    } else {
      AERROR << "Second Get Nearest Point From Routing With Dis Close";
    }

    if (!found_waypoint &&
        find_nearest_waypoint_on_route(&adc_waypoint_, max_route_lane_distance,
                                       enable_relaxed_route_lane_search)) {
      AINFO << "Fallback to nearest routing lane after nearest lane search "
            << "failed. relaxed=" << enable_relaxed_route_lane_search
            << " fallback_waypoint=" << adc_waypoint_.DebugString();
      found_waypoint = true;
    }

    if (!found_waypoint) {
      return false;
    }
  }

  int route_index = GetWaypointIndex(adc_waypoint_);
  if (route_index < 0 ||
      route_index >= static_cast<int>(route_indices_.size())) {
    LaneWaypoint fallback_waypoint;
    if (find_nearest_waypoint_on_route(&fallback_waypoint,
                                       max_route_lane_distance,
                                       enable_relaxed_route_lane_search)) {
      AINFO << "Fallback to nearest routing lane. "
            << "relaxed=" << enable_relaxed_route_lane_search
            << " "
            << "original_waypoint=" << adc_waypoint_.DebugString()
            << " fallback_waypoint=" << fallback_waypoint.DebugString();
      adc_waypoint_ = fallback_waypoint;
      route_index = GetWaypointIndex(adc_waypoint_);
    }
  }

  if (route_index < 0 ||
      route_index >= static_cast<int>(route_indices_.size())) {
    AERROR << "Cannot find waypoint: " << adc_waypoint_.DebugString();
    return false;
  }

  // Track how many routing request waypoints the adc have passed.
  UpdateNextRoutingWaypointIndex(route_index);
  adc_route_index_ = route_index;
  UpdateRoutingRange(adc_route_index_);

  if (routing_waypoint_index_.empty()) {
    AERROR << "No routing waypoint index.";
    return false;
  }

  if (next_routing_waypoint_index_ == routing_waypoint_index_.size() - 1) {
    stop_for_destination_ = true;
  }
  return true;
}

bool PncMap::IsNewRouting(const routing::RoutingResponse &routing) const {
  return IsNewRouting(routing_, routing);
}

bool PncMap::IsNewRouting(const routing::RoutingResponse &prev,
                          const routing::RoutingResponse &routing) {
  if (!ValidateRouting(routing)) {
    ADEBUG << "The provided routing is invalid.";
    return false;
  }
  if (prev.has_header() && routing.has_header()) {
    bool is_different_routing =
        prev.header().sequence_num() != routing.header().sequence_num() &&
        prev.header().timestamp_sec() != routing.header().timestamp_sec();
    return is_different_routing;
  }
  return true;
  // return !common::util::IsProtoEqual(prev, routing);
}

bool PncMap::UpdateRoutingResponse(routing::RoutingResponse routing) {
  range_lane_ids_.clear();
  route_indices_.clear();
  all_lane_ids_.clear();
  current_route_segment_lane_ids_.clear();

  for (int road_index = 0; road_index < routing.road_size(); ++road_index) {
    auto road_segment = routing.mutable_road(road_index);
    for (int passage_index = 0; passage_index < road_segment->passage_size();
         ++passage_index) {
      auto passage = road_segment->passage(passage_index);
      for (int lane_index = 0; lane_index < passage.segment_size();
           ++lane_index) {
        if (passage.segment(lane_index).id().empty()) {
          AERROR << "Current lane_index" << lane_index
                 << " | Failed to get lane id!";
          return false;
        }
        auto lane = hdmap_->GetLaneById(
            hdmap::MakeMapId(passage.segment(lane_index).id()));
        if (nullptr == lane) {
          AERROR << "Current lane_index" << lane_index
                 << " | Failed to get lane!";
          return false;
        }
        all_lane_ids_.insert(passage.segment(lane_index).id());
        route_indices_.emplace_back();
        route_indices_.back().segment =
            ToLaneSegment(passage.segment(lane_index));
        if (nullptr == route_indices_.back().segment.lane) {
          AERROR << "Failed to get lane segment from passage.";
          return false;
        }
        route_indices_.back().index = {road_index, passage_index, lane_index};
      }
    }
  }

  range_start_ = 0;
  range_end_ = 0;
  adc_route_index_ = -1;
  next_routing_waypoint_index_ = 0;
  UpdateRoutingRange(adc_route_index_);

  routing_waypoint_index_.clear();
  const auto &request_waypoints = routing.routing_request().waypoint();
  if (request_waypoints.empty()) {
    AERROR << "Invalid routing: no request waypoints.";
    return false;
  }
  int i = 0;
  for (size_t j = 0; j < route_indices_.size(); ++j) {
    while (i < request_waypoints.size() &&
           RouteSegments::WithinLaneSegment(route_indices_[j].segment,
                                            request_waypoints.Get(i))) {
      routing_waypoint_index_.emplace_back(
          LaneWaypoint(route_indices_[j].segment.lane,
                       request_waypoints.Get(i).s()),
          j);
      ++i;
    }
  }
  routing_ = routing;
  adc_waypoint_ = LaneWaypoint();
  stop_for_destination_ = false;
  return true;
}

const routing::RoutingResponse &PncMap::routing_response() const {
  return routing_;
}

bool PncMap::ValidateRouting(const RoutingResponse &routing) {
  const int num_road = routing.road_size();
  if (0 == num_road) {
    AERROR << "Route is empty.";
    return false;
  }
  if (!routing.has_routing_request() ||
      routing.routing_request().waypoint_size() < 2) {
    AERROR << "Routing does not have request.";
    return false;
  }
  for (const auto &waypoint : routing.routing_request().waypoint()) {
    if (!waypoint.has_id() || !waypoint.has_s()) {
      AERROR << "Routing waypoint has no lane_id or s.";
      return false;
    }
  }
  return true;
}

int PncMap::SearchForwardWaypointIndex(int start,
                                       const LaneWaypoint &waypoint) const {
  int i = std::max(start, 0);
  while (
      i < static_cast<int>(route_indices_.size()) &&
      !RouteSegments::WithinLaneSegment(route_indices_[i].segment, waypoint)) {
    ++i;
  }
  return i;
}

int PncMap::SearchBackwardWaypointIndex(int start,
                                        const LaneWaypoint &waypoint) const {
  int i = std::min(static_cast<int>(route_indices_.size() - 1), start);
  while (i >= 0 && !RouteSegments::WithinLaneSegment(route_indices_[i].segment,
                                                     waypoint)) {
    --i;
  }
  return i;
}

int PncMap::NextWaypointIndex(int index) const {
  if (index >= static_cast<int>(route_indices_.size() - 1)) {
    return static_cast<int>(route_indices_.size()) - 1;
  } else if (index < 0) {
    return 0;
  } else {
    return index + 1;
  }
}

int PncMap::GetWaypointIndex(const LaneWaypoint &waypoint) const {
  int forward_index = SearchForwardWaypointIndex(adc_route_index_, waypoint);
  if (forward_index >= static_cast<int>(route_indices_.size())) {
    return SearchBackwardWaypointIndex(adc_route_index_, waypoint);
  }
  if (forward_index == adc_route_index_ ||
      forward_index == adc_route_index_ + 1) {
    return forward_index;
  }
  auto backward_index = SearchBackwardWaypointIndex(adc_route_index_, waypoint);
  if (backward_index < 0) {
    return forward_index;
  }

  return (backward_index + 1 == adc_route_index_) ? backward_index
                                                  : forward_index;
}

bool PncMap::PassageToSegments(routing::Passage passage,
                               RouteSegments *segments) const {
  CHECK_NOTNULL(segments);
  segments->clear();
  for (const auto &lane : passage.segment()) {
    auto lane_ptr = hdmap_->GetLaneById(hdmap::MakeMapId(lane.id()));
    if (!lane_ptr) {
      AERROR << "Failed to find lane: " << lane.id();
      return false;
    }
    segments->emplace_back(lane_ptr, std::max(0.0, lane.start_s()),
                           std::min(lane_ptr->total_length(), lane.end_s()));
  }
  return !segments->empty();
}

bool PncMap::GetNeighborPassages(const routing::RoadSegment &road,
                                 const int &overtake_status, int start_passage,
                                 std::vector<int> *const drive_passages) const {
  CHECK_GE(start_passage, 0);
  CHECK_LE(start_passage, road.passage_size());
  CHECK_NOTNULL(drive_passages);
  drive_passages->clear();
  const auto &source_passage = road.passage(start_passage);
  drive_passages->emplace_back(start_passage);
  // Convert Passage to RouteSegments in PNCMAP for generating RouteSegments
  RouteSegments source_segments;
  if (!PassageToSegments(source_passage, &source_segments)) {
    AERROR << "Failed to convert passage to segments";
    return false;
  }

  // 1. If in overtake state
  if (GetNeighborPassagesForOvertake(road, overtake_status, start_passage,
                                     drive_passages, source_segments)) {
    return true;
  }

  auto overtake_direction = ConvertOvertakeDirection(overtake_status);
  if (routing::FORWARD != source_passage.change_lane_type() &&
      overtake_direction != OvertakeDirection::DEFAULT) {
    const PointENU nearest_point =
        adc_waypoint_.lane->GetSmoothPoint(adc_waypoint_.s);
    common::SLPoint sl;
    LaneWaypoint segment_waypoint;
    source_segments.GetProjection(nearest_point, &sl, &segment_waypoint);
    double adc_route_segment_remain_length = std::fmax(
        0.0,
        route_indices_[range_start_].segment.lane->total_length() - sl.s());
    if (adc_route_segment_remain_length >
        FLAGS_usable_route_lc_min_remain_distance) {
      return true;
  }
  }
  // 2. If the current passage is about to exit, it means that it is about to
  //    enter the next passage
  if (source_passage.can_exit()) {  // No need to change lane
    return true;
  }
  // 3. If the next waypoint is in the current passage, must reach the next
  //    waypoint before change lanes
  if (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
      source_segments.IsWaypointOnSegment(
          routing_waypoint_index_[next_routing_waypoint_index_].waypoint)) {
    ADEBUG << "Need to pass next waypoint[" << next_routing_waypoint_index_
           << "] before change lane";
    return true;
  }
  // 4. If the current passage is a left or right turn passage,
  //    query all lanes corresponding to the left or right of the current lane
  //    from hdmap, and then compare with the current passage to find the common
  //    lane, which is the final adjacent lane.
  std::unordered_set<std::string> neighbor_lanes;
  neighbor_lanes.clear();
  if (routing::LEFT == source_passage.change_lane_type()) {
    for (const auto &segment : source_segments) {
      for (const auto &left_id :
           segment.lane->lane().left_neighbor_forward_lane_id()) {
        neighbor_lanes.emplace(left_id.id());
      }
    }
  } else if (routing::RIGHT == source_passage.change_lane_type()) {
    for (const auto &segment : source_segments) {
      for (const auto &right_id :
           segment.lane->lane().right_neighbor_forward_lane_id()) {
        neighbor_lanes.emplace(right_id.id());
      }
    }
  }

  for (int i = 0; i < road.passage_size(); ++i) {
    if (i == start_passage) {
      continue;
    }
    for (const auto &segment : road.passage(i).segment()) {
      if (neighbor_lanes.count(segment.id())) {
        drive_passages->emplace_back(i);
        break;
      }
    }
  }
  return true;
}

bool PncMap::GetNeighborPassagesForOvertake(
    const routing::RoadSegment &road, const int &overtake_status,
    const int start_passage, std::vector<int> *const drive_passages,
    const RouteSegments &source_segments) const {
  const auto &source_passage = road.passage(start_passage);
  std::unordered_set<std::string> neighbor_lanes;
  neighbor_lanes.clear();
  auto overtake_direction = ConvertOvertakeDirection(overtake_status);
  if (overtake_direction == OvertakeDirection::LEFT &&
      routing::RIGHT != source_passage.change_lane_type()) {
    for (const auto &segment : source_segments) {
      for (const auto &left_id :
           segment.lane->lane().left_neighbor_forward_lane_id()) {
        neighbor_lanes.emplace(left_id.id());
      }
    }
  } else if (overtake_direction == OvertakeDirection::RIGHT &&
             routing::LEFT != source_passage.change_lane_type()) {
    for (const auto &segment : source_segments) {
      for (const auto &right_id :
           segment.lane->lane().right_neighbor_forward_lane_id()) {
        neighbor_lanes.emplace(right_id.id());
      }
    }
  } else {
    return false;
  }

  for (int i = 0; i < road.passage_size(); ++i) {
    if (i == start_passage) {
      continue;
    }
    const auto &target_passage = road.passage(i);
    for (const auto &segment : target_passage.segment()) {
      if (neighbor_lanes.count(segment.id())) {
        drive_passages->emplace_back(i);
        break;
      }
    }
  }
  return true;
}

PncMap::OvertakeDirection PncMap::ConvertOvertakeDirection(
    const int &overtake_status) const {
  switch (overtake_status) {
    case static_cast<int>(OvertakeDirection::DEFAULT):
      return OvertakeDirection::DEFAULT;
    case static_cast<int>(OvertakeDirection::LEFT):
      return OvertakeDirection::LEFT;
    case static_cast<int>(OvertakeDirection::RIGHT):
      return OvertakeDirection::RIGHT;
    case static_cast<int>(OvertakeDirection::OVERTAKE):
      return OvertakeDirection::OVERTAKE;
    default:
      return OvertakeDirection::DEFAULT;
  }
}
bool PncMap::GetRouteSegments(const VehicleState &vehicle_state,
                              const int &overtake_status,
                              const std::string &lane_borrow_lane_id,
                              const std::string &lane_change_lane_id,
                              std::list<RouteSegments> *const route_segments) {
  lane_borrow_lane_id_ = lane_borrow_lane_id;
  lane_change_lane_id_ = lane_change_lane_id;
  double look_forward_distance =
      LookForwardDistance(vehicle_state.linear_velocity());
  double look_backward_distance = FLAGS_look_backward_distance;
  return GetRouteSegments(vehicle_state, overtake_status,
                          look_backward_distance, look_forward_distance,
                          route_segments);
}

bool PncMap::GetRouteSegments(const VehicleState &vehicle_state,
                              const int &overtake_status,
                              const double backward_length,
                              const double forward_length,
                              std::list<RouteSegments> *const route_segments) {
  if (!UpdateVehicleState(vehicle_state)) {
    AERROR << "Failed to update vehicle state in pnc_map.";
    return false;
  }
  // Vehicle has to be this close to lane center before considering change
  // lane
  if (!adc_waypoint_.lane || adc_route_index_ < 0 ||
      adc_route_index_ >= static_cast<int>(route_indices_.size())) {
    AERROR << "Invalid vehicle state in pnc_map, update vehicle state first.";
    return false;
  }
  const auto &route_index = route_indices_[adc_route_index_].index;
  const int road_index = route_index[0];
  int passage_index = route_index[1];
  const auto &road = routing_.road(road_index);
  // Raw filter to find all neighboring passages
  std::vector<int> drive_passages;
  // use last passage when no lanechange.(because we added passage)
  std::string prefixion = "road_";
  const auto prefix_size = prefixion.size();
  if (lane_borrow_lane_id_.size() > prefix_size) {
    passage_index = atoi(lane_borrow_lane_id_.substr(prefix_size).c_str());
  } else if (ConvertOvertakeDirection(overtake_status) ==
                 OvertakeDirection::DEFAULT &&
             lane_change_lane_id_.size() > prefix_size) {
    passage_index = atoi(lane_change_lane_id_.substr(prefix_size).c_str());
  }
  GetNeighborPassages(road, overtake_status, passage_index, &drive_passages);
  if (!UpdateRouteSegments(drive_passages, road, passage_index, backward_length,
                           forward_length, route_segments)) {
    return false;
  }

  // update all route segment lane ids
  current_route_segment_lane_ids_.clear();
  for (auto it_route = route_segments->begin();
       it_route != route_segments->end(); ++it_route) {
    for (auto it = it_route->begin(); it != it_route->end(); ++it) {
      current_route_segment_lane_ids_.insert(it->lane->id().id());
    }
  }

  return !route_segments->empty();
}

bool PncMap::UpdateRouteSegments(
    const std::vector<int> &drive_passages, const routing::RoadSegment &road,
    const int passage_index, const double backward_length,
    const double forward_length,
    std::list<RouteSegments> *const route_segments) {
  for (const int index : drive_passages) {
    const auto &passage = road.passage(index);
    RouteSegments segments;
    if (!PassageToSegments(passage, &segments)) {
      ADEBUG << "Failed to convert passage to lane segments.";
      continue;
    }
    const PointENU nearest_point =
        index == passage_index
            ? adc_waypoint_.lane->GetSmoothPoint(adc_waypoint_.s)
            : PointFactory::ToPointENU(adc_state_);
    common::SLPoint sl;
    LaneWaypoint segment_waypoint;
    if (!segments.GetProjection(nearest_point, &sl, &segment_waypoint)) {
      ADEBUG << "Failed to get projection from point: "
             << nearest_point.ShortDebugString();
      continue;
    }
    if (index != passage_index) {
      if (!segments.CanDriveFrom(adc_waypoint_)) {
        ADEBUG << "You cannot drive from current waypoint to passage: "
               << index;
        continue;
      }
    }
    route_segments->emplace_back();
    const auto last_waypoint = segments.LastWaypoint();
    if (!ExtendSegments(segments, sl.s() - backward_length,
                        sl.s() + forward_length, &route_segments->back())) {
      AERROR << "Failed to extend segments with s=" << sl.s()
             << ", backward: " << backward_length
             << ", forward: " << forward_length;
      return false;
    }
    if (route_segments->back().IsWaypointOnSegment(last_waypoint)) {
      route_segments->back().SetRouteEndWaypoint(last_waypoint);
    }
    route_segments->back().SetCanExit(passage.can_exit());
    route_segments->back().SetNextAction(passage.change_lane_type());
    const std::string route_segment_id = absl::StrCat("road", "_", index);
    route_segments->back().SetId(route_segment_id);
    route_segments->back().SetStopForDestination(stop_for_destination_);
    if (index == passage_index) {
      route_segments->back().SetIsOnSegment(true);
      route_segments->back().SetPreviousAction(routing::FORWARD);
    } else if (sl.l() > 0) {
      route_segments->back().SetPreviousAction(routing::RIGHT);
    } else {
      route_segments->back().SetPreviousAction(routing::LEFT);
    }
  }
  return true;
}

bool PncMap::LongFunWithGetNearestPointFromRouting(
    const VehicleState &state,
    std::vector<LaneInfoConstPtr> *const lanes) const {
  const auto point = PointFactory::ToPointENU(state);
  /*
  const int status =
      hdmap_->GetLanesWithHeading(point, kMaxDistance, state.heading(),
                                  kMaxHeading + kHeadingBuffer, *&lanes);
  ADEBUG << "generate_reference_line_with_heading_lanes:" << lanes->size();
  if (status < 0) {
    AERROR << "Failed to get lane from point: " << point.ShortDebugString();
    return false;
  }
  if (lanes->empty()) {
    AERROR << "No valid lane found within " << kMaxDistance
           << " meters with heading " << state.heading();
    return false;
  }
  */

  // get lanes
  int status = hdmap_->GetLanesWithHeading(point, kMaxDistance, state.heading(),
                   kMaxHeading + kHeadingBuffer, *&lanes);
  ADEBUG << "generate_reference_line_with_heading_lanes: " << lanes->size();
  if (status < 0 || lanes->empty()) {       // get lanes failed
    // get parking spaces
    std::vector<ParkingSpaceInfoConstPtr> parking_spaces;
    status = hdmap_->GetParkingSpaces(point, kMaxDistance, &parking_spaces);
    if (status < 0 || parking_spaces.empty()) {  // get parking spaces failed
      AERROR << "Failed to get parking space from point "
             << point.ShortDebugString();
      return false;
    }
    for (auto it : parking_spaces) {
      // point is in parking space
      if (it->polygon().IsPointIn(common::math::Vec2d(point.x(), point.y()))) {
        // parking space overlap lane
        auto lane = hdmap_->GetLaneById(it->lane_id());
        if (nullptr != lane) {
          lanes->push_back(lane);
        }
      }
    }
  }

  if (lanes->empty()) {
    AERROR << "No valid lane found within " << kMaxDistance
           << " meters with heading " << state.heading();
    return false;
  }

  ADEBUG << "lane id : " << lanes->front()->id().id();
  return true;
}

bool PncMap::GetNearestPointFromRouting(const common::VehicleState &state,
                                        LaneWaypoint *waypoint) const {
  waypoint->lane = nullptr;
  std::vector<LaneInfoConstPtr> lanes;
  if (!LongFunWithGetNearestPointFromRouting(state, &lanes)) {
    return false;
  }
  const auto point = PointFactory::ToPointENU(state);

  std::vector<LaneInfoConstPtr> valid_lanes;
  std::copy_if(lanes.begin(), lanes.end(), std::back_inserter(valid_lanes),
               [&](LaneInfoConstPtr ptr) {
                 ADEBUG << "range_lane_ids_.count(ptr->lane().id().id()) = "
                        << range_lane_ids_.count(ptr->lane().id().id());
                 return range_lane_ids_.count(ptr->lane().id().id()) > 0;
               });
  if (valid_lanes.empty()) {
    std::copy_if(lanes.begin(), lanes.end(), std::back_inserter(valid_lanes),
                 [&](LaneInfoConstPtr ptr) {
                   ADEBUG << "all_lane_ids_.count(ptr->lane().id().id()) = "
                          << all_lane_ids_.count(ptr->lane().id().id());
                   return all_lane_ids_.count(ptr->lane().id().id()) > 0;
                 });
  }

  ADEBUG << "valid_lanes = " << valid_lanes.size();
  // Get nearest_waypoints for current position
  double min_distance = std::numeric_limits<double>::infinity();
  for (const auto &lane : valid_lanes) {
    if (0 == range_lane_ids_.count(lane->id().id())) {
      continue;
    }
    if (!current_route_segment_lane_ids_.empty() &&
        0 == current_route_segment_lane_ids_.count(lane->id().id())) {
      continue;
    }
    {
      double s = 0.0;
      double l = 0.0;
      if (!lane->GetProjection({point.x(), point.y()}, &s, &l)) {
        AERROR << "fail to get projection";
        return false;
      }
      // Use large epsilon to allow projection diff
      static constexpr double kEpsilon = 0.5;
      if (s > (lane->total_length() + kEpsilon) || (s + kEpsilon) < 0.0) {
        continue;
      }
    }

    double distance = 0.0;
    common::PointENU map_point =
        lane->GetNearestPoint({point.x(), point.y()}, &distance);
    if (distance < min_distance) {
      min_distance = distance;
      double s = 0.0;
      double l = 0.0;
      if (!lane->GetProjection({map_point.x(), map_point.y()}, &s, &l)) {
        AERROR << "Failed to get projection for map_point: "
               << map_point.DebugString();
        return false;
      }
      waypoint->lane = lane;
      waypoint->s = s;
    }
    ADEBUG << "distance" << distance;
  }
  if (nullptr == waypoint->lane) {
    AERROR << "Failed to find nearest point: " << point.ShortDebugString();
  }
  return waypoint->lane != nullptr;
}

LaneInfoConstPtr PncMap::GetRouteSuccessor(LaneInfoConstPtr lane) const {
  if (lane->lane().successor_id().empty()) {
    return nullptr;
  }
  hdmap::Id preferred_id = lane->lane().successor_id(0);
  std::vector<hdmap::Id> candidate_ids;
  candidate_ids.clear();
  for (const auto &lane_id : lane->lane().successor_id()) {
    if (range_lane_ids_.count(lane_id.id()) != 0) {
      candidate_ids.push_back(lane_id);
    }
  }

  if (1 == candidate_ids.size()) {
    preferred_id = candidate_ids[0];
  } else if (candidate_ids.size() > 1) {
    for (const auto &candidate_id : candidate_ids) {
      auto candidate_lane = hdmap_->GetLaneById(candidate_id);
      if (LeftLaneExitInCandidate(candidate_lane, candidate_ids)) {
        preferred_id = candidate_id;
      break;
    }
  }
  } else {
    // nothind to do
  }

  return hdmap_->GetLaneById(preferred_id);
}

bool PncMap::LeftLaneExitInCandidate(
    LaneInfoConstPtr lane, const std::vector<hdmap::Id> &candidate_ids) const {
  if (nullptr == lane) {
    return false;
  }

  for (const auto &lane_id : lane->lane().left_neighbor_forward_lane_id()) {
    for (const auto &candidate_id : candidate_ids) {
      if (0 == std::strcmp(lane_id.id().c_str(), candidate_id.id().c_str())) {
        return true;
      }
    }
  }

  return false;
}

LaneInfoConstPtr PncMap::GetRoutePredecessor(LaneInfoConstPtr lane) const {
  if (lane->lane().predecessor_id().empty()) {
    return nullptr;
  }

  std::unordered_set<std::string> predecessor_ids;
  for (const auto &lane_id : lane->lane().predecessor_id()) {
    predecessor_ids.insert(lane_id.id());
  }

  hdmap::Id preferred_id = lane->lane().predecessor_id(0);
  auto preferred_lane = hdmap_->GetLaneById(preferred_id);
  bool preferred_lane_is_turn = false;
  if(preferred_lane != nullptr){
    preferred_lane_is_turn = preferred_lane->lane().turn() ==Lane::LEFT_TURN
    || preferred_lane->lane().turn() ==Lane::RIGHT_TURN ||preferred_lane->lane().turn() ==Lane::U_TURN;
  }
  // AINFO<<"preferred_lane_is_turn = "<<preferred_lane_is_turn;
  if(preferred_lane_is_turn){
  for (const auto &predecessor_id : lane->lane().predecessor_id()) {
    // AINFO<<"predecessor_id = "<< predecessor_id.id();
      auto predecess_lane = hdmap_->GetLaneById(predecessor_id);
      
      if(predecess_lane!=nullptr){
       bool  is_turn_lane = predecess_lane->lane().turn() ==Lane::LEFT_TURN
    || predecess_lane->lane().turn() ==Lane::RIGHT_TURN ||predecess_lane->lane().turn() ==Lane::U_TURN;
        if(!is_turn_lane){
          preferred_id = predecessor_id;
          break;
        }
      }
  }
   
  }
  for (const auto &route_index : route_indices_) {
    auto &lane = route_index.segment.lane->id();
    if (predecessor_ids.count(lane.id()) != 0) {
      preferred_id = lane;
      break;
    }
  }
  return hdmap_->GetLaneById(preferred_id);
}

bool PncMap::ExtendSegments(const RouteSegments &segments,
                            const common::PointENU &point, double look_backward,
                            double look_forward,
                            RouteSegments *extended_segments) {
  common::SLPoint sl;
  LaneWaypoint waypoint;
  if (!segments.GetProjection(point, &sl, &waypoint)) {
    AERROR << "point: " << point.ShortDebugString() << " is not on segment";
    return false;
  }
  return ExtendSegments(segments, sl.s() - look_backward, sl.s() + look_forward,
                        extended_segments);
}

bool PncMap::ExtendSegments(const RouteSegments &segments, double start_s,
                            double end_s,
                            RouteSegments *const truncated_segments) const {
  if (segments.empty()) {
    AERROR << "The input segments is empty";
    return false;
  }
  CHECK_NOTNULL(truncated_segments);
  truncated_segments->SetProperties(segments);

  if (start_s >= end_s) {
    AERROR << "start_s(" << start_s << " >= end_s(" << end_s << ")";
    return false;
  }
  std::unordered_set<std::string> unique_lanes;
  // Extend the trajectory towards the start of the trajectory.
  ExtendPredecessorSegments(segments, start_s, &unique_lanes,
                            truncated_segments);

  double router_s = 0;
  for (const auto &lane_segment : segments) {
    const double adjusted_start_s = std::max(
        start_s - router_s + lane_segment.start_s, lane_segment.start_s);
    const double adjusted_end_s =
        std::min(end_s - router_s + lane_segment.start_s, lane_segment.end_s);
    if (adjusted_start_s < adjusted_end_s) {
      if (!truncated_segments->empty() &&
          truncated_segments->back().lane->id().id() ==
              lane_segment.lane->id().id()) {
        truncated_segments->back().end_s = adjusted_end_s;
      } else if (unique_lanes.find(lane_segment.lane->id().id()) ==
                 unique_lanes.end()) {
        truncated_segments->emplace_back(lane_segment.lane, adjusted_start_s,
                                         adjusted_end_s);
        unique_lanes.insert(lane_segment.lane->id().id());
      } else {
        return true;
      }
    }
    router_s += (lane_segment.end_s - lane_segment.start_s);
    if (router_s > end_s) {
      break;
    }
  }

  // Extend the trajectory towards the end of the trajectory.
  ExtendSuccessorSegments(segments, end_s, router_s, &unique_lanes,
                          truncated_segments);

    return true;
  }

void PncMap::ExtendPredecessorSegments(
    const RouteSegments &segments, double start_s,
    std::unordered_set<std::string> *const unique_lanes,
    RouteSegments *const truncated_segments) const {
  if (start_s < 0) {
    const auto &first_segment = *segments.begin();
    auto lane = first_segment.lane;
    double s = first_segment.start_s;
    double extend_s = -start_s;
    std::vector<LaneSegment> extended_lane_segments;
    while (extend_s > kRouteEpsilon) {
      if (s <= kRouteEpsilon) {
        lane = GetRoutePredecessor(lane);
        if (nullptr == lane ||
            unique_lanes->find(lane->id().id()) != unique_lanes->end()) {
          break;
        }
        s = lane->total_length();
      } else {
        const double length = std::min(s, extend_s);
        extended_lane_segments.emplace_back(lane, s - length, s);
        extend_s -= length;
        s -= length;
        unique_lanes->insert(lane->id().id());
      }
    }
    truncated_segments->insert(truncated_segments->begin(),
                               extended_lane_segments.rbegin(),
                               extended_lane_segments.rend());
  }
}

void PncMap::ExtendSuccessorSegments(
    const RouteSegments &segments, double end_s, double router_s,
    std::unordered_set<std::string> *const unique_lanes,
    RouteSegments *const truncated_segments) const {
  if (router_s < end_s && !truncated_segments->empty()) {
    auto &back = truncated_segments->back();
    if (back.lane->total_length() > back.end_s) {
      double origin_end_s = back.end_s;
      back.end_s =
          std::min(back.end_s + end_s - router_s, back.lane->total_length());
      router_s += back.end_s - origin_end_s;
    }
  }
  auto last_lane = segments.back().lane;
  while (router_s < end_s - kRouteEpsilon) {
    last_lane = GetRouteSuccessor(last_lane);
    if (nullptr == last_lane ||
        unique_lanes->find(last_lane->id().id()) != unique_lanes->end()) {
      break;
    }
    const double length = std::min(end_s - router_s, last_lane->total_length());
    truncated_segments->emplace_back(last_lane, 0, length);
    unique_lanes->insert(last_lane->id().id());
    router_s += length;
  }
}

void PncMap::AppendLaneToPoints(LaneInfoConstPtr lane, const double start_s,
                                const double end_s,
                                std::vector<MapPathPoint> *const points) {
  if (nullptr == points || start_s >= end_s) {
    return;
  }
  double accumulate_s = 0.0;
  for (size_t i = 0; i < lane->points().size(); ++i) {
    if (accumulate_s >= start_s && accumulate_s <= end_s) {
      points->emplace_back(lane->points()[i], lane->headings()[i],
                           LaneWaypoint(lane, accumulate_s));
    }
    if (i < lane->segments().size()) {
      const auto &segment = lane->segments()[i];
      const double next_accumulate_s = accumulate_s + segment.length();
      if (start_s > accumulate_s && start_s < next_accumulate_s) {
        points->emplace_back(segment.start() + segment.unit_direction() *
                                                   (start_s - accumulate_s),
                             lane->headings()[i], LaneWaypoint(lane, start_s));
      }
      if (end_s > accumulate_s && end_s < next_accumulate_s) {
        points->emplace_back(
            segment.start() + segment.unit_direction() * (end_s - accumulate_s),
            lane->headings()[i], LaneWaypoint(lane, end_s));
      }
      accumulate_s = next_accumulate_s;
    }
    if (accumulate_s > end_s) {
      break;
    }
  }
}

bool PncMap::TurnAroundChangeGetNearestPointFromRouting(
    const VehicleState &state, LaneWaypoint *waypoint) const {
  waypoint->lane = nullptr;
  std::vector<LaneInfoConstPtr> lanes;
  const auto point = PointFactory::ToPointENU(state);

  const int status =
      hdmap_->GetLanesWithNearPos(point, kTurnAroundSelectRange, &lanes);
  ADEBUG << "generate_reference_line_with_dis_lanes:" << lanes.size();
  if (status < 0) {
    AERROR << "Failed to get lane from point: " << point.ShortDebugString();
    return false;
  }
  if (lanes.empty()) {
    AERROR << "No valid lane found within " << kMaxDistance
           << " meters with heading " << state.heading();
    return false;
  }

  ADEBUG << "lane id : " << lanes[0]->id().id();

  std::vector<LaneInfoConstPtr> valid_lanes;
  std::copy_if(lanes.begin(), lanes.end(), std::back_inserter(valid_lanes),
               [&](LaneInfoConstPtr ptr) {
                 ADEBUG << "range_lane_ids_.count(ptr->lane().id().id()) = "
                        << range_lane_ids_.count(ptr->lane().id().id());
                 return range_lane_ids_.count(ptr->lane().id().id()) > 0;
               });
  if (valid_lanes.empty()) {
    std::copy_if(lanes.begin(), lanes.end(), std::back_inserter(valid_lanes),
                 [&](LaneInfoConstPtr ptr) {
                   ADEBUG << "all_lane_ids_.count(ptr->lane().id().id()) = "
                          << all_lane_ids_.count(ptr->lane().id().id());
                   return all_lane_ids_.count(ptr->lane().id().id()) > 0;
                 });
  }

  ADEBUG << "valid_lanes = " << valid_lanes.size();
  // Get nearest_waypoints for current position
  double min_distance = std::numeric_limits<double>::infinity();
  for (const auto &lane : valid_lanes) {
    if (0 == range_lane_ids_.count(lane->id().id())) {
      continue;
    }
    {
      double s = 0.0;
      double l = 0.0;
      if (!lane->GetProjection({point.x(), point.y()}, &s, &l)) {
        AERROR << "fail to get projection";
        return false;
      }
      // Use large epsilon to allow projection diff
      static constexpr double kEpsilon = 0.5;
      if (s > (lane->total_length() + kEpsilon) || (s + kEpsilon) < 0.0) {
        continue;
      }
    }

    double distance = 0.0;
    common::PointENU map_point =
        lane->GetNearestPoint({point.x(), point.y()}, &distance);
    if (distance < min_distance) {
      min_distance = distance;
      double s = 0.0;
      double l = 0.0;
      if (!lane->GetProjection({map_point.x(), map_point.y()}, &s, &l)) {
        AERROR << "Failed to get projection for map_point: "
               << map_point.DebugString();
        return false;
      }
      waypoint->lane = lane;
      waypoint->s = s;
    }
    ADEBUG << "distance" << distance;
  }
  if (nullptr == waypoint->lane) {
    ADEBUG << "Failed to find nearest point: " << point.ShortDebugString();
  }
  return waypoint->lane != nullptr;
}

}  // namespace hdmap
}  // namespace century
