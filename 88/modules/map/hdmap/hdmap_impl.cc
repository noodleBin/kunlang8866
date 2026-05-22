/* Copyright 2017 The Century Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/

#include "modules/map/hdmap/hdmap_impl.h"

#include <algorithm>
#include <limits>
#include <mutex>
#include <set>
#include <unordered_set>

#include "absl/strings/match.h"

#include "cyber/common/file.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/adapter/opendrive_adapter.h"

namespace century {
namespace hdmap {
namespace {

using century::common::PointENU;
using century::common::math::AABoxKDTreeParams;
using century::common::math::Vec2d;

Id CreateHDMapId(const std::string& string_id) {
  Id id;
  id.set_id(string_id);
  return id;
}

// default lanes search radius in GetForwardNearestSignalsOnLane
constexpr double kLanesSearchRange = 10.0;
// backward search distance in GetForwardNearestSignalsOnLane
constexpr int kBackwardDistance = 4;
// Regional search radius
constexpr int kRegionalSearchRadius = 2;

}  // namespace

int HDMapImpl::LoadMapFromFile(const std::string& map_filename) {
  Clear();
  // TODO(All) seems map_ can be changed to a local variable of this
  // function, but test will fail if I do so. if so.
  if (absl::EndsWith(map_filename, ".xml")) {
    if (!adapter::OpendriveAdapter::LoadData(map_filename, &map_)) {
      return -1;
    }
  } else if (!cyber::common::GetProtoFromFile(map_filename, &map_)) {
    return -1;
  }

  return LoadMapFromProto(map_);
}

int HDMapImpl::LoadMapFromProto(const Map& map_proto) {
  if (&map_proto != &map_) {  // avoid an unnecessary copy
    Clear();
    map_ = map_proto;
  }

#define PROCESS_MAP_ELEMENT(Table, Name, Class)   \
  for (const auto& item : map_.Name()) {          \
    Table[item.id().id()].reset(new Class(item)); \
  }
  PROCESS_MAP_ELEMENT(lane_table_, lane, LaneInfo);
  PROCESS_MAP_ELEMENT(junction_table_, junction, JunctionInfo);
  PROCESS_MAP_ELEMENT(signal_table_, signal, SignalInfo);
  PROCESS_MAP_ELEMENT(crosswalk_table_, crosswalk, CrosswalkInfo);
  PROCESS_MAP_ELEMENT(stop_sign_table_, stop_sign, StopSignInfo);
  PROCESS_MAP_ELEMENT(yield_sign_table_, yield, YieldSignInfo);
  PROCESS_MAP_ELEMENT(clear_area_table_, clear_area, ClearAreaInfo);
  PROCESS_MAP_ELEMENT(speed_bump_table_, speed_bump, SpeedBumpInfo);
  PROCESS_MAP_ELEMENT(parking_space_table_, parking_space, ParkingSpaceInfo);
  PROCESS_MAP_ELEMENT(parking_lot_table_, parking_lot, ParkingLotInfo);
  PROCESS_MAP_ELEMENT(pnc_junction_table_, pnc_junction, PNCJunctionInfo);
  PROCESS_MAP_ELEMENT(rsu_table_, rsu, RSUInfo);
  PROCESS_MAP_ELEMENT(overlap_table_, overlap, OverlapInfo);
  PROCESS_MAP_ELEMENT(road_table_, road, RoadInfo);
  PROCESS_MAP_ELEMENT(obstacle_table_, obstacle, ObstacleInfo);
  PROCESS_MAP_ELEMENT(electric_fence_table_, electric_fence, ElectricFenceInfo);
  // parking space associate to parking lot and lane
  for (auto& parking_space : parking_space_table_) {
    for (const auto overlap_id :
         parking_space.second->parking_space().overlap_id()) {
      auto overlap = overlap_table_[overlap_id.id()];
      for (const auto object : overlap->overlap().object()) {
        if (parking_lot_table_.find(object.id().id()) !=
            parking_lot_table_.end()) {
          parking_space.second->set_parking_lot_id(object.id());
        } else if (lane_table_.find(object.id().id()) != lane_table_.end()) {
          parking_space.second->set_lane_id(object.id());
          parking_space.second->set_lane_start_s(
              object.lane_overlap_info().start_s());
          parking_space.second->set_lane_end_s(
              object.lane_overlap_info().end_s());
        }
      }
    }
  }

  // PROCESS_MAP_ELEMENT(polygon_table_, polygon, PolygonInfo);
  for (const auto& road_ptr_pair : road_table_) {
    const auto& road_id = road_ptr_pair.second->id();
    for (const auto& road_section : road_ptr_pair.second->sections()) {
      const auto& section_id = road_section.id();
      for (const auto& lane_id : road_section.lane_id()) {
        auto iter = lane_table_.find(lane_id.id());
        if (iter != lane_table_.end()) {
          iter->second->set_road_id(road_id);
          iter->second->set_section_id(section_id);
        } else {
          AFATAL << "Unknown lane id: " << lane_id.id();
        }
      }
    }
  }

  for (const auto& lane_ptr_pair : lane_table_) {
    lane_ptr_pair.second->PostProcess(*this);
  }
  for (const auto& junction_ptr_pair : junction_table_) {
    junction_ptr_pair.second->PostProcess(*this);
  }
  for (const auto& stop_sign_ptr_pair : stop_sign_table_) {
    stop_sign_ptr_pair.second->PostProcess(*this);
  }
  BuildLaneSegmentKDTree();
  BuildJunctionPolygonKDTree();
  BuildSignalSegmentKDTree();
  BuildCrosswalkPolygonKDTree();
  BuildStopSignSegmentKDTree();
  BuildYieldSignSegmentKDTree();
  BuildClearAreaPolygonKDTree();
  BuildSpeedBumpSegmentKDTree();
  BuildParkingSpacePolygonKDTree();
  BuildParkingLotPolygonKDTree();
  BuildPNCJunctionPolygonKDTree();
  BuildObstaclePolygonKDTree();
  BuildElectricFencePolygonKDTree();
  return 0;
}

LaneInfoConstPtr HDMapImpl::GetLaneById(const Id& id) const {
  LaneTable::const_iterator it = lane_table_.find(id.id());
  return it != lane_table_.end() ? it->second : nullptr;
}

JunctionInfoConstPtr HDMapImpl::GetJunctionById(const Id& id) const {
  JunctionTable::const_iterator it = junction_table_.find(id.id());
  return it != junction_table_.end() ? it->second : nullptr;
}

SignalInfoConstPtr HDMapImpl::GetSignalById(const Id& id) const {
  SignalTable::const_iterator it = signal_table_.find(id.id());
  return it != signal_table_.end() ? it->second : nullptr;
}

CrosswalkInfoConstPtr HDMapImpl::GetCrosswalkById(const Id& id) const {
  CrosswalkTable::const_iterator it = crosswalk_table_.find(id.id());
  return it != crosswalk_table_.end() ? it->second : nullptr;
}

StopSignInfoConstPtr HDMapImpl::GetStopSignById(const Id& id) const {
  StopSignTable::const_iterator it = stop_sign_table_.find(id.id());
  return it != stop_sign_table_.end() ? it->second : nullptr;
}

YieldSignInfoConstPtr HDMapImpl::GetYieldSignById(const Id& id) const {
  YieldSignTable::const_iterator it = yield_sign_table_.find(id.id());
  return it != yield_sign_table_.end() ? it->second : nullptr;
}

ClearAreaInfoConstPtr HDMapImpl::GetClearAreaById(const Id& id) const {
  ClearAreaTable::const_iterator it = clear_area_table_.find(id.id());
  return it != clear_area_table_.end() ? it->second : nullptr;
}

SpeedBumpInfoConstPtr HDMapImpl::GetSpeedBumpById(const Id& id) const {
  SpeedBumpTable::const_iterator it = speed_bump_table_.find(id.id());
  return it != speed_bump_table_.end() ? it->second : nullptr;
}

OverlapInfoConstPtr HDMapImpl::GetOverlapById(const Id& id) const {
  OverlapTable::const_iterator it = overlap_table_.find(id.id());
  return it != overlap_table_.end() ? it->second : nullptr;
}

RoadInfoConstPtr HDMapImpl::GetRoadById(const Id& id) const {
  RoadTable::const_iterator it = road_table_.find(id.id());
  return it != road_table_.end() ? it->second : nullptr;
}

ParkingSpaceInfoConstPtr HDMapImpl::GetParkingSpaceById(const Id& id) const {
  ParkingSpaceTable::const_iterator it = parking_space_table_.find(id.id());
  return it != parking_space_table_.end() ? it->second : nullptr;
}

ParkingLotInfoConstPtr HDMapImpl::GetParkingLotById(const Id& id) const {
  ParkingLotTable::const_iterator it = parking_lot_table_.find(id.id());
  return it != parking_lot_table_.end() ? it->second : nullptr;
}

PNCJunctionInfoConstPtr HDMapImpl::GetPNCJunctionById(const Id& id) const {
  PNCJunctionTable::const_iterator it = pnc_junction_table_.find(id.id());
  return it != pnc_junction_table_.end() ? it->second : nullptr;
}

RSUInfoConstPtr HDMapImpl::GetRSUById(const Id& id) const {
  RSUTable::const_iterator it = rsu_table_.find(id.id());
  return it != rsu_table_.end() ? it->second : nullptr;
}

ObstacleInfoConstPtr HDMapImpl::GetObstacleAreaById(const Id& id) const {
  ObstacleTable::const_iterator it = obstacle_table_.find(id.id());
  return it != obstacle_table_.end() ? it->second : nullptr;
}

ElectricFenceInfoConstPtr HDMapImpl::GetElectricFenceById(const Id& id) const {
  ElectricFenceTable::const_iterator it = electric_fence_table_.find(id.id());
  return it != electric_fence_table_.end() ? it->second : nullptr;
}

int HDMapImpl::GetLanes(const PointENU& point, double distance,
                        std::vector<LaneInfoConstPtr>* lanes) const {
  return GetLanes({point.x(), point.y()}, distance, lanes);
}

int HDMapImpl::GetLanes(const Vec2d& point, double distance,
                        std::vector<LaneInfoConstPtr>* lanes) const {
  if (lanes == nullptr || lane_segment_kdtree_ == nullptr) {
    return -1;
  }
  lanes->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *lane_segment_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto& id : ids) {
    lanes->emplace_back(GetLaneById(CreateHDMapId(id)));
  }
  return 0;
}

int HDMapImpl::GetRoads(const PointENU& point, double distance,
                        std::vector<RoadInfoConstPtr>* roads) const {
  return GetRoads({point.x(), point.y()}, distance, roads);
}

int HDMapImpl::GetRoads(const Vec2d& point, double distance,
                        std::vector<RoadInfoConstPtr>* roads) const {
  std::vector<LaneInfoConstPtr> lanes;
  if (GetLanes(point, distance, &lanes) != 0) {
    return -1;
  }
  std::unordered_set<std::string> road_ids;
  for (auto& lane : lanes) {
    if (!lane->road_id().id().empty()) {
      road_ids.insert(lane->road_id().id());
    }
  }

  for (auto& road_id : road_ids) {
    RoadInfoConstPtr road = GetRoadById(CreateHDMapId(road_id));
    CHECK_NOTNULL(road);
    roads->push_back(road);
  }

  return 0;
}

int HDMapImpl::GetObstacleAreas(
    const PointENU& point, double distance,
    std::vector<ObstacleInfoConstPtr>* obstacles) const {
  return GetObstacleAreas({point.x(), point.y()}, distance, obstacles);
}

int HDMapImpl::GetObstacleAreas(
    const Vec2d& point, double distance,
    std::vector<ObstacleInfoConstPtr>* obstacles) const {
  if (obstacles == nullptr || obstacle_polygon_kdtree_ == nullptr) {
    return -1;
  }

  obstacles->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *obstacle_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto& id : ids) {
    obstacles->emplace_back(GetObstacleAreaById(CreateHDMapId(id)));
  }
  return 0;
}

int HDMapImpl::GetJunctions(
    const PointENU& point, double distance,
    std::vector<JunctionInfoConstPtr>* junctions) const {
  return GetJunctions({point.x(), point.y()}, distance, junctions);
}

int HDMapImpl::GetJunctions(
    const Vec2d& point, double distance,
    std::vector<JunctionInfoConstPtr>* junctions) const {
  if (junctions == nullptr || junction_polygon_kdtree_ == nullptr) {
    return -1;
  }
  junctions->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *junction_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto& id : ids) {
    junctions->emplace_back(GetJunctionById(CreateHDMapId(id)));
  }
  return 0;
}

int HDMapImpl::GetElectricFences(
    const PointENU& point, double distance,
    std::vector<ElectricFenceInfoConstPtr>* fences) const {
  return GetElectricFences({point.x(), point.y()}, distance, fences);
}

int HDMapImpl::GetElectricFences(
    const Vec2d& point, double distance,
    std::vector<ElectricFenceInfoConstPtr>* fences) const {
  if (fences == nullptr || electric_fence_polygon_kdtree_ == nullptr) {
    return -1;
  }
  fences->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *electric_fence_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto& id : ids) {
    fences->emplace_back(GetElectricFenceById(CreateHDMapId(id)));
  }
  return 0;
}

int HDMapImpl::GetSignals(const PointENU& point, double distance,
                          std::vector<SignalInfoConstPtr>* signals) const {
  return GetSignals({point.x(), point.y()}, distance, signals);
}

int HDMapImpl::GetSignals(const Vec2d& point, double distance,
                          std::vector<SignalInfoConstPtr>* signals) const {
  if (signals == nullptr || signal_segment_kdtree_ == nullptr) {
    return -1;
  }
  signals->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *signal_segment_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto& id : ids) {
    signals->emplace_back(GetSignalById(CreateHDMapId(id)));
  }
  return 0;
}

int HDMapImpl::GetCrosswalks(
    const PointENU& point, double distance,
    std::vector<CrosswalkInfoConstPtr>* crosswalks) const {
  return GetCrosswalks({point.x(), point.y()}, distance, crosswalks);
}

int HDMapImpl::GetCrosswalks(
    const Vec2d& point, double distance,
    std::vector<CrosswalkInfoConstPtr>* crosswalks) const {
  if (crosswalks == nullptr || crosswalk_polygon_kdtree_ == nullptr) {
    return -1;
  }
  crosswalks->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *crosswalk_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto& id : ids) {
    crosswalks->emplace_back(GetCrosswalkById(CreateHDMapId(id)));
  }
  return 0;
}

int HDMapImpl::GetStopSigns(
    const PointENU& point, double distance,
    std::vector<StopSignInfoConstPtr>* stop_signs) const {
  return GetStopSigns({point.x(), point.y()}, distance, stop_signs);
}

int HDMapImpl::GetStopSigns(
    const Vec2d& point, double distance,
    std::vector<StopSignInfoConstPtr>* stop_signs) const {
  if (stop_signs == nullptr || stop_sign_segment_kdtree_ == nullptr) {
    return -1;
  }
  stop_signs->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *stop_sign_segment_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto& id : ids) {
    stop_signs->emplace_back(GetStopSignById(CreateHDMapId(id)));
  }
  return 0;
}

int HDMapImpl::GetYieldSigns(
    const PointENU& point, double distance,
    std::vector<YieldSignInfoConstPtr>* yield_signs) const {
  return GetYieldSigns({point.x(), point.y()}, distance, yield_signs);
}

int HDMapImpl::GetYieldSigns(
    const Vec2d& point, double distance,
    std::vector<YieldSignInfoConstPtr>* yield_signs) const {
  if (yield_signs == nullptr || yield_sign_segment_kdtree_ == nullptr) {
    return -1;
  }
  yield_signs->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *yield_sign_segment_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto& id : ids) {
    yield_signs->emplace_back(GetYieldSignById(CreateHDMapId(id)));
  }

  return 0;
}

int HDMapImpl::GetClearAreas(
    const PointENU& point, double distance,
    std::vector<ClearAreaInfoConstPtr>* clear_areas) const {
  return GetClearAreas({point.x(), point.y()}, distance, clear_areas);
}

int HDMapImpl::GetClearAreas(
    const Vec2d& point, double distance,
    std::vector<ClearAreaInfoConstPtr>* clear_areas) const {
  if (clear_areas == nullptr || clear_area_polygon_kdtree_ == nullptr) {
    return -1;
  }
  clear_areas->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *clear_area_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto& id : ids) {
    clear_areas->emplace_back(GetClearAreaById(CreateHDMapId(id)));
  }

  return 0;
}

int HDMapImpl::GetSpeedBumps(
    const PointENU& point, double distance,
    std::vector<SpeedBumpInfoConstPtr>* speed_bumps) const {
  return GetSpeedBumps({point.x(), point.y()}, distance, speed_bumps);
}

int HDMapImpl::GetSpeedBumps(
    const Vec2d& point, double distance,
    std::vector<SpeedBumpInfoConstPtr>* speed_bumps) const {
  if (speed_bumps == nullptr || speed_bump_segment_kdtree_ == nullptr) {
    return -1;
  }
  speed_bumps->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *speed_bump_segment_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto& id : ids) {
    speed_bumps->emplace_back(GetSpeedBumpById(CreateHDMapId(id)));
  }

  return 0;
}

int HDMapImpl::GetParkingSpaces(
    const PointENU& point, double distance,
    std::vector<ParkingSpaceInfoConstPtr>* parking_spaces) const {
  return GetParkingSpaces({point.x(), point.y()}, distance, parking_spaces);
}

int HDMapImpl::GetParkingSpaces(
    const Vec2d& point, double distance,
    std::vector<ParkingSpaceInfoConstPtr>* parking_spaces) const {
  if (parking_spaces == nullptr || parking_space_polygon_kdtree_ == nullptr) {
    return -1;
  }
  parking_spaces->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *parking_space_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto& id : ids) {
    parking_spaces->emplace_back(GetParkingSpaceById(CreateHDMapId(id)));
  }

  return 0;
}

int HDMapImpl::GetParkingLots(
    const PointENU& point, double distance,
    std::vector<ParkingLotInfoConstPtr>* parking_lots) const {
  return GetParkingLots({point.x(), point.y()}, distance, parking_lots);
}

int HDMapImpl::GetParkingLots(
    const Vec2d& point, double distance,
    std::vector<ParkingLotInfoConstPtr>* parking_lots) const {
  if (parking_lots == nullptr || parking_lot_polygon_kdtree_ == nullptr) {
    return -1;
  }
  parking_lots->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *parking_lot_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto& id : ids) {
    parking_lots->emplace_back(GetParkingLotById(CreateHDMapId(id)));
  }

  return 0;
}

int HDMapImpl::GetPNCJunctions(
    const century::common::PointENU& point, double distance,
    std::vector<PNCJunctionInfoConstPtr>* pnc_junctions) const {
  return GetPNCJunctions({point.x(), point.y()}, distance, pnc_junctions);
}

int HDMapImpl::GetPNCJunctions(
    const century::common::math::Vec2d& point, double distance,
    std::vector<PNCJunctionInfoConstPtr>* pnc_junctions) const {
  if (pnc_junctions == nullptr || pnc_junction_polygon_kdtree_ == nullptr) {
    return -1;
  }
  pnc_junctions->clear();

  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *pnc_junction_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }

  for (const auto& id : ids) {
    pnc_junctions->emplace_back(GetPNCJunctionById(CreateHDMapId(id)));
  }

  return 0;
}

int HDMapImpl::GetNearestLane(const PointENU& point,
                              LaneInfoConstPtr* nearest_lane, double* nearest_s,
                              double* nearest_l) const {
  return GetNearestLane({point.x(), point.y()}, nearest_lane, nearest_s,
                        nearest_l);
}

int HDMapImpl::GetNearestLane(const Vec2d& point,
                              LaneInfoConstPtr* nearest_lane, double* nearest_s,
                              double* nearest_l) const {
  CHECK_NOTNULL(nearest_lane);
  CHECK_NOTNULL(nearest_s);
  CHECK_NOTNULL(nearest_l);
  const auto* segment_object = lane_segment_kdtree_->GetNearestObject(point);
  if (segment_object == nullptr) {
    return -1;
  }
  const Id& lane_id = segment_object->object()->id();
  *nearest_lane = GetLaneById(lane_id);
  ACHECK(*nearest_lane);
  const int id = segment_object->id();
  const auto& segment = (*nearest_lane)->segments()[id];
  Vec2d nearest_pt;
  segment.DistanceTo(point, &nearest_pt);
  *nearest_s = (*nearest_lane)->accumulate_s()[id] +
               nearest_pt.DistanceTo(segment.start());
  *nearest_l = segment.unit_direction().CrossProd(point - segment.start());

  return 0;
}

int HDMapImpl::GetNearestLaneWithHeading(
    const PointENU& point, const double distance, const double central_heading,
    const double max_heading_difference, LaneInfoConstPtr* nearest_lane,
    double* nearest_s, double* nearest_l) const {
  return GetNearestLaneWithHeading({point.x(), point.y()}, distance,
                                   central_heading, max_heading_difference,
                                   nearest_lane, nearest_s, nearest_l);
}

int HDMapImpl::GetNearestLaneWithHeading(
    const Vec2d& point, const double distance, const double central_heading,
    const double max_heading_difference, LaneInfoConstPtr* nearest_lane,
    double* nearest_s, double* nearest_l) const {
  CHECK_NOTNULL(nearest_lane);
  CHECK_NOTNULL(nearest_s);
  CHECK_NOTNULL(nearest_l);

  std::vector<LaneInfoConstPtr> lanes;
  if (GetLanesWithHeading(point, distance, central_heading,
                          max_heading_difference, &lanes) != 0) {
    return -1;
  }

  double s = 0;
  size_t s_index = 0;
  Vec2d map_point;
  double min_distance = distance;
  for (const auto& lane : lanes) {
    double s_offset = 0.0;
    int s_offset_index = 0;
    double distance =
        lane->DistanceTo(point, &map_point, &s_offset, &s_offset_index);
    if (distance < min_distance) {
      min_distance = distance;
      *nearest_lane = lane;
      s = s_offset;
      s_index = s_offset_index;
    }
  }

  if (*nearest_lane == nullptr) {
    return -1;
  }

  *nearest_s = s;
  int segment_index = static_cast<int>(
      std::min(s_index, (*nearest_lane)->segments().size() - 1));
  const auto& segment_2d = (*nearest_lane)->segments()[segment_index];
  *nearest_l =
      segment_2d.unit_direction().CrossProd(point - segment_2d.start());

  return 0;
}

int HDMapImpl::GetLanesWithHeading(const PointENU& point, const double distance,
                                   const double central_heading,
                                   const double max_heading_difference,
                                   std::vector<LaneInfoConstPtr>* lanes) const {
  return GetLanesWithHeading({point.x(), point.y()}, distance, central_heading,
                             max_heading_difference, lanes);
}

int HDMapImpl::GetLanesWithHeading(const Vec2d& point, const double distance,
                                   const double central_heading,
                                   const double max_heading_difference,
                                   std::vector<LaneInfoConstPtr>* lanes) const {
  CHECK_NOTNULL(lanes);
  std::vector<LaneInfoConstPtr> all_lanes;
  const int status = GetLanes(point, distance, &all_lanes);
  if (status < 0 || all_lanes.empty()) {
    return -1;
  }

  lanes->clear();
  for (auto& lane : all_lanes) {
    Vec2d proj_pt(0.0, 0.0);
    double s_offset = 0.0;
    int s_offset_index = 0;
    double dis = lane->DistanceTo(point, &proj_pt, &s_offset, &s_offset_index);
    if (dis <= distance) {
      double heading_diff =
          fabs(lane->headings()[s_offset_index] - central_heading);
      if (fabs(century::common::math::NormalizeAngle(heading_diff)) <=
          max_heading_difference) {
        lanes->push_back(lane);
      }
    }
  }

  return 0;
}

int HDMapImpl::GetRoadBoundaries(
    const PointENU& point, double radius,
    std::vector<RoadROIBoundaryPtr>* road_boundaries,
    std::vector<JunctionBoundaryPtr>* junctions) const {
  CHECK_NOTNULL(road_boundaries);
  CHECK_NOTNULL(junctions);

  road_boundaries->clear();
  junctions->clear();

  std::vector<LaneInfoConstPtr> lanes;
  if (GetLanes(point, radius, &lanes) != 0 || lanes.empty()) {
    return -1;
  }

  std::unordered_set<std::string> junction_id_set;
  std::unordered_set<std::string> road_section_id_set;
  for (const auto& lane : lanes) {
    const auto road_id = lane->road_id();
    const auto section_id = lane->section_id();
    std::string unique_id = road_id.id() + section_id.id();
    if (road_section_id_set.count(unique_id) > 0) {
      continue;
    }
    road_section_id_set.insert(unique_id);
    const auto road_ptr = GetRoadById(road_id);
    if (road_ptr == nullptr) {
      AERROR << "road id [" << road_id.id() << "] is not found.";
      continue;
    }
    if (road_ptr->has_junction_id()) {
      const Id junction_id = road_ptr->junction_id();
      if (junction_id_set.count(junction_id.id()) > 0) {
        continue;
      }
      junction_id_set.insert(junction_id.id());
      JunctionBoundaryPtr junction_boundary_ptr(new JunctionBoundary());
      junction_boundary_ptr->junction_info = GetJunctionById(junction_id);
      if (junction_boundary_ptr->junction_info == nullptr) {
        AERROR << "junction id [" << junction_id.id() << "] is not found.";
        continue;
      }
      junctions->push_back(junction_boundary_ptr);
    } else {
      RoadROIBoundaryPtr road_boundary_ptr(new RoadROIBoundary());
      road_boundary_ptr->mutable_id()->CopyFrom(road_ptr->id());
      for (const auto& section : road_ptr->sections()) {
        if (section.id().id() == section_id.id()) {
          road_boundary_ptr->add_road_boundaries()->CopyFrom(
              section.boundary());
        }
      }
      road_boundaries->push_back(road_boundary_ptr);
    }
  }

  return 0;
}

int HDMapImpl::GetRoadBoundaries(
    const PointENU& point, double radius,
    std::vector<RoadRoiPtr>* road_boundaries,
    std::vector<JunctionInfoConstPtr>* junctions) const {
  if (road_boundaries == nullptr || junctions == nullptr) {
    AERROR << "the pointer in parameter is null";
    return -1;
  }
  road_boundaries->clear();
  junctions->clear();
  std::vector<RoadInfoConstPtr> roads;
  if (GetRoads(point, radius, &roads) != 0) {
    AERROR << "can not get roads in the range.";
    return -1;
  }

  CreateRoadBoundary(roads, road_boundaries, junctions);

  return 0;
}

void HDMapImpl::CreateRoadBoundary(
    const std::vector<RoadInfoConstPtr>& roads,
    std::vector<RoadRoiPtr>* road_boundaries,
    std::vector<JunctionInfoConstPtr>* junctions) const {
  std::set<std::string> junction_id_set;

  for (const auto& road_ptr : roads) {
    if (road_ptr->has_junction_id()) {
      JunctionInfoConstPtr junction_ptr =
          GetJunctionById(road_ptr->junction_id());
      if (junction_id_set.find(junction_ptr->id().id()) ==
          junction_id_set.end()) {
        junctions->push_back(junction_ptr);
        junction_id_set.insert(junction_ptr->id().id());
      }
    } else {
      RoadRoiPtr road_boundary_ptr(new RoadRoi());
      const std::vector<century::hdmap::RoadBoundary>& temp_road_boundaries =
          road_ptr->GetBoundaries();
      road_boundary_ptr->id = road_ptr->id();
      for (const auto& temp_road_boundary : temp_road_boundaries) {
        century::hdmap::BoundaryPolygon boundary_polygon =
            temp_road_boundary.outer_polygon();
        for (const auto& edge : boundary_polygon.edge()) {
          if (edge.type() == century::hdmap::BoundaryEdge::LEFT_BOUNDARY) {
            for (const auto& s : edge.curve().segment()) {
              for (const auto& p : s.line_segment().point()) {
                road_boundary_ptr->left_boundary.line_points.push_back(p);
              }
            }
          }
          if (edge.type() == century::hdmap::BoundaryEdge::RIGHT_BOUNDARY) {
            for (const auto& s : edge.curve().segment()) {
              for (const auto& p : s.line_segment().point()) {
                road_boundary_ptr->right_boundary.line_points.push_back(p);
              }
            }
          }
        }
        if (temp_road_boundary.hole_size() != 0) {
          for (const auto& hole : temp_road_boundary.hole()) {
            PolygonBoundary hole_boundary;
            for (const auto& edge : hole.edge()) {
              if (edge.type() == century::hdmap::BoundaryEdge::NORMAL) {
                for (const auto& s : edge.curve().segment()) {
                  for (const auto& p : s.line_segment().point()) {
                    hole_boundary.polygon_points.push_back(p);
                  }
                }
              }
            }
            road_boundary_ptr->holes_boundary.push_back(hole_boundary);
          }
        }
      }
      road_boundaries->push_back(road_boundary_ptr);
    }
  }
}

int HDMapImpl::GetRoadBoundaries(
    const std::vector<Id>& road_ids, std::vector<RoadRoiPtr>* road_boundaries,
    std::vector<JunctionInfoConstPtr>* junctions) const {
  if (road_boundaries == nullptr || junctions == nullptr) {
    AERROR << "the pointer in parameter is null";
    return -1;
  }
  road_boundaries->clear();
  junctions->clear();
  std::vector<RoadInfoConstPtr> roads;
  for (auto& id : road_ids) {
    const auto road_ptr = GetRoadById(id);
    if (nullptr != road_ptr) roads.emplace_back(road_ptr);
  }
  CreateRoadBoundary(roads, road_boundaries, junctions);
  return 0;
}

int HDMapImpl::GetRoadBoundaries(
    const PointENU& point, double radius, const std::vector<Id>& ref_ids,
    std::vector<RoadRoiPtr>* road_boundaries,
    std::vector<JunctionInfoConstPtr>* junctions) const {
  if (road_boundaries == nullptr || junctions == nullptr) {
    AERROR << "the pointer in parameter is null";
    return -1;
  }
  AINFO << "ref_ids size " << ref_ids.size();
  road_boundaries->clear();
  junctions->clear();
  std::set<std::string> junction_id_set;
  std::vector<RoadInfoConstPtr> roads;
  if (GetRoads(point, radius, &roads) != 0) {
    AERROR << "can not get roads in the range.";
    return -1;
  }
  CreateRoadBoundary(ref_ids, roads, road_boundaries, junctions);
  return 0;
}

bool HDMapImpl::IsInRodArea(const century::common::PointENU& point) const {
  std::vector<ObstacleInfoConstPtr> obstacles;
  GetObstacleAreas(point, kRegionalSearchRadius, &obstacles);
  for (auto obstacle : obstacles) {
    if (obstacle->polygon().IsPointIn({point.x(), point.y()})) {
      AINFO << "In Rod area points:" << point.x() << "," << point.y();
      return true;
    }
  }
  return false;
}

bool HDMapImpl::Check(RoadInfoConstPtr road_ptr,
                      const std::vector<Id>& ref_ids) const {
  bool is_in_ref = false;
  for (const auto& section : road_ptr->sections()) {
    const auto& road_lane_ids = section.lane_id();
    for (const auto& lane_id : road_lane_ids) {
      for (const auto& ref_lane_id : ref_ids) {
        if (ref_lane_id.id() == lane_id.id()) {
          is_in_ref = true;
          break;
        }
      }
    }
    if (is_in_ref) {
      break;
    }
  }

  return is_in_ref;
}

void HDMapImpl::CreateRoadBoundary(
    const std::vector<Id>& ref_ids, const std::vector<RoadInfoConstPtr>& roads,
    std::vector<RoadRoiPtr>* road_boundaries,
    std::vector<JunctionInfoConstPtr>* junctions) const {
  std::set<std::string> junction_id_set;

  for (const auto& road_ptr : roads) {
    if (road_ptr->has_junction_id()) {
      JunctionInfoConstPtr junction_ptr =
          GetJunctionById(road_ptr->junction_id());
      if (junction_id_set.find(junction_ptr->id().id()) ==
          junction_id_set.end()) {
        junctions->push_back(junction_ptr);
        junction_id_set.insert(junction_ptr->id().id());
      }
    } else {
      if (!Check(road_ptr, ref_ids)) {
        continue;
      }
      RoadRoiPtr road_boundary_ptr(new RoadRoi());
      const std::vector<century::hdmap::RoadBoundary>& temp_road_boundaries =
          road_ptr->GetBoundaries();
      road_boundary_ptr->id = road_ptr->id();
      for (const auto& temp_road_boundary : temp_road_boundaries) {
        century::hdmap::BoundaryPolygon boundary_polygon =
            temp_road_boundary.outer_polygon();
        for (const auto& edge : boundary_polygon.edge()) {
          if (edge.type() == century::hdmap::BoundaryEdge::LEFT_BOUNDARY) {
            for (const auto& s : edge.curve().segment()) {
              for (const auto& p : s.line_segment().point()) {
                road_boundary_ptr->left_boundary.line_points.push_back(p);
              }
            }
          }
          if (edge.type() == century::hdmap::BoundaryEdge::RIGHT_BOUNDARY) {
            for (const auto& s : edge.curve().segment()) {
              for (const auto& p : s.line_segment().point()) {
                road_boundary_ptr->right_boundary.line_points.push_back(p);
              }
            }
          }
        }
        if (temp_road_boundary.hole_size() != 0) {
          for (const auto& hole : temp_road_boundary.hole()) {
            PolygonBoundary hole_boundary;
            for (const auto& edge : hole.edge()) {
              if (edge.type() == century::hdmap::BoundaryEdge::NORMAL) {
                for (const auto& s : edge.curve().segment()) {
                  for (const auto& p : s.line_segment().point()) {
                    hole_boundary.polygon_points.push_back(p);
                  }
                }
              }
            }
            road_boundary_ptr->holes_boundary.push_back(hole_boundary);
          }
        }
      }
      road_boundaries->push_back(road_boundary_ptr);
    }
  }
}

void HDMapImpl::ExtractRoadAndJunctionRois(
    const std::vector<RoadInfoConstPtr>& roads,
    std::vector<PolygonRoiPtr>* polygons_roi,
    std::set<std::string>* polygon_id_set, std::vector<RoadRoiPtr>* roads_roi) {
  for (const auto& road_ptr : roads) {
    // get junction polygon
    if (road_ptr->has_junction_id()) {
      JunctionInfoConstPtr junction_ptr =
          GetJunctionById(road_ptr->junction_id());
      if (polygon_id_set->find(junction_ptr->id().id()) ==
          polygon_id_set->end()) {
        PolygonRoiPtr polygon_roi_ptr(new PolygonRoi());
        polygon_roi_ptr->polygon = junction_ptr->polygon();
        polygon_roi_ptr->attribute.type = PolygonType::JUNCTION_POLYGON;
        polygon_roi_ptr->attribute.id = junction_ptr->id();
        polygons_roi->push_back(polygon_roi_ptr);
        polygon_id_set->insert(junction_ptr->id().id());
      }
    } else {
      // get road boundary
      RoadRoiPtr road_boundary_ptr(new RoadRoi());
      std::vector<century::hdmap::RoadBoundary> temp_roads_roi;
      temp_roads_roi = road_ptr->GetBoundaries();
      if (!temp_roads_roi.empty()) {
        road_boundary_ptr->id = road_ptr->id();
        for (const auto& temp_road_boundary : temp_roads_roi) {
          century::hdmap::BoundaryPolygon boundary_polygon =
              temp_road_boundary.outer_polygon();
          for (const auto& edge : boundary_polygon.edge()) {
            if (edge.type() == century::hdmap::BoundaryEdge::LEFT_BOUNDARY) {
              for (const auto& s : edge.curve().segment()) {
                for (const auto& p : s.line_segment().point()) {
                  road_boundary_ptr->left_boundary.line_points.push_back(p);
                }
              }
            }
            if (edge.type() == century::hdmap::BoundaryEdge::RIGHT_BOUNDARY) {
              for (const auto& s : edge.curve().segment()) {
                for (const auto& p : s.line_segment().point()) {
                  road_boundary_ptr->right_boundary.line_points.push_back(p);
                }
              }
            }
          }
          if (temp_road_boundary.hole_size() != 0) {
            for (const auto& hole : temp_road_boundary.hole()) {
              PolygonBoundary hole_boundary;
              for (const auto& edge : hole.edge()) {
                if (edge.type() == century::hdmap::BoundaryEdge::NORMAL) {
                  for (const auto& s : edge.curve().segment()) {
                    for (const auto& p : s.line_segment().point()) {
                      hole_boundary.polygon_points.push_back(p);
                    }
                  }
                }
              }
              road_boundary_ptr->holes_boundary.push_back(hole_boundary);
            }
          }
        }
        roads_roi->push_back(road_boundary_ptr);
      }
    }
  }
}

void HDMapImpl::ExtractParkingSpacePolygons(
    const std::vector<LaneInfoConstPtr>& lanes,
    std::vector<PolygonRoiPtr>* polygons_roi,
    std::set<std::string>* polygon_id_set) {
  for (const auto& lane_ptr : lanes) {
    // get parking space polygon
    for (const auto& overlap_id : lane_ptr->lane().overlap_id()) {
      OverlapInfoConstPtr overlap_ptr = GetOverlapById(overlap_id);
      for (int i = 0; i < overlap_ptr->overlap().object_size(); ++i) {
        if (overlap_ptr->overlap().object(i).id().id() == lane_ptr->id().id()) {
          continue;
        } else {
          ParkingSpaceInfoConstPtr parkingspace_ptr =
              GetParkingSpaceById(overlap_ptr->overlap().object(i).id());
          if (parkingspace_ptr != nullptr) {
            if (polygon_id_set->find(parkingspace_ptr->id().id()) ==
                polygon_id_set->end()) {
              PolygonRoiPtr polygon_roi_ptr(new PolygonRoi());
              polygon_roi_ptr->polygon = parkingspace_ptr->polygon();
              polygon_roi_ptr->attribute.type =
                  PolygonType::PARKINGSPACE_POLYGON;
              polygon_roi_ptr->attribute.id = parkingspace_ptr->id();
              polygons_roi->push_back(polygon_roi_ptr);
              polygon_id_set->insert(parkingspace_ptr->id().id());
            }
          }
        }
      }
    }
  }
}

int HDMapImpl::GetRoi(const century::common::PointENU& point, double radius,
                      std::vector<RoadRoiPtr>* roads_roi,
                      std::vector<PolygonRoiPtr>* polygons_roi) {
  if (roads_roi == nullptr || polygons_roi == nullptr) {
    AERROR << "the pointer in parameter is null";
    return -1;
  }
  roads_roi->clear();
  polygons_roi->clear();
  std::set<std::string> polygon_id_set;
  std::vector<RoadInfoConstPtr> roads;
  std::vector<LaneInfoConstPtr> lanes;
  if (GetRoads(point, radius, &roads) != 0) {
    AERROR << "can not get roads in the range.";
    return -1;
  }
  if (GetLanes(point, radius, &lanes) != 0) {
    AERROR << "can not get lanes in the range.";
    return -1;
  }
  ExtractRoadAndJunctionRois(roads, polygons_roi, &polygon_id_set, roads_roi);
  ExtractParkingSpacePolygons(lanes, polygons_roi, &polygon_id_set);
  return 0;
}

int HDMapImpl::FindSurroundingLanes(
    const century::common::PointENU& point,
    const century::common::math::Vec2d& car_point,
    std::vector<LaneInfoConstPtr>* surrounding_lanes) const {
  std::vector<LaneInfoConstPtr> temp_surrounding_lanes;
  if (GetLanes(point, kLanesSearchRange, &temp_surrounding_lanes) == -1) {
    AINFO << "Can not find lanes around car.";
    return -1;
  }
  for (const auto& surround_lane : temp_surrounding_lanes) {
    if (surround_lane->IsOnLane(car_point)) {
      surrounding_lanes->push_back(surround_lane);
    }
  }
  if (surrounding_lanes->empty()) {
    AINFO << "Car is not on lane.";
    return -1;
  }
  return 0;
}
LaneInfoConstPtr HDMapImpl::DetermineStartLane(
    const std::vector<LaneInfoConstPtr>& surrounding_lanes,
    const century::common::math::Vec2d& car_point, double* nearest_s,
    double* nearest_l) const {
  int s_index = 0;
  century::common::math::Vec2d map_point;

  LaneInfoConstPtr lane_ptr = nullptr;
  for (const auto& lane : surrounding_lanes) {
    if (!lane->signals().empty()) {
      lane_ptr = lane;
      *nearest_l =
          lane_ptr->DistanceTo(car_point, &map_point, nearest_s, &s_index);
      break;
    }
  }
  return lane_ptr;
}

void HDMapImpl::FindSignalsOnLane(
    LaneInfoConstPtr lane_ptr, double s_start, double unused_distance,
    std::vector<SignalInfoConstPtr>* signals) const {
  while (lane_ptr != nullptr) {
    double signal_min_dist = std::numeric_limits<double>::infinity();
    std::vector<SignalInfoConstPtr> min_dist_signal_ptr;
    for (const auto& overlap_id : lane_ptr->lane().overlap_id()) {
      OverlapInfoConstPtr overlap_ptr = GetOverlapById(overlap_id);
      double lane_overlap_offset_s = 0.0;
      SignalInfoConstPtr signal_ptr = nullptr;
      for (int i = 0; i < overlap_ptr->overlap().object_size(); ++i) {
        if (overlap_ptr->overlap().object(i).id().id() == lane_ptr->id().id()) {
          lane_overlap_offset_s =
              overlap_ptr->overlap().object(i).lane_overlap_info().start_s() -
              s_start;
          continue;
        }
        signal_ptr = GetSignalById(overlap_ptr->overlap().object(i).id());
        if (signal_ptr == nullptr || lane_overlap_offset_s < 0.0) {
          break;
        }
        if (lane_overlap_offset_s < signal_min_dist) {
          signal_min_dist = lane_overlap_offset_s;
          min_dist_signal_ptr.clear();
          min_dist_signal_ptr.push_back(signal_ptr);
        } else if (lane_overlap_offset_s < (signal_min_dist + 0.1) &&
                   lane_overlap_offset_s > (signal_min_dist - 0.1)) {
          min_dist_signal_ptr.push_back(signal_ptr);
        }
      }
    }
    if (!min_dist_signal_ptr.empty() && unused_distance >= signal_min_dist) {
      *signals = min_dist_signal_ptr;
      break;
    }
    unused_distance = unused_distance - (lane_ptr->total_length() - s_start);
    if (unused_distance <= 0) {
      break;
    }
    LaneInfoConstPtr tmp_lane_ptr = nullptr;
    for (const auto& successor_lane_id : lane_ptr->lane().successor_id()) {
      tmp_lane_ptr = GetLaneById(successor_lane_id);
      if (tmp_lane_ptr->lane().turn() == century::hdmap::Lane::NO_TURN) {
        break;
      }
    }
    lane_ptr = tmp_lane_ptr;
    s_start = 0;
  }
}

int HDMapImpl::GetForwardNearestSignalsOnLane(
    const century::common::PointENU& point, const double distance,
    std::vector<SignalInfoConstPtr>* signals) const {
  CHECK_NOTNULL(signals);

  signals->clear();
  // LaneInfoConstPtr lane_ptr = nullptr;
  double nearest_s = 0.0;
  double nearest_l = 0.0;

  std::vector<LaneInfoConstPtr> surrounding_lanes;
  century::common::math::Vec2d car_point;
  car_point.set_x(point.x());
  car_point.set_y(point.y());

  if (0 != FindSurroundingLanes(point, car_point, &surrounding_lanes)) {
    return -1;
  }

  auto lane_ptr =
      DetermineStartLane(surrounding_lanes, car_point, &nearest_s, &nearest_l);

  if (lane_ptr == nullptr) {
    GetNearestLane(point, &lane_ptr, &nearest_s, &nearest_l);
    if (lane_ptr == nullptr) {
      return -1;
    }
  }
  double unused_distance = distance + kBackwardDistance;
  double s_start = nearest_s;
  double s = nearest_s;
  double back_distance = kBackwardDistance;
  while (s < back_distance) {
    for (const auto& predecessor_lane_id : lane_ptr->lane().predecessor_id()) {
      lane_ptr = GetLaneById(predecessor_lane_id);
      if (lane_ptr->lane().turn() == century::hdmap::Lane::NO_TURN) {
        break;
      }
    }
    back_distance = back_distance - s;
    s = lane_ptr->total_length();
  }
  s_start = s - back_distance;
  FindSignalsOnLane(lane_ptr, s_start, unused_distance, signals);
  return 0;
}

int HDMapImpl::GetStopSignAssociatedStopSigns(
    const Id& id, std::vector<StopSignInfoConstPtr>* stop_signs) const {
  CHECK_NOTNULL(stop_signs);

  const auto& stop_sign = GetStopSignById(id);
  if (stop_sign == nullptr) {
    return -1;
  }

  std::vector<Id> associate_stop_sign_ids;
  const auto junction_ids = stop_sign->OverlapJunctionIds();
  for (const auto& junction_id : junction_ids) {
    const auto& junction = GetJunctionById(junction_id);
    if (junction == nullptr) {
      continue;
    }
    const auto stop_sign_ids = junction->OverlapStopSignIds();
    std::copy(stop_sign_ids.begin(), stop_sign_ids.end(),
              std::back_inserter(associate_stop_sign_ids));
  }

  std::vector<Id> associate_lane_ids;
  for (const auto& stop_sign_id : associate_stop_sign_ids) {
    if (stop_sign_id.id() == id.id()) {
      // exclude current stop sign
      continue;
    }
    const auto& stop_sign = GetStopSignById(stop_sign_id);
    if (stop_sign == nullptr) {
      continue;
    }
    stop_signs->push_back(stop_sign);
  }

  return 0;
}

int HDMapImpl::GetStopSignAssociatedLanes(
    const Id& id, std::vector<LaneInfoConstPtr>* lanes) const {
  CHECK_NOTNULL(lanes);

  const auto& stop_sign = GetStopSignById(id);
  if (stop_sign == nullptr) {
    return -1;
  }

  std::vector<Id> associate_stop_sign_ids;
  const auto junction_ids = stop_sign->OverlapJunctionIds();
  for (const auto& junction_id : junction_ids) {
    const auto& junction = GetJunctionById(junction_id);
    if (junction == nullptr) {
      continue;
    }
    const auto stop_sign_ids = junction->OverlapStopSignIds();
    std::copy(stop_sign_ids.begin(), stop_sign_ids.end(),
              std::back_inserter(associate_stop_sign_ids));
  }

  std::vector<Id> associate_lane_ids;
  for (const auto& stop_sign_id : associate_stop_sign_ids) {
    if (stop_sign_id.id() == id.id()) {
      // exclude current stop sign
      continue;
    }
    const auto& stop_sign = GetStopSignById(stop_sign_id);
    if (stop_sign == nullptr) {
      continue;
    }
    const auto lane_ids = stop_sign->OverlapLaneIds();
    std::copy(lane_ids.begin(), lane_ids.end(),
              std::back_inserter(associate_lane_ids));
  }

  for (const auto lane_id : associate_lane_ids) {
    const auto& lane = GetLaneById(lane_id);
    if (lane == nullptr) {
      continue;
    }
    lanes->push_back(lane);
  }

  return 0;
}

int HDMapImpl::GetLocalMap(const century::common::PointENU& point,
                           const std::pair<double, double>& range,
                           Map* local_map) const {
  CHECK_NOTNULL(local_map);

  double distance = std::max(range.first, range.second);
  CHECK_GT(distance, 0.0);

#define GETITEMS(type, func, items) \
  std::vector<type> items;          \
  func(point, distance, &items);

  GETITEMS(LaneInfoConstPtr, GetLanes, lanes);
  GETITEMS(JunctionInfoConstPtr, GetJunctions, junctions);
  GETITEMS(CrosswalkInfoConstPtr, GetCrosswalks, crosswalks);
  GETITEMS(SignalInfoConstPtr, GetSignals, signals);
  GETITEMS(StopSignInfoConstPtr, GetStopSigns, stop_signs);
  GETITEMS(YieldSignInfoConstPtr, GetYieldSigns, yield_signs);
  GETITEMS(ClearAreaInfoConstPtr, GetClearAreas, clear_areas);
  GETITEMS(SpeedBumpInfoConstPtr, GetSpeedBumps, speed_bumps);
  GETITEMS(RoadInfoConstPtr, GetRoads, roads);
  GETITEMS(ParkingSpaceInfoConstPtr, GetParkingSpaces, parking_spaces);
  std::unordered_set<std::string> map_element_ids;
  std::vector<Id> overlap_ids;

#define ADDITEMS(items, add_func, map_add_item)              \
  for (auto& item_ptr : items) {                             \
    map_element_ids.insert(item_ptr->id().id());             \
    std::copy(item_ptr->add_func().overlap_id().begin(),     \
              item_ptr->add_func().overlap_id().end(),       \
              std::back_inserter(overlap_ids));              \
    *local_map->add_##map_add_item() = item_ptr->add_func(); \
  }

  ADDITEMS(lanes, lane, lane);
  ADDITEMS(crosswalks, crosswalk, crosswalk);
  ADDITEMS(junctions, junction, junction);
  ADDITEMS(signals, signal, signal);
  ADDITEMS(stop_signs, stop_sign, stop_sign);
  ADDITEMS(yield_signs, yield_sign, yield);
  ADDITEMS(clear_areas, clear_area, clear_area);
  ADDITEMS(speed_bumps, speed_bump, speed_bump);
  for (auto& road_ptr : roads) {
    map_element_ids.insert(road_ptr->id().id());
    *local_map->add_road() = road_ptr->road();
  }
  ADDITEMS(parking_spaces, parking_space, parking_space);

  for (auto& overlap_id : overlap_ids) {
    auto overlap_ptr = GetOverlapById(overlap_id);
    if (overlap_ptr == nullptr) {
      AERROR << "overlpa id [" << overlap_id.id() << "] is not found.";
      continue;
    }

    bool need_delete = false;
    for (auto& overlap_object : overlap_ptr->overlap().object()) {
      if (map_element_ids.count(overlap_object.id().id()) <= 0) {
        need_delete = true;
      }
    }

    if (!need_delete) {
      *local_map->add_overlap() = overlap_ptr->overlap();
    }
  }

  return 0;
}

int HDMapImpl::GetForwardNearestRSUs(const century::common::PointENU& point,
                                     double distance, double central_heading,
                                     double max_heading_difference,
                                     std::vector<RSUInfoConstPtr>* rsus) const {
  CHECK_NOTNULL(rsus);

  rsus->clear();
  LaneInfoConstPtr lane_ptr = nullptr;
  century::common::math::Vec2d target_point(point.x(), point.y());

  double nearest_s = 0.0;
  double nearest_l = 0.0;
  if (GetNearestLaneWithHeading(target_point, distance, central_heading,
                                max_heading_difference, &lane_ptr, &nearest_s,
                                &nearest_l) == -1) {
    AERROR << "Fail to get nearest lanes";
    return -1;
  }

  if (lane_ptr == nullptr) {
    AERROR << "Fail to get nearest lanes";
    return -1;
  }

  double s = 0;
  double real_distance = distance + nearest_s;
  const std::string nearst_lane_id = lane_ptr->id().id();

  while (s < real_distance) {
    s += lane_ptr->total_length();
    std::vector<std::pair<double, JunctionInfoConstPtr>> overlap_junctions;
    double start_s = 0;
    for (size_t x = 0; x < lane_ptr->junctions().size(); ++x) {
      const auto overlap_ptr = lane_ptr->junctions()[x];
      for (int i = 0; i < overlap_ptr->overlap().object_size(); ++i) {
        const auto& overlap_object = overlap_ptr->overlap().object(i);
        if (overlap_object.id().id() == lane_ptr->id().id()) {
          start_s = overlap_object.lane_overlap_info().start_s();
          continue;
        }

        const auto junction_ptr = GetJunctionById(overlap_object.id());
        CHECK_NOTNULL(junction_ptr);
        if (nearst_lane_id == lane_ptr->id().id() &&
            !junction_ptr->polygon().IsPointIn(target_point)) {
          if (nearest_s > start_s) {
            continue;
          }
        }

        overlap_junctions.push_back(std::make_pair(start_s, junction_ptr));
      }
    }
    std::sort(overlap_junctions.begin(), overlap_junctions.end());
    if (!DuplicateChecker(overlap_junctions, rsus)) {
      return -1;
    }

    for (const auto suc_lane_id : lane_ptr->lane().successor_id()) {
      LaneInfoConstPtr suc_lane_ptr = GetLaneById(suc_lane_id);
      if (lane_ptr->lane().successor_id_size() > 1) {
        if (suc_lane_ptr->lane().turn() == century::hdmap::Lane::NO_TURN) {
          lane_ptr = suc_lane_ptr;
          break;
        }
      } else {
        lane_ptr = suc_lane_ptr;
        break;
      }
    }
  }

  return 0;
}

bool HDMapImpl::DuplicateChecker(
    const std::vector<std::pair<double, JunctionInfoConstPtr>>&
        overlap_junctions,
    std::vector<RSUInfoConstPtr>* rsus) const {
  std::set<std::string> duplicate_checker;
  for (const auto& overlap_junction : overlap_junctions) {
    const auto& junction = overlap_junction.second;
    if (duplicate_checker.count(junction->id().id()) > 0) {
      continue;
    }
    duplicate_checker.insert(junction->id().id());

    for (const auto& overlap_id : junction->junction().overlap_id()) {
      OverlapInfoConstPtr overlap_ptr = GetOverlapById(overlap_id);
      CHECK_NOTNULL(overlap_ptr);
      for (int i = 0; i < overlap_ptr->overlap().object_size(); ++i) {
        const auto& overlap_object = overlap_ptr->overlap().object(i);
        if (!overlap_object.has_rsu_overlap_info()) {
          continue;
        }

        const auto rsu_ptr = GetRSUById(overlap_object.id());
        if (rsu_ptr != nullptr) {
          rsus->push_back(rsu_ptr);
        }
      }
    }

    if (!rsus->empty()) {
      return false;
    }
  }

  if (!rsus->empty()) {
    return false;
  }
  return true;
}

template <class Table, class BoxTable, class KDTree>
void HDMapImpl::BuildSegmentKDTree(const Table& table,
                                   const AABoxKDTreeParams& params,
                                   BoxTable* const box_table,
                                   std::unique_ptr<KDTree>* const kdtree) {
  box_table->clear();
  for (const auto& info_with_id : table) {
    const auto* info = info_with_id.second.get();
    for (size_t id = 0; id < info->segments().size(); ++id) {
      const auto& segment = info->segments()[id];
      box_table->emplace_back(
          century::common::math::AABox2d(segment.start(), segment.end()), info,
          &segment, id);
    }
  }
  kdtree->reset(new KDTree(*box_table, params));
}

template <class Table, class BoxTable, class KDTree>
void HDMapImpl::BuildPolygonKDTree(const Table& table,
                                   const AABoxKDTreeParams& params,
                                   BoxTable* const box_table,
                                   std::unique_ptr<KDTree>* const kdtree) {
  box_table->clear();
  for (const auto& info_with_id : table) {
    const auto* info = info_with_id.second.get();
    const auto& polygon = info->polygon();
    box_table->emplace_back(polygon.AABoundingBox(), info, &polygon, 0);
  }
  kdtree->reset(new KDTree(*box_table, params));
}

void HDMapImpl::BuildLaneSegmentKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 16;
  BuildSegmentKDTree(lane_table_, params, &lane_segment_boxes_,
                     &lane_segment_kdtree_);
}

void HDMapImpl::BuildJunctionPolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 1;
  BuildPolygonKDTree(junction_table_, params, &junction_polygon_boxes_,
                     &junction_polygon_kdtree_);
}

void HDMapImpl::BuildCrosswalkPolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 1;
  BuildPolygonKDTree(crosswalk_table_, params, &crosswalk_polygon_boxes_,
                     &crosswalk_polygon_kdtree_);
}

void HDMapImpl::BuildSignalSegmentKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 4;
  BuildSegmentKDTree(signal_table_, params, &signal_segment_boxes_,
                     &signal_segment_kdtree_);
}

void HDMapImpl::BuildStopSignSegmentKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 4;
  BuildSegmentKDTree(stop_sign_table_, params, &stop_sign_segment_boxes_,
                     &stop_sign_segment_kdtree_);
}

void HDMapImpl::BuildYieldSignSegmentKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 4;
  BuildSegmentKDTree(yield_sign_table_, params, &yield_sign_segment_boxes_,
                     &yield_sign_segment_kdtree_);
}

void HDMapImpl::BuildClearAreaPolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 4;
  BuildPolygonKDTree(clear_area_table_, params, &clear_area_polygon_boxes_,
                     &clear_area_polygon_kdtree_);
}

void HDMapImpl::BuildSpeedBumpSegmentKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 4;
  BuildSegmentKDTree(speed_bump_table_, params, &speed_bump_segment_boxes_,
                     &speed_bump_segment_kdtree_);
}

void HDMapImpl::BuildParkingLotPolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;
  params.max_leaf_size = 4;
  BuildPolygonKDTree(parking_lot_table_, params, &parking_lot_polygon_boxes_,
                     &parking_lot_polygon_kdtree_);
}

void HDMapImpl::BuildParkingSpacePolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 4;
  BuildPolygonKDTree(parking_space_table_, params,
                     &parking_space_polygon_boxes_,
                     &parking_space_polygon_kdtree_);
}

void HDMapImpl::BuildPNCJunctionPolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 1;
  BuildPolygonKDTree(pnc_junction_table_, params, &pnc_junction_polygon_boxes_,
                     &pnc_junction_polygon_kdtree_);
}

void HDMapImpl::BuildObstaclePolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 1.0;  // meters.
  params.max_leaf_size = 1;
  BuildPolygonKDTree(obstacle_table_, params, &obstacle_polygon_boxes_,
                     &obstacle_polygon_kdtree_);
}

void HDMapImpl::BuildElectricFencePolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 1;
  BuildPolygonKDTree(electric_fence_table_, params,
                     &electric_fence_polygon_boxes_,
                     &electric_fence_polygon_kdtree_);
}

template <class KDTree>
int HDMapImpl::SearchObjects(const Vec2d& center, const double radius,
                             const KDTree& kdtree,
                             std::vector<std::string>* const results) {
  static std::mutex mutex_search_object;
  UNIQUE_LOCK_MULTITHREAD(mutex_search_object);
  if (nullptr == results) {
    AINFO << "hdmap_impl: results is empty!";
    return -1;
  }
  auto objects = kdtree.GetObjects(center, radius);
  std::unordered_set<std::string> result_ids;
  result_ids.reserve(objects.size());
  for (const auto* object_ptr : objects) {
    result_ids.insert(object_ptr->object()->id().id());
  }

  results->reserve(result_ids.size());
  results->assign(result_ids.begin(), result_ids.end());
  return 0;
}

void HDMapImpl::Clear() {
  map_.Clear();
  lane_table_.clear();
  junction_table_.clear();
  signal_table_.clear();
  crosswalk_table_.clear();
  stop_sign_table_.clear();
  yield_sign_table_.clear();
  parking_space_table_.clear();
  parking_lot_table_.clear();
  overlap_table_.clear();
  rsu_table_.clear();
  electric_fence_table_.clear();
  lane_segment_boxes_.clear();
  lane_segment_kdtree_.reset(nullptr);
  junction_polygon_boxes_.clear();
  junction_polygon_kdtree_.reset(nullptr);
  crosswalk_polygon_boxes_.clear();
  crosswalk_polygon_kdtree_.reset(nullptr);
  signal_segment_boxes_.clear();
  signal_segment_kdtree_.reset(nullptr);
  stop_sign_segment_boxes_.clear();
  stop_sign_segment_kdtree_.reset(nullptr);
  yield_sign_segment_boxes_.clear();
  yield_sign_segment_kdtree_.reset(nullptr);
  clear_area_polygon_boxes_.clear();
  clear_area_polygon_kdtree_.reset(nullptr);
  speed_bump_segment_boxes_.clear();
  speed_bump_segment_kdtree_.reset(nullptr);
  parking_space_polygon_boxes_.clear();
  parking_space_polygon_kdtree_.reset(nullptr);
  parking_lot_polygon_boxes_.clear();
  parking_lot_polygon_kdtree_.reset(nullptr);
  pnc_junction_polygon_boxes_.clear();
  pnc_junction_polygon_kdtree_.reset(nullptr);
  obstacle_polygon_boxes_.clear();
  obstacle_polygon_kdtree_.reset(nullptr);
  electric_fence_polygon_boxes_.clear();
  electric_fence_polygon_kdtree_.reset(nullptr);
}

int HDMapImpl::GetLanesWithNearPos(const PointENU& point, const double distance,
                                   std::vector<LaneInfoConstPtr>* lanes) const {
  return GetLanesWithNearPos({point.x(), point.y()}, distance, lanes);
}

int HDMapImpl::GetLanesWithNearPos(const Vec2d& point, const double distance,
                                   std::vector<LaneInfoConstPtr>* lanes) const {
  CHECK_NOTNULL(lanes);
  std::vector<LaneInfoConstPtr> all_lanes;
  const int status = GetLanes(point, distance, &all_lanes);
  if (status < 0 || all_lanes.empty()) {
    return -1;
  }

  lanes->clear();
  for (auto& lane : all_lanes) {
    Vec2d proj_pt(0.0, 0.0);
    double s_offset = 0.0;
    int s_offset_index = 0;
    double dis = lane->DistanceTo(point, &proj_pt, &s_offset, &s_offset_index);
    if (dis <= distance) {
      lanes->emplace_back(lane);
    }
  }

  return 0;
}

}  // namespace hdmap
}  // namespace century
