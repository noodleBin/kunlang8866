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
#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "cyber/common/log.h"
#include "modules/map/proto/map.pb.h"

namespace century {
namespace hdmap {
namespace adapter {

using PbHeader = century::hdmap::Header;
using PbRoad = century::hdmap::Road;
using PbRoadSection = century::hdmap::RoadSection;
using PbLane = century::hdmap::Lane;
using PbJunction = century::hdmap::Junction;
using PbSignal = century::hdmap::Signal;
using PbSubSignal = century::hdmap::Subsignal;
using PbRSU = century::hdmap::RSU;
using PbCrosswalk = century::hdmap::Crosswalk;
using PbParkingSpace = century::hdmap::ParkingSpace;
using PbSpeedBump = century::hdmap::SpeedBump;
using PbStopSign = century::hdmap::StopSign;
using PbYieldSign = century::hdmap::YieldSign;
using PbObjectOverlapInfo = century::hdmap::ObjectOverlapInfo;
using PbOverlap = century::hdmap::Overlap;
using PbClearArea = century::hdmap::ClearArea;
using PbLineSegment = century::hdmap::LineSegment;
using PbCurveSegment = century::hdmap::CurveSegment;
using PbCurve = century::hdmap::Curve;
using PbPoint3D = century::common::PointENU;
using PbLaneType = century::hdmap::Lane_LaneType;
using PbTurnType = century::hdmap::Lane_LaneTurn;
using PbID = century::hdmap::Id;
using PbLaneBoundary = century::hdmap::LaneBoundary;
using PbLaneBoundaryTypeType = century::hdmap::LaneBoundaryType_Type;
using PbPolygon = century::hdmap::Polygon;
using PbBoundaryPolygon = century::hdmap::BoundaryPolygon;
using PbBoundaryEdge = century::hdmap::BoundaryEdge;
using PbRegionOverlap = century::hdmap::RegionOverlapInfo;
using PbPNCJunction = century::hdmap::PNCJunction;

using PbLaneDirection = century::hdmap::Lane_LaneDirection;
using PbSignalType = century::hdmap::Signal_Type;
using PbSubSignalType = century::hdmap::Subsignal_Type;
using PbStopSignType = century::hdmap::StopSign_StopType;
using PbBoundaryEdgeType = century::hdmap::BoundaryEdge_Type;
using PbRoadType = century::hdmap::Road_Type;
using PbSignInfoType = century::hdmap::SignInfo::Type;
using PbPassageType = century::hdmap::Passage_Type;
using PbPassageGroup = century::hdmap::PassageGroup;

struct StopLineInternal {
  std::string id;
  PbCurve curve;
};

struct StopSignInternal {
  std::string id;
  PbStopSign stop_sign;
  std::unordered_set<std::string> stop_line_ids;
};

struct YieldSignInternal {
  std::string id;
  PbYieldSign yield_sign;
  std::unordered_set<std::string> stop_line_ids;
};

struct TrafficLightInternal {
  std::string id;
  PbSignal traffic_light;
  std::unordered_set<std::string> stop_line_ids;
};

struct OverlapWithLane {
  std::string object_id;
  double start_s;
  double end_s;
  bool is_merge;

  std::string region_overlap_id;
  std::vector<PbRegionOverlap> region_overlaps;
};

struct OverlapWithJunction {
  std::string object_id;
};

struct LaneInternal {
  PbLane lane;
  std::vector<OverlapWithLane> overlap_signals;
  std::vector<OverlapWithLane> overlap_objects;
  std::vector<OverlapWithLane> overlap_junctions;
  std::vector<OverlapWithLane> overlap_lanes;
};

struct JunctionInternal {
  PbJunction junction;
  std::unordered_set<std::string> road_ids;
  std::vector<OverlapWithJunction> overlap_with_junctions;
};

struct RoadSectionInternal {
  std::string id;
  PbRoadSection section;
  std::vector<LaneInternal> lanes;
};

struct RoadInternal {
  std::string id;
  PbRoad road;

  bool in_junction;
  std::string junction_id;

  std::string type;

  std::vector<RoadSectionInternal> sections;

  std::vector<TrafficLightInternal> traffic_lights;
  // std::vector<RSUInternal> rsus;
  std::vector<StopSignInternal> stop_signs;
  std::vector<YieldSignInternal> yield_signs;
  std::vector<PbCrosswalk> crosswalks;
  std::vector<PbClearArea> clear_areas;
  std::vector<PbSpeedBump> speed_bumps;
  std::vector<StopLineInternal> stop_lines;
  std::vector<PbParkingSpace> parking_spaces;
  std::vector<PbPNCJunction> pnc_junctions;

  RoadInternal() : in_junction(false) { junction_id = ""; }
};

struct RSUInternal {
  std::string id;
  PbRSU rsu;
};

struct ObjectInternal {
  std::vector<RSUInternal> rsus;
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace century
