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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/map/proto/map_clear_area.pb.h"
#include "modules/map/proto/map_crosswalk.pb.h"
#include "modules/map/proto/map_electric_fence.pb.h"
#include "modules/map/proto/map_id.pb.h"
#include "modules/map/proto/map_junction.pb.h"
#include "modules/map/proto/map_lane.pb.h"
#include "modules/map/proto/map_obstacle.pb.h"
#include "modules/map/proto/map_overlap.pb.h"
#include "modules/map/proto/map_parking_space.pb.h"
#include "modules/map/proto/map_pnc_junction.pb.h"
#include "modules/map/proto/map_road.pb.h"
#include "modules/map/proto/map_rsu.pb.h"
#include "modules/map/proto/map_signal.pb.h"
#include "modules/map/proto/map_speed_bump.pb.h"
#include "modules/map/proto/map_stop_sign.pb.h"
#include "modules/map/proto/map_yield_sign.pb.h"

#include "modules/common/math/aabox2d.h"
#include "modules/common/math/aaboxkdtree2d.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"

// Resolve compilation errors caused by duplicate names of signal slots in Qt
#undef signals

/**
 * @namespace century::hdmap
 * @brief century::hdmap
 */
namespace century {
namespace hdmap {

template <class Object, class GeoObject>
class ObjectWithAABox {
 public:
  ObjectWithAABox(const century::common::math::AABox2d &aabox,
                  const Object *object, const GeoObject *geo_object,
                  const int id)
      : aabox_(aabox), object_(object), geo_object_(geo_object), id_(id) {}
  ~ObjectWithAABox() {}
  const century::common::math::AABox2d &aabox() const { return aabox_; }
  double DistanceTo(const century::common::math::Vec2d &point) const {
    return geo_object_->DistanceTo(point);
  }
  double DistanceSquareTo(const century::common::math::Vec2d &point) const {
    return geo_object_->DistanceSquareTo(point);
  }
  const Object *object() const { return object_; }
  const GeoObject *geo_object() const { return geo_object_; }
  int id() const { return id_; }

 private:
  century::common::math::AABox2d aabox_;
  const Object *object_;
  const GeoObject *geo_object_;
  int id_;
};

class LaneInfo;
class JunctionInfo;
class CrosswalkInfo;
class SignalInfo;
class StopSignInfo;
class YieldSignInfo;
class OverlapInfo;
class ClearAreaInfo;
class SpeedBumpInfo;
class RoadInfo;
class ParkingSpaceInfo;
class ParkingLotInfo;
class PNCJunctionInfo;
class RSUInfo;
class ObstacleInfo;
class ElectricFenceInfo;
class HDMapImpl;

struct LineBoundary {
  std::vector<century::common::PointENU> line_points;
};
struct PolygonBoundary {
  std::vector<century::common::PointENU> polygon_points;
};

enum class PolygonType {
  JUNCTION_POLYGON = 0,
  PARKINGSPACE_POLYGON = 1,
  ROAD_HOLE_POLYGON = 2,
};

struct RoiAttribute {
  PolygonType type;
  Id id;
};

struct PolygonRoi {
  century::common::math::Polygon2d polygon;
  RoiAttribute attribute;
};

struct RoadRoi {
  Id id;
  LineBoundary left_boundary;
  LineBoundary right_boundary;
  std::vector<PolygonBoundary> holes_boundary;
};

using LaneSegmentBox =
    ObjectWithAABox<LaneInfo, century::common::math::LineSegment2d>;
using LaneSegmentKDTree = century::common::math::AABoxKDTree2d<LaneSegmentBox>;
using OverlapInfoConstPtr = std::shared_ptr<const OverlapInfo>;
using LaneInfoConstPtr = std::shared_ptr<const LaneInfo>;
using JunctionInfoConstPtr = std::shared_ptr<const JunctionInfo>;
using SignalInfoConstPtr = std::shared_ptr<const SignalInfo>;
using CrosswalkInfoConstPtr = std::shared_ptr<const CrosswalkInfo>;
using StopSignInfoConstPtr = std::shared_ptr<const StopSignInfo>;
using YieldSignInfoConstPtr = std::shared_ptr<const YieldSignInfo>;
using ClearAreaInfoConstPtr = std::shared_ptr<const ClearAreaInfo>;
using SpeedBumpInfoConstPtr = std::shared_ptr<const SpeedBumpInfo>;
using RoadInfoConstPtr = std::shared_ptr<const RoadInfo>;
using ParkingSpaceInfoConstPtr = std::shared_ptr<const ParkingSpaceInfo>;
using ParkingLotInfoConstPtr = std::shared_ptr<const ParkingLotInfo>;
using RoadROIBoundaryPtr = std::shared_ptr<RoadROIBoundary>;
using PolygonRoiPtr = std::shared_ptr<PolygonRoi>;
using RoadRoiPtr = std::shared_ptr<RoadRoi>;
using PNCJunctionInfoConstPtr = std::shared_ptr<const PNCJunctionInfo>;
using RSUInfoConstPtr = std::shared_ptr<const RSUInfo>;
using ObstacleInfoConstPtr = std::shared_ptr<const ObstacleInfo>;
using ElectricFenceInfoConstPtr = std::shared_ptr<const ElectricFenceInfo>;

class LaneInfo {
 public:
  explicit LaneInfo(const Lane &lane);

  const Id &id() const { return lane_.id(); }
  const Id &road_id() const { return road_id_; }
  const Id &section_id() const { return section_id_; }
  const Lane &lane() const { return lane_; }
  const std::vector<century::common::math::Vec2d> &points() const {
    return points_;
  }
  const std::vector<century::common::math::Vec2d> &unit_directions() const {
    return unit_directions_;
  }
  double Heading(const double s) const;
  double Curvature(const double s) const;
  const std::vector<double> &headings() const { return headings_; }
  const std::vector<century::common::math::LineSegment2d> &segments() const {
    return segments_;
  }
  const std::vector<double> &accumulate_s() const { return accumulated_s_; }
  const std::vector<OverlapInfoConstPtr> &overlaps() const { return overlaps_; }
  const std::vector<OverlapInfoConstPtr> &cross_lanes() const {
    return cross_lanes_;
  }
  const std::vector<OverlapInfoConstPtr> &signals() const { return signals_; }
  const std::vector<OverlapInfoConstPtr> &yield_signs() const {
    return yield_signs_;
  }
  const std::vector<OverlapInfoConstPtr> &stop_signs() const {
    return stop_signs_;
  }
  const std::vector<OverlapInfoConstPtr> &crosswalks() const {
    return crosswalks_;
  }
  const std::vector<OverlapInfoConstPtr> &junctions() const {
    return junctions_;
  }
  const std::vector<OverlapInfoConstPtr> &clear_areas() const {
    return clear_areas_;
  }
  const std::vector<OverlapInfoConstPtr> &speed_bumps() const {
    return speed_bumps_;
  }
  const std::vector<OverlapInfoConstPtr> &parking_spaces() const {
    return parking_spaces_;
  }
  const std::vector<OverlapInfoConstPtr> &pnc_junctions() const {
    return pnc_junctions_;
  }
  double total_length() const { return total_length_; }
  using SampledWidth = std::pair<double, double>;
  const std::vector<SampledWidth> &sampled_left_width() const {
    return sampled_left_width_;
  }
  const std::vector<SampledWidth> &sampled_right_width() const {
    return sampled_right_width_;
  }
  void GetWidth(const double s, double *left_width, double *right_width) const;
  double GetWidth(const double s) const;
  double GetEffectiveWidth(const double s) const;

  const std::vector<SampledWidth> &sampled_left_road_width() const {
    return sampled_left_road_width_;
  }
  const std::vector<SampledWidth> &sampled_right_road_width() const {
    return sampled_right_road_width_;
  }
  void GetRoadWidth(const double s, double *left_width,
                    double *right_width) const;
  double GetRoadWidth(const double s) const;

  bool IsOnLane(const century::common::math::Vec2d &point) const;
  bool IsOnLane(const century::common::math::Box2d &box) const;

  century::common::PointENU GetSmoothPoint(double s) const;
  double DistanceTo(const century::common::math::Vec2d &point) const;
  double DistanceTo(const century::common::math::Vec2d &point,
                    century::common::math::Vec2d *map_point, double *s_offset,
                    int *s_offset_index) const;
  century::common::PointENU GetNearestPoint(
      const century::common::math::Vec2d &point, double *distance) const;
  bool GetProjection(const century::common::math::Vec2d &point,
                     double *accumulate_s, double *lateral) const;

 private:
  friend class HDMapImpl;
  friend class RoadInfo;
  void Init();
  void PostProcess(const HDMapImpl &map_instance);
  void UpdateOverlaps(const HDMapImpl &map_instance);
  double GetWidthFromSample(const std::vector<LaneInfo::SampledWidth> &samples,
                            const double s) const;
  void CreateKDTree();
  void set_road_id(const Id &road_id) { road_id_ = road_id; }
  void set_section_id(const Id &section_id) { section_id_ = section_id; }

 private:
  const Lane &lane_;
  std::vector<century::common::math::Vec2d> points_;
  std::vector<century::common::math::Vec2d> unit_directions_;
  std::vector<double> headings_;
  std::vector<century::common::math::LineSegment2d> segments_;
  std::vector<double> accumulated_s_;
  std::vector<std::string> overlap_ids_;
  std::vector<OverlapInfoConstPtr> overlaps_;
  std::vector<OverlapInfoConstPtr> cross_lanes_;
  std::vector<OverlapInfoConstPtr> signals_;
  std::vector<OverlapInfoConstPtr> yield_signs_;
  std::vector<OverlapInfoConstPtr> stop_signs_;
  std::vector<OverlapInfoConstPtr> crosswalks_;
  std::vector<OverlapInfoConstPtr> junctions_;
  std::vector<OverlapInfoConstPtr> clear_areas_;
  std::vector<OverlapInfoConstPtr> speed_bumps_;
  std::vector<OverlapInfoConstPtr> parking_spaces_;
  std::vector<OverlapInfoConstPtr> pnc_junctions_;
  double total_length_ = 0.0;
  std::vector<SampledWidth> sampled_left_width_;
  std::vector<SampledWidth> sampled_right_width_;

  std::vector<SampledWidth> sampled_left_road_width_;
  std::vector<SampledWidth> sampled_right_road_width_;

  std::vector<LaneSegmentBox> segment_box_list_;
  std::unique_ptr<LaneSegmentKDTree> lane_segment_kdtree_;

  Id road_id_;
  Id section_id_;
};

class JunctionInfo {
 public:
  explicit JunctionInfo(const Junction &junction);

  const Id &id() const { return junction_.id(); }
  const Junction &junction() const { return junction_; }
  const century::common::math::Polygon2d &polygon() const { return polygon_; }

  const std::vector<Id> &OverlapStopSignIds() const {
    return overlap_stop_sign_ids_;
  }

  const std::vector<Id> &OverlapSignalIds() const {
    return overlap_signal_ids_;
  }

 private:
  friend class HDMapImpl;
  void Init();
  void PostProcess(const HDMapImpl &map_instance);
  void UpdateOverlaps(const HDMapImpl &map_instance);

 private:
  const Junction &junction_;
  century::common::math::Polygon2d polygon_;

  std::vector<Id> overlap_stop_sign_ids_;
  std::vector<Id> overlap_signal_ids_;
  std::vector<Id> overlap_ids_;
};
using JunctionPolygonBox =
    ObjectWithAABox<JunctionInfo, century::common::math::Polygon2d>;
using JunctionPolygonKDTree =
    century::common::math::AABoxKDTree2d<JunctionPolygonBox>;

class ElectricFenceInfo {
 public:
  explicit ElectricFenceInfo(const ElectricFence &electric_fence);

  const Id &id() const { return electric_fence_.id(); }
  const ElectricFence &electric_fence() const { return electric_fence_; }
  const century::common::math::Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const ElectricFence &electric_fence_;
  century::common::math::Polygon2d polygon_;
};
using ElectricFencePolygonBox =
    ObjectWithAABox<ElectricFenceInfo, century::common::math::Polygon2d>;
using ElectricFencePolygonKDTree =
    century::common::math::AABoxKDTree2d<ElectricFencePolygonBox>;

class SignalInfo {
 public:
  explicit SignalInfo(const Signal &signal);

  const Id &id() const { return signal_.id(); }
  const Signal &signal() const { return signal_; }
  const std::vector<century::common::math::LineSegment2d> &segments() const {
    return segments_;
  }

 private:
  void Init();

 private:
  const Signal &signal_;
  std::vector<century::common::math::LineSegment2d> segments_;
};
using SignalSegmentBox =
    ObjectWithAABox<SignalInfo, century::common::math::LineSegment2d>;
using SignalSegmentKDTree =
    century::common::math::AABoxKDTree2d<SignalSegmentBox>;

class CrosswalkInfo {
 public:
  explicit CrosswalkInfo(const Crosswalk &crosswalk);

  const Id &id() const { return crosswalk_.id(); }
  const Crosswalk &crosswalk() const { return crosswalk_; }
  const century::common::math::Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const Crosswalk &crosswalk_;
  century::common::math::Polygon2d polygon_;
};
using CrosswalkPolygonBox =
    ObjectWithAABox<CrosswalkInfo, century::common::math::Polygon2d>;
using CrosswalkPolygonKDTree =
    century::common::math::AABoxKDTree2d<CrosswalkPolygonBox>;

class StopSignInfo {
 public:
  explicit StopSignInfo(const StopSign &stop_sign);

  const Id &id() const { return stop_sign_.id(); }
  const StopSign &stop_sign() const { return stop_sign_; }
  const std::vector<century::common::math::LineSegment2d> &segments() const {
    return segments_;
  }
  const std::vector<Id> &OverlapLaneIds() const { return overlap_lane_ids_; }
  const std::vector<Id> &OverlapJunctionIds() const {
    return overlap_junction_ids_;
  }

 private:
  friend class HDMapImpl;
  void init();
  void PostProcess(const HDMapImpl &map_instance);
  void UpdateOverlaps(const HDMapImpl &map_instance);

 private:
  const StopSign &stop_sign_;
  std::vector<century::common::math::LineSegment2d> segments_;

  std::vector<Id> overlap_lane_ids_;
  std::vector<Id> overlap_junction_ids_;
  std::vector<Id> overlap_ids_;
};
using StopSignSegmentBox =
    ObjectWithAABox<StopSignInfo, century::common::math::LineSegment2d>;
using StopSignSegmentKDTree =
    century::common::math::AABoxKDTree2d<StopSignSegmentBox>;

class YieldSignInfo {
 public:
  explicit YieldSignInfo(const YieldSign &yield_sign);

  const Id &id() const { return yield_sign_.id(); }
  const YieldSign &yield_sign() const { return yield_sign_; }
  const std::vector<century::common::math::LineSegment2d> &segments() const {
    return segments_;
  }

 private:
  void Init();

 private:
  const YieldSign &yield_sign_;
  std::vector<century::common::math::LineSegment2d> segments_;
};
using YieldSignSegmentBox =
    ObjectWithAABox<YieldSignInfo, century::common::math::LineSegment2d>;
using YieldSignSegmentKDTree =
    century::common::math::AABoxKDTree2d<YieldSignSegmentBox>;

class ClearAreaInfo {
 public:
  explicit ClearAreaInfo(const ClearArea &clear_area);

  const Id &id() const { return clear_area_.id(); }
  const ClearArea &clear_area() const { return clear_area_; }
  const century::common::math::Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const ClearArea &clear_area_;
  century::common::math::Polygon2d polygon_;
};
using ClearAreaPolygonBox =
    ObjectWithAABox<ClearAreaInfo, century::common::math::Polygon2d>;
using ClearAreaPolygonKDTree =
    century::common::math::AABoxKDTree2d<ClearAreaPolygonBox>;

class SpeedBumpInfo {
 public:
  explicit SpeedBumpInfo(const SpeedBump &speed_bump);

  const Id &id() const { return speed_bump_.id(); }
  const SpeedBump &speed_bump() const { return speed_bump_; }
  const std::vector<century::common::math::LineSegment2d> &segments() const {
    return segments_;
  }

 private:
  void Init();

 private:
  const SpeedBump &speed_bump_;
  std::vector<century::common::math::LineSegment2d> segments_;
};
using SpeedBumpSegmentBox =
    ObjectWithAABox<SpeedBumpInfo, century::common::math::LineSegment2d>;
using SpeedBumpSegmentKDTree =
    century::common::math::AABoxKDTree2d<SpeedBumpSegmentBox>;

class OverlapInfo {
 public:
  explicit OverlapInfo(const Overlap &overlap);

  const Id &id() const { return overlap_.id(); }
  const Overlap &overlap() const { return overlap_; }
  const ObjectOverlapInfo *GetObjectOverlapInfo(const Id &id) const;

 private:
  const Overlap &overlap_;
};

class RoadInfo {
 public:
  explicit RoadInfo(const Road &road);
  const Id &id() const { return road_.id(); }
  const Road &road() const { return road_; }
  const std::vector<RoadSection> &sections() const { return sections_; }

  const Id &junction_id() const { return road_.junction_id(); }
  bool has_junction_id() const { return road_.has_junction_id(); }

  const std::vector<RoadBoundary> &GetBoundaries() const;

  century::hdmap::Road_Type type() const { return road_.type(); }

 private:
  Road road_;
  std::vector<RoadSection> sections_;
  std::vector<RoadBoundary> road_boundaries_;
};

class ParkingLotInfo {
 public:
  explicit ParkingLotInfo(const ParkingLot &parkinglot);
  const Id &id() const { return parking_lot_.id(); }
  const ParkingLot &parking_lot() const { return parking_lot_; }
  const century::common::math::Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  ParkingLot parking_lot_;
  century::common::math::Polygon2d polygon_;
};
using ParkingLotPolygonBox =
    ObjectWithAABox<ParkingLotInfo, century::common::math::Polygon2d>;
using ParkingLotPolygonKDTree =
    century::common::math::AABoxKDTree2d<ParkingLotPolygonBox>;

class ParkingSpaceInfo {
 public:
  explicit ParkingSpaceInfo(const ParkingSpace &parkingspace);
  const Id &id() const { return parking_space_.id(); }
  const Id &parking_lot_id() const { return parking_lot_id_; }
  void set_parking_lot_id(Id id) { parking_lot_id_ = id; }
  const Id &lane_id() const { return lane_id_; }
  void set_lane_id(Id id) { lane_id_ = id; }
  const double lane_start_s() const { return start_s_; }
  void set_lane_start_s(const double start_s) { start_s_ = start_s; }
  const double lane_end_s() const { return end_s_; }
  void set_lane_end_s(const double end_s) { end_s_ = end_s; }
  const ParkingSpace &parking_space() const { return parking_space_; }
  const century::common::math::Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const ParkingSpace &parking_space_;
  Id parking_lot_id_;
  Id lane_id_;
  double start_s_ = 0.0, end_s_ = 0.0;
  century::common::math::Polygon2d polygon_;
};
using ParkingSpacePolygonBox =
    ObjectWithAABox<ParkingSpaceInfo, century::common::math::Polygon2d>;
using ParkingSpacePolygonKDTree =
    century::common::math::AABoxKDTree2d<ParkingSpacePolygonBox>;

class PNCJunctionInfo {
 public:
  explicit PNCJunctionInfo(const PNCJunction &pnc_junction);

  const Id &id() const { return junction_.id(); }
  const PNCJunction &pnc_junction() const { return junction_; }
  const century::common::math::Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const PNCJunction &junction_;
  century::common::math::Polygon2d polygon_;

  std::vector<Id> overlap_ids_;
};
using PNCJunctionPolygonBox =
    ObjectWithAABox<PNCJunctionInfo, century::common::math::Polygon2d>;
using PNCJunctionPolygonKDTree =
    century::common::math::AABoxKDTree2d<PNCJunctionPolygonBox>;

struct JunctionBoundary {
  JunctionInfoConstPtr junction_info;
};

using JunctionBoundaryPtr = std::shared_ptr<JunctionBoundary>;

class RSUInfo {
 public:
  explicit RSUInfo(const RSU &rsu);

  const Id &id() const { return _rsu.id(); }
  const RSU &rsu() const { return _rsu; }

 private:
  RSU _rsu;
};

class ObstacleInfo {
 public:
  explicit ObstacleInfo(const Obstacle &obstacle);

  const Id &id() const { return obstacle_.id(); }
  const Obstacle &obstacle() const { return obstacle_; }
  const century::common::math::Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const Obstacle &obstacle_;
  century::common::math::Polygon2d polygon_;
};

using ObstaclePolygonBox =
    ObjectWithAABox<ObstacleInfo, century::common::math::Polygon2d>;
using ObstaclePolygonKDTree =
    century::common::math::AABoxKDTree2d<ObstaclePolygonBox>;

}  // namespace hdmap
}  // namespace century
