#pragma once

#include <float.h>

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/map/proto/map_junction.pb.h"
#include "modules/routing/proto/routing_config.pb.h"
#include "modules/routing/proto/routing_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/core/navigator.h"

namespace century {
namespace routing {

using century::common::ErrorCode;
using century::common::PointENU;
using century::common::VehicleConfigHelper;
using century::common::math::Box2d;
using century::common::math::Polygon2d;
using century::common::math::Vec2d;
using century::hdmap::Junction;
using century::hdmap::JunctionInfoConstPtr;
using century::hdmap::ParkingSpaceInfoConstPtr;
enum SearchDirection {
  FORWARD_DIRECTION = 0u,
  LEFT_DIRECTION = 1u,
  RIGHT_DIRECTION = 2u,
};

enum SparseType {
  S_ALL = 0u,
  S_FRONT = 1u,
  S_CENTER = 2u,
  S_BACK = 3u,
  S_FRONT_CENTER = 4u,
  S_FRONT_BACK = 5u,
  S_CENTER_BACK = 6u,
};

class RoutingMath {
 public:
  RoutingMath() {}
  virtual ~RoutingMath() = default;

 public:
  bool Init(const hdmap::HDMap *hdmap);
  bool IsLaneUsable(const std::shared_ptr<const hdmap::LaneInfo> lane_info,
                    const common::PointENU &point, double &s);
  bool IsPointWithinRangeLine(const common::PointENU &c,
                              const std::string_view fix_point_area);
  bool IsPointWithinRangeLine(const common::PointENU &c,
                              const common::PointENU &b,
                              const common::PointENU &a);
  bool IsJunctionContainAdc(const LaneWaypoint &lane_waypoint,
                            const hdmap::JunctionInfo &junction_info);
  bool IsRelativeTargetPointToWeat(const PointENU &front_point,
                                   const PointENU &back_point,
                                   const std::string_view fix_point_area);
  bool IsFixedPointClose(const common::PointENU &frist,
                         const common::PointENU &second,
                         double min_point_distance = 2.0);
  bool IsPointInSpecificJunction(const LaneWaypoint &lane_waypoint,
                                 const int junction_type);
  bool IsSatisfyLaneChange(const RoadSegment &road_passage,
                           const double &lane_change_length);
  bool IsLaneInRoad(const std::string &lane_id, const RoadSegment &road);
  std::vector<common::PointENU> GetFixedPoints(std::string_view id);
  double GetTwoPointsDirection(const common::PointENU &from,
                               const common::PointENU &to);
  double GetTwoPointsDistance(const common::PointENU &from,
                              const common::PointENU &to);
  bool GetLaneNearestPointByIdAndPoint(
      const hdmap::Id &neighbor_id, const LaneWaypoint &lane_waypoint,
      std::shared_ptr<const hdmap::LaneInfo> &target_lane,
      common::PointENU &neast_point);
  PointENU GetLaneNearestPointByIdAndPoint(const hdmap::Id &neighbor_id,
                                           const LaneWaypoint &lane_waypoint);
  bool GetWheelCraneProjectionLaneInfor(LaneWaypoint &lane_waypoint);
  void GetWaypointInfoByPointProjection(
      const std::shared_ptr<const hdmap::LaneInfo> target_lane,
      const common::PointENU &neast_point, LaneWaypoint &lane_waypoint);
  double GetNoTurnLaneHeading(const double &stacker_heading,
                              const LaneWaypoint &lane_waypoint,
                              bool is_to_front);
  std::vector<common::PointENU> GetHumanShapePoints(
      const common::PointENU &point, const std::string_view fix_point_area,
      bool is_in_ego_lane, bool is_west);
  bool GetWaypointAlongSuccessorIdByDistance(const LaneWaypoint &adc_waypoint,
                                             LaneWaypoint &end_lane_waypoint,
                                             double move_distance);
  bool GetWaypointAlongPredecessorIdByDistance(const LaneWaypoint &adc_waypoint,
                                               LaneWaypoint &end_lane_waypoint,
                                               double move_distance);
  bool GetCraneProjectionLaneInfor(LaneWaypoint &lane_waypoint);
  bool GetNearestProjection(
      const LaneWaypoint &lane_waypoint,
      std::shared_ptr<const hdmap::LaneInfo> &out_lane,
      common::PointENU &out_point);
  bool GetHeadingProjection(
      const LaneWaypoint &lane_waypoint,
      std::shared_ptr<const hdmap::LaneInfo> &out_lane,
      common::PointENU &out_point);
  bool GetHeadingProjectionFromLane(
      const LaneWaypoint &lane_waypoint,
      const std::shared_ptr<const hdmap::LaneInfo> &start_lane,
      double start_s,
      const common::PointENU &nearest_point,
      std::shared_ptr<const hdmap::LaneInfo> &out_lane,
      common::PointENU &out_point);

  // Helpers for GetHeadingProjectionFromLane
  double CalcHeadingDiff(
      const LaneWaypoint &lane_waypoint,
      const std::shared_ptr<const hdmap::LaneInfo> &lane, double s);
  std::shared_ptr<const hdmap::LaneInfo> GetStraightSuccessor(
      const std::shared_ptr<const hdmap::LaneInfo> &lane);
  std::shared_ptr<const hdmap::LaneInfo> GetStraightPredecessor(
      const std::shared_ptr<const hdmap::LaneInfo> &lane);
  bool AdvanceAlongLane(
      std::shared_ptr<const hdmap::LaneInfo> &lane, double &s,
      double step, int &lane_count);
  bool GetParkingID(const common::PointENU &parking_point,
                    std::string *parking_space_id);
  std::vector<std::string> GetBlacklistedLanes(
      const std::string_view blacklist_id, bool is_only_add_reverse = false);
  std::vector<std::string> GetBlacklistedLanes(
      const common::PointENU &near_point, const double &min_point_distance,
      const std::string_view blacklist_id, bool is_only_add_reverse = false);

  std::vector<common::PointENU> GetSparsePointOfLane(
      const std::shared_ptr<const hdmap::LaneInfo> lane,
      const SparseType &sparse_type);
  bool GetSearchDirection(
      const LaneWaypoint &lane_waypoint, const hdmap::Id &neighbor_id,
      std::unordered_map<SearchDirection, std::vector<hdmap::Id>>
          &target_lanes);
  bool MoveLaneWaypointAlongLane(const double &move_heading,
                                 const double move_distance,
                                 LaneWaypoint &adc_waypoint);
  common::PointENU MoveAlongLine(const common::PointENU &proj,
                                 const std::string_view fix_point_area,
                                 const double distance, const bool forward);
  common::PointENU ProjectPointToLine(const common::PointENU &p,
                                      const std::string_view fix_point_area);
  common::PointENU ProjectPointToLine(const PointENU &p, const PointENU &b,
                                      const PointENU &a);
  double PointToLineDistance(const common::PointENU &p,
                             const common::PointENU &lineStart,
                             const common::PointENU &lineEnd);
  bool SearchNeighborLaneInfo(
      const std::unordered_map<SearchDirection, std::vector<hdmap::Id>>
          &target_lanes,
      LaneWaypoint &lane_waypoint);
  bool SearchLeftNeighborLaneInfo(const hdmap::Id &target_lane_id,
                                  LaneWaypoint &lane_waypoint);
  bool SearchRightNeighborLaneInfo(const hdmap::Id &target_lane_id,
                                   LaneWaypoint &lane_waypoint);
  void SearchSparsePointAndReserveTypeBySegments(
      const RoadSegment &road_passage, const int &road_iter,
      const bool end_road_iter, std::vector<common::PointENU> &road_points,
      bool &is_is_reverse_type);
  bool SearchNearLaneWaypoint(const common::PointENU &target_point,
                              std::vector<LaneWaypoint> &vec_lane_waypoint);
  bool IsTargetLaneStraight(const std::string &target_lane_id,
                           const std::string_view fixed_point_id,
                           double search_radius = 1.0);

 private:
  const hdmap::HDMap *hdmap_ = nullptr;
  routing::FixedPoints fixed_points_;
  routing::CraneProjectionConfig crane_proj_config_;
};

}  // namespace routing
}  // namespace century