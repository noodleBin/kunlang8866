/* Copyright 2022 The Century Authors. All Rights Reserved.

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
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/map/proto/map.pb.h"
#include "modules/map/proto/map_clear_area.pb.h"
#include "modules/map/proto/map_crosswalk.pb.h"
#include "modules/map/proto/map_electric_fence.pb.h"
#include "modules/map/proto/map_geometry.pb.h"
#include "modules/map/proto/map_junction.pb.h"
#include "modules/map/proto/map_lane.pb.h"
#include "modules/map/proto/map_obstacle.pb.h"
#include "modules/map/proto/map_overlap.pb.h"
#include "modules/map/proto/map_parking_space.pb.h"
#include "modules/map/proto/map_pnc_junction.pb.h"
#include "modules/map/proto/map_signal.pb.h"
#include "modules/map/proto/map_speed_bump.pb.h"
#include "modules/map/proto/map_stop_sign.pb.h"
#include "modules/map/proto/map_yield_sign.pb.h"

#include "modules/common/math/aabox2d.h"
#include "modules/common/math/aaboxkdtree2d.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/map/hdmap/hdmap_common.h"

/**
 * @namespace century::hdmap
 * @brief century::hdmap
 */
namespace century {
namespace hdmap {

/**
 * @class HDMapImpl
 *
 * @brief High-precision map loader implement.
 */
class HDMapImpl {
 public:
  using LaneTable = std::unordered_map<std::string, std::shared_ptr<LaneInfo>>;
  using JunctionTable =
      std::unordered_map<std::string, std::shared_ptr<JunctionInfo>>;
  using SignalTable =
      std::unordered_map<std::string, std::shared_ptr<SignalInfo>>;
  using CrosswalkTable =
      std::unordered_map<std::string, std::shared_ptr<CrosswalkInfo>>;
  using StopSignTable =
      std::unordered_map<std::string, std::shared_ptr<StopSignInfo>>;
  using YieldSignTable =
      std::unordered_map<std::string, std::shared_ptr<YieldSignInfo>>;
  using ClearAreaTable =
      std::unordered_map<std::string, std::shared_ptr<ClearAreaInfo>>;
  using SpeedBumpTable =
      std::unordered_map<std::string, std::shared_ptr<SpeedBumpInfo>>;
  using OverlapTable =
      std::unordered_map<std::string, std::shared_ptr<OverlapInfo>>;
  using RoadTable = std::unordered_map<std::string, std::shared_ptr<RoadInfo>>;
  using ParkingSpaceTable =
      std::unordered_map<std::string, std::shared_ptr<ParkingSpaceInfo>>;
  using ParkingLotTable =
      std::unordered_map<std::string, std::shared_ptr<ParkingLotInfo>>;
  using PNCJunctionTable =
      std::unordered_map<std::string, std::shared_ptr<PNCJunctionInfo>>;
  using RSUTable = std::unordered_map<std::string, std::shared_ptr<RSUInfo>>;
  using ObstacleTable =
      std::unordered_map<std::string, std::shared_ptr<ObstacleInfo>>;
  using ElectricFenceTable =
      std::unordered_map<std::string, std::shared_ptr<ElectricFenceInfo>>;

 public:
  /**
   * @brief load map from local file
   * @param map_filename path of map data file
   * @return 0:success, otherwise failed
   */
  int LoadMapFromFile(const std::string& map_filename);

  /**
   * @brief load map from a protobuf message
   * @param map_proto map data in protobuf format
   * @return 0:success, otherwise failed
   */
  int LoadMapFromProto(const Map& map_proto);

  LaneInfoConstPtr GetLaneById(const Id& id) const;
  JunctionInfoConstPtr GetJunctionById(const Id& id) const;
  SignalInfoConstPtr GetSignalById(const Id& id) const;
  CrosswalkInfoConstPtr GetCrosswalkById(const Id& id) const;
  StopSignInfoConstPtr GetStopSignById(const Id& id) const;
  YieldSignInfoConstPtr GetYieldSignById(const Id& id) const;
  ClearAreaInfoConstPtr GetClearAreaById(const Id& id) const;
  SpeedBumpInfoConstPtr GetSpeedBumpById(const Id& id) const;
  OverlapInfoConstPtr GetOverlapById(const Id& id) const;
  RoadInfoConstPtr GetRoadById(const Id& id) const;
  ParkingSpaceInfoConstPtr GetParkingSpaceById(const Id& id) const;
  ParkingLotInfoConstPtr GetParkingLotById(const Id& id) const;
  PNCJunctionInfoConstPtr GetPNCJunctionById(const Id& id) const;
  RSUInfoConstPtr GetRSUById(const Id& id) const;
  ObstacleInfoConstPtr GetObstacleAreaById(const Id& id) const;
  ElectricFenceInfoConstPtr GetElectricFenceById(const Id& id) const;

  /**
   * @brief get all lanes in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param lanes store all lanes in target range
   * @return 0:success, otherwise failed
   */
  int GetLanes(const century::common::PointENU& point, double distance,
               std::vector<LaneInfoConstPtr>* lanes) const;
  /**
   * @brief get all junctions in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param junctions store all junctions in target range
   * @return 0:success, otherwise failed
   */
  int GetJunctions(const century::common::PointENU& point, double distance,
                   std::vector<JunctionInfoConstPtr>* junctions) const;
  /**
   * @brief get all electric fences in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param electric_fences store all electric fences in target range
   * @return 0:success, otherwise failed
   */
  int GetElectricFences(
      const century::common::PointENU& point, double distance,
      std::vector<ElectricFenceInfoConstPtr>* electric_fences) const;
  /**
   * @brief get all crosswalks in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param crosswalks store all crosswalks in target range
   * @return 0:success, otherwise failed
   */
  int GetCrosswalks(const century::common::PointENU& point, double distance,
                    std::vector<CrosswalkInfoConstPtr>* crosswalks) const;
  /**
   * @brief get all signals in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param signals store all signals in target range
   * @return 0:success, otherwise failed
   */
  int GetSignals(const century::common::PointENU& point, double distance,
                 std::vector<SignalInfoConstPtr>* signals) const;
  /**
   * @brief get all stop signs in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param stop signs store all stop signs in target range
   * @return 0:success, otherwise failed
   */
  int GetStopSigns(const century::common::PointENU& point, double distance,
                   std::vector<StopSignInfoConstPtr>* stop_signs) const;
  /**
   * @brief get all yield signs in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param yield signs store all yield signs in target range
   * @return 0:success, otherwise failed
   */
  int GetYieldSigns(const century::common::PointENU& point, double distance,
                    std::vector<YieldSignInfoConstPtr>* yield_signs) const;
  /**
   * @brief get all clear areas in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param clear_areas store all clear areas in target range
   * @return 0:success, otherwise failed
   */
  int GetClearAreas(const century::common::PointENU& point, double distance,
                    std::vector<ClearAreaInfoConstPtr>* clear_areas) const;
  /**
   * @brief get all speed bumps in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param speed_bumps store all speed bumps in target range
   * @return 0:success, otherwise failed
   */
  int GetSpeedBumps(const century::common::PointENU& point, double distance,
                    std::vector<SpeedBumpInfoConstPtr>* speed_bumps) const;
  /**
   * @brief get all roads in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param roads store all roads in target range
   * @return 0:success, otherwise failed
   */
  int GetRoads(const century::common::PointENU& point, double distance,
               std::vector<RoadInfoConstPtr>* roads) const;

  int GetObstacleAreas(const century::common::PointENU& point, double distance,
                       std::vector<ObstacleInfoConstPtr>* obstacles) const;

  bool IsInRodArea(const century::common::PointENU& point) const;

  /**
   * @brief get all parking space in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param parking_spaces store all parking spaces in target range
   * @return 0:success, otherwise failed
   */
  int GetParkingSpaces(
      const century::common::PointENU& point, double distance,
      std::vector<ParkingSpaceInfoConstPtr>* parking_spaces) const;

  /**
   * @brief get all pnc junctions in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param junctions store all junctions in target range
   * @return 0:success, otherwise failed
   */
  int GetPNCJunctions(
      const century::common::PointENU& point, double distance,
      std::vector<PNCJunctionInfoConstPtr>* pnc_junctions) const;

  /*
   * @brief get all parking lots in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param parking lots store all parking lots in target range
   * @return 0:success, otherwise failed
   */
  int GetParkingLots(const century::common::PointENU& point, double distance,
                     std::vector<ParkingLotInfoConstPtr>* parking_lots) const;

  /*
   * @brief get all parking lots in certail range
   * @param point the central point of the range
   * @param distance the search radius
   * @param parking lots store all parking lots in target range
   * @return 0:success, otherwise failed
   */
  int GetParkingLots(const century::common::math::Vec2d& point, double distance,
                     std::vector<ParkingLotInfoConstPtr>* parking_lots) const;

  /**
   * @brief get nearest lane from target point,
   * @param point the target point
   * @param nearest_lane the nearest lane that match search conditions
   * @param nearest_s the offset from lane start point along lane center line
   * @param nearest_l the lateral offset from lane center line
   * @return 0:success, otherwise, failed.
   */
  int GetNearestLane(const century::common::PointENU& point,
                     LaneInfoConstPtr* nearest_lane, double* nearest_s,
                     double* nearest_l) const;
  /**
   * @brief get the nearest lane within a certain range by pose
   * @param point the target position
   * @param distance the search radius
   * @param central_heading the base heading
   * @param max_heading_difference the heading range
   * @param nearest_lane the nearest lane that match search conditions
   * @param nearest_s the offset from lane start point along lane center line
   * @param nearest_l the lateral offset from lane center line
   * @return 0:success, otherwise, failed.
   */
  int GetNearestLaneWithHeading(const century::common::PointENU& point,
                                const double distance,
                                const double central_heading,
                                const double max_heading_difference,
                                LaneInfoConstPtr* nearest_lane,
                                double* nearest_s, double* nearest_l) const;
  /**
   * @brief get all lanes within a certain range by pose
   * @param point the target position
   * @param distance the search radius
   * @param central_heading the base heading
   * @param max_heading_difference the heading range
   * @param nearest_lane all lanes that match search conditions
   * @return 0:success, otherwise, failed.
   */
  int GetLanesWithHeading(const century::common::PointENU& point,
                          const double distance, const double central_heading,
                          const double max_heading_difference,
                          std::vector<LaneInfoConstPtr>* lanes) const;
  /**
   * @brief get all road and junctions boundaries within certain range
   * @param point the target position
   * @param radius the search radius
   * @param road_boundaries the roads' boundaries
   * @param junctions the junctions' boundaries
   * @return 0:success, otherwise failed
   */
  int GetRoadBoundaries(const century::common::PointENU& point, double radius,
                        std::vector<RoadROIBoundaryPtr>* road_boundaries,
                        std::vector<JunctionBoundaryPtr>* junctions) const;
  /**
   * @brief get all road boundaries and junctions within certain range
   * @param point the target position
   * @param radius the search radius
   * @param road_boundaries the roads' boundaries
   * @param junctions the junctions
   * @return 0:success, otherwise failed
   */
  int GetRoadBoundaries(const century::common::PointENU& point, double radius,
                        std::vector<RoadRoiPtr>* road_boundaries,
                        std::vector<JunctionInfoConstPtr>* junctions) const;
  /**
   * @brief get all road boundaries and junctions in certain id collection
   * @param road_ids the road id collection
   * @param road_boundaries the roads' boundaries
   * @param junctions the junctions' boundaries
   * @return 0:success, otherwise failed
   */
  int GetRoadBoundaries(const std::vector<Id>& road_ids,
                        std::vector<RoadRoiPtr>* road_boundaries,
                        std::vector<JunctionInfoConstPtr>* junctions) const;

  /**
   * @brief get all road and junctions boundaries within certain range
   * @param point the target position
   * @param radius the search radius
   * @param ref_ids lanes id in reference line
   * @param road_boundaries the roads' boundaries
   * @param junctions the junctions' boundaries
   * @return 0:success, otherwise failed
   */
  int GetRoadBoundaries(const century::common::PointENU& point, double radius,
                        const std::vector<Id>& ref_ids,
                        std::vector<RoadRoiPtr>* road_boundaries,
                        std::vector<JunctionInfoConstPtr>* junctions) const;

  /**
   * @brief get ROI within certain range
   * @param point the target position
   * @param radius the search radius
   * @param roads_roi the roads' boundaries
   * @param polygons_roi the junctions' boundaries
   * @return 0:success, otherwise failed
   */
  int GetRoi(const century::common::PointENU& point, double radius,
             std::vector<RoadRoiPtr>* roads_roi,
             std::vector<PolygonRoiPtr>* polygons_roi);
  /**
   * @brief get forward nearest signals within certain range on the lane
   *        if there are two signals related to one stop line,
   *        return both signals.
   * @param point the target position
   * @param distance the forward search distance
   * @param signals all signals match conditions
   * @return 0:success, otherwise failed
   */
  int GetForwardNearestSignalsOnLane(
      const century::common::PointENU& point, const double distance,
      std::vector<SignalInfoConstPtr>* signals) const;

  /**
   * @brief get all other stop signs associated with a stop sign
   *        in the same junction
   * @param id id of stop sign
   * @param stop_signs stop signs associated
   * @return 0:success, otherwise failed
   */
  int GetStopSignAssociatedStopSigns(
      const Id& id, std::vector<StopSignInfoConstPtr>* stop_signs) const;

  /**
   * @brief get all lanes associated with a stop sign in the same junction
   * @param id id of stop sign
   * @param lanes all lanes match conditions
   * @return 0:success, otherwise failed
   */
  int GetStopSignAssociatedLanes(const Id& id,
                                 std::vector<LaneInfoConstPtr>* lanes) const;

  /**
   * @brief get a local map which is identical to the origin map except that all
   * map elements without overlap with the given region are deleted.
   * @param point the target position
   * @param range the size of local map region, [width, height]
   * @param local_map local map in proto format
   * @return 0:success, otherwise failed
   */
  int GetLocalMap(const century::common::PointENU& point,
                  const std::pair<double, double>& range, Map* local_map) const;

  /**
   * @brief get forward nearest rsus within certain range
   * @param point the target position
   * @param distance the forward search distance
   * @param central_heading the base heading
   * @param max_heading_difference the heading range
   * @param rsus all rsus that match search conditions
   * @return 0:success, otherwise failed
   */
  int GetForwardNearestRSUs(const century::common::PointENU& point,
                            double distance, double central_heading,
                            double max_heading_difference,
                            std::vector<RSUInfoConstPtr>* rsus) const;

  /**
   * @brief get all lanes within a certain range by pose
   * @param point the target position
   * @param distance the search radius
   * @param nearest_lane all lanes that match search conditions
   * @return 0:success, otherwise, failed.
   */
  int GetLanesWithNearPos(const century::common::PointENU& point,
                          const double distance,
                          std::vector<LaneInfoConstPtr>* lanes) const;

 private:
  int GetLanes(const century::common::math::Vec2d& point, double distance,
               std::vector<LaneInfoConstPtr>* lanes) const;
  int GetJunctions(const century::common::math::Vec2d& point, double distance,
                   std::vector<JunctionInfoConstPtr>* junctions) const;
  int GetElectricFences(
      const century::common::math::Vec2d& point, double distance,
      std::vector<ElectricFenceInfoConstPtr>* electric_fences) const;
  int GetCrosswalks(const century::common::math::Vec2d& point, double distance,
                    std::vector<CrosswalkInfoConstPtr>* crosswalks) const;
  int GetSignals(const century::common::math::Vec2d& point, double distance,
                 std::vector<SignalInfoConstPtr>* signals) const;
  int GetStopSigns(const century::common::math::Vec2d& point, double distance,
                   std::vector<StopSignInfoConstPtr>* stop_signs) const;
  int GetYieldSigns(const century::common::math::Vec2d& point, double distance,
                    std::vector<YieldSignInfoConstPtr>* yield_signs) const;
  int GetClearAreas(const century::common::math::Vec2d& point, double distance,
                    std::vector<ClearAreaInfoConstPtr>* clear_areas) const;
  int GetSpeedBumps(const century::common::math::Vec2d& point, double distance,
                    std::vector<SpeedBumpInfoConstPtr>* speed_bumps) const;
  int GetParkingSpaces(
      const century::common::math::Vec2d& point, double distance,
      std::vector<ParkingSpaceInfoConstPtr>* parking_spaces) const;
  int GetPNCJunctions(
      const century::common::math::Vec2d& point, double distance,
      std::vector<PNCJunctionInfoConstPtr>* pnc_junctions) const;
  int GetNearestLane(const century::common::math::Vec2d& point,
                     LaneInfoConstPtr* nearest_lane, double* nearest_s,
                     double* nearest_l) const;
  int GetNearestLaneWithHeading(const century::common::math::Vec2d& point,
                                const double distance,
                                const double central_heading,
                                const double max_heading_difference,
                                LaneInfoConstPtr* nearest_lane,
                                double* nearest_s, double* nearest_l) const;
  int GetLanesWithHeading(const century::common::math::Vec2d& point,
                          const double distance, const double central_heading,
                          const double max_heading_difference,
                          std::vector<LaneInfoConstPtr>* lanes) const;
  int GetRoads(const century::common::math::Vec2d& point, double distance,
               std::vector<RoadInfoConstPtr>* roads) const;
  int GetObstacleAreas(const century::common::math::Vec2d& point,
                       double distance,
                       std::vector<ObstacleInfoConstPtr>* obstacles) const;

  template <class Table, class BoxTable, class KDTree>
  static void BuildSegmentKDTree(
      const Table& table,
      const century::common::math::AABoxKDTreeParams& params,
      BoxTable* const box_table, std::unique_ptr<KDTree>* const kdtree);

  template <class Table, class BoxTable, class KDTree>
  static void BuildPolygonKDTree(
      const Table& table,
      const century::common::math::AABoxKDTreeParams& params,
      BoxTable* const box_table, std::unique_ptr<KDTree>* const kdtree);

  void BuildLaneSegmentKDTree();
  void BuildJunctionPolygonKDTree();
  void BuildCrosswalkPolygonKDTree();
  void BuildSignalSegmentKDTree();
  void BuildStopSignSegmentKDTree();
  void BuildYieldSignSegmentKDTree();
  void BuildClearAreaPolygonKDTree();
  void BuildSpeedBumpSegmentKDTree();
  void BuildParkingSpacePolygonKDTree();
  void BuildParkingLotPolygonKDTree();
  void BuildPNCJunctionPolygonKDTree();
  void BuildObstaclePolygonKDTree();
  void BuildElectricFencePolygonKDTree();

  template <class KDTree>
  static int SearchObjects(const century::common::math::Vec2d& center,
                           const double radius, const KDTree& kdtree,
                           std::vector<std::string>* const results);

  void Clear();

  int GetLanesWithNearPos(const century::common::math::Vec2d& point,
                          const double distance,
                          std::vector<LaneInfoConstPtr>* lanes) const;
  bool DuplicateChecker(
      const std::vector<std::pair<double, JunctionInfoConstPtr>>&
          overlap_junctions,
      std::vector<RSUInfoConstPtr>* rsus) const;
  int FindSurroundingLanes(
      const century::common::PointENU& point,
      const century::common::math::Vec2d& car_point,
      std::vector<LaneInfoConstPtr>* surrounding_lanes) const;
  LaneInfoConstPtr DetermineStartLane(
      const std::vector<LaneInfoConstPtr>& surrounding_lanes,
      const century::common::math::Vec2d& car_point, double* nearest_s,
      double* nearest_l) const;
  void FindSignalsOnLane(LaneInfoConstPtr lane_ptr, double s_start,
                         double unused_distance,
                         std::vector<SignalInfoConstPtr>* signals) const;
  void ExtractRoadAndJunctionRois(const std::vector<RoadInfoConstPtr>& roads,
                                  std::vector<PolygonRoiPtr>* polygons_roi,
                                  std::set<std::string>* polygon_id_set,
                                  std::vector<RoadRoiPtr>* roads_roi);
  void ExtractParkingSpacePolygons(const std::vector<LaneInfoConstPtr>& lanes,
                                   std::vector<PolygonRoiPtr>* polygons_roi,
                                   std::set<std::string>* polygon_id_set);
  void CreateRoadBoundary(const std::vector<RoadInfoConstPtr>& roads,
                          std::vector<RoadRoiPtr>* road_boundaries,
                          std::vector<JunctionInfoConstPtr>* junctions) const;
  void CreateRoadBoundary(const std::vector<Id>& ref_ids,
                          const std::vector<RoadInfoConstPtr>& roads,
                          std::vector<RoadRoiPtr>* road_boundaries,
                          std::vector<JunctionInfoConstPtr>* junctions) const;
  bool Check(RoadInfoConstPtr road_ptr, const std::vector<Id>& ref_ids) const;

 private:
  Map map_;
  LaneTable lane_table_;
  JunctionTable junction_table_;
  CrosswalkTable crosswalk_table_;
  SignalTable signal_table_;
  StopSignTable stop_sign_table_;
  YieldSignTable yield_sign_table_;
  ClearAreaTable clear_area_table_;
  SpeedBumpTable speed_bump_table_;
  OverlapTable overlap_table_;
  RoadTable road_table_;
  ParkingSpaceTable parking_space_table_;
  ParkingLotTable parking_lot_table_;
  PNCJunctionTable pnc_junction_table_;
  RSUTable rsu_table_;
  ObstacleTable obstacle_table_;
  ElectricFenceTable electric_fence_table_;

  std::vector<LaneSegmentBox> lane_segment_boxes_;
  std::unique_ptr<LaneSegmentKDTree> lane_segment_kdtree_;

  std::vector<JunctionPolygonBox> junction_polygon_boxes_;
  std::unique_ptr<JunctionPolygonKDTree> junction_polygon_kdtree_;

  std::vector<CrosswalkPolygonBox> crosswalk_polygon_boxes_;
  std::unique_ptr<CrosswalkPolygonKDTree> crosswalk_polygon_kdtree_;

  std::vector<SignalSegmentBox> signal_segment_boxes_;
  std::unique_ptr<SignalSegmentKDTree> signal_segment_kdtree_;

  std::vector<StopSignSegmentBox> stop_sign_segment_boxes_;
  std::unique_ptr<StopSignSegmentKDTree> stop_sign_segment_kdtree_;

  std::vector<YieldSignSegmentBox> yield_sign_segment_boxes_;
  std::unique_ptr<YieldSignSegmentKDTree> yield_sign_segment_kdtree_;

  std::vector<ClearAreaPolygonBox> clear_area_polygon_boxes_;
  std::unique_ptr<ClearAreaPolygonKDTree> clear_area_polygon_kdtree_;

  std::vector<SpeedBumpSegmentBox> speed_bump_segment_boxes_;
  std::unique_ptr<SpeedBumpSegmentKDTree> speed_bump_segment_kdtree_;

  std::vector<ParkingSpacePolygonBox> parking_space_polygon_boxes_;
  std::unique_ptr<ParkingSpacePolygonKDTree> parking_space_polygon_kdtree_;

  std::vector<ParkingLotPolygonBox> parking_lot_polygon_boxes_;
  std::unique_ptr<ParkingLotPolygonKDTree> parking_lot_polygon_kdtree_;

  std::vector<PNCJunctionPolygonBox> pnc_junction_polygon_boxes_;
  std::unique_ptr<PNCJunctionPolygonKDTree> pnc_junction_polygon_kdtree_;

  std::vector<ObstaclePolygonBox> obstacle_polygon_boxes_;
  std::unique_ptr<ObstaclePolygonKDTree> obstacle_polygon_kdtree_;

  std::vector<ElectricFencePolygonBox> electric_fence_polygon_boxes_;
  std::unique_ptr<ElectricFencePolygonKDTree> electric_fence_polygon_kdtree_;
};

}  // namespace hdmap
}  // namespace century
