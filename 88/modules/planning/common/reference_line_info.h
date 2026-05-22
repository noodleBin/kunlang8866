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

#pragma once

#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/proto/drive_state.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/planning/proto/lattice_structure.pb.h"
#include "modules/planning/proto/pass_stacker_response.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/historical_tracking_algorithms/hysteresis_interval.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_boundary.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/proto/barrier.pb.h"

namespace century {
namespace planning {
using century::planning::PassStackerRequest;
enum class PassageType {
  SELF = 0,
  LEFT = 1,
  RIGHT = 2,
};

enum class CheckDirection {
  LEFT = 1,
  RIGHT = 2,
  BOTH = 3,
};

struct EStopStatus {
  EStopStatus() {
    is_estop = false;
    reason = "";
  }
  EStopStatus(bool estop_in, std::string reason_in) {
    is_estop = estop_in;
    reason = reason_in;
  }
  // is_estop == true when emergency stop
  bool is_estop;
  std::string reason;
};

/**
 * @class ReferenceLineInfo
 * @brief ReferenceLineInfo holds all data for one reference line.
 */
class ReferenceLineInfo {
 public:
  enum class LaneType { LeftForward, LeftReverse, RightForward, RightReverse };
  ReferenceLineInfo() = default;

  ReferenceLineInfo(const common::VehicleState& vehicle_state,
                    const common::TrajectoryPoint& adc_planning_point,
                    const ReferenceLine& reference_line,
                    const hdmap::RouteSegments& segments);

  bool Init(const std::vector<const Obstacle*>& obstacles);

  bool AddObstacles(const std::vector<const Obstacle*>& obstacles);
  Obstacle* AddObstacle(const Obstacle* obstacle);

  const common::VehicleState& vehicle_state() const { return vehicle_state_; }

  PathDecision* path_decision();
  const PathDecision& path_decision() const;

  const ReferenceLine& reference_line() const;
  ReferenceLine* mutable_reference_line();

  double SDistanceToDestination() const;
  bool ReachedDestination() const;

  void SetTrajectory(const DiscretizedTrajectory& trajectory);
  const DiscretizedTrajectory& trajectory() const;

  double Cost() const { return cost_; }
  void AddCost(double cost) { cost_ += cost; }
  void SetCost(double cost) { cost_ = cost; }
  double PriorityCost() const { return priority_cost_; }
  void SetPriorityCost(double cost) { priority_cost_ = cost; }
  // For lattice planner'speed planning target
  void SetLatticeStopPoint(const StopPoint& stop_point);
  void SetLatticeCruiseSpeed(double speed);
  const PlanningTarget& planning_target() const { return planning_target_; }
  void AddSlowdownObstacle(const std::string& obstacle) {
    slowdown_obstacles_.emplace_back(obstacle);
  }
  const std::vector<std::string> GetSlowdownObstacles() {
    return slowdown_obstacles_;
  }
  void AddCrossSlowdownObstacle(const std::string& obstacle) {
    cross_slowdown_obstacles_.emplace_back(obstacle);
  }
  const std::vector<std::string> GetCrossSlowdownObstacles() {
    return cross_slowdown_obstacles_;
  }
  void AddReverseSlowdownObstacle(const std::string& obstacle) {
    reverse_slowdown_obstacles_.emplace_back(obstacle);
  }
  const std::vector<std::string> GetReverseSlowdownObstacles() {
    return reverse_slowdown_obstacles_;
  }
  const SpeedLimit& speed_limit() const { return speed_limit_; }
  SpeedLimit* mutable_speed_limit() { return &speed_limit_; }

  void AddKeepApproachingObstacle(const std::string& obs_id) {
    keep_approaching_obstacles_.emplace_back(obs_id);
  }
  const std::vector<std::string>& GetKeepApproachingObstacles() const {
    return keep_approaching_obstacles_;
  }

  void AddIgnoreObstacle(const std::string& obs_id) {
    ignore_obstacles_.emplace_back(obs_id);
  }

  const std::vector<std::string>& GetIgnoregObstacles() const {
    return ignore_obstacles_;
  }
  void FillInPathPointInTurn(const std::pair<double,bool>& is_in_turn) {
    path_point_in_turn_vector_.emplace_back(is_in_turn);
  }

  const std::vector<std::pair<double,bool>>& InPathPointInTurn() const {
    return path_point_in_turn_vector_;
  }
bool FindClosestPointInTurn(double target);
  void ReleaseKeepApproachingObstacles() {
    std::vector<std::string>().swap(keep_approaching_obstacles_);
  }

  void SetCruiseSpeed(double speed) { cruise_speed_ = speed; }
  double GetCruiseSpeed() const;
  void SetIsNeedToSpeedLimit(bool is_need_to_speed_limit) {
    is_need_to_speed_limit_ = is_need_to_speed_limit;
  }
  bool IsNeedToSpeedLimit() const { return is_need_to_speed_limit_; }
  void SetIsNeedResetBorrowState(bool is_need_reset_borrow_state) {
    is_need_reset_borrow_state_ = is_need_reset_borrow_state;
  }
  bool IsNeedResetBorrowState() const { return is_need_reset_borrow_state_; }
  const bool IsAuxiliaryRoad() const { return is_auxiliary_road_; }
  void SetIsAuxiliaryRoad(const bool is_auxiliary_road) {
    is_auxiliary_road_ = is_auxiliary_road;
  }
  void SetIsInHighRoadRightLaneOnIntersection(bool is_in_high_road_right_lane,
                                              bool at_left_turn,
                                              bool is_direct_turn) {
    is_in_high_road_right_lane_ = is_in_high_road_right_lane;
    at_left_turn_ = at_left_turn;
    is_direct_turn_ = is_direct_turn;
  }
  void SetIsReachTurnLane(bool is_reach_turn_lane, bool at_left_turn,
                          bool at_right_turn) {
    is_reach_turn_lane_ = is_reach_turn_lane;
    at_left_turn_ = at_left_turn;
    at_right_turn_ = at_right_turn;
  }
  bool IsReachTurnLane() { return is_reach_turn_lane_; }
  void SetTurnInfo(bool is_direct_turn_lane, bool is_need_to_turn,
                   bool at_left_turn, bool at_right_turn, bool path_turn_left,
                   bool path_turn_right) {
    is_direct_turn_lane_ = is_direct_turn_lane;
    is_need_to_turn_ = is_need_to_turn;
    at_left_turn_ = at_left_turn;
    at_right_turn_ = at_right_turn;
    path_turn_right_ = path_turn_right;
    path_turn_left_ = path_turn_left;
  }
  void GetTurnInfo(bool* is_direct_turn_lane, bool* is_need_to_turn,
                   bool* at_left_turn, bool* at_right_turn,
                   bool* path_turn_left, bool* path_turn_right) {
    *is_direct_turn_lane = is_direct_turn_lane_;
    *is_need_to_turn = is_need_to_turn_;
    *at_left_turn = at_left_turn_;
    *at_right_turn = at_right_turn_;
    *path_turn_left = path_turn_left_;
    *path_turn_right = path_turn_right_;
  }
  bool IsInHighRoadRightLaneOnIntersection(bool* at_left_turn,
                                           bool* is_direct_turn) const {
    *at_left_turn = at_left_turn_;
    *is_direct_turn = is_direct_turn_;
    return is_in_high_road_right_lane_;
  }
  void SetIsNeedSlowBreaking(bool is_need_slow_breaking, double min_stop_decel,
                             double min_stop_s) {
    is_need_slow_breaking_ = is_need_slow_breaking;
    min_stop_decel_ = min_stop_decel;
    min_stop_s_ = min_stop_s;
  }
  bool IsNeedSlowBreaking(double* min_stop_decel, double* min_stop_s) const {
    *min_stop_decel = min_stop_decel_;
    *min_stop_s = min_stop_s_;
    return is_need_slow_breaking_;
  }

  void SetSlowerBreakingParam(double target_speed, double target_accel) {
    is_need_to_slower_breaking_for_target_speed_ = true;
    target_speed_for_slower_breaking_ = target_speed;
    target_accel_for_slower_breaking_ = target_accel;
  }

  bool GetSlowerBreakingParam(double* target_speed,
                              double* target_accel) const {
    *target_speed = target_speed_for_slower_breaking_;
    *target_accel = target_accel_for_slower_breaking_;
    return is_need_to_slower_breaking_for_target_speed_;
  }

  void SetSlowlyBreakingParamForApproachObs(double target_speed,
                                            double target_accel) {
    is_need_to_slower_breaking_for_approach_obs_ = true;
    target_speed_for_approach_obs_slower_breaking_ = target_speed;
    target_accel_for_approach_obs_slower_breaking_ = target_accel;
  }

  bool GetSlowlyBreakingParamForApproachObs(double* target_speed,
                                            double* target_accel) const {
    *target_speed = target_speed_for_approach_obs_slower_breaking_;
    *target_accel = target_accel_for_approach_obs_slower_breaking_;
    return is_need_to_slower_breaking_for_approach_obs_;
  }

  hdmap::LaneInfoConstPtr LocateLaneInfo(const double s) const;
  bool HasNeighborLane(const double check_s);

  double GetLeftNeighborLaneWidth(const double s);
  double GetRightNeighborLaneWidth(const double s);
  bool GetNeighborLaneInfo(const ReferenceLineInfo::LaneType lane_type,
                           const double s, hdmap::Id* ptr_lane_id,
                           double* ptr_lane_width) const;
  /**
   * @brief check if current reference line is started from another reference
   *line info line. The method is to check if the start point of current
   *reference line is on previous reference line info.
   * @return returns true if current reference line starts on previous reference
   *line, otherwise false.
   **/
  bool IsStartFrom(const ReferenceLineInfo& previous_reference_line_info) const;

  void SetCommonJunction() { is_common_junction_ = true; }
  void ResetCommonJunction() { is_common_junction_ = false; }
  bool GetCommonJunction() const { return is_common_junction_; }
  // double GetDriveAreaStartS() const { return drive_area_start_s_; }
  // double GetDriveAreaEndS() const { return drive_area_end_s_; }

  void SetNeedTurnRight() { is_need_turn_right_ = true; }
  bool GetNeedTurnRight() const { return is_need_turn_right_; }

  void SetIsInDiagonalRoad(bool is_in_diagonal_road) {
    is_in_diagonal_road_ = is_in_diagonal_road;
  }
  bool IsInDiagonalRoad() const { return is_in_diagonal_road_; }

  void SetIsNearTurn(bool is_near_turn) { is_near_turn_ = is_near_turn; }
  bool IsNearTurn() const { return is_near_turn_; }

  void SetIsInNeedToBackward(bool is_need_to_backward) {
    is_need_to_backward_ = is_need_to_backward;
  }
  bool IsInNeedToBackward() const { return is_need_to_backward_; }

  void SetDiagonalRoadHeading(double diaggonal_road_heading) {
    diaggonal_road_heading_ = diaggonal_road_heading;
  }
  double DiagonalRoadHeading() const { return diaggonal_road_heading_; }

  void SetNeedDiagonal(bool is_need_diagonal) {
    is_need_diagonal_ = is_need_diagonal;
  }
  bool NeedDiagonal() const { return is_need_diagonal_; }
    void SetIsTinyAdjustType(bool is_tiny_adjust_type) {
    is_tiny_adjust_type_ = is_tiny_adjust_type;
  }
  bool IsTinyAdjustType() const { return is_tiny_adjust_type_; }
  void SetIsNewStackerSesponse(bool is_new_stacker_response) {
    is_new_stacker_response_ = is_new_stacker_response;
  }
  bool IsNewStackerSesponse() const { return is_new_stacker_response_; }
  void SetIsPathStraight(bool is_straight_path) {
    is_straight_path_ = is_straight_path;
  }
  bool IsPathStraight() const { return is_straight_path_; }
  void SetEnableShrinkCollisionBuffer(bool enable_shrink_collision_buffer) {
    enable_shrink_collision_buffer_ = enable_shrink_collision_buffer;
  }
  bool EnableShrinkCollisionBuffer() const { return enable_shrink_collision_buffer_; }
  PassStackerRequest NeedNoticeStacker() const { return need_notice_stacker_; }
  void SetNeedNoticeStacker(PassStackerRequest need_notice_stacker) {
    need_notice_stacker_ = need_notice_stacker;
  };
  PassStackerRequest HasPassStackerRequest() { return pass_stacker_request_; };
  void SetPassStackerRequest(PassStackerRequest pass_stacker_request) {
    pass_stacker_request_ = pass_stacker_request;
  };

  const std::unordered_map<std::string, SLBoundary> GetIgvBoundaryMap() const {
    return igv_boundary_map_;
  };
  std::unordered_map<std::string, SLBoundary> IgvBoundaryMap() {
    return igv_boundary_map_;
  };
  void SetIgvBoundaryMap(
      std::unordered_map<std::string, SLBoundary> igv_boundary_map) {
    igv_boundary_map_ = igv_boundary_map;
  };
  std::unordered_map<std::string, SLBoundary> StackerBoundaryMap() {
    return stacker_boundary_map_;
  };
  void SetStackerBoundaryMap(
      std::unordered_map<std::string, SLBoundary> stacker_boundary_map) {
    stacker_boundary_map_ = stacker_boundary_map;
  };
  std::unordered_map<std::string, SLBoundary> WheelcraneBoundaryMap() {
    return wheelcrane_boundary_map_;
  };
  void SetWheelcraneBoundaryMap(
      std::unordered_map<std::string, SLBoundary> wheelcrane_boundary_map) {
    wheelcrane_boundary_map_ = wheelcrane_boundary_map;
  };
  planning::Barrier Barrier() { return barrier_; };
  void SetBarrier(planning::Barrier barrier) { barrier_ = barrier; };
  void SetIsWestIn(bool west_in) { west_in_ = west_in; }
  bool IsWestIn() const { return west_in_; }
  void SetIsWestOut(bool west_out) { west_out_ = west_out; }
  bool IsWestOut() const { return west_out_; }
  void SetIsEastIn(bool east_in) { east_in_ = east_in; }
  bool IsEastIn() const { return east_in_; }
  void SetIsEastOut(bool east_out) { east_out_ = east_out; }
  bool IsEastOut() const { return east_out_; }
  void SetIsPassingGate(bool is_passing_gate) {
    is_passing_gate_ = is_passing_gate;
  }
  bool IsPassingGate() const { return is_passing_gate_; }
  void SetInitPointHeading(double init_point_heading) {
    init_point_heading_ = init_point_heading;
  }
  double InitPointHeading() const { return init_point_heading_; }
  void SetRequestEndL(double request_end_l) { request_end_l_ = request_end_l; }
  double RequestEndL() const { return request_end_l_; }
  std::vector<std::string> GetRefObstacleIds() const {
    return ref_obstacle_id_;
  };
  void SetRefObstacleIds(const std::vector<std::string>& ref_obstacle_id) {
    ref_obstacle_id_ = ref_obstacle_id;
  };

  planning_internal::Debug* mutable_debug() { return &debug_; }
  const planning_internal::Debug& debug() const { return debug_; }
  LatencyStats* mutable_latency_stats() { return &latency_stats_; }
  const LatencyStats& latency_stats() const { return latency_stats_; }

  const PathData& path_data() const;
  const PathData& fallback_path_data() const;
  const SpeedData& speed_data() const;
  PathData* mutable_path_data();
  PathData* mutable_fallback_path_data();
  SpeedData* mutable_speed_data();

  const RSSInfo& rss_info() const;
  RSSInfo* mutable_rss_info();
  // aggregate final result together by some configuration
  bool CombinePathAndSpeedProfile(
      const double relative_time, const double start_s,
      DiscretizedTrajectory* discretized_trajectory);

  // adjust trajectory if it starts from cur_vehicle postion rather planning
  // init point from upstream
  bool AdjustTrajectoryWhichStartsFromCurrentPos(
      const common::TrajectoryPoint& planning_start_point,
      const std::vector<common::TrajectoryPoint>& trajectory,
      DiscretizedTrajectory* adjusted_trajectory);

  const SLBoundary& AdcSlBoundary() const;
  bool AdcSlBoundaryAt(const common::PathPoint& path_point,
                       SLBoundary* const adc_sl_boundary) const;
  std::string PathSpeedDebugString() const;

  /**
   * Check if the current reference line is a change lane reference line, i.e.,
   * ADC's current position is not on this reference line.
   */
  bool IsChangeLanePath() const;

  /**
   * Check if the current reference line is the neighbor of the vehicle
   * current position
   */
  bool IsNeighborLanePath() const;

  /**
   * Check if the current road is fitted the feature of mixed traffic scenario.
   */
  bool IsMixedTraffic() const;

  /**
   * Check if there are many non-motorized vehicle in the city driving road.
   */
  bool HasNonMotorizedVehicle();

  /**
   * Check if there are many retrograde obstacle in the non motor vehicle lane.
   */
  bool HasRetrogradeObstacleOnBicycleLane();

  /**
   * Set if the vehicle can drive following this reference line
   * A planner need to set this value to true if the reference line is OK
   */
  void SetDrivable(bool drivable);
  bool IsDrivable() const;

  void SetTrafficLightRequest(const bool traffic_light_request);
  bool TrafficLightRequest() const;

  void SetNearTrafficLightStopLine(const bool near_traffic_light_stop_line);
  bool IsNearTrafficLightStopLine() const;

  void SetLaneBorrowFailRequest(const bool lane_borrow_fail_request) {
    lane_borrow_fail_request_ = lane_borrow_fail_request;
  }
  bool LaneBorrowFailRequest() const { return lane_borrow_fail_request_; }
  void SetIsLowStartUp(const bool is_low_start_up) {
    is_low_start_up_ = is_low_start_up;
  }
  bool IsLowStartUp() const { return is_low_start_up_; }

  void ExportEngageAdvice(common::EngageAdvice* engage_advice,
                          PlanningContext* planning_context) const;

  const hdmap::RouteSegments& Lanes() const;
  std::list<hdmap::Id> TargetLaneId() const;

  void ExportDecision(DecisionResult* decision_result,
                      PlanningContext* planning_context) const;

  void SetJunctionRightOfWay(const double junction_s,
                             const bool is_protected) const;

  ADCTrajectory::RightOfWayStatus GetRightOfWayStatus() const;

  hdmap::Lane::LaneTurn GetPathTurnType(const double s) const;

  bool GetIntersectionRightofWayStatus(
      const hdmap::PathOverlap& pnc_junction_overlap) const;

  double OffsetToOtherReferenceLine() const {
    return offset_to_other_reference_line_;
  }
  void SetOffsetToOtherReferenceLine(const double offset) {
    offset_to_other_reference_line_ = offset;
  }

  const std::vector<PathBoundary>& GetCandidatePathBoundaries() const;

  void SetCandidatePathBoundaries(
      std::vector<PathBoundary>&& candidate_path_boundaries);

  const std::vector<PathData>& GetCandidatePathData() const;

  void SetCandidatePathData(std::vector<PathData>&& candidate_path_data);

  Obstacle* GetBlockingObstacle() const { return blocking_obstacle_; }
  void SetBlockingObstacle(const std::string& blocking_obstacle_id);

  bool is_path_lane_borrow() const { return is_path_lane_borrow_; }
  void set_is_path_lane_borrow(const bool is_path_lane_borrow) {
    is_path_lane_borrow_ = is_path_lane_borrow;
  }

  uint32_t GetPriority() const { return reference_line_.GetPriority(); }

  void SetPriority(uint32_t priority) { reference_line_.SetPriority(priority); }

  void set_trajectory_type(
      const ADCTrajectory::TrajectoryType trajectory_type) {
    trajectory_type_ = trajectory_type;
  }

  ADCTrajectory::TrajectoryType trajectory_type() const {
    return trajectory_type_;
  }

  StGraphData* mutable_st_graph_data() { return &st_graph_data_; }

  const StGraphData& st_graph_data() { return st_graph_data_; }

  // different types of overlaps that can be handled by different scenarios.
  enum OverlapType {
    CLEAR_AREA = 1,
    CROSSWALK = 2,
    OBSTACLE = 3,
    PNC_JUNCTION = 4,
    SIGNAL = 5,
    STOP_SIGN = 6,
    YIELD_SIGN = 7,
    JUNCTION = 8,
    GATE_AREA = 9,
    DRIVE_AREA = 10,
  };

  const std::vector<std::pair<OverlapType, hdmap::PathOverlap>>&
  FirstEncounteredOverlaps() const {
    return first_encounter_overlaps_;
  }

  int GetPnCJunction(const double s,
                     hdmap::PathOverlap* pnc_junction_overlap) const;

  bool GetJunctionRange(double* start_s, double* end_s) const;

  bool IsSRangeInCommonJunction(const double start_s, const double end_s) const;

  bool IsObstacleOverlapWithJunction(
      const Obstacle& obs, const hdmap::JunctionInfo& junction_info) const;

  bool IsObstacleInCommonJunction(const Obstacle& obs) const;

  bool IsObstacleInGateArea(const Obstacle& obs) const;

  bool IsObstacleInDriveArea(const Obstacle& obs) const;

  std::vector<common::SLPoint> GetAllStopDecisionSLPoint() const;

  void SetTurnSignal(const common::VehicleSignal::TurnSignal& turn_signal);
  void SetEmergencyLight();

  void set_path_reusable(const bool path_reusable) {
    path_reusable_ = path_reusable;
  }

  bool path_reusable() const { return path_reusable_; }

  void set_lane_change_path_reusable(const bool reusable) {
    lane_change_path_reusable_ = reusable;
  }

  bool lane_change_path_reusable() const { return lane_change_path_reusable_; }

  century::routing::LaneWaypoint get_routing_end() const {
    return routing_end_;
  }
  void set_routing_end(const century::routing::LaneWaypoint routing_end) {
    routing_end_ = routing_end;
  }

  double get_end_state_l() const { return end_state_l_; }
  void set_end_state_l(const double end_state_l) { end_state_l_ = end_state_l; }

  hdmap::Lane_LaneType GetLaneType() const { return lane_type_; }
  void setLaneType(hdmap::Lane_LaneType lane_type) { lane_type_ = lane_type; }

  bool IsInPlayStreet(const double s);

  void GetRoadWidthBasedAdc(double* const road_left_width,
                            double* const road_right_width) const;
  bool AdcIsOnRouteLane(PlanningContext* planning_context) const;
  bool AdcIsOnAllRouteLane(PlanningContext* planning_context) const;
  bool AdcIsOnOverlapJunction() const;
  bool GetRoutePassageEndPoint(PlanningContext* planning_context,
                               PassageType passage_type,
                               common::PointENU* const end_point);
  void GetEndPoint(hdmap::LaneInfoConstPtr lane_info_ptr,
                   common::PointENU* const end_point);
  bool GetStartLaneID(PassageType passage_type,
                      std::string* const start_lane_id);
  bool CheckIsHeadbang(const double obstacle_distance, const double check_l);

  /** @brief
   * Calculate the remaining distance that can be traveled on adjacent lanes
   * based on the routing end points of adjacent lanes at the current self
   * driving position or the solid line if current lane has neither left
   * neighbor routing lane nor right neighbor routing lane
   */
  double GetRemainDistanceForBack(PlanningContext* planning_context);
  double GetRemainDistanceToSolidLine();
  double GetRemainDistanceToJunction();
  double GetRemainDistanceToTurnLane();
  double GetRemainDistanceToSignal();
  double GetRemainDistanceToCurbArea();
  bool GetRemainDistanceToRoutingEndPoint(const CheckDirection& check_direction,
                                          PlanningContext* planning_context,
                                          double* const remain_distance);
  bool HasRoutingEnd(PlanningContext* planning_context,
                     const PassageType& passage_type);

  // whole adc four corner points in lane
  bool IsAdcLocatedInLane() const { return is_adc_located_in_lane_; }
  bool IsAdcCoverInLane() const { return is_adc_cover_in_lane_; }
  // whole adc body in lane, and adc is straighten
  bool IsAdcPostureStraight() const { return is_adc_posture_straight_; }
  bool CheckAdcPostureStraight();
  // one of adc four corner points is in lane
  bool IsAdcOnReferenceLine() const { return is_on_reference_line_; }
  bool IsAdcInCommonJunction() const { return is_adc_in_common_junction_; }
  void SetIsAdcInCommonJunction(bool is_adc_in_common_junction) {
    is_adc_in_common_junction_ = is_adc_in_common_junction;
  }
  bool IsAdcInTrafficLightJunction() const {
    return is_adc_in_traffic_light_junction_;
  }
  bool IsAdcInGateArea() const { return is_adc_in_gate_area_; }
  bool IsAdcInDriveArea() const { return is_adc_in_drive_area_; }
  void set_is_on_reference_line() { is_on_reference_line_ = true; }
  // adc center point in lane
  bool IsAdcCenterInLane() const { return is_adc_center_in_lane_; }

  /** @brief
   * It's a vector number
   */
  double GetAdcLongitudinalVelocity() const {
    return adc_longitudinal_velocity_;
  }

  /** @brief
   * It's a vector number
   */
  double GetAdcLateralVelocity() const { return adc_lateral_velocity_; }

  ADCTrajectory::ADBehavior GetTurnBehavior() const;
  const std::pair<double, double>& GetLaneWidthBaseOnAdcCenter() const {
    return lane_width_base_on_adc_center_;
  }

  double GetObstacleCheckCuttinBuffer(const Obstacle* obstacle);
  bool GetIsClearToChangeLane() const { return is_clear_to_change_lane_; }

  // different check level when in lane change due to the lateral distance of
  // adc to the target lane
  enum CheckLevel {
    NO_CHECK = 0,
    PRE_CHECK = 1,
    CHANGE_CHECK = 2,
  };

  const double GetRemainLaneChangeTime() const;
  bool LaneChangePathWillPassOverlap() const;
  bool LaneChangePathWillPassSolidLine();
  bool LaneChangeSafeCheck(const CheckLevel& check_level,
                           const Obstacle* obstacle) const;
  bool IsObstacleBlockAdc(const Obstacle* obstacle) const;
  bool GetHasNeighborForwardLane() const { return has_neighbor_forward_lane_; }
  bool IsOriginalRefLine(const bool& is_turn_left) const;
  bool IsLocatedInSolidLine(const SLBoundary& sl_boundary) const;
  bool IsInLeftNeighborSolidLine(const SLBoundary& sl_boundary);
  bool IsInEitherSolidLine(const SLBoundary& sl_boundary);
  bool IsInLeftNeighborLine(const SLBoundary& sl_boundary);
  bool IsLaneBoundaryPassable(bool check_left, bool check_right,
                              double check_s) const;
  bool IsLeftLaneBoundaryCurb(const double s);

  bool NoGreenInJunction() const { return no_green_in_junction_; }
  void SetNoGreenInJunction(bool no_green_in_junction) {
    no_green_in_junction_ = no_green_in_junction;
  }
  bool IsOnLane(const Obstacle& obs) const;

  void SetEStopStatus(const EStopStatus& estop_status) {
    estop_status_ = estop_status;
  }
  void SetEStopStatus(const bool is_estop, const std::string& reason) {
    estop_status_.is_estop = is_estop;
    estop_status_.reason = reason;
  }
  const EStopStatus& GetEStopStatus() const { return estop_status_; }

  void SetADCLocatedInMergeLane(const bool is_in_merge_lane) {
    adc_located_in_merge_lane_ = is_in_merge_lane;
  }
  bool IsADCLocatedInMergeLane();

  bool IsADCLocatedInAuxiliaryRoad();
  bool IsADCInCurbArea();

  double GetAdcHeadingDiffWithRefLine();

  bool HasLeftNeighborRoutingLane(const std::string& direction,
                                  PlanningContext* planning_context);

  bool IsInNearJunctionLaneBorrowScenario(PlanningContext* planning_context);

  void SetRiskShiftResult(const double shift_value, const double confidence);
  const std::pair<double, double>& GetRiskShiftResult() const {
    return risk_shift_result_;
  }
  bool GetIsAtTurn() { return is_at_turn_; }
  bool GetIsAtLeftTurn() { return is_at_left_turn_; }
  bool GetIsAtRightTurn() { return is_at_right_turn_; }
  const bool IsLeftLaneChange() const;
  const bool IsRightLaneChange() const;

  const double GetMaxKappaFabs() const;
  /*
   * @brief get lane width by s
   * @param s  position
   * @return (left_width, right_width)
   */
  std::pair<double, double> GetLaneWidthByS(const double s) const;

  /*
   * @brief get reference line heading by s
   * @param s  position
   * @return heading
   */
  double GetHeadingByS(const double s) const;

 private:
  void InitFirstOverlaps();

  bool CheckChangeLane() const;

  void SetTurnSignalBasedOnLaneTurnType(
      common::VehicleSignal* vehicle_signal) const;

  void ExportVehicleSignal(common::VehicleSignal* vehicle_signal) const;

  bool IsIrrelevantObstacle(const Obstacle& obstacle);

  bool IsCautionObstacleIrrelevant(const Obstacle& obstacle,
                                   bool is_obs_on_ref_lane,
                                   bool is_adc_on_ref_lane);

  void MakeDecision(DecisionResult* decision_result,
                    PlanningContext* planning_context) const;

  int MakeMainStopDecision(DecisionResult* decision_result) const;

  void MakeMainMissionCompleteDecision(DecisionResult* decision_result,
                                       PlanningContext* planning_context) const;

  void MakeEStopDecision(DecisionResult* decision_result) const;

  void SetObjectDecisions(ObjectDecisions* object_decisions) const;

  bool AddObstacleHelper(const std::shared_ptr<Obstacle>& obstacle);

  bool GetFirstOverlap(const std::vector<hdmap::PathOverlap>& path_overlaps,
                       hdmap::PathOverlap* path_overlap);

  bool GetFirstTrafficLightJunction(hdmap::PathOverlap* path_overlap_junction);

  bool GetFirstGateArea(hdmap::PathOverlap* path_overlap_gate_area);

  bool GetFirstDriveArea(hdmap::PathOverlap* path_overlap_drive_area);

  void GetAdcBasedLaneWidth();

  void CheckIsClearToChangeLane();

  void ComputeRemainLaneChangeTime();

  bool HysteresisFilter(const double obstacle_distance,
                        const double safe_distance,
                        const double distance_buffer,
                        const bool is_obstacle_blocking) const;
  bool IsAdcOnLane(const SLBoundary& sl_boundary) const;

  double LaneChangeLateralTTC(const Obstacle& obstacle) const;
  bool GetObstacleSLBoundary(const Obstacle& obstacle,
                             SLBoundary* const perception_sl) const;
  bool SetTurnSignalBasedTurnType(common::VehicleSignal* vehicle_signal) const;
  void SetTurnSignalBasedPath(const FrenetFramePath& frenet_path,
                              common::VehicleSignal* vehicle_signal) const;
  bool CheckToChangeLane();
  void AddObsLateralAndLonSpeed(const Obstacle& obstacle,
                                const planning::SLBoundary& adc_sl);
  void PreBuildObstacleSLboudnary(const Obstacle& obstacle,
                                  SLBoundary* perception_sl);
  bool CheckIsNeedUsePreSlboundary(const Obstacle& obstacle,
                                   const SLBoundary& perception_sl);
  bool GetObstacleSLBoundary(const Obstacle& obstacle,
                             SLBoundary* const perception_sl,
                             bool is_need_prebuild) const;
  bool IsCautionBicycle(const Obstacle& obstacle, bool is_adc_on_ref_lane,
                        bool is_obs_on_ref_lane);

 private:
  static std::unordered_map<std::string, bool> junction_right_of_way_map_;
  const common::VehicleState vehicle_state_;
  const common::TrajectoryPoint adc_planning_point_;
  ReferenceLine reference_line_;

  /**
   * @brief this is the number that measures the goodness of this reference
   * line. The lower the better.
   */
  double cost_ = 0.0;

  bool is_drivable_ = true;
  bool traffic_light_request_ = false;
  bool is_near_traffic_light_stop_line_ = false;
  bool lane_borrow_fail_request_ = false;
  bool is_low_start_up_ = false;
  PathDecision path_decision_;

  Obstacle* blocking_obstacle_;

  std::vector<PathBoundary> candidate_path_boundaries_;
  std::vector<PathData> candidate_path_data_;

  PathData path_data_;
  PathData fallback_path_data_;
  SpeedData speed_data_;

  DiscretizedTrajectory discretized_trajectory_;

  RSSInfo rss_info_;

  EStopStatus estop_status_;

  /**
   * @brief SL boundary of stitching point (starting point of plan trajectory)
   * relative to the reference line
   */
  SLBoundary adc_sl_boundary_;

  planning_internal::Debug debug_;
  LatencyStats latency_stats_;

  hdmap::RouteSegments lanes_;

  /**
   * @brief one of four body corner points in lane
   */
  bool is_on_reference_line_ = false;
  bool is_adc_in_common_junction_ = false;
  bool is_adc_in_traffic_light_junction_ = false;
  bool is_adc_in_gate_area_ = false;
  bool is_adc_in_drive_area_ = false;
  /**
   * @brief whole adc box in lane
   */
  bool is_adc_located_in_lane_ = false;
  bool is_adc_cover_in_lane_ = false;

  /**
   * @brief adc center point in lane
   */
  bool is_adc_center_in_lane_ = false;

  bool is_adc_posture_straight_ = false;

  bool is_path_lane_borrow_ = false;

  bool no_green_in_junction_ = false;

  bool is_auxiliary_road_ = false;

  ADCTrajectory::RightOfWayStatus status_ = ADCTrajectory::UNPROTECTED;

  double offset_to_other_reference_line_ = 0.0;

  double priority_cost_ = 0.0;

  double adc_longitudinal_velocity_ = 0.0;
  double adc_lateral_velocity_ = 0.0;

  PlanningTarget planning_target_;

  ADCTrajectory::TrajectoryType trajectory_type_ = ADCTrajectory::UNKNOWN;

  /**
   * Overlaps encountered in the first time along the reference line in front
   * of the vehicle
   */
  std::vector<std::pair<OverlapType, hdmap::PathOverlap>>
      first_encounter_overlaps_;

  /**
   * @brief Data generated by speed_bounds_decider for constructing st_graph
   * for different st optimizer
   */
  StGraphData st_graph_data_;
  std::vector<std::string> slowdown_obstacles_;
  std::vector<std::string> cross_slowdown_obstacles_;
  std::vector<std::string> reverse_slowdown_obstacles_;
  std::vector<std::string> ref_obstacle_id_;
  common::VehicleSignal vehicle_signal_;
  SpeedLimit speed_limit_;

  std::vector<std::string> keep_approaching_obstacles_;
  std::vector<std::string> ignore_obstacles_;
  std::vector<std::pair<double,bool>> path_point_in_turn_vector_;;
  bool is_common_junction_ = false;

  // Right turn signal needs to be activated, default false
  bool is_need_turn_right_ = false;
  bool is_near_turn_ = false;
  bool is_in_diagonal_road_ = false;
  bool is_need_to_backward_ = false;
  double diaggonal_road_heading_ = 0.0;
  bool is_need_diagonal_ = false;
  bool is_tiny_adjust_type_ = false;
  bool is_new_stacker_response_ = false;
  PassStackerRequest need_notice_stacker_;
  bool is_straight_path_ = false;
  bool enable_shrink_collision_buffer_ = false;
  PassStackerRequest pass_stacker_request_;
  std::unordered_map<std::string, SLBoundary> igv_boundary_map_;
  std::unordered_map<std::string, SLBoundary> stacker_boundary_map_;
    std::unordered_map<std::string, SLBoundary> wheelcrane_boundary_map_;
  planning::Barrier barrier_;
  bool west_in_ = false;
  bool west_out_ = false;
  bool east_in_ = false;
  bool east_out_ = false;
  bool is_passing_gate_= false;
  double init_point_heading_ = 0.0;
  double request_end_l_ = 0.0;
  bool is_need_reset_borrow_state_ = false;

  double cruise_speed_ = 0.0;

  bool is_need_to_speed_limit_ = false;
  bool path_reusable_ = false;
  bool lane_change_path_reusable_ = false;
  bool is_in_high_road_right_lane_ = false;
  bool is_reach_turn_lane_ = false;
  bool is_need_to_turn_ = false;
  bool is_direct_turn_lane_ = false;
  bool at_left_turn_ = false;
  bool at_right_turn_ = false;
  bool path_turn_right_ = false;
  bool path_turn_left_ = false;
  bool is_direct_turn_ = false;
  bool is_need_slow_breaking_ = false;
  double min_stop_decel_ = -4.0;
  double min_stop_s_ = std::numeric_limits<double>::max();

  bool is_need_to_slower_breaking_for_target_speed_ = false;
  bool is_need_to_slower_breaking_for_approach_obs_ = false;

  double target_speed_for_slower_breaking_ = 0.0;
  double target_accel_for_slower_breaking_ = 0.0;
  double target_speed_for_approach_obs_slower_breaking_ = 0.0;
  double target_accel_for_approach_obs_slower_breaking_ = 0.0;

  century::routing::LaneWaypoint routing_end_;

  double end_state_l_;

  hdmap::Lane_LaneType lane_type_ = hdmap::Lane::CITY_DRIVING;

  static size_t turn_left_count_;
  static size_t turn_right_count_;
  static int turn_left_continue_count_;
  static int turn_right_continue_count_;

  /**
   * @brief land width in (adc_start_s + adc_end_s)*0.5, first is left width,
   * second is right width
   */
  std::pair<double, double> lane_width_base_on_adc_center_ = {0.0, 0.0};

  bool is_clear_to_change_lane_ = false;
  double remain_lane_change_time_ = 0.0;

  bool has_neighbor_forward_lane_ = false;

  bool has_set_remain_distance_for_back_ = false;
  double remain_distance_for_back_ = std::numeric_limits<double>::max();

  bool has_set_remain_distance_to_solid_line_ = false;
  double remain_distance_to_solid_line_ = std::numeric_limits<double>::max();

  bool has_set_remain_distance_to_junction_ = false;
  double remain_distance_to_junction_ = std::numeric_limits<double>::max();

  bool has_set_remain_distance_to_signal_ = false;
  double remain_distance_to_signal_ = std::numeric_limits<double>::max();

  bool has_set_remain_distance_to_turn_lane_ = false;
  double remain_distance_to_turn_lane_ = std::numeric_limits<double>::max();

  bool has_set_remain_distance_to_curb_area_ = false;
  double remain_distance_to_curb_area_ = std::numeric_limits<double>::max();

  bool has_set_remain_distance_to_left_routing_end_ = false;
  double remain_distance_to_left_routing_end_ =
      std::numeric_limits<double>::max();

  bool has_set_remain_distance_to_right_routing_end_ = false;
  double remain_distance_to_right_routing_end_ =
      std::numeric_limits<double>::max();

  bool adc_located_in_merge_lane_ = false;

  bool adc_located_in_auxiliary_road_ = false;

  double heading_diff_between_adc_and_ref_ = 0.0;
  bool has_set_adc_heading_diff_with_ref_ = false;

  std::pair<double, double> risk_shift_result_ = {0.0, 0.0};

  /**
   * @brief collect all obstacles that may affect the lane change safety
   */
  std::vector<const Obstacle*> risk_obstacles_;

  bool is_at_turn_ = false;
  bool is_at_left_turn_ = false;
  bool is_at_right_turn_ = false;
  // double drive_area_start_s_=0.0;
  // double drive_area_end_s_=0.0;

  DISALLOW_COPY_AND_ASSIGN(ReferenceLineInfo);
};

}  // namespace planning
}  // namespace century
