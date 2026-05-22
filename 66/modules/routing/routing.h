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

#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/map/proto/map_junction.pb.h"
#include "modules/routing/proto/routing_config.pb.h"
#include "modules/routing/proto/routing_point.pb.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/routing/common/routing_math.h"
#include "modules/routing/core/navigator.h"

namespace century {
namespace routing {

enum FillLaneInfoType {
  FRIST = 0u,
  ALL = 1u,
};
enum IndexWaypoint {
  INDEX_NONE = 0u,
  INDEX_FRIST = 1u,
  INDEX_BACK = 2u,
  INDEX_ALL = 3u,
};
class Routing {
  // friend class RoutingTestBase;
 public:
  Routing();

  /**
   * @brief module name
   */
  std::string Name() const;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  century::common::Status Init();

  /**
   * @brief module start function
   * @return start status
   */
  century::common::Status Start();

  /**
   * @brief destructor
   */
  virtual ~Routing() = default;

  bool Process(const std::shared_ptr<RoutingRequest> &routing_request,
               RoutingResponse *const routing_response,
               std::vector<RoutingResponse> &routing_responses);

 private:
  // Helper struct for SetReRoutingPoint
  struct RoutingState {
    bool is_close_boundary_point = false;
    bool is_in_J4_east = false;
    bool is_in_near_J4_east = false;
  };
 private:
  void InitParameters(const std::shared_ptr<RoutingRequest> &routing_request);
  void ProjectLastWaypointForRelocation(RoutingRequest *const routing_request);
  void FilterWaypointsForReachStacker(RoutingRequest *const routing_request);
  bool GetWaypointInfoForParking(
      const int index, const hdmap::ParkingSpaceInfoConstPtr parking_space,
      LaneWaypoint &waypoint_info);
  bool GetWaypointInfo(
      const int index,
      const std::vector<std::shared_ptr<const hdmap::LaneInfo>> &lanes,
      LaneWaypoint &waypoint_info, bool is_additional_lane,
      bool set_lane_point = false);
  void SetReRoutingPoint(RoutingRequest *const fixed_request,
                         const std::string_view junction_type);
  bool SetReRoutingPoint(RoutingRequest *const fixed_request,
                         const int junction_type, bool is_human_shaped);
  void SetReRoutingPointForDemonstrate(RoutingRequest *const fixed_request);
  bool IsNeedLanechangeFirst(
      std::vector<RoutingRequest> *const fixed_routing_requests);
  void IsNeedTwoRouting(RoutingRequest *const fixed_request);
  bool DetectHuamanShapedRouting(RoutingRequest *const fixed_request);
  void DetectJunctionAreaRouting(RoutingRequest *const fixed_request);
  void SetReverseReRoutingPoint(RoutingRequest *const fixed_request);
  bool GenerateAdditionalRoutingRequests(const RoutingRequest &target_request);
  void ApplyHeadingAndPushRequest(RoutingRequest &new_request,
      LaneWaypoint *first_way_point, double adc_heading, double lane_heading);
  void GenerateFourWaypointRoutes(const LaneWaypoint &adc_waypoint,
      const std::vector<LaneWaypoint> &vec1,
      double adc_heading, double lane_heading,
      const RoutingRequest &target_request);
  void FillLaneInfoIfMissing(
      const RoutingRequest &routing_request,
      std::vector<RoutingRequest> *const fixed_routing_request);
  bool FillTwoWayTaskByOverlapPoint(
      const RoutingRequest &routing_request,
      std::vector<RoutingRequest> *const fixed_routing_request);
  bool FillParkingID(RoutingResponse *const routing_response);

  void SetErrorCode(const common::ErrorCode &error_code_id,
                    const std::string &error_string,
                    common::StatusPb *const error_code);

  double NormalRoutingSearchShortRoute(
      const std::vector<RoutingRequest> &fixed_requests,
      RoutingResponse *const routing_response, double *const routing_min_lenght,
      bool is_need_lanechange_first);

  void AllNormalRoutingSearchShortRoute(
      double &normal_min_routing_length, double &routing_min_lenght,
      RoutingResponse *const routing_response,
      std::vector<RoutingResponse> &routing_responses);

  double CalculateTwoSegmentRoutingLength(
      const RoutingRequest &four_point_request,
      RoutingResponse *const routing_response, double *const routing_min_lenght,
      double *const first_segment_length = nullptr);

  void LogRoutingRequestsInfo(const std::vector<RoutingRequest> &routing_requests);

  bool JudgeInPlayStreet(const RoutingRequest &routing_request);
  // TODO(yt): Refactoring code
  bool IsRoutingFirstPointHeadingReverse(
      const std::shared_ptr<RoutingRequest> &routing_request);
  bool FillFristOrAllPointLaneInfo(const FillLaneInfoType &type,
                                   const RoutingRequest &routing_request,
                                   RoutingRequest *const fixed_request);

  bool FillEndPointLaneInfoForTinyAdjustment(
      const RoutingRequest &routing_request,
      RoutingRequest *const fixed_request);
  bool FillEndPointLaneInfoForFirstBox(const RoutingRequest &routing_request,
                                       RoutingRequest *const fixed_request);
  bool FillEndPointLaneInfoForProjection(const RoutingRequest &routing_request,
                                         RoutingRequest *const fixed_request);
  bool IsExcludeThisRoutingRequest(const RoutingRequest &fixed_routing_request,
                                   const RoutingResponse &routing_response,
                                   const double &temp_routing_min_lenght);
  bool IsLaneChangeDistanceClose(const RoutingRequest &fixed_routing_request,
                                 const RoutingResponse &routing_response,
                                 const double &temp_routing_min_lenght);
  bool IsSpecialAreasOnlyuseUturn(const RoutingResponse &routing_response);
  bool IsUseReverseTrajectory(const double &routing_min_lenght,
                              const RoutingRequest &routing_request,
                              RoutingResponse *const routing_response);
  bool IsReverseType(const RoutingResponse &routing_response);
  bool IsLaneChangeLengthValid(const RoutingResponse &routing_response,
                               int huaman_shaped_level);
  void FixedRoutingRequestWaypointTurnAround();
  void AddBlacklistedLanes(RoutingRequest *const fixed_request);
  void ApplyBlacklistRule(RoutingRequest *const request,
                          const std::string_view block_name,
                          bool is_only_add_reverse);
  void ApplyBlacklistRuleForRightSide(RoutingRequest *const request);
  void ApplyBlacklistRuleForD7J4W(RoutingRequest *const request);
  void RunLoopRunning(const std::shared_ptr<RoutingRequest> &routing_request);
  void HandingTaskInvolvingDeadEndRoad(RoutingRequest *const fixed_request);
  bool IsDeadEndRoad(RoutingRequest *const fixed_request);
  void SpecialTypeForwardEndPoint(RoutingResponse *const routing_response);
  bool FillEndPointLaneInfoForFrontTrajection(
      RoutingRequest *const fixed_request);
  bool DoubleBoxOperationForPositionOfFirstBox(
      RoutingResponse *const routing_response);
  bool IsBeingRerouing(const RoutingRequest &fixed_request);
  bool IsBackCar(const LaneWaypoint &lane_waypoint);
  bool CalculateCostAndCheckReverse(
      const double &routing_min_lenght, RoutingResponse &routing_response,
      std::vector<RoutingResponse> &routing_responses);
  void PrioritizeResponsesByCost(
      const RoutingResponse &routing_response,
      std::vector<RoutingResponse> &routing_responses);
  void LogInfo();
  bool FindBestMatchingLane(
      const int index,
      const std::vector<std::shared_ptr<const hdmap::LaneInfo>> &lanes,
      LaneWaypoint &waypoint_info, bool is_additional_lane);
  bool IsLaneHeadingAcceptable(
      const int index, const std::shared_ptr<const hdmap::LaneInfo> &lane,
      double s, double diff_heading, const LaneWaypoint &waypoint_info);
  double AdjustDiffHeadingForNoTurn(
      const std::shared_ptr<const hdmap::LaneInfo> &lane, double diff_heading);
  void AddAdditionalLaneWaypoint(
      const int index, const std::shared_ptr<const hdmap::LaneInfo> &lane,
      double s, const LaneWaypoint &waypoint_info);
  bool HandleNearestLane(const int index, LaneWaypoint &waypoint_info,
                         bool is_additional_lane);
  void CollectNearbyLanes(const int index, const LaneWaypoint &waypoint_info,
                          const std::shared_ptr<const hdmap::LaneInfo> &lane);
  void AddNearbyLaneWaypoints(
      const int index,
      const std::vector<std::shared_ptr<const hdmap::LaneInfo>> &nearby_lanes,
      const common::PointENU &projection_point,
      const LaneWaypoint &waypoint_info);
  void SnapWaypointToLane(LaneWaypoint &waypoint_info);
  common::PointENU CreatePointENU(double x, double y);
  bool HandleReroutingWithExistingPoints(RoutingRequest *const fixed_request,
                                        LaneWaypoint *second_waypoint);
  bool ProcessJunctionRouting(const int junction_type,
                             const LaneWaypoint &front_waypoint,
                             LaneWaypoint *second_waypoint,
                             bool is_two_point_distance_far,
                             double temp_human_shape_distance,
                             RoutingState *state);
  bool ProcessJ4_1Routing(const LaneWaypoint &front_waypoint,
                         LaneWaypoint *second_waypoint,
                         bool is_two_point_distance_far,
                         double temp_human_shape_distance,
                         RoutingState *state);
  bool ProcessT4T5T8Routing(const LaneWaypoint &front_waypoint,
                           LaneWaypoint *second_waypoint,
                           bool is_two_point_distance_far,
                           double temp_human_shape_distance,
                           RoutingState *state);
  void ProcessTargetPointWest(LaneWaypoint *second_waypoint,
                             double temp_human_shape_distance,
                             RoutingState *state);
  void ProcessEgoPointEast(const LaneWaypoint &front_waypoint,
                          bool is_two_point_distance_far,
                          double temp_human_shape_distance,
                          RoutingState *state);
  void ProcessT4T5T8West(const LaneWaypoint &front_waypoint,
                        bool is_two_point_distance_far,
                        RoutingState *state);
  void ProcessT4T5T8East(LaneWaypoint *second_waypoint,
                        double temp_human_shape_distance,
                        RoutingState *state);
  void AddBoundaryPoint(routing::LocalRoutingType type,
                       const std::string &line_id, bool move_along,
                       bool is_reverse);
  void UpdateSecondWaypointAndRequest(RoutingRequest *const fixed_request,
                                     LaneWaypoint *second_waypoint,
                                     const RoutingState &state);
  void SetFinalRoutingState(RoutingRequest *const fixed_request);

 private:
  std::unique_ptr<Navigator> navigator_ptr_;
  std::unique_ptr<century::routing::RoutingMath> routing_math_ptr_;
  const hdmap::HDMap *hdmap_ = nullptr;
  bool is_heading_reverse_ = false;
  bool is_start_point_overlap_calculate_ = false;
  bool is_need_lanechange_first_ = false;
  bool is_human_shaped_ = false;
  bool is_front_box_ = false;
  bool is_backward_ = false;
  int in_special_areas_ = 0;
  bool is_use_reverse_trajectory_ = false;
  double veh_heading_;
  bool is_add_responses_ = true;
  bool is_need_shortest_route_ = true;
  bool is_special_area_no_rerouting_ = false;
  bool is_add_rerouting_lane_ = false;
  bool is_lane_change_detector_ = true;
  bool is_huaman_shaped_driver = false;
  common::monitor::MonitorLogBuffer monitor_logger_buffer_;
  std::vector<RoutingRequest> fixed_routing_request_;
  common::PointENU next_point_, last_point_;
  std::unordered_map<int, std::vector<LaneWaypoint>>
      additional_lane_waypoint_map_;
  std::vector<LaneWaypoint> next_point_lane_waypoint_;
  std::vector<
      std::pair<routing::LocalRoutingType, std::vector<common::PointENU>>>
      human_shape_points_;
};

}  // namespace routing
}  // namespace century
