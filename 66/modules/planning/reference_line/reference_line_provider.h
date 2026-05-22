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
 * @file reference_line_provider.h
 *
 * @brief Declaration of the class ReferenceLineProvider.
 */

#pragma once

#include <list>
#include <memory>
#include <queue>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "cyber/cyber.h"
#include "modules/common/util/factory.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/math/smoothing_spline/spline_2d_solver.h"
#include "modules/planning/reference_line/discrete_points_reference_line_smoother.h"
#include "modules/planning/reference_line/qp_spline_reference_line_smoother.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/spiral_reference_line_smoother.h"

/**
 * @namespace century::planning
 * @brief century::planning
 */
namespace century {
namespace planning {

/**
 * @class ReferenceLineProvider
 * @brief The class of ReferenceLineProvider.
 *        It provides smoothed reference line to planning.
 */
class ReferenceLineProvider {
 public:
  ReferenceLineProvider() = default;
  ReferenceLineProvider(
      const common::VehicleStateProvider* vehicle_state_provider,
      const hdmap::HDMap* base_map,
      const std::shared_ptr<relative_map::MapMsg>& relative_map = nullptr);

  /**
   * @brief Default destructor.
   */
  ~ReferenceLineProvider();

  bool UpdateRoutingResponse(const routing::RoutingResponse& routing);
  bool CanCreateReferenceLineForRouting(
      const routing::RoutingResponse& routing,
      const common::VehicleState& vehicle_state);

  void UpdateVehicleState(const common::VehicleState& vehicle_state);

  void UpdateLaneBorrowLaneId(const std::string& lane_borrow_lane_id);
  void UpdateOvertakeStatus(const OvertakeStatus::Status& overtake_status);
  void UpdateLaneChangeLaneId(const std::string& lane_change_lane_id);

  bool Start();

  void Stop();

  void Wait();

  bool IsStop();

  bool GetReferenceLines(std::list<ReferenceLine>* reference_lines,
                         std::list<hdmap::RouteSegments>* segments);

  double LastTimeDelay();

  std::vector<routing::LaneWaypoint> FutureRouteWaypoints();

  bool UpdatedReferenceLine() { return is_reference_line_updated_.load(); }

  void SetPlanningContext(PlanningContext* context);

 private:
  /**
   * @brief Use PncMap to create reference line and the corresponding segments
   * based on routing and current position. This is a thread safe function.
   * @return true if !reference_lines.empty() && reference_lines.size() ==
   *                 segments.size();
   **/
  bool CreateReferenceLine(std::list<ReferenceLine>* reference_lines,
                           std::list<hdmap::RouteSegments>* segments);

  /**
   * @brief store the computed reference line. This function can avoid
   * unnecessary copy if the reference lines are the same.
   */
  void UpdateReferenceLine(
      const std::list<ReferenceLine>& reference_lines,
      const std::list<hdmap::RouteSegments>& route_segments);

  void GenerateThread();
  void IsValidReferenceLine();
  void PrioritzeChangeLane(std::list<hdmap::RouteSegments>* route_segments);

  bool CreateRouteSegments(const common::VehicleState& vehicle_state,
                           std::list<hdmap::RouteSegments>* segments);

  bool IsReferenceLineSmoothValid(const ReferenceLine& raw,
                                  const ReferenceLine& smoothed) const;

  bool SmoothReferenceLine(const ReferenceLine& raw_reference_line,
                           ReferenceLine* reference_line);

  bool SmoothPrefixedReferenceLine(const ReferenceLine& prefix_ref,
                                   const ReferenceLine& raw_ref,
                                   ReferenceLine* reference_line);

  void GetAnchorPoints(const ReferenceLine& reference_line,
                       std::vector<AnchorPoint>* anchor_points) const;

  bool SmoothRouteSegment(const hdmap::RouteSegments& segments,
                          ReferenceLine* reference_line);

  /**
   * @brief This function creates a smoothed forward reference line
   * based on the given segments.
   */
  bool ExtendReferenceLine(const common::VehicleState& state,
                           hdmap::RouteSegments* segments,
                           ReferenceLine* reference_line);

  AnchorPoint GetAnchorPoint(const ReferenceLine& reference_line,
                             double s) const;

  bool GetReferenceLinesFromRelativeMap(
      std::list<ReferenceLine>* reference_lines,
      std::list<hdmap::RouteSegments>* segments);

  /**
   * @brief This function get adc lane info from navigation path and map
   * by vehicle state.
   */
  bool GetNearestWayPointFromNavigationPath(
      const common::VehicleState& state,
      const std::unordered_set<std::string>& navigation_lane_ids,
      hdmap::LaneWaypoint* waypoint);

  bool Shrink(const common::SLPoint& sl, ReferenceLine* ref,
              hdmap::RouteSegments* segments);

  std::string GetCurrentPathId(
      const std::list<hdmap::RouteSegments>& route_segments) const;
  bool SmoothPreparation(const hdmap::RouteSegments& segment_properties,
                         hdmap::RouteSegments* shifted_segments,
                         const ReferenceLine& prev_ref,
                         const hdmap::RouteSegments& prev_segment,
                         ReferenceLine* reference_line,
                         hdmap::RouteSegments* segments);
  bool GetCurrentLane(const hdmap::HDMap& hdmap, std::string* adc_lane_id,
                      uint32_t* adc_lane_priority,
                      std::vector<std::string>* left_neighbor_lane_ids,
                      std::vector<std::string>* right_neighbor_lane_ids,
                      hdmap::LaneWaypoint* adc_lane_way_point,
                      std::list<ReferenceLine>* reference_lines,
                      std::list<hdmap::RouteSegments>* segments);
  void GetNeighborLane(bool is_lane_change_needed,
                       const std::vector<std::string>& left_neighbor_lane_ids,
                       const std::vector<std::string>& right_neighbor_lane_ids,
                       const std::pair<std::string, uint32_t>& target_lane_pair,
                       const hdmap::LaneWaypoint adc_lane_way_point,
                       routing::ChangeLaneType* lane_change_type,
                       std::string* nearest_neighbor_lane_id);
  void GetSegmentForLaneChange(bool is_lane_change_needed,
                               const std::string& lane_id,
                               const std::string& adc_lane_id,
                               const routing::ChangeLaneType& lane_change_type,
                               const std::string& nearest_neighbor_lane_id,
                               hdmap::RouteSegments* segment);

 private:
  bool is_initialized_ = false;
  std::atomic<bool> is_stop_{false};

  std::unique_ptr<ReferenceLineSmoother> smoother_;
  ReferenceLineSmootherConfig smoother_config_;

  std::mutex pnc_map_mutex_;
  std::unique_ptr<hdmap::PncMap> pnc_map_;

  // Used in Navigation mode
  std::shared_ptr<relative_map::MapMsg> relative_map_;

  std::mutex vehicle_state_mutex_;
  common::VehicleState vehicle_state_;

  std::mutex lane_borrow_mutex_;
  std::string lane_borrow_lane_id_;

  std::mutex lane_change_mutex_;
  std::string lane_change_lane_id_;

  std::mutex overtake_status_mutex_;
  OvertakeStatus::Status overtake_status_ = OvertakeStatus::DEFAULT;

  std::mutex routing_mutex_;
  routing::RoutingResponse routing_;
  bool has_routing_ = false;

  std::mutex reference_lines_mutex_;
  std::list<ReferenceLine> reference_lines_;
  std::list<hdmap::RouteSegments> route_segments_;
  double last_calculation_time_ = 0.0;

  std::queue<std::list<ReferenceLine>> reference_line_history_;
  std::queue<std::list<hdmap::RouteSegments>> route_segments_history_;

  std::future<void> task_future_;

  std::atomic<bool> is_reference_line_updated_{true};

  const common::VehicleStateProvider* vehicle_state_provider_ = nullptr;

  PlanningContext* context_;
};

}  // namespace planning
}  // namespace century
