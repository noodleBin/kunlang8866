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

#pragma once

#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/map/pnc_map/path.h"
#include "modules/common/util/point_factory.h"

namespace century {
namespace planning {
namespace util {

enum OffsetDirection { NONE = 0, LEFT, RIGHT };

/**
 * @brief Coordinate type
 */
enum class CoordinateSystem {
    RFU = 0,  // RFU
    FLU = 1   // FLU
};

enum MovingObstacleType {
  NO_MOVING = 0,
  // obstacle straight forward moving in the same ADC direction
  STRAIGHT_FORWARD = 1,
  // obstacle forward moving from left to right based on ADC heading
  LEFT_FORWARD = 2,
  // obstacle forward moving from right to left based on ADC heading
  RIGHT_FORWARD = 3,
  // obstacle lateral moving from left to right based on ADC heading
  LEFT_CROSSING = 4,
  // obstacle lateral moving from right to left based on ADC heading
  RIGHT_CROSSING = 5,
  // obstacle reverse moving from left to right based on ADC heading
  LEFT_REVERSING = 6,
  // obstacle reverse moving from right to left based on ADC heading
  RIGHT_REVERSING = 7,
};

common::math::Box2d ConstructionEgoBox(const common::math::Vec2d& ego_pt,
                                       const double& ego_heading);

bool IsVehicleCollisionWithElectricFence(
    const century::common::VehicleState& vehicle_state,
    const double& lat_velocity, const double& lon_velocity,
    const bool is_auto_state);
double GetElectricFenceLatDynamicBuff(const double& lat_velocity,
                                    const double& lon_velocity,
                                    double& lat_buff, double& lon_buff);

bool GetNearestPointFromRoutingLane(
    const common::VehicleState& state,
    const century::routing::RoutingRequest& rouring_request,
    common::PointENU& nearest_point, double& nearest_heading);

bool offsetPoint(double x, double y, double dx, double dy,
                 double offsetDistance, OffsetDirection direction,
                 std::pair<double, double>& offset_point,
                 CoordinateSystem coordSystem);

std::pair<double, double> offsetPoint(
    double x, double y, double dx, double dy, double offsetDistance,
    OffsetDirection direction,
    CoordinateSystem coordSystem = CoordinateSystem::RFU);

// brief: if first point ahead of second point, return true, else return false
bool CompareTwoPointsWithReference(const ReferenceLineInfo& reference_line_info,
                                   const common::math::Vec2d first_point,
                                   const common::math::Vec2d second_point);

void DealWithStopObstacle(
    const century::planning::ReferenceLineInfo& reference_line_info,
    const planning::ObjectDecision& decision,
    const planning::ObjectDecisionType& object_decision_type,
    const common::VehicleState& vehicle_state,
    const double& stop_count_threshold_for_rescue,
    const double& reference_end_s_threshold, size_t& rescue_stop_cnt,
    bool& is_stop_by_obstacle);

bool IsNeedIgnore(
    const ReferenceLineInfo& reference_line_info,
    const century::common::VehicleState& vehicle_state,
    const Obstacle* obstacle, const double ref_end_threshold);

bool IsIgv(const ReferenceLineInfo& reference_line_info,
           const Obstacle* decision_obstacle);

bool IsReachReferenceLineEnd(const ReferenceLineInfo& reference_line_info,
                             const century::common::VehicleState& vehicle_state,
                             const double end_threshold);

bool IsJunctionContainAdc(const century::common::VehicleState& vehicle_state,
                          const hdmap::JunctionInfo& junction_info);

bool IsAdcInJunction(common::PointENU adc_pt_enu,
                     const century::common::VehicleState vehicle_state,
                     century::hdmap::Junction::Type target_junction_type);

bool IsVehicleStateValid(const century::common::VehicleState& vehicle_state);

bool IsDifferentRouting(const century::routing::RoutingResponse& first,
                        const century::routing::RoutingResponse& second);

double GetADCStopDeceleration(
    century::common::VehicleStateProvider* vehicle_state,
    const double adc_front_edge_s, const double stop_line_s);

bool CheckStopSignOnReferenceLine(const ReferenceLineInfo& reference_line_info,
                                  const std::string& stop_sign_overlap_id);

bool CheckTrafficLightOnReferenceLine(
    const ReferenceLineInfo& reference_line_info,
    const std::string& traffic_light_overlap_id);

bool CheckInsidePnCJunction(const ReferenceLineInfo& reference_line_info);

void GetFilesByPath(const boost::filesystem::path& path,
                    std::vector<std::string>* files);

hdmap::Lane_LaneType GetLaneTypeAt(const ReferenceLine& reference_line,
                                   const double s);

MovingObstacleType GetMovingObstacleType(
    const Obstacle* obstacle, const common::VehicleState& vehicle_state,
    const ReferenceLine& reference_line);
void SetMovingObstacleType(const double theta_diff, const double buffer_radian,
                           const double forward_radian,
                           MovingObstacleType* const type);
std::string GetMovingObstacleTypeName(MovingObstacleType type);

bool IsNewRouting(const PlanningContext* context);
bool IsLaneBorrow(const PlanningContext* context);
bool IsLaneChange(const PlanningContext* context);
bool IsOvertake(const PlanningContext* context);
bool IsNarrowAreaMode(const PlanningContext* context);
bool IsMixedTraffic(const PlanningContext* context);
bool IsAdcInJunction(const PlanningContext* context);
bool IsNeedLoosenPathConstrains(const PlanningContext* context);

void SetFallBackLastFramePath(
    const std::vector<common::PathPoint>& last_discretized_path_points,
    const std::vector<common::FrenetFramePoint>& last_frenet_frame_points,
    const common::SLPoint& last_sl_point, const double diff_s,
    std::vector<common::FrenetFramePoint>* const frenet_path,
    const common::SLPoint& sl_point_of_match_point_in_last_frame,
    DiscretizedPath* discretized_path, double* time, double* loop);
double GetFrontNearestObstaclesDistance(
    const std::vector<Obstacle>& path_obstacles,
    const std::vector<Obstacle> obstacles,
    const std::vector<common::FrenetFramePoint>& frenet_path,
    const double diff_s, const SLBoundary& adc_sl_boundary,
    const common::TrajectoryPoint& init_point,
    const common::SLPoint& sl_point_of_match_point_in_last_frame,
    const common::VehicleParam& vehicle_param);
bool SetFallBackSpeedData(const common::TrajectoryPoint& init_point,
                          const double nearest_distance,
                          SpeedData* const speed_data, const bool& is_sim);
bool CombinePathAndSpeedProfile(
    const double relative_time, const double start_s,
    const DiscretizedPath& path_data, const SpeedData& speed_data,
    DiscretizedTrajectory* const discretized_trajectory);

bool CompareTwoStringIsEqual(const std::string& string1,
                             const std::string& string2);

double GetObstacleConfidence(const double s);

bool IsLongitudinalOverlap(const SLBoundary& sl_boundary1,
                           const SLBoundary& sl_boundary2);

bool IsLateralOverlap(const SLBoundary& sl_boundary1,
                      const SLBoundary& sl_boundary2);

bool IsJunctionContainAdc(const century::common::VehicleState& vehicle_state,
                          const hdmap::JunctionInfo& junction_info);

bool IsGateJunctionContainAdc(
    common::PointENU adc_pt_enu,
    const century::common::VehicleState vehicle_state);

bool GetRemainDisToTurn(const ReferenceLineInfo& reference_line_info,
                        const double distance_threshold, const double step,
                        const double kappa_threshold, double& distance);

bool WithLeftObsSideBySide(const SLBoundary& obs_boundary,
                           const SLBoundary& adc_boundary);

/*
 * @brief check whether the reserved lane width for obstacle is sufficient
 *        to pass through
 * @param reference_line_info   reference line
 * @param obs_boundary   obstacle polygon (SL coordinate axis)
 * @param vehicle_width  vehicle width
 */
bool WithPassObsSideBySide(const ReferenceLineInfo& reference_line_info,
                           const SLBoundary& obs_boundary,
                           const double vehicle_width);

/*
 * @brief check whether value is within (start, end)
 * @param value  check value
 * @param start  minimum threshold
 * @param end    maximum threshold
 */
template <typename T>
bool WithinBound(T value, T start, T end) {
  if (start > end) {
    std::swap(start, end);
  }
  return (value >= start && value <= end);
}

/*
 * @brief check whether two ranges have common parts
 * @param (l1_s, l1_e) range1
 * @param (l2_s, l2_e) range2
 */
template <typename T>
bool HasCommon(T l1_s, T l1_e, T l2_s, T l2_e) {
  if (l1_s > l1_e) {
    std::swap(l1_s, l1_e);
  }
  if (l2_s > l2_e) {
    std::swap(l2_s, l2_e);
  }
  return (WithinBound(l1_s, l2_s, l2_e) || WithinBound(l1_e, l2_s, l2_e) ||
          WithinBound(l2_s, l1_s, l1_e) || WithinBound(l2_e, l1_s, l1_e));
}

}  // namespace util
}  // namespace planning
}  // namespace century
