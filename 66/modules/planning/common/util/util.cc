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

#include "modules/planning/common/util/util.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/planning_gflags.h"

namespace century {
namespace planning {
namespace util {

using century::common::SLPoint;
using century::common::VehicleState;
using century::common::math::Polygon2d;
using century::cyber::Clock;
using century::hdmap::PathOverlap;
using century::routing::RoutingResponse;
using century::routing::RoutingRequest;

namespace {
constexpr double kDegrees = 90.0;
constexpr double kFallBackMaxPathLength = 200.0;
constexpr double kFlags_fallback_boundary_buffer = 0.3;
constexpr double kFlags_fallback_boundary_buffer_for_dynamic = 5;
constexpr double kFallBackVelocity = 4.0;
constexpr double kFallBackFrontObstacleBuffer = 1.0;
constexpr double kFallBackStopBuffer = 2.0;
constexpr double kFallBackMaxDeceleration = -3.5;
constexpr double kMinimum = 1e-8;
constexpr double kDecelerationThreshold = -0.1;
constexpr double kFallBackTLength = 6.0;
constexpr double kFallBackDeletaT = 0.2;
constexpr double kTinynum = 0.1;
constexpr double kEpsilon = 1E-2;
constexpr double kSideBySideLatThreshold = 2.8;
constexpr double kSafeLatThreshold = 0.5;
constexpr double kLongBuffer = 0.5;
constexpr double kLongMaxBuffer = 3.0;
constexpr double kLatBuffer = 0.2;
constexpr double kMaxLatBuffer = 0.8;
constexpr double kElectricFenceSearchRadius = 100.0;
constexpr double kNarrowPassLatBuffer = -0.175;
constexpr double kIgvlatBuffer = 0.3;
constexpr double kIgvlonBuffer = 1;
}  // namespace

common::math::Box2d ConstructionEgoBox(const common::math::Vec2d& ego_pt,
                                       const double& ego_heading) {
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  common::math::Vec2d ego_center_map_frame(
      (vehicle_param.front_edge_to_center() -
       vehicle_param.back_edge_to_center()) *
          0.5,
      (vehicle_param.left_edge_to_center() -
       vehicle_param.right_edge_to_center()) *
          0.5);
  ego_center_map_frame.SelfRotate(ego_heading);
  ego_center_map_frame.set_x(ego_pt.x() + ego_center_map_frame.x());
  ego_center_map_frame.set_y(ego_pt.y() + ego_center_map_frame.y());
  common::math::Box2d adc_box(ego_center_map_frame, ego_heading,
                              vehicle_param.length(), vehicle_param.width());
  return adc_box;
}

bool IsVehicleCollisionWithElectricFence(
    const century::common::VehicleState& vehicle_state,
    const double& lat_velocity, const double& lon_velocity,
    const bool is_auto_state) {
  const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  std::vector<hdmap::ElectricFenceInfoConstPtr> electric_fences;
  std::vector<hdmap::JunctionInfoConstPtr> junctions;
  bool is_in_drivable_area = is_auto_state ? false : true;
  bool is_collision = is_auto_state ? true : false;
  double x = vehicle_state.x();
  double y = vehicle_state.y();
  double heading = vehicle_state.heading();
  common::PointENU veh_point_enu;
  veh_point_enu.set_x(x);
  veh_point_enu.set_y(y);
  double LongBuffer = kLongBuffer;
  double LatBuffer = kLatBuffer;
  // AINFO << "LongBuffer:" << LongBuffer << ",LatBuffer:" << LatBuffer;
  if (0 == base_map_ptr->GetElectricFences(
               veh_point_enu, kElectricFenceSearchRadius, &electric_fences)) {
    if (FLAGS_allow_narrow_pass &&
        0 == base_map_ptr->GetJunctions(veh_point_enu, 0, &junctions)) {
      for (const auto& ptr_junction : junctions) {
        if (hdmap::Junction::DongJiaZhen_Gate ==
            ptr_junction->junction().type()) {
          // LongBuffer = .0;
          LatBuffer = kNarrowPassLatBuffer;
          break;
        }
      }
    }
    auto veh_box = common::VehicleConfigHelper::Instance()->GetBoundingBox(
        x, y, heading, LongBuffer, LatBuffer);
    double max_lat_buff, max_lon_buff;
    double lon_offset = GetElectricFenceLatDynamicBuff(
        lat_velocity, lon_velocity, max_lat_buff, max_lon_buff);
    AINFO << "lat_velocity:" << lat_velocity << ",max_lat_buff:" << max_lat_buff
          << ",max_lon_buff:" << max_lon_buff
          << ",lon_velocity:" << lon_velocity << ",lon_offset:" << lon_offset;
    // Check whether electric fence's polygon contain ADC bounding box.
    for (const auto& ele : electric_fences) {
      const auto& polygon = ele->polygon();  // ele->electric_fence().polygon();
      // AINFO << "electric fence id:" << ele->electric_fence().id().id();
      if (is_auto_state) {
        if (ele->electric_fence().type() == hdmap::ElectricFence::DRIVABLE) {
          if (polygon.Contains(Polygon2d(veh_box))) {
            // AINFO << "type:"
            //       <<
            //       ele->electric_fence().Type_Name(ele->electric_fence().type())
            //       << " VVVVVV";
            is_in_drivable_area = true;
          } else {
            AERROR << "electric fence id:" << ele->electric_fence().id().id()
                   << ",type:"
                   << ele->electric_fence().Type_Name(
                          ele->electric_fence().type())
                   << ",x" << x << ",y:" << y << ",heading:" << heading
                   << " Veh is not in drivable area";
            is_in_drivable_area = false;
            break;
          }
        } else if (ele->electric_fence().type() ==
                   hdmap::ElectricFence::NOT_DRIVABLE) {
          if (ele->electric_fence().area_type() ==
              hdmap::ElectricFence::RAILWAY) {
            auto temp_veh_box =
                common::VehicleConfigHelper::Instance()->GetBoundingBox(
                    x, y, heading, LongBuffer, max_lat_buff);
            if (polygon.HasOverlap(Polygon2d(temp_veh_box))) {
              is_collision = true;
              AERROR << "electric fence id:" << ele->electric_fence().id().id()
                     << ",type:"
                     << ele->electric_fence().Type_Name(
                            ele->electric_fence().type())
                     << ",area_type:"
                     << ele->electric_fence().AreaType_Name(
                            ele->electric_fence().area_type())
                     << ",x" << x << ",y:" << y << ",heading:" << heading
                     << ",max_lat_buff:" << max_lat_buff
                     << ",max_lon_buff:" << max_lon_buff
                     << ",lat_velocity:" << lat_velocity
                     << ",lon_velocity:" << lon_velocity
                     << ",lon_offset:" << lon_offset << " Veh is collision";
              break;
            } else {
              is_collision = false;
            }
          } else {
            if (polygon.HasOverlap(Polygon2d(veh_box))) {
              is_collision = true;
              AERROR << "electric fence id:" << ele->electric_fence().id().id()
                     << ",type:"
                     << ele->electric_fence().Type_Name(
                            ele->electric_fence().type())
                     << ",x" << x << ",y:" << y << ",heading:" << heading
                     << " Veh is collision";
              break;
            } else {
              // AINFO << "type:"
              //       <<
              //       ele->electric_fence().Type_Name(ele->electric_fence().type())
              //       << " VVVVVV";
              is_collision = false;
            }
          }
        }
      } else {
        if (ele->electric_fence().area_type() ==
            hdmap::ElectricFence::RAILWAY) {
          auto temp_veh_box =
              common::VehicleConfigHelper::Instance()->GetBoundingBox(
                  x, y, heading, max_lon_buff, lon_offset, max_lat_buff);
          if (polygon.HasOverlap(Polygon2d(temp_veh_box))) {
            is_collision = true;
            AERROR << "electric fence id:" << ele->electric_fence().id().id()
                   << ",type:"
                   << ele->electric_fence().Type_Name(
                          ele->electric_fence().type())
                   << ",area_type:"
                   << ele->electric_fence().AreaType_Name(
                          ele->electric_fence().area_type())
                   << ",x" << x << ",y:" << y << ",heading:" << heading
                   << ",max_lat_buff:" << max_lat_buff
                   << ",lat_velocity:" << lat_velocity
                   << ",max_lon_buff:" << max_lon_buff
                   << ",lon_velocity:" << lon_velocity
                   << ",lon_offset:" << lon_offset << " Veh is collision";
            break;
          } else {
            is_collision = false;
          }
        }
        // else if (ele->electric_fence().area_type() ==
        //              hdmap::ElectricFence::GLOBAL) {
        //     if (polygon.Contains(Polygon2d(veh_box))) {
        //       is_in_drivable_area = true;
        //     } else {
        //       AERROR << "electric fence id:" <<
        //       ele->electric_fence().id().id()
        //              << ",type:"
        //              << ele->electric_fence().Type_Name(
        //                     ele->electric_fence().type())
        //              << ",area_type:"
        //              << ele->electric_fence().AreaType_Name(
        //                     ele->electric_fence().area_type())
        //              << ",x" << x << ",y:" << y << ",heading:" << heading
        //              << " Veh is not in drivable area";
        //       is_in_drivable_area = false;
        //       break;
        //     }
        //   }
      }
    }
  } else {
    AERROR << "electric_fence_info is empty.";
  }
  return is_collision || !is_in_drivable_area;
}

/**
 * @param x x point
 * @param y y point
 * @param dx x direction verctor
 * @param dy y direction verctor
 * @param offsetDistance offset direction
 * @param direction offset direction
 * @param offset_point final offset_point
 * @param coordSystem coordSystem type: RFU or FLU
 * @return new_x and new_y after offset
 */
bool offsetPoint(double x, double y, double dx, double dy,
                 double offsetDistance, OffsetDirection direction,
                 std::pair<double, double>& offset_point,
                 CoordinateSystem coordSystem) {
  double length = std::sqrt(dx * dx + dy * dy);
  if (common::util::IsZero(length)) {
    offset_point = {x, y};
    return false;
  }

  double ux = dx / length;
  double uy = dy / length;
  double perpX, perpY;
  if (CoordinateSystem::RFU == coordSystem) {
    // RFU
    if (OffsetDirection::LEFT == direction) {
      perpX = -uy;
      perpY = ux;
    } else {
      perpX = uy;
      perpY = -ux;
    }
  } else {
    // FLU
    if (OffsetDirection::LEFT == direction) {
      // left offset: along Y axis
      perpX = 0.0;
      perpY = -1.0;
    } else {
      // right offset: along Y+ axis
      perpX = 0.0;
      perpY = 1.0;
    }

    if (std::abs(ux) > 1e-10 || std::abs(uy) > 1e-10) {
      double leftY = -1.0;
      double rightY = 1.0;

      if (OffsetDirection::LEFT == direction) {
        perpX = -uy * leftY;
        perpY = ux * leftY;
      } else {
        perpX = -uy * rightY;
        perpY = ux * rightY;
      }
    }
  }

  // calculate offset vector
  double offsetX = offsetDistance * perpX;
  double offsetY = offsetDistance * perpY;
  offset_point = {x + offsetX, y + offsetY};
  return true;
}

/**
 * @brief offset point along direction based on coordinate
 * @param x x point
 * @param y y point
 * @param dx x direction verctor
 * @param dy y direction verctor
 * @param offsetDistance offset direction
 * @param direction offset direction
 * @param coordSystem coordSystem type: RFU or FLU
 * @return offset new point
 */
std::pair<double, double> offsetPoint(
    double x, double y, double dx, double dy, double offsetDistance,
    OffsetDirection direction,
    CoordinateSystem coordSystem) {
  // arrordong to direction, adjust the sign of offsetDistance
  std::pair<double, double> offset_point{0.0, 0.0};
  offsetPoint(x, y, dx, dy, offsetDistance, direction, offset_point, coordSystem);
  return offset_point;
}

bool GetNearestPointFromRoutingLane(
    const common::VehicleState& state,
    const century::routing::RoutingRequest& rouring_request,
    common::PointENU& nearest_point, double& nearest_heading) {

  const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(base_map_ptr);
  double end_lane_s = 0.0;
  double end_lane_l = 0.0;
  hdmap::LaneInfoConstPtr car_lane;
  double distance = 0.0;
  common::PointENU lane_end_pose_enu =
      common::util::PointFactory::ToPointENU(state);
  base_map_ptr->GetNearestLane(lane_end_pose_enu, &car_lane, &end_lane_s,
                               &end_lane_l);
  if (nullptr == car_lane.get() ||
      (car_lane->id().id() != rouring_request.waypoint()
                                  .at(rouring_request.waypoint().size() - 1)
                                  .id())) {
    AERROR << "hdmap_->GetNearestLane failed with pos x: "
           << lane_end_pose_enu.x() << " y: " << lane_end_pose_enu.y();
    if (nullptr != car_lane.get()) {
      AERROR << "find_id is not equal to routing_id: " << car_lane->id().id()
             << ", routing_id: "
             << rouring_request.waypoint()
                    .at(rouring_request.waypoint().size() - 1)
                    .id();
    }
    return false;
  }
  AINFO << "get nearest lane id: " << car_lane->id().id()
        << ", routing_lane_id: "
        << rouring_request.waypoint()
               .at(rouring_request.waypoint().size() - 1)
               .id();

  double s = 0.0;
  double l = 0.0;
  if (!car_lane->GetProjection({lane_end_pose_enu.x(), lane_end_pose_enu.y()},
                               &s, &l)) {
    AERROR << "fail to get projection with point: (" << lane_end_pose_enu.x()
           << ", " << lane_end_pose_enu.y() << ")";
    return false;
  }
  static constexpr double kEpsilon = 0.5;
  ADEBUG << "s_in_target_lane: " << s << ", l: " << l;
  if (s > (car_lane->total_length() + kEpsilon) || (s + kEpsilon) < 0.0) {
    AERROR << "s is out of range: " << s;
    return false;
  }
  nearest_point = car_lane->GetNearestPoint(
      {lane_end_pose_enu.x(), lane_end_pose_enu.y()}, &distance);
  if (nearest_heading < 1e-10) {
    // default waypoint is 0
    // way point heading invaild, use nearest lane point's heading
    nearest_heading = car_lane->Heading(s);
  }
  AINFO << "point to nearest lane distance: " << distance
        << ", nearest_point: (" << nearest_point.x() << ", "
        << nearest_point.y() << ", " << nearest_heading * 180 / M_PI << ")";
  return true;
}

bool CompareTwoPointsWithReference(const ReferenceLineInfo& reference_line_info,
                                   const common::math::Vec2d first_point,
                                   const common::math::Vec2d second_point) {
  // used in QingDaoPort because this condition only happened here
  const auto &reference_line = reference_line_info.reference_line();
  common::SLPoint first_position_sl;
  common::SLPoint target_position_sl;
  if (!reference_line.XYToSL(first_point, &first_position_sl)) {
    AERROR << "Failed to convert first position to SL.";
    return false;
  }
  if (!reference_line.XYToSL(second_point, &target_position_sl)) {
    AERROR << "Failed to convert target position to SL.";
    return false;
  }
  AINFO << "first_point_s: " << first_position_sl.s()
        << ", target_point_s: " << target_position_sl.s();
  if (first_position_sl.s() > target_position_sl.s()) {
    return true;
  }
  return false;
}

void DealWithStopObstacle(
    const century::planning::ReferenceLineInfo& reference_line_info,
    const planning::ObjectDecision& decision,
    const planning::ObjectDecisionType& object_decision_type,
    const common::VehicleState& vehicle_state,
    const double& stop_count_threshold_for_rescue,
    const double& reference_end_s_threshold, size_t& rescue_stop_cnt,
    bool& is_stop_by_obstacle) {
  if (StopReasonCode::STOP_REASON_OBSTACLE !=
      object_decision_type.stop().reason_code()) {
    return;
  }
  rescue_stop_cnt++;
  if (rescue_stop_cnt >= stop_count_threshold_for_rescue) {
    is_stop_by_obstacle = true;
  }
  AINFO << "rescue because of stopping obstacle, id: "
        << std::to_string(decision.perception_id())
        << ", stop_count: " << rescue_stop_cnt;
  for (const auto* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (std::to_string(decision.perception_id()) == obstacle->Id()) {
      if (IsNeedIgnore(reference_line_info, vehicle_state, obstacle,
                       reference_end_s_threshold)) {
        AINFO << "obstacle_id: " << obstacle->Id() << " is igv.";
        is_stop_by_obstacle = false;
      }
      for (const auto& poly_pt : obstacle->PerceptionPolygon().points()) {
        AINFO << "rescue_obstacle_info, x: " << poly_pt.x()
              << ", y: " << poly_pt.y();
      }
      break;
    }
  }
  return;
}

bool IsNeedIgnore(
    const ReferenceLineInfo& reference_line_info,
    const century::common::VehicleState& vehicle_state,
    const Obstacle* obstacle, const double ref_end_threshold) {
  if (IsIgv(reference_line_info, obstacle)) {
    AERROR << "blocked by Igv, do not need rescue.";
    return true;
  }
  // reference line end, ignore
  if (IsReachReferenceLineEnd(reference_line_info, vehicle_state,
                              ref_end_threshold)) {
    AERROR << "reached reference line end, do not need rescue.";
    return true;
  }
  return false;
}

bool IsReachReferenceLineEnd(const ReferenceLineInfo& reference_line_info,
                             const century::common::VehicleState& vehicle_state,
                             const double end_threshold) {
  const auto& reference_line_points =
      reference_line_info.reference_line().reference_points();
  if (reference_line_points.empty()) {
    AERROR << "reference line is empty.";
    return false;
  }
  const auto& ref_end_point = reference_line_points.back();
  common::SLPoint ego_sl_point;
  common::SLPoint ref_end_sl_point;
  common::math::Vec2d ego_xy = {vehicle_state.x(), vehicle_state.y()};
  ADEBUG << "loca, x: " << ego_xy.x() << ", y: " << ego_xy.y()
         << ", speed: " << vehicle_state.linear_acceleration()
         << ", ref_end_point,x: " << ref_end_point.x()
         << ", y: " << ref_end_point.y();
  if (!reference_line_info.reference_line().XYToSL(ego_xy, &ego_sl_point) ||
      !reference_line_info.reference_line().XYToSL(ref_end_point,
                                                   &ref_end_sl_point)) {
    AERROR << __func__ << ", Failed to convert ego position to SL.";
    return false;
  }
  AINFO << "ego_sl_point.s(): " << ego_sl_point.s()
        << ", ref_end_point.s(): " << ref_end_sl_point.s();
  if (std::abs(ref_end_sl_point.s() - ego_sl_point.s()) < end_threshold) {
    return true;
  }
  return false;
}

bool IsIgv(const ReferenceLineInfo& reference_line_info,
           const Obstacle* decision_obstacle) {
  const auto& obs_sl = decision_obstacle->PerceptionSLBoundary();
  auto igv_boundary_map = reference_line_info.GetIgvBoundaryMap();
  if (igv_boundary_map.empty()) {
    AERROR << "error, no v2x igv infos.";
    return false;
  }
  for (const auto& igv_boundary : igv_boundary_map) {
    const auto& vehicle_sl = igv_boundary.second;
    double consider_start_l = vehicle_sl.start_l() - kIgvlatBuffer;
    double consider_end_l = vehicle_sl.end_l() + kIgvlatBuffer;
    double consider_start_s = vehicle_sl.start_s() - kIgvlonBuffer;
    double consider_end_s = vehicle_sl.end_s() - kIgvlonBuffer;
    if (obs_sl.start_s() <= consider_end_s &&
        obs_sl.end_s() >= consider_start_s &&
        obs_sl.start_l() <= consider_end_l &&
        obs_sl.end_l() >= consider_start_l) {
      AINFO << "obs " << decision_obstacle->Id() << "is igv!! ";
      return true;
    }
  }
  return false;
}

bool IsAdcInJunction(common::PointENU adc_pt_enu,
                     const century::common::VehicleState vehicle_state,
                     century::hdmap::Junction::Type target_junction_type) {
  const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  std::vector<hdmap::JunctionInfoConstPtr> junctions;
  if (0 == base_map_ptr->GetJunctions(adc_pt_enu, 1.0, &junctions)) {
    for (const auto& ptr_junction : junctions) {
      if (target_junction_type == ptr_junction->junction().type()) {
        if (IsJunctionContainAdc(vehicle_state, *ptr_junction)) {
          AINFO << target_junction_type << " junction contain adc ";
          return true;
        }
      } else {
        continue;
      }
    }
  }
  return false;
}

bool IsVehicleStateValid(const VehicleState& vehicle_state) {
  if (std::isnan(vehicle_state.x()) || std::isnan(vehicle_state.y()) ||
      std::isnan(vehicle_state.z()) || std::isnan(vehicle_state.heading()) ||
      std::isnan(vehicle_state.kappa()) ||
      std::isnan(vehicle_state.linear_velocity()) ||
      std::isnan(vehicle_state.linear_acceleration())) {
    AERROR << "has nan value in vehicle state.";
    return false;
  }
  return true;
}

bool IsDifferentRouting(const RoutingResponse& first,
                        const RoutingResponse& second) {
  if (!second.has_header()) {
    return false;
  }
  if (first.has_header() && second.has_header()) {
    return first.header().sequence_num() != second.header().sequence_num();
  }
  return true;
}

double GetADCStopDeceleration(
    century::common::VehicleStateProvider* vehicle_state,
    const double adc_front_edge_s, const double stop_line_s) {
  double adc_speed = vehicle_state->linear_velocity();
  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();
  if (adc_speed < max_adc_stop_speed) {
    return 0.0;
  }

  double stop_distance = 0;

  if (stop_line_s > adc_front_edge_s) {
    stop_distance = stop_line_s - adc_front_edge_s;
  }
  if (stop_distance < 1e-5) {
    return std::numeric_limits<double>::max();
  }
  return 0.0 == stop_distance ? 0.0
                              : (adc_speed * adc_speed) / (2 * stop_distance);
}

double GetElectricFenceLatDynamicBuff(const double& lat_velocity,
                                      const double& lon_velocity,
                                      double& lat_buff, double& lon_buff) {
  if (std::abs(lat_velocity) < FLAGS_electric_fence_min_lat_velocity) {
    lat_buff = kLatBuffer;
  } else {
    double vec2 = std::abs(lat_velocity) * std::abs(lat_velocity);
    double value =
        vec2 / (2 * FLAGS_electric_fence_max_deceleration) + kLatBuffer / 2;
    lat_buff = std::clamp(value, kLatBuffer, kMaxLatBuffer);
  }
  if (std::abs(lon_velocity) < FLAGS_electric_fence_min_lon_velocity) {
    lon_buff = kLongBuffer;
  } else {
    double vec2 = std::abs(lon_velocity) * std::abs(lon_velocity);
    double value =
        vec2 / (2 * FLAGS_electric_fence_max_deceleration) + kLatBuffer / 2;
    lon_buff = std::clamp(value, kLongBuffer, kLongMaxBuffer);
  }
  double lon_offset = (lon_buff - kLongBuffer) / 2;
  lon_buff = (lon_buff + kLongBuffer) / 2;
  return lon_velocity < 0.0 ? -lon_offset : lon_offset;
}

/*
 * @brief: check if a stop_sign_overlap is still along reference_line
 */
bool CheckStopSignOnReferenceLine(const ReferenceLineInfo& reference_line_info,
                                  const std::string& stop_sign_overlap_id) {
  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info.reference_line().map_path().stop_sign_overlaps();
  auto stop_sign_overlap_it =
      std::find_if(stop_sign_overlaps.begin(), stop_sign_overlaps.end(),
                   [&stop_sign_overlap_id](const PathOverlap& overlap) {
                     return overlap.object_id == stop_sign_overlap_id;
                   });
  return (stop_sign_overlap_it != stop_sign_overlaps.end());
}

/*
 * @brief: check if a traffic_light_overlap is still along reference_line
 */
bool CheckTrafficLightOnReferenceLine(
    const ReferenceLineInfo& reference_line_info,
    const std::string& traffic_light_overlap_id) {
  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  auto traffic_light_overlap_it =
      std::find_if(traffic_light_overlaps.begin(), traffic_light_overlaps.end(),
                   [&traffic_light_overlap_id](const PathOverlap& overlap) {
                     return overlap.object_id == traffic_light_overlap_id;
                   });
  return (traffic_light_overlap_it != traffic_light_overlaps.end());
}

/*
 * @brief: check if ADC is till inside a pnc-junction
 */
bool CheckInsidePnCJunction(const ReferenceLineInfo& reference_line_info) {
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s();

  hdmap::PathOverlap pnc_junction_overlap;
  reference_line_info.GetPnCJunction(adc_front_edge_s, &pnc_junction_overlap);
  if (pnc_junction_overlap.object_id.empty()) {
    return false;
  }

  static constexpr double kIntersectionPassDist = 2.0;  // unit: m
  const double distance_adc_pass_intersection =
      adc_back_edge_s - pnc_junction_overlap.end_s;
  ADEBUG << "distance_adc_pass_intersection[" << distance_adc_pass_intersection
         << "] pnc_junction_overlap[" << pnc_junction_overlap.object_id
         << "] start_s[" << pnc_junction_overlap.start_s << "]";

  return distance_adc_pass_intersection < kIntersectionPassDist;
}

/*
 * @brief: get files at a path
 */
void GetFilesByPath(const boost::filesystem::path& path,
                    std::vector<std::string>* files) {
  ACHECK(files);
  if (!boost::filesystem::exists(path)) {
    return;
  }
  if (boost::filesystem::is_regular_file(path)) {
    AINFO << "Found record file: " << path.c_str();
    files->emplace_back(path.c_str());
    return;
  }
  if (boost::filesystem::is_directory(path)) {
    for (auto& entry : boost::make_iterator_range(
             boost::filesystem::directory_iterator(path), {})) {
      GetFilesByPath(entry.path(), files);
    }
  }
}

hdmap::Lane::LaneType GetLaneTypeAt(const ReferenceLine& reference_line,
                                    const double s) {
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  reference_line.GetLaneFromS(s, &lanes);
  if (!lanes.empty() && nullptr != lanes.front()) {
    return lanes.front()->lane().type();
  }

  return hdmap::Lane::NONE;
}

MovingObstacleType GetMovingObstacleType(
    const Obstacle* obstacle, const common::VehicleState& vehicle_state,
    const ReferenceLine& reference_line) {
  MovingObstacleType type = MovingObstacleType::NO_MOVING;
  if (obstacle->IsStatic()) {
    ADEBUG << "obstacle[" << obstacle->Id() << "] is static, return NO_MOVING.";
    return type;
  }

  const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
  double obs_theta = obstacle->Perception().theta();
  if (obstacle->HasTrajectory()) {
    obs_theta = obstacle->Trajectory().trajectory_point(0).path_point().theta();
  }

  double obs_s = (obstacle_sl.start_s() + obstacle_sl.end_s()) * 0.5;
  ReferencePoint reference_point =
      reference_line.GetNearestReferencePoint(obs_s);
  double buffer_radian = FLAGS_buffer_degrees / kDegrees * M_PI_2;
  double forward_radian = FLAGS_forward_buffer_degrees / kDegrees * M_PI_2;
  double theta_diff = century::common::math::NormalizeAngle(
      obs_theta - reference_point.heading());
  ADEBUG << "Obstacle[" << obstacle->Id()
         << "] and ADC theta diff : " << theta_diff * kDegrees / M_PI_2
         << " degrees.";
  if (theta_diff < -M_PI || theta_diff > M_PI) {
    ADEBUG
        << "Obstacle[" << obstacle->Id()
        << "] and ADC theta diff not in range[-M_PI ~ M_PI], return NO_MOVING.";
    return type;
  }

  SetMovingObstacleType(theta_diff, buffer_radian, forward_radian, &type);

  if (MovingObstacleType::LEFT_FORWARD == type ||
      MovingObstacleType::RIGHT_FORWARD == type ||
      MovingObstacleType::STRAIGHT_FORWARD == type) {
    const double adc_heading = vehicle_state.heading();
    ADEBUG << "adc_heading == " << adc_heading;
    double theta_diff_with_car =
        century::common::math::NormalizeAngle(obs_theta - adc_heading);
    if (theta_diff_with_car >= M_PI_2 - buffer_radian &&
        theta_diff_with_car <= M_PI_2 + buffer_radian) {
      type = MovingObstacleType::RIGHT_CROSSING;
    } else if (theta_diff_with_car >= -M_PI_2 - buffer_radian &&
               theta_diff_with_car <= -M_PI_2 + buffer_radian) {
      type = MovingObstacleType::LEFT_CROSSING;
    }
  }
  return type;
}

void SetMovingObstacleType(const double theta_diff, const double buffer_radian,
                           const double forward_radian,
                           MovingObstacleType* const type) {
  if (theta_diff >= M_PI_2 - buffer_radian &&
      theta_diff <= M_PI_2 + buffer_radian) {
    *type = MovingObstacleType::RIGHT_CROSSING;
  } else if (theta_diff >= -M_PI_2 - buffer_radian &&
             theta_diff <= -M_PI_2 + buffer_radian) {
    *type = MovingObstacleType::LEFT_CROSSING;
  } else if (theta_diff < -M_PI_2 - buffer_radian) {
    *type = MovingObstacleType::LEFT_REVERSING;
  } else if (theta_diff > M_PI_2 + buffer_radian) {
    *type = MovingObstacleType::RIGHT_REVERSING;
  } else if (theta_diff > -M_PI_2 + buffer_radian &&
             theta_diff <= -forward_radian) {
    *type = MovingObstacleType::LEFT_FORWARD;
  } else if (theta_diff >= forward_radian &&
             theta_diff < M_PI_2 - buffer_radian) {
    *type = MovingObstacleType::RIGHT_FORWARD;
  } else {
    *type = MovingObstacleType::STRAIGHT_FORWARD;
  }
}

std::string GetMovingObstacleTypeName(MovingObstacleType type) {
  if (MovingObstacleType::LEFT_CROSSING == type) {
    return "LEFT_CROSSING";
  } else if (MovingObstacleType::RIGHT_CROSSING == type) {
    return "RIGHT_CROSSING";
  } else if (MovingObstacleType::LEFT_REVERSING == type) {
    return "LEFT_REVERSING";
  } else if (MovingObstacleType::RIGHT_REVERSING == type) {
    return "RIGHT_REVERSING";
  } else if (MovingObstacleType::LEFT_FORWARD == type) {
    return "LEFT_FORWARD";
  } else if (MovingObstacleType::RIGHT_FORWARD == type) {
    return "RIGHT_FORWARD";
  } else if (MovingObstacleType::STRAIGHT_FORWARD == type) {
    return "STRAIGHT_FORWARD";
  } else {
    return "NO_MOVING";
  }
}

bool IsNewRouting(const PlanningContext* context) {
  const auto& rerouting_status = context->planning_status().rerouting();
  return rerouting_status.is_new_routing();
}

bool IsLaneBorrow(const PlanningContext* context) {
  const auto& path_decider_status = context->planning_status().path_decider();
  return path_decider_status.is_in_path_lane_borrow_scenario();
}

bool IsLaneChange(const PlanningContext* context) {
  const auto& lane_change = context->planning_status().change_lane();
  return ChangeLaneStatus::IN_CHANGE_LANE == lane_change.status();
}

bool IsOvertake(const PlanningContext* context) {
  const auto& overtake = context->planning_status().overtake();
  return overtake.is_in_overtake_status();
}

bool IsNarrowAreaMode(const PlanningContext* context) {
  const auto& path_decider_status = context->planning_status().path_decider();
  return path_decider_status.adc_enter_narrow_area();
}

bool IsMixedTraffic(const PlanningContext* context) {
  const auto& mixed_traffic_status = context->planning_status().mixed_traffic();
  return FLAGS_enable_enter_mixed_flow_mode &&
         mixed_traffic_status.history_mixed_traffic();
}

bool IsAdcInJunction(const PlanningContext* context) {
  const auto& path_decider_status = context->planning_status().path_decider();
  return path_decider_status.adc_located_in_junction();
}

bool IsNeedLoosenPathConstrains(const PlanningContext* context) {
  const auto& path_decider_status = context->planning_status().path_decider();
  if (path_decider_status.adc_located_in_junction() ||
      path_decider_status.adc_enter_narrow_area() ||
      path_decider_status.need_loosen_path_constrain()) {
    return true;
  }
  return false;
}

void SetFallBackLastFramePath(
    const std::vector<common::PathPoint>& last_discretized_path_points,
    const std::vector<common::FrenetFramePoint>& last_frenet_frame_points,
    const common::SLPoint& last_sl_point, const double diff_s,
    std::vector<common::FrenetFramePoint>* const frenet_path,
    const common::SLPoint& sl_point_of_match_point_in_last_frame,
    DiscretizedPath* discretized_path, double* time, double* loop) {
  CHECK_NOTNULL(frenet_path);

  double start_s = 0.0;
  bool is_updated_start = false;

  std::vector<common::PathPoint> discretized_path_points;
  common::PathPoint path_point;
  common::FrenetFramePoint frenet_frame_point;
  double get_time_start = Clock::NowInSeconds();
  for (size_t i = 0; i < last_frenet_frame_points.size() - 1; ++i) {
    if (last_frenet_frame_points.at(i).s() <
        sl_point_of_match_point_in_last_frame.s()) {
      continue;
    }
    if (!is_updated_start) {
      start_s = last_frenet_frame_points.at(i).s();
      is_updated_start = true;
    }
    const double current_s = last_frenet_frame_points.at(i).s() - start_s;
    frenet_frame_point.set_s(sl_point_of_match_point_in_last_frame.s() +
                             current_s);
    frenet_frame_point.set_l(last_frenet_frame_points.at(i).l());
    frenet_frame_point.set_dl(last_frenet_frame_points.at(i).dl());
    frenet_frame_point.set_ddl(last_frenet_frame_points.at(i).ddl());
    frenet_path->emplace_back(std::move(frenet_frame_point));
    path_point = last_discretized_path_points.at(i);
    discretized_path_points.emplace_back(std::move(path_point));
    loop++;
  }
  *time = (Clock::NowInSeconds() - get_time_start) * 1000;
  *discretized_path = DiscretizedPath(std::move(discretized_path_points));
}

double GetFrontNearestObstaclesDistance(
    const std::vector<Obstacle>& path_obstacles,
    const std::vector<Obstacle> obstacles,
    const std::vector<common::FrenetFramePoint>& frenet_path,
    const double diff_s, const SLBoundary& adc_sl_boundary,
    const common::TrajectoryPoint& init_point,
    const common::SLPoint& sl_point_of_match_point_in_last_frame,
    const common::VehicleParam& vehicle_param) {
  const double half_adc_width = vehicle_param.width() / 2.0;
  const double half_adc_length = vehicle_param.length() / 2.0;
  const auto adc_speed = init_point.v();
  const auto adc_heading = init_point.path_point().theta();
  double nearest_distance = kFallBackMaxPathLength;

  for (size_t i = 0; i < path_obstacles.size(); i++) {
    const auto obstacle = obstacles.at(i);

    auto obstacle_sl = path_obstacles.at(i).PerceptionSLBoundary();
    if (obstacle.IsVirtual()) {
      ADEBUG << "has vistual";
      continue;
    }
    // virtual obstacl's theta no assigned , so need to skip first;
    double obstacle_theta = 0.0;
    obstacle_theta = obstacle.Perception().theta();
    double theta_diff = std::fabs(
        century::common::math::NormalizeAngle(obstacle_theta - adc_heading));

    if (obstacle_sl.start_s() < adc_sl_boundary.end_s()) {
      continue;
    }

    // skip obstacles ahead the maximum path distance.
    if (obstacle_sl.start_s() - adc_sl_boundary.end_s() >
        kFallBackMaxPathLength) {
      continue;
    }
    // // skip fast obstacles driving in the same
    if (obstacle.speed() > 1.5 * adc_speed && std::fabs(theta_diff) < M_PI_2) {
      continue;
    }

    const std::vector<common::FrenetFramePoint>& path_points = frenet_path;
    for (const auto& curr_point_on_path : path_points) {
      if (curr_point_on_path.s() - sl_point_of_match_point_in_last_frame.s() >
          kFallBackMaxPathLength) {
        break;
      }
      bool no_overlap = true;

      if (obstacle.IsStatic()) {
        no_overlap =
            ((curr_point_on_path.s() + half_adc_length <
                  obstacle_sl.start_s() ||
              curr_point_on_path.s() - half_adc_length > obstacle_sl.end_s()) ||
             (curr_point_on_path.l() + half_adc_width +
                      kFlags_fallback_boundary_buffer <
                  obstacle_sl.start_l() ||
              curr_point_on_path.l() - half_adc_width -
                      kFlags_fallback_boundary_buffer >
                  obstacle_sl.end_l()));
      } else {
        // dynamic obstacle need to yeild and no overtake
        double coeff = obstacle.speed() / kFallBackVelocity;
        if (theta_diff >= 0 && theta_diff < M_PI_4) {
          no_overlap =
              obstacle_sl.start_l() >
                  curr_point_on_path.l() + half_adc_width +
                      kFallBackFrontObstacleBuffer * coeff ||
              obstacle_sl.end_l() < curr_point_on_path.l() - half_adc_width -
                                        kFallBackFrontObstacleBuffer * coeff;
        } else {
          no_overlap =
              obstacle_sl.start_l() >
                  curr_point_on_path.l() + half_adc_width +
                      kFlags_fallback_boundary_buffer_for_dynamic * coeff ||
              obstacle_sl.end_l() <
                  curr_point_on_path.l() - half_adc_width -
                      kFlags_fallback_boundary_buffer_for_dynamic * coeff;
        }
      }
      if (!no_overlap && (obstacle_sl.start_s() - adc_sl_boundary.end_s()) -
                                 diff_s - kFallBackStopBuffer <
                             nearest_distance) {
        //  Braking distance
        nearest_distance = obstacle_sl.start_s() - adc_sl_boundary.end_s() -
                           diff_s - kFallBackStopBuffer;
        break;
      }
    }
  }
  return nearest_distance;
}

bool SetFallBackSpeedData(const common::TrajectoryPoint& init_point,
                          const double nearest_distance,
                          SpeedData* const speed_data, const bool& is_sim) {
  CHECK_NOTNULL(speed_data);

  double adc_speed = init_point.v();
  const double max_s_length =
      -0.5 * adc_speed * adc_speed / kFallBackMaxDeceleration;
  double deceleration = 0.0;
  // with max deceleration
  if (max_s_length > nearest_distance - kFallBackFrontObstacleBuffer) {
    deceleration = kFallBackMaxDeceleration;
  } else {
    deceleration = -0.5 * adc_speed * adc_speed /
                   (nearest_distance - kFallBackFrontObstacleBuffer + kMinimum);
    deceleration = std::max(kFallBackMaxDeceleration, deceleration);
  }

  // Don't condsider the accelaration or const velocity case.
  if (deceleration > kDecelerationThreshold) {
    if (is_sim &&
        std::fabs(nearest_distance - kFallBackMaxPathLength) < kTinynum) {
      // adc_speed = FLAGS_planning_upper_speed_limit;
    }  //

    for (double t = 0.0; t < kFallBackTLength; t += kFallBackDeletaT) {
      double s = adc_speed * t;
      double v = adc_speed;
      double a = 0;
      double da = 0;
      speed_data->AppendSpeedPoint(s, t, v, a, da);
    }
    return true;
  }
  //}

  double t_v0 = -adc_speed / deceleration;
  double t = 0.0;
  for (t = 0.0; t < std::min(t_v0, kFallBackTLength); t += kFallBackDeletaT) {
    double s = adc_speed * t + 0.5 * deceleration * t * t;
    double v = adc_speed + deceleration * t;
    double a = deceleration;
    double da = 0;
    speed_data->AppendSpeedPoint(s, t, v, a, da);
  }

  if (t_v0 < kFallBackTLength) {
    ADEBUG << " extend trajectory";
    for (; t < kFallBackTLength; t += kFallBackDeletaT) {
      double s = std::fabs(0.5 * adc_speed * adc_speed / deceleration);
      double v = 0.0;
      double a = 0.0;
      double da = 0;
      speed_data->AppendSpeedPoint(s, t, v, a, da);
    }
  }
  return true;
}

bool CombinePathAndSpeedProfile(
    const double relative_time, const double start_s,
    const DiscretizedPath& path_data, const SpeedData& speed_data,
    DiscretizedTrajectory* const discretized_trajectory) {
  CHECK_NOTNULL(discretized_trajectory);

  if (path_data.empty()) {
    return false;
  }

  for (double cur_rel_time = 0.0; cur_rel_time < kFallBackTLength;
       cur_rel_time += kFallBackDeletaT) {
    common::SpeedPoint speed_point;
    if (!speed_data.EvaluateByTime(cur_rel_time, &speed_point)) {
      AERROR << "Fail to get speed point with relative time " << cur_rel_time;
      return false;
    }

    if (speed_point.s() > path_data.Length()) {
      break;
    }
    common::PathPoint path_point = path_data.Evaluate(speed_point.s());
    path_point.set_s(path_point.s() + start_s);

    common::TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(speed_point.v());
    trajectory_point.set_a(speed_point.a());
    trajectory_point.set_relative_time(speed_point.t() + relative_time);
    discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
  }
  return true;
}

bool CompareTwoStringIsEqual(const std::string& string1,
                             const std::string& string2) {
  if (std::strcmp(string1.c_str(), string2.c_str()) == 0) {
    return true;
  }
  return false;
}

// range: from 0 to 1
double GetObstacleConfidence(const double s) {
  double x =
      std::fmax(0.0, s - FLAGS_min_extremely_accurate_perception_distance) /
      std::fmax(kEpsilon, FLAGS_half_confidence_level_perception_distance);
  double index = -x * x * 0.5;
  return std::exp(index);
}

bool IsLongitudinalOverlap(const SLBoundary& sl_boundary1,
                           const SLBoundary& sl_boundary2) {
  if (sl_boundary1.start_s() > sl_boundary2.end_s() ||
      sl_boundary1.end_s() < sl_boundary2.start_s()) {
    return false;
  }
  return true;
}

bool IsLateralOverlap(const SLBoundary& sl_boundary1,
                      const SLBoundary& sl_boundary2) {
  if (sl_boundary1.start_l() > sl_boundary2.end_l() ||
      sl_boundary1.end_l() < sl_boundary2.start_l()) {
    return false;
  }
  return true;
}

bool WithLeftObsSideBySide(const SLBoundary& obs_boundary,
                           const SLBoundary& adc_boundary) {
  double adc_center_s = (adc_boundary.start_s() + adc_boundary.end_s()) * 0.5;
  if (obs_boundary.start_s() < adc_boundary.end_s() &&
      obs_boundary.end_s() > adc_center_s &&
      obs_boundary.start_l() - adc_boundary.end_l() < kSideBySideLatThreshold &&
      obs_boundary.start_l() > adc_boundary.end_l()) {
    return true;
  }
  return false;
}

bool WithPassObsSideBySide(const ReferenceLineInfo& reference_line_info,
                           const SLBoundary& obs_boundary,
                           const double vehicle_width) {
  double start_l = obs_boundary.start_l();
  double end_l = obs_boundary.end_l();
  double obs_center = (obs_boundary.start_s() + obs_boundary.end_s()) * 0.5;
  // get obstacle position lane width
  auto width = reference_line_info.GetLaneWidthByS(obs_center);
  return (std::fmax(width.first - end_l, start_l + width.second) >
          vehicle_width + kSafeLatThreshold);
}

bool IsJunctionContainAdc(const century::common::VehicleState& vehicle_state,
                          const hdmap::JunctionInfo& junction_info) {
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  // Compute the ADC bounding box.
  common::math::Vec2d ego_center_map_frame(
      (vehicle_param.front_edge_to_center() -
       vehicle_param.back_edge_to_center()) *
          0.5,
      (vehicle_param.left_edge_to_center() -
       vehicle_param.right_edge_to_center()) *
          0.5);
  ego_center_map_frame.SelfRotate(vehicle_state.heading());
  ego_center_map_frame.set_x(vehicle_state.x() + ego_center_map_frame.x());
  ego_center_map_frame.set_y(vehicle_state.y() + ego_center_map_frame.y());
  common::math::Box2d adc_box(ego_center_map_frame, vehicle_state.heading(),
                              vehicle_param.length(), vehicle_param.width());
  // Check whether Junction's polygon contain ADC bounding box.
  const auto& polygon = junction_info.polygon();
  return polygon.Contains(Polygon2d(adc_box));
}

bool IsGateJunctionContainAdc(
    common::PointENU adc_pt_enu,
    const century::common::VehicleState vehicle_state) {
  if (!FLAGS_allow_narrow_pass) {
    return false;
  }
  const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  std::vector<hdmap::JunctionInfoConstPtr> junctions;
  if (0 == base_map_ptr->GetJunctions(adc_pt_enu, 1.0, &junctions)) {
    for (const auto& ptr_junction : junctions) {
      if (hdmap::Junction::DongJiaZhen_Gate ==
          ptr_junction->junction().type()) {
        if (IsJunctionContainAdc(vehicle_state, *ptr_junction)) {
          AINFO << "Dongjiazhen_gate junction contain adc ";
          return true;
        }
      } else {
        continue;
      }
    }
  }
  return false;
}

bool GetRemainDisToTurn(const ReferenceLineInfo& reference_line_info,
                        const double distance_threshold, const double step,
                        const double kappa_threshold, double& distance) {
  const auto& reference_line = reference_line_info.reference_line();
  double check_s = reference_line_info.AdcSlBoundary().start_s();
  bool has_turn = false;
  double total_distance = 0.0;
  bool is_turn_scene = false;
  century::hdmap::Lane::LaneTurn turn_type;
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  for (const auto& lane_id : reference_line_info.TargetLaneId()) {
    auto lane_temp = hdmap->GetLaneById(lane_id);
    if (lane_temp->lane().has_turn() &&
        (hdmap::Lane::NO_TURN == lane_temp->lane().turn() ||
         hdmap::Lane::U_TURN == lane_temp->lane().turn())) {
      continue;
    }
    lanes.emplace_back(lane_temp);
  }
  if (lanes.empty()) {
    AERROR << "No turn left/right scene lane, do not need openspace/rescue.";
    return false;
  }

  // Traveling target lane
  while (check_s < std::min(reference_line_info.AdcSlBoundary().start_s() +
                                distance_threshold,
                            reference_line.Length())) {
    ReferencePoint nearest_ref_point =
        reference_line.GetNearestReferencePoint(check_s);
    if (!lanes.empty() && lanes.front() != nullptr) {
      const auto lane = lanes.front();
      ADEBUG << "current_lane id: " << lane->lane().id().id()
             << ", current_check_s: " << check_s
             << ", kappa: " << nearest_ref_point.kappa();
      if (lane->lane().has_turn()) {
        if (hdmap::Lane::LEFT_TURN == lane->lane().turn()) {
          turn_type = hdmap::Lane::LEFT_TURN;
          has_turn = true;
        } else if (hdmap::Lane::RIGHT_TURN == lane->lane().turn()) {
          turn_type = hdmap::Lane::RIGHT_TURN;
          has_turn = true;
        } else {
          break;
        }
      }
      if (has_turn && std::fabs(nearest_ref_point.kappa()) > kappa_threshold) {
        AINFO << "nearest point kappa(" << nearest_ref_point.kappa()
              << ") > kappa_threshold(" << kappa_threshold
              << ") && has turn. is TURN SCENE.";
        distance = total_distance;
        is_turn_scene = true;
        break;
      }
    }
    check_s += step;
    // There are sequential requirement here.
    total_distance +=
        reference_line.GetNearestReferencePoint(check_s).DistanceTo(
            nearest_ref_point);
    ADEBUG << "--- is_turn_scene: " << is_turn_scene
           << ", has_turn: " << has_turn << ", turn_type: " << turn_type
           << ", check_s: " << check_s
           << ", kappa: " << nearest_ref_point.kappa()
           << ", kappa_threshold: " << kappa_threshold
           << ", total_distance: " << total_distance << ", delta_s: "
           << reference_line.GetNearestReferencePoint(check_s).DistanceTo(
                  nearest_ref_point)
           << ", adc_start_s: "
           << reference_line_info.AdcSlBoundary().start_s();
  }
  AINFO << "is_turn_scene: " << is_turn_scene
        << ", distance_to_turn: " << distance;
  return is_turn_scene;
}

}  // namespace util
}  // namespace planning
}  // namespace century
