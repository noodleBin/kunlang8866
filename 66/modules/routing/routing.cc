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

#include "modules/routing/routing.h"

#include <float.h>

#include <limits>
#include <sstream>
#include <unordered_map>
#include <utility>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/point_factory.h"
#include "modules/routing/common/routing_gflags.h"

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

namespace {
constexpr int kTwowaypointSize = 2;           // param value
constexpr int kThreewaypointSize = 3;         // param value
constexpr int kFourwaypointSize = 4;          // param value
constexpr uint32_t kMaxLaneChangeScheme = 1;
constexpr uint32_t kMinLaneChangeScheme = 2;
}

std::string Routing::Name() const { return FLAGS_routing_node_name; }

Routing::Routing()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::ROUTING) {}

century::common::Status Routing::Init() {
  const auto routing_map_file = century::hdmap::RoutingMapFile();
  AINFO << "Use routing topology graph path: " << routing_map_file;
  navigator_ptr_.reset(new Navigator(routing_map_file));
  routing_math_ptr_.reset(new century::routing::RoutingMath());

  hdmap_ = century::hdmap::HDMapUtil::BaseMapPtr();
  ACHECK(hdmap_) << "Failed to load map file:" << century::hdmap::BaseMapFile();
  if (!routing_math_ptr_->Init(hdmap_)) {
    return century::common::Status(ErrorCode::ROUTING_ERROR,
                                   "hdmap_ is nullptr");
  }
  return century::common::Status::OK();
}

century::common::Status Routing::Start() {
  if (!navigator_ptr_->IsReady()) {
    AERROR << "Navigator is not ready!";
    return century::common::Status(ErrorCode::ROUTING_ERROR,
                                   "Navigator not ready");
  }
  AINFO << "Routing service is ready.";
  monitor_logger_buffer_.INFO("Routing started");
  return century::common::Status::OK();
}

bool Routing::FillFristOrAllPointLaneInfo(const FillLaneInfoType& type,
                                          const RoutingRequest& routing_request,
                                          RoutingRequest* const fixed_request) {
  int waypoint_size =
      type == FillLaneInfoType::FRIST ? 1 : routing_request.waypoint_size();
  bool is_additional_lane = type == FillLaneInfoType::FRIST ? false : true;
  AINFO << "type:" << int(type) << ",waypoint_size:" << waypoint_size
        << ",is_additional_lane:" << is_additional_lane;
  for (int i = 0; i < waypoint_size; ++i) {
    const auto& lane_waypoint = routing_request.waypoint(i);
    auto point = lane_waypoint.pose();
    std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
    hdmap_->GetLanes(point, FLAGS_adc_lane_range, &lanes);
    // check in parking space
    hdmap::ParkingSpaceInfoConstPtr parking_space = nullptr;
    std::vector<hdmap::ParkingSpaceInfoConstPtr> parking_spaces_vector;
    hdmap_->GetParkingSpaces(point, FLAGS_routing_lane_search_radius, &parking_spaces_vector);
    for (const auto& it : parking_spaces_vector) {
      if (it->polygon().IsPointIn(common::math::Vec2d(point.x(), point.y()))) {
        parking_space = it;
        break;
      }
    }
    if (lanes.empty() && nullptr == parking_space) {
      AERROR << "Failed to find nearest lane or parking space from map at "
                "position: "
             << point.DebugString();
      return false;
    }
    LaneWaypoint waypoint_info(lane_waypoint);
    if (nullptr != parking_space) {
      if (GetWaypointInfoForParking(i, parking_space, waypoint_info)) {
        fixed_request->add_waypoint()->CopyFrom(waypoint_info);
        AINFO << "Success to fill parking space waypoint info for waypoint("
              << i << ")";
      }
    } else {
      if (GetWaypointInfo(i, lanes, waypoint_info, is_additional_lane)) {
        fixed_request->add_waypoint()->CopyFrom(waypoint_info);
        AINFO << "Success to fill waypoint info for waypoint(" << i << ")";
      }
    }
  }
  return true;
}

void Routing::ApplyBlacklistRule(RoutingRequest* const request,
                                 const std::string_view block_name,
                                 bool is_only_add_reverse) {
  auto lane_ids =
      routing_math_ptr_->GetBlacklistedLanes(block_name, is_only_add_reverse);
  if (lane_ids.empty()) {
    return;
  }
  std::ostringstream oss;
  oss << block_name << ": ";
  for (const auto& lane_id : lane_ids) {
    auto* lane = request->add_blacklisted_lane();
    lane->set_id(lane_id);
    oss << lane_id << ",";
  }
  AINFO << oss.str();
}

void Routing::ApplyBlacklistRuleForRightSide(RoutingRequest* const request) {
  std::vector<std::string> lane_ids;
  auto front_waypoint = request->waypoint(0);
  auto back_waypoint = request->waypoint(request->waypoint().size() - 1);
  auto front_lane_ids_max = routing_math_ptr_->GetBlacklistedLanes(
      front_waypoint.pose(), FLAGS_max_blacklisted_lane_distance,
      "MAX_BLOCK_RIGHT_REVERSE", true);
  auto back_lane_ids_max = routing_math_ptr_->GetBlacklistedLanes(
      back_waypoint.pose(), FLAGS_max_blacklisted_lane_distance,
      "MAX_BLOCK_RIGHT_REVERSE", true);
  auto front_lane_ids_min = routing_math_ptr_->GetBlacklistedLanes(
      front_waypoint.pose(), FLAGS_min_blacklisted_lane_distance,
      "MIN_BLOCK_RIGHT_REVERSE", true);
  auto back_lane_ids_min = routing_math_ptr_->GetBlacklistedLanes(
      back_waypoint.pose(), FLAGS_min_blacklisted_lane_distance,
      "MIN_BLOCK_RIGHT_REVERSE", true);
  std::vector<std::string> front_lane_ids;
  front_lane_ids.reserve(front_lane_ids_max.size() + front_lane_ids_min.size());
  front_lane_ids.insert(front_lane_ids.end(), front_lane_ids_max.begin(),
                        front_lane_ids_max.end());
  front_lane_ids.insert(front_lane_ids.end(), front_lane_ids_min.begin(),
                        front_lane_ids_min.end());
  std::vector<std::string> back_lane_ids;
  back_lane_ids.reserve(back_lane_ids_max.size() + back_lane_ids_min.size());
  back_lane_ids.insert(back_lane_ids.end(), back_lane_ids_max.begin(),
                       back_lane_ids_max.end());
  back_lane_ids.insert(back_lane_ids.end(), back_lane_ids_min.begin(),
                       back_lane_ids_min.end());

  std::unordered_set<std::string> temp_lane_ids(front_lane_ids.begin(),
                                                front_lane_ids.end());
  for (const auto& str : back_lane_ids) {
    if (temp_lane_ids.find(str) != temp_lane_ids.end()) {
      if (std::find(lane_ids.begin(), lane_ids.end(), str) == lane_ids.end()) {
        lane_ids.emplace_back(str);
      }
    }
  }
  if (lane_ids.empty()) {
    return;
  }
  std::ostringstream oss;
  oss << "BLOCK_RIGHT_REVERSE: ";
  for (const auto& lane_id : lane_ids) {
    auto* lane = request->add_blacklisted_lane();
    lane->set_id(lane_id);
    oss << lane_id << ",";
  }
  AINFO << oss.str();
}

void Routing::ApplyBlacklistRuleForD7J4W(RoutingRequest* const request) {
  auto front_waypoint = request->waypoint(0);
  auto back_waypoint = request->waypoint(request->waypoint().size() - 1);
  if (front_waypoint.id().empty() || back_waypoint.id().empty()) {
    AINFO << "ApplyBlacklistRuleForD7J4W: front or back waypoint has no id, "
             "skip blacklist";
    return;
  }
  auto front_lane_ids = routing_math_ptr_->GetBlacklistedLanes(
      front_waypoint.pose(), FLAGS_d7j4w_blacklisted_lane_distance,
      "BLOCK_D7_J4W", false);
  auto back_lane_ids = routing_math_ptr_->GetBlacklistedLanes(
      back_waypoint.pose(), FLAGS_d7j4w_blacklisted_lane_distance,
      "BLOCK_D7_J4W", false);

  if (front_lane_ids.empty() || back_lane_ids.empty()) {
    return;
  }
  std::ostringstream oss;
  oss << "BLOCK_D7_J4W: ";
  for (const auto& lane_id : front_lane_ids) {
    auto* lane = request->add_blacklisted_lane();
    lane->set_id(lane_id);
    oss << lane_id << ",";
  }
  AINFO << oss.str();
}

void Routing::AddBlacklistedLanes(RoutingRequest* const fixed_request) {
  if (!FLAGS_enable_add_blacklisted_lanes) {
    return;
  }
  auto front_waypoint = fixed_request->waypoint(0);
  auto back_waypoint =
      fixed_request->waypoint(fixed_request->waypoint().size() - 1);
  if (!routing_math_ptr_->IsPointInSpecificJunction(front_waypoint,
                                                    Junction::T4_T5_T8) &&
      !routing_math_ptr_->IsPointInSpecificJunction(back_waypoint,
                                                    Junction::T4_T5_T8)) {
    ApplyBlacklistRule(fixed_request, "BLOCK_T4_T5_T8", false);
  }
  if (!(routing_math_ptr_->IsPointInSpecificJunction(back_waypoint,
                                                     Junction::J4_EAST) &&
        (!routing_math_ptr_->IsPointInSpecificJunction(front_waypoint,
                                                       Junction::T4_T5_T8) &&
         !routing_math_ptr_->IsPointInSpecificJunction(
             front_waypoint, Junction::BLOCKING_AREA_J4) &&
         !routing_math_ptr_->IsPointInSpecificJunction(
             front_waypoint, Junction::J4_1_MIDDLE))) &&
      (routing_math_ptr_->IsPointInSpecificJunction(
           front_waypoint, Junction::BLOCKING_AREA_J4) &&
       !routing_math_ptr_->IsPointInSpecificJunction(front_waypoint,
                                                     Junction::J4_EAST) &&
       !routing_math_ptr_->IsPointInSpecificJunction(
           back_waypoint, Junction::BLOCKING_AREA_J4) &&
       !routing_math_ptr_->IsPointInSpecificJunction(back_waypoint,
                                                     Junction::T4_T5_T8))) {
    ApplyBlacklistRule(fixed_request, "BLOCK_J4_EAST", false);
  }
  if (routing::RE_ROUTING_D7 ==
      fixed_request->rerouting_info().block_rerouting().rerouting_type()) {
    ApplyBlacklistRule(fixed_request, "BLOCK_D7_11", false);
  }
  if (!routing_math_ptr_->IsPointInSpecificJunction(back_waypoint,
                                                    Junction::G_AREA) &&
      !routing_math_ptr_->IsPointInSpecificJunction(back_waypoint,
                                                    Junction::J4_EAST) &&
      !routing_math_ptr_->IsPointInSpecificJunction(front_waypoint,
                                                    Junction::T4_T5_T8) &&
      !routing_math_ptr_->IsPointInSpecificJunction(
          front_waypoint, Junction::BLOCKING_AREA_J4) &&
      !routing_math_ptr_->IsPointInSpecificJunction(front_waypoint,
                                                    Junction::J4_1_MIDDLE) &&
      !routing_math_ptr_->IsPointInSpecificJunction(back_waypoint,
                                                    Junction::D_M3L_AREA) &&
      !routing_math_ptr_->IsPointInSpecificJunction(back_waypoint,
                                                    Junction::M3H_M2L_AREA) &&
      !routing_math_ptr_->IsPointInSpecificJunction(back_waypoint,
                                                    Junction::M2H_M1_AREA)) {
    ApplyBlacklistRuleForRightSide(fixed_request);
  }
  ApplyBlacklistRuleForD7J4W(fixed_request);
  // AINFO << "FLAGS_project_scenario:" << FLAGS_project_scenario;
  // if (FLAGS_project_scenario == "DJZ" &&
  // if (FLAGS_project_scenario == "DJZ" &&
  //     FLAGS_enable_add_djz_blacklisted_lanes) {
  //   // for (const auto& fixed_black_point :
  //   //      fixed_points_[FixedPointArea::DJZ_1 + kFixedAreaOffset]) {
  //   //   LaneWaypoint waypoint_info(fixed_request->waypoint(0));
  //   //   waypoint_info.mutable_pose()->set_x(fixed_black_point.x);
  //   //   waypoint_info.mutable_pose()->set_y(fixed_black_point.y);
  //   //   std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
  //   //   hdmap_->GetLanes(waypoint_info.pose(), FLAGS_adc_lane_range, &lanes);
  //   //   for (const auto& lane_id : lanes) {
  //   //     auto lane = fixed_request->add_blacklisted_lane();
  //   //     lane->set_id(lane_id->id().id());
  //   //     AINFO << "DongJiaZhen add blacklisted lane:" <<
  //   lane_id->id().id();
  //   //   }
  //   // }
  //   for (const auto& fixed_black_point :
  //        fixed_points_[FixedPointArea::DJZ_2 + kFixedAreaOffset]) {
  //     double two_front_points_distance =
  //         routing_math_ptr_->GetTwoPointsDistance(front_waypoint,
  //         fixed_black_point);
  //     double two_back_points_distance =
  //         routing_math_ptr_->GetTwoPointsDistance(back_waypoint,
  //         fixed_black_point);
  //     if (two_front_points_distance < FLAGS_djz_max_blacklisted_lane_distance ||
  //         two_back_points_distance < FLAGS_djz_max_blacklisted_lane_distance) {
  //       AINFO << "two_front_points_distance is close, exclude blacklist
  //       lane."; return;
  //     }
  //   }
  //   for (const auto& fixed_black_point :
  //        fixed_points_[FixedPointArea::DJZ_2 + kFixedAreaOffset]) {
  //     LaneWaypoint waypoint_info(fixed_request->waypoint(0));
  //     waypoint_info.mutable_pose()->set_x(fixed_black_point.x);
  //     waypoint_info.mutable_pose()->set_y(fixed_black_point.y);
  //     std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
  //     hdmap_->GetLanes(waypoint_info.pose(), kRangeofAdcBlackLane, &lanes);
  //     for (const auto& lane_id : lanes) {
  //       auto lane = fixed_request->add_blacklisted_lane();
  //       lane->set_id(lane_id->id().id());
  //       AINFO << "DongJiaZhen add blacklisted lane:" << lane_id->id().id();
  //     }
  //   }
  // }
}

bool Routing::FillEndPointLaneInfoForTinyAdjustment(
    const RoutingRequest& routing_request,
    RoutingRequest* const fixed_request) {
  if (routing_request.waypoint().size() >= kTwowaypointSize) {
    auto first_waypoint = routing_request.waypoint(0);
    auto last_waypoint =
        routing_request.waypoint(routing_request.waypoint().size() - 1);
    double distance =
        std::hypot(last_waypoint.pose().x() - first_waypoint.pose().x(),
                   last_waypoint.pose().y() - first_waypoint.pose().y());
    if (distance > FLAGS_max_stacker_position_distance) {
      AERROR << "FillEndPointLaneInfoForTinyAdjustment failed: distance "
                "between first "
             << "and last waypoint (" << distance
             << "m) exceeds maximum allowed "
             << "distance (" << FLAGS_max_stacker_position_distance << "m). "
             << "Stacker position is considered unreliable.";
      return false;
    }
  }

  auto adc_waypoint = fixed_request->waypoint(0);
  double total_length = fixed_request->tiny_adjustment_distance();
  double move_heading;
  if ((!FLAGS_enable_multi_mode_tiny_adjustment ||
       routing::DREAMVIEW_ADIUSTMENT == fixed_request->tiny_adjustment_type() ||
       routing::DEFAULT_ADIUSTMENT == fixed_request->tiny_adjustment_type()) &&
      fixed_request->task_type() !=
          routing::LOADING_OPERATIONAREA_SAMEDIRECTION_2 &&
      fixed_request->task_type() !=
          routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_2) {
    if (fixed_request->task_type() != routing::TINY_ADJUSTMENT_BACK &&
        fixed_request->task_type() != routing::TINY_ADJUSTMENT_FRONT &&
        fixed_request->task_type() != routing::TINY_ADJUSTMENT_STOP) {
      AERROR << "FillEndPointLaneInfoForTinyAdjustment faild!!";
      return false;
    }
    auto adc_lane_id = adc_waypoint.id();
    auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(adc_lane_id));
    move_heading = lane->Heading(adc_waypoint.s());
    // AINFO << "total_length:" << total_length << ",adc_heading:" <<
    // adc_heading
    //       << ",lane_heading:" << lane_heading;
    if ((routing::TINY_ADJUSTMENT_BACK == fixed_request->task_type() &&
         !is_heading_reverse_) ||
        (routing::TINY_ADJUSTMENT_FRONT == fixed_request->task_type() &&
         is_heading_reverse_)) {
      move_heading = move_heading + M_PI;
    }
  } else if (routing::STACKER_ADIUSTMENT ==
             fixed_request->tiny_adjustment_type()) {
    if (fixed_request->task_type() != routing::TINY_ADJUSTMENT_LEFT &&
        fixed_request->task_type() != routing::TINY_ADJUSTMENT_RIGHT &&
        fixed_request->task_type() != routing::TINY_ADJUSTMENT_STOP) {
      AERROR << "FillEndPointLaneInfoForTinyAdjustment faild!!";
      return false;
    }
    double stacker_heading =
        routing_request.waypoint(routing_request.waypoint().size() - 1)
            .heading();
    // auto target_waypoint =
    //     fixed_request->mutable_waypoint(routing_request.waypoint().size() -
    //     1);
    if (routing::TINY_ADJUSTMENT_LEFT == fixed_request->task_type()) {
      move_heading = routing_math_ptr_->GetNoTurnLaneHeading(
          stacker_heading, adc_waypoint, false);
    } else {
      move_heading = routing_math_ptr_->GetNoTurnLaneHeading(
          stacker_heading, adc_waypoint, true);
    }
  } else if (routing::LOADING_OPERATIONAREA_SAMEDIRECTION_2 ==
                 fixed_request->task_type() ||
             routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_2 ==
                 fixed_request->task_type()) {
    double adc_heading = routing_request.waypoint(0).heading();
    auto adc_lane_id = adc_waypoint.id();
    auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(adc_lane_id));
    move_heading = lane->Heading(adc_waypoint.s());
    double diff_heading =
        std::fabs(common::math::NormalizeAngle(move_heading - adc_heading));
    if ((diff_heading > M_PI_2 && !is_backward_) ||
        (diff_heading < M_PI_2 && is_backward_)) {
      move_heading = move_heading + M_PI;
    }
    total_length = FLAGS_dual_box_exit_distance;
  }
  LaneWaypoint end_waypoint(adc_waypoint);
  if (FLAGS_enable_tiny_along_lane) {
    AINFO << "go use the waypoint calculated along the lane";
    routing_math_ptr_->MoveLaneWaypointAlongLane(move_heading, total_length,
                                                 end_waypoint);
  } else {
    double new_x =
        adc_waypoint.pose().x() + total_length * std::cos(move_heading);
    double new_y =
        adc_waypoint.pose().y() + total_length * std::sin(move_heading);
    end_waypoint.mutable_pose()->set_x(new_x);
    end_waypoint.mutable_pose()->set_y(new_y);
  }
  LaneWaypoint waypoint_info(end_waypoint);
  std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
  hdmap_->GetLanes(end_waypoint.pose(), FLAGS_adc_lane_range, &lanes);
  if (GetWaypointInfo(1, lanes, waypoint_info, false, true)) {
    fixed_request->add_waypoint()->CopyFrom(waypoint_info);
    return true;
  }
  return false;
}

bool Routing::FillEndPointLaneInfoForFrontTrajection(
    RoutingRequest* const fixed_request) {
  auto adc_waypoint = fixed_request->waypoint(0);
  LaneWaypoint lane_waypoint(adc_waypoint), end_lane_waypoint(adc_waypoint);
  if (!is_heading_reverse_) {
    double heading =
        common::math::NormalizeAngle(adc_waypoint.heading() + M_PI);
    // last_task_veh_start_heading_ = heading;
    lane_waypoint.set_heading(heading);
  }
  auto point = lane_waypoint.pose();
  std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
  hdmap_->GetLanes(point, FLAGS_adc_lane_range, &lanes);
  if (!lanes.empty()) {
    if (GetWaypointInfo(0, lanes, lane_waypoint, false)) {
      routing_math_ptr_->GetWaypointAlongSuccessorIdByDistance(
          adc_waypoint, end_lane_waypoint, FLAGS_adc_advance_distance);
      fixed_request->add_waypoint()->CopyFrom(end_lane_waypoint);
      AINFO << "Success to fill waypoint info for Front Trajection";
      return true;
    }
  }
  return false;
}

bool Routing::FillEndPointLaneInfoForFirstBox(
    const RoutingRequest& routing_request,
    RoutingRequest* const fixed_request) {
  AINFO << "== FillEndPointLaneInfoForFirstBox ==";
  auto lane_waypoint =
      routing_request.waypoint(routing_request.waypoint_size() - 1);

  double adc_heading = lane_waypoint.heading();
  double total_length = routing_request.tiny_adjustment_distance();
  if (routing::LOADING_OPERATIONAREA_SAMEDIRECTION_1 ==
      routing_request.task_type()) {
    total_length = -FLAGS_vehicle_length * 0.25;
  } else if (routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_1 ==
             routing_request.task_type()) {
    total_length = FLAGS_vehicle_length * 0.25;
  }
  AINFO << "total_length = " << total_length;

  double new_x =
      lane_waypoint.pose().x() + total_length * std::cos(adc_heading);
  double new_y =
      lane_waypoint.pose().y() + total_length * std::sin(adc_heading);
  AINFO << "lane_waypoint.pose().x():" << lane_waypoint.pose().x()
        << ",y:" << lane_waypoint.pose().y();
  // xy changge ,lane,id maybe to change
  AINFO << "new_x:" << new_x << ",new_y:" << new_y;
  lane_waypoint.mutable_pose()->set_x(new_x);
  lane_waypoint.mutable_pose()->set_y(new_y);
  fixed_request->add_waypoint()->CopyFrom(lane_waypoint);
  return true;
}

bool Routing::IsDeadEndRoad(RoutingRequest* const fixed_request) {
  bool result = false;
  //   auto fix_points_tuple = std::make_tuple(
  //       fixed_points_[FixedPointArea::DJZ_dead_end_enter + kFixedAreaOffset],
  //       fixed_points_[FixedPointArea::DJZ_enter + kFixedAreaOffset],
  //       fixed_points_[FixedPointArea::DJZ_dead_end_exit + kFixedAreaOffset],
  //       fixed_points_[FixedPointArea::DJZ_exit + kFixedAreaOffset],
  //       fixed_points_[FixedPointArea::DJZ_other_dead_end_enter +
  //                     kFixedAreaOffset],
  //       fixed_points_[FixedPointArea::DJZ_other_dead_end_exit +
  //                     kFixedAreaOffset]);
  //   fixed_request->clear_dead_end_waypoint();
  //   auto front_waypoint = fixed_request->waypoint(0);
  //   auto back_waypoint =
  //       fixed_request->waypoint(fixed_request->waypoint().size() - 1);
  //   LaneWaypoint lane_waypoint(back_waypoint);
  //   std::vector<LaneWaypoint> vec_lane_waypoint;
  //   for (int i = 0; i < std::get<2>(fix_points_tuple).size(); i++) {
  //     auto front_dist =
  //         routing_math_ptr_->PointToLineDistance(front_waypoint,
  //         std::get<2>(fix_points_tuple)[i],
  //                             std::get<3>(fix_points_tuple)[i]);
  //     if (front_dist > FLAGS_adc_two_point_distance) {
  //       continue;
  //     } else {
  //       if (IsPointWithinRangeLine(front_waypoint,
  //                                  std::get<2>(fix_points_tuple)[i],
  //                                  std::get<3>(fix_points_tuple)[i])) {
  //         if (!IsFixedPointClose(front_waypoint,
  //                                std::get<3>(fix_points_tuple)[i])) {
  //           auto back_dist = routing_math_ptr_->PointToLineDistance(
  //               back_waypoint, std::get<2>(fix_points_tuple)[i],
  //               std::get<5>(fix_points_tuple)[i]);
  //           if (back_dist > FLAGS_adc_two_point_distance) {
  //             lane_waypoint.mutable_pose()->set_x(
  //                 std::get<3>(fix_points_tuple)[i].x);
  //             lane_waypoint.mutable_pose()->set_y(
  //                 std::get<3>(fix_points_tuple)[i].y);
  //             vec_lane_waypoint.push_back(lane_waypoint);
  //             result = true;
  //             AINFO << "the front point is driving out of a dead end ==> " <<
  //             i
  //                   << ",front_dist:" << front_dist << ",back_dist:" <<
  //                   back_dist;
  //           } else {
  //             if (!IsPointWithinRangeLine(back_waypoint,
  //                                         std::get<2>(fix_points_tuple)[i],
  //                                         std::get<5>(fix_points_tuple)[i]))
  //                                         {
  //               lane_waypoint.mutable_pose()->set_x(
  //                   std::get<3>(fix_points_tuple)[i].x);
  //               lane_waypoint.mutable_pose()->set_y(
  //                   std::get<3>(fix_points_tuple)[i].y);
  //               vec_lane_waypoint.push_back(lane_waypoint);
  //               result = true;
  //               AINFO << "the front point is driving out of a dead end ==> "
  //               << i
  //                     << ",front_dist:" << front_dist
  //                     << ",back_dist:" << back_dist;
  //             }
  //           }
  //         }
  //       }
  //     }
  //   }
  //   for (int i = 0; i < std::get<0>(fix_points_tuple).size(); i++) {
  //     auto back_dist =
  //         routing_math_ptr_->PointToLineDistance(back_waypoint,
  //         std::get<0>(fix_points_tuple)[i],
  //                             std::get<1>(fix_points_tuple)[i]);
  //     if (back_dist > FLAGS_adc_two_point_distance) {
  //       continue;
  //     } else {
  //       if (IsPointWithinRangeLine(back_waypoint,
  //                                  std::get<0>(fix_points_tuple)[i],
  //                                  std::get<1>(fix_points_tuple)[i])) {
  //         if (!IsFixedPointClose(back_waypoint,
  //                                std::get<1>(fix_points_tuple)[i])) {
  //           auto front_dist = routing_math_ptr_->PointToLineDistance(
  //               front_waypoint, std::get<0>(fix_points_tuple)[i],
  //               std::get<4>(fix_points_tuple)[i]);
  //           if (front_dist > FLAGS_adc_two_point_distance) {
  //             lane_waypoint.mutable_pose()->set_x(
  //                 std::get<1>(fix_points_tuple)[i].x);
  //             lane_waypoint.mutable_pose()->set_y(
  //                 std::get<1>(fix_points_tuple)[i].y);
  //             vec_lane_waypoint.push_back(lane_waypoint);
  //             result = true;
  //             AINFO << "the back point is dead end ==> " << i
  //                   << ",back_dist:" << back_dist << ",front_dist:" <<
  //                   front_dist;
  //           } else {
  //             if (!IsPointWithinRangeLine(front_waypoint,
  //                                         std::get<0>(fix_points_tuple)[i],
  //                                         std::get<4>(fix_points_tuple)[i]))
  //                                         {
  //               lane_waypoint.mutable_pose()->set_x(
  //                   std::get<1>(fix_points_tuple)[i].x);
  //               lane_waypoint.mutable_pose()->set_y(
  //                   std::get<1>(fix_points_tuple)[i].y);
  //               vec_lane_waypoint.push_back(lane_waypoint);
  //               result = true;
  //               AINFO << "the back point is dead end ==> " << i
  //                     << ",back_dist:" << back_dist
  //                     << ",front_dist:" << front_dist;
  //             }
  //           }
  //         }
  //       }
  //     }
  //   }
  //   if (result) {
  //     fixed_request->clear_waypoint();
  //     fixed_request->add_waypoint()->CopyFrom(front_waypoint);
  //     fixed_request->add_waypoint()->CopyFrom(vec_lane_waypoint.front());
  //     AINFO << "vec_lane_waypoint ---->";
  //     for (int i = 0; i < vec_lane_waypoint.size(); i++) {
  //       AINFO << "x:" << vec_lane_waypoint[i].pose().x()
  //             << ",y:" << vec_lane_waypoint[i].pose().y();
  //       if (i == 0) continue;
  //       fixed_request->add_dead_end_waypoint()->CopyFrom(vec_lane_waypoint[i]);
  //     }
  //     fixed_request->add_dead_end_waypoint()->CopyFrom(back_waypoint);
  //     fixed_request->set_dead_end_level(0);
  //     fixed_request->set_is_dead_end_two_routing(true);
  //   }
  return result;
}

bool Routing::FillEndPointLaneInfoForProjection(
    const RoutingRequest& routing_request,
    RoutingRequest* const fixed_request) {
  AINFO << "== FillEndPointLaneInfoForProjection ==";
  auto lane_waypoint =
      routing_request.waypoint(routing_request.waypoint_size() - 1);
  if (routing::SITE_COORDINATE == routing_request.coordinate_type() ||
      routing::STACKER_COORDINATE == routing_request.coordinate_type()) {
    if (routing_math_ptr_->GetCraneProjectionLaneInfor(lane_waypoint)) {
      fixed_request->add_waypoint()->CopyFrom(lane_waypoint);
      return true;
    }
  } else if (routing::WHEEL_CRANE_COORDINATE ==
             routing_request.coordinate_type()) {
    if (routing_math_ptr_->GetWheelCraneProjectionLaneInfor(lane_waypoint)) {
      fixed_request->add_waypoint()->CopyFrom(lane_waypoint);
      return true;
    }
  }
  return false;
}

bool Routing::GetWaypointInfoForParking(
    const int index, const hdmap::ParkingSpaceInfoConstPtr parking_space,
    LaneWaypoint& waypoint_info) {
  // waypoint is in parking space
  waypoint_info.set_id(parking_space->lane_id().id());
  AINFO << "waypoint is in parking space: " << parking_space->lane_id().id();
  // set parking space center point
  if (!parking_space->polygon().points().empty()) {
    double center_x = 0.0, center_y = 0.0;
    for (const auto& corner_point : parking_space->polygon().points()) {
      center_x += corner_point.x();
      center_y += corner_point.y();
    }
    center_x /= double(parking_space->polygon().points().size());
    center_y /= double(parking_space->polygon().points().size());
    waypoint_info.mutable_pose()->set_x(center_x);
    waypoint_info.mutable_pose()->set_y(center_y);
  }
  // nearest point to find lane infor
  LaneWaypoint new_lane_waypoint(waypoint_info);
  PointENU neast_point = routing_math_ptr_->GetLaneNearestPointByIdAndPoint(
      parking_space->lane_id(), new_lane_waypoint);
  double accumulate_s = 0.0;
  double lateral = 0.0;
  common::math::Vec2d point;
  point.set_x(neast_point.x());
  point.set_y(neast_point.y());
  auto target_lane = hdmap_->GetLaneById(parking_space->lane_id());
  if (!target_lane->GetProjection(point, &accumulate_s, &lateral)) {
    AINFO << "GetProjection is false";
    waypoint_info.set_s(0.0);
  }
  waypoint_info.set_s(accumulate_s);
  AINFO << "accumulate_s:" << accumulate_s;
  waypoint_info.set_heading(parking_space->parking_space().heading());
  new_lane_waypoint.mutable_pose()->set_x(neast_point.x());
  new_lane_waypoint.mutable_pose()->set_y(neast_point.y());
  new_lane_waypoint.set_heading(target_lane->Heading(accumulate_s));
  AINFO << "neast point:" << neast_point.x() << "," << neast_point.y();
  std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
  hdmap_->GetLanes(new_lane_waypoint.pose(), FLAGS_adc_lane_range, &lanes);
  GetWaypointInfo(index, lanes, new_lane_waypoint, true);
  // additional lane waypoint change to parking space waypoint
  if (additional_lane_waypoint_map_.count(index) != 0) {
    for (auto& waypoint : additional_lane_waypoint_map_[index]) {
      waypoint.mutable_pose()->set_x(waypoint_info.pose().x());
      waypoint.mutable_pose()->set_y(waypoint_info.pose().y());
    }
  }
  return true;
}

bool Routing::GetWaypointInfo(
    const int index,
    const std::vector<std::shared_ptr<const hdmap::LaneInfo>>& lanes,
    LaneWaypoint& waypoint_info, bool is_additional_lane, bool set_lane_point) {
  bool have_routing_lane =
      FindBestMatchingLane(index, lanes, waypoint_info, is_additional_lane);
  if (!have_routing_lane) {
    AERROR << "No get lane.";
    if (!HandleNearestLane(index, waypoint_info, is_additional_lane)) {
      return false;
    }
  }
  if (set_lane_point && index != 0) {
    SnapWaypointToLane(waypoint_info);
  }
  return true;
}

bool Routing::FindBestMatchingLane(
    const int index,
    const std::vector<std::shared_ptr<const hdmap::LaneInfo>>& lanes,
    LaneWaypoint& waypoint_info, bool is_additional_lane) {
  bool have_routing_lane = false;
  double min_diff_heading = DBL_MAX;
  for (const auto& lane : lanes) {
    double s = 0.0;
    if (!routing_math_ptr_->IsLaneUsable(lane, waypoint_info.pose(), s)) {
      continue;
    }
    double current_diff_heading = std::fabs(common::math::NormalizeAngle(
        lane->Heading(s) - waypoint_info.heading()));
    if (!IsLaneHeadingAcceptable(index, lane, s, current_diff_heading,
                                 waypoint_info)) {
      continue;
    }
    have_routing_lane = true;
    current_diff_heading =
        AdjustDiffHeadingForNoTurn(lane, current_diff_heading);
    if (current_diff_heading < min_diff_heading) {
      waypoint_info.set_id(lane->id().id());
      waypoint_info.set_s(s);
      min_diff_heading = current_diff_heading;
    }
    if (is_additional_lane) {
      AINFO << index << "   additional lane: " << lane->id().id();
      AddAdditionalLaneWaypoint(index, lane, s, waypoint_info);
    }
  }

  return have_routing_lane;
}

bool Routing::IsLaneHeadingAcceptable(
    const int index, const std::shared_ptr<const hdmap::LaneInfo>& lane,
    double s, double diff_heading, const LaneWaypoint& waypoint_info) {
  if (index != 0) {
    return true;
  }
  const bool is_heading_too_large =
      diff_heading > FLAGS_adc_lane_heading_diff;
  const bool is_not_opposite =
      diff_heading < (M_PI - FLAGS_adc_lane_heading_diff);

  if (is_heading_too_large && is_not_opposite) {
    AERROR << lane->id().id()
           << " not used, current_diff_heading: " << diff_heading << "="
           << lane->Heading(s) << "," << waypoint_info.heading();
    return false;
  }
  return true;
}

double Routing::AdjustDiffHeadingForNoTurn(
    const std::shared_ptr<const hdmap::LaneInfo>& lane, double diff_heading) {
  if (century::hdmap::Lane::NO_TURN == lane->lane().turn() &&
      diff_heading < FLAGS_adc_lane_heading_diff) {
    return 0.0;
  }
  return diff_heading;
}

void Routing::AddAdditionalLaneWaypoint(
    const int index, const std::shared_ptr<const hdmap::LaneInfo>& lane,
    double s, const LaneWaypoint& waypoint_info) {
  LaneWaypoint new_lane_waypoint(waypoint_info);
  new_lane_waypoint.set_id(lane->id().id());
  new_lane_waypoint.set_s(s);
  additional_lane_waypoint_map_[index].emplace_back(
      std::move(new_lane_waypoint));
}

bool Routing::HandleNearestLane(const int index, LaneWaypoint& waypoint_info,
                                bool is_additional_lane) {
  double nearest_s, nearest_l;
  std::shared_ptr<const hdmap::LaneInfo> lane;
  if (hdmap_->GetNearestLane(waypoint_info.pose(), &lane, &nearest_s,
                             &nearest_l) < 0) {
    AERROR << "get nearest lane failed ! ";
    return false;
  }
  waypoint_info.set_s(nearest_s);
  waypoint_info.set_id(lane->id().id());
  if (is_additional_lane) {
    CollectNearbyLanes(index, waypoint_info, lane);
  }
  return true;
}

void Routing::CollectNearbyLanes(
    const int index, const LaneWaypoint& waypoint_info,
    const std::shared_ptr<const hdmap::LaneInfo>& lane) {
  common::math::Vec2d original_point(waypoint_info.pose().x(),
                                     waypoint_info.pose().y());
  double accumulate_s = 0.0;
  double lateral = 0.0;
  if (!lane->GetProjection(original_point, &accumulate_s, &lateral)) {
    return;
  }
  const common::PointENU& projection_point = lane->GetSmoothPoint(accumulate_s);
  std::vector<std::shared_ptr<const hdmap::LaneInfo>> nearby_lanes;
  if (hdmap_->GetLanesWithNearPos(projection_point, FLAGS_min_adc_lane_range,
                                  &nearby_lanes) != 0) {
    return;
  }
  AddNearbyLaneWaypoints(index, nearby_lanes, projection_point, waypoint_info);
}

void Routing::AddNearbyLaneWaypoints(
    const int index,
    const std::vector<std::shared_ptr<const hdmap::LaneInfo>>& nearby_lanes,
    const common::PointENU& projection_point,
    const LaneWaypoint& waypoint_info) {
  const common::math::Vec2d vec_point(projection_point.x(),
                                      projection_point.y());
  for (const auto& nearby_lane : nearby_lanes) {
    double nearby_s = 0.0;
    double nearby_lateral = 0.0;
    if (nearby_lane->GetProjection(vec_point, &nearby_s, &nearby_lateral)) {
      LaneWaypoint nearby_waypoint;
      nearby_waypoint.set_id(nearby_lane->id().id());
      nearby_waypoint.set_s(nearby_s);
      nearby_waypoint.mutable_pose()->set_x(projection_point.x());
      nearby_waypoint.mutable_pose()->set_y(projection_point.y());
      nearby_waypoint.set_heading(waypoint_info.heading());
      additional_lane_waypoint_map_[index].emplace_back(nearby_waypoint);
      AINFO << index << "   additional lane: " << nearby_lane->id().id();
    }
  }
}

void Routing::SnapWaypointToLane(LaneWaypoint& waypoint_info) {
  auto waypoint_lane =
      hdmap_->GetLaneById(hdmap::MakeMapId(waypoint_info.id()));
  double distance_temp = 0.0;
  common::math::Vec2d point(waypoint_info.pose().x(), waypoint_info.pose().y());
  const auto& nearest_point =
      waypoint_lane->GetNearestPoint(point, &distance_temp);
  waypoint_info.mutable_pose()->set_x(nearest_point.x());
  waypoint_info.mutable_pose()->set_y(nearest_point.y());
}

common::PointENU Routing::CreatePointENU(double x, double y) {
  common::PointENU point;
  point.set_x(x);
  point.set_y(y);
  return point;
}

void Routing::ApplyHeadingAndPushRequest(RoutingRequest& new_request,
                                         LaneWaypoint* first_way_point,
                                         double adc_heading,
                                         double lane_heading) {
  double diff_heading =
      century::common::math::NormalizeAngle(adc_heading - lane_heading);
  if (std::fabs(diff_heading) > 3 * M_PI_4) {
    double heading = century::common::math::NormalizeAngle(adc_heading + M_PI);
    first_way_point->set_heading(heading);
    new_request.set_is_backward(is_heading_reverse_ ? false : true);
  } else {
    new_request.set_is_backward(is_heading_reverse_ ? true : false);
  }
  fixed_routing_request_.push_back(new_request);
}

void Routing::GenerateFourWaypointRoutes(const LaneWaypoint& adc_waypoint,
                                         const std::vector<LaneWaypoint>& vec1,
                                         double adc_heading,
                                         double lane_heading,
                                         const RoutingRequest& target_request) {
  for (size_t i = 0; i < next_point_lane_waypoint_.size(); ++i) {
    for (size_t j = 0; j < next_point_lane_waypoint_.size(); ++j) {
      // Skip same waypoint
      if (next_point_lane_waypoint_[i].id() ==
          next_point_lane_waypoint_[j].id()) {
        continue;
      }
      for (const auto& end_waypoint : vec1) {
        RoutingRequest new_request(target_request);
        new_request.clear_waypoint();
        new_request.add_waypoint()->CopyFrom(adc_waypoint);
        new_request.add_waypoint()->CopyFrom(next_point_lane_waypoint_[i]);
        new_request.add_waypoint()->CopyFrom(next_point_lane_waypoint_[j]);
        new_request.add_waypoint()->CopyFrom(end_waypoint);
        auto* first_way_point = new_request.mutable_waypoint(0);
        new_request.mutable_rerouting_info()->set_is_rerouting(true);
        new_request.mutable_rerouting_info()->set_is_merge_routing(true);
        ApplyHeadingAndPushRequest(new_request, first_way_point, adc_heading,
                                   lane_heading);
      }
    }
  }
}

bool Routing::GenerateAdditionalRoutingRequests(
    const RoutingRequest& target_request) {
  int waypoint_size = target_request.waypoint().size() - 1;
  auto back_waypoint = target_request.waypoint(waypoint_size);
  fixed_routing_request_.clear();
  if (additional_lane_waypoint_map_.size() <= 1 ||
      additional_lane_waypoint_map_.find(0) ==
          additional_lane_waypoint_map_.end() ||
      additional_lane_waypoint_map_.find(waypoint_size) ==
          additional_lane_waypoint_map_.end()) {
    return false;
  }
  const auto& vec0 = additional_lane_waypoint_map_[0];
  const auto& vec1 = additional_lane_waypoint_map_[waypoint_size];
  for (const auto& adc_waypoint : vec0) {
    auto adc_lane = hdmap_->GetLaneById(hdmap::MakeMapId(adc_waypoint.id()));
    auto s = adc_waypoint.s();
    double lane_heading = adc_lane->Heading(s);
    double adc_heading = adc_waypoint.heading();
    for (const auto traget_waypoint : vec1) {
      RoutingRequest new_request(target_request);
      new_request.clear_waypoint();
      new_request.add_waypoint()->CopyFrom(adc_waypoint);
      new_request.add_waypoint()->CopyFrom(traget_waypoint);
      auto* first_way_point = new_request.mutable_waypoint(0);
      ApplyHeadingAndPushRequest(new_request, first_way_point, adc_heading,
                                 lane_heading);
    }
    if (!is_add_rerouting_lane_) {
      continue;
    }
    if (target_request.rerouting_info().is_rerouting()) {
      for (const auto& traget_waypoint : next_point_lane_waypoint_) {
        RoutingRequest new_request(target_request);
        new_request.clear_waypoint();
        new_request.add_waypoint()->CopyFrom(adc_waypoint);
        new_request.add_waypoint()->CopyFrom(traget_waypoint);
        auto* first_way_point = new_request.mutable_waypoint(0);
        new_request.mutable_rerouting_info()->set_is_rerouting(false);
        new_request.mutable_rerouting_info()->set_is_skip_rerouting(true);
        ApplyHeadingAndPushRequest(new_request, first_way_point, adc_heading,
                                   lane_heading);
      }
    } else if (next_point_lane_waypoint_.size() > 1) {
      // is_two_routing == false: vec0 -> next_point_lane_waypoint_ ->
      // next_point_lane_waypoint_ -> vec1 Generate 4 waypoints: vec0 ->
      // next[i] -> next[j] -> vec1 Skip when next[i].id() == next[j].id()
      // or next_point_lane_waypoint_ has only 1 element
      GenerateFourWaypointRoutes(adc_waypoint, vec1, adc_heading, lane_heading,
                                 target_request);
    }
  }
  LogRoutingRequestsInfo(fixed_routing_request_);
  return true;
}

void Routing::LogRoutingRequestsInfo(
    const std::vector<RoutingRequest>& routing_requests) {
  for (size_t req_idx = 0; req_idx < routing_requests.size(); ++req_idx) {
    const auto& request = routing_requests[req_idx];
    std::stringstream waypoint_info;
    waypoint_info << "req[" << req_idx << "," << request.waypoint().size()
                  << "]: ";
    for (int i = 0; i < request.waypoint().size(); ++i) {
      const auto& wp = request.waypoint(i);
      waypoint_info << "wp[" << i << "]:" << wp.id() << "(" << wp.pose().x()
                    << "," << wp.pose().y() << ")";
      if (i < request.waypoint().size() - 1) {
        waypoint_info << " -> ";
      }
    }
    AINFO << waypoint_info.str();
    AINFO << "backward:" << request.is_backward()
          << ", is_merge_routing:" << request.rerouting_info().is_merge_routing()
          << ", is_two_routing:" << request.rerouting_info().is_rerouting()
          << ", is_skip_rerouting:" << request.rerouting_info().is_skip_rerouting();
  }
}

bool Routing::FillTwoWayTaskByOverlapPoint(
    const RoutingRequest& routing_request,
    std::vector<RoutingRequest>* const fixed_routing_request) {
  if (is_start_point_overlap_calculate_ ||
      FLAGS_enable_full_path_twoway_driving) {
    AINFO << "==FillTwoWayTaskByOverlapPoint==";
    auto target_request = fixed_routing_request->at(0);
    if (is_heading_reverse_) {
      for (auto& fixed_routing_temp : *fixed_routing_request) {
        if (fixed_routing_temp.is_backward()) {
          fixed_routing_temp.set_is_backward(false);
        } else {
          fixed_routing_temp.set_is_backward(true);
        }
      }
    }
    if (target_request.waypoint().size() < 1) {
      AERROR << "no has less two point";
      return false;
    }
    if (GenerateAdditionalRoutingRequests(target_request)) {
      return true;
    }
  }
  return false;
}

bool Routing::SetReRoutingPoint(RoutingRequest* const fixed_request,
                                const int junction_type, bool is_human_shaped) {
  auto second_waypoint =
      fixed_request->mutable_waypoint(fixed_request->waypoint().size() - 1);
  auto front_waypoint = fixed_request->waypoint(0);

  AINFO << "fixed_request->rerouting_info().huaman_shaped().level():"
        << fixed_request->rerouting_info().huaman_shaped().level();

  if (fixed_request->rerouting_info().huaman_shaped().level() > FLAGS_max_rerouting_times) {
    human_shape_points_.clear();
  }

  if (!is_human_shaped ||
      fixed_request->rerouting_info().huaman_shaped().level() <= FLAGS_max_rerouting_times) {
    return HandleReroutingWithExistingPoints(fixed_request, second_waypoint);
  }

  last_point_.set_x(second_waypoint->pose().x());
  last_point_.set_y(second_waypoint->pose().y());

  const bool is_two_point_distance_far =
      routing_math_ptr_->GetTwoPointsDistance(
          front_waypoint.pose(), second_waypoint->pose()) > FLAGS_two_point_distance;

  double temp_human_shape_distance = FLAGS_human_shape_distance;
  if (FLAGS_enable_yard_static_wait_distance &&
      routing::YARD_WAITINGAREA_STATIC == fixed_request->task_type()) {
    temp_human_shape_distance += FLAGS_yard_static_wait_distance;
  }

  RoutingState state;
  if (!ProcessJunctionRouting(junction_type, front_waypoint, second_waypoint,
                              is_two_point_distance_far,
                              temp_human_shape_distance, &state)) {
    return false;
  }

  UpdateSecondWaypointAndRequest(fixed_request, second_waypoint, state);
  SetFinalRoutingState(fixed_request);
  return true;
}

bool Routing::HandleReroutingWithExistingPoints(
    RoutingRequest* const fixed_request, LaneWaypoint* second_waypoint) {
  const auto kValidLocalRoutingTypes = {routing::ROUTING_J4_TO_T4T5_EAST,
                                        routing::ROUTING_T4T5_TO_J4_WEST,
                                        routing::ROUTING_J4_TO_T4T5_MIDDLE,
                                        routing::ROUTING_T4T5_TO_J4_MIDDLE,
                                        routing::ROUTING_J4_TO_T4T5_EAST_NEAR,
                                        routing::ROUTING_J4_TO_T4T5,
                                        routing::ROUTING_T4T5_TO_J4};

  const auto local_routing_type = fixed_request->local_routing_type();
  const bool is_valid_type =
      std::find(kValidLocalRoutingTypes.begin(), kValidLocalRoutingTypes.end(),
                local_routing_type) != kValidLocalRoutingTypes.end();

  if (!is_valid_type || human_shape_points_.empty()) {
    return false;
  }

  if (fixed_request->local_routing_type() != human_shape_points_[0].first) {
    return false;
  }

  if (FLAGS_max_rerouting_times == fixed_request->rerouting_info().huaman_shaped().level()) {
    second_waypoint->mutable_pose()->set_x(last_point_.x());
    second_waypoint->mutable_pose()->set_y(last_point_.y());
    is_start_point_overlap_calculate_ = true;
    is_human_shaped_ = true;
    human_shape_points_.clear();
    return true;
  }

  const auto& fixed_point =
      human_shape_points_[0].second.at(fixed_request->rerouting_info().huaman_shaped().level());
  second_waypoint->mutable_pose()->set_x(fixed_point.x());
  second_waypoint->mutable_pose()->set_y(fixed_point.y());
  SetFinalRoutingState(fixed_request);
  return true;
}

bool Routing::ProcessJunctionRouting(const int junction_type,
                                     const LaneWaypoint& front_waypoint,
                                     LaneWaypoint* second_waypoint,
                                     bool is_two_point_distance_far,
                                     double temp_human_shape_distance,
                                     RoutingState* state) {
  if (junction_type == Junction::J4_1) {
    AINFO << "--------> Junction::J4_1 <--------";
    return ProcessJ4_1Routing(front_waypoint, second_waypoint,
                              is_two_point_distance_far,
                              temp_human_shape_distance, state);
  }
  if (junction_type == Junction::T4_T5_T8) {
    AINFO << "--------> Junction::T4_T5_T8 <--------";
    return ProcessT4T5T8Routing(front_waypoint, second_waypoint,
                                is_two_point_distance_far,
                                temp_human_shape_distance, state);
  }
  AERROR << "junction_type is error, not to drive using huaman shaped";
  return false;
}

bool Routing::ProcessJ4_1Routing(const LaneWaypoint& front_waypoint,
                                 LaneWaypoint* second_waypoint,
                                 bool is_two_point_distance_far,
                                 double temp_human_shape_distance,
                                 RoutingState* state) {
  if (routing_math_ptr_->IsRelativeTargetPointToWeat(
          front_waypoint.pose(), second_waypoint->pose(), "HS_J4_5")) {
    AINFO << "-- ego is at the west, considering the target point";
    ProcessTargetPointWest(second_waypoint, temp_human_shape_distance, state);
  } else {
    AINFO << "-- ego is at the east, considering the ego point";
    ProcessEgoPointEast(front_waypoint, is_two_point_distance_far,
                        temp_human_shape_distance, state);
  }
  return true;
}

void Routing::ProcessTargetPointWest(LaneWaypoint* second_waypoint,
                                     double temp_human_shape_distance,
                                     RoutingState* state) {
  auto point = routing_math_ptr_->MoveAlongLine(
      second_waypoint->pose(), "HS_J4_5", temp_human_shape_distance, true);

  if (routing_math_ptr_->IsPointWithinRangeLine(point, "HS_J4_5")) {
    human_shape_points_.emplace_back(
        routing::ROUTING_J4_TO_T4T5,
        routing_math_ptr_->GetHumanShapePoints(point, "HS_J4_5", false, false));
    AINFO << "-- target point moves eastward and in the line";
  } else {
    AddBoundaryPoint(routing::ROUTING_J4_TO_T4T5, "HS_J4_5", false, false);
    AINFO << "-- target point moves eastward and reaches the critical "
             "point in the east";
  }
}

void Routing::ProcessEgoPointEast(const LaneWaypoint& front_waypoint,
                                  bool is_two_point_distance_far,
                                  double temp_human_shape_distance,
                                  RoutingState* state) {
  auto index_J4_1_point = routing_math_ptr_->GetFixedPoints("HS_J4_1");

  if (routing_math_ptr_->IsFixedPointClose(front_waypoint.pose(),
                                           index_J4_1_point.front())) {
    human_shape_points_.emplace_back(
        routing::ROUTING_J4_TO_T4T5,
        routing_math_ptr_->GetHumanShapePoints(index_J4_1_point.front(),
                                               "HS_J4_1", true, false));
    state->is_close_boundary_point = true;
    state->is_in_J4_east = true;
    AINFO << "-- ego point reaches the critical point in the east";
  } else if (!routing_math_ptr_->IsPointWithinRangeLine(front_waypoint.pose(),
                                                        "HS_J4_1")) {
    AddBoundaryPoint(routing::ROUTING_J4_TO_T4T5, "HS_J4_1", true, false);
    state->is_in_J4_east = true;
    AINFO << "-- ego point not in the line";
  } else {
    auto point = routing_math_ptr_->MoveAlongLine(
        front_waypoint.pose(), "HS_J4_1",
        is_two_point_distance_far ? FLAGS_human_shape_distance
                                  : temp_human_shape_distance,
        true);
    if (routing_math_ptr_->IsPointWithinRangeLine(point, "HS_J4_1")) {
      human_shape_points_.emplace_back(routing::ROUTING_J4_TO_T4T5,
                                       routing_math_ptr_->GetHumanShapePoints(
                                           point, "HS_J4_1", true, false));
      AINFO << "-- ego point moves eastward and in the line";
    } else {
      AddBoundaryPoint(routing::ROUTING_J4_TO_T4T5, "HS_J4_1", true, false);
      state->is_in_near_J4_east = true;
      AINFO << "-- ego point moves eastward and not in the line";
    }
  }
}

bool Routing::ProcessT4T5T8Routing(const LaneWaypoint& front_waypoint,
                                   LaneWaypoint* second_waypoint,
                                   bool is_two_point_distance_far,
                                   double temp_human_shape_distance,
                                   RoutingState* state) {
  if (routing_math_ptr_->IsRelativeTargetPointToWeat(
          front_waypoint.pose(), second_waypoint->pose(), "HS_J4_1")) {
    AINFO << "-- ego is at the west, considering the ego point";
    ProcessT4T5T8West(front_waypoint, is_two_point_distance_far, state);
  } else {
    AINFO << "-- ego is at the east, considering the traget point";
    ProcessT4T5T8East(second_waypoint, temp_human_shape_distance, state);
  }

  if (routing_math_ptr_->IsPointInSpecificJunction(*second_waypoint,
                                                   Junction::J4_WEST)) {
    human_shape_points_.at(0).first = routing::ROUTING_T4T5_TO_J4_WEST;
  }
  return true;
}

void Routing::ProcessT4T5T8West(const LaneWaypoint& front_waypoint,
                                bool is_two_point_distance_far,
                                RoutingState* state) {
  auto index_J4_5_point = routing_math_ptr_->GetFixedPoints("HS_J4_5");

  if (routing_math_ptr_->IsFixedPointClose(front_waypoint.pose(),
                                           index_J4_5_point.back())) {
    human_shape_points_.emplace_back(
        routing::ROUTING_T4T5_TO_J4,
        routing_math_ptr_->GetHumanShapePoints(index_J4_5_point.back(),
                                               "HS_J4_5", true, true));
    state->is_close_boundary_point = true;
    AINFO << "-- ego point reaches the critical point in the west";
  } else {
    const double distance = is_two_point_distance_far
                                ? FLAGS_human_shape_distance
                                : FLAGS_human_shape_distance;
    auto point = routing_math_ptr_->MoveAlongLine(front_waypoint.pose(),
                                                  "HS_J4_5", distance, false);
    if (routing_math_ptr_->IsPointWithinRangeLine(point, "HS_J4_5")) {
      human_shape_points_.emplace_back(
          routing::ROUTING_T4T5_TO_J4,
          routing_math_ptr_->GetHumanShapePoints(point, "HS_J4_5", true, true));
      AINFO << "-- ego point moves westward and in the line";
    } else {
      human_shape_points_.emplace_back(
          routing::ROUTING_T4T5_TO_J4,
          routing_math_ptr_->GetHumanShapePoints(index_J4_5_point.back(),
                                                 "HS_J4_5", true, true));
      AINFO << "-- ego point moves westward and in the line";
    }
  }
}

void Routing::ProcessT4T5T8East(LaneWaypoint* second_waypoint,
                                double temp_human_shape_distance,
                                RoutingState* state) {
  auto index_J4_1_temp_point =
      routing_math_ptr_->GetFixedPoints("HS_J4_1_temp");

  if (routing_math_ptr_->IsFixedPointClose(second_waypoint->pose(),
                                           index_J4_1_temp_point.back()) ||
      !routing_math_ptr_->IsPointWithinRangeLine(second_waypoint->pose(),
                                                 "HS_J4_1_temp")) {
    human_shape_points_.emplace_back(
        routing::ROUTING_T4T5_TO_J4,
        routing_math_ptr_->GetHumanShapePoints(index_J4_1_temp_point.back(),
                                               "HS_J4_1", false, true));
    AINFO << "-- target point reaches the critical point in the west or "
             "not in the line";
  } else {
    auto point = routing_math_ptr_->MoveAlongLine(
        second_waypoint->pose(), "HS_J4_1_temp", temp_human_shape_distance,
        false);
    if (routing_math_ptr_->IsPointWithinRangeLine(point, "HS_J4_1_temp")) {
      human_shape_points_.emplace_back(routing::ROUTING_T4T5_TO_J4,
                                       routing_math_ptr_->GetHumanShapePoints(
                                           point, "HS_J4_1", false, true));
      AINFO << "-- target point moves westward and in the line";
    } else {
      human_shape_points_.emplace_back(
          routing::ROUTING_T4T5_TO_J4,
          routing_math_ptr_->GetHumanShapePoints(index_J4_1_temp_point.back(),
                                                 "HS_J4_1", false, true));
      AINFO << "-- target point moves westward and not in the line";
    }
  }
}

void Routing::AddBoundaryPoint(routing::LocalRoutingType type,
                               const std::string& line_id, bool move_along,
                               bool is_reverse) {
  auto fixed_points = routing_math_ptr_->GetFixedPoints(line_id);
  auto& point = move_along ? fixed_points.front() : fixed_points.back();
  human_shape_points_.emplace_back(
      type, routing_math_ptr_->GetHumanShapePoints(point, line_id, move_along,
                                                   is_reverse));
}

void Routing::UpdateSecondWaypointAndRequest(
    RoutingRequest* const fixed_request, LaneWaypoint* second_waypoint,
    const RoutingState& state) {
  auto& first_entry = human_shape_points_[0];

  if (state.is_in_J4_east) {
    first_entry.first = routing::ROUTING_J4_TO_T4T5_EAST;
  } else if (state.is_in_near_J4_east) {
    first_entry.first = routing::ROUTING_J4_TO_T4T5_EAST_NEAR;
  }

  fixed_request->set_local_routing_type(first_entry.first);

  if (state.is_close_boundary_point) {
    fixed_request->mutable_rerouting_info()->mutable_huaman_shaped()->set_level(1);
    const auto& fixed_point = first_entry.second.at(1);
    second_waypoint->mutable_pose()->set_x(fixed_point.x());
    second_waypoint->mutable_pose()->set_y(fixed_point.y());
  } else {
    fixed_request->mutable_rerouting_info()->mutable_huaman_shaped()->set_level(0);
    second_waypoint->mutable_pose()->set_x(first_entry.second.front().x());
    second_waypoint->mutable_pose()->set_y(first_entry.second.front().y());
  }
}

void Routing::SetFinalRoutingState(RoutingRequest* const fixed_request) {
  fixed_request->mutable_rerouting_info()->set_is_rerouting(true);
  is_start_point_overlap_calculate_ = true;
  is_human_shaped_ = true;
}

void Routing::SetReRoutingPoint(RoutingRequest* const fixed_request,
                                const std::string_view junction_type) {
  // save the last point to planning rerouting.
  auto second_waypoint =
      fixed_request->mutable_waypoint(fixed_request->waypoint().size() - 1);
  // auto second_way_point = fixed_request->mutable_waypoint()->begin() + 1;

  auto front_waypoint = fixed_request->waypoint(0);
  next_point_.set_x(second_waypoint->pose().x());
  next_point_.set_y(second_waypoint->pose().y());
  double second_point_x, second_point_y;

  auto temp_fixed_points = routing_math_ptr_->GetFixedPoints(junction_type);
  auto& front_point = temp_fixed_points.front();
  second_point_x = front_point.x();
  second_point_y = front_point.y();
  if ("rerouting_J4_EAST" == junction_type || "rerouting_J4" == junction_type) {
    double dx = front_waypoint.pose().x() - second_point_x;
    double dy = front_waypoint.pose().y() - second_point_y;
    double heading_front_to_back = std::atan2(dy, dx);
    double junction_distance = sqrt(pow(dx, 2) + pow(dy, 2));
    if (junction_distance < FLAGS_adc_two_point_distance) {
      return;
    }
    double diff_heading = common::math::NormalizeAngle(FLAGS_j4_east_fixed_heading -
                                                       heading_front_to_back);
    if (std::fabs(diff_heading) >= M_PI_2) {
      return;
    }
  } else if ("rerouting_J4_1_MIDDLE" == junction_type) {
    for (const auto& fixed_point : temp_fixed_points) {
      double dx = fixed_point.x() - front_waypoint.pose().x();
      double dy = fixed_point.y() - front_waypoint.pose().y();
      double junction_distance = sqrt(pow(dx, 2) + pow(dy, 2));
      if (junction_distance < FLAGS_adc_two_point_distance) {
        return;
      }
    }
  } else {
    if (temp_fixed_points.size() > 2) {
      for (const auto& fixed_point : temp_fixed_points) {
        double dx = fixed_point.x() - front_waypoint.pose().x();
        double dy = fixed_point.y() - front_waypoint.pose().y();
        double junction_distance = sqrt(pow(dx, 2) + pow(dy, 2));
        if (junction_distance < FLAGS_junction_two_point_distance) {
          break;
        }
        second_point_x = fixed_point.x();
        second_point_y = fixed_point.y();
      }
    }
  }
  second_waypoint->mutable_pose()->set_x(second_point_x);
  second_waypoint->mutable_pose()->set_y(second_point_y);

  if ("rerouting_DJZ_D_WEST" == junction_type) {
    fixed_request->mutable_rerouting_info()->mutable_huaman_shaped()->set_is_part_rerouting(true);
  } else {
    fixed_request->mutable_rerouting_info()->set_is_rerouting(true);
  }
  is_start_point_overlap_calculate_ = true;
}

bool Routing::DetectHuamanShapedRouting(
    RoutingRequest* const fixed_request) {
  auto front_waypoint = fixed_request->waypoint(0);
  auto back_waypoint =
      fixed_request->waypoint(fixed_request->waypoint().size() - 1);
  bool is_huaman_shaped_driver = false;
  auto two_waypoint_distance = routing_math_ptr_->GetTwoPointsDistance(
      front_waypoint.pose(), back_waypoint.pose());
  bool is_too_close = (two_waypoint_distance < FLAGS_human_shape_max_distance);
  if (FLAGS_enable_huaman_shaped_driver && is_too_close &&
      routing::NOPROCESS == fixed_request->multi_routing_type()) {
    if (routing_math_ptr_->IsPointInSpecificJunction(front_waypoint,
                                                     Junction::J4_1) &&
        routing_math_ptr_->IsPointInSpecificJunction(back_waypoint,
                                                     Junction::T4_T5_T8)) {
      AINFO << "start point in J4_1, end point in T4_T5_T8";
      if (SetReRoutingPoint(fixed_request, Junction::J4_1, true)) {
        is_huaman_shaped_driver = true;
      }
    } else if (routing_math_ptr_->IsPointInSpecificJunction(back_waypoint,
                                                            Junction::J4_1) &&
               routing_math_ptr_->IsPointInSpecificJunction(
                   front_waypoint, Junction::T4_T5_T8)) {
      AINFO << "start point in T4_T5_T8, end point in J4_1 not in J4_WEST";
      if (SetReRoutingPoint(fixed_request, Junction::T4_T5_T8, true)) {
        is_huaman_shaped_driver = true;
      }
    } else if (fixed_request->rerouting_info().huaman_shaped().level() <= FLAGS_max_rerouting_times) {
      AINFO << "--------huaman_shaped_level: "
            << fixed_request->rerouting_info().huaman_shaped().level();
      if (SetReRoutingPoint(fixed_request, Junction::T4_T5_T8, false)) {
        is_huaman_shaped_driver = true;
      }
    }
  }
  return is_huaman_shaped_driver;
}

void Routing::DetectJunctionAreaRouting(
    RoutingRequest* const fixed_request) {
  auto front_waypoint = fixed_request->waypoint(0);
  auto back_waypoint =
      fixed_request->waypoint(fixed_request->waypoint().size() - 1);
  // adc in j4_east and next point not in j4_east.
  if (routing_math_ptr_->IsPointInSpecificJunction(front_waypoint,
                                                   Junction::J4_EAST) &&
      !routing_math_ptr_->IsPointInSpecificJunction(
          back_waypoint, Junction::BLOCKING_AREA_J4) &&
      !routing_math_ptr_->IsPointInSpecificJunction(back_waypoint,
                                                    Junction::T4_T5_T8)) {
    AINFO << "start point in J4_EAST, end point not in J4 and T4_T5_T8";
    SetReRoutingPoint(fixed_request, "rerouting_J4_EAST");
    in_special_areas_ = Junction::J4_EAST;
  } else if (routing_math_ptr_->IsPointInSpecificJunction(
                 front_waypoint, Junction::J4_EAST) &&
             !routing_math_ptr_->IsPointInSpecificJunction(
                 back_waypoint, Junction::BLOCKING_AREA_J4) &&
             routing_math_ptr_->IsPointInSpecificJunction(
                 back_waypoint, Junction::T4_T5_T8)) {
    AINFO << "start point in J4_EAST, end point in T4_T5_T8 and not in J4";
    SetReRoutingPoint(fixed_request, "rerouting_J4");
    // in J1_EAST area, to working outside the J1
  } else if (routing_math_ptr_->IsPointInSpecificJunction(
                 front_waypoint, Junction::J1_EAST) &&
             !routing_math_ptr_->IsPointInSpecificJunction(
                 back_waypoint, Junction::BLOCKING_AREA_J1)) {
    AINFO << "start point in J1_EAST, end point not in J1";
    SetReRoutingPoint(fixed_request, "rerouting_J1_EAST");
    is_special_area_no_rerouting_ = true;
    // not in J4 area, to working at the J4_WEST
  } else if (routing_math_ptr_->IsPointInSpecificJunction(
                 back_waypoint, Junction::J4_WEST) &&
             !routing_math_ptr_->IsPointInSpecificJunction(
                 front_waypoint, Junction::BLOCKING_AREA_J4)) {
    AINFO << "start point not in J4_WEST, end point in J4_WEST";
    is_special_area_no_rerouting_ = true;
    SetReRoutingPoint(fixed_request, "rerouting_J4_WEST");
    // in J4 or J1 area, to working at the J4_WEST or J1_WEST
  } else if (!routing_math_ptr_->IsPointInSpecificJunction(
                 front_waypoint, Junction::BLOCKING_AREA_J1) &&
             routing_math_ptr_->IsPointInSpecificJunction(
                 back_waypoint, Junction::J1_WEST)) {
    AINFO << "start point not in J1, end point in J1_WEST";
    // SetReRoutingPoint(fixed_request, "rerouting_J1_WEST");
    // not in J2-J3 area, to working at the J2_WEST
  } else if (!routing_math_ptr_->IsPointInSpecificJunction(
                 front_waypoint, Junction::BLOCKING_AREA_J2J3) &&
             routing_math_ptr_->IsPointInSpecificJunction(
                 back_waypoint, Junction::J2_WEST)) {
    AINFO << "start point not in J1J3, end point in J2_WEST";
    SetReRoutingPoint(fixed_request, "rerouting_J2_WEST");
  } else if (routing_math_ptr_->IsPointInSpecificJunction(
                 front_waypoint, Junction::J2_EAST) &&
             !routing_math_ptr_->IsPointInSpecificJunction(
                 back_waypoint, Junction::BLOCKING_AREA_J2J3)) {
    AINFO << "start point in J2_EAST, end point not in J2J3";
    SetReRoutingPoint(fixed_request, "rerouting_J2_EAST");
  } else if (!routing_math_ptr_->IsPointInSpecificJunction(
                 front_waypoint, Junction::BLOCKING_AREA_J2J3) &&
             routing_math_ptr_->IsPointInSpecificJunction(
                 back_waypoint, Junction::J3_WEST)) {
    AINFO << "start point not in J2J3, end point in J3_WEST";
    SetReRoutingPoint(fixed_request, "rerouting_J3_WEST");
  } else if (routing_math_ptr_->IsPointInSpecificJunction(
                 front_waypoint, Junction::J3_EAST) &&
             !routing_math_ptr_->IsPointInSpecificJunction(
                 back_waypoint, Junction::BLOCKING_AREA_J2J3)) {
    AINFO << "start point in J3_EAST, end point not in J2J3";
    SetReRoutingPoint(fixed_request, "rerouting_J3_EAST");
  } else if (routing_math_ptr_->IsPointInSpecificJunction(
                 front_waypoint, Junction::J4_1_MIDDLE) &&
             (routing_math_ptr_->IsPointInSpecificJunction(
                  back_waypoint, Junction::G_AREA) ||
              routing_math_ptr_->IsPointInSpecificJunction(
                  back_waypoint, Junction::T6_T7_W1_W2))) {
    AINFO << "start point in J4_1_MIDDLE, end point in G_AREA or T6_T7_W1_W2";
    SetReRoutingPoint(fixed_request, "rerouting_J4_1_MIDDLE");
  } else if (routing_math_ptr_->IsPointInSpecificJunction(
                 back_waypoint, Junction::J4_EAST) &&
             (!routing_math_ptr_->IsPointInSpecificJunction(
                  front_waypoint, Junction::T4_T5_T8) &&
              !routing_math_ptr_->IsPointInSpecificJunction(
                  front_waypoint, Junction::BLOCKING_AREA_J4) &&
              !routing_math_ptr_->IsPointInSpecificJunction(
                  front_waypoint, Junction::J4_1_MIDDLE))) {
    AINFO << "end point in J4_EAST, start point in T4_T5_T8 BLOCKING_AREA_J4 "
             "or J4_1_MIDDLE";
    SetReRoutingPoint(fixed_request, "rrouting_J4_EAST");
    is_special_area_no_rerouting_ = true;
  } else {
    is_start_point_overlap_calculate_ = false;
  }
  if (routing_math_ptr_->IsPointInSpecificJunction(back_waypoint,
                                                   Junction::G_AREA) ||
      routing_math_ptr_->IsPointInSpecificJunction(back_waypoint,
                                                   Junction::D_M3L_AREA) ||
      routing_math_ptr_->IsPointInSpecificJunction(back_waypoint,
                                                   Junction::M3H_M2L_AREA) ||
      routing_math_ptr_->IsPointInSpecificJunction(back_waypoint,
                                                   Junction::M2H_M1_AREA)) {
    AINFO << "end point maybe in G_AREA M2H_M1_AREA M3H_M2L_AREA D_M3L_AREA";
    is_lane_change_detector_ = false;
  }
}

void Routing::SetReverseReRoutingPoint(RoutingRequest* const fixed_request) {
  auto* last_waypoint =
      fixed_request->mutable_waypoint(fixed_request->waypoint().size() - 1);
  next_point_.set_x(last_waypoint->pose().x());
  next_point_.set_y(last_waypoint->pose().y());
  const auto& backward_pt =
      fixed_request->rerouting_info().reverse_rerouting().backward_point();
  last_waypoint->mutable_pose()->set_x(backward_pt.pose().x());
  last_waypoint->mutable_pose()->set_y(backward_pt.pose().y());
  last_waypoint->set_id(backward_pt.id());
  last_waypoint->set_s(backward_pt.s());
  fixed_request->mutable_rerouting_info()->set_is_rerouting(true);
}

void Routing::IsNeedTwoRouting(RoutingRequest* const fixed_request) {
  AINFO << "== IsNeedTwoRouting ==";
  if (fixed_request->waypoint_size() < 2) {
    is_start_point_overlap_calculate_ = false;
    AINFO << "fixed_request->waypoint_size() is less 2";
    return;
  }
  auto front_waypoint = fixed_request->waypoint(0);
  auto back_waypoint =
      fixed_request->waypoint(fixed_request->waypoint().size() - 1);
  const int initial_huaman_shaped_level =
      fixed_request->rerouting_info().huaman_shaped().level();
  fixed_request->set_is_reverse_trajectory(false);
  fixed_request->set_is_backward(false);
  fixed_request->mutable_rerouting_info()->set_is_rerouting(false);
  fixed_request->mutable_rerouting_info()->set_is_skip_rerouting(false);
  fixed_request->mutable_rerouting_info()->set_is_merge_routing(false);
  bool is_huaman_shaped_driver = false;
  if (routing::PART_ROUTING_HUMAN_SHAPED ==
      fixed_request->rerouting_info().huaman_shaped().part_routing_type()) {
    fixed_request->mutable_rerouting_info()
        ->mutable_huaman_shaped()
        ->set_part_routing_type(routing::PART_ROUTING_DEFAULT);
    SetReRoutingPoint(fixed_request, "rerouting_DJZ_D_WEST");
  }
  if (fixed_request->rerouting_info().reverse_rerouting().is_rerouting()) {
    SetReverseReRoutingPoint(fixed_request);
  } else {
    is_huaman_shaped_driver = DetectHuamanShapedRouting(fixed_request);
    if (!is_huaman_shaped_driver) {
      DetectJunctionAreaRouting(fixed_request);
    }
    if (!is_special_area_no_rerouting_ &&
        routing_math_ptr_->IsPointInSpecificJunction(front_waypoint,
                                                     Junction::J4_WEST) &&
        (!routing_math_ptr_->IsPointInSpecificJunction(back_waypoint,
                                                       Junction::T4_T5_T8) &&
         !routing_math_ptr_->IsPointInSpecificJunction(
             back_waypoint, Junction::BLOCKING_AREA_J4) &&
         !routing_math_ptr_->IsPointInSpecificJunction(
             back_waypoint, Junction::J4_1_MIDDLE))) {
      AINFO << "start point in J4_WEST, end point in T4_T5_T8 BLOCKING_AREA_J4 "
               "or J4_1_MIDDLE";
      is_special_area_no_rerouting_ = true;
      auto temp_fixed_points =
          routing_math_ptr_->GetFixedPoints("rerouting_J4_WEST");
      next_point_.set_x(temp_fixed_points.front().x());
      next_point_.set_y(temp_fixed_points.front().y());
    }
    if (!is_special_area_no_rerouting_ &&
        (routing_math_ptr_->IsPointInSpecificJunction(front_waypoint,
                                                     Junction::J1_EAST) ||
        routing_math_ptr_->IsPointInSpecificJunction(back_waypoint,
                                                       Junction::J1_EAST))) {
      AINFO << "start or end point in J1_EAST";
      is_special_area_no_rerouting_ = true;
      auto temp_fixed_points =
          routing_math_ptr_->GetFixedPoints("rerouting_J1_EAST_near");
      next_point_.set_x(temp_fixed_points.front().x());
      next_point_.set_y(temp_fixed_points.front().y());
    }
    if (!is_special_area_no_rerouting_ &&
        (routing_math_ptr_->IsPointInSpecificJunction(front_waypoint,
                                                     Junction::J1_WEST) ||
        routing_math_ptr_->IsPointInSpecificJunction(back_waypoint,
                                                       Junction::J1_WEST))) {
      AINFO << "start or end point in J1_WEST";
      is_special_area_no_rerouting_ = true;
      auto temp_fixed_points =
          routing_math_ptr_->GetFixedPoints("rerouting_J1_WEST");
      next_point_.set_x(temp_fixed_points.front().x());
      next_point_.set_y(temp_fixed_points.front().y());
    }
  }
  if (FLAGS_enable_default_shortest_path_output &&
      FLAGS_enable_shortest_path_non_replan && is_special_area_no_rerouting_ &&
      is_need_shortest_route_) {
    AINFO << "search next point near LaneWaypoint";
    is_add_rerouting_lane_ = true;
    routing_math_ptr_->SearchNearLaneWaypoint(next_point_,
                                              next_point_lane_waypoint_);
  }
  if ("DJZ" == FLAGS_project_scenario && FLAGS_enable_DJZ_demonstrate) {
    SetReRoutingPointForDemonstrate(fixed_request);
  }
  next_point_.set_x(back_waypoint.pose().x());
  next_point_.set_y(back_waypoint.pose().y());
  if (is_human_shaped_) {
    auto next_routing_request_point = fixed_request->mutable_rerouting_info()
                                          ->mutable_huaman_shaped()
                                          ->mutable_next_waypoint()
                                          ->mutable_pose();
    next_routing_request_point->set_x(last_point_.x());
    next_routing_request_point->set_y(last_point_.y());
  } else if ((!is_huaman_shaped_driver &&
              fixed_request->rerouting_info().is_rerouting()) ||
             is_special_area_no_rerouting_) {
    auto last_routing_request_point = fixed_request->mutable_rerouting_info()
                                          ->mutable_last_waypoint()
                                          ->mutable_pose();
    last_routing_request_point->set_x(next_point_.x());
    last_routing_request_point->set_y(next_point_.y());
  }
  if (is_human_shaped_ && is_huaman_shaped_driver &&
      initial_huaman_shaped_level > FLAGS_max_rerouting_times &&
      fixed_request->rerouting_info().huaman_shaped().level() <
          FLAGS_max_rerouting_times) {
    auto last_routing_request_point = fixed_request->mutable_rerouting_info()
                                          ->mutable_last_waypoint()
                                          ->mutable_pose();
    last_routing_request_point->set_x(next_point_.x());
    last_routing_request_point->set_y(next_point_.y());
  }
}

void Routing::SetReRoutingPointForDemonstrate(
    RoutingRequest* const fixed_request) {
  if (fixed_request->task_type() != routing::YARD_WAITINGAREA_STATIC ||
      fixed_request->local_routing_type() == routing::ROUTING_PLANNING) {
    return;
  }
  auto front_waypoint = fixed_request->waypoint(0);
  auto* second_waypoint =
      fixed_request->mutable_waypoint(fixed_request->waypoint().size() - 1);
  auto demo_fixed_point = routing_math_ptr_->GetFixedPoints("DJZ_DEMO");
  AINFO << "demo_fixed_point size:" << demo_fixed_point.size();
  for (size_t i = 0; i < demo_fixed_point.size() / 4; ++i) {
    auto front_dist = routing_math_ptr_->PointToLineDistance(
        front_waypoint.pose(), demo_fixed_point[2 * i],
        demo_fixed_point[2 * i + 1]);
    if (front_dist >= FLAGS_adc_two_point_distance) {
      continue;
    }
    if (!routing_math_ptr_->IsPointWithinRangeLine(
            front_waypoint.pose(), demo_fixed_point[2 * i],
            demo_fixed_point[2 * i + 1])) {
      continue;
    }
    auto back_dist = routing_math_ptr_->PointToLineDistance(
        second_waypoint->pose(), demo_fixed_point[2 * i],
        demo_fixed_point[2 * i + 1]);
    if (back_dist < FLAGS_adc_two_point_distance &&
        routing_math_ptr_->IsPointWithinRangeLine(
            second_waypoint->pose(), demo_fixed_point[2 * i],
            demo_fixed_point[2 * i + 1])) {
      continue;
    }
    double min_diff_heading = DBL_MAX;
    auto point = front_waypoint.pose();
    std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
    hdmap_->GetLanes(point, FLAGS_adc_lane_range, &lanes);
    double move_heading = FLAGS_demo_heading;
    if (lanes.empty()) {
      return;
    }
    for (size_t j = 0; j < lanes.size(); ++j) {
      double s = 0.0;
      if (!routing_math_ptr_->IsLaneUsable(lanes[j], point, s)) {
        continue;
      }
      double current_diff_heading =
          std::fabs(common::math::NormalizeAngle(lanes[j]->Heading(s) -
                                                 FLAGS_demo_heading));
      if (current_diff_heading > M_PI_4) {
        continue;
      }
      if (min_diff_heading > current_diff_heading) {
        move_heading = lanes[j]->Heading(s);
        min_diff_heading = current_diff_heading;
      }
    }
    double move_diatance = FLAGS_DJZ_demonstrate_move_disatnce;
    if (FLAGS_DJZ_demonstrate_scheme == kMaxLaneChangeScheme || 1 == i) {
      move_diatance = FLAGS_DJZ_demonstrate_max_lane_change_disatnce;
    } else if (FLAGS_DJZ_demonstrate_scheme == kMinLaneChangeScheme) {
      move_diatance = FLAGS_DJZ_demonstrate_min_lane_change_disatnce;
    }
    double new_x =
        front_waypoint.pose().x() + move_diatance * std::cos(move_heading);
    double new_y =
        front_waypoint.pose().y() + move_diatance * std::sin(move_heading);
    if (FLAGS_DJZ_demonstrate_scheme == kMaxLaneChangeScheme ||
        FLAGS_DJZ_demonstrate_scheme == kMinLaneChangeScheme || 1 == i) {
      PointENU pt;
      pt.set_x(new_x);
      pt.set_y(new_y);
      auto project_point = routing_math_ptr_->ProjectPointToLine(
          pt, demo_fixed_point[2 * i + 4], demo_fixed_point[2 * i + 5]);
      new_x = project_point.x();
      new_y = project_point.y();
    }
    next_point_.set_x(second_waypoint->pose().x());
    next_point_.set_y(second_waypoint->pose().y());
    second_waypoint->mutable_pose()->set_x(new_x);
    second_waypoint->mutable_pose()->set_y(new_y);
    fixed_request->mutable_rerouting_info()->set_is_rerouting(true);
    is_start_point_overlap_calculate_ = true;
  }
}

void Routing::HandingTaskInvolvingDeadEndRoad(
    RoutingRequest* const fixed_request) {
  auto part_routing_type = fixed_request->rerouting_info().huaman_shaped().part_routing_type();
  AINFO << "======HandingTaskInvolvingDeadEndRoad=======";
  if (FLAGS_project_scenario != "DJZ" || !FLAGS_enable_djz_handing_dead_end ||
      fixed_request->rerouting_info().is_rerouting() || fixed_request->rerouting_info().huaman_shaped().is_part_rerouting() ||
      part_routing_type == routing::PART_ROUTING_HUMAN_SHAPED) {
    return;
  }
  if (!fixed_request->rerouting_info().dead_road().is_rerouting()) {
    if (IsDeadEndRoad(fixed_request)) {
    }
  } else if (fixed_request->rerouting_info().dead_road().is_rerouting() &&
             fixed_request->rerouting_info().dead_road().level() <
                 fixed_request->rerouting_info().dead_road().waypoint_size() - 1) {
    fixed_request->mutable_rerouting_info()->mutable_dead_road()->set_level(999);
    fixed_request->mutable_rerouting_info()->mutable_dead_road()->clear_waypoint();
    fixed_request->mutable_rerouting_info()->mutable_dead_road()->set_is_rerouting(false);
  }
}

void Routing::FillLaneInfoIfMissing(
    const RoutingRequest& routing_request,
    std::vector<RoutingRequest>* const fixed_routing_request) {
  CHECK_NOTNULL(fixed_routing_request);
  RoutingRequest fixed_request(routing_request);
  bool is_regular_type = true;
  if (routing_request.waypoint_size() < kTwowaypointSize) {
    AERROR << "routing_request waypoint size is less 2 ";
    return;
  }
  auto routing_task_type = fixed_request.task_type();
  auto local_routing_type = fixed_request.local_routing_type();
  auto part_routing_type = fixed_request.rerouting_info().huaman_shaped().part_routing_type();
  if (routing::ROUTING_PLANNING == local_routing_type ||
      routing::ROUTING_LOOP == local_routing_type ||
      routing::ROUTING_J4_TO_T4T5_EAST == local_routing_type ||
      routing::ROUTING_T4T5_TO_J4_WEST == local_routing_type ||
      routing::ROUTING_J4_TO_T4T5_MIDDLE == local_routing_type ||
      routing::ROUTING_T4T5_TO_J4_MIDDLE == local_routing_type ||
      routing::PART_ROUTING_HUMAN_SHAPED == part_routing_type) {
    routing_task_type = routing::DEFAULT;
  }
  // TODO(yt): add bidirectional dual box operation process.
  switch (routing_task_type) {
    case routing::TINY_ADJUSTMENT_FRONT:
    case routing::TINY_ADJUSTMENT_BACK:
    case routing::TINY_ADJUSTMENT_STOP:
    case routing::TINY_ADJUSTMENT_LEFT:
    case routing::TINY_ADJUSTMENT_RIGHT:
    case routing::LOADING_OPERATIONAREA_SAMEDIRECTION_2:
    case routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_2:
      fixed_request.clear_waypoint();
      FillFristOrAllPointLaneInfo(FRIST, routing_request, &fixed_request) &&
          FillEndPointLaneInfoForTinyAdjustment(routing_request,
                                                &fixed_request);
      break;
    case routing::RAILWAY_WAITINGAREA_DYNAMIC:
    case routing::RAILWAY_OPERATIONAREA_DYNAMIC:
    case routing::LOADING_OPERATIONAREA_SAMEDIRECTION_1:
    case routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_0:
    case routing::LOADING_OPERATIONAREA_SAMEDIRECTION_3_1:
    case routing::LOADING_OPERATIONAREA_SAMEDIRECTION_3_0:
    case routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_1:
    case routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_1:
      fixed_request.clear_waypoint();
      FillFristOrAllPointLaneInfo(FRIST, routing_request, &fixed_request) &&
          FillEndPointLaneInfoForProjection(routing_request, &fixed_request);
      break;
    default:
      AINFO << "task type may not be a regular type.";
      is_regular_type = false;
      break;
  }
  IsNeedTwoRouting(&fixed_request);
  HandingTaskInvolvingDeadEndRoad(&fixed_request);
  if (is_regular_type) {
    RoutingRequest temp_fixed_request(fixed_request);
    temp_fixed_request.clear_waypoint();
    FillFristOrAllPointLaneInfo(ALL, fixed_request, &temp_fixed_request);
  } else {
    RoutingRequest temp_fixed_request(fixed_request);
    fixed_request.clear_waypoint();
    FillFristOrAllPointLaneInfo(ALL, temp_fixed_request, &fixed_request);
  }
  fixed_routing_request->emplace_back(std::move(fixed_request));
  return;
}

bool Routing::FillParkingID(RoutingResponse* const routing_response) {
  CHECK_NOTNULL(routing_response);

  auto routing_request = routing_response->mutable_routing_request();
  routing_request->clear_parking_info();
  // routing_request->waypoint(0) is ego self position
  for (int i = 1; i < routing_request->waypoint_size(); ++i) {
    // get parking spaces
    std::vector<hdmap::ParkingSpaceInfoConstPtr> parking_spaces;
    common::math::Vec2d point(routing_request->waypoint(i).pose().x(),
                              routing_request->waypoint(i).pose().y());
    hdmap_->GetParkingSpaces(routing_request->waypoint(i).pose(), FLAGS_routing_lane_search_radius,
                             &parking_spaces);
    for (const auto& it : parking_spaces) {
      // waypoint is in parking space polygon
      if (it->polygon().IsPointIn(point)) {
        routing_request->mutable_parking_info()->set_parking_space_id(
            it->id().id());
        routing_request->mutable_parking_info()
            ->mutable_parking_point()
            ->CopyFrom(routing_request->waypoint(i).pose());
        routing_request->mutable_parking_info()
            ->mutable_corner_point()
            ->CopyFrom(it->parking_space().polygon());
        return true;
      }
    }
  }

  ADEBUG << "Failed to fill parking ID";
  return false;
}

bool Routing::IsNeedLanechangeFirst(
    std::vector<RoutingRequest>* const fixed_routing_requests) {
  AINFO << "== IsNeedLanechangeFirst ==";
  if (fixed_routing_requests->empty() || !FLAGS_enable_prefer_lane_change) {
    return false;
  }
  RoutingRequest fixed_request(
      fixed_routing_requests->at(fixed_routing_requests->size() - 1));
  if (fixed_request.waypoint_size() < 2) {
    return false;
  }
  auto front_waypoint = fixed_request.waypoint(0);
  auto back_waypoint =
      fixed_request.waypoint(fixed_request.waypoint().size() - 1);
  if ((routing_math_ptr_->IsPointInSpecificJunction(
           front_waypoint, Junction::BLOCKING_AREA_J4) &&
       routing_math_ptr_->IsPointInSpecificJunction(back_waypoint,
                                                    Junction::T4_T5_T8)) ||
      (routing_math_ptr_->IsPointInSpecificJunction(front_waypoint,
                                                    Junction::T4_T5_T8) &&
       routing_math_ptr_->IsPointInSpecificJunction(
           back_waypoint, Junction::BLOCKING_AREA_J4))) {
    // in_special_areas_ = Junction::BLOCKING_AREA_J4;
    return true;
  }
  if (FLAGS_enable_all_prefer_lane_change || "DJZ" == FLAGS_project_scenario) {
    return true;
  }
  return false;
}

void Routing::RunLoopRunning(
    const std::shared_ptr<RoutingRequest>& routing_request) {
  if (routing_request->waypoint_size() < kTwowaypointSize ||
      routing::ROUTING_PLANNING == routing_request->local_routing_type() ||
      routing::ROUTING_J4_TO_T4T5_EAST ==
          routing_request->local_routing_type() ||
      routing::ROUTING_T4T5_TO_J4_WEST ==
          routing_request->local_routing_type() ||
      routing::ROUTING_J4_TO_T4T5_MIDDLE ==
          routing_request->local_routing_type() ||
      routing::ROUTING_T4T5_TO_J4_MIDDLE ==
          routing_request->local_routing_type()) {
    return;
  }
  if (!FLAGS_enable_loop_running ||
      (!routing_request->is_loop_running() &&
       !routing_request->is_one_click_loop_running())) {
    return;
  }
  is_need_shortest_route_ = false;
  auto front_waypoint = routing_request->waypoint(0);
  auto* loop_run = routing_request->mutable_rerouting_info()->mutable_loop_run();
  if (loop_run->waypoint_size() < 1) {
    if ("QDG" != FLAGS_project_scenario) {
      return;
    }
    // The first point is the position of the vehicle itself
    if (routing_request->is_one_click_loop_running()) {
      auto temp_points = routing_math_ptr_->GetFixedPoints("LOOP_RUN_QDG");
      for (const auto& temp_point : temp_points) {
        double junction_distance = routing_math_ptr_->GetTwoPointsDistance(
            front_waypoint.pose(), temp_point);
        if (junction_distance < FLAGS_rerouting_distance) {
          continue;
        }
        routing_request->clear_waypoint();
        LaneWaypoint third_waypoint;
        auto third_point =
            routing_math_ptr_->GetFixedPoints("LOOP_QDG_D7").front();
        third_waypoint.mutable_pose()->set_x(third_point.x());
        third_waypoint.mutable_pose()->set_y(third_point.y());
        routing_request->add_waypoint()->CopyFrom(third_waypoint);
        LaneWaypoint second_waypoint;
        second_waypoint.mutable_pose()->set_x(temp_point.x());
        second_waypoint.mutable_pose()->set_y(temp_point.y());
        routing_request->add_waypoint()->CopyFrom(second_waypoint);
        break;
      }
    }
    for (int i = 0; i < routing_request->waypoint_size(); ++i) {
      LaneWaypoint lane_waypoint(routing_request->waypoint(i));
      if (0 == i) {
        loop_run->add_waypoint()->CopyFrom(front_waypoint);
      }
      loop_run->add_waypoint()->CopyFrom(lane_waypoint);
    }
  } else {
    loop_run->clear_waypoint();
    for (int i = 0; i < routing_request->waypoint_size(); ++i) {
      LaneWaypoint lane_waypoint(routing_request->waypoint(i));
      loop_run->add_waypoint()->CopyFrom(lane_waypoint);
    }
  }
  routing_request->clear_waypoint();
  if (loop_run->waypoint_size() < kTwowaypointSize + 1) {
    return;
  }
  for (int i = 0; i < kTwowaypointSize + 1; ++i) {
    if (1 == i) {
      continue;
    }
    LaneWaypoint lanewaypoint(loop_run->waypoint(i));
    routing_request->add_waypoint()->CopyFrom(lanewaypoint);
  }
}

bool Routing::IsRoutingFirstPointHeadingReverse(
    const std::shared_ptr<RoutingRequest>& routing_request) {
  const auto& lane_waypoint = routing_request->waypoint(0);
  veh_heading_ = lane_waypoint.heading();
  if (!FLAGS_enable_full_path_twoway_driving) {
    return false;
  }
  auto point = lane_waypoint.pose();
  std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
  for (int i = 0; i < FLAGS_lane_search_loop_count; ++i) {
    hdmap_->GetLanesWithHeading(point, FLAGS_routing_lane_search_radius + i * FLAGS_routing_lane_search_radius,
                                lane_waypoint.heading(), M_PI_4, &lanes);
    if (lanes.size() > 2) {
      break;
    }
  }
  if (lanes.empty()) {
    AINFO << "routing first point heading reverse!";
    auto first_way_point = routing_request->mutable_waypoint()->begin();
    double heading = century::common::math::NormalizeAngle(
        first_way_point->heading() + M_PI);
    first_way_point->set_heading(heading);
    return true;
  } else {
    return false;
  }
}

bool Routing::IsLaneChangeDistanceClose(
    const RoutingRequest& fixed_routing_request,
    const RoutingResponse& routing_response,
    const double& temp_routing_min_lenght) {
  if (!FLAGS_enable_lane_change_close_distance_check ||
      !is_need_lanechange_first_ ||
      temp_routing_min_lenght > FLAGS_lane_change_distance ||
      routing_response.road_size() < 1 ||
      routing_response.road(0).passage_size() < 3 ||
      Junction::J4_EAST == in_special_areas_ || is_human_shaped_) {
    return false;
  }
  if (routing_response.road_size() != 1) {
    return false;
  }

  auto front_waypoint = fixed_routing_request.waypoint(0);
  auto back_waypoint = fixed_routing_request.waypoint(
      fixed_routing_request.waypoint().size() - 1);
  double distance =
      std::hypot(back_waypoint.pose().x() - front_waypoint.pose().x(),
                 back_waypoint.pose().y() - front_waypoint.pose().y());
  double max_distance = 2.0 * temp_routing_min_lenght;
  if (distance > max_distance) {
    return false;
  }

  const auto& road = routing_response.road(0);
  bool front_in_road =
      routing_math_ptr_->IsLaneInRoad(front_waypoint.id(), road);
  bool back_in_road = routing_math_ptr_->IsLaneInRoad(back_waypoint.id(), road);

  if (!front_in_road || !back_in_road) {
    return false;
  }
  // Original heading check logic
  double dx = back_waypoint.pose().x() - front_waypoint.pose().x();
  double dy = back_waypoint.pose().y() - front_waypoint.pose().y();
  double heading_front_to_back = std::atan2(dy, dx);
  auto frist_lane = hdmap_->GetLaneById(hdmap::MakeMapId(front_waypoint.id()));
  auto end_lane = hdmap_->GetLaneById(hdmap::MakeMapId(back_waypoint.id()));
  double frist_diff_heading = common::math::NormalizeAngle(
      frist_lane->Heading(front_waypoint.s()) - heading_front_to_back);
  double end_diff_heading = common::math::NormalizeAngle(
      end_lane->Heading(back_waypoint.s()) - heading_front_to_back);
  if (std::fabs(frist_diff_heading) < M_PI_2 &&
      std::fabs(end_diff_heading) < M_PI_2) {
    return true;
  }
  return false;
}

bool Routing::IsSpecialAreasOnlyuseUturn(
    const RoutingResponse& routing_response) {
  if (!is_need_lanechange_first_ || routing_response.road_size() < 1 ||
      routing_response.road(0).passage_size() < 3 ||
      Junction::J4_EAST == in_special_areas_) {
    return true;
  }
  if (FLAGS_enable_only_u_turn && is_need_lanechange_first_ &&
      // in_special_areas_ == Junction::BLOCKING_AREA_J4 &&
      routing_response.road(0).passage_size() > 4) {
    return false;
  }
  return true;
}

bool Routing::IsUseReverseTrajectory(const double& routing_min_lenght,
                                     const RoutingRequest& routing_request,
                                     RoutingResponse* const routing_response) {
  is_use_reverse_trajectory_ = false;
  if (routing_min_lenght > 1.5 * routing_request.tiny_adjustment_distance() &&
      (routing_request.task_type() == routing::TINY_ADJUSTMENT_FRONT ||
       routing_request.task_type() == routing::TINY_ADJUSTMENT_BACK ||
       routing_request.task_type() == routing::TINY_ADJUSTMENT_LEFT ||
       routing_request.task_type() == routing::TINY_ADJUSTMENT_RIGHT)) {
    AINFO << "---------- Use reverse trajectory ----------";
    RoutingResponse temp_routing_response(*routing_response);
    if (routing_response->has_routing_request()) {
      return true;
    }
  } else if (routing_min_lenght > 3 * FLAGS_dual_box_exit_distance &&
             (routing_request.task_type() ==
                  UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_0 ||
              routing_request.task_type() ==
                  UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_1 ||
              routing_request.task_type() ==
                  LOADING_OPERATIONAREA_SAMEDIRECTION_3_0 ||
              routing_request.task_type() ==
                  LOADING_OPERATIONAREA_SAMEDIRECTION_3_1)) {
    AINFO << "---------- Use reverse trajectory ----------";
    RoutingResponse temp_routing_response(*routing_response);
    if (routing_response->has_routing_request()) {
      is_use_reverse_trajectory_ = true;
      return true;
    }
  }
  return false;
}

bool Routing::IsReverseType(const RoutingResponse& routing_response) {
  auto task_type = routing_response.routing_request().task_type();
  if ((!FLAGS_enable_default_shortest_path_output ||
       !is_need_shortest_route_) &&
      routing_response.is_reverse_type() &&
      (task_type != routing::TINY_ADJUSTMENT_FRONT &&
       task_type != routing::TINY_ADJUSTMENT_BACK &&
       task_type != routing::TINY_ADJUSTMENT_LEFT &&
       task_type != routing::TINY_ADJUSTMENT_RIGHT &&
       task_type != routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_0 &&
       task_type != routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_1 &&
       task_type != routing::LOADING_OPERATIONAREA_SAMEDIRECTION_3_0 &&
       task_type != routing::LOADING_OPERATIONAREA_SAMEDIRECTION_3_1 &&
       task_type != routing::LOADING_OPERATIONAREA_SAMEDIRECTION_2 &&
       task_type != routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_2)) {
    return true;
  }
  return false;
}

bool Routing::IsLaneChangeLengthValid(const RoutingResponse& routing_response,
                                      int huaman_shaped_level) {
  int road_size = routing_response.road_size();
  if (0 == road_size || huaman_shaped_level <= FLAGS_max_rerouting_times) {
    return true;
  }
  if (is_add_rerouting_lane_) {
    int passage_size = routing_response.road(0).passage_size();
    if (passage_size > 1) {
      if (!routing_math_ptr_->IsSatisfyLaneChange(routing_response.road(0),
                                                  FLAGS_lane_change_length)) {
        AINFO << "Rerouting check road(0) failed - road_size:" << road_size
              << ", passage_size:" << passage_size;
        return false;
      }
    }
  }
  if (FLAGS_enable_lane_change_length_determination &&
      is_lane_change_detector_) {
    int passage_size = routing_response.road(road_size - 1).passage_size();
    if (passage_size > 1) {
      if (!routing_math_ptr_->IsSatisfyLaneChange(
              routing_response.road(road_size - 1), FLAGS_min_lane_change_length)) {
        AINFO << "Last road check failed - road_size:" << road_size
              << ", road_index:" << road_size - 1
              << ", passage_size:" << passage_size;
        return false;
      }
    }
  }

  return true;
}

bool Routing::IsExcludeThisRoutingRequest(
    const RoutingRequest& fixed_routing_request,
    const RoutingResponse& routing_response,
    const double& temp_routing_min_lenght) {
  if (std::numeric_limits<double>::max() == temp_routing_min_lenght) {
    AINFO << "Exclude this routing request, because of length too long.";
    return true;
  }
  if (IsLaneChangeDistanceClose(fixed_routing_request, routing_response,
                                temp_routing_min_lenght)) {
    AINFO << "Exclude this routing request, because of lanechange distance.";
    return true;
  }
  if (!IsSpecialAreasOnlyuseUturn(routing_response)) {
    AINFO << "Exclude this routing request, because of not u turn.";
    return true;
  }
  if (IsReverseType(routing_response)) {
    AINFO << "Exclude this routing request, because of reverse type.";
    return true;
  }
  if (!IsLaneChangeLengthValid(
          routing_response,
          fixed_routing_request.rerouting_info().huaman_shaped().level())) {
    AINFO << "Exclude this routing request, because of lane change length too "
             "short.";
    return true;
  }
  return false;
}

bool Routing::CalculateCostAndCheckReverse(
    const double& routing_min_lenght, RoutingResponse& routing_response,
    std::vector<RoutingResponse>& routing_responses) {
  if (std::numeric_limits<double>::max() == routing_min_lenght ||
      routing_response.road_size() < 1) {
    return false;
  }
  bool is_is_reverse_type = false;
  auto adc_point = routing_response.routing_request().waypoint(0).pose();
  auto target_point =
      routing_response.routing_request()
          .waypoint(routing_response.routing_request().waypoint().size() - 1)
          .pose();
  int road_iter = 0;
  std::vector<common::PointENU> road_points;
  road_points.emplace_back(adc_point);
  const int total_roads = routing_response.road_size();
  for (const auto& road_passage : routing_response.road()) {
    road_iter++;
    bool is_last_road = (road_iter == total_roads);
    routing_math_ptr_->SearchSparsePointAndReserveTypeBySegments(
        road_passage, road_iter, is_last_road, road_points, is_is_reverse_type);
  }
  road_points.emplace_back(target_point);
  for (const auto& road_point : road_points) {
    routing_response.add_road_points()->CopyFrom(road_point);
  }
  if (FLAGS_enable_default_shortest_path_output) {
    routing_response.set_cost(routing_min_lenght);
  } else {
    routing_response.set_cost(is_is_reverse_type
                                  ? routing_min_lenght + FLAGS_reverse_type_cost
                                  : routing_min_lenght);
  }
  routing_response.set_is_reverse_type(is_is_reverse_type);
  if (is_add_responses_) {
    routing_responses.push_back(routing_response);
  }
  return true;
}

void Routing::PrioritizeResponsesByCost(
    const RoutingResponse& routing_response,
    std::vector<RoutingResponse>& routing_responses) {
  if (!routing_response.has_routing_request() ||
      routing_response.road_size() < 1) {
    return;
  }
  std::sort(routing_responses.begin(), routing_responses.end(),
            [](const RoutingResponse& a, const RoutingResponse& b) {
              return a.cost() < b.cost();
            });
  if (!FLAGS_enable_default_shortest_path_output) {
    auto targetCost = routing_response.cost();
    routing_responses.erase(
        std::remove_if(routing_responses.begin(), routing_responses.end(),
                       [targetCost](const RoutingResponse& response) {
                         return std::fabs(response.cost() - targetCost) < 1e-6;
                       }),
        routing_responses.end());
    if (!routing_responses.empty()) {
      routing_responses.insert(routing_responses.begin(), routing_response);
    }
  }
  if (routing_responses.size() > FLAGS_max_number_responses_output) {
    routing_responses.resize(FLAGS_max_number_responses_output);
  }
}

void Routing::AllNormalRoutingSearchShortRoute(
    double& normal_min_routing_length, double& routing_min_lenght,
    RoutingResponse* const routing_response,
    std::vector<RoutingResponse>& routing_responses) {
  double min_routing_length = std::numeric_limits<double>::max();
  int last_straight_level = 0;
  for (size_t i = 0; i < fixed_routing_request_.size(); ++i) {
    int straight_level = 0;
    auto fixed_routing_request = fixed_routing_request_.at(i);
    AINFO << "-- " << i
          << " --> is_backward:" << fixed_routing_request.is_backward()
          << ",is_reverse_trajectory:"
          << fixed_routing_request.is_reverse_trajectory()
          << ",is_two_routing:" << fixed_routing_request.rerouting_info().is_rerouting();

    // Check if this is a four-point rerouting request
    bool is_four_point_rerouting =
        (fixed_routing_request.waypoint().size() == kFourwaypointSize &&
         fixed_routing_request.rerouting_info().is_merge_routing());

    RoutingResponse temp_routing_response;
    double temp_routing_min_lenght = std::numeric_limits<double>::max();
    double temp_routing_length = std::numeric_limits<double>::max();
    double first_segment_length = std::numeric_limits<double>::max();

    if (is_four_point_rerouting) {
      AINFO << "-- Processing four-point two-segment rerouting request";
      temp_routing_length = CalculateTwoSegmentRoutingLength(
          fixed_routing_request, &temp_routing_response,
          &temp_routing_min_lenght, &first_segment_length);
    } else {
      std::vector<RoutingRequest> temp_fixed_requests;
      temp_fixed_requests.push_back(fixed_routing_request);
      temp_routing_length = NormalRoutingSearchShortRoute(
          temp_fixed_requests, &temp_routing_response, &temp_routing_min_lenght,
          is_need_lanechange_first_);
      first_segment_length = temp_routing_length;
    }

    // For four-point rerouting, use first_segment_length for cost calculation
    // and exclusion
    double length_for_cost_check = is_four_point_rerouting
                                       ? first_segment_length
                                       : temp_routing_min_lenght;

    CalculateCostAndCheckReverse(length_for_cost_check, temp_routing_response,
                                 routing_responses);
    if (IsExcludeThisRoutingRequest(fixed_routing_request,
                                    temp_routing_response,
                                    length_for_cost_check)) {
      continue;
    }

    const int waypoint_size = fixed_routing_request.waypoint().size();
    const int last_waypoint_index = waypoint_size - 1;
    auto target_lane_id =
        fixed_routing_request.waypoint(last_waypoint_index).id();
    auto target_lane = hdmap_->GetLaneById(hdmap::MakeMapId(target_lane_id));
    auto first_lane_id = fixed_routing_request.waypoint(0).id();
    auto first_lane = hdmap_->GetLaneById(hdmap::MakeMapId(first_lane_id));

    if (target_lane == nullptr || first_lane == nullptr) {
      AERROR << "Failed to get lane: target_lane=" << target_lane_id
             << ", first_lane=" << first_lane_id;
      continue;
    }

    const bool target_is_no_turn =
        (target_lane->lane().turn() == century::hdmap::Lane::NO_TURN);
    const bool first_is_no_turn =
        (first_lane->lane().turn() == century::hdmap::Lane::NO_TURN);
    const bool no_merge_lanes =
        !target_lane->lane().is_merge() && !first_lane->lane().is_merge();
    bool target_lane_near_fixed_point = routing_math_ptr_->IsTargetLaneStraight(
        target_lane_id, "STRAIGHT_CHECK_GENERAL", 2 * FLAGS_routing_lane_search_radius);
    if (target_lane_near_fixed_point) {
      AINFO << "Target lane " << target_lane_id
            << " confirmed as straight via STRAIGHT_CHECK_GENERAL fixed points";
    }
    // straight_level: 2=all straight, 1=partial straight, 0=all turn
    if ((target_is_no_turn && first_is_no_turn && no_merge_lanes) ||
        target_lane_near_fixed_point) {
      straight_level = 2;
    } else if (target_is_no_turn || first_is_no_turn) {
      straight_level = 1;
    }
    if (fixed_routing_request.rerouting_info().is_skip_rerouting()) {
      temp_routing_length -= FLAGS_rerouting_length_diff;
    }
    // route selection by straight_level priority:
    // higher level preferred, tolerance scales with level difference
    bool should_use = false;
    int level_diff = std::abs(straight_level - last_straight_level);
    double tolerance =
        (level_diff > 1) ? FLAGS_max_route_distance_diff : FLAGS_min_route_distance_diff;
    if (0 == i) {
      should_use = true;
    } else if (straight_level == last_straight_level) {
      should_use = (temp_routing_length < min_routing_length);
    } else if (straight_level > last_straight_level) {
      should_use = (temp_routing_length - min_routing_length < tolerance);
    } else {
      should_use = (min_routing_length - temp_routing_length > tolerance);
    }
    if (should_use) {
      last_straight_level = straight_level;
      normal_min_routing_length = temp_routing_length;
      min_routing_length = temp_routing_length;
      routing_min_lenght = temp_routing_length;
      *routing_response = temp_routing_response;
      AINFO << "---- used routing request, straight_level=" << straight_level
            << " ----";
    }
    AINFO << "END";
  }
  PrioritizeResponsesByCost(*routing_response, routing_responses);
}

double Routing::CalculateTwoSegmentRoutingLength(
    const RoutingRequest& four_point_request,
    RoutingResponse* const routing_response, double* const routing_min_lenght,
    double* const first_segment_length) {
  int waypoint_count = four_point_request.waypoint().size();

  // Only support 4 waypoints
  if (waypoint_count != kFourwaypointSize) {
    AERROR << "CalculateTwoSegmentRoutingLength: Expected 4 waypoints, got "
           << waypoint_count;
    return std::numeric_limits<double>::max();
  }

  // Create first segment request (waypoint 0 to waypoint 1)
  RoutingRequest first_segment_request;
  first_segment_request.CopyFrom(four_point_request);
  first_segment_request.clear_waypoint();
  first_segment_request.add_waypoint()->CopyFrom(
      four_point_request.waypoint(0));
  first_segment_request.add_waypoint()->CopyFrom(
      four_point_request.waypoint(1));

  // Create second segment request (waypoint 2 to waypoint 3)
  // Note: Skips waypoint 1 to waypoint 2, directly routes from waypoint 2 to
  // waypoint 3
  RoutingRequest second_segment_request;
  second_segment_request.CopyFrom(four_point_request);
  second_segment_request.clear_waypoint();
  second_segment_request.add_waypoint()->CopyFrom(
      four_point_request.waypoint(2));
  second_segment_request.add_waypoint()->CopyFrom(
      four_point_request.waypoint(3));

  // Search routes for both segments
  std::vector<RoutingRequest> first_segment_requests;
  first_segment_requests.push_back(first_segment_request);
  RoutingResponse first_segment_response;
  double first_segment_min_length = std::numeric_limits<double>::max();
  double first_seg_length = NormalRoutingSearchShortRoute(
      first_segment_requests, &first_segment_response,
      &first_segment_min_length, is_need_lanechange_first_);

  std::vector<RoutingRequest> second_segment_requests;
  second_segment_requests.push_back(second_segment_request);
  RoutingResponse second_segment_response;
  double second_segment_min_length = std::numeric_limits<double>::max();
  double second_seg_length = NormalRoutingSearchShortRoute(
      second_segment_requests, &second_segment_response,
      &second_segment_min_length, is_need_lanechange_first_);

  // Calculate total length
  double total_length = std::numeric_limits<double>::max();
  if (first_seg_length < std::numeric_limits<double>::max() &&
      second_seg_length < std::numeric_limits<double>::max()) {
    total_length = first_seg_length + second_seg_length + FLAGS_rerouting_length_diff;

    routing_response->CopyFrom(first_segment_response);
    routing_response->mutable_measurement()->set_distance(total_length);

    AINFO << "Four-point two-segment routing: "
          << "segment 0->1 length = " << first_seg_length
          << ", segment 2->3 length = " << second_seg_length
          << ", total length = " << total_length;
  } else {
    AERROR << "Failed to calculate four-point routing: "
           << "segment 0->1 = " << first_seg_length
           << ", segment 2->3 = " << second_seg_length;
  }

  // Output first segment length if requested
  if (first_segment_length != nullptr) {
    *first_segment_length = first_seg_length;
  }

  *routing_min_lenght = total_length;
  return total_length;
}

void Routing::FixedRoutingRequestWaypointTurnAround() {
  for (auto& fixed_routing_request : fixed_routing_request_) {
    fixed_routing_request.mutable_waypoint();
    RoutingRequest fixed_request(fixed_routing_request);
    fixed_routing_request.clear_waypoint();
    for (int i = fixed_request.waypoint().size() - 1; i >= 0; i--) {
      fixed_routing_request.add_waypoint()->CopyFrom(fixed_request.waypoint(i));
    }
  }
}

bool Routing::IsBackCar(const LaneWaypoint& lane_waypoint) {
  auto lane_id = lane_waypoint.id();
  hdmap::Id hd_lane_id = hdmap::MakeMapId(lane_id);
  auto hdmap_lane = hdmap_->GetLaneById(hd_lane_id);
  double lane_heading = hdmap_lane->Heading(lane_waypoint.s());
  double diff_heading =
      common::math::NormalizeAngle(veh_heading_ - lane_heading);
  AINFO << "diff_heading:" << diff_heading << ",lane_heading:" << lane_heading;
  if (std::fabs(diff_heading) > M_PI_2) {
    return true;
  } else {
    return false;
  }
}

void Routing::SpecialTypeForwardEndPoint(
    RoutingResponse* const routing_response) {
  if (routing_response->has_routing_request()) {
    RoutingRequest fixed_request(routing_response->routing_request());
    if ((fixed_request.task_type() == UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_0 ||
         fixed_request.task_type() == UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_1 ||
         fixed_request.task_type() == LOADING_OPERATIONAREA_SAMEDIRECTION_3_0 ||
         fixed_request.task_type() == LOADING_OPERATIONAREA_SAMEDIRECTION_3_1 ||
         fixed_request.task_type() ==
             routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_1) &&
        !IsBeingRerouing(fixed_request) && fixed_request.waypoint_size() > 1) {
      AINFO << "Start executing a special type of endpoint and push it forward "
               "a certain distance.";
      double normal_min_routing_length = std::numeric_limits<double>::max();
      auto front_waypoint = fixed_request.mutable_waypoint(0);
      auto back_waypoint =
          fixed_request.mutable_waypoint(fixed_request.waypoint().size() - 1);
      LaneWaypoint end_back_lane_waypoint(*back_waypoint);
      LaneWaypoint end_front_lane_waypoint(*front_waypoint);
      if (is_use_reverse_trajectory_) {
        auto is_back_car = IsBackCar(*back_waypoint);
        if ((is_front_box_ && is_back_car) ||
            (!is_front_box_ && !is_back_car)) {
          AINFO << "Advance (use reverse traj)--> is_front_box_:"
                << is_front_box_ << ",is_back_car:" << is_back_car;
          routing_math_ptr_->GetWaypointAlongSuccessorIdByDistance(
              end_front_lane_waypoint, *front_waypoint, FLAGS_vehicle_length / 4);
        } else {
          AINFO << "Beyond (use reverse traj)--> is_front_box_:"
                << is_front_box_ << ",is_back_car:" << is_back_car;
          routing_math_ptr_->GetWaypointAlongPredecessorIdByDistance(
              end_front_lane_waypoint, *front_waypoint, FLAGS_vehicle_length / 4);
        }
      } else {
        routing_math_ptr_->GetWaypointAlongSuccessorIdByDistance(
            end_back_lane_waypoint, *back_waypoint, FLAGS_vehicle_length / 4);
      }

      RoutingResponse temp_routing_response;
      if (navigator_ptr_->SearchRoute(fixed_request, &temp_routing_response,
                                      is_need_lanechange_first_)) {
        const double routing_length =
            temp_routing_response.measurement().distance();
        if (routing_length < normal_min_routing_length) {
          routing_response->CopyFrom(temp_routing_response);
          normal_min_routing_length = routing_length;
        }
      }
      FillParkingID(routing_response);
    }
  }
}

bool Routing::IsBeingRerouing(const RoutingRequest& fixed_request) {
  if (fixed_request.rerouting_info().is_rerouting() ||
      fixed_request.rerouting_info().huaman_shaped().is_part_rerouting() ||
      fixed_request.rerouting_info().dead_road().is_rerouting()) {
    return true;
  }
  return false;
}

bool Routing::DoubleBoxOperationForPositionOfFirstBox(
    RoutingResponse* const routing_response) {
  if (routing_response->has_routing_request()) {
    RoutingRequest fixed_request(routing_response->routing_request());
    if ((fixed_request.task_type() == UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_0 ||
         fixed_request.task_type() == UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_1 ||
         fixed_request.task_type() == LOADING_OPERATIONAREA_SAMEDIRECTION_3_0 ||
         fixed_request.task_type() == LOADING_OPERATIONAREA_SAMEDIRECTION_3_1 ||
         fixed_request.task_type() == UNLOAD_OPERATIONAREA_SAMEDIRECTION_1) &&
        !IsBeingRerouing(fixed_request)) {
      if (!is_use_reverse_trajectory_ &&
          fixed_request.task_type() != UNLOAD_OPERATIONAREA_SAMEDIRECTION_1) {
        if ((is_front_box_ && fixed_request.is_backward()) ||
            (!is_front_box_ && !fixed_request.is_backward())) {
          AINFO << "Advance--> is_front_box_:" << is_front_box_
                << ",is_backward:" << fixed_request.is_backward();
          auto mut_routing_request =
              routing_response->mutable_routing_request();
          mut_routing_request->set_is_early_stop(true);
        } else {
          AINFO << "Beyond--> is_front_box_:" << is_front_box_
                << ",is_backward:" << fixed_request.is_backward();
          SpecialTypeForwardEndPoint(routing_response);
          auto mut_routing_request =
              routing_response->mutable_routing_request();
          mut_routing_request->set_is_early_stop(false);
        }
      } else {
        AINFO << "Else-->";
        SpecialTypeForwardEndPoint(routing_response);
      }
    }
    RoutingRequest routing_request(routing_response->routing_request());
    if (routing_request.task_type() == LOADING_OPERATIONAREA_SAMEDIRECTION_1) {
      if (!IsBeingRerouing(routing_request) && routing_request.is_backward()) {
        is_front_box_ = false;
      } else {
        is_front_box_ = true;
      }
    } else if (routing_request.task_type() ==
               UNLOAD_OPERATIONAREA_SAMEDIRECTION_1) {
      if (!IsBeingRerouing(routing_request) && routing_request.is_backward()) {
        is_front_box_ = true;
      } else {
        is_front_box_ = false;
      }
    }
    if (routing_request.task_type() == LOADING_OPERATIONAREA_SAMEDIRECTION_1 ||
        routing_request.task_type() == UNLOAD_OPERATIONAREA_SAMEDIRECTION_1) {
      if (routing_request.is_backward()) {
        is_backward_ = true;
      } else {
        is_backward_ = false;
      }
    }
    AINFO << "is_front_box_:" << is_front_box_
          << ",is_backward_:" << is_backward_;
  }
  return true;
}

void Routing::SetErrorCode(const common::ErrorCode& error_code_id,
                           const std::string& error_string,
                           common::StatusPb* const error_code) {
  error_code->set_error_code(error_code_id);
  error_code->set_msg(error_string);
  if (common::ErrorCode::OK == error_code_id) {
    ADEBUG << error_string.c_str();
  } else {
    AERROR << error_string.c_str();
  }
}

double Routing::NormalRoutingSearchShortRoute(
    const std::vector<RoutingRequest>& fixed_requests,
    RoutingResponse* const routing_response, double* const routing_min_lenght,
    bool is_need_lanechange_first) {
  double normal_min_routing_length = std::numeric_limits<double>::max();
  for (const auto& fixed_request : fixed_requests) {
    RoutingRequest request_with_blacklist(fixed_request);
    RoutingResponse temp_routing_response;
    AddBlacklistedLanes(&request_with_blacklist);
    if (navigator_ptr_->SearchRoute(request_with_blacklist,
                                    &temp_routing_response,
                                    is_need_lanechange_first)) {
      const double routing_length =
          temp_routing_response.measurement().distance();
      if (routing_length < normal_min_routing_length) {
        routing_response->CopyFrom(temp_routing_response);
        normal_min_routing_length = routing_length;
      }
    }
    FillParkingID(routing_response);
  }
  *routing_min_lenght = normal_min_routing_length;
  AINFO << "Normal Routing min short lenth " << normal_min_routing_length;
  if (normal_min_routing_length < std::numeric_limits<double>::max()) {
    monitor_logger_buffer_.INFO("Normal Routing success!");
    // AINFO << "Normal Routing success ";
  }
  return normal_min_routing_length;
}

void Routing::LogInfo() {
  AINFO << "is_heading_reverse_:" << is_heading_reverse_
        << ",is_need_lanechange_first_:" << is_need_lanechange_first_
        << ",is_human_shaped_:" << is_human_shaped_
        << ",is_add_responses_:" << is_add_responses_
        << ",is_special_area_no_rerouting_:" << is_special_area_no_rerouting_
        << ",is_need_shortest_route_:" << is_need_shortest_route_
        << ",next_point_lane_waypoint_.size():"
        << next_point_lane_waypoint_.size()
        << ",in_special_areas_:" << in_special_areas_;
}

void Routing::InitParameters(
    const std::shared_ptr<RoutingRequest>& routing_request) {
  in_special_areas_ = Junction::IN_ROAD;
  is_human_shaped_ = false;
  is_special_area_no_rerouting_ = false;
  is_add_rerouting_lane_ = false;
  is_lane_change_detector_ = true;
  additional_lane_waypoint_map_.clear();
  next_point_lane_waypoint_.clear();
  is_need_shortest_route_ =
      FLAGS_enable_default_shortest_path_output ? true : false;
  RunLoopRunning(routing_request);
  is_heading_reverse_ = IsRoutingFirstPointHeadingReverse(routing_request);
  ProjectLastWaypointForRelocation(routing_request.get());
  FilterWaypointsForReachStacker(routing_request.get());
}

void Routing::FilterWaypointsForReachStacker(
    RoutingRequest* const routing_request) {
  if (routing_request->dreamview_task_type() != routing::SEARCH_REACH_STACKER) {
    return;
  }
  if (routing_request->waypoint_size() != kThreewaypointSize) {
    return;
  }

  auto first_waypoint = routing_request->waypoint(0);
  auto second_waypoint = routing_request->waypoint(1);
  auto third_waypoint = routing_request->waypoint(2);

  // Calculate projection for the 2nd waypoint
  auto second_projection = second_waypoint;
  if (!routing_math_ptr_->GetCraneProjectionLaneInfor(second_projection)) {
    AERROR << "Failed to get projection for the 2nd waypoint";
    return;
  }

  auto task_type = routing_request->task_type();
  bool is_operation_area_type =
      routing::RAILWAY_WAITINGAREA_DYNAMIC == task_type ||
      routing::RAILWAY_OPERATIONAREA_DYNAMIC == task_type ||
      routing::LOADING_OPERATIONAREA_SAMEDIRECTION_1 == task_type ||
      routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_0 == task_type ||
      routing::LOADING_OPERATIONAREA_SAMEDIRECTION_3_1 == task_type ||
      routing::LOADING_OPERATIONAREA_SAMEDIRECTION_3_0 == task_type ||
      routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_1 == task_type ||
      routing::UNLOAD_OPERATIONAREA_SAMEDIRECTION_1 == task_type;

  if (is_operation_area_type && FLAGS_enable_reach_stacker_end_projection) {
    // Calculate 3rd point's projection following
    // FillEndPointLaneInfoForProjection
    auto third_projection = third_waypoint;
    bool projection_success = false;
    if (routing::SITE_COORDINATE == routing_request->coordinate_type() ||
        routing::STACKER_COORDINATE == routing_request->coordinate_type()) {
      projection_success =
          routing_math_ptr_->GetCraneProjectionLaneInfor(third_projection);
    } else if (routing::WHEEL_CRANE_COORDINATE ==
               routing_request->coordinate_type()) {
      projection_success =
          routing_math_ptr_->GetWheelCraneProjectionLaneInfor(third_projection);
    }
    if (!projection_success) {
      AERROR << "Failed to get projection for the 3rd waypoint";
      return;
    }
    // Compare 3rd projection with 2nd projection
    double distance = std::sqrt(
        std::pow(third_projection.pose().x() - second_projection.pose().x(),
                 2) +
        std::pow(third_projection.pose().y() - second_projection.pose().y(),
                 2));
    routing_request->clear_waypoint();
    if (distance > FLAGS_reach_stacker_projection_distance_threshold) {
      AINFO << "3rd projection far from 2nd projection (" << distance
            << "m), keep 1st and 2nd waypoint";
      routing_request->add_waypoint()->CopyFrom(first_waypoint);
      routing_request->add_waypoint()->CopyFrom(second_waypoint);
    } else {
      AINFO << "3rd projection close to 2nd projection (" << distance
            << "m), keep 1st and 3rd waypoint";
      routing_request->add_waypoint()->CopyFrom(first_waypoint);
      routing_request->add_waypoint()->CopyFrom(third_waypoint);
    }
  } else {
    // Compare 3rd point directly with 2nd point's projection
    double distance = std::sqrt(
        std::pow(third_waypoint.pose().x() - second_projection.pose().x(), 2) +
        std::pow(third_waypoint.pose().y() - second_projection.pose().y(), 2));
    routing_request->clear_waypoint();
    if (distance > FLAGS_reach_stacker_projection_distance_threshold) {
      AINFO << "3rd point far from 2nd projection (" << distance
            << "m), keep 1st and 2nd projection";
      routing_request->add_waypoint()->CopyFrom(first_waypoint);
      routing_request->add_waypoint()->CopyFrom(second_projection);
    } else {
      AINFO << "3rd point close to 2nd projection (" << distance
            << "m), keep 1st and 3rd waypoint";
      routing_request->add_waypoint()->CopyFrom(first_waypoint);
      routing_request->add_waypoint()->CopyFrom(third_waypoint);
    }
  }
}

void Routing::ProjectLastWaypointForRelocation(
    RoutingRequest* const routing_request) {
  if (!routing_request || routing_request->waypoint_size() == 0) {
    return;
  }

  if (routing_request->task_type() != routing::TEMPORARY_VEH_RELOCATION &&
      routing_request->task_type() != routing::SEARCH_REACH_STACKER) {
    return;
  }

  // Get the last waypoint
  const int last_waypoint_index = routing_request->waypoint_size() - 1;
  auto last_waypoint = routing_request->mutable_waypoint(last_waypoint_index);
  const auto& pose = last_waypoint->pose();
  common::math::Vec2d point(pose.x(), pose.y());

  // Incremental lane search: start with FLAGS_min_adc_lane_range (1m), increase by
  // 1m, max 30m
  double search_radius = FLAGS_min_adc_lane_range;
  double max_distance = FLAGS_max_lane_search_distance;
  std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;

  while (search_radius <= max_distance) {
    hdmap_->GetLanes(pose, search_radius, &lanes);
    if (!lanes.empty()) {
      AINFO << "Found " << lanes.size()
            << " lanes for last waypoint with radius " << search_radius << "m";
      break;
    }
    search_radius += FLAGS_lane_search_increment;
  }

  if (lanes.empty()) {
    AERROR << "Failed to find any lane within max distance " << max_distance
           << "m for last waypoint at position: (" << pose.x() << ", "
           << pose.y() << ")";
    return;
  }

  for (const auto& lane : lanes) {
    double s = 0.0;
    double l = 0.0;
    if (!lane->GetProjection(point, &s, &l)) {
      continue;
    }

    if (s < 0.0 || s > lane->total_length()) {
      continue;
    }

    const auto& projected_point = lane->GetSmoothPoint(s);
    const double original_x = pose.x();
    const double original_y = pose.y();

    last_waypoint->mutable_pose()->set_x(projected_point.x());
    last_waypoint->mutable_pose()->set_y(projected_point.y());
    last_waypoint->set_s(s);
    last_waypoint->set_id(lane->id().id());

    AINFO << "Waypoint projected - "
          << "BEFORE: (" << original_x << ", " << original_y << ") "
          << "AFTER: (" << projected_point.x() << ", " << projected_point.y()
          << ")";
    return;
  }

  AWARN << "No suitable lane found for projection";
}

bool Routing::Process(const std::shared_ptr<RoutingRequest>& routing_request,
                      RoutingResponse* const routing_response,
                      std::vector<RoutingResponse>& routing_responses) {
  CHECK_NOTNULL(routing_response);
  InitParameters(routing_request);
  std::vector<RoutingRequest> fixed_requests;
  FillLaneInfoIfMissing(*routing_request, &fixed_requests);
  is_need_lanechange_first_ = IsNeedLanechangeFirst(&fixed_requests);
  auto need_to_research =
      FillTwoWayTaskByOverlapPoint(*routing_request, &fixed_requests);
  double normal_min_routing_length = std::numeric_limits<double>::max();
  double routing_min_lenght = std::numeric_limits<double>::max();
  if (need_to_research) {
    AllNormalRoutingSearchShortRoute(normal_min_routing_length,
                                     routing_min_lenght, routing_response,
                                     routing_responses);
  } else {
    if (!fixed_requests.empty()) {
      auto request = fixed_requests.at(0);
      AINFO << "--> is_backward:" << request.is_backward()
            << ",is_reverse_trajectory:" << request.is_reverse_trajectory()
            << ",is_two_routing:" << request.rerouting_info().is_rerouting()
            << ",is_merge_routing():" << request.rerouting_info().is_merge_routing();
    }
    normal_min_routing_length = NormalRoutingSearchShortRoute(
        fixed_requests, routing_response, &routing_min_lenght,
        is_need_lanechange_first_);
  }

  if (IsUseReverseTrajectory(routing_min_lenght, *routing_request,
                             routing_response)) {
    normal_min_routing_length = std::numeric_limits<double>::max();
    routing_min_lenght = std::numeric_limits<double>::max();
    FixedRoutingRequestWaypointTurnAround();
    AllNormalRoutingSearchShortRoute(normal_min_routing_length,
                                     routing_min_lenght, routing_response,
                                     routing_responses);
    if (routing_response->has_routing_request()) {
      routing_response->mutable_routing_request()->set_is_reverse_trajectory(
          true);
    }
  }
  DoubleBoxOperationForPositionOfFirstBox(routing_response);
  LogInfo();
  if (routing_min_lenght != std::numeric_limits<double>::max()) {
    if (routing_min_lenght == normal_min_routing_length) {
      if (routing_request->multi_routing_type() == routing::NOPROCESS) {
        AINFO << "Use Normal Routing";
      } else if (routing_request->multi_routing_type() ==
                 routing::PROCESSINGSTART) {
        AINFO << "Select multiple routing routes and send temporary parking";
      }
      return true;
    }
  } else {
    AERROR << "All Routing Filed ";
    RoutingResponse status_routing_response;
    SetErrorCode(ErrorCode::ROUTING_ERROR_REQUEST,
                 "Error encountered when reading request point! | The vehicle "
                 "or routing point is in the intersection",
                 status_routing_response.mutable_status());
    RoutingRequest fixed_request(*routing_request);
    auto temp_routing_request =
        status_routing_response.mutable_routing_request();
    fixed_request.set_task_type(routing::TINY_ADJUSTMENT_STOP);
    temp_routing_request->CopyFrom(fixed_request);
    routing_response->CopyFrom(status_routing_response);
  }

  AERROR << "Failed to search route with navigator.";
  monitor_logger_buffer_.WARN("Routing failed! " +
                              routing_response->status().msg());
  return false;
}
}  // namespace routing
}  // namespace century
