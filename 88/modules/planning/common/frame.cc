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
 * @file frame.cc
 **/
#include "modules/planning/common/frame.h"

#include <algorithm>
#include <limits>

#include "absl/strings/str_cat.h"

#include "modules/routing/proto/routing.pb.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/feature_output.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/reference_line/reference_line_provider.h"

namespace century {
namespace planning {

using century::common::ErrorCode;
using century::common::SLPoint;
using century::common::Status;
using century::common::math::Box2d;
using century::common::math::Polygon2d;
using century::common::math::Vec2d;
using century::cyber::Clock;
using century::cyber::common::GlobalData;
using century::hdmap::LaneBoundaryType;
using century::prediction::PredictionObstacles;

namespace {
// Super traffic light id, manual remote sending from tbox
constexpr const char *kSuperTrafficLightId = "1";
constexpr size_t kMinPolygonPointSize = 3;
constexpr int kMaxTotalCounter = 5;
constexpr int kMaxMixedScenarioCounter = 3;
constexpr double kSuperTrafficLightConfidence = 2.0;
constexpr double kEStopSpeed = 0.15;
constexpr double kWidthBuffer = 0.5;
constexpr int kInvalidHuamanShapedLevel = 999;
}  // namespace

DrivingAction Frame::pad_msg_driving_action_ = DrivingAction::NONE;

FrameHistory::FrameHistory()
    : IndexedQueue<uint32_t, Frame>(FLAGS_max_frame_history_num) {}

Frame::Frame(uint32_t sequence_num)
    : sequence_num_(sequence_num),
      monitor_logger_buffer_(common::monitor::MonitorMessageItem::PLANNING) {}

Frame::Frame(uint32_t sequence_num, const LocalView &local_view,
             const common::TrajectoryPoint &planning_start_point,
             const common::VehicleState &vehicle_state,
             ReferenceLineProvider *reference_line_provider)
    : sequence_num_(sequence_num),
      local_view_(local_view),
      planning_start_point_(planning_start_point),
      vehicle_state_(vehicle_state),
      reference_line_provider_(reference_line_provider),
      monitor_logger_buffer_(common::monitor::MonitorMessageItem::PLANNING) {
  static_area_polygon_.clear();
  costmap_polygons_.clear();
  if (FLAGS_enable_use_costmap) {
    if (local_view_.prediction_obstacles != nullptr) {
      bool polygon_convex_err = false;
      for (const auto &area :
           local_view_.prediction_obstacles->perception_static_areas()) {
        std::vector<century::common::Point3D> points;
        for (const auto &p : area.polygon_point()) {
          points.emplace_back(p);
        }
        if (points.size() < kMinPolygonPointSize) {
          AERROR << "error !points.size() < 3";
          continue;
        }
        std::vector<Vec2d> polygon_points;
        for (const auto &pt : points) {
          polygon_points.emplace_back(pt.x(), pt.y());
        }
        Polygon2d perception_polygon;
        if (!Polygon2d::ComputeConvexHull(polygon_points,
                                          &perception_polygon)) {
          polygon_convex_err = true;
          continue;
        }
        static_area_polygon_.emplace_back(area.static_area_type(), points);
        costmap_polygons_.emplace_back(perception_polygon);
      }
      if (polygon_convex_err) {
        AERROR << "[costmap] perception_polygon ComputeConvexHull error";
      }
    }
  }
}

Frame::Frame(uint32_t sequence_num, const LocalView &local_view,
             const common::TrajectoryPoint &planning_start_point,
             const common::VehicleState &vehicle_state)
    : Frame(sequence_num, local_view, planning_start_point, vehicle_state,
            nullptr) {}

const common::TrajectoryPoint &Frame::PlanningStartPoint() const {
  return planning_start_point_;
}

const common::VehicleState &Frame::vehicle_state() const {
  return vehicle_state_;
}

bool Frame::CheckReroutingInput() {
  if (FLAGS_use_navigation_mode) {
    AERROR << "Rerouting not supported in navigation mode";
    return false;
  }
  if (local_view_.routing == nullptr) {
    AERROR << "No previous routing available";
    return false;
  }
  if (!hdmap_) {
    AERROR << "Invalid HD Map.";
    return false;
  }
  return true;
}

void Frame::FillFirstPoint(routing::RoutingRequest &request) {
  auto temp_request = local_view_.routing->routing_request();
  auto *start_point = request.add_waypoint();
  auto point = common::util::PointFactory::ToPointENU(vehicle_state_);
  start_point->mutable_pose()->CopyFrom(point);
  const double original_heading =
      local_view_.localization_estimate->pose().heading();
  start_point->set_heading(original_heading);
}

bool Frame::FillEndPoint(routing::RoutingRequest &request,
                         const ReroutingType &type, double move_distance) {
  const auto &orig_request = local_view_.routing->routing_request();
  const auto &rerouting_info = request.rerouting_info();

  switch (type) {
    case ReroutingType::REVERSE: {
      if (rerouting_info.is_rerouting() &&
          !rerouting_info.reverse_rerouting().is_rerouting()) {
        request.add_waypoint()->CopyFrom(
            rerouting_info.last_waypoint());
      } else {
        request.add_waypoint()->CopyFrom(
            orig_request.waypoint(orig_request.waypoint_size() - 1));
      }
      if (!FillBackwardPoint(request, move_distance)) {
        return false;
      }
      break;
    }
    case ReroutingType::NO_SPECIAL:
    case ReroutingType::HUMAN_SHAPED: {
      request.add_waypoint()->CopyFrom(
          orig_request.rerouting_info().huaman_shaped().next_waypoint());
      break;
    }
    case ReroutingType::SECOND_REROUTING: {
      request.add_waypoint()->CopyFrom(
          orig_request.rerouting_info().last_waypoint());
      break;
    }
    case ReroutingType::D7_BLOCK: {
      if (rerouting_info.is_rerouting() ||
          rerouting_info.dead_road().is_rerouting()) {
        request.add_waypoint()->CopyFrom(
            orig_request.rerouting_info().last_waypoint());
      } else {
        request.add_waypoint()->CopyFrom(
            orig_request.waypoint(orig_request.waypoint_size() - 1));
      }
      break;
    }
    case ReroutingType::DEAD_END_ROAD: {
      auto *dead_road = request.mutable_rerouting_info()->mutable_dead_road();
      if (dead_road->level() <= dead_road->waypoint_size() - 1) {
        auto iter = dead_road->level();
        request.add_waypoint()->CopyFrom(
            dead_road->waypoint(static_cast<int>(iter)));
        dead_road->set_level(iter + 1);
      } else {
        dead_road->set_level(kInvalidHuamanShapedLevel);
        dead_road->clear_waypoint();
        dead_road->set_is_rerouting(false);
      }
      break;
    }
    case ReroutingType::LOOP_RUNING: {
      const auto &loop_run = rerouting_info.loop_run();
      for (int i = 2; i < loop_run.waypoint_size(); ++i) {
        request.add_waypoint()->CopyFrom(loop_run.waypoint(i));
      }
      request.add_waypoint()->CopyFrom(loop_run.waypoint(1));
      break;
    }
    case ReroutingType::PART_HUMAN_SHAPED: {
      request.add_waypoint()->CopyFrom(
          orig_request.waypoint(orig_request.waypoint_size() - 1));
      break;
    }
    default:
      break;
  }
  return true;
}

// Rerouting
bool Frame::Rerouting(PlanningContext *planning_context,
                      const ReroutingType type, double move_distance) {
  if (!CheckReroutingInput()) {
    return false;
  }
  auto request = local_view_.routing->routing_request();
  request.clear_header();
  request.clear_waypoint();
  request.clear_blacklisted_lane();

  // fill point
  FillFirstPoint(request);
  if (!FillEndPoint(request, type, move_distance)) {
    AERROR << "Failed to fill end point for rerouting";
    return false;
  }

  if (type == ReroutingType::NO_SPECIAL) {
    request.set_local_routing_type(routing::ROUTING_DEFAULT);
  } else if (type == ReroutingType::REVERSE) {
    request.mutable_rerouting_info()
        ->mutable_reverse_rerouting()
        ->set_is_rerouting(true);
    request.mutable_rerouting_info()->mutable_huaman_shaped()->set_level(
        kInvalidHuamanShapedLevel);
    request.set_local_routing_type(routing::ROUTING_PLANNING);
  } else if (type == ReroutingType::HUMAN_SHAPED) {
    int huaman_shaped_level =
        request.rerouting_info().huaman_shaped().level() + 1;
    request.mutable_rerouting_info()->mutable_huaman_shaped()->set_level(
        huaman_shaped_level);
    request.mutable_rerouting_info()
        ->mutable_reverse_rerouting()
        ->set_is_rerouting(false);
  } else if (type == ReroutingType::DEAD_END_ROAD) {
  } else if (type == ReroutingType::LOOP_RUNING) {
    request.set_local_routing_type(routing::ROUTING_LOOP);
    request.mutable_rerouting_info()->mutable_huaman_shaped()->set_level(
        kInvalidHuamanShapedLevel);
    request.set_is_loop_running(true);
    request.mutable_rerouting_info()
        ->mutable_reverse_rerouting()
        ->set_is_rerouting(false);
  } else if (type == ReroutingType::PART_HUMAN_SHAPED) {
    request.mutable_rerouting_info()
        ->mutable_huaman_shaped()
        ->set_part_routing_type(routing::PART_ROUTING_HUMAN_SHAPED);
    request.mutable_rerouting_info()
        ->mutable_reverse_rerouting()
        ->set_is_rerouting(false);
  } else if (type == ReroutingType::SECOND_REROUTING) {
    request.set_local_routing_type(routing::ROUTING_PLANNING);
    request.mutable_rerouting_info()->mutable_huaman_shaped()->set_level(
        kInvalidHuamanShapedLevel);
    request.mutable_rerouting_info()
        ->mutable_huaman_shaped()
        ->set_is_part_rerouting(false);
    request.mutable_rerouting_info()->set_is_rerouting(false);
    request.mutable_rerouting_info()
        ->mutable_reverse_rerouting()
        ->set_is_rerouting(false);
  } else if (type == ReroutingType::D7_BLOCK) {
    request.mutable_rerouting_info()->set_is_rerouting(false);
    request.mutable_rerouting_info()->mutable_huaman_shaped()->set_level(
        kInvalidHuamanShapedLevel);
    request.mutable_rerouting_info()
        ->mutable_huaman_shaped()
        ->set_is_part_rerouting(false);
    request.set_local_routing_type(routing::ROUTING_DEFAULT);
    request.mutable_rerouting_info()
        ->mutable_block_rerouting()
        ->set_rerouting_type(routing::RE_ROUTING_D7);
    request.mutable_rerouting_info()
        ->mutable_reverse_rerouting()
        ->set_is_rerouting(false);
  }

  if (request.waypoint_size() <= 1) {
    AERROR << "Failed to find future waypoints";
    return false;
  }
  auto *rerouting =
      planning_context->mutable_planning_status()->mutable_rerouting();
  rerouting->set_need_rerouting(true);
  *rerouting->mutable_routing_request() = request;
  monitor_logger_buffer_.INFO("Planning send Rerouting request");
  return true;
}

bool Frame::FillBackwardPoint(routing::RoutingRequest &request,
                              double move_distance) {
  // find best reference line info
  const ReferenceLineInfo *best_ref_info = nullptr;
  double min_cost = std::numeric_limits<double>::infinity();
  for (const auto &reference_line_info : reference_line_info_) {
    if (reference_line_info.IsDrivable() &&
        reference_line_info.Cost() < min_cost) {
      best_ref_info = &reference_line_info;
      min_cost = reference_line_info.Cost();
    }
  }
  if (nullptr == best_ref_info) {
    AERROR << "Failed to find best reference line info";
    return false;
  }

  // get vehicle position on the reference line
  auto vehicle_point = common::util::PointFactory::ToPointENU(vehicle_state_);
  common::SLPoint vehicle_sl;
  if (!best_ref_info->reference_line().XYToSL(
          {vehicle_point.x(), vehicle_point.y()}, &vehicle_sl)) {
    AERROR << "Failed to project vehicle position to reference line";
    return false;
  }
  // find the lane on reference line at vehicle's s position
  const auto &lane_segments =
      best_ref_info->reference_line().GetMapPath().lane_segments();
  hdmap::LaneInfoConstPtr current_lane = nullptr;
  double current_s = 0.0;
  double accumulated_s = 0.0;
  for (const auto &seg : lane_segments) {
    double seg_length = seg.Length();
    if (accumulated_s + seg_length >= vehicle_sl.s()) {
      current_lane = seg.lane;
      current_s = seg.start_s + (vehicle_sl.s() - accumulated_s);
      break;
    }
    accumulated_s += seg_length;
  }
  if (nullptr == current_lane) {
    AERROR << "Failed to find lane on reference line for vehicle position";
    return false;
  }

  const bool is_rerouting =
      request.rerouting_info().is_rerouting() &&
      request.rerouting_info().reverse_rerouting().is_rerouting();

  auto *backward_pt = request.mutable_rerouting_info()
                          ->mutable_reverse_rerouting()
                          ->mutable_backward_point();

  double remaining_s = move_distance;

  if (is_rerouting) {
    // search forward from current position (successor lanes)
    double available_s = current_lane->lane().length() - current_s;
    if (available_s >= remaining_s) {
      double target_s = current_s + remaining_s;
      auto target_point = current_lane->GetSmoothPoint(target_s);
      backward_pt->mutable_pose()->set_x(target_point.x());
      backward_pt->mutable_pose()->set_y(target_point.y());
      backward_pt->set_id(current_lane->id().id());
      backward_pt->set_s(target_s);
      return true;
    }

    remaining_s -= available_s;
    auto search_lane = current_lane;
    while (remaining_s > 0.0) {
      auto successor_ids = search_lane->lane().successor_id();
      if (successor_ids.empty()) {
        AERROR << "No successor lane found, remaining distance: "
               << remaining_s;
        return false;
      }
      hdmap::LaneInfoConstPtr selected_lane = nullptr;
      for (const auto &id : successor_ids) {
        auto candidate = hdmap_->GetLaneById(id);
        if (nullptr == candidate) {
          continue;
        }
        if (hdmap::Lane::NO_TURN == candidate->lane().turn()) {
          selected_lane = candidate;
          break;
        }
        if (nullptr == selected_lane) {
          selected_lane = candidate;
        }
      }
      if (nullptr == selected_lane) {
        AERROR << "Failed to find valid successor lane";
        return false;
      }

      double lane_length = selected_lane->lane().length();
      if (lane_length >= remaining_s) {
        auto target_point = selected_lane->GetSmoothPoint(remaining_s);
        backward_pt->mutable_pose()->set_x(target_point.x());
        backward_pt->mutable_pose()->set_y(target_point.y());
        backward_pt->set_id(selected_lane->id().id());
        backward_pt->set_s(remaining_s);
        return true;
      }
      remaining_s -= lane_length;
      search_lane = selected_lane;
    }
  } else {
    // search backward from current position (predecessor lanes)
    double available_s = current_s;
    if (available_s >= remaining_s) {
      double target_s = current_s - remaining_s;
      auto target_point = current_lane->GetSmoothPoint(target_s);
      backward_pt->mutable_pose()->set_x(target_point.x());
      backward_pt->mutable_pose()->set_y(target_point.y());
      backward_pt->set_id(current_lane->id().id());
      backward_pt->set_s(target_s);
      return true;
    }

    remaining_s -= available_s;
    auto search_lane = current_lane;
    while (remaining_s > 0.0) {
      auto predecessor_ids = search_lane->lane().predecessor_id();
      if (predecessor_ids.empty()) {
        AERROR << "No predecessor lane found, remaining distance: "
               << remaining_s;
        return false;
      }
      hdmap::LaneInfoConstPtr selected_lane = nullptr;
      for (const auto &id : predecessor_ids) {
        auto candidate = hdmap_->GetLaneById(id);
        if (nullptr == candidate) {
          continue;
        }
        if (hdmap::Lane::NO_TURN == candidate->lane().turn()) {
          selected_lane = candidate;
          break;
        }
        if (nullptr == selected_lane) {
          selected_lane = candidate;
        }
      }
      if (nullptr == selected_lane) {
        AERROR << "Failed to find valid predecessor lane";
        return false;
      }

      double lane_length = selected_lane->lane().length();
      if (lane_length >= remaining_s) {
        double target_s = lane_length - remaining_s;
        auto target_point = selected_lane->GetSmoothPoint(target_s);
        backward_pt->mutable_pose()->set_x(target_point.x());
        backward_pt->mutable_pose()->set_y(target_point.y());
        backward_pt->set_id(selected_lane->id().id());
        backward_pt->set_s(target_s);
        return true;
      }
      remaining_s -= lane_length;
      search_lane = selected_lane;
    }
  }
  return false;
}

const std::list<ReferenceLineInfo> &Frame::reference_line_info() const {
  return reference_line_info_;
}

std::list<ReferenceLineInfo> *Frame::mutable_reference_line_info() {
  return &reference_line_info_;
}

void Frame::UpdateReferenceLinePriority(
    const std::map<std::string, uint32_t> &id_to_priority) {
  for (const auto &pair : id_to_priority) {
    const auto id = pair.first;
    const auto priority = pair.second;
    auto ref_line_info_itr =
        std::find_if(reference_line_info_.begin(), reference_line_info_.end(),
                     [&id](const ReferenceLineInfo &ref_line_info) {
                       return ref_line_info.Lanes().Id() == id;
                     });
    if (ref_line_info_itr != reference_line_info_.end()) {
      ref_line_info_itr->SetPriority(priority);
    }
  }
}

bool Frame::CreateReferenceLineInfo(
    const std::list<ReferenceLine> &reference_lines,
    const std::list<hdmap::RouteSegments> &segments) {
  reference_line_info_.clear();
  auto ref_line_iter = reference_lines.begin();
  auto segments_iter = segments.begin();
  while (ref_line_iter != reference_lines.end()) {
    if (segments_iter->StopForDestination()) {
      is_near_destination_ = true;
    }
    reference_line_info_.emplace_back(vehicle_state_, planning_start_point_,
                                      *ref_line_iter, *segments_iter);
    ++ref_line_iter;
    ++segments_iter;
  }

  if (2 == reference_line_info_.size()) {
    common::math::Vec2d xy_point(vehicle_state_.x(), vehicle_state_.y());
    common::SLPoint first_sl;
    if (!reference_line_info_.front().reference_line().XYToSL(xy_point,
                                                              &first_sl)) {
      return false;
    }
    common::SLPoint second_sl;
    if (!reference_line_info_.back().reference_line().XYToSL(xy_point,
                                                             &second_sl)) {
      return false;
    }
    const double offset = first_sl.l() - second_sl.l();
    reference_line_info_.front().SetOffsetToOtherReferenceLine(offset);
    reference_line_info_.back().SetOffsetToOtherReferenceLine(-offset);
  }

  bool has_valid_reference_line = false;
  for (auto &ref_info : reference_line_info_) {
    if (!ref_info.Init(obstacles())) {
      AERROR << "Failed to init reference line";
    } else {
      has_valid_reference_line = true;
    }
  }

  // check mixed traffic
  if (1 == reference_line_info_.size()) {
    static int total_counter = 0;
    static int mixed_scenario_counter = 0;
    if (reference_line_info_.front().IsMixedTraffic()) {
      mixed_traffic_type_ = MixedTrafficType::STATIC_ROAD_MEETED;
      ++total_counter;
      if (reference_line_info_.front().HasRetrogradeObstacleOnBicycleLane() ||
          reference_line_info_.front().HasNonMotorizedVehicle()) {
        ++mixed_scenario_counter;
      }
      if (total_counter >= kMaxTotalCounter) {
        if (mixed_scenario_counter >= kMaxMixedScenarioCounter) {
          mixed_traffic_type_ = MixedTrafficType::DYNAMIC_OBSTACLE_MEETED;
        }
        total_counter = 0;
        mixed_scenario_counter = 0;
      }
    } else {
      mixed_traffic_type_ = MixedTrafficType::UNKNOWN;
      total_counter = 0;
      mixed_scenario_counter = 0;
    }
  }

  return has_valid_reference_line;
}

/**
 * @brief: create static virtual object with lane width,
 *         mainly used for virtual stop wall
 */
const Obstacle *Frame::CreateStopObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const double obstacle_s) {
  if (reference_line_info == nullptr) {
    AERROR << "reference_line_info nullptr";
    return nullptr;
  }

  const auto &reference_line = reference_line_info->reference_line();
  const double box_center_s = obstacle_s + FLAGS_virtual_stop_wall_length * 0.5;
  auto box_center = reference_line.GetReferencePoint(box_center_s);
  double heading = reference_line.GetReferencePoint(obstacle_s).heading();
  static constexpr double kStopWallWidth = 4.0;
  Box2d stop_wall_box{box_center, heading, FLAGS_virtual_stop_wall_length,
                      kStopWallWidth};

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

/**
 * @brief: create static virtual object with lane width,
 *         mainly used for virtual stop wall
 */
const Obstacle *Frame::CreateStopObstacle(const std::string &obstacle_id,
                                          const std::string &lane_id,
                                          const double lane_s) {
  if (!hdmap_) {
    AERROR << "Invalid HD Map.";
    return nullptr;
  }
  const auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(lane_id));
  if (!lane) {
    AERROR << "Failed to find lane[" << lane_id << "]";
    return nullptr;
  }

  double dest_lane_s = std::max(0.0, lane_s);
  auto dest_point = lane->GetSmoothPoint(dest_lane_s);

  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  lane->GetWidth(dest_lane_s, &lane_left_width, &lane_right_width);

  Box2d stop_wall_box{{dest_point.x(), dest_point.y()},
                      lane->Heading(dest_lane_s),
                      FLAGS_virtual_stop_wall_length,
                      lane_left_width + lane_right_width - kWidthBuffer};

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

/**
 * @brief: create static virtual object with lane width,
 */
const Obstacle *Frame::CreateStaticObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const double obstacle_start_s,
    const double obstacle_end_s) {
  if (reference_line_info == nullptr) {
    AERROR << "reference_line_info nullptr";
    return nullptr;
  }

  const auto &reference_line = reference_line_info->reference_line();

  // start_xy
  common::SLPoint sl_point;
  sl_point.set_s(obstacle_start_s);
  sl_point.set_l(0.0);
  common::math::Vec2d obstacle_start_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_start_xy)) {
    AERROR << "Failed to get start_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }

  // end_xy
  sl_point.set_s(obstacle_end_s);
  sl_point.set_l(0.0);
  common::math::Vec2d obstacle_end_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_end_xy)) {
    AERROR << "Failed to get end_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }

  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  if (!reference_line.GetLaneWidth(obstacle_start_s, &left_lane_width,
                                   &right_lane_width)) {
    AERROR << "Failed to get lane width at s[" << obstacle_start_s << "]";
    return nullptr;
  }

  common::math::Box2d obstacle_box{
      common::math::LineSegment2d(obstacle_start_xy, obstacle_end_xy),
      left_lane_width + right_lane_width};

  return CreateStaticVirtualObstacle(obstacle_id, obstacle_box);
}

const Obstacle *Frame::CreateStaticObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const Vec2d &center, const double heading,
    const double length, const double width) {
  common::math::Box2d obs_box(center, heading, length, width);
  const auto *object = obstacles_.Find(obstacle_id);
  if (object) {
    AWARN << "obstacle " << obstacle_id << " already exist.";
    return object;
  }
  auto *ptr = obstacles_.Add(
      obstacle_id, *Obstacle::CreateStaticObstacles(obstacle_id, obs_box));

  if (!ptr) {
    AERROR << "Failed to create obstacle " << obstacle_id;
  }
  return ptr;
}
const Obstacle *Frame::CreateStackerObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const Vec2d &center, const double heading,
    const double length, const double width) {
  common::math::Box2d obs_box(center, heading, length, width);
  const auto *object = obstacles_.Find(obstacle_id);
  if (object) {
    AWARN << "obstacle " << obstacle_id << " already exist.";
    return object;
  }
  auto *ptr = obstacles_.Add(
      obstacle_id, *Obstacle::CreateStackerObstacles(obstacle_id, obs_box));

  if (!ptr) {
    AERROR << "Failed to create obstacle " << obstacle_id;
  }
  return ptr;
}
const Obstacle *Frame::CreateWheelCraneObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const Vec2d &center, const double heading,
    const double length, const double width) {
  common::math::Box2d obs_box(center, heading, length, width);
  const auto *object = obstacles_.Find(obstacle_id);
  if (object) {
    AWARN << "obstacle " << obstacle_id << " already exist.";
    return object;
  }
  auto *ptr = obstacles_.Add(
      obstacle_id, *Obstacle::CreateWheelCraneObstacles(obstacle_id, obs_box));

  if (!ptr) {
    AERROR << "Failed to create obstacle " << obstacle_id;
  }
  return ptr;
}
const Obstacle *Frame::CreateStackerObstacleWithID(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const Vec2d &center, const double heading,
    const double length, const double width, const double speed) {
  common::math::Box2d obs_box(center, heading, length, width);
  const auto *object = obstacles_.Find(obstacle_id);
  if (object) {
    AWARN << "obstacle " << obstacle_id << " already exist.";
    return object;
  }
  auto *ptr = obstacles_.Add(
      obstacle_id,
      *Obstacle::CreateStackerObstaclesWithID(obstacle_id, obs_box, speed));

  if (!ptr) {
    AERROR << "Failed to create obstacle " << obstacle_id;
  }
  return ptr;
}

const Obstacle *Frame::CreateStaticVirtualObstacle(const std::string &id,
                                                   const Box2d &box) {
  const auto *object = obstacles_.Find(id);
  if (object) {
    AWARN << "obstacle " << id << " already exist.";
    return object;
  }
  auto *ptr =
      obstacles_.Add(id, *Obstacle::CreateStaticVirtualObstacles(id, box));
  if (!ptr) {
    AERROR << "Failed to create virtual obstacle " << id;
  }
  return ptr;
}

Status Frame::Init(
    const common::VehicleStateProvider *vehicle_state_provider,
    const std::list<ReferenceLine> &reference_lines,
    const std::list<hdmap::RouteSegments> &segments,
    const std::vector<routing::LaneWaypoint> &future_route_waypoints,
    const EgoInfo *ego_info) {
  // TODO(QiL): refactor this to avoid redundant nullptr checks in scenarios.
  auto status = InitFrameData(vehicle_state_provider, ego_info);
  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }
  if (!CreateReferenceLineInfo(reference_lines, segments)) {
    const std::string msg = "Failed to init reference line info.";
    AERROR << msg;

    if (nullptr != local_view_.routing && local_view_.routing->has_header() &&
        std::abs(Clock::NowInSeconds() -
                 local_view_.routing->header().timestamp_sec()) <
            FLAGS_signal_expire_time_sec &&
        vehicle_state_.linear_velocity() < kEStopSpeed) {
      return Status(ErrorCode::PLANNING_ERROR_NEED_RESTART, msg);
    } else {
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
  future_route_waypoints_ = future_route_waypoints;
  overtake_state_ = ADCTrajectory::DEFAULT;
  return Status::OK();
}

Status Frame::InitForOpenSpace(
    const common::VehicleStateProvider *vehicle_state_provider,
    const EgoInfo *ego_info) {
  return InitFrameData(vehicle_state_provider, ego_info);
}

Status Frame::InitFrameData(
    const common::VehicleStateProvider *vehicle_state_provider,
    const EgoInfo *ego_info) {
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
  vehicle_state_ = vehicle_state_provider->vehicle_state();
  if (!util::IsVehicleStateValid(vehicle_state_)) {
    AERROR << "Adc init point is not set";
    return Status(ErrorCode::PLANNING_ERROR, "Adc init point is not set");
  }
  ADEBUG << "Enabled align prediction time ? : " << std::boolalpha
         << FLAGS_align_prediction_time;

  if (FLAGS_align_prediction_time) {
    auto prediction = *(local_view_.prediction_obstacles);
    AlignPredictionTime(vehicle_state_.timestamp(), &prediction);
    local_view_.prediction_obstacles->CopyFrom(prediction);
  }
  // AINFO << "*local_view_.prediction_obstacles.time_stamp = "
  //       << std::setprecision(16)
  //       << local_view_.prediction_obstacles->header().timestamp_sec();
  SetRecordTime(local_view_.prediction_obstacles->header().timestamp_sec());
  for (auto &ptr :
       Obstacle::CreateObstacles(*local_view_.prediction_obstacles)) {
    AddObstacle(*ptr);
  }
  if (planning_start_point_.v() < 1e-3) {
    const auto *collision_obstacle = FindCollisionObstacle(ego_info);
    if (collision_obstacle != nullptr) {
      const std::string msg = absl::StrCat("Found collision with obstacle: ",
                                           collision_obstacle->Id());
      AERROR << msg;
      monitor_logger_buffer_.ERROR(msg);
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  ReadTrafficLights();
  ReadSuperTrafficLights();
  ReadPadMsgDrivingAction();

  return Status::OK();
}

const Obstacle *Frame::FindCollisionObstacle(const EgoInfo *ego_info) const {
  if (obstacles_.Items().empty()) {
    return nullptr;
  }

  const auto &adc_polygon = Polygon2d(ego_info->ego_box());
  for (const auto &obstacle : obstacles_.Items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }

    const auto &obstacle_polygon = obstacle->PerceptionPolygon();
    if (obstacle_polygon.HasOverlap(adc_polygon)) {
      return obstacle;
    }
  }
  return nullptr;
}

uint32_t Frame::SequenceNum() const { return sequence_num_; }

std::string Frame::DebugString() const {
  return absl::StrCat("Frame: ", sequence_num_);
}

void Frame::RecordInputDebug(planning_internal::Debug *debug) {
  if (!debug) {
    ADEBUG << "Skip record input into debug";
    return;
  }
  auto *planning_debug_data = debug->mutable_planning_data();
  auto *adc_position = planning_debug_data->mutable_adc_position();
  adc_position->CopyFrom(*local_view_.localization_estimate);

  auto debug_chassis = planning_debug_data->mutable_chassis();
  debug_chassis->CopyFrom(*local_view_.chassis);

  if (!FLAGS_use_navigation_mode) {
    auto debug_routing = planning_debug_data->mutable_routing();
    debug_routing->CopyFrom(*local_view_.routing);
  }

  planning_debug_data->mutable_prediction_header()->CopyFrom(
      local_view_.prediction_obstacles->header());
  /*
  auto relative_map = AdapterManager::GetRelativeMap();
  if (!relative_map->Empty()) {
    planning_debug_data->mutable_relative_map()->mutable_header()->CopyFrom(
        relative_map->GetLatestObserved().header());
  }
  */
}

void Frame::AlignPredictionTime(const double planning_start_time,
                                PredictionObstacles *prediction_obstacles) {
  if (!prediction_obstacles || !prediction_obstacles->has_header() ||
      !prediction_obstacles->header().has_timestamp_sec()) {
    return;
  }
  double prediction_header_time =
      prediction_obstacles->header().timestamp_sec();
  for (auto &obstacle : *prediction_obstacles->mutable_prediction_obstacle()) {
    for (auto &trajectory : *obstacle.mutable_trajectory()) {
      for (auto &point : *trajectory.mutable_trajectory_point()) {
        point.set_relative_time(prediction_header_time + point.relative_time() -
                                planning_start_time);
      }
      if (!trajectory.trajectory_point().empty() &&
          trajectory.trajectory_point().begin()->relative_time() < 0) {
        auto it = trajectory.trajectory_point().begin();
        while (it != trajectory.trajectory_point().end() &&
               it->relative_time() < 0) {
          ++it;
        }
        trajectory.mutable_trajectory_point()->erase(
            trajectory.trajectory_point().begin(), it);
      }
    }
  }
}

Obstacle *Frame::Find(const std::string &id) { return obstacles_.Find(id); }

void Frame::AddObstacle(const Obstacle &obstacle) {
  obstacles_.Add(obstacle.Id(), obstacle);
}

void Frame::AddOpenSpaceRoiObstacle(const Obstacle &obstacle) {
  open_space_roi_obstacles_.Add(obstacle.Id(), obstacle);
}

void Frame::ReadTrafficLights() {
  traffic_lights_.clear();

  const auto &traffic_light_detection = local_view_.traffic_light;
  if (nullptr == traffic_light_detection) {
    return;
  }

  if (GlobalData::Instance()->GetSimulationState() ==
      century::cyber::common::SimulationState::NO_SIMULATION) {
    const double delay = Clock::NowInSeconds() -
                         traffic_light_detection->header().timestamp_sec();
    if (std::abs(delay) > FLAGS_signal_expire_time_sec) {
      ADEBUG << "traffic signals msg is expired, delay = " << delay
             << " seconds.";
      return;
    }
  }

  for (const auto &traffic_light : traffic_light_detection->traffic_light()) {
    traffic_lights_[traffic_light.id()] = &traffic_light;
  }
  // traffic_light_insight num is noless than  traffic_light
  traffic_insight_status_.clear();
  for (const auto &light : traffic_light_detection->tl_in_pixel_status()) {
    // AINFO << "light.id() " << light.id();
    // traffic_lights_center_[light.id()] = &light.center_point();
    traffic_insight_status_[light.id()] = &light;
    AINFO << "light.status() " << light.status() << "light.id() " << light.id();
  }
}

void Frame::ReadSuperTrafficLights() {
  super_traffic_lights_.clear();

  const auto &super_traffic_light = local_view_.super_traffic_light;
  if (nullptr == super_traffic_light) {
    return;
  }

  if (GlobalData::Instance()->GetSimulationState() ==
      century::cyber::common::SimulationState::NO_SIMULATION) {
    const double delay =
        Clock::NowInSeconds() - super_traffic_light->header().timestamp_sec();
    if (std::abs(delay) > FLAGS_signal_expire_time_sec) {
      AWARN << "super traffic signals msg is expired, delay = " << delay
            << " seconds.";
      return;
    }
  }

  super_traffic_lights_[super_traffic_light->id()] = super_traffic_light.get();
}

// wentaoliu: first , judge whether to adjust pose
// if needed, then fill light_pose
bool Frame::GetSignalCenterFromId(const std::string &traffic_light_id,
                                  century::common::PointENU *light_pose) {
  const auto *result_insight = century::common::util::FindPtrOrNull(
      traffic_insight_status_, traffic_light_id);
  if (nullptr == result_insight) {
    ADEBUG << "id match error , not enter astar";
    return false;
  }

  if (result_insight->status()) {
    ADEBUG << "id is insight return false";
    return false;
  }

  light_pose->set_x(result_insight->center_point().x());
  light_pose->set_y(result_insight->center_point().y());
  return true;
}

void Frame::GetSignal(const std::string &traffic_light_id,
                      perception::TrafficLight *const traffic_results) const {
  const auto *result_super = century::common::util::FindPtrOrNull(
      super_traffic_lights_, traffic_light_id);

  if (nullptr == result_super) {
    const auto *result =
        century::common::util::FindPtrOrNull(traffic_lights_, traffic_light_id);

    if (nullptr == result) {
      AERROR
          << "Failed find from perception traffic light list, reference line "
             "signal id : "
          << traffic_light_id;
      traffic_results->set_id(traffic_light_id);
      traffic_results->set_color(perception::TrafficLight::UNKNOWN);
      traffic_results->set_confidence(0.0);
      traffic_results->set_tracking_time(0.0);

    } else {
      traffic_results->set_id(result->id());
      traffic_results->set_color(result->color());
      traffic_results->set_confidence(result->confidence());
      traffic_results->set_tracking_time(result->tracking_time());
    }
  } else {
    traffic_results->set_id(result_super->id());
    traffic_results->set_color(perception::TrafficLight::GREEN);
    traffic_results->set_confidence(kSuperTrafficLightConfidence);
    traffic_results->set_tracking_time(0.0);
  }
}

bool Frame::IsNewestSuperTrafficLight() {
  // Do not check in the recorder playback state
  if (GlobalData::Instance()->GetSimulationState() ==
      century::cyber::common::SimulationState::
          RECORDER_PLAYBACK_WITH_SIM_CONTROL) {
    return true;
  }

  const auto *result_super = century::common::util::FindPtrOrNull(
      super_traffic_lights_, kSuperTrafficLightId);
  if (nullptr != result_super) {
    return true;
  }

  AWARN << "Can not find newest super traffic light. "
           "super_traffic_lights_.size() = "
        << super_traffic_lights_.size();
  return false;
}

bool Frame::NoGreenTrafficLight(const std::string &traffic_light_id) {
  const auto *result =
      century::common::util::FindPtrOrNull(traffic_lights_, traffic_light_id);
  if (nullptr == result) {
    return false;
  } else {
    return perception::TrafficLight::GREEN != result->color();
  }
}

void Frame::ReadPadMsgDrivingAction() {
  if (local_view_.pad_msg) {
    if (local_view_.pad_msg->has_action()) {
      pad_msg_driving_action_ = local_view_.pad_msg->action();
    }
  }
}

void Frame::ResetPadMsgDrivingAction() {
  pad_msg_driving_action_ = DrivingAction::NONE;
}

void Frame::SetPlanningContext(PlanningContext *context) { context_ = context; }

void Frame::SetCurrentTrafficLightId(const std::string &traffic_light_id) {
  current_traffic_light_id_ = traffic_light_id;
}

const std::string &Frame::GetCurrentTrafficLightId() const {
  return current_traffic_light_id_;
}

const ReferenceLineInfo *Frame::FindDriveReferenceLineInfo() {
  double min_cost = std::numeric_limits<double>::infinity();
  drive_reference_line_info_ = nullptr;
  for (const auto &reference_line_info : reference_line_info_) {
    if (reference_line_info.IsDrivable() &&
        reference_line_info.Cost() < min_cost) {
      drive_reference_line_info_ = &reference_line_info;
      min_cost = reference_line_info.Cost();
    }
  }
  return drive_reference_line_info_;
}

const ReferenceLineInfo *Frame::FindTargetReferenceLineInfo() {
  const ReferenceLineInfo *target_reference_line_info = nullptr;
  for (const auto &reference_line_info : reference_line_info_) {
    if (reference_line_info.IsChangeLanePath()) {
      return &reference_line_info;
    }
    target_reference_line_info = &reference_line_info;
  }
  return target_reference_line_info;
}

const ReferenceLineInfo *Frame::FindFailedReferenceLineInfo() {
  for (const auto &reference_line_info : reference_line_info_) {
    // Find the unsuccessful lane-change path
    if (!reference_line_info.IsDrivable() &&
        reference_line_info.IsChangeLanePath()) {
      return &reference_line_info;
    }
  }
  return nullptr;
}

const ReferenceLineInfo *Frame::DriveReferenceLineInfo() const {
  return drive_reference_line_info_;
}

const std::vector<const Obstacle *> Frame::obstacles() const {
  return obstacles_.Items();
}

const std::vector<const Obstacle *> Frame::open_space_roi_obstacles() const {
  return open_space_roi_obstacles_.Items();
}

void Frame::UpdateOvertakeStatus(
    const OvertakeStatus::Status &overtake_status) {
  reference_line_provider_->UpdateOvertakeStatus(overtake_status);
}

void Frame::UpdateLaneBorrowLaneId(const std::string &lane_borrow_lane_id) {
  reference_line_provider_->UpdateLaneBorrowLaneId(lane_borrow_lane_id);
}

void Frame::UpdateLaneChangeLaneId(const std::string &lane_change_lane_id) {
  reference_line_provider_->UpdateLaneChangeLaneId(lane_change_lane_id);
}

}  // namespace planning
}  // namespace century
