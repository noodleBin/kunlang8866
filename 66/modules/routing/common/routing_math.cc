/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
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

#include "modules/routing/common/routing_math.h"

namespace {

constexpr double kProjectionEpsilon = 1e-6;

template <typename LaneIdContainer>
std::shared_ptr<const century::hdmap::LaneInfo> GetPreferredNoTurnLane(
    const century::hdmap::HDMap* hdmap, const LaneIdContainer& lane_ids) {
  std::shared_ptr<const century::hdmap::LaneInfo> first_valid = nullptr;
  for (const auto& lane_id : lane_ids) {
    auto candidate = hdmap->GetLaneById(lane_id);
    if (nullptr == candidate) {
      continue;
    }
    if (century::hdmap::Lane::NO_TURN == candidate->lane().turn()) {
      return candidate;
    }
    if (nullptr == first_valid) {
      first_valid = candidate;
    }
  }
  return first_valid;
}

}  // namespace

namespace century {
namespace routing {

bool RoutingMath::Init(const hdmap::HDMap* hdmap) {
  if (!century::cyber::common::GetProtoFromFile(FLAGS_routing_point_file,
                                                &fixed_points_)) {
    AERROR << "failed to load file: " << FLAGS_routing_point_file;
    return false;
  }
  if (nullptr == hdmap) {
    return false;
  }
  hdmap_ = hdmap;

  routing::RoutingConfig routing_config;
  if (century::cyber::common::GetProtoFromFile(FLAGS_routing_conf_file,
                                               &routing_config) &&
      routing_config.has_crane_projection_config()) {
    crane_proj_config_ = routing_config.crane_projection_config();
  }
  AINFO << "CraneProjectionMode=" << crane_proj_config_.mode();
  return true;
}

bool RoutingMath::IsLaneUsable(
    const std::shared_ptr<const hdmap::LaneInfo> lane_info,
    const PointENU& point, double& s) {
  double l = 0.0;
  lane_info->GetProjection({point.x(), point.y()}, &s, &l);
  if (s < 0 || s > lane_info->total_length()) {
    return false;
  }
  double left_width = 0.0;
  double right_width = 0.0;
  lane_info->GetWidth(s, &left_width, &right_width);
  if (l > left_width || l < -right_width) {
    return false;
  }
  return true;
}

bool RoutingMath::IsPointWithinRangeLine(
    const PointENU& c, const std::string_view fix_point_area) {
  auto temp_fixed_points = GetFixedPoints(fix_point_area);
  auto b = temp_fixed_points.front();
  auto a = temp_fixed_points.back();
  double abx = b.x() - a.x();
  double aby = b.y() - a.y();
  double acx = c.x() - a.x();
  double acy = c.y() - a.y();
  double abDotAb = abx * abx + aby * aby;
  if (common::util::IsZero(abDotAb)) {
    return false;
  }
  double t = (acx * abx + acy * aby) / abDotAb;
  return t >= 0 && t <= 1;
}

bool RoutingMath::IsPointWithinRangeLine(const PointENU& c, const PointENU& b,
                                         const PointENU& a) {
  double abx = b.x() - a.x();
  double aby = b.y() - a.y();
  double acx = c.x() - a.x();
  double acy = c.y() - a.y();
  double abDotAb = abx * abx + aby * aby;
  if (common::util::IsZero(abDotAb)) {
    return false;
  }
  double t = (acx * abx + acy * aby) / abDotAb;
  return t >= 0 && t <= 1;
}

bool RoutingMath::IsPointInSpecificJunction(const LaneWaypoint& lane_waypoint,
                                            const int junction_type) {
  std::vector<JunctionInfoConstPtr> junctions;
  common::PointENU point_enu;
  common::math::Vec2d point(lane_waypoint.pose().x(), lane_waypoint.pose().y());
  point_enu.set_x(lane_waypoint.pose().x());
  point_enu.set_y(lane_waypoint.pose().y());
  if (0 == hdmap_->GetJunctions(point_enu, FLAGS_routing_junction_search_radius, &junctions)) {
    for (const auto& ptr_junction : junctions) {
      if (ptr_junction->polygon().IsPointIn(point)) {
        // AINFO << "junction: "
        //       << ptr_junction->junction().Type_Name(
        //              ptr_junction->junction().type());
      } else {
        continue;
      }
      if (junction_type == ptr_junction->junction().type()) {
        return true;
      }
    }
  }
  return false;
}

bool RoutingMath::IsJunctionContainAdc(
    const LaneWaypoint& lane_waypoint,
    const hdmap::JunctionInfo& junction_info) {
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  // Compute the ADC bounding box.
  Vec2d ego_center_map_frame((vehicle_param.front_edge_to_center() -
                              vehicle_param.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param.left_edge_to_center() -
                              vehicle_param.right_edge_to_center()) *
                                 0.5);
  ego_center_map_frame.SelfRotate(lane_waypoint.heading());
  ego_center_map_frame.set_x(lane_waypoint.pose().x() +
                             ego_center_map_frame.x());
  ego_center_map_frame.set_y(lane_waypoint.pose().y() +
                             ego_center_map_frame.y());
  Box2d adc_box(ego_center_map_frame, lane_waypoint.heading(),
                vehicle_param.length(), vehicle_param.width());
  // Check whether Junction's polygon contain ADC bounding box.
  const auto& polygon = junction_info.polygon();
  return polygon.Contains(Polygon2d(adc_box));
}

bool RoutingMath::IsRelativeTargetPointToWeat(
    const PointENU& front_point, const PointENU& back_point,
    const std::string_view fix_point_area) {
  auto proj_point = ProjectPointToLine(front_point, fix_point_area);
  double heading_back_to_fixed = GetTwoPointsDirection(back_point, proj_point);
  double diff_back_heading =
      common::math::NormalizeAngle(FLAGS_j4_west_fixed_heading - heading_back_to_fixed);
  if (diff_back_heading <= M_PI_2) {
    return true;
  } else {
    return false;
  }
}

bool RoutingMath::IsFixedPointClose(const PointENU& frist,
                                    const PointENU& second,
                                    double min_point_distance) {
  double two_front_points_distance = GetTwoPointsDistance(frist, second);
  if (two_front_points_distance < min_point_distance) {
    return true;
  }
  return false;
}

bool RoutingMath::IsSatisfyLaneChange(const RoadSegment& road_passage,
                                      const double& lane_change_length) {
  int lane_change_number = road_passage.passage_size();
  double min_segment_length = std::numeric_limits<double>::max();

  for (const auto& lane_segment : road_passage.passage()) {
    double current_segment_length = 0.0;
    for (const auto& segment : lane_segment.segment()) {
      current_segment_length = current_segment_length + (segment.end_s() - segment.start_s());
    }
    if (current_segment_length < min_segment_length) {
      min_segment_length = current_segment_length;
    }
  }
  return min_segment_length > (lane_change_length * (lane_change_number - 1));
}

bool RoutingMath::IsLaneInRoad(const std::string& lane_id,
                               const RoadSegment& road) {
  for (const auto& passage : road.passage()) {
    for (const auto& segment : passage.segment()) {
      if (segment.id() == lane_id) {
        return true;
      }
    }
  }
  return false;
}

std::vector<PointENU> RoutingMath::GetFixedPoints(std::string_view id) {
  for (const auto& fixed_points : fixed_points_.fixed_points()) {
    if (id == fixed_points.id()) {
      return std::vector<PointENU>(fixed_points.pose().begin(),
                                   fixed_points.pose().end());
    }
  }
  return std::vector<PointENU>();
}

double RoutingMath::GetTwoPointsDirection(const common::PointENU& frist,
                                          const common::PointENU& second) {
  double dx = second.x() - frist.x();
  double dy = second.y() - frist.y();
  return std::atan2(dy, dx);
}

double RoutingMath::GetTwoPointsDistance(const common::PointENU& frist,
                                         const common::PointENU& second) {
  double dx = second.x() - frist.x();
  double dy = second.y() - frist.y();
  return sqrt(pow(dx, 2) + pow(dy, 2));
}

bool RoutingMath::GetLaneNearestPointByIdAndPoint(
    const hdmap::Id& neighbor_id, const LaneWaypoint& lane_waypoint,
    std::shared_ptr<const hdmap::LaneInfo>& target_lane,
    PointENU& neast_point) {
  target_lane = hdmap_->GetLaneById(neighbor_id);
  if (nullptr == target_lane) {
    AERROR << "Get lane id failed:" << neighbor_id.id();
    return false;
  }
  common::math::Vec2d point;
  point.set_x(lane_waypoint.pose().x());
  point.set_y(lane_waypoint.pose().y());
  double distance_temp = 0.0;
  neast_point = target_lane->GetNearestPoint(point, &distance_temp);
  return true;
}

PointENU RoutingMath::GetLaneNearestPointByIdAndPoint(
    const hdmap::Id& neighbor_id, const LaneWaypoint& lane_waypoint) {
  auto target_lane = hdmap_->GetLaneById(neighbor_id);
  common::math::Vec2d point;
  point.set_x(lane_waypoint.pose().x());
  point.set_y(lane_waypoint.pose().y());
  double distance_temp = 0.0;
  return target_lane->GetNearestPoint(point, &distance_temp);
}

bool RoutingMath::GetWheelCraneProjectionLaneInfor(
    LaneWaypoint& lane_waypoint) {
  LaneWaypoint new_lane_waypoint(lane_waypoint);
  auto proj_point = ProjectPointToLine(new_lane_waypoint.pose(), "WHOLE_J4_1");
  lane_waypoint.mutable_pose()->set_x(proj_point.x());
  lane_waypoint.mutable_pose()->set_y(proj_point.y());
  return true;
}

void RoutingMath::GetWaypointInfoByPointProjection(
    const std::shared_ptr<const hdmap::LaneInfo> target_lane,
    const PointENU& neast_point, LaneWaypoint& lane_waypoint) {
  double accumulate_s = 0.0;
  double lateral = 0.0;
  common::math::Vec2d point;
  point.set_x(neast_point.x());
  point.set_y(neast_point.y());
  if (!target_lane->GetProjection(point, &accumulate_s, &lateral)) {
    AINFO << "GetProjection is false";
  }
  hdmap::Id target_id = target_lane->id();
  lane_waypoint.mutable_pose()->set_x(neast_point.x());
  lane_waypoint.mutable_pose()->set_y(neast_point.y());
  lane_waypoint.set_id(target_lane->lane().id().id());
  lane_waypoint.set_s(accumulate_s);
  lane_waypoint.set_heading(target_lane->Heading(accumulate_s));
  AINFO << "Get crane lane success";
}

double RoutingMath::GetNoTurnLaneHeading(const double& stacker_heading,
                                         const LaneWaypoint& lane_waypoint,
                                         bool is_to_front) {
  auto adc_lane_id = lane_waypoint.id();
  auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(adc_lane_id));
  double heading = lane->Heading(lane_waypoint.s());
  double temp_stacker_heading =
      common::math::NormalizeAngle(stacker_heading - M_PI_2);
  double diff_heading =
      std::fabs(common::math::NormalizeAngle(heading - temp_stacker_heading));
  AINFO << "heading:" << heading << ",stacker_heading:" << stacker_heading
        << ",temp_stacker_heading:" << temp_stacker_heading
        << ",is_to_front:" << is_to_front << ",diff_heading:"
        << diff_heading;  // << ",diff_heading1:" << diff_heading1;
  double temp_heading = common::math::NormalizeAngle((heading + M_PI));
  if (is_to_front) {
    heading = (diff_heading < M_PI_2 ? heading : temp_heading);
  } else {
    heading = (diff_heading < M_PI_2 ? temp_heading : heading);
  }
  return heading;
}

std::vector<PointENU> RoutingMath::GetHumanShapePoints(
    const PointENU& point, const std::string_view fix_point_area,
    bool is_in_ego_lane, bool is_west) {
  std::vector<PointENU> human_shape_points;
  auto point_J4_1 = ProjectPointToLine(point, "HS_J4_1");
  auto point_J4_3_temp = ProjectPointToLine(point, "HS_J4_3");
  auto point_J4_3 =
      MoveAlongLine(point_J4_3_temp, "HS_J4_3",
                    FLAGS_human_shape_midlle_distance, is_west ? false : true);
  auto point_J4_5 = ProjectPointToLine(point, "HS_J4_5");
  if ("HS_J4_1" == fix_point_area) {
    if (is_in_ego_lane) {
      human_shape_points.push_back(point_J4_1);
      human_shape_points.push_back(point_J4_3);
      human_shape_points.push_back(point_J4_5);
    } else {
      human_shape_points.push_back(point_J4_5);
      human_shape_points.push_back(point_J4_3);
      human_shape_points.push_back(point_J4_1);
    }
  } else if ("HS_J4_5" == fix_point_area) {
    if (is_in_ego_lane) {
      human_shape_points.push_back(point_J4_5);
      human_shape_points.push_back(point_J4_3);
      human_shape_points.push_back(point_J4_1);
    } else {
      human_shape_points.push_back(point_J4_1);
      human_shape_points.push_back(point_J4_3);
      human_shape_points.push_back(point_J4_5);
    }
  }
  return human_shape_points;
}

bool RoutingMath::GetSearchDirection(
    const LaneWaypoint& lane_waypoint, const hdmap::Id& lane_id,
    std::unordered_map<SearchDirection, std::vector<hdmap::Id>>& target_lanes) {
  auto near_lane = hdmap_->GetLaneById(lane_id);
  double s = 0.0, l = 0.0;
  common::math::Vec2d point;
  point.set_x(lane_waypoint.pose().x());
  point.set_y(lane_waypoint.pose().y());
  near_lane->GetProjection(point, &s, &l);
  if (s >= 0 && s <= near_lane->total_length()) {
    double heading = near_lane->Heading(s);
    double stacker_heading = lane_waypoint.heading();
    double temp_stacker_heading =
        common::math::NormalizeAngle(stacker_heading - M_PI_2);
    double diff_heading =
        std::fabs(common::math::NormalizeAngle(heading - temp_stacker_heading));
    AINFO << "ID:" << lane_id.id() << ",heading:" << heading
          << ",stacker_heading:" << stacker_heading
          << ",temp_stacker_heading:" << temp_stacker_heading;
    AINFO << ",s:" << s << ",diff_heading:" << diff_heading
          << ",total_length:" << near_lane->total_length();
    const bool is_no_turn =
        (century::hdmap::Lane::NO_TURN == near_lane->lane().turn());
    if (diff_heading < M_PI_2) {
      auto& lanes = target_lanes[LEFT_DIRECTION];
      if (is_no_turn) {
        lanes.insert(lanes.begin(), lane_id);
      } else {
        lanes.emplace_back(lane_id);
      }
      AINFO << "LEFT_DIRECTION is_no_turn:" << is_no_turn;
      return true;
    } else if (diff_heading > M_PI_2) {
      auto& lanes = target_lanes[RIGHT_DIRECTION];
      if (is_no_turn) {
        lanes.insert(lanes.begin(), lane_id);
      } else {
        lanes.emplace_back(lane_id);
      }
      AINFO << "RIGHT_DIRECTION is_no_turn:" << is_no_turn;
      return true;
    }
  } else {
    AERROR << "ID:" << lane_id.id() << " not within the range of the interval";
  }
  return false;
}

bool RoutingMath::GetWaypointAlongSuccessorIdByDistance(
    const LaneWaypoint& adc_waypoint, LaneWaypoint& end_lane_waypoint,
    double move_distance) {
  auto adc_lane_id = adc_waypoint.id();
  hdmap::Id preferred_id = hdmap::MakeMapId(adc_lane_id);
  auto end_point_lane = hdmap_->GetLaneById(preferred_id);
  double end_point_distance = end_point_lane->total_length() - adc_waypoint.s();
  double end_point_lane_s = 0.0;
  while (end_point_distance <= move_distance &&
         !end_point_lane->lane().successor_id().empty()) {
    end_point_lane_s = end_point_distance;
    preferred_id = end_point_lane->lane().successor_id(0);
    // Find the same heading
    double max_diff_heading = std::numeric_limits<double>::max();
    for (size_t i = 0; i < end_point_lane->lane().successor_id().size(); ++i) {
      double predecessor_lane_length =
          hdmap_->GetLaneById(end_point_lane->lane().successor_id(i))
              ->total_length();
      double target_lane_length = end_point_lane->total_length();
      double predecessor_lane_heading =
          hdmap_->GetLaneById(end_point_lane->lane().successor_id(i))
              ->Heading(0.5 * predecessor_lane_length);
      double target_lane_heading =
          end_point_lane->Heading(0.5 * target_lane_length);
      double diff_heading = common::math::NormalizeAngle(
          predecessor_lane_heading - target_lane_heading);
      AINFO << "diff_heading = " << diff_heading;
      if (std::fabs(diff_heading) < max_diff_heading) {
        preferred_id = end_point_lane->lane().successor_id(i);
        max_diff_heading = std::fabs(diff_heading);
      }
    }
    AINFO << "preferred_id = " << preferred_id.id();
    AINFO << "preferred_id.S = "
          << hdmap_->GetLaneById(preferred_id)->total_length();
    end_point_distance =
        end_point_distance + hdmap_->GetLaneById(preferred_id)->total_length();
    end_point_lane = hdmap_->GetLaneById(preferred_id);
    AINFO << "end_point_distance = " << end_point_distance;
  }

  end_point_lane_s = end_point_lane_s > 0.0 ? move_distance - end_point_lane_s
                                            : move_distance + adc_waypoint.s();
  AINFO << "end_point_lane_s = " << end_point_lane_s;
  const auto& point =
      hdmap_->GetLaneById(preferred_id)->GetSmoothPoint(end_point_lane_s);
  end_lane_waypoint.mutable_pose()->set_x(point.x());
  end_lane_waypoint.mutable_pose()->set_y(point.y());
  end_lane_waypoint.set_id(hdmap_->GetLaneById(preferred_id)->id().id());
  end_lane_waypoint.set_s(end_point_lane_s);
  if (end_point_distance <= move_distance &&
      end_point_lane->lane().successor_id().empty()) {
    return false;
  }
  return true;
}

bool RoutingMath::GetWaypointAlongPredecessorIdByDistance(
    const LaneWaypoint& adc_waypoint, LaneWaypoint& end_lane_waypoint,
    double move_distance) {
  auto adc_lane_id = adc_waypoint.id();
  hdmap::Id preferred_id = hdmap::MakeMapId(adc_lane_id);
  auto end_point_lane = hdmap_->GetLaneById(preferred_id);
  double end_point_distance = adc_waypoint.s();
  double end_point_lane_s = 0.0;
  while (end_point_distance <= move_distance &&
         !end_point_lane->lane().predecessor_id().empty()) {
    end_point_lane_s = end_point_distance;
    preferred_id = end_point_lane->lane().predecessor_id(0);
    // Find the same heading
    double max_diff_heading = std::numeric_limits<double>::max();
    for (size_t i = 0; i < end_point_lane->lane().predecessor_id().size();
         ++i) {
      double predecessor_lane_length =
          hdmap_->GetLaneById(end_point_lane->lane().predecessor_id(i))
              ->total_length();
      double target_lane_length = end_point_lane->total_length();
      double predecessor_lane_heading =
          hdmap_->GetLaneById(end_point_lane->lane().predecessor_id(i))
              ->Heading(0.5 * predecessor_lane_length);
      double target_lane_heading =
          end_point_lane->Heading(0.5 * target_lane_length);
      double diff_heading = common::math::NormalizeAngle(
          predecessor_lane_heading - target_lane_heading);
      AINFO << "diff_heading = " << diff_heading
            << ",id:" << end_point_lane->lane().predecessor_id(i).id()
            << ",max_diff_heading:" << max_diff_heading;
      // AINFO << "--------------std::fabs(diff_heading):" <<
      // std::fabs(diff_heading);
      if (std::fabs(diff_heading) < max_diff_heading) {
        preferred_id = end_point_lane->lane().predecessor_id(i);
        max_diff_heading = std::fabs(diff_heading);
      }
    }
    AINFO << "preferred_id = " << preferred_id.id();
    AINFO << "preferred_id.S = "
          << hdmap_->GetLaneById(preferred_id)->total_length();
    end_point_distance =
        end_point_distance + hdmap_->GetLaneById(preferred_id)->total_length();
    end_point_lane = hdmap_->GetLaneById(preferred_id);
    AINFO << "end_point_distance = " << end_point_distance;
  }

  end_point_lane_s = end_point_lane_s > 0.0 ? end_point_lane_s - move_distance
                                            : adc_waypoint.s() - move_distance;
  AINFO << "end_point_lane_s = " << end_point_lane_s;
  const auto& point =
      hdmap_->GetLaneById(preferred_id)->GetSmoothPoint(end_point_lane_s);
  end_lane_waypoint.mutable_pose()->set_x(point.x());
  end_lane_waypoint.mutable_pose()->set_y(point.y());
  end_lane_waypoint.set_id(hdmap_->GetLaneById(preferred_id)->id().id());
  end_lane_waypoint.set_s(end_point_lane_s);
  if (end_point_distance <= move_distance &&
      end_point_lane->lane().successor_id().empty()) {
    return false;
  }
  return true;
}

bool RoutingMath::GetNearestProjection(
    const LaneWaypoint& lane_waypoint,
    std::shared_ptr<const hdmap::LaneInfo>& out_lane,
    PointENU& out_point) {
  std::shared_ptr<const hdmap::LaneInfo> crane_lane;
  double nearest_lon_s, nearest_lat_l;
  if (hdmap_->GetNearestLane(lane_waypoint.pose(), &crane_lane, &nearest_lon_s,
                             &nearest_lat_l) < 0) {
    AERROR << "GetNearestProjection: GetNearestLane failed";
    return false;
  }
  AINFO << "crane_lane->lane().is_merge() = "
        << crane_lane->lane().is_merge();
  std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
  hdmap_->GetLanes(lane_waypoint.pose(), crane_proj_config_.adc_lane_range(),
                   &lanes);
  std::unordered_map<SearchDirection, std::vector<hdmap::Id>> target_lanes;
  if (lanes.size() > 1 && !crane_lane->lane().is_merge()) {
    for (size_t j = 0; j < lanes.size(); ++j) {
      AINFO << "< " << j << " >";
      GetSearchDirection(lane_waypoint, lanes[j]->id(), target_lanes);
    }
    if (!target_lanes.empty()) {
      LaneWaypoint temp_waypoint(lane_waypoint);
      if (SearchNeighborLaneInfo(target_lanes, temp_waypoint)) {
        out_lane = hdmap_->GetLaneById(
            hdmap::MakeMapId(temp_waypoint.id()));
        out_point.set_x(temp_waypoint.pose().x());
        out_point.set_y(temp_waypoint.pose().y());
        AINFO << "NearestProjection (neighbor): x=" << out_point.x()
              << ", y=" << out_point.y();
        return true;
      }
      return false;
    }
  }
  // No neighbor lane or single lane, project to crane_lane directly
  double distance_temp = 0.0;
  common::math::Vec2d point(lane_waypoint.pose().x(),
                            lane_waypoint.pose().y());
  const auto& neast_point =
      crane_lane->GetNearestPoint(point, &distance_temp);
  out_lane = crane_lane;
  out_point.set_x(neast_point.x());
  out_point.set_y(neast_point.y());
  AINFO << "NearestProjection: x=" << out_point.x()
        << ", y=" << out_point.y();
  return true;
}

bool RoutingMath::GetHeadingProjection(
    const LaneWaypoint& lane_waypoint,
    std::shared_ptr<const hdmap::LaneInfo>& out_lane,
    PointENU& out_point) {
  using TargetLaneMap = std::unordered_map<SearchDirection,
                                           std::vector<hdmap::Id>>;
  std::shared_ptr<const hdmap::LaneInfo> crane_lane;
  double nearest_lon_s, nearest_lat_l;
  if (hdmap_->GetNearestLane(lane_waypoint.pose(), &crane_lane, &nearest_lon_s,
                             &nearest_lat_l) < 0) {
    AERROR << "GetHeadingProjection: GetNearestLane failed";
    return false;
  }
  auto walk_to_neighbor_edge_lane =
      [&](const hdmap::Id& lane_id,
          SearchDirection direction) -> std::shared_ptr<const hdmap::LaneInfo> {
    auto lane = hdmap_->GetLaneById(lane_id);
    if (nullptr == lane) {
      return nullptr;
    }
    if (LEFT_DIRECTION == direction) {
      while (lane->lane().left_neighbor_forward_lane_id().size() > 0) {
        lane = hdmap_->GetLaneById(lane->lane().left_neighbor_forward_lane_id(0));
      }
    } else if (RIGHT_DIRECTION == direction) {
      while (lane->lane().right_neighbor_forward_lane_id().size() > 0) {
        lane = hdmap_->GetLaneById(
            lane->lane().right_neighbor_forward_lane_id(0));
      }
    }
    return lane;
  };
  std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
  hdmap_->GetLanes(lane_waypoint.pose(), crane_proj_config_.adc_lane_range(),
                   &lanes);
  TargetLaneMap target_lanes;
  if (lanes.size() > 1 && !crane_lane->lane().is_merge()) {
    for (const auto& lane : lanes) {
      GetSearchDirection(lane_waypoint, lane->id(), target_lanes);
    }
  }
  auto target_lane = crane_lane;
  for (const auto& [direction, lane_ids] : target_lanes) {
    if (lane_ids.empty()) {
      continue;
    }
    for (const auto& lane_id : lane_ids) {
      auto temp_lane = walk_to_neighbor_edge_lane(lane_id, direction);
      if (nullptr == temp_lane) {
        continue;
      }
      target_lane = temp_lane;
      AINFO << "HeadingProjection: walk to "
            << (LEFT_DIRECTION == direction ? "left-most lane "
                                            : "right-most lane ")
            << target_lane->id().id();
      break;
    }
    if (target_lane != crane_lane) {
      break;
    }
  }
  common::math::Vec2d wp_vec(lane_waypoint.pose().x(),
                             lane_waypoint.pose().y());
  double distance_temp = 0.0;
  auto nearest_pt = target_lane->GetNearestPoint(wp_vec, &distance_temp);
  double nearest_s = 0.0;
  double nearest_l = 0.0;
  common::math::Vec2d nearest_vec(nearest_pt.x(), nearest_pt.y());
  target_lane->GetProjection(nearest_vec, &nearest_s, &nearest_l);
  const double kMinDist = crane_proj_config_.proj_min_point_dist();
  const double kCoarseStep = crane_proj_config_.heading_proj_coarse_step();
  const double kFineStep = crane_proj_config_.heading_proj_fine_step();
  const int kMaxIter = crane_proj_config_.heading_proj_max_iter();
  const double kMaxDiff =
      crane_proj_config_.heading_proj_max_diff_deg() * M_PI / 180.0;
  auto set_projection_output =
      [&](const std::shared_ptr<const hdmap::LaneInfo>& lane, double s) {
    auto proj = lane->GetSmoothPoint(s);
    out_lane = lane;
    out_point.set_x(proj.x());
    out_point.set_y(proj.y());
  };
  auto calc_heading_diff =
      [&](const std::shared_ptr<const hdmap::LaneInfo>& lane, double s) {
    auto pt = lane->GetSmoothPoint(s);
    double dx = pt.x() - lane_waypoint.pose().x();
    double dy = pt.y() - lane_waypoint.pose().y();
    if (std::sqrt(dx * dx + dy * dy) < kMinDist) {
      return M_PI;
    }
    double direction = std::atan2(dy, dx);
    return std::fabs(
        common::math::NormalizeAngle(direction - lane_waypoint.heading()));
  };
  auto search_best_s =
      [&](const std::shared_ptr<const hdmap::LaneInfo>& lane, double start_s) {
    double best_s = start_s;
    double best_diff = calc_heading_diff(lane, start_s);
    for (int sign : {1, -1}) {
      double prev_diff = best_diff;
      for (double offset = kCoarseStep;; offset += kCoarseStep) {
        double s = start_s + sign * offset;
        if (s < 0.0 || s > lane->total_length()) {
          break;
        }
        double diff = calc_heading_diff(lane, s);
        if (diff < best_diff) {
          best_diff = diff;
          best_s = s;
        }
        if (diff <= prev_diff) {
          prev_diff = diff;
          continue;
        }
        double fine_start = s - sign * kCoarseStep;
        double fine_end = s;
        if (fine_start > fine_end) {
          std::swap(fine_start, fine_end);
        }
        for (double fine_s = fine_start; fine_s <= fine_end;
             fine_s += kFineStep) {
          double fine_diff = calc_heading_diff(lane, fine_s);
          if (fine_diff < best_diff) {
            best_diff = fine_diff;
            best_s = fine_s;
          }
        }
        break;
      }
    }
    return std::make_pair(best_s, best_diff);
  };
  auto current_lane = target_lane;
  double current_start_s = nearest_s;
  for (int iter = 0; iter < kMaxIter; ++iter) {
    auto [best_s, best_diff] = search_best_s(current_lane, current_start_s);

    AINFO << "HeadingProjection iter=" << iter
          << ", lane=" << current_lane->id().id()
          << ", best_s=" << best_s << ", best_diff=" << best_diff
          << ", total_length=" << current_lane->total_length();

    if (best_diff >= kMaxDiff) {
      AINFO << "HeadingProjection: no valid match on lane "
            << current_lane->id().id();
      return false;
    }
    const double kBoundaryMargin =
        crane_proj_config_.heading_proj_boundary_margin();
    bool at_start = (best_s < kBoundaryMargin);
    bool at_end = (best_s > current_lane->total_length() - kBoundaryMargin);
    if (!at_start && !at_end) {
      set_projection_output(current_lane, best_s);
      AINFO << "HeadingProjection final: x=" << out_point.x()
            << ", y=" << out_point.y()
            << ", heading_diff=" << best_diff;
      return true;
    }
    std::shared_ptr<const hdmap::LaneInfo> next_lane = nullptr;
    if (at_start && !current_lane->lane().predecessor_id().empty()) {
      next_lane = hdmap_->GetLaneById(current_lane->lane().predecessor_id(0));
    } else if (at_end && !current_lane->lane().successor_id().empty()) {
      next_lane = hdmap_->GetLaneById(current_lane->lane().successor_id(0));
    }
    if (!next_lane) {
      set_projection_output(current_lane, best_s);
      AINFO << "HeadingProjection boundary final: x=" << out_point.x()
            << ", y=" << out_point.y()
            << ", heading_diff=" << best_diff;
      return true;
    }
    AINFO << "HeadingProjection: at " << (at_start ? "start" : "end")
          << ", try "
          << (at_start ? "predecessor " : "successor ")
          << next_lane->id().id();
    for (const auto& [direction, lane_ids] : target_lanes) {
      if (lane_ids.empty()) {
        continue;
      }
      next_lane = walk_to_neighbor_edge_lane(next_lane->id(), direction);
      break;
    }
    current_start_s = at_start ? next_lane->total_length() : 0.0;
    current_lane = next_lane;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Helpers for GetHeadingProjectionFromLane
// ---------------------------------------------------------------------------

double RoutingMath::CalcHeadingDiff(
    const LaneWaypoint& lane_waypoint,
    const std::shared_ptr<const hdmap::LaneInfo>& lane, double s) {
  const double kMinDist = crane_proj_config_.proj_min_point_dist();
  auto pt = lane->GetSmoothPoint(s);
  double dx = pt.x() - lane_waypoint.pose().x();
  double dy = pt.y() - lane_waypoint.pose().y();
  if (std::sqrt(dx * dx + dy * dy) < kMinDist) {
    return M_PI;
  }
  double direction = std::atan2(dy, dx);
  return std::fabs(
      common::math::NormalizeAngle(direction - lane_waypoint.heading()));
}

std::shared_ptr<const hdmap::LaneInfo> RoutingMath::GetStraightSuccessor(
    const std::shared_ptr<const hdmap::LaneInfo>& lane) {
  if (lane->lane().successor_id().empty()) {
    return nullptr;
  }
  return GetPreferredNoTurnLane(hdmap_, lane->lane().successor_id());
}

std::shared_ptr<const hdmap::LaneInfo> RoutingMath::GetStraightPredecessor(
    const std::shared_ptr<const hdmap::LaneInfo>& lane) {
  if (lane->lane().predecessor_id().empty()) {
    return nullptr;
  }
  return GetPreferredNoTurnLane(hdmap_, lane->lane().predecessor_id());
}

bool RoutingMath::AdvanceAlongLane(
    std::shared_ptr<const hdmap::LaneInfo>& lane, double& s,
    double step, int& lane_count) {
  s += step;
  if (s > lane->total_length()) {
    auto next = GetStraightSuccessor(lane);
    if (!next) {
      return false;
    }
    s -= lane->total_length();
    lane = next;
    ++lane_count;
    AINFO << "HeadingProjectionFromLane: cross to successor "
          << lane->id().id();
  } else if (s < 0.0) {
    auto prev = GetStraightPredecessor(lane);
    if (!prev) {
      return false;
    }
    s += prev->total_length();
    lane = prev;
    ++lane_count;
    AINFO << "HeadingProjectionFromLane: cross to predecessor "
          << lane->id().id();
  }
  return true;
}

// ---------------------------------------------------------------------------
// GetHeadingProjectionFromLane: coarse(+1m) then fine(-0.1m) search
// ---------------------------------------------------------------------------

bool RoutingMath::GetHeadingProjectionFromLane(
    const LaneWaypoint& lane_waypoint,
    const std::shared_ptr<const hdmap::LaneInfo>& start_lane,
    double start_s,
    const PointENU& nearest_point,
    std::shared_ptr<const hdmap::LaneInfo>& out_lane,
    PointENU& out_point) {
  const double kCoarseStep =
      crane_proj_config_.heading_from_lane_coarse_step();
  const double kFineStep = crane_proj_config_.heading_from_lane_fine_step();
  const int kMaxLaneCount =
      crane_proj_config_.heading_from_lane_max_lane_count();
  const double kMaxSearchDist =
      crane_proj_config_.heading_from_lane_max_search_dist();
  const double kMaxDiffThreshold =
      crane_proj_config_.heading_from_lane_init_max_diff_deg() * M_PI / 180.0;

  double init_diff = CalcHeadingDiff(lane_waypoint, start_lane, start_s);
  AINFO << "HeadingProjectionFromLane: init_diff=" << init_diff
        << " at start_s=" << start_s
        << ", lane=" << start_lane->id().id();

  if (init_diff > kMaxDiffThreshold) {
    AINFO << "HeadingProjectionFromLane: init heading diff > "
          << crane_proj_config_.heading_from_lane_init_max_diff_deg()
          << " deg, return";
    return false;
  }

  // Best result tracker — also tracks arc-length from start_s for blend
  auto best_lane = start_lane;
  double best_s = start_s;
  double best_diff = init_diff;
  double best_arc_from_start = 0.0;  // arc-length along lane to best position

  // Phase 1: Coarse forward search (+1m)
  auto cur_lane = start_lane;
  double cur_s = start_s;
  double prev_diff = init_diff;
  double total_dist = 0.0;
  int lane_count = 0;
  bool need_fine_refine = false;
  std::shared_ptr<const hdmap::LaneInfo> fine_start_lane;
  double fine_start_s = 0.0;
  double fine_start_arc = 0.0;  // arc-length from start_s to fine_start

  while (total_dist < kMaxSearchDist && lane_count < kMaxLaneCount) {
    auto next_lane = cur_lane;
    double next_s = cur_s;
    if (!AdvanceAlongLane(next_lane, next_s, kCoarseStep, lane_count)) {
      break;
    }
    total_dist += kCoarseStep;
    if (total_dist > kMaxSearchDist) {
      AINFO << "break total_dist: " << total_dist;
      break;
    }
    double diff = CalcHeadingDiff(lane_waypoint, next_lane, next_s);
    if (diff < best_diff) {
      best_diff = diff;
      best_s = next_s;
      best_lane = next_lane;
      best_arc_from_start = total_dist;
    }
    if (diff > prev_diff) {
      fine_start_lane = next_lane;
      fine_start_s = next_s;
      fine_start_arc = total_dist;
      need_fine_refine = true;
      AINFO << "HeadingProjectionFromLane: fine refine triggered"
            << " fine_start_lane=" << fine_start_lane->id().id()
            << " fine_start_s=" << fine_start_s
            << " fine_start_arc=" << fine_start_arc
            << " diff=" << diff << " lane_count:" << lane_count;
      break;
    }
    prev_diff = diff;
    cur_lane = next_lane;
    cur_s = next_s;
  }

  // Phase 2: Fine backward refinement (-0.1m)
  if (need_fine_refine) {
    auto ref_lane = fine_start_lane;
    double ref_s = fine_start_s;
    int fine_lc = lane_count;
    double fine_dist = 0.0;
    prev_diff = CalcHeadingDiff(lane_waypoint, ref_lane, ref_s);

    while (fine_dist < kMaxSearchDist + kCoarseStep && fine_lc < kMaxLaneCount) {
      auto next_lane = ref_lane;
      double next_s = ref_s;
      if (!AdvanceAlongLane(next_lane, next_s, -kFineStep, fine_lc)) {
        break;
      }
      fine_dist += kFineStep;

      double diff = CalcHeadingDiff(lane_waypoint, next_lane, next_s);
      if (diff < best_diff) {
        best_diff = diff;
        best_s = next_s;
        best_lane = next_lane;
        // fine phase goes backward, so arc = fine_start_arc - fine_dist
        best_arc_from_start = fine_start_arc - fine_dist;
      }
      if (diff > prev_diff) {
        AINFO << "break diff: " << diff << "prev_diff: " << prev_diff;
        break;
      }
      prev_diff = diff;
      ref_lane = next_lane;
      ref_s = next_s;
    }
  }

  if (best_diff < init_diff) {
    auto proj = best_lane->GetSmoothPoint(best_s);
    out_lane = best_lane;
    out_point.set_x(proj.x());
    out_point.set_y(proj.y());
    AINFO << "HeadingProjectionFromLane: found, diff=" << best_diff
          << ", s=" << best_s << ", arc=" << best_arc_from_start
          << ", lane=" << best_lane->id().id();

    if (crane_proj_config_.enable_lateral_blend()) {
      // a: ego → nearest_point Euclidean distance
      double ex = lane_waypoint.pose().x() - nearest_point.x();
      double ey = lane_waypoint.pose().y() - nearest_point.y();
      double a = std::sqrt(ex * ex + ey * ey);
      double b = a - crane_proj_config_.lateral_blend_offset();

      // s: arc-length from nearest to heading projection along lane geometry
      double s = best_arc_from_start;

      AINFO << "LateralBlend: a=" << a << " b=" << b << " s=" << s;

      // ratio in [0,1]: 0 means stay at nearest, 1 means full heading
      double ratio =
          (a > kProjectionEpsilon && b > 0.0) ? std::min(b / a, 1.0) : 0.0;

      if (ratio > kProjectionEpsilon) {
        // s1: target arc-length from start_s along real lane geometry
        double s1 = s * ratio;
        auto blend_lane = start_lane;
        double blend_s = start_s;
        int blend_lc = 0;
        if (AdvanceAlongLane(blend_lane, blend_s, s1, blend_lc)) {
          auto blend_proj = blend_lane->GetSmoothPoint(blend_s);
          out_lane = blend_lane;
          out_point.set_x(blend_proj.x());
          out_point.set_y(blend_proj.y());
          AINFO << "LateralBlend: ratio=" << ratio << " s1=" << s1
                << " blend_lane=" << blend_lane->id().id()
                << " blend_s=" << blend_s;
        }
      } else {
        // ego within offset — return nearest projection point
        auto nearest_proj = start_lane->GetSmoothPoint(start_s);
        out_lane = start_lane;
        out_point.set_x(nearest_proj.x());
        out_point.set_y(nearest_proj.y());
        AINFO << "LateralBlend: ratio=0, use nearest projection";
      }
    }

    return true;
  }

  AINFO << "HeadingProjectionFromLane: no better projection found";
  return false;
}

bool RoutingMath::GetCraneProjectionLaneInfor(LaneWaypoint& lane_waypoint) {
  // Step 1: Get nearest projection (always required as base fallback)
  std::shared_ptr<const hdmap::LaneInfo> nearest_lane;
  PointENU nearest_point;
  bool has_nearest = GetNearestProjection(lane_waypoint, nearest_lane,
                                          nearest_point);

  if (!has_nearest) {
    AERROR << "------> unable to calculate the CraneProjectionLaneIn !";
    double nearest_s, nearest_l;
    std::shared_ptr<const hdmap::LaneInfo> lane;
    if (hdmap_->GetNearestLane(lane_waypoint.pose(), &lane, &nearest_s,
                               &nearest_l) < 0) {
      AERROR << "GetNearestLane failed";
      return false;
    }
    double distance_temp = 0.0;
    common::math::Vec2d point;
    point.set_x(lane_waypoint.pose().x());
    point.set_y(lane_waypoint.pose().y());
    const auto& neast_point = lane->GetNearestPoint(point, &distance_temp);
    lane_waypoint.mutable_pose()->set_x(neast_point.x());
    lane_waypoint.mutable_pose()->set_y(neast_point.y());
    lane_waypoint.set_s(nearest_s);
    lane_waypoint.set_id(lane->id().id());
    return true;
  }

  const auto mode = crane_proj_config_.mode();

  // NEAREST_ONLY: skip heading computation entirely
  if (mode == CraneProjectionMode::NEAREST_ONLY) {
    AINFO << "CraneProjection: NEAREST_ONLY";
    GetWaypointInfoByPointProjection(nearest_lane, nearest_point, lane_waypoint);
    return true;
  }

  // Step 2: Compute heading projection from nearest lane
  double proj_s = 0.0;
  double proj_l = 0.0;
  common::math::Vec2d proj_vec(nearest_point.x(), nearest_point.y());
  nearest_lane->GetProjection(proj_vec, &proj_s, &proj_l);

  std::shared_ptr<const hdmap::LaneInfo> heading_lane;
  PointENU heading_point;
  bool has_heading = GetHeadingProjectionFromLane(
      lane_waypoint, nearest_lane, proj_s, nearest_point,
      heading_lane, heading_point);

  if (!has_heading) {
    AINFO << "CraneProjection: heading unavailable, fallback to nearest";
    GetWaypointInfoByPointProjection(nearest_lane, nearest_point, lane_waypoint);
    return true;
  }

  double dx = nearest_point.x() - heading_point.x();
  double dy = nearest_point.y() - heading_point.y();
  double dist = std::sqrt(dx * dx + dy * dy);
  AINFO << "CraneProjection: mode=" << mode << " two-projection dist=" << dist;
  if (mode == CraneProjectionMode::HEADING_WITH_NEAREST) {
      GetWaypointInfoByPointProjection(heading_lane, heading_point,
                                       lane_waypoint);
      return true;
  }
  if (mode == CraneProjectionMode::BLEND_BOTH) {
    const double threshold =
        crane_proj_config_.heading_nearest_distance_threshold();
    if (dist > threshold) {
      AINFO << "dist > " << threshold << "m, use nearest projection";
      GetWaypointInfoByPointProjection(nearest_lane, nearest_point,
                                       lane_waypoint);
    } else {
      AINFO << "dist <= " << threshold << "m, use heading projection";
      GetWaypointInfoByPointProjection(heading_lane, heading_point,
                                       lane_waypoint);
    }
    return true;
  }

  return false;
}

bool RoutingMath::GetParkingID(const PointENU& parking_point,
                               std::string* parking_space_id) {
  // search current parking space id associated with parking point.
  constexpr double kDistance = 0.01;  // meter
  std::vector<ParkingSpaceInfoConstPtr> parking_spaces;
  if (0 ==
      hdmap_->GetParkingSpaces(parking_point, kDistance, &parking_spaces)) {
    *parking_space_id = parking_spaces.front()->id().id();
    return true;
  }
  return false;
}

std::vector<common::PointENU> RoutingMath::GetSparsePointOfLane(
    const std::shared_ptr<const hdmap::LaneInfo> lane,
    const SparseType& sparse_type) {
  std::vector<common::PointENU> lane_points;
  // double accumulate_s = 0.0, lateral = 0.0;
  double target_lane_length = lane->total_length();
  if (S_ALL == sparse_type) {
    lane_points.emplace_back(lane->GetSmoothPoint(0.0));
    lane_points.emplace_back(lane->GetSmoothPoint(0.5 * target_lane_length));
    lane_points.emplace_back(lane->GetSmoothPoint(target_lane_length));
  } else if (S_FRONT == sparse_type) {
    lane_points.emplace_back(lane->GetSmoothPoint(0.0));
  } else if (S_BACK == sparse_type) {
    lane_points.emplace_back(lane->GetSmoothPoint(target_lane_length));
  } else if (S_CENTER_BACK == sparse_type) {
    lane_points.emplace_back(lane->GetSmoothPoint(0.5 * target_lane_length));
    lane_points.emplace_back(lane->GetSmoothPoint(0.5 * target_lane_length));
  } else if (S_FRONT_CENTER == sparse_type) {
    lane_points.emplace_back(lane->GetSmoothPoint(0.0));
    lane_points.emplace_back(lane->GetSmoothPoint(0.5 * target_lane_length));
  } else if (S_CENTER == sparse_type) {
    lane_points.emplace_back(lane->GetSmoothPoint(0.5 * target_lane_length));
  } else if (S_FRONT_BACK == sparse_type) {
    lane_points.emplace_back(lane->GetSmoothPoint(0.0));
    lane_points.emplace_back(lane->GetSmoothPoint(target_lane_length));
  } else {
    AERROR << "No suitable sparse_type found";
  }
  // if (lane->GetProjection(point_vec, &accumulate_s, &lateral)) {
  //   if (lateral < FLAGS_adc_lane_range && accumulate_s > 0.5 * target_lane_length)
  //   {
  //     return point;
  //   }
  // }
  return lane_points;
}

std::vector<std::string> RoutingMath::GetBlacklistedLanes(
    const std::string_view blacklist_id, bool is_only_add_reverse) {
  std::vector<std::string> id_vec;
  auto temp_fixed_points = GetFixedPoints(blacklist_id);
  for (const auto& fixed_black_point : temp_fixed_points) {
    std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
    hdmap_->GetLanes(fixed_black_point, FLAGS_adc_black_lane_range, &lanes);
    for (const auto& lane_id : lanes) {
      if (is_only_add_reverse) {
        if (lane_id->lane().is_reverse_road()) {
          id_vec.emplace_back(lane_id->id().id());
        }
      } else {
        id_vec.emplace_back(lane_id->id().id());
      }
    }
  }
  return id_vec;
}

std::vector<std::string> RoutingMath::GetBlacklistedLanes(
    const common::PointENU& near_point, const double& min_point_distance,
    const std::string_view blacklist_id, bool is_only_add_reverse) {
  std::vector<std::string> id_vec;
  auto temp_fixed_points = GetFixedPoints(blacklist_id);
  for (const auto& fixed_black_point : temp_fixed_points) {
    if (IsFixedPointClose(fixed_black_point, near_point, min_point_distance)) {
      continue;
    }
    std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
    hdmap_->GetLanes(fixed_black_point, FLAGS_adc_black_lane_range, &lanes);
    for (const auto& lane_id : lanes) {
      if (is_only_add_reverse) {
        if (lane_id->lane().is_reverse_road()) {
          id_vec.emplace_back(lane_id->id().id());
        }
      } else {
        id_vec.emplace_back(lane_id->id().id());
      }
    }
  }
  return id_vec;
}

bool RoutingMath::MoveLaneWaypointAlongLane(const double& move_heading,
                                            const double move_distance,
                                            LaneWaypoint& adc_waypoint) {
  std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
  hdmap_->GetLanes(adc_waypoint.pose(), FLAGS_adc_lane_range, &lanes);
  if (!lanes.empty()) {
    size_t valuable_iter = SIZE_MAX;
    double valuable_diff_heading = 0;
    LaneWaypoint lane_waypoint(adc_waypoint);
    for (size_t j = 0; j < lanes.size(); ++j) {
      double s = 0.0;
      if (!IsLaneUsable(lanes[j], adc_waypoint.pose(), s)) {
        continue;
      }
      double current_diff_heading = std::fabs(
          common::math::NormalizeAngle(lanes[j]->Heading(s) - move_heading));
      if ((current_diff_heading > FLAGS_min_heading_diff) &&
          (current_diff_heading < (M_PI - FLAGS_min_heading_diff))) {
        continue;
      }
      AINFO << "current_diff_heading:" << current_diff_heading
            << ",move_heading: " << move_heading
            << ",lanes[j]->Heading(s):" << lanes[j]->Heading(s)
            << ",id:" << lanes[j]->id().id();
      // lane_waypoint.mutable_pose()->set_x(point.x());
      // lane_waypoint.mutable_pose()->set_y(point.y());
      lane_waypoint.set_id(lanes[j]->id().id());
      lane_waypoint.set_s(s);
      if (century::hdmap::Lane::NO_TURN == lanes[j]->lane().turn() &&
          current_diff_heading < FLAGS_adc_lane_heading_diff) {
        valuable_iter = j;
        valuable_diff_heading = current_diff_heading;
        break;
      }
      valuable_iter = j;
      valuable_diff_heading = current_diff_heading;
    }
    if (valuable_iter != SIZE_MAX) {
      if (valuable_diff_heading < FLAGS_min_heading_diff) {
        GetWaypointAlongSuccessorIdByDistance(lane_waypoint, adc_waypoint,
                                              move_distance);
      } else {
        GetWaypointAlongPredecessorIdByDistance(lane_waypoint, adc_waypoint,
                                                move_distance);
      }
      return true;
    }
  }
  return false;
}

PointENU RoutingMath::ProjectPointToLine(
    const PointENU& p, const std::string_view fix_point_area) {
  auto temp_fixed_points = GetFixedPoints(fix_point_area);
  auto b = temp_fixed_points.front();
  auto a = temp_fixed_points.back();
  double apx = p.x() - a.x();
  double apy = p.y() - a.y();
  double abx = b.x() - a.x();
  double aby = b.y() - a.y();
  double ab2 = abx * abx + aby * aby;
  if (common::util::IsZero(ab2)) {
    return a;
  }
  double t = (apx * abx + apy * aby) / ab2;
  PointENU pt;
  pt.set_x(a.x() + t * abx);
  pt.set_y(a.y() + t * aby);
  return pt;
}

PointENU RoutingMath::ProjectPointToLine(const PointENU& p, const PointENU& b,
                                         const PointENU& a) {
  double apx = p.x() - a.x();
  double apy = p.y() - a.y();
  double abx = b.x() - a.x();
  double aby = b.y() - a.y();
  double ab2 = abx * abx + aby * aby;
  if (common::util::IsZero(ab2)) {
    return a;
  }
  double t = (apx * abx + apy * aby) / ab2;
  PointENU pt;
  pt.set_x(a.x() + t * abx);
  pt.set_y(a.y() + t * aby);
  return pt;
}

double RoutingMath::PointToLineDistance(const PointENU& p,
                                        const PointENU& lineStart,
                                        const PointENU& lineEnd) {
  double A = lineEnd.y() - lineStart.y();
  double B = lineStart.x() - lineEnd.x();
  double C = lineEnd.x() * lineStart.y() - lineStart.x() * lineEnd.y();
  return std::abs(A * p.x() + B * p.y() + C) / std::sqrt(A * A + B * B);
}

PointENU RoutingMath::MoveAlongLine(const PointENU& proj,
                                    const std::string_view fix_point_area,
                                    const double distance, const bool forward) {
  auto temp_fixed_points = GetFixedPoints(fix_point_area);
  auto b = temp_fixed_points.front();
  auto a = temp_fixed_points.back();
  double dx = b.x() - a.x();
  double dy = b.y() - a.y();
  double lineLength = std::sqrt(dx * dx + dy * dy);
  PointENU pt;
  if (common::util::IsZero(lineLength)) {
    pt.set_x(proj.x());
    pt.set_y(proj.y());
    return pt;
  }
  double unitDx = dx / lineLength;
  double unitDy = dy / lineLength;
  double sign = forward ? 1.0 : -1.0;
  pt.set_x(proj.x() + sign * distance * unitDx);
  pt.set_y(proj.y() + sign * distance * unitDy);
  return pt;
}

bool RoutingMath::SearchRightNeighborLaneInfo(const hdmap::Id& target_lane_id,
                                              LaneWaypoint& lane_waypoint) {
  AINFO << "== SearchRightNeighborLaneInfo ==";
  auto target_lane = hdmap_->GetLaneById(target_lane_id);
  PointENU neast_point =
      GetLaneNearestPointByIdAndPoint(target_lane_id, lane_waypoint);
  while (target_lane->lane().right_neighbor_forward_lane_id().size() > 0) {
    AINFO << "Get right forward neighbor lane id:"
          << target_lane->lane().right_neighbor_forward_lane_id(0).id();
    if (!GetLaneNearestPointByIdAndPoint(
            target_lane->lane().right_neighbor_forward_lane_id(0),
            lane_waypoint, target_lane, neast_point)) {
      return false;
    }
  }
  GetWaypointInfoByPointProjection(target_lane, neast_point, lane_waypoint);
  return true;
}

bool RoutingMath::SearchLeftNeighborLaneInfo(const hdmap::Id& target_lane_id,
                                             LaneWaypoint& lane_waypoint) {
  AINFO << "== SearchLeftNeighborLaneInfo ==";
  auto target_lane = hdmap_->GetLaneById(target_lane_id);
  PointENU neast_point =
      GetLaneNearestPointByIdAndPoint(target_lane_id, lane_waypoint);
  // Continue to check if there is an adjacent lane on the left side
  while (target_lane->lane().left_neighbor_forward_lane_id().size() > 0) {
    AINFO << "Get left forward neighbor lane id:"
          << target_lane->lane().left_neighbor_forward_lane_id(0).id();
    if (!GetLaneNearestPointByIdAndPoint(
            target_lane->lane().left_neighbor_forward_lane_id(0), lane_waypoint,
            target_lane, neast_point)) {
      return false;
    }
  }
  GetWaypointInfoByPointProjection(target_lane, neast_point, lane_waypoint);
  return true;
}

bool RoutingMath::SearchNeighborLaneInfo(
    const std::unordered_map<SearchDirection, std::vector<hdmap::Id>>&
        target_lanes,
    LaneWaypoint& lane_waypoint) {
  if (target_lanes.empty()) {
    return false;
  }
  bool search_success = false;
  for (const auto& [direction, lane_ids] : target_lanes) {
    if (lane_ids.empty()) {
      AERROR << "No lane IDs for direction: " << direction;
      continue;
    }
    switch (direction) {
      case SearchDirection::LEFT_DIRECTION:
        for (const auto& lane_id : lane_ids) {
          if (SearchLeftNeighborLaneInfo(lane_id, lane_waypoint)) {
            AINFO << "Successfully found left neighbor lane: " << lane_id.id();
            search_success = true;
            break;
          } else {
            AERROR << "Failed to search left neighbor lane: " << lane_id.id();
          }
        }
        break;
      case SearchDirection::RIGHT_DIRECTION:
        for (const auto& lane_id : lane_ids) {
          if (SearchRightNeighborLaneInfo(lane_id, lane_waypoint)) {
            AINFO << "Successfully found right neighbor lane: " << lane_id.id();
            search_success = true;
            break;
          } else {
            AERROR << "Failed to search right neighbor lane: " << lane_id.id();
          }
        }
        break;
      case SearchDirection::FORWARD_DIRECTION:
        break;
      default:
        AERROR << "Unknown search direction: " << direction;
        break;
    }
    if (search_success) {
      break;
    }
  }
  return search_success;
}

void RoutingMath::SearchSparsePointAndReserveTypeBySegments(
    const RoadSegment& road_passage, const int& road_iter,
    const bool end_road_iter, std::vector<common::PointENU>& road_points,
    bool& is_is_reverse_type) {
  static bool last_segment_straight = false;
  int passage_iter = 0, segment_iter = 0;
  routing::SparseType sparse_type = S_ALL;
  for (const auto& lane_segment : road_passage.passage()) {
    bool is_mult_passage = (road_passage.passage_size() > 1) ? true : false;
    if (routing::FORWARD == lane_segment.change_lane_type()) {
      passage_iter++;
      segment_iter = 0;
      for (const auto& segment : lane_segment.segment()) {
        segment_iter++;
        auto target_lane = hdmap_->GetLaneById(hdmap::MakeMapId(segment.id()));
        auto segment_straight =
            (century::hdmap::Lane::NO_TURN == target_lane->lane().turn())
                ? true
                : false;
        auto is_end_lane =
            (end_road_iter && road_passage.passage_size() == passage_iter &&
             lane_segment.segment_size() == segment_iter)
                ? true
                : false;
        if (1 == road_iter && 1 == passage_iter && 1 == segment_iter) {
          if (is_end_lane) {
            if (is_mult_passage) {
              sparse_type = S_FRONT;
            }
          } else {
            if (segment_straight) {
              if (is_mult_passage) {
                sparse_type = S_FRONT_BACK;
              } else {
                sparse_type = S_BACK;
              }
            } else {
              if (is_mult_passage) {
                sparse_type = S_ALL;
              } else {
                sparse_type = S_CENTER_BACK;
              }
            }
          }
        } else if (is_mult_passage && 1 == segment_iter) {
          if (is_end_lane) {
            sparse_type = S_FRONT;
          } else {
            if (segment_straight) {
              sparse_type = S_FRONT_BACK;
            } else {
              sparse_type = S_ALL;
            }
          }
        } else if (is_end_lane) {
          return;
        } else if (segment_straight) {
          if (last_segment_straight) {
            road_points.pop_back();
          }
          sparse_type = S_BACK;
        } else if (!segment_straight) {
          sparse_type = S_CENTER_BACK;
        } else {
          AERROR << "===================sparse_point==========================="
                    "====";
        }
        auto sparse_points = GetSparsePointOfLane(target_lane, sparse_type);
        if (target_lane->lane().is_reverse_road()) {
          is_is_reverse_type = true;
        }
        road_points.insert(road_points.end(), sparse_points.begin(),
                           sparse_points.end());
        last_segment_straight = segment_straight;
      }
    }
  }
}

bool RoutingMath::SearchNearLaneWaypoint(
    const common::PointENU& target_point,
    std::vector<LaneWaypoint>& vec_lane_waypoint) {
  std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
  bool have_routing_lane = false;
  LaneWaypoint new_lane_waypoint;
  new_lane_waypoint.mutable_pose()->set_x(target_point.x());
  new_lane_waypoint.mutable_pose()->set_y(target_point.y());
  hdmap_->GetLanes(target_point, FLAGS_adc_lane_range, &lanes);
  for (size_t j = 0; j < lanes.size(); ++j) {
    double s = 0.0;
    if (!IsLaneUsable(lanes[j], target_point, s)) {
      continue;
    }
    have_routing_lane = true;
    new_lane_waypoint.set_s(s);
    new_lane_waypoint.set_id(lanes[j]->id().id());
    vec_lane_waypoint.emplace_back(new_lane_waypoint);
  }
  if (!have_routing_lane) {
    AERROR << "No get adc lane.";
    double nearest_s, nearest_l;
    std::shared_ptr<const hdmap::LaneInfo> lane;
    if (hdmap_->GetNearestLane(target_point, &lane, &nearest_s, &nearest_l) <
        0) {
      AERROR << "get nearest lane failed ! ";
      return false;
    } else {
      new_lane_waypoint.set_s(nearest_s);
      new_lane_waypoint.set_id(lane->id().id());
      vec_lane_waypoint.emplace_back(new_lane_waypoint);
    }
  }
  return true;
}

bool RoutingMath::IsTargetLaneStraight(const std::string& target_lane_id,
                                       const std::string_view fixed_point_id,
                                       double search_radius) {
  auto fixed_points = GetFixedPoints(fixed_point_id);
  if (fixed_points.empty()) {
    AERROR << "No fixed points found for id: " << fixed_point_id;
    return false;
  }
  for (const auto& fixed_point : fixed_points) {
    std::vector<std::shared_ptr<const hdmap::LaneInfo>> nearby_lanes;
    if (hdmap_->GetLanes(fixed_point, search_radius, &nearby_lanes) == 0) {
      for (const auto& lane : nearby_lanes) {
        if (lane->id().id() == target_lane_id) {
          AINFO << "Target lane " << target_lane_id
                << " found near fixed point " << fixed_point_id
                << " at (" << fixed_point.x() << ", " << fixed_point.y() << ")";
          return true;
        }
      }
    }
  }
  return false;
}

}  // namespace routing
}  // namespace century
