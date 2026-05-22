/**
 * @file openspace_common.cc
 * @author
 * @brief
 * @version 0.1
 * @date 2025-11-05
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "modules/planning/open_space/openspace_common/openspace_common.h"

namespace century {
namespace planning {

planning::OpenspaceReason OpenspaceCommon::openspace_reason_ =
    OpenspaceReason::NO_OPENSPACE;

bool OpenspaceCommon::is_reverse_driving_ = false;
bool OpenspaceCommon::is_reverse_routing_ = false;

OpenspaceCommon::OpenspaceCommon() {
  turn_type_ = century::hdmap::Lane::NO_TURN;
  ACHECK(cyber::common::GetProtoFromFile(FLAGS_openspace_config_file,
                                         &common_config_))
      << "Failed to read openspace common config file: "
      << FLAGS_openspace_config_file;
  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
  hdmap_ = century::hdmap::HDMapUtil::BaseMapPtr();
  ACHECK(hdmap_) << "Failed to load map";
}

bool OpenspaceCommon::SelectNeighborTurnTypeLane(
    std::vector<std::string>& neigh_turn_lane_ids) {
  if (neigh_turn_lane_ids.empty()) {
    AERROR << "[Warning]neighbor lane is empty, return false.";
    return false;
  }
  auto new_neigh_lane_ids = neigh_turn_lane_ids;
  neigh_turn_lane_ids.clear();
  for (const auto& id : new_neigh_lane_ids) {
    auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(id));
    if (lane->lane().has_turn() &&
        (hdmap::Lane::LEFT_TURN == lane->lane().turn() ||
         hdmap::Lane::RIGHT_TURN == lane->lane().turn())) {
      neigh_turn_lane_ids.emplace_back(id);
    }
  }
  return true;
}

void OpenspaceCommon::TraverseLaneSuccessorsWithDepth(
    const std::pair<std::string, ReferenceLineInfo::LaneType>&
        start_lane_id_map,
    const int max_depth, const century::hdmap::Lane::LaneTurn target_turn_type,
    std::vector<std::string>* result) {
  std::queue<std::pair<std::string, int>> id_infos_q;
  std::unordered_set<std::string> visited;

  std::string start_lane_id = start_lane_id_map.first;
  id_infos_q.emplace(start_lane_id, 0);
  visited.insert(start_lane_id);

  while (!id_infos_q.empty()) {
    auto [lane_id, depth] = id_infos_q.front();
    id_infos_q.pop();

    // exceed the maximum traversal depth
    if (depth > max_depth) {
      continue;
    }
    result->emplace_back(lane_id);
    const auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(lane_id));
    if (nullptr == lane) {
      continue;
    }

    // if depth == max_depth, do not expand further
    if (depth == max_depth) {
      continue;
    }

    for (const auto& successor : lane->lane().successor_id()) {
      const std::string& next_id = successor.id();
      if (visited.count(next_id)) {
        continue;
      }
      visited.insert(next_id);
      id_infos_q.emplace(next_id, depth + 1);
    }
  }
  return;
}

void OpenspaceCommon::FilterNeighborLanesAccordingToTurnType(
    century::hdmap::Lane::LaneTurn turn_type,
    std::unordered_map<std::string, ReferenceLineInfo::LaneType>&
        neigh_lane_id_map) {
  std::unordered_map<std::string, ReferenceLineInfo::LaneType>::iterator it =
      neigh_lane_id_map.begin();
  if (hdmap::Lane::LEFT_TURN == turn_type) {
    while (it != neigh_lane_id_map.end()) {
      if (ReferenceLineInfo::LaneType::LeftForward == it->second ||
          ReferenceLineInfo::LaneType::LeftReverse == it->second) {
        it = neigh_lane_id_map.erase(it);
      } else {
        ++it;
      }
    }
  } else if (hdmap::Lane::RIGHT_TURN == turn_type) {
    while (it != neigh_lane_id_map.end()) {
      if (ReferenceLineInfo::LaneType::RightForward == it->second ||
          ReferenceLineInfo::LaneType::RightReverse == it->second) {
        it = neigh_lane_id_map.erase(it);
      } else {
        ++it;
      }
    }
  }
  return;
}

bool OpenspaceCommon::FindTurnTypeInTargetLane(
    const std::vector<century::hdmap::Id>& lane_ids,
    century::hdmap::Lane::LaneTurn& type) {
  century::hdmap::Lane::LaneTurn turn_type = century::hdmap::Lane::NO_TURN;
  // get first target lane id turn type
  // filter lane id with turn type
  const auto it_id =
      std::find_if(lane_ids.begin(), lane_ids.end(),
                   [this, &turn_type](const century::hdmap::Id& id) {
                     auto lane = hdmap_->GetLaneById(id);
                     if (!lane) {
                       return false;
                     }
                     if (lane->lane().has_turn()) {
                       const auto current_turn = lane->lane().turn();
                       if (hdmap::Lane::LEFT_TURN == current_turn ||
                           hdmap::Lane::RIGHT_TURN == current_turn) {
                         turn_type = current_turn;
                         return true;
                       }
                     }
                     return false;
                   });
  type = turn_type;

  if (it_id == lane_ids.end()) {
    return false;
  }
  ADEBUG << "find_result_id: " << it_id->id() << ", turn_type: " << turn_type;
  return true;
}

/*
  Filter the lane IDs and their corresponding boundaries based on the direction
  of the turn
*/
bool OpenspaceCommon::SelectNeighborLaneInfos(
    const std::vector<century::hdmap::Id>& lane_ids,
    std::unordered_map<std::string, ReferenceLineInfo::LaneType>&
        neigh_lane_id_map,
    std::vector<century::hdmap::Id>& select_neigh_lane_ids) {
  if (lane_ids.empty()) {
    AERROR << "[input]lane_ids is empty";
    return false;
  }
  if (neigh_lane_id_map.empty()) {
    AERROR << "[input]neigh_lane_id_map is empty";
    return false;
  }

  FindTurnTypeInTargetLane(lane_ids, turn_type_);

  FilterNeighborLanesAccordingToTurnType(turn_type_, neigh_lane_id_map);
  ADEBUG << "turn_type: " << turn_type_
         << ", neigh_lane_id_map.size: " << neigh_lane_id_map.size();
  for (const auto& lane_id : neigh_lane_id_map) {
    ADEBUG << " After filtering based on the turning type of lanes, lane_id: "
           << lane_id.first
           << ", lane_type: " << static_cast<int>(lane_id.second);
  }
  std::vector<std::string> neigh_turn_lane_ids;
  for (const auto& neighbor_lane_id : neigh_lane_id_map) {
    TraverseLaneSuccessorsWithDepth(neighbor_lane_id,
                                    common_config_.max_search_depth(),
                                    turn_type_, &neigh_turn_lane_ids);
  }
  select_neigh_lane_ids.reserve(neigh_turn_lane_ids.size());
  for (const auto& neighbor_lane_id : neigh_turn_lane_ids) {
    select_neigh_lane_ids.emplace_back(hdmap::MakeMapId(neighbor_lane_id));
  }
  return true;
}

bool OpenspaceCommon::GetNeighborLaneBoundaryFromMap(
    const hdmap::Path& nearby_path,
    std::vector<century::hdmap::Id> neighbor_lane_ids,
    const double center_line_s, const double longi_range_start_s,
    const double longi_range_end_s, const double search_radius,
    const double ignore_road_boundary_dis_thre, const double step,
    const Vec2d& origin_point, const double origin_heading,
    std::vector<Vec2d>* left_lane_boundary,
    std::vector<Vec2d>* right_lane_boundary) {
  double start_s = center_line_s - longi_range_start_s;
  double end_s = center_line_s + longi_range_end_s;

  double check_point_s = start_s;
  while (check_point_s <= end_s) {
    const hdmap::MapPathPoint check_point =
        nearby_path.GetSmoothPoint(check_point_s);
    std::vector<hdmap::RoadRoiPtr> road_boundaries;
    std::vector<hdmap::JunctionInfoConstPtr> junctions;
    common::PointENU check_point_xy;
    check_point_xy.set_x(check_point.x());
    check_point_xy.set_y(check_point.y());

    int result = hdmap_->GetRoadBoundaries(check_point_xy, search_radius,
                                           neighbor_lane_ids, &road_boundaries,
                                           &junctions);
    if (0 != result) {
      AERROR << __func__
             << "-- get boundary from map failed, result: " << result;
      return false;
    }
    ADEBUG << "road boundaries size: " << road_boundaries.size();
    if (road_boundaries.empty()) {
      AERROR
          << __func__
          << "[ERROR] road_boundaries size is empty or first element is null!";
      return false;
    }

    const auto& road_roi = *road_boundaries.at(0);
    ProcessBoundaryPoints(road_roi.left_boundary, check_point,
                          ignore_road_boundary_dis_thre, left_lane_boundary);
    ProcessBoundaryPoints(road_roi.right_boundary, check_point,
                          ignore_road_boundary_dis_thre, right_lane_boundary);
    check_point_s += step;
  }
  if (left_lane_boundary->size() < 2 || right_lane_boundary->size() < 2) {
    AERROR << "left_point_size or right_point_size is < 2!";
    return false;
  }

  for (size_t i = 0; i < left_lane_boundary->size(); ++i) {
    auto point = left_lane_boundary->at(i);
    ADEBUG << "left, " << i << "neighbor lane boundary point: [" << point.x()
           << ", " << point.y() << "]";
    OpenspaceUtil::World2Origin(origin_point, origin_heading, point);
  }
  for (size_t i = 0; i < right_lane_boundary->size(); ++i) {
    auto point = right_lane_boundary->at(i);
    ADEBUG << "right, " << i << "neighbor lane boundary point: [" << point.x()
           << ", " << point.y() << "]";
    OpenspaceUtil::World2Origin(origin_point, origin_heading, point);
  }
  return true;
}

bool OpenspaceCommon::UpdateBoundaryWithNeighborLane(
    const century::hdmap::Lane::LaneTurn turn_type,
    const std::vector<century::common::math::Vec2d>& neighbor_left_boundaries,
    const std::vector<century::common::math::Vec2d>& neighbor_right_boundaries,
    std::vector<century::common::math::Vec2d>& left_boundaries,
    std::vector<century::common::math::Vec2d>& right_boundaries) {
  if (neighbor_left_boundaries.size() < 2 ||
      neighbor_right_boundaries.size() < 2) {
    AERROR << "neighbor_left_boundaries or "
           << "neighbor_right_boundaries size is < 2!";
    return false;
  }

  if (century::hdmap::Lane::LEFT_TURN == turn_type) {
    // update right boundary with neighbor lane boundaries
    right_boundaries.clear();
    right_boundaries = std::move(neighbor_right_boundaries);
    ADEBUG << "turn_type: " << static_cast<int>(turn_type)
           << ", update boundary. use neighbor_right_boundaries.";
  } else if (century::hdmap::Lane::RIGHT_TURN == turn_type) {
    // update left boundary with neighbor lane boundaries
    left_boundaries.clear();
    left_boundaries = std::move(neighbor_left_boundaries);
    ADEBUG << "turn_type: " << static_cast<int>(turn_type)
           << ", update boundary. use neighbor_left_boundaries.";
  } else {
    return false;
  }
  return true;
}

bool OpenspaceCommon::ExpandNeighborLaneIdForBoundary(
    const century::common::VehicleStateProvider* const vehicle_state,
    const planning::Frame* frame, const hdmap::HDMap* hd_map,
    const double curr_s, std::vector<century::hdmap::Id>& target_lane_ids,
    std::vector<century::hdmap::Id>& neigh_lane_ids,
    double* const neighbor_lane_width) {
  if (nullptr == frame || frame->reference_line_info().empty()) {
    AERROR << "frame is null or reference_line_info is empty";
    return false;
  }
  std::unordered_map<std::string, ReferenceLineInfo::LaneType>
      neigh_lane_id_map;
  std::vector<century::hdmap::Id> select_neigh_lane_ids;
  double vehicle_lane_s = 0.0;
  double vehicle_lane_l = 0.0;
  hdmap::LaneInfoConstPtr car_lane;
  century::common::PointENU car_pose;
  neigh_lane_ids.clear();
  const auto& reference_line_info = frame->reference_line_info().front();
  car_pose.set_x(vehicle_state->x());
  car_pose.set_y(vehicle_state->y());
  for (const auto& id : reference_line_info.TargetLaneId()) {
    auto target_lane = hd_map->GetLaneById(id);
    target_lane_ids.emplace_back(id);
  }
  hd_map->GetNearestLane(car_pose, &car_lane, &vehicle_lane_s, &vehicle_lane_l);

  hdmap::Id neighbor_lane_id;
  const std::array<ReferenceLineInfo::LaneType, 4> lane_types = {
      ReferenceLineInfo::LaneType::LeftForward,
      ReferenceLineInfo::LaneType::LeftReverse,
      ReferenceLineInfo::LaneType::RightForward,
      ReferenceLineInfo::LaneType::RightReverse};

  for (const auto& lane_type : lane_types) {
    if (reference_line_info.GetNeighborLaneInfo(
            lane_type, curr_s, &neighbor_lane_id, neighbor_lane_width)) {
      neigh_lane_id_map.emplace(
          std::make_pair(neighbor_lane_id.id(), lane_type));
      ADEBUG << "has " << std::to_string(static_cast<int>(lane_type))
             << " neighbor lane, id: " << neighbor_lane_id.id()
             << ", lane_type: " << static_cast<int>(lane_type);
    } else {
      ADEBUG << "no " << std::to_string(static_cast<int>(lane_type))
             << " neighbor lane";
    }
  }

  SelectNeighborLaneInfos(target_lane_ids, neigh_lane_id_map,
                          select_neigh_lane_ids);

  if (select_neigh_lane_ids.empty()) {
    AERROR << "neighbor lane are empty, return false, use target lane id.";
    return false;
  }
  for (const auto& ele : select_neigh_lane_ids) {
    hdmap::Id id_temp;
    id_temp.set_id(ele.id());
    target_lane_ids.emplace_back(id_temp);
    neigh_lane_ids.emplace_back(id_temp);
  }
  return true;
}

bool OpenspaceCommon::isInSpecialArea(
    const century::common::VehicleState& vehicle_state,
    const OpenspaceCommonConfig& common_config) {
  if (IsAreaContainAdc(vehicle_state, common_config, SpecialAreaType::G_AREA)) {
    return true;
  }
  return false;
}

bool OpenspaceCommon::OffsetBoundaryWithTurnType(
    const planning::Frame* const frame,
    const century::common::VehicleState& vehicle_state,
    std::vector<century::common::math::Vec2d>& left_lane_boundary,
    std::vector<century::common::math::Vec2d>& right_lane_boundary,
    const double& offset_config) {
  if (nullptr == frame || frame->reference_line_info().empty()) {
    AERROR << "frame is null or reference_line_info is empty";
    return false;
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  std::vector<century::hdmap::Id> target_lane_ids;
  for (const auto& id : reference_line_info.TargetLaneId()) {
    target_lane_ids.emplace_back(id);
  }

  century::hdmap::Lane::LaneTurn turn_type = century::hdmap::Lane::NO_TURN;
  FindTurnTypeInTargetLane(target_lane_ids, turn_type);
  if (turn_type != century::hdmap::Lane::LEFT_TURN &&
      turn_type != century::hdmap::Lane::RIGHT_TURN) {
    AINFO << "type is not left turn or right turn, type: " << turn_type
           << ", do not need offset.";
    return false;
  }

  std::vector<century::common::math::Vec2d>* boundary_to_offset = nullptr;
  util::OffsetDirection offset_direction = util::OffsetDirection::NONE;
  double offset_conf = offset_config;
  if (century::hdmap::Lane::LEFT_TURN == turn_type) {
    boundary_to_offset = &right_lane_boundary;
    offset_direction = util::OffsetDirection::RIGHT;
  } else {
    boundary_to_offset = &left_lane_boundary;
    offset_direction = util::OffsetDirection::LEFT;
  }
  ADEBUG << "lane_turn_type: " << turn_type
         << ", offset_direction: " << offset_direction
         << ", offset_conf: " << offset_conf;

  if (boundary_to_offset->empty()) {
    AERROR << "Boundary to offset is empty.";
    return false;
  }

  std::vector<century::common::math::Vec2d> offset_boundary;
  OpenspaceUtil::RemoveSamePoints(boundary_to_offset);
  std::vector<double> boundary_headings =
      OpenspaceUtil::CalculateHeadings(*boundary_to_offset);
  offset_boundary.reserve(boundary_to_offset->size());

  for (size_t i = 0; i < boundary_to_offset->size(); ++i) {
    const auto& ele = boundary_to_offset->at(i);
    const auto& heading = boundary_headings.at(i);
    double dx_offset = std::cos(heading);
    double dy_offset = std::sin(heading);
    auto offset_point =
        util::offsetPoint(ele.x(), ele.y(), dx_offset, dy_offset, offset_conf,
                          offset_direction, util::CoordinateSystem::RFU);
    offset_boundary.emplace_back(offset_point.first, offset_point.second);
  }
  *boundary_to_offset = std::move(offset_boundary);
  return true;
}

bool OpenspaceCommon::CalTheta(HybridAStartResult& result) {
  ADEBUG << "OpenspaceCommon::CalTheta";
  if (result.x.empty() || result.y.empty() || result.phi.empty()) {
    AERROR << "x_vec or y_vec or phi_vec is empty";
    return false;
  }
  if (result.x.size() != result.y.size() ||
      result.x.size() != result.phi.size()) {
    AERROR << "x_vec.size() != y_vec.size() or x_vec.size() != phi_vec.size()";
    return false;
  }
  ADEBUG << "result.x.size(): " << result.x.size();
  for (size_t i = 0; i < result.x.size(); ++i) {
    if (i + 1 < result.x.size()) {
      double dx = result.x.at(i + 1) - result.x.at(i);
      double dy = result.y.at(i + 1) - result.y.at(i);
      // this method already consider reverse condition, do not check is reverse
      result.phi.at(i) = std::atan2(dy, dx);
    } else if (result.x.size() > 1) {
      result.phi.at(i) = result.phi.at(i - 1);
    } else {
      result.phi.at(i) = 0.0;
    }
  }
  return true;
}

/*
 * @Description: Based on the agreed interface between planning and control,
 * clarify the final issued theta:
 *  is_backward_traj is true: whatever gear position is D or R, theta is always equal vehicle heading + M_PI; 
 *  is_backward_traj is false: whatever gear position is D or R, theta is always same with vehicle heading;
 * @Param: orin_heading: the origin theta
 *  is_backward_traj:
 *    is reverse routing: true -> is_backward_traj: true
 *    is reverse routing: false -> is_backward_traj: false
 * @Param: is_reverse: is gear R
 * @Return: final theta
 */
double OpenspaceCommon::ConfirmFinalTheta(const double& orin_heading,
                                          const double& is_backward_traj,
                                          const double& is_reverse) {
  double final_theta = orin_heading;
  if (is_backward_traj && !is_reverse) {
    ADEBUG << "is_backward_traj, gear is D, theta reverse!";
    final_theta = OpenspaceUtil::HeadingReversal(final_theta, 0);
  }
  return final_theta;
}

void OpenspaceCommon::CalTheta(const bool is_reverse,
                               planning::DiscretizedTrajectory* trajectory) {
  ADEBUG << "openspace:" << __func__ << ", is_reverse: " << is_reverse;
  size_t trajectory_size = trajectory->size();
  for (size_t i = 0; i < trajectory_size; ++i) {
    if (i + 1 < trajectory_size) {
      double dx = trajectory->at(i + 1).path_point().x() -
                  trajectory->at(i).path_point().x();
      double dy = trajectory->at(i + 1).path_point().y() -
                  trajectory->at(i).path_point().y();
      double theta_yaw = std::atan2(dy, dx);
      // Notices: hybrid a star start point theta is already + 180 when routing
      // reverse driving
      theta_yaw = ConfirmFinalTheta(theta_yaw, is_reverse_routing_, is_reverse);
      trajectory->at(i).mutable_path_point()->set_theta(theta_yaw);
    } else if (trajectory->size() > 1) {
      trajectory->at(i).mutable_path_point()->set_theta(
          trajectory->at(i - 1).path_point().theta());
    } else {
      trajectory->at(i).mutable_path_point()->set_theta(0.0);
    }
  }
  ADEBUG << "is_reverse_routing:" << is_reverse_routing_;
  return;
}

void OpenspaceCommon::CalTrajS(
    std::vector<century::common::TrajectoryPoint>* trajectory,
    const bool is_gear_reverse) {
  if (nullptr == trajectory || trajectory->empty()) {
    AERROR << "trajectory is empty, return.";
    return;
  }

  const double sign = (is_reverse_routing_ == is_gear_reverse) ? 1.0 : -1.0;
  trajectory->front().mutable_path_point()->set_s(0.0);
  for (size_t i = 1; i < trajectory->size(); ++i) {
    auto& curr_point = *trajectory->at(i).mutable_path_point();
    const auto& prev_point = trajectory->at(i - 1).path_point();
    double dx = curr_point.x() - prev_point.x();
    double dy = curr_point.y() - prev_point.y();
    double distance = std::hypot(dx, dy);
    curr_point.set_s(prev_point.s() + sign * distance);
  }
  // debug
  for (size_t i = 0; i < trajectory->size(); ++i) {
    const auto& point = trajectory->at(i).path_point();
    ADEBUG << "CalTrajS, i:" << i << ", x: " << point.x()
                        << ", y: " << point.y() << ", s: " << point.s()
                        << ", theta: " << point.theta() * 180 / M_PI;
  }
  return;
}

bool OpenspaceCommon::IsAreaContainAdc(
    const century::common::VehicleState& vehicle_state,
    const OpenspaceCommonConfig& area_config,
    const SpecialAreaType& target_area) {
  const double center_x_offset = (vehicle_param_.front_edge_to_center() -
                                  vehicle_param_.back_edge_to_center()) *
                                 0.5;
  const double center_y_offset = (vehicle_param_.left_edge_to_center() -
                                  vehicle_param_.right_edge_to_center()) *
                                 0.5;

  common::math::Vec2d ego_center(center_x_offset, center_y_offset);
  ego_center.SelfRotate(vehicle_state.heading());
  ego_center.set_x(ego_center.x() + vehicle_state.x());
  ego_center.set_y(ego_center.y() + vehicle_state.y());

  common::math::Box2d adc_box(ego_center, vehicle_state.heading(),
                              vehicle_param_.length(), vehicle_param_.width());
  const common::math::Polygon2d adc_polygon(adc_box);

  for (const auto& special_area : area_config.special_areas()) {
    if (special_area.area_type() != target_area) {
      continue;
    }
    const auto& polygon_msg = special_area.area_polygon();
    std::vector<common::math::Vec2d> vertices;
    vertices.reserve(polygon_msg.point_size());
    for (const auto& pt : polygon_msg.point()) {
      vertices.emplace_back(pt.x(), pt.y());
    }
    return common::math::Polygon2d(vertices).Contains(adc_polygon);
  }
  return false;
}

bool OpenspaceCommon::IsRoutingReverseDriving(
    const century::planning::LocalView& local_view) {
  bool is_reverse_routing_drive = (nullptr == local_view.chassis)
                                      ? false
                                      : (local_view.chassis->gear_location() ==
                                         canbus::Chassis::GEAR_REVERSE);
  if (local_view.routing != nullptr) {
    // if true,need change localization's heading.
    // 1.directly reverse. 2.T4T5backward request from planning. 3.backward
    // first in J4_east.
    is_reverse_routing_drive =
        local_view.routing->routing_request().task_type() ==
            routing::BACKWARD_ROUTING_FROM_PLANNING_REROUTING ||
        local_view.routing->routing_request().task_type() ==
            routing::BACKWARD_ROUTING_DIRECTLY ||
        local_view.routing->routing_request().task_type() ==
            routing::BACKWARD_ROUTING_NEED_PLANNING_REROUTING ||
        local_view.routing->routing_request().task_type() ==
            routing::TWO_ROUTING_BACK ||
        local_view.routing->routing_request().is_backward();
    is_reverse_routing_ = is_reverse_routing_drive;
  }
  AINFO << "is_reverse_routing_: " << is_reverse_routing_;
  return is_reverse_routing_drive;
}

bool OpenspaceCommon::CheckAdcErrorStates(
    const century::common::VehicleState& vehicle_state,
    const TrajGearPair& traj_info) {
  if (!common_config_.enable_check_error_states()) {
    AERROR << "error state check is disabled, return false.";
    return false;
  }
  const common::math::Vec2d ego_point(vehicle_state.x(), vehicle_state.y());
  const double ego_heading = vehicle_state.heading();
  const auto& trajectory = traj_info.first;
  if (CheckLateralErrorState(ego_point, trajectory)) {
    AERROR << "error, lateral error is too large, check lateral "
              "error state failed.";
    return true;
  }
  if (CheckHeadingErrorState(ego_point, ego_heading, trajectory)) {
    AERROR << "error, heading error is too large, check heading "
              "error state failed.";
    return true;
  }
  return false;
}

bool OpenspaceCommon::CheckLateralErrorState(
    const common::math::Vec2d& ego_pt,
    const century::planning::DiscretizedTrajectory& trajectory) {
  using common::math::Vec2d;
  const int size = static_cast<int>(trajectory.size());
  if (0 == size) {
    return false;
  }

  const auto nearest_index = trajectory.QueryNearestPoint(ego_pt);
  const auto& nearest_pt = trajectory.TrajectoryPointAt(nearest_index);
  double lateral_error = std::numeric_limits<double>::max();

  ADEBUG << "traj_size: " << trajectory.size();
  ADEBUG << "nearest_index: " << nearest_index << ", ego_pt: (" << ego_pt.x()
         << ", " << ego_pt.y() << ")"
         << ", nearest_pt: (" << nearest_pt.path_point().x() << ", "
         << nearest_pt.path_point().y() << ")";
  if (1 == size) {
    lateral_error = ego_pt.DistanceTo(
        Vec2d(nearest_pt.path_point().x(), nearest_pt.path_point().y()));
  } else if (0 == nearest_index) {
    const auto& next_pt = trajectory.TrajectoryPointAt(1);
    common::math::LineSegment2d nearest_line_seg(
        Vec2d(nearest_pt.path_point().x(), nearest_pt.path_point().y()),
        Vec2d(next_pt.path_point().x(), next_pt.path_point().y()));
    lateral_error = nearest_line_seg.DistanceTo(ego_pt);
  } else if (nearest_index == size - 1) {
    const auto& prev_pt = trajectory.TrajectoryPointAt(nearest_index - 1);
    common::math::LineSegment2d nearest_line_seg(
        Vec2d(prev_pt.path_point().x(), prev_pt.path_point().y()),
        Vec2d(nearest_pt.path_point().x(), nearest_pt.path_point().y()));
    lateral_error = nearest_line_seg.DistanceTo(ego_pt);
  } else {
    const auto& prev_pt = trajectory.TrajectoryPointAt(nearest_index - 1);
    const auto& next_pt = trajectory.TrajectoryPointAt(nearest_index + 1);

    common::math::LineSegment2d prev_line_seg(
        Vec2d(prev_pt.path_point().x(), prev_pt.path_point().y()),
        Vec2d(nearest_pt.path_point().x(), nearest_pt.path_point().y()));
    common::math::LineSegment2d next_line_seg(
        Vec2d(nearest_pt.path_point().x(), nearest_pt.path_point().y()),
        Vec2d(next_pt.path_point().x(), next_pt.path_point().y()));
    double dist_prev = prev_line_seg.DistanceTo(ego_pt);
    double dist_next = next_line_seg.DistanceTo(ego_pt);
    lateral_error = std::min(dist_prev, dist_next);
  }
  ADEBUG << "calculate_lateral_error: " << lateral_error;
  if (lateral_error > common_config_.lateral_error_threshold()) {
    AINFO << "lateral_error: " << lateral_error
          << " exceed the lateral_error_threshold: "
          << common_config_.lateral_error_threshold()
          << ", lateral_error check failed.";
    return true;
  }
  return false;
}

bool OpenspaceCommon::CheckHeadingErrorState(
    const common::math::Vec2d& ego_pt, const double& ego_heading,
    const century::planning::DiscretizedTrajectory& trajectory) {
  ADEBUG << __func__;
  if (trajectory.empty()) {
    return false;
  }
  const auto& nearest_index = trajectory.QueryNearestPoint(ego_pt);
  const auto& nearest_pt = trajectory.TrajectoryPointAt(nearest_index);
  const double target_theta = nearest_pt.path_point().theta();

  ADEBUG << "is_reverse_routing_: " << is_reverse_routing_
         << ", target_theta: " << target_theta * common::util::RAD2ANG
         << ", ego_heading: " << ego_heading * common::util::RAD2ANG;

  /*
    It should be noted that there is no need to check is_reverse_routing_ here,
    as the theta used for searching is already the result of positioning + 180
    degrees
  */
  double heading_diff = target_theta - ego_heading;
  double normalize_heading_diff =
      common::math::NormalizeAngle(target_theta - ego_heading);
  AINFO << "calculate_heading_diff: " << heading_diff * common::util::RAD2ANG
        << ", calculate_normalize_heading_diff: "
        << normalize_heading_diff * common::util::RAD2ANG;
  if (std::fabs(normalize_heading_diff) >=
      common_config_.heading_angle_error_threshold() * common::util::ANG2RAD) {
    AINFO << "normalize_heading_angle_error: "
          << normalize_heading_diff * common::util::RAD2ANG
          << " (degree), exceed the heading_error_threshold: "
          << common_config_.heading_angle_error_threshold()
          << " (deg), heading_error check failed. "
          << "is_reverse: " << (is_reverse_routing_ ? "true" : "false");
    return true;
  }
  return false;
}

}  // namespace planning
}  // namespace century