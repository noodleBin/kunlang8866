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

/*
 * @file hybrid_a_star.cc
 */
#include "modules/planning/open_space/coarse_trajectory_generator/hybrid_a_star.h"

#include <chrono>
#include <fstream>
#include <limits>

#include "modules/common/math/linear_interpolation.h"
#include "modules/planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"
namespace century {
namespace planning {

// constructor
HybridAStar::HybridAStar(const PlannerOpenSpaceConfig& open_space_conf) {
  // configure
  planner_open_space_config_.CopyFrom(open_space_conf);
  // reed-sheep curve generator
  reed_shepp_generator_ =
      std::make_unique<ReedShepp>(vehicle_param_, planner_open_space_config_);
  // A* planner
  grid_a_star_heuristic_generator_ =
      std::make_unique<GridSearch>(planner_open_space_config_);
  // search for the number of adjacent grids
  next_node_num_ =
      planner_open_space_config_.warm_start_config().next_node_num();
  CHECK_NE(next_node_num_, 2UL);
  force_forward_init_ =
      planner_open_space_config_.warm_start_config().force_forward_init();
  // kappa contraint ratio
  traj_kappa_contraint_ratio_ = planner_open_space_config_.warm_start_config()
                                    .traj_kappa_contraint_ratio();
  // set max front wheel angle
  SetMaxWheelAngle(traj_kappa_contraint_ratio_);
  step_size_ = planner_open_space_config_.warm_start_config().step_size();
  // map grid resolution
  xy_grid_resolution_ =
      planner_open_space_config_.warm_start_config().xy_grid_resolution();
  // sample period
  delta_t_ = planner_open_space_config_.delta_t();
  traj_forward_penalty_ =
      planner_open_space_config_.warm_start_config().traj_forward_penalty();
  traj_back_penalty_ =
      planner_open_space_config_.warm_start_config().traj_back_penalty();
  traj_gear_switch_penalty_ =
      planner_open_space_config_.warm_start_config().traj_gear_switch_penalty();
  traj_steer_penalty_ =
      planner_open_space_config_.warm_start_config().traj_steer_penalty();
  traj_steer_change_penalty_ = planner_open_space_config_.warm_start_config()
                                   .traj_steer_change_penalty();
  traj_end_heading_error_penalty_ =
      planner_open_space_config_.warm_start_config()
          .traj_end_heading_error_penalty();

  astar_first_long_buffer_ =
      planner_open_space_config_.warm_start_config().astar_first_long_buffer();
  astar_first_lat_buffer_ =
      planner_open_space_config_.warm_start_config().astar_first_lat_buffer();

  search_near_destination_x_threshold_ =
      planner_open_space_config_.warm_start_config()
          .search_near_destination_x_threshold();
  search_near_destination_y_threshold_ =
      planner_open_space_config_.warm_start_config()
          .search_near_destination_y_threshold();
  search_near_destination_theta_threshold_ =
      planner_open_space_config_.warm_start_config()
          .search_near_destination_theta_threshold();
  astar_max_time_first_ =
      planner_open_space_config_.warm_start_config().astar_max_time_first();
  astar_max_time_second_ =
      planner_open_space_config_.warm_start_config().astar_max_time_second();
  goal_dist_cost_ =
      planner_open_space_config_.warm_start_config().goal_dist_cost();
  near_obs_dist_ =
      planner_open_space_config_.warm_start_config().near_obs_dist();
  near_obs_dist_cost_ =
      planner_open_space_config_.warm_start_config().near_obs_dist_cost();
  enable_voronoi_field_ =
      planner_open_space_config_.warm_start_config().enable_voronoi_field();
  voronoi_obs_acting_dis_ =
      planner_open_space_config_.warm_start_config().voronoi_obs_acting_dis();
  voronoi_dist_cost_ =
      planner_open_space_config_.warm_start_config().voronoi_dist_cost();

  // Added for Hybrid a star Collision detection safety distance
  hybrid_a_star_node_radius_ = planner_open_space_config_.warm_start_config()
                                   .hybrid_a_star_node_radius();
  hybrid_a_star_bounds_radius_ = planner_open_space_config_.warm_start_config()
                                     .hybrid_a_star_bounds_radius();
  is_hybrid_a_star_bounds_check_ =
      planner_open_space_config_.warm_start_config()
          .is_hybrid_a_star_bounds_check();

  is_hybrid_debug_ =
      planner_open_space_config_.warm_start_config().is_hybrid_debug();
  heuristic_type_ =
      planner_open_space_config_.warm_start_config().heuristic_type();
  rs_num_peroid_ =
      planner_open_space_config_.warm_start_config().rs_num_peroid();
  is_hybrid_update_cost_ =
      planner_open_space_config_.warm_start_config().is_hybrid_update_cost();

  grid_a_star_xy_resolution_ =
      open_space_conf.warm_start_config().grid_a_star_xy_resolution();
  grid_a_star_node_radius_ =
      open_space_conf.warm_start_config().grid_a_star_node_radius();
  // Convert coordinates to grid coordinates in advance
  is_convert_to_grid_coordinates_ =
      open_space_conf.warm_start_config().is_convert_to_grid_coordinates();
  is_xy_bounds_check_ =
      open_space_conf.warm_start_config().is_xy_bounds_check();
  grid_a_star_bounds_radius_ =
      open_space_conf.warm_start_config().grid_a_star_bounds_radius();

  extra_length_ =
      planner_open_space_config_.warm_start_config().astar_first_long_buffer();
  extra_width_ =
      planner_open_space_config_.warm_start_config().astar_first_lat_buffer();

  hybrid_use_rs_dis_ =
      planner_open_space_config_.warm_start_config().hybrid_use_rs_dis();
}

// set max front wheel angle
void HybridAStar::SetMaxWheelAngle(const double traj_kappa_contraint_ratio) {
  double min_turn_radius = vehicle_param_.min_turn_radius();
  if (common::util::IsFloatEqual(min_turn_radius, 0.0)) {
    AERROR << "the min_turn_radius is zero! check min_turn_radius config!";
    min_turn_radius = kMinTurnRadius;
  }
  double max_front_steer_angle =
      vehicle_param_.steer_ratio() *
      std::atan(vehicle_param_.wheel_base() / 2.0 / min_turn_radius);
  double final_ratio =
      vehicle_param_.steer_ratio() * traj_kappa_contraint_ratio;
  if (common::util::IsFloatEqual(final_ratio, 0.0)) {
    AERROR << "the final_ratio is zero! check steer_ratio and "
              "traj_kappa_contraint_ratio config!";
    final_ratio = kBaseRatio;
  }
  step_size_ /= traj_kappa_contraint_ratio;
  max_front_wheel_angle_ = max_front_steer_angle / final_ratio;
  next_node_num_ = int(max_front_wheel_angle_ /
      planner_open_space_config_.warm_start_config().phi_grid_resolution()) * 4;
  AINFO << "min_turn_radius = " << min_turn_radius;
  AINFO << "max_front_steer_angle = " << max_front_steer_angle;
  AINFO << "final_ratio = " << final_ratio;
  AINFO << "max_steer_angle = " << vehicle_param_.max_steer_angle();
  AINFO << "max_front_wheel_angle_ = " << max_front_wheel_angle_;
  AINFO << "next_node_num = " << next_node_num_;
}

// add kappa contraint
void HybridAStar::SetKappaContraintConfig(
    const KappaContraintRatioConfig& kappa_config) {
  traj_kappa_contraint_ratio_ = kappa_config.traj_kappa_contraint_ratio();
  SetMaxWheelAngle(traj_kappa_contraint_ratio_);
  astar_max_time_first_ = kappa_config.astar_max_time_first();
  astar_max_time_second_ = kappa_config.astar_max_time_second();
  reed_shepp_generator_->SetKappaContraintConfig(
      kappa_config.traj_kappa_contraint_ratio());
}

// add default kappa contraint
void HybridAStar::ResetDefaultKappaContraint() {
  traj_kappa_contraint_ratio_ = planner_open_space_config_.warm_start_config()
                                    .traj_kappa_contraint_ratio();
  SetMaxWheelAngle(traj_kappa_contraint_ratio_);
  astar_max_time_first_ =
      planner_open_space_config_.warm_start_config().astar_max_time_first();
  astar_max_time_second_ =
      planner_open_space_config_.warm_start_config().astar_max_time_second();
  reed_shepp_generator_->ResetDefaultKappaContraint();
}

// the distance btween current_node and end_node
double HybridAStar::CalCurrentNodeToEndNodeDis(
    const std::shared_ptr<Node3d> current_node) {
  return std::hypot((end_node_->GetX() - current_node->GetX()),
                    (end_node_->GetY() - current_node->GetY()));
}

// expand node using reedshepp curve
bool HybridAStar::AnalyticExpansion(const double& astar_start_time,
                                    const size_t force_forward_init,
                                    const std::shared_ptr<Node3d> current_node,
                                    const bool use_pure_astar,
                                    const hdmap::Path* nearby_path) {
  // distance to end node, hybrid_use_rs_dis_: 5.0m
  double dis = CalCurrentNodeToEndNodeDis(current_node);
  if (!use_pure_astar &&
      (dis <= hybrid_use_rs_dis_ ||
       Clock::NowInSeconds() - astar_start_time > kWillOverTime)) {
    std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
        std::make_shared<ReedSheppPath>();

    if (planner_open_space_config_.warm_start_config().use_bestest_rs()) {
      if (!reed_shepp_generator_->BestestRSP(current_node, end_node_,
                                             reeds_shepp_to_check)) {
        AERROR << "BestestRSP failed";
        return false;
      }
    } else {
      if (!reed_shepp_generator_->ShortestRSP(current_node, end_node_,
                                              reeds_shepp_to_check)) {
        AERROR << "ShortestRSP failed";
        return false;
      }
    }

    // generate node3d and check validity
    if (!RSPCheck(reeds_shepp_to_check)) {
      return false;
    }
    if (force_forward_init > 0 && !InitForwardCheck(reeds_shepp_to_check)) {
      return false;
    }

    // load the whole RSP as nodes and add to the close set
    final_node_ = LoadRSPinCS(reeds_shepp_to_check, current_node);
    if (!IsNearGoal2(final_node_, true, nearby_path)) {
      return false;
    }
    ADEBUG << "current_node node index id: " << current_node->GetIndex();
    ADEBUG << "current_node node (x,y,phi): [" << current_node->GetX() << ","
           << current_node->GetY() << "," << current_node->GetPhi() << "]";
    ADEBUG << "current_node node total cost F: " << current_node->GetCost()
           << " G:" << current_node->GetTrajCost()
           << " H:" << current_node->GetHeuCost();
    if (is_hybrid_debug_) {
      century::planning_internal::OpenSpaceDebug open_space_debug;
      open_space_debug.set_is_first_hybrid_debug(false);
      open_space_debug.mutable_hybrid_rs_debug()->set_is_rs_succeed(true);

      century::planning_internal::HybridPointDebug hybrid_rs_start_point;
      hybrid_rs_start_point.set_x(current_node->GetX());
      hybrid_rs_start_point.set_y(current_node->GetY());
      hybrid_rs_start_point.set_phi(current_node->GetPhi());

      auto hybrid_rs_start_point_ptr =
          open_space_debug.mutable_hybrid_rs_debug()
              ->mutable_hybrid_rs_start_point();
      hybrid_rs_start_point_ptr->CopyFrom(hybrid_rs_start_point);

      // Send many times
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      size_t send_times = 2;
      for (size_t i = 0; i < send_times; ++i) {
        open_space_writer_->Write(open_space_debug);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
  } else {
    // check near goal
    if (IsNearGoal2(current_node, false, nearby_path)) {
      close_set_.emplace(current_node->GetIndex(), current_node);
      ADEBUG << "pure astar plan arrived goal";
      // load the whole RSP as nodes and add to the close set
      final_node_ = current_node;
    } else {
      return false;
    }
  }
  return true;
}

// generate node3d and check validity
bool HybridAStar::RSPCheck(
    const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end) {
  // construct node3d
  std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, planner_open_space_config_));
  // validity check
  return ValidityCheck(node);
}
bool HybridAStar::InitForwardCheck(
    const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end) {
  return !reeds_shepp_to_end->gear.empty() && reeds_shepp_to_end->gear[0];
}

// check if it is near path or end pose
bool HybridAStar::IsNearGoal2(std::shared_ptr<Node3d> node,
                              bool rs_node,
                              const hdmap::Path *nearby_path) {
  // all points in the node
  const auto& traversed_x = node->GetXs();
  const auto& traversed_y = node->GetYs();
  const auto& traversed_phi = node->GetPhis();
  // end pose
  const auto& end_x = end_node_->GetX();
  const auto& end_y = end_node_->GetY();
  const auto& end_phi = end_node_->GetPhi();
  bool find_end = false;         // reach end flag
  if (nullptr == nearby_path || 0 == nearby_path->length()) {   // parking in
    if (rs_node) {
      return IsNearGoal(node);
    }
    // check near end pose
    for (size_t i = 0; i < traversed_x.size(); ++i) {
      if (std::fabs(end_x - traversed_x[i]) <=
              search_near_destination_x_threshold_ &&
          std::fabs(end_y - traversed_y[i]) <=
              search_near_destination_y_threshold_ &&
          (std::fabs(NormalizeAngle(end_phi - traversed_phi[i])) <=
               search_near_destination_theta_threshold_ ||
           std::fabs(NormalizeAngle(-end_phi - traversed_phi[i])) <=
               search_near_destination_theta_threshold_)) {
        std::vector<double> xs(traversed_x.begin(), traversed_x.begin() + i + 1);
        std::vector<double> ys(traversed_y.begin(), traversed_y.begin() + i + 1);
        std::vector<double> phis(traversed_phi.begin(), traversed_phi.begin() + i + 1);
        node->SetTraversed(xs, ys, phis, XYbounds_, planner_open_space_config_);
        find_end = true;
        break;
      }
    }
  } else {                                                      // parking out
    // check near path
    for (size_t i = 0; i < traversed_x.size(); ++i) {
      // origin ---> world
      Vec2d point(traversed_x[i], traversed_y[i]);
      point.SelfRotate(rotate_angle_);
      point += translate_origin_;
      double heading = NormalizeAngle(traversed_phi[i] + rotate_angle_);
      // bounding box
      const auto& bounding_box = Node3d::GetBoundingBoxWithBuffer(
          vehicle_param_, point.x(), point.y(), heading, astar_long_buffer_,
          astar_lat_buffer_);
      // xy ---> sl
      double s = 0.0, l = 0.0;
      nearby_path->GetNearestPoint(point, &s, &l);
      auto smooth_point = nearby_path->GetSmoothPoint(s);
      // on lane
      if (nearby_path->IsOnPath(bounding_box) && std::fabs(NormalizeAngle(
              heading - smooth_point.heading())) < M_PI_2) {
        std::vector<double> xs(traversed_x.begin(), traversed_x.begin() + i + 1);
        std::vector<double> ys(traversed_y.begin(), traversed_y.begin() + i + 1);
        std::vector<double> phis(traversed_phi.begin(), traversed_phi.begin() + i + 1);
        node->SetTraversed(xs, ys, phis, XYbounds_, planner_open_space_config_);
        find_end = true;
        break;
      }
    }
  }
  AINFO << "find_end: " << find_end;
  return find_end;
}

bool HybridAStar::IsNearGoal(std::shared_ptr<Node3d> node) {
  const auto& cur_x = node->GetX();
  const auto& cur_y = node->GetY();
  const auto& cur_phi = node->GetPhi();
  const auto& end_x = end_node_->GetX();
  const auto& end_y = end_node_->GetY();
  const auto& end_phi = end_node_->GetPhi();
  if (std::fabs(end_x - cur_x) > search_near_destination_x_threshold_) {
    return false;
  }
  if (std::fabs(end_y - cur_y) > search_near_destination_y_threshold_) {
    return false;
  }
  if (std::fabs(end_phi - cur_phi) > search_near_destination_theta_threshold_ ||
      std::fabs(NormalizeAngle(-end_phi - cur_phi)) >
          search_near_destination_theta_threshold_) {
    return false;
  }
  return true;
}

// check whether the node is valid
bool HybridAStar::ValidityCheck(std::shared_ptr<Node3d> node,
                                bool is_first_node) {
  CHECK_NOTNULL(node);
  CHECK_GT(node->GetStepSize(), 0U);

  // points size contained in the node
  size_t node_step_size = node->GetStepSize();
  // points contained in the node
  const auto& traversed_x = node->GetXs();
  const auto& traversed_y = node->GetYs();
  const auto& traversed_phi = node->GetPhis();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;
  if (1 == node_step_size) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

  for (size_t i = check_start_index; i < node_step_size; ++i) {
    // if out off XYbounds_, return false
    if (traversed_x[i] > XYbounds_[1] || traversed_x[i] < XYbounds_[0] ||
        traversed_y[i] > XYbounds_[3] || traversed_y[i] < XYbounds_[2]) {
      AERROR << "out off XYbounds_, return false.";
      return false;
    }
  }

  if (obstacles_linesegments_vec_.empty()) {
    return true;
  }
  for (size_t i = check_start_index; i < node_step_size; ++i) {
    double max_buff_adjustment_distance =
        planner_open_space_config_.warm_start_config()
            .max_buff_adjustment_distance();
    double min_buff_adjustment_distance =
        planner_open_space_config_.warm_start_config()
            .min_buff_adjustment_distance();
    double min_astar_long_buffer =
        planner_open_space_config_.warm_start_config().min_astar_long_buffer();
    double min_astar_lat_buffer =
        planner_open_space_config_.warm_start_config().min_astar_lat_buffer();
    if (start_collision_flag_ && is_first_node) {
      double traversed2start_dis =
          std::hypot((traversed_x[i] - start_node_->GetX()),
                     (traversed_y[i] - start_node_->GetY()));
      if (traversed2start_dis < min_buff_adjustment_distance) {
        astar_long_buffer_ = min_astar_long_buffer;
        astar_lat_buffer_ = min_astar_lat_buffer;

      } else if (traversed2start_dis < max_buff_adjustment_distance) {
        astar_long_buffer_ = common::math::lerp(
            min_astar_long_buffer, 0, astar_first_long_buffer_,
            max_buff_adjustment_distance, traversed2start_dis);
        astar_lat_buffer_ = common::math::lerp(
            min_astar_lat_buffer, 0, astar_first_long_buffer_,
            max_buff_adjustment_distance, traversed2start_dis);
      }
    } else {
      astar_long_buffer_ = astar_first_long_buffer_;
      astar_lat_buffer_ = astar_first_lat_buffer_;
    }

    // bounding box
    const auto& bounding_box = Node3d::GetBoundingBoxWithBuffer(
        vehicle_param_, traversed_x[i], traversed_y[i], traversed_phi[i],
        astar_long_buffer_, astar_lat_buffer_);
    // collision with obstacles_linesegments_vec_
    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
      for (const common::math::LineSegment2d& linesegment :
           obstacle_linesegments) {
        if (bounding_box.HasOverlap(linesegment)) {
          Vec2d point_start(linesegment.start().x(), linesegment.start().y());
          point_start.SelfRotate(rotate_angle_);
          point_start += translate_origin_;
          Vec2d point_end(linesegment.end().x(), linesegment.end().y());
          point_end.SelfRotate(rotate_angle_);
          point_end += translate_origin_;
          Vec2d coll_node(traversed_x[i], traversed_y[i]);
          coll_node.SelfRotate(rotate_angle_);
          coll_node += translate_origin_;
          ADEBUG << "collision_node: [" << coll_node.x() << ", "
                 << coll_node.y() << "]";
          ADEBUG << "collision start at x: " << point_start.x();
          ADEBUG << "collision start at y: " << point_start.y();
          ADEBUG << "collision end at x: " << point_end.x();
          ADEBUG << "collision end at y: " << point_end.y();
          return false;
        }
      }
    }
  }
  return true;
}

std::shared_ptr<Node3d> HybridAStar::LoadRSPinCS(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
    std::shared_ptr<Node3d> current_node) {
  // construct node3d
  std::shared_ptr<Node3d> end_node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, planner_open_space_config_));
  // set previous node
  end_node->SetPre(current_node);
  // insert into close list
  close_set_.emplace(end_node->GetIndex(), end_node);
  return end_node;
}

bool HybridAStar::IsStartNode(std::shared_ptr<Node3d> node) {
  return (nullptr == node->GetPreNode());
}

double HybridAStar::CalculateDirectionalDistance(std::shared_ptr<Node3d> node) {
  // start node allow change direction
  if (nullptr == node->GetPreNode()) {
    return 0.0;
  }

  double total_distance = 0.0;
  bool current_direction = node->GetDirec();
  std::shared_ptr<Node3d> current = node;
  // backtrace until direction change or reach start node
  while (current != nullptr) {
    // if start node, stop backtrace
    if (nullptr == current->GetPreNode()) {
      break;
    }
    // check direction is consistent
    if (current->GetDirec() != current_direction) {
      break;
    }
    // accumulate the distance of the current node
    const auto& xs = current->GetXs();
    const auto& ys = current->GetYs();
    if (xs.size() > 1) {
      for (size_t i = 1; i < xs.size(); ++i) {
        total_distance += std::hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]);
      }
    }
    current = current->GetPreNode();
  }
  return total_distance;
}

// generate next node
std::shared_ptr<Node3d> HybridAStar::Next_node_generator(
    std::shared_ptr<Node3d> current_node, size_t next_node_index) {
  double steering = 0.0;
  double traveled_distance = 0.0;

  // check ist allow direction change (forward to backward or backward to forward)
  bool next_direction =
      (next_node_index < static_cast<double>(next_node_num_) * 0.5);
  if (current_node->GetDirec() != next_direction) {
    if (IsStartNode(current_node)) {
      ADEBUG << "Allow direction change at start node";
    } else {
      double current_direction_distance =
          CalculateDirectionalDistance(current_node);
      if (current_direction_distance <
          planner_open_space_config_.warm_start_config()
              .same_direction_force_distance()) {
        return nullptr;
      }
    }
  }

  if (next_node_index < static_cast<double>(next_node_num_) * 0.5) {
    steering = -max_front_wheel_angle_ +
               (2.0 * max_front_wheel_angle_ /
                (static_cast<double>(next_node_num_) * 0.5 - 1)) *
                   static_cast<double>(next_node_index);
    traveled_distance = step_size_;
  } else {
    size_t index = next_node_index - next_node_num_ / 2;
    steering = -max_front_wheel_angle_ +
               (2.0 * max_front_wheel_angle_ /
                (static_cast<double>(next_node_num_) * 0.5 - 1)) *
                   static_cast<double>(index);
    traveled_distance = -step_size_;
  }

  double arc = xy_grid_resolution_;
  std::vector<double> intermediate_x;
  std::vector<double> intermediate_y;
  std::vector<double> intermediate_phi;
  double last_x = current_node->GetX();
  double last_y = current_node->GetY();
  double last_phi = current_node->GetPhi();
  intermediate_x.emplace_back(last_x);
  intermediate_y.emplace_back(last_y);
  intermediate_phi.emplace_back(last_phi);
  for (size_t i = 0; i < arc / step_size_; ++i) {
    const double next_phi = common::math::NormalizeAngle(
        last_phi +
        traveled_distance * std::tan(steering) / vehicle_param_.wheel_base());
    const double next_x = last_x + traveled_distance * std::cos(next_phi);
    const double next_y = last_y + traveled_distance * std::sin(next_phi);

    // out of XY boundary
    if (next_x > XYbounds_[1] || next_x < XYbounds_[0] ||
        next_y > XYbounds_[3] || next_y < XYbounds_[2]) {
      break;
    }
    intermediate_x.emplace_back(next_x);
    intermediate_y.emplace_back(next_y);
    intermediate_phi.emplace_back(next_phi);
    last_x = next_x;
    last_y = next_y;
    last_phi = next_phi;
  }
  if (intermediate_x.size() <= 1) {
    return nullptr;
  }

  std::shared_ptr<Node3d> next_node = std::shared_ptr<Node3d>(
      new Node3d(intermediate_x, intermediate_y, intermediate_phi, XYbounds_,
                 planner_open_space_config_));
  next_node->SetPre(current_node);
  next_node->SetDirec(traveled_distance > 0.0);
  next_node->SetSteer(steering);
  return next_node;
}

// compute next_node f_cost
void HybridAStar::CalculateNodeCost(const double& astar_start_time,
                                    std::shared_ptr<Node3d> current_node,
                                    std::shared_ptr<Node3d> next_node,
                                    const hdmap::Path *nearby_path) {
  // next_node g_cost
  next_node->SetTrajCost(current_node->GetTrajCost() +
                         TrajCost(current_node, next_node));

  // overtime
  double will_overtime_cnt =
      ((Clock::NowInSeconds() - astar_start_time) > kWillOverTime)
          ? ((Clock::NowInSeconds() - astar_start_time) / kWillOverTime)
          : 1.0;

  // distance to start node
  double current2start_dis =
      std::hypot((current_node->GetX() - start_node_->GetX()),
                 (current_node->GetY() - start_node_->GetY()));
  // distance to end node
  double current2end_dis =
      std::hypot((current_node->GetX() - end_node_->GetX()),
                 (current_node->GetY() - end_node_->GetY()));

  // evaluate heuristic cost
  double optimal_path_cost = 0.0;
  double end_cost = 0.0;
  std::shared_ptr<ReedSheppPath> shortest_rs_path =
      std::make_shared<ReedSheppPath>();
  if (nullptr == nearby_path || 0.0 == nearby_path->length()) {
    // to end_node_ cost
    end_cost = (goal_dist_cost_ * will_overtime_cnt) *
                std::hypot((end_node_->GetX() - next_node->GetX()),
                           (end_node_->GetY() - next_node->GetY()));
    if (reed_shepp_generator_->ShortestRSP(current_node, end_node_,
                                           shortest_rs_path)) {
      end_cost = shortest_rs_path->total_length;
    }
    optimal_path_cost += end_cost;
  } else {
    // origin ---> world
    Vec2d point(next_node->GetX(), next_node->GetY());
    point.SelfRotate(rotate_angle_);
    point += translate_origin_;
    double s = 0.0, l = 0.0;
    nearby_path->GetNearestPoint(point, &s, &l);
    auto smooth_point = nearby_path->GetSmoothPoint(s);
    end_cost = (goal_dist_cost_ * will_overtime_cnt) * std::fabs(l);
    optimal_path_cost += end_cost;
    optimal_path_cost += std::fabs(NormalizeAngle(
        smooth_point.heading() - next_node->GetPhi())) * 0.5;
  }
  // front_obs cost
  double obs_cost = current2end_dis > kNearEndDis
                        ? (near_obs_dist_cost_ *
                           std::hypot((end_node_->GetX() - next_node->GetX()),
                                      (end_node_->GetY() - next_node->GetY())) *
                           NearObstacleHeuristic(next_node))
                        : 0.0;
  optimal_path_cost += obs_cost;
  // voronoi cost
  double voronoi_cost = 0.0;
  if (enable_voronoi_field_ && distanceTransform_.size() > 0 &&
      current2start_dis > kFront2Center && current2end_dis > kFront2Center &&
      will_overtime_cnt <= kEnableVoronoiTimeCnt) {
    const ObsPoint map_ld_point =
        ObsPoint(static_cast<int>(XYbounds_[0] * kMeter2Decimeter),
                 static_cast<int>(XYbounds_[2] * kMeter2Decimeter));
    voronoi_cost =
        voronoi_dist_cost_ *
        GetVoronoiValue(static_cast<int>(next_node->GetX() * kMeter2Decimeter),
                        static_cast<int>(next_node->GetY() * kMeter2Decimeter),
                        map_width_, map_height_, map_ld_point) /
        kMeter2Decimeter;
  }
  voronoi_cost = voronoi_cost < end_cost ? voronoi_cost : 0.0;
  optimal_path_cost += voronoi_cost;
  // next node h_cost
  next_node->SetHeuCost(optimal_path_cost);
}

// current_node to next_node g_cost
double HybridAStar::TrajCost(std::shared_ptr<Node3d> current_node,
                             std::shared_ptr<Node3d> next_node) {
  // evaluate cost on the trajectory and add current cost
  double piecewise_cost = 0.0;
  if (next_node->GetDirec()) {
    // dorward
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_forward_penalty_;
  } else {
    if (force_forward_init_ > 0) {
      piecewise_cost = force_forward_init_ * kPenalizeForward;
    }
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_back_penalty_;
  }

  // direction change cost
  if (current_node->GetDirec() != next_node->GetDirec()) {
    piecewise_cost += traj_gear_switch_penalty_;
  }

  double near_end_node_ratio =
      (std::hypot((next_node->GetX() - start_node_->GetX()),
                  (next_node->GetY() - start_node_->GetY())) /
       std::hypot((end_node_->GetX() - start_node_->GetX()),
                  (end_node_->GetY() - start_node_->GetY())));
  piecewise_cost += traj_steer_penalty_ * std::abs(next_node->GetSteer()) *
                    near_end_node_ratio;
  piecewise_cost += traj_steer_change_penalty_ *
                    std::abs(next_node->GetSteer() - current_node->GetSteer()) *
                    near_end_node_ratio;
  piecewise_cost += traj_end_heading_error_penalty_ *
                    std::abs(end_node_->GetPhi() - current_node->GetPhi()) *
                    near_end_node_ratio * near_end_node_ratio;
  return piecewise_cost;
}

double HybridAStar::HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node) {
  return grid_a_star_heuristic_generator_->CheckDpMap(next_node->GetX(),
                                                      next_node->GetY());
}

double HybridAStar::NearObstacleHeuristic(std::shared_ptr<Node3d> node) {
  CHECK_NOTNULL(node);
  CHECK_GT(node->GetStepSize(), 0U);

  size_t node_step_size = node->GetStepSize();
  const auto& traversed_x = node->GetXs();
  const auto& traversed_y = node->GetYs();
  const auto& traversed_phi = node->GetPhis();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;
  if (1 == node_step_size) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

  for (size_t i = check_start_index; i < node_step_size; ++i) {
    // if out off XYbounds_, return false
    if (traversed_x[i] > XYbounds_[1] || traversed_x[i] < XYbounds_[0] ||
        traversed_y[i] > XYbounds_[3] || traversed_y[i] < XYbounds_[2]) {
      ADEBUG << "out off XYbounds";
      return 1.0;
    }
  }
  if (obstacles_linesegments_vec_.empty()) {
    return 0.0;
  }
  double nearest_dist = std::numeric_limits<double>::max();
  for (size_t i = check_start_index; i < node_step_size; ++i) {
    // get scan box
    const auto& scan_box = Node3d::GetObsCostBoxWithBuffer(
        vehicle_param_, traversed_x[i], traversed_y[i], traversed_phi[i],
        extra_length_, extra_width_);
    // collision with obstacles_linesegments_vec_
    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
      for (const common::math::LineSegment2d& linesegment :
           obstacle_linesegments) {
        if (scan_box.DistanceTo(linesegment) < nearest_dist) {
          nearest_dist = scan_box.DistanceTo(linesegment);
        }
      }
    }
  }
  if (nearest_dist > near_obs_dist_) {
    return 0.0;  // %
  } else {
    return 1.0 - (nearest_dist / near_obs_dist_);  // %
  }
  return 0.0;  // %
}

// collect trajectory
bool HybridAStar::GetResult(HybridAStartResult* result, bool flag) {
  std::shared_ptr<Node3d> current_node = final_node_;

  std::vector<double> hybrid_a_x;
  std::vector<double> hybrid_a_y;
  std::vector<double> hybrid_a_phi;
  while (current_node->GetPreNode() != nullptr) {
    std::vector<double> x = current_node->GetXs();
    std::vector<double> y = current_node->GetYs();
    std::vector<double> phi = current_node->GetPhis();
    if (x.empty() || y.empty() || phi.empty()) {
      AERROR << "result size check failed";
      return false;
    }
    if (x.size() != y.size() || x.size() != phi.size()) {
      AERROR << "states sizes are not equal";
      return false;
    }
    std::reverse(x.begin(), x.end());
    std::reverse(y.begin(), y.end());
    std::reverse(phi.begin(), phi.end());
    x.pop_back();
    y.pop_back();
    phi.pop_back();
    hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
    hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
    hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
    current_node = current_node->GetPreNode();
  }
  hybrid_a_x.emplace_back(current_node->GetX());
  hybrid_a_y.emplace_back(current_node->GetY());
  hybrid_a_phi.emplace_back(current_node->GetPhi());
  if (flag) {
    std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
    std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
    std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
  }
  (*result).x = hybrid_a_x;
  (*result).y = hybrid_a_y;
  (*result).phi = hybrid_a_phi;

  // compute v, a, steer
  if (!GetTemporalProfile(result)) {
    AERROR << "GetSpeedProfile from Hybrid Astar path fails";
    return false;
  }

  if (result->x.size() != result->y.size() ||
      result->x.size() != result->phi.size()) {
    AERROR << "state sizes not equal, "
           << "result->x.size(): " << result->x.size() << "result->y.size()"
           << result->y.size() << "result->phi.size()" << result->phi.size();
    return false;
  }
  if (result->a.size() != result->steer.size() ||
      result->x.size() - result->a.size() != 1 ||
      result->x.size() != result->v.size()) {
    AERROR << "control sizes not equal or not right";
    AERROR << " acceleration size: " << result->a.size();
    AERROR << " steer size: " << result->steer.size();
    AERROR << " x size: " << result->x.size() << "result->v.size()"
           << result->v.size();
    return false;
  }

  return true;
}

// compute velocity and acceleration
bool HybridAStar::GenerateSpeedAcceleration(HybridAStartResult* result) {
  // Sanity Check
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    AERROR << "result size check when generating speed and acceleration fail";
    return false;
  }
  size_t x_size = result->x.size();
  double traverled_distance = step_size_;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  if (2 == x_size) {
    x.emplace_back(result->x.front());
    y.emplace_back(result->y.front());
    phi.emplace_back(result->phi.front());
    x.emplace_back((result->x.front() + result->x.back()) * 0.5);
    y.emplace_back((result->y.front() + result->y.back()) * 0.5);
    phi.emplace_back((result->phi.front() + result->phi.back()) * 0.5);
    x.emplace_back(result->x.back());
    y.emplace_back(result->y.back());
    phi.emplace_back(result->phi.back());
    traverled_distance = step_size_ * 0.5;
    result->x = x;
    result->y = y;
    result->phi = phi;
  }
  x_size = result->x.size();

  double target_v =  planner_open_space_config_.distance_approach_config()
                         .max_speed_forward();
  double target_a = planner_open_space_config_.distance_approach_config()
                        .max_acceleration_forward();
  if (!result->gear) {
    target_v = planner_open_space_config_.distance_approach_config()
                   .max_speed_reverse();
    target_a = planner_open_space_config_.distance_approach_config()
                   .max_acceleration_reverse();
  }

  double accumulate_s = 0.0;
  result->v.emplace_back(0.0);
  result->accumulated_s.clear();
  result->accumulated_s.emplace_back(accumulate_s);
  for (size_t i = 1; i < x_size; ++i) {
    accumulate_s = std::hypot(result->x[i] - result->x[i - 1],
                              result->y[i] - result->y[i - 1]);
    result->accumulated_s.emplace_back(accumulate_s);
    double v = sqrt(pow(result->v.back(), 2) + target_a * traverled_distance);
    if (v > target_v) {
      v = target_v;
    }
    if (v * v / (2.0 * target_a) > (x_size - 1 - i) * traverled_distance) {
      v = sqrt(pow(result->v.back(), 2) - 2.0 * target_a * traverled_distance);
    }
    result->v.emplace_back(v);
  }
  result->v.back() = 0.0;

  result->t.emplace_back(0.0);
  for (size_t i = 1; i < x_size; ++i) {
    double v0 = result->v[i - 1];
    double v1 = result->v[i];
    double s = std::fabs((v1 * v1 - v0 * v0) / (2.0 * target_a));
    double t = 0.0;
    if (i < x_size - 1) {
      t = std::fabs((v1 - v0) / target_a) + std::fabs((traverled_distance - s) / v1);
    } else {
      double a = (v1 * v1 - v0 * v0) / (2.0 * traverled_distance);
      t = std::fabs((v1 - v0) / a);
    }
    result->t.emplace_back(t);
  }

  if (!result->gear) {
    for (size_t i = 0; i < x_size; ++i) {
      result->v[i] *= -1.0;
      result->accumulated_s[i] *= -1.0;
    }
  }

  for (size_t i = 1; i < x_size; ++i) {
    result->a.emplace_back((result->v[i] - result->v[i - 1]) / result->t[i]);
  }

  // load steering from phi
  for (size_t i = 0; i + 1 < x_size; ++i) {
    double discrete_steer = (result->phi[i + 1] - result->phi[i]) *
                            (vehicle_param_.wheel_base() / 2.0) / traverled_distance;
    if (result->v[i] > 0.0) {
      discrete_steer = std::atan(discrete_steer);
    } else {
      discrete_steer = std::atan(-discrete_steer);
    }
    result->steer.emplace_back(discrete_steer);
  }

  return true;
}

// segment trajectory
bool HybridAStar::TrajectoryPartition(
    const HybridAStartResult& result,
    std::vector<HybridAStartResult>* partitioned_result) {
  const auto& x = result.x;
  const auto& y = result.y;
  const auto& phi = result.phi;
  if (x.size() != y.size() || x.size() != phi.size()) {
    AERROR << "states sizes are not equal when do trajectory partitioning of "
              "Hybrid A Star result";
    return false;
  }

  size_t horizon = x.size();
  partitioned_result->clear();
  partitioned_result->emplace_back();
  auto* current_traj = &(partitioned_result->back());
  double heading_angle = phi.front();
  const Vec2d init_tracking_vector(x[1] - x[0], y[1] - y[0]);
  double tracking_angle = init_tracking_vector.Angle();
  bool current_gear =
      std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
      (M_PI_2);
  for (size_t i = 0; i < horizon - 1; ++i) {
    heading_angle = phi[i];
    const Vec2d tracking_vector(x[i + 1] - x[i], y[i + 1] - y[i]);
    tracking_angle = tracking_vector.Angle();
    bool gear =
        std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
        (M_PI_2);
    if (gear != current_gear) {
      current_traj->x.emplace_back(x[i]);
      current_traj->y.emplace_back(y[i]);
      current_traj->phi.emplace_back(phi[i]);
      current_traj->gear = current_gear;
      partitioned_result->emplace_back();
      current_traj = &(partitioned_result->back());
      current_gear = gear;
    }
    current_traj->x.emplace_back(x[i]);
    current_traj->y.emplace_back(y[i]);
    current_traj->phi.emplace_back(phi[i]);
  }
  current_traj->x.emplace_back(x.back());
  current_traj->y.emplace_back(y.back());
  current_traj->phi.emplace_back(phi.back());
  current_traj->gear = current_gear;

  const auto start_timestamp = std::chrono::system_clock::now();

  // Retrieve v, a and steer from path
  // teb does not use speed profile from hybrid_a_star.
  // should use FLAGS_enable_use_teb to pass this process.
  for (auto& result : *partitioned_result) {
    // compute velocity and acceleration
    if (!GenerateSpeedAcceleration(&result)) {
      AERROR << "GenerateSpeedAcceleration fail";
      return false;
    }
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  AINFO << "speed profile total time: " << diff.count() * 1000.0 << " ms.";
  return true;
}

// compute v, a, steer
bool HybridAStar::GetTemporalProfile(HybridAStartResult* result) {
  std::vector<HybridAStartResult> partitioned_results;
  // segment trajectory
  if (!TrajectoryPartition(*result, &partitioned_results)) {
    AERROR << "TrajectoryPartition fail";
    return false;
  }
  // merge trajectory
  HybridAStartResult stitched_result;
  for (const auto& result : partitioned_results) {
    std::copy(result.x.begin(), result.x.end() - 1,
              std::back_inserter(stitched_result.x));
    std::copy(result.y.begin(), result.y.end() - 1,
              std::back_inserter(stitched_result.y));
    std::copy(result.phi.begin(), result.phi.end() - 1,
              std::back_inserter(stitched_result.phi));
    std::copy(result.v.begin(), result.v.end() - 1,
              std::back_inserter(stitched_result.v));
    std::copy(result.t.begin(), result.t.end(),
              std::back_inserter(stitched_result.t));
    std::copy(result.a.begin(), result.a.end(),
              std::back_inserter(stitched_result.a));
    std::copy(result.steer.begin(), result.steer.end(),
              std::back_inserter(stitched_result.steer));
  }
  stitched_result.x.emplace_back(partitioned_results.back().x.back());
  stitched_result.y.emplace_back(partitioned_results.back().y.back());
  stitched_result.phi.emplace_back(partitioned_results.back().phi.back());
  stitched_result.v.emplace_back(partitioned_results.back().v.back());
  stitched_result.t.emplace_back(partitioned_results.back().t.back());
  *result = stitched_result;
  return true;
}

// parking in function
bool HybridAStar::Plan(
    const std::vector<std::vector<Vec2d>>& world_obstacles_vertices_vec,
    double sx, double sy, double sphi, double ex, double ey, double ephi,
    double rotate_angle, const Vec2d& translate_origin,
    const std::vector<double>& XYbounds, const std::vector<Obstacle>& obstacles,
    const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec,
    HybridAStartResult* result, bool use_pure_astar, bool* start_collision_flag) {
  return Plan(world_obstacles_vertices_vec, sx, sy, sphi, ex, ey, ephi,
              rotate_angle, translate_origin, XYbounds, obstacles,
              obstacles_vertices_vec, result, nullptr, use_pure_astar,
              start_collision_flag);
}

// WYQ_Mark_Process 3
// parking out function
bool HybridAStar::Plan(
    const std::vector<std::vector<Vec2d>>& world_obstacles_vertices_vec,
    double sx, double sy, double sphi, double ex, double ey, double ephi,
    double rotate_angle, const Vec2d& translate_origin,
    const std::vector<double>& XYbounds, const std::vector<Obstacle>& obstacles,
    const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec,
    HybridAStartResult* result, const hdmap::Path *nearby_path,
    bool use_pure_astar, bool* start_collision_flag) {
  // update world --> map
  rotate_angle_ = rotate_angle;
  translate_origin_ = translate_origin;
  start_collision_flag_ = false;
  if (start_collision_flag != nullptr) {
    *start_collision_flag = start_collision_flag_;
  }
  if (is_hybrid_debug_) {       // create a cyber node
    CreateCyber();
  }
  if (FLAGS_start_collision_buff_adjustment) {
    astar_first_long_buffer_ = planner_open_space_config_.warm_start_config()
                                   .astar_first_long_buffer();
    astar_first_lat_buffer_ =
        planner_open_space_config_.warm_start_config().astar_first_lat_buffer();
  } else {
    if (!last_success_) {
      astar_first_long_buffer_ *= 0.5;
      astar_first_lat_buffer_ *= 0.5;
    } else {
      astar_first_long_buffer_ = planner_open_space_config_.warm_start_config()
                                     .astar_first_long_buffer();
      astar_first_lat_buffer_ = planner_open_space_config_.warm_start_config()
                                    .astar_first_lat_buffer();
    }
  }
  last_success_ = false;
  std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec;
  for (const auto& obstacle_vertices : obstacles_vertices_vec) {
    size_t vertices_num = obstacle_vertices.size();
    std::vector<common::math::LineSegment2d> obstacle_linesegments;
    for (size_t i = 0; i < vertices_num - 1; ++i) {
      common::math::LineSegment2d line_segment = common::math::LineSegment2d(
          obstacle_vertices[i], obstacle_vertices[i + 1]);
      obstacle_linesegments.emplace_back(line_segment);
    }
    if (!obstacle_linesegments.empty()) {
      obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
    }
  }
  obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);
  XYbounds_ = XYbounds;
  if (enable_voronoi_field_) {
    auto start = std::chrono::high_resolution_clock::now();
    int x = 0, y = 0;
    century::common::math::Vec2d point_temp;
    std::vector<ObsPoint> all_obstacle_points;
    for (auto& roi_boundary : world_obstacles_vertices_vec) {
      for (auto& point : roi_boundary) {
        point_temp = point;
        x = point_temp.x() * kMeter2Decimeter;  // m --> dm
        y = point_temp.y() * kMeter2Decimeter;  // m --> dm
        all_obstacle_points.emplace_back(x, y);
      }
    }
    if (obstacles.size() > 0) {
      // world obs 2 local Get local obs pouints
      for (auto& obstacle : obstacles) {
        if (obstacle.IsOutOfOpenSpaceROI()) {
          continue;
        }
        // transform to map
        for (auto& point : obstacle.PerceptionPolygon().GetAllPoints()) {
          point_temp = point;
          point_temp -= translate_origin_;
          point_temp.SelfRotate(-rotate_angle_);
          x = point_temp.x() * kMeter2Decimeter;  // m2dm
          y = point_temp.y() * kMeter2Decimeter;  // m2dm
          all_obstacle_points.emplace_back(x, y);
        }
      }
    }
    const ObsPoint map_ld_point =
        ObsPoint(static_cast<int>(XYbounds_[0] * kMeter2Decimeter),
                 static_cast<int>(XYbounds_[2] * kMeter2Decimeter));
    const int map_width = static_cast<int>((XYbounds_[1] - XYbounds_[0]) *
                                           kMeter2Decimeter);  // m --> dm
    const int map_height = static_cast<int>((XYbounds_[3] - XYbounds_[2]) *
                                            kMeter2Decimeter);  // m --> dm
    map_width_ = map_width;
    map_height_ = map_height;
    distanceTransform_.resize(map_width * map_height, INF);
    CalCulateBorderDistances(all_obstacle_points, map_width, map_height,
                             map_ld_point);
    all_obstacle_points.clear();
    all_obstacle_points.shrink_to_fit();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_milli =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count();
    AERROR << "Elapsed time in milliseconds : " << duration_milli << " ms";
  }

  if (is_hybrid_debug_) {           // publish obstacle list
    WriteObsLine2dData();
  }
  if (is_hybrid_debug_) {           // publish ROI
    WriteXYbounds();
  }

  // load nodes and obstacles
  AINFO << "start node " << sx << '\t' << sy << '\t' << sphi;
  AINFO << "end node " << ex << '\t' << ey << '\t' << ephi;
  start_node_.reset(
      new Node3d({sx}, {sy}, {sphi}, XYbounds_, planner_open_space_config_));
  end_node_.reset(
      new Node3d({ex}, {ey}, {ephi}, XYbounds_, planner_open_space_config_));

  if (is_hybrid_debug_) {           // publish end pose
    WriteStartEndPointData();
  }
  astar_long_buffer_ = astar_first_long_buffer_;
  astar_lat_buffer_ = astar_first_lat_buffer_;
  // final check
  if (!ValidityCheck(start_node_, true)) {           // check start node valid
    if (FLAGS_start_collision_buff_adjustment) {
      start_collision_flag_ = true;
      if (start_collision_flag != nullptr) {
        *start_collision_flag = start_collision_flag_;
      }
      if (!ValidityCheck(start_node_, true)) {  // check start node valid
        AERROR << "error, start_node in collision with obstacles(Reduce buff)";
        ha_debug_.insert(
            OpenspaceCommon::HybridAStarDebugStatus::START_COLLISION);
        return false;
      }

    } else {
      Vec2d test_point(start_node_->GetX(), start_node_->GetY());
      test_point.SelfRotate(rotate_angle_);
      test_point += translate_origin_;
      AERROR << "error, start_node in collision with obstacles, x: "
             << test_point.x() << ", y: " << test_point.y();
      ha_debug_.insert(
          OpenspaceCommon::HybridAStarDebugStatus::START_COLLISION);
      return false;
    }
  }

  // check end node valid
  if (nullptr == nearby_path && !ValidityCheck(end_node_)) {
    Vec2d test_point(end_node_->GetX(), end_node_->GetY());
    test_point.SelfRotate(rotate_angle_);
    test_point += translate_origin_;
    AERROR << "error, end_node in collision with obstacles, x: "
           << test_point.x() << ", y: " << test_point.y();
    ha_debug_.insert(OpenspaceCommon::HybridAStarDebugStatus::END_COLLISION);
    return false;
  }
  if (!Search(use_pure_astar, nearby_path)) {    // search fail
    AERROR << "ERROR, first search failed, swap start node and end node, "
              "search again";
    // swap start node and end node, search again
    start_node_.reset(
        new Node3d({ex}, {ey}, {ephi}, XYbounds_, planner_open_space_config_));
    end_node_.reset(
        new Node3d({sx}, {sy}, {sphi}, XYbounds_, planner_open_space_config_));
    if (!Search(use_pure_astar, nullptr)) {     // search fail
      AERROR << "ERROR, second search failed, return false";
      return false;
    } else {
      if (!GetResult(result, false)) {
        AERROR << "GetResult failed";
        ha_debug_.insert(OpenspaceCommon::HybridAStarDebugStatus::OTHERS);
        return false;
      }
    }
  } else {
    if (!GetResult(result, true)) {
      AERROR << "GetResult failed";
      ha_debug_.insert(OpenspaceCommon::HybridAStarDebugStatus::OTHERS);
      return false;
    }
  }

  (*result).result_size = result->x.size();
  (*result).rs_node_x = rs_node_->GetX();
  (*result).rs_node_y = rs_node_->GetY();
  (*result).rs_node_phi = rs_node_->GetPhi();
  (*result).traj_kappa_contraint_ratio = traj_kappa_contraint_ratio_;

  HybridAStarDebugInfo(is_hybrid_debug_, result);

  if (enable_voronoi_field_ && distanceTransform_.size() > 0) {
    distanceTransform_.clear();
    distanceTransform_.shrink_to_fit();
  }
  last_success_ = true;
  return true;
}

void HybridAStar::HybridAStarDebugInfo(const bool is_hybrid_debug,
                                       const HybridAStartResult* result) {
  if (is_hybrid_debug) {
    size_t result_size = result->x.size();
    century::planning_internal::OpenSpaceDebug open_space_debug;
    open_space_debug.set_is_first_hybrid_debug(false);

    century::planning_internal::HybridDebugResult hybrid_debug_result;
    hybrid_debug_result.set_result_size(result_size);
    hybrid_debug_result.set_heuristic_type(heuristic_type_);
    hybrid_debug_result.set_rs_num_peroid(rs_num_peroid_);
    hybrid_debug_result.set_is_hybrid_update_cost(is_hybrid_update_cost_);
    // Added for Hybrid a star Collision detection safety distance
    hybrid_debug_result.set_xy_grid_resolution(xy_grid_resolution_);
    hybrid_debug_result.set_hybrid_a_star_node_radius(
        hybrid_a_star_node_radius_);
    hybrid_debug_result.set_hybrid_a_star_bounds_radius(
        hybrid_a_star_bounds_radius_);
    hybrid_debug_result.set_is_hybrid_a_star_bounds_check(
        is_hybrid_a_star_bounds_check_);

    // Grid a star for heuristic
    hybrid_debug_result.set_grid_a_star_xy_resolution(
        grid_a_star_xy_resolution_);
    hybrid_debug_result.set_grid_a_star_node_radius(grid_a_star_node_radius_);
    hybrid_debug_result.set_is_convert_to_grid_coordinates(
        is_convert_to_grid_coordinates_);
    hybrid_debug_result.set_is_xy_bounds_check(is_xy_bounds_check_);
    hybrid_debug_result.set_grid_a_star_bounds_radius(
        grid_a_star_bounds_radius_);

    auto hybrid_debug_result_ptr =
        open_space_debug.mutable_hybrid_debug_result();
    hybrid_debug_result_ptr->CopyFrom(hybrid_debug_result);

    for (size_t i = 0; i < result_size; ++i) {
      century::planning_internal::HybridPointDebug hybrid_result_point;
      hybrid_result_point.set_x(result->x.at(i));
      hybrid_result_point.set_y(result->y.at(i));
      hybrid_result_point.set_phi(result->phi.at(i));
      auto* open_space_debug_ptr = open_space_debug.add_result_points();
      open_space_debug_ptr->CopyFrom(hybrid_result_point);
    }
    size_t send_times = 2;
    for (size_t i = 1; i < send_times; ++i) {
      open_space_writer_->Write(open_space_debug);
    }
  }
  return;
}

// hybrid astar search function
bool HybridAStar::Search(const bool use_pure_astar, const hdmap::Path *nearby_path) {
  // clear containers
  open_set_.clear();
  close_set_.clear();
  open_pq_ = decltype(open_pq_)();
  final_node_ = nullptr;

  // load openset. pq
  open_set_.emplace(start_node_->GetIndex(), start_node_);
  open_pq_.emplace(start_node_->GetIndex(), start_node_->GetCost());

  // Hybrid A* begins
  size_t explored_node_num = 0;        // explore node number
  size_t search_rs_num = 0;            // search reedshepp curve node number
  size_t run_node_num = 0;             // run node number
  double astar_start_time = Clock::NowInSeconds();   // search start timestamp
  double rs_time = 0.0;                // reedshepp curve duration
  double cyber_time = 0.0;             //

  AINFO << "next_node_num: " << next_node_num_;
  AINFO << "astar_max_time_first: " << astar_max_time_first_
        << ", astar_max_time_second: " << astar_max_time_second_;
  AINFO << "force_forward_init_" << force_forward_init_;

  while (!open_pq_.empty()) {
    ++run_node_num;

    // take out the lowest cost neighboring node
    const std::string current_id = open_pq_.top().first;
    open_pq_.pop();
    std::shared_ptr<Node3d> current_node = open_set_[current_id];

    std::vector<Node3d> hybrid_node;
    if (is_hybrid_debug_) {
      hybrid_node.emplace_back(*current_node);
    }

    ++search_rs_num;
    const double rs_start_time = Clock::NowInSeconds();
    // reedshepp curve expand or check is near goal
    if (AnalyticExpansion(astar_start_time, force_forward_init_, current_node,
                          use_pure_astar, nearby_path)) {
      rs_node_ = current_node;
      break;
    }
    const double rs_end_time = Clock::NowInSeconds();
    rs_time += rs_end_time - rs_start_time;

    // insert current node into close list
    close_set_.emplace(current_node->GetIndex(), current_node);

    for (size_t i = 0; i < next_node_num_; ++i) {
      // next node
      std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);
      // boundary check failure handle
      if (nullptr == next_node) {
        continue;
      }
      // check if the node is already in the close set
      if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
        continue;
      }
      // validity check
      if (!ValidityCheck(next_node)) {
        continue;
      }
      if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
        if (is_hybrid_debug_) {
          hybrid_node.emplace_back(*next_node);
        }

        explored_node_num++;
        // compute next node f_cost
        CalculateNodeCost(astar_start_time, current_node, next_node, nearby_path);
        open_set_.emplace(next_node->GetIndex(), next_node);
        open_pq_.emplace(next_node->GetIndex(), next_node->GetCost());
      }
    }
    if (force_forward_init_ > 0) {
      --force_forward_init_;
    }

    if (is_hybrid_debug_) {
      const double cyber_start_time = Clock::NowInSeconds();
      WriteData(hybrid_node);
      // sleep some times ms
      size_t hybrid_debug_next_node_sleep_ms =
          planner_open_space_config_.warm_start_config()
              .hybrid_debug_next_node_sleep_ms();
      // delay ms
      DelayMilliSecond(hybrid_debug_next_node_sleep_ms);
      const double cyber_end_time = Clock::NowInSeconds();
      cyber_time += cyber_end_time - cyber_start_time;
    }

    // overtime check
    if (!is_map_boundary_) {             // no map boundary
      // !is_map_boundary_ means large boundary, cost more time
      // so usually astar_max_time_first_ >= astar_max_time_first_
      if (Clock::NowInSeconds() - astar_start_time > astar_max_time_first_) {
        AERROR << "ERROR, Hybrid A searching run_node_num " << run_node_num
               << " with time " << astar_max_time_first_;
        ha_debug_.insert(
            OpenspaceCommon::HybridAStarDebugStatus::PLAN_OVERTIME2);
        return false;
      }
    } else {
      if (Clock::NowInSeconds() - astar_start_time > astar_max_time_second_) {
        AERROR << "ERROR, Use Boundary Hybrid A searching run_node_num "
               << run_node_num << " with time " << astar_max_time_second_;
        ha_debug_.insert(
            OpenspaceCommon::HybridAStarDebugStatus::PLAN_OVERTIME);
        return false;
      }
    }
  }
  if (nullptr == final_node_) {
    AERROR << "ERROR, Hybrid A searching return null ptr(open_set ran out)";
    ha_debug_.insert(OpenspaceCommon::HybridAStarDebugStatus::SEARCH_FAIL);
    return false;
  }

  return true;
}

const double HybridAStar::GetVoronoiValue(const int x, const int y,
                                          const int nx, const int ny,
                                          const ObsPoint& map_ld_point) {
  double value = 0.0;
  if ((y - map_ld_point.Y()) >= ny || (x - map_ld_point.X()) >= nx ||
      (y - map_ld_point.Y()) < 0 || (x - map_ld_point.X()) < 0) {
    return voronoi_obs_acting_dis_;
  }
  double dis =
      distanceTransform_[(y - map_ld_point.Y()) * nx + (x - map_ld_point.X())];
  if (dis > voronoi_obs_acting_dis_) {
    value = 0.0;
  } else {
    value = voronoi_obs_acting_dis_ -
            voronoi_obs_acting_dis_ * (dis / voronoi_obs_acting_dis_);
  }
  return value;
}

void HybridAStar::CalCulateBorderDistances(
    const std::vector<ObsPoint>& all_obs_point, const int nx, const int ny,
    const ObsPoint& map_ld_point) {
  // Step 1: Create a binary image where pixels on color borders are foreground
  // (1) and others are background (0).
  std::vector<int> binaryImage(nx * ny, 0);

  for (auto const& point : all_obs_point) {
    // Map boundary protection
    if ((point.Y() - map_ld_point.Y()) >= ny ||
        (point.X() - map_ld_point.X()) >= nx ||
        (point.Y() - map_ld_point.Y()) < 0 ||
        (point.X() - map_ld_point.X()) < 0) {
      continue;
    }
    binaryImage[(point.Y() - map_ld_point.Y()) * nx +
                (point.X() - map_ld_point.X())] = 1;
  }

  // Step 2: Apply distance transform to the binary image.
  // AERROR << "*distanceTransform_ " << distanceTransform_.size();
  for (int y = 0; y < ny; ++y) {
    for (int x = 0; x < nx; ++x) {
      if (1 == binaryImage[y * nx + x]) {
        distanceTransform_[y * nx + x] = 0.0f;
      } else {
        if (x > 0) {
          distanceTransform_[y * nx + x] =
              std::min(distanceTransform_[y * nx + x],
                       distanceTransform_[y * nx + x - 1] + 1.0f);
        }
        if (y > 0) {
          distanceTransform_[y * nx + x] =
              std::min(distanceTransform_[y * nx + x],
                       distanceTransform_[(y - 1) * nx + x] + 1.0f);
        }
      }
    }
  }
  binaryImage.clear();
  binaryImage.shrink_to_fit();

  for (int y = ny - 1; y >= 0; --y) {
    for (int x = nx - 1; x >= 0; --x) {
      if (x < nx - 1) {
        distanceTransform_[y * nx + x] =
            std::min(distanceTransform_[y * nx + x],
                     distanceTransform_[y * nx + x + 1] + 1.0f);
      }
      if (y < ny - 1) {
        distanceTransform_[y * nx + x] =
            std::min(distanceTransform_[y * nx + x],
                     distanceTransform_[(y + 1) * nx + x] + 1.0f);
      }
    }
  }
  return;
}

// create a cyber node
void HybridAStar::CreateCyber() {
  if (!is_hybrid_node_init_) {
    std::unique_ptr<century::cyber::Node> node =
        century::cyber::CreateNode("hybrid node", "hybrid");
    open_space_writer_ =
        node->CreateWriter<century::planning_internal::OpenSpaceDebug>(
            "/century/open_space_debug");
    is_hybrid_node_init_ = true;
  }

  // First Send No Data To Clear View Shows
  if (is_hybrid_debug_) {
    century::planning_internal::OpenSpaceDebug open_space_debug;
    open_space_debug.set_is_first_hybrid_debug(true);
    // Send many times
    size_t send_times = 2;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    for (size_t i = 0; i < send_times; ++i) {
      bool is_success = open_space_writer_->Write(open_space_debug);
      if (!is_success) {
        AERROR << "hybrid a star send failed.";
      }
    }
  }
}

void HybridAStar::WriteData(const std::vector<Node3d>& hybrid_node) {
  century::planning_internal::OpenSpaceDebug open_space_debug;
  open_space_debug.set_is_first_hybrid_debug(false);
  for (size_t i = 0; i < hybrid_node.size(); i++) {
    century::planning_internal::HybridPointDebug hybrid_debug;
    hybrid_debug.set_x(hybrid_node[i].GetX());
    hybrid_debug.set_y(hybrid_node[i].GetY());
    hybrid_debug.set_phi(hybrid_node[i].GetPhi());
    auto* open_space_debug_ptr = open_space_debug.add_hybrid_debug();
    open_space_debug_ptr->CopyFrom(hybrid_debug);
  }
  open_space_writer_->Write(open_space_debug);
}

void century::planning::HybridAStar::DelayMilliSecond(int msec) {
  std::this_thread::sleep_for(std::chrono::milliseconds(msec));
}

void HybridAStar::DelaySecond(int sec) {
  std::this_thread::sleep_for(std::chrono::seconds(sec));
}

void HybridAStar::WriteObsLine2dData() {
  century::planning_internal::OpenSpaceDebug open_space_debug;
  open_space_debug.set_is_first_hybrid_debug(false);
  open_space_debug.mutable_hybrid_rs_debug()->set_is_rs_succeed(false);
  open_space_debug.set_is_use_a_star_path(false);
  ADEBUG << "obstacles_linesegments_vec_ size"
         << obstacles_linesegments_vec_.size();

  size_t index = 0;
  for (const auto& obstacles_linesegments : obstacles_linesegments_vec_) {
    size_t vertices_num = obstacles_linesegments.size();
    ADEBUG << "obstacles_linesegments size:" << vertices_num;
    century::planning_internal::HybridObsLineSegment2d hybrid_obs_line2d;
    for (size_t i = 0; i < vertices_num; ++i) {
      common::math::LineSegment2d line_segment = obstacles_linesegments.at(i);
      hybrid_obs_line2d.add_x(line_segment.start().x());
      hybrid_obs_line2d.add_y(line_segment.start().y());
      hybrid_obs_line2d.add_x(line_segment.end().x());
      hybrid_obs_line2d.add_y(line_segment.end().y());
      ADEBUG << "obstacle x1 " << line_segment.start().x();
      ADEBUG << "obstacle x2 " << line_segment.end().x();
    }
    auto* hybrid_obs_line2d_ptr = open_space_debug.add_hybrid_obs_line2d();
    hybrid_obs_line2d_ptr->CopyFrom(hybrid_obs_line2d);
    index++;
  }

  // Send many times
  DelayMilliSecond(10);
  size_t send_times = 1;
  for (size_t i = 0; i < send_times; ++i) {
    DelayMilliSecond(10);
    open_space_writer_->Write(open_space_debug);
  }
  DelayMilliSecond(10);
}

void HybridAStar::WriteStartEndPointData() {
  century::planning_internal::OpenSpaceDebug open_space_debug;
  open_space_debug.set_is_first_hybrid_debug(false);
  open_space_debug.mutable_hybrid_rs_debug()->set_is_rs_succeed(false);
  open_space_debug.set_is_use_a_star_path(false);
  open_space_debug.mutable_hybrid_search_info()
      ->set_is_send_start_end_point(true);

  century::planning_internal::HybridPointDebug hybrid_start_point;
  hybrid_start_point.set_x(start_node_->GetX());
  hybrid_start_point.set_y(start_node_->GetY());
  hybrid_start_point.set_phi(start_node_->GetPhi());
  century::planning_internal::HybridPointDebug hybrid_end_point;
  hybrid_end_point.set_x(end_node_->GetX());
  hybrid_end_point.set_y(end_node_->GetY());
  hybrid_end_point.set_phi(end_node_->GetPhi());

  auto* search_point_ptr = open_space_debug.mutable_hybrid_search_info();
  search_point_ptr->mutable_hybrid_start_point()->CopyFrom(hybrid_start_point);
  search_point_ptr->mutable_hybrid_end_point()->CopyFrom(hybrid_end_point);

  // Send many times
  DelayMilliSecond(10);
  size_t send_times = 2;
  for (size_t i = 0; i < send_times; ++i) {
    DelayMilliSecond(10);
    open_space_writer_->Write(open_space_debug);
  }
  DelayMilliSecond(10);
}

void HybridAStar::WriteXYbounds() {
  century::planning_internal::OpenSpaceDebug open_space_debug;
  open_space_debug.set_is_first_hybrid_debug(false);
  open_space_debug.mutable_hybrid_rs_debug()->set_is_rs_succeed(false);
  open_space_debug.set_is_use_a_star_path(false);
  open_space_debug.mutable_hybrid_xy_bounds_info()->set_is_send_xy_bounds(true);

  century::planning_internal::HybridXYboundsInfo hybrid_xy_bounds_info;
  hybrid_xy_bounds_info.set_x0(XYbounds_[0]);
  hybrid_xy_bounds_info.set_x1(XYbounds_[1]);
  hybrid_xy_bounds_info.set_y0(XYbounds_[2]);
  hybrid_xy_bounds_info.set_y1(XYbounds_[3]);
  open_space_debug.mutable_hybrid_xy_bounds_info()->CopyFrom(
      hybrid_xy_bounds_info);

  // Send many times
  DelayMilliSecond(10);
  size_t send_times = 2;
  for (size_t i = 0; i < send_times; ++i) {
    DelayMilliSecond(10);
    open_space_writer_->Write(open_space_debug);
  }
  DelayMilliSecond(10);
}

// generate A* path
bool HybridAStar::GenerateAStarPath() {
  // A Star
  double a_star_time = Clock::NowInSeconds();
  bool success = grid_a_star_heuristic_generator_->GenerateAStarPath(
      start_node_->GetX(), start_node_->GetY(), end_node_->GetX(),
      end_node_->GetY(), XYbounds_, obstacles_linesegments_vec_,
      &a_star_result_);
  AINFO << "a star path time " << Clock::NowInSeconds() - a_star_time << " s.";
  double h_cost = a_star_result_.path_cost;
  if (success) {
    AINFO << "Hybrid A Get H with GenerateAStarPath Succeed, H Cost is "
          << h_cost;

  } else {
    AINFO << "Hybrid A Get H with GenerateAStarPath Failed, H Cost is "
          << h_cost;
  }
  is_a_star_success_ = success;
  return is_a_star_success_;
}

}  // namespace planning
}  // namespace century
