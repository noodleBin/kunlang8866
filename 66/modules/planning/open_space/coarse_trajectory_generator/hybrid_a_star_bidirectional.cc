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

/**
 * @file: hybrid_a_star_bidirectional.cc
 **/

#include "modules/planning/open_space/coarse_trajectory_generator/hybrid_a_star_bidirectional.h"

namespace century {
namespace planning {

std::vector<std::vector<common::math::Vec2d>>
    HybridAStarBidirectional::obstacle_points_;

HybridAStarBidirectional::HybridAStarBidirectional(
    const PlannerOpenSpaceConfig& open_space_conf) {
  planner_open_space_config_.CopyFrom(open_space_conf);
  open_space_bidirection_config_ =
      planner_open_space_config_.warm_start_config()
          .openspace_bidirection_config();

  reed_shepp_generator_ =
      std::make_unique<ReedShepp>(vehicle_param_, planner_open_space_config_);
  grid_a_star_heuristic_generator_ =
      std::make_unique<GridSearch>(planner_open_space_config_);
  next_node_num_ =
      planner_open_space_config_.warm_start_config().next_node_num();
  CHECK_NE(next_node_num_, 2UL);
  force_forward_init_ =
      planner_open_space_config_.warm_start_config().force_forward_init();
  traj_kappa_contraint_ratio_ = planner_open_space_config_.warm_start_config()
                                    .traj_kappa_contraint_ratio();

  SetMaxWheelAngle(traj_kappa_contraint_ratio_);
  sample_step_size_ = planner_open_space_config_.warm_start_config().step_size();
  xy_grid_resolution_ =
      planner_open_space_config_.warm_start_config().xy_grid_resolution();
  delta_t_ = planner_open_space_config_.delta_t();
  traj_forward_penalty_ =
      planner_open_space_config_.warm_start_config().traj_forward_penalty();
  traj_back_penalty_ =
      planner_open_space_config_.warm_start_config().traj_back_penalty();
  traj_gear_switch_penalty_ =
      open_space_bidirection_config_.traj_gear_switch_penalty();
  traj_steer_penalty_ = open_space_bidirection_config_.traj_steer_penalty();
  traj_steer_change_penalty_ =
      open_space_bidirection_config_.traj_steer_change_penalty();
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

  bidirection_astar_search_time_threshold_ =
      open_space_bidirection_config_.bidirection_astar_search_time_threshold();
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

  is_change_start_end_ = false;

  ADEBUG << "construct HybridAStar.";
  ADEBUG
      << "astar_max_time_first: "
      << planner_open_space_config_.warm_start_config().astar_max_time_first()
      << ", astar_max_time_second: "
      << planner_open_space_config_.warm_start_config().astar_max_time_second();
  ADEBUG << "astar_first_long_buffer: "
         << planner_open_space_config_.warm_start_config()
                .astar_first_long_buffer()
         << ", astar_first_lat_buffer: "
         << planner_open_space_config_.warm_start_config()
                .astar_first_lat_buffer();
  ADEBUG << "hybrid_use_rs_dis: "
         << planner_open_space_config_.warm_start_config().hybrid_use_rs_dis();

  ADEBUG << "next_node_num_ = " << next_node_num_;
  ADEBUG << "max_front_wheel_angle_ = " << max_front_wheel_angle_;
  ADEBUG << "sample_step_size_ = " << sample_step_size_;
  ADEBUG << "xy_grid_resolution_ = " << xy_grid_resolution_;
  ADEBUG << "delta_t_ = " << delta_t_;
  ADEBUG << "traj_forward_penalty_ = " << traj_forward_penalty_;
  ADEBUG << "traj_back_penalty_ = " << traj_back_penalty_;
  ADEBUG << "traj_gear_switch_penalty_ = " << traj_gear_switch_penalty_;
  ADEBUG << "traj_steer_penalty_ = " << traj_steer_penalty_;
  ADEBUG << "traj_steer_change_penalty_ = " << traj_steer_change_penalty_;
  ADEBUG << "traj_end_heading_error_penalty_ = "
         << traj_end_heading_error_penalty_;
  ADEBUG << "is_hybrid_debug_ = " << is_hybrid_debug_;
  ADEBUG << "grid_a_star_xy_resolution_ = " << grid_a_star_xy_resolution_;
  ADEBUG << "grid_a_star_node_radius_ = " << grid_a_star_node_radius_;
}

double HybridAStarBidirectional::ObstacleCost(
    const std::shared_ptr<century::planning::Node3d>& node,
    const std::vector<std::vector<common::math::Vec2d>>& obstacle_points) {
  const double d = Distance(node->GetX(), node->GetY(), obstacle_points);
  constexpr double kSafeDist = 1.0;
  constexpr double kHardKill = 0.3;
  constexpr double kWeight = 50.0;
  if (d < kHardKill) {
    return std::numeric_limits<double>::infinity();
  }

  if (d > kSafeDist) {
    ADEBUG << "  The distance between the node and the obstacle "
              "is greater than the safe distance("
           << kSafeDist << "), do not penalty, return 0.";
    return 0.0;
  }
  return kWeight * (kSafeDist - d) * (kSafeDist - d);
}

void HybridAStarBidirectional::SetMaxWheelAngle(
    const double traj_kappa_contraint_ratio) {
  double min_turn_radius = vehicle_param_.min_turn_radius();
  if (common::util::IsFloatEqual(min_turn_radius, 0.0)) {
    min_turn_radius = kMinTurnRadius;
  }
  double max_front_steer_angle =
      vehicle_param_.steer_ratio() *
      std::atan(vehicle_param_.wheel_base() / 2.0 / min_turn_radius);
  double final_ratio =
      vehicle_param_.steer_ratio() * traj_kappa_contraint_ratio;
  if (common::util::IsFloatEqual(final_ratio, 0.0)) {
    final_ratio = kBaseRatio;
  }
  sample_step_size_ /= traj_kappa_contraint_ratio;
  max_front_wheel_angle_ = max_front_steer_angle / final_ratio;
  next_node_num_ = int(max_front_wheel_angle_ /
                       planner_open_space_config_.warm_start_config()
                           .phi_grid_resolution()) *
                   4;
  AINFO << "min_turn_radius = " << min_turn_radius;
  AINFO << "max_front_steer_angle = " << max_front_steer_angle;
  AINFO << "final_ratio = " << final_ratio;
  AINFO << "max_steer_angle = " << vehicle_param_.max_steer_angle();
  AINFO << "max_front_wheel_angle_ = " << max_front_wheel_angle_;
  AINFO << "next_node_num = " << next_node_num_;
}

void HybridAStarBidirectional::SetKappaContraintConfig(
    const KappaContraintRatioConfig& kappa_config) {
  traj_kappa_contraint_ratio_ = kappa_config.traj_kappa_contraint_ratio();
  SetMaxWheelAngle(traj_kappa_contraint_ratio_);
  astar_max_time_first_ = kappa_config.astar_max_time_first();
  astar_max_time_second_ = kappa_config.astar_max_time_second();
  reed_shepp_generator_->SetKappaContraintConfig(
      kappa_config.traj_kappa_contraint_ratio());
}

void HybridAStarBidirectional::ResetDefaultKappaContraint() {
  traj_kappa_contraint_ratio_ = planner_open_space_config_.warm_start_config()
                                    .traj_kappa_contraint_ratio();
  SetMaxWheelAngle(traj_kappa_contraint_ratio_);
  astar_max_time_first_ =
      planner_open_space_config_.warm_start_config().astar_max_time_first();
  astar_max_time_second_ =
      planner_open_space_config_.warm_start_config().astar_max_time_second();
  reed_shepp_generator_->ResetDefaultKappaContraint();
}

void HybridAStarBidirectional::ResetSearchState() {
  meet_node_forward_ = nullptr;
  meet_node_backward_ = nullptr;
  final_node_ = nullptr;

  open_set_.clear();
  close_set_.clear();
  open_pq_ = decltype(open_pq_)();

  open_set_rev_.clear();
  close_set_rev_.clear();
  open_pq_rev_ = decltype(open_pq_rev_)();
}

double HybridAStarBidirectional::CalCurrentNodeToEndNodeDis(
    const std::shared_ptr<Node3d> current_node) {
  return std::hypot((end_node_->GetX() - current_node->GetX()),
                    (end_node_->GetY() - current_node->GetY()));
}

// expand node using reedshepp curve
bool HybridAStarBidirectional::AnalyticExpansion(
    const double& astar_start_time, const size_t force_forward_init,
    const std::shared_ptr<Node3d> current_node, const bool use_pure_astar,
    const hdmap::Path* nearby_path) {
  double dis = CalCurrentNodeToEndNodeDis(current_node);
  if (!use_pure_astar &&
      (dis <= hybrid_use_rs_dis_ ||
       Clock::NowInSeconds() - astar_start_time > kWillOverTime)) {
    std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
        std::make_shared<ReedSheppPath>();

    if (planner_open_space_config_.warm_start_config().use_bestest_rs()) {
      if (!reed_shepp_generator_->BestestRSP(current_node, end_node_,
                                             reeds_shepp_to_check)) {
        AERROR << " BestestRSP failed";
        return false;
      }
    } else {
      if (!reed_shepp_generator_->ShortestRSP(current_node, end_node_,
                                              reeds_shepp_to_check)) {
        AERROR << " ShortestRSP failed";
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
    ADEBUG << "current_node node index id: " << current_node->GetIndexString();
    ADEBUG << "current_node node (x,y,phi): [" << current_node->GetX() << ","
           << current_node->GetY() << "," << current_node->GetPhi() << "]";
    ADEBUG << "current_node node total cost F: " << current_node->GetCost()
           << " G:" << current_node->GetTrajCost()
           << " H:" << current_node->GetHeuCost();
    AddRSDebugInfo(current_node.get(), is_hybrid_debug_);
  } else {
    if (IsNearGoal2(current_node, false, nearby_path)) {
      close_set_.emplace(current_node->GetIntIndex(), current_node);
      // load the whole RSP as nodes and add to the close set
      final_node_ = current_node;
    } else {
      return false;
    }
  }
  return true;
}

// generate node3d and check validity
bool HybridAStarBidirectional::RSPCheck(
    const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end) {
  std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, planner_open_space_config_));
  return ValidityCheck(node);
}

bool HybridAStarBidirectional::InitForwardCheck(
    const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end) {
  return !reeds_shepp_to_end->gear.empty() && reeds_shepp_to_end->gear[0];
}

bool HybridAStarBidirectional::RemoveCloseNodes(
    const double min_distance_threshold,
    std::vector<std::shared_ptr<Node3d>>* nodes) {
  if (nodes->size() < 2) {
    AERROR << __func__ << ", input node is empty, return false.";
    return false;
  }
  std::shared_ptr<Node3d> last_kept = nodes->front();
  // new_end torwards the end of the vector element that need keep
  auto new_end = std::remove_if(
      nodes->begin() + 1, nodes->end(),
      [&last_kept, min_distance_threshold](
          const std::shared_ptr<Node3d> p) mutable -> bool {
        if (last_kept->DistanceTo(*(p.get())) >= min_distance_threshold) {
          last_kept = p;
          return false;
        }
        return true;
      });
  nodes->erase(new_end, nodes->end());
  return true;
}

bool HybridAStarBidirectional::NodeCostFilter(
    const std::shared_ptr<Node3d> curr_node,
    const std::unordered_map<NodeIndex, std::shared_ptr<Node3d>>& node_set,
    std::vector<std::shared_ptr<Node3d>>* output_node) {
  if (node_set.empty()) {
    AERROR << __func__ << ", node_set is empty, return false.";
    return false;
  }

  for (const auto& oppo_node : node_set) {
    output_node->emplace_back(oppo_node.second);
  }
  // in order to avoid too much invaild rs connect times
  if (!RemoveCloseNodes(
          open_space_bidirection_config_.min_close_dis_threshold(),
          output_node)) {
    return false;
  }

  std::sort(output_node->begin(), output_node->end(),
            [curr_node](const std::shared_ptr<Node3d>& a,
                            const std::shared_ptr<Node3d>& b) {
              double dx_a = a->GetX() - curr_node->GetX();
              double dy_a = a->GetY() - curr_node->GetY();
              double dist_a = dx_a * dx_a + dy_a * dy_a;

              double dx_b = b->GetX() - curr_node->GetX();
              double dy_b = b->GetY() - curr_node->GetY();
              double dist_b = dx_b * dx_b + dy_b * dy_b;

              if (std::fabs(dist_a - dist_b) > 1e-6) {
                return dist_a < dist_b;
              }
              return a->GetCost() < b->GetCost();
            });
  // cut size to rs_connect_max_num：20
  if (output_node->size() >
      open_space_bidirection_config_.rs_connect_max_num()) {
    output_node->erase(output_node->begin() +
                           open_space_bidirection_config_.rs_connect_max_num(),
                       output_node->end());
  }
  return true;
}

bool HybridAStarBidirectional::TryRSConnectOppoSide(
    std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> oppo_node,
    std::shared_ptr<century::planning::ReedSheppPath> optimal_path) {
  if (!reed_shepp_generator_->ShortestRSP(cur_node, oppo_node, optimal_path)) {
    AERROR << __func__ << ", ShortestRSP failed, return false.";
    return false;
  }
  if (!RSPCheck(optimal_path)) {
    AERROR << ", shortest rs path collision check failed, return false.";
    return false;
  }
  return true;
}

bool HybridAStarBidirectional::MeetInMiddle(
    std::shared_ptr<Node3d> current_node,
    std::unordered_map<NodeIndex, std::shared_ptr<Node3d>>& oppo_set,
    const bool is_forward_search, std::vector<HybridAStartResult>* result) {
  std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
      std::make_shared<ReedSheppPath>();
  constexpr double meet_angle_threshold = 0.2;

  std::vector<std::shared_ptr<Node3d>> oppo_nodes_vec;
  if (!NodeCostFilter(current_node, oppo_set, &oppo_nodes_vec)) {
    AERROR << "filter node failed,return false.";
    return false;
  }

  for (const auto& oppo_node : oppo_nodes_vec) {
    if (TryRSConnectOppoSide(current_node, oppo_node, reeds_shepp_to_check)) {
      meet_node_forward_ = is_forward_search
                               ? LoadRSPinCS(reeds_shepp_to_check, current_node,
                                             is_forward_search)
                               : oppo_node;
      meet_node_backward_ = is_forward_search
                                ? oppo_node
                                : LoadRSPinCS(reeds_shepp_to_check,
                                              current_node, is_forward_search);
      AINFO << __func__ << ", [success]RS curve connect successfully!!!!";
      return true;
    }

    double distance = std::hypot(current_node->GetX() - oppo_node->GetX(),
                                 current_node->GetY() - oppo_node->GetY());
    double x_diff = std::fabs(current_node->GetX() - oppo_node->GetX());
    double y_diff = std::fabs(current_node->GetY() - oppo_node->GetY());
    double intersection_x = INT_MAX;
    double intersection_y = INT_MAX;
    double dist_intersection = INT_MAX;
    ADEBUG << "meet condition, distance: " << distance << ", x_diff: " << x_diff
           << ", y_diff: " << y_diff;
    if (distance <= open_space_bidirection_config_.meet_distance_threshold() &&
        x_diff <= open_space_bidirection_config_.meet_x_diff_threshold() &&
        y_diff <= open_space_bidirection_config_.meet_y_diff_threshold()) {
      double angle_diff = std::abs(
          NormalizeAngle(current_node->GetPhi() - oppo_node->GetPhi()));
      bool angle_match =
          (angle_diff <=
           open_space_bidirection_config_.meet_angle_threshold()) ||
          (std::abs(angle_diff - M_PI) <= meet_angle_threshold);

      if (util::FindLineIntersection(current_node->GetX(), current_node->GetY(),
                                     current_node->GetPhi(), oppo_node->GetX(),
                                     oppo_node->GetY(), oppo_node->GetPhi(),
                                     intersection_x, intersection_y, true)) {
        common::math::Vec2d intersection_point(intersection_x, intersection_y);
        dist_intersection =
            std::min(intersection_point.DistanceTo(common::math::Vec2d(
                         current_node->GetX(), current_node->GetY())),
                     intersection_point.DistanceTo(common::math::Vec2d(
                         oppo_node->GetX(), oppo_node->GetY())));
      }
      AINFO << "near meeting, min distance: " << dist_intersection
            << ", angle match: " << angle_match;
      if (angle_match &&
          dist_intersection < open_space_bidirection_config_
                                  .meet_distance_intersection_threshold()) {
        meet_node_forward_ = is_forward_search ? current_node : oppo_node;
        meet_node_backward_ = is_forward_search ? oppo_node : current_node;
        AINFO << "meeting! dis: " << distance
              << ", min_dis_to_intersection: " << dist_intersection
              << ", angle_diff: " << angle_diff * 180 / M_PI << "°";
        return true;
      }
    }
  }
  AERROR << "MeetInMiddle failed, return false.";
  return false;
}

bool HybridAStarBidirectional::IsBidirectionalGoal(std::shared_ptr<Node3d> node,
                                                   bool is_forward_search) {
  std::shared_ptr<Node3d> goal_node =
      is_forward_search ? end_node_ : start_node_;
  std::shared_ptr<Node3d> goal_node_rev =
      is_forward_search ? end_node_rev_ : start_node_rev_;

  double distance = std::hypot(node->GetX() - goal_node->GetX(),
                               node->GetY() - goal_node->GetY());
  double angle_diff =
      std::abs(NormalizeAngle(node->GetPhi() - goal_node->GetPhi()));

  bool goal_match =
      (distance <= open_space_bidirection_config_.goal_distance_threshold()) &&
      (angle_diff <= open_space_bidirection_config_.goal_angle_threshold());

  double distance_rev = std::hypot(node->GetX() - goal_node_rev->GetX(),
                                   node->GetY() - goal_node_rev->GetY());
  double angle_diff_rev =
      std::abs(NormalizeAngle(node->GetPhi() - goal_node_rev->GetPhi()));

  bool goal_rev_match =
      (distance_rev <=
       open_space_bidirection_config_.goal_distance_threshold()) &&
      (std::abs(angle_diff_rev - M_PI) <=
       open_space_bidirection_config_.goal_angle_threshold());

  ADEBUG << (is_forward_search ? "forward" : "backward")
         << "distance: " << distance
         << ", angle_diff: " << angle_diff * 180 / M_PI
         << ", distance_rev: " << distance_rev
         << ", angle_diff_rev: " << angle_diff_rev * 180 / M_PI;
  AINFO << "goal_match: " << goal_match
        << ", goal_rev_match: " << goal_rev_match;

  if (goal_match || goal_rev_match) {
    AINFO << __func__
          << ", [search success], The node successfully searched to the end "
             "point!";
    AINFO << (is_forward_search ? "forward" : "backward")
          << "search reached goal, dis: "
          << (goal_match ? distance : distance_rev) << ", angle_diff: "
          << (goal_match ? angle_diff : angle_diff_rev) * 180 / M_PI;
    if (goal_match) {
      meet_node_forward_ = is_forward_search ? node : start_node_;
      meet_node_backward_ = is_forward_search ? end_node_ : node;
    } else {
      meet_node_forward_ = is_forward_search ? node : start_node_rev_;
      meet_node_backward_ = is_forward_search ? end_node_rev_ : node;
    }
    return true;
  }

  return false;
}

// check if it is near path or end pose
bool HybridAStarBidirectional::IsNearGoal2(std::shared_ptr<Node3d> node,
                                           bool rs_node,
                                           const hdmap::Path* nearby_path) {
  const auto& traversed_x = node->GetXs();
  const auto& traversed_y = node->GetYs();
  const auto& traversed_phi = node->GetPhis();

  const auto& end_x = end_node_->GetX();
  const auto& end_y = end_node_->GetY();
  const auto& end_phi = end_node_->GetPhi();
  bool find_end = false;
  if (nullptr == nearby_path || 0 == nearby_path->length()) {
    // park in
    if (rs_node) {
      return IsNearGoal(node);
    }
    for (size_t i = 0; i < traversed_x.size(); ++i) {
      if (std::fabs(end_x - traversed_x[i]) <=
              search_near_destination_x_threshold_ &&
          std::fabs(end_y - traversed_y[i]) <=
              search_near_destination_y_threshold_ &&
          (std::fabs(NormalizeAngle(end_phi - traversed_phi[i])) <=
               search_near_destination_theta_threshold_ ||
           std::fabs(NormalizeAngle(-end_phi - traversed_phi[i])) <=
               search_near_destination_theta_threshold_)) {
        std::vector<double> xs(traversed_x.begin(),
                               traversed_x.begin() + i + 1);
        std::vector<double> ys(traversed_y.begin(),
                               traversed_y.begin() + i + 1);
        std::vector<double> phis(traversed_phi.begin(),
                                 traversed_phi.begin() + i + 1);
        node->SetTraversed(xs, ys, phis, XYbounds_, planner_open_space_config_);
        find_end = true;
        break;
      }
    }
  } else {
    // park out
    for (size_t i = 0; i < traversed_x.size(); ++i) {
      // origin ---> world
      Vec2d point(traversed_x[i], traversed_y[i]);
      point.SelfRotate(rotate_angle_);
      point += translate_origin_;
      double heading = NormalizeAngle(traversed_phi[i] + rotate_angle_);
      const auto& bounding_box = Node3d::GetBoundingBoxWithBuffer(
          vehicle_param_, point.x(), point.y(), heading, astar_long_buffer_,
          astar_lat_buffer_);
      double s = 0.0, l = 0.0;
      nearby_path->GetNearestPoint(point, &s, &l);
      auto smooth_point = nearby_path->GetSmoothPoint(s);
      if (nearby_path->IsOnPath(bounding_box) &&
          std::fabs(NormalizeAngle(heading - smooth_point.heading())) <
              M_PI_2) {
        std::vector<double> xs(traversed_x.begin(),
                               traversed_x.begin() + i + 1);
        std::vector<double> ys(traversed_y.begin(),
                               traversed_y.begin() + i + 1);
        std::vector<double> phis(traversed_phi.begin(),
                                 traversed_phi.begin() + i + 1);
        node->SetTraversed(xs, ys, phis, XYbounds_, planner_open_space_config_);
        find_end = true;
        break;
      }
    }
  }
  AINFO << "find_end: " << find_end;
  return find_end;
}

bool HybridAStarBidirectional::IsNearGoal(std::shared_ptr<Node3d> node) {
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

bool HybridAStarBidirectional::ExpandNode(std::shared_ptr<Node3d> current_node,
                                          const bool use_pure_astar,
                                          const hdmap::Path* nearby_path,
                                          bool is_forward_search) {
  auto& open_set = is_forward_search ? open_set_ : open_set_rev_;
  auto& open_pq = is_forward_search ? open_pq_ : open_pq_rev_;
  auto& close_set = is_forward_search ? close_set_ : close_set_rev_;
  size_t explored_node_num = 0;
  double astar_start_time = Clock::NowInSeconds();

  // insert current node into close list
  close_set.emplace(current_node->GetIntIndex(), current_node);

  for (size_t i = 0; i < next_node_num_; ++i) {
    std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);
    if (nullptr == next_node) {
      continue;
    }
    if (close_set.find(next_node->GetIntIndex()) != close_set.end()) {
      continue;
    }
    if (!ValidityCheckBidirection(next_node)) {
      continue;
    }
    if (is_forward_search) {
      CalculateBidirectionNodeCost(astar_start_time, current_node, next_node,
                                   nearby_path);
    } else {
      CalculateBidirectionNodeCostRev(astar_start_time, current_node, next_node,
                                      nearby_path);
    }

    if (open_set.find(next_node->GetIntIndex()) == open_set.end()) {
      explored_node_num++;
      open_set.emplace(next_node->GetIntIndex(), next_node);
      open_pq.emplace(next_node->GetIntIndex(), next_node->GetCost());
    }
  }
  if (force_forward_init_ > 0) {
    --force_forward_init_;
  }
  return true;
}

// check whether the node is valid
bool HybridAStarBidirectional::ValidityCheck(std::shared_ptr<Node3d> node,
                                             bool is_first_node) {
  CHECK_NOTNULL(node);
  CHECK_GT(node->GetStepSize(), 0U);

  size_t node_step_size = node->GetStepSize();
  const auto& traversed_x = node->GetXs();
  const auto& traversed_y = node->GetYs();
  const auto& traversed_phi = node->GetPhis();
  size_t check_start_index = 0;
  if (1 == node_step_size) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

  for (size_t i = check_start_index; i < node_step_size; ++i) {
    if (traversed_x[i] > XYbounds_[1] || traversed_x[i] < XYbounds_[0] ||
        traversed_y[i] > XYbounds_[3] || traversed_y[i] < XYbounds_[2]) {
      AERROR << " out off XYbounds_, return false!";
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

    const auto& bounding_box = Node3d::GetBoundingBoxWithBuffer(
        vehicle_param_, traversed_x[i], traversed_y[i], traversed_phi[i],
        astar_long_buffer_, astar_lat_buffer_);
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
          double coll_node_heading =
              NormalizeAngle(traversed_phi[i] + rotate_angle_);
          ADEBUG << "collision_node: [" << coll_node.x() << ", "
                 << coll_node.y() << ", "
                 << coll_node_heading * common::util::RAD2ANG << "]";
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

bool HybridAStarBidirectional::ValidityCheckBidirection(
    std::shared_ptr<Node3d> node) {
  CHECK_NOTNULL(node);
  CHECK_GT(node->GetStepSize(), 0U);

  if (obstacles_linesegments_vec_.empty()) {
    return true;
  }

  size_t node_step_size = node->GetStepSize();
  const auto& traversed_x = node->GetXs();
  const auto& traversed_y = node->GetYs();
  const auto& traversed_phi = node->GetPhis();

  size_t check_start_index = 0;
  if (1 == node_step_size) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

  for (size_t i = check_start_index; i < node_step_size; ++i) {
    if (traversed_x[i] > XYbounds_[1] || traversed_x[i] < XYbounds_[0] ||
        traversed_y[i] > XYbounds_[3] || traversed_y[i] < XYbounds_[2]) {
      ADEBUG << "out off XYbounds_, return false!";
      return false;
    }
    const auto& bounding_box = Node3d::GetBoundingBoxWithBuffer(
        vehicle_param_, traversed_x[i], traversed_y[i], traversed_phi[i],
        astar_long_buffer_, astar_lat_buffer_);
    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
      for (const common::math::LineSegment2d& linesegment :
           obstacle_linesegments) {
        if (bounding_box.HasOverlap(linesegment)) {
          return false;
        }
      }
    }
  }
  return true;
}

std::shared_ptr<Node3d> HybridAStarBidirectional::LoadRSPinCS(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
    std::shared_ptr<Node3d> current_node, const bool is_forward_search) {
  std::shared_ptr<Node3d> end_node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, planner_open_space_config_));
  end_node->SetPre(current_node);
  auto& close_set = is_forward_search ? close_set_ : close_set_rev_;
  close_set.emplace(end_node->GetIntIndex(), end_node);
  return end_node;
}

bool HybridAStarBidirectional::IsStartNode(std::shared_ptr<Node3d> node) {
  return node->GetPreNode() == nullptr;
}

double HybridAStarBidirectional::CalculateDirectionalDistance(
    std::shared_ptr<Node3d> node) {
  if (nullptr == node->GetPreNode()) {
    return 0.0;
  }

  double total_distance = 0.0;
  bool current_direction = node->GetDirec();
  std::shared_ptr<Node3d> current = node;
  // backtrace until direction change or reach start node
  while (current != nullptr) {
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

std::shared_ptr<Node3d> HybridAStarBidirectional::GenerateStraightNode(
    std::shared_ptr<Node3d> current_node, bool next_direction) {
  double traveled_distance = next_direction ? sample_step_size_ : -sample_step_size_;
  double total_arc = xy_grid_resolution_;
  double steering = 0.0;

  std::vector<double> intermediate_x;
  std::vector<double> intermediate_y;
  std::vector<double> intermediate_phi;

  double last_x = current_node->GetX();
  double last_y = current_node->GetY();
  double last_phi = current_node->GetPhi();

  intermediate_x.emplace_back(last_x);
  intermediate_y.emplace_back(last_y);
  intermediate_phi.emplace_back(last_phi);

  for (size_t i = 0; i < total_arc / sample_step_size_; ++i) {
    const double next_phi = last_phi;
    const double next_x = last_x + traveled_distance * std::cos(next_phi);
    const double next_y = last_y + traveled_distance * std::sin(next_phi);
    if (next_x > XYbounds_[1] || next_x < XYbounds_[0] ||
        next_y > XYbounds_[3] || next_y < XYbounds_[2]) {
      break;
    }

    intermediate_x.emplace_back(next_x);
    intermediate_y.emplace_back(next_y);
    intermediate_phi.emplace_back(next_phi);
    last_x = next_x;
    last_y = next_y;
  }

  if (intermediate_x.size() <= 1) {
    return nullptr;
  }

  std::shared_ptr<Node3d> next_node =
      std::make_shared<Node3d>(intermediate_x, intermediate_y, intermediate_phi,
                               XYbounds_, planner_open_space_config_);

  next_node->SetPre(current_node);
  next_node->SetDirec(current_node->GetDirec());
  next_node->SetSteer(steering);
  return next_node;
}

std::shared_ptr<Node3d> HybridAStarBidirectional::Next_node_generator(
    std::shared_ptr<Node3d> current_node, size_t next_node_index) {
  double steering = 0.0;
  double traveled_distance = 0.0;

  bool next_direction =
      (next_node_index < static_cast<double>(next_node_num_) * 0.5);
  if (current_node->GetDirec() != next_direction) {
    double current_direction_distance =
        CalculateDirectionalDistance(current_node);
    if (current_direction_distance <
        planner_open_space_config_.warm_start_config()
            .same_direction_force_distance()) {
      AINFO << "Gear switch detected, forcing straight line exploration.";
      return GenerateStraightNode(current_node, next_direction);
    }
  }

  if (next_node_index < static_cast<double>(next_node_num_) * 0.5) {
    steering = -max_front_wheel_angle_ +
               (2.0 * max_front_wheel_angle_ /
                (static_cast<double>(next_node_num_) * 0.5 - 1)) *
                   static_cast<double>(next_node_index);
    traveled_distance = sample_step_size_;
  } else {
    size_t index = next_node_index - next_node_num_ / 2;
    steering = -max_front_wheel_angle_ +
               (2.0 * max_front_wheel_angle_ /
                (static_cast<double>(next_node_num_) * 0.5 - 1)) *
                   static_cast<double>(index);
    traveled_distance = -sample_step_size_;
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
  for (size_t i = 0; i < arc / sample_step_size_; ++i) {
    const double next_phi = common::math::NormalizeAngle(
        last_phi +
        traveled_distance * std::tan(steering) / vehicle_param_.wheel_base());
    const double next_x = last_x + traveled_distance * std::cos(next_phi);
    const double next_y = last_y + traveled_distance * std::sin(next_phi);
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

bool HybridAStarBidirectional::CalcRSHeutisticCost(
    const std::shared_ptr<Node3d> next_node,
    const std::shared_ptr<Node3d> goal_node,
    std::shared_ptr<ReedSheppPath> rs_path, double& rs_cost) {
  double relaxed_rs = 0.0;
  const double dx = next_node->GetX() - goal_node->GetX();
  const double dy = next_node->GetY() - goal_node->GetY();
  const double euclidean_dist = std::sqrt(dx * dx + dy * dy);
  if (nullptr == reed_shepp_generator_) {
    AERROR << "reed_shepp_generator is nullptr.";
    return false;
  }
  AINFO << "get_openspace_reason: "
        << static_cast<int>(OpenspaceCommon::get_openspace_reason());
  if (OpenspaceCommon::get_openspace_reason() !=
          planning::OpenspaceReason::OPERATION &&
      reed_shepp_generator_->ShortestRSP(next_node, goal_node, rs_path)) {
    relaxed_rs = rs_path->total_length;
  } else {
    relaxed_rs = euclidean_dist;
  }

  const double w = std::min(
      euclidean_dist / open_space_bidirection_config_.heuristic_switch_dist(),
      1.0);
  ADEBUG << "weight: " << w << ", relaxed_rs: " << relaxed_rs
         << ", euclidean_dist: " << euclidean_dist;
  rs_cost = w * relaxed_rs + (1.0 - w) * euclidean_dist;
  return true;
}

void HybridAStarBidirectional::CalculateBidirectionNodeCostRev(
    const double& astar_start_time, std::shared_ptr<Node3d> current_node,
    std::shared_ptr<Node3d> next_node, const hdmap::Path* nearby_path) {
  next_node->SetTrajCost(current_node->GetTrajCost() +
                         TrajCost(current_node, next_node, false));

  double rs_cost = 0.0;
  std::shared_ptr<ReedSheppPath> shortest_rs_path =
      std::make_shared<ReedSheppPath>();
  if (CalcRSHeutisticCost(next_node, start_node_, shortest_rs_path, rs_cost)) {
    ADEBUG << "backward search, rs_cost: " << rs_cost;
  }

  double lateral_cost = 0.0;
  if (nearby_path && nearby_path->length() > common::util::kMathEpsilon) {
    Vec2d point(next_node->GetX(), next_node->GetY());
    point.SelfRotate(rotate_angle_);
    point += translate_origin_;
    double s = 0.0, l = 0.0;
    nearby_path->GetNearestPoint(point, &s, &l);
    lateral_cost = std::fabs(l);
  }

  double heading_cost = 0.0;
  if (nearby_path && nearby_path->length() > common::util::kMathEpsilon) {
    Vec2d point(next_node->GetX(), next_node->GetY());
    point.SelfRotate(rotate_angle_);
    point += translate_origin_;
    double s = 0.0, l = 0.0;
    nearby_path->GetNearestPoint(point, &s, &l);
    auto smooth_point = nearby_path->GetSmoothPoint(s);
    double hd = NormalizeAngle(smooth_point.heading() - next_node->GetPhi());
    double abs_hd = std::fabs(hd);
    heading_cost = std::min(abs_hd, std::fabs(M_PI - abs_hd));
  } else {
    double hd = NormalizeAngle(start_node_->GetPhi() - next_node->GetPhi());
    double abs_hd = std::fabs(hd);
    heading_cost = std::min(abs_hd, std::fabs(M_PI - abs_hd));
  }

  double raw_obs = NearObstacleHeuristic(next_node);
  double current2end_dis =
      std::hypot((current_node->GetX() - end_node_->GetX()),
                 (current_node->GetY() - end_node_->GetY()));
  double obs_cost = current2end_dis > kNearEndDis ? raw_obs : 0.0;

  double obs_distance_cost = ObstacleCost(next_node, obstacle_points_);
  ADEBUG << "backward search, obs_distance_cost: " << obs_distance_cost;
  double dp_map_weight = 1.0;

  double optimal_path_cost = 0.0;
  optimal_path_cost +=
      open_space_bidirection_config_.search_cost_config().rs_weight() * rs_cost;
  optimal_path_cost +=
      open_space_bidirection_config_.search_cost_config().lateral_weight() *
      lateral_cost;
  optimal_path_cost +=
      open_space_bidirection_config_.search_cost_config().heading_weight() *
      heading_cost;
  optimal_path_cost +=
      open_space_bidirection_config_.search_cost_config().obs_weight() *
      obs_cost;
  optimal_path_cost += dp_map_weight * obs_distance_cost;

  next_node->SetHeuCost(optimal_path_cost);
  ADEBUG << "Calculation of barkward search, node cost - id: "
         << next_node->GetIndexString()
         << ", total_cost: " << next_node->GetCost()
         << ", traj_cost: " << next_node->GetTrajCost()
         << ", heuistic_cost: " << optimal_path_cost << "(" << rs_cost << ", "
         << lateral_cost << ", " << heading_cost << ", " << obs_cost << ", "
         << obs_distance_cost << ")";
  return;
}

void HybridAStarBidirectional::CalculateBidirectionNodeCost(
    const double& astar_start_time, std::shared_ptr<Node3d> current_node,
    std::shared_ptr<Node3d> next_node, const hdmap::Path* nearby_path) {
  next_node->SetTrajCost(current_node->GetTrajCost() +
                         TrajCost(current_node, next_node));

  double rs_cost = 0.0;
  std::shared_ptr<ReedSheppPath> shortest_rs_path =
      std::make_shared<ReedSheppPath>();
  CalcRSHeutisticCost(next_node, end_node_, shortest_rs_path, rs_cost);

  double lateral_cost = 0.0;
  if (nearby_path && nearby_path->length() > common::util::kMathEpsilon) {
    Vec2d point(next_node->GetX(), next_node->GetY());
    point.SelfRotate(rotate_angle_);
    point += translate_origin_;
    double s = 0.0, l = 0.0;
    nearby_path->GetNearestPoint(point, &s, &l);
    lateral_cost = std::fabs(l);
  }

  double heading_cost = 0.0;
  if (nearby_path && nearby_path->length() > common::util::kMathEpsilon) {
    Vec2d point(next_node->GetX(), next_node->GetY());
    point.SelfRotate(rotate_angle_);
    point += translate_origin_;
    double s = 0.0, l = 0.0;
    nearby_path->GetNearestPoint(point, &s, &l);
    auto smooth_point = nearby_path->GetSmoothPoint(s);
    double hd = NormalizeAngle(smooth_point.heading() - next_node->GetPhi());
    double abs_hd = std::fabs(hd);
    heading_cost = std::min(abs_hd, std::fabs(M_PI - abs_hd));
  } else {
    double hd = NormalizeAngle(end_node_->GetPhi() - next_node->GetPhi());
    double abs_hd = std::fabs(hd);
    heading_cost = std::min(abs_hd, std::fabs(M_PI - abs_hd));
  }

  double raw_obs = NearObstacleHeuristic(next_node);
  double current2end_dis =
      std::hypot((current_node->GetX() - end_node_->GetX()),
                 (current_node->GetY() - end_node_->GetY()));
  double obs_cost = current2end_dis > kNearEndDis ? raw_obs : 0.0;

  double obs_distance_cost = ObstacleCost(next_node, obstacle_points_);
  ADEBUG << "forward search, obs_distance_cost: " << obs_distance_cost;
  double dp_map_weight = 1.0;

  double optimal_path_cost = 0.0;
  optimal_path_cost +=
      open_space_bidirection_config_.search_cost_config().rs_weight() * rs_cost;
  optimal_path_cost +=
      open_space_bidirection_config_.search_cost_config().lateral_weight() *
      lateral_cost;
  optimal_path_cost +=
      open_space_bidirection_config_.search_cost_config().heading_weight() *
      heading_cost;
  optimal_path_cost +=
      open_space_bidirection_config_.search_cost_config().obs_weight() *
      obs_cost;
  optimal_path_cost += dp_map_weight * obs_distance_cost;

  next_node->SetHeuCost(optimal_path_cost);
  return;
}

double HybridAStarBidirectional::TrajCost(
    std::shared_ptr<Node3d> current_node,
    std::shared_ptr<Node3d> next_node,
    const bool is_forward_search) {
  auto start_node =
      GetBidirectionalStartNode(is_forward_search, start_node_, end_node_);
  auto end_node =
      GetBidirectionalEndNode(is_forward_search, start_node_, end_node_);

  double piecewise_cost = 0.0;
  const double segment_length =
      static_cast<double>(next_node->GetStepSize() - 1) * sample_step_size_;

  if (next_node->GetDirec()) {
    // forward
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      sample_step_size_ * traj_forward_penalty_;
  } else {
    if (force_forward_init_ > 0) {
      piecewise_cost = force_forward_init_ * kPenalizeForward;
    }
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      sample_step_size_ * traj_back_penalty_;
  }

  ADEBUG << "1、piecewise_cost: " << piecewise_cost;
  if (current_node->GetDirec() != next_node->GetDirec()) {
    piecewise_cost += traj_gear_switch_penalty_;
    ADEBUG << "2、gear_changed_cost: " << traj_gear_switch_penalty_;
  }

  piecewise_cost +=
      traj_steer_penalty_ * std::abs(next_node->GetSteer()) * segment_length;

  ADEBUG << "3、steering_cost: "
         << traj_steer_penalty_ * std::abs(next_node->GetSteer()) *
                segment_length;

  piecewise_cost +=
      traj_steer_change_penalty_ *
      std::abs(next_node->GetSteer() - current_node->GetSteer()) *
      segment_length;

  ADEBUG << "4、steering_changed_cost: "
         << traj_steer_change_penalty_ *
                std::abs(next_node->GetSteer() - current_node->GetSteer()) *
                segment_length;

  ADEBUG << "5、The sum of g-cost from the current node to the next node: "
         << piecewise_cost;
  return piecewise_cost;
}

double HybridAStarBidirectional::HoloObstacleHeuristic(
    std::shared_ptr<Node3d> next_node) {
  return grid_a_star_heuristic_generator_->CheckDpMap(next_node->GetX(),
                                                      next_node->GetY());
}

double HybridAStarBidirectional::NearObstacleHeuristic(
    std::shared_ptr<Node3d> node) {
  CHECK_NOTNULL(node);
  CHECK_GT(node->GetStepSize(), 0U);

  size_t node_step_size = node->GetStepSize();
  const auto& traversed_x = node->GetXs();
  const auto& traversed_y = node->GetYs();
  const auto& traversed_phi = node->GetPhis();

  size_t check_start_index = 0;
  if (1 == node_step_size) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

  for (size_t i = check_start_index; i < node_step_size; ++i) {
    if (traversed_x[i] > XYbounds_[1] || traversed_x[i] < XYbounds_[0] ||
        traversed_y[i] > XYbounds_[3] || traversed_y[i] < XYbounds_[2]) {
      AERROR << "out off XYbounds.";
      return 1.0;
    }
  }
  if (obstacles_linesegments_vec_.empty()) {
    return 0.0;
  }
  double nearest_dist = std::numeric_limits<double>::max();
  for (size_t i = check_start_index; i < node_step_size; ++i) {
    const auto& scan_box = Node3d::GetObsCostBoxWithBuffer(
        vehicle_param_, traversed_x[i], traversed_y[i], traversed_phi[i],
        extra_length_, extra_width_);
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
    return 0.0;
  } else {
    return 1.0 - (nearest_dist / near_obs_dist_);
  }
  return 0.0;
}

bool HybridAStarBidirectional::ConnectResult(std::shared_ptr<Node3d> f,
                                             std::shared_ptr<Node3d> b,
                                             HybridAStartResult* final_result) {
  if (!f || !b) {
    AERROR << "error, Invalid encounter node";
    return false;
  }
  std::vector<double> forward_x, forward_y, forward_phi;
  std::shared_ptr<Node3d> current = f;
  while (current->GetPreNode() != nullptr) {
    std::vector<double> x_vec = current->GetXs();
    std::vector<double> y_vec = current->GetYs();
    std::vector<double> phi_vec = current->GetPhis();
    if (x_vec.empty() || y_vec.empty() || phi_vec.empty()) {
      AERROR << "error, result size check failed";
      return false;
    }
    if (x_vec.size() != y_vec.size() || x_vec.size() != phi_vec.size()) {
      AERROR << "error, states sizes are not equal";
      return false;
    }

    std::reverse(x_vec.begin(), x_vec.end());
    std::reverse(y_vec.begin(), y_vec.end());
    std::reverse(phi_vec.begin(), phi_vec.end());
    x_vec.pop_back();
    y_vec.pop_back();
    phi_vec.pop_back();
    forward_x.insert(forward_x.end(), x_vec.begin(), x_vec.end());
    forward_y.insert(forward_y.end(), y_vec.begin(), y_vec.end());
    forward_phi.insert(forward_phi.end(), phi_vec.begin(), phi_vec.end());
    current = current->GetPreNode();
  }
  std::reverse(forward_x.begin(), forward_x.end());
  std::reverse(forward_y.begin(), forward_y.end());
  std::reverse(forward_phi.begin(), forward_phi.end());

  std::vector<double> backward_x, backward_y, backward_phi;
  current = b;
  while (current->GetPreNode() != nullptr) {
    std::vector<double> x_vec = current->GetXs();
    std::vector<double> y_vec = current->GetYs();
    std::vector<double> phi_vec = current->GetPhis();

    if (x_vec.empty() || y_vec.empty() || phi_vec.empty()) {
      AERROR << "error, result size check failed";
      return false;
    }
    if (x_vec.size() != y_vec.size() || x_vec.size() != phi_vec.size()) {
      AERROR << "error, states sizes are not equal";
      return false;
    }
    std::reverse(x_vec.begin(), x_vec.end());
    std::reverse(y_vec.begin(), y_vec.end());
    std::reverse(phi_vec.begin(), phi_vec.end());
    x_vec.pop_back();
    y_vec.pop_back();
    phi_vec.pop_back();
    backward_x.insert(backward_x.end(), x_vec.begin(), x_vec.end());
    backward_y.insert(backward_y.end(), y_vec.begin(), y_vec.end());
    backward_phi.insert(backward_phi.end(), phi_vec.begin(), phi_vec.end());
    current = current->GetPreNode();
  }

  (*final_result).x = forward_x;
  (*final_result).y = forward_y;
  (*final_result).phi = forward_phi;

  if (!backward_x.empty()) {
    size_t start_index = 1;
    if (forward_x.empty() || std::abs(forward_x.back() - backward_x[0]) > 0.1 ||
        std::abs(forward_y.back() - backward_y[0]) > 0.1) {
      start_index = 0;
    }

    for (size_t i = start_index; i < backward_x.size(); ++i) {
      (*final_result).x.emplace_back(backward_x[i]);
      (*final_result).y.emplace_back(backward_y[i]);
      (*final_result)
          .phi.emplace_back(OpenspaceUtil::HeadingReversal(backward_phi[i]));
    }
  }
  AINFO << "Path stitching completed, size: " << (*final_result).x.size();

  if (!GetTemporalProfile(final_result)) {
    AERROR << "error, GetSpeedProfile from Hybrid Astar path failed";
    return false;
  }

  if (final_result->x.size() != final_result->y.size() ||
      final_result->x.size() != final_result->phi.size()) {
    return false;
  }
  if (final_result->a.size() != final_result->steer.size() ||
      final_result->x.size() - final_result->a.size() != 1 ||
      final_result->x.size() != final_result->v.size()) {
    AERROR << "errror, control sizes not equal or not right";
    AERROR << "errror,  acceleration_size: " << final_result->a.size()
           << ",  steer_size: " << final_result->steer.size()
           << ",  x_size: " << final_result->x.size()
           << ", v_size: " << final_result->v.size();
    return false;
  }
  return true;
}

bool HybridAStarBidirectional::GetResult(HybridAStartResult* result,
                                         bool is_normal_order) {
  std::shared_ptr<Node3d> current_node = final_node_;

  std::vector<double> hybrid_a_x;
  std::vector<double> hybrid_a_y;
  std::vector<double> hybrid_a_phi;
  while (current_node->GetPreNode() != nullptr) {
    std::vector<double> x = current_node->GetXs();
    std::vector<double> y = current_node->GetYs();
    std::vector<double> phi = current_node->GetPhis();
    if (x.empty() || y.empty() || phi.empty()) {
      AERROR << "error, result size check failed";
      return false;
    }
    if (x.size() != y.size() || x.size() != phi.size()) {
      AERROR << "error, states sizes are not equal";
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
  if (is_normal_order) {
    std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
    std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
    std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
  }
  (*result).x = hybrid_a_x;
  (*result).y = hybrid_a_y;
  (*result).phi = hybrid_a_phi;

  // compute v, a, steer
  if (!GetTemporalProfile(result)) {
    AERROR << "error, GetSpeedProfile from Hybrid Astar path failed";
    return false;
  }

  if (result->x.size() != result->y.size() ||
      result->x.size() != result->phi.size()) {
    AERROR << "error, state sizes not equal, "
           << "result->x.size(): " << result->x.size() << "result->y.size()"
           << result->y.size() << "result->phi.size()" << result->phi.size();
    return false;
  }
  if (result->a.size() != result->steer.size() ||
      result->x.size() - result->a.size() != 1 ||
      result->x.size() != result->v.size()) {
    AERROR << "error, control sizes not equal or not right";
    AERROR << "error,  acceleration_size: " << result->a.size()
           << ",  steer_size: " << result->steer.size()
           << ", x_size: " << result->x.size()
           << ", v_size: " << result->v.size();
    return false;
  }

  return true;
}

bool HybridAStarBidirectional::GenerateSpeedAcceleration(
    HybridAStartResult* result) {
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    AERROR << "error, result size check when generating speed and acceleration failed";
    return false;
  }
  size_t x_size = result->x.size();
  double traverled_distance = sample_step_size_;
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
    traverled_distance = sample_step_size_ * 0.5;
    result->x = x;
    result->y = y;
    result->phi = phi;
  }
  x_size = result->x.size();

  double target_v =
      planner_open_space_config_.distance_approach_config().max_speed_forward();
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
      t = std::fabs((v1 - v0) / target_a) +
          std::fabs((traverled_distance - s) / v1);
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

  for (size_t i = 0; i + 1 < x_size; ++i) {
    double discrete_steer = (result->phi[i + 1] - result->phi[i]) *
                            (vehicle_param_.wheel_base() / 2.0) /
                            traverled_distance;
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
bool HybridAStarBidirectional::TrajectoryPartition(
    const HybridAStartResult& result,
    std::vector<HybridAStartResult>* partitioned_result) {
  const auto& x = result.x;
  const auto& y = result.y;
  const auto& phi = result.phi;
  if (x.empty() || y.empty() || phi.empty()) {
    AERROR << "error, result is empty, return false.";
    return false;
  }
  if (x.size() != y.size() || x.size() != phi.size()) {
    AERROR << "error, states sizes are not equal when do trajectory "
              "partitioning of "
              "Hybrid A Star result";
    return false;
  }

  AERROR << "result.x_size: " << result.x.size()
         << ", y_size: " << result.y.size()
         << ", phi_size: " << result.phi.size();

  size_t horizon = x.size();
  partitioned_result->clear();
  partitioned_result->emplace_back();
  auto* current_traj = &(partitioned_result->back());
  // heading_angle: flu
  double heading_angle = phi.front();
  const Vec2d init_tracking_vector(x[1] - x[0], y[1] - y[0]);
  double tracking_angle = init_tracking_vector.Angle();
  // true - D, false - R
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
  AINFO << "partitioned_result_size: " << partitioned_result->size();

  // Retrieve v, a and steer from path
  // teb does not use speed profile from hybrid_a_star.
  // should use FLAGS_enable_use_teb to pass this process.
  for (auto& result : *partitioned_result) {
    if (!GenerateSpeedAcceleration(&result)) {
      AERROR << "error, GenerateSpeedAcceleration failed";
      return false;
    }
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  ADEBUG << "speed profile total time: " << diff.count() * 1000.0 << " ms.";
  return true;
}

bool HybridAStarBidirectional::GetTemporalProfile(HybridAStartResult* result) {
  std::vector<HybridAStartResult> partitioned_results;
  if (!TrajectoryPartition(*result, &partitioned_results)) {
    AERROR << "error, TrajectoryPartition failed";
    return false;
  }

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

bool HybridAStarBidirectional::Plan(
    const std::vector<std::vector<Vec2d>>& world_obstacles_vertices_vec,
    double sx, double sy, double sphi, double ex, double ey, double ephi,
    double rotate_angle, const Vec2d& translate_origin,
    const std::vector<double>& XYbounds, const std::vector<Obstacle>& obstacles,
    const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec,
    HybridAStartResult* result, bool use_pure_astar,
    bool* start_collision_flag) {
  return BidirectionalPlan(world_obstacles_vertices_vec, sx, sy, sphi, ex, ey,
                           ephi, rotate_angle, translate_origin, XYbounds,
                           obstacles, obstacles_vertices_vec, result, nullptr,
                           use_pure_astar, start_collision_flag);
}

bool HybridAStarBidirectional::BidirectionalPlan(
    const std::vector<std::vector<Vec2d>>& world_obstacles_vertices_vec,
    double sx, double sy, double sphi, double ex, double ey, double ephi,
    double rotate_angle, const Vec2d& translate_origin,
    const std::vector<double>& XYbounds, const std::vector<Obstacle>& obstacles,
    const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec,
    HybridAStartResult* result, const hdmap::Path* nearby_path,
    bool use_pure_astar, bool* start_collision_flag) {
  // update world --> map
  rotate_angle_ = rotate_angle;
  translate_origin_ = translate_origin;
  grid_a_star_heuristic_generator_->SetRotateAngle(rotate_angle);
  grid_a_star_heuristic_generator_->SetOriginPoint(translate_origin);

  start_collision_flag_ = false;
  if (start_collision_flag != nullptr) {
    *start_collision_flag = start_collision_flag_;
  }

  if (is_hybrid_debug_) {
    CreateCyber();
  }

  if (FLAGS_start_collision_buff_adjustment) {
    astar_first_long_buffer_ = planner_open_space_config_.warm_start_config()
                                   .astar_first_long_buffer();
    astar_first_lat_buffer_ =
        planner_open_space_config_.warm_start_config().astar_first_lat_buffer();
  } else {
    if (!last_success_) {
      AERROR << "reduce buffer because last plan failed.";
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

  // A set of line segments composed of all obstacles:
  // obstacles_linesegments_vec
  std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec;
  ConvertPointsToLineSegments(obstacles_vertices_vec,
                              obstacles_linesegments_vec);
  obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);
  XYbounds_ = XYbounds;

  Vec2d xy0(XYbounds_[0], XYbounds_[2]);
  Vec2d xy1(XYbounds_[1], XYbounds_[3]);
  xy0.SelfRotate(rotate_angle_);
  xy0 += translate_origin_;
  xy1.SelfRotate(rotate_angle_);
  xy1 += translate_origin_;
  AINFO << "XYboundsMap x0:" << xy0.x();
  AINFO << "XYboundsMap x1:" << xy1.x();
  AINFO << "XYboundsMap y0:" << xy0.y();
  AINFO << "XYboundsMap y1:" << xy1.y();
  AINFO << "XYbounds x0:" << XYbounds_[0];
  AINFO << "XYbounds x1:" << XYbounds_[1];
  AINFO << "XYbounds y0:" << XYbounds_[2];
  AINFO << "XYbounds y1:" << XYbounds_[3];

  start_node_.reset(
      new Node3d({sx}, {sy}, {sphi}, XYbounds_, planner_open_space_config_));
  end_node_.reset(
      new Node3d({ex}, {ey}, {ephi}, XYbounds_, planner_open_space_config_));

  double sphi_rev = NormalizeAngle(sphi + M_PI);
  start_node_rev_.reset(new Node3d({sx}, {sy}, {sphi_rev}, XYbounds_,
                                   planner_open_space_config_));

  double ephi_rev = NormalizeAngle(ephi + M_PI);
  end_node_rev_.reset(new Node3d({ex}, {ey}, {ephi_rev}, XYbounds_,
                                 planner_open_space_config_));

  AINFO << "forawrd search, start: (" << sx << ", " << sy << ", "
        << sphi * 180 / M_PI << "°)";
  AINFO << "backward search, start: (" << sx << ", " << sy << ", "
        << sphi_rev * 180 / M_PI << "°)";
  AINFO << "forawrd search, end: (" << ex << ", " << ey << ", "
        << ephi * 180 / M_PI << "°)";
  AINFO << "backward search, end: (" << ex << ", " << ey << ", "
        << ephi_rev * 180 / M_PI << "°)";

  if (is_hybrid_debug_) {
    WriteObsLine2dData();      // publish obstacle list
    WriteXYbounds();           // publish ROI
    WriteStartEndPointData();  // publish end pose
  }

  astar_long_buffer_ = astar_first_long_buffer_;
  astar_lat_buffer_ = astar_first_lat_buffer_;

  if (!ValidityCheck(start_node_, true)) {
    if (FLAGS_start_collision_buff_adjustment) {
      start_collision_flag_ = true;
      if (start_collision_flag != nullptr) {
        *start_collision_flag = start_collision_flag_;
      }
      if (!ValidityCheck(start_node_, true)) {
        AERROR << "error, start_node in collision with obstacles(Reduce buff)";
        ha_debug_.insert(OpenspaceCommon::HybridAStarDebugStatus::START_COLLISION);
        return false;
      }

    } else {
      ha_debug_.insert(
          OpenspaceCommon::HybridAStarDebugStatus::START_COLLISION);
      return false;
    }
  }

  if (nullptr == nearby_path && !ValidityCheck(end_node_)) {
    ha_debug_.insert(OpenspaceCommon::HybridAStarDebugStatus::END_COLLISION);
    return false;
  }

  if (enable_bidirectional_search_) {
    bool bidirectional_success =
        BidirectionalSearch(use_pure_astar, nearby_path);
    if (!bidirectional_success) {
      AERROR << "error, bidirectional search failed, use normal search as fallback.";
      ResetSearchState();
      enable_bidirectional_search_ = false;
      if (!PerformSingleSearch(use_pure_astar, nearby_path, sx, sy, sphi, ex,
                               ey, ephi, result)) {
        return false;
      }
    } else {
      ha_debug_.insert(
          OpenspaceCommon::HybridAStarDebugStatus::BIDIRECTION_SAERCH_SUCCESS);
      if (enable_bidirectional_search_ && meet_node_forward_ &&
          meet_node_backward_) {
        if (!ConnectResult(meet_node_forward_, meet_node_backward_, result)) {
          AERROR
              << "bidirectional search get traj result failed, return false!!!";
          ha_debug_.insert(OpenspaceCommon::HybridAStarDebugStatus::
                               GET_BIDIRECTION_RESULT_FAILED);
          return false;
        }
      }
    }
  } else {
    if (!PerformSingleSearch(use_pure_astar, nearby_path, sx, sy, sphi, ex, ey,
                             ephi, result)) {
      return false;
    }
  }

  (*result).result_size = result->x.size();
  (*result).traj_kappa_contraint_ratio = traj_kappa_contraint_ratio_;
  AddHybridAStarParaDebugInfo(result, is_hybrid_debug_);
  last_success_ = true;
  return true;
}

bool HybridAStarBidirectional::PerformSingleSearch(
    const bool use_pure_astar, const hdmap::Path* nearby_path, const double sx,
    const double sy, const double sphi, const double ex, const double ey,
    const double ephi, HybridAStartResult* result) {
  AINFO << __func__;
  if (!Search(use_pure_astar, nearby_path)) {
    AERROR << "ERROR, first search failed, swap start node and "
              "end node, search again";
    start_node_.reset(
        new Node3d({ex}, {ey}, {ephi}, XYbounds_, planner_open_space_config_));
    end_node_.reset(
        new Node3d({sx}, {sy}, {sphi}, XYbounds_, planner_open_space_config_));
    is_change_start_end_ = true;

    if (!Search(use_pure_astar, nullptr)) {
      AERROR << "ERROR, second search failed, return false";
      return false;
    } else {
      if (!GetResult(result, false)) {
        AERROR << "ERROR, GetResult failed";
        ha_debug_.insert(
            OpenspaceCommon::HybridAStarDebugStatus::GET_SINGLE_RESULT_FAILED);
        return false;
      }
    }
  } else {
    if (!GetResult(result, true)) {
      AERROR << "ERROR,GetResult failed";
      ha_debug_.insert(
          OpenspaceCommon::HybridAStarDebugStatus::GET_SINGLE_RESULT_FAILED);
      return false;
    }
  }
  return true;
}

bool HybridAStarBidirectional::BidirectionalSearch(
    const bool use_pure_astar, const hdmap::Path* nearby_path) {
  size_t forward_expand_num = 0;
  size_t backward_expand_num = 0;
  ResetSearchState();

  double astar_start_time = Clock::NowInSeconds();
  AINFO << "enter Bidirectional Search time: " << astar_start_time;
  size_t iteration_count = 0;

  open_set_.emplace(start_node_->GetIntIndex(), start_node_);
  open_pq_.emplace(start_node_->GetIntIndex(), start_node_->GetCost());

  open_set_rev_.emplace(end_node_rev_->GetIntIndex(), end_node_rev_);
  open_pq_rev_.emplace(end_node_rev_->GetIntIndex(), end_node_rev_->GetCost());

  while ((!open_pq_.empty() || !open_pq_rev_.empty()) &&
         iteration_count < max_bidirectional_iterations_) {
    iteration_count++;
    bool expand_forward = true;
    ADEBUG << "open_pq_size: " << open_pq_.size()
           << ", open_pq_rev_size: " << open_pq_rev_.size()
           << ", close_set_size: " << close_set_.size()
           << ", close_set_rev_size: " << close_set_rev_.size();
    if (open_pq_.empty()) {
      expand_forward = false;
    } else if (open_pq_rev_.empty()) {
      expand_forward = true;
    } else {
      expand_forward = (open_pq_.size() <= open_pq_rev_.size());
    }
    ADEBUG << "expand_forward: " << expand_forward;

    if (expand_forward) {
      forward_expand_num++;
      const NodeIndex current_id = open_pq_.top().first;
      open_pq_.pop();
      std::shared_ptr<Node3d> current_node = open_set_[current_id];

      double time_meet_node = Clock::NowInSeconds();
      if (MeetInMiddle(current_node, close_set_rev_, true, nullptr)) {
        AERROR << "using rs path, forward and backward search tree meet";
        return true;
      }
      ADEBUG << "    [time-consuming] for-search start and end can use rs path "
                "connect time: "
             << std::fixed << std::setprecision(9)
             << (Clock::NowInSeconds() - time_meet_node) * 1000;

      double time_bi_goal_node = Clock::NowInSeconds();
      if (IsBidirectionalGoal(current_node, true)) {
        AERROR << "for-search tree is reaching end!";
        return true;
      }
      ADEBUG << "    [time-consuming] for-search node is reach end time: "
             << std::fixed << std::setprecision(9)
             << (Clock::NowInSeconds() - time_bi_goal_node) * 1000;

      double time_expand_node = Clock::NowInSeconds();
      ExpandNode(current_node, use_pure_astar, nearby_path, true);
      ADEBUG << "    [time-consuming] finished expand forward node time: "
             << std::fixed << std::setprecision(9)
             << (Clock::NowInSeconds() - time_expand_node) * 1000;
    } else {
      backward_expand_num ++;
      const NodeIndex current_id = open_pq_rev_.top().first;
      open_pq_rev_.pop();
      std::shared_ptr<Node3d> current_node = open_set_rev_[current_id];

      double time_meet_node = Clock::NowInSeconds();
      if (MeetInMiddle(current_node, close_set_, false, nullptr)) {
        AERROR << "using rs path, backward and forward search tree meet";
        return true;
      }
      ADEBUG
          << "    [time-consuming] back-search start and end can use rs path "
             "connect time: "
          << std::fixed << std::setprecision(9)
          << (Clock::NowInSeconds() - time_meet_node) * 1000;

      double time_bi_goal_node = Clock::NowInSeconds();
      if (IsBidirectionalGoal(current_node, false)) {
        AERROR << "back-search tree is reaching end!";
        return true;
      }
      ADEBUG
          << "    [time-consuming] back-search node is reach end time: "
          << std::fixed << std::setprecision(9)
          << (Clock::NowInSeconds() - time_bi_goal_node) * 1000;

      double time_expand_node = Clock::NowInSeconds();
      ExpandNode(current_node, use_pure_astar, nearby_path, false);
      ADEBUG << "    [time-consuming] finished expand backward node time: "
             << std::fixed << std::setprecision(9)
             << (Clock::NowInSeconds() - time_expand_node) * 1000;
    }

    ADEBUG << "current_time: " << std::fixed << std::setprecision(9)
           << Clock::NowInSeconds() * 1000
           << ", [time-consuming] current time-consuming: " << std::fixed
           << std::setprecision(9)
           << (Clock::NowInSeconds() - astar_start_time) * 1000
           << ", expand time from start: " << forward_expand_num
           << ", expand time from end: " << backward_expand_num;
    if (Clock::NowInSeconds() - astar_start_time >
        bidirection_astar_search_time_threshold_) {
      AERROR << "Bidirectional search timeout, iteration_count: "
             << iteration_count << ", [time-consuming] time: "
             << Clock::NowInSeconds() - astar_start_time
             << ", threshold: " << bidirection_astar_search_time_threshold_
             << ", return false.";
      AERROR << "timeout, open_pq_size: " << open_pq_.size()
             << ", open_pq_rev_size: " << open_pq_rev_.size();
      ha_debug_.insert(OpenspaceCommon::HybridAStarDebugStatus::PLAN_OVERTIME);
      return false;
    }
  }

  if (iteration_count > max_bidirectional_iterations_) {
    AERROR << "Bidirectional search has exhausted the maximum number of "
              "iterations and failed to find a path, return false";
    ha_debug_.insert(
        OpenspaceCommon::HybridAStarDebugStatus::PLAN_ITERATION_EXCEEDED);
  }

  if (open_pq_.empty() && open_pq_rev_.empty()) {
    AERROR << "ERROR, Hybrid A searching , open_set and "
              "open_set_reverse ran out, return false.";
    ha_debug_.insert(OpenspaceCommon::HybridAStarDebugStatus::SEARCH_FAIL);
  }
  return false;
}

bool HybridAStarBidirectional::Search(const bool use_pure_astar,
                                      const hdmap::Path* nearby_path) {
  open_set_.clear();
  close_set_.clear();
  open_pq_ = decltype(open_pq_)();
  final_node_ = nullptr;

  open_set_.emplace(start_node_->GetIntIndex(), start_node_);
  open_pq_.emplace(start_node_->GetIntIndex(), start_node_->GetCost());

  size_t explored_node_num = 0;
  size_t search_rs_num = 0;
  size_t run_node_num = 0;
  double astar_start_time = Clock::NowInSeconds();
  double rs_time = 0.0;
  double cyber_time = 0.0;

  ADEBUG << "next_node_num: " << next_node_num_;
  ADEBUG << "astar_max_time_first: " << astar_max_time_first_
         << ", astar_max_time_second: " << astar_max_time_second_;
  ADEBUG << "force_forward_init_" << force_forward_init_;

  while (!open_pq_.empty()) {
    ++run_node_num;

    // take out the lowest cost neighboring node
    const NodeIndex current_id = open_pq_.top().first;
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
    close_set_.emplace(current_node->GetIntIndex(), current_node);

    for (size_t i = 0; i < next_node_num_; ++i) {
      std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);
      if (nullptr == next_node) {
        continue;
      }
      // check if the node is already in the close set
      if (close_set_.find(next_node->GetIntIndex()) != close_set_.end()) {
        continue;
      }
      if (!ValidityCheck(next_node)) {
        continue;
      }
      if (open_set_.find(next_node->GetIntIndex()) == open_set_.end()) {
        if (is_hybrid_debug_) {
          hybrid_node.emplace_back(*next_node);
        }

        explored_node_num++;
        CalculateBidirectionNodeCost(astar_start_time, current_node, next_node,
                                     nearby_path);
        open_set_.emplace(next_node->GetIntIndex(), next_node);
        open_pq_.emplace(next_node->GetIntIndex(), next_node->GetCost());
      }
    }
    if (force_forward_init_ > 0) {
      --force_forward_init_;
    }

    if (is_hybrid_debug_) {
      const double cyber_start_time = Clock::NowInSeconds();
      WriteData(hybrid_node);
      size_t hybrid_debug_next_node_sleep_ms =
          planner_open_space_config_.warm_start_config()
              .hybrid_debug_next_node_sleep_ms();
      DelayMilliSecond(hybrid_debug_next_node_sleep_ms);
      const double cyber_end_time = Clock::NowInSeconds();
      cyber_time += cyber_end_time - cyber_start_time;
    }

    if (!is_map_boundary_) {
      // !is_map_boundary_ means large boundary, cost more time
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
    AERROR << "error, Hybrid A searching return null ptr(open_set ran out)";
    ha_debug_.insert(
          OpenspaceCommon::HybridAStarDebugStatus::SEARCH_FAIL);
    return false;
  }

  return true;
}

void HybridAStarBidirectional::CreateCyber() {
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

void HybridAStarBidirectional::WriteData(
    const std::vector<Node3d>& hybrid_node) {
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

void century::planning::HybridAStarBidirectional::DelayMilliSecond(int msec) {
  std::this_thread::sleep_for(std::chrono::milliseconds(msec));
}

void HybridAStarBidirectional::DelaySecond(int sec) {
  std::this_thread::sleep_for(std::chrono::seconds(sec));
}

void HybridAStarBidirectional::WriteObsLine2dData() {
  century::planning_internal::OpenSpaceDebug open_space_debug;
  open_space_debug.set_is_first_hybrid_debug(false);
  open_space_debug.mutable_hybrid_rs_debug()->set_is_rs_succeed(false);
  open_space_debug.set_is_use_a_star_path(false);
  ADEBUG << "obstacles_linesegments_vec_ size"
         << obstacles_linesegments_vec_.size();

  size_t index = 0;
  for (const auto& obstacles_linesegments : obstacles_linesegments_vec_) {
    size_t vertices_num = obstacles_linesegments.size();
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

  DelayMilliSecond(10);
  size_t send_times = 1;
  for (size_t i = 0; i < send_times; ++i) {
    DelayMilliSecond(10);
    open_space_writer_->Write(open_space_debug);
  }
  DelayMilliSecond(10);
}

void HybridAStarBidirectional::WriteStartEndPointData() {
  common::math::Vec2d world_start_pt, world_end_pt;
  double world_start_heading, world_end_heading;
  common::math::Vec2d start_node(start_node_->GetX(), start_node_->GetY());
  common::math::Vec2d end_node(end_node_->GetX(), end_node_->GetY());
  common::util::Origin2World(translate_origin_, rotate_angle_, start_node,
                             start_node_->GetPhi(), world_start_pt,
                             world_start_heading);
  common::util::Origin2World(translate_origin_, rotate_angle_, end_node,
                             end_node_->GetPhi(), world_end_pt,
                             world_end_heading);

  century::planning_internal::OpenSpaceDebug open_space_debug;
  open_space_debug.set_is_first_hybrid_debug(false);
  open_space_debug.mutable_hybrid_rs_debug()->set_is_rs_succeed(false);
  open_space_debug.set_is_use_a_star_path(false);
  open_space_debug.mutable_hybrid_search_info()
      ->set_is_send_start_end_point(true);

  century::planning_internal::HybridPointDebug hybrid_start_point;
  hybrid_start_point.set_x(world_start_pt.x());
  hybrid_start_point.set_y(world_start_pt.y());
  hybrid_start_point.set_phi(world_start_heading * common::util::RAD2ANG);
  century::planning_internal::HybridPointDebug hybrid_end_point;
  hybrid_end_point.set_x(world_end_pt.x());
  hybrid_end_point.set_y(world_end_pt.y());
  hybrid_end_point.set_phi(world_end_heading * common::util::RAD2ANG);

  auto* search_point_ptr = open_space_debug.mutable_hybrid_search_info();
  search_point_ptr->mutable_hybrid_start_point()->CopyFrom(hybrid_start_point);
  search_point_ptr->mutable_hybrid_end_point()->CopyFrom(hybrid_end_point);

  DelayMilliSecond(10);
  size_t send_times = 2;
  for (size_t i = 0; i < send_times; ++i) {
    DelayMilliSecond(10);
    open_space_writer_->Write(open_space_debug);
  }
  DelayMilliSecond(10);
}

void HybridAStarBidirectional::WriteXYbounds() {
  common::math::Vec2d world_xy0, world_xy1;
  double world_xy0_heading, world_xy1_heading;
  common::math::Vec2d xy0(XYbounds_[0], XYbounds_[2]);
  common::math::Vec2d xy1(XYbounds_[1], XYbounds_[3]);
  century::planning_internal::OpenSpaceDebug open_space_debug;
  open_space_debug.set_is_first_hybrid_debug(false);
  open_space_debug.mutable_hybrid_rs_debug()->set_is_rs_succeed(false);
  open_space_debug.set_is_use_a_star_path(false);
  open_space_debug.mutable_hybrid_xy_bounds_info()->set_is_send_xy_bounds(true);

  century::planning_internal::HybridXYboundsInfo hybrid_xy_bounds_info;
  common::util::Origin2World(translate_origin_, rotate_angle_, xy0, 0,
                             world_xy0, world_xy0_heading);
  common::util::Origin2World(translate_origin_, rotate_angle_, xy1, 0,
                             world_xy1, world_xy1_heading);
  hybrid_xy_bounds_info.set_x0(world_xy0.x());
  hybrid_xy_bounds_info.set_x1(world_xy1.x());
  hybrid_xy_bounds_info.set_y0(world_xy0.y());
  hybrid_xy_bounds_info.set_y1(world_xy1.y());
  open_space_debug.mutable_hybrid_xy_bounds_info()->CopyFrom(
      hybrid_xy_bounds_info);

  DelayMilliSecond(10);
  size_t send_times = 2;
  for (size_t i = 0; i < send_times; ++i) {
    DelayMilliSecond(10);
    open_space_writer_->Write(open_space_debug);
  }
  DelayMilliSecond(10);
}

void HybridAStarBidirectional::AddRSDebugInfo(
    const century::planning::Node3d* const current_node,
    const bool enable_hybrid_debug) {
  if (!enable_hybrid_debug) {
    return;
  }
  century::planning_internal::OpenSpaceDebug open_space_debug;
  open_space_debug.mutable_hybrid_rs_debug()->set_is_rs_succeed(true);

  century::planning_internal::HybridPointDebug hybrid_rs_start_point;
  hybrid_rs_start_point.set_x(current_node->GetX());
  hybrid_rs_start_point.set_y(current_node->GetY());
  hybrid_rs_start_point.set_phi(current_node->GetPhi());

  auto hybrid_rs_start_point_ptr = open_space_debug.mutable_hybrid_rs_debug()
                                       ->mutable_hybrid_rs_start_point();
  hybrid_rs_start_point_ptr->CopyFrom(hybrid_rs_start_point);
  open_space_debug.mutable_hybrid_rs_debug()->set_use_rs_dis(
      hybrid_use_rs_dis_);

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  size_t send_times = 2;
  for (size_t i = 0; i < send_times; ++i) {
    open_space_writer_->Write(open_space_debug);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return;
}

void HybridAStarBidirectional::AddHybridAStarParaDebugInfo(
    const HybridAStartResult* const result, const bool enable_hybrid_debug) {
  if (!enable_hybrid_debug) {
    return;
  }
  size_t result_size = result->x.size();
  century::planning_internal::OpenSpaceDebug open_space_debug;
  open_space_debug.set_is_first_hybrid_debug(false);

  century::planning_internal::HybridDebugResult hybrid_debug_result;
  hybrid_debug_result.set_result_size(result_size);
  hybrid_debug_result.set_heuristic_type(heuristic_type_);
  hybrid_debug_result.set_rs_num_peroid(rs_num_peroid_);
  hybrid_debug_result.set_is_hybrid_update_cost(is_hybrid_update_cost_);
  hybrid_debug_result.set_xy_grid_resolution(xy_grid_resolution_);
  hybrid_debug_result.set_hybrid_a_star_node_radius(hybrid_a_star_node_radius_);
  hybrid_debug_result.set_hybrid_a_star_bounds_radius(
      hybrid_a_star_bounds_radius_);

  hybrid_debug_result.set_astar_first_long_buffer(astar_first_long_buffer_);
  hybrid_debug_result.set_astar_first_lat_buffer(astar_first_lat_buffer_);

  hybrid_debug_result.set_search_near_dest_x_threshold(
      search_near_destination_x_threshold_);
  hybrid_debug_result.set_search_near_dest_y_threshold(
      search_near_destination_y_threshold_);
  hybrid_debug_result.set_search_near_dest_angle_threshold(
      search_near_destination_theta_threshold_);

  hybrid_debug_result.set_astar_max_time_first(astar_max_time_first_);
  hybrid_debug_result.set_astar_max_time_second(astar_max_time_second_);

  hybrid_debug_result.set_grid_a_star_xy_resolution(grid_a_star_xy_resolution_);
  hybrid_debug_result.set_grid_a_star_node_radius(grid_a_star_node_radius_);
  hybrid_debug_result.set_is_convert_to_grid_coordinates(
      is_convert_to_grid_coordinates_);
  hybrid_debug_result.set_is_xy_bounds_check(is_xy_bounds_check_);
  hybrid_debug_result.set_grid_a_star_bounds_radius(grid_a_star_bounds_radius_);

  auto hybrid_debug_result_ptr = open_space_debug.mutable_hybrid_debug_result();
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

void HybridAStarBidirectional::PrintMeetNode() {
  {
    if (nullptr != meet_node_forward_ && nullptr != meet_node_backward_) {
      Vec2d forward_node(meet_node_forward_->GetX(),
                         meet_node_forward_->GetY());
      forward_node.SelfRotate(rotate_angle_);
      forward_node += translate_origin_;
      double forward_heading =
          NormalizeAngle(meet_node_forward_->GetPhi() + rotate_angle_);

      Vec2d opposite_node(meet_node_backward_->GetX(),
                          meet_node_backward_->GetY());
      opposite_node.SelfRotate(rotate_angle_);
      opposite_node += translate_origin_;
      double oppo_heading =
          NormalizeAngle(meet_node_backward_->GetPhi() + rotate_angle_);

      AINFO << "meeting forward node: (" << forward_node.x() << ", "
            << forward_node.y() << ", " << forward_heading * 180 / M_PI << "°)";
      AINFO << "meeting backward node: (" << opposite_node.x() << ", "
            << opposite_node.y() << ", " << oppo_heading * 180 / M_PI << "°)";
    }
  }
  return;
}

}  // namespace planning
}  // namespace century