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
 * @file: hybrid_a_star_bidirectional.h
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/openspace_bidirection_config.pb.h"
#include "modules/planning/proto/planner_open_space_config.pb.h"

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/normal_util.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/math_util.h"
#include "modules/planning/open_space/coarse_trajectory_generator/grid_search.h"
#include "modules/planning/open_space/coarse_trajectory_generator/node3d.h"
#include "modules/planning/open_space/coarse_trajectory_generator/reeds_shepp_path.h"
#include "modules/planning/open_space/openspace_common/openspace_common.h"
#include "modules/planning/tasks/utils/openspace_util.h"

using century::common::math::Vec2d;
using NodeIndex = int64_t;

namespace century {
namespace planning {

class HybridAStarBidirectional {
 public:
  explicit HybridAStarBidirectional(
      const PlannerOpenSpaceConfig& open_space_conf);

  virtual ~HybridAStarBidirectional() = default;

  /*
   * @brief plan trajectory
   * @param world_obstacles_vertices_vec      obstacles polygon list
   * @param (sx, sy, sphi)                    start pose
   * @param (ex, ey, ephi)                    end pose
   * @param rotate_angle, translate_origin    world --> map
   * @param XYbounds                          map ROI
   * @param obstacles                         obstacles
   * @param result                            planned trajectory
   * @param nearby_path                       reference line
   * @param use_pure_astar                    not use reed-shepp curve flag
   * @param start_collision_flag              start pose collision flag
   * @return success: true   fail: false
   * */
  bool Plan(const std::vector<std::vector<Vec2d>>& world_obstacles_vertices_vec,
            double sx, double sy, double sphi, double ex, double ey,
            double ephi, double rotate_angle, const Vec2d& translate_origin,
            const std::vector<double>& XYbounds,
            const std::vector<Obstacle>& obstacles,
            const std::vector<std::vector<common::math::Vec2d>>&
                obstacles_vertices_vec,
            HybridAStartResult* result, const hdmap::Path* nearby_path,
            bool use_pure_astar = false, bool* start_collision_flag = nullptr);

  bool BidirectionalPlan(
      const std::vector<std::vector<Vec2d>>& world_obstacles_vertices_vec,
      double sx, double sy, double sphi, double ex, double ey, double ephi,
      double rotate_angle, const Vec2d& translate_origin,
      const std::vector<double>& XYbounds,
      const std::vector<Obstacle>& obstacles,
      const std::vector<std::vector<common::math::Vec2d>>&
          obstacles_vertices_vec,
      HybridAStartResult* result, const hdmap::Path* nearby_path,
      bool use_pure_astar = false, bool* start_collision_flag = nullptr);

  /*
   * @brief plan trajectory
   * @param world_obstacles_vertices_vec      obstacles polygon list
   * @param (sx, sy, sphi)                    start pose
   * @param (ex, ey, ephi)                    end pose
   * @param rotate_angle, translate_origin    world --> map
   * @param XYbounds                          map ROI
   * @param obstacles                         obstacles
   * @param result                            planned trajectory
   * @param use_pure_astar                    not use reed-shepp curve flag
   * @param start_collision_flag              start pose collision flag
   * @return success: true  fail: false
   * */
  bool Plan(const std::vector<std::vector<Vec2d>>& world_obstacles_vertices_vec,
            double sx, double sy, double sphi, double ex, double ey,
            double ephi, double rotate_angle, const Vec2d& translate_origin,
            const std::vector<double>& XYbounds,
            const std::vector<Obstacle>& obstacles,
            const std::vector<std::vector<common::math::Vec2d>>&
                obstacles_vertices_vec,
            HybridAStartResult* result, bool use_pure_astar = false,
            bool* start_collision_flag = nullptr);

  bool TrajectoryPartition(const HybridAStartResult& result,
                           std::vector<HybridAStartResult>* partitioned_result);

  void SetKappaContraintConfig(const KappaContraintRatioConfig& kappa_config);

  void ResetDefaultKappaContraint();

  bool GetAStarPath(GridAStartResult* result) {
    *result = a_star_result_;
    return true;
  }

  const PlannerOpenSpaceConfig& GetConfig() const {
    return planner_open_space_config_;
  }

  void SetIsMapBoundary(bool is_map_boundary) {
    is_map_boundary_ = is_map_boundary;
  }

 private:
  void CreateCyber();
  void WriteData(const std::vector<Node3d>& hybrid_node);
  void DelayMilliSecond(int msec);
  void DelaySecond(int sec);
  void WriteObsLine2dData();
  void WriteStartEndPointData();
  void WriteXYbounds();

  /*
   * @brief hybrid astar search function
   * @param use_pure_astar  not use pure astar flag
   * @param nearby_path     reference line
   * @return success: true  fail: false
   */
  bool Search(bool use_pure_astar, const hdmap::Path* nearby_path);

  double CalCurrentNodeToEndNodeDis(const std::shared_ptr<Node3d> current_node);
  // expand node using reed-shepp curve
  bool AnalyticExpansion(const double& astar_start_time,
                         const size_t force_forward_init,
                         const std::shared_ptr<Node3d> current_node,
                         const bool use_pure_astar,
                         const hdmap::Path* nearby_path);

  /*
   * @brief check if it is near end pose
   * @param node   node
   */
  bool IsNearGoal(std::shared_ptr<Node3d> node);

  /*
   * @brief check if it is start node
   * @param node   node
   */
  bool IsStartNode(std::shared_ptr<Node3d> node);

  /*
   * @brief calculate total distance in same direction
   * @param node   node
   */
  double CalculateDirectionalDistance(std::shared_ptr<Node3d> node);

  /*
   * @brief check if it is near path or end pose
   * @param node                 node
   * @param rs_node              reed shepp node flag
   * @param nearby_path          near path
   */
  bool IsNearGoal2(std::shared_ptr<Node3d> node, bool rs_node,
                   const hdmap::Path* nearby_path);

  bool ValidityCheck(std::shared_ptr<Node3d> node, bool is_first_node = false);

  bool ValidityCheckBidirection(std::shared_ptr<Node3d> node);

  bool RSPCheck(const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end);
  bool InitForwardCheck(
      const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end);
  // load the whole RSP as nodes and add to the close set
  std::shared_ptr<Node3d> LoadRSPinCS(
      const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
      std::shared_ptr<Node3d> current_node,
      const bool is_forward_search = true);

  std::shared_ptr<Node3d> GenerateStraightNode(
      std::shared_ptr<Node3d> current_node, bool next_direction);
  /*
   * @brief generate next node
   * @param current_node     current node
   * @param next_node_index    steer sample index
   * */
  std::shared_ptr<Node3d> Next_node_generator(
      std::shared_ptr<Node3d> current_node, size_t next_node_index);

  double TrajCost(std::shared_ptr<Node3d> current_node,
                  std::shared_ptr<Node3d> next_node,
                  const bool is_forward_search = true);
  double HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node);
  double NearObstacleHeuristic(std::shared_ptr<Node3d> next_node);

  bool GetResult(HybridAStartResult* result, bool is_normal_order);

  bool GetTemporalProfile(HybridAStartResult* result);
  bool GenerateSpeedAcceleration(HybridAStartResult* result);
  bool GenerateSCurveSpeedAcceleration(HybridAStartResult* result);

  /*
   * @brief set max front wheel angle
   * @param traj_kappa_contraint_ratio   kappa constraint ratio
   * */
  void SetMaxWheelAngle(const double traj_kappa_contraint_ratio);

  std::shared_ptr<Node3d> GetBidirectionalStartNode(
      const bool is_forward_search,
      const std::shared_ptr<Node3d> origin_start_node,
      const std::shared_ptr<Node3d> origin_end_node) {
    return is_forward_search ? origin_start_node : origin_end_node;
  };

  std::shared_ptr<Node3d> GetBidirectionalEndNode(
      const bool is_forward_search,
      const std::shared_ptr<Node3d> origin_start_node,
      const std::shared_ptr<Node3d> origin_end_node) {
    return is_forward_search ? origin_end_node : origin_start_node;
  };

  bool PerformSingleSearch(const bool use_pure_astar,
                           const hdmap::Path* nearby_path, const double sx,
                           const double sy, const double sphi, const double ex,
                           const double ey, const double ephi,
                           HybridAStartResult* result);

  bool BidirectionalSearch(const bool use_pure_astar,
                           const hdmap::Path* nearby_path);

  bool RemoveCloseNodes(const double min_distance_threshold,
                        std::vector<std::shared_ptr<Node3d>>* nodes);

  bool NodeCostFilter(
      const std::shared_ptr<Node3d> curr_node,
      const std::unordered_map<NodeIndex, std::shared_ptr<Node3d>>& node_set,
      std::vector<std::shared_ptr<Node3d>>* output_node);

  bool TryRSConnectOppoSide(
      std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> oppo_node,
      std::shared_ptr<century::planning::ReedSheppPath> optimal_path);

  bool MeetInMiddle(
      std::shared_ptr<Node3d> current_node,
      std::unordered_map<NodeIndex, std::shared_ptr<Node3d>>& oppo_set,
      const bool is_forward_search, std::vector<HybridAStartResult>* result);

  bool ConnectResult(std::shared_ptr<Node3d> f, std::shared_ptr<Node3d> b,
                     HybridAStartResult* final_result);

  bool ExpandNode(std::shared_ptr<Node3d> current_node,
                  const bool use_pure_astar, const hdmap::Path* nearby_path,
                  bool is_forward_search);

  bool CalcRSHeutisticCost(const std::shared_ptr<Node3d> next_node,
                           const std::shared_ptr<Node3d> goal_node,
                           std::shared_ptr<ReedSheppPath> rs_path,
                           double& rs_cost);

  void CalculateBidirectionNodeCost(const double& astar_start_time,
                                    std::shared_ptr<Node3d> current_node,
                                    std::shared_ptr<Node3d> next_node,
                                    const hdmap::Path* nearby_path);

  void CalculateBidirectionNodeCostRev(const double& astar_start_time,
                                       std::shared_ptr<Node3d> current_node,
                                       std::shared_ptr<Node3d> next_node,
                                       const hdmap::Path* nearby_path);

  bool IsBidirectionalGoal(std::shared_ptr<Node3d> node, bool is_forward);

  void ResetSearchState();

  void AddRSDebugInfo(const century::planning::Node3d* const current_node,
                      const bool enable_hybrid_debug);

  void AddHybridAStarParaDebugInfo(const HybridAStartResult* const result,
                                   const bool enable_hybrid_debug);

  void PrintMeetNode();

 private:
  PlannerOpenSpaceConfig planner_open_space_config_;
  OpenSpaceBidirectionConfig open_space_bidirection_config_;
  common::VehicleParam vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  size_t next_node_num_ = 0;
  size_t force_forward_init_ = 0;
  double max_front_wheel_angle_ = 0.0;
  double sample_step_size_ = 0.0;
  double xy_grid_resolution_ = 0.0;
  double delta_t_ = 0.0;
  double traj_forward_penalty_ = 0.0;
  double traj_back_penalty_ = 0.0;
  double traj_gear_switch_penalty_ = 0.0;
  double traj_steer_penalty_ = 0.0;
  double traj_steer_change_penalty_ = 0.0;
  double traj_end_heading_error_penalty_ = 0.0;
  double heu_rs_forward_penalty_ = 0.0;
  double heu_rs_back_penalty_ = 0.0;
  double heu_rs_gear_switch_penalty_ = 0.0;
  double heu_rs_steer_penalty_ = 0.0;
  double heu_rs_steer_change_penalty_ = 0.0;

  double astar_first_long_buffer_ = 0.0;
  double astar_first_lat_buffer_ = 0.0;

  double search_near_destination_x_threshold_ = 0.0;
  double search_near_destination_y_threshold_ = 0.0;
  double search_near_destination_theta_threshold_ = 0.0;

  double astar_max_time_first_ = 0.0;
  double astar_max_time_second_ = 0.0;
  double goal_dist_cost_ = 0.0;
  double near_obs_dist_ = 0.0;
  double near_obs_dist_cost_ = 0.0;

  double bidirection_astar_search_time_threshold_ = 0.0;

  double traj_kappa_contraint_ratio_ = 0.0;
  double hybrid_a_star_node_radius_ = 0.5;
  double hybrid_a_star_bounds_radius_ = 0.5;
  bool is_hybrid_a_star_bounds_check_ = false;
  bool is_hybrid_debug_ = false;
  bool is_hybrid_node_init_ = false;
  // 0:DpMap  1:A Star  2:EulerDistance  3:ManhattanDistance
  uint32_t heuristic_type_ = 0;
  uint32_t rs_num_peroid_ = 1;
  bool is_hybrid_update_cost_ = false;

  double grid_a_star_xy_resolution_ = 0.5;
  double grid_a_star_node_radius_ = 0.5;
  bool is_convert_to_grid_coordinates_ = true;
  bool is_xy_bounds_check_ = false;
  double grid_a_star_bounds_radius_ = 0.5;
  std::vector<Vec2d> a_star_obstacles_list_;
  std::vector<common::math::LineSegment2d> a_star_linesegments_vec_;

  double astar_long_buffer_;
  double astar_lat_buffer_;

  double extra_length_;
  double extra_width_;

  double hybrid_use_rs_dis_ = 0.0;

  std::vector<double> XYbounds_;
  std::shared_ptr<Node3d> start_node_;
  std::shared_ptr<Node3d> end_node_;
  std::shared_ptr<Node3d> rs_node_;
  std::shared_ptr<Node3d> final_node_;

  std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec_;

  bool enable_voronoi_field_ = false;
  int voronoi_obs_acting_dis_ = 25;
  double voronoi_dist_cost_ = 10;
  std::vector<float> distanceTransform_;

  int map_width_;
  int map_height_;

  struct cmp {
    bool operator()(const std::pair<NodeIndex, double>& left,
                    const std::pair<NodeIndex, double>& right) const {
      return left.second >= right.second;
    }
  };

  std::priority_queue<std::pair<NodeIndex, double>,
                      std::vector<std::pair<NodeIndex, double>>, cmp>
      open_pq_;

  std::unordered_map<NodeIndex, std::shared_ptr<Node3d>> open_set_;
  std::unordered_map<NodeIndex, std::shared_ptr<Node3d>> close_set_;

  std::shared_ptr<Node3d> start_node_rev_;
  std::shared_ptr<Node3d> end_node_rev_;

  std::unordered_map<NodeIndex, std::shared_ptr<Node3d>> open_set_rev_;
  std::unordered_map<NodeIndex, std::shared_ptr<Node3d>> close_set_rev_;
  std::priority_queue<std::pair<NodeIndex, double>,
                      std::vector<std::pair<NodeIndex, double>>, cmp>
      open_pq_rev_;

  std::shared_ptr<Node3d> meet_node_forward_;
  std::shared_ptr<Node3d> meet_node_backward_;

  bool enable_bidirectional_search_ = true;
  int max_bidirectional_iterations_ = 5000;

  std::unique_ptr<ReedShepp> reed_shepp_generator_;
  std::unique_ptr<GridSearch> grid_a_star_heuristic_generator_;


  std::shared_ptr<cyber::Writer<century::planning_internal::OpenSpaceDebug>>
      open_space_writer_;

  GridAStartResult a_star_result_;
  std::unique_ptr<OpenspaceCommon> openspace_common_ =
      std::make_unique<OpenspaceCommon>();

  std::unordered_set<OpenspaceCommon::HybridAStarDebugStatus> ha_debug_;

  bool is_a_star_success_ = false;
  bool is_map_boundary_ = false;
  bool last_success_ = true;
  bool start_collision_flag_ = false;
  bool is_change_start_end_ = false;

  // map ---> world
  double rotate_angle_;
  common::math::Vec2d translate_origin_;

 public:
  static void Build(
      const std::vector<std::vector<common::math::Vec2d>>& obstacle_points) {
    obstacle_points_ = obstacle_points;
  }

 private:
  double ObstacleCost(
      const std::shared_ptr<century::planning::Node3d>& node,
      const std::vector<std::vector<common::math::Vec2d>>& obstacle_points);

  double Distance(double x, double y,
                  const std::vector<std::vector<common::math::Vec2d>>&
                      obstacle_points) const {
    double min_dist = std::numeric_limits<double>::infinity();
    for (const auto& single_obstacle_points : obstacle_points) {
      for (const auto& p : single_obstacle_points) {
        double dx = x - p.x();
        double dy = y - p.y();
        min_dist = std::min(min_dist, std::hypot(dx, dy));
      }
    }
    return min_dist;
  }

  static std::vector<std::vector<common::math::Vec2d>> obstacle_points_;
};

}  // namespace planning
}  // namespace century