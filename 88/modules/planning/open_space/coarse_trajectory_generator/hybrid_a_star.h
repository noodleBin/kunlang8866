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
 * @file
 */

#pragma once

#include <algorithm>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/math/vec2d.h"
using century::common::math::Vec2d;
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/planner_open_space_config.pb.h"

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/open_space/openspace_common/openspace_common.h"
#include "modules/planning/open_space/coarse_trajectory_generator/grid_search.h"
#include "modules/planning/open_space/coarse_trajectory_generator/node3d.h"
#include "modules/planning/open_space/coarse_trajectory_generator/reeds_shepp_path.h"

namespace century {
namespace planning {

class HybridAStar {
 public:
  // constructor
  explicit HybridAStar(const PlannerOpenSpaceConfig& open_space_conf);
  // destructor
  virtual ~HybridAStar() = default;

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
            HybridAStartResult* result, const hdmap::Path *nearby_path,
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

  const double GetVoronoiValue(const int x, const int y, const int nx,
                               const int ny, const ObsPoint& map_ld_point);

  void CalCulateBorderDistances(const std::vector<ObsPoint>& all_obs_point,
                                const int nx, const int ny,
                                const ObsPoint& map_ld_point);

  // segment trajectory
  bool TrajectoryPartition(const HybridAStartResult& result,
                           std::vector<HybridAStartResult>* partitioned_result);

  // add kappa constraint
  void SetKappaContraintConfig(const KappaContraintRatioConfig& kappa_config);

  // add default kappa contraint
  void ResetDefaultKappaContraint();

  // generate A* path
  bool GenerateAStarPath();
  bool GetAStarPath(GridAStartResult* result) {
    *result = a_star_result_;
    return true;
  }

  // configure
  const PlannerOpenSpaceConfig& GetConfig() const {
    return planner_open_space_config_;
  }

  void SetIsMapBoundary(bool is_map_boundary) {
    is_map_boundary_ = is_map_boundary;
  }

 private:
  // -----------------------------------------------------
  // create a cyber node
  void CreateCyber();
  void WriteData(const std::vector<Node3d>& hybrid_node);
  void DelayMilliSecond(int msec);
  void DelaySecond(int sec);
  void WriteObsLine2dData();
  void WriteStartEndPointData();
  void WriteXYbounds();
  // -----------------------------------------------------

  /*
   * @brief hybrid astar search function
   * @param use_pure_astar  not use pure astar flag
   * @param nearby_path     reference line
   * @return success: true  fail: false
   */
  bool Search(bool use_pure_astar, const hdmap::Path *nearby_path);

  // the distance between current_node and end_node
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
                   const hdmap::Path *nearby_path);
  // check collision and validity
  bool ValidityCheck(std::shared_ptr<Node3d> node, bool is_first_node = false);
  // check Reeds Shepp path collision and validity
  bool RSPCheck(const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end);
  bool InitForwardCheck(
      const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end);
  // load the whole RSP as nodes and add to the close set
  std::shared_ptr<Node3d> LoadRSPinCS(
      const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
      std::shared_ptr<Node3d> current_node);
  /*
   * @brief generate next node
   * @param current_node     current node
   * @param next_node_index    steer sample index
   * */
  std::shared_ptr<Node3d> Next_node_generator(
      std::shared_ptr<Node3d> current_node, size_t next_node_index);
  // compute next node f_cost
  void CalculateNodeCost(const double& astar_start_time,
                         std::shared_ptr<Node3d> current_node,
                         std::shared_ptr<Node3d> next_node,
                         const hdmap::Path *nearby_path);
  // current_node to next_node g_cost
  double TrajCost(std::shared_ptr<Node3d> current_node,
                  std::shared_ptr<Node3d> next_node);
  double HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node);
  double NearObstacleHeuristic(std::shared_ptr<Node3d> next_node);
  // collect result
  bool GetResult(HybridAStartResult* result, bool flag);
  // compute v, a, steer
  bool GetTemporalProfile(HybridAStartResult* result);
  // compute velocity and acceleration
  bool GenerateSpeedAcceleration(HybridAStartResult* result);
  bool GenerateSCurveSpeedAcceleration(HybridAStartResult* result);

  /*
   * @brief set max front wheel angle
   * @param traj_kappa_contraint_ratio   kappa constraint ratio
   * */
  void SetMaxWheelAngle(const double traj_kappa_contraint_ratio);

  void HybridAStarDebugInfo(const bool is_hybrid_debug,
                            const HybridAStartResult* result);

 private:
  // configure
  PlannerOpenSpaceConfig planner_open_space_config_;
  // vehicle parameter
  common::VehicleParam vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  // search for the number of adjacent grids
  size_t next_node_num_ = 0;
  size_t force_forward_init_ = 0;
  // max front wheel angle
  double max_front_wheel_angle_ = 0.0;
  // sample step size
  double step_size_ = 0.0;
  // map grid resolution
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

  // search finish threshold
  double search_near_destination_x_threshold_ = 0.0;
  double search_near_destination_y_threshold_ = 0.0;
  double search_near_destination_theta_threshold_ = 0.0;

  double astar_max_time_first_ = 0.0;
  double astar_max_time_second_ = 0.0;
  double goal_dist_cost_ = 0.0;
  double near_obs_dist_ = 0.0;
  double near_obs_dist_cost_ = 0.0;

  // kappa contraint ratio
  double traj_kappa_contraint_ratio_ = 0.0;
  // -----------------------------------------------------
  // Added for Hybrid a star Collision detection safety distance
  double hybrid_a_star_node_radius_ = 0.5;
  double hybrid_a_star_bounds_radius_ = 0.5;
  bool is_hybrid_a_star_bounds_check_ = false;
  bool is_hybrid_debug_ = false;
  bool is_hybrid_node_init_ = false;
  // 0:DpMap  1:A Star  2:EulerDistance  3:ManhattanDistance
  uint32_t heuristic_type_ = 0;
  uint32_t rs_num_peroid_ = 1;
  bool is_hybrid_update_cost_ = false;
  // Grid a star for heuristic
  double grid_a_star_xy_resolution_ = 0.5;
  double grid_a_star_node_radius_ = 0.5;
  // Add boundary check weihuahsen
  bool is_convert_to_grid_coordinates_ = true;
  bool is_xy_bounds_check_ = false;
  double grid_a_star_bounds_radius_ = 0.5;
  std::vector<Vec2d> a_star_obstacles_list_;
  std::vector<common::math::LineSegment2d> a_star_linesegments_vec_;
  // -----------------------------------------------------

  // collision detection buffer
  double astar_long_buffer_;
  double astar_lat_buffer_;

  // cal node near obs cost buffer
  double extra_length_;
  double extra_width_;

  // enable rs dis
  double hybrid_use_rs_dis_ = 0.0;

  // map ROI
  std::vector<double> XYbounds_;
  // start node
  std::shared_ptr<Node3d> start_node_;
  // end nose
  std::shared_ptr<Node3d> end_node_;
  // -----------------------------------------------------
  // reed-sheep curve node
  std::shared_ptr<Node3d> rs_node_;
  // -----------------------------------------------------
  // search final node
  std::shared_ptr<Node3d> final_node_;

  // obstacle polygon list
  std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec_;

  bool enable_voronoi_field_ = false;
  int voronoi_obs_acting_dis_ = 25;  // dm
  double voronoi_dist_cost_ = 10;    // cost
  std::vector<float> distanceTransform_;
  // map size
  int map_width_;
  int map_height_;

  struct cmp {
    bool operator()(const std::pair<std::string, double>& left,
                    const std::pair<std::string, double>& right) const {
      return left.second >= right.second;
    }
  };
  // priority queue
  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>, cmp>
      open_pq_;
  // open list
  std::unordered_map<std::string, std::shared_ptr<Node3d>> open_set_;
  // close list
  std::unordered_map<std::string, std::shared_ptr<Node3d>> close_set_;
  // reed-shepp curve generator
  std::unique_ptr<ReedShepp> reed_shepp_generator_;
  // A* planner
  std::unique_ptr<GridSearch> grid_a_star_heuristic_generator_;

  // -----------------------------------------------------
  // Added By WeiHuaShen For Debug Hybrid a star View
  std::shared_ptr<cyber::Writer<century::planning_internal::OpenSpaceDebug>>
      open_space_writer_;
  // -----------------------------------------------------

  // A* plan result
  GridAStartResult a_star_result_;
  // new debug for hybrid a star
  std::unordered_set<OpenspaceCommon::HybridAStarDebugStatus> ha_debug_;
  // A* plan succeed flag
  bool is_a_star_success_ = false;
  // use map boundary flag
  bool is_map_boundary_ = false;
  bool last_success_ = true;
  // start pose collsion flag
  bool start_collision_flag_ = false;

  // map ---> world
  double rotate_angle_;
  common::math::Vec2d translate_origin_;
};

}  // namespace planning
}  // namespace century
