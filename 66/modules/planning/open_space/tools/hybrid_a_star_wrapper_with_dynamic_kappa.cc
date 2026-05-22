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

#include "cyber/common/file.h"
#include "modules/planning/open_space/coarse_trajectory_generator/hybrid_a_star.h"
#include "modules/planning/open_space/openspace_common/openspace_common.h"

namespace century {
namespace planning {

using century::cyber::Clock;

class HybridAObstacleContainer {
 public:
  HybridAObstacleContainer() = default;

  void AddVirtualObstacle(double* obstacle_x, double* obstacle_y,
                          int vertice_num) {
    std::vector<common::math::Vec2d> obstacle_vertices;
    for (int i = 0; i < vertice_num; ++i) {
      common::math::Vec2d vertice(obstacle_x[i], obstacle_y[i]);
      obstacle_vertices.emplace_back(vertice);
    }
    obstacles_list.emplace_back(obstacle_vertices);
  }

  void ClearAllObstacle() { obstacles_list.clear(); }

  const std::vector<std::vector<common::math::Vec2d>>&
  GetObstaclesVerticesVec() {
    return obstacles_list;
  }

 private:
  std::vector<std::vector<common::math::Vec2d>> obstacles_list;
};

class HybridAResultContainer {
 public:
  HybridAResultContainer() = default;

  void LoadResult() {
    x_ = std::move(result_.x);
    y_ = std::move(result_.y);
    phi_ = std::move(result_.phi);
    v_ = std::move(result_.v);
    a_ = std::move(result_.a);
    steer_ = std::move(result_.steer);
    // ------------------------------------------------------
    result_size_ = std::move(result_.result_size);
    explored_node_num_ = std::move(result_.explored_node_num);
    run_node_num_ = std::move(result_.run_node_num);
    search_rs_num_ = std::move(result_.search_rs_num);
    astar_total_time_ = std::move(result_.astar_total_time);
    heuristic_time_ = std::move(result_.heuristic_time);
    rs_time_ = std::move(result_.rs_time);
    rs_node_x_ = std::move(result_.rs_node_x);
    rs_node_y_ = std::move(result_.rs_node_y);
    rs_node_phi_ = std::move(result_.rs_node_phi);
    traj_kappa_contraint_ratio_ = std::move(result_.traj_kappa_contraint_ratio);
    // ------------------------------------------------------
  }

  std::vector<double>* GetX() { return &x_; }
  std::vector<double>* GetY() { return &y_; }
  std::vector<double>* GetPhi() { return &phi_; }
  std::vector<double>* GetV() { return &v_; }
  std::vector<double>* GetA() { return &a_; }
  std::vector<double>* GetSteer() { return &steer_; }
  HybridAStartResult* PrepareResult() { return &result_; }

  // ------------------------------------------------------
  // Added some other result
  size_t GetResultSize() const { return result_size_; }
  size_t GetExploredNodeNum() const { return explored_node_num_; }
  size_t GetRunNodeNum() const { return run_node_num_; }
  size_t GetSearchRsNum() const { return search_rs_num_; }
  double GetAstarTotalTime() const { return astar_total_time_; }
  double GetHeuristicTime() const { return heuristic_time_; }
  double GetRsTime() const { return rs_time_; }
  double GetRsNodeX() const { return rs_node_x_; }
  double GetRsNodeY() const { return rs_node_y_; }
  double GetRsNodePhi() const { return rs_node_phi_; }
  double GetTrajKappaContraintRatio() const {
    return traj_kappa_contraint_ratio_;
  }
  // ------------------------------------------------------

 private:
  HybridAStartResult result_;
  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> phi_;
  std::vector<double> v_;
  std::vector<double> a_;
  std::vector<double> steer_;

  // ------------------------------------------------------
  // Added some other result
  size_t result_size_ = 0;
  size_t explored_node_num_ = 0;
  size_t run_node_num_ = 0;
  size_t search_rs_num_ = 0;
  double astar_total_time_ = 0.0;
  double heuristic_time_ = 0.0;
  double rs_time_ = 0.0;
  double rs_node_x_ = 0.0;
  double rs_node_y_ = 0.0;
  double rs_node_phi_ = 0.0;
  double traj_kappa_contraint_ratio_ = 0.0;
  // ------------------------------------------------------
};

extern "C" {
HybridAStar* CreatePlannerPtr() {
  century::planning::PlannerOpenSpaceConfig planner_open_space_config_;

  //------------------------------------------------------------
  FLAGS_planner_open_space_config_filename =
      "/century/modules/tools/open_space_visualization/hybird_a_star/conf/"
      "planner_open_space_config.pb.txt";
  FLAGS_astar_first_long_buffer = 0.3;
  FLAGS_astar_first_lat_buffer = 0.5;
  AERROR << "---file---\n" << FLAGS_planner_open_space_config_filename;
  //------------------------------------------------------------

  ACHECK(century::cyber::common::GetProtoFromFile(
      FLAGS_planner_open_space_config_filename, &planner_open_space_config_))
      << "Failed to load open space config file "
      << FLAGS_planner_open_space_config_filename;
  AERROR << "---file---\n"
         << planner_open_space_config_.warm_start_config().DebugString();

  //------------------------------------------------------------
  // Whether to partition the trajectory first and do smoothing in parallel
  AERROR << "FLAGS_enable_parallel_trajectory_smoothing "
         << FLAGS_enable_parallel_trajectory_smoothing;
  // FLAGS_enable_parallel_trajectory_smoothing = true;

  std::string flag_file_path = "/century/modules/planning/conf/planning.conf";
  google::SetCommandLineOption("flagfile", flag_file_path.c_str());

  //------------------------------------------------------------
  // Whether to partition the trajectory first and do smoothing in parallel
  AERROR << "now FLAGS_enable_parallel_trajectory_smoothing "
         << FLAGS_enable_parallel_trajectory_smoothing;

  // FLAGS_enable_parallel_trajectory_smoothing = false;

  AERROR << "after set FLAGS_enable_parallel_trajectory_smoothing "
         << FLAGS_enable_parallel_trajectory_smoothing;

  // Whether use s-curve (piecewise_jerk) for smoothing Hybrid Astar
  // speed/acceleration.
  AERROR << "now FLAGS_use_s_curve_speed_smooth "
         << FLAGS_use_s_curve_speed_smooth;

  FLAGS_use_s_curve_speed_smooth = true;
  FLAGS_use_s_curve_speed_smooth = false;

  AERROR << "after set FLAGS_use_s_curve_speed_smooth "
         << FLAGS_use_s_curve_speed_smooth;
  //------------------------------------------------------------

  return new HybridAStar(planner_open_space_config_);
}

HybridAObstacleContainer* CreateObstaclesPtr() {
  return new HybridAObstacleContainer();
}

HybridAResultContainer* CreateResultPtr() {
  return new HybridAResultContainer();
}

void AddVirtualObstacle(HybridAObstacleContainer* obstacles_ptr,
                        double* obstacle_x, double* obstacle_y,
                        int vertice_num) {
  obstacles_ptr->AddVirtualObstacle(obstacle_x, obstacle_y, vertice_num);
}

void ClearAllObstacle(HybridAObstacleContainer* obstacles_ptr) {
  obstacles_ptr->ClearAllObstacle();
}

bool PlanWithDynamicAdjustKappa(HybridAStar* planner_ptr,
                                HybridAObstacleContainer* obstacles_ptr,
                                HybridAResultContainer* result_ptr, double sx,
                                double sy, double sphi, double ex, double ey,
                                double ephi, double* XYbounds) {
  std::vector<double> XYbounds_(XYbounds, XYbounds + 4);
  AERROR << "------------PlanWithDynamicAdjustKappa------------------";
  PlannerOpenSpaceConfig config = planner_ptr->GetConfig();
  auto& kappa_config = config.warm_start_config().kappa_contraint_config();
  AERROR << "kappa_config.size() " << kappa_config.size();
  double plan_time = 0.0;
  double max_plan_time =
      std::max(config.warm_start_config().astar_max_time_first(),
               config.warm_start_config().astar_max_time_second());
  for (int i = 0; i < kappa_config.size(); ++i) {
    AINFO << "now use kappa_contraint_configs_ is "
          << kappa_config[i].traj_kappa_contraint_ratio();
    planner_ptr->SetKappaContraintConfig(kappa_config[i]);
    const double start_time = Clock::NowInSeconds();

    // WYQ To be adapted：empty data
    const std::vector<century::planning::Obstacle> obs;
    const std::vector<std::vector<Vec2d>> world_obstacles_vertices_vec;
    const double rotate_angle = 0.0;
    const Vec2d translate_origin;
    bool plan_result = planner_ptr->Plan(
        world_obstacles_vertices_vec, sx, sy, sphi, ex, ey, ephi, rotate_angle,
        translate_origin, XYbounds_, obs,
        obstacles_ptr->GetObstaclesVerticesVec(), result_ptr->PrepareResult());
    if (plan_result) {
      AINFO << "dynamic adjust kappa for hybrid astar solved successfully!";
      const double end_time = Clock::NowInSeconds();
      plan_time += end_time - start_time;
      AINFO << "this plan diff time  is " << (end_time - start_time) << " s.";
      AERROR << "total plan time is " << plan_time << " s.";
      return true;
    } else {
      const double end_time = Clock::NowInSeconds();
      plan_time += end_time - start_time;
      std::shared_ptr<OpenspaceCommon> openspace_common_ptr =
          std::make_shared<OpenspaceCommon>();
      AINFO << "this hybrid astar solved failed!"
            << " index = " << i;
      for (const auto& it : openspace_common_ptr->GetHADebugStatus()) {
        AINFO << "reason = " << static_cast<int>(it);
      }
      AINFO << "now diff time  is " << (end_time - start_time) << " s.";
      AINFO << "now plan_time is " << plan_time << " s.";
    }

    if (plan_time > max_plan_time || (i == kappa_config.size() - 1)) {
      AERROR << "dynamic adjust kappa for hybrid astar failed. plan_time is "
             << plan_time << " s.";
      return false;
    }
  }

  return false;
}

bool Plan(HybridAStar* planner_ptr, HybridAObstacleContainer* obstacles_ptr,
          HybridAResultContainer* result_ptr, double sx, double sy, double sphi,
          double ex, double ey, double ephi, double* XYbounds) {
  std::vector<double> XYbounds_(XYbounds, XYbounds + 4);
  AERROR << "------------Plan------------------";

  // WYQ To be adapted：empty data
  const std::vector<century::planning::Obstacle> obs;
  const std::vector<std::vector<Vec2d>> world_obstacles_vertices_vec;
  const double rotate_angle = 0.0;
  const Vec2d translate_origin;
  return planner_ptr->Plan(world_obstacles_vertices_vec, sx, sy, sphi, ex, ey,
                           ephi, rotate_angle, translate_origin, XYbounds_, obs,
                           obstacles_ptr->GetObstaclesVerticesVec(),
                           result_ptr->PrepareResult());
}

void GetResult(HybridAResultContainer* result_ptr, double* x, double* y,
               double* phi, double* v, double* a, double* steer,
               size_t* output_size) {
  result_ptr->LoadResult();
  size_t size = result_ptr->GetX()->size();
  std::cout << "return size is " << size << std::endl;
  //  x y phi v
  for (size_t i = 0; i < size; ++i) {
    x[i] = result_ptr->GetX()->at(i);
    y[i] = result_ptr->GetY()->at(i);
    phi[i] = result_ptr->GetPhi()->at(i);
    v[i] = result_ptr->GetV()->at(i);
  }

  //  a steer
  for (size_t i = 0; i < size - 1; ++i) {
    a[i] = result_ptr->GetA()->at(i);
    steer[i] = result_ptr->GetSteer()->at(i);
  }
  *output_size = size;
}

// ------------------------------------------------------
// Added some other result
void GetResultMore(HybridAResultContainer* result_ptr, double* x, double* y,
                   double* phi, double* v, double* a, double* steer,
                   size_t* output_size, size_t* explored_node_num,
                   size_t* run_node_num, size_t* search_rs_num,
                   double* astar_total_time, double* heuristic_time,
                   double* rs_time, double* rs_node_x, double* rs_node_y,
                   double* rs_node_phi, double* traj_kappa_contraint_ratio) {
  result_ptr->LoadResult();
  size_t size = result_ptr->GetX()->size();
  std::cout << "return size is " << size << std::endl;
  //  x y phi v
  for (size_t i = 0; i < size; ++i) {
    x[i] = result_ptr->GetX()->at(i);
    y[i] = result_ptr->GetY()->at(i);
    phi[i] = result_ptr->GetPhi()->at(i);
    v[i] = result_ptr->GetV()->at(i);
  }

  //  a steer
  for (size_t i = 0; i < size - 1; ++i) {
    a[i] = result_ptr->GetA()->at(i);
    steer[i] = result_ptr->GetSteer()->at(i);
  }
  *output_size = size;

  // ------------------------------------------------------
  *explored_node_num = result_ptr->GetExploredNodeNum();
  *run_node_num = result_ptr->GetRunNodeNum();
  *search_rs_num = result_ptr->GetSearchRsNum();
  *astar_total_time = result_ptr->GetAstarTotalTime();
  *heuristic_time = result_ptr->GetHeuristicTime();
  *rs_time = result_ptr->GetRsTime();
  *rs_node_x = result_ptr->GetRsNodeX();
  *rs_node_y = result_ptr->GetRsNodeY();
  *rs_node_phi = result_ptr->GetRsNodePhi();
  *traj_kappa_contraint_ratio = result_ptr->GetTrajKappaContraintRatio();
  // ------------------------------------------------------
}
// ------------------------------------------------------
};

}  // namespace planning
}  // namespace century
