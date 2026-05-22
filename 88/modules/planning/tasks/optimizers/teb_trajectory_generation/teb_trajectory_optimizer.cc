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
 * @file
 **/

#include "modules/planning/tasks/optimizers/teb_trajectory_generation/teb_trajectory_optimizer.h"

#include "modules/planning/math/discrete_points_math.h"

namespace century {
namespace planning {

using century::common::ErrorCode;
using century::common::PathPoint;
using century::common::Status;
using century::common::TrajectoryPoint;
using century::common::math::Box2d;
using century::common::math::NormalizeAngle;
using century::common::math::Vec2d;
using century::cyber::Clock;

namespace {
constexpr double kTimeInterval = 0.25;
constexpr double kReverseSpeed = -0.4;
constexpr int kMinSize = 3;
constexpr double kEpsilon = 1.0e-3;
}  // namespace

TEBTrajectoryOptimizer::TEBTrajectoryOptimizer(
    const TEBTrajectoryOptimizerConfig& config)
    : config_(config) {
  ConvertToTebConfig();
  // Initialize hybrid astar class pointer
  warm_start_.reset(new HybridAStar(config.planner_open_space_config()));

  // Initialize dual variable warm start class pointer
  dual_variable_warm_start_.reset(
      new DualVariableWarmStartProblem(config.planner_open_space_config()));

  // Initialize distance approach trajectory smootherclass pointer
  distance_approach_.reset(
      new DistanceApproachProblem(config.planner_open_space_config()));

  // Initialize iterative anchoring smoother config class pointer
  iterative_anchoring_smoother_.reset(
      new IterativeAnchoringSmoother(config.planner_open_space_config()));
  adc_model_ = boost::make_shared<CircularRobotFootprint>(1.0);
  teb_visual_ = TebVisualizationPtr(new TebVisualization(teb_config_));
  teb_planner_.reset(new TebOptimalPlanner(
      teb_config_, &teb_obstacles_, adc_model_, teb_visual_, &via_points_));
}

TEBTrajectoryOptimizer::TEBTrajectoryOptimizer(
    const TEBTrajectoryOptimizerConfig& config,
    century::planning::RescueStatus* rescue_status)
    : config_(config) {
  ConvertToTebConfig();

  auto& kappa_config = config_.planner_open_space_config()
                           .warm_start_config()
                           .kappa_contraint_config();
  AINFO << "kappa_config.size() " << kappa_config.size();
  for (int i = 0; i < kappa_config.size(); ++i) {
    kappa_contraint_configs_.emplace_back(kappa_config[i]);
    AINFO << "traj_kappa_contraint_ratio "
          << kappa_config[i].traj_kappa_contraint_ratio();
    AINFO << "astar_max_time_first " << kappa_config[i].astar_max_time_first();
    AINFO << "astar_max_time_second "
          << kappa_config[i].astar_max_time_second();
  }

  // Initialize hybrid astar class pointer
  warm_start_.reset(new HybridAStar(config.planner_open_space_config()));

  adc_model_ = boost::make_shared<CircularRobotFootprint>(1.0);
  teb_visual_ = TebVisualizationPtr(new TebVisualization(teb_config_));
  teb_planner_.reset(new TebOptimalPlanner(
      teb_config_, &teb_obstacles_, adc_model_, teb_visual_, &via_points_));

  rescue_status_ = rescue_status;
}

void TEBTrajectoryOptimizer::ClearPlanner() {
  teb_planner_->ClearPlanner();
  optimized_trajectory_.Clear();
  stitching_trajectory_.clear();
}

void TEBTrajectoryOptimizer::SetUseKappaContraint(bool use_kappa_contraint) {
  use_kappa_contraint_ = use_kappa_contraint;
}

void TEBTrajectoryOptimizer::UpdateDebugInfo(
    planning_internal::OpenSpaceDebug* open_space_debug) {
  open_space_debug->MergeFrom(open_space_debug_);
}

// new part plan
// WYQ_Mark_Process 1
Status TEBTrajectoryOptimizer::Plan(
    const std::vector<std::vector<Vec2d>>& world_obstacles_vertices_vec,
    const std::vector<common::TrajectoryPoint>& stitching_trajectory,
    const std::vector<double>& start_pose, const std::vector<double>& end_pose,
    const std::vector<double>& XYbounds, double rotate_angle,
    const Vec2d& translate_origin, const Eigen::MatrixXi& obstacles_edges_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
    const std::vector<Obstacle>& obstacles,
    const std::vector<std::vector<Vec2d>>& boundary_vertices_vec,
    const StaticAreaPolygons& costmap_obstacles, bool use_map_bound,
    bool* start_collision_flag) {
  if (XYbounds.empty() || end_pose.empty() || obstacles_edges_num.cols() == 0 ||
      obstacles_A.cols() == 0 || obstacles_b.cols() == 0 ||
      start_pose.size() < 2) {
    AERROR << "TEBTrajectoryOptimizer input data not ready";
    return Status(ErrorCode::PLANNING_ERROR,
                  "TEBTrajectoryOptimizer input data not ready");
  }
  stitching_trajectory_ = stitching_trajectory;
  // Init trajectory point is the stitching point from last trajectory
  const common::TrajectoryPoint trajectory_stitching_point =
      stitching_trajectory.back();

  start_pose_ = start_pose;
  end_pose_ = end_pose;
  double dx = end_pose[0] - start_pose[0];
  double dy = end_pose[1] - start_pose[1];
  double ds = std::hypot(dx, dy);
  // Generate Stop trajectory if init point close to destination
  if (ds <
      config_.planner_open_space_config().is_near_destination_threshold()) {
    AERROR << "--Planning init point is close to destination, skip new "
              "trajectory generation";
    return Status(ErrorCode::OK,
                  "Planning init point is close to destination, skip new "
                  "trajectory generation");
  }

  // init x, y, z would be rotated.
  double init_x = start_pose_[0];
  double init_y = start_pose_[1];
  double init_phi = common::math::NormalizeAngle(start_pose[2] - rotate_angle);

  // Result container for warm start (initial velocity is assumed to be 0 for now)
  HybridAStartResult result;
  const auto astar_start_timestamp = std::chrono::system_clock::now();
  warm_start_->SetIsMapBoundary(use_map_bound);

  // plan with dynamic adjust kappa
  if (FLAGS_enable_teb_plan_with_dynamic_adjust_kappa && use_kappa_contraint_) {
    AINFO << "enable_teb_with_dynamic_adjust_kappa and use_kappa_contraint";
    const auto& status = ProcessTebPlanWithKappa(
        world_obstacles_vertices_vec, init_x, init_y, init_phi, XYbounds,
        rotate_angle, translate_origin, obstacles_vertices_vec, obstacles,
        trajectory_stitching_point, result, start_collision_flag);
    if (Status::OK() != status) {
      return Status(ErrorCode::PLANNING_ERROR, status.error_message());
    }
  } else {
    // WYQ_Mark_Process 2
    warm_start_->ResetDefaultKappaContraint();
    ADEBUG << "use pure astar control"
           << config_.planner_open_space_config().use_pure_astar();
    // All obstacle point sets：obstacles_vertices_vec
    if (warm_start_->Plan(world_obstacles_vertices_vec, init_x, init_y,
                          init_phi, end_pose[0], end_pose[1], end_pose[2],
                          rotate_angle, translate_origin, XYbounds, obstacles,
                          obstacles_vertices_vec, &result,
                          config_.planner_open_space_config().use_pure_astar(),
                          start_collision_flag)) {
      AINFO << "hybrid astar warm start problem solved successfully!";
    } else {
      optimized_trajectory_.clear();
      AERROR << "hybrid astar warm start problem failed to solve";
        for (const auto& it : openspace_common_ptr_->GetHADebugStatus()) {
          AINFO << "  reason = " << static_cast<int>(it);
        }

      // record debug info
      if (FLAGS_enable_openspace_record_debug) {
        open_space_debug_.Clear();
        RecordDebugInfo(result, trajectory_stitching_point, translate_origin,
                        rotate_angle, start_pose, end_pose, XYbounds,
                        obstacles_vertices_vec);
      }

      return Status(ErrorCode::PLANNING_ERROR,
                    "State warm start problem failed to solve");
    }
  }

  const auto start_timestamp = std::chrono::system_clock::now();

  std::chrono::duration<double> diff1 = start_timestamp - astar_start_timestamp;
  AINFO << "warm start use time  " << diff1.count() * 1000.0 << " ms.";

  PoseSE2 start(start_pose[0], start_pose[1],
                common::math::NormalizeAngle(start_pose[2] - rotate_angle));

  PoseSE2 end(end_pose[0], end_pose[1], end_pose[2]);

  teb_obstacles_.clear();

  if (FLAGS_enable_use_origin_obstacle) {
    ConvertObstacleToPolygon(obstacles, rotate_angle, translate_origin);
  }

  ConvertObstacleToLine(boundary_vertices_vec);

  ConvertStaticAreaToPolygon(costmap_obstacles, rotate_angle, translate_origin);

  Vec2d adc_position{start_pose[0], start_pose[1]};
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  const double adc_length = vehicle_config.vehicle_param().length();
  const double adc_width = vehicle_config.vehicle_param().width();
  const double back_edge = vehicle_config.vehicle_param().back_edge_to_center();
  Box2d adc_box(adc_position, start.theta(), adc_length, adc_width);
  double shift_distance = adc_length * 0.5 - back_edge;
  Vec2d shift_vec{shift_distance * std::cos(start.theta()),
                  shift_distance * std::sin(start.theta())};
  adc_box.Shift(shift_vec);
  const auto& corners = adc_box.GetAllCorners();

  Point2dContainer adc_points;
  for (const auto corner : corners) {
    adc_points.emplace_back(Eigen::Vector2d(corner.x() - start_pose[0],
                                            corner.y() - start_pose[1]));
  }

  adc_model_ = boost::make_shared<PolygonRobotFootprint>(adc_points);
  teb_planner_->ClearPlanner();

  Twist robot_vel;
  robot_vel.linear.x() = 0;
  robot_vel.linear.y() = 0;
  robot_vel.linear.z() = 0;
  robot_vel.angular.x() = 0;
  robot_vel.angular.y() = 0;
  robot_vel.angular.z() = 0;
  optimized_trajectory_.clear();

  std::vector<HybridAStartResult> partition_trajectories;
  if (!warm_start_->TrajectoryPartition(result, &partition_trajectories)) {
    AERROR << "Hybrid Astar partition failed";
    return Status(ErrorCode::PLANNING_ERROR, "Hybrid Astar partition failed");
  }

  if (result.x.size() <= 1) {
    AERROR << "Hybrid Astar planning failed";
    return Status(ErrorCode::PLANNING_ERROR, "Hybrid Astar planning failed");
  }
  size_t size = partition_trajectories.size();
  double last_relative_time = 0;
  const auto& status = ProcessPartitionTrajectories(
      partition_trajectories, size, result, rotate_angle, translate_origin,
      last_relative_time, robot_vel);
  if (Status::OK() != status) {
    return Status(ErrorCode::PLANNING_ERROR, status.error_message());
  }

  // double set for protect
  for (size_t i = 0; i < optimized_trajectory_.size(); ++i) {
    optimized_trajectory_[i].set_relative_time(i * kTimeInterval);
  }

  const auto end_timestamp0 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff0 = end_timestamp0 - start_timestamp;
  AINFO << "open space trajectory smoother  time before combine: "
        << diff0.count() * 1000.0 << " ms.";

  // record debug info
  if (FLAGS_enable_openspace_record_debug) {
    open_space_debug_.Clear();
    RecordDebugInfo(result, trajectory_stitching_point, translate_origin,
                    rotate_angle, start_pose, end_pose, XYbounds,
                    obstacles_vertices_vec);
  }

  return Status::OK();
}

common::Status TEBTrajectoryOptimizer::ProcessPartitionTrajectories(
    const std::vector<HybridAStartResult>& partition_trajectories,
    const size_t& size, const HybridAStartResult& result, double rotate_angle,
    const Vec2d& translate_origin, double& last_relative_time,
    Twist& robot_vel) {
  for (size_t j = 0; j < size; ++j) {
    const auto partition_result = partition_trajectories[j];
    transformed_plan_.clear();
    PoseStamped pose_stamped;
    DiscretizedPath path_data;
    for (size_t i = 0; i < partition_result.x.size(); ++i) {
      pose_stamped.header.seq = i;
      pose_stamped.header.stamp = i;
      pose_stamped.header.frame_id = i;

      pose_stamped.pose.position.x = partition_result.x[i];
      pose_stamped.pose.position.y = partition_result.y[i];
      pose_stamped.pose.position.z = 0;
      ADEBUG << "pose_stamped.pose.position.x=="
             << pose_stamped.pose.position.x;
      pose_stamped.pose.orientation.x = 0;
      pose_stamped.pose.orientation.y = 0;
      pose_stamped.pose.orientation.z = 0;
      // temp use , interface is also use orientation.w for phi
      pose_stamped.pose.orientation.w = partition_result.phi[i];
      transformed_plan_.emplace_back(pose_stamped);
    }
    double dx = partition_result.x.back() - partition_result.x.front();
    double dy = partition_result.y.back() - partition_result.y.front();
    AINFO << "partition traj ds^2 " << dx * dx + dy * dy;

    if (FLAGS_enable_reuse_teb_plan) {
      AINFO << "is reverse traj , skip teb plan";
      std::vector<std::pair<double, double>> xy_points;
      for (size_t i = 0; i < result.x.size(); ++i) {
        Vec2d path_point(result.x[i], result.y[i]);
        path_point.SelfRotate(rotate_angle);
        path_point += translate_origin;
        xy_points.emplace_back(path_point.x(), path_point.y());
      }
      if (!SetPathProfile(xy_points, &path_data)) {
        const std::string msg = "teb plan error2";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
      // add speed and use origin theta
      // to refine the last point relative time
      for (size_t i = 0; i < path_data.size(); ++i) {
        common::TrajectoryPoint trajectory_point;
        trajectory_point.mutable_path_point()->CopyFrom(path_data[i]);

        trajectory_point.set_relative_time(kTimeInterval * i);
        trajectory_point.set_v(result.v[i]);
        ADEBUG << "trajectory_point.mutable_path_point()->theta() "
               << trajectory_point.mutable_path_point()->theta()
               << "result.phi [i]" << result.phi[i];
        trajectory_point.mutable_path_point()->set_theta(
            NormalizeAngle(result.phi[i] + rotate_angle));
        trajectory_point.set_a(0);
        optimized_trajectory_.AppendTrajectoryPoint(trajectory_point);
      }
      break;
    }

    if (((0 != j && j != size - 1) && dx * dx + dy * dy < 1.0 * 1.0) ||
        partition_result.x.size() < kMinSize) {
      AINFO << partition_result.x.size() << "point is too less, continue ";
      continue;
    }

    AINFO << "result.v.at(0)" << result.v.at(0) << "result.v.at(1)"
          << result.v[1] << "result.x.size() " << result.x.size()
          << "partition_trajectories size " << size;
    if ((partition_result.v[1] < -kEpsilon && 0 == j) ||
        (partition_result.x.size() <= kMinSize && 0 == j)) {
      AINFO << "is reverse traj , skip teb plan";
      std::vector<std::pair<double, double>> xy_points;
      for (size_t i = 0; i < partition_result.x.size(); ++i) {
        Vec2d path_point(partition_result.x[i], partition_result.y[i]);
        path_point.SelfRotate(rotate_angle);
        path_point += translate_origin;
        xy_points.emplace_back(path_point.x(), path_point.y());
      }
      if (!SetPathProfile(xy_points, &path_data)) {
        const std::string msg = "teb plan error2";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
      // add speed and use origin theta
      // to refine the last point relative time
      for (size_t i = 0; i < path_data.size(); ++i) {
        common::TrajectoryPoint trajectory_point;
        trajectory_point.mutable_path_point()->CopyFrom(path_data[i]);

        trajectory_point.set_relative_time(last_relative_time +
                                           kTimeInterval * i);
        if (i == path_data.size() - 1) {
          last_relative_time += kTimeInterval * i + kTimeInterval;
          trajectory_point.set_v(0);
        } else {
          trajectory_point.set_v(kReverseSpeed);
        }
        trajectory_point.mutable_path_point()->set_theta(
            NormalizeAngle(partition_result.phi[i] + rotate_angle));
        trajectory_point.set_a(0);
        optimized_trajectory_.AppendTrajectoryPoint(trajectory_point);
      }
      AINFO << "is reverse traj , skip teb plan";
      continue;
    }
    const auto start_timestamp = std::chrono::system_clock::now();
    teb_planner_->ClearPlanner();
    teb_planner_->Plan(transformed_plan_, &robot_vel,
                       teb_config_.goal_tolerance.free_goal_vel);
    const auto start_timestamp2 = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = start_timestamp2 - start_timestamp;
    AINFO << "teb plan  use time  " << diff.count() * 1000.0 << " ms.";
    // vi
    std::vector<Eigen::Vector3f> path;
    teb_planner_->GetFullTrajectory(&path);
    std::vector<Twist> speed_info;
    teb_planner_->GetProtoTrajInfo(&speed_info);
    AINFO << "path.size()" << path.size() << "speed_info.size()"
          << speed_info.size();
    if (path.size() < 2 || speed_info.size() < 2) {
      const std::string msg = "teb plan error";
      AERROR << msg;
      optimized_trajectory_.clear();
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    std::vector<std::pair<double, double>> xy_points;
    for (size_t i = 0; i < path.size(); ++i) {
      Vec2d path_point(path[i][0], path[i][1]);
      path_point.SelfRotate(rotate_angle);
      path_point += translate_origin;
      xy_points.emplace_back(path_point.x(), path_point.y());
    }

    if (!SetPathProfile(xy_points, &path_data)) {
      const std::string msg = "teb plan error2";
      AERROR << msg;
      optimized_trajectory_.clear();
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    // add speed and use origin theta
    // to refine the last point relative time
    for (size_t i = 0; i < path_data.size(); ++i) {
      common::TrajectoryPoint trajectory_point;
      trajectory_point.mutable_path_point()->CopyFrom(path_data[i]);
      ADEBUG << "path x " << std::setprecision(9)
             << trajectory_point.path_point().x();
      ADEBUG << "path y " << std::setprecision(9)
             << trajectory_point.path_point().y();
      trajectory_point.set_relative_time(last_relative_time +
                                         speed_info[i].linear.y());

      if (i == path_data.size() - 1) {
        last_relative_time += speed_info[i].linear.y() + kTimeInterval;
      }
      trajectory_point.set_v(speed_info[i].linear.x());
      trajectory_point.mutable_path_point()->set_theta(
          NormalizeAngle(path[i][2] + rotate_angle));
      if (speed_info[i].linear.x() < 0) {
        trajectory_point.mutable_path_point()->set_theta(NormalizeAngle(
            trajectory_point.mutable_path_point()->theta() + M_PI));
      }
      trajectory_point.set_a(speed_info[i].linear.z());
      double kappa =
          (speed_info[i].linear.x() < 0 ? -1 : 1) * path_data[i].kappa();

      trajectory_point.mutable_path_point()->set_kappa(kappa);

      optimized_trajectory_.AppendTrajectoryPoint(trajectory_point);
    }
  }
  return Status::OK();
}

common::Status TEBTrajectoryOptimizer::ProcessTebPlanWithKappa(
    const std::vector<std::vector<Vec2d>>& world_obstacles_vertices_vec,
    const double& init_x, const double& init_y, const double& init_phi,
    const std::vector<double>& XYbounds, double rotate_angle,
    const Vec2d& translate_origin,
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
    const std::vector<Obstacle>& obstacles,
    const common::TrajectoryPoint& traj_point, HybridAStartResult& result,
    bool* start_collision_flag) {
  double plan_time = 0.0;
  double max_plan_time = std::max(config_.planner_open_space_config()
                                      .warm_start_config()
                                      .astar_max_time_first(),
                                  config_.planner_open_space_config()
                                      .warm_start_config()
                                      .astar_max_time_second());
  for (size_t i = 0; i < kappa_contraint_configs_.size(); ++i) {
    AINFO << "now use kappa_contraint_configs_ is "
          << kappa_contraint_configs_[i].traj_kappa_contraint_ratio();
    warm_start_->SetKappaContraintConfig(kappa_contraint_configs_[i]);
    const double start_time = Clock::NowInSeconds();
    if (warm_start_->Plan(world_obstacles_vertices_vec, init_x, init_y,
                          init_phi, end_pose_[0], end_pose_[1], end_pose_[2],
                          rotate_angle, translate_origin, XYbounds, obstacles,
                          obstacles_vertices_vec, &result, false,
                          start_collision_flag)) {
      AINFO << "dynamic adjust kappa for hybrid astar solved successfully!";
      const double end_time = Clock::NowInSeconds();
      plan_time += end_time - start_time;
      AINFO << "this plan diff time  is " << (end_time - start_time) << " s.";
      AINFO << "total plan time is " << plan_time << " s.";
      break;
    } else {
      optimized_trajectory_.clear();
      const double end_time = Clock::NowInSeconds();
      plan_time += end_time - start_time;
      AINFO << "this hybrid astar solved failed!"
            << " index = " << i;
      for (const auto& it : openspace_common_ptr_->GetHADebugStatus()) {
        AINFO << "  reason = " << static_cast<int>(it);
      }
      AINFO << "now diff time  is " << (end_time - start_time) << " s.";
      AINFO << "now plan_time is " << plan_time << " s.";
    }

    if (plan_time > max_plan_time ||
        (i == kappa_contraint_configs_.size() - 1)) {
      optimized_trajectory_.clear();
      AERROR << "dynamic adjust kappa for hybrid astar failed. plan_time is "
             << plan_time << " s.";

      // record debug info
      if (FLAGS_enable_openspace_record_debug) {
        open_space_debug_.Clear();
        RecordDebugInfo(result, traj_point, translate_origin,
                        rotate_angle, start_pose_, end_pose_, XYbounds,
                        obstacles_vertices_vec);
      }

      return Status(ErrorCode::PLANNING_ERROR,
                    "dynamic adjust kappa for hybrid astar failed.");
    }
  }
  return Status::OK();
}

void TEBTrajectoryOptimizer::PathPointNormalizing(double rotate_angle,
                                                  const Vec2d& translate_origin,
                                                  double* x, double* y,
                                                  double* phi) {
  *x -= translate_origin.x();
  *y -= translate_origin.y();
  double tmp_x = *x;
  *x = (*x) * std::cos(-rotate_angle) - (*y) * std::sin(-rotate_angle);
  *y = tmp_x * std::sin(-rotate_angle) + (*y) * std::cos(-rotate_angle);
  *phi = common::math::NormalizeAngle(*phi - rotate_angle);
}

void TEBTrajectoryOptimizer::RecordDebugInfo(
    const HybridAStartResult& result,
    const common::TrajectoryPoint& trajectory_stitching_point,
    const Vec2d& translate_origin, const double rotate_angle,
    const std::vector<double>& start_pose, const std::vector<double>& end_pose,
    const std::vector<double>& XYbounds,
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec) {
  open_space_debug_.Clear();
  // load information about trajectory stitching point
  auto* a_star_search_info_ptr = open_space_debug_.mutable_hybrid_search_info();
  a_star_search_info_ptr->CopyFrom(trajectory_stitching_point);
  // load translation origin and heading angle
  auto* roi_shift_point = a_star_search_info_ptr->mutable_roi_shift_point();
  // pathpoint
  roi_shift_point->mutable_path_point()->set_x(translate_origin.x());
  roi_shift_point->mutable_path_point()->set_y(translate_origin.y());
  roi_shift_point->mutable_path_point()->set_theta(rotate_angle);

  // load start_pose into debug
  double start_phi = common::math::NormalizeAngle(start_pose[2] - rotate_angle);
  auto* start_point = open_space_debug_.mutable_hybrid_search_info()
                          ->mutable_start_point();
  start_point->mutable_path_point()->set_x(start_pose[0]);
  start_point->mutable_path_point()->set_y(start_pose[1]);
  start_point->mutable_path_point()->set_theta(start_phi);

  // load end_pose into debug
  auto* end_point =
      open_space_debug_.mutable_hybrid_search_info()->mutable_end_point();
  end_point->mutable_path_point()->set_x(end_pose[0]);
  end_point->mutable_path_point()->set_y(end_pose[1]);
  end_point->mutable_path_point()->set_theta(end_pose[2]);
  end_point->set_v(end_pose[3]);

  RecordWarmStartInfo(result);

  double relative_time = 0;
  // load smoothed trajectory
  size_t horizon = optimized_trajectory_.size();
  auto* a_star_optimization_info_ptr =
      open_space_debug_.mutable_astar_optimization_info();
  auto* smoothed_trajectory =
      a_star_optimization_info_ptr->mutable_smoothed_trajectory();
  smoothed_trajectory->Clear();
  for (size_t i = 0; i < horizon; ++i) {
    auto* smoothed_point = smoothed_trajectory->add_vehicle_motion_point();

    double init_x = optimized_trajectory_[i].path_point().x();
    double init_y = optimized_trajectory_[i].path_point().y();
    double init_phi = optimized_trajectory_[i].path_point().theta();
    // Rotate and scale the state
    PathPointNormalizing(rotate_angle, translate_origin, &init_x, &init_y,
                         &init_phi);

    smoothed_point->mutable_trajectory_point()->mutable_path_point()->set_x(
        init_x);
    smoothed_point->mutable_trajectory_point()->mutable_path_point()->set_y(
        init_y);
    smoothed_point->mutable_trajectory_point()->mutable_path_point()->set_theta(
        init_phi);

    smoothed_point->mutable_trajectory_point()->set_v(
        optimized_trajectory_[i].v());

    if (i != horizon - 1) {
      smoothed_point->set_steer(optimized_trajectory_[i].steer());
      smoothed_point->mutable_trajectory_point()->set_a(
          optimized_trajectory_[i].a());
      relative_time += optimized_trajectory_[i].relative_time();
      smoothed_point->mutable_trajectory_point()->set_relative_time(
          relative_time);
    }
  }

  // load xy boundary (xmin, xmax, ymin, ymax)
  open_space_debug_.clear_xy_boundary();
  open_space_debug_.add_xy_boundary(XYbounds[0]);
  open_space_debug_.add_xy_boundary(XYbounds[1]);
  open_space_debug_.add_xy_boundary(XYbounds[2]);
  open_space_debug_.add_xy_boundary(XYbounds[3]);

  // load obstacles
  open_space_debug_.clear_obstacles();
  for (const auto& obstacle_vertices : obstacles_vertices_vec) {
    auto* obstacle_ptr = open_space_debug_.add_obstacles();
    for (const auto& vertex : obstacle_vertices) {
      obstacle_ptr->add_vertices_x_coords(vertex.x());
      obstacle_ptr->add_vertices_y_coords(vertex.y());
    }
  }
}

// new
Status TEBTrajectoryOptimizer::UpdateTEB(
    const std::vector<common::TrajectoryPoint>& stitching_trajectory,
    const std::vector<double>& start_pose, const std::vector<double>& end_pose,
    const std::vector<double>& XYbounds, double rotate_angle,
    const Vec2d& translate_origin, const Eigen::MatrixXi& obstacles_edges_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
    const std::vector<Obstacle>& obstacles,
    const std::vector<std::vector<Vec2d>>& boundary_vertices_vec,
    const StaticAreaPolygons& costmap_obstacles) {
  if (XYbounds.empty() || end_pose.empty() || obstacles_edges_num.cols() == 0 ||
      obstacles_A.cols() == 0 || obstacles_b.cols() == 0 ||
      start_pose.size() < 2) {
    AERROR << "TEBTrajectoryOptimizer input data not ready";
    return Status(ErrorCode::PLANNING_ERROR,
                  "TEBTrajectoryOptimizer input data not ready");
  }

  // Initiate initial states
  stitching_trajectory_ = stitching_trajectory;

  // Init trajectory point is the stitching point from last trajectory
  const common::TrajectoryPoint trajectory_stitching_point =
      stitching_trajectory.back();

  start_pose_ = start_pose;
  end_pose_ = end_pose;
  double dx = end_pose[0] - start_pose[0];
  double dy = end_pose[1] - start_pose[1];
  double ds = std::hypot(dx, dy);
  ADEBUG << " stitching_trajectory_ " << stitching_trajectory_.size()
         << " optimized_trajectory_ " << optimized_trajectory_.NumOfPoints()
         << "ds = " << ds;
  // Generate Stop trajectory if init point close to destination
  if (ds <
      config_.planner_open_space_config().is_near_destination_threshold()) {
    AERROR << "Planning init point is close to destination, skip new "
              "trajectory generation";
    rescue_status_->set_reached_destination(true);
    return Status(ErrorCode::OK,
                  "---Planning init point is close to destination, skip new "
                  "trajectory generation");
  }

  // TebConfig config;
  PoseSE2 start(start_pose[0], start_pose[1],
                common::math::NormalizeAngle(start_pose[2] - rotate_angle));
  PoseSE2 end(end_pose[0], end_pose[1], end_pose[2]);

  teb_obstacles_.clear();

  if (FLAGS_enable_use_origin_obstacle) {
    ConvertObstacleToPolygon(obstacles, rotate_angle, translate_origin);
  }

  ConvertObstacleToLine(boundary_vertices_vec);

  if (FLAGS_enable_use_costmap) {
    ConvertStaticAreaToPolygon(costmap_obstacles, rotate_angle,
                               translate_origin);
  }

  Vec2d adc_position{start_pose[0], start_pose[1]};
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  const double adc_length = vehicle_config.vehicle_param().length();
  const double adc_width = vehicle_config.vehicle_param().width();
  const double back_edge = vehicle_config.vehicle_param().back_edge_to_center();
  Box2d adc_box(adc_position, start.theta(), adc_length, adc_width);
  double shift_distance = adc_length * 0.5 - back_edge;
  Vec2d shift_vec{shift_distance * std::cos(start.theta()),
                  shift_distance * std::sin(start.theta())};
  adc_box.Shift(shift_vec);

  const auto& corners = adc_box.GetAllCorners();
  AINFO << "start_pose[0] " << start_pose[0] << "start_pose[1] "
        << start_pose[1];

  Twist robot_vel;
  robot_vel.linear.x() = start_pose_[3];
  robot_vel.linear.y() = 0;
  robot_vel.linear.z() = 0;
  robot_vel.angular.x() = 0;
  robot_vel.angular.y() = 0;
  robot_vel.angular.z() = 0;

  const auto start_timestamp = std::chrono::system_clock::now();
  teb_planner_->Plan(start, end, &robot_vel,
                     teb_config_.goal_tolerance.free_goal_vel);
  const auto start_timestamp2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = start_timestamp2 - start_timestamp;
  AINFO << "teb plan  use time  " << diff.count() * 1000.0 << " ms.";
  // vi
  std::vector<Eigen::Vector3f> path;
  teb_planner_->GetFullTrajectory(&path);
  std::vector<Twist> speed_info;
  teb_planner_->GetProtoTrajInfo(&speed_info);
  velocity_profile_.clear();
  teb_planner_->GetVelocityProfile(&velocity_profile_);
  AINFO << "path.size()" << path.size() << "speed_info.size()"
        << speed_info.size();
  if (path.size() < 2 || speed_info.size() < 2) {
    const std::string msg = "teb plan error";
    AERROR << msg;
    optimized_trajectory_.clear();
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  std::vector<std::pair<double, double>> xy_points;
  for (size_t i = 0; i < path.size(); ++i) {
    Vec2d path_point(path[i][0], path[i][1]);
    path_point.SelfRotate(rotate_angle);
    path_point += translate_origin;
    xy_points.emplace_back(path_point.x(), path_point.y());
  }

  DiscretizedPath path_data;
  if (!SetPathProfile(xy_points, &path_data)) {
    const std::string msg = "teb plan error2";
    AERROR << msg;
    optimized_trajectory_.clear();
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // add speed and use origin theta
  // to refine the last point relative time
  optimized_trajectory_.clear();

  Vec2d adc_world_pose(start_pose[0], start_pose[1]);
  adc_world_pose.SelfRotate(rotate_angle);
  adc_world_pose += translate_origin;

  for (size_t i = 0; i < path_data.size(); ++i) {
    common::TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_data[i]);
    ADEBUG << "path x " << std::setprecision(9)
           << trajectory_point.path_point().x();
    ADEBUG << "path y " << std::setprecision(9)
           << trajectory_point.path_point().y();

    if (i + 1 == path_data.size()) {
      AINFO << "path x " << std::setprecision(9)
            << trajectory_point.path_point().x();
      AINFO << "path y " << std::setprecision(9)
            << trajectory_point.path_point().y();
    }

    double v = speed_info[i].linear.x();
    v = std::min(std::max(kReverseSpeed, v), teb_config_.robot.max_vel_x);

    trajectory_point.set_relative_time(i * kTimeInterval);
    trajectory_point.set_v(v);

    ADEBUG << "speed_info[i].linear.y() t" << speed_info[i].linear.y();
    ADEBUG << "speed_info[i].linear.x() v" << speed_info[i].linear.x();
    ADEBUG << "i " << i;

    ADEBUG << "path theta " << NormalizeAngle(path[i][2] + rotate_angle)
           << "trajectory_theta"
           << trajectory_point.mutable_path_point()->theta();
    trajectory_point.mutable_path_point()->set_theta(
        NormalizeAngle(path[i][2] + rotate_angle));

    double kappa =
        (speed_info[i].linear.x() < 0 ? -1 : 1) * path_data[i].kappa();

    trajectory_point.mutable_path_point()->set_kappa(kappa);
    trajectory_point.set_a(0);
    optimized_trajectory_.AppendTrajectoryPoint(trajectory_point);
  }

  const auto end_timestamp0 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff0 = end_timestamp0 - start_timestamp;
  AINFO << "open space trajectory smoother  time before combine: "
        << diff0.count() * 1000.0 << " ms.";

  // record debug info
  if (FLAGS_enable_openspace_record_debug) {
    open_space_debug_.Clear();
    HybridAStartResult result;
    RecordDebugInfo(result, trajectory_stitching_point, translate_origin,
                    rotate_angle, start_pose, end_pose, XYbounds,
                    obstacles_vertices_vec);
  }

  return Status::OK();
}

void TEBTrajectoryOptimizer::ConvertObstacleToLine(
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec) {
  for (const auto& obstacle_vertices : obstacles_vertices_vec) {
    size_t vertices_num = obstacle_vertices.size();
    for (size_t i = 0; i + 1 < vertices_num; ++i) {
      teb_obstacles_.emplace_back(boost::make_shared<LineObstacle>(
          obstacle_vertices[i].x(), obstacle_vertices[i].y(),
          obstacle_vertices[i + 1].x(), obstacle_vertices[i + 1].y()));
    }
  }
}

void TEBTrajectoryOptimizer::ConvertObstacleToCircle(
    const std::vector<Obstacle>& obstacles, double rotate_angle,
    const common::math::Vec2d& translate_origin) {
  for (const auto& obstacle : obstacles) {
    auto obstacle_box = obstacle.PerceptionBoundingBox();
    Vec2d center_point(obstacle_box.center().x(), obstacle_box.center().y());
    center_point -= translate_origin;
    center_point.SelfRotate(-rotate_angle);
    ADEBUG << "obstacle_box.center().x() " << center_point.x();
    ADEBUG << "obstacle_box.center().y() " << center_point.y();
    teb_obstacles_.emplace_back(boost::make_shared<CircularObstacle>(
        center_point.x(), center_point.y(),
        std::hypot(0.5 * obstacle_box.length(), 0.5 * obstacle_box.width())));
  }
}

void TEBTrajectoryOptimizer::ConvertObstacleToPolygon(
    const std::vector<Obstacle>& obstacles, double rotate_angle,
    const common::math::Vec2d& translate_origin) {
  for (const auto& obstacle : obstacles) {
    Point2dContainer obs_points;
    for (auto pt : obstacle.PerceptionPolygon().points()) {
      pt -= translate_origin;
      pt.SelfRotate(-rotate_angle);
      obs_points.emplace_back(Eigen::Vector2d(pt.x(), pt.y()));
    }
    if (obs_points.size() < 3) {
      AERROR << "waring : obs_points.size() < 3";
    }
    teb_obstacles_.emplace_back(
        boost::make_shared<PolygonObstacle>(obs_points));
  }
}

void TEBTrajectoryOptimizer::ConvertStaticAreaToPolygon(
    const StaticAreaPolygons& costmap_obstacles, double rotate_angle,
    const common::math::Vec2d& translate_origin) {
  int count = 0;
  for (const auto& obstacle : costmap_obstacles) {
    count++;
    Point2dContainer obs_points;
    // TODO(all): to use type   obstacle.first()
    for (auto pt : obstacle.second) {
      Vec2d point(pt.x(), pt.y());
      point -= translate_origin;
      point.SelfRotate(-rotate_angle);
      obs_points.emplace_back(Eigen::Vector2d(point.x(), point.y()));
    }
    if (obs_points.size() < 3) {
      AERROR << "waring : obs_points.size() < 3";
      continue;
    }
    teb_obstacles_.emplace_back(
        boost::make_shared<PolygonObstacle>(obs_points));
  }
  AINFO << "costmap_obstacles num  " << count;
}

void TEBTrajectoryOptimizer::ConvertViaPoints() {
  via_points_.clear();
  if (start_pose_.empty() || end_pose_.empty()) {
    AERROR << "start_pose_.empty() || end_pose_.empty()";
    return;
  }

  double dx = end_pose_[0] - start_pose_[0];
  double dy = end_pose_[1] - start_pose_[1];

  double ds = std::hypot(dx, dy);
  double interval_s = 1;
  if (ds >= interval_s) {
    for (double i = interval_s; i + 0.5 * interval_s < ds; i += interval_s) {
      double point_x =
          start_pose_[0] + (i / ds) * (end_pose_[0] - start_pose_[0]);
      double point_y =
          start_pose_[1] + (i / ds) * (end_pose_[1] - start_pose_[1]);
      via_points_.emplace_back(Eigen::Vector2d(point_x, point_y));
    }
  }
  via_points_.emplace_back(Eigen::Vector2d(end_pose_[0], end_pose_[1]));
}

bool TEBTrajectoryOptimizer::SetPathProfile(
    const std::vector<std::pair<double, double>>& point2d,
    DiscretizedPath* raw_path_points) {
  CHECK_NOTNULL(raw_path_points);
  raw_path_points->clear();
  // Compute path profile
  std::vector<double> headings;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  std::vector<double> accumulated_s;
  if (!DiscretePointsMath::ComputePathProfile(
          point2d, &headings, &accumulated_s, &kappas, &dkappas)) {
    return false;
  }
  CHECK_EQ(point2d.size(), headings.size());
  CHECK_EQ(point2d.size(), kappas.size());
  CHECK_EQ(point2d.size(), dkappas.size());
  CHECK_EQ(point2d.size(), accumulated_s.size());

  // Load into path point
  size_t points_size = point2d.size();
  for (size_t i = 0; i < points_size; ++i) {
    PathPoint path_point;
    path_point.set_x(point2d[i].first);
    path_point.set_y(point2d[i].second);
    path_point.set_theta(headings[i]);
    path_point.set_s(accumulated_s[i]);
    path_point.set_kappa(kappas[i]);
    AINFO << "kappa " << kappas[i] << "dkappa[i]" << dkappas[i] << "s"
          << accumulated_s[i];
    path_point.set_dkappa(dkappas[i]);
    raw_path_points->emplace_back(std::move(path_point));
  }
  return true;
}

void TEBTrajectoryOptimizer::ConvertToTebConfig() {
  ConvertToTebConfigTrajectory();
  ConvertToTebConfigRobot();
  ConvertToTebConfigGoalTolerance();
  ConvertToTebConfigObstacles();
  ConvertToTebConfigOptimization();
  ConvertToTebConfigHomotopy();
  ConvertToTebConfigRecovery();
  return;
}

void TEBTrajectoryOptimizer::ConvertToTebConfigTrajectory() {
  const PlannerTebConfig& teb_config = config_.planner_teb_config();

  teb_config_.odom_topic = teb_config.odom_topic();
  teb_config_.map_frame = teb_config.map_frame();

  // Trajectory
  auto teb_trajectory = teb_config.trajectory();
  teb_config_.trajectory.teb_autosize = teb_trajectory.teb_autosize();
  teb_config_.trajectory.dt_ref = teb_trajectory.dt_ref();
  teb_config_.trajectory.dt_hysteresis = teb_trajectory.dt_hysteresis();
  teb_config_.trajectory.min_samples = teb_trajectory.min_samples();
  teb_config_.trajectory.max_samples = teb_trajectory.max_samples();
  teb_config_.trajectory.global_plan_overwrite_orientation =
      teb_trajectory.global_plan_overwrite_orientation();
  teb_config_.trajectory.allow_init_with_backwards_motion =
      teb_trajectory.allow_init_with_backwards_motion();
  teb_config_.trajectory.global_plan_viapoint_sep =
      teb_trajectory.global_plan_viapoint_sep();
  teb_config_.trajectory.via_points_ordered =
      teb_trajectory.via_points_ordered();
  teb_config_.trajectory.max_global_plan_lookahead_dist =
      teb_trajectory.max_global_plan_lookahead_dist();
  teb_config_.trajectory.global_plan_prune_distance =
      teb_trajectory.global_plan_prune_distance();
  teb_config_.trajectory.exact_arc_length = teb_trajectory.exact_arc_length();
  teb_config_.trajectory.force_reinit_new_goal_dist =
      teb_trajectory.force_reinit_new_goal_dist();
  teb_config_.trajectory.force_reinit_new_goal_angular =
      teb_trajectory.force_reinit_new_goal_angular();
  teb_config_.trajectory.feasibility_check_no_poses =
      teb_trajectory.feasibility_check_no_poses();
  teb_config_.trajectory.publish_feedback = teb_trajectory.publish_feedback();
  teb_config_.trajectory.min_resolution_collision_check_angular =
      teb_trajectory.min_resolution_collision_check_angular();
  teb_config_.trajectory.control_look_ahead_poses =
      teb_trajectory.control_look_ahead_poses();
}

void TEBTrajectoryOptimizer::ConvertToTebConfigRobot() {
  const PlannerTebConfig& teb_config = config_.planner_teb_config();

  teb_config_.odom_topic = teb_config.odom_topic();
  teb_config_.map_frame = teb_config.map_frame();

  // Robot
  auto teb_robot = teb_config.robot();
  teb_config_.robot.max_vel_x = teb_robot.max_vel_x();
  teb_config_.robot.max_vel_x_backwards = teb_robot.max_vel_x_backwards();
  teb_config_.robot.max_vel_y = teb_robot.max_vel_y();
  teb_config_.robot.max_vel_theta = teb_robot.max_vel_theta();
  teb_config_.robot.acc_lim_x = teb_robot.acc_lim_x();
  teb_config_.robot.acc_lim_y = teb_robot.acc_lim_y();
  teb_config_.robot.acc_lim_theta = teb_robot.acc_lim_theta();
  teb_config_.robot.jerk_lim_x = teb_robot.jerk_lim_x();
  teb_config_.robot.jerk_lim_theta = teb_robot.jerk_lim_theta();
  teb_config_.robot.min_turning_radius = teb_robot.min_turning_radius();
  teb_config_.robot.wheelbase = teb_robot.wheelbase();
  teb_config_.robot.cmd_angle_instead_rotvel =
      teb_robot.cmd_angle_instead_rotvel();
  teb_config_.robot.is_footprint_dynamic = teb_robot.is_footprint_dynamic();
}

void TEBTrajectoryOptimizer::ConvertToTebConfigGoalTolerance() {
  const PlannerTebConfig& teb_config = config_.planner_teb_config();

  teb_config_.odom_topic = teb_config.odom_topic();
  teb_config_.map_frame = teb_config.map_frame();

  // GoalTolerance
  auto teb_goal_tolerance = teb_config.goal_tolerance();
  teb_config_.goal_tolerance.xy_goal_tolerance =
      teb_goal_tolerance.xy_goal_tolerance();
  teb_config_.goal_tolerance.yaw_goal_tolerance =
      teb_goal_tolerance.yaw_goal_tolerance();
  teb_config_.goal_tolerance.free_goal_vel = teb_goal_tolerance.free_goal_vel();
  teb_config_.goal_tolerance.complete_global_plan =
      teb_goal_tolerance.complete_global_plan();
}

void TEBTrajectoryOptimizer::ConvertToTebConfigObstacles() {
  const PlannerTebConfig& teb_config = config_.planner_teb_config();

  teb_config_.odom_topic = teb_config.odom_topic();
  teb_config_.map_frame = teb_config.map_frame();

  // Obstacles
  auto teb_obstacles = teb_config.obstacles();
  teb_config_.obstacles.min_obstacle_dist = teb_obstacles.min_obstacle_dist();
  teb_config_.obstacles.inflation_dist = teb_obstacles.inflation_dist();
  teb_config_.obstacles.dynamic_obstacle_inflation_dist =
      teb_obstacles.dynamic_obstacle_inflation_dist();
  teb_config_.obstacles.include_dynamic_obstacles =
      teb_obstacles.include_dynamic_obstacles();
  teb_config_.obstacles.include_costmap_obstacles =
      teb_obstacles.include_costmap_obstacles();
  teb_config_.obstacles.costmap_obstacles_behind_robot_dist =
      teb_obstacles.costmap_obstacles_behind_robot_dist();
  teb_config_.obstacles.obstacle_poses_affected =
      teb_obstacles.obstacle_poses_affected();
  teb_config_.obstacles.legacy_obstacle_association =
      teb_obstacles.legacy_obstacle_association();
  teb_config_.obstacles.obstacle_association_force_inclusion_factor =
      teb_obstacles.obstacle_association_force_inclusion_factor();
  teb_config_.obstacles.obstacle_association_cutoff_s =
      teb_obstacles.obstacle_association_cutoff_s();
  teb_config_.obstacles.costmap_converter_plugin =
      teb_obstacles.costmap_converter_plugin();
  teb_config_.obstacles.costmap_converter_spin_thread =
      teb_obstacles.costmap_converter_spin_thread();
  teb_config_.obstacles.costmap_converter_rate =
      teb_obstacles.costmap_converter_rate();
}

void TEBTrajectoryOptimizer::ConvertToTebConfigOptimization() {
  const PlannerTebConfig& teb_config = config_.planner_teb_config();

  teb_config_.odom_topic = teb_config.odom_topic();
  teb_config_.map_frame = teb_config.map_frame();

  // Optimization
  auto teb_optim = teb_config.optim();
  teb_config_.optim.no_inner_iterations = teb_optim.no_inner_iterations();
  teb_config_.optim.no_outer_iterations = teb_optim.no_outer_iterations();
  teb_config_.optim.optimization_activate = teb_optim.optimization_activate();
  teb_config_.optim.optimization_verbose = teb_optim.optimization_verbose();
  teb_config_.optim.penalty_epsilon = teb_optim.penalty_epsilon();
  teb_config_.optim.weight_max_vel_x = teb_optim.weight_max_vel_x();
  teb_config_.optim.weight_max_vel_y = teb_optim.weight_max_vel_y();
  teb_config_.optim.weight_max_vel_theta = teb_optim.weight_max_vel_theta();
  teb_config_.optim.weight_acc_lim_x = teb_optim.weight_acc_lim_x();
  teb_config_.optim.weight_acc_lim_y = teb_optim.weight_acc_lim_y();
  teb_config_.optim.weight_acc_lim_theta = teb_optim.weight_acc_lim_theta();
  teb_config_.optim.weight_kinematics_nh = teb_optim.weight_kinematics_nh();
  teb_config_.optim.weight_kinematics_forward_drive =
      teb_optim.weight_kinematics_forward_drive();
  teb_config_.optim.weight_kinematics_turning_radius =
      teb_optim.weight_kinematics_turning_radius();
  teb_config_.optim.weight_optimaltime = teb_optim.weight_optimaltime();
  teb_config_.optim.weight_shortest_path = teb_optim.weight_shortest_path();
  teb_config_.optim.weight_obstacle = teb_optim.weight_obstacle();
  teb_config_.optim.weight_inflation = teb_optim.weight_inflation();
  teb_config_.optim.weight_dynamic_obstacle =
      teb_optim.weight_dynamic_obstacle();
  teb_config_.optim.weight_dynamic_obstacle_inflation =
      teb_optim.weight_dynamic_obstacle_inflation();
  teb_config_.optim.weight_viapoint = teb_optim.weight_viapoint();
  teb_config_.optim.weight_prefer_rotdir = teb_optim.weight_prefer_rotdir();

  teb_config_.optim.weight_adapt_factor = teb_optim.weight_adapt_factor();
  teb_config_.optim.obstacle_cost_exponent = teb_optim.obstacle_cost_exponent();

  teb_config_.optim.weight_jerk_lim_x = teb_optim.weight_jerk_lim_x();
  teb_config_.optim.weight_jerk_lim_theta = teb_optim.weight_jerk_lim_theta();
}

void TEBTrajectoryOptimizer::ConvertToTebConfigHomotopy() {
  const PlannerTebConfig& teb_config = config_.planner_teb_config();

  teb_config_.odom_topic = teb_config.odom_topic();
  teb_config_.map_frame = teb_config.map_frame();

  // Homotopy Class Planner
  auto teb_hcp = teb_config.hcp();
  teb_config_.hcp.enable_homotopy_class_planning =
      teb_hcp.enable_homotopy_class_planning();
  teb_config_.hcp.enable_multithreading = teb_hcp.enable_multithreading();
  teb_config_.hcp.simple_exploration = teb_hcp.simple_exploration();
  teb_config_.hcp.max_number_classes = teb_hcp.max_number_classes();
  teb_config_.hcp.selection_cost_hysteresis =
      teb_hcp.selection_cost_hysteresis();
  teb_config_.hcp.selection_prefer_initial_plan =
      teb_hcp.selection_prefer_initial_plan();
  teb_config_.hcp.selection_obst_cost_scale =
      teb_hcp.selection_obst_cost_scale();
  teb_config_.hcp.selection_viapoint_cost_scale =
      teb_hcp.selection_viapoint_cost_scale();
  teb_config_.hcp.selection_alternative_time_cost =
      teb_hcp.selection_alternative_time_cost();

  teb_config_.hcp.obstacle_keypoint_offset = teb_hcp.obstacle_keypoint_offset();
  teb_config_.hcp.obstacle_heading_threshold =
      teb_hcp.obstacle_heading_threshold();
  teb_config_.hcp.roadmap_graph_no_samples = teb_hcp.roadmap_graph_no_samples();
  teb_config_.hcp.roadmap_graph_area_width = teb_hcp.roadmap_graph_area_width();
  teb_config_.hcp.roadmap_graph_area_length_scale =
      teb_hcp.roadmap_graph_area_length_scale();
  teb_config_.hcp.h_signature_prescaler = teb_hcp.h_signature_prescaler();
  teb_config_.hcp.h_signature_threshold = teb_hcp.h_signature_threshold();
  teb_config_.hcp.switching_blocking_period =
      teb_hcp.switching_blocking_period();

  teb_config_.hcp.viapoints_all_candidates = teb_hcp.viapoints_all_candidates();

  teb_config_.hcp.visualize_hc_graph = teb_hcp.visualize_hc_graph();
  teb_config_.hcp.visualize_with_time_as_z_axis_scale =
      teb_hcp.visualize_with_time_as_z_axis_scale();
  teb_config_.hcp.delete_detours_backwards = teb_hcp.delete_detours_backwards();
  teb_config_.hcp.detours_orientation_tolerance =
      teb_hcp.detours_orientation_tolerance();
  teb_config_.hcp.length_start_orientation_vector =
      teb_hcp.length_start_orientation_vector();
  teb_config_.hcp.max_ratio_detours_duration_best_duration =
      teb_hcp.max_ratio_detours_duration_best_duration();
}

void TEBTrajectoryOptimizer::ConvertToTebConfigRecovery() {
  const PlannerTebConfig& teb_config = config_.planner_teb_config();

  teb_config_.odom_topic = teb_config.odom_topic();
  teb_config_.map_frame = teb_config.map_frame();

  // Recovery
  auto teb_recovery = teb_config.recovery();
  teb_config_.recovery.shrink_horizon_backup =
      teb_recovery.shrink_horizon_backup();
  teb_config_.recovery.shrink_horizon_min_duration =
      teb_recovery.shrink_horizon_min_duration();
  teb_config_.recovery.oscillation_recovery =
      teb_recovery.oscillation_recovery();
  teb_config_.recovery.oscillation_v_eps = teb_recovery.oscillation_v_eps();
  teb_config_.recovery.oscillation_omega_eps =
      teb_recovery.oscillation_omega_eps();
  teb_config_.recovery.oscillation_recovery_min_duration =
      teb_recovery.oscillation_recovery_min_duration();
  teb_config_.recovery.oscillation_filter_duration =
      teb_recovery.oscillation_filter_duration();
}

bool comp(size_t a, size_t b) { return (a < b); }

void TEBTrajectoryOptimizer::RecordWarmStartInfo(
    const HybridAStartResult& result) {
  // load warm start trajectory
  auto horizon = std::min(
      {result.x.size(), result.y.size(), result.phi.size(), result.v.size()},
      comp);
  // AINFO << "result.x.size(): " << result.x.size()
  //          << " result.y.size(): " << result.y.size()
  //          << " result.phi.size(): " << result.phi.size()
  //          << " result.v.size(): " << result.v.size()
  //          << " result.steer.size(): " << result.steer.size()
  //          << " result.a.size(): " << result.a.size();
  // AINFO << "RecordWarmStartInfo_horizon: " << horizon;
  if (horizon) {
    open_space_debug_.set_is_plan_success(true);
  }
  auto* a_star_optimization_info_ptr =
      open_space_debug_.mutable_astar_optimization_info();
  auto* warm_start_trajectory =
      a_star_optimization_info_ptr->mutable_warm_start_trajectory();
  warm_start_trajectory->Clear();
  for (size_t i = 0; i < horizon; ++i) {
    auto* warm_start_point = warm_start_trajectory->add_vehicle_motion_point();
    warm_start_point->mutable_trajectory_point()->mutable_path_point()->set_x(
        result.x[i]);
    warm_start_point->mutable_trajectory_point()->mutable_path_point()->set_y(
        result.y[i]);
    warm_start_point->mutable_trajectory_point()
        ->mutable_path_point()
        ->set_theta(result.phi[i]);
    // delete horizon use result.v.size() after must open
    // if (horizon != 1) {
    //   warm_start_point->mutable_trajectory_point()->set_v(result.v[i]);
    // }
    warm_start_point->mutable_trajectory_point()->set_v(result.v[i]);

    if (i != horizon - 1) {
      warm_start_point->set_steer(result.steer[i]);
      warm_start_point->mutable_trajectory_point()->set_a(result.a[i]);
    }
  }
}
}  // namespace planning
}  // namespace century
