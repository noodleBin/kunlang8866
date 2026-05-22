/******************************************************************************
 * Copyright 2023 The Century Authors. All Rights Reserved.
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

#include "modules/planning/open_space/teb/optimal_planner.h"

#include <algorithm>
#include <limits>
#include <map>
#include <memory>

#include <boost/thread/once.hpp>
#include <boost/thread/thread.hpp>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"

namespace century {
namespace planning {

// ============== Implementation ===================

TebOptimalPlanner::TebOptimalPlanner()
    : cfg_(NULL),
      obstacles_(NULL),
      via_points_(NULL),
      cost_(HUGE_VAL),
      prefer_rotdir_(RotType::none),
      robot_model_(new PointRobotFootprint()),
      initialized_(false),
      optimized_(false) {}

TebOptimalPlanner::TebOptimalPlanner(const TebConfig& cfg,
                                     ObstContainer* obstacles,
                                     RobotFootprintModelPtr robot_model,
                                     TebVisualizationPtr visual,
                                     const ViaPointContainer* via_points) {
  Initialize(cfg, obstacles, robot_model, visual, via_points);
}

TebOptimalPlanner::~TebOptimalPlanner() { ClearGraph(); }

void TebOptimalPlanner::Initialize(const TebConfig& cfg,
                                   ObstContainer* obstacles,
                                   RobotFootprintModelPtr robot_model,
                                   TebVisualizationPtr visual,
                                   const ViaPointContainer* via_points) {
  // init optimizer (set solver and block ordering settings)
  optimizer_ = InitOptimizer();

  cfg_ = &cfg;
  obstacles_ = obstacles;
  robot_model_ = robot_model;
  via_points_ = via_points;
  cost_ = HUGE_VAL;
  prefer_rotdir_ = RotType::none;
  SetVisualization(visual);

  vel_start_.first = true;
  vel_start_.second.linear.x() = 0;
  vel_start_.second.linear.y() = 0;
  vel_start_.second.angular.z() = 0;

  vel_goal_.first = true;
  vel_goal_.second.linear.x() = 0;
  vel_goal_.second.linear.y() = 0;
  vel_goal_.second.angular.z() = 0;
  initialized_ = true;
}

void TebOptimalPlanner::SetVisualization(TebVisualizationPtr visualization) {
  visualization_ = visualization;
}

void TebOptimalPlanner::Visualize() {
  if (!visualization_) {
    return;
  }

  visualization_->PublishLocalPlanAndPoses(teb_);

  if (cfg_->trajectory.publish_feedback) {
    visualization_->PublishFeedbackMessage(*this, *obstacles_);
  }
}

/*
 * registers custom vertices and edges in g2o framework
 */
void TebOptimalPlanner::RegisterG2OTypes() {
  g2o::Factory* factory = g2o::Factory::instance();
  factory->registerType(
      "VERTEX_POSE",
      std::make_shared<g2o::HyperGraphElementCreator<VertexPose>>());
  factory->registerType(
      "VERTEX_TIMEDIFF",
      std::make_shared<g2o::HyperGraphElementCreator<VertexTimeDiff>>());

  factory->registerType(
      "EDGE_TIME_OPTIMAL",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeTimeOptimal>>());
  factory->registerType(
      "EDGE_SHORTEST_PATH",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeShortestPath>>());
  factory->registerType(
      "EDGE_VELOCITY",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocity>>());
  factory->registerType(
      "EDGE_VELOCITY_HOLONOMIC",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>>());
  factory->registerType(
      "EDGE_ACCELERATION",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeAcceleration>>());
  factory->registerType(
      "EDGE_ACCELERATION_START",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationStart>>());
  factory->registerType(
      "EDGE_ACCELERATION_GOAL",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationGoal>>());
  factory->registerType(
      "EDGE_ACCELERATION_HOLONOMIC",
      std::make_shared<
          g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>>());
  factory->registerType(
      "EDGE_ACCELERATION_HOLONOMIC_START",
      std::make_shared<
          g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>>());
  factory->registerType(
      "EDGE_ACCELERATION_HOLONOMIC_GOAL",
      std::make_shared<
          g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>>());
  factory->registerType(
      "EDGE_KINEMATICS_DIFF_DRIVE",
      std::make_shared<
          g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>>());
  factory->registerType(
      "EDGE_KINEMATICS_CARLIKE",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>>());

  factory->registerType(
      "EDGE_JERK", std::make_shared<g2o::HyperGraphElementCreator<EdgeJerk>>());

  factory->registerType(
      "EDGE_JERKSTART",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeJerkStart>>());

  factory->registerType(
      "EDGE_JERKGOAL",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeJerkGoal>>());

  factory->registerType(
      "EDGE_OBSTACLE",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeObstacle>>());
  factory->registerType(
      "EDGE_INFLATED_OBSTACLE",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeInflatedObstacle>>());
  factory->registerType(
      "EDGE_DYNAMIC_OBSTACLE",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeDynamicObstacle>>());
  factory->registerType(
      "EDGE_VIA_POINT",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeViaPoint>>());
  factory->registerType(
      "EDGE_PREFER_ROTDIR",
      std::make_shared<g2o::HyperGraphElementCreator<EdgePreferRotDir>>());
}

/*
 * initialize g2o optimizer. Set solver settings here.
 * Return: pointer to new SparseOptimizer Object.
 */
boost::shared_ptr<g2o::SparseOptimizer> TebOptimalPlanner::InitOptimizer() {
  // Call register_g2o_types once, even for multiple TebOptimalPlanner instances
  // (thread-safe)
  static boost::once_flag flag = BOOST_ONCE_INIT;
  boost::call_once(&RegisterG2OTypes, flag);

  // allocating the optimizer
  boost::shared_ptr<g2o::SparseOptimizer> optimizer =
      boost::make_shared<g2o::SparseOptimizer>();
  std::unique_ptr<TEBLinearSolver> linear_solver(
      new TEBLinearSolver());  // see typedef in optimization.h
  linear_solver->setBlockOrdering(true);
  std::unique_ptr<TEBBlockSolver> block_solver(
      new TEBBlockSolver(std::move(linear_solver)));
  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

  optimizer->setAlgorithm(solver);

  optimizer->initMultiThreading();  // required for >Eigen 3.1

  return optimizer;
}

bool TebOptimalPlanner::OptimizeTEB(int iterations_innerloop,
                                    int iterations_outerloop,
                                    bool compute_cost_afterwards,
                                    double obst_cost_scale,
                                    double viapoint_cost_scale,
                                    bool alternative_time_cost) {
  if (cfg_->optim.optimization_activate == false) return false;

  bool success = false;
  optimized_ = false;

  double weight_multiplier = 1.0;
  AERROR << "teb_.SizePoses()" << teb_.SizePoses();
  // TODO(roesmann): we introduced the non-fast mode with the support of dynamic
  // obstacles
  //                (which leads to better results in terms of x-y-t homotopy
  //                planning).
  //                 however, we have not tested this mode intensively yet, so
  //                 we keep the legacy fast mode as default until we finish our
  //                 tests.
  bool fast_mode = !cfg_->obstacles.include_dynamic_obstacles;

  for (int i = 0; i < iterations_outerloop; ++i) {
    if (cfg_->trajectory.teb_autosize) {
      teb_.AutoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis,
                      cfg_->trajectory.min_samples,
                      cfg_->trajectory.max_samples, fast_mode);
    }
    AINFO << "teb_.SizePoses()" << teb_.SizePoses();
    success = BuildGraph(weight_multiplier);
    if (!success) {
      ClearGraph();
      return false;
    }
    success = OptimizeGraph(iterations_innerloop, false);
    if (!success) {
      ClearGraph();
      return false;
    }
    optimized_ = true;

    if (compute_cost_afterwards &&
        i == iterations_outerloop -
                 1)  // compute cost vec only in the last iteration
      ComputeCurrentCost(obst_cost_scale, viapoint_cost_scale,
                         alternative_time_cost);

    ClearGraph();

    weight_multiplier *= cfg_->optim.weight_adapt_factor;
  }

  return true;
}

void TebOptimalPlanner::SetVelocityStart(const Twist& vel_start) {
  vel_start_.first = true;
  vel_start_.second.linear.x() = vel_start.linear.x();
  vel_start_.second.linear.y() = vel_start.linear.y();
  vel_start_.second.angular.z() = vel_start.angular.z();
}

void TebOptimalPlanner::SetVelocityGoal(const Twist& vel_goal) {
  vel_goal_.first = true;
  vel_goal_.second = vel_goal;
}

bool TebOptimalPlanner::Plan(const std::vector<PoseStamped>& initial_plan,
                             const Twist* start_vel, bool free_goal_vel) {
  if (!teb_.IsInit()) {
    teb_.InitTrajectoryToGoal(
        initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta,
        cfg_->trajectory.global_plan_overwrite_orientation,
        cfg_->trajectory.min_samples,
        cfg_->trajectory.allow_init_with_backwards_motion);
  } else {
    // warm start
    // PoseSE2 start_(initial_plan.front().pose);
    // PoseSE2 goal_(initial_plan.back().pose);
    // --------------------------------------------
    PoseSE2 start_(initial_plan.front().pose.position.x,
                   initial_plan.front().pose.position.y,
                   initial_plan.front().pose.orientation.w);
    PoseSE2 goal_(initial_plan.back().pose.position.x,
                  initial_plan.back().pose.position.y,
                  initial_plan.back().pose.orientation.w);
    // --------------------------------------------
    // actual warm start!
    if (teb_.SizePoses() > 0 &&
        (goal_.position() - teb_.BackPose().position()).norm() <
            cfg_->trajectory.force_reinit_new_goal_dist &&
        fabs(g2o::normalize_theta(goal_.theta() - teb_.BackPose().theta())) <
            cfg_->trajectory.force_reinit_new_goal_angular) {
      // update TEB
      teb_.UpdateAndPruneTEB(start_, goal_, cfg_->trajectory.min_samples);
    } else {
      // goal too far away -> reinit
      AERROR << "New goal: distance to existing goal is higher than the "
                "specified threshold. Reinitalizing trajectories.";
      teb_.ClearTimedElasticBand();
      teb_.InitTrajectoryToGoal(
          initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta,
          cfg_->trajectory.global_plan_overwrite_orientation,
          cfg_->trajectory.min_samples,
          cfg_->trajectory.allow_init_with_backwards_motion);
    }
  }

  if (start_vel) {
    SetVelocityStart(*start_vel);
  }

  if (free_goal_vel) {
    SetVelocityGoalFree();
  } else {
    // we just reactivate and use the previously set velocity (should
    // be zero if nothing was modified)
    vel_goal_.first = true;
  }

  // now optimize
  return OptimizeTEB(cfg_->optim.no_inner_iterations,
                     cfg_->optim.no_outer_iterations);
}

bool TebOptimalPlanner::Plan(const Pose& start, const Pose& goal,
                             const Twist* start_vel, bool free_goal_vel) {
  PoseSE2 start_(start);
  PoseSE2 goal_(goal);
  return Plan(start_, goal_, start_vel);
}

bool TebOptimalPlanner::Plan(const PoseSE2& start, const PoseSE2& goal,
                             const Twist* start_vel, bool free_goal_vel) {
  if (!teb_.IsInit()) {
    // init trajectory
    // 0 intermediate samples, but
    // dt=1 -> AutoResize will add
    // more samples before calling
    // first optimization
    teb_.InitTrajectoryToGoal(
        start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples,
        cfg_->trajectory.allow_init_with_backwards_motion);
  } else {
    // warm start
    AINFO << "teb warm start";
    if (teb_.SizePoses() > 0 &&
        (goal.position() - teb_.BackPose().position()).norm() <
            cfg_->trajectory.force_reinit_new_goal_dist &&
        fabs(g2o::normalize_theta(goal.theta() - teb_.BackPose().theta())) <
            cfg_->trajectory.force_reinit_new_goal_angular) {
      // actual warm start!
      teb_.UpdateAndPruneTEB(start, goal, cfg_->trajectory.min_samples);
    } else {
      // goal too far away -> reinit
      AERROR << "New goal: distance to existing goal is higher than the "
                "specified threshold. Reinitalizing trajectories.";
      teb_.ClearTimedElasticBand();
      teb_.InitTrajectoryToGoal(
          start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples,
          cfg_->trajectory.allow_init_with_backwards_motion);
    }
  }

  if (start_vel) {
    SetVelocityStart(*start_vel);
  }

  if (free_goal_vel) {
    SetVelocityGoalFree();
  } else {
    // we just reactivate and use the previously set velocity (should
    // be zero if nothing was modified)
    vel_goal_.first = true;
  }

  // now optimize
  return OptimizeTEB(cfg_->optim.no_inner_iterations,
                     cfg_->optim.no_outer_iterations);
}

bool TebOptimalPlanner::BuildGraph(double weight_multiplier) {
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty()) {
    AERROR << "Cannot build graph, because it is not empty. Call graphClear()!";
    return false;
  }
  optimizer_->setComputeBatchStatistics(
      cfg_->recovery.divergence_detection_enable);

  // add TEB vertices
  AddTEBVertices();

  // // add Edges// (local cost functions)
  if (cfg_->obstacles.legacy_obstacle_association) {
    // add revelant obstacles (local cost functions)
    AddEdgesObstaclesLegacy(weight_multiplier);
  } else {
    AddEdgesObstaclesRos(weight_multiplier);
  }

  if (cfg_->obstacles.include_dynamic_obstacles) {
    AddEdgesDynamicObstacles();
  }

  AddEdgesViaPoints();

  AddEdgesVelocity();

  AddEdgesAcceleration();

  AddEdgesTimeOptimal();

  AddEdgesShortestPath();

  // lwt: need test, may not much use
  // AddEdgesSmooth();

  AddEdgesJerk();

  if (cfg_->robot.min_turning_radius == 0 ||
      cfg_->optim.weight_kinematics_turning_radius == 0) {
    AddEdgesKinematicsDiffDrive();  // we have a differential drive robot
  } else {
    AddEdgesKinematicsCarlike();  // we have a carlike robot since the turning
                                  // radius is bounded from below.
  }

  AddEdgesPreferRotDir();

  if (cfg_->optim.weight_velocity_obstacle_ratio > 0) {
    AddEdgesVelocityObstacleRatio();
  }

  return true;
}

bool TebOptimalPlanner::OptimizeGraph(int no_iterations, bool clear_after) {
  if (cfg_->robot.max_vel_x < 0.01) {
    AERROR << "OptimizeGraph(): Robot Max Velocity is smaller than 0.01m/s. "
              "Optimizing aborted...";
    if (clear_after) ClearGraph();
    return false;
  }

  if (!teb_.IsInit() || teb_.SizePoses() < cfg_->trajectory.min_samples) {
    AERROR << "OptimizeGraph(): TEB is empty or has too less elements. "
              "Skipping optimization.";
    if (clear_after) ClearGraph();
    return false;
  }

  optimizer_->setVerbose(cfg_->optim.optimization_verbose);
  optimizer_->initializeOptimization();

  int iter = optimizer_->optimize(no_iterations);

  // Save Hessian for visualization
  //  g2o::OptimizationAlgorithmLevenberg* lm =
  //  dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (optimizer_->solver());
  //  lm->solver()->saveHessian("~/MasterThesis/Matlab/Hessian.txt");

  if (!iter) {
    AERROR << "OptimizeGraph(): Optimization failed! iter = " << iter;
    return false;
  }

  if (clear_after) ClearGraph();

  return true;
}

void TebOptimalPlanner::ClearGraph() {
  // clear optimizer states
  if (optimizer_) {
    // we will delete all edges but keep the vertices.
    // before doing so, we will delete the link from the vertices to the edges.
    auto& vertices = optimizer_->vertices();
    for (auto& v : vertices) {
      v.second->edges().clear();
    }

    // necessary, because optimizer->clear deletes
    // pointer-targets (therefore it deletes TEB states!)
    optimizer_->vertices().clear();

    optimizer_->clear();
  }
}

void TebOptimalPlanner::AddTEBVertices() {
  // add vertices to graph
  AERROR_IF(cfg_->optim.optimization_verbose) << "Adding TEB vertices... ";
  unsigned int id_counter = 0;  // used for vertices ids
  obstacles_per_vertex_.resize(teb_.SizePoses());
  auto iter_obstacle = obstacles_per_vertex_.begin();
  for (int i = 0; i < teb_.SizePoses(); ++i) {
    teb_.PoseVertex(i)->setId(id_counter++);
    optimizer_->addVertex(teb_.PoseVertex(i));
    if (teb_.SizeTimeDiffs() != 0 && i < teb_.SizeTimeDiffs()) {
      teb_.TimeDiffVertex(i)->setId(id_counter++);
      optimizer_->addVertex(teb_.TimeDiffVertex(i));
    }
    iter_obstacle->clear();
    (iter_obstacle++)->reserve(obstacles_->size());
  }
}

void TebOptimalPlanner::AddEdgesObstaclesRos(double weight_multiplier) {
  // if weight equals zero skip adding edges!
  if (cfg_->optim.weight_obstacle == 0 || weight_multiplier == 0 ||
      obstacles_ == nullptr) {
    return;
  }
  bool inflated =
      cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;
  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);

  Eigen::Matrix<double, 2, 2> information_inflated;
  information_inflated << cfg_->optim.weight_obstacle * weight_multiplier, 0, 0,
      cfg_->optim.weight_inflation;

  auto iter_obstacle = obstacles_per_vertex_.begin();

  // iterate all teb points, skipping the last and, if the
  // EdgeVelocityObstacleRatio edges should not be created, the first one too

  for (int i = cfg_->optim.weight_velocity_obstacle_ratio == 0 ? 1 : 0;
       i < teb_.SizePoses() - 1; ++i) {
    double left_min_dist = std::numeric_limits<double>::max(),
           right_min_dist = std::numeric_limits<double>::max();

    ObstaclePtr left_obstacle, right_obstacle;

    const Eigen::Vector2d pose_orient = teb_.Pose(i).OrientationUnitVec();

    // iterate obstacles
    for (const ObstaclePtr& obst : *obstacles_) {
      // we handle dynamic obstacles differently below
      if (cfg_->obstacles.include_dynamic_obstacles && obst->IsDynamic())
        continue;

      // calculate distance to robot model
      double dist = robot_model_->CalculateDistance(teb_.Pose(i), obst.get());

      // force considering obstacle if really close to the current pose
      if (dist <
          cfg_->obstacles.min_obstacle_dist *
              cfg_->obstacles.obstacle_association_force_inclusion_factor) {
        iter_obstacle->emplace_back(obst);
        continue;
      }

      // cut-off distance
      if (dist > cfg_->obstacles.min_obstacle_dist *
                     cfg_->obstacles.obstacle_association_cutoff_factor) {
        continue;
      }

      // determine side (left or right) and assign obstacle if closer than the
      // previous one
      // left
      if (Cross2d(pose_orient, obst->GetCentroid()) > 0) {
        if (dist < left_min_dist) {
          left_min_dist = dist;
          left_obstacle = obst;
        }
      } else {
        if (dist < right_min_dist) {
          right_min_dist = dist;
          right_obstacle = obst;
        }
      }
    }

    if (left_obstacle) iter_obstacle->emplace_back(left_obstacle);
    if (right_obstacle) iter_obstacle->emplace_back(right_obstacle);

    // continue here to ignore obstacles for the first pose, but use them later
    // to create the EdgeVelocityObstacleRatio edges
    if (i == 0) {
      ++iter_obstacle;
      continue;
    }

    // create obstacle edges
    for (const ObstaclePtr obst : *iter_obstacle) {
      if (inflated) {
        EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
        dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        dist_bandpt_obst->setInformation(information_inflated);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst.get());
        optimizer_->addEdge(dist_bandpt_obst);
      } else {
        EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst.get());
        optimizer_->addEdge(dist_bandpt_obst);
      }
    }
    ++iter_obstacle;
  }
}

void TebOptimalPlanner::AddEdgesObstaclesLegacy(double weight_multiplier) {
  if (cfg_->optim.weight_obstacle == 0 || weight_multiplier == 0 ||
      obstacles_ == nullptr)
    return;  // if weight equals zero skip adding edges!

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);

  Eigen::Matrix<double, 2, 2> information_inflated;
  information_inflated << cfg_->optim.weight_obstacle * weight_multiplier, 0, 0,
      cfg_->optim.weight_inflation;

  bool inflated =
      cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;

  for (ObstContainer::const_iterator obst = obstacles_->begin();
       obst != obstacles_->end(); ++obst) {
    if (cfg_->obstacles.include_dynamic_obstacles &&
        (*obst)->IsDynamic())  // we handle dynamic obstacles differently below
      continue;

    int index = cfg_->obstacles.obstacle_poses_affected >= teb_.SizePoses()
                    ? teb_.SizePoses() / 2
                    : teb_.FindClosestTrajectoryPose(*(obst->get()));

    // check if obstacle is outside index-range between start and goal
    if ((index <= 1) ||
        (index > teb_.SizePoses() -
                     2))  // start and goal are fixed and findNearestBandpoint
                          // finds first or last conf if intersection point is
                          // outside the range
      continue;

    if (inflated) {
      EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
      dist_bandpt_obst->setVertex(0, teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information_inflated);
      dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst->get());
      optimizer_->addEdge(dist_bandpt_obst);
    } else {
      EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
      dist_bandpt_obst->setVertex(0, teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information);
      dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst->get());
      optimizer_->addEdge(dist_bandpt_obst);
    }

    for (int neighbourIdx = 0;
         neighbourIdx < floor(cfg_->obstacles.obstacle_poses_affected / 2);
         neighbourIdx++) {
      if (index + neighbourIdx < teb_.SizePoses()) {
        if (inflated) {
          EdgeInflatedObstacle* dist_bandpt_obst_n_r = new EdgeInflatedObstacle;
          dist_bandpt_obst_n_r->setVertex(
              0, teb_.PoseVertex(index + neighbourIdx));
          dist_bandpt_obst_n_r->setInformation(information_inflated);
          dist_bandpt_obst_n_r->setParameters(*cfg_, robot_model_.get(),
                                              obst->get());
          optimizer_->addEdge(dist_bandpt_obst_n_r);
        } else {
          EdgeObstacle* dist_bandpt_obst_n_r = new EdgeObstacle;
          dist_bandpt_obst_n_r->setVertex(
              0, teb_.PoseVertex(index + neighbourIdx));
          dist_bandpt_obst_n_r->setInformation(information);
          dist_bandpt_obst_n_r->setParameters(*cfg_, robot_model_.get(),
                                              obst->get());
          optimizer_->addEdge(dist_bandpt_obst_n_r);
        }
      }

      // needs to be casted to int to allow negative values
      if (index - neighbourIdx >= 0) {
        if (inflated) {
          EdgeInflatedObstacle* dist_bandpt_obst_n_l = new EdgeInflatedObstacle;
          dist_bandpt_obst_n_l->setVertex(
              0, teb_.PoseVertex(index - neighbourIdx));
          dist_bandpt_obst_n_l->setInformation(information_inflated);
          dist_bandpt_obst_n_l->setParameters(*cfg_, robot_model_.get(),
                                              obst->get());
          optimizer_->addEdge(dist_bandpt_obst_n_l);
        } else {
          EdgeObstacle* dist_bandpt_obst_n_l = new EdgeObstacle;
          dist_bandpt_obst_n_l->setVertex(
              0, teb_.PoseVertex(index - neighbourIdx));
          dist_bandpt_obst_n_l->setInformation(information);
          dist_bandpt_obst_n_l->setParameters(*cfg_, robot_model_.get(),
                                              obst->get());
          optimizer_->addEdge(dist_bandpt_obst_n_l);
        }
      }
    }
  }
}

void TebOptimalPlanner::AddEdgesDynamicObstacles(double weight_multiplier) {
  if (cfg_->optim.weight_obstacle == 0 || weight_multiplier == 0 ||
      obstacles_ == NULL)
    return;  // if weight equals zero skip adding edges!

  Eigen::Matrix<double, 2, 2> information;
  information(0, 0) = cfg_->optim.weight_dynamic_obstacle * weight_multiplier;
  information(1, 1) = cfg_->optim.weight_dynamic_obstacle_inflation;
  information(0, 1) = information(1, 0) = 0;

  for (ObstContainer::const_iterator obst = obstacles_->begin();
       obst != obstacles_->end(); ++obst) {
    if (!(*obst)->IsDynamic()) continue;

    // Skip first and last pose, as they are fixed
    double time = teb_.TimeDiff(0);
    for (int i = 1; i < teb_.SizePoses() - 1; ++i) {
      EdgeDynamicObstacle* dynobst_edge = new EdgeDynamicObstacle(time);
      dynobst_edge->setVertex(0, teb_.PoseVertex(i));
      dynobst_edge->setInformation(information);
      dynobst_edge->SetParameters(*cfg_, robot_model_.get(), obst->get());
      optimizer_->addEdge(dynobst_edge);
      time +=
          teb_.TimeDiff(i);  // we do not need to check the time diff bounds,
                             // since we iterate to "< SizePoses()-1".
    }
  }
}

void TebOptimalPlanner::AddEdgesViaPoints() {
  if (cfg_->optim.weight_viapoint == 0 || via_points_ == NULL ||
      via_points_->empty())
    return;  // if weight equals zero skip adding edges!

  int start_pose_idx = 0;

  AINFO << "via_points_ size " << via_points_->size();
  int n = teb_.SizePoses();
  if (n < 3)  // we do not have any degrees of freedom for reaching via-points
    return;

  for (ViaPointContainer::const_iterator vp_it = via_points_->begin();
       vp_it != via_points_->end(); ++vp_it) {
    int index = teb_.FindClosestTrajectoryPose(*vp_it, NULL, start_pose_idx);
    if (cfg_->trajectory.via_points_ordered)
      start_pose_idx =
          index +
          2;  // skip a point to have a DOF inbetween for further via-points

    // check if point conicides with goal or is located behind it
    if (index > n - 2)
      index =
          n - 2;  // set to a pose before the goal, since we can move it away!
    // check if point coincides with start or is located before it
    if (index < 1) {
      if (cfg_->trajectory.via_points_ordered) {
        index = 1;  // try to connect the via point with the second (and
                    // non-fixed) pose. It is likely that autoresize adds new
                    // poses inbetween later.
      } else {
        // ADEBUG("TebOptimalPlanner::AddEdgesViaPoints(): skipping a
        // via-point that is close or behind the current robot pose.");
        continue;  // skip via points really close or behind the current robot
                   // pose
      }
    }
    Eigen::Matrix<double, 1, 1> information;
    information.fill(cfg_->optim.weight_viapoint);

    EdgeViaPoint* edge_viapoint = new EdgeViaPoint;
    edge_viapoint->setVertex(0, teb_.PoseVertex(index));
    edge_viapoint->setInformation(information);
    edge_viapoint->SetParameters(*cfg_, &(*vp_it));
    optimizer_->addEdge(edge_viapoint);
  }
}

void TebOptimalPlanner::AddEdgesVelocity() {
  // non-holonomic robot
  if (0 == cfg_->robot.max_vel_y) {
    // if weight equals zero skip adding edges!
    if (0 == cfg_->optim.weight_max_vel_x &&
        0 == cfg_->optim.weight_max_vel_theta) {
      return;
    }

    int n = teb_.SizePoses();
    Eigen::Matrix<double, 2, 2> information;
    information(0, 0) = cfg_->optim.weight_max_vel_x;
    information(1, 1) = cfg_->optim.weight_max_vel_theta;
    information(0, 1) = 0.0;
    information(1, 0) = 0.0;

    for (int i = 0; i < n - 1; ++i) {
      EdgeVelocity* velocity_edge = new EdgeVelocity;
      velocity_edge->setVertex(0, teb_.PoseVertex(i));
      velocity_edge->setVertex(1, teb_.PoseVertex(i + 1));
      velocity_edge->setVertex(2, teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->SetTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    }
  } else {
    // holonomic-robot
    // if weight equals zero skip adding edges!
    if (cfg_->optim.weight_max_vel_x == 0 &&
        cfg_->optim.weight_max_vel_y == 0 &&
        cfg_->optim.weight_max_vel_theta == 0) {
      return;
    }

    int n = teb_.SizePoses();
    Eigen::Matrix<double, 3, 3> information;
    information.fill(0);
    information(0, 0) = cfg_->optim.weight_max_vel_x;
    information(1, 1) = cfg_->optim.weight_max_vel_y;
    information(2, 2) = cfg_->optim.weight_max_vel_theta;

    for (int i = 0; i < n - 1; ++i) {
      EdgeVelocityHolonomic* velocity_edge = new EdgeVelocityHolonomic;
      velocity_edge->setVertex(0, teb_.PoseVertex(i));
      velocity_edge->setVertex(1, teb_.PoseVertex(i + 1));
      velocity_edge->setVertex(2, teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->SetTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    }
  }
}

void TebOptimalPlanner::AddEdgesAcceleration() {
  // if weight equals zero skip adding edges!
  if (cfg_->optim.weight_acc_lim_x == 0 &&
      cfg_->optim.weight_acc_lim_theta == 0)
    return;

  int n = teb_.SizePoses();

  // non-holonomic robot
  if (0 == cfg_->robot.max_vel_y || 0 == cfg_->robot.acc_lim_y) {
    Eigen::Matrix<double, 2, 2> information;
    information.fill(0);
    information(0, 0) = cfg_->optim.weight_acc_lim_x;
    information(1, 1) = cfg_->optim.weight_acc_lim_theta;

    // check if an initial velocity should be taken into accound
    if (vel_start_.first) {
      EdgeAccelerationStart* acceleration_edge = new EdgeAccelerationStart;
      acceleration_edge->setVertex(0, teb_.PoseVertex(0));
      acceleration_edge->setVertex(1, teb_.PoseVertex(1));
      acceleration_edge->setVertex(2, teb_.TimeDiffVertex(0));
      acceleration_edge->SetInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->SetTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i = 0; i < n - 2; ++i) {
      EdgeAcceleration* acceleration_edge = new EdgeAcceleration;
      acceleration_edge->setVertex(0, teb_.PoseVertex(i));
      acceleration_edge->setVertex(1, teb_.PoseVertex(i + 1));
      acceleration_edge->setVertex(2, teb_.PoseVertex(i + 2));
      acceleration_edge->setVertex(3, teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4, teb_.TimeDiffVertex(i + 1));
      acceleration_edge->setInformation(information);
      acceleration_edge->SetTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // check if a goal velocity should be taken into accound
    if (vel_goal_.first) {
      EdgeAccelerationGoal* acceleration_edge = new EdgeAccelerationGoal;
      acceleration_edge->setVertex(0, teb_.PoseVertex(n - 2));
      acceleration_edge->setVertex(1, teb_.PoseVertex(n - 1));
      acceleration_edge->setVertex(
          2, teb_.TimeDiffVertex(teb_.SizeTimeDiffs() - 1));
      acceleration_edge->SetInitialVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->SetTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
  } else {
    // holonomic robot
    AddEdgesAccelerationHolonomicRobot();
  }
}

void TebOptimalPlanner::AddEdgesAccelerationHolonomicRobot() {
  // holonomic robot
  int n = teb_.SizePoses();
  Eigen::Matrix<double, 3, 3> information;
  information.fill(0);
  information(0, 0) = cfg_->optim.weight_acc_lim_x;
  information(1, 1) = cfg_->optim.weight_acc_lim_y;
  information(2, 2) = cfg_->optim.weight_acc_lim_theta;

  // check if an initial velocity should be taken into accound
  if (vel_start_.first) {
    EdgeAccelerationHolonomicStart* acceleration_edge =
        new EdgeAccelerationHolonomicStart;
    acceleration_edge->setVertex(0, teb_.PoseVertex(0));
    acceleration_edge->setVertex(1, teb_.PoseVertex(1));
    acceleration_edge->setVertex(2, teb_.TimeDiffVertex(0));
    acceleration_edge->SetInitialVelocity(vel_start_.second);
    acceleration_edge->setInformation(information);
    acceleration_edge->SetTebConfig(*cfg_);
    optimizer_->addEdge(acceleration_edge);
  }

  // now add the usual acceleration edge for each tuple of three teb poses
  for (int i = 0; i < n - 2; ++i) {
    EdgeAccelerationHolonomic* acceleration_edge =
        new EdgeAccelerationHolonomic;
    acceleration_edge->setVertex(0, teb_.PoseVertex(i));
    acceleration_edge->setVertex(1, teb_.PoseVertex(i + 1));
    acceleration_edge->setVertex(2, teb_.PoseVertex(i + 2));
    acceleration_edge->setVertex(3, teb_.TimeDiffVertex(i));
    acceleration_edge->setVertex(4, teb_.TimeDiffVertex(i + 1));
    acceleration_edge->setInformation(information);
    acceleration_edge->SetTebConfig(*cfg_);
    optimizer_->addEdge(acceleration_edge);
  }

  // check if a goal velocity should be taken into accound
  if (vel_goal_.first) {
    EdgeAccelerationHolonomicGoal* acceleration_edge =
        new EdgeAccelerationHolonomicGoal;
    acceleration_edge->setVertex(0, teb_.PoseVertex(n - 2));
    acceleration_edge->setVertex(1, teb_.PoseVertex(n - 1));
    acceleration_edge->setVertex(2,
                                 teb_.TimeDiffVertex(teb_.SizeTimeDiffs() - 1));
    acceleration_edge->SetInitialVelocity(vel_goal_.second);
    acceleration_edge->setInformation(information);
    acceleration_edge->SetTebConfig(*cfg_);
    optimizer_->addEdge(acceleration_edge);
  }
}

void TebOptimalPlanner::AddEdgesTimeOptimal() {
  // if weight equals zero skip adding edges!
  if (cfg_->optim.weight_optimaltime == 0) return;

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_->optim.weight_optimaltime);

  for (int i = 0; i < teb_.SizeTimeDiffs(); ++i) {
    EdgeTimeOptimal* timeoptimal_edge = new EdgeTimeOptimal;
    timeoptimal_edge->setVertex(0, teb_.TimeDiffVertex(i));
    timeoptimal_edge->setInformation(information);
    timeoptimal_edge->SetTebConfig(*cfg_);
    optimizer_->addEdge(timeoptimal_edge);
  }
}

void TebOptimalPlanner::AddEdgesShortestPath() {
  // if weight equals zero skip adding edges!
  if (cfg_->optim.weight_shortest_path == 0) return;

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_->optim.weight_shortest_path);

  for (int i = 0; i < teb_.SizePoses() - 1; ++i) {
    EdgeShortestPath* shortest_path_edge = new EdgeShortestPath;
    shortest_path_edge->setVertex(0, teb_.PoseVertex(i));
    shortest_path_edge->setVertex(1, teb_.PoseVertex(i + 1));
    shortest_path_edge->setInformation(information);
    shortest_path_edge->SetTebConfig(*cfg_);
    optimizer_->addEdge(shortest_path_edge);
  }
}

void TebOptimalPlanner::AddEdgesSmooth() {
  if (cfg_->optim.weight_dynamic_obstacle == 0) return;  // temp use

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_->optim.weight_dynamic_obstacle);
  // AINFO << "222222222";
  for (int i = 1; i + 1 < teb_.SizePoses(); ++i) {
    EdgeSmooth* smooth_edge = new EdgeSmooth;
    smooth_edge->setVertex(0, teb_.PoseVertex(i - 1));
    smooth_edge->setVertex(1, teb_.PoseVertex(i));
    smooth_edge->setVertex(2, teb_.PoseVertex(i + 1));
    smooth_edge->setInformation(information);
    smooth_edge->SetTebConfig(*cfg_);
    optimizer_->addEdge(smooth_edge);
    // AINFO << "iii"<< i;
  }
}

void TebOptimalPlanner::AddEdgesJerk() {
  if (cfg_->optim.weight_jerk_lim_x == 0 &&
      cfg_->optim.weight_jerk_lim_theta == 0)
    return;  // if weight equals zero skip adding edges!
  int n = teb_.SizePoses();
  Eigen::Matrix<double, 2, 2> information;
  information.fill(0);
  information(0, 0) = cfg_->optim.weight_jerk_lim_x;
  information(1, 1) = cfg_->optim.weight_jerk_lim_theta;

  if (vel_start_.first) {
    EdgeJerkStart* jerk_edge = new EdgeJerkStart;
    jerk_edge->setVertex(0, teb_.PoseVertex(0));
    jerk_edge->setVertex(1, teb_.PoseVertex(1));
    jerk_edge->setVertex(2, teb_.PoseVertex(2));

    jerk_edge->setVertex(3, teb_.TimeDiffVertex(0));
    jerk_edge->setVertex(4, teb_.TimeDiffVertex(1));

    jerk_edge->SetInitialVelocity(vel_start_.second);
    jerk_edge->setInformation(information);
    jerk_edge->SetTebConfig(*cfg_);
    optimizer_->addEdge(jerk_edge);
  }

  // now add the usual acceleration edge for each tuple of three teb poses
  for (int i = 0; i < n - 3; ++i) {
    EdgeJerk* jerk_edge = new EdgeJerk;
    jerk_edge->setVertex(0, teb_.PoseVertex(i));
    jerk_edge->setVertex(1, teb_.PoseVertex(i + 1));
    jerk_edge->setVertex(2, teb_.PoseVertex(i + 2));
    jerk_edge->setVertex(3, teb_.PoseVertex(i + 3));

    jerk_edge->setVertex(4, teb_.TimeDiffVertex(i));
    jerk_edge->setVertex(5, teb_.TimeDiffVertex(i + 1));
    jerk_edge->setVertex(6, teb_.TimeDiffVertex(i + 2));

    jerk_edge->setInformation(information);
    jerk_edge->SetTebConfig(*cfg_);
    optimizer_->addEdge(jerk_edge);
  }

  // check if a goal velocity should be taken into accound
  if (vel_goal_.first) {
    EdgeJerkGoal* jerk_edge = new EdgeJerkGoal;
    jerk_edge->setVertex(0, teb_.PoseVertex(n - 3));
    jerk_edge->setVertex(1, teb_.PoseVertex(n - 2));
    jerk_edge->setVertex(2, teb_.PoseVertex(n - 1));
    jerk_edge->setVertex(3, teb_.TimeDiffVertex(teb_.SizeTimeDiffs() - 2));
    jerk_edge->setVertex(3, teb_.TimeDiffVertex(teb_.SizeTimeDiffs() - 1));
    jerk_edge->SetInitialVelocity(vel_goal_.second);
    jerk_edge->setInformation(information);
    jerk_edge->SetTebConfig(*cfg_);
    optimizer_->addEdge(jerk_edge);
  }
}

void TebOptimalPlanner::AddEdgesKinematicsDiffDrive() {
  // if weight equals zero skip adding edges!
  if (cfg_->optim.weight_kinematics_nh == 0 &&
      cfg_->optim.weight_kinematics_forward_drive == 0)
    return;

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double, 2, 2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive;

  // ignore twiced start only
  for (int i = 0; i < teb_.SizePoses() - 1; ++i) {
    EdgeKinematicsDiffDrive* kinematics_edge = new EdgeKinematicsDiffDrive;
    kinematics_edge->setVertex(0, teb_.PoseVertex(i));
    kinematics_edge->setVertex(1, teb_.PoseVertex(i + 1));
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->SetTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }
}

void TebOptimalPlanner::AddEdgesKinematicsCarlike() {
  // if weight equals zero skip adding edges!
  if (0 == cfg_->optim.weight_kinematics_nh &&
      0 == cfg_->optim.weight_kinematics_turning_radius)
    return;

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double, 3, 3> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_turning_radius;
  information_kinematics(2, 2) = cfg_->optim.weight_kinematics_forward_drive;

  // ignore twiced start only
  for (int i = 0; i < teb_.SizePoses() - 1; ++i) {
    EdgeKinematicsCarlike* kinematics_edge = new EdgeKinematicsCarlike;
    kinematics_edge->setVertex(0, teb_.PoseVertex(i));
    kinematics_edge->setVertex(1, teb_.PoseVertex(i + 1));
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->SetTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }
}

void TebOptimalPlanner::AddEdgesPreferRotDir() {
  // TODO(roesmann): Note, these edges can result in odd predictions, in
  // particular
  //                we can observe a substantional mismatch between open- and
  //                closed-loop planning leading to a poor control performance.
  //                At the moment, we keep these functionality for oscillation
  //                recovery: Activating the edge for a short time period might
  //                not be crucial and could move the robot to a new
  //                oscillation-free state. This needs to be analyzed in more
  //                detail!
  if (prefer_rotdir_ == RotType::none || 0 == cfg_->optim.weight_prefer_rotdir)
    return;  // if weight equals zero skip adding edges!

  if (prefer_rotdir_ != RotType::right && prefer_rotdir_ != RotType::left) {
    AERROR << "TebOptimalPlanner::AddEdgesPreferRotDir(): unsupported RotType "
              "selected. Skipping edge creation.";
    return;
  }

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double, 1, 1> information_rotdir;
  information_rotdir.fill(cfg_->optim.weight_prefer_rotdir);

  // currently: apply to first 3 rotations
  for (int i = 0; i < teb_.SizePoses() - 1 && i < 3; ++i) {
    EdgePreferRotDir* rotdir_edge = new EdgePreferRotDir;
    rotdir_edge->setVertex(0, teb_.PoseVertex(i));
    rotdir_edge->setVertex(1, teb_.PoseVertex(i + 1));
    rotdir_edge->setInformation(information_rotdir);

    if (prefer_rotdir_ == RotType::left)
      rotdir_edge->PreferLeft();
    else if (prefer_rotdir_ == RotType::right)
      rotdir_edge->PreferRight();

    optimizer_->addEdge(rotdir_edge);
  }
}

void TebOptimalPlanner::AddEdgesVelocityObstacleRatio() {
  Eigen::Matrix<double, 2, 2> information;
  information(0, 0) = cfg_->optim.weight_velocity_obstacle_ratio;
  information(1, 1) = cfg_->optim.weight_velocity_obstacle_ratio;
  information(0, 1) = information(1, 0) = 0;

  auto iter_obstacle = obstacles_per_vertex_.begin();

  for (int index = 0; index < teb_.SizePoses() - 1; ++index) {
    for (const ObstaclePtr obstacle : (*iter_obstacle++)) {
      EdgeVelocityObstacleRatio* edge = new EdgeVelocityObstacleRatio;
      edge->setVertex(0, teb_.PoseVertex(index));
      edge->setVertex(1, teb_.PoseVertex(index + 1));
      edge->setVertex(2, teb_.TimeDiffVertex(index));
      edge->setInformation(information);
      edge->SetParameters(*cfg_, robot_model_.get(), obstacle.get());
      optimizer_->addEdge(edge);
    }
  }
}

void TebOptimalPlanner::ComputeCurrentCost(double obst_cost_scale,
                                           double viapoint_cost_scale,
                                           bool alternative_time_cost) {
  // check if graph is empty/exist  -> important if function is called between
  // BuildGraph and OptimizeGraph/ClearGraph
  bool graph_exist_flag(false);
  if (optimizer_->edges().empty() && optimizer_->vertices().empty()) {
    // here the graph is build again, for time efficiency make sure to call this
    // function between BuildGraph and Optimize (deleted), but it depends on the
    // application
    BuildGraph();
    optimizer_->initializeOptimization();
  } else {
    graph_exist_flag = true;
  }

  optimizer_->computeInitialGuess();

  cost_ = 0;

  if (alternative_time_cost) {
    cost_ += teb_.GetSumOfAllTimeDiffs();
    // TEST we use SumOfAllTimeDiffs() here, because edge cost depends on number
    // of samples, which is not always the same for similar TEBs, since we are
    // using an AutoResize Function with hysteresis.
  }

  // now we need pointers to all edges -> calculate error for each edge-type
  // since we aren't storing edge pointers, we need to check every edge
  for (std::vector<g2o::OptimizableGraph::Edge*>::const_iterator it =
           optimizer_->activeEdges().begin();
       it != optimizer_->activeEdges().end(); it++) {
    double cur_cost = (*it)->chi2();

    if (dynamic_cast<EdgeObstacle*>(*it) != nullptr ||
        dynamic_cast<EdgeInflatedObstacle*>(*it) != nullptr ||
        dynamic_cast<EdgeDynamicObstacle*>(*it) != nullptr) {
      cur_cost *= obst_cost_scale;
    } else if (dynamic_cast<EdgeViaPoint*>(*it) != nullptr) {
      cur_cost *= viapoint_cost_scale;
    } else if (dynamic_cast<EdgeTimeOptimal*>(*it) != nullptr &&
               alternative_time_cost) {
      continue;  // skip these edges if alternative_time_cost is active
    }
    cost_ += cur_cost;
  }

  // delete temporary created graph
  if (!graph_exist_flag) ClearGraph();
}

void TebOptimalPlanner::ExtractVelocity(const PoseSE2& pose1,
                                        const PoseSE2& pose2, double dt,
                                        float* vx, float* vy,
                                        float* omega) const {
  if (dt == 0) {
    *vx = 0;
    *vy = 0;
    *omega = 0;
    return;
  }

  Eigen::Vector2d deltaS = pose2.position() - pose1.position();

  // nonholonomic robot
  if (0 == cfg_->robot.max_vel_y) {
    Eigen::Vector2d conf1dir(cos(pose1.theta()), sin(pose1.theta()));
    // translational velocity
    double dir = deltaS.dot(conf1dir);
    *vx = static_cast<double>(g2o::sign(dir) * deltaS.norm() / dt);
    *vy = 0;
  } else {
    // holonomic robot
    // transform pose 2 into the current robot frame (pose1)
    // for velocities only the rotation of the direction vector is necessary.
    // (map->pose1-frame: inverse 2d rotation matrix)
    double cos_theta1 = std::cos(pose1.theta());
    double sin_theta1 = std::sin(pose1.theta());
    double p1_dx = cos_theta1 * deltaS.x() + sin_theta1 * deltaS.y();
    double p1_dy = -sin_theta1 * deltaS.x() + cos_theta1 * deltaS.y();
    *vx = p1_dx / dt;
    *vy = p1_dy / dt;
  }

  // rotational velocity
  double orientdiff = g2o::normalize_theta(pose2.theta() - pose1.theta());
  *omega = orientdiff / dt;
}

bool TebOptimalPlanner::GetVelocityCommand(float* vx, float* vy, float* omega,
                                           int look_ahead_poses) const {
  if (teb_.SizePoses() < 2) {
    AERROR << "TebOptimalPlanner::getVelocityCommand(): The trajectory "
              "contains less than 2 poses. Make sure to init and optimize/plan "
              "the trajectory fist.";
    *vx = 0;
    *vy = 0;
    *omega = 0;
    return false;
  }

  look_ahead_poses =
      std::max(1, std::min(look_ahead_poses, teb_.SizePoses() - 1));
  double dt = 0.0;
  for (int counter = 0; counter < look_ahead_poses; ++counter) {
    dt += teb_.TimeDiff(counter);
    if (dt >= cfg_->trajectory.dt_ref * look_ahead_poses) {
      // TODO(all): change to look-ahead time? Refine
      // trajectory?
      look_ahead_poses = counter + 1;
      break;
    }
  }

  if (dt <= 0) {
    AERROR
        << "TebOptimalPlanner::getVelocityCommand() - timediff<=0 is invalid!";
    *vx = 0;
    *vy = 0;
    *omega = 0;
    return false;
  }

  // Get velocity from the first two configurations
  ExtractVelocity(teb_.Pose(0), teb_.Pose(look_ahead_poses), dt, vx, vy, omega);
  return true;
}

void TebOptimalPlanner::GetProtoTrajInfo(std::vector<Twist>* velocity_profile) {
  int n = teb_.SizePoses();
  velocity_profile->resize(n);

  // start velocity
  velocity_profile->front().linear.y() = 0;
  velocity_profile->front().linear.z() = 0;
  // velocity_profile.front().angular.z() = vel_start_.second.angular.z();

  for (int i = 1; i < n; ++i) {
    velocity_profile->at(i).linear.z() = 0;
    velocity_profile->at(i).angular.x() = velocity_profile->at(i).angular.y() =
        0;
    ExtractVelocity(teb_.Pose(i - 1), teb_.Pose(i), teb_.TimeDiff(i - 1),
                    &velocity_profile->at(i).linear.x(),
                    &velocity_profile->at(i).linear.y(),
                    &velocity_profile->at(i).angular.z());

    // t
    velocity_profile->at(i).linear.y() =
        velocity_profile->at(i - 1).linear.y() +
        std::max(teb_.TimeDiff(i - 1), 0.01);

    //  a
    velocity_profile->at(i).linear.z() =
        (velocity_profile->at(i).linear.x() -
         velocity_profile->at(i - 1).linear.x()) /
        (std::max(teb_.TimeDiff(i - 1), 0.01));
  }

  // start velocity
  velocity_profile->front().linear.y() = 0;
  velocity_profile->front().linear.z() = 0;
  velocity_profile->front().linear.x() =
      0.5 * velocity_profile->at(1).linear.x();

  // goal velocity
  velocity_profile->back().linear.x() = vel_goal_.second.linear.x();
  velocity_profile->back().linear.z() = 0;
  // velocity_profile.back().angular.z() = vel_goal_.second.angular.z();
}

void TebOptimalPlanner::GetVelocityProfile(
    std::vector<Twist>* velocity_profile) const {
  int n = teb_.SizePoses();
  velocity_profile->resize(n);

  // start velocity
  velocity_profile->front().linear.x() = vel_start_.second.linear.x();
  velocity_profile->front().linear.y() = vel_start_.second.linear.y();
  velocity_profile->front().linear.z() = 0;
  velocity_profile->front().angular.x() = 0;
  velocity_profile->front().angular.y() = 0;
  velocity_profile->front().angular.z() = vel_start_.second.angular.z();

  // intermediate velocity
  for (int i = 1; i < n - 1; ++i) {
    velocity_profile->at(i).linear.z() = 0;
    velocity_profile->at(i).angular.x() = 0;
    velocity_profile->at(i).angular.y() = 0;
    ExtractVelocity(teb_.Pose(i - 1), teb_.Pose(i), teb_.TimeDiff(i - 1),
                    &velocity_profile->at(i).linear.x(),
                    &velocity_profile->at(i).linear.y(),
                    &velocity_profile->at(i).angular.z());
  }

  // goal velocity
  velocity_profile->back().linear.x() = vel_goal_.second.linear.x();
  velocity_profile->back().linear.y() = vel_goal_.second.linear.y();
  velocity_profile->back().linear.z() = 0;
  velocity_profile->back().angular.x() = 0;
  velocity_profile->back().angular.y() = 0;
  velocity_profile->back().angular.z() = vel_goal_.second.angular.z();
}

void TebOptimalPlanner::GetFullTrajectory(
    std::vector<Eigen::Vector3f>* trajectory) const {
  int n = teb_.SizePoses();
  trajectory->resize(n);

  if (n == 0) {
    return;
  }

  // start
  Eigen::Vector3f& start = trajectory->front();
  start[0] = teb_.Pose(0).x();
  start[1] = teb_.Pose(0).y();
  start[2] = teb_.Pose(0).theta();

  // intermediate points
  for (int i = 1; i < n - 1; ++i) {
    Eigen::Vector3f& point = trajectory->at(i);
    point[0] = teb_.Pose(i).x();
    point[1] = teb_.Pose(i).y();
    point[2] = teb_.Pose(i).theta();
  }

  // goal
  Eigen::Vector3f& goal = trajectory->back();
  goal[0] = teb_.Pose(n - 1).x();
  goal[1] = teb_.Pose(n - 1).y();
  goal[2] = teb_.Pose(n - 1).theta();
}

bool TebOptimalPlanner::IsTrajectoryFeasible(void) {
  int look_ahead_idx = teb_.SizePoses() - 1;

  for (int i = 0; i < look_ahead_idx; ++i) {
    // get car line
    Eigen::Vector2d line_start(teb_.Pose(i).x(), teb_.Pose(i).y());
    Eigen::Vector2d line_end(teb_.Pose(i + 1).x(), teb_.Pose(i + 1).y());
    // cfg_->obstacles.min_obstacle_dist
    // double min_dist = 0.5;
    double min_dist = cfg_->obstacles.min_obstacle_dist;
    // iterate obstacles
    for (const ObstaclePtr& obst : *obstacles_) {
      bool isOverLap =
          obst->CheckLineIntersection(line_start, line_end, min_dist);
      bool isCollision = obst->CheckCollision(line_start, min_dist);
      if (isOverLap || isCollision) {
        AERROR << "---isOverLap---";
        return false;
      }
    }
  }
  return true;
}

}  // namespace planning
}  // namespace century
