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

#include "modules/planning/tasks/deciders/teb_planner_decider/teb_fallback_decider.h"

namespace century {
namespace planning {
using century::common::Status;
using century::common::TrajectoryPoint;
using century::common::math::Box2d;
using century::common::math::Polygon2d;
using century::common::math::Vec2d;
namespace {
constexpr double kDeltaT = 0.2;                 // m
constexpr double kCollisionCheckDeltaT = 0.15;  // s
constexpr double kMaxDistanceOffset = 100.7;    // m , 100 means close
constexpr int kSamePointsNum = 20;              //
constexpr double kPedCheckTime = 0.8;           //
constexpr double kVehicleMaxAcc = 4.0;
constexpr double kVehicleMaxDec = -4.0;
constexpr double kStopBuffer = 1.0;
constexpr double kAverage = 0.5;
constexpr double kDynamicCoffi = 0.4;
constexpr double kMinValidSpeed = 0.25;    // m/s
constexpr double kStopSpeed = 0.05;        // m/s
constexpr size_t kVehBlockCnt = 4;         //
constexpr double kReverseS = 1.6 + 0.001;  //
constexpr double kReachDist = 0.5;         //
constexpr double kMinTTCS_1 = 2.5;         //
constexpr double kMinTTCS_2 = 2.5;         //
constexpr double kPercent = 0.01;          //
constexpr double kMinSpeed = 1.0;          // m/s

constexpr size_t kZero = 0;
constexpr double kFallBackDelayTime = 2.0;
constexpr double kLonBuffer = 0.2;
}  // namespace

TEBFallbackDecider::TEBFallbackDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  ego_length_bare_ = vehicle_config.vehicle_param().length();
  ego_width_bare_ = vehicle_config.vehicle_param().width();
  ego_length_ =
      vehicle_config.vehicle_param().length() + FLAGS_astar_first_long_buffer;
  ego_width_ =
      vehicle_config.vehicle_param().width() + FLAGS_rescue_hybird_lat_buffer;
  shift_distance_ =
      ego_length_ * 0.5 - vehicle_config.vehicle_param().back_edge_to_center();
  distance_center_ = vehicle_config.vehicle_param().back_edge_to_center();
}

bool TEBFallbackDecider::QuardraticFormulaLowerSolution(const double a,
                                                        const double b,
                                                        const double c,
                                                        double* sol) {
  // quardratic formula: ax^2 + bx + c = 0, return lower solution
  // TODO(QiL): use const from common::math
  const double kEpsilon = 1e-6;
  *sol = 0.0;
  if (std::abs(a) < kEpsilon) {
    return false;
  }

  double tmp = b * b - 4 * a * c;
  if (tmp < kEpsilon) {
    return false;
  }
  double sol1 = (-b + std::sqrt(tmp)) / (2.0 * a);
  double sol2 = (-b - std::sqrt(tmp)) / (2.0 * a);

  *sol = std::abs(std::min(sol1, sol2));
  ADEBUG << "QuardraticFormulaLowerSolution finished with sol: " << *sol
         << "sol1: " << sol1 << ", sol2: " << sol2 << "a: " << a << "b: " << b
         << "c: " << c;
  return true;
}
void TEBFallbackDecider::DealBuffer() {
  if (injector_->deal_start_block_) {
    AINFO << "remove buffer";
    ego_length_ = ego_length_bare_;
    ego_width_ = ego_width_bare_;
    shift_distance_ = ego_length_ * 0.5 - distance_center_;
  } else {
    ego_length_ = ego_length_bare_ + FLAGS_astar_first_long_buffer;
    ego_width_ = ego_width_bare_ + FLAGS_rescue_hybird_lat_buffer;
    shift_distance_ = ego_length_ * 0.5 - distance_center_;
  }
}

void TEBFallbackDecider::CalFallBackIndexWithDelayTime(
    TrajGearPair* trajectory_gear_pair, size_t* current_idx) {
  double gear_d = (*trajectory_gear_pair).second == canbus::Chassis::GEAR_DRIVE
                      ? true
                      : false;
  double delay_s = kFallBackDelayTime *
                   std::fabs(injector_->vehicle_state()->linear_velocity());
  double collision_index_s =
      (*trajectory_gear_pair).first[*current_idx].path_point().s();
  double advanced_s =
      gear_d ? collision_index_s - delay_s : collision_index_s + delay_s;
  size_t first_collision_index_temp = *current_idx;
  for (size_t i = first_collision_index_temp; i > 0; --i) {
    if (gear_d &&
        (*trajectory_gear_pair).first[i].path_point().s() <= advanced_s) {
      *current_idx = i;
      break;
    }
    if (!gear_d &&
        (*trajectory_gear_pair).first[i].path_point().s() >= advanced_s) {
      *current_idx = i;
      break;
    }
  }
  return;
}

Status TEBFallbackDecider::Process(Frame* frame) {
  DealBuffer();
  if (FLAGS_enable_ignore_teb_fallback_check && injector_->deal_start_block_) {
    return Status::OK();
  }
  size_t first_collision_index = 0;
  const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                            injector_->vehicle_state()->y()};
  auto trajectory_pb =
      frame->open_space_info().chosen_partitioned_trajectory().first;
  const size_t point_size = trajectory_pb.NumOfPoints();
  size_t fallback_start_index =
      std::max(std::min(trajectory_pb.QueryNearestPoint(adc_position) + 1,
                        point_size - 1),
               kZero);

  // change flag
  frame_->mutable_open_space_info()->set_fallback_flag(true);
  // generate fallback trajectory base on current partition trajectory
  // vehicle speed is decreased to zero inside safety distance
  TrajGearPair fallback_trajectory_pair_candidate =
      frame->open_space_info().chosen_partitioned_trajectory();

  ConvertCostmap();
  // only for uturn
  ProcessBlockScenario(frame_->obstacles(), &fallback_start_index,
                       &fallback_trajectory_pair_candidate);

  bool fallback = false;
  CheckCollisionForFallback(
      frame->open_space_info().chosen_partitioned_trajectory(),
      frame->obstacles(), &fallback_start_index, &first_collision_index,
      &fallback);

  if (TEBTarStatus::STOP == frame->open_space_info().tar_status()) {
    // AINFO << "fallback due to tar status stop";
    first_collision_index = 0;
    fallback = true;
  }

  // wyq test switch
  // first_collision_index = 0;
  // fallback = true;

  CalFallBackIndexWithDelayTime(&fallback_trajectory_pair_candidate,
                                &first_collision_index);

  if (fallback) {
    double gear_dir =
        fallback_trajectory_pair_candidate.second == canbus::Chassis::GEAR_DRIVE
            ? 1.0
            : -1.0;
    const auto future_collision_point =
        fallback_trajectory_pair_candidate.first[first_collision_index];

    // Fallback starts from current location but with vehicle velocity
    auto fallback_start_point =
        fallback_trajectory_pair_candidate.first[fallback_start_index];

    fallback_start_point.set_v(
        gear_dir *
        std::min(kMinSpeed, std::fabs(fallback_trajectory_pair_candidate
                                          .first[fallback_start_index]
                                          .v())));

    *(frame_->mutable_open_space_info()->mutable_future_collision_point()) =
        future_collision_point;

    // min stop distance: (max_acc)
    double min_stop_distance = 0.5 * fallback_start_point.v() *
                               fallback_start_point.v() / kVehicleMaxAcc,
           stop_distance =
               fallback_trajectory_pair_candidate.second ==
                       canbus::Chassis::GEAR_DRIVE
                   ? std::max(future_collision_point.path_point().s() -
                                  fallback_start_point.path_point().s() -
                                  kStopBuffer,
                              0.0)
                   : std::min(future_collision_point.path_point().s() -
                                  fallback_start_point.path_point().s() +
                                  kStopBuffer,
                              0.0);

    // AINFO << "stop distance : " << stop_distance << "fallback_start_index "
    //       << fallback_start_index << "fallback_start_point.v() "
    //       << fallback_start_point.v();
    stop_distance = fallback_trajectory_pair_candidate.second ==
                            canbus::Chassis::GEAR_REVERSE
                        ? std::min(-1 * min_stop_distance, stop_distance)
                        : std::max(min_stop_distance, stop_distance);

    // Search stop index in chosen trajectory by distance
    size_t stop_index = fallback_start_index;

    for (size_t i = fallback_start_index;
         i < fallback_trajectory_pair_candidate.first.NumOfPoints(); ++i) {
      if (std::abs(
              fallback_trajectory_pair_candidate.first[i].path_point().s()) >=
          std::abs(fallback_start_point.path_point().s() + stop_distance)) {
        stop_index = i;
        break;
      }
    }

    for (size_t i = 0; i < fallback_start_index; ++i) {
      fallback_trajectory_pair_candidate.first[i].set_v(
          fallback_start_point.v());  // 0.0
    }

    // If stop_index == fallback_start_index;
    if (fallback_start_index == stop_index ||
        (FLAGS_enable_directly_stop && is_dynamic_collision_)) {
      // 1. Set fallback start speed to 0, acceleration to max acceleration.
      // AINFO << "Stop distance within safety buffer, stop now!";
      fallback_trajectory_pair_candidate.first[fallback_start_index].set_v(
          gear_dir * std::min(std::fabs(fallback_trajectory_pair_candidate
                                            .first[fallback_start_index]
                                            .v()),
                              kMinSpeed));  // 0.0
      // lwt: use origin path
      for (size_t i = fallback_start_index;
           i < fallback_trajectory_pair_candidate.first.NumOfPoints(); ++i) {
        fallback_trajectory_pair_candidate.first[i].set_v(
            gear_dir *
            std::min(std::fabs(fallback_trajectory_pair_candidate.first[i].v()),
                     kMinSpeed));  // 0.0
      }

      *(frame_->mutable_open_space_info()->mutable_fallback_trajectory()) =
          fallback_trajectory_pair_candidate;

      return Status::OK();
    }

    // If stop_index > fallback_start_index
    // lwt : use origin path : the speed between  fallback_start_index and
    // stop_index may not zero
    for (size_t i = fallback_start_index;
         i < fallback_trajectory_pair_candidate.first.NumOfPoints(); ++i) {
      fallback_trajectory_pair_candidate.first[i].set_v(
          gear_dir *
          std::min(std::fabs(fallback_trajectory_pair_candidate.first[i].v()),
                   kMinSpeed));  // 0.0
    }

    *(frame_->mutable_open_space_info()->mutable_fallback_trajectory()) =
        fallback_trajectory_pair_candidate;
  } else {
    frame_->mutable_open_space_info()->set_fallback_flag(false);
  }
  return Status::OK();
}

void TEBFallbackDecider::CheckCollisionForFallback(
    const TrajGearPair& trajectory_gear_pair,
    const std::vector<const Obstacle*>& obstacles, const size_t* current_index,
    size_t* first_collision_index, bool* fallback) {
  if (!FLAGS_enable_collision_check_for_teb_speed) {
    *fallback = !IsCollisionFreeTrajectory(
        trajectory_gear_pair, obstacles, current_index, first_collision_index);
  }
  double dynamic_collsion_check_start_time = Clock::NowInSeconds();
  if (FLAGS_enable_dynamic_collsion_check) {
    is_dynamic_collision_ = IsADCWillCollisionUseDynamic(
        trajectory_gear_pair, obstacles, first_collision_index);
    AINFO << "first_collision_index = " << *first_collision_index;
    if (is_dynamic_collision_) {
      *fallback = true;
      AINFO << "IsADCWillCollisionUseDynamic return true, not collision free";
    }
  }
  double dynamic_collsion_check_end_time = Clock::NowInSeconds();
  AINFO << "dynamic_collsion_check_use_time = "
        << dynamic_collsion_check_end_time - dynamic_collsion_check_start_time;
}

bool TEBFallbackDecider::IsCollisionFreeTrajectory(
    const TrajGearPair& trajectory_gear_pair,
    const std::vector<const Obstacle*>& obstacles, const size_t* current_index,
    size_t* first_collision_index) {
  if (FLAGS_enable_dynamic_collsion_check) {
    is_dynamic_collision_ = IsADCWillCollisionUseDynamic(
        trajectory_gear_pair, obstacles, current_index);
    if (is_dynamic_collision_) {
      *first_collision_index = *current_index;
      AINFO << "IsADCWillCollisionUseDynamic return true, not collision free";
      return false;
    }
  }
  if (IsADCWillCollisionWithDynamicVehicle(trajectory_gear_pair, obstacles,
                                           current_index)) {
    *first_collision_index = *current_index;
    AINFO << "IsADCWillCollisionWithDynamicVehicle return true, not collision "
             "free";
    return false;
  }
  // prediction time resolution: FLAGS_trajectory_time_resolution
  auto trajectory_pb = trajectory_gear_pair.first;
  const size_t point_size = trajectory_pb.NumOfPoints();

  // test
  // canbus::Chassis::GEAR_REVERSE
  // auto gear = trajectory_gear_pair.second;
  // if (canbus::Chassis::GEAR_REVERSE == gear) {
  //   return true;
  // }
  AINFO << "current_index:" << (*current_index) << " relative_time:"
        << trajectory_pb.TrajectoryPointAt(*current_index).relative_time();
  const double adc_speed =
      std::fabs(injector_->vehicle_state()->linear_velocity());
  const double ttc_s2 =
      std::max(kMinTTCS_2, adc_speed * FLAGS_teb_obs_prediction_time);
  for (size_t i = *current_index; i < point_size; ++i) {
    const auto& trajectory_point = trajectory_pb.TrajectoryPointAt(i);
    double rel_s =
        trajectory_point.path_point().s() -
        trajectory_pb.TrajectoryPointAt(*current_index).path_point().s();
    if (rel_s > ttc_s2) {
      return true;
    }

    ADEBUG << "i: [" << i << "]"
           << "trajectory_point.relative_time() "
           << trajectory_point.relative_time();
    double ego_theta = trajectory_point.path_point().theta();
    Box2d ego_box(
        {trajectory_point.path_point().x(), trajectory_point.path_point().y()},
        ego_theta, ego_length_, ego_width_);
    Vec2d shift_vec{shift_distance_ * std::cos(ego_theta),
                    shift_distance_ * std::sin(ego_theta)};
    ego_box.Shift(shift_vec);

    auto gear = trajectory_gear_pair.second;
    if (!(canbus::Chassis::GEAR_DRIVE == gear &&
          FLAGS_enable_collision_check_for_teb_speed)) {
      if (CheckObstacleCollision(rel_s, ego_box, obstacles)) {
        *first_collision_index = i;
        AINFO << "first_collision_index: [" << i << "]";
        return false;
      }
    }
    // --------------------------------------
  }
  return true;
}

void TEBFallbackDecider::ConvertCostmap() {
  if (!FLAGS_enable_use_costmap) {
    return;
  }
  const auto& costmap_obstacles = frame_->static_area_polygon();
  costmap_polygons_.clear();
  common::math::Polygon2d perception_polygon;

  for (const auto& obstacle : costmap_obstacles) {
    std::vector<Vec2d> polygon_points;
    for (auto pt : obstacle.second) {
      polygon_points.emplace_back(Vec2d(pt.x(), pt.y()));
    }
    if (!common::math::Polygon2d::ComputeConvexHull(polygon_points,
                                                    &perception_polygon)) {
      AERROR << "waring : ComputeConvexHull failded, continue";
      continue;
    }
    costmap_polygons_.emplace_back(perception_polygon);
  }
}

// means temp close
bool TEBFallbackDecider::IsADCDeviationTrajectory(
    const TrajGearPair& trajectory_gear_pair) {
  return false;
  auto trajectory_pb = trajectory_gear_pair.first;
  const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                            injector_->vehicle_state()->y()};
  double adc_heading = injector_->vehicle_state()->heading();

  size_t current_index = trajectory_pb.QueryNearestPoint(adc_position);
  const auto& trajectory_point = trajectory_pb.TrajectoryPointAt(current_index);

  double dx =
      trajectory_point.path_point().x() - injector_->vehicle_state()->x();
  double dy =
      trajectory_point.path_point().y() - injector_->vehicle_state()->y();
  double ds = std::hypot(dx, dy);

  if (ds > kMaxDistanceOffset) {
    AINFO << " adc nearest point ds = " << ds << " need stop";
    return true;
  }

  double target_heading = frame_->open_space_info().end_heading();
  double delta_yaw = common::math::NormalizeAngle(target_heading - adc_heading);

  if (std::fabs(delta_yaw) > (M_PI_2 + M_PI_4)) {
    AERROR << " target_heading -adc_heading  = " << delta_yaw << " need stop ";
    // to add scenario judge for uturn
    // return true;
  }
  return false;
}

// function by shift traj
bool TEBFallbackDecider::IsADCWillCollision(
    const TrajGearPair& trajectory_gear_pair,
    const std::vector<const Obstacle*>& obstacles,
    const size_t* current_index) {
  const auto& trajectory_pb = trajectory_gear_pair.first;
  const size_t point_size = trajectory_pb.NumOfPoints();

  const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                            injector_->vehicle_state()->y()};

  const auto& nearest_point = trajectory_pb.TrajectoryPointAt(*current_index);

  const common::math::Vec2d delta_vec = {
      adc_position.x() - nearest_point.path_point().x(),
      adc_position.y() - nearest_point.path_point().y()};
  const double check_s = kMinTTCS_2;
  for (size_t i = *current_index; i < point_size; ++i) {
    const auto& trajectory_point = trajectory_pb.TrajectoryPointAt(i);
    double rel_s = std::fabs(
        trajectory_point.path_point().s() -
        trajectory_pb.TrajectoryPointAt(*current_index).path_point().s());
    if (rel_s > check_s) {
      ADEBUG << "i: [" << i << "]"
             << "rel_s < check_s is free, think is collision free ";
      return false;
    }

    double ego_theta = trajectory_point.path_point().theta();
    Box2d ego_box({trajectory_point.path_point().x() + delta_vec.x(),
                   trajectory_point.path_point().y() + delta_vec.y()},
                  ego_theta, ego_length_, ego_width_);
    Vec2d shift_vec{shift_distance_ * std::cos(ego_theta),
                    shift_distance_ * std::sin(ego_theta)};
    ego_box.Shift(shift_vec);
    const auto& adc_polygon = Polygon2d(ego_box);

    for (const auto& polygon : costmap_polygons_) {
      if (polygon.HasOverlap(adc_polygon)) {
        AINFO << "i: [" << i << "]"
              << "trajectory_point.relative_time() "
              << trajectory_point.relative_time()
              << "HasOverlap  costmap_polygons_ ";
        return true;
      }
    }

    for (const auto* obstacle : obstacles) {
      if (obstacle->IsVirtual()) {
        continue;
      }

      const auto& obs_type = obstacle->Perception().type();
      if (perception::PerceptionObstacle::UNKNOWN == obs_type ||
          perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type ||
          perception::PerceptionObstacle::UNKNOWN_UNMOVABLE == obs_type ||
          FLAGS_enable_openspace_use_polygon_plan) {
        const auto& obstacle_polygon = obstacle->PerceptionPolygon();
        if (obstacle_polygon.HasOverlap(adc_polygon)) {
          AINFO << "i: [" << i << "]"
                << "trajectory_point.relative_time() "
                << trajectory_point.relative_time() << "obstacle id "
                << obstacle->Id() << "obs_type " << obs_type;
          return true;
        }
      } else {
        const auto& box = obstacle->PerceptionBoundingBox();
        if (ego_box.HasOverlap(box)) {
          AINFO << "i: [" << i << "]"
                << "trajectory_point.relative_time() "
                << trajectory_point.relative_time() << "obstacle id "
                << obstacle->Id() << "obs_type " << obs_type;
          return true;
        }
      }
    }
  }
  return false;
}

// lwt: algorithm theory
bool TEBFallbackDecider::IsADCWillCollisionUseDynamic(
    const TrajGearPair& trajectory_gear_pair,
    const std::vector<const Obstacle*>& obstacles,
    const size_t* current_index) {
  const auto& trajectory_pb = trajectory_gear_pair.first;
  const auto& gear = trajectory_gear_pair.second;

  if (canbus::Chassis::GEAR_REVERSE == gear) {
    if (!FLAGS_enable_new_teb_test_func2) {
      return false;
    }
    return IsADCBackCollisionUseDynamic(trajectory_gear_pair, obstacles,
                                        current_index);
  }

  const common::math::Vec2d adc_position{injector_->vehicle_state()->x(),
                                         injector_->vehicle_state()->y()};

  const auto& nearest_point = trajectory_pb.TrajectoryPointAt(*current_index);
  ADEBUG << "nearest_point.v() " << nearest_point.v();

  // use injector_->vehicle_state()->linear_velocity();?
  const double use_v = std::max(
      kMinValidSpeed, std::fabs(injector_->vehicle_state()->linear_velocity()));
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double steer_angle = vehicle_config.vehicle_param().max_steer_angle() *
                       injector_->vehicle_state()->steering_percentage() *
                       kPercent;
  const double theta_0 = injector_->vehicle_state()->heading();
  const double theta_delta = FLAGS_fallback_collsion_preview_time * use_v *
                             std::tan(steer_angle) /
                             (vehicle_config.vehicle_param().wheel_base()*0.5);

  const double aver_theta =
      common::math::NormalizeAngle(theta_0 + kDynamicCoffi * theta_delta);
  const double s = use_v * FLAGS_fallback_collsion_preview_time;

  const common::math::Vec2d estimate_pos{
      adc_position.x() + s * std::cos(aver_theta),
      adc_position.y() + s * std::sin(aver_theta)};

  ADEBUG << "aver_theta" << aver_theta << "std::sin(aver_theta)"
         << std::sin(aver_theta) << "theta_0" << theta_0;
  ADEBUG << "injector_->vehicle_state()->steering_percentage() "
         << injector_->vehicle_state()->steering_percentage() << "use_v "
         << use_v << "theta_delta " << theta_delta;
  ADEBUG << "estimate_pos x" << std::setprecision(9) << estimate_pos.x()
         << std::setprecision(9) << "estimate_pos y " << estimate_pos.y()
         << "injector_->vehicle_state()->x() " << std::setprecision(9)
         << injector_->vehicle_state()->x() << "injector_->vehicle_state()->y()"
         << std::setprecision(9) << injector_->vehicle_state()->y();

  double ego_length = vehicle_config.vehicle_param().length() + kLonBuffer;
  Box2d ego_box({estimate_pos.x(), estimate_pos.y()}, aver_theta, ego_length,
                ego_width_);
  century::common::Point3D dynamic_adc_position;
  dynamic_adc_position.set_x(estimate_pos.x());
  dynamic_adc_position.set_y(estimate_pos.y());
  dynamic_adc_position.set_z(aver_theta);

  frame_->set_dynamic_adc_position(dynamic_adc_position);
  Vec2d shift_vec{shift_distance_ * std::cos(theta_0),
                  shift_distance_ * std::sin(theta_0)};
  ego_box.Shift(shift_vec);
  const auto& adc_polygon = Polygon2d(ego_box);

  // for (const auto& polygon : costmap_polygons_) {
  //   if (polygon.HasOverlap(adc_polygon)) {
  //     AINFO << "IsADCWillCollisionUseDynamic"
  //           << "HasOverlap  costmap_polygons_ ";
  //     return true;
  //   }
  // }

  for (const auto* obstacle : obstacles) {
    if (obstacle->IsVirtual()) {
      continue;
    }

    const auto& obs_type = obstacle->Perception().type();
    if (perception::PerceptionObstacle::UNKNOWN == obs_type ||
        perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type ||
        perception::PerceptionObstacle::UNKNOWN_UNMOVABLE == obs_type ||
        FLAGS_enable_openspace_use_polygon_plan) {
      const auto& obstacle_polygon = obstacle->PerceptionPolygon();
      if (obstacle_polygon.HasOverlap(adc_polygon)) {
        AINFO << "IsADCWillCollisionUseDynamic" << obstacle->Id() << "obs_type "
              << obs_type;
        return true;
      }
    } else {
      const auto& box = obstacle->PerceptionBoundingBox();
      if (ego_box.HasOverlap(box)) {
        AINFO << "IsADCWillCollisionUseDynamic"
              << "obstacle id " << obstacle->Id() << "obs_type " << obs_type;
        return true;
      }
    }
  }

  return false;
}

// lwt: algorithm theory
bool TEBFallbackDecider::IsADCBackCollisionUseDynamic(
    const TrajGearPair& trajectory_gear_pair,
    const std::vector<const Obstacle*>& obstacles,
    const size_t* current_index) {
  const common::math::Vec2d adc_position{injector_->vehicle_state()->x(),
                                         injector_->vehicle_state()->y()};
  const double use_v =
      std::min(-kMinValidSpeed,
               -std::fabs(injector_->vehicle_state()->linear_velocity()));
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double steer_angle = vehicle_config.vehicle_param().max_steer_angle() *
                       injector_->vehicle_state()->steering_percentage() *
                       kPercent;
  const double theta_0 = injector_->vehicle_state()->heading();
  const double theta_delta = FLAGS_fallback_collsion_preview_time * use_v *
                             std::tan(steer_angle) /
                             (vehicle_config.vehicle_param().wheel_base()*0.5);

  const double aver_theta =
      common::math::NormalizeAngle(theta_0 + kDynamicCoffi * theta_delta);
  const double s = use_v * FLAGS_fallback_collsion_preview_time;

  const common::math::Vec2d estimate_pos{
      adc_position.x() + s * std::cos(aver_theta),
      adc_position.y() + s * std::sin(aver_theta)};

  AINFO << "aver_theta" << aver_theta << "std::sin(aver_theta)"
        << std::sin(aver_theta) << "theta_0" << theta_0;
  ADEBUG << "injector_->vehicle_state()->steering_percentage() "
         << injector_->vehicle_state()->steering_percentage() << "use_v "
         << use_v << "theta_delta " << theta_delta;
  AINFO << "estimate_pos x" << std::setprecision(9) << estimate_pos.x()
        << std::setprecision(9) << "estimate_pos y " << estimate_pos.y()
        << "injector_->vehicle_state()->x() " << std::setprecision(9)
        << injector_->vehicle_state()->x() << "injector_->vehicle_state()->y()"
        << std::setprecision(9) << injector_->vehicle_state()->y();

  // directly use ego_width_bare_?
  double ego_length = vehicle_config.vehicle_param().length() + kLonBuffer;
  Box2d ego_box({estimate_pos.x(), estimate_pos.y()}, aver_theta, ego_length,
                ego_width_);
  century::common::Point3D dynamic_adc_position;
  dynamic_adc_position.set_x(estimate_pos.x());
  dynamic_adc_position.set_y(estimate_pos.y());
  dynamic_adc_position.set_z(aver_theta);

  frame_->set_dynamic_adc_position(dynamic_adc_position);
  double shitf_dist =
      kAverage * ego_length - FLAGS_astar_first_long_buffer - distance_center_;
  Vec2d shift_vec{shitf_dist * std::cos(theta_0),
                  shitf_dist * std::sin(theta_0)};
  ego_box.Shift(shift_vec);
  const auto& adc_polygon = Polygon2d(ego_box);

  // for (const auto& polygon : costmap_polygons_) {
  //   if (polygon.HasOverlap(adc_polygon)) {
  //     AINFO << "IsADCBackCollisionUseDynamic"
  //           << "HasOverlap  costmap_polygons_ ";
  //     return true;
  //   }
  // }

  for (const auto* obstacle : obstacles) {
    if (obstacle->IsVirtual()) {
      continue;
    }

    const auto& obs_type = obstacle->Perception().type();
    if (perception::PerceptionObstacle::UNKNOWN == obs_type ||
        perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type ||
        perception::PerceptionObstacle::UNKNOWN_UNMOVABLE == obs_type ||
        FLAGS_enable_openspace_use_polygon_plan) {
      const auto& obstacle_polygon = obstacle->PerceptionPolygon();
      if (obstacle_polygon.HasOverlap(adc_polygon)) {
        AINFO << "IsADCBackCollisionUseDynamic" << obstacle->Id() << "obs_type "
              << obs_type;
        return true;
      }
    } else {
      const auto& box = obstacle->PerceptionBoundingBox();
      if (ego_box.HasOverlap(box)) {
        AINFO << "IsADCBackCollisionUseDynamic"
              << "obstacle id " << obstacle->Id() << "obs_type " << obs_type;
        return true;
      }
    }
  }

  return false;
}

bool TEBFallbackDecider::IsADCWillCollisionWithDynamicVehicle(
    const TrajGearPair& trajectory_gear_pair,
    const std::vector<const Obstacle*>& obstacles,
    const size_t* current_index) {
  const auto& trajectory_pb = trajectory_gear_pair.first;
  const size_t point_size = trajectory_pb.NumOfPoints();
  for (size_t i = *current_index; i < point_size; ++i) {
    const auto& trajectory_point = trajectory_pb.TrajectoryPointAt(i);
    double ego_theta = trajectory_point.path_point().theta();
    Box2d ego_box(
        {trajectory_point.path_point().x(), trajectory_point.path_point().y()},
        ego_theta, ego_length_, ego_width_);
    Vec2d shift_vec{shift_distance_ * std::cos(ego_theta),
                    shift_distance_ * std::sin(ego_theta)};
    ego_box.Shift(shift_vec);
    for (const auto* obstacle : obstacles) {
      if (obstacle->IsStatic() || obstacle->IsVirtual()) {
        continue;
      }
      const auto& obs_type = obstacle->Perception().type();
      if (perception::PerceptionObstacle::VEHICLE != obs_type) {
        continue;
      }
      double relative_time = 0.0;
      while (relative_time < FLAGS_teb_obs_prediction_time) {
        const auto& point = obstacle->GetPointAtTime(relative_time);

        const auto& obs_box = obstacle->GetBoundingBox(point);
        if (ego_box.HasOverlap(obs_box)) {
          AINFO << "dynamic obj check collision: [" << i << "]"
                << "relative_time() " << trajectory_point.relative_time()
                << "obstacle id " << obstacle->Id() << "obstacle->speed() "
                << obstacle->speed() << "obs_type " << obs_type;
          frame_->mutable_open_space_info()->set_is_blocked_by_dynamic_obj(
              true);
          return true;
        }
        relative_time += kDeltaT;
      }
    }
  }

  return false;
}

bool TEBFallbackDecider::IsADCWillCollisionWithStaticVehicle(
    const std::vector<const Obstacle*>& obstacles, const size_t* current_index,
    const TrajGearPair& trajectory_gear_pair) {
  const auto& trajectory_pb = trajectory_gear_pair.first;
  const size_t point_size = trajectory_pb.NumOfPoints();
  for (size_t i = *current_index; i < point_size; ++i) {
    const auto& trajectory_point = trajectory_pb.TrajectoryPointAt(i);

    const double adc_speed =
        std::fabs(injector_->vehicle_state()->linear_velocity());
    const double ttc_s2 =
        std::max(kMinTTCS_2, adc_speed * FLAGS_teb_obs_prediction_time);

    double rel_s = std::fabs(
        trajectory_point.path_point().s() -
        trajectory_pb.TrajectoryPointAt(*current_index).path_point().s());
    if (rel_s > ttc_s2) {
      break;
    }
    double ego_theta = trajectory_point.path_point().theta();
    Box2d ego_box(
        {trajectory_point.path_point().x(), trajectory_point.path_point().y()},
        ego_theta, ego_length_, ego_width_);
    Vec2d shift_vec{shift_distance_ * std::cos(ego_theta),
                    shift_distance_ * std::sin(ego_theta)};
    ego_box.Shift(shift_vec);
    for (const auto* obstacle : obstacles) {
      const auto& obs_type = obstacle->Perception().type();
      if (!obstacle->IsStatic() || obstacle->IsVirtual() ||
          perception::PerceptionObstacle::VEHICLE != obs_type) {
        continue;
      }

      const auto& obs_box = obstacle->PerceptionBoundingBox();
      if (ego_box.HasOverlap(obs_box)) {
        AINFO << "static obj check collision: [" << i << "]"
              << "relative_time() " << trajectory_point.relative_time()
              << "obstacle id " << obstacle->Id();
        return true;
      }
    }
  }

  return false;
}

void TEBFallbackDecider::ReverseTrajectory(
    const size_t* current_idx, const TrajGearPair* trajectory_gear_pair) {
  std::vector<common::TrajectoryPoint> trajectory_points;
  const auto& trajectory_pb = trajectory_gear_pair->first;
  const size_t point_size = trajectory_pb.NumOfPoints();
  trajectory_points.clear();
  for (int i = static_cast<int>(point_size - 1); i >= 0; --i) {
    auto trajectory_point = trajectory_pb.TrajectoryPointAt(i);
    double delta_s = std::fabs(
        trajectory_point.path_point().s() -
        trajectory_pb.TrajectoryPointAt(*current_idx).path_point().s());
    if (delta_s < kReverseS) {
      // lwt: after observe,  not need reverse  path_point().s()
      trajectory_point.set_relative_time(-trajectory_point.relative_time());
      double v =
          std::max(FLAGS_teb_back_speed,
                   std::min(-trajectory_point.v(), FLAGS_teb_min_front_speed));
      trajectory_point.set_v(v);
      trajectory_points.emplace_back(std::move(trajectory_point));
    }
  }
  trajectory_points.back().set_v(0.0);
  DiscretizedTrajectory reverse_traj(trajectory_points);
  auto* reverse_traj_pair =
      frame_->mutable_open_space_info()->mutable_reverse_trajectory();

  reverse_traj_pair->second =
      trajectory_gear_pair->second == canbus::Chassis::GEAR_REVERSE
          ? canbus::Chassis::GEAR_DRIVE
          : canbus::Chassis::GEAR_REVERSE;

  reverse_traj_pair->first = reverse_traj;
  AINFO << "trajectory_points size " << trajectory_points.size()
        << "reverse_traj_pair->second " << reverse_traj_pair->second;
}

void TEBFallbackDecider::ProcessBlockScenario(
    const std::vector<const Obstacle*>& obstacles, size_t* current_idx,
    TrajGearPair* trajectory_gear_pair) {
  AINFO << "fallback_start_indstop_indexex " << *current_idx;
  if (!FLAGS_enable_new_teb_test_func) {
    return;
  }
  static size_t static_collision_cnt = 0;
  const auto scenario_type = injector_->planning_context()
                                 ->planning_status()
                                 .scenario()
                                 .scenario_type();
  //  uturn is 19
  if (ScenarioConfig::UTURN_TEB != scenario_type) {
    injector_->is_uturn_blocked_ = false;
    static_collision_cnt = 0;
    return;
  }
  if (IsADCWillCollisionWithStaticVehicle(obstacles, current_idx,
                                          *trajectory_gear_pair)) {
    ++static_collision_cnt;
  } else {
    static_collision_cnt = 0;
  }
  const auto& start_point = trajectory_gear_pair->first.front().path_point();
  double dist_to_start =
      std::sqrt((start_point.x() - injector_->vehicle_state()->x()) *
                    (start_point.x() - injector_->vehicle_state()->x()) +
                (start_point.y() - injector_->vehicle_state()->y()) *
                    (start_point.y() - injector_->vehicle_state()->y()));
  if ((static_collision_cnt > kVehBlockCnt &&
       std::fabs(injector_->vehicle_state()->linear_velocity()) < kStopSpeed) &&
      !injector_->is_uturn_blocked_ && dist_to_start > kReachDist) {
    ReverseTrajectory(current_idx, trajectory_gear_pair);
    injector_->is_uturn_blocked_ = true;
    *trajectory_gear_pair =
        frame_->mutable_open_space_info()->reverse_trajectory();
  }

  auto* previous_frame = injector_->use_thread_in_play_street()
                             ? injector_->frame_teb_history()->Latest()
                             : injector_->frame_history()->Latest();

  if (injector_->is_uturn_blocked_ &&
      previous_frame->open_space_info().reverse_trajectory().first.size() > 0) {
    auto* reverse_trajectory =
        frame_->mutable_open_space_info()->mutable_reverse_trajectory();
    *reverse_trajectory =
        previous_frame->open_space_info().reverse_trajectory();
    *trajectory_gear_pair = *reverse_trajectory;
  }

  if (previous_frame->open_space_info().reverse_trajectory().first.empty() &&
      !static_collision_cnt) {
    injector_->is_uturn_blocked_ = false;
  }
  auto* chosen_traj = frame_->mutable_open_space_info()
                          ->mutable_chosen_partitioned_trajectory();
  *chosen_traj = *trajectory_gear_pair;
  const auto& end_point = trajectory_gear_pair->first.back().path_point();
  double dist =
      std::sqrt((end_point.x() - injector_->vehicle_state()->x()) *
                    (end_point.x() - injector_->vehicle_state()->x()) +
                (end_point.y() - injector_->vehicle_state()->y()) *
                    (end_point.y() - injector_->vehicle_state()->y()));
  std::string gear = trajectory_gear_pair->second == canbus::Chassis::GEAR_DRIVE
                         ? " D "
                         : " R ";
  AINFO << "is_uturn_blocked_ " << injector_->is_uturn_blocked_
        << "static_collision_cnt " << static_collision_cnt << "prev size "
        << previous_frame->open_space_info().reverse_trajectory().first.size()
        << "dist " << dist << "gear" << gear << "v "
        << injector_->vehicle_state()->linear_velocity();
  const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                            injector_->vehicle_state()->y()};
  *current_idx = trajectory_gear_pair->first.QueryNearestPoint(adc_position);
  if (std::fabs(injector_->vehicle_state()->linear_velocity()) < kStopSpeed &&
      dist < kReachDist) {
    static_collision_cnt = 0;
    injector_->is_uturn_blocked_ = false;
    AINFO << "kReachDist reverse end";
  }
}

bool TEBFallbackDecider::CheckObstacleCollision(
    const double relative_s, const common::math::Box2d& ego_box,
    const std::vector<const Obstacle*>& obstacles) {
  const double adc_speed =
      std::fabs(injector_->vehicle_state()->linear_velocity());
  const double ttc_s1 =
      std::max(kMinTTCS_1, adc_speed * FLAGS_teb_static_obs_ttc);
  const auto& adc_polygon = Polygon2d(ego_box);
  for (const auto* obstacle : obstacles) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    const auto& obs_type = obstacle->Perception().type();
    if (obstacle->IsStatic()) {
      if (relative_s > ttc_s1) {
        continue;
      }
      if (perception::PerceptionObstacle::UNKNOWN == obs_type ||
          perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type ||
          perception::PerceptionObstacle::UNKNOWN_UNMOVABLE == obs_type ||
          FLAGS_enable_openspace_use_polygon_plan) {
        const auto& obstacle_polygon = obstacle->PerceptionPolygon();
        if (obstacle_polygon.HasOverlap(adc_polygon)) {
          AINFO << ": ["
                << "]"
                << "relative_s " << relative_s << "obstacle id "
                << obstacle->Id() << "obs_type " << obs_type;
          return true;
        }
      } else {
        const auto& box = obstacle->PerceptionBoundingBox();
        if (ego_box.HasOverlap(box)) {
          AINFO << ": ["
                << "]"
                << "relative_s " << relative_s << "obstacle id "
                << obstacle->Id() << "obs_type " << obs_type;
          return true;
        }
      }
    } else {
      double relative_t = 0.0;
      double check_time = perception::PerceptionObstacle::PEDESTRIAN == obs_type
                              ? kPedCheckTime
                              : FLAGS_teb_obs_prediction_time;
      while (relative_t < check_time) {
        const auto& point = obstacle->GetPointAtTime(relative_t);
        if (perception::PerceptionObstacle::UNKNOWN == obs_type ||
            perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type ||
            FLAGS_enable_openspace_use_polygon_plan) {
          const Polygon2d& obs_polygon = obstacle->GetPolygon(point);
          if (obs_polygon.HasOverlap(adc_polygon)) {
            AINFO << ": ["
                  << "]"
                  << "relative_s " << relative_s << "obstacle id "
                  << obstacle->Id() << "obstacle->speed() " << obstacle->speed()
                  << "obs_type " << obs_type;
            return true;
          }
        } else {
          const auto& obs_box = obstacle->GetBoundingBox(point);
          if (ego_box.HasOverlap(obs_box)) {
            AINFO << ": ["
                  << "]"
                  << "relative_s " << relative_s << "obstacle id "
                  << obstacle->Id() << "obstacle->speed() " << obstacle->speed()
                  << "obs_type " << obs_type;
            return true;
          }
        }
        relative_t += kDeltaT;
      }
    }
  }

  for (const auto& polygon : costmap_polygons_) {
    if (polygon.HasOverlap(adc_polygon)) {
      AINFO << "relative_s" << relative_s << "HasOverlap  costmap_polygons_ ";
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace century
