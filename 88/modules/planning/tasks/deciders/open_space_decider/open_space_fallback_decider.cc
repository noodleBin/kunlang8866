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

#include "modules/planning/tasks/deciders/open_space_decider/open_space_fallback_decider.h"

namespace century {
namespace planning {
using century::common::Status;
using century::common::TrajectoryPoint;
using century::common::math::Box2d;
using century::common::math::Polygon2d;
using century::common::math::Vec2d;
namespace {
constexpr double kDeltaT = 0.25;                // m
constexpr double kCollisionCheckDeltaT = 0.15;  // s
constexpr double kMaxDistanceOffset = 100.7;    // m , 100 means close
constexpr int kSamePointsNum = 20;              //

constexpr double kMaxAcc = 4.0;   // vehicle_config.max_acceleration();
constexpr double kMaxDec = -4.0;  // vehicle_config.max_deceleration();
}  // namespace

OpenSpaceFallbackDecider::OpenSpaceFallbackDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {}

bool OpenSpaceFallbackDecider::QuardraticFormulaLowerSolution(const double a,
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

void OpenSpaceFallbackDecider::EmergencyStop(TrajGearPair& traj_info) {
  AERROR << "ADC tracking error is too large, triggering immediate stop!";
  const double current_v = injector_->vehicle_state()->linear_velocity();
  const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                            injector_->vehicle_state()->y()};
  size_t current_index = traj_info.first.QueryNearestPoint(adc_position);
  for (size_t i = current_index; i < traj_info.first.NumOfPoints(); ++i) {
    if (i == current_index) {
      traj_info.first.at(i).set_v(current_v);
    } else {
      traj_info.first.at(i).set_v(0.0);
    }
    traj_info.first.at(i).set_a(kMaxDec);
  }
  *(frame_->mutable_open_space_info()->mutable_fallback_trajectory()) =
      traj_info;
  return;
}

Status OpenSpaceFallbackDecider::Process(Frame* frame) {
  size_t first_collision_index = 0;
  size_t fallback_start_index = 0;

  // change flag
  frame_->mutable_open_space_info()->set_fallback_flag(true);
  // generate fallback trajectory base on current partition trajectory
  // vehicle speed is decreased to zero inside safety distance
  TrajGearPair fallback_trajectory_pair_candidate =
      frame->open_space_info().chosen_partitioned_trajectory();

  if (openspace_common_.CheckAdcErrorStates(
          injector_->vehicle_state()->vehicle_state(),
          fallback_trajectory_pair_candidate)) {
    AERROR  << "check adc error states failed, return ok.";
    frame_->mutable_open_space_info()->set_fallback_flag(true);
    EmergencyStop(fallback_trajectory_pair_candidate);
    return Status::OK();
  }

  if (!IsCollisionFreeTrajectory(
          frame->open_space_info().chosen_partitioned_trajectory(),
          frame->obstacles(), &fallback_start_index, &first_collision_index)) {
    AERROR << "fall back decider, IsCollisionFreeTrajectory "
              "false!!! send fallback trajectory.";
    const auto future_collision_point =
        fallback_trajectory_pair_candidate.first[first_collision_index];

    // Fallback starts from current location but with vehicle velocity
    auto fallback_start_point =
        fallback_trajectory_pair_candidate.first[fallback_start_index];

    fallback_start_point.set_v(
        injector_->vehicle_state()->vehicle_state().linear_velocity());

    *(frame_->mutable_open_space_info()->mutable_future_collision_point()) =
        future_collision_point;

    // min stop distance: (max_acc)
    double min_stop_distance =
        0.5 * fallback_start_point.v() * fallback_start_point.v() / 4.0;

    double stop_distance =
        fallback_trajectory_pair_candidate.second == canbus::Chassis::GEAR_DRIVE
            ? std::max(future_collision_point.path_point().s() -
                           fallback_start_point.path_point().s() - 1.0,
                       0.0)
            : std::min(future_collision_point.path_point().s() -
                           fallback_start_point.path_point().s() + 1.0,
                       0.0);

    double stop_deceleration =
        fallback_trajectory_pair_candidate.second ==
                canbus::Chassis::GEAR_REVERSE
            ? std::min(fallback_start_point.v() * fallback_start_point.v() /
                           (2.0 * (stop_distance + 1e-6)),
                       kMaxAcc)
            : std::max(-fallback_start_point.v() * fallback_start_point.v() /
                           (2.0 * (stop_distance + 1e-6)),
                       kMaxDec);

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
          fallback_start_point.v());
      fallback_trajectory_pair_candidate.first[i].set_a(stop_deceleration);
    }

    // If stop_index == fallback_start_index;
    if (fallback_start_index == stop_index) {
      // lwt: use origin path
      for (size_t i = fallback_start_index;
           i < fallback_trajectory_pair_candidate.first.NumOfPoints(); ++i) {
        fallback_trajectory_pair_candidate.first[i].set_v(0.0);
      }

      *(frame_->mutable_open_space_info()->mutable_fallback_trajectory()) =
          fallback_trajectory_pair_candidate;

      return Status::OK();
    }

    for (size_t i = fallback_start_index; i <= stop_index; ++i) {
      double new_relative_time = 0.0;
      double c =
          -2.0 * fallback_trajectory_pair_candidate.first[i].path_point().s();

      if (QuardraticFormulaLowerSolution(stop_deceleration,
                                         2.0 * fallback_start_point.v(), c,
                                         &new_relative_time) &&
          std::abs(
              fallback_trajectory_pair_candidate.first[i].path_point().s()) <=
              std::abs(stop_distance)) {
        fallback_trajectory_pair_candidate.first[i].set_v(0.0);
        fallback_trajectory_pair_candidate.first[i].set_a(stop_deceleration);

        fallback_trajectory_pair_candidate.first[i].set_relative_time(
            new_relative_time);
      } else {
        AERROR << "QuardraticFormulaLowerSolution solving failed, stop "
                 "immediately!";
        fallback_trajectory_pair_candidate.first[i].set_v(0.0);
      }
    }

    for (size_t i = stop_index;
         i < fallback_trajectory_pair_candidate.first.NumOfPoints(); ++i) {
      fallback_trajectory_pair_candidate.first[i].set_v(0.0);
    }

    *(frame_->mutable_open_space_info()->mutable_fallback_trajectory()) =
        fallback_trajectory_pair_candidate;
  } else {
    frame_->mutable_open_space_info()->set_fallback_flag(false);
  }
  return Status::OK();
}

bool OpenSpaceFallbackDecider::IsCollisionFreeTrajectory(
    const TrajGearPair& trajectory_gear_pair,
    const std::vector<const Obstacle*>& obstacles, size_t* current_index,
    size_t* first_collision_index) {
  // prediction time resolution: FLAGS_trajectory_time_resolution
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_length = FLAGS_enable_astar_fallback_buffer
                          ? vehicle_config.vehicle_param().length() +
                                FLAGS_astar_first_long_buffer
                          : vehicle_config.vehicle_param().length();
  double ego_width = FLAGS_enable_astar_fallback_buffer
                         ? vehicle_config.vehicle_param().width() +
                               FLAGS_astar_first_lat_buffer
                         : vehicle_config.vehicle_param().width();
  auto trajectory_pb = trajectory_gear_pair.first;
  const size_t point_size = trajectory_pb.NumOfPoints();
  const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                            injector_->vehicle_state()->y()};
  // *current_index = trajectory_pb.QueryLowerBoundPoint(0.0);
  *current_index = trajectory_pb.QueryNearestPoint(adc_position);
  for (size_t i = *current_index; i < point_size; ++i) {
    const auto& trajectory_point = trajectory_pb.TrajectoryPointAt(i);
    double delta_t =
        trajectory_point.relative_time() -
        trajectory_pb.TrajectoryPointAt(*current_index).relative_time();
    if (delta_t > config_.open_space_fallback_decider_config()
                      .open_space_prediction_time_period()) {
      return true;
    }

    double ego_theta = trajectory_point.path_point().theta();
    Box2d ego_box(
        {trajectory_point.path_point().x(), trajectory_point.path_point().y()},
        ego_theta, ego_length, ego_width);
    double shift_distance =
        (ego_length) * 0.5 -
        vehicle_config.vehicle_param().back_edge_to_center();
    Vec2d shift_vec{shift_distance * std::cos(ego_theta),
                    shift_distance * std::sin(ego_theta)};
    ego_box.Shift(shift_vec);
    if (CheckObstacleCollision(delta_t, ego_box, obstacles)) {
      *first_collision_index = i;
      AERROR << "first_collision_index: [" << i << "]";
      return false;
    }
  }
  return true;
}

bool OpenSpaceFallbackDecider::IsADCDeviationTrajectory(
    const TrajGearPair& trajectory_gear_pair) {
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

  ADEBUG << "current_index: " << current_index << ", adc_position: ("
         << adc_position.x() << ", " << adc_position.y() << ")"
         << ", nearest_pt: (" << trajectory_point.path_point().x() << ", "
         << trajectory_point.path_point().y() << ")"
         << ", distance: " << ds;

  if (ds > kMaxDistanceOffset) {
    ADEBUG << " adc nearest point ds = " << ds << " need stop";
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

bool OpenSpaceFallbackDecider::CheckObstacleCollision(
    const double relative_time, const common::math::Box2d& ego_box,
    const std::vector<const Obstacle*>& obstacles) {
  double static_or_ped_check_time = FLAGS_fallback_collsion_reduction_time *
                                    config_.open_space_fallback_decider_config()
                                        .open_space_prediction_time_period();
  for (const auto* obstacle : obstacles) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    const auto& obs_type = obstacle->Perception().type();
    if (obstacle->IsStatic()) {
      if (relative_time > static_or_ped_check_time) {
        continue;
      }
      if (perception::PerceptionObstacle::UNKNOWN == obs_type ||
          perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type ||
          perception::PerceptionObstacle::UNKNOWN_UNMOVABLE == obs_type ||
          FLAGS_enable_openspace_use_polygon_plan) {
        const auto& obstacle_polygon = obstacle->PerceptionPolygon();
        const auto& adc_polygon = Polygon2d(ego_box);
        if (obstacle_polygon.HasOverlap(adc_polygon)) {
          ADEBUG << "first_collision_index: ["
                << "]"
                << "trajectory_point.relative_time() " << relative_time
                << "obstacle id " << obstacle->Id() << "obs_type " << obs_type;
          return true;
        }
      } else {
        const auto& box = obstacle->PerceptionBoundingBox();
        if (ego_box.HasOverlap(box)) {
          ADEBUG << "first_collision_index: ["
                << "]"
                << "relative_time " << relative_time << "obstacle id "
                << obstacle->Id() << "obs_type " << obs_type;
          return true;
        }
      }
    } else {
      double relative_t = 0.0;
      double check_time = perception::PerceptionObstacle::PEDESTRIAN == obs_type
                              ? static_or_ped_check_time
                              : config_.open_space_fallback_decider_config()
                                    .open_space_prediction_time_period();
      while (relative_t < check_time) {
        const auto& point = obstacle->GetPointAtTime(relative_t);
        if (perception::PerceptionObstacle::UNKNOWN == obs_type ||
            perception::PerceptionObstacle::UNKNOWN_MOVABLE == obs_type ||
            FLAGS_enable_openspace_use_polygon_plan) {
          const Polygon2d& obs_polygon = obstacle->GetPolygon(point);
          const auto& adc_polygon = Polygon2d(ego_box);
          if (obs_polygon.HasOverlap(adc_polygon)) {
            ADEBUG << "first_collision_index: ["
                  << "]"
                  << "trajectory_point.relative_time() " << relative_time
                  << "obstacle id " << obstacle->Id() << "obstacle->speed() "
                  << obstacle->speed() << "obs_type " << obs_type;
            return true;
          }
        } else {
          const auto& obs_box = obstacle->GetBoundingBox(point);
          if (ego_box.HasOverlap(obs_box)) {
            ADEBUG << "first_collision_index: ["
                  << "]"
                  << "trajectory_point.relative_time() " << relative_time
                  << "obstacle id " << obstacle->Id() << "obstacle->speed() "
                  << obstacle->speed() << "obs_type " << obs_type;
            return true;
          }
        }
        relative_t += kDeltaT;
      }
    }
  }
  return false;
}

}  // namespace planning
}  // namespace century
