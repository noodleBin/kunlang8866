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

#include "modules/planning/common/trajectory_stitcher.h"

#include <algorithm>

#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/angle.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_model/vehicle_model.h"
#include "modules/planning/common/planning_gflags.h"

namespace century {
namespace planning {

using century::common::TrajectoryPoint;
using century::common::VehicleModel;
using century::common::VehicleState;
using century::common::math::InterpolateUsingLinearApproximation;
using century::common::math::Vec2d;

namespace {
constexpr double kEpsilon = 1.0e-6;
constexpr double kEpsilon_v = 0.1;
constexpr double kEpsilon_a = 0.4;
constexpr double kReinitRatio = 0.3;
constexpr double kMinBeginRelTime = -0.5;
constexpr double kDiagonalSpeed = 5.0;
constexpr double kMinTrajectoryLength = 1.5;
constexpr double kStitchingLongTime = -5.0;
}  // namespace

void TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
    const double planning_cycle_time, const common::VehicleState& vehicle_state,
    common::TrajectoryPoint* const reinit_point) {
  CHECK_NOTNULL(reinit_point);

  auto* path_point = reinit_point->mutable_path_point();
  path_point->set_s(0.0);
  path_point->set_x(vehicle_state.x());
  path_point->set_y(vehicle_state.y());
  path_point->set_z(vehicle_state.z());
  path_point->set_theta(vehicle_state.heading());
  path_point->set_kappa(vehicle_state.kappa());
  reinit_point->set_v(vehicle_state.linear_velocity());
  reinit_point->set_a(vehicle_state.linear_acceleration());
  reinit_point->set_relative_time(planning_cycle_time);
  return;
}

void TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const double planning_cycle_time, const common::VehicleState& vehicle_state,
    const common::TrajectoryPoint* matched_point,
    std::vector<common::TrajectoryPoint>* const stitching_trajectory) {
  CHECK_NOTNULL(stitching_trajectory);

  TrajectoryPoint reinit_point;
  // TODO(Jinyun/Yu): adjust kEpsilon if corrected IMU acceleration provided
  if (std::abs(vehicle_state.linear_velocity()) < kEpsilon_v &&
      std::abs(vehicle_state.linear_acceleration()) < kEpsilon_a) {
    ComputeTrajectoryPointFromVehicleState(planning_cycle_time, vehicle_state,
                                           &reinit_point);
  } else {
    /**
     * When the ADC has speed, it does not directly use the ADC's current pose
     * as the starting point for re-planning, but interpolates a starting
     * point based on the ADC's speed for re-planning, so as to improve
     * driving smoothness and safety.
     *
     */
    if (matched_point != nullptr) {
        AINFO << "ComputeTrajectoryPointFromVehicleState";
        AINFO<<"planning_cycle_time = "<<planning_cycle_time;
        ComputeTrajectoryPointFromVehicleState(planning_cycle_time,
                                               vehicle_state, &reinit_point);

        auto* init_path_point = reinit_point.mutable_path_point();
        double adc_v = vehicle_state.linear_velocity();
        auto matched_path_point = matched_point->path_point();
        double adc_x = vehicle_state.x();
        double adc_y = vehicle_state.y();
        double matched_x = matched_path_point.x();
        double matched_y = matched_path_point.y();
        AINFO << "adc_x = " << adc_x << "   adc_y = " << adc_y
              << "  matched_x = " << matched_x << "  matched_y = " << matched_y;
        double diff_x = matched_x - adc_x;
        double diff_y = matched_y - adc_y;
        double distance = std::sqrt(diff_x * diff_x + diff_y * diff_y);
        matched_path_point.set_s(distance);
          AINFO << "distance = " << distance;
        double interpolated_s = kReinitRatio * distance;
        AINFO<<"interpolated_s_1 = "<<interpolated_s;
        double matched_v = matched_point->v();
        // In this scope, matched_v is always greater than 0.
        if (common::util::IsFloatEqual(matched_v, 0.0)) {
          AERROR << "the matched_v is zero!";
          matched_v = FLAGS_numerical_epsilon;
        }
        AINFO<<"adc_v = "<<adc_v<<"  matched_v = "<<matched_v;
        AINFO << "reinit_point.a = " << reinit_point.a()
              << "   matched_point.a = " << matched_point->a();
        interpolated_s *= adc_v / matched_v;
        interpolated_s = std::min(interpolated_s, distance);
        AINFO << "interpolated_s_2 = " << interpolated_s;

        auto interpolated_path_point = InterpolateUsingLinearApproximation(
            *init_path_point, matched_path_point, interpolated_s);
        AINFO << "init_path_point.s() = " << init_path_point->s()
              << "   matched_path_point.s() = " << matched_path_point.s()
              << "    interpolated_path_point.s() = "
              << interpolated_path_point.s();
        AINFO << "inter_x = " << interpolated_path_point.x()
              << "   inter_y = " << interpolated_path_point.y();
        init_path_point->CopyFrom(interpolated_path_point);
        // The s value of the replanning starting point is 0
        init_path_point->set_s(0.0);
        AINFO << "reinit_point kappa = " << reinit_point.path_point().kappa();
        AINFO << "matched_path_point kappa = " << matched_path_point.kappa();
    } else if (FLAGS_enable_stitching_with_prediction) {
      AINFO << "FLAGS_enable_stitching_with_prediction";
      VehicleState predicted_vehicle_state;
      predicted_vehicle_state =
          VehicleModel::Predict(planning_cycle_time, vehicle_state);
      ComputeTrajectoryPointFromVehicleState(
          planning_cycle_time, predicted_vehicle_state, &reinit_point);
    }
  }

  if (!stitching_trajectory->empty()) {
    stitching_trajectory->clear();
  }
  stitching_trajectory->emplace_back(std::move(reinit_point));

  return;
}

void TrajectoryStitcher::ComputeReinitOpenSpaceStitchingTrajectory(
    const double planning_cycle_time, const common::VehicleState& vehicle_state,
    const common::TrajectoryPoint* matched_point,
    std::vector<common::TrajectoryPoint>* const stitching_trajectory) {
  CHECK_NOTNULL(stitching_trajectory);

  TrajectoryPoint reinit_point;
  // TODO(Jinyun/Yu): adjust kEpsilon if corrected IMU acceleration provided
  if (std::abs(vehicle_state.linear_velocity()) < kEpsilon_v &&
      std::abs(vehicle_state.linear_acceleration()) < kEpsilon_a) {
    ComputeTrajectoryPointFromVehicleState(planning_cycle_time, vehicle_state,
                                           &reinit_point);
  } else {
    /**
     * When the ADC has speed, it does not directly use the ADC's current pose
     * as the starting point for re-planning, but interpolates a starting
     * point based on the ADC's speed for re-planning, so as to improve
     * driving smoothness and safety.
     *
     */
    if (matched_point != nullptr) {
      ComputeTrajectoryPointFromVehicleState(planning_cycle_time, vehicle_state,
                                             &reinit_point);

      auto* init_path_point = reinit_point.mutable_path_point();
      double adc_v = vehicle_state.linear_velocity();
      auto matched_path_point = matched_point->path_point();
      double adc_x = vehicle_state.x();
      double adc_y = vehicle_state.y();
      double matched_x = matched_path_point.x();
      double matched_y = matched_path_point.y();
      double diff_x = matched_x - adc_x;
      double diff_y = matched_y - adc_y;
      double distance = std::sqrt(diff_x * diff_x + diff_y * diff_y);
      matched_path_point.set_s(distance);

      double interpolated_s = kReinitRatio * distance;
      double matched_v = matched_point->v();
      // In this scope, matched_v is always greater than 0.
      if (common::util::IsFloatEqual(matched_v, 0.0)) {
        AERROR << "the matched_v is zero!";
        matched_v = FLAGS_numerical_epsilon;
      }
      interpolated_s *= adc_v / matched_v;
      interpolated_s = std::min(interpolated_s, distance);

      auto interpolated_path_point = InterpolateUsingLinearApproximation(
          *init_path_point, matched_path_point, interpolated_s);
      init_path_point->CopyFrom(interpolated_path_point);
      // The s value of the replanning starting point is 0
      init_path_point->set_s(0.0);
    } else if (FLAGS_enable_stitching_with_prediction) {
      VehicleState predicted_vehicle_state;
      predicted_vehicle_state =
          VehicleModel::Predict(planning_cycle_time, vehicle_state);
      ComputeTrajectoryPointFromVehicleState(
          planning_cycle_time, predicted_vehicle_state, &reinit_point);
    }
  }

  if (!stitching_trajectory->empty()) {
    stitching_trajectory->clear();
  }
  stitching_trajectory->emplace_back(std::move(reinit_point));

  return;
}

// only used in navigation mode
void TrajectoryStitcher::TransformLastPublishedTrajectory(
    const double x_diff, const double y_diff, const double theta_diff,
    PublishableTrajectory* prev_trajectory) {
  if (!prev_trajectory) {
    return;
  }

  // R^-1
  double cos_theta = std::cos(theta_diff);
  double sin_theta = -std::sin(theta_diff);

  // -R^-1 * t
  auto tx = -(cos_theta * x_diff - sin_theta * y_diff);
  auto ty = -(sin_theta * x_diff + cos_theta * y_diff);

  std::for_each(prev_trajectory->begin(), prev_trajectory->end(),
                [&cos_theta, &sin_theta, &tx, &ty,
                 &theta_diff](common::TrajectoryPoint& p) {
                  auto x = p.path_point().x();
                  auto y = p.path_point().y();
                  auto theta = p.path_point().theta();

                  auto x_new = cos_theta * x - sin_theta * y + tx;
                  auto y_new = sin_theta * x + cos_theta * y + ty;
                  auto theta_new =
                      common::math::NormalizeAngle(theta - theta_diff);

                  p.mutable_path_point()->set_x(x_new);
                  p.mutable_path_point()->set_y(y_new);
                  p.mutable_path_point()->set_theta(theta_new);
                });
}

bool TrajectoryStitcher::IsNeedReinitStitchingTrajectory(
    const common::VehicleState& vehicle_state, const double current_timestamp,
    const double planning_cycle_time,
    const PublishableTrajectory* prev_trajectory,
    std::vector<common::TrajectoryPoint>* const stitching_trajectory,
    std::string* const replan_reason,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (!FLAGS_enable_trajectory_stitcher) {
    *replan_reason = "stitch is disabled by gflag.";
    ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                     nullptr, stitching_trajectory);
    return false;
  }
  if (injector->is_new_routing_for_replan_) {
    *replan_reason = "is new routing replan.";
    ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                     nullptr, stitching_trajectory);
    return false;
  }
  if (prev_trajectory) {
    if (!prev_trajectory->empty()) {
      double last_trajectory_length = prev_trajectory->back().path_point().s() -
                                      prev_trajectory->front().path_point().s();
      // AINFO << "last trajectory length = " << last_trajectory_length;
      // double replan_longitudinal_distance_threshold =
      //     FLAGS_replan_longitudinal_distance_threshold;
      // AINFO << "replan_longitudinal_distance_threshold = "
      //       << replan_longitudinal_distance_threshold;
      // stop trajectory & no start up trajectory
      if (last_trajectory_length < kMinTrajectoryLength &&
          prev_trajectory->front().relative_time() < kStitchingLongTime) {
        *replan_reason = "is short trajectory replan.";
        ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                         nullptr, stitching_trajectory);
        return false;
      }
    }
  }
  if (injector->NeedToReplan() &&
      std::fabs(injector->vehicle_state()->linear_velocity()) < kDiagonalSpeed) {
    // if(0){
        *replan_reason = "replan for no no need diagonal.";
    AINFO << "replan for no no need diagonal ";
    ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                     nullptr, stitching_trajectory);
    injector->SetNeedToReplan(false);
    // injector->SetLastFrameNeedDiagonal(true);
    return false;
  }
  if (!prev_trajectory) {
    *replan_reason = "replan for no previous trajectory.";
    ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                     nullptr, stitching_trajectory);
    return false;
  }

  if (FLAGS_enable_TEB_thread && injector->exit_from_teb_.load()) {
    *replan_reason = "replan for last is teb.";
    ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                     nullptr, stitching_trajectory);
    return false;
  }

  if (vehicle_state.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    *replan_reason = "replan for manual mode.";
    ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                     nullptr, stitching_trajectory);
    return false;
  }

  // if (injector->NeedReinitStartPoint()) {
  //   *replan_reason = "replan for NeedReinitStartPoint.";
  //   ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
  //                                    nullptr, stitching_trajectory);
  //   ADEBUG << *replan_reason;
  //   return false;
  // }

  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  if (prev_trajectory_size == 0) {
    ADEBUG << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use "
              "origin car status instead.";
    *replan_reason = "replan for empty previous trajectory.";
    ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                     nullptr, stitching_trajectory);
    return false;
  }
  const double veh_rel_time =
      current_timestamp - prev_trajectory->header_time();

  size_t time_matched_index =
      prev_trajectory->QueryLowerBoundPoint(veh_rel_time);

  if (time_matched_index == 0 &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time() &&
      FLAGS_enable_replan_for_smaller_start_point) {
    AWARN << "current time smaller than the previous trajectory's first time";
    *replan_reason =
        "replan for current time smaller than the previous trajectory's first "
        "time.";
    ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                     nullptr, stitching_trajectory);
    return false;
  }
  if (time_matched_index + 1 >= prev_trajectory_size) {
    AWARN << "current time beyond the previous trajectory's last time"
          << "time_matched_index =" << time_matched_index
          << "prev_trajectory_size " << prev_trajectory_size
          << "veh_rel_time= " << veh_rel_time;
    *replan_reason =
        "replan for current time beyond the previous trajectory's last time";
    ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                     nullptr, stitching_trajectory);
    return false;
  }
  auto time_matched_point = prev_trajectory->TrajectoryPointAt(
      static_cast<uint32_t>(time_matched_index));

  if (!time_matched_point.has_path_point()) {
    *replan_reason = "replan for previous trajectory missed path point";
    ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                     nullptr, stitching_trajectory);
    return false;
  }

  return true;
}

/* Planning from current vehicle state if:
   1. the auto-driving mode is off
   (or) 2. we don't have the trajectory from last planning cycle
   (or) 3. the position deviation from actual and target is too high
*/
bool TrajectoryStitcher::ComputeStitchingTrajectory(
    const common::VehicleState& vehicle_state, const double current_timestamp,
    const double planning_cycle_time, const size_t preserved_points_num,
    const bool replan_by_offset, const PublishableTrajectory* prev_trajectory,
    std::vector<common::TrajectoryPoint>* const stitching_trajectory,
    std::string* const replan_reason,
    const std::shared_ptr<DependencyInjector>& injector) {
  // prev_trajectory may be null.
  // CHECK_NOTNULL(prev_trajectory);
  CHECK_NOTNULL(stitching_trajectory);
  CHECK_NOTNULL(replan_reason);
  bool is_need_reinit = IsNeedReinitStitchingTrajectory(
      vehicle_state, current_timestamp, planning_cycle_time, prev_trajectory,
      stitching_trajectory, replan_reason, injector);
  if (!is_need_reinit) {
    return false;
  }

  const double veh_rel_time =
      current_timestamp - prev_trajectory->header_time();

  size_t time_matched_index =
      prev_trajectory->QueryLowerBoundPoint(veh_rel_time);

  auto time_matched_point = prev_trajectory->TrajectoryPointAt(
      static_cast<uint32_t>(time_matched_index));

  size_t position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
      {vehicle_state.x(), vehicle_state.y()}, kEpsilon);

  auto position_matched_point = prev_trajectory->TrajectoryPointAt(
      static_cast<uint32_t>(position_matched_index));

  if (ReplanForDiffTooLarge(replan_by_offset, planning_cycle_time,
                            vehicle_state, &time_matched_point,
                            position_matched_point, stitching_trajectory,
                            replan_reason)) {
    return false;
  }

  double forward_rel_time = veh_rel_time + planning_cycle_time;

  size_t forward_time_index =
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

  size_t min_begin_time_index = 0;
  if (injector->IsPoorStatusOfTaskFailure()) {
    min_begin_time_index =
        prev_trajectory->QueryLowerBoundPoint(kMinBeginRelTime);
  }

  ADEBUG << "Position matched index:\t" << position_matched_index;
  ADEBUG << "Time matched index:\t" << time_matched_index;
  ADEBUG << "prev_trajectory size:\t" << prev_trajectory->size();
  ADEBUG << "min_begin_time_index:\t" << min_begin_time_index;

  auto matched_index = std::min(time_matched_index, position_matched_index);

  stitching_trajectory->assign(
      prev_trajectory->begin() +
          std::max(static_cast<int>(min_begin_time_index),
                   static_cast<int>(matched_index - preserved_points_num)),
      prev_trajectory->begin() + forward_time_index + 1);
  ADEBUG << "stitching_trajectory size: " << stitching_trajectory->size();

  const double zero_s = stitching_trajectory->back().path_point().s();
  for (auto& tp : *stitching_trajectory) {
    if (!tp.has_path_point()) {
      *replan_reason = "replan for previous trajectory missed path point";
      ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                       &time_matched_point,
                                       stitching_trajectory);
      return false;
    }
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -
                         current_timestamp);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
  }
  if(injector->NeedReinitStartPoint()){
    // AINFO<<"stitching_trajectory->back().before = "<<stitching_trajectory->back().a();
    // AINFO<<"set acc for vehicle = "<<injector->vehicle_state()->linear_acceleration();
    stitching_trajectory->back().set_a(injector->vehicle_state()->linear_acceleration());
  }

  return true;
}

bool TrajectoryStitcher::ReplanForDiffTooLarge(
    const bool replan_by_offset, const double planning_cycle_time,
    const common::VehicleState& vehicle_state,
    common::TrajectoryPoint* const time_matched_point,
    const common::TrajectoryPoint& position_matched_point,
    std::vector<common::TrajectoryPoint>* const stitching_trajectory,
    std::string* const replan_reason) {
  std::pair<double, double> frenet_sd;
  ComputePositionProjection(vehicle_state.x(), vehicle_state.y(),
                            position_matched_point, &frenet_sd);

  if (replan_by_offset) {
    auto lon_diff = time_matched_point->path_point().s() - frenet_sd.first;
    auto lat_diff = frenet_sd.second;

    ADEBUG << "Control lateral diff: " << lat_diff
           << ", longitudinal diff: " << lon_diff;

    if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold) {
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lat_diff = ",
          lat_diff);
      AERROR << msg;
      *replan_reason = msg;
      ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                       time_matched_point,
                                       stitching_trajectory);
      return true;
    }

    if (std::fabs(lon_diff) > FLAGS_replan_longitudinal_distance_threshold) {
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lon_diff = ",
          lon_diff);
      AERROR << msg;
      *replan_reason = msg;
      ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                       time_matched_point,
                                       stitching_trajectory);
      return true;
    }

    auto xy_diff = common::math::Vec2d(time_matched_point->path_point().x(),
                                       time_matched_point->path_point().y())
                       .DistanceTo(common::math::Vec2d(
                           position_matched_point.path_point().x(),
                           position_matched_point.path_point().y()));
    ADEBUG << "cartesian xy_diff: " << xy_diff;

    if (std::fabs(xy_diff) > FLAGS_replan_longitudinal_distance_threshold) {
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. xy_diff = ",
          xy_diff);
      AERROR << msg;
      *replan_reason = msg;
      ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                       time_matched_point,
                                       stitching_trajectory);
      return true;
    }
  } else {
    ADEBUG << "replan according to certain amount of lat and lon offset is "
              "disabled";
  }
  return false;
}

bool TrajectoryStitcher::IsNeedReinitOpenSpaceStitchingTrajectory(
    const common::VehicleState& vehicle_state, const double current_timestamp,
    const double planning_cycle_time,
    const PublishableTrajectory* prev_trajectory,
    std::vector<common::TrajectoryPoint>* const stitching_trajectory,
    std::string* const replan_reason) {
  if (vehicle_state.waiting_steering_flag()) {
    *replan_reason = "teb turn and stop.";
    ADEBUG << "teb turn and stop.";
    ComputeReinitOpenSpaceStitchingTrajectory(
        planning_cycle_time, vehicle_state, nullptr, stitching_trajectory);
    return false;
  }

  if (!FLAGS_enable_trajectory_stitcher) {
    *replan_reason = "stitch is disabled by gflag.";
    ComputeReinitOpenSpaceStitchingTrajectory(
        planning_cycle_time, vehicle_state, nullptr, stitching_trajectory);
    return false;
  }

  if (!prev_trajectory) {
    *replan_reason = "replan for no previous trajectory.";
    ComputeReinitOpenSpaceStitchingTrajectory(
        planning_cycle_time, vehicle_state, nullptr, stitching_trajectory);
    return false;
  }

  if (vehicle_state.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    *replan_reason = "replan for manual mode.";
    ComputeReinitOpenSpaceStitchingTrajectory(
        planning_cycle_time, vehicle_state, nullptr, stitching_trajectory);
    return false;
  }

  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  if (prev_trajectory_size == 0) {
    ADEBUG << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use "
              "origin car status instead.";
    *replan_reason = "replan for empty previous trajectory.";
    ComputeReinitOpenSpaceStitchingTrajectory(
        planning_cycle_time, vehicle_state, nullptr, stitching_trajectory);
    return false;
  }

  const double veh_rel_time =
      current_timestamp - prev_trajectory->header_time();

  size_t time_matched_index =
      prev_trajectory->QueryLowerBoundPoint(veh_rel_time);

  if (time_matched_index == 0 &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time() &&
      FLAGS_enable_replan_for_smaller_start_point) {
    AWARN << "current time smaller than the previous trajectory's first time";
    *replan_reason =
        "replan for current time smaller than the previous trajectory's first "
        "time.";
    ComputeReinitOpenSpaceStitchingTrajectory(
        planning_cycle_time, vehicle_state, nullptr, stitching_trajectory);
    return false;
  }

  if (time_matched_index + 1 >= prev_trajectory_size) {
    AWARN << "current time beyond the previous trajectory's last time"
          << "time_matched_index =" << time_matched_index
          << "prev_trajectory_size " << prev_trajectory_size
          << "veh_rel_time= " << veh_rel_time;
    *replan_reason =
        "replan for current time beyond the previous trajectory's last time";
    ComputeReinitOpenSpaceStitchingTrajectory(
        planning_cycle_time, vehicle_state, nullptr, stitching_trajectory);
    return false;
  }

  auto time_matched_point = prev_trajectory->TrajectoryPointAt(
      static_cast<uint32_t>(time_matched_index));

  if (!time_matched_point.has_path_point()) {
    *replan_reason = "replan for previous trajectory missed path point";
    ComputeReinitOpenSpaceStitchingTrajectory(
        planning_cycle_time, vehicle_state, nullptr, stitching_trajectory);
    return false;
  }
  return true;
}

/* Planning from current vehicle state if:
   1. the auto-driving mode is off
   (or) 2. we don't have the trajectory from last planning cycle
   (or) 3. the position deviation from actual and target is too high
*/
void TrajectoryStitcher::ComputeOpenSpaceStitchingTrajectory(
    const common::VehicleState& vehicle_state, const double current_timestamp,
    const double planning_cycle_time, const size_t preserved_points_num,
    const bool replan_by_offset, const PublishableTrajectory* prev_trajectory,
    std::vector<common::TrajectoryPoint>* const stitching_trajectory,
    std::string* const replan_reason, bool is_poor_status) {
  // prev_trajectory may be null.
  // CHECK_NOTNULL(prev_trajectory);
  CHECK_NOTNULL(stitching_trajectory);
  CHECK_NOTNULL(replan_reason);

  bool is_need_reinit = IsNeedReinitOpenSpaceStitchingTrajectory(
      vehicle_state, current_timestamp, planning_cycle_time, prev_trajectory,
      stitching_trajectory, replan_reason);
  if (!is_need_reinit) {
    return;
  }

  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  const double veh_rel_time =
      current_timestamp - prev_trajectory->header_time();

  size_t time_matched_index =
      prev_trajectory->QueryLowerBoundPoint(veh_rel_time);

  auto time_matched_point = prev_trajectory->TrajectoryPointAt(
      static_cast<uint32_t>(time_matched_index));

  size_t position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
      {vehicle_state.x(), vehicle_state.y()}, kEpsilon);

  std::pair<double, double> frenet_sd;
  ComputePositionProjection(vehicle_state.x(), vehicle_state.y(),
                            prev_trajectory->TrajectoryPointAt(
                                static_cast<uint32_t>(position_matched_index)),
                            &frenet_sd);

  if (replan_by_offset) {
    auto lon_diff = time_matched_point.path_point().s() - frenet_sd.first;
    auto lat_diff = frenet_sd.second;

    ADEBUG << "Control lateral diff: " << lat_diff
           << ", longitudinal diff: " << lon_diff;

    if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold) {
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lat_diff = ",
          lat_diff);
      AERROR << msg;
      *replan_reason = msg;
      ComputeReinitOpenSpaceStitchingTrajectory(
          planning_cycle_time, vehicle_state, &time_matched_point,
          stitching_trajectory);
      return;
    }

    if (std::fabs(lon_diff) > FLAGS_replan_longitudinal_distance_threshold) {
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lon_diff = ",
          lon_diff);
      AERROR << msg;
      *replan_reason = msg;
      ComputeReinitOpenSpaceStitchingTrajectory(
          planning_cycle_time, vehicle_state, &time_matched_point,
          stitching_trajectory);
      return;
    }
  } else {
    ADEBUG << "replan according to certain amount of lat and lon offset is "
              "disabled";
  }

  double forward_rel_time = veh_rel_time + planning_cycle_time;
  size_t forward_time_index =
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

  size_t min_begin_time_index = 0;
  if (is_poor_status) {
    min_begin_time_index =
        prev_trajectory->QueryLowerBoundPoint(kMinBeginRelTime);
  }

  // ADEBUG << "Position matched index:\t" << position_matched_index;
  // ADEBUG << "Time matched index:\t" << time_matched_index;
  // ADEBUG << "prev_trajectory size:\t" << prev_trajectory->size();
  // ADEBUG << "min_begin_time_index:\t" << min_begin_time_index;
  auto matched_index = std::min(time_matched_index, position_matched_index);
  // ADEBUG << "matched_index:\t" << matched_index;
  // ADEBUG << "forward_time_index:\t" << forward_time_index;
  forward_time_index = std::max(forward_time_index, position_matched_index);
  forward_time_index = std::min(forward_time_index, prev_trajectory_size - 1);
  // ADEBUG << "after forward_time_index:\t" << forward_time_index;
  // ADEBUG << "veh_rel_time:\t" << veh_rel_time;
  // ADEBUG << "forward_rel_time:\t" << forward_rel_time;
  // ADEBUG << "prev_trajectory->header_time():\t"
  //        << prev_trajectory->header_time();
  // ADEBUG << "current_timestamp:\t" << current_timestamp;
  // ADEBUG << "preserved_points_num:\t" << preserved_points_num;
  // ADEBUG << "max:\t"
  //        << std::max(static_cast<int>(min_begin_time_index),
  //                    static_cast<int>(matched_index - preserved_points_num));

  stitching_trajectory->assign(
      prev_trajectory->begin() +
          std::max(static_cast<int>(min_begin_time_index),
                   static_cast<int>(matched_index - preserved_points_num)),
      prev_trajectory->begin() + forward_time_index + 1);
  ADEBUG << "stitching_trajectory size: " << stitching_trajectory->size();

  const double zero_s = stitching_trajectory->back().path_point().s();
  for (auto& tp : *stitching_trajectory) {
    if (!tp.has_path_point()) {
      *replan_reason = "replan for previous trajectory missed path point";
      ComputeReinitOpenSpaceStitchingTrajectory(
          planning_cycle_time, vehicle_state, &time_matched_point,
          stitching_trajectory);
      return;
    }
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -
                         current_timestamp);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
  }

  return;
}

/* Planning from current vehicle state if:
   1. the auto-driving mode is off
   (or) 2. we don't have the trajectory from last planning cycle
   (or) 3. the position deviation from actual and target is too high
*/
void TrajectoryStitcher::ComputeOpenspaceTrajectory(
    const common::VehicleState& vehicle_state, const double current_timestamp,
    const double planning_cycle_time, const size_t preserved_points_num,
    const bool replan_by_offset, const PublishableTrajectory* prev_trajectory,
    std::vector<common::TrajectoryPoint>* const stitching_trajectory,
    std::string* const replan_reason) {
  // prev_trajectory may be null.
  // CHECK_NOTNULL(prev_trajectory);
  CHECK_NOTNULL(stitching_trajectory);
  CHECK_NOTNULL(replan_reason);

  if (!FLAGS_enable_trajectory_stitcher) {
    *replan_reason = "stitch is disabled by gflag.";
    ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                     nullptr, stitching_trajectory);
    return;
  }
  if (!prev_trajectory) {
    *replan_reason = "replan for no previous trajectory.";
    ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                     nullptr, stitching_trajectory);
    return;
  }

  if (vehicle_state.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    *replan_reason = "replan for manual mode.";
    ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                     nullptr, stitching_trajectory);
    return;
  }

  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  if (prev_trajectory_size == 0) {
    ADEBUG << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use "
              "origin car status instead.";
    *replan_reason = "replan for empty previous trajectory.";
    ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                     nullptr, stitching_trajectory);
    return;
  }

  const double veh_rel_time =
      current_timestamp - prev_trajectory->header_time();

  size_t time_matched_index =
      prev_trajectory->QueryLowerBoundPoint(veh_rel_time);

  if (time_matched_index == 0 &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time()) {
    AWARN << "current time smaller than the previous trajectory's first time";
    *replan_reason =
        "replan for current time smaller than the previous trajectory's first "
        "time.";
    ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                     nullptr, stitching_trajectory);
    return;
  }

  if (time_matched_index + 1 >= prev_trajectory_size) {
    AWARN << "current time beyond the previous trajectory's last time"
          << "time_matched_index =" << time_matched_index
          << "prev_trajectory_size " << prev_trajectory_size
          << "veh_rel_time= " << veh_rel_time;
  }
  ADEBUG << "veh_rel_time= " << veh_rel_time;

  size_t position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
      {vehicle_state.x(), vehicle_state.y()}, kEpsilon);

  std::pair<double, double> frenet_sd;
  ComputePositionProjection(vehicle_state.x(), vehicle_state.y(),
                            prev_trajectory->TrajectoryPointAt(
                                static_cast<uint32_t>(position_matched_index)),
                            &frenet_sd);

  double forward_rel_time = veh_rel_time + planning_cycle_time;

  size_t forward_time_index =
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

  ADEBUG << "Position matched index:\t" << position_matched_index;
  ADEBUG << "Time matched index:\t" << time_matched_index;

  size_t matched_index = position_matched_index;
  std::vector<common::TrajectoryPoint> test_traj;
  test_traj.assign(
      prev_trajectory->begin() +
          std::max(0, static_cast<int>(matched_index - preserved_points_num)),
      prev_trajectory->end());

  stitching_trajectory->assign(
      prev_trajectory->begin() +
          std::max(0, static_cast<int>(matched_index - preserved_points_num)),
      prev_trajectory->begin() + forward_time_index + 1);
  AINFO << "stitching_trajectory size: " << stitching_trajectory->size()
        << "test_traj size " << test_traj.size();

  const double zero_s = stitching_trajectory->back().path_point().s();
  for (auto& tp : *stitching_trajectory) {
    if (!tp.has_path_point()) {
      *replan_reason = "replan for previous trajectory missed path point";
      ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state,
                                       nullptr, stitching_trajectory);
      return;
    }
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -
                         current_timestamp);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
  }

  return;
}

void TrajectoryStitcher::ComputePositionProjection(
    const double x, const double y,
    const common::TrajectoryPoint& matched_point,
    std::pair<double, double>* const frenet_sd) {
  CHECK_NOTNULL(frenet_sd);

  Vec2d v(x - matched_point.path_point().x(),
          y - matched_point.path_point().y());
  Vec2d n(std::cos(matched_point.path_point().theta()),
          std::sin(matched_point.path_point().theta()));

  frenet_sd->first = v.InnerProd(n) + matched_point.path_point().s();
  frenet_sd->second = v.CrossProd(n);

  return;
}

void TrajectoryStitcher::ComputeFallbackStitchingTrajectory(
    const common::VehicleState& vehicle_state, const double current_timestamp,
    const double planning_cycle_time,
    const PublishableTrajectory* prev_trajectory,
    std::vector<common::TrajectoryPoint>* const stitching_trajectory) {
  if (stitching_trajectory == nullptr) {
    ADEBUG << "TrajectoryStitcher::ComputeStitchingTrajectory: The output "
              "pointer stitching_trajectory is null. ";
    return;
  }

  if (!prev_trajectory) {
    ComputeFallbackReinitStitchingTrajectory(vehicle_state, nullptr,
                                             stitching_trajectory);
    return;
  }

  auto prev_trajectory_size = prev_trajectory->NumOfPoints();

  if (prev_trajectory_size == 0) {
    ADEBUG << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use "
              "origin car status instead.";
    ComputeFallbackReinitStitchingTrajectory(vehicle_state, nullptr,
                                             stitching_trajectory);
    return;
  }

  const double veh_rel_time =
      current_timestamp - prev_trajectory->header_time();

  auto matched_index = prev_trajectory->QueryLowerBoundPoint(veh_rel_time);

  if (matched_index == 0 &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time()) {
    AWARN << "current time smaller than the previous trajectory's first time";
    ComputeFallbackReinitStitchingTrajectory(vehicle_state, nullptr,
                                             stitching_trajectory);
    return;
  }
  if (matched_index + 1 >= prev_trajectory_size) {
    AWARN << "current time beyond the previous trajectory's last time";
    ComputeFallbackReinitStitchingTrajectory(vehicle_state, nullptr,
                                             stitching_trajectory);
    return;
  }

  auto matched_point = prev_trajectory->Evaluate(veh_rel_time);

  if (!matched_point.has_path_point()) {
    ComputeFallbackReinitStitchingTrajectory(vehicle_state, nullptr,
                                             stitching_trajectory);
    return;
  }

  std::pair<double, double> frenet_sd;
  ComputeFallbackPositionProjection(vehicle_state.x(), vehicle_state.y(),
                                    *prev_trajectory, &frenet_sd);

  auto lon_diff = std::fabs(frenet_sd.first);
  auto lat_diff = std::fabs(frenet_sd.second);

  ADEBUG << "Control lateral diff: " << lat_diff
         << ", longitudinal diff: " << lon_diff;

  if (lat_diff > FLAGS_replan_lateral_distance_threshold ||
      lon_diff > FLAGS_replan_longitudinal_distance_threshold) {
    ADEBUG << "the distance between matched point and actual position is too "
              "large. Replan is triggered. lat_diff = "
           << lat_diff << ", lon_diff = " << lon_diff;
    ComputeFallbackReinitStitchingTrajectory(vehicle_state, &matched_point,
                                             stitching_trajectory);
    return;
  }

  double forward_rel_time =
      prev_trajectory->TrajectoryPointAt(matched_index).relative_time() +
      planning_cycle_time;

  std::size_t forward_index =
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

  ADEBUG << "matched_index: " << matched_index;
  stitching_trajectory->assign(
      prev_trajectory->begin() +
          std::max(0, static_cast<int>(matched_index - 1)),
      prev_trajectory->begin() + forward_index + 1);

  const double zero_s = matched_point.path_point().s();

  for (auto& tp : *stitching_trajectory) {
    if (!tp.has_path_point()) {
      ComputeFallbackReinitStitchingTrajectory(vehicle_state, &matched_point,
                                               stitching_trajectory);
      return;
    }
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -
                         current_timestamp);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
  }

  return;
}
void TrajectoryStitcher::ComputeFallbackReinitStitchingTrajectory(
    const common::VehicleState& vehicle_state,
    const common::TrajectoryPoint* matched_point,
    std::vector<common::TrajectoryPoint>* const stitching_trajectory) {
  if (stitching_trajectory == nullptr) {
    ADEBUG << "TrajectoryStitcher::ComputeReinitStitchingTrajectory: The "
              "output pointer stitching_trajectory is null. ";
    return;
  }
  stitching_trajectory->clear();

  TrajectoryPoint init_point;
  auto* init_path_point = init_point.mutable_path_point();
  init_path_point->set_x(vehicle_state.x());
  init_path_point->set_y(vehicle_state.y());
  init_path_point->set_z(vehicle_state.z());
  init_path_point->set_theta(vehicle_state.heading());
  init_path_point->set_kappa(vehicle_state.kappa());
  init_path_point->set_s(0.0);
  init_point.set_v(vehicle_state.linear_velocity());
  init_point.set_a(vehicle_state.linear_acceleration());
  init_point.set_relative_time(0.0);

  /**
   * When the ADC has speed, it does not directly use the ADC's current pose as
   * the starting point for re-planning, but interpolates a starting point based
   * on the ADC's speed for re-planning, so as to improve driving smoothness and
   * safety.
   *
   */
  double adc_v = vehicle_state.linear_velocity();
  // if (matched_point != nullptr && adc_v > FLAGS_max_stop_speed &&
  //     matched_point->v() > FLAGS_max_stop_speed) {
  if (matched_point != nullptr && adc_v > FLAGS_max_stop_speed &&
      matched_point->v() > FLAGS_max_stop_speed) {
    auto matched_path_point = matched_point->path_point();
    double adc_x = vehicle_state.x();
    double adc_y = vehicle_state.y();
    double matched_x = matched_path_point.x();
    double matched_y = matched_path_point.y();
    double diff_x = matched_x - adc_x;
    double diff_y = matched_y - adc_y;
    double distance = std::sqrt(diff_x * diff_x + diff_y * diff_y);
    matched_path_point.set_s(distance);

    // double interpolated_s = kReinitRatio * distance;
    double interpolated_s = 0.7 * distance;
    double matched_v = matched_point->v();
    // In this scope, matched_v is always greater than 0.
    if (common::util::IsFloatEqual(matched_v, 0.0)) {
      AERROR << "the matched_v is zero!";
      matched_v = FLAGS_numerical_epsilon;
    }
    interpolated_s *= adc_v / matched_v;
    interpolated_s = std::min(interpolated_s, distance);

    auto interpolated_path_point = InterpolateUsingLinearApproximation(
        *init_path_point, matched_path_point, interpolated_s);
    init_path_point->CopyFrom(interpolated_path_point);
    // The s value of the replanning starting point is 0
    init_path_point->set_s(0.0);
  }

  stitching_trajectory->emplace_back(init_point);
  return;
}

void TrajectoryStitcher::ComputeFallbackPositionProjection(
    const double x, const double y,
    const PublishableTrajectory& prev_trajectory,
    std::pair<double, double>* const frenet_sd) {
  if (frenet_sd == nullptr) {
    ADEBUG << "TrajectoryStitcher::ComputePositionProjection: The output "
              "pointer frenet_sd is null. ";
    return;
  }

  auto index = prev_trajectory.QueryNearestPoint({x, y});
  auto p = prev_trajectory.TrajectoryPointAt(index);

  common::math::Vec2d v(x - p.path_point().x(), y - p.path_point().y());
  common::math::Vec2d n(std::cos(p.path_point().theta()),
                        std::sin(p.path_point().theta()));

  frenet_sd->first = v.InnerProd(n) + p.path_point().s();
  frenet_sd->second = v.CrossProd(n);
  return;
}
}  // namespace planning
}  // namespace century
