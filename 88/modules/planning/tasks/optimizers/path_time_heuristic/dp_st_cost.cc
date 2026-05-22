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
#include "modules/planning/tasks/optimizers/path_time_heuristic/dp_st_cost.h"

#include <algorithm>
#include <limits>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/tasks/deciders/utils/path_decider_obstacle_utils.h"
#include "modules/planning/tasks/utils/st_gap_estimator.h"

namespace century {
namespace planning {
namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kMin = 0.0001;
constexpr double kCruiseTime = 3.0;
constexpr double kTimes = 1.1;
constexpr double kMaxAcc = 6.0;
constexpr double kComfortableAcc = 3.0;
constexpr double kMaxMixedFlowSpeed = 4.0;
constexpr double kEpsilon = 0.1;
constexpr double kEpsilon1 = 1.0e-10;
constexpr size_t kAccelShift = 100UL;
constexpr size_t kJertShift = 200UL;
constexpr double kOverTakingGap = 20.0;
constexpr double kMinOvertakeAcc = -0.1;
constexpr double kMinDecel = -6.0;
}  // namespace

DpStCost::DpStCost(const DpStSpeedOptimizerConfig& config, const double total_t,
                   const double total_s,
                   const std::vector<const Obstacle*>& obstacles,
                   const STDrivableBoundary& st_drivable_boundary,
                   const common::TrajectoryPoint& init_point)
    : config_(config),
      obstacles_(obstacles),
      st_drivable_boundary_(st_drivable_boundary),
      init_point_(init_point),
      unit_t_(config.unit_t()),
      total_s_(total_s) {
  CHECK_GT(unit_t_, 0.0);
  int index = 0;
  for (const auto& obstacle : obstacles) {
    boundary_map_[obstacle->path_st_boundary().id()] = index++;
  }

  AddToKeepClearRange(obstacles);

  const auto dimension_t =
      static_cast<uint32_t>(std::ceil(total_t / static_cast<double>(unit_t_))) +
      1;
  boundary_cost_.resize(obstacles_.size());
  for (auto& vec : boundary_cost_) {
    vec.resize(dimension_t, std::make_pair(-1.0, -1.0));
  }
  accel_cost_.fill(-1.0);
  jerk_cost_.fill(-1.0);
  half_adc_decel_ = std::fabs(common::VehicleConfigHelper::Instance()
                                  ->GetConfig()
                                  .vehicle_param()
                                  .max_deceleration() *
                              0.5);
}

void DpStCost::AddToKeepClearRange(
    const std::vector<const Obstacle*>& obstacles) {
  for (const auto& obstacle : obstacles) {
    if (obstacle->path_st_boundary().IsEmpty()) {
      continue;
    }
    if (obstacle->path_st_boundary().boundary_type() !=
        STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    double start_s = obstacle->path_st_boundary().min_s();
    double end_s = obstacle->path_st_boundary().max_s();
    keep_clear_range_.emplace_back(start_s, end_s);
  }
  SortAndMergeRange(&keep_clear_range_);
}

void DpStCost::SortAndMergeRange(
    std::vector<std::pair<double, double>>* keep_clear_range) {
  if (!keep_clear_range || keep_clear_range->empty()) {
    return;
  }
  std::sort(keep_clear_range->begin(), keep_clear_range->end());
  size_t i = 0;
  size_t j = i + 1;
  while (j < keep_clear_range->size()) {
    if ((*keep_clear_range)[i].second < (*keep_clear_range)[j].first) {
      ++i;
      ++j;
    } else {
      (*keep_clear_range)[i].second = std::max((*keep_clear_range)[i].second,
                                               (*keep_clear_range)[j].second);
      ++j;
    }
  }
  keep_clear_range->resize(i + 1);
}

bool DpStCost::InKeepClearRange(double s) const {
  for (const auto& p : keep_clear_range_) {
    if (p.first <= s && p.second >= s) {
      return true;
    }
  }
  return false;
}

double DpStCost::GetObstacleCost(const bool is_need_slow_breaking,
                                 const StGraphPoint& st_graph_point) {
  const double s = st_graph_point.point().s();
  const double t = st_graph_point.point().t();
  is_need_slow_breaking_ = is_need_slow_breaking;
  double cost = 0.0;
  if (FLAGS_use_st_drivable_boundary) {
    // TODO(Jiancheng): move to configs
    static constexpr double boundary_resolution = 0.1;
    int index = static_cast<int>(t / boundary_resolution);
    const double lower_bound =
        st_drivable_boundary_.st_boundary(index).s_lower();
    const double upper_bound =
        st_drivable_boundary_.st_boundary(index).s_upper();

    if (s > upper_bound || s < lower_bound) {
      return kInf;
    }
  }
  cost = GetObstacleBoundaryCost(st_graph_point, s, t, cost);
  return cost * unit_t_;
}

double DpStCost::GetSpatialPotentialCost(const StGraphPoint& point) {
  return (total_s_ - point.point().s()) * config_.spatial_potential_penalty();
}

double DpStCost::GetReferenceCost(const STPoint& point,
                                  const STPoint& reference_point) const {
  return config_.reference_weight() * (point.s() - reference_point.s()) *
         (point.s() - reference_point.s()) * unit_t_;
}

double DpStCost::GetSpeedCost(const STPoint& first, const STPoint& second,
                              const double speed_limit,
                              const double cruise_speed) const {
  double cost = 0.0;
  const double speed = (second.s() - first.s()) / unit_t_;
  if (speed < 0) {
    return kInf;
  }

  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();
  if (speed < max_adc_stop_speed && InKeepClearRange(second.s())) {
    // first.s in range
    cost += config_.keep_clear_low_speed_penalty() * unit_t_ *
            config_.default_speed_cost();
  }
  double det_speed = 0.0;
  if (speed_limit < kMin) {
    det_speed = speed - speed_limit;
  } else {
    det_speed = (speed - speed_limit) / speed_limit;
  }

  if (det_speed > 0) {
    cost += config_.exceed_speed_penalty() * config_.default_speed_cost() *
            (det_speed * det_speed) * unit_t_;
  } else if (det_speed < 0) {
    cost += config_.low_speed_penalty() * config_.default_speed_cost() *
            -det_speed * unit_t_;
  }

  if (FLAGS_enable_dp_reference_speed) {
    double diff_speed = 0.0;
    if (cruise_speed < kMin) {
      diff_speed = speed - cruise_speed;
    } else {
      diff_speed = (speed - cruise_speed) / cruise_speed;
    }
    cost += config_.reference_speed_penalty() * config_.default_speed_cost() *
            fabs(diff_speed) * unit_t_;
  }

  return cost;
}

double DpStCost::GetAccelCost(const double accel) {
  double cost = 0.0;
  const size_t accel_key =
      static_cast<size_t>(accel / kEpsilon + 0.5 + kAccelShift);
  if (accel_key >= accel_cost_.size()) {
    return kInf;
  }

  if (accel_cost_[accel_key] < 0.0) {
    const double accel_sq = accel * accel;
    double max_acc = config_.max_acceleration();
    double max_dec = config_.max_deceleration();
    double accel_penalty = config_.accel_penalty();
    double decel_penalty = config_.decel_penalty();

    if (accel > 0.0) {
      cost = accel_penalty * accel_sq;
    } else {
      cost = decel_penalty * accel_sq;
    }
    cost += accel_sq * decel_penalty * decel_penalty /
                (1 + std::exp(1.0 * (accel - max_dec))) +
            accel_sq * accel_penalty * accel_penalty /
                (1 + std::exp(-1.0 * (accel - max_acc)));
    accel_cost_[accel_key] = cost;
  } else {
    cost = accel_cost_[accel_key];
  }
  return cost * unit_t_;
}

double DpStCost::GetAccelCostByThreePoints(const STPoint& first,
                                           const STPoint& second,
                                           const STPoint& third) {
  double accel = (first.s() + third.s() - 2 * second.s()) / (unit_t_ * unit_t_);
  return GetAccelCost(accel);
}

double DpStCost::GetAccelCostByTwoPoints(const double pre_speed,
                                         const STPoint& pre_point,
                                         const STPoint& curr_point) {
  double current_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  double accel = (current_speed - pre_speed) / unit_t_;
  return GetAccelCost(accel);
}

double DpStCost::JerkCost(const double jerk) {
  double cost = 0.0;
  const size_t jerk_key =
      static_cast<size_t>(jerk / kEpsilon + 0.5 + kJertShift);
  if (jerk_key >= jerk_cost_.size()) {
    return kInf;
  }

  if (jerk_cost_[jerk_key] < 0.0) {
    double jerk_sq = jerk * jerk;
    if (jerk > 0) {
      cost = config_.positive_jerk_coeff() * jerk_sq * unit_t_;
    } else {
      cost = config_.negative_jerk_coeff() * jerk_sq * unit_t_;
    }
    jerk_cost_[jerk_key] = cost;
  } else {
    cost = jerk_cost_[jerk_key];
  }

  // TODO(All): normalize to unit_t_
  return cost;
}

double DpStCost::GetJerkCostByFourPoints(const STPoint& first,
                                         const STPoint& second,
                                         const STPoint& third,
                                         const STPoint& fourth) {
  double jerk = (fourth.s() - 3 * third.s() + 3 * second.s() - first.s()) /
                (unit_t_ * unit_t_ * unit_t_);
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByTwoPoints(const double pre_speed,
                                        const double pre_acc,
                                        const STPoint& pre_point,
                                        const STPoint& curr_point) {
  const double curr_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  const double curr_accel = (curr_speed - pre_speed) / unit_t_;
  const double jerk = (curr_accel - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByThreePoints(const double first_speed,
                                          const STPoint& first,
                                          const STPoint& second,
                                          const STPoint& third) {
  const double pre_speed = (second.s() - first.s()) / unit_t_;
  const double pre_acc = (pre_speed - first_speed) / unit_t_;
  const double curr_speed = (third.s() - second.s()) / unit_t_;
  const double curr_acc = (curr_speed - pre_speed) / unit_t_;
  const double jerk = (curr_acc - pre_acc) / unit_t_;
  return JerkCost(jerk);
}
double DpStCost::GetFollowDistance(
    const century::planning::STBoundary& boundary) const {
  double follow_distance_s = config_.safe_distance();

  bool is_same_direction =
      boundary.bottom_left_point().s() <= boundary.bottom_right_point().s();
  ADEBUG << "boundary.bottom_left_point().s() = "
         << boundary.bottom_left_point().s()
         << "        boundary.bottom_right_point().s()="
         << boundary.bottom_right_point().s();

  double follow_min_distance = FLAGS_min_stop_distance_obstacle;
  if (is_same_direction) {
    // too close to follow.
    double diff_v = init_point_.v() - boundary.obs_v();
    follow_min_distance = diff_v * diff_v * 0.5 / half_adc_decel_ +
                          FLAGS_min_stop_distance_obstacle +
                          config_.sparse_unit_s();

    // adjust distance based on speed changes
    double lon_buffer = InterpolationLookUp(
        init_point_.v(), config_.min_adc_speed(),
        FLAGS_planning_upper_speed_limit, 0.0, config_.safe_distance_base());

    follow_min_distance = follow_min_distance + lon_buffer;
    ADEBUG << "same direction_follow_min_distance = " << follow_min_distance;
  } else {
    follow_min_distance =
        init_point_.v() * init_point_.v() * 0.5 / half_adc_decel_ +
        FLAGS_min_stop_distance_obstacle;
    ADEBUG << "no_same_direction_follow_min_distance = " << follow_min_distance;
  }

  follow_distance_s =
      std::min(config_.safe_distance(),
               std::max(follow_min_distance, config_.safe_distance_base()));
  double dis_ratio = 1.0;
  double speed_ratio = boundary.obs_v() / (init_point_.v() + kEpsilon1);
  if (is_same_direction && speed_ratio > config_.faster_speed_ratio()) {
    dis_ratio =
        InterpolationLookUp(speed_ratio, config_.faster_speed_ratio(),
                            config_.very_faster_speed_ratio(),
                            config_.safe_distance_ratio_for_faster_obs(),
                            config_.safe_distance_ratio_for_very_faster_obs());
  }
  follow_distance_s = InterpolationLookUp(
      init_point_.v(), config_.min_speed_for_safe_distance(),
      FLAGS_planning_upper_speed_limit, config_.safe_distance_base(),
      config_.safe_distance() * dis_ratio);
  // 1.cutin obstacle or yield obstacle or follow obstacle;
  // 2.obs_v > adc_v * 1.1
  if (FLAGS_enable_use_radical_decision &&
      boundary.obs_v() > kTimes * init_point_.v() &&
      init_point_.v() < kMaxMixedFlowSpeed) {
    if (is_same_direction) {
      follow_distance_s =
          init_point_.v() * init_point_.v() * 0.5 / half_adc_decel_ -
          boundary.obs_v() * boundary.obs_v() * 0.5 / kMaxAcc;
    } else {
      follow_distance_s =
          init_point_.v() * init_point_.v() * 0.5 / half_adc_decel_ +
          boundary.obs_v() * boundary.obs_v() * 0.5 / kComfortableAcc;
    }
  }

  return follow_distance_s;
}

double DpStCost::GetFollowDistanceForHighSpeed(
    const century::planning::STBoundary& boundary) const {
  double follow_distance_s = config_.safe_distance();

  bool is_same_direction =
      boundary.bottom_left_point().s() <= boundary.bottom_right_point().s();
  ADEBUG << "boundary.bottom_left_point().s() = "
         << boundary.bottom_left_point().s()
         << "        boundary.bottom_right_point().s()="
         << boundary.bottom_right_point().s();

  double follow_min_distance = FLAGS_min_stop_distance_obstacle;
  if (is_same_direction) {
    // too close to follow.
    double diff_v = init_point_.v() - boundary.obs_v();
    follow_min_distance =
        diff_v * diff_v * 0.5 / half_adc_decel_ + config_.sparse_unit_s();

    double lon_buffer = InterpolationLookUp(
        boundary.obs_v(), FLAGS_play_street_speed_limit,
        FLAGS_planning_upper_speed_limit, config_.safe_distance_min(),
        config_.safe_distance_max());

    follow_min_distance = follow_min_distance + lon_buffer;
    ADEBUG << "same direction_follow_min_distance = " << follow_min_distance;
  } else {
    follow_min_distance =
        init_point_.v() * init_point_.v() * 0.5 / half_adc_decel_ +
        FLAGS_min_stop_distance_obstacle;
    ADEBUG << "no_same_direction_follow_min_distance = " << follow_min_distance;
  }

  follow_distance_s =
      std::min(config_.safe_distance_max(),
               std::max(follow_min_distance, config_.safe_distance_min()));
  // 1.cutin obstacle or yield obstacle or follow obstacle;
  // 2.obs_v > adc_v * 1.1
  if (FLAGS_enable_use_radical_decision &&
      boundary.obs_v() > kTimes * init_point_.v() &&
      init_point_.v() < kMaxMixedFlowSpeed) {
    if (is_same_direction) {
      follow_distance_s =
          init_point_.v() * init_point_.v() * 0.5 / half_adc_decel_ -
          boundary.obs_v() * boundary.obs_v() * 0.5 / kMaxAcc;
    } else {
      follow_distance_s =
          init_point_.v() * init_point_.v() * 0.5 / half_adc_decel_ +
          boundary.obs_v() * boundary.obs_v() * 0.5 / kComfortableAcc;
    }
  }

  return follow_distance_s;
}

bool DpStCost::CheckIsSlowBreakingObstacle(const Obstacle& obstacle) {
  if (obstacle.IsSlowBreakingObstacle() && is_need_slow_breaking_) {
    return true;
  }
  return false;
}

double DpStCost::GetObstacleBoundaryCost(const StGraphPoint& st_graph_point,
                                         const double& s, const double& t,
                                         const double& cost) {
  double obs_cost = cost;
  for (const auto* obstacle : obstacles_) {
    // Not applying obstacle approaching cost to virtual obstacle like created
    // stop fences
    if (obstacle->IsVirtual()) {
      continue;
    }
    if (CheckIsSlowBreakingObstacle(*obstacle)) {
      continue;
    }
    // Stop obstacles are assumed to have a safety margin when mapping them out,
    // so repelling force in dp st is not needed as it is designed to have adc
    // stop right at the stop distance we design in prior mapping process
    if (obstacle->LongitudinalDecision().has_stop()) {
      continue;
    }

    auto boundary = obstacle->path_st_boundary();

    if (boundary.min_s() > FLAGS_speed_lon_decision_horizon) {
      continue;
    }
    if (t < boundary.min_t() || t > boundary.max_t()) {
      continue;
    }
    if (boundary.IsPointInBoundary(st_graph_point.point())) {
      return kInf;
    }
    double s_upper = 0.0;
    double s_lower = 0.0;

    int boundary_index = boundary_map_[boundary.id()];
    if (boundary_cost_[boundary_index][st_graph_point.index_t()].first < 0.0) {
      boundary.GetBoundarySRange(t, &s_upper, &s_lower);
      boundary_cost_[boundary_index][st_graph_point.index_t()] =
          std::make_pair(s_upper, s_lower);
    } else {
      s_upper = boundary_cost_[boundary_index][st_graph_point.index_t()].first;
      s_lower = boundary_cost_[boundary_index][st_graph_point.index_t()].second;
    }
    if (s < s_lower) {
      // TODO(ZhongHao): change safe_distance according to ADC speed and
      // Obstacle speed.

      double follow_distance_s = GetFollowDistance(boundary);

      if (FLAGS_enable_high_speed) {
        follow_distance_s = GetFollowDistanceForHighSpeed(boundary);
      }

      if (s + follow_distance_s < s_lower) {
        continue;
      } else {
        auto s_diff = follow_distance_s - s_lower + s;
        obs_cost += config_.obstacle_weight() *
                    config_.default_obstacle_cost() * s_diff * s_diff;
      }
    } else if (s > s_upper) {
      // TODO(zongxingguo): remove because we extend upper for boundary;
      if (init_point_.a() < kMinOvertakeAcc) {
        if (FLAGS_enable_high_speed) {
          const double overtake_distance_s =
              StGapEstimator::EstimateProperOvertakingGap(boundary.obs_v(),
                                                          init_point_.v());
          AINFO << "overtake_distance_s = " << overtake_distance_s;
          if (s > s_upper + overtake_distance_s + kOverTakingGap) {
            continue;
          } else if (s > s_upper + overtake_distance_s &&
                     s < s_upper + overtake_distance_s + kOverTakingGap) {
            auto s_diff = overtake_distance_s + s_upper - s;
            obs_cost += config_.obstacle_overtake_weight() *
                        config_.obstacle_weight() *
                        config_.default_obstacle_cost() * s_diff * s_diff;
          } else {
            obs_cost += kInf;
          }
        } else {
          const double overtake_distance_s =
              StGapEstimator::EstimateSafeOvertakingGap();
          if (s >
              s_upper + overtake_distance_s) {  // or calculated from velocity
            continue;
          } else {
            auto s_diff = overtake_distance_s + s_upper - s;
            obs_cost += config_.obstacle_overtake_weight() *
                        config_.obstacle_weight() *
                        config_.default_obstacle_cost() * s_diff * s_diff;
          }
        }
      }
    }
  }
  return obs_cost;
}

}  // namespace planning
}  // namespace century
