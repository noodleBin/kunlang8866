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
 * @file kinematic_deceleration_speed_optimizer.cc
 **/

#include "modules/planning/common/kinematic_speed_optimizer/kinematic_deceleration_speed_optimizer.h"

#include <algorithm>
#include <limits>

#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_profile_generator.h"

namespace century {
namespace planning {

namespace {
constexpr double kEpsilon = 1e-10;
constexpr double kAccelBufferUpperLimit = 0.0;
constexpr double kAccelBufferLowerLimit = -0.5;
}  // namespace

void KinematicDecelerationSpeedOptimizer::Init(
    const double v0, const double a0,
    const RestrictInitAccelLimit restrict_init_accel,
    const double specified_value) {
  v0_ = v0 < kEpsilon ? kEpsilon : v0;
  a0_ = a0;
  switch (restrict_init_accel) {
    case RESTRICT_UPPER_LIMIT:
      a0_ = std::min(a0_, kAccelBufferUpperLimit);
      break;
    case RESTRICT_LOWER_LIMIT:
      a0_ = std::max(a0_, kAccelBufferLowerLimit);
      break;
    case RESTRICT_LOWER_AND_UPPER_LIMIT:
      a0_ = std::max(std::min(a0_, kAccelBufferUpperLimit),
                     kAccelBufferLowerLimit);
      break;
    case RESTRICT_TO_SPECIFIED_VALUE:
      a0_ = specified_value;
      break;
    default:
      break;
  }
  ADEBUG << "v0:(" << v0_ << "), a0:(" << a0_ << "), restrict_init_accel:("
         << restrict_init_accel << ")";
}

double KinematicDecelerationSpeedOptimizer::GenerateSpeedDataByConstantJerk(
    const double jerk, SpeedData* const speed_data) {
  ADEBUG << "constant jerk: " << jerk << std::endl;
  double s = 0.0, v = 0.0, a = 0.0;
  speed_data->clear();
  for (size_t i = 0UL; i < num_of_knots_; ++i) {
    double curr_t = i * delta_t_;
    s = v0_ * curr_t + 0.5 * a0_ * curr_t * curr_t +
        jerk * curr_t * curr_t * curr_t / 6.0;
    v = v0_ + a0_ * curr_t + 0.5 * jerk * curr_t * curr_t;
    a = a0_ + jerk * curr_t;
    if (curr_t > FLAGS_fallback_total_time &&
        v > FLAGS_planning_upper_speed_limit) {
      break;
    }
    // Avoid the very last points when already stopped
    if (v < kEpsilon) {
      ADEBUG << "KinematicDecelerationSpeedOptimizer calculate speed is "
                "negative.";
      ADEBUG << "current time:(" << curr_t << "), s(" << s << "), v(" << v
             << "), a(" << a << ")";
      ADEBUG << "[init status] v0:(" << v0_ << "), a0:(" << a0_ << ")";
      break;
    }
    speed_data->AppendSpeedPoint(s, curr_t, v, a, jerk);
  }
  SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);
  return s;
}

double KinematicDecelerationSpeedOptimizer::GenerateSpeedDataByConstantAccel(
    const double accel, SpeedData* const speed_data) {
  ADEBUG << "constant accel: " << accel << std::endl;
  double s = 0.0, v = 0.0, a = 0.0, jerk = 0.0;
  speed_data->clear();
  // first: the acceleration changes from a0_ to the input accel at a constant
  // jerk
  // second: deceleration by constant accelete value.
  double t1 = 0.0;
  double jerk1 = 0.0;
  if (accel < a0_) {
    t1 = (accel - a0_) / FLAGS_min_jerk_for_const_accel_speed_plan;
    jerk1 = FLAGS_min_jerk_for_const_accel_speed_plan;
  } else {
    t1 = (accel - a0_) / FLAGS_max_jerk_for_const_accel_speed_plan;
    jerk1 = FLAGS_max_jerk_for_const_accel_speed_plan;
  }
  const double s1 = v0_ * t1 + 0.5 * a0_ * t1 * t1 + jerk1 * t1 * t1 * t1 / 6.0;
  const double v1 = v0_ + a0_ * t1 + 0.5 * jerk1 * t1 * t1;
  const double a1 = a0_ + jerk1 * t1;
  for (size_t i = 0UL; i < num_of_knots_; ++i) {
    double curr_t = i * delta_t_;
    if (curr_t <= t1) {
      s = v0_ * curr_t + 0.5 * a0_ * curr_t * curr_t +
          jerk1 * curr_t * curr_t * curr_t / 6.0;
      v = v0_ + a0_ * curr_t + 0.5 * jerk1 * curr_t * curr_t;
      a = a0_ + jerk1 * curr_t;
      jerk = jerk1;
    } else {
      double t2 = curr_t - t1;
      s = s1 + v1 * t2 + 0.5 * a1 * t2 * t2;
      v = v1 + a1 * t2;
      a = a1;
      jerk = 0.0;
    }
    if (curr_t > FLAGS_fallback_total_time &&
        v > FLAGS_planning_upper_speed_limit) {
      break;
    }
    // Avoid the very last points when already stopped
    if (v < kEpsilon) {
      ADEBUG << "KinematicDecelerationSpeedOptimizer calculate speed is "
                "negative.";
      ADEBUG << "current time:(" << curr_t << "), s(" << s << "), v(" << v
             << "), a(" << a << ")";
      ADEBUG << "[init status] v0:(" << v0_ << "), a0:(" << a0_ << ")";
      break;
    }
    speed_data->AppendSpeedPoint(s, curr_t, v, a, jerk);
  }
  SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);
  return s;
}

double
KinematicDecelerationSpeedOptimizer::GenerateSpeedDataByConstantAccelAndSpeed(
    const double target_accel, const double target_speed,
    SpeedData* const speed_data) {
  ADEBUG << "constant target accel: " << target_accel
         << "\n\t\tconstant target speed: " << target_speed << std::endl;
  double s = 0.0, v = 0.0, a = 0.0, jerk = 0.0;
  speed_data->clear();
  if (target_speed > v0_ || target_accel > -kEpsilon) {
    // move forward at an average speed.
    for (size_t i = 0UL; i < num_of_knots_; ++i) {
      double curr_t = i * delta_t_;
      s = v0_ * curr_t;
      v = v0_;
      a = 0.0;
      jerk = 0.0;
      speed_data->AppendSpeedPoint(s, curr_t, v, a, jerk);
    }
    SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);
    return s;
  }
  // first: the acceleration changes from a0_ to the input accel at a constant
  // jerk.
  // second: deceleration to the target speed by constant target accelete value.
  // third: forward by the target_speed.
  double t1 = 0.0, jerk1 = 0.0;
  if (target_accel < a0_) {
    t1 = (target_accel - a0_) / FLAGS_min_jerk_for_const_accel_speed_plan;
    jerk1 = FLAGS_min_jerk_for_const_accel_speed_plan;
  } else {
    t1 = (target_accel - a0_) / FLAGS_max_jerk_for_const_accel_speed_plan;
    jerk1 = FLAGS_max_jerk_for_const_accel_speed_plan;
  }
  double s1 = v0_ * t1 + 0.5 * a0_ * t1 * t1 + jerk1 * t1 * t1 * t1 / 6.0;
  double v1 = v0_ + a0_ * t1 + 0.5 * jerk1 * t1 * t1;
  double a1 = a0_ + jerk1 * t1;

  double t2 = 0.0;
  if (v1 > target_speed) {
    t2 = (target_speed - v1) / target_accel + t1;
  } else {
    // 0.5*J*t^2 + a*t + v_0 = v_t
    t1 = (-a0_ - sqrt(a0_ * a0_ - 2.0 * jerk1 * (v0_ - target_speed))) / jerk1;
    s1 = v0_ * t1 + 0.5 * a0_ * t1 * t1 + jerk1 * t1 * t1 * t1 / 6.0;
    v1 = v0_ + a0_ * t1 + 0.5 * jerk1 * t1 * t1;
    a1 = a0_ + jerk1 * t1;
    t2 = t1;
  }
  const double s2 = s1 + v1 * (t2 - t1) + 0.5 * a1 * (t2 - t1) * (t2 - t1);
  const double v2 = v1 + a1 * (t2 - t1);
  for (size_t i = 0UL; i < num_of_knots_; ++i) {
    double curr_t = i * delta_t_;
    if (curr_t <= t1) {
      s = v0_ * curr_t + 0.5 * a0_ * curr_t * curr_t +
          jerk1 * curr_t * curr_t * curr_t / 6.0;
      v = v0_ + a0_ * curr_t + 0.5 * jerk1 * curr_t * curr_t;
      a = a0_ + jerk1 * curr_t;
      jerk = jerk1;
    } else if (curr_t <= t2) {
      double curr_t2 = curr_t - t1;
      s = s1 + v1 * curr_t2 + 0.5 * a1 * curr_t2 * curr_t2;
      v = v1 + a1 * curr_t2;
      a = a1;
      jerk = 0.0;
    } else {
      // average speed move
      s = s2 + v2 * (curr_t - t2);
      v = v2;
      a = 0.0;
      jerk = 0.0;
    }
    if (curr_t > FLAGS_fallback_total_time &&
        v > FLAGS_planning_upper_speed_limit) {
      break;
    }
    // Avoid the very last points when already stopped
    if (v < kEpsilon) {
      ADEBUG << "KinematicDecelerationSpeedOptimizer calculate speed is "
                "negative."
             << "\n\t\tcurrent time:(" << curr_t << "), s(" << s << "), v(" << v
             << "), a(" << a << ")"
             << "\n\t\t[init status] v0:(" << v0_ << "), a0:(" << a0_ << ")";
      break;
    }
    speed_data->AppendSpeedPoint(s, curr_t, v, a, jerk);
  }
  SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);
  return s;
}

}  // namespace planning
}  // namespace century
