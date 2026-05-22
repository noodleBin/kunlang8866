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
 * @file two_piecewise_jerk_speed_optimizer.cc
 **/

#include "modules/planning/common/kinematic_speed_optimizer/two_piecewise_jerk_speed_optimizer.h"

#include <algorithm>
#include <limits>

#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_profile_generator.h"

namespace century {
namespace planning {

namespace {
constexpr double kMinStartSpeedBuffer = 0.5;
constexpr double kEpsilon = 1e-10;
}  // namespace

KinematicSpeedBaseOptimizer::KinematicInitStatus
TwoPiecewiseJerkSpeedOptimizer::Init(double s0, double v0, double a0) {
  initialized_ = NONE_INITIALIZATION;
  s0_ = s0 < kEpsilon ? kEpsilon : s0;
  v0_ = v0 < kEpsilon ? kEpsilon : v0;
  a0_ = a0;
  ADEBUG << "s0:(" << s0_ << "), v0:(" << v0_ << "), a0:(" << a0_ << ")";
  if (s0_ <= config_.endpoint_distance_min_buffer()) {
    ADEBUG << "ADC has arrived the destination, need stop.";
    return initialized_;
  }
  if (v0_ <= config_.stop_speed_buffer()) {
    if (s0_ <= config_.endpoint_distance_max_buffer()) {
      ADEBUG << "ADC has arrived the destination.";
    } else {
      ADEBUG << "ADC hasn't arrived the destination, need start.";
      a0_ = std::max(a0_, config_.min_stoped_accel());
      initialized_ = TWO_PIECEWISE_START_INITIALIZATION;
      CalculateKeyStateByInitialization();
    }
    return initialized_;
  }
  max_stop_time_ =
      config_.min_stop_time() + std::sqrt(s0) / config_.average_stop_decel();

  double delta = 9.0 * v0_ * v0_ + 12.0 * a0_ * s0_;
  InitByDelta(delta);
  if (t_all_ > max_stop_time_) {
    initialized_ = TWO_PIECEWISE_START_INITIALIZATION;
  }
  if (jerk1_ < FLAGS_longitudinal_jerk_lower_bound ||
      jerk2_ < FLAGS_longitudinal_jerk_lower_bound ||
      a1_ < config_.min_deceleration()) {
    initialized_ = TWO_PIECEWISE_STOP_INITIALIZATION;
  }
  CalculateKeyStateByInitialization();
  return initialized_;
}

void TwoPiecewiseJerkSpeedOptimizer::InitByDelta(const double delta) {
  ADEBUG << "delta = " << delta;
  if (delta >= 0.0) {
    if (common::util::IsFloatEqual(a0_, 0.0)) {
      t_all_ = 2.0 * s0_ / v0_;
    } else {
      t_all_ = (-3.0 * v0_ + std::sqrt(delta)) / a0_;
    }
    t_all_ = (t_all_ < kEpsilon) ? kEpsilon : t_all_;
    jerk1_ = -(3 * a0_ * t_all_ + 4 * v0_) / (t_all_ * t_all_);
    jerk2_ = (a0_ * t_all_ + 4 * v0_) / (t_all_ * t_all_);
    t1_ = 0.5 * t_all_;
    s1_ = v0_ * t1_ + 0.5 * a0_ * t1_ * t1_ + jerk1_ * t1_ * t1_ * t1_ / 6.0;
    v1_ = v0_ + a0_ * t1_ + 0.5 * jerk1_ * t1_ * t1_;
    a1_ = a0_ + jerk1_ * t1_;
    initialized_ = TWO_PIECEWISE_COMMON_INITIALIZATION;
  } else {
    jerk1_ = FLAGS_longitudinal_jerk_upper_bound;
    if (2.0 * jerk1_ * v0_ > a0_ * a0_) {
      t1_ = -a0_ / jerk1_;
      v1_ = v0_ + a0_ * t1_ + 0.5 * jerk1_ * t1_ * t1_;
      if (v1_ > config_.stop_speed_buffer()) {
        jerk2_ = 0.0;
        t_all_ = t1_ + (s0_ - s1_) / v1_;
      } else {
        t1_ += std::sqrt(a0_ * a0_ -
                         2.0 * jerk1_ * (v0_ - config_.stop_speed_buffer())) /
               jerk1_;
        v1_ = v0_ + a0_ * t1_ + 0.5 * jerk1_ * t1_ * t1_;
        jerk2_ = FLAGS_longitudinal_jerk_upper_steady;
        t_all_ = std::max(t1_, FLAGS_fallback_total_time);
      }
      s1_ = v0_ * t1_ + 0.5 * a0_ * t1_ * t1_ + jerk1_ * t1_ * t1_ * t1_ / 6.0;
      a1_ = a0_ + jerk1_ * t1_;
      initialized_ = TWO_PIECEWISE_SLIDE_INITIALIZATION;
    } else {
      initialized_ = TWO_PIECEWISE_START_INITIALIZATION;
    }
  }
}

bool TwoPiecewiseJerkSpeedOptimizer::GenerateSpeedDataByStopDecision(
    SpeedData* const speed_data) {
  ADEBUG << "TwoPiecewiseJerk initialization: " << initialized_ << std::endl
         << "t_all: " << t_all_ << std::endl
         << "jerk1: " << jerk1_ << std::endl
         << "jerk2: " << jerk2_ << std::endl
         << "t1: " << t1_ << std::endl
         << "s1: " << s1_ << std::endl
         << "v1: " << v1_ << std::endl
         << "a1: " << a1_;
  double s = 0.0, v = 0.0, a = 0.0, jerk = 0.0;
  speed_data->clear();
  if (NONE_INITIALIZATION == initialized_) {
    speed_data->AppendSpeedPoint(s, 0.0, v, a, jerk);
    ADEBUG << "using fallback speed plan.";
  } else {
    double last_s = 0.0;
    for (size_t i = 0UL; i < num_of_knots_; ++i) {
      double curr_t = i * delta_t_;
      if (curr_t <= t1_) {
        s = v0_ * curr_t + 0.5 * a0_ * curr_t * curr_t +
            jerk1_ * curr_t * curr_t * curr_t / 6.0;
        v = v0_ + a0_ * curr_t + 0.5 * jerk1_ * curr_t * curr_t;
        a = a0_ + jerk1_ * curr_t;
        jerk = jerk1_;
      } else if (curr_t <= t_all_) {
        double t2 = curr_t - t1_;
        s = s1_ + v1_ * t2 + 0.5 * a1_ * t2 * t2 + jerk2_ * t2 * t2 * t2 / 6.0;
        v = v1_ + a1_ * t2 + 0.5 * jerk2_ * t2 * t2;
        a = a1_ + jerk2_ * t2;
        jerk = jerk2_;
      } else {
        ADEBUG << "arrived the stop point early.";
        break;
      }
      if (curr_t > FLAGS_fallback_total_time &&
          v > FLAGS_planning_upper_speed_limit) {
        break;
      }
      // Avoid the very last points when already stopped
      if (v < kEpsilon) {
        ADEBUG << "TwoPiecewiseJerkSpeedOptimizer calculate speed is negative.";
        ADEBUG << "current time:(" << curr_t << "), s(" << s << "), v(" << v
               << "), a(" << a << ")";
        ADEBUG << "[init status] s0:(" << s0_ << "), v0:(" << v0_ << "), a0:("
               << a0_ << ")";
        if (TWO_PIECEWISE_STOP_INITIALIZATION == initialized_) {
          break;
        } else {
          v = kEpsilon;
          s = last_s + kEpsilon;
        }
      }
      if (s < last_s) {
        s = last_s + kEpsilon;
      }
      last_s = s;
      speed_data->AppendSpeedPoint(s, curr_t, v, a, jerk);
    }
  }
  SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);
  return true;
}

void TwoPiecewiseJerkSpeedOptimizer::CalculateKeyStateByInitialization() {
  switch (initialized_) {
    case TWO_PIECEWISE_START_INITIALIZATION: {
      double v_target = config_.speed_up_ratio() * s0_ / max_stop_time_;
      if (v_target <= v0_ + kMinStartSpeedBuffer) {
        AWARN << "Acceleration start target speed is lower than init speed!";
        v_target = v0_ + kMinStartSpeedBuffer;
      }
      if (v_target > FLAGS_planning_upper_speed_limit) {
        AWARN << "Acceleration start target speed is bigger than upper limit "
                 "speed!";
        v_target = FLAGS_planning_upper_speed_limit;
      }
      jerk1_ = FLAGS_longitudinal_jerk_upper_steady;
      jerk2_ = 0.0;
      t1_ = (-a0_ + std::sqrt(a0_ * a0_ + 2.0 * jerk1_ * (v_target - v0_))) /
            jerk1_;
      s1_ = v0_ * t1_ + 0.5 * a0_ * t1_ * t1_ + jerk1_ * t1_ * t1_ * t1_ / 6.0;
      v1_ = v0_ + a0_ * t1_ + 0.5 * jerk1_ * t1_ * t1_;
      a1_ = a0_ + jerk1_ * t1_;
      t_all_ = std::max(t1_, FLAGS_fallback_total_time);
      break;
    }
    case TWO_PIECEWISE_STOP_INITIALIZATION: {
      double decel = -v0_ * v0_ / (2.0 * s0_);
      double s_buffer = std::max(
          v0_ * (decel - a0_) / FLAGS_longitudinal_jerk_lower_bound, 0.0);
      double s_amend =
          (s0_ - s_buffer > kEpsilon) ? (s0_ - s_buffer) : kEpsilon;
      double decel_target = -v0_ * v0_ / (2.0 * s_amend);
      if (decel_target < config_.min_deceleration()) {
        decel_target = config_.min_deceleration();
      }
      jerk1_ = FLAGS_longitudinal_jerk_lower_bound;
      jerk2_ = FLAGS_longitudinal_jerk_lower_bound;
      t1_ = (decel_target - a0_) / jerk1_;
      s1_ = v0_ * t1_ + 0.5 * a0_ * t1_ * t1_ + jerk1_ * t1_ * t1_ * t1_ / 6.0;
      v1_ = v0_ + a0_ * t1_ + 0.5 * jerk1_ * t1_ * t1_;
      a1_ = a0_ + jerk1_ * t1_;
      t_all_ = std::numeric_limits<double>::max();
      break;
    }
    default:
      ADEBUG << "Nothing to do.";
      break;
  }
}

}  // namespace planning
}  // namespace century
