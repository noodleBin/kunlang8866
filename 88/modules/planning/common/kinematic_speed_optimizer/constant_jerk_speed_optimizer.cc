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
 * @file constant_jerk_speed_optimizer.cc
 **/

#include "modules/planning/common/kinematic_speed_optimizer/constant_jerk_speed_optimizer.h"

#include "cyber/common/log.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_profile_generator.h"

namespace century {
namespace planning {

namespace {
static constexpr double kMinInitialSpeed = 1e-1;
static constexpr double kMinInitialDistance = 1e-2;
static constexpr double kMinStopTime = 1e-1;
}  // namespace

KinematicSpeedBaseOptimizer::KinematicInitStatus
ConstantJerkSpeedOptimizer::Init(double s0, double v0, double a0) {
  initialized_ = NONE_INITIALIZATION;
  s0_ = s0;
  v0_ = v0;
  a0_ = a0;
  ADEBUG << "s0:(" << s0_ << "), v0:(" << v0_ << "), a0:(" << a0_ << ")";
  if (v0_ <= kMinInitialSpeed && s0_ <= kMinInitialDistance) {
    ADEBUG << "ADC is arrived the destination.";
    return initialized_;
  }
  initialized_ = PrimaryInit();
  if (NONE_INITIALIZATION == initialized_) {
    initialized_ = AdvancedInit();
  }
  ADEBUG << "the initialized_ is: " << initialized_;
  return initialized_;
}

bool ConstantJerkSpeedOptimizer::GenerateSpeedDataByStopDecision(
    SpeedData* const speed_data) {
  if (NONE_INITIALIZATION == initialized_) {
    return false;
  }
  double s = 0.0, v = 0.0, a = 0.0;
  speed_data->clear();
  for (size_t i = 0UL; i < num_of_knots_; ++i) {
    double curr_t = i * delta_t_;
    if (curr_t <= transition_t_) {
      s = v0_ * curr_t + 0.5 * a0_ * curr_t * curr_t +
          jerk_ * curr_t * curr_t * curr_t / 6.0;
      v = v0_ + a0_ * curr_t + 0.5 * jerk_ * curr_t * curr_t;
      a = a0_ + jerk_ * curr_t;
      if (curr_t > transition_t_ - delta_t_) {
        if (CONSTANT_PRIMARY_INITIALIZATION == initialized_) {
          ADEBUG << "the stop point(PRIMARY_INITIALIZATION):";
        } else if (CONSTANT_ADVANCED_INITIALIZATION == initialized_) {
          ADEBUG << "the transition point(CONSTANT_ADVANCED_INITIALIZATION):";
        }
        ADEBUG << "s(" << s << "), v(" << v << "), a(" << a << ")";
      }
    } else if (curr_t <= stop_t_) {
      double max_dec = common::VehicleConfigHelper::GetConfig()
                           .vehicle_param()
                           .max_deceleration();
      s = transition_s_ + transition_v_ * (curr_t - transition_t_) +
          0.5 * max_dec * (curr_t - transition_t_) * (curr_t - transition_t_);
      v = transition_v_ + max_dec * (curr_t - transition_t_);
      a = max_dec;
      if (curr_t > stop_t_ - delta_t_) {
        if (CONSTANT_PRIMARY_INITIALIZATION == initialized_) {
          ADEBUG << "NEVER ARRIVED.";
        } else if (CONSTANT_ADVANCED_INITIALIZATION == initialized_) {
          ADEBUG << "the stop point(CONSTANT_ADVANCED_INITIALIZATION):";
        }
        ADEBUG << "s(" << s << "), v(" << v << "), a(" << a << ")";
      }
    } else {
      ADEBUG << "arrived the stop point early.";
      break;
    }
    // Avoid the very last points when already stopped
    if (v < 0.0) {
      ADEBUG << "ConstantJerkSpeedOptimizer calculate speed is negative.";
      ADEBUG << "current time:(" << curr_t << "), s(" << s << "), v(" << v
             << "), a(" << a << ")";
      ADEBUG << "[init status] s0:(" << s0_ << "), v0:(" << v0_ << "), a0:("
             << a0_ << ")";
      break;
    }
    speed_data->AppendSpeedPoint(s, curr_t, v, a, jerk_);
  }
  SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);
  return true;
}

KinematicSpeedBaseOptimizer::KinematicInitStatus
ConstantJerkSpeedOptimizer::PrimaryInit() {
  KinematicInitStatus initialized = NONE_INITIALIZATION;
  double delta = 4.0 * v0_ * v0_ + 6.0 * a0_ * s0_;
  double am = 0.0;
  double max_dec = common::VehicleConfigHelper::GetConfig()
                       .vehicle_param()
                       .max_deceleration();
  if (delta >= 0.0) {
    if (common::util::IsFloatEqual(a0_, 0.0)) {
      stop_t_ = 1.5 * s0_ / v0_;
    } else {
      stop_t_ = (-2.0 * v0_ + std::sqrt(delta)) / a0_;
    }
    jerk_ = -2.0 * (a0_ * stop_t_ + v0_) / (stop_t_ * stop_t_);
    am = a0_ + jerk_ * stop_t_;
    ADEBUG << "(CONSTANT_PRIMARY_INITIALIZATION) Jerk:(" << jerk_ << "), am:("
           << am << ")";
    if (am >= max_dec && jerk_ >= FLAGS_longitudinal_jerk_lower_bound) {
      transition_t_ = stop_t_;
      transition_v_ = 0.0;
      transition_s_ = s0_;
      initialized = CONSTANT_PRIMARY_INITIALIZATION;
    } else {
      ADEBUG << "Deceleration or deceleration jerk exceed the threshold, "
                "CONSTANT_ADVANCED_INITIALIZATION will be executed.";
    }
  } else {
    ADEBUG << "There is no solution to the equation of "
              "CONSTANT_PRIMARY_INITIALIZATION, "
              "CONSTANT_ADVANCED_INITIALIZATION will be executed.";
    ADEBUG << "delta = " << delta;
  }
  return initialized;
}

KinematicSpeedBaseOptimizer::KinematicInitStatus
ConstantJerkSpeedOptimizer::AdvancedInit() {
  KinematicInitStatus initialized = NONE_INITIALIZATION;
  double max_dec = common::VehicleConfigHelper::GetConfig()
                       .vehicle_param()
                       .max_deceleration();
  double coeff_a = (max_dec + 2.0 * a0_) / 6.0 -
                   0.125 * (max_dec + a0_) * (max_dec + a0_) / max_dec;
  double coeff_b = 0.5 * (max_dec - a0_) * v0_ / max_dec;
  double coeff_c = -0.5 * v0_ * v0_ / max_dec - s0_;
  double delta = coeff_b * coeff_b - 4.0 * coeff_a * coeff_c;
  if (delta < 0.0) {
    ADEBUG << "There is no solution to the equation of "
              "CONSTANT_ADVANCED_INITIALIZATION.";
    ADEBUG << "delta = " << delta;
    return initialized;
  }
  if (common::util::IsFloatEqual(coeff_a, 0.0)) {
    if (common::util::IsFloatEqual(coeff_b, 0.0)) {
      AERROR << "the coeff_b is zero!";
      coeff_b = FLAGS_numerical_epsilon;
    }
    transition_t_ = -coeff_c / coeff_b;
  } else {
    transition_t_ = 0.5 * (-coeff_b + std::sqrt(delta)) / coeff_a;
  }
  if (transition_t_ < kMinStopTime) {
    ADEBUG << "Invilid transition time.";
    return initialized;
  }
  jerk_ = (max_dec - a0_) / transition_t_;
  ADEBUG << "(CONSTANT_ADVANCED_INITIALIZATION) Jerk:(" << jerk_ << ")";
  if (jerk_ < FLAGS_longitudinal_jerk_lower_bound) {
    AWARN << "jerk has exceeded the maximum deceleration.";
    return initialized;
  }
  transition_v_ = v0_ + 0.5 * (max_dec + a0_) * transition_t_;
  transition_s_ = (max_dec + 2.0 * a0_) * transition_t_ * transition_t_ / 6.0 +
                  v0_ * transition_t_;
  stop_t_ = transition_t_ - transition_v_ / max_dec;
  initialized = CONSTANT_ADVANCED_INITIALIZATION;
  return initialized;
}

}  // namespace planning
}  // namespace century
