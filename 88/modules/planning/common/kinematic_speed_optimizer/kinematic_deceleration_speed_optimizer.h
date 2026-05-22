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
 * @file kinematic_deceleration_speed_optimizer.h
 **/

#pragma once

#include "modules/planning/proto/task_config.pb.h"

#include "modules/planning/common/speed/speed_data.h"
namespace century {
namespace planning {

class KinematicDecelerationSpeedOptimizer {
 public:
  enum RestrictInitAccelLimit {
    NONE_RESTRICT = 0,
    RESTRICT_UPPER_LIMIT = 1,
    RESTRICT_LOWER_LIMIT = 2,
    RESTRICT_LOWER_AND_UPPER_LIMIT = 3,
    RESTRICT_TO_SPECIFIED_VALUE = 4
  };

 public:
  KinematicDecelerationSpeedOptimizer() = delete;
  KinematicDecelerationSpeedOptimizer(
      size_t num_of_knots, double delta_t,
      const PiecewiseJerkSpeedOptimizerConfig& config)
      : num_of_knots_(num_of_knots), delta_t_(delta_t), config_(config) {}
  virtual ~KinematicDecelerationSpeedOptimizer() {}
  virtual void Init(
      const double v0, const double a0,
      const RestrictInitAccelLimit restrict_init_accel = NONE_RESTRICT,
      const double specified_value = 0.0);

  double GenerateSpeedDataByConstantJerk(const double jerk,
                                         SpeedData* const speed_data);
  double GenerateSpeedDataByConstantAccel(const double accel,
                                          SpeedData* const speed_data);
  double GenerateSpeedDataByConstantAccelAndSpeed(const double target_accel,
                                                  const double target_speed,
                                                  SpeedData* const speed_data);

 protected:
  RestrictInitAccelLimit restrict_init_accel_ = NONE_RESTRICT;
  double v0_;
  double a0_;
  size_t num_of_knots_;
  double delta_t_;
  PiecewiseJerkSpeedOptimizerConfig config_;
};

}  // namespace planning
}  // namespace century
