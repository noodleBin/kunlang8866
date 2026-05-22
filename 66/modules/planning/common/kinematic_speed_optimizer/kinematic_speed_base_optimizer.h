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
 * @file kinematic_speed_base_optimizer.h
 **/

#pragma once

#include "modules/planning/proto/task_config.pb.h"

#include "modules/planning/common/speed/speed_data.h"
#include "modules/common/util/util.h"
namespace century {
namespace planning {

class KinematicSpeedBaseOptimizer {
 public:
  enum KinematicInitStatus {
    NONE_INITIALIZATION = 0,
    CONSTANT_PRIMARY_INITIALIZATION = 1,
    CONSTANT_ADVANCED_INITIALIZATION = 2,
    TWO_PIECEWISE_COMMON_INITIALIZATION = 3,
    TWO_PIECEWISE_SLIDE_INITIALIZATION = 4,
    TWO_PIECEWISE_START_INITIALIZATION = 5,
    TWO_PIECEWISE_STOP_INITIALIZATION = 6
  };

 public:
  KinematicSpeedBaseOptimizer() = delete;
  KinematicSpeedBaseOptimizer(size_t num_of_knots, double delta_t,
                              const PiecewiseJerkSpeedOptimizerConfig& config)
      : num_of_knots_(num_of_knots), delta_t_(delta_t), config_(config) {}
  virtual ~KinematicSpeedBaseOptimizer() {}
  virtual KinematicInitStatus Init(double s0, double v0, double a0) = 0;
  virtual bool GenerateSpeedDataByStopDecision(SpeedData* speed_data) = 0;

 protected:
  double s0_;
  double v0_;
  double a0_;
  size_t num_of_knots_;
  double delta_t_;
  double max_stop_time_;
  PiecewiseJerkSpeedOptimizerConfig config_;
  KinematicInitStatus initialized_ = NONE_INITIALIZATION;
};

}  // namespace planning
}  // namespace century
