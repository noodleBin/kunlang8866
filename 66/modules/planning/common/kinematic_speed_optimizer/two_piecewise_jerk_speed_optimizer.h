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
 * @file two_piecewise_jerk_speed_optimizer.h
 **/

#pragma once

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/kinematic_speed_optimizer/kinematic_speed_base_optimizer.h"
#include "modules/planning/common/speed/speed_data.h"
namespace century {
namespace planning {

class TwoPiecewiseJerkSpeedOptimizer : public KinematicSpeedBaseOptimizer {
 public:
  TwoPiecewiseJerkSpeedOptimizer() = delete;
  TwoPiecewiseJerkSpeedOptimizer(
      size_t num_of_knots, double delta_t,
      const PiecewiseJerkSpeedOptimizerConfig& config)
      : KinematicSpeedBaseOptimizer(num_of_knots, delta_t, config) {}
  virtual ~TwoPiecewiseJerkSpeedOptimizer() {}
  KinematicInitStatus Init(double s0, double v0, double a0) override;
  bool GenerateSpeedDataByStopDecision(SpeedData* speed_data) override;

 private:
  void CalculateKeyStateByInitialization();
  void InitByDelta(const double delta);

 private:
  double t_all_ = 0.0;
  double t1_ = 0.0;
  double s1_ = 0.0;
  double v1_ = 0.0;
  double a1_ = 0.0;
  double jerk1_ = 0.0;
  double jerk2_ = 0.0;
};

}  // namespace planning
}  // namespace century
