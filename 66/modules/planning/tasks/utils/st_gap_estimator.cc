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

#include "modules/planning/tasks/utils/st_gap_estimator.h"

#include <algorithm>
#include <cmath>

#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/tasks/deciders/utils/path_decider_obstacle_utils.h"

namespace century {
namespace planning {

// TODO(Jinyun): move to configs
static constexpr double kOvertakeTimeBuffer = 3.0;    // in seconds
static constexpr double kMinOvertakeDistance = 10.0;  // in meters
static constexpr double kDpSafetyDistance = 20.0;     // in meters
static constexpr double kDpSafetyTimeBuffer = 3.0;    // in meters
static constexpr double kLaneFollowMaxDecel = 1.0;
static constexpr double kObsMaxDecel = 2.0;
static constexpr double kObsMinDecel = 3.0;
static constexpr double kMaxYieldBuffer = 5.0;
static constexpr double kVehicleMaxDecel = 6.0;
static constexpr double kMinxFollowDistance = 15.0;
static constexpr double kLowSpeed = 3.0;

// TODO(Jinyun): unite gap calculation in dp st and speed decider
double StGapEstimator::EstimateSafeOvertakingGap() { return kDpSafetyDistance; }

double StGapEstimator::EstimateSafeFollowingGap(const double target_obs_speed) {
  return target_obs_speed * kDpSafetyTimeBuffer;
}

double StGapEstimator::EstimateSafeYieldingGap() {
  return FLAGS_yield_distance;
}

// TODO(zongxingguo): add more variables to overtaking gap calculation
double StGapEstimator::EstimateProperOvertakingGap(
    const double target_obs_speed, const double adc_speed) {
  // check use time or use breakdistance.
  double min_speed =
      std::max(std::fabs(target_obs_speed), std::fabs(adc_speed));
  double overtake_distance_s = InterpolationLookUp(
      min_speed, FLAGS_play_street_speed_limit,
      FLAGS_planning_upper_speed_limit, FLAGS_min_overtake_longitude_buffer,
      FLAGS_max_overtake_longitude_buffer);
  return overtake_distance_s;
}

// TODO(Jinyun): add more variables to follow gap calculation
double StGapEstimator::EstimateProperFollowingGap(const double adc_speed,
                                                  const double obs_speed) {
  double follow_diatance = std::fmax(adc_speed * adc_speed * 0.5 / kLaneFollowMaxDecel -
                       obs_speed * obs_speed * 0.5 / kObsMaxDecel,
                   FLAGS_follow_min_distance);
  if(std::fabs(adc_speed) > kLowSpeed){
    follow_diatance = std::fmax(follow_diatance,kMinxFollowDistance);
  }
  return follow_diatance;
}

// TODO(Jinyun): add more variables to yielding gap calculation
double StGapEstimator::EstimateProperYieldingGap(const double adc_speed,
                                                 const double obs_speed) {
  double yield_distance = FLAGS_yield_distance;
  // consider reverse obstacle.
  yield_distance =
      std::min(std::max(FLAGS_yield_distance,
                        adc_speed * adc_speed * 0.5 / kLaneFollowMaxDecel),
               FLAGS_max_yield_buffer);

  if (FLAGS_enable_high_speed) {
    yield_distance =
        std::min(std::max(FLAGS_yield_distance,
                          adc_speed * adc_speed * 0.5 / kLaneFollowMaxDecel),
                 kMaxYieldBuffer);
    // yield_distance =
    //     std::max(FLAGS_yield_distance,
    //              adc_speed * adc_speed * 0.5 / kLaneFollowMaxDecel);
    // have too large yield distance.
    // if (yield_distance > kMaxYieldBuffer) {
    //   yield_distance = adc_speed * adc_speed * 0.5 / kVehicleMaxDecel;
    // }
  }

  return yield_distance;
}

}  // namespace planning
}  // namespace century
