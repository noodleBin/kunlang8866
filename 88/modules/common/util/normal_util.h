/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
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
 * @file normal_util.h
 * @brief Some frequently used data values.
 */

#pragma once
#include <cstdint>

#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"

/**
 * @namespace century::common::util
 * @brief century::common::util
 */
namespace century {
namespace common {
namespace util {

constexpr double RAD2ANG = 57.2957795;
constexpr double ANG2RAD = 0.01745329;

constexpr double kMathEpsilon = 1e-10;

double InverseInterpolationLookUp(const double x, const double x_lower,
                                  const double x_upper, double y_lower,
                                  double y_upper);

template <typename T>
void Origin2World(const T& origin_point, const double origin_heading,
                  const T& convert_origin_pt, const double convert_heading,
                  T& world_point, double& world_heading) {
  // origin ---> world
  T point(convert_origin_pt.x(), convert_origin_pt.y());
  point.SelfRotate(origin_heading);
  point += origin_point;
  world_heading =
      common::math::NormalizeAngle(convert_heading + origin_heading);
  world_point.set_x(point.x());
  world_point.set_y(point.y());
  return;
}

class TurnStateDetector {
 public:
  enum class TurnState { STRAIGHT = 0, TURNING = 1 };

  enum class TurnDirection { NONE = 0, LEFT = 1, RIGHT = -1 };
  /*
    enter_turn_threshold = 0.12 // rad
    exit_turn_threshold = 0.08  // rad
    enter_hold_frames = 3       // ≈150 ms
    exit_hold_frames = 5        // ≈250 ms
  */

  TurnStateDetector(double enter_turn_threshold, double exit_turn_threshold,
                    uint32_t enter_hold_frames, uint32_t exit_hold_frames)
      : enter_turn_threshold_(enter_turn_threshold),
        exit_turn_threshold_(exit_turn_threshold),
        enter_hold_frames_(enter_hold_frames),
        exit_hold_frames_(exit_hold_frames),
        state_(TurnState::STRAIGHT),
        direction_(TurnDirection::NONE),
        enter_counter_(0),
        exit_counter_(0) {}

  void Update(double average_wheel_angle);

  bool IsTurning() const { return state_ == TurnState::TURNING; }

  TurnDirection GetTurnDirection() const { return direction_; }

  TurnState GetState() const { return state_; }

  void Reset(double enter_turn_threshold, double exit_turn_threshold,
             uint32_t enter_hold_frames, uint32_t exit_hold_frames);

 private:
  // angle threshold (hystheresis)
  double enter_turn_threshold_;
  double exit_turn_threshold_;

  // time hystheresis
  uint32_t enter_hold_frames_;
  uint32_t exit_hold_frames_;

  // state
  TurnState state_;
  TurnDirection direction_;

  // counter
  uint32_t enter_counter_;
  uint32_t exit_counter_;
};

}  // namespace util
}  // namespace common
}  // namespace century