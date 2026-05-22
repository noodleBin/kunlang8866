
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
 * @file normal_util.cc
 **/

#include "modules/common/util/normal_util.h"

namespace century {
namespace common {
namespace util {

void TurnStateDetector::Update(double average_wheel_angle) {
  const double abs_angle = std::fabs(average_wheel_angle);
  // AINFO << "abs_angle: " << abs_angle
  //       << ", state_: " << static_cast<int>(state_);
  AINFO << "enter_turn_threshold_: " << enter_turn_threshold_
        << ", exit_turn_threshold_: " << exit_turn_threshold_
        << ", enter_hold_frames_: " << enter_hold_frames_
        << ", exit_hold_frames_: " << exit_hold_frames_;
  switch (state_) {
    case TurnState::STRAIGHT: {
      if (abs_angle >= enter_turn_threshold_) {
        ++enter_counter_;
        // AINFO << "STRAIGHT-enter_counter_: " << enter_counter_;
        if (enter_counter_ >= enter_hold_frames_) {
          state_ = TurnState::TURNING;
          enter_counter_ = 0;
          if (average_wheel_angle > kMathEpsilon) {
            direction_ = TurnDirection::LEFT;
          } else {
            direction_ = TurnDirection::RIGHT;
          }
          // AINFO << "TURNING!!!! direction_: " << static_cast<int>(direction_);
        }
      } else {
        enter_counter_ = 0;
      }
      break;
    }

    case TurnState::TURNING: {
      if (abs_angle <= exit_turn_threshold_) {
        ++exit_counter_;
        // AINFO << "TURNING-exit_counter_: " << exit_counter_;
        if (exit_counter_ >= exit_hold_frames_) {
          state_ = TurnState::STRAIGHT;
          exit_counter_ = 0;
          direction_ = TurnDirection::NONE;
          // AINFO << "STRAIGHT!!!!";
        }
      } else {
        exit_counter_ = 0;
      }
      break;
    }

    default:
      break;
  }
}

void TurnStateDetector::Reset(double enter_turn_threshold,
                              double exit_turn_threshold,
                              uint32_t enter_hold_frames,
                              uint32_t exit_hold_frames) {
  enter_turn_threshold_ = enter_turn_threshold;
  exit_turn_threshold_ = exit_turn_threshold;
  enter_hold_frames_ = enter_hold_frames;
  exit_hold_frames_ = exit_hold_frames;
  // state_ = TurnState::STRAIGHT;
  // direction_ = TurnDirection::NONE;
  // enter_counter_ = 0;
  // exit_counter_ = 0;
}

double InverseInterpolationLookUp(const double x, const double x_lower,
                                  const double x_upper, double y_lower,
                                  double y_upper) {
    if (std::fabs(x_upper - x_lower) <= kMathEpsilon) {
    AERROR << "input x axis diff is too small";
    return y_lower;
  }

  if (y_lower > y_upper) {
    std::swap(y_lower, y_upper);
  }

  if (x <= x_lower) {
    return y_upper;
  } else if (x >= x_upper) {
    return y_lower;
  } else {
    return y_upper - (x - x_lower) * (y_upper - y_lower) / (x_upper - x_lower);
  }
}

}  // namespace util
}  // namespace common
}  // namespace century