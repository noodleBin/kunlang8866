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

#pragma once

#include "modules/planning/proto/planning.pb.h"

namespace century {
namespace planning {

enum class OverTakeStatus : uint8_t {
  DEFAULT,
  PREPARE,
  TURN,
  OVERTAKE,
  RETURN,
  HOLD,
  FINISH,
  FAIL,
};

class TickCheck {
 public:
  static void Set(const OverTakeStatus &state) {
    if (state_ != state) {
      state_ = state;
      tick_ = 0;
    } else {
      ++tick_;
    }
  }

  static uint64_t Tick(const OverTakeStatus &state) {
    return (state_ != state) ? 0 : tick_;
  }

 private:
  static OverTakeStatus state_;
  static uint64_t tick_;
};

ADCTrajectory::OvertakeReportState Transfer2ReportOvertakeState(
    const OverTakeStatus &status) {
  switch (status) {
    case OverTakeStatus::DEFAULT:
      return ADCTrajectory::DEFAULT;
    case OverTakeStatus::PREPARE:
      return ADCTrajectory::PREPARE;
    case OverTakeStatus::TURN:
      return ADCTrajectory::TURN;
    case OverTakeStatus::OVERTAKE:
      return ADCTrajectory::OVERTAKE;
    case OverTakeStatus::RETURN:
      return ADCTrajectory::RETURN;
    case OverTakeStatus::HOLD:
      return ADCTrajectory::HOLD;
    case OverTakeStatus::FINISH:
      return ADCTrajectory::FINISH;
    case OverTakeStatus::FAIL:
      return ADCTrajectory::FAIL;

    default:
      return ADCTrajectory::DEFAULT;
  }
}

}  // namespace planning
}  // namespace century
