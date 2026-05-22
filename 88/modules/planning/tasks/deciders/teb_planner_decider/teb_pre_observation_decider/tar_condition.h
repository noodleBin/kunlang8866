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
#include <memory>

#include "modules/planning/tasks/deciders/teb_planner_decider/teb_pre_observation_decider/teb_tar_fsm_common.h"
namespace century {
namespace planning {
constexpr double kTimeNormal2Creep = 8.0;
constexpr double kDistNormal2Creep = 20.0;
constexpr double kTimeCreep2Stop = 3.0;
constexpr double kDistCreep2Stop = 10.0;
constexpr double kDistNormal2Stop = 10.0;
constexpr double kTimeNormal2Stop = 3.0;
constexpr double kDistCreep2Normal = 20.0;
constexpr double kTimeCreep2Normal = 8.0;
constexpr double kStopSpeed = 0.05;
constexpr double kTarDuration1 = 1.0;
constexpr double kTarDuration2 = 0.5;
constexpr int kStopCycle = 50;

class TarFsmCondition {
 public:
  TarFsmCondition() = default;
  using TarInfo = std::shared_ptr<TarVehicleInfo>;
  // 1
  bool Normal2Creep(const TarInfo& tar, Frame* const frame) {
    if (!tar->Valid()) {
      return false;
    }
    if (tar->DistanceToAdc() < kDistNormal2Creep ||
        tar->CollisionTime() < kTimeNormal2Creep) {
      if (tar->Duration() > kTarDuration1) {
        return true;
      }
    }
    return false;
  }
  bool Normal2Stop(const TarInfo& tar, Frame* const frame) {
    if (!tar->Valid()) {
      return false;
    }
    if (tar->DistanceToAdc() < kDistNormal2Stop ||
        tar->CollisionTime() < kTimeNormal2Stop) {
      if (tar->Duration() > kTarDuration2) {
        return true;
      }
    }
    return false;
  }
  // 2
  bool Creep2Stop(const TarInfo& tar, Frame* const frame) {
    if (!tar->Valid()) {
      return false;
    }
    if (tar->DistanceToAdc() < kDistCreep2Stop ||
        tar->CollisionTime() < kTimeNormal2Stop) {
      return true;
    }
    return false;
  }
  bool Creep2Normal(const TarInfo& tar, Frame* const frame) {
    if (!tar->Valid() || (tar->CollisionTime() > kTimeCreep2Normal &&
                          tar->DistanceToAdc() > kDistCreep2Normal)) {
      return true;
    }
    return false;
  }
  // 3
  bool Stop2Normal(const TarInfo& tar, Frame* const frame) {
    if (!tar->Valid() &&
        std::fabs(frame->vehicle_state().linear_velocity()) < kStopSpeed) {
      return true;
    }
    return false;
  }
  bool Stop2Yield(const TarInfo& tar, Frame* const frame) {
    if (tar->Valid() &&
        std::fabs(frame->vehicle_state().linear_velocity()) < kStopSpeed &&
        frame->mutable_open_space_info()->stop_time() > kStopCycle) {
      return true;
    }
    return false;
  }
  // 4
  bool Yield2Fail(const TarInfo& tar, Frame* const frame) {
    // TODO(zhiqiang.ding)
    return false;
  }
  bool Yield2Stop(const TarInfo& tar, Frame* const frame) {
    // TODO(zhiqiang.ding)
    if (std::fabs(frame->vehicle_state().linear_velocity()) < kStopSpeed &&
        tar->LastPlanSuccess()) {
      return true;
    }
    return false;
  }
};
}  // namespace planning
}  // namespace century
