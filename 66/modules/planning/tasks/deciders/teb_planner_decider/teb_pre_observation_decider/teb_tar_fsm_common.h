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

#include <algorithm>
#include <functional>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "modules/planning/proto/planning.pb.h"

#include "cyber/time/clock.h"

namespace century {
namespace planning {
using century::cyber::Clock;
constexpr double kIgnoreDistance = 60;
constexpr double kPrpCheckTimeDefault = 1.0;
constexpr uint64_t kMaxTick = 2000;
constexpr uint32_t kMaxStopTime = 2000;
constexpr uint64_t kTarStatusTimeMax = 1200U;          // 120s
constexpr uint64_t kNanNormalTarStatusTimeMax = 200U;  // 20s

enum class TEBTarStatus : uint8_t {
  NORMAL = 1,
  CREEP = 2,
  STOP = 4,
  YIELD = 8,
  FAIL = 16,
};
class TarTickCheck {
 public:
  static void Set(const TEBTarStatus& state) {
    if (tar_state_ != state) {
      tar_state_ = state;
      tar_tick_ = 0;
    } else {
      ++tar_tick_;
      tar_tick_ = std::min(kMaxTick, tar_tick_);
    }
    if (tar_state_ == TEBTarStatus::NORMAL) {
      tar_big_tick_ = 0;
    } else {
      tar_big_tick_++;
      tar_tick_ = std::min(kMaxTick, tar_big_tick_);
    }
  }

  static uint64_t Tick(const TEBTarStatus& state) {
    return (tar_state_ != state) ? 0 : tar_tick_;
  }
  static uint64_t BigTick() { return tar_big_tick_; }

 private:
  static TEBTarStatus tar_state_;
  static uint64_t tar_tick_;
  static uint64_t tar_big_tick_;
};

class TarVehicleInfo {
 public:
  using IdPair = std::pair<std::string, int32_t>;
  TarVehicleInfo() = default;
  ~TarVehicleInfo() = default;
  bool Valid() { return valid_; }
  void SetValid(const bool flag) { valid_ = flag; }
  const std::string& Id() { return id_; }
  void SetId(const std::string& str) { id_ = str; }
  int32_t PerId() { return per_id_; }
  void SetPerId(const int32_t per) { per_id_ = per; }
  double LongDist() { return s_; }
  void SetLongDist(const double s) { s_ = s; }
  double LatDist() { return l_; }
  void SetLatDist(const double l) { l_ = l; }
  double Duration(const double t) { return std::max(t - start_s_, 0.0); }
  double Duration() { return std::max(Clock::NowInSeconds() - start_s_, 0.0); }
  void SetStartTime(const double t) { start_s_ = t; }
  bool LastPlanSuccess() { return last_plan_success_; }
  void SetLastPlanSuccess(bool t) { last_plan_success_ = t; }
  void ResetById(const IdPair& id) {
    // here it is matching scucess whether any id.
    if (valid_ && (id.first == id_ || id.second == per_id_)) {
    } else if (id.first == "\0" || id.second == 0) {
      Clear();
    } else {
      Clear();
      valid_ = true;
      id_ = id.first;
      per_id_ = id.second;
      start_s_ = Clock::NowInSeconds();
    }
  }
  void Clear() {
    id_ = "\0";
    per_id_ = 0;
    valid_ = false;
    s_ = 0.0;
    l_ = 0.0;
    start_s_ = 0.0;
    dist_ = 0.0;
    speed_ = 0.0;
    collision_time_ = std::numeric_limits<double>::infinity();
  }
  double DistanceToAdc() { return dist_; }
  void SetDistanceToAdc(const double d) { dist_ = d; }
  void SetSpeed(const double s) { speed_ = s; }
  double Speed() { return speed_; }
  double CollisionTime() { return collision_time_; }
  void SetCollisionTime(const double t) { collision_time_ = t; }

 private:
  std::string id_ = "\0";
  int32_t per_id_ = 0;
  bool valid_ = false;
  double s_ = 0.0;
  double l_ = 0.0;
  double start_s_ = 0.0;  // s
  double dist_ = 0.0;
  double speed_ = 0.0;
  double collision_time_ = std::numeric_limits<double>::infinity();
  double last_plan_success_ = false;  // no need clear, only reset.
};

}  // namespace planning
}  // namespace century
