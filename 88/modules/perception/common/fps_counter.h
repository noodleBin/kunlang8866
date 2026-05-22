/******************************************************************************
 * Copyright 2026 The century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <deque>

#include "cyber/time/time.h"

namespace century {
namespace perception {

namespace {
constexpr int kTimeStampsThreshold = 2;
constexpr double kMinTimeDuration = 1e-6;
}  // namespace

class FpsCounter {
 public:
  explicit FpsCounter(double window_sec = 2.0) : window_sec_(window_sec) {}

  double GetFps() {
    const double now = cyber::Time::Now().ToSecond();
    timestamps_.push_back(now);

    while (!timestamps_.empty()) {
      if (now - timestamps_.front() > window_sec_) {
        timestamps_.pop_front();
      } else {
        break;
      }
    }

    if (timestamps_.size() < kTimeStampsThreshold) {
      return 0.0;
    }

    const double duration = timestamps_.back() - timestamps_.front();
    return duration > kMinTimeDuration ? timestamps_.size() / duration : 0.0;
  }

 private:
  double window_sec_;
  std::deque<double> timestamps_;
};

}  // namespace perception
}  // namespace century
