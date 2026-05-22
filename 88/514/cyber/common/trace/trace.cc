/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
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

#include "cyber/common/trace/trace.h"

#include <time.h>

#include <algorithm>
#include <chrono>

namespace century {
namespace cyber {
namespace common {
namespace {

uint64_t TimespecDiffUs(const timespec& start, const timespec& end) {
  int64_t sec_diff = static_cast<int64_t>(end.tv_sec) -
                     static_cast<int64_t>(start.tv_sec);
  int64_t nsec_diff = static_cast<int64_t>(end.tv_nsec) -
                      static_cast<int64_t>(start.tv_nsec);
  int64_t total_us = sec_diff * 1000000LL + nsec_diff / 1000LL;
  return static_cast<uint64_t>(std::max<int64_t>(0, total_us));
}

}  // namespace

Trace::Trace() : start_time_(std::chrono::steady_clock::now()) {
  has_thread_cpu_start_time_ =
      (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &thread_cpu_start_time_) == 0);
}

Trace::~Trace() = default;

void Trace::Finish() {
  if (finished_) {
    return;
  }
  finished_ = true;

  if (!Enabled()) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto elapsed_us_count =
      std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_)
          .count();
  const auto elapsed_us =
      static_cast<uint64_t>(std::max<int64_t>(0, elapsed_us_count));
  uint64_t thread_cpu_us = 0;
  if (has_thread_cpu_start_time_) {
    timespec thread_cpu_end_time = {0, 0};
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &thread_cpu_end_time) == 0) {
      thread_cpu_us =
          TimespecDiffUs(thread_cpu_start_time_, thread_cpu_end_time);
    }
  }

  OnTraceEnd(elapsed_us, thread_cpu_us);
}

}  // namespace common
}  // namespace cyber
}  // namespace century
