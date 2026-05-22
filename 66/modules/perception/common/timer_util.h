/******************************************************************************
 * Copyright 2025 The century Authors. All Rights Reserved.
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

#include "absl/strings/str_cat.h"

#include "cyber/common/macros.h"
#include "cyber/time/time.h"

namespace century {
namespace perception {
namespace common {

class Timer {
 public:
  Timer() = default;

  // no-thread safe.
  void Start();

  // return the elapsed time,
  // also output msg and time in glog.
  // automatically start a new timer.
  // no-thread safe.
  int64_t End(const std::string& msg);

 private:
  century::cyber::Time start_time_;
  century::cyber::Time end_time_;

  DISALLOW_COPY_AND_ASSIGN(Timer);
};

}  // namespace common
}  // namespace perception
}  // namespace century

#define PERCEPTION_PERF_BLOCK_START()             \
  century::perception::common::Timer _timer_; \
  _timer_.Start()
#define PERCEPTION_PERF_BLOCK_END(msg) _timer_.End(msg)
#define PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(indicator, msg) \
  _timer_.End(absl::StrCat(indicator, "_", msg))
