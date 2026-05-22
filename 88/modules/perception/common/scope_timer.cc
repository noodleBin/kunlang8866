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

#include "modules/perception/common/scope_timer.h"

#include <iomanip>
#include <utility>

#include "cyber/common/log.h"

namespace century {
namespace perception {
namespace common {

double GetCurrentUnixTimestamp() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return static_cast<double>(tv.tv_sec) +
         static_cast<double>(tv.tv_usec) / 1000000.0;
}

SimpleTimer::SimpleTimer() {
  Start();
}

void SimpleTimer::Start() {
  gettimeofday(&start_time_, nullptr);
}

double SimpleTimer::End(const std::string& msg) {
  struct timeval end_time;
  gettimeofday(&end_time, nullptr);
  long seconds = end_time.tv_sec - start_time_.tv_sec;
  long useconds = end_time.tv_usec - start_time_.tv_usec;
  double elapsed_ms = seconds * 1000.0 + useconds / 1000.0;
  double unix_ts = static_cast<double>(end_time.tv_sec) +
                   static_cast<double>(end_time.tv_usec) / 1000000.0;
  AINFO << std::fixed << std::setprecision(3) << "[Timer] [" << unix_ts << "] "
        << msg << " elapsed: " << elapsed_ms << " ms";
  return elapsed_ms;
}

FunctionScopeTimer::FunctionScopeTimer(std::string name)
    : name_(std::move(name)), start_unix_(GetCurrentUnixTimestamp()) {
  AINFO << std::fixed << std::setprecision(6) << "[FuncEnter] " << name_
        << " start=" << start_unix_;
}

FunctionScopeTimer::~FunctionScopeTimer() {
  const double end_unix = GetCurrentUnixTimestamp();
  const double elapsed_ms = (end_unix - start_unix_) * 1000.0;
  AINFO << std::fixed << std::setprecision(3) << "[FuncExit] " << name_
        << " end=" << std::setprecision(6) << end_unix << std::setprecision(3)
        << " elapsed=" << elapsed_ms << " ms";
}

}  // namespace common
}  // namespace perception
}  // namespace century
