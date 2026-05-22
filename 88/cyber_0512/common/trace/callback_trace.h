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

#ifndef CYBER_COMMON_CALLBACK_TRACE_H_
#define CYBER_COMMON_CALLBACK_TRACE_H_

#include <time.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>

namespace century {
namespace cyber {
namespace common {

std::string DemangleTypeName(const char* type_name);
bool CallbackTraceEnabled();
bool CallbackTraceShouldTrace(const char* type, const char* name);
void RecordCallbackTraceEnd(const char* type, const char* name,
                            const char* detail, uint64_t elapsed_us,
                            uint64_t thread_cpu_us);

inline bool CallbackTraceShouldTrace(const std::string& type,
                                     const std::string& name) {
  return CallbackTraceShouldTrace(type.c_str(), name.c_str());
}

class CallbackTrace {
 public:
  CallbackTrace(const std::string& type, const std::string& name)
      : CallbackTrace(type.c_str(), name.c_str(), "") {}

  CallbackTrace(const std::string& type, const std::string& name,
                const std::string& detail)
      : CallbackTrace(type.c_str(), name.c_str(), detail.c_str()) {}

  CallbackTrace(const char* type, const char* name, const char* detail)
      : type_(type == nullptr ? "" : type),
        name_(name == nullptr ? "" : name),
        detail_(detail == nullptr ? "" : detail),
        start_time_(std::chrono::steady_clock::now()) {
    has_thread_cpu_start_time_ =
        (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &thread_cpu_start_time_) == 0);
  }

  ~CallbackTrace() { Finish(); }

  CallbackTrace(const CallbackTrace&) = delete;
  CallbackTrace& operator=(const CallbackTrace&) = delete;

 private:
  static uint64_t TimespecDiffUs(const timespec& start, const timespec& end) {
    int64_t sec_diff =
        static_cast<int64_t>(end.tv_sec) - static_cast<int64_t>(start.tv_sec);
    int64_t nsec_diff =
        static_cast<int64_t>(end.tv_nsec) - static_cast<int64_t>(start.tv_nsec);
    int64_t total_us = sec_diff * 1000000LL + nsec_diff / 1000LL;
    return static_cast<uint64_t>(std::max<int64_t>(0, total_us));
  }

  void Finish() {
    if (finished_) {
      return;
    }
    finished_ = true;

    if (!CallbackTraceEnabled()) {
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

    RecordCallbackTraceEnd(type_.c_str(), name_.c_str(), detail_.c_str(),
                           elapsed_us, thread_cpu_us);
  }

  std::string type_;
  std::string name_;
  std::string detail_;
  std::chrono::steady_clock::time_point start_time_;
  timespec thread_cpu_start_time_ = {0, 0};
  bool has_thread_cpu_start_time_ = false;
  bool finished_ = false;
};

}  // namespace common
}  // namespace cyber
}  // namespace century

#endif  // CYBER_COMMON_CALLBACK_TRACE_H_
