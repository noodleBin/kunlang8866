/******************************************************************************
 * Copyright 2024 The Move-X Authors. All Rights Reserved.
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

#include <chrono>
#include <string>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "cyber/time/time.h"
namespace century {

using TimeUs = uint64_t;

constexpr TimeUs kMillisecond = 1000;
constexpr TimeUs kSecond = 1000 * kMillisecond;
constexpr TimeUs kMinute = 60 * kSecond;
constexpr TimeUs kHour = 60 * kMinute;
constexpr TimeUs kDay = 24 * kHour;
constexpr TimeUs kWeek = 7 * kDay;

constexpr TimeUs FromNanoseconds(uint64_t t) { return (t + 500) / 1000; }

constexpr TimeUs FromSeconds(double t) {
  return static_cast<TimeUs>(t * kSecond + 0.5);
}

// TimeUs FromCyberTime(const cyber::Time &t) {
//   return static_cast<TimeUs>(t.ToMicrosecond());
// }

// inline TimeUs Now() { return FromCyberTime(cyber::Time::Now()); }

inline cyber::Time ToCyberTime(TimeUs t) {
  auto T = new cyber::Time(t * 1000);
  return *T;
}

inline double TimeDiffSeconds(TimeUs t_start, TimeUs t_end) {
  return static_cast<double>(t_end - t_start) / static_cast<double>(kSecond);
}

class RuntimeCounter {
 public:
  explicit RuntimeCounter(bool is_print = true) : is_print_(is_print) {
    // time_ = std::chrono::steady_clock::now();
  }

  void ResetCounter(const std::string& str = "") {
    time_ = century::cyber::Clock::NowInSeconds();
    if (true == is_print_) {
      AINFO << str << " ResetCounter...";
    }
  }

  double Duration(const std::string& str = "") {
    double time_duration = century::cyber::Clock::NowInSeconds() - time_;
    if (true == is_print_) {
      AINFO << str << " Duration, Cost(s): " << time_duration;
    }
    return time_duration;
  }

  void StartCounter(const std::string& str = "") {
    time_ = century::cyber::Clock::NowInSeconds();
    if (true == is_print_) {
      AINFO << str << " StartCounter...";
    }
  }

  void EndCounter(const std::string& str = "") {
    double time_duration = century::cyber::Clock::NowInSeconds() - time_;
    time_duration_ = time_duration;
    if (true == is_print_) {
      AINFO << str << " EndCounter, Cost(s): " << time_duration;
    }
  }

  double GetDurationTime() const { return time_duration_; }

 private:
  // std::chrono::steady_clock::time_point time_;
  double time_;
  double time_duration_;
  bool is_print_;
};

}  // namespace century
