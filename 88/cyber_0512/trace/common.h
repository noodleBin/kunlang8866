/******************************************************************************
 * Copyright 2023 The century Authors. All Rights Reserved.
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

#include <time.h>
#include <type_traits>
#include <string>
#include <utility>
#include "cyber/base/macros.h"
#include "cyber/trace/trace_enums.h"

namespace century {
namespace trace {
#define MACRO_STRING(x) #x

#define HAS_MEMBER_GET(member, default_val)                                    \
  template <typename T, typename = std::enable_if_t<std::is_class<T>::value>>  \
  struct Has_##member {                                                        \
    template <typename U>                                                      \
    static constexpr auto Check(int val)                                       \
        -> decltype(std::declval<U>().member, std::true_type());               \
    template <typename U>                                                      \
    static constexpr std::false_type Check(...);                               \
    static constexpr bool value =                                              \
        std::is_same<decltype(Check<T>(0)), std::true_type>::value;            \
  };                                                                           \
  template <typename T>                                                        \
  struct Get_##member {                                                        \
    template <typename U, std::enable_if_t<Has_##member<U>::value>* = nullptr> \
    static constexpr auto Get() {                                              \
      return T::member;                                                        \
    }                                                                          \
    template <typename U,                                                      \
              std::enable_if_t<!Has_##member<U>::value>* = nullptr>            \
    static constexpr auto Get() {                                              \
      return default_val;                                                      \
    }                                                                          \
    static constexpr auto value = Get<T>();                                    \
  };

struct DefaultTraceConfig {
  static constexpr const char* kFilePrefix = "/century/data/log/trace_";
  static constexpr uint8_t kMaxFileNumber = 5;
  static constexpr uint64_t kMaxLogSize = 50UL << 20;
  static constexpr bool kEnableWrite = false;
  static constexpr uint64_t kBatchSize = 128;
  static constexpr uint64_t kHeaderSize = 512UL << 10;
};

struct alignas(CACHELINE_SIZE) TraceData {
  uint64_t timestamp;
  pthread_t tid;
  TimestampType time_type;
  TaskType task_type;
  uint64_t task_id;

  TraceData() {}

  TraceData(uint64_t timestamp_, pthread_t tid_, TimestampType time_type_,
            TaskType task_type_, uint64_t task_id_)
      : timestamp(timestamp_),
        tid(tid_),
        time_type(time_type_),
        task_type(task_type_),
        task_id(task_id_) {}

  TraceData(TraceData&& other)
      : timestamp(other.timestamp),
        tid(other.tid),
        time_type(other.time_type),
        task_type(other.task_type),
        task_id(other.task_id) {
    other.timestamp = 0;
    other.tid = 0;
    other.time_type = TimestampType::TIMESTAMP_UNINITIALIZED;
    other.task_type = TaskType::TASK_UNINITIALIZED;
  }

  TraceData(const TraceData& other)
      : timestamp(other.timestamp),
        tid(other.tid),
        time_type(other.time_type),
        task_type(other.task_type),
        task_id(other.task_id) {}

  TraceData& operator=(const TraceData& other) {
    if (this != &other) {
      timestamp = other.timestamp;
      tid = other.tid;
      time_type = other.time_type;
      task_type = other.task_type;
      task_id = other.task_id;
    }
    return *this;
  }

  TraceData& operator=(TraceData&& other) {
    if (this != &other) {
      timestamp = std::move(other.timestamp);
      tid = std::move(other.tid);
      time_type = std::move(other.time_type);
      task_type = std::move(other.task_type);
      task_id = std::move(other.task_id);
    }
    return *this;
  }
};

uint64_t Timespec2Nanoseconds(const timespec& ts);

bool PathExists(const char* path);

std::string GetProcessName();

}  // namespace trace
}  // namespace century
