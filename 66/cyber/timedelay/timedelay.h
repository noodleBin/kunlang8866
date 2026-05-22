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

#include <libgen.h>
#include <string.h>
#include <sys/syscall.h>

#include <atomic>
#include <experimental/filesystem>
#include "cyber/timedelay/timewrite.h"

#define TIMEDELAY_TRACE_ENABLE

#ifdef TIMEDELAY_TRACE_ENABLE
#define TIMEDELAY_TRACE(tag, ...) _TIMEDELAY_TRACE(tag, __VA_ARGS__)
#else
#define TIMEDELAY_TRACE(tag, ...)
#endif

#define DELAYTRACE_DEFAULT Default
#define DELAYTRACE_CANBUS Canbus
#define DELAYTRACE_CONTROLL Controll
#define DELAYTRACE_PLANNING Planning
#define DELAYTRACE_PREDICTION Prediction
#define DELAYTRACE_FUSION Fusion
#define DELAYTRACE_LIDAR Lidar
#define DELAYTRACE_MONITOR Monitor

namespace century {
namespace timedelay {
class Timedelay {
 public:
  static Timedelay *Instance() {
    if (nullptr == instance_) {
      std::lock_guard<std::mutex> lk(mtx_);
      if (nullptr == instance_) {
        instance_ = new Timedelay();
        atexit(Destroy);
      }
    }
    return instance_;
  }

  void RecordDefault(const char *format, ...) {
    if (stop_.load(std::memory_order_acquire)) {
      return;
    }
    char data[1024] = {0};
    va_list arg_ptr;
    va_start(arg_ptr, format);
    int size = vsnprintf(data, sizeof(data) - 1, format, arg_ptr);
    va_end(arg_ptr);
    writer_.Record(writer_.WRITETYPE_DEFAULT,
                   DELAYTRACE_TOSTRING(DELAYTRACE_DEFAULT), data, size);
  }

  void RecordCanbus(const char *tag, uint64_t seq1, uint64_t seq2) {
    if (stop_.load(std::memory_order_acquire)) {
      return;
    }
    char data[1024] = {0};
    struct timespec tm;
    clock_gettime(CLOCK_REALTIME, &tm);
    snprintf_s(data, sizeof(data), "[%s][%lu][%lu][%lu%09ld]\n", tag, seq1,
               seq2, tm.tv_sec, tm.tv_nsec);
    writer_.Record(writer_.WRITETYPE_CANBUS,
                   DELAYTRACE_TOSTRING(DELAYTRACE_CANBUS), data, strlen(data));
  }

  void RecordControll(const char *tag, uint64_t seq1, uint64_t seq2) {
    if (stop_.load(std::memory_order_acquire)) {
      return;
    }
    char data[1024] = {0};
    struct timespec tm;
    clock_gettime(CLOCK_REALTIME, &tm);
    snprintf_s(data, sizeof(data), "[%s][%lu][%lu][%lu%09ld]\n", tag, seq1,
               seq2, tm.tv_sec, tm.tv_nsec);
    writer_.Record(writer_.WRITETYPE_CONTROLL,
                   DELAYTRACE_TOSTRING(DELAYTRACE_CONTROLL), data,
                   strlen(data));
  }

  void RecordPlanning(const char *tag, uint64_t seq1, uint64_t seq2) {
    if (stop_.load(std::memory_order_acquire)) {
      return;
    }
    char data[1024] = {0};
    struct timespec tm;
    clock_gettime(CLOCK_REALTIME, &tm);
    snprintf_s(data, sizeof(data), "[%s][%lu][%lu][%lu%09ld]\n", tag, seq1,
               seq2, tm.tv_sec, tm.tv_nsec);
    writer_.Record(writer_.WRITETYPE_PLANNING,
                   DELAYTRACE_TOSTRING(DELAYTRACE_PLANNING), data,
                   strlen(data));
  }

  void RecordPrediction(const char *tag, uint64_t seq1, uint64_t seq2) {
    if (stop_.load(std::memory_order_acquire)) {
      return;
    }
    char data[1024] = {0};
    struct timespec tm;
    clock_gettime(CLOCK_REALTIME, &tm);
    snprintf_s(data, sizeof(data), "[%s][%lu][%lu][%lu%09ld]\n", tag, seq1,
               seq2, tm.tv_sec, tm.tv_nsec);
    writer_.Record(writer_.WRITETYPE_PREDICTION,
                   DELAYTRACE_TOSTRING(DELAYTRACE_PREDICTION), data,
                   strlen(data));
  }

  void RecordFusion(const char *tag, uint64_t seq1, uint64_t seq2) {
    if (stop_.load(std::memory_order_acquire)) {
      return;
    }
    char data[1024] = {0};
    struct timespec tm;
    clock_gettime(CLOCK_REALTIME, &tm);
    snprintf_s(data, sizeof(data), "[%s][%lu][%lu][%lu%09ld]\n", tag, seq1,
               seq2, tm.tv_sec, tm.tv_nsec);
    writer_.Record(writer_.WRITETYPE_FUSION,
                   DELAYTRACE_TOSTRING(DELAYTRACE_FUSION), data, strlen(data));
  }

  void RecordLidar(const char *tag, uint64_t seq1, uint64_t seq2,
                   uint64_t timestamp) {
    if (stop_.load(std::memory_order_acquire)) {
      return;
    }
    char data[1024] = {0};
    struct timespec tm;
    clock_gettime(CLOCK_REALTIME, &tm);
    snprintf_s(data, sizeof(data), "[%s][%lu][%lu][%09ld][%lu%09ld]\n", tag,
               seq1, seq2, timestamp, tm.tv_sec, tm.tv_nsec);
    writer_.Record(writer_.WRITETYPE_LIDAR,
                   DELAYTRACE_TOSTRING(DELAYTRACE_LIDAR), data, strlen(data));
  }

  void RecordMonitor(const char *tag, uint64_t seq1) {
    if (stop_.load(std::memory_order_acquire)) {
      return;
    }
    char data[1024] = {0};
    struct timespec tm;
    clock_gettime(CLOCK_REALTIME, &tm);
    snprintf_s(data, sizeof(data), "[%s][%lu][%lu%09ld]\n", tag, seq1,
               tm.tv_sec, tm.tv_nsec);
#ifdef __x86_64__
    writer_.Record(writer_.WRITETYPE_MONITOR,
                   DELAYTRACE_TOSTRING(DELAYTRACE_MONITOR) "_adplan", data,
                   strlen(data));
#else
    writer_.Record(writer_.WRITETYPE_MONITOR,
                   DELAYTRACE_TOSTRING(DELAYTRACE_MONITOR) "_adsens", data,
                   strlen(data));
#endif
  }

 private:
  static Timedelay *instance_;
  static std::mutex mtx_;
  std::atomic_bool stop_{true};
  Timewrite writer_;

  Timedelay() {
    if (writer_.Init()) {
      stop_.store(false, std::memory_order_relaxed);
    }
  }

  ~Timedelay() {
    stop_.store(true, std::memory_order_release);
    writer_.Stop();
  }

  static void Destroy() {
    if (nullptr != instance_) {
      delete instance_;
      instance_ = nullptr;
    }
  }
};

Timedelay *Timedelay::instance_ = nullptr;
std::mutex Timedelay::mtx_;
}  // namespace timedelay
}  // namespace century
