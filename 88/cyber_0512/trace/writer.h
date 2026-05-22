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

#include <sys/syscall.h>
#include <libgen.h>
#include <cstdlib>
#include <string>
#include <queue>
#include <fstream>
#include <utility>
#include <vector>
#include <chrono>
#include <unordered_map>
#include <type_traits>
#include <experimental/filesystem>

#include "concurrent_queue/concurrent_queue.h"
#include "cyber/trace/common.h"
#include "cyber/threadlib/threadlib.h"

namespace century {
namespace trace {

namespace fs = std::experimental::filesystem;

HAS_MEMBER_GET(kFilePrefix, DefaultTraceConfig::kFilePrefix)
HAS_MEMBER_GET(kMaxFileNumber, DefaultTraceConfig::kMaxFileNumber)
HAS_MEMBER_GET(kMaxLogSize, DefaultTraceConfig::kMaxLogSize)
HAS_MEMBER_GET(kEnableWrite, DefaultTraceConfig::kEnableWrite)
HAS_MEMBER_GET(kBatchSize, DefaultTraceConfig::kBatchSize)
HAS_MEMBER_GET(kHeaderSize, DefaultTraceConfig::kHeaderSize)

template <typename T, typename U>
class Writer {
 public:
  void Init() {
    char* enable_trace_env = std::getenv(kEnableTraceEnv_);
    if (enable_trace_env) {
      if (0 == strcmp(enable_trace_env, "true")) {
        enable_write_ = true;
      } else if (0 == strcmp(enable_trace_env, "false")) {
        enable_write_ = false;
      } else {
        AWARN << "Value for environment variable " << kEnableTraceEnv_
              << " is not valid(" << enable_trace_env
              << "), please use \"true\" or \"false\", fallback to default "
                 "value: "
              << std::boolalpha << kEnableWrite_;
      }
    } else {
      AWARN << "Environment variable " << kEnableTraceEnv_
            << " is not set, fallback to default value: " << std::boolalpha
            << kEnableWrite_;
    }

    if (!enable_write_) {
      return;
    }
    file_name_prefix_ =
        kFilePrefix_ + GetProcessName() + "-" + std::to_string(getpid());

    fs::path dir = fs::path(file_name_prefix_).parent_path();
    if (!PathExists(dir.c_str())) {
      AERROR << "cannot find directory " << dir;
      return;
    }
    if (!InitHeader()) {
      AERROR << "cannot init file header";
      return;
    }
    if (!InitWriter()) {
      AERROR << "cannot init trace writer";
      return;
    }
    stop_.store(false, std::memory_order_relaxed);
    dump_thread_ = cyber::Thread(kDumpThreadName_, [this]() { WriteTrace(); });
  }

  template <typename V>
  void Record(V&& data) {
    if (!enable_write_) {
      return;
    }
    thread_local pid_t tid = syscall(SYS_gettid);
    thread_local bool vec_exist = false;
    if (!vec_exist) {
      while (pool_occupied_.test_and_set(std::memory_order_acquire)) {
      }
      std::vector<std::remove_reference_t<V>> vec;
      vec.reserve(kBatchSize_);
      vec_pool_[tid] = std::move(vec);
      vec_exist = true;
      pool_occupied_.clear(std::memory_order_release);
    }

    if (!stop_.load(std::memory_order_acquire)) {
      thread_local bool vec_first_read = true;
      thread_local std::vector<std::remove_reference_t<V>>* batch;
      if (vec_first_read) {
        while (pool_occupied_.test_and_set(std::memory_order_acquire)) {
        }
        batch = &vec_pool_[tid];
        vec_first_read = false;
        pool_occupied_.clear(std::memory_order_release);
      }
      batch->emplace_back(std::forward<V>(data));
      if (batch->size() >= kBatchSize_) {
        queue_.enqueue(*batch);
        batch->clear();
      }
    }
  }

  void Stop() {
    if (!enable_write_) {
      return;
    }
    stop_.store(true, std::memory_order_release);
    FlushAtStop();
    if (dump_thread_.Joinable()) {
      dump_thread_.Join();
    }
  }

 private:
  bool InitWriter() {
    static uint8_t file_num = 0;
    if (file_num >= kMaxFileNumber_) {
      std::string delete_file_name =
          file_name_prefix_ + "." + std::to_string(file_num - kMaxFileNumber_);
      if (0 != std::remove(delete_file_name.c_str())) {
        AERROR << "cannot delete file " << delete_file_name;
      }
    }
    std::string current_file_name =
        file_name_prefix_ + "." + std::to_string(file_num++);
    if (writer_.is_open()) {
      writer_.close();
    }
    writer_ =
        std::ofstream(current_file_name, std::ios::out | std::ios::binary);
    if (!writer_) {
      return false;
    }
    WriteHeader();
    return true;
  }

  bool InitHeader() {
    std::ifstream inputFile(kHeaderFile_);
    if (!inputFile) {
      AERROR << "Error opening input file.";
      return false;
    }
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    base_monotonic_ = Timespec2Nanoseconds(ts);
    std::chrono::system_clock::time_point now =
        std::chrono::system_clock::now();
    base_system_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       now.time_since_epoch())
                       .count();

    fileContent_ = std::string((std::istreambuf_iterator<char>(inputFile)),
                               (std::istreambuf_iterator<char>()));
    inputFile.close();
    return true;
  }

  void WriteHeader() {
    writer_.write(reinterpret_cast<const char*>(&base_monotonic_),
                  sizeof(base_monotonic_));
    writer_.write(reinterpret_cast<const char*>(&base_system_),
                  sizeof(base_system_));
    writer_.write(fileContent_.c_str(), fileContent_.size());
    writer_.seekp(kHeaderSize_);
  }

  void WriteTrace() {
    std::vector<U> batch;
    batch.reserve(kBatchSize_);
    auto dequeue_write = [this](auto& batch) {
      batch.clear();
      while (queue_.try_dequeue(batch)) {
        WriteBatch(batch);
      }
    };

    while (!stop_.load(std::memory_order_acquire)) {
      dequeue_write(batch);
      std::this_thread::sleep_for(std::chrono::milliseconds(kSleepMs));
    }
    writer_.flush();
  }

  void WriteBatch(const std::vector<U>& batch) {
    static uint64_t log_num = 0;
    if (log_num >= kMaxLogNum_) {
      log_num = 0;
      if (!InitWriter()) {
        AERROR << "cannot init trace writer";
        return;
      }
    }
    writer_.write(reinterpret_cast<const char*>(batch.data()),
                  batch.size() * sizeof(U));
    log_num += batch.size();
  }

  void FlushAtStop() {
    for (auto it = vec_pool_.begin(); it != vec_pool_.end(); ++it) {
      WriteBatch(it->second);
    }
  }

 private:
  std::atomic_bool stop_{true};
  std::atomic_flag pool_occupied_{ATOMIC_FLAG_INIT};
  uint64_t base_monotonic_;
  uint64_t base_system_;
  moodycamel::ConcurrentQueue<std::vector<U>> queue_;
  std::unordered_map<pid_t, std::vector<U>> vec_pool_;
  cyber::Thread dump_thread_;
  const char* kDumpThreadName_ = "trace_dump";
  const int kSleepMs = 10;
  std::string file_name_prefix_;
  std::string fileContent_;
  std::ofstream writer_;
  static constexpr const char* kFilePrefix_ = Get_kFilePrefix<T>::value;
  static constexpr uint8_t kMaxFileNumber_ = Get_kMaxFileNumber<T>::value;
  static constexpr uint64_t kMaxLogSize_ = Get_kMaxLogSize<T>::value;
  static constexpr bool kEnableWrite_ = Get_kEnableWrite<T>::value;
  static constexpr uint64_t kBatchSize_ = Get_kBatchSize<T>::value;
  static constexpr uint64_t kHeaderSize_ = Get_kHeaderSize<T>::value;
  static constexpr uint64_t kMaxLogNum_ =
      (kMaxLogSize_ - kHeaderSize_) / sizeof(U);
  static constexpr const char* kHeaderFile_ =
      "/century/cyber/trace/trace_enums.h";
  static constexpr const char* kEnableTraceEnv_ = "ENABLE_TRACE";
  bool enable_write_ = kEnableWrite_;
};
}  // namespace trace
}  // namespace century
