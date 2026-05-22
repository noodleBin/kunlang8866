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

#include "cyber/common/trace/callback_trace.h"

#include <dirent.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <sys/sysinfo.h>
#include <time.h>
#include <unistd.h>

#include <cxxabi.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "gflags/gflags.h"

#include "cyber/common/log.h"

DEFINE_bool(enable_callback_trace, false,
            "Enable cyber callback latency trace.");
DEFINE_int64(callback_trace_threshold_ms, 100,
             "Cyber callback trace warning threshold in milliseconds.");
DEFINE_int64(callback_trace_snapshot_interval_ms, 1000,
             "Minimum interval between callback resource snapshots.");
DEFINE_int64(callback_trace_stats_interval_sec, 10,
             "Callback threshold learning window interval in seconds.");
DEFINE_string(callback_trace_stats_file,
              "/century/data/log/callback_trace_stats.log",
              "Callback abnormal output file. If empty, use "
              "FLAGS_log_dir/callback_trace_stats.log.");
DEFINE_int64(callback_trace_component_min_threshold_ms, 20,
             "Minimum learned threshold for component callbacks.");
DEFINE_int64(callback_trace_timer_component_min_threshold_ms, 20,
             "Minimum learned threshold for timer_component callbacks.");
DEFINE_int64(callback_trace_reader_min_threshold_ms, 1,
             "Minimum learned threshold for reader callbacks.");
DEFINE_int64(callback_trace_component_hard_limit_ms, 500,
             "Hard upper limit for learned component callback thresholds.");
DEFINE_int64(callback_trace_timer_component_hard_limit_ms, 500,
             "Hard upper limit for learniiied timer_component callback thresholds.");
DEFINE_int64(callback_trace_reader_hard_limit_ms, 50,
             "Hard upper limit for learned reader callback thresholds.");

namespace century {
namespace cyber {
namespace common {
namespace {

std::mutex g_snapshot_mutex;
std::chrono::steady_clock::time_point g_last_snapshot_time =
    std::chrono::steady_clock::time_point::min();

struct ThresholdDecision {
  int64_t threshold_ms = 0;
  std::string source = "flag";
};

struct RecordDecision {
  ThresholdDecision threshold;
  bool is_abnormal = false;
  bool should_emit_file = false;
};

struct CallbackStat {
  std::string type;
  std::string name;
  std::string detail;
  uint64_t count = 0;
  uint64_t total_us = 0;
  int64_t learned_threshold_ms = 0;
  std::string learned_threshold_source;
  uint64_t abnormal_count = 0;
  uint64_t abnormal_max_us = 0;
  uint64_t abnormal_last_us = 0;
  int64_t abnormal_threshold_ms = 0;
  std::string abnormal_threshold_source;
  std::chrono::steady_clock::time_point last_abnormal_emit_time =
      std::chrono::steady_clock::time_point::min();

  void Update(uint64_t elapsed_us) {
    ++count;
    total_us += elapsed_us;
  }

  void MarkAbnormal(uint64_t elapsed_us, const ThresholdDecision& threshold) {
    ++abnormal_count;
    abnormal_max_us = std::max(abnormal_max_us, elapsed_us);
    abnormal_last_us = elapsed_us;
    abnormal_threshold_ms = threshold.threshold_ms;
    abnormal_threshold_source = threshold.source;
  }

  void Reset() {
    count = 0;
    total_us = 0;
    abnormal_count = 0;
    abnormal_max_us = 0;
    abnormal_last_us = 0;
    abnormal_threshold_ms = 0;
    abnormal_threshold_source.clear();
  }
};

std::mutex g_stats_mutex;
std::unordered_map<std::string, CallbackStat> g_callback_stats;
std::chrono::steady_clock::time_point g_last_stats_output_time =
    std::chrono::steady_clock::time_point::min();

double UsToMs(uint64_t us) {
  return static_cast<double>(us) / 1000.0;
}

int64_t MinThresholdMs(const std::string& type) {
  if (type == "component") {
    return FLAGS_callback_trace_component_min_threshold_ms;
  }
  if (type == "timer_component") {
    return FLAGS_callback_trace_timer_component_min_threshold_ms;
  }
  if (type == "reader") {
    return FLAGS_callback_trace_reader_min_threshold_ms;
  }
  return FLAGS_callback_trace_threshold_ms;
}

int64_t HardLimitMs(const std::string& type) {
  if (type == "component") {
    return FLAGS_callback_trace_component_hard_limit_ms;
  }
  if (type == "timer_component") {
    return FLAGS_callback_trace_timer_component_hard_limit_ms;
  }
  if (type == "reader") {
    return FLAGS_callback_trace_reader_hard_limit_ms;
  }
  return FLAGS_callback_trace_threshold_ms;
}

ThresholdDecision ComputeThresholdDecision(const CallbackStat& stat) {
  ThresholdDecision decision;
  const auto min_threshold_ms = std::max<int64_t>(1, MinThresholdMs(stat.type));
  const auto hard_limit_ms = std::max<int64_t>(min_threshold_ms,
                                               HardLimitMs(stat.type));
  double threshold_ms = static_cast<double>(min_threshold_ms);

  if (stat.count == 0) {
    decision.threshold_ms = min_threshold_ms;
    decision.source = "min_threshold";
    return decision;
  }

  const double avg_us =
      static_cast<double>(stat.total_us) / static_cast<double>(stat.count);
  threshold_ms = UsToMs(static_cast<uint64_t>(std::ceil(avg_us * 2.0)));
  decision.source = "avg_x2";

  threshold_ms = std::max(threshold_ms, static_cast<double>(min_threshold_ms));
  threshold_ms = std::min(threshold_ms, static_cast<double>(hard_limit_ms));
  decision.threshold_ms = static_cast<int64_t>(std::ceil(threshold_ms));
  return decision;
}

ThresholdDecision RuntimeThresholdDecision(const CallbackStat& stat) {
  ThresholdDecision decision;
  const auto min_threshold_ms = std::max<int64_t>(1, MinThresholdMs(stat.type));

  if (stat.count < 5) {
    decision.threshold_ms = min_threshold_ms;
    decision.source = "min_threshold";
    return decision;
  }

  if (stat.learned_threshold_ms > 0) {
    decision.threshold_ms = stat.learned_threshold_ms;
    decision.source = stat.learned_threshold_source.empty()
                          ? "prev_window_avg_x2"
                          : stat.learned_threshold_source;
    return decision;
  }

  decision.threshold_ms = min_threshold_ms;
  decision.source = "min_threshold";
  return decision;
}

std::string CurrentTimeString() {
  std::time_t now = std::time(nullptr);
  char buffer[32] = {0};
  std::tm tm_now;
  localtime_r(&now, &tm_now);
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm_now);
  return buffer;
}

std::string CallbackStatsFile() {
  if (!FLAGS_callback_trace_stats_file.empty()) {
    return FLAGS_callback_trace_stats_file;
  }
  if (!FLAGS_log_dir.empty()) {
    return FLAGS_log_dir + "/callback_trace_stats.log";
  }
  return "callback_trace_stats.log";
}

std::string CallbackStatsKey(const std::string& type, const std::string& name,
                             const std::string& detail) {
  return type + "|" + name + "|" + detail;
}

std::string FormatCallbackAbnormal(const std::string& type,
                                   const std::string& name,
                                   const std::string& detail,
                                   uint64_t elapsed_us,
                                   uint64_t thread_cpu_us,
                                   const ThresholdDecision& threshold) {
  std::ostringstream ss;
  ss << "[callback_abnormal]"
     << " time=\"" << CurrentTimeString() << "\""
     << " type=" << type << " name=" << name;
  if (!detail.empty()) {
    ss << " detail=" << detail;
  }
  ss << " elapsed_ms=" << UsToMs(elapsed_us)
     << " thread_cpu_ms=" << UsToMs(thread_cpu_us)
     << " threshold_ms=" << threshold.threshold_ms
     << " threshold_source=" << threshold.source
     << " tid=" << static_cast<int>(syscall(SYS_gettid));
  return ss.str();
}

std::string FormatResourceSnapshotLine(const std::string& type,
                                       const std::string& name,
                                       const std::string& detail,
                                       const std::string& snapshot) {
  std::ostringstream ss;
  ss << "[callback_resource_snapshot]"
     << " time=\"" << CurrentTimeString() << "\""
     << " type=" << type << " name=" << name;
  if (!detail.empty()) {
    ss << " detail=" << detail;
  }
  ss << " tid=" << static_cast<int>(syscall(SYS_gettid)) << snapshot;
  return ss.str();
}

void AppendCallbackAbnormals(const std::vector<std::string>& lines) {
  if (lines.empty()) {
    return;
  }

  std::ofstream output(CallbackStatsFile(), std::ios::app);
  if (!output.is_open()) {
    return;
  }

  for (const auto& line : lines) {
    output << line << '\n';
  }
}

RecordDecision RecordCallbackStat(const std::string& type,
                                  const std::string& name,
                                  const std::string& detail,
                                  uint64_t elapsed_us,
                                  uint64_t thread_cpu_us) {
  RecordDecision result;
  const auto now = std::chrono::steady_clock::now();
  const auto interval =
      std::chrono::seconds(FLAGS_callback_trace_stats_interval_sec);
  const auto emit_interval = std::chrono::seconds(1);

  {
    std::lock_guard<std::mutex> lock(g_stats_mutex);
    if (g_last_stats_output_time ==
        std::chrono::steady_clock::time_point::min()) {
      g_last_stats_output_time = now;
    } else if (FLAGS_callback_trace_stats_interval_sec > 0 &&
               now - g_last_stats_output_time >= interval) {
      g_last_stats_output_time = now;

      for (auto& item : g_callback_stats) {
        auto& callback_stat = item.second;
        if (callback_stat.count > 0) {
          auto learned = ComputeThresholdDecision(callback_stat);
          callback_stat.learned_threshold_ms = learned.threshold_ms;
          callback_stat.learned_threshold_source = "prev_window_avg_x2";
        }
        callback_stat.Reset();
      }
    }

    auto& stat = g_callback_stats[CallbackStatsKey(type, name, detail)];
    if (stat.type.empty()) {
      stat.type = type;
      stat.name = name;
      stat.detail = detail;
    }
    result.threshold = RuntimeThresholdDecision(stat);
    if (result.threshold.threshold_ms > 0 &&
        elapsed_us >=
            static_cast<uint64_t>(result.threshold.threshold_ms) * 1000ULL) {
      result.is_abnormal = true;
      stat.MarkAbnormal(elapsed_us, result.threshold);
      if (stat.last_abnormal_emit_time ==
              std::chrono::steady_clock::time_point::min() ||
          now - stat.last_abnormal_emit_time >= emit_interval) {
        result.should_emit_file = true;
        stat.last_abnormal_emit_time = now;
      }
    }
    stat.Update(elapsed_us);
  }

  if (result.should_emit_file) {
    AppendCallbackAbnormals(
        {FormatCallbackAbnormal(type, name, detail, elapsed_us, thread_cpu_us,
                                result.threshold)});
  }
  return result;
}

std::string ReadStatusValue(const std::string& key) {
  std::ifstream status("/proc/self/status");
  std::string line;
  while (std::getline(status, line)) {
    if (line.compare(0, key.size(), key) == 0) {
      return line.substr(key.size());
    }
  }
  return " unknown";
}

int CountOpenFds() {
  DIR* dir = opendir("/proc/self/fd");
  if (dir == nullptr) {
    return -1;
  }

  int count = 0;
  while (readdir(dir) != nullptr) {
    ++count;
  }
  closedir(dir);
  return count >= 2 ? count - 2 : count;
}

double TimevalToSeconds(const timeval& time) {
  return static_cast<double>(time.tv_sec) +
         static_cast<double>(time.tv_usec) / 1000000.0;
}

std::string BuildResourceSnapshot() {
  std::ostringstream ss;

  struct sysinfo info;
  if (sysinfo(&info) == 0) {
    const auto mem_unit = static_cast<uint64_t>(info.mem_unit);
    const auto total_mb = info.totalram * mem_unit / 1024 / 1024;
    const auto free_mb = info.freeram * mem_unit / 1024 / 1024;
    const auto swap_total_mb = info.totalswap * mem_unit / 1024 / 1024;
    const auto swap_free_mb = info.freeswap * mem_unit / 1024 / 1024;
    ss << " loadavg=" << info.loads[0] / 65536.0 << ","
       << info.loads[1] / 65536.0 << "," << info.loads[2] / 65536.0
       << " mem_free_mb=" << free_mb << "/" << total_mb
       << " swap_free_mb=" << swap_free_mb << "/" << swap_total_mb;
  }

  struct rusage usage;
  if (getrusage(RUSAGE_SELF, &usage) == 0) {
    ss << " proc_cpu_s=" << TimevalToSeconds(usage.ru_utime) << "+"
       << TimevalToSeconds(usage.ru_stime)
       << " maxrss_kb=" << usage.ru_maxrss
       << " ctx_switch=" << usage.ru_nvcsw << "+"
       << usage.ru_nivcsw;
  }

  ss << " vmrss=" << ReadStatusValue("VmRSS:")
     << " vmsize=" << ReadStatusValue("VmSize:")
     << " threads=" << ReadStatusValue("Threads:")
     << " tid=" << static_cast<int>(syscall(SYS_gettid))
     << " fds=" << CountOpenFds();

  return ss.str();
}

bool ShouldPrintSnapshot() {
  const auto now = std::chrono::steady_clock::now();
  const auto interval =
      std::chrono::milliseconds(FLAGS_callback_trace_snapshot_interval_ms);

  std::lock_guard<std::mutex> lock(g_snapshot_mutex);
  if (g_last_snapshot_time == std::chrono::steady_clock::time_point::min() ||
      now - g_last_snapshot_time >= interval) {
    g_last_snapshot_time = now;
    return true;
  }
  return false;
}

}  // namespace

CallbackTrace::CallbackTrace(const std::string& type, const std::string& name,
                             int64_t threshold_ms)
    : type_(type), name_(name), threshold_ms_(threshold_ms) {}

CallbackTrace::CallbackTrace(const std::string& type, const std::string& name,
                             const std::string& detail,
                             int64_t threshold_ms)
    : type_(type),
      name_(name),
      detail_(detail),
      threshold_ms_(threshold_ms) {}

CallbackTrace::~CallbackTrace() { Finish(); }

bool CallbackTrace::Enabled() const { return CallbackTraceEnabled(); }

void CallbackTrace::OnTraceEnd(uint64_t elapsed_us, uint64_t thread_cpu_us) {
  RecordDecision record_decision =
      RecordCallbackStat(type_, name_, detail_, elapsed_us, thread_cpu_us);
  if (!record_decision.is_abnormal) {
    return;
  }

  if (record_decision.should_emit_file && ShouldPrintSnapshot()) {
    const auto snapshot = BuildResourceSnapshot();
    AppendCallbackAbnormals(
        {FormatResourceSnapshotLine(type_, name_, detail_, snapshot)});
  }
}

std::string DemangleTypeName(const char* type_name) {
  int status = 0;
  std::unique_ptr<char, void (*)(void*)> demangled(
      abi::__cxa_demangle(type_name, nullptr, nullptr, &status), std::free);
  if (status == 0 && demangled != nullptr) {
    return demangled.get();
  }
  return type_name == nullptr ? "" : type_name;
}

bool CallbackTraceEnabled() { return FLAGS_enable_callback_trace; }

}  // namespace common
}  // namespace cyber
}  // namespace century
