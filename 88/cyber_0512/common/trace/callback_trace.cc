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
#include <unistd.h>

#include <cxxabi.h>
#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "cyber/common/log.h"

namespace century {
namespace cyber {
namespace common {
namespace {

std::string ToUpper(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) { return std::toupper(ch); });
  return value;
}

std::vector<std::string> ProcessArgs() {
  std::ifstream cmdline("/proc/self/cmdline", std::ios::in | std::ios::binary);
  std::vector<std::string> args;
  std::string content((std::istreambuf_iterator<char>(cmdline)),
                      std::istreambuf_iterator<char>());
  std::string current;
  for (char ch : content) {
    if (ch == '\0') {
      if (!current.empty()) {
        args.emplace_back(current);
        current.clear();
      }
    } else {
      current.push_back(ch);
    }
  }
  if (!current.empty()) {
    args.emplace_back(current);
  }
  return args;
}

std::string ReadConfigValue(const std::string& name) {
  if (const char* value = std::getenv(name.c_str())) {
    return value;
  }
  const auto upper_name = ToUpper(name);
  if (const char* value = std::getenv(upper_name.c_str())) {
    return value;
  }

  static const std::vector<std::string> args = ProcessArgs();
  const auto flag_prefix = "--" + name + "=";
  const auto bool_flag = "--" + name;
  const auto no_bool_flag = "--no" + name;
  std::string value;
  for (size_t index = 0; index < args.size(); ++index) {
    const auto& arg = args[index];
    if (arg.compare(0, flag_prefix.size(), flag_prefix) == 0) {
      value = arg.substr(flag_prefix.size());
    } else if (arg == bool_flag) {
      if (index + 1 < args.size() && args[index + 1].compare(0, 2, "--") != 0) {
        value = args[index + 1];
      } else {
        value = "true";
      }
    } else if (arg == no_bool_flag) {
      value = "false";
    }
  }
  return value;
}

bool ParseBoolConfig(const std::string& value, bool default_value) {
  if (value.empty()) {
    return default_value;
  }
  auto normalized = ToUpper(value);
  if (normalized == "1" || normalized == "TRUE" || normalized == "T" ||
      normalized == "YES" || normalized == "Y" || normalized == "ON") {
    return true;
  }
  if (normalized == "0" || normalized == "FALSE" || normalized == "F" ||
      normalized == "NO" || normalized == "N" || normalized == "OFF") {
    return false;
  }
  return default_value;
}

int64_t ParseInt64Config(const std::string& value, int64_t default_value) {
  if (value.empty()) {
    return default_value;
  }
  char* end = nullptr;
  const int64_t parsed = std::strtoll(value.c_str(), &end, 10);
  if (end == value.c_str() || *end != '\0') {
    return default_value;
  }
  return parsed;
}

double ParseDoubleConfig(const std::string& value, double default_value) {
  if (value.empty()) {
    return default_value;
  }
  char* end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0' || !std::isfinite(parsed)) {
    return default_value;
  }
  return parsed;
}

bool EnableCallbackTraceConfig() {
  static const bool value =
      ParseBoolConfig(ReadConfigValue("enable_callback_trace"), true);
  return value;
}

int64_t CallbackTraceThresholdMs() {
  static const int64_t value =
      ParseInt64Config(ReadConfigValue("callback_trace_threshold_ms"), 50);
  return value;
}

int64_t CallbackTraceSnapshotIntervalMs() {
  static const int64_t value = ParseInt64Config(
      ReadConfigValue("callback_trace_snapshot_interval_ms"), 0);
  return value;
}

int64_t CallbackTraceAbnormalEmitIntervalMs() {
  static const int64_t value = ParseInt64Config(
      ReadConfigValue("callback_trace_abnormal_emit_interval_ms"), 0);
  return value;
}

int64_t CallbackTraceStatsIntervalSec() {
  static const int64_t value =
      ParseInt64Config(ReadConfigValue("callback_trace_stats_interval_sec"), 10);
  return value;
}

int64_t CallbackTraceWindowSampleLimit() {
  static const int64_t value = std::max<int64_t>(
      0, ParseInt64Config(
             ReadConfigValue("callback_trace_window_sample_limit"), 1024));
  return value;
}

double CallbackTraceAbnormalThresholdMultiplier() {
  static const double value = std::max(
      1.0, ParseDoubleConfig(
               ReadConfigValue("callback_trace_abnormal_threshold_multiplier"),
               2.0));
  return value;
}

int64_t CallbackTraceComponentMinThresholdMs() {
  static const int64_t value = ParseInt64Config(
      ReadConfigValue("callback_trace_component_min_threshold_ms"), 10);
  return value;
}

int64_t CallbackTraceTimerComponentMinThresholdMs() {
  static const int64_t value = ParseInt64Config(
      ReadConfigValue("callback_trace_timer_component_min_threshold_ms"), 10);
  return value;
}

int64_t CallbackTraceReaderMinThresholdMs() {
  static const int64_t value = ParseInt64Config(
      ReadConfigValue("callback_trace_reader_min_threshold_ms"), 1);
  return value;
}

int64_t CallbackTraceComponentHardLimitMs() {
  static const int64_t value = ParseInt64Config(
      ReadConfigValue("callback_trace_component_hard_limit_ms"), 200);
  return value;
}

int64_t CallbackTraceTimerComponentHardLimitMs() {
  static const int64_t value = ParseInt64Config(
      ReadConfigValue("callback_trace_timer_component_hard_limit_ms"), 200);
  return value;
}

int64_t CallbackTraceReaderHardLimitMs() {
  static const int64_t value = ParseInt64Config(
      ReadConfigValue("callback_trace_reader_hard_limit_ms"), 20);
  return value;
}

std::mutex g_snapshot_mutex;
std::chrono::steady_clock::time_point g_last_snapshot_time =
    std::chrono::steady_clock::time_point::min();

struct ThresholdDecision {
  int64_t threshold_ms = 0;
  std::string source = "flag";
};

struct RecordDecision {
  ThresholdDecision threshold;
  uint64_t window_count = 0;
  double window_avg_us = 0.0;
  uint64_t window_max_us = 0;
  uint64_t window_p95_us = 0;
  uint64_t abnormal_count = 0;
  bool is_abnormal = false;
  bool should_emit_file = false;
};

struct CallbackStat {
  std::string type;
  std::string name;
  std::string detail;
  uint64_t count = 0;
  uint64_t total_us = 0;
  uint64_t window_max_us = 0;
  std::vector<uint64_t> elapsed_samples_us;
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
    window_max_us = std::max(window_max_us, elapsed_us);

    const auto sample_limit = CallbackTraceWindowSampleLimit();
    if (sample_limit <= 0) {
      return;
    }
    const auto limit = static_cast<size_t>(sample_limit);
    if (elapsed_samples_us.size() < limit) {
      elapsed_samples_us.emplace_back(elapsed_us);
    } else {
      elapsed_samples_us[(count - 1) % limit] = elapsed_us;
    }
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
    window_max_us = 0;
    elapsed_samples_us.clear();
    abnormal_count = 0;
    abnormal_max_us = 0;
    abnormal_last_us = 0;
    abnormal_threshold_ms = 0;
    abnormal_threshold_source.clear();
  }
};

struct StatsShard {
  std::mutex mutex;
  std::unordered_map<std::string, CallbackStat> callback_stats;
  std::chrono::steady_clock::time_point last_stats_output_time =
      std::chrono::steady_clock::time_point::min();
};

constexpr size_t kStatsShardCount = 64;
std::array<StatsShard, kStatsShardCount> g_stats_shards;

double UsToMs(uint64_t us) {
  return static_cast<double>(us) / 1000.0;
}

double UsToMs(double us) {
  return us / 1000.0;
}

uint64_t PercentileUs(const std::vector<uint64_t>& samples,
                      double percentile) {
  if (samples.empty()) {
    return 0;
  }

  auto values = samples;
  const auto clamped_percentile = std::max(0.0, std::min(1.0, percentile));
  size_t index = static_cast<size_t>(
      std::ceil(clamped_percentile * static_cast<double>(values.size())));
  if (index == 0) {
    index = 1;
  }
  --index;
  index = std::min(index, values.size() - 1);

  auto nth = values.begin() + static_cast<std::ptrdiff_t>(index);
  std::nth_element(values.begin(), nth, values.end());
  return *nth;
}

std::string AbnormalThresholdSource() {
  std::ostringstream ss;
  ss << "avg_x" << CallbackTraceAbnormalThresholdMultiplier();
  return ss.str();
}

std::string PrevWindowAbnormalThresholdSource() {
  std::ostringstream ss;
  ss << "prev_window_avg_x" << CallbackTraceAbnormalThresholdMultiplier();
  return ss.str();
}

int64_t MinThresholdMs(const std::string& type) {
  if (type == "component") {
    return CallbackTraceComponentMinThresholdMs();
  }
  if (type == "timer_component") {
    return CallbackTraceTimerComponentMinThresholdMs();
  }
  if (type == "reader") {
    return CallbackTraceReaderMinThresholdMs();
  }
  return CallbackTraceThresholdMs();
}

int64_t HardLimitMs(const std::string& type) {
  if (type == "component") {
    return CallbackTraceComponentHardLimitMs();
  }
  if (type == "timer_component") {
    return CallbackTraceTimerComponentHardLimitMs();
  }
  if (type == "reader") {
    return CallbackTraceReaderHardLimitMs();
  }
  return CallbackTraceThresholdMs();
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
  threshold_ms = UsToMs(static_cast<uint64_t>(std::ceil(
      avg_us * CallbackTraceAbnormalThresholdMultiplier())));
  decision.source = AbnormalThresholdSource();

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
                          ? PrevWindowAbnormalThresholdSource()
                          : stat.learned_threshold_source;
    return decision;
  }

  decision.threshold_ms = min_threshold_ms;
  decision.source = "min_threshold";
  return decision;
}

std::string CallbackStatsKey(const std::string& type, const std::string& name,
                             const std::string& detail) {
  return type + "|" + name + "|" + detail;
}

StatsShard& StatsShardForKey(const std::string& key) {
  const auto shard_index = std::hash<std::string>{}(key) % kStatsShardCount;
  return g_stats_shards[shard_index];
}

std::string FormatCallbackAbnormal(const std::string& type,
                                   const std::string& name,
                                   const std::string& detail,
                                   uint64_t elapsed_us,
                                   uint64_t thread_cpu_us,
                                   const ThresholdDecision& threshold,
                                   uint64_t window_count,
                                   double window_avg_us,
                                   uint64_t window_max_us,
                                   uint64_t window_p95_us,
                                   uint64_t abnormal_count) {
  std::ostringstream ss;
  ss << "[callback_abnormal]"
     << " type=" << type << " name=" << name;
  if (!detail.empty()) {
    ss << " detail=" << detail;
  }
  ss << " elapsed_ms=" << UsToMs(elapsed_us)
     << " thread_cpu_ms=" << UsToMs(thread_cpu_us)
     << " threshold_ms=" << threshold.threshold_ms
     << " threshold_source=" << threshold.source
     << " window_count=" << window_count
     << " window_avg_ms=" << UsToMs(window_avg_us)
     << " window_max_ms=" << UsToMs(window_max_us)
     << " window_p95_ms=" << UsToMs(window_p95_us)
     << " abnormal_count=" << abnormal_count;
  return ss.str();
}

std::string FormatResourceSnapshotLine(const std::string& type,
                                       const std::string& name,
                                       const std::string& detail,
                                       const std::string& snapshot) {
  std::ostringstream ss;
  ss << "[callback_resource_snapshot]"
     << " type=" << type << " name=" << name;
  if (!detail.empty()) {
    ss << " detail=" << detail;
  }
  ss << snapshot;
  return ss.str();
}

void AppendCallbackAbnormals(const std::vector<std::string>& lines) {
  if (lines.empty()) {
    return;
  }

  for (const auto& line : lines) {
    ALOG_MODULE("callback_trace", INFO) << line;
  }
}

RecordDecision RecordCallbackStat(const std::string& type,
                                  const std::string& name,
                                  const std::string& detail,
                                  uint64_t elapsed_us,
                                  uint64_t thread_cpu_us) {
  RecordDecision result;
  const auto now = std::chrono::steady_clock::now();
  const auto stats_interval_sec = CallbackTraceStatsIntervalSec();
  const auto interval = std::chrono::seconds(stats_interval_sec);
  const auto emit_interval_ms = CallbackTraceAbnormalEmitIntervalMs();
  const auto emit_interval = std::chrono::milliseconds(emit_interval_ms);
  const auto stats_key = CallbackStatsKey(type, name, detail);
  auto& shard = StatsShardForKey(stats_key);

  {
    std::lock_guard<std::mutex> lock(shard.mutex);
    if (shard.last_stats_output_time ==
        std::chrono::steady_clock::time_point::min()) {
      shard.last_stats_output_time = now;
    } else if (stats_interval_sec > 0 &&
               now - shard.last_stats_output_time >= interval) {
      shard.last_stats_output_time = now;

      for (auto& item : shard.callback_stats) {
        auto& callback_stat = item.second;
        if (callback_stat.count > 0) {
          auto learned = ComputeThresholdDecision(callback_stat);
          callback_stat.learned_threshold_ms = learned.threshold_ms;
          callback_stat.learned_threshold_source =
              PrevWindowAbnormalThresholdSource();
        }
        callback_stat.Reset();
      }
    }

    auto& stat = shard.callback_stats[stats_key];
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
      if (emit_interval_ms <= 0 ||
          stat.last_abnormal_emit_time ==
              std::chrono::steady_clock::time_point::min() ||
          now - stat.last_abnormal_emit_time >= emit_interval) {
        result.should_emit_file = true;
        stat.last_abnormal_emit_time = now;
      }
    }
    stat.Update(elapsed_us);
    if (result.is_abnormal) {
      result.window_count = stat.count;
      result.window_avg_us =
          stat.count == 0 ? 0.0
                          : static_cast<double>(stat.total_us) /
                                static_cast<double>(stat.count);
      result.window_max_us = stat.window_max_us;
      result.window_p95_us = PercentileUs(stat.elapsed_samples_us, 0.95);
      result.abnormal_count = stat.abnormal_count;
    }
  }

  if (result.should_emit_file) {
    AppendCallbackAbnormals(
        {FormatCallbackAbnormal(type, name, detail, elapsed_us, thread_cpu_us,
                                result.threshold, result.window_count,
                                result.window_avg_us, result.window_max_us,
                                result.window_p95_us,
                                result.abnormal_count)});
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

uint64_t ReadMemInfoKb(const std::string& key) {
  std::ifstream meminfo("/proc/meminfo");
  std::string label;
  uint64_t value = 0;
  std::string unit;
  while (meminfo >> label >> value >> unit) {
    if (label == key) {
      return value;
    }
  }
  return 0;
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
    const auto mem_available_kb = ReadMemInfoKb("MemAvailable:");
    const auto mem_available_mb = mem_available_kb / 1024;
    const auto mem_used_base_mb =
        mem_available_mb > 0 ? mem_available_mb : free_mb;
    const double mem_used_percent =
        total_mb == 0
            ? 0.0
            : (1.0 - static_cast<double>(mem_used_base_mb) /
                         static_cast<double>(total_mb)) *
                  100.0;
    const double swap_used_percent =
        info.totalswap == 0
            ? 0.0
            : (1.0 - static_cast<double>(info.freeswap) /
                         static_cast<double>(info.totalswap)) *
                  100.0;
    ss << " loadavg=" << info.loads[0] / 65536.0 << ","
       << info.loads[1] / 65536.0 << "," << info.loads[2] / 65536.0
       << " mem_free_mb=" << free_mb << "/" << total_mb
       << " mem_available_mb=" << mem_available_mb
       << " mem_used_percent=" << mem_used_percent
       << " swap_free_mb=" << swap_free_mb << "/" << swap_total_mb
       << " swap_used_percent=" << swap_used_percent;
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
     << " fds=" << CountOpenFds();

  return ss.str();
}

bool ShouldPrintSnapshot() {
  const auto snapshot_interval_ms = CallbackTraceSnapshotIntervalMs();
  if (snapshot_interval_ms < 0) {
    return false;
  }
  if (snapshot_interval_ms == 0) {
    return true;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto interval = std::chrono::milliseconds(snapshot_interval_ms);

  std::lock_guard<std::mutex> lock(g_snapshot_mutex);
  if (g_last_snapshot_time == std::chrono::steady_clock::time_point::min() ||
      now - g_last_snapshot_time >= interval) {
    g_last_snapshot_time = now;
    return true;
  }
  return false;
}

}  // namespace

void RecordCallbackTraceEnd(const char* type, const char* name,
                            const char* detail, uint64_t elapsed_us,
                            uint64_t thread_cpu_us) {
  const std::string type_text = type == nullptr ? "" : type;
  const std::string name_text = name == nullptr ? "" : name;
  const std::string detail_text = detail == nullptr ? "" : detail;
  RecordDecision record_decision =
      RecordCallbackStat(type_text, name_text, detail_text, elapsed_us,
                         thread_cpu_us);
  if (!record_decision.is_abnormal) {
    return;
  }

  if (record_decision.should_emit_file && ShouldPrintSnapshot()) {
    const auto snapshot = BuildResourceSnapshot();
    AppendCallbackAbnormals({FormatResourceSnapshotLine(
        type_text, name_text, detail_text, snapshot)});
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

bool CallbackTraceEnabled() { return EnableCallbackTraceConfig(); }

bool CallbackTraceShouldTrace(const char* type, const char* name) {
  (void)type;
  (void)name;
  return CallbackTraceEnabled();
}

}  // namespace common
}  // namespace cyber
}  // namespace century
