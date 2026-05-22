/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#include <sys/resource.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <google/protobuf/arena.h>

#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "modules/drivers/proto/pointcloud.pb.h"

namespace {

using century::cyber::Rate;
using century::cyber::Time;
using century::drivers::PointCloudPacked;

#pragma pack(push, 1)
struct PointPacked {
  float x;
  float y;
  float z;
  uint16_t intensity;
  uint16_t ring;
  double timestamp;
};
#pragma pack(pop)

static_assert(sizeof(PointPacked) == 24, "PointPacked layout changed.");

struct Options {
  std::string mode = "both";
  std::string group = "both";
  int topics = 6;
  int points = 57600;
  int bytes = 0;
  double hz = 10.0;
  int seconds = 0;
};

struct Stats {
  std::mutex mutex;
  uint64_t count = 0;
  uint64_t bytes = 0;
  double latency_sum_us = 0.0;
  double latency_min_us = std::numeric_limits<double>::max();
  double latency_max_us = 0.0;

  uint64_t last_count = 0;
  uint64_t last_bytes = 0;
  double last_latency_sum_us = 0.0;
};

struct PublisherTarget {
  std::shared_ptr<century::cyber::Writer<PointCloudPacked>> writer;
  bool use_arena = false;
};

double ProcessCpuSeconds() {
  rusage usage;
  if (getrusage(RUSAGE_SELF, &usage) != 0) {
    return 0.0;
  }
  const double user = usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6;
  const double sys = usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
  return user + sys;
}

std::string GetArgValue(int argc, char** argv, const std::string& key,
                        const std::string& default_value) {
  const std::string prefix = "--" + key + "=";
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg.find(prefix) == 0) {
      return arg.substr(prefix.size());
    }
  }
  return default_value;
}

bool HasGroup(const std::string& selected, const std::string& group) {
  return selected == "both" || selected == group;
}

std::vector<std::string> BuildTopics(const std::string& group, int count) {
  std::vector<std::string> topics;
  topics.reserve(count);
  for (int i = 0; i < count; ++i) {
    topics.emplace_back("/century/benchmark/packed_cloud/" + group + "_" +
                        std::to_string(i));
  }
  return topics;
}

Options ParseOptions(int argc, char** argv) {
  Options options;
  if (argc > 1 && std::string(argv[1]).find("--") != 0) {
    options.mode = argv[1];
  }
  options.group = GetArgValue(argc, argv, "group", options.group);
  options.topics = std::atoi(GetArgValue(argc, argv, "topics", "6").c_str());
  options.points = std::atoi(GetArgValue(argc, argv, "points", "57600").c_str());
  options.bytes = std::atoi(GetArgValue(argc, argv, "bytes", "0").c_str());
  options.hz = std::atof(GetArgValue(argc, argv, "hz", "10").c_str());
  options.seconds =
      std::atoi(GetArgValue(argc, argv, "seconds", "0").c_str());
  options.topics = std::max(1, options.topics);
  options.points = std::max(1, options.points);
  options.bytes = std::max(0, options.bytes);
  options.hz = std::max(0.1, options.hz);
  return options;
}

size_t DataSize(const Options& options) {
  return options.bytes > 0
             ? static_cast<size_t>(options.bytes)
             : static_cast<size_t>(options.points) * sizeof(PointPacked);
}

void FillPointData(std::string* data, size_t data_bytes) {
  data->resize(data_bytes);
  if (data_bytes == 0) {
    return;
  }

  PointPacked point;
  point.x = 1.0f;
  point.y = 2.0f;
  point.z = 3.0f;
  point.intensity = 100;
  point.ring = 7;
  point.timestamp = 12345.6789;

  char* dst = &(*data)[0];
  const size_t first = std::min(data_bytes, sizeof(PointPacked));
  std::memcpy(dst, &point, first);
  size_t filled = first;
  while (filled < data_bytes) {
    const size_t copy_size = std::min(filled, data_bytes - filled);
    std::memcpy(dst + filled, dst, copy_size);
    filled += copy_size;
  }
}

void FillMessage(PointCloudPacked* msg, uint32_t seq, size_t data_bytes) {
  const uint64_t now_ns = Time::Now().ToNanosecond();
  msg->mutable_header()->set_sequence_num(seq);
  msg->mutable_header()->set_timestamp_sec(now_ns / 1e9);
  msg->mutable_header()->set_lidar_timestamp(now_ns);
  msg->mutable_header()->set_module_name("packed_cloud_perf");
  msg->set_frame_id("benchmark_lidar");
  msg->set_measuretime(now_ns / 1e9);
  msg->set_width(data_bytes / sizeof(PointPacked));
  msg->set_height(1);
  msg->set_is_dense(true);
  msg->set_point_size(data_bytes / sizeof(PointPacked));

  FillPointData(msg->mutable_data(), data_bytes);
}

void PrintUsage() {
  std::cerr
      << "Usage:\n"
      << "  packed_cloud_perf pub [--group=arena|shm|both] [--topics=6]"
      << " [--points=57600] [--bytes=0] [--hz=10] [--seconds=0]\n"
      << "  packed_cloud_perf sub [--group=arena|shm|both] [--topics=6]"
      << " [--seconds=0]\n"
      << "  packed_cloud_perf rawsub [--group=arena|shm|both] [--topics=6]"
      << " [--seconds=0]\n"
      << "Run pub/sub as different processes to benchmark SHM/arena_shm.\n";
}

void RunPublisher(const Options& options) {
  auto node = century::cyber::CreateNode("packed_cloud_perf_pub");
  std::vector<PublisherTarget> writers;

  if (HasGroup(options.group, "arena")) {
    for (const auto& topic : BuildTopics("arena", options.topics)) {
      writers.push_back({node->CreateWriter<PointCloudPacked>(topic), true});
    }
  }
  if (HasGroup(options.group, "shm")) {
    for (const auto& topic : BuildTopics("shm", options.topics)) {
      writers.push_back({node->CreateWriter<PointCloudPacked>(topic), false});
    }
  }

  const size_t data_bytes = DataSize(options);

  Rate rate(options.hz);
  uint32_t seq = 0;
  const auto start = std::chrono::steady_clock::now();
  auto last = start;
  uint64_t sent_since_last = 0;
  uint64_t bytes_since_last = 0;
  double last_cpu = ProcessCpuSeconds();
  while (century::cyber::OK()) {
    if (options.seconds > 0) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::steady_clock::now() - start);
      if (elapsed.count() >= options.seconds) {
        break;
      }
    }
    std::shared_ptr<PointCloudPacked> shm_msg;
    for (auto& target : writers) {
      std::shared_ptr<PointCloudPacked> msg;
      if (target.use_arena) {
        msg = target.writer->AcquireMessage();
      } else {
        if (shm_msg == nullptr) {
          shm_msg = std::make_shared<PointCloudPacked>();
          FillMessage(shm_msg.get(), seq++, data_bytes);
        }
        msg = shm_msg;
      }
      if (msg == nullptr) {
        continue;
      }
      if (target.use_arena) {
        FillMessage(msg.get(), seq++, data_bytes);
      }
      const uint64_t msg_bytes = static_cast<uint64_t>(data_bytes);
      target.writer->Write(msg);
      ++sent_since_last;
      bytes_since_last += msg_bytes;
    }

    const auto now = std::chrono::steady_clock::now();
    const double wall =
        std::chrono::duration_cast<std::chrono::duration<double>>(now - last)
            .count();
    if (wall >= 1.0) {
      const double cpu = ProcessCpuSeconds();
      const double cpu_percent =
          wall > 0.0 ? (cpu - last_cpu) / wall * 100.0 : 0.0;
      const double mbps = bytes_since_last * 8.0 / wall / 1000.0 / 1000.0;
      std::cout << std::fixed << std::setprecision(2)
                << "pub sent=" << sent_since_last << "/s"
                << " bandwidth=" << mbps << "Mbps"
                << " proc_cpu=" << cpu_percent << "%\n";
      std::cout.flush();
      sent_since_last = 0;
      bytes_since_last = 0;
      last = now;
      last_cpu = cpu;
    }
    rate.Sleep();
  }
}

void UpdateStats(Stats* stats, const std::shared_ptr<PointCloudPacked>& msg) {
  const uint64_t now_ns = Time::Now().ToNanosecond();
  const uint64_t send_ns = msg->header().lidar_timestamp();
  const double latency_us =
      send_ns > 0 && now_ns >= send_ns ? (now_ns - send_ns) / 1000.0 : 0.0;
  const uint64_t bytes = static_cast<uint64_t>(msg->ByteSizeLong());

  std::lock_guard<std::mutex> lock(stats->mutex);
  ++stats->count;
  stats->bytes += bytes;
  stats->latency_sum_us += latency_us;
  stats->latency_min_us = std::min(stats->latency_min_us, latency_us);
  stats->latency_max_us = std::max(stats->latency_max_us, latency_us);
}

void PrintStatsLine(const std::string& name, Stats* stats, double cpu_percent) {
  std::lock_guard<std::mutex> lock(stats->mutex);
  const uint64_t delta_count = stats->count - stats->last_count;
  const uint64_t delta_bytes = stats->bytes - stats->last_bytes;
  const double delta_latency = stats->latency_sum_us - stats->last_latency_sum_us;
  const double avg_us = delta_count == 0 ? 0.0 : delta_latency / delta_count;
  const double mbps = delta_bytes * 8.0 / 1000.0 / 1000.0;
  std::cout << std::fixed << std::setprecision(2)
            << name << " recv=" << delta_count << "/s"
            << " bandwidth=" << mbps << "Mbps"
            << " avg_latency=" << avg_us << "us"
            << " min_latency="
            << (stats->latency_min_us == std::numeric_limits<double>::max()
                    ? 0.0
                    : stats->latency_min_us)
            << "us max_latency=" << stats->latency_max_us
            << "us proc_cpu=" << cpu_percent << "%\n";
  stats->last_count = stats->count;
  stats->last_bytes = stats->bytes;
  stats->last_latency_sum_us = stats->latency_sum_us;
  stats->latency_min_us = std::numeric_limits<double>::max();
  stats->latency_max_us = 0.0;
}

void RunSubscriber(const Options& options) {
  auto node = century::cyber::CreateNode("packed_cloud_perf_sub");
  Stats arena_stats;
  Stats shm_stats;
  std::vector<std::shared_ptr<century::cyber::Reader<PointCloudPacked>>> readers;

  if (HasGroup(options.group, "arena")) {
    for (const auto& topic : BuildTopics("arena", options.topics)) {
      readers.emplace_back(node->CreateReader<PointCloudPacked>(
          topic, [&arena_stats](const std::shared_ptr<PointCloudPacked>& msg) {
            UpdateStats(&arena_stats, msg);
          }));
    }
  }
  if (HasGroup(options.group, "shm")) {
    for (const auto& topic : BuildTopics("shm", options.topics)) {
      readers.emplace_back(node->CreateReader<PointCloudPacked>(
          topic, [&shm_stats](const std::shared_ptr<PointCloudPacked>& msg) {
            UpdateStats(&shm_stats, msg);
          }));
    }
  }

  auto start = std::chrono::steady_clock::now();
  auto last = start;
  double last_cpu = ProcessCpuSeconds();
  while (century::cyber::OK()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const auto now = std::chrono::steady_clock::now();
    const double wall =
        std::chrono::duration_cast<std::chrono::duration<double>>(now - last)
            .count();
    const double cpu = ProcessCpuSeconds();
    const double cpu_percent = wall > 0.0 ? (cpu - last_cpu) / wall * 100.0 : 0.0;

    if (HasGroup(options.group, "arena")) {
      PrintStatsLine("arena_shm", &arena_stats, cpu_percent);
    }
    if (HasGroup(options.group, "shm")) {
      PrintStatsLine("shm      ", &shm_stats, cpu_percent);
    }
    std::cout.flush();
    last = now;
    last_cpu = cpu;

    if (options.seconds > 0) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
          now - start);
      if (elapsed.count() >= options.seconds) {
        break;
      }
    }
  }
}

void RunRawSubscriber(const Options& options) {
  auto node = century::cyber::CreateNode("packed_cloud_perf_raw_sub");
  Stats arena_stats;
  Stats shm_stats;
  std::vector<std::shared_ptr<
      century::cyber::Reader<century::cyber::message::RawMessage>>>
      readers;

  auto update_raw = [](Stats* stats, const std::shared_ptr<
                                         century::cyber::message::RawMessage>& msg) {
    std::lock_guard<std::mutex> lock(stats->mutex);
    ++stats->count;
    stats->bytes += msg->message.size();
  };

  if (HasGroup(options.group, "arena")) {
    for (const auto& topic : BuildTopics("arena", options.topics)) {
      readers.emplace_back(node->CreateReader<century::cyber::message::RawMessage>(
          topic, [&arena_stats, update_raw](
                     const std::shared_ptr<century::cyber::message::RawMessage>& msg) {
            update_raw(&arena_stats, msg);
          }));
    }
  }
  if (HasGroup(options.group, "shm")) {
    for (const auto& topic : BuildTopics("shm", options.topics)) {
      readers.emplace_back(node->CreateReader<century::cyber::message::RawMessage>(
          topic, [&shm_stats, update_raw](
                     const std::shared_ptr<century::cyber::message::RawMessage>& msg) {
            update_raw(&shm_stats, msg);
          }));
    }
  }

  auto start = std::chrono::steady_clock::now();
  auto last = start;
  double last_cpu = ProcessCpuSeconds();
  while (century::cyber::OK()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const auto now = std::chrono::steady_clock::now();
    const double wall =
        std::chrono::duration_cast<std::chrono::duration<double>>(now - last)
            .count();
    const double cpu = ProcessCpuSeconds();
    const double cpu_percent = wall > 0.0 ? (cpu - last_cpu) / wall * 100.0 : 0.0;

    if (HasGroup(options.group, "arena")) {
      PrintStatsLine("raw_arena", &arena_stats, cpu_percent);
    }
    if (HasGroup(options.group, "shm")) {
      PrintStatsLine("raw_shm  ", &shm_stats, cpu_percent);
    }
    std::cout.flush();
    last = now;
    last_cpu = cpu;

    if (options.seconds > 0) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
          now - start);
      if (elapsed.count() >= options.seconds) {
        break;
      }
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  const Options options = ParseOptions(argc, argv);
  if (options.mode != "pub" && options.mode != "sub" &&
      options.mode != "rawsub" && options.mode != "both") {
    PrintUsage();
    return 1;
  }

  century::cyber::Init(argv[0]);
  if (options.mode == "pub") {
    RunPublisher(options);
  } else if (options.mode == "sub") {
    RunSubscriber(options);
  } else if (options.mode == "rawsub") {
    RunRawSubscriber(options);
  } else {
    std::thread sub_thread([&options]() { RunSubscriber(options); });
    std::this_thread::sleep_for(std::chrono::seconds(1));
    RunPublisher(options);
    sub_thread.join();
  }
  return 0;
}
