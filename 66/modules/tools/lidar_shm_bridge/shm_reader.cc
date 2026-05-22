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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "cyber/transport/shm/shm_queue.h"
#include "modules/drivers/lidar/robosense/rs_driver/src/rs_driver/msg/point_cloud_msg.hpp"
#include "modules/tools/lidar_shm_bridge/tf_static_shm.h"

namespace century {
namespace tools {
namespace {

using century::cyber::transport::shm_queue;

constexpr char kTfTopic[] = "/tf";
constexpr char kTfStaticTopic[] = "/tf_static";
constexpr size_t kDefaultPreviewPointCount = 5;
constexpr size_t kDefaultPreviewTransformCount = 5;
constexpr double kDefaultDurationSec = 5.0;

std::mutex g_print_mutex;

const std::vector<std::string>& LidarTopics() {
  static const std::vector<std::string> kTopics = {
      "/lidar/bp/front_left", "/lidar/bp/front_right",
      "/lidar/bp/rear_left", "/lidar/bp/rear_right",
      "/lidar/helios/front_left", "/lidar/helios/rear_right",
  };
  return kTopics;
}

void PrintUsage(const char* argv0) {
  std::cerr << "用法:\n"
            << "  " << argv0 << " tf [持续秒数] [打印条数]\n"
            << "  " << argv0 << " tf_static [持续秒数] [打印条数]\n"
            << "  " << argv0 << " cloud <topic> [持续秒数] [打印条数]\n"
            << "  " << argv0 << " all [持续秒数] [打印条数]\n"
            << "  " << argv0 << " rate tf [持续秒数]\n"
            << "  " << argv0 << " rate tf_static [持续秒数]\n"
            << "  " << argv0 << " rate cloud <topic> [持续秒数]\n"
            << "  " << argv0 << " rate all [持续秒数]\n\n"
            << "示例:\n"
            << "  " << argv0 << " tf 5 10\n"
            << "  " << argv0 << " tf_static 5 20\n"
            << "  " << argv0 << " cloud /lidar/bp/front_left 5 10\n"
            << "  " << argv0 << " all 10 3\n"
            << "  " << argv0 << " rate tf 10\n"
            << "  " << argv0 << " rate cloud /lidar/bp/front_left 10\n"
            << "  " << argv0 << " rate tf_static 10\n"
            << "  " << argv0 << " rate all 10\n";
}

double ParseDurationSeconds(const char* value) {
  try {
    return std::stod(value);
  } catch (const std::exception&) {
    return -1.0;
  }
}

int ParsePreviewCount(const char* value) {
  try {
    return std::stoi(value);
  } catch (const std::exception&) {
    return -1;
  }
}

void PrintPointCloudMessage(const std::string& topic,
                            const std::shared_ptr<PointCloudShm>& msg,
                            const size_t preview_count_limit) {
  if (msg == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> lock(g_print_mutex);
  std::cout << std::fixed << std::setprecision(6)
            << "[cloud] topic=" << topic
            << " seq=" << msg->sequence_num
            << " timestamp=" << msg->timestamp
            << " measuretime=" << msg->measuretime
            << " width=" << msg->width
            << " height=" << msg->height
            << " points=" << msg->points_num
            << " is_dense=" << msg->is_dense << "\n";

  const size_t preview_count =
      std::min(static_cast<size_t>(msg->points_num), preview_count_limit);
  for (size_t i = 0; i < preview_count; ++i) {
    const auto& point = msg->points[i];
    std::cout << "  point[" << i << "] x=" << point.x << " y=" << point.y
              << " z=" << point.z
              << " intensity=" << static_cast<int>(point.intensity)
              << " ring=" << point.ring
              << " timestamp=" << point.timestamp << "\n";
  }
  std::cout << std::flush;
}

void PrintTransformMessage(const char* label,
                           const std::shared_ptr<TransformStampedsShm>& msg,
                           const size_t preview_count_limit) {
  if (msg == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> lock(g_print_mutex);
  std::cout << std::fixed << std::setprecision(6)
            << "[" << label << "] topic=" << label
            << " seq=" << msg->sequence_num
            << " timestamp=" << msg->timestamp_sec
            << " transform_count=" << msg->transform_count << "\n";

  const size_t preview_count =
      std::min(static_cast<size_t>(msg->transform_count), preview_count_limit);
  for (size_t i = 0; i < preview_count; ++i) {
    const auto& tf = msg->transforms[i];
    std::cout << "  tf[" << i << "] " << tf.frame_id << " -> "
              << tf.child_frame_id << " seq=" << tf.sequence_num
              << " timestamp=" << tf.timestamp_sec
              << " t=(" << tf.translation_x << ", " << tf.translation_y
              << ", " << tf.translation_z << ")"
              << " q=(" << tf.rotation_qx << ", " << tf.rotation_qy
              << ", " << tf.rotation_qz << ", " << tf.rotation_qw << ")"
              << "\n";
  }
  std::cout << std::flush;
}

void PrintRateSummary(const std::string& label, const uint64_t count,
                      const double duration_sec) {
  const double hz = duration_sec > 0.0 ? static_cast<double>(count) / duration_sec
                                       : 0.0;
  std::lock_guard<std::mutex> lock(g_print_mutex);
  std::cout << std::fixed << std::setprecision(3)
            << "[rate] target=" << label << " duration_sec=" << duration_sec
            << " messages=" << count << " avg_hz=" << hz << "\n";
}

int RunTransformReader(const std::string& topic, const char* label,
                       double duration_sec, const size_t preview_count) {
  shm_queue<TransformStampedsShm> queue(
      topic,
      [label, preview_count](const std::shared_ptr<TransformStampedsShm>& msg) {
        PrintTransformMessage(label, msg, preview_count);
      });
  std::this_thread::sleep_for(std::chrono::duration<double>(duration_sec));
  return 0;
}

int RunCloudReader(const std::string& topic, double duration_sec,
                   const size_t preview_count) {
  shm_queue<PointCloudShm> queue(
      topic,
      [topic, preview_count](const std::shared_ptr<PointCloudShm>& msg) {
        PrintPointCloudMessage(topic, msg, preview_count);
      });
  std::this_thread::sleep_for(std::chrono::duration<double>(duration_sec));
  return 0;
}

int RunAllReader(double duration_sec, const size_t preview_count) {
  std::vector<std::unique_ptr<shm_queue<PointCloudShm>>> cloud_queues;
  cloud_queues.reserve(LidarTopics().size());
  for (const auto& topic : LidarTopics()) {
    cloud_queues.emplace_back(std::make_unique<shm_queue<PointCloudShm>>(
        topic,
        [topic, preview_count](const std::shared_ptr<PointCloudShm>& msg) {
          PrintPointCloudMessage(topic, msg, preview_count);
        }));
  }

  shm_queue<TransformStampedsShm> tf_queue(
      kTfTopic,
      [preview_count](const std::shared_ptr<TransformStampedsShm>& msg) {
        PrintTransformMessage(kTfTopic, msg, preview_count);
      });
  shm_queue<TransformStampedsShm> tf_static_queue(
      kTfStaticTopic,
      [preview_count](const std::shared_ptr<TransformStampedsShm>& msg) {
        PrintTransformMessage(kTfStaticTopic, msg, preview_count);
      });

  std::this_thread::sleep_for(std::chrono::duration<double>(duration_sec));
  return 0;
}

int RunTransformRate(const std::string& topic, double duration_sec) {
  auto counter = std::make_shared<std::atomic<uint64_t>>(0);
  shm_queue<TransformStampedsShm> queue(
      topic,
      [counter](const std::shared_ptr<TransformStampedsShm>&) {
        counter->fetch_add(1, std::memory_order_relaxed);
      });
  std::this_thread::sleep_for(std::chrono::duration<double>(duration_sec));
  PrintRateSummary(topic, counter->load(std::memory_order_relaxed), duration_sec);
  return 0;
}

int RunCloudRate(const std::string& topic, double duration_sec) {
  auto counter = std::make_shared<std::atomic<uint64_t>>(0);
  shm_queue<PointCloudShm> queue(
      topic,
      [counter](const std::shared_ptr<PointCloudShm>&) {
        counter->fetch_add(1, std::memory_order_relaxed);
      });
  std::this_thread::sleep_for(std::chrono::duration<double>(duration_sec));
  PrintRateSummary(topic, counter->load(std::memory_order_relaxed), duration_sec);
  return 0;
}

int RunAllRate(double duration_sec) {
  struct TopicCounter {
    std::string topic;
    std::shared_ptr<std::atomic<uint64_t>> counter;
  };

  std::vector<TopicCounter> topic_counters;
  std::vector<std::unique_ptr<shm_queue<PointCloudShm>>> cloud_queues;
  topic_counters.reserve(LidarTopics().size() + 2);
  cloud_queues.reserve(LidarTopics().size());

  for (const auto& topic : LidarTopics()) {
    auto counter = std::make_shared<std::atomic<uint64_t>>(0);
    topic_counters.push_back({topic, counter});
    cloud_queues.emplace_back(std::make_unique<shm_queue<PointCloudShm>>(
        topic,
        [counter](const std::shared_ptr<PointCloudShm>&) {
          counter->fetch_add(1, std::memory_order_relaxed);
        }));
  }

  auto tf_counter = std::make_shared<std::atomic<uint64_t>>(0);
  topic_counters.push_back({kTfTopic, tf_counter});
  shm_queue<TransformStampedsShm> tf_queue(
      kTfTopic,
      [tf_counter](const std::shared_ptr<TransformStampedsShm>&) {
        tf_counter->fetch_add(1, std::memory_order_relaxed);
      });

  auto tf_static_counter = std::make_shared<std::atomic<uint64_t>>(0);
  topic_counters.push_back({kTfStaticTopic, tf_static_counter});
  shm_queue<TransformStampedsShm> tf_static_queue(
      kTfStaticTopic,
      [tf_static_counter](const std::shared_ptr<TransformStampedsShm>&) {
        tf_static_counter->fetch_add(1, std::memory_order_relaxed);
      });

  std::this_thread::sleep_for(std::chrono::duration<double>(duration_sec));
  for (const auto& entry : topic_counters) {
    PrintRateSummary(entry.topic, entry.counter->load(std::memory_order_relaxed),
                     duration_sec);
  }
  return 0;
}

}  // namespace

int Run(int argc, char** argv) {
  if (argc < 2) {
    PrintUsage(argv[0]);
    return 1;
  }

  const std::string mode = argv[1];
  if (mode == "tf") {
    const double duration_sec = argc >= 3 ? ParseDurationSeconds(argv[2])
                                          : kDefaultDurationSec;
    const int preview_count =
        argc >= 4 ? ParsePreviewCount(argv[3])
                  : static_cast<int>(kDefaultPreviewTransformCount);
    if (duration_sec <= 0.0 || preview_count <= 0) {
      PrintUsage(argv[0]);
      return 1;
    }
    return RunTransformReader(kTfTopic, kTfTopic, duration_sec,
                              static_cast<size_t>(preview_count));
  }

  if (mode == "tf_static") {
    const double duration_sec = argc >= 3 ? ParseDurationSeconds(argv[2])
                                          : kDefaultDurationSec;
    const int preview_count =
        argc >= 4 ? ParsePreviewCount(argv[3])
                  : static_cast<int>(kDefaultPreviewTransformCount);
    if (duration_sec <= 0.0 || preview_count <= 0) {
      PrintUsage(argv[0]);
      return 1;
    }
    return RunTransformReader(kTfStaticTopic, kTfStaticTopic, duration_sec,
                              static_cast<size_t>(preview_count));
  }

  if (mode == "cloud") {
    if (argc < 3) {
      PrintUsage(argv[0]);
      return 1;
    }
    const std::string topic = argv[2];
    const double duration_sec = argc >= 4 ? ParseDurationSeconds(argv[3])
                                          : kDefaultDurationSec;
    const int preview_count =
        argc >= 5 ? ParsePreviewCount(argv[4])
                  : static_cast<int>(kDefaultPreviewPointCount);
    if (duration_sec <= 0.0 || preview_count <= 0) {
      PrintUsage(argv[0]);
      return 1;
    }
    return RunCloudReader(topic, duration_sec,
                          static_cast<size_t>(preview_count));
  }

  if (mode == "all") {
    const double duration_sec = argc >= 3 ? ParseDurationSeconds(argv[2])
                                          : kDefaultDurationSec;
    const int preview_count =
        argc >= 4 ? ParsePreviewCount(argv[3])
                  : static_cast<int>(kDefaultPreviewPointCount);
    if (duration_sec <= 0.0 || preview_count <= 0) {
      PrintUsage(argv[0]);
      return 1;
    }
    return RunAllReader(duration_sec, static_cast<size_t>(preview_count));
  }

  if (mode == "rate") {
    if (argc < 3) {
      PrintUsage(argv[0]);
      return 1;
    }
    const std::string rate_target = argv[2];
    if (rate_target == "tf") {
      const double duration_sec = argc >= 4 ? ParseDurationSeconds(argv[3])
                                            : kDefaultDurationSec;
      if (duration_sec <= 0.0) {
        PrintUsage(argv[0]);
        return 1;
      }
      return RunTransformRate(kTfTopic, duration_sec);
    }
    if (rate_target == "tf_static") {
      const double duration_sec = argc >= 4 ? ParseDurationSeconds(argv[3])
                                            : kDefaultDurationSec;
      if (duration_sec <= 0.0) {
        PrintUsage(argv[0]);
        return 1;
      }
      return RunTransformRate(kTfStaticTopic, duration_sec);
    }
    if (rate_target == "cloud") {
      if (argc < 4) {
        PrintUsage(argv[0]);
        return 1;
      }
      const std::string topic = argv[3];
      const double duration_sec = argc >= 5 ? ParseDurationSeconds(argv[4])
                                            : kDefaultDurationSec;
      if (duration_sec <= 0.0) {
        PrintUsage(argv[0]);
        return 1;
      }
      return RunCloudRate(topic, duration_sec);
    }
    if (rate_target == "all") {
      const double duration_sec = argc >= 4 ? ParseDurationSeconds(argv[3])
                                            : kDefaultDurationSec;
      if (duration_sec <= 0.0) {
        PrintUsage(argv[0]);
        return 1;
      }
      return RunAllRate(duration_sec);
    }
    PrintUsage(argv[0]);
    return 1;
  }

  PrintUsage(argv[0]);
  return 1;
}

}  // namespace tools
}  // namespace century

int main(int argc, char** argv) {
  return century::tools::Run(argc, argv);
}
