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
#include "modules/tools/cyber_ros_bridge/cyber_ros_bridge_config.h"
#include "modules/tools/cyber_ros_bridge/tf_static_shm.h"

namespace century {
namespace tools {
namespace {

using century::cyber::transport::shm_queue;

CyberRosBridgeRuntimeConfig g_runtime_config;
std::mutex g_print_mutex;

const CyberRosBridgeRuntimeConfig& RuntimeConfig() { return g_runtime_config; }

const std::vector<std::string>& LidarTopics() {
  return RuntimeConfig().shm_reader.lidar_topics;
}

void PrintUsage(const char* argv0) {
  const auto& bridge_config = RuntimeConfig().lidar_shm_bridge;
  const auto& reader_config = RuntimeConfig().shm_reader;
  const auto& example_topic = LidarTopics().front();
  std::cerr << "Usage:\n"
            << "  " << argv0 << " " << reader_config.mode_tf << " ["
            << reader_config.arg_duration_sec << "] ["
            << reader_config.arg_preview_count << "]\n"
            << "  " << argv0 << " " << reader_config.mode_tf_static << " ["
            << reader_config.arg_duration_sec << "] ["
            << reader_config.arg_preview_count << "]\n"
            << "  " << argv0 << " " << reader_config.mode_cloud << " <"
            << reader_config.arg_topic << "> ["
            << reader_config.arg_duration_sec << "] ["
            << reader_config.arg_preview_count << "]\n"
            << "  " << argv0 << " " << reader_config.mode_compare << " <"
            << reader_config.arg_topic << "> ["
            << reader_config.arg_duration_sec << "]\n"
            << "  " << argv0 << " " << reader_config.mode_all << " ["
            << reader_config.arg_duration_sec << "] ["
            << reader_config.arg_preview_count << "]\n"
            << "  " << argv0 << " " << reader_config.mode_rate << " "
            << reader_config.mode_tf << " [" << reader_config.arg_duration_sec
            << "]\n"
            << "  " << argv0 << " " << reader_config.mode_rate << " "
            << reader_config.mode_tf_static << " ["
            << reader_config.arg_duration_sec << "]\n"
            << "  " << argv0 << " " << reader_config.mode_rate << " "
            << reader_config.mode_cloud << " <" << reader_config.arg_topic
            << "> [" << reader_config.arg_duration_sec << "]\n"
            << "  " << argv0 << " " << reader_config.mode_rate << " "
            << reader_config.mode_all << " [" << reader_config.arg_duration_sec
            << "]\n\n"
            << "Examples:\n"
            << "  " << argv0 << " " << reader_config.mode_tf << " 5 10\n"
            << "  " << argv0 << " " << reader_config.mode_tf_static
            << " 5 20\n"
            << "  " << argv0 << " " << reader_config.mode_cloud << " "
            << example_topic << " 5 10\n"
            << "  " << argv0 << " " << reader_config.mode_compare << " "
            << example_topic << " 10\n"
            << "  " << argv0 << " " << reader_config.mode_all << " 10 3\n"
            << "  " << argv0 << " " << reader_config.mode_rate << " "
            << reader_config.mode_tf << " 10\n"
            << "  " << argv0 << " " << reader_config.mode_rate << " "
            << reader_config.mode_cloud << " " << example_topic << " 10\n"
            << "  " << argv0 << " " << reader_config.mode_rate << " "
            << reader_config.mode_tf_static << " 10\n"
            << "  " << argv0 << " " << reader_config.mode_rate << " "
            << reader_config.mode_all << " 10\n"
            << "Configured tf topics: " << bridge_config.tf_topic << ", "
            << bridge_config.tf_static_topic << "\n";
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
  if (nullptr == msg) {
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

void PrintTransformMessage(const std::string& label,
                           const std::shared_ptr<TransformStampedsShm>& msg,
                           const size_t preview_count_limit) {
  if (nullptr == msg) {
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

struct TimestampCompareState {
  explicit TimestampCompareState(const std::string& topic) : cloud_topic(topic) {}

  std::mutex mutex;
  std::string cloud_topic;
  double latest_tf_timestamp = 0.0;
  uint32_t latest_tf_seq = 0;
  double latest_cloud_timestamp = 0.0;
  double latest_cloud_measuretime = 0.0;
  uint32_t latest_cloud_seq = 0;
  double last_printed_tf_timestamp = -1.0;
  double last_printed_cloud_timestamp = -1.0;
  double last_printed_cloud_measuretime = -1.0;
};

void MaybePrintTimestampComparison(
    const std::shared_ptr<TimestampCompareState>& state) {
  std::lock_guard<std::mutex> state_lock(state->mutex);
  if (state->latest_tf_timestamp <= 0.0 || state->latest_cloud_timestamp <= 0.0) {
    return;
  }

  if (state->last_printed_tf_timestamp == state->latest_tf_timestamp &&
      state->last_printed_cloud_timestamp == state->latest_cloud_timestamp &&
      state->last_printed_cloud_measuretime == state->latest_cloud_measuretime) {
    return;
  }

  state->last_printed_tf_timestamp = state->latest_tf_timestamp;
  state->last_printed_cloud_timestamp = state->latest_cloud_timestamp;
  state->last_printed_cloud_measuretime = state->latest_cloud_measuretime;

  const double delta_header_sec =
      state->latest_tf_timestamp - state->latest_cloud_timestamp;
  const double delta_measuretime_sec =
      state->latest_tf_timestamp - state->latest_cloud_measuretime;

  std::lock_guard<std::mutex> print_lock(g_print_mutex);
  std::cout << std::fixed << std::setprecision(6)
            << "[compare] tf_topic=" << RuntimeConfig().lidar_shm_bridge.tf_topic
            << " cloud_topic=" << state->cloud_topic
            << " tf_seq=" << state->latest_tf_seq
            << " tf_timestamp=" << state->latest_tf_timestamp
            << " cloud_seq=" << state->latest_cloud_seq
            << " cloud_timestamp=" << state->latest_cloud_timestamp
            << " cloud_measuretime=" << state->latest_cloud_measuretime
            << " delta_header_sec=" << delta_header_sec
            << " delta_header_ms=" << (delta_header_sec * 1000.0)
            << " delta_measuretime_sec=" << delta_measuretime_sec
            << " delta_measuretime_ms=" << (delta_measuretime_sec * 1000.0)
            << "\n";
}

int RunTransformReader(const std::string& topic, const std::string& label,
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
  const auto& bridge_config = RuntimeConfig().lidar_shm_bridge;
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
      bridge_config.tf_topic,
      [preview_count](const std::shared_ptr<TransformStampedsShm>& msg) {
        PrintTransformMessage(RuntimeConfig().lidar_shm_bridge.tf_topic, msg,
                              preview_count);
      });
  shm_queue<TransformStampedsShm> tf_static_queue(
      bridge_config.tf_static_topic,
      [preview_count](const std::shared_ptr<TransformStampedsShm>& msg) {
        PrintTransformMessage(RuntimeConfig().lidar_shm_bridge.tf_static_topic,
                              msg, preview_count);
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

int RunTfCloudTimestampCompare(const std::string& topic, double duration_sec) {
  const auto& bridge_config = RuntimeConfig().lidar_shm_bridge;
  auto state = std::make_shared<TimestampCompareState>(topic);
  shm_queue<TransformStampedsShm> tf_queue(
      bridge_config.tf_topic,
      [state](const std::shared_ptr<TransformStampedsShm>& msg) {
        if (nullptr == msg) {
          return;
        }
        {
          std::lock_guard<std::mutex> lock(state->mutex);
          state->latest_tf_seq = msg->sequence_num;
          state->latest_tf_timestamp = msg->timestamp_sec;
        }
        MaybePrintTimestampComparison(state);
      });
  shm_queue<PointCloudShm> cloud_queue(
      topic,
      [state](const std::shared_ptr<PointCloudShm>& msg) {
        if (nullptr == msg) {
          return;
        }
        {
          std::lock_guard<std::mutex> lock(state->mutex);
          state->latest_cloud_seq = msg->sequence_num;
          state->latest_cloud_timestamp = msg->timestamp;
          state->latest_cloud_measuretime = msg->measuretime;
        }
        MaybePrintTimestampComparison(state);
      });
  std::this_thread::sleep_for(std::chrono::duration<double>(duration_sec));
  return 0;
}

int RunAllRate(double duration_sec) {
  const auto& bridge_config = RuntimeConfig().lidar_shm_bridge;
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
  topic_counters.push_back({bridge_config.tf_topic, tf_counter});
  shm_queue<TransformStampedsShm> tf_queue(
      bridge_config.tf_topic,
      [tf_counter](const std::shared_ptr<TransformStampedsShm>&) {
        tf_counter->fetch_add(1, std::memory_order_relaxed);
      });

  auto tf_static_counter = std::make_shared<std::atomic<uint64_t>>(0);
  topic_counters.push_back({bridge_config.tf_static_topic, tf_static_counter});
  shm_queue<TransformStampedsShm> tf_static_queue(
      bridge_config.tf_static_topic,
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
  if (!LoadCyberRosBridgeRuntimeConfig(&g_runtime_config)) {
    return 1;
  }
  if (argc < 2) {
    PrintUsage(argv[0]);
    return 1;
  }

  const auto& bridge_config = RuntimeConfig().lidar_shm_bridge;
  const auto& reader_config = RuntimeConfig().shm_reader;
  const std::string mode = argv[1];
  if (reader_config.mode_tf == mode) {
    const double duration_sec = argc >= 3 ? ParseDurationSeconds(argv[2])
                                          : reader_config.default_duration_sec;
    const int preview_count =
        argc >= 4 ? ParsePreviewCount(argv[3])
                  : static_cast<int>(reader_config.default_preview_transform_count);
    if (duration_sec <= 0.0 || preview_count <= 0) {
      PrintUsage(argv[0]);
      return 1;
    }
    return RunTransformReader(bridge_config.tf_topic, bridge_config.tf_topic,
                              duration_sec,
                              static_cast<size_t>(preview_count));
  }

  if (reader_config.mode_tf_static == mode) {
    const double duration_sec = argc >= 3 ? ParseDurationSeconds(argv[2])
                                          : reader_config.default_duration_sec;
    const int preview_count =
        argc >= 4 ? ParsePreviewCount(argv[3])
                  : static_cast<int>(reader_config.default_preview_transform_count);
    if (duration_sec <= 0.0 || preview_count <= 0) {
      PrintUsage(argv[0]);
      return 1;
    }
    return RunTransformReader(bridge_config.tf_static_topic,
                              bridge_config.tf_static_topic, duration_sec,
                              static_cast<size_t>(preview_count));
  }

  if (reader_config.mode_cloud == mode) {
    if (argc < 3) {
      PrintUsage(argv[0]);
      return 1;
    }
    const std::string topic = argv[2];
    const double duration_sec = argc >= 4 ? ParseDurationSeconds(argv[3])
                                          : reader_config.default_duration_sec;
    const int preview_count =
        argc >= 5 ? ParsePreviewCount(argv[4])
                  : static_cast<int>(reader_config.default_preview_point_count);
    if (duration_sec <= 0.0 || preview_count <= 0) {
      PrintUsage(argv[0]);
      return 1;
    }
    return RunCloudReader(topic, duration_sec,
                          static_cast<size_t>(preview_count));
  }

  if (reader_config.mode_compare == mode) {
    if (argc < 3) {
      PrintUsage(argv[0]);
      return 1;
    }
    const std::string topic = argv[2];
    const double duration_sec = argc >= 4 ? ParseDurationSeconds(argv[3])
                                          : reader_config.default_duration_sec;
    if (duration_sec <= 0.0) {
      PrintUsage(argv[0]);
      return 1;
    }
    return RunTfCloudTimestampCompare(topic, duration_sec);
  }

  if (reader_config.mode_all == mode) {
    const double duration_sec = argc >= 3 ? ParseDurationSeconds(argv[2])
                                          : reader_config.default_duration_sec;
    const int preview_count =
        argc >= 4 ? ParsePreviewCount(argv[3])
                  : static_cast<int>(reader_config.default_preview_point_count);
    if (duration_sec <= 0.0 || preview_count <= 0) {
      PrintUsage(argv[0]);
      return 1;
    }
    return RunAllReader(duration_sec, static_cast<size_t>(preview_count));
  }

  if (reader_config.mode_rate == mode) {
    if (argc < 3) {
      PrintUsage(argv[0]);
      return 1;
    }
    const std::string rate_target = argv[2];
    if (reader_config.mode_tf == rate_target) {
      const double duration_sec = argc >= 4 ? ParseDurationSeconds(argv[3])
                                            : reader_config.default_duration_sec;
      if (duration_sec <= 0.0) {
        PrintUsage(argv[0]);
        return 1;
      }
      return RunTransformRate(bridge_config.tf_topic, duration_sec);
    }
    if (reader_config.mode_tf_static == rate_target) {
      const double duration_sec = argc >= 4 ? ParseDurationSeconds(argv[3])
                                            : reader_config.default_duration_sec;
      if (duration_sec <= 0.0) {
        PrintUsage(argv[0]);
        return 1;
      }
      return RunTransformRate(bridge_config.tf_static_topic, duration_sec);
    }
    if (reader_config.mode_cloud == rate_target) {
      if (argc < 4) {
        PrintUsage(argv[0]);
        return 1;
      }
      const std::string topic = argv[3];
      const double duration_sec = argc >= 5 ? ParseDurationSeconds(argv[4])
                                            : reader_config.default_duration_sec;
      if (duration_sec <= 0.0) {
        PrintUsage(argv[0]);
        return 1;
      }
      return RunCloudRate(topic, duration_sec);
    }
    if (reader_config.mode_all == rate_target) {
      const double duration_sec = argc >= 4 ? ParseDurationSeconds(argv[3])
                                            : reader_config.default_duration_sec;
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
