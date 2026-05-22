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
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "cyber/common/file.h"
#include "cyber/message/raw_message.h"
#include "cyber/service_discovery/topology_manager.h"
#include "cyber/transport/shm/shm_queue.h"
#include "modules/common/util/util.h"
#include "modules/drivers/lidar/robosense/rs_driver/src/rs_driver/msg/point_cloud_msg.hpp"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/tools/cyber_ros_bridge/cyber_ros_bridge_config.h"
#include "modules/tools/cyber_ros_bridge/tf_static_shm.h"
#include "modules/transform/proto/transform.pb.h"

namespace century {
namespace tools {
namespace {

using century::common::util::IsFloatEqual;
using century::cyber::message::RawMessage;
using century::cyber::service_discovery::TopologyManager;
using century::cyber::transport::shm_queue;

CyberRosBridgeRuntimeConfig g_runtime_config;

const CyberRosBridgeRuntimeConfig& RuntimeConfig() { return g_runtime_config; }

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

uint8_t ClampIntensity(const uint32_t intensity) {
  return static_cast<uint8_t>(
      std::min<uint32_t>(intensity, std::numeric_limits<uint8_t>::max()));
}

uint16_t ClampRing(const uint32_t ring) {
  return static_cast<uint16_t>(
      std::min<uint32_t>(ring, std::numeric_limits<uint16_t>::max()));
}

template <size_t N>
void CopyString(const std::string& src, char (&dst)[N]) {
  static_assert(N > 0, "destination buffer must not be empty");
  std::memset(dst, 0, N);
  std::strncpy(dst, src.c_str(), N - 1);
}

void FillTransformStampedsShmFromTransformMessage(
    const transform::TransformStampeds& tf_msg, TransformStampedsShm* shm_msg,
    const char* topic) {
  const size_t transform_count =
      std::min(static_cast<size_t>(tf_msg.transforms_size()),
               kMaxTfStaticTransforms);
  if (static_cast<size_t>(tf_msg.transforms_size()) != transform_count) {
    AWARN << "Topic " << topic
          << " transform count exceeds shm capacity, truncating from "
          << tf_msg.transforms_size() << " to " << transform_count;
  }

  shm_msg->sequence_num = tf_msg.has_header() ? tf_msg.header().sequence_num() : 0;
  shm_msg->timestamp_sec =
      tf_msg.has_header() ? tf_msg.header().timestamp_sec() : 0.0;
  shm_msg->transform_count = static_cast<uint32_t>(transform_count);

  for (size_t i = 0; i < transform_count; ++i) {
    const auto& src = tf_msg.transforms(static_cast<int>(i));
    auto& dst = shm_msg->transforms[i];
    dst.timestamp_sec = src.has_header() ? src.header().timestamp_sec() : 0.0;
    dst.sequence_num = src.has_header() ? src.header().sequence_num() : 0;
    CopyString(src.has_header() ? src.header().frame_id() : std::string(),
               dst.frame_id);
    CopyString(src.child_frame_id(), dst.child_frame_id);
    dst.translation_x = src.transform().translation().x();
    dst.translation_y = src.transform().translation().y();
    dst.translation_z = src.transform().translation().z();
    dst.rotation_qx = src.transform().rotation().qx();
    dst.rotation_qy = src.transform().rotation().qy();
    dst.rotation_qz = src.transform().rotation().qz();
    dst.rotation_qw = src.transform().rotation().qw();
  }

  if (IsFloatEqual(shm_msg->timestamp_sec, 0.0) && transform_count > 0) {
    shm_msg->timestamp_sec = shm_msg->transforms[0].timestamp_sec;
  }
}

void FillTransformStampedsShmFromLocalization(
    const localization::LocalizationEstimate& localization,
    TransformStampedsShm* shm_msg) {
  const auto& bridge_config = RuntimeConfig().lidar_shm_bridge;
  shm_msg->transform_count = 1;
  shm_msg->sequence_num =
      localization.has_header() ? localization.header().sequence_num() : 0;
  shm_msg->timestamp_sec = localization.measurement_time();
  if (IsFloatEqual(shm_msg->timestamp_sec, 0.0) && localization.has_header()) {
    shm_msg->timestamp_sec = localization.header().timestamp_sec();
  }

  auto& dst = shm_msg->transforms[0];
  dst.sequence_num = shm_msg->sequence_num;
  dst.timestamp_sec = shm_msg->timestamp_sec;
  CopyString(bridge_config.tf_frame_id, dst.frame_id);
  CopyString(bridge_config.tf_child_frame_id, dst.child_frame_id);
  dst.translation_x = localization.pose().position().x();
  dst.translation_y = localization.pose().position().y();
  dst.translation_z = localization.pose().position().z();
  dst.rotation_qx = localization.pose().orientation().qx();
  dst.rotation_qy = localization.pose().orientation().qy();
  dst.rotation_qz = localization.pose().orientation().qz();
  dst.rotation_qw = localization.pose().orientation().qw();
}

void FillTransformStampedsShmFromPoseWithCov(
    const localization::PoseWithCov& pose_with_cov,
    TransformStampedsShm* shm_msg) {
  const auto& bridge_config = RuntimeConfig().lidar_shm_bridge;
  shm_msg->transform_count = 1;
  shm_msg->sequence_num =
      pose_with_cov.has_header() ? pose_with_cov.header().sequence_num() : 0;
  shm_msg->timestamp_sec =
      pose_with_cov.has_header() ? pose_with_cov.header().timestamp_sec() : 0.0;

  auto& dst = shm_msg->transforms[0];
  dst.sequence_num = shm_msg->sequence_num;
  dst.timestamp_sec = shm_msg->timestamp_sec;
  CopyString(bridge_config.tf_frame_id, dst.frame_id);
  CopyString(bridge_config.tf_child_frame_id, dst.child_frame_id);
  dst.translation_x = pose_with_cov.position().x();
  dst.translation_y = pose_with_cov.position().y();
  dst.translation_z = pose_with_cov.position().z();
  dst.rotation_qx = pose_with_cov.orientation().qx();
  dst.rotation_qy = pose_with_cov.orientation().qy();
  dst.rotation_qz = pose_with_cov.orientation().qz();
  dst.rotation_qw = pose_with_cov.orientation().qw();
}

void ConfigureDefaultLogDir() {
  const auto& log_dir = RuntimeConfig().lidar_shm_bridge.default_log_dir;
  if (!cyber::common::EnsureDirectory(log_dir)) {
    std::cerr << "Failed to ensure log directory: " << log_dir << std::endl;
    return;
  }
  FLAGS_log_dir = log_dir;
}

class LidarTopicBridge {
 public:
  LidarTopicBridge(cyber::Node* node, const LidarTopicRuntimeConfig& config)
      : topic_(config.topic),
        child_frame_id_(config.child_frame_id),
        queue_(std::make_unique<shm_queue<PointCloudShm>>(topic_)) {
    reader_ = node->CreateReader<RawMessage>(
        topic_, [this](const std::shared_ptr<RawMessage>& raw_msg) {
          OnRawMessage(raw_msg);
        });
    AINFO << "Created lidar shm bridge for topic: " << topic_
          << ", child_frame_id: " << child_frame_id_;
  }

 private:
  bool ShouldLog(const size_t count) const {
    const auto& bridge_config = RuntimeConfig().lidar_shm_bridge;
    return count <= bridge_config.verbose_startup_log_count ||
           0 == (count % bridge_config.periodic_log_interval);
  }

  void OnRawMessage(const std::shared_ptr<RawMessage>& raw_msg) {
    if (nullptr == raw_msg) {
      return;
    }

    ++recv_count_;

    thread_local PointCloudShm shm_msg = {};
    shm_msg = PointCloudShm{};
    std::string msg_type;

    if (!ConvertToShm(*raw_msg, &shm_msg, &msg_type)) {
      ++convert_fail_count_;
      if (ShouldLog(convert_fail_count_)) {
        AWARN << "[lidar_shm_bridge] topic=" << topic_
              << " child_frame_id=" << child_frame_id_
              << " recv_count=" << recv_count_
              << " convert_fail_count=" << convert_fail_count_
              << " raw_bytes=" << raw_msg->message.size()
              << " recv_ok=true shm_write_ok=false"
              << " reason=convert_failed";
      }
      return;
    }

    if (!queue_->Write(&shm_msg)) {
      ++shm_write_fail_count_;
      AWARN << "[lidar_shm_bridge] topic=" << topic_
            << " child_frame_id=" << child_frame_id_
            << " recv_count=" << recv_count_
            << " shm_write_fail_count=" << shm_write_fail_count_
            << " msg_type=" << msg_type
            << " raw_bytes=" << raw_msg->message.size()
            << " points=" << shm_msg.points_num
            << " timestamp=" << shm_msg.timestamp
            << " recv_ok=true shm_write_ok=false"
            << " reason=queue_write_failed";
      return;
    }

    ++shm_write_count_;
    if (ShouldLog(shm_write_count_)) {
      AINFO << "[lidar_shm_bridge] topic=" << topic_
            << " child_frame_id=" << child_frame_id_
            << " recv_count=" << recv_count_
            << " shm_write_count=" << shm_write_count_
            << " msg_type=" << msg_type
            << " raw_bytes=" << raw_msg->message.size()
            << " points=" << shm_msg.points_num
            << " seq=" << shm_msg.sequence_num
            << " timestamp=" << shm_msg.timestamp
            << " measuretime=" << shm_msg.measuretime
            << " recv_ok=true shm_write_ok=true";
    }
  }

  bool ConvertToShm(const RawMessage& raw_msg, PointCloudShm* shm_msg,
                    std::string* msg_type_out) {
    const auto& bridge_config = RuntimeConfig().lidar_shm_bridge;
    std::string msg_type;
    TopologyManager::Instance()->channel_manager()->GetMsgType(topic_, &msg_type);
    if (nullptr != msg_type_out) {
      *msg_type_out = msg_type;
    }
    if (msg_type.empty()) {
      if (1 == (++unknown_type_count_ % 100)) {
        AWARN << "Waiting for channel type of topic: " << topic_;
      }
      return false;
    }

    if (bridge_config.point_xyzirt_cloud_type == msg_type) {
      drivers::PointXYZIRTCloud cloud_msg;
      if (!cloud_msg.ParseFromString(raw_msg.message)) {
        AWARN << "Failed to parse PointXYZIRTCloud from topic: " << topic_;
        return false;
      }
      FillFromPointCloud(cloud_msg, shm_msg);
      return true;
    }

    if (bridge_config.point_cloud_packed_type == msg_type) {
      drivers::PointCloudPacked packed_msg;
      if (!packed_msg.ParseFromString(raw_msg.message)) {
        AWARN << "Failed to parse PointCloudPacked from topic: " << topic_;
        return false;
      }
      FillFromPackedCloud(packed_msg, shm_msg);
      return true;
    }

    if (1 == (++unsupported_type_count_ % 100)) {
      AWARN << "Unsupported message type on " << topic_ << ": " << msg_type;
    }
    return false;
  }

  void FillFromPointCloud(const drivers::PointXYZIRTCloud& cloud_msg,
                          PointCloudShm* shm_msg) const {
    const size_t point_count =
        std::min(static_cast<size_t>(cloud_msg.point_size()),
                 static_cast<size_t>(NUM_POINTS));
    if (static_cast<size_t>(cloud_msg.point_size()) != point_count) {
      AWARN << "Topic " << topic_
            << " point count exceeds shm capacity, truncating from "
            << cloud_msg.point_size() << " to " << point_count;
    }

    shm_msg->points_num = static_cast<uint32_t>(point_count);
    shm_msg->is_dense = cloud_msg.is_dense();
    shm_msg->sequence_num =
        cloud_msg.has_header() ? cloud_msg.header().sequence_num() : 0;
    shm_msg->timestamp = cloud_msg.has_header() ? cloud_msg.header().timestamp_sec()
                                                : 0.0;
    shm_msg->measuretime = cloud_msg.measuretime();
    if (!cloud_msg.has_width() || !cloud_msg.has_height() ||
        static_cast<size_t>(cloud_msg.width()) * cloud_msg.height() != point_count) {
      shm_msg->width = static_cast<uint32_t>(point_count);
      shm_msg->height = point_count > 0 ? 1 : 0;
    } else {
      shm_msg->width = cloud_msg.width();
      shm_msg->height = cloud_msg.height();
    }

    if (IsFloatEqual(shm_msg->timestamp, 0.0) && point_count > 0) {
      shm_msg->timestamp = cloud_msg.point(0).timestamp();
    }
    if (IsFloatEqual(shm_msg->measuretime, 0.0)) {
      shm_msg->measuretime = shm_msg->timestamp;
    }

    for (size_t i = 0; i < point_count; ++i) {
      const auto& src = cloud_msg.point(static_cast<int>(i));
      auto& dst = shm_msg->points[i];
      dst.x = src.x();
      dst.y = src.y();
      dst.z = src.z();
      dst.intensity = ClampIntensity(src.intensity());
      dst.ring = ClampRing(src.ring());
      dst.timestamp = src.timestamp();
    }
  }

  void FillFromPackedCloud(const drivers::PointCloudPacked& packed_msg,
                           PointCloudShm* shm_msg) const {
    const size_t available_points = packed_msg.data().size() / sizeof(PointPacked);
    const size_t packed_points = packed_msg.has_point_size()
                                     ? packed_msg.point_size()
                                     : available_points;
    const size_t point_count =
        std::min(std::min(packed_points, available_points),
                 static_cast<size_t>(NUM_POINTS));

    if (packed_points != point_count) {
      AWARN << "Topic " << topic_
            << " packed point count exceeds shm capacity or payload size, truncating from "
            << packed_points << " to " << point_count;
    }

    shm_msg->points_num = static_cast<uint32_t>(point_count);
    shm_msg->is_dense = packed_msg.is_dense();
    shm_msg->sequence_num =
        packed_msg.has_header() ? packed_msg.header().sequence_num() : 0;
    shm_msg->timestamp = packed_msg.has_header()
                             ? packed_msg.header().timestamp_sec()
                             : 0.0;
    shm_msg->measuretime = packed_msg.measuretime();
    if (!packed_msg.has_width() || !packed_msg.has_height() ||
        static_cast<size_t>(packed_msg.width()) * packed_msg.height() != point_count) {
      shm_msg->width = static_cast<uint32_t>(point_count);
      shm_msg->height = point_count > 0 ? 1 : 0;
    } else {
      shm_msg->width = packed_msg.width();
      shm_msg->height = packed_msg.height();
    }

    if (0 == point_count) {
      if (IsFloatEqual(shm_msg->measuretime, 0.0)) {
        shm_msg->measuretime = shm_msg->timestamp;
      }
      return;
    }

    for (size_t i = 0; i < point_count; ++i) {
      PointPacked src;
      std::memcpy(&src, packed_msg.data().data() + i * sizeof(PointPacked),
                  sizeof(PointPacked));
      auto& dst = shm_msg->points[i];
      dst.x = src.x;
      dst.y = src.y;
      dst.z = src.z;
      dst.intensity = ClampIntensity(src.intensity);
      dst.ring = src.ring;
      dst.timestamp = src.timestamp;
    }

    if (IsFloatEqual(shm_msg->timestamp, 0.0)) {
      PointPacked first_point;
      std::memcpy(&first_point, packed_msg.data().data(), sizeof(PointPacked));
      shm_msg->timestamp = first_point.timestamp;
    }
    if (IsFloatEqual(shm_msg->measuretime, 0.0)) {
      shm_msg->measuretime = shm_msg->timestamp;
    }
  }

 private:
  std::string topic_;
  std::string child_frame_id_;
  size_t recv_count_ = 0;
  size_t shm_write_count_ = 0;
  size_t convert_fail_count_ = 0;
  size_t shm_write_fail_count_ = 0;
  size_t unknown_type_count_ = 0;
  size_t unsupported_type_count_ = 0;
  std::unique_ptr<shm_queue<PointCloudShm>> queue_;
  std::shared_ptr<cyber::Reader<RawMessage>> reader_;
};

class TfStaticBridge {
 public:
  explicit TfStaticBridge(cyber::Node* node,
                          const LidarShmBridgeRuntimeConfig& config)
      : topic_(config.tf_static_topic),
        expected_msg_type_(config.transform_stampeds_type),
        queue_(std::make_unique<shm_queue<TransformStampedsShm>>(topic_)) {
    reader_ = node->CreateReader<RawMessage>(
        topic_, [this](const std::shared_ptr<RawMessage>& raw_msg) {
          OnRawMessage(raw_msg);
        });
    AINFO << "Created tf_static shm bridge for topic: " << topic_;
  }

 private:
  bool ShouldLog(const size_t count) const {
    const auto& bridge_config = RuntimeConfig().lidar_shm_bridge;
    return count <= bridge_config.verbose_startup_log_count ||
           0 == (count % bridge_config.periodic_log_interval);
  }

  void OnRawMessage(const std::shared_ptr<RawMessage>& raw_msg) {
    if (nullptr == raw_msg) {
      return;
    }

    ++recv_count_;

    thread_local TransformStampedsShm shm_msg = {};
    shm_msg = TransformStampedsShm{};
    std::string msg_type;

    if (!ConvertToShm(*raw_msg, &shm_msg, &msg_type)) {
      ++convert_fail_count_;
      if (ShouldLog(convert_fail_count_)) {
        AWARN << "[tf_static_bridge] topic=" << topic_
              << " recv_count=" << recv_count_
              << " convert_fail_count=" << convert_fail_count_
              << " raw_bytes=" << raw_msg->message.size()
              << " recv_ok=true shm_write_ok=false"
              << " reason=convert_failed";
      }
      return;
    }

    if (!queue_->Write(&shm_msg)) {
      ++shm_write_fail_count_;
      AWARN << "[tf_static_bridge] topic=" << topic_
            << " recv_count=" << recv_count_
            << " shm_write_fail_count=" << shm_write_fail_count_
            << " msg_type=" << msg_type
            << " raw_bytes=" << raw_msg->message.size()
            << " transform_count=" << shm_msg.transform_count
            << " timestamp=" << shm_msg.timestamp_sec
            << " recv_ok=true shm_write_ok=false"
            << " reason=queue_write_failed";
      return;
    }

    ++shm_write_count_;
    if (ShouldLog(shm_write_count_)) {
      AINFO << "[tf_static_bridge] topic=" << topic_
            << " recv_count=" << recv_count_
            << " shm_write_count=" << shm_write_count_
            << " msg_type=" << msg_type
            << " raw_bytes=" << raw_msg->message.size()
            << " transform_count=" << shm_msg.transform_count
            << " seq=" << shm_msg.sequence_num
            << " timestamp=" << shm_msg.timestamp_sec
            << " recv_ok=true shm_write_ok=true";
    }
  }

  bool ConvertToShm(const RawMessage& raw_msg, TransformStampedsShm* shm_msg,
                    std::string* msg_type_out) {
    std::string msg_type;
    TopologyManager::Instance()->channel_manager()->GetMsgType(topic_, &msg_type);
    if (nullptr != msg_type_out) {
      *msg_type_out = msg_type;
    }
    if (msg_type.empty()) {
      if (1 == (++unknown_type_count_ % 100)) {
        AWARN << "Waiting for channel type of topic: " << topic_;
      }
      return false;
    }

    if (expected_msg_type_ != msg_type) {
      if (1 == (++unsupported_type_count_ % 100)) {
        AWARN << "Unsupported message type on " << topic_ << ": " << msg_type;
      }
      return false;
    }

    transform::TransformStampeds tf_msg;
    if (!tf_msg.ParseFromString(raw_msg.message)) {
      AWARN << "Failed to parse TransformStampeds from topic: " << topic_;
      return false;
    }
    FillTransformStampedsShmFromTransformMessage(tf_msg, shm_msg, topic_.c_str());
    return true;
  }

 private:
  std::string topic_;
  std::string expected_msg_type_;
  size_t recv_count_ = 0;
  size_t shm_write_count_ = 0;
  size_t convert_fail_count_ = 0;
  size_t shm_write_fail_count_ = 0;
  size_t unknown_type_count_ = 0;
  size_t unsupported_type_count_ = 0;
  std::unique_ptr<shm_queue<TransformStampedsShm>> queue_;
  std::shared_ptr<cyber::Reader<RawMessage>> reader_;
};

class LocalizationTfBridge {
 public:
  explicit LocalizationTfBridge(cyber::Node* node,
                                const LidarShmBridgeRuntimeConfig& config)
      : source_topic_(config.lidar_pose_topic),
        target_topic_(config.tf_topic),
        localization_estimate_type_(config.localization_estimate_type),
        pose_with_cov_type_(config.pose_with_cov_type),
        queue_(std::make_unique<shm_queue<TransformStampedsShm>>(target_topic_)) {
    reader_ = node->CreateReader<RawMessage>(
        source_topic_, [this](const std::shared_ptr<RawMessage>& raw_msg) {
          OnRawMessage(raw_msg);
        });
    AINFO << "Created localization tf shm bridge from topic: "
          << source_topic_ << " to shm topic: " << target_topic_;
  }

 private:
  bool ShouldLog(const size_t count) const {
    const auto& bridge_config = RuntimeConfig().lidar_shm_bridge;
    return count <= bridge_config.verbose_startup_log_count ||
           0 == (count % bridge_config.periodic_log_interval);
  }

  void OnRawMessage(const std::shared_ptr<RawMessage>& raw_msg) {
    if (nullptr == raw_msg) {
      return;
    }

    ++recv_count_;

    thread_local TransformStampedsShm shm_msg = {};
    shm_msg = TransformStampedsShm{};
    std::string msg_type;

    if (!ConvertToShm(*raw_msg, &shm_msg, &msg_type)) {
      ++convert_fail_count_;
      if (ShouldLog(convert_fail_count_)) {
        AWARN << "[tf_bridge] source_topic=" << source_topic_
              << " target_topic=" << target_topic_
              << " recv_count=" << recv_count_
              << " convert_fail_count=" << convert_fail_count_
              << " raw_bytes=" << raw_msg->message.size()
              << " recv_ok=true shm_write_ok=false"
              << " reason=convert_failed";
      }
      return;
    }

    if (!queue_->Write(&shm_msg)) {
      ++shm_write_fail_count_;
      AWARN << "[tf_bridge] source_topic=" << source_topic_
            << " target_topic=" << target_topic_
            << " recv_count=" << recv_count_
            << " shm_write_fail_count=" << shm_write_fail_count_
            << " msg_type=" << msg_type
            << " raw_bytes=" << raw_msg->message.size()
            << " transform_count=" << shm_msg.transform_count
            << " timestamp=" << shm_msg.timestamp_sec
            << " recv_ok=true shm_write_ok=false"
            << " reason=queue_write_failed";
      return;
    }

    ++shm_write_count_;
    if (ShouldLog(shm_write_count_)) {
      AINFO << "[tf_bridge] source_topic=" << source_topic_
            << " target_topic=" << target_topic_
            << " recv_count=" << recv_count_
            << " shm_write_count=" << shm_write_count_
            << " msg_type=" << msg_type
            << " raw_bytes=" << raw_msg->message.size()
            << " transform_count=" << shm_msg.transform_count
            << " seq=" << shm_msg.sequence_num
            << " timestamp=" << shm_msg.timestamp_sec
            << " recv_ok=true shm_write_ok=true";
    }
  }

  bool ConvertToShm(const RawMessage& raw_msg, TransformStampedsShm* shm_msg,
                    std::string* msg_type_out) {
    std::string msg_type;
    TopologyManager::Instance()->channel_manager()->GetMsgType(source_topic_,
                                                               &msg_type);
    if (nullptr != msg_type_out) {
      *msg_type_out = msg_type;
    }
    if (msg_type.empty()) {
      if (1 == (++unknown_type_count_ % 100)) {
        AWARN << "Waiting for channel type of topic: " << source_topic_;
      }
      return false;
    }

    if (localization_estimate_type_ == msg_type) {
      localization::LocalizationEstimate localization;
      if (!localization.ParseFromString(raw_msg.message)) {
        AWARN << "Failed to parse LocalizationEstimate from topic: "
              << source_topic_;
        return false;
      }
      FillTransformStampedsShmFromLocalization(localization, shm_msg);
      return true;
    }

    if (pose_with_cov_type_ == msg_type) {
      localization::PoseWithCov pose_with_cov;
      if (!pose_with_cov.ParseFromString(raw_msg.message)) {
        AWARN << "Failed to parse PoseWithCov from topic: " << source_topic_;
        return false;
      }
      FillTransformStampedsShmFromPoseWithCov(pose_with_cov, shm_msg);
      return true;
    }

    if (1 == (++unsupported_type_count_ % 100)) {
      AWARN << "Unsupported message type on " << source_topic_ << ": "
            << msg_type;
    }
    return false;
  }

 private:
  std::string source_topic_;
  std::string target_topic_;
  std::string localization_estimate_type_;
  std::string pose_with_cov_type_;
  size_t recv_count_ = 0;
  size_t shm_write_count_ = 0;
  size_t convert_fail_count_ = 0;
  size_t shm_write_fail_count_ = 0;
  size_t unknown_type_count_ = 0;
  size_t unsupported_type_count_ = 0;
  std::unique_ptr<shm_queue<TransformStampedsShm>> queue_;
  std::shared_ptr<cyber::Reader<RawMessage>> reader_;
};

}  // namespace

int Run(int argc, char** argv) {
  if (!LoadCyberRosBridgeRuntimeConfig(&g_runtime_config)) {
    return 1;
  }
  ConfigureDefaultLogDir();
  cyber::Init(argv[0]);
  const auto& bridge_config = RuntimeConfig().lidar_shm_bridge;
  auto node = cyber::CreateNode(bridge_config.node_name);

  std::vector<std::unique_ptr<LidarTopicBridge>> bridges;
  bridges.reserve(bridge_config.lidar_configs.size());
  for (const auto& config : bridge_config.lidar_configs) {
    bridges.push_back(std::make_unique<LidarTopicBridge>(node.get(), config));
  }
  auto tf_static_bridge =
      std::make_unique<TfStaticBridge>(node.get(), bridge_config);
  auto localization_tf_bridge =
      std::make_unique<LocalizationTfBridge>(node.get(), bridge_config);

  AINFO << "Lidar shm bridge is running.";
  cyber::WaitForShutdown();
  return 0;
}

}  // namespace tools
}  // namespace century

int main(int argc, char** argv) {
  return century::tools::Run(argc, argv);
}
