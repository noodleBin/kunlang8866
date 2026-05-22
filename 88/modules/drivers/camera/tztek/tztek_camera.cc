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

#include "tztek_camera.h"

#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

#include "enabled_camera_mask.h"
#include "platform_config.h"
#include "v4l2_buffer_config.h"
#include "v4l2_thread_config.h"
#include "v4l2_timing_config.h"

namespace {
const char *PixelFormatToString(uint32_t format) {
  switch (format) {
    case 0:
      return "RAW";
    case 1:
      return "YUYV";
    case 2:
      return "UYVY";
    default:
      return "UNKNOWN";
  }
}

bool WriteSysfsValue(const char *path, const char *value) {
  if (path == nullptr || value == nullptr) {
    return false;
  }
  const int fd = open(path, O_WRONLY | O_CLOEXEC);
  if (fd < 0) {
    AERROR << "open sysfs failed, path = " << path << ", errno = " << errno
           << ", " << strerror(errno);
    return false;
  }
  const size_t len = strlen(value);
  const ssize_t written = write(fd, value, len);
  const int saved_errno = errno;
  close(fd);
  if (written != static_cast<ssize_t>(len)) {
    AERROR << "write sysfs failed, path = " << path << ", errno = "
           << saved_errno << ", " << strerror(saved_errno);
    return false;
  }
  return true;
}
}  // namespace

namespace century {
namespace drivers {
namespace camera {

TztekCamera::TztekCamera() : node_(century::cyber::CreateNode("camera")) {}

TztekCamera::~TztekCamera() {}

century::cyber::proto::QosProfile TztekCamera::CreateQosProfile() {
  century::cyber::proto::QosProfile qos;
  qos.set_history(century::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
  qos.set_reliability(
      century::cyber::proto::QosReliabilityPolicy::RELIABILITY_BEST_EFFORT);
  qos.set_durability(
      century::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
  return qos;
}

std::shared_ptr<Writer<Image>> TztekCamera::CreateImageWriter(
    const std::string channel) {
  if (!node_) {
    AERROR << "CreateImageWriter failed: node is null.";
    return nullptr;
  }
  century::cyber::proto::RoleAttributes writer_attr;
  writer_attr.set_channel_name(channel);
  *writer_attr.mutable_qos_profile() = CreateQosProfile();
  return node_->CreateWriter<Image>(writer_attr);
}

std::shared_ptr<Writer<CompressedImage>>
TztekCamera::CreateCompressedImageWriter(const std::string channel) {
  if (!node_) {
    AERROR << "CreateCompressedImageWriter failed: node is null.";
    return nullptr;
  }
  century::cyber::proto::RoleAttributes writer_attr;
  writer_attr.set_channel_name(channel);
  *writer_attr.mutable_qos_profile() = CreateQosProfile();
  return node_->CreateWriter<CompressedImage>(writer_attr);
}

bool TztekCamera::Init(std::shared_ptr<Config> camera_config) {
  AINFO << "TztekCamera init.";
  cameras_.clear();
  if (!node_) {
    AERROR << "camera cyber node is null.";
    return false;
  }

  if (camera_config == nullptr) {
    AERROR << "camera_config is null.";
    return false;
  }
  const std::string hb_j5dev_path = ResolveHbJ5devPath(
      camera_config->platform_config().horizon_j5dev_path(),
      camera_config->hb_j5dev_path());
  if (hb_j5dev_path.empty()) {
    AERROR
        << "hb_j5dev_path and platform_config.horizon_j5dev_path are both empty.";
    return false;
  }

  const uint32_t enabled_camera_mask = ResolveEnabledCameraMask(
      camera_config->enabled_camera_mask(), [](const std::string& warning) {
        AWARN << warning;
      });
  const std::vector<int> channel_indices =
      ExpandEnabledCameraChannels(enabled_camera_mask);
  if (channel_indices.empty()) {
    AERROR << "no enabled camera channels by mask.";
    return false;
  }

  std::vector<std::string> configured_power_value_paths;
  for (const auto& path :
       camera_config->platform_config().camera_power_gpio().power_value_paths()) {
    configured_power_value_paths.push_back(path);
  }
  const std::vector<std::string> power_value_paths =
      ResolveCameraPowerValuePaths(configured_power_value_paths,
                                   [](const std::string& warning) {
                                     AWARN << warning;
                                   });

  if (ResetPower(power_value_paths) != 0) {
    AWARN << "ResetPower failed, continue initialization.";
  }

  int ret = hb_vin_init(0, hb_j5dev_path.c_str());
  if (ret < 0) {
    AERROR << "Failed to hb_vin_init.";
    return false;
  }

  output_raw_ = camera_config->output_raw();
  const uint32_t configured_jpeg_quality =
      camera_config->compress_conf().jpeg_quality();
  const int jpeg_quality = ResolveJpegQuality(configured_jpeg_quality);
  if ((configured_jpeg_quality < 1U || configured_jpeg_quality > 100U) &&
      configured_jpeg_quality !=
          static_cast<uint32_t>(kDefaultJpegQuality)) {
    AWARN << "invalid jpeg_quality: " << configured_jpeg_quality
          << ", fallback to default " << kDefaultJpegQuality;
  }
  const uint32_t configured_v4l2_buffer_count =
      camera_config->v4l2_buffer_count();
  const int v4l2_buffer_count =
      ResolveV4l2BufferCount(configured_v4l2_buffer_count);
  if ((configured_v4l2_buffer_count <
       static_cast<uint32_t>(kMinV4l2BufferCount) ||
       configured_v4l2_buffer_count >
           static_cast<uint32_t>(kMaxV4l2BufferCount)) &&
      configured_v4l2_buffer_count !=
          static_cast<uint32_t>(kDefaultV4l2BufferCount)) {
    AWARN << "invalid v4l2_buffer_count: " << configured_v4l2_buffer_count
          << ", fallback to default " << kDefaultV4l2BufferCount;
  }
  V4l2TimingOptions configured_v4l2_timing;
  configured_v4l2_timing.init_device_delay_ms =
      camera_config->v4l2_timing_config().init_device_delay_ms();
  configured_v4l2_timing.clear_buffer_timeout_ms =
      camera_config->v4l2_timing_config().clear_buffer_timeout_ms();
  configured_v4l2_timing.first_capture_delay_ms =
      camera_config->v4l2_timing_config().first_capture_delay_ms();
  configured_v4l2_timing.start_capture_retry_delay_ms =
      camera_config->v4l2_timing_config().start_capture_retry_delay_ms();
  configured_v4l2_timing.poll_min_timeout_ms =
      camera_config->v4l2_timing_config().poll_min_timeout_ms();
  configured_v4l2_timing.poll_max_timeout_ms =
      camera_config->v4l2_timing_config().poll_max_timeout_ms();
  configured_v4l2_timing.grab_failure_retry_delay_ms =
      camera_config->v4l2_timing_config().grab_failure_retry_delay_ms();
  const V4l2TimingOptions v4l2_timing_options =
      ResolveV4l2TimingOptions(configured_v4l2_timing,
                               [](const std::string& warning) {
                                 AWARN << warning;
                               });
  V4l2ThreadOptions configured_v4l2_thread;
  configured_v4l2_thread.enable_realtime_scheduling =
      camera_config->v4l2_thread_config().enable_realtime_scheduling();
  configured_v4l2_thread.realtime_priority =
      camera_config->v4l2_thread_config().realtime_priority();
  configured_v4l2_thread.enable_cpu_affinity =
      camera_config->v4l2_thread_config().enable_cpu_affinity();
  configured_v4l2_thread.cpu_affinity_core_count =
      camera_config->v4l2_thread_config().cpu_affinity_core_count();
  for (const auto& affinity :
       camera_config->v4l2_thread_config().channel_cpu_affinities()) {
    configured_v4l2_thread.channel_cpu_affinities[affinity.channel()] =
        affinity.core();
  }
  const V4l2ThreadOptions v4l2_thread_options =
      ResolveV4l2ThreadOptions(configured_v4l2_thread,
                               [](const std::string& warning) {
                                 AWARN << warning;
                               });
  const int cam_num = static_cast<int>(channel_indices.size());
  for (int i = 0; i < cam_num; i++) {
    std::string raw_output_channel;
    std::string compressed_output_channel;
    std::string frame_id;
    bool topic_found = false;
    const int pipe_id = channel_indices[i];
    uint32_t video = 0, width = 0, height = 0, fps = 0, format = 0;
    ret = hb_port_mapping(pipe_id, &video, &width, &height, &fps, &format);
    if (ret != 0) {
      AERROR << "Failed to hb_port_mapping, pipeid = " << pipe_id;
      cameras_.clear();
      return false;
    }
    AINFO << "camera i: " << i << ", pipeid: " << pipe_id << ", video: " << video
          << ", width: " << width << ", height: " << height << ", fps: " << fps
          << ", format: " << PixelFormatToString(format);
    for (const auto &topic_config : camera_config->topic_configs()) {
      const int topic_index = topic_config.video_index();
      if (topic_index == static_cast<int>(video) || topic_index == pipe_id ||
          topic_index == i) {
        topic_found = true;
        raw_output_channel = topic_config.raw_output_channel();
        compressed_output_channel = topic_config.compressed_output_channel();
        frame_id = topic_config.frame_id();
        break;
      }
    }
    if (!topic_found) {
      char tmp[256] = {0};
      (void)snprintf(tmp, sizeof(tmp), "/century/sensor/camera/video%d/image", i);
      raw_output_channel = std::string(tmp);
      (void)snprintf(tmp, sizeof(tmp),
                     "/century/sensor/camera/video%d/image/compressed", i);
      compressed_output_channel = std::string(tmp);
      (void)snprintf(tmp, sizeof(tmp), "camera_video%d", i);
      frame_id = std::string(tmp);
    } else {
      char tmp[256] = {0};
      if (raw_output_channel.empty()) {
        (void)snprintf(tmp, sizeof(tmp), "/century/sensor/camera/video%d/image",
                       i);
        raw_output_channel = std::string(tmp);
      }
      if (frame_id.empty()) {
        (void)snprintf(tmp, sizeof(tmp), "camera_video%d", i);
        frame_id = std::string(tmp);
      }
    }
    AINFO << "camera pipeline i: " << i << ", raw="
          << (output_raw_ ? raw_output_channel : std::string("disabled"))
          << ", compressed="
          << (compressed_output_channel.empty() ? std::string("disabled")
                                                : compressed_output_channel)
          << ", frame_id=" << frame_id;

    std::shared_ptr<Writer<Image>> raw_image_writer = nullptr;
    if (output_raw_) {
      raw_image_writer = CreateImageWriter(raw_output_channel);
      if (raw_image_writer == nullptr) {
        AERROR << "failed to create raw writer, channel = " << raw_output_channel;
        cameras_.clear();
        return false;
      }
    }
    std::shared_ptr<Writer<CompressedImage>> compressed_image_writer = nullptr;
    if (!compressed_output_channel.empty()) {
      compressed_image_writer =
          CreateCompressedImageWriter(compressed_output_channel);
      if (compressed_image_writer == nullptr) {
        AERROR << "failed to create compressed writer, channel = "
               << compressed_output_channel;
        cameras_.clear();
        return false;
      }
    }
    width = camera_config->width();
    height = camera_config->height();
    const int crop_mode = static_cast<int>(camera_config->crop_mode());
    auto camera = std::unique_ptr<CameraManager>(new CameraManager(
        pipe_id, video, width, height, fps, format, raw_image_writer,
        compressed_image_writer, frame_id, output_raw_, crop_mode,
        jpeg_quality, v4l2_buffer_count, v4l2_timing_options,
        v4l2_thread_options));
    if (!camera->Init()) {
      AERROR << "Failed to do camera: " << i << " camera->Init().";
      cameras_.clear();
      return false;
    }
    cameras_.push_back(std::move(camera));
  }
  AINFO << "All cameras finished camera->Init().";

  for (auto &camera : cameras_) {
    if (camera) {
      camera->Start();
    }
  }

  return true;
}

int TztekCamera::ResetPower(const std::vector<std::string>& power_value_paths) {
  AINFO << "start reset power.";
  bool ok = true;
  for (const auto& node : power_value_paths) {
    if (!WriteSysfsValue(node.c_str(), "0")) {
      ok = false;
    }
  }
  sleep(1);
  AINFO << "power is off.";
  for (const auto& node : power_value_paths) {
    if (!WriteSysfsValue(node.c_str(), "1")) {
      ok = false;
    }
  }
  sleep(1);
  AINFO << "power on.";
  return ok ? 0 : -1;
}
}  // namespace camera
}  // namespace drivers
}  // namespace century
