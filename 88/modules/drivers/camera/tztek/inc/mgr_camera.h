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

#pragma once

#include <sys/time.h>
#include <unistd.h>

#include <chrono>
#include <vector>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "camera_type.h"
#include "jpeg_quality.h"
#include "jpeg_encode_api.h"
#include "libyuv.h"
#include "mgr_camera_jpegenc.h"
#include "pixel_format_utils.h"
#include "v4l2_buffer_config.h"
#include "v4l2_thread_config.h"
#include "v4l2_timing_config.h"
#include "v4l2cam.hpp"
#include "vin_log.h"

#include "modules/drivers/camera/proto/config.pb.h"
#include "modules/drivers/proto/sensor_image.pb.h"

#include "cyber/cyber.h"

namespace century {
namespace drivers {
namespace camera {
using century::cyber::Reader;
using century::cyber::Writer;
using century::drivers::Image;
using century::drivers::camera::config::Config;

class CameraManager {
 public:
  CameraManager(int pipe_id, int video_index, int width, int height,
              int fps, int format, std::shared_ptr<Writer<Image>> image_writer,
              std::shared_ptr<Writer<CompressedImage>> compressed_image_writer,
              const std::string frame_id, bool output_raw, int crop_mode,
              int jpeg_quality, int v4l2_buffer_count,
              const V4l2TimingOptions& v4l2_timing_options,
              const V4l2ThreadOptions& v4l2_thread_options);
  ~CameraManager();
  bool Init();
  void Start();

  static void CallbackImage(int channel, struct timespec frame_time, int width,
                            int height, unsigned char *data, int data_len,
                            void *user_data);

  friend void CAMERA_CALL JpegEncCallback(unsigned char *data, int data_len,
                                          void *user_data);

  long timestamp_ms_;

 private:
  bool CreateHandle();    
  bool InitHandle();
  void SetCamPublish();
  void UpdateRawEncodingByPixelFormat(uint32_t pixfmt);
  bool IsJpegSupportedPixelFormat(uint32_t pixfmt) const;
  void PublishRawImage(struct timespec frame_time, int width, int height,
                       const unsigned char *data, int data_len);

  // YUV422 缩放函数（按裁剪模式）
  bool ScaleYUV4221920x1536To1280x720ByCropMode(const uint8_t* src,
                                                 int src_stride,
                                                 uint8_t* dst,
                                                 int dst_stride);

 private:
  std::mutex mutex_;         
  std::unique_ptr<V4l2Camera> camera_ = nullptr;
  uint32_t height_;
  uint32_t width_;
  int channel_;
  int fps_;
  int config_format_;
  int video_index_;
  uint32_t configured_width_;
  uint32_t configured_height_;
  bool output_1280_mode_{false};
  int crop_mode_{0};
  uint32_t negotiated_pixel_format_{0};
  long frame_counter_;
  long previous_timestamp_ms_{0};
  std::string frame_id_;
  std::string raw_encoding_{"yuyv"};
  std::unique_ptr<CameraJpegEncoder> jpeg_encoder_ = nullptr;
  std::shared_ptr<Writer<Image>> image_writer_ = nullptr;
  std::shared_ptr<Writer<CompressedImage>> compressed_image_writer_ = nullptr;
  bool output_raw_ = false;
  bool started_{false};
  int jpeg_quality_{kDefaultJpegQuality};
  int v4l2_buffer_count_{kDefaultV4l2BufferCount};
  V4l2TimingOptions v4l2_timing_options_;
  V4l2ThreadOptions v4l2_thread_options_;

  // 缩放相关 buffer
  struct ScaleBuffers {
    // 源 I420 (1920×1536): Y + U + V
    std::vector<uint8_t> src_i420;
    // 目标 I420 (1280×720): Y + U + V
    std::vector<uint8_t> dst_i420;
    // 目标 YUYV (1280×720)
    std::vector<uint8_t> dst_yuyv;
  } scale_bufs_;
  bool scale_initialized_ = false;
};
}  // namespace camera
}  // namespace drivers
}  // namespace century
