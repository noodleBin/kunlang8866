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

#include "mgr_camera.h"

#include <linux/videodev2.h>
#include <sys/stat.h>
#include <chrono>
#include <ctime>
#include <cerrno>
#include <iomanip>
#include <map>
#include <sstream>
#include <vector>

// #define SAVE_PIC
namespace century {
namespace drivers {
namespace camera {
namespace {
constexpr int kCaptureWidth = 1920;
constexpr int kCaptureHeight = 1536;
constexpr int kOutputWidth1920 = 1920;
constexpr int kOutputHeight1920 = 1536;
constexpr int kOutputWidth1280 = 1280;
constexpr int kOutputHeight1280 = 720;
constexpr int kCropModeTop = 0;
constexpr int kCropModeCenter = 1;
constexpr int kCropModeBottom = 2;
#if CAMERA_LATENCY_PROBE_ENABLED
constexpr double kLatencyProbeWarnMs = 60.0;
constexpr double kLatencyProbeStageWarnMs = 20.0;

double TimespecToSec(const timespec& ts) {
  return static_cast<double>(ts.tv_sec) +
         static_cast<double>(ts.tv_nsec) / 1000000000.0;
}

template <typename ClockTimePoint>
double DurationMs(ClockTimePoint end, ClockTimePoint beginning) {
  return static_cast<double>(
             std::chrono::duration_cast<std::chrono::microseconds>(
                 end - beginning)
                 .count()) /
         1000.0;
}
#endif  // CAMERA_LATENCY_PROBE_ENABLED

const char *CropModeToString(int crop_mode) {
  switch (crop_mode) {
    case kCropModeTop:
      return "TOP";
    case kCropModeCenter:
      return "CENTER";
    case kCropModeBottom:
      return "BOTTOM";
    default:
      return "UNKNOWN";
  }
}

uint32_t ConfigFormatToV4l2PixFmt(int format) {
  switch (format) {
    case 0:
      return V4L2_PIX_FMT_SRGGB12;
    case 2:
      return V4L2_PIX_FMT_UYVY;
    case 1:
    default:
      return V4L2_PIX_FMT_YUYV;
  }
}

const char *V4l2PixFmtToString(uint32_t pixfmt) {
  switch (pixfmt) {
    case V4L2_PIX_FMT_YUYV:
      return "YUYV";
    case V4L2_PIX_FMT_UYVY:
      return "UYVY";
    case V4L2_PIX_FMT_YVYU:
      return "YVYU";
    case V4L2_PIX_FMT_VYUY:
      return "VYUY";
    case V4L2_PIX_FMT_SRGGB12:
      return "SRGGB12";
    default:
      return "UNKNOWN";
  }
}

double NormalizeTo100ms(const timespec& ts) {
  constexpr int64_t kNsPerSec = 1'000'000'000LL;
  constexpr int64_t kNsPerMs  = 1'000'000LL;
  // 
  int64_t ms_in_sec = ts.tv_nsec / kNsPerMs;  // [0, 999]
  int64_t ms_100 = (ms_in_sec / 100) * 100;

  int64_t new_ns =
      static_cast<int64_t>(ts.tv_sec) * kNsPerSec +
      ms_100 * kNsPerMs +
      (ts.tv_nsec - ms_in_sec * kNsPerMs);

  return static_cast<double>(new_ns) / kNsPerSec;
}
}  // namespace

CameraManager::CameraManager(
    int pipe_id, int video_index, int width, int height, int fps,
    int format, std::shared_ptr<Writer<Image>> image_writer,
    std::shared_ptr<Writer<CompressedImage>> compressed_image_writer,
    const std::string frame_id, bool output_raw, int crop_mode,
    int jpeg_quality, int v4l2_buffer_count,
    const V4l2TimingOptions& v4l2_timing_options,
    const V4l2ThreadOptions& v4l2_thread_options) {
  channel_ = pipe_id;
  video_index_ = video_index;
  configured_width_ = static_cast<uint32_t>(width);
  configured_height_ = static_cast<uint32_t>(height);
  output_1280_mode_ =
      (configured_width_ == kOutputWidth1280 && configured_height_ == kOutputHeight1280);
  width_ = kCaptureWidth;
  height_ = kCaptureHeight;
  fps_ = fps;
  config_format_ = format;
  timestamp_ms_ = 0;
  frame_counter_ = 0;
  frame_id_ = frame_id;
  image_writer_ = image_writer;
  compressed_image_writer_ = compressed_image_writer;
  output_raw_ = output_raw;
  crop_mode_ = crop_mode;
  jpeg_quality_ = jpeg_quality;
  v4l2_buffer_count_ = v4l2_buffer_count;
  v4l2_timing_options_ = v4l2_timing_options;
  v4l2_thread_options_ = v4l2_thread_options;
  negotiated_pixel_format_ = ConfigFormatToV4l2PixFmt(config_format_);
  UpdateRawEncodingByPixelFormat(negotiated_pixel_format_);

  // 预分配缩放所需的 buffer
  // 源 I420 (1920×1536): Y + U + V
  scale_bufs_.src_i420.resize(1920 * 1536 + 960 * 768 + 960 * 768);
  // 目标 I420 (1280×720): Y + U + V
  scale_bufs_.dst_i420.resize(1280 * 720 + 640 * 360 + 640 * 360);
  // 目标 YUYV (1280×720)
  scale_bufs_.dst_yuyv.resize(1280 * 720 * 2);
  scale_initialized_ = true;
}

CameraManager::~CameraManager() {
  jpeg_encoder_.reset();
  camera_.reset();
  started_ = false;
}

bool CameraManager::Init() {
  started_ = false;
  if (!output_raw_ && compressed_image_writer_ == nullptr) {
    AWARN << "camera manager channel " << channel_
          << " has no enabled output writer.";
  }
  if (CreateHandle()) {
    AINFO << "camera manager channel " << channel_ << " CreateHandle success.";
    if (InitHandle()) {
      AINFO << "camera manager init channel " << channel_ << " success.";
      return true;
    } else {
      AERROR << "camera manager channel " << channel_ << " InitHandle failed.";
    }
  } else {
    AERROR << "camera manager channel " << channel_ << " CreateHandle failed.";
  }
  jpeg_encoder_.reset();
  camera_.reset();
  AERROR << "camera manager init channel " << channel_ << " failed.";
  return false;
}

void CameraManager::Start() {
  if (camera_ == nullptr) {
    AERROR << "camera manager channel " << channel_
           << " start failed: camera handle is null.";
    return;
  }
  if (started_) {
    return;
  }
  if (camera_->StartAcquire() < 0) {
    AERROR << "camera manager channel " << channel_
           << " StartAcquire failed.";
    return;
  }
  started_ = true;
}

bool CameraManager::CreateHandle() {
  camera_.reset(new V4l2Camera(channel_, video_index_, width_,
                                    height_, fps_, config_format_,
                                    v4l2_buffer_count_,
                                    v4l2_timing_options_,
                                    v4l2_thread_options_));
  jpeg_encoder_.reset();
  return true;
}

bool CameraManager::InitHandle() {
  if (camera_ == nullptr) {
    return false;
  }

  if (camera_->Init() < 0) {
    return false;
  }
  width_ = static_cast<uint32_t>(camera_->GetWidth());
  height_ = static_cast<uint32_t>(camera_->GetHeight());
  negotiated_pixel_format_ = camera_->GetPixelFormat();
  UpdateRawEncodingByPixelFormat(negotiated_pixel_format_);
  AINFO << "camera channel " << channel_ << " negotiated format: "
        << width_ << "x" << height_ << ", pixfmt = "
        << V4l2PixFmtToString(negotiated_pixel_format_) << "(" << negotiated_pixel_format_
        << ")";

  if (!((configured_width_ == kOutputWidth1920 && configured_height_ == kOutputHeight1920) ||
        (configured_width_ == kOutputWidth1280 && configured_height_ == kOutputHeight1280))) {
    AERROR << "unsupported configured resolution, channel " << channel_ << ": "
           << configured_width_ << "x" << configured_height_
           << ", only 1920x1536 and 1280x720 are allowed";
    return false;
  }
  output_1280_mode_ =
      (configured_width_ == kOutputWidth1280 && configured_height_ == kOutputHeight1280);

  if (crop_mode_ != kCropModeTop && crop_mode_ != kCropModeCenter &&
      crop_mode_ != kCropModeBottom) {
    AERROR << "unsupported crop_mode, channel " << channel_ << ": " << crop_mode_
           << ", only TOP(0)/CENTER(1)/BOTTOM(2) are allowed";
    return false;
  }

  AINFO << "camera channel " << channel_ << " output mode: "
        << (output_1280_mode_ ? "1280x720" : "1920x1536")
        << ", crop_mode=" << CropModeToString(crop_mode_) << "(" << crop_mode_ << ")";

  if (width_ != kCaptureWidth || height_ != kCaptureHeight) {
    AERROR << "unexpected negotiated resolution, channel " << channel_ << ": "
           << width_ << "x" << height_
           << ", expected fixed capture " << kCaptureWidth << "x" << kCaptureHeight;
    return false;
  }

  SetCamPublish();
  if (compressed_image_writer_ != nullptr) {
    if (!IsJpegSupportedPixelFormat(negotiated_pixel_format_)) {
      AWARN << "jpeg encoder disabled on channel " << channel_
            << " due to unsupported pixfmt: "
            << V4l2PixFmtToString(negotiated_pixel_format_) << "("
            << negotiated_pixel_format_ << ")";
      jpeg_encoder_.reset();
    } else {
      const int jpeg_out_width = output_1280_mode_ ? kOutputWidth1280 : kOutputWidth1920;
      const int jpeg_out_height = output_1280_mode_ ? kOutputHeight1280 : kOutputHeight1920;
      jpeg_encoder_.reset(new CameraJpegEncoder(
          channel_, jpeg_out_width, jpeg_out_height,
          jpeg_quality_, compressed_image_writer_, frame_id_));
      if (!jpeg_encoder_->Init()) {
        return false;
      }
    }
  }
  return true;
}

void CameraManager::SetCamPublish() {
  if (camera_) {
    camera_->SetDataCallback(&CameraManager::CallbackImage, this);
  }
}

void CameraManager::PublishRawImage(struct timespec frame_timespec, int width, int height,
                                 const unsigned char *data, int data_len) {
  if (!output_raw_ || image_writer_ == nullptr) {
    return;
  }
  if (data == nullptr || data_len <= 0 || width <= 0 || height <= 0) {
    AWARN << "PublishRawImage invalid input, chan = " << channel_
          << ", size = " << data_len << ", w = " << width
          << ", h = " << height;
    return;
  }

  // auto header_time = NormalizeTo100ms(frame_timespec);
  static std::atomic<uint64_t> sequence_num = {0};
  auto header_time = (double)frame_timespec.tv_sec + (double)frame_timespec.tv_nsec / 1e9;
  auto measurement_time = century::cyber::Time::Now().ToSecond();
  // auto delay_time = measurement_time - header_time;
  // AINFO << "raw image send time delay: " << delay_time << " s.";
  // if (delay_time >= 0.1) {
  //   AWARN << "raw image send time delay: " << delay_time << " s.";
  // }
  
  auto image = std::make_shared<Image>();
  image->mutable_header()->set_frame_id(frame_id_);
  image->mutable_header()->set_timestamp_sec(header_time);
  image->mutable_header()->set_sequence_num(
    static_cast<unsigned int>(sequence_num.fetch_add(1)));
  image->set_measurement_time(measurement_time);
  image->set_width(width);
  image->set_height(height);
  image->set_encoding(raw_encoding_);

  int step = CalculatePackedImageStride(width, negotiated_pixel_format_);
  if (height > 0 && data_len >= height) {
    const int inferred_step = data_len / height;
    if (inferred_step > step) {
      step = inferred_step;
    }
  }
  image->set_step(step);
  image->set_data(reinterpret_cast<const char *>(data), data_len);
  image_writer_->Write(image);
}

bool CameraManager::ScaleYUV4221920x1536To1280x720ByCropMode(const uint8_t* src,
                                                              int src_stride,
                                                              uint8_t* dst,
                                                              int dst_stride) {
  using namespace std::chrono;

  if (!scale_initialized_) {
    AERROR << "Scale buffers not initialized";
    return false;
  }

  // Step 1: YUYV -> I420 (1920×1536)
  auto t1 = high_resolution_clock::now();

  uint8_t* src_y = scale_bufs_.src_i420.data();
  uint8_t* src_u = src_y + 1920 * 1536;
  uint8_t* src_v = src_u + 960 * 768;

  int ret = -1;
  if (negotiated_pixel_format_ == V4L2_PIX_FMT_YUYV) {
    ret = libyuv::YUY2ToI420(src, src_stride,
                             src_y, 1920,
                             src_u, 960,
                             src_v, 960,
                             1920, 1536);
  } else if (negotiated_pixel_format_ == V4L2_PIX_FMT_UYVY) {
    ret = libyuv::UYVYToI420(src, src_stride,
                             src_y, 1920,
                             src_u, 960,
                             src_v, 960,
                             1920, 1536);
  } else {
    AERROR << "Step 1 unsupported pixel format for scaling: "
           << V4l2PixFmtToString(negotiated_pixel_format_) << "("
           << negotiated_pixel_format_ << ")";
    return false;
  }
  if (ret != 0) {
    AERROR << "Step 1 (YUV422ToI420) failed, ret=" << ret
           << ", pixfmt=" << V4l2PixFmtToString(negotiated_pixel_format_)
           << "(" << negotiated_pixel_format_ << ")";
    return false;
  }
  auto t2 = high_resolution_clock::now();

  // Step 2: Crop + Scale to 1280×720
  // TOP: crop_y=456, CENTER: crop_y=228, BOTTOM: crop_y=0
  int crop_y = 0;
  if (crop_mode_ == kCropModeTop) {
    crop_y = 456;
  } else if (crop_mode_ == kCropModeCenter) {
    crop_y = 228;
  } else if (crop_mode_ == kCropModeBottom) {
    crop_y = 0;
  } else {
    AERROR << "unsupported crop_mode in scaling, channel=" << channel_
           << ", crop_mode=" << crop_mode_;
    return false;
  }

  const int crop_y_uv = crop_y / 2;
  const int crop_height = 1080;      // 1536 - 456

  uint8_t* dst_y = scale_bufs_.dst_i420.data();
  uint8_t* dst_u = dst_y + 1280 * 720;
  uint8_t* dst_v = dst_u + 640 * 360;

  ret = libyuv::I420Scale(src_y + crop_y * 1920, 1920,
                          src_u + crop_y_uv * 960, 960,
                          src_v + crop_y_uv * 960, 960,
                          1920, crop_height,  // Source size after crop
                          dst_y, 1280,
                          dst_u, 640,
                          dst_v, 640,
                          1280, 720,
                          libyuv::kFilterNone);
  if (ret != 0) {
    AERROR << "Step 2 (I420Scale) failed, ret=" << ret;
    return false;
  }
  auto t3 = high_resolution_clock::now();

  // Step 3: I420 -> YUV422 packed (1280×720)
  if (negotiated_pixel_format_ == V4L2_PIX_FMT_YUYV) {
    ret = libyuv::I420ToYUY2(dst_y, 1280,
                             dst_u, 640,
                             dst_v, 640,
                             dst, dst_stride,
                             1280, 720);
    if (ret != 0) {
      AERROR << "Step 3 (I420ToYUY2) failed, ret=" << ret
             << ", pixfmt=" << V4l2PixFmtToString(negotiated_pixel_format_)
             << "(" << negotiated_pixel_format_ << ")";
      return false;
    }
  } else if (negotiated_pixel_format_ == V4L2_PIX_FMT_UYVY) {
    ret = libyuv::I420ToUYVY(dst_y, 1280,
                             dst_u, 640,
                             dst_v, 640,
                             dst, dst_stride,
                             1280, 720);
    if (ret != 0) {
      AERROR << "Step 3 (I420ToUYVY) failed, ret=" << ret
             << ", pixfmt=" << V4l2PixFmtToString(negotiated_pixel_format_)
             << "(" << negotiated_pixel_format_ << ")";
      return false;
    }
  } else {
    AERROR << "Step 3 unsupported pixel format for scaling: "
           << V4l2PixFmtToString(negotiated_pixel_format_) << "("
           << negotiated_pixel_format_ << ")";
    return false;
  }
  auto t4 = high_resolution_clock::now();

  // Log timing for each step
  auto step1_us = duration_cast<microseconds>(t2 - t1).count();
  auto step2_us = duration_cast<microseconds>(t3 - t2).count();
  auto step3_us = duration_cast<microseconds>(t4 - t3).count();
  auto total_us = duration_cast<microseconds>(t4 - t1).count();

#if CAMERA_LATENCY_PROBE_ENABLED
  if (total_us >= static_cast<int64_t>(kLatencyProbeStageWarnMs * 1000.0)) {
    AINFO << "Scale timing: step1=" << step1_us << "us, step2=" << step2_us
          << "us, step3=" << step3_us << "us, total=" << total_us << "us";
  }
#endif  // CAMERA_LATENCY_PROBE_ENABLED

  return true;
}

void CameraManager::CallbackImage(int channel, struct timespec frame_time, int width,
                               int height, unsigned char *data, int data_len,
                               void *user_data) {
#if CAMERA_LATENCY_PROBE_ENABLED
  const auto callback_begin = std::chrono::steady_clock::now();
#endif  // CAMERA_LATENCY_PROBE_ENABLED
  CameraManager *camera_mgr = static_cast<CameraManager *>(user_data);
  if (camera_mgr == nullptr || data == nullptr || data_len <= 0) {
    return;
  }

  if (width <= 0 || height <= 0) {
    AWARN << "invalid frame size, chan = " << channel << ", w = " << width
          << ", h = " << height << ", size = " << data_len;
    return;
  }

  struct timeval tv;
  gettimeofday(&tv, nullptr);
  {
    std::lock_guard<std::mutex> lock(camera_mgr->mutex_);
    camera_mgr->previous_timestamp_ms_ = tv.tv_sec * 1000 + tv.tv_usec / 1000;
    camera_mgr->timestamp_ms_ = frame_time.tv_sec * 1000 + frame_time.tv_nsec / 1000000;
    camera_mgr->frame_counter_++;
  }
#if CAMERA_LATENCY_PROBE_ENABLED
  const auto metadata_done = std::chrono::steady_clock::now();
#endif  // CAMERA_LATENCY_PROBE_ENABLED

  // 准备输出参数（可能被模式修改）
  constexpr int kJpegI420Size1920 = kOutputWidth1920 * kOutputHeight1920 * 3 / 2;
  constexpr int kJpegI420Size1280 = kOutputWidth1280 * kOutputHeight1280 * 3 / 2;
  int out_width = width;
  int out_height = height;
  unsigned char* out_data = data;
  int out_data_len = data_len;

  int jpeg_width = 0;
  int jpeg_height = 0;
  int jpeg_i420_size = 0;
  unsigned char* jpeg_i420_data = nullptr;
  bool jpeg_ready = false;

  int input_stride = CalculatePackedImageStride(
      width, camera_mgr->negotiated_pixel_format_);
  if (height > 0 && data_len >= height) {
    int inferred_stride = data_len / height;
    if (inferred_stride >= input_stride) {
      input_stride = inferred_stride;
    }
  }
  const int64_t required_input_size =
      static_cast<int64_t>(input_stride) * static_cast<int64_t>(height);
  if (static_cast<int64_t>(data_len) < required_input_size) {
    AERROR << "Skip frame due to insufficient input buffer, channel=" << channel
           << ", width=" << width << ", height=" << height
           << ", input_stride=" << input_stride << ", data_len=" << data_len
           << ", required_input_size=" << required_input_size;
    return;
  }
#if CAMERA_LATENCY_PROBE_ENABLED
  const auto validation_done = std::chrono::steady_clock::now();
#endif  // CAMERA_LATENCY_PROBE_ENABLED

  if (camera_mgr->output_1280_mode_) {
    // 1280x720 模式：固定采集 1920x1536 后走缩放流程
    if (width == kCaptureWidth && height == kCaptureHeight &&
        camera_mgr->IsJpegSupportedPixelFormat(camera_mgr->negotiated_pixel_format_)) {
      const int dst_stride = kOutputWidth1280 * 2;
      if (camera_mgr->ScaleYUV4221920x1536To1280x720ByCropMode(
              data, input_stride,
              camera_mgr->scale_bufs_.dst_yuyv.data(), dst_stride)) {
        out_width = kOutputWidth1280;
        out_height = kOutputHeight1280;
        out_data = camera_mgr->scale_bufs_.dst_yuyv.data();
        out_data_len = kOutputWidth1280 * kOutputHeight1280 * 2;

        jpeg_width = kOutputWidth1280;
        jpeg_height = kOutputHeight1280;
        jpeg_i420_size = kJpegI420Size1280;
        jpeg_i420_data = camera_mgr->scale_bufs_.dst_i420.data();
        jpeg_ready = true;
      } else {
        AERROR << "Scale failed, skip frame for channel " << channel;
        return;  // 转换失败，跳过该帧
      }
    } else if (width == kCaptureWidth && height == kCaptureHeight) {
      static std::map<int, uint32_t> unsupported_scale_pixfmt_counts;
      static std::mutex unsupported_scale_pixfmt_counts_mutex;
      uint32_t warn_count = 0;
      {
        std::lock_guard<std::mutex> lock(unsupported_scale_pixfmt_counts_mutex);
        uint32_t& warn_count_ref = unsupported_scale_pixfmt_counts[channel];
        ++warn_count_ref;
        warn_count = warn_count_ref;
      }
      if (warn_count == 1 || warn_count % 100 == 0) {
        AWARN << "Skip scaling for unsupported pixfmt on 1920x1536 frame, channel="
              << channel << ", pixfmt="
              << V4l2PixFmtToString(camera_mgr->negotiated_pixel_format_)
              << "(" << camera_mgr->negotiated_pixel_format_ << ")"
              << ", warn_count=" << warn_count;
      }
      return;
    } else {
      AWARN << "Unexpected input resolution in 1280 mode, skip frame, channel="
            << channel << ", width=" << width << ", height=" << height;
      return;
    }
  } else {
    // 1920x1536 模式：不缩放，直接将输入转 I420 后送 JPEG
    if (width == kCaptureWidth && height == kCaptureHeight &&
        camera_mgr->IsJpegSupportedPixelFormat(camera_mgr->negotiated_pixel_format_)) {
      uint8_t* src_y = camera_mgr->scale_bufs_.src_i420.data();
      uint8_t* src_u = src_y + kCaptureWidth * kCaptureHeight;
      uint8_t* src_v = src_u + kCaptureWidth * kCaptureHeight / 4;

      int ret = -1;
      if (camera_mgr->negotiated_pixel_format_ == V4L2_PIX_FMT_YUYV) {
        ret = libyuv::YUY2ToI420(data, input_stride, src_y, kCaptureWidth,
                                 src_u, kCaptureWidth / 2, src_v,
                                 kCaptureWidth / 2, kCaptureWidth,
                                 kCaptureHeight);
      } else if (camera_mgr->negotiated_pixel_format_ == V4L2_PIX_FMT_UYVY) {
        ret = libyuv::UYVYToI420(data, input_stride, src_y, kCaptureWidth,
                                 src_u, kCaptureWidth / 2, src_v,
                                 kCaptureWidth / 2, kCaptureWidth,
                                 kCaptureHeight);
      }

      if (ret != 0) {
        AERROR << "YUV422ToI420 for JPEG failed, channel=" << channel
               << ", ret=" << ret << ", pixfmt="
               << V4l2PixFmtToString(camera_mgr->negotiated_pixel_format_) << "("
               << camera_mgr->negotiated_pixel_format_ << ")";
        return;
      }

      jpeg_width = kOutputWidth1920;
      jpeg_height = kOutputHeight1920;
      jpeg_i420_size = kJpegI420Size1920;
      jpeg_i420_data = camera_mgr->scale_bufs_.src_i420.data();
      jpeg_ready = true;
    }
  }
#if CAMERA_LATENCY_PROBE_ENABLED
  const auto pre_publish_done = std::chrono::steady_clock::now();
  const double callback_to_publish_delay_ms =
      (century::cyber::Time::Now().ToSecond() - TimespecToSec(frame_time)) *
      1000.0;
#endif  // CAMERA_LATENCY_PROBE_ENABLED

  // 发布可能经过缩放的图像
  camera_mgr->PublishRawImage(frame_time, out_width, out_height,
                               out_data, out_data_len);
#if CAMERA_LATENCY_PROBE_ENABLED
  const auto publish_done = std::chrono::steady_clock::now();
  const auto jpeg_begin = std::chrono::steady_clock::now();
#endif  // CAMERA_LATENCY_PROBE_ENABLED

  // JPEG 使用模式对应的 I420 数据
  if (camera_mgr->jpeg_encoder_ != nullptr &&
      camera_mgr->compressed_image_writer_ != nullptr) {
    if (jpeg_ready && jpeg_i420_data != nullptr &&
        jpeg_width > 0 && jpeg_height > 0 && jpeg_i420_size > 0) {
      camera_mgr->jpeg_encoder_->SaveI420(channel, frame_time,
                                          jpeg_width, jpeg_height,
                                          jpeg_i420_data,
                                          jpeg_i420_size);
    } else {
      static std::map<int, uint32_t> jpeg_skip_counts;
      static std::mutex jpeg_skip_counts_mutex;
      uint32_t skip_count = 0;
      {
        std::lock_guard<std::mutex> lock(jpeg_skip_counts_mutex);
        uint32_t& skip_count_ref = jpeg_skip_counts[channel];
        ++skip_count_ref;
        skip_count = skip_count_ref;
      }
      if (skip_count == 1 || skip_count % 100 == 0) {
        AWARN << "Skip JPEG for frame, channel=" << channel
              << ", mode=" << (camera_mgr->output_1280_mode_ ? "1280" : "1920")
              << ", width=" << width << ", height=" << height
              << ", skip_count=" << skip_count;
      }
    }
  }
#if CAMERA_LATENCY_PROBE_ENABLED
  const auto jpeg_done = std::chrono::steady_clock::now();
  const double metadata_ms = DurationMs(metadata_done, callback_begin);
  const double validation_ms = DurationMs(validation_done, metadata_done);
  const double pre_publish_ms = DurationMs(pre_publish_done, validation_done);
  const double publish_raw_ms = DurationMs(publish_done, pre_publish_done);
  const double jpeg_input_ms = DurationMs(jpeg_done, jpeg_begin);
  const double callback_total_ms = DurationMs(jpeg_done, callback_begin);
  if (callback_to_publish_delay_ms >= kLatencyProbeWarnMs ||
      pre_publish_ms >= kLatencyProbeStageWarnMs ||
      publish_raw_ms >= kLatencyProbeStageWarnMs ||
      jpeg_input_ms >= kLatencyProbeStageWarnMs ||
      callback_total_ms >= kLatencyProbeWarnMs) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9)
        << "[camera_latency_probe][callback]"
        << " chan=" << channel
        << ", frame_time_sec=" << TimespecToSec(frame_time)
        << std::setprecision(3)
        << ", callback_to_publish_delay_ms="
        << callback_to_publish_delay_ms
        << ", metadata_ms=" << metadata_ms
        << ", validation_ms=" << validation_ms
        << ", pre_publish_ms=" << pre_publish_ms
        << ", publish_raw_ms=" << publish_raw_ms
        << ", jpeg_input_ms=" << jpeg_input_ms
        << ", callback_total_ms=" << callback_total_ms
        << ", input=" << width << "x" << height
        << ", output=" << out_width << "x" << out_height
        << ", data_len=" << data_len
        << ", out_data_len=" << out_data_len
        << ", output_1280_mode="
        << (camera_mgr->output_1280_mode_ ? "true" : "false")
        << ", jpeg_ready=" << (jpeg_ready ? "true" : "false");
    AWARN << oss.str();
  }
#endif  // CAMERA_LATENCY_PROBE_ENABLED

  // 每秒保存一张图片用于对比（仅对 1920x1536 分辨率）
#ifdef SAVE_PIC
  if (width == 1920 && height == 1536) {
    static std::map<int, time_t> last_save_time;
    static std::mutex last_save_time_mutex;
    time_t current_time = frame_time.tv_sec;

    bool should_save = false;
    {
      std::lock_guard<std::mutex> lock(last_save_time_mutex);
      auto it = last_save_time.find(channel);
      if (it == last_save_time.end() || it->second != current_time) {
        last_save_time[channel] = current_time;
        should_save = true;
      }
    }

    // 检查是否需要保存（每秒一次）
    if (should_save) {
      
      // 创建目录（如果不存在）
      const char* save_dir = "/century/data/pics";
      struct stat st;
      if (stat(save_dir, &st) != 0) {
        // 目录不存在，尝试创建
        if (mkdir(save_dir, 0755) != 0 && errno != EEXIST) {
          AWARN << "Failed to create directory: " << save_dir << ", errno: " << errno;
        }
      }
      
      // 生成时间戳字符串
      char timestamp_str[32];
      struct tm tm_info;
      localtime_r(&current_time, &tm_info);
      strftime(timestamp_str, sizeof(timestamp_str), "%Y%m%d_%H%M%S", &tm_info);
      
      // 保存原始分辨率图像（1920x1536）
      char raw_filename[256];
      snprintf(raw_filename, sizeof(raw_filename), "%s/%s_chan%d_%dx%d.yuv",
               save_dir, timestamp_str, channel, width, height);
      FILE* raw_fp = fopen(raw_filename, "wb");
      if (raw_fp) {
        fwrite(data, 1, data_len, raw_fp);
        fclose(raw_fp);
        AINFO << "Saved raw image: " << raw_filename << " (" << data_len << " bytes)";
      } else {
        AERROR << "Failed to save raw image: " << raw_filename;
      }
      
      // 保存缩放后图像（1280x720）
      char scaled_filename[256];
      snprintf(scaled_filename, sizeof(scaled_filename), "%s/%s_chan%d_%dx%d.yuv",
               save_dir, timestamp_str, channel, out_width, out_height);
      FILE* scaled_fp = fopen(scaled_filename, "wb");
      if (scaled_fp) {
        fwrite(out_data, 1, out_data_len, scaled_fp);
        fclose(scaled_fp);
        AINFO << "Saved scaled image: " << scaled_filename << " (" << out_data_len << " bytes)";
      } else {
        AERROR << "Failed to save scaled image: " << scaled_filename;
      }
    }
  }
  #endif
}

void CameraManager::UpdateRawEncodingByPixelFormat(uint32_t pixfmt) {
  switch (pixfmt) {
    case V4L2_PIX_FMT_UYVY:
      raw_encoding_ = "uyvy";
      break;
    case V4L2_PIX_FMT_YVYU:
      raw_encoding_ = "yvyu";
      break;
    case V4L2_PIX_FMT_VYUY:
      raw_encoding_ = "vyuy";
      break;
    case V4L2_PIX_FMT_SRGGB12:
      raw_encoding_ = "bayer_rggb12";
      break;
    case V4L2_PIX_FMT_YUYV:
    default:
      raw_encoding_ = "yuyv";
      break;
  }
}

bool CameraManager::IsJpegSupportedPixelFormat(uint32_t pixfmt) const {
  return pixfmt == V4L2_PIX_FMT_YUYV || pixfmt == V4L2_PIX_FMT_UYVY;
}
}  // namespace camera
}  // namespace drivers
}  // namespace century
