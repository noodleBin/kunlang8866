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

#include "v4l2cam.hpp"

#include <algorithm>
#include <assert.h>
#include <dlfcn.h>
#include <errno.h>
#include <exception>
#include <fcntl.h>
#include <getopt.h>
#include <iomanip>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <sched.h>
#include <vector>

#include "libyuv.h"
#include "vin_log.h"

static NvColorFormat kNvColorFormats[] = {
    /* TODO: add more pixel format mapping */
    {V4L2_PIX_FMT_UYVY, NVBUF_COLOR_FORMAT_UYVY},
    {V4L2_PIX_FMT_VYUY, NVBUF_COLOR_FORMAT_VYUY},
    {V4L2_PIX_FMT_YUYV, NVBUF_COLOR_FORMAT_YUYV},
    {V4L2_PIX_FMT_YVYU, NVBUF_COLOR_FORMAT_YVYU},
    {V4L2_PIX_FMT_GREY, NVBUF_COLOR_FORMAT_GRAY8},
    {V4L2_PIX_FMT_YUV420M, NVBUF_COLOR_FORMAT_YUV420},
};

static NvBufSurfaceColorFormat GetNvbufColorFormat(unsigned int v4l2_pixfmt) {
  for (unsigned int i = 0; i < sizeof(kNvColorFormats) / sizeof(kNvColorFormats[0]);
       i++) {
    if (v4l2_pixfmt == kNvColorFormats[i].v4l2_pixfmt) {
      return kNvColorFormats[i].nvbuf_color;
    }
  }

  return NVBUF_COLOR_FORMAT_INVALID;
}

#define difftimeval(end, beginning) \
  ((end.tv_sec - beginning.tv_sec) * 1000000 + end.tv_usec - beginning.tv_usec)

static constexpr uint32_t kDefaultV4l2VideoFormat = V4L2_PIX_FMT_YUYV;
#if CAMERA_LATENCY_PROBE_ENABLED
static constexpr double kLatencyProbeWarnMs = 60.0;
static constexpr double kLatencyProbeStageWarnMs = 20.0;

static double TimevalDiffMs(const timeval& end, const timeval& beginning) {
  return static_cast<double>(difftimeval(end, beginning)) / 1000.0;
}

static double TimespecToSec(const timespec& ts) {
  return static_cast<double>(ts.tv_sec) +
         static_cast<double>(ts.tv_nsec) / 1000000000.0;
}
#endif  // CAMERA_LATENCY_PROBE_ENABLED

static uint32_t ResolveV4l2PixFmt(int format) {
  switch (format) {
    case 0:
      return V4L2_PIX_FMT_SRGGB12;
    case 1:
      return V4L2_PIX_FMT_YUYV;
    case 2:
      return V4L2_PIX_FMT_UYVY;
    default:
      AINFO << "not support format:" << format << ", default use YUYV";
      return V4L2_PIX_FMT_YUYV;
  }
}

static bool IsStreamParmUnsupportedErrno(int err) {
  return (err == ENOTTY || err == EINVAL || err == EOPNOTSUPP ||
          err == ENOSYS);
}

static constexpr int kRecoverThreshold = 8;
static volatile sig_atomic_t g_sigint_received = 0;
static bool g_sigint_handler_registered = false;

static void HandleSigint(int signum) {
  if (signum == SIGINT) {
    g_sigint_received = 1;
  }
}

static void RegisterSigintHandlerIfDefault() {
  if (g_sigint_handler_registered) {
    return;
  }
  struct sigaction cur_action;
  memset(&cur_action, 0, sizeof(cur_action));
  if (sigaction(SIGINT, nullptr, &cur_action) != 0) {
    return;
  }
  if (cur_action.sa_handler != SIG_DFL) {
    g_sigint_handler_registered = true;
    return;
  }

  struct sigaction sig_action;
  memset(&sig_action, 0, sizeof(sig_action));
  sig_action.sa_handler = HandleSigint;
  sigemptyset(&sig_action.sa_mask);
  sig_action.sa_flags = SA_RESTART;
  if (sigaction(SIGINT, &sig_action, nullptr) == 0) {
    g_sigint_handler_registered = true;
  }
}

V4l2Camera::V4l2Camera(int pipe_id, int video_index, int width, int height,
                   int fps, int format, int buffer_count,
                   const century::drivers::camera::V4l2TimingOptions& timing_options,
                   const century::drivers::camera::V4l2ThreadOptions& thread_options) {
  video_fd_ = -1;
  channel_ = pipe_id;
  width_ = width;
  height_ = height;
  fps_ = fps;
  config_format_ = format;
  v4l2_pixel_format_ = kDefaultV4l2VideoFormat;
  buffer_count_ = century::drivers::camera::ResolveV4l2BufferCount(
      buffer_count >= 0 ? static_cast<uint32_t>(buffer_count) : 0u);
  if (buffer_count_ != buffer_count) {
    AWARN << "invalid v4l2 buffer count:" << buffer_count
          << ", fallback to " << buffer_count_;
  }
  timing_options_ = century::drivers::camera::ResolveV4l2TimingOptions(
      timing_options, [](const std::string& warning) { AWARN << warning; });
  thread_options_ = century::drivers::camera::ResolveV4l2ThreadOptions(
      thread_options, [](const std::string& warning) { AWARN << warning; });
  char device_node[64] = {0};
  (void)snprintf(device_node, sizeof(device_node), "/dev/video%d", video_index);
  device_name_ = device_node;
  is_running_.store(false);
  is_streaming_ = false;
  fault_ = 0;
  error_code_ = 0;
  is_first_capture_ = true;
  consecutive_grab_fail_count_ = 0;
  last_sequence_ = 0;
  last_dq_errno_ = 0;
  is_stream_param_unsupported_ = false;
  frame_count_ = 0;
  skipped_frame_count_ = 0;
  last_frame_timestamp_ = 0;
  surface_ = nullptr;
  last_v4l2_timestamp_ = {0, 0};
  nsec_offset_ = 0;
  RegisterSigintHandlerIfDefault();
}

V4l2Camera::~V4l2Camera() {
  StopAcquire();
  Release();
}

int V4l2Camera::Init() {
  ReleaseDevice();
  do {
    GetOffset();

    struct stat st;
    if (stat(device_name_.c_str(), &st) < 0) {
      AWARN << "invalid video device: " << device_name_
            << ", errno: " << errno << ", " << strerror(errno);
      break;
    }
    if (!S_ISCHR(st.st_mode)) {
      AWARN << "invalid video node type (not char device): " << device_name_;
      break;
    }

    video_fd_ =
        open(device_name_.c_str(), O_RDWR | O_CLOEXEC /*| O_NONBLOCK*/, 0);
    if (-1 == video_fd_) {
      AWARN << "cannot open: " << device_name_ << ", errno: " << errno << ", "
            << strerror(errno);
      break;
    }
    if (!CheckCapabilities()) {
      AWARN << "CheckCapabilities failed, devname = " << device_name_;
      break;
    }
    if (!SetVideoFormat()) {
      AWARN << "SetVideoFormat failed, devname = " << device_name_;
      break;
    }
    is_stream_param_unsupported_ = false;
    if (!SetStreamFps()) {
      AWARN << "SetStreamFps failed, continue with driver default fps, devname = "
            << device_name_;
    }
    usleep(timing_options_.init_device_delay_ms * 1000U);

    if (!InitDevice()) {
      AINFO << "InitDevice failed, devname = " << device_name_;
      break;
    }

    return 0;

  } while (0);

  ReleaseDevice();

  return -1;
}

int V4l2Camera::Release() {
  ReleaseDevice();
  return 0;
}

int V4l2Camera::StartAcquire() {
  if (is_running_.load()) {
    return 0;
  }
  if (video_fd_ < 0 || nv_buffers_ == nullptr) {
    AWARN << "camera is not initialized, devname = " << device_name_;
    return -1;
  }
  g_sigint_received = 0;
  is_first_capture_ = true;
  is_running_.store(true);
  try {
    grab_thread_.reset(new std::thread(&V4l2Camera::GrabRoutine, this));
  } catch (const std::exception &e) {
    is_running_.store(false);
    AWARN << "failed to start capture thread, devname = " << device_name_
          << ", error: " << e.what();
    return -1;
  }
  return 0;
}

int V4l2Camera::StopAcquire() {
  is_running_.store(false);
  if (grab_thread_ != nullptr) {
    if (grab_thread_->joinable()) {
      grab_thread_->join();
    }
    grab_thread_.reset();
  }
  if (is_streaming_) {
    StopCapture();
  }
  is_first_capture_ = true;
  return 0;
}

void V4l2Camera::SetDataCallback(V4l2DataCallback callback, void *user_data) {
  user_data_ = user_data;
  data_callback_ = callback;
}

int V4l2Camera::ClearBuffer() { return ClearBufferInternal(); }

int V4l2Camera::QueueDmabuf(uint32_t index) {
  if (video_fd_ < 0 || nv_buffers_ == nullptr ||
      index >= static_cast<uint32_t>(buffer_count_)) {
    return -1;
  }
  if (nv_buffers_[index].dmabuff_fd < 0) {
    AWARN << "invalid dmabuf fd, index = " << index << ", dev = " << device_name_;
    return -1;
  }

  struct v4l2_buffer buf;
  memset(&buf, 0, sizeof(buf));
  buf.index = index;
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_DMABUF;
  buf.m.fd = (unsigned long)nv_buffers_[index].dmabuff_fd;
  if (Xioctl(video_fd_, VIDIOC_QBUF, &buf) < 0) {
    last_dq_errno_ = errno;
    AWARN << "VIDIOC_QBUF failed, dev = " << device_name_
          << ", index = " << index << ", errno: " << errno << ", "
          << strerror(errno);
    return -1;
  }
  return 0;
}

int V4l2Camera::WaitForBufferReady(int timeout_ms) {
  if (video_fd_ < 0) {
    return -1;
  }

  struct pollfd pfd;
  memset(&pfd, 0, sizeof(pfd));
  pfd.fd = video_fd_;
  pfd.events = POLLIN | POLLPRI | POLLERR;

  while (is_running_.load()) {
    int ret = poll(&pfd, 1, timeout_ms);
    if (ret > 0) {
      if (pfd.revents & POLLERR) {
        AWARN << "poll return POLLERR, dev = " << device_name_;
        last_dq_errno_ = EIO;
        return -1;
      }
      if (pfd.revents & (POLLIN | POLLPRI)) {
        return 1;
      }
      continue;
    }
    if (ret == 0) {
      last_dq_errno_ = ETIMEDOUT;
      return 0;
    }
    if (errno == EINTR) {
      continue;
    }
    AWARN << "poll failed, dev = " << device_name_ << ", errno: " << errno
          << ", error: " << strerror(errno);
    last_dq_errno_ = errno;
    return -1;
  }
  return -1;
}

int V4l2Camera::GrabImage() {
  unsigned char *src_data = nullptr;
  int src_data_len = 0;
#if CAMERA_LATENCY_PROBE_ENABLED
  struct timeval t_grab_begin;
  struct timeval t_wait_done;
  struct timeval t_dq_done;
  struct timeval t_sync_done;
  struct timeval t_callback_done;
  struct timeval t_qbuf_done;
  gettimeofday(&t_grab_begin, nullptr);
#endif  // CAMERA_LATENCY_PROBE_ENABLED
  int timeout_ms = static_cast<int>(timing_options_.poll_min_timeout_ms);
  if (fps_ > 0) {
    timeout_ms = 3000 / fps_;
    if (timeout_ms < static_cast<int>(timing_options_.poll_min_timeout_ms)) {
      timeout_ms = static_cast<int>(timing_options_.poll_min_timeout_ms);
    }
    if (timeout_ms > static_cast<int>(timing_options_.poll_max_timeout_ms)) {
      timeout_ms = static_cast<int>(timing_options_.poll_max_timeout_ms);
    }
  }

  const int ready = WaitForBufferReady(timeout_ms);
#if CAMERA_LATENCY_PROBE_ENABLED
  gettimeofday(&t_wait_done, nullptr);
#endif  // CAMERA_LATENCY_PROBE_ENABLED
  if (ready <= 0) {
    if (ready == 0) {
      AWARN << "poll timeout, dev = " << device_name_;
    }
    return -1;
  }

  struct v4l2_buffer buf;
  memset(&buf, 0, sizeof(struct v4l2_buffer));

  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_DMABUF;
  if (Xioctl(video_fd_, VIDIOC_DQBUF, &buf) < 0) {
    last_dq_errno_ = errno;
    if (errno == EAGAIN) {
      return -1;
    }
    AWARN << "Xioctl VIDIOC_DQBUF failed, dev = " << device_name_
          << ", errno: " << errno << ", error: " << strerror(errno);
    return -1;
  }
#if CAMERA_LATENCY_PROBE_ENABLED
  gettimeofday(&t_dq_done, nullptr);
#endif  // CAMERA_LATENCY_PROBE_ENABLED
  last_dq_errno_ = 0;

  if (buf.index >= static_cast<uint32_t>(buffer_count_)) {
    last_dq_errno_ = EINVAL;
    AWARN << "invalid buffer index: " << buf.index << ", dev = " << device_name_;
    return -1;
  }
  if (buf.flags & V4L2_BUF_FLAG_ERROR) {
    AWARN << "driver reports frame error, chan = " << channel_
          << ", sequence = " << buf.sequence;
  }
  if (last_sequence_ != 0 && buf.sequence > last_sequence_ + 1) {
    AWARN << "frame dropped on chan " << channel_ << ", last sequence = "
          << last_sequence_ << ", current sequence = " << buf.sequence;
  }
  last_sequence_ = buf.sequence;

  int ret = -1;
#if CAMERA_LATENCY_PROBE_ENABLED
  struct timespec ft;
  bool has_frame_time = false;
#endif  // CAMERA_LATENCY_PROBE_ENABLED
  do {
    if (NvBufSurfaceFromFd(nv_buffers_[buf.index].dmabuff_fd,
                           (void **)(&surface_)) < 0) {
      AWARN << "Cannot get NvBufSurface from fd.";
      break;
    }
    if (NvBufSurfaceSyncForCpu(surface_, 0, 0) < 0) {
      AWARN << "Cannot sync capture buffer for cpu.";
      break;
    }
#if CAMERA_LATENCY_PROBE_ENABLED
    gettimeofday(&t_sync_done, nullptr);
#endif  // CAMERA_LATENCY_PROBE_ENABLED
    src_data = (unsigned char *)surface_->surfaceList[0].mappedAddr.addr[0];
    if (src_data == nullptr) {
      AWARN << "capture buffer addr is null, dev = " << device_name_;
      break;
    }

    src_data_len = surface_->surfaceList[0].dataSize;
    if (buf.bytesused > 0 && buf.bytesused <= surface_->surfaceList[0].dataSize) {
      src_data_len = buf.bytesused;
    }

    struct timeval tv;
    gettimeofday(&tv, nullptr);
    last_v4l2_timestamp_.tv_sec = buf.timestamp.tv_sec;
    last_v4l2_timestamp_.tv_usec = buf.timestamp.tv_usec;

    if ((buf.timestamp.tv_sec == 0 && buf.timestamp.tv_usec == 0) ||
        skipped_frame_count_ < 0) {
      AWARN << "[chan:" << channel_ << "]systime:" << tv.tv_sec << "."
            << tv.tv_usec << ", buftime:" << buf.timestamp.tv_sec << "."
            << buf.timestamp.tv_usec;
      break;
    }

#if CAMERA_LATENCY_PROBE_ENABLED
    RtcpuToRealtime(buf.timestamp, &ft);
    has_frame_time = true;
#else
    struct timespec ft;
    RtcpuToRealtime(buf.timestamp, &ft);
#endif  // CAMERA_LATENCY_PROBE_ENABLED
    if (data_callback_) {
      data_callback_(channel_, ft, width_, height_, src_data, src_data_len, user_data_);
    }
#if CAMERA_LATENCY_PROBE_ENABLED
    gettimeofday(&t_callback_done, nullptr);
#endif  // CAMERA_LATENCY_PROBE_ENABLED
    ret = 0;
  } while (0);

  if (ret != 0 && last_dq_errno_ == 0) {
    last_dq_errno_ = EIO;
  }

  if (QueueDmabuf(buf.index) < 0) {
    return -1;
  }
#if CAMERA_LATENCY_PROBE_ENABLED
  gettimeofday(&t_qbuf_done, nullptr);

  if (ret == 0 && has_frame_time) {
    const double wait_ms = TimevalDiffMs(t_wait_done, t_grab_begin);
    const double dqbuf_ms = TimevalDiffMs(t_dq_done, t_wait_done);
    const double sync_ms = TimevalDiffMs(t_sync_done, t_dq_done);
    const double callback_ms = TimevalDiffMs(t_callback_done, t_sync_done);
    const double qbuf_ms = TimevalDiffMs(t_qbuf_done, t_callback_done);
    const double total_ms = TimevalDiffMs(t_qbuf_done, t_grab_begin);
    const double frame_age_after_callback_ms =
        (static_cast<double>(t_callback_done.tv_sec) +
         static_cast<double>(t_callback_done.tv_usec) / 1000000.0 -
         TimespecToSec(ft)) *
        1000.0;
    if (frame_age_after_callback_ms >= kLatencyProbeWarnMs ||
        dqbuf_ms >= kLatencyProbeStageWarnMs ||
        sync_ms >= kLatencyProbeStageWarnMs ||
        callback_ms >= kLatencyProbeStageWarnMs ||
        qbuf_ms >= kLatencyProbeStageWarnMs) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(9)
          << "[camera_latency_probe][v4l2]"
          << " chan=" << channel_
          << ", seq=" << buf.sequence
          << ", frame_time_sec=" << TimespecToSec(ft)
          << std::setprecision(3)
          << ", wait_ms=" << wait_ms
          << ", dqbuf_ms=" << dqbuf_ms
          << ", sync_ms=" << sync_ms
          << ", callback_ms=" << callback_ms
          << ", qbuf_ms=" << qbuf_ms
          << ", total_ms=" << total_ms
          << ", frame_age_after_callback_ms="
          << frame_age_after_callback_ms
          << ", buffer_index=" << buf.index
          << ", bytesused=" << buf.bytesused;
      AWARN << oss.str();
    }
  }
#endif  // CAMERA_LATENCY_PROBE_ENABLED

  if (ret == 0) {
    frame_count_++;
  }
  return ret;
}

void V4l2Camera::RecoverStream() {
  if (video_fd_ < 0 || nv_buffers_ == nullptr) {
    return;
  }
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  // 1. stop stream
  Xioctl(video_fd_, VIDIOC_STREAMOFF, &type);

  // 2. request buffers
  struct v4l2_requestbuffers rb;
  memset(&rb, 0, sizeof(rb));
  rb.count = static_cast<uint32_t>(buffer_count_);
  rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rb.memory = V4L2_MEMORY_DMABUF;
  if (Xioctl(video_fd_, VIDIOC_REQBUFS, &rb) < 0) {
    AINFO << "RecoverStream: REQBUFS failed: " << strerror(errno);
    return;
  }

  // 3. Query + QBUF
  for (unsigned i = 0; i < rb.count; ++i) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.index = i;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_DMABUF;
    buf.m.fd = nv_buffers_[i].dmabuff_fd;

    if (Xioctl(video_fd_, VIDIOC_QUERYBUF, &buf) < 0) {
      AINFO << "RecoverStream: QUERYBUF idx=" << i
            << " failed: " << strerror(errno);
    }
    if (QueueDmabuf(i) < 0) {
      AINFO << "RecoverStream: QBUF idx=" << i << " failed";
    }
  }

  // 4. start stream
  if (Xioctl(video_fd_, VIDIOC_STREAMON, &type) < 0) {
    AINFO << "RecoverStream: STREAMON failed: " << strerror(errno);
  } else {
    is_streaming_ = true;
    last_sequence_ = 0;
    last_dq_errno_ = 0;
  }
}

int V4l2Camera::StartCapture() {
  if (video_fd_ < 0) {
    AINFO << "video fd is invlaid, devname = " << device_name_;
    return -1;
  }
  if (nv_buffers_ == nullptr) {
    AINFO << "capture buffers are not initialized, devname = " << device_name_;
    return -1;
  }
  if (is_streaming_) {
    return 0;
  }

  enum v4l2_buf_type type;

  for (int index = 0; index < buffer_count_; ++index) {
    if (QueueDmabuf(index) < 0) {
      AINFO << "QueueDmabuf failed at StartCapture, index = " << index
            << ", devname = " << device_name_;
      return -1;
    }
  }

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (Xioctl(video_fd_, VIDIOC_STREAMON, &type) < 0) {
    AINFO << "Xioctl VIDIOC_STREAMON, devname = " << device_name_.c_str();
    return -1;
  }
  is_streaming_ = true;
  consecutive_grab_fail_count_ = 0;
  last_sequence_ = 0;
  last_dq_errno_ = 0;
  ClearBufferInternal();
  return 0;
}

int V4l2Camera::StopCapture() {
  if (!is_streaming_ || video_fd_ < 0) {
    is_streaming_ = false;
    return 0;
  }
  enum v4l2_buf_type type;
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (Xioctl(video_fd_, VIDIOC_STREAMOFF, &type) < 0) {
    if (errno == EINVAL) {
      is_streaming_ = false;
      last_sequence_ = 0;
      return 0;
    }
    AINFO << "Xioctl VIDIOC_STREAMOFF, devname = " << device_name_.c_str();
    return -1;
  }
  is_streaming_ = false;
  last_sequence_ = 0;
  consecutive_grab_fail_count_ = 0;
  last_dq_errno_ = 0;
  return 0;
}

bool V4l2Camera::ClearBufferInternal() {
  if (!is_streaming_ || video_fd_ < 0) {
    return true;
  }
  struct v4l2_buffer buf;

  for (int i = 0; i < buffer_count_; ++i) {
    memset(&buf, 0, sizeof(struct v4l2_buffer));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_DMABUF;

    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(video_fd_, &rset);
    struct timeval tv;
    tv.tv_sec = timing_options_.clear_buffer_timeout_ms / 1000U;
    tv.tv_usec = (timing_options_.clear_buffer_timeout_ms % 1000U) * 1000U;
    int retsel = select(video_fd_ + 1, &rset, nullptr, nullptr, &tv);
    if (retsel <= 0) {
      if (retsel < 0 && errno == EINTR) {
        continue;
      }
      break;
    }
    if (Xioctl(video_fd_, VIDIOC_DQBUF, &buf) < 0) {
      last_dq_errno_ = errno;
      if (errno == EAGAIN) {
        continue;
      }
      AWARN << "Xioctl VIDIOC_DQBUF failed, dev = " << device_name_
            << ", errno: " << errno << ", error: " << strerror(errno);
      break;
    }
    last_dq_errno_ = 0;

    if (buf.index >= static_cast<uint32_t>(buffer_count_) ||
        nv_buffers_[buf.index].dmabuff_fd < 0) {
      AWARN << "invalid buffer for ClearBufferInternal, index = " << buf.index
            << ", dev = " << device_name_;
      break;
    }
    if (QueueDmabuf(buf.index) < 0) {
      break;
    }
  }
  return true;
}

bool V4l2Camera::InitDevice() { return InitDma(); }

void V4l2Camera::GrabRoutine() {
  pthread_t native_handle = pthread_self();
  if (thread_options_.enable_realtime_scheduling) {
    struct sched_param param;
    const int prio_max = sched_get_priority_max(SCHED_RR);
    const int prio_min = sched_get_priority_min(SCHED_RR);
    if (prio_max >= 0 && prio_min >= 0) {
      param.sched_priority = std::max(
          prio_min, std::min(thread_options_.realtime_priority, prio_max));
    } else {
      param.sched_priority = 0;
    }
    int sched_ret = pthread_setschedparam(native_handle, SCHED_RR, &param);
    if (sched_ret != 0) {
      if (sched_ret == EPERM || sched_ret == EACCES) {
        AINFO << "realtime scheduling is not permitted, chan = " << channel_
              << ", continue with default scheduler. Hint: grant CAP_SYS_NICE or "
                 "run with elevated privileges if RT scheduling is required.";
      } else {
        AWARN << "pthread_setschedparam failed, chan = " << channel_
              << ", err = " << sched_ret << ", " << strerror(sched_ret);
      }
    }
  } else {
    AINFO << "realtime scheduling disabled, chan = " << channel_;
  }

  if (thread_options_.enable_cpu_affinity) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    const uint32_t core_id = century::drivers::camera::ResolveCpuAffinityCore(
        thread_options_, static_cast<uint32_t>(channel_));
    CPU_SET(static_cast<int>(core_id), &cpuset);
    int affinity_ret =
        pthread_setaffinity_np(native_handle, sizeof(cpu_set_t), &cpuset);
    if (affinity_ret != 0) {
      AWARN << "pthread_setaffinity_np failed, chan = " << channel_
            << ", core = " << core_id << ", err = " << affinity_ret;
    } else {
      AINFO << "Camera thread bound to CPU core " << core_id
            << ", chan = " << channel_;
    }
  } else {
    AINFO << "CPU affinity disabled, chan = " << channel_;
  }

  while (is_running_.load()) {
    if (g_sigint_received) {
      is_running_.store(false);
      if (is_streaming_) {
        StopCapture();
      }
      break;
    }
    if (is_first_capture_) {
      is_first_capture_ = false;
      usleep(timing_options_.first_capture_delay_ms * 1000U);
      if (StartCapture() < 0) {
        usleep(timing_options_.start_capture_retry_delay_ms * 1000U);
        continue;
      }
      while (timestamp_queue_.size() > 0) {
        timestamp_queue_.pop_front();
      }
    }
    if (!is_running_.load()) {
      break;
    }
    if (GrabImage() == -1) {
      if (last_dq_errno_ != EAGAIN) {
        ++consecutive_grab_fail_count_;
      }
      if (is_streaming_ && consecutive_grab_fail_count_ >= kRecoverThreshold) {
        AWARN << "grab failed continuously on chan " << channel_
              << ", try recovering stream, last errno = " << last_dq_errno_;
        RecoverStream();
        consecutive_grab_fail_count_ = 0;
      }
      usleep(timing_options_.grab_failure_retry_delay_ms * 1000U);
      continue;
    }
    consecutive_grab_fail_count_ = 0;
  }
}

bool V4l2Camera::ReleaseDevice() {
  if (video_fd_ != -1 && is_streaming_) {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    Xioctl(video_fd_, VIDIOC_STREAMOFF, &type);
  }
  is_streaming_ = false;
  consecutive_grab_fail_count_ = 0;
  last_sequence_ = 0;
  last_dq_errno_ = 0;

  if (video_fd_ != -1) {
    struct v4l2_requestbuffers rb;
    memset(&rb, 0, sizeof(rb));
    rb.count = 0;
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rb.memory = V4L2_MEMORY_DMABUF;
    Xioctl(video_fd_, VIDIOC_REQBUFS, &rb);
  }

  if (video_fd_ != -1) {
    close(video_fd_);
    video_fd_ = -1;
  }

  if (nv_buffers_) {
    for (int i = 0; i < buffer_count_; ++i) {
      int dmabuf_fd = nv_buffers_[i].dmabuff_fd;
      if (dmabuf_fd >= 0) {
        NvBufSurface *surface_ptr = nullptr;
        if (NvBufSurfaceFromFd(dmabuf_fd, (void **)&surface_ptr) == 0 && surface_ptr) {
          NvBufSurfaceUnMap(surface_ptr, 0, 0);
        }
        NvBufSurf::NvDestroy(dmabuf_fd);
        nv_buffers_[i].dmabuff_fd = -1;
      }
    }
    free(nv_buffers_);
    nv_buffers_ = nullptr;
  }

  return true;
}

bool V4l2Camera::InitDma() {
  NvBufSurf::NvCommonAllocateParams allocate_params = {0};
  std::vector<int> fd(buffer_count_, -1);

  auto cleanup = [&]() {
    for (int i = 0; i < buffer_count_; ++i) {
      if (nv_buffers_ && nv_buffers_[i].dmabuff_fd >= 0) {
        NvBufSurface *surface_ptr = nullptr;
        if (NvBufSurfaceFromFd(nv_buffers_[i].dmabuff_fd, (void **)&surface_ptr) == 0 &&
            surface_ptr) {
          NvBufSurfaceUnMap(surface_ptr, 0, 0);
        }
        NvBufSurf::NvDestroy(nv_buffers_[i].dmabuff_fd);
        nv_buffers_[i].dmabuff_fd = -1;
      }
      if (fd[i] >= 0) {
        NvBufSurf::NvDestroy(fd[i]);
        fd[i] = -1;
      }
    }
    if (nv_buffers_) {
      free(nv_buffers_);
      nv_buffers_ = nullptr;
    }
  };

  /* Allocate global buffer context */
  nv_buffers_ =
      static_cast<NvBuffer *>(calloc(static_cast<size_t>(buffer_count_),
                                     sizeof(NvBuffer)));
  if (nv_buffers_ == nullptr) {
    AINFO << "Failed to allocate global buffer context";
    return false;
  }
  for (int i = 0; i < buffer_count_; ++i) {
    nv_buffers_[i].dmabuff_fd = -1;
  }

  allocate_params.memType = NVBUF_MEM_SURFACE_ARRAY;
  allocate_params.width = width_;
  allocate_params.height = height_;
  allocate_params.layout = NVBUF_LAYOUT_PITCH;
  allocate_params.colorFormat = GetNvbufColorFormat(v4l2_pixel_format_);
  if (allocate_params.colorFormat == NVBUF_COLOR_FORMAT_INVALID) {
    AWARN << "unsupported pix format(" << v4l2_pixel_format_
          << ") for NvBufSurface, fallback to YUYV";
    allocate_params.colorFormat = GetNvbufColorFormat(V4L2_PIX_FMT_YUYV);
  }
  allocate_params.memtag = NvBufSurfaceTag_CAMERA;
  if (NvBufSurf::NvAllocate(&allocate_params, buffer_count_, fd.data())) {
    AWARN << "Failed to create NvBuffer.";
    cleanup();
    return false;
  }

  /* Create buffer and provide it with camera */
  for (int index = 0; index < buffer_count_; ++index) {
    NvBufSurface *surface_ptr = nullptr;

    nv_buffers_[index].dmabuff_fd = fd[index];
    fd[index] = -1;

    if (-1 == NvBufSurfaceFromFd(nv_buffers_[index].dmabuff_fd, (void **)(&surface_ptr))) {
      AWARN << "Failed to get NvBuffer parameters.";
      cleanup();
      return false;
    }

    /* TODO: add multi-planar support
       Currently only supports YUV422 interlaced single-planar */

    if (-1 == NvBufSurfaceMap(surface_ptr, 0, 0, NVBUF_MAP_READ_WRITE)) {
      AWARN << "Failed to map buffer.";
      cleanup();
      return false;
    }
    nv_buffers_[index].start =
        (unsigned char *)surface_ptr->surfaceList[0].mappedAddr.addr[0];
    nv_buffers_[index].size = surface_ptr->surfaceList[0].dataSize;
  }

  /* Request camera v4l2 buffer */
  struct v4l2_requestbuffers rb;
  memset(&rb, 0, sizeof(rb));
  rb.count = static_cast<uint32_t>(buffer_count_);
  rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rb.memory = V4L2_MEMORY_DMABUF;
  if (Xioctl(video_fd_, VIDIOC_REQBUFS, &rb) < 0) {
    AWARN << "Failed to request v4l2 buffers: " << strerror(errno) << ", errno"
          << errno;
    cleanup();
    return false;
  }
  if (rb.count != static_cast<uint32_t>(buffer_count_)) {
    AWARN << "V4l2 buffer number is not as desired.";
    cleanup();
    return false;
  }

  for (int index = 0; index < buffer_count_; ++index) {
    struct v4l2_buffer buf;

    /* Query camera v4l2 buf length */
    memset(&buf, 0, sizeof buf);
    buf.index = index;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_DMABUF;

    if (Xioctl(video_fd_, VIDIOC_QUERYBUF, &buf) < 0) {
      AWARN << "Failed to query buff: " << strerror(errno)
            << ", errno: " << errno;
      cleanup();
      return false;
    }
    /* TODO: add support for multi-planer
       Enqueue empty v4l2 buff into camera capture plane */
    buf.m.fd = (unsigned long)nv_buffers_[index].dmabuff_fd;
    if (buf.length != nv_buffers_[index].size) {
      AWARN << "Camera v4l2 buf length is not expected.";
      nv_buffers_[index].size = buf.length;
    }
  }
  return true;
}

bool V4l2Camera::SetVideoFormat() {
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));

  v4l2_pixel_format_ = ResolveV4l2PixFmt(config_format_);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = width_;
  fmt.fmt.pix.height = height_;
  fmt.fmt.pix.pixelformat = v4l2_pixel_format_;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

  if (-1 == Xioctl(video_fd_, VIDIOC_S_FMT, &fmt)) {
    AWARN << "Xioctl VIDIOC_S_FMT failed, devname=" << device_name_
          << ", errno: " << errno << ", " << strerror(errno);
    return false;
  }

  memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == Xioctl(video_fd_, VIDIOC_G_FMT, &fmt)) {
    AWARN << "Xioctl VIDIOC_G_FMT failed, devname=" << device_name_
          << ", errno: " << errno << ", " << strerror(errno);
    return false;
  }

  if (fmt.fmt.pix.width != width_ || fmt.fmt.pix.height != height_ ||
      fmt.fmt.pix.pixelformat != v4l2_pixel_format_) {
    AWARN << "requested format not fully supported by driver. req("
          << width_ << "x" << height_ << ", " << v4l2_pixel_format_
          << "), actual(" << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height
          << ", " << fmt.fmt.pix.pixelformat << ")";
  }

  width_ = fmt.fmt.pix.width;
  height_ = fmt.fmt.pix.height;
  v4l2_pixel_format_ = fmt.fmt.pix.pixelformat;
  return true;
}

bool V4l2Camera::SetStreamFps() {
  if (video_fd_ < 0 || fps_ <= 0) {
    return true;
  }
  if (is_stream_param_unsupported_) {
    return true;
  }

  struct v4l2_streamparm streamparm;
  memset(&streamparm, 0, sizeof(streamparm));
  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (Xioctl(video_fd_, VIDIOC_G_PARM, &streamparm) < 0) {
    const int saved_errno = errno;
    if (IsStreamParmUnsupportedErrno(saved_errno)) {
      is_stream_param_unsupported_ = true;
      AINFO << "stream fps control is not supported by device, use driver "
               "default fps, devname = "
            << device_name_;
      return true;
    }
    AWARN << "VIDIOC_G_PARM failed, devname = " << device_name_
          << ", errno: " << saved_errno << ", " << strerror(saved_errno);
    return false;
  }

  if (!(streamparm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)) {
    is_stream_param_unsupported_ = true;
    AINFO << "driver does not support V4L2_CAP_TIMEPERFRAME, use default fps, "
          << "devname = " << device_name_;
    return true;
  }

  streamparm.parm.capture.timeperframe.numerator = 1;
  streamparm.parm.capture.timeperframe.denominator =
      static_cast<uint32_t>(fps_);
  if (Xioctl(video_fd_, VIDIOC_S_PARM, &streamparm) < 0) {
    const int saved_errno = errno;
    if (IsStreamParmUnsupportedErrno(saved_errno)) {
      is_stream_param_unsupported_ = true;
      AINFO << "stream fps set is not supported by device, use driver default "
               "fps, devname = "
            << device_name_;
      return true;
    }
    AWARN << "VIDIOC_S_PARM failed, devname = " << device_name_
          << ", errno: " << saved_errno << ", " << strerror(saved_errno);
    return false;
  }

  memset(&streamparm, 0, sizeof(streamparm));
  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (Xioctl(video_fd_, VIDIOC_G_PARM, &streamparm) == 0 &&
      streamparm.parm.capture.timeperframe.numerator > 0) {
    const auto num = streamparm.parm.capture.timeperframe.numerator;
    const auto den = streamparm.parm.capture.timeperframe.denominator;
    if (num > 0 && den >= num) {
      const int actual_fps = den / num;
      if (actual_fps > 0 && actual_fps != fps_) {
        AWARN << "requested fps(" << fps_ << ") differs from actual fps("
              << actual_fps << "), devname = " << device_name_;
        fps_ = actual_fps;
      }
    }
  }
  return true;
}

bool V4l2Camera::CheckCapabilities() {
  struct v4l2_capability cap;
  if (-1 == Xioctl(video_fd_, VIDIOC_QUERYCAP, &cap)) {
    AWARN << "Xioctl VIDIOC_QUERYCAP failed, devname=" << device_name_
          << ", errno: " << errno << ", " << strerror(errno);
    return false;
  }

  uint32_t capabilities = cap.capabilities;
  if (cap.capabilities & V4L2_CAP_DEVICE_CAPS) {
    capabilities = cap.device_caps;
  }

  if (!(capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    if (capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
      AWARN << "device only supports multi-planar capture, current driver path "
               "supports single-planar only";
    }
    AWARN << "capabilities not support V4L2_CAP_VIDEO_CAPTURE";
    return false;
  }

  if (!(capabilities & V4L2_CAP_STREAMING)) {
    AWARN << "capabilities not support V4L2_CAP_STREAMING";
    return false;
  }
  return true;
}

int V4l2Camera::Xioctl(int fd, unsigned long request, void *arg) {
  int r;
  do {
    r = ioctl(fd, request, arg);
  } while (-1 == r && EINTR == errno);

  return r;
}

long ReadOffsetNs() {
  unsigned long raw_nsec, tsc_ns;
  unsigned long cycles, frq;
  struct timespec tp;

  asm volatile("mrs %0, cntfrq_el0" : "=r"(frq));
  asm volatile("mrs %0, cntvct_el0" : "=r"(cycles));

  clock_gettime(CLOCK_MONOTONIC_RAW, &tp);

  tsc_ns = (cycles * 100 / (frq / 10000)) * 1000;
  raw_nsec = tp.tv_sec * 1000000000 + tp.tv_nsec;
  long offset_ns = llabs(tsc_ns - raw_nsec);

  return offset_ns;
}

int V4l2Camera::GetOffset() {
  char tmp_str[128] = {0};
  FILE *offset_file =
      fopen("/sys/devices/system/clocksource/clocksource0/offset_ns", "r");
  if (offset_file) {
    fgets(tmp_str, sizeof(tmp_str), offset_file);
    nsec_offset_ = atol(tmp_str);
    fclose(offset_file);
  } else {
    nsec_offset_ = ReadOffsetNs();
    if (nsec_offset_ < 0) {
      AWARN << "read offset failed, nsec_offset_ = " << nsec_offset_;
      return -1;
    }
  }
  return 0;
}

void V4l2Camera::RtcpuToRealtime(timeval rtcpu_time, timespec *real_time) {
  struct timespec real_sample, monotonic_sample, monotonic_time, time_diff;
  const int64_t kNsecPerSec = 1000000000;

  // printf("[chan:%d]rtcpu_time: %ld.%09ld\n", channel_, rtcpu_time.tv_sec,
  // rtcpu_time.tv_usec*1000);
  long long monotonic_ns =
      rtcpu_time.tv_sec * kNsecPerSec + rtcpu_time.tv_usec * 1000 - nsec_offset_;
  monotonic_time.tv_sec = monotonic_ns / kNsecPerSec;
  monotonic_time.tv_nsec = monotonic_ns % kNsecPerSec;
  // printf("[chan:%d] monotonic_time: %ld.%09ld\n",
  // channel_,monotonic_time.tv_sec, monotonic_time.tv_nsec);

  clock_gettime(CLOCK_MONOTONIC_RAW, &monotonic_sample);
  clock_gettime(CLOCK_REALTIME, &real_sample);

  time_diff.tv_sec = real_sample.tv_sec - monotonic_sample.tv_sec;
  time_diff.tv_nsec = real_sample.tv_nsec - monotonic_sample.tv_nsec;

  real_time->tv_sec = monotonic_time.tv_sec + time_diff.tv_sec;
  real_time->tv_nsec = monotonic_time.tv_nsec + time_diff.tv_nsec;
  if (real_time->tv_nsec >= kNsecPerSec) {
    ++real_time->tv_sec;
    real_time->tv_nsec -= kNsecPerSec;
  } else if (real_time->tv_nsec < 0) {
    --real_time->tv_sec;
    real_time->tv_nsec += kNsecPerSec;
  }
  return;
}
