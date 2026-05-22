/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
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

#include <assert.h>
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "libyuv.h"
#include "vin_log.h"

static nv_color_fmt nvcolor_fmt[] = {
    /* TODO: add more pixel format mapping */
    {V4L2_PIX_FMT_UYVY, NVBUF_COLOR_FORMAT_UYVY},
    {V4L2_PIX_FMT_VYUY, NVBUF_COLOR_FORMAT_VYUY},
    {V4L2_PIX_FMT_YUYV, NVBUF_COLOR_FORMAT_YUYV},
    {V4L2_PIX_FMT_YVYU, NVBUF_COLOR_FORMAT_YVYU},
    {V4L2_PIX_FMT_GREY, NVBUF_COLOR_FORMAT_GRAY8},
    {V4L2_PIX_FMT_YUV420M, NVBUF_COLOR_FORMAT_YUV420},
};

static NvBufSurfaceColorFormat get_nvbuff_color_fmt(unsigned int v4l2_pixfmt) {
  unsigned i;

  for (i = 0; i < sizeof(nvcolor_fmt) / sizeof(nvcolor_fmt[0]); i++) {
    if (v4l2_pixfmt == nvcolor_fmt[i].v4l2_pixfmt)
      return nvcolor_fmt[i].nvbuff_color;
  }

  return NVBUF_COLOR_FORMAT_INVALID;
}

#define V4L2_BUFFER_LENGHT 2
#define V4L2_VIDEO_FORMAT V4L2_PIX_FMT_YUYV
#define difftimeval(end, beginning) \
  ((end.tv_sec - beginning.tv_sec) * 1000000 + end.tv_usec - beginning.tv_usec)

static CV4l2Cam *g_cam_instance = nullptr;
static void handle_sigint(int signum) {
  if (signum != SIGINT) return;
  if (g_cam_instance) {
    g_cam_instance->StopAcquire();
    g_cam_instance->Release();
  }

  _exit(0);
}

CV4l2Cam::CV4l2Cam(int dwPipeId, int dwVideoIndex, int dwWidth, int dwHeight,
                   int dwFps, int format) {
  m_videoFd = -1;
  m_nChan = dwPipeId;
  m_dwWidth = dwWidth;
  m_dwHeight = dwHeight;
  m_nFps = dwFps;
  m_dwFormat = format;
  char tmp[64] = {0};
  (void)snprintf(tmp, sizeof(tmp), "/dev/video%d", dwVideoIndex);
  m_strDevName = tmp;
  m_bRunning = false;
  m_nFirstCapture = true;
  m_dwFrameCnt = 0;
  m_llSkippedFrameNum = 0;
  m_lNsecOffset = 0;
  if (!g_cam_instance) {
    g_cam_instance = this;
    ::signal(SIGINT, handle_sigint);
  }
}

CV4l2Cam::~CV4l2Cam() {
  StopAcquire();
  Release();
  if (g_cam_instance == this) {
    g_cam_instance = nullptr;
  }
}

int CV4l2Cam::Init() {
  do {
    GetOffset();

    m_videoFd =
        open(m_strDevName.c_str(), O_RDWR /* required */ /*| O_NONBLOCK*/, 0);
    if (-1 == m_videoFd) {
      AINFO << "cannot open: " << m_strDevName << ", errno: " << errno << ", "
            << strerror(errno);
      break;
    }
    if (!checkCapabilities()) {
      AINFO << "checkCapabilities failed, devname = " << m_strDevName;
      break;
    }
    if (!setVideoFmt()) {
      AINFO << "setVideoFmt failed, devname = " << m_strDevName;
      break;
    }
    usleep(1000 * 1000);

    if (!initDevice()) {
      AINFO << "initDevice failed, devname = " << m_strDevName;
      break;
    }

    return 0;

  } while (0);

  if (m_videoFd != -1) {
    close(m_videoFd);
    m_videoFd = -1;
  }

  return -1;
}

int CV4l2Cam::Release() {
  releaseDevice();
  return 0;
}

int CV4l2Cam::StartAcquire() {
  m_bRunning = true;
  m_pGrabThread.reset(new std::thread(&CV4l2Cam::GrabRoutine, this));
  return 0;
}

int CV4l2Cam::StopAcquire() {
  m_bRunning = false;
  if (m_pGrabThread != nullptr) {
    if (m_pGrabThread->joinable()) {
      m_pGrabThread->join();
    }
  }
  stopCapture();
  return 0;
}

void CV4l2Cam::SetDataCallback(PV4L2_DATACALLBACK cb, void *pUserData) {
  m_pUserData = pUserData;
  m_Cb = cb;
}

int CV4l2Cam::ClearBuffer() { return clearBuffer(); }

int CV4l2Cam::grabImg() {
  unsigned char *srcData = NULL;
  int srcDatalen = 0;
  int retsel = 0;

  struct timeval t1, t2;
  gettimeofday(&t1, nullptr);

  fd_set rset;
  fd_set eset;
  for (int i = 0; i < 10; i++) {
    FD_ZERO(&rset);
    FD_ZERO(&eset);
    FD_SET(m_videoFd, &rset);
    FD_SET(m_videoFd, &eset);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 20 * 1000;
    retsel = select(m_videoFd + 1, &rset, NULL, &eset, &tv);
    if (retsel > 0) {
      break;
    }
  }

  if (FD_ISSET(m_videoFd, &rset) == false) {
    if (FD_ISSET(m_videoFd, &eset)) {
      AINFO << "select failed, dev = " << m_strDevName << ", errno: " << errno
            << ", error: " << strerror(errno);
    } else {
      AINFO << "select timeout, dev = " << m_strDevName;
    }

    if (m_Cb) {
    }
    return -1;
  }

  gettimeofday(&t2, nullptr);

  int nSelDiff =
      (t2.tv_sec - t1.tv_sec) * 1000 + (t2.tv_usec - t1.tv_usec) / 1000;
  if (nSelDiff >= 1.5 * 1000 / m_nFps) {
    AINFO << "chan " << m_nChan << ", select cost " << nSelDiff << "ms.";
  }

  struct v4l2_buffer buf;
  memset(&buf, 0, sizeof(struct v4l2_buffer));

  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_DMABUF;
  if (xioctl(m_videoFd, VIDIOC_DQBUF, &buf) < 0) {
    AINFO << "xioctl VIDIOC_DQBUF failed, dev = " << m_strDevName
          << ", errno: " << errno << ", error: " << strerror(errno);
    return -1;
  }
  if (NvBufSurfaceFromFd(m_pNvbuff[buf.index].dmabuff_fd, (void **)(&m_pSurf)) <
      0) {
    AINFO << "Cannot get NvBufSurface from fd.";
    return -1;
  }
  if (NvBufSurfaceSyncForDevice(m_pSurf, 0, 0) < 0) {
    AINFO << "Cannot sync output buffer.";
    return -1;
  }
  srcData = (unsigned char *)m_pSurf->surfaceList[0].mappedAddr.addr[0];
  srcDatalen = m_pSurf->surfaceList[0].dataSize;

  struct timeval tv;
  gettimeofday(&tv, nullptr);

  m_lstV4l2Timestamp.tv_sec = buf.timestamp.tv_sec;
  m_lstV4l2Timestamp.tv_usec = buf.timestamp.tv_usec;

  if ((buf.timestamp.tv_sec == 0 && buf.timestamp.tv_usec == 0) ||
      m_llSkippedFrameNum < 0) {
    debug_err("[chan:%d]systime:%ld.%06ld, buftime:%ld.%06ld\n", m_nChan,
              tv.tv_sec, tv.tv_usec, buf.timestamp.tv_sec,
              buf.timestamp.tv_usec);
    return -1;
  } else {
    struct timespec ft;
    rtcpuToRealtime(buf.timestamp, &ft);
    if (m_Cb) {
      m_Cb(m_nChan, ft, m_dwWidth, m_dwHeight, srcData, srcDatalen,
           m_pUserData);
    }
  }
  if (xioctl(m_videoFd, VIDIOC_QBUF, &buf) < 0) {
    AINFO << "xioctl VIDIOC_QBUF failed, dev = " << m_strDevName
          << ", errno: " << errno << ", error: " << strerror(errno);
    return -1;
  }

  m_dwFrameCnt++;
  return 0;
}

void CV4l2Cam::recoverStream() {
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  // 1. stop stream
  xioctl(m_videoFd, VIDIOC_STREAMOFF, &type);

  // 2. request buffers
  struct v4l2_requestbuffers rb;
  memset(&rb, 0, sizeof(rb));
  rb.count = V4L2_BUFFER_LENGHT;
  rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rb.memory = V4L2_MEMORY_DMABUF;
  if (ioctl(m_videoFd, VIDIOC_REQBUFS, &rb) < 0) {
    debug_err("recoverStream: REQBUFS failed: %s\n", strerror(errno));
    return;
  }

  // 3. Query + QBUF
  for (unsigned i = 0; i < rb.count; ++i) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.index = i;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_DMABUF;
    buf.m.fd = m_pNvbuff[i].dmabuff_fd;

    if (ioctl(m_videoFd, VIDIOC_QUERYBUF, &buf) < 0) {
      debug_err("recoverStream: QUERYBUF idx=%d failed: %s\n", i,
                strerror(errno));
    }
    if (ioctl(m_videoFd, VIDIOC_QBUF, &buf) < 0) {
      debug_err("recoverStream: QBUF idx=%d failed: %s\n", i, strerror(errno));
    }
  }

  // 4. start stream
  if (xioctl(m_videoFd, VIDIOC_STREAMON, &type) < 0) {
    debug_err("recoverStream: STREAMON failed: %s\n", strerror(errno));
  }
}

int CV4l2Cam::startCapture() {
  if (m_videoFd < 0) {
    AINFO << "video fd is invlaid, devname = " << m_strDevName;
    return -1;
  }

  enum v4l2_buf_type type;

  for (unsigned int index = 0; index < V4L2_BUFFER_LENGHT; index++) {
    struct v4l2_buffer buf;

    memset(&buf, 0, sizeof buf);
    buf.index = index;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_DMABUF;
    buf.m.fd = (unsigned long)m_pNvbuff[index].dmabuff_fd;
    if (xioctl(m_videoFd, VIDIOC_QBUF, &buf) < 0) {
      AINFO << "xioctl VIDIOC_QBUF, devname = " << m_strDevName;
      return -1;
    }
  }

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(m_videoFd, VIDIOC_STREAMON, &type) < 0) {
    AINFO << "xioctl VIDIOC_STREAMON, devname = " << m_strDevName.c_str();
    return -1;
  }
  clearBuffer();
  return 0;
}

int CV4l2Cam::stopCapture() {
  enum v4l2_buf_type type;
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(m_videoFd, VIDIOC_STREAMOFF, &type) < 0) {
    AINFO << "xioctl VIDIOC_STREAMOFF, devname = " << m_strDevName.c_str();
    return -1;
  }
  return 0;
}

bool CV4l2Cam::clearBuffer() {
  struct v4l2_buffer buf;

  for (int i = 0; i < V4L2_BUFFER_LENGHT; i++) {
    memset(&buf, 0, sizeof(struct v4l2_buffer));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(m_videoFd, &rset);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100 * 1000;
    int retsel = select(m_videoFd + 1, &rset, NULL, NULL, &tv);
    if (retsel <= 0) {
      break;
    }
    if (xioctl(m_videoFd, VIDIOC_DQBUF, &buf) < 0) {
      AINFO << "xioctl VIDIOC_DQBUF failed, dev = " << m_strDevName
            << ", errno: " << errno << ", error: " << strerror(errno);
      break;
    }

    if (xioctl(m_videoFd, VIDIOC_QBUF, &buf) < 0) {
      AINFO << "xioctl VIDIOC_QBUF failed, dev = " << m_strDevName
            << ", errno: " << errno << ", error: " << strerror(errno);
      break;
    }
  }
  return true;
}

bool CV4l2Cam::initDevice() { return initDma(); }

void CV4l2Cam::GrabRoutine() {
  pthread_t nativeHandle = pthread_self();
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setschedpolicy(&attr, SCHED_RR);
  struct sched_param param;
  param.sched_priority = 90;
  pthread_attr_setschedparam(&attr, &param);
  pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  pthread_setschedparam(nativeHandle, SCHED_RR, &param);
  pthread_attr_destroy(&attr);

  while (m_bRunning) {
    if (m_nFirstCapture) {
      m_nFirstCapture = false;
      usleep(5 * 1000);
      startCapture();
      while (m_queueTs.size() > 0) {
        m_queueTs.pop_front();
      }
    }
    if (m_bRunning == false) {
      break;
    }
    if (grabImg() == -1) {
      usleep(5 * 1000);
      continue;
    }
  }
}

bool CV4l2Cam::releaseDevice() {
  if (m_videoFd != -1) {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(m_videoFd, VIDIOC_STREAMOFF, &type);
  }

  if (m_videoFd != -1) {
    close(m_videoFd);
    m_videoFd = -1;
  }

  // for (unsigned i = 0; i < V4L2_BUFFER_LENGHT; i++) {
  //   if (m_pNvbuff[i].dmabuff_fd)
  //   NvBufSurf::NvDestroy(m_pNvbuff[i].dmabuff_fd);
  // }
  if (m_pNvbuff) {
    for (unsigned i = 0; i < V4L2_BUFFER_LENGHT; i++) {
      int dmabuf_fd = m_pNvbuff[i].dmabuff_fd;
      if (dmabuf_fd > 0) {
        NvBufSurface *pSurf = nullptr;
        if (NvBufSurfaceFromFd(dmabuf_fd, (void **)&pSurf) == 0 && pSurf) {
          NvBufSurfaceUnMap(pSurf, 0, 0);
        }
        NvBufSurf::NvDestroy(dmabuf_fd);
        m_pNvbuff[i].dmabuff_fd = 0;
      }
    }
  }

  free(m_pNvbuff);
  m_pNvbuff = nullptr;

  return true;
}

bool CV4l2Cam::initDma() {
  bool bRet = true;
  NvBufSurf::NvCommonAllocateParams camparams = {0};
  int fd[V4L2_BUFFER_LENGHT] = {0};

  /* Allocate global buffer context */
  m_pNvbuff = (nv_buffer *)malloc(V4L2_BUFFER_LENGHT * sizeof(nv_buffer));
  if (m_pNvbuff == nullptr) {
    AINFO << "Failed to allocate global buffer context";
    return false;
  }

  camparams.memType = NVBUF_MEM_SURFACE_ARRAY;
  camparams.width = m_dwWidth;
  camparams.height = m_dwHeight;
  camparams.layout = NVBUF_LAYOUT_PITCH;
  camparams.colorFormat = get_nvbuff_color_fmt(V4L2_PIX_FMT_YUYV);
  camparams.memtag = NvBufSurfaceTag_CAMERA;
  if (NvBufSurf::NvAllocate(&camparams, V4L2_BUFFER_LENGHT, fd)) {
    AINFO << "Failed to create NvBuffer.";
    return false;
  }

  /* Create buffer and provide it with camera */
  for (unsigned int index = 0; index < V4L2_BUFFER_LENGHT; index++) {
    NvBufSurface *pSurf = nullptr;

    m_pNvbuff[index].dmabuff_fd = fd[index];

    if (-1 == NvBufSurfaceFromFd(fd[index], (void **)(&pSurf))) {
      AINFO << "Failed to get NvBuffer parameters.";
      return false;
    }

    /* TODO: add multi-planar support
       Currently only supports YUV422 interlaced single-planar */

    if (-1 == NvBufSurfaceMap(pSurf, 0, 0, NVBUF_MAP_READ_WRITE)) {
      AINFO << "Failed to map buffer.";
      return false;
    }
    m_pNvbuff[index].start =
        (unsigned char *)pSurf->surfaceList[0].mappedAddr.addr[0];
    m_pNvbuff[index].size = pSurf->surfaceList[0].dataSize;
  }

  /* Request camera v4l2 buffer */
  struct v4l2_requestbuffers rb;
  memset(&rb, 0, sizeof(rb));
  rb.count = V4L2_BUFFER_LENGHT;
  rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rb.memory = V4L2_MEMORY_DMABUF;
  if (ioctl(m_videoFd, VIDIOC_REQBUFS, &rb) < 0) {
    AINFO << "Failed to request v4l2 buffers: " << strerror(errno) << ", errno"
          << errno;
    return false;
  }
  if (rb.count != V4L2_BUFFER_LENGHT) {
    AINFO << "V4l2 buffer number is not as desired.";
    return false;
  }

  for (unsigned int index = 0; index < V4L2_BUFFER_LENGHT; index++) {
    struct v4l2_buffer buf;

    /* Query camera v4l2 buf length */
    memset(&buf, 0, sizeof buf);
    buf.index = index;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_DMABUF;

    if (ioctl(m_videoFd, VIDIOC_QUERYBUF, &buf) < 0) {
      AINFO << "Failed to query buff: " << strerror(errno)
            << ", errno: " << errno;
      return false;
    }
    /* TODO: add support for multi-planer
       Enqueue empty v4l2 buff into camera capture plane */
    buf.m.fd = (unsigned long)m_pNvbuff[index].dmabuff_fd;
    if (buf.length != m_pNvbuff[index].size) {
      AWARN << "Camera v4l2 buf length is not expected.";
      m_pNvbuff[index].size = buf.length;
    }
  }
  return bRet;
}

bool CV4l2Cam::setVideoFmt() {
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = m_dwWidth;
  fmt.fmt.pix.height = m_dwHeight;

  switch (m_dwFormat) {
    case 0:
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SRGGB12;
      break;
    case 1:
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      break;
    case 2:
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
      break;
    default:
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      debug_err("not support format:%d, default use YUYV\n", m_dwFormat);
      break;
  }

  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

  if (-1 == xioctl(m_videoFd, VIDIOC_S_FMT, &fmt)) {
    debug_err("xioctl  VIDIOC_S_FMT failed,devname=%s,errno: %d, %s\n",
              m_strDevName.c_str(), errno, strerror(errno));
    return false;
  }
  return true;
}

bool CV4l2Cam::checkCapabilities() {
  struct v4l2_capability cap;
  if (-1 == xioctl(m_videoFd, VIDIOC_QUERYCAP, &cap)) {
    debug_err("xioctl  VIDIOC_QUERYCAP failed,devname=%s,errno: %d, %s\n",
              m_strDevName.c_str(), errno, strerror(errno));
    return false;
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    debug_err("capabilities not support V4L2_CAP_VIDEO_CAPTURE \n");
    return false;
  }

  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    debug_err("capabilities not support V4L2_CAP_STREAMING \n");
    return false;
  }
  return true;
}

int CV4l2Cam::xioctl(int fh, int request, void *arg) {
  int r;
  do {
    r = ioctl(fh, request, arg);
  } while (-1 == r && EINTR == errno);

  return r;
}

long readOffsetNs() {
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

int CV4l2Cam::GetOffset() {
  char tmpStr[128] = {0};
  FILE *pf =
      fopen("/sys/devices/system/clocksource/clocksource0/offset_ns", "r");
  if (pf) {
    fgets(tmpStr, sizeof(tmpStr), pf);
    m_lNsecOffset = atol(tmpStr);
    fclose(pf);
  } else {
    m_lNsecOffset = readOffsetNs();
    if (m_lNsecOffset < 0) {
      debug_err("read offset failed, m_lNsecOffset = %ld\n", m_lNsecOffset);
      return -1;
    }
  }
  return 0;
}

void CV4l2Cam::rtcpuToRealtime(timeval rtcpu_time, timespec *real_time) {
  struct timespec real_sample, monotonic_sample, monotonic_time, time_diff;
  const int64_t NSEC_PER_SEC = 1000000000;

  // printf("[chan:%d]rtcpu_time: %ld.%09ld\n", m_nChan, rtcpu_time.tv_sec,
  // rtcpu_time.tv_usec*1000);
  long long ns = rtcpu_time.tv_sec * NSEC_PER_SEC + rtcpu_time.tv_usec * 1000 -
                 m_lNsecOffset;
  monotonic_time.tv_sec = ns / NSEC_PER_SEC;
  monotonic_time.tv_nsec = ns % NSEC_PER_SEC;
  // printf("[chan:%d] monotonic_time: %ld.%09ld\n",
  // m_nChan,monotonic_time.tv_sec, monotonic_time.tv_nsec);

  clock_gettime(CLOCK_MONOTONIC_RAW, &monotonic_sample);
  clock_gettime(CLOCK_REALTIME, &real_sample);

  time_diff.tv_sec = real_sample.tv_sec - monotonic_sample.tv_sec;
  time_diff.tv_nsec = real_sample.tv_nsec - monotonic_sample.tv_nsec;

  real_time->tv_sec = monotonic_time.tv_sec + time_diff.tv_sec;
  real_time->tv_nsec = monotonic_time.tv_nsec + time_diff.tv_nsec;
  if (real_time->tv_nsec >= NSEC_PER_SEC) {
    ++real_time->tv_sec;
    real_time->tv_nsec -= NSEC_PER_SEC;
  } else if (real_time->tv_nsec < 0) {
    --real_time->tv_sec;
    real_time->tv_nsec += NSEC_PER_SEC;
  }
  return;
}
