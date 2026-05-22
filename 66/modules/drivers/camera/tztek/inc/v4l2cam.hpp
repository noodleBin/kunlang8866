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

#ifndef _H_V4L2CAM_H
#define _H_V4L2CAM_H
#include <semaphore.h>
#include <string.h>

#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <linux/videodev2.h>

#include "NvBufSurface.h"
#include "camera_type.h"

#include "cyber/cyber.h"

typedef struct {
  unsigned char *start;
  unsigned int size;
  int dmabuff_fd;
} nv_buffer;

typedef struct {
  unsigned int v4l2_pixfmt;
  NvBufSurfaceColorFormat nvbuff_color;
} nv_color_fmt;

#define TIMESTAMP_MODE_HARD 0
#define TIMESTAMP_MODE_HARD_ONCE 1
#define TIMESTAMP_MODE_SOFT 2

typedef void (*PV4L2_DATACALLBACK)(int nChan, struct timespec stTime,
                                   int nWidth, int nHeight,
                                   unsigned char *pData, int nDatalen,
                                   void *pUserData);

class CV4l2Cam {
 public:
  CV4l2Cam(int dwPipeId, int dwVideoIndex, int dwWidth, int dwHeight, int dwFps,
           int format);
  ~CV4l2Cam();
  int Init();
  int Release();
  int StartAcquire();
  int StopAcquire();
  void GrabRoutine();
  void SetDataCallback(PV4L2_DATACALLBACK cb, void *pUserData);
  int GetFps() { return m_nFps; }
  int ClearBuffer();

 protected:
  int grabImg();
  int startCapture();
  int stopCapture();
  void rtcpuToRealtime(timeval rtcpu_time, timespec *real_time);
  bool initDevice();
  bool releaseDevice();
  bool clearBuffer();
  bool initDma();
  bool setVideoFmt();
  bool checkCapabilities();
  int xioctl(int fh, int request, void *arg);
  int GetOffset();
  void recoverStream();

 private:
  bool m_bRunning;
  int m_dwFault;
  int m_nChan;
  int m_nFps;
  int m_nErr;
  long m_llSkippedFrameNum;
  uint32_t m_dwFrameCnt;
  int32_t m_videoFd;
  uint32_t m_dwHeight;
  uint32_t m_dwWidth;
  std::string m_strDevName;
  std::deque<timespec> m_queueTs;
  sem_t semTrig;
  std::mutex m_mutex;
  std::unique_ptr<std::thread> m_pGrabThread = nullptr;
  PV4L2_DATACALLBACK m_Cb = nullptr;
  void *m_pUserData = nullptr;
  bool m_nFirstCapture;
  uint64_t m_lstFrameTimestamp;
  long m_lNsecOffset;

  struct timeval m_lstV4l2Timestamp;
  int m_dwFormat;
  nv_buffer *m_pNvbuff = nullptr;
  NvBufSurface *m_pSurf = nullptr;
};
#endif
