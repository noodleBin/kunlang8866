#pragma once
#include <string>
#include <memory>
#include <queue>

#include <bits/stdc++.h>

#include "camera_type.h"
#include "jpeg_encode_api.h"
#include "vin_log.h"

#include "cyber/cyber.h"
#include "modules/drivers/proto/sensor_image.pb.h"

namespace century {
namespace drivers {
namespace camera {
using century::cyber::Reader;
using century::cyber::Writer;
using century::drivers::Image;
// using century::drivers::camera::config::Config;
class CMgrCameraJpegEnc {
 public:
  CMgrCameraJpegEnc(
      int dwChan, int dwWidth, int dwHeight, int dwQuality,
      std::shared_ptr<Writer<CompressedImage>> compressed_image_writer,
      const std::string frame_id);
  ~CMgrCameraJpegEnc();
  virtual bool Init();
  virtual bool Release();
  bool Save(int nChan, struct timespec stTime, int nWidth, int nHeight,
            unsigned char *pData, int nDatalen);
  friend void CAMERA_CALL JpegEncCallBack(unsigned char *data, int datalen,
                                          void *userdata);

 protected:
  bool createHandle();
  bool destroyHandle();

 private:
  int m_dwChan;
  int m_dwWidth;
  int m_dwHeight;
  int m_dwQuality;
  void *m_pEncHandle = nullptr;
  std::mutex m_mutex;
  unsigned char *m_pYuv = nullptr;
  std::queue<struct timespec> m_stTimes;
  std::shared_ptr<CompressedImage> compressed_image_ = nullptr;
  std::shared_ptr<Writer<CompressedImage>> compressed_image_writer_ = nullptr;
};
}  // namespace camera
}  // namespace drivers
}  // namespace century