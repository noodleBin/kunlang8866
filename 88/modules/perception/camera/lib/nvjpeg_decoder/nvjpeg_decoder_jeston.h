#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <cuda_runtime.h>

#include "NvBuffer.h"
#include "NvJpegDecoder.h"
#include "cuda_runtime_api.h"
#include "nvbuf_utils.h"

namespace century {
namespace perception {
namespace camera {

extern "C" void fisheye_undistort_resize_cuda(const unsigned char* input,
                                              int in_w, int in_h,
                                              const float* mapx,
                                              const float* mapy,
                                              unsigned char* output, int out_w,
                                              int out_h, cudaStream_t& stream);

extern "C" void nv12_to_rgb_cuda(const unsigned char* y_plane,
                                 const unsigned char* uv_plane, int pitch_y,
                                 int pitch_uv, int width, int height,
                                 unsigned char* rgb_output,
                                 cudaStream_t stream);

extern "C" void yuv420_to_rgb_cuda(const unsigned char* y_plane,
                                   const unsigned char* u_plane,
                                   const unsigned char* v_plane, int width,
                                   int height, unsigned char* rgb_output,
                                   cudaStream_t stream);

extern "C" void yuv422_to_rgb_cuda(const unsigned char* yuv_data, int width,
                                   int height, unsigned char* rgb_data,
                                   cudaStream_t stream);

extern "C" void yuyv_to_rgb_cuda(const unsigned char* yuyv_data, int width,
                                 int height, unsigned char* rgb_output,
                                 cudaStream_t stream);

class NvJpegDecoder {
 public:
  NvJpegDecoder(std::string name, int device_id, int width, int height,
                int channel, bool use_undistortion = false);
  ~NvJpegDecoder();

  bool Decode(const uint8_t* jpeg_data, size_t jpeg_size, int* width,
              int* height);

  /**
   * @brief Decode raw YUV420 data to RGB on GPU
   * @param raw_data YUV420 data (I420 format: Y plane + U plane + V plane)
   * @param data_size Size of raw data in bytes (width * height * 1.5)
   * @param width Image width
   * @param height Image height
   * @param encoding Format: "yuv420", "i420", "nv12", "yuv422", "uyvy"
   * @return true if successful
   */
  bool DecodeRaw(const uint8_t* raw_data, size_t data_size, int width,
                 int height, const std::string& encoding);

  bool FillBlackImage();

  uint8_t* GetHostBuffer();
  uint8_t* GetDeviceBuffer() const;
  cudaStream_t GetStream() const;

  /**
   * @brief Get the CUDA event that signals when frame processing is complete
   * @return cudaEvent_t that can be used for cross-stream synchronization
   */
  cudaEvent_t GetReadyEvent() const { return frame_ready_event_; }

  bool SetDMap(std::vector<float>& dmapx, std::vector<float>& dmapy);

  /**
   * @brief Set external CUDA stream to use (for priority control)
   * @param external_stream The external stream to use
   */
  void SetExternalStream(cudaStream_t external_stream);
  void ClearExternalStream();
  void SaveDecodedData(const std::string& filename);

  /**
   * @brief Get the number of frames processed by this decoder
   */
  int GetFrameCount() const { return cam_frame_count_; }

 private:
  std::string name_;
  int device_id_;
  int width_, height_, channel_;
  bool use_undistortion_;
  std::recursive_mutex mutex_;
  cudaStream_t stream_;
  cudaStream_t external_stream_ = nullptr;  // External low-priority stream.
  uint8_t* h_rgb_ = nullptr;
  uint8_t* d_rgb_ = nullptr;
  uint8_t* d_dst_ = nullptr;
  void* d_yuv_y_ = nullptr;
  void* d_yuv_u_ = nullptr;
  void* d_yuv_v_ = nullptr;
  float* d_mapx_ = nullptr;
  float* d_mapy_ = nullptr;

  // CUDA event for signaling frame completion
  cudaEvent_t frame_ready_event_ = nullptr;

  // Temporary YUV buffer for raw decoding
  uint8_t* d_yuv_temp_ = nullptr;
  size_t d_yuv_temp_size_ = 0;

  // YUYV buffer for raw decoding (width * height * 2 bytes)
  uint8_t* d_yuyv_temp_ = nullptr;
  size_t d_yuyv_temp_size_ = 0;

  std::unique_ptr<::NvJPEGDecoder> nvJpegDecoderImpl_;
  NvBuffer* nv_buffer_ = nullptr;

  bool ConvertYuv420ToRgb(uint32_t width, uint32_t height);

  bool ProcessYuv420(const uint8_t* raw_data, size_t data_size, int width,
                     int height);

  bool ProcessYuyv(const uint8_t* raw_data, size_t data_size, int width,
                   int height);

  int cam_frame_count_ = 0;
  static int globalFrameCount_;
};

}  // namespace camera
}  // namespace perception
}  // namespace century
