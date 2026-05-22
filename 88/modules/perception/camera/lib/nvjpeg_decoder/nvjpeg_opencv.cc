#include "modules/perception/camera/lib/nvjpeg_decoder/nvjpeg_opencv.h"

#include <cstring>

#include "cyber/common/log.h"

namespace century {
namespace perception {
namespace camera {

NvJpegDecoder::NvJpegDecoder(std::string name, int device_id, int width,
                             int height, int channel, bool use_undistortion)
    : name_(std::move(name)),
      device_id_(device_id),
      width_(width),
      height_(height),
      channel_(channel),
      use_undistortion_(use_undistortion) {
  cudaSetDevice(device_id_);
  cudaStreamCreate(&stream_);
  cudaEventCreateWithFlags(&frame_ready_event_, cudaEventDisableTiming);

  size_t bytes = static_cast<size_t>(width_) * height_ * channel_;
  size_t yuv_size = width_ * height_ * 3 / 2;

  // pinned host buffer
  cudaMallocHost(&h_rgb_, bytes);

  // device buffers
  cudaMalloc(&d_rgb_, bytes);
  if (use_undistortion_) {
    cudaMalloc(&d_dst_, bytes);
  }

  // YUV420 buffers
  cudaMalloc(&d_yuv_y_, width_ * height_);
  cudaMalloc(&d_yuv_u_, (width_ / 2) * (height_ / 2));
  cudaMalloc(&d_yuv_v_, (width_ / 2) * (height_ / 2));
  cudaMalloc(&d_yuv_temp_, yuv_size);

  // Allocate YUYV buffer (width * height * 2 bytes)
  cudaMalloc(&d_yuyv_temp_, width_ * height_ * 2);
}

NvJpegDecoder::~NvJpegDecoder() {
  cudaSetDevice(device_id_);

  if (d_yuv_y_) {
    cudaFree(d_yuv_y_);
  }
  if (d_yuv_u_) {
    cudaFree(d_yuv_u_);
  }
  if (d_yuv_v_) {
    cudaFree(d_yuv_v_);
  }
  if (d_yuv_temp_) {
    cudaFree(d_yuv_temp_);
  }
  if (d_yuyv_temp_) {
    cudaFree(d_yuyv_temp_);
  }
  if (d_mapx_) {
    cudaFree(d_mapx_);
  }
  if (d_mapy_) {
    cudaFree(d_mapy_);
  }
  if (d_dst_) {
    cudaFree(d_dst_);
  }
  if (d_rgb_) {
    cudaFree(d_rgb_);
  }
  if (h_rgb_) {
    cudaFreeHost(h_rgb_);
  }
  if (frame_ready_event_) {
    cudaEventDestroy(frame_ready_event_);
  }

  if (stream_) {
    cudaStreamDestroy(stream_);
  }
}

bool NvJpegDecoder::SetDMap(std::vector<float>& dmapx,
                            std::vector<float>& dmapy) {
  if (!use_undistortion_) {
    return false;
  }

  cudaSetDevice(device_id_);

  size_t size = static_cast<size_t>(width_) * height_ * sizeof(float);

  cudaMalloc(&d_mapx_, size);
  cudaMalloc(&d_mapy_, size);

  cudaMemcpyAsync(d_mapx_, dmapx.data(), size, cudaMemcpyHostToDevice, stream_);
  cudaMemcpyAsync(d_mapy_, dmapy.data(), size, cudaMemcpyHostToDevice, stream_);

  cudaStreamSynchronize(stream_);
  return true;
}

bool NvJpegDecoder::ProcessYuv420(const uint8_t* raw_data, size_t data_size,
                                  int width, int height) {
  size_t expected_size = width * height * 3 / 2;
  if (data_size != expected_size) {
    AERROR << "YUV420 data size mismatch. Expected: " << expected_size
           << ", Got: " << data_size;
    return false;
  }

  // Use the external low-priority stream when available.
  cudaStream_t work_stream = external_stream_ ? external_stream_ : stream_;

  // Upload YUV420 data to GPU
  cudaError_t err = cudaMemcpyAsync(d_yuv_temp_, raw_data, data_size,
                                    cudaMemcpyHostToDevice, work_stream);
  if (err != cudaSuccess) {
    AERROR << "Failed to upload YUV420 data to GPU: "
           << cudaGetErrorString(err);
    return false;
  }

  // Split YUV420 planes
  size_t y_size = width * height;
  size_t uv_size = (width / 2) * (height / 2);

  cudaMemcpyAsync(d_yuv_y_, d_yuv_temp_, y_size, cudaMemcpyDeviceToDevice,
                  work_stream);
  cudaMemcpyAsync(d_yuv_u_, d_yuv_temp_ + y_size, uv_size,
                  cudaMemcpyDeviceToDevice, work_stream);
  cudaMemcpyAsync(d_yuv_v_, d_yuv_temp_ + y_size + uv_size, uv_size,
                  cudaMemcpyDeviceToDevice, work_stream);

  // YUV420 -> RGB conversion on GPU
  yuv420_to_rgb_cuda(d_yuv_y_, d_yuv_u_, d_yuv_v_, width, height, d_rgb_,
                     work_stream);

  // Apply undistortion if enabled
  if (use_undistortion_) {
    if (!d_mapx_ || !d_mapy_) {
      AERROR << "[" << name_ << "] dmap not set";
      return false;
    }
    fisheye_undistort_resize_cuda(d_rgb_, width, height, d_mapx_, d_mapy_,
                                  d_dst_, width_, height_, work_stream);
  }

  cudaEventRecord(frame_ready_event_, work_stream);
  return true;
}

bool NvJpegDecoder::ProcessYuyv(const uint8_t* raw_data, size_t data_size,
                                int width, int height) {
  size_t expected_size = width * height * 2;
  if (data_size != expected_size) {
    AERROR << "YUYV data size mismatch. Expected: " << expected_size
           << ", Got: " << data_size;
    return false;
  }

  // Use the external low-priority stream when available.
  cudaStream_t work_stream = external_stream_ ? external_stream_ : stream_;

  cudaError_t err = cudaMemcpyAsync(d_yuyv_temp_, raw_data, data_size,
                                    cudaMemcpyHostToDevice, work_stream);
  if (err != cudaSuccess) {
    AERROR << "Failed to upload YUYV data to GPU: " << cudaGetErrorString(err);
    return false;
  }

  yuyv_to_rgb_cuda(d_yuyv_temp_, width, height, d_rgb_, work_stream);

  if (use_undistortion_) {
    if (!d_mapx_ || !d_mapy_) {
      AERROR << "[" << name_ << "] dmap not set";
      return false;
    }
    fisheye_undistort_resize_cuda(d_rgb_, width, height, d_mapx_, d_mapy_,
                                  d_dst_, width_, height_, work_stream);
  }

  cudaEventRecord(frame_ready_event_, work_stream);
  return true;
}

bool NvJpegDecoder::Decode(const uint8_t* jpeg_data, size_t jpeg_size,
                           int* width, int* height) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!jpeg_data || jpeg_size < 10) {
    return false;
  }

  std::vector<uint8_t> buf(jpeg_data, jpeg_data + jpeg_size);
  cpu_bgr_ = cv::imdecode(buf, cv::IMREAD_COLOR);
  if (cpu_bgr_.empty()) {
    AERROR << "[" << name_ << "] OpenCV imdecode failed";
    return false;
  }

  *width = cpu_bgr_.cols;
  *height = cpu_bgr_.rows;

  // BGR -> RGB (aligned with nvjpeg RGBI)
  cv::cvtColor(cpu_bgr_, cpu_rgb_, cv::COLOR_BGR2RGB);

  size_t bytes = static_cast<size_t>(*width) * (*height) * channel_;

  // Use the external low-priority stream when available.
  cudaStream_t work_stream = external_stream_ ? external_stream_ : stream_;

  // CPU -> GPU
  cudaMemcpyAsync(d_rgb_, cpu_rgb_.data, bytes, cudaMemcpyHostToDevice,
                  work_stream);

  if (use_undistortion_) {
    if (!d_mapx_ || !d_mapy_) {
      AERROR << "[" << name_ << "] dmap not set";
      return false;
    }

    fisheye_undistort_resize_cuda(d_rgb_, *width, *height, d_mapx_, d_mapy_,
                                  d_dst_, width_, height_, work_stream);
  }

  cudaEventRecord(frame_ready_event_, work_stream);
  return true;
}

bool NvJpegDecoder::DecodeRaw(const uint8_t* raw_data, size_t data_size,
                              int width, int height,
                              const std::string& encoding) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (width != width_ || height != height_) {
    AERROR << "DecodeRaw: Image dimensions mismatch. Expected: " << width_
           << "x" << height_ << ", Got: " << width << "x" << height;
    return false;
  }

  if (nullptr == raw_data || 0 == data_size) {
    AERROR << "DecodeRaw: Invalid input data";
    return false;
  }

  bool success = false;

  if ("yuv420" == encoding || "i420" == encoding || "I420" == encoding) {
    success = ProcessYuv420(raw_data, data_size, width, height);
  } else if ("yuyv" == encoding || "YUYV" == encoding || "yuv422" == encoding ||
             "UYVY" == encoding || "uyvy" == encoding) {
    success = ProcessYuyv(raw_data, data_size, width, height);
  } else if ("rgb8" == encoding || "RGB8" == encoding) {
    // Use the external low-priority stream when available.
    cudaStream_t work_stream = external_stream_ ? external_stream_ : stream_;

    cudaMemcpyAsync(d_rgb_, raw_data, width * height * 3,
                    cudaMemcpyHostToDevice, work_stream);
    if (use_undistortion_) {
      if (!d_mapx_ || !d_mapy_) {
        AERROR << "[" << name_ << "] dmap not set";
        return false;
      }
      fisheye_undistort_resize_cuda(d_rgb_, width, height, d_mapx_, d_mapy_,
                                    d_dst_, width_, height_, work_stream);
    }
    cudaEventRecord(frame_ready_event_, work_stream);
    success = true;
  } else {
    AERROR << "DecodeRaw: Unsupported encoding: " << encoding;
    return false;
  }

  return success;
}

bool NvJpegDecoder::FillBlackImage() {
  std::lock_guard<std::mutex> lock(mutex_);

  cudaStream_t work_stream = external_stream_ ? external_stream_ : stream_;
  size_t bytes = static_cast<size_t>(width_) * height_ * channel_;

  cudaError_t err = cudaMemsetAsync(d_rgb_, 0, bytes, work_stream);
  if (err != cudaSuccess) {
    AERROR << "Failed to fill black RGB image: " << cudaGetErrorString(err);
    return false;
  }

  if (use_undistortion_) {
    err = cudaMemsetAsync(d_dst_, 0, bytes, work_stream);
    if (err != cudaSuccess) {
      AERROR << "Failed to fill black undistorted image: "
             << cudaGetErrorString(err);
      return false;
    }
  }

  cudaEventRecord(frame_ready_event_, work_stream);
  return true;
}

uint8_t* NvJpegDecoder::GetHostBuffer() const {
  uint8_t* src = use_undistortion_ ? d_dst_ : d_rgb_;
  size_t bytes = static_cast<size_t>(width_) * height_ * channel_;

  cudaMemcpyAsync(h_rgb_, src, bytes, cudaMemcpyDeviceToHost, stream_);
  cudaStreamSynchronize(stream_);
  return h_rgb_;
}

uint8_t* NvJpegDecoder::GetDeviceBuffer() const {
  return use_undistortion_ ? d_dst_ : d_rgb_;
}

cudaStream_t NvJpegDecoder::GetStream() const {
  return external_stream_ ? external_stream_ : stream_;
}

void NvJpegDecoder::SetExternalStream(cudaStream_t external_stream) {
  std::lock_guard<std::mutex> lock(mutex_);
  // Store the external stream for low-priority decode work.
  external_stream_ = external_stream;
  AINFO << "Set external (low-priority) stream for camera decoder: " << name_;
}

void NvJpegDecoder::ClearExternalStream() {
  std::lock_guard<std::mutex> lock(mutex_);
  external_stream_ = nullptr;
}

}  // namespace camera
}  // namespace perception
}  // namespace century
