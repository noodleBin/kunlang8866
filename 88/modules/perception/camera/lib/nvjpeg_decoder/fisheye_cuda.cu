#include <math.h>
#include <stdio.h>

#include <cuda_runtime.h>

#define BLOCK 16

__device__ __forceinline__ unsigned char clamp_uchar(float v) {
  v = fminf(255.f, fmaxf(0.f, v));
  return (unsigned char)(v + 0.5f);
}

__global__ void undistort_kernel(const unsigned char* src, int w, int h,
                                 const float* mapx, const float* mapy,
                                 unsigned char* dst) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= w || y >= h) {
    return;
  }

  int idx = y * w + x;

  float fx = mapx[idx];
  float fy = mapy[idx];

  if (fx < 0 || fy < 0 || fx >= w - 1 || fy >= h - 1) {
    dst[idx * 3 + 0] = 0;
    dst[idx * 3 + 1] = 0;
    dst[idx * 3 + 2] = 0;
    return;
  }

  int x0 = (int)floorf(fx);
  int y0 = (int)floorf(fy);
  int x1 = x0 + 1;
  int y1 = y0 + 1;

  float dx = fx - x0;
  float dy = fy - y0;

  for (int c = 0; c < 3; ++c) {
    float p00 = src[(y0 * w + x0) * 3 + c];
    float p10 = src[(y0 * w + x1) * 3 + c];
    float p01 = src[(y1 * w + x0) * 3 + c];
    float p11 = src[(y1 * w + x1) * 3 + c];

    float v = (1 - dx) * (1 - dy) * p00 + dx * (1 - dy) * p10 +
              (1 - dx) * dy * p01 + dx * dy * p11;

    dst[idx * 3 + c] = clamp_uchar(v);
  }
}

__global__ void resize_kernel(const unsigned char* src, int in_w, int in_h,
                              unsigned char* dst, int out_w, int out_h) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= out_w || y >= out_h) {
    return;
  }

  float scale_x = (float)in_w / out_w;
  float scale_y = (float)in_h / out_h;

  float fx = (x + 0.5f) * scale_x - 0.5f;
  float fy = (y + 0.5f) * scale_y - 0.5f;

  int x0 = (int)floorf(fx);
  int y0 = (int)floorf(fy);
  int x1 = min(x0 + 1, in_w - 1);
  int y1 = min(y0 + 1, in_h - 1);

  float dx = fx - x0;
  float dy = fy - y0;

  for (int c = 0; c < 3; ++c) {
    float p00 = src[(y0 * in_w + x0) * 3 + c];
    float p10 = src[(y0 * in_w + x1) * 3 + c];
    float p01 = src[(y1 * in_w + x0) * 3 + c];
    float p11 = src[(y1 * in_w + x1) * 3 + c];

    float v = (1 - dx) * (1 - dy) * p00 + dx * (1 - dy) * p10 +
              (1 - dx) * dy * p01 + dx * dy * p11;

    dst[(y * out_w + x) * 3 + c] = clamp_uchar(v);
  }
}

extern "C" void fisheye_undistort_resize_cuda(const unsigned char* input,
                                              int in_w, int in_h,
                                              const float* mapx,
                                              const float* mapy,
                                              unsigned char* output, int out_w,
                                              int out_h, cudaStream_t& stream) {
  dim3 block(BLOCK, BLOCK);
  dim3 grid1((in_w + BLOCK - 1) / BLOCK, (in_h + BLOCK - 1) / BLOCK);

  undistort_kernel<<<grid1, block, 0, stream>>>(input, in_w, in_h, mapx, mapy,
                                                output);
}

__global__ void yuv422_to_rgb_kernel(const unsigned char* yuv_data, int width,
                                     int height, unsigned char* rgb_data) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= width || y >= height) {
    return;
  }

  int rgb_idx = (y * width + x) * 3;
  int yuv_idx = y * width * 2 + x * 2;

  unsigned char y_val = yuv_data[yuv_idx];
  unsigned char u_val = yuv_data[(y * width * 2) + (x / 2) * 4 + 1];
  unsigned char v_val = yuv_data[(y * width * 2) + (x / 2) * 4 + 3];

  if (0 == x % 2) {
    u_val = yuv_data[yuv_idx + 1];
    v_val = yuv_data[yuv_idx + 3];
  }

  int c = y_val - 16;
  int d = u_val - 128;
  int e = v_val - 128;

  int r = (298 * c + 409 * e + 128) >> 8;
  int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
  int b = (298 * c + 516 * d + 128) >> 8;

  rgb_data[rgb_idx] = clamp_uchar(r);
  rgb_data[rgb_idx + 1] = clamp_uchar(g);
  rgb_data[rgb_idx + 2] = clamp_uchar(b);
}

extern "C" void yuv422_to_rgb_cuda(const unsigned char* yuv_data, int width,
                                   int height, unsigned char* rgb_data,
                                   cudaStream_t stream) {
  dim3 block(BLOCK, BLOCK);
  dim3 grid((width + BLOCK - 1) / BLOCK, (height + BLOCK - 1) / BLOCK);

  yuv422_to_rgb_kernel<<<grid, block, 0, stream>>>(yuv_data, width, height,
                                                   rgb_data);
}

__global__ void nv12_to_rgb_kernel(const unsigned char* y_plane,
                                   const unsigned char* uv_plane, int pitch_y,
                                   int pitch_uv, int width, int height,
                                   unsigned char* rgb_output) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= width || y >= height) {
    return;
  }

  unsigned char y_val = y_plane[y * pitch_y + x];

  int uv_x = x / 2;
  int uv_y = y / 2;
  // Jetson NvJPEGDecoder outputs NV21 (VU order), not NV12 (UV order)
  unsigned char v_val = uv_plane[uv_y * pitch_uv + uv_x * 2];
  unsigned char u_val = uv_plane[uv_y * pitch_uv + uv_x * 2 + 1];

  int y_int = (int)y_val - 16;
  int u_int = (int)u_val - 128;
  int v_int = (int)v_val - 128;

  int r = (298 * y_int + 409 * v_int + 128) >> 8;
  int g = (298 * y_int - 100 * u_int - 208 * v_int + 128) >> 8;
  int b = (298 * y_int + 516 * u_int + 128) >> 8;

  r = r < 0 ? 0 : (r > 255 ? 255 : r);
  g = g < 0 ? 0 : (g > 255 ? 255 : g);
  b = b < 0 ? 0 : (b > 255 ? 255 : b);

  int rgb_idx = (y * width + x) * 3;
  rgb_output[rgb_idx + 0] = (unsigned char)r;
  rgb_output[rgb_idx + 1] = (unsigned char)g;
  rgb_output[rgb_idx + 2] = (unsigned char)b;
}

extern "C" void nv12_to_rgb_cuda(const unsigned char* y_plane,
                                 const unsigned char* uv_plane, int pitch_y,
                                 int pitch_uv, int width, int height,
                                 unsigned char* rgb_output,
                                 cudaStream_t stream) {
  dim3 block(BLOCK, BLOCK);
  dim3 grid((width + BLOCK - 1) / BLOCK, (height + BLOCK - 1) / BLOCK);

  nv12_to_rgb_kernel<<<grid, block, 0, stream>>>(
      y_plane, uv_plane, pitch_y, pitch_uv, width, height, rgb_output);
}

__global__ void yuv420_to_rgb_kernel(const unsigned char* y_plane,
                                     const unsigned char* u_plane,
                                     const unsigned char* v_plane, int width,
                                     int height, unsigned char* rgb_output) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= width || y >= height) {
    return;
  }

  unsigned char y_val = y_plane[y * width + x];

  int uv_x = x / 2;
  int uv_y = y / 2;
  unsigned char u_val = u_plane[uv_y * (width / 2) + uv_x];
  unsigned char v_val = v_plane[uv_y * (width / 2) + uv_x];

  int y_int = (int)y_val - 16;
  int u_int = (int)u_val - 128;
  int v_int = (int)v_val - 128;

  int r = (298 * y_int + 409 * v_int + 128) >> 8;
  int g = (298 * y_int - 100 * u_int - 208 * v_int + 128) >> 8;
  int b = (298 * y_int + 516 * u_int + 128) >> 8;

  r = r < 0 ? 0 : (r > 255 ? 255 : r);
  g = g < 0 ? 0 : (g > 255 ? 255 : g);
  b = b < 0 ? 0 : (b > 255 ? 255 : b);

  int rgb_idx = (y * width + x) * 3;
  rgb_output[rgb_idx + 0] = (unsigned char)r;
  rgb_output[rgb_idx + 1] = (unsigned char)g;
  rgb_output[rgb_idx + 2] = (unsigned char)b;
}

extern "C" void yuv420_to_rgb_cuda(const unsigned char* y_plane,
                                   const unsigned char* u_plane,
                                   const unsigned char* v_plane, int width,
                                   int height, unsigned char* rgb_output,
                                   cudaStream_t stream) {
  dim3 block(BLOCK, BLOCK);
  dim3 grid((width + BLOCK - 1) / BLOCK, (height + BLOCK - 1) / BLOCK);

  yuv420_to_rgb_kernel<<<grid, block, 0, stream>>>(y_plane, u_plane, v_plane,
                                                   width, height, rgb_output);
}

__global__ void yuyv_to_rgb_kernel(const unsigned char* yuyv_data, int width,
                                   int height, unsigned char* rgb_output) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= width || y >= height) {
    return;
  }

  int yuyv_idx = (y * width + x) * 2;
  int rgb_idx = (y * width + x) * 3;

  unsigned char y_val = yuyv_data[yuyv_idx];
  unsigned char u_val = yuyv_data[yuyv_idx + 1 - (x % 2) * 2];
  unsigned char v_val = yuyv_data[yuyv_idx + 3 - (x % 2) * 2];

  int y_int = (int)y_val - 16;
  int u_int = (int)u_val - 128;
  int v_int = (int)v_val - 128;

  int r = (298 * y_int + 409 * v_int + 128) >> 8;
  int g = (298 * y_int - 100 * u_int - 208 * v_int + 128) >> 8;
  int b = (298 * y_int + 516 * u_int + 128) >> 8;

  r = r < 0 ? 0 : (r > 255 ? 255 : r);
  g = g < 0 ? 0 : (g > 255 ? 255 : g);
  b = b < 0 ? 0 : (b > 255 ? 255 : b);

  rgb_output[rgb_idx + 0] = (unsigned char)r;
  rgb_output[rgb_idx + 1] = (unsigned char)g;
  rgb_output[rgb_idx + 2] = (unsigned char)b;
}

extern "C" void yuyv_to_rgb_cuda(const unsigned char* yuyv_data, int width,
                                 int height, unsigned char* rgb_output,
                                 cudaStream_t stream) {
  dim3 block(BLOCK, BLOCK);
  dim3 grid((width + BLOCK - 1) / BLOCK, (height + BLOCK - 1) / BLOCK);

  yuyv_to_rgb_kernel<<<grid, block, 0, stream>>>(yuyv_data, width, height,
                                                 rgb_output);
}
