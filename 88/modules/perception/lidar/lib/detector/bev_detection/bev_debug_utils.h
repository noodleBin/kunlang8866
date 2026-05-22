#pragma once

#include <string>
#include <vector>

#include <cuda_runtime.h>

#include "modules/perception/lidar/lib/detector/bev_detection/bevfusion/head-transbbox.hpp"
#include "modules/perception/lidar/lib/detector/dnn_common/common/dtype.hpp"
#include "modules/perception/lidar/lib/detector/dnn_common/common/tensor.hpp"

namespace century {
namespace perception {
namespace lidar {

class BevDebugUtils {
 public:
  BevDebugUtils();
  ~BevDebugUtils();

  void SetOutputDir(const std::string& dir);
  void SetEnabled(bool enabled);

  template <typename T>
  void SaveTensor(const void* data, const std::vector<T>& shape,
                  const std::string& filename, size_t element_size,
                  cudaStream_t stream);

  nvtype::half* LoadTensor(const std::string& filepath,
                           std::vector<int64_t>& shape, cudaStream_t stream);

  float* LoadTensorFloat(const std::string& filepath,
                         std::vector<int64_t>& shape, cudaStream_t stream);

  void CompareWithPython(const void* gpu_data, const std::string& py_filename,
                         const std::string& tag, size_t element_size,
                         cudaStream_t stream);

  // Compare the 6 TRT head outputs against python dumps by matching each
  // python query to the nearest c++ query in BEV (x,y) pixel space. This
  // avoids the false-positive errors caused by argsort/TopK tie-break
  // differences between mmdet3d and TensorRT when the 200 proposals happen
  // to coincide in position but are stored in a different order.
  //
  // cpp_* pointers are device buffers of nvtype::half with layouts:
  //   reg: [1,2,200]  height: [1,1,200]  dim: [1,3,200]
  //   rot: [1,2,200]  vel:    [1,2,200]  score: [1,15,200]
  void CompareHeadByQueryMatch(const void* cpp_reg, const void* cpp_height,
                               const void* cpp_dim, const void* cpp_rot,
                               const void* cpp_vel, const void* cpp_score,
                               cudaStream_t stream);

  // Compare the decoded final bboxes against Python dumps
  // (py/09_pred_bboxes.bin / py/09_pred_scores.bin / py/09_pred_labels.bin).
  // Matches each C++ box to its nearest Python box of the same label within
  // a small position radius, so score-threshold boundary differences don't
  // pollute the stats.
  void CompareFinalBBoxes(
      const std::vector<bevfusion::head::transbbox::BoundingBox>& cpp_boxes);

  void SaveCameraImages(const unsigned char** images, int num_cameras,
                        int width, int height, const std::string& filename);

  void SaveLidarPoints(const nvtype::half* points, int num_points,
                       int num_features, const std::string& filename,
                       cudaStream_t stream);

  /**
   * @brief Load Python voxel data for debugging comparison
   * @param voxel_features Output device pointer for voxel features
   * @param voxel_coords Output device pointer for voxel coordinates
   * @param num_voxels Output number of voxels
   * @param voxel_dim Output voxel dimension
   * @param stream CUDA stream
   * @return true if loading succeeds
   */
  bool LoadPythonVoxels(void** voxel_features, void** voxel_coords,
                        int64_t& num_voxels, int64_t& voxel_dim,
                        cudaStream_t stream);

  bool IsEnabled() const {
    return enabled_;
  }

 private:
  bool enabled_ = false;
  std::string output_dir_ = "/century/data/bevfusion_debug/";
};

}  // namespace lidar
}  // namespace perception
}  // namespace century
