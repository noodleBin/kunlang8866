#pragma once
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "modules/perception/base/object.h"
#include "modules/perception/base/point_cloud.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/lidar/lib/interface/base_lidar_detector.h"
#include "modules/perception/lidar/lib/detector/center_point_detection/src/centerpoint.h"

namespace century {
namespace perception {
namespace lidar {

using Object = base::Object;
using namespace century::perception::lidar::centerpoint;

class CenterPointDetector : public BaseLidarDetector {
 public:
  CenterPointDetector();
  virtual ~CenterPointDetector();

  bool Init(const LidarDetectorInitOptions& options =
                LidarDetectorInitOptions()) override;

  bool Detect(const LidarDetectorOptions& options, LidarFrame* frame) override;

  std::string Name() const override { return "CenterPointDetector"; }

 private:
  // std::shared_ptr<pointpillar::lidar::Core> CreateCore() noexcept;
  void GetObjects(const std::vector<Bndbox>& bboxes,
                  LidarFrame* frame) noexcept;
  base::ObjectSubType GetObjectSubType(const int label);

 private:
  std::vector<float> input_raw_;
  int count = 0;
  int max_points_ = 0;
  int feature_number_ = 0;
  cudaStream_t stream_;
  float *d_points_ = nullptr;  
  std::shared_ptr<base::AttributePointCloud<base::PointF>> original_cloud_;
  std::shared_ptr<CenterPointVoxel> centerpoint_ptr;
  CenterPointDetectionConfig centerpoint_detection_config_;
  
};


}  // namespace lidar
}  // namespace perception
}  // namespace century