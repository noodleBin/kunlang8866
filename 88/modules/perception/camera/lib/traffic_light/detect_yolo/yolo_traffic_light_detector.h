#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/perception/base/image_8u.h"
#include "modules/perception/camera/lib/interface/base_traffic_light_detector.h"
#include "modules/perception/camera/lib/traffic_light/proto/detection.pb.h"

#include "modules/perception/camera/lib/traffic_light/detect_yolo/model.hpp"
#include "modules/perception/camera/lib/traffic_light/detect_yolo/option.hpp"
#include "modules/perception/camera/lib/traffic_light/detect_yolo/result.hpp"

namespace century {
namespace perception {
namespace camera {

class YoloTrafficLightDetector : public BaseTrafficLightDetector {
 public:
  YoloTrafficLightDetector() = default;
  ~YoloTrafficLightDetector() = default;
  bool Init(const TrafficLightDetectorInitOptions &options) override;
  bool Detect(const TrafficLightDetectorOptions &options,
              CameraFrame *frame) override;
  std::string Name() const override;
  YoloTrafficLightDetector(const YoloTrafficLightDetector &) = delete;
  YoloTrafficLightDetector &operator=(const YoloTrafficLightDetector &) = delete;
 private:
  std::unique_ptr<DetectModel> model_ = nullptr;
  traffic_light::detection::DetectionParam detection_param_;
  std::shared_ptr<base::Image8U> image_buffer_ = nullptr;
  DataProvider::ImageOptions image_options_;
  int gpu_id_ = 0;
};

}
}
}
