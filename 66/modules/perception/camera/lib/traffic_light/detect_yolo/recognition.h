#pragma once

#include "modules/perception/camera/lib/interface/base_traffic_light_detector.h"
#include "modules/perception/base/traffic_light.h"

namespace century {
namespace perception {
namespace camera {

class TrafficLightRecognition : public BaseTrafficLightDetector {
 public:
  TrafficLightRecognition() = default;
  ~TrafficLightRecognition() = default;

  bool Init(const TrafficLightDetectorInitOptions& options) override;

  bool Detect(const TrafficLightDetectorOptions& options,
              CameraFrame* frame) override;

  std::string Name() const override;

 private:
  base::TLColor MapYoloClassToColor(int class_id);
};

}
}
}