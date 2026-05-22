#include "modules/perception/camera/lib/traffic_light/detect_yolo/recognition.h"

#include "cyber/common/log.h"

namespace century {
namespace perception {
namespace camera {

bool TrafficLightRecognition::Init(
    const TrafficLightDetectorInitOptions& options) {
  AINFO << "TrafficLightRecognition init success (YOLO passthrough mode).";
  return true;
}

bool TrafficLightRecognition::Detect(const TrafficLightDetectorOptions& options,
                                     CameraFrame* frame) {
  if (nullptr == frame) {
    return false;
  }

  for (base::TrafficLightPtr light : frame->traffic_lights) {
    if (!light->region.is_detected) {
      light->status.color = base::TLColor::TL_UNKNOWN_COLOR;
      light->status.confidence = 0.0;
      continue;
    }

    light->status.confidence = light->region.detect_score;
  }

  return true;
}

base::TLColor TrafficLightRecognition::MapYoloClassToColor(int class_id) {
  switch (class_id) {
    case 0:
      return base::TLColor::TL_RED;
    case 1:
      return base::TLColor::TL_YELLOW;
    case 2:
      return base::TLColor::TL_GREEN;
    case 3:
      return base::TLColor::TL_BLACK;
    default:
      return base::TLColor::TL_UNKNOWN_COLOR;
  }
}

std::string TrafficLightRecognition::Name() const {
  return "TrafficLightRecognition";
}

REGISTER_TRAFFIC_LIGHT_DETECTOR(TrafficLightRecognition);

}
}
}
