#include "modules/perception/camera/lib/traffic_light/detect_yolo/yolo_traffic_light_detector.h"

#include "absl/strings/str_cat.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera/common/util.h"
#include "opencv2/opencv.hpp"

namespace century {
namespace perception {
namespace camera {

using cyber::common::GetAbsolutePath;

bool YoloTrafficLightDetector::Init(
    const TrafficLightDetectorInitOptions &options) {
  const std::string proto_path =
      GetAbsolutePath(options.root_dir, options.conf_file);
  if (!cyber::common::GetProtoFromFile(proto_path, &detection_param_)) {
    AERROR << "Load proto param failed, file: " << proto_path;
    return false;
  }

  const std::string engine_file =
      GetAbsolutePath(options.root_dir, detection_param_.model_name());
  AINFO << "Loading YOLO Engine from: " << engine_file;

  InferOption infer_option;
  infer_option.enableSwapRB();
  infer_option.setInputDimensions(1920, 1080);

  gpu_id_ = options.gpu_id;
  AINFO << "Using GPU ID: " << gpu_id_;

  cudaSetDevice(gpu_id_);

  try {
    model_ = std::make_unique<DetectModel>(engine_file, infer_option);
  } catch (const std::exception &e) {
    AERROR << "Failed to init YOLO model: " << e.what();
    return false;
  }

  image_options_.target_color = base::Color::BGR;
  image_options_.do_crop = false;

  image_buffer_.reset(new base::Image8U(1080, 1920, base::Color::BGR));
  AINFO << "YoloTrafficLightDetector Init Success.";
  return true;
}

bool YoloTrafficLightDetector::Detect(const TrafficLightDetectorOptions &options,
                                      CameraFrame *frame) {
  if (nullptr == frame || nullptr == frame->data_provider) {
    AERROR << "Input frame is null";
    return false;
  }

  if (!frame->data_provider->GetImage(image_options_, image_buffer_.get())) {
    AERROR << "GetImage failed";
    return false;
  }

  {
    cv::Mat debug_image(image_buffer_->rows(), image_buffer_->cols(), CV_8UC3,
                        const_cast<uint8_t *>(image_buffer_->cpu_data()));
    cv::imwrite(absl::StrCat(
                    "/century/modules/perception/camera/lib/traffic_light/"
                    "detect_yolo/yolo_img_input/",
                    std::to_string(frame->timestamp), ".jpg"),
                debug_image);
  }

  const int width = image_buffer_->cols();
  const int height = image_buffer_->rows();
  const int channels = image_buffer_->channels();
  const int step = image_buffer_->width_step();
  const int expected_step = width * channels;

  AINFO << "Image Debug Info: "
        << " W=" << width
        << " H=" << height
        << " C=" << channels
        << " Step=" << step
        << " Expected=" << expected_step;

  if (step != expected_step) {
    AERROR << "Image memory is not continuous, padding bytes per row: "
           << step - expected_step;
  }

  const Image img(image_buffer_->cpu_data(), image_buffer_->cols(),
                  image_buffer_->rows());

  DetectRes result;
  try {
    result = model_->predict(img);
  } catch (const std::exception &e) {
    AERROR << "Inference failed: " << e.what();
    return false;
  }

  AINFO << "YOLO detected " << result.num << " lights.";

  for (int i = 0; i < result.num; ++i) {
    const auto &box = result.boxes[i];
    const int cls = result.classes[i];
    const float score = result.scores[i];

    base::TrafficLightPtr light(new base::TrafficLight);
    light->region.detection_roi.x = box.left;
    light->region.detection_roi.y = box.top;
    light->region.detection_roi.width = box.right - box.left;
    light->region.detection_roi.height = box.bottom - box.top;

    light->region.detect_score = score;
    light->region.is_detected = true;

    century::perception::base::TLColor color =
        century::perception::base::TLColor::TL_UNKNOWN_COLOR;

    switch (cls) {
    case 0:
        color = century::perception::base::TLColor::TL_RED;
        break;
    case 1:
        color = century::perception::base::TLColor::TL_GREEN;
        break;
    case 2:
        color = century::perception::base::TLColor::TL_RED;
        break;
    case 3:
        color = century::perception::base::TLColor::TL_GREEN;
        break;
    case 4:
        color = century::perception::base::TLColor::TL_RED;
        break;
    case 5:
        color = century::perception::base::TLColor::TL_GREEN;
        break;
    case 6:
        color = century::perception::base::TLColor::TL_RED;
        break;
    case 7:
        color = century::perception::base::TLColor::TL_GREEN;
        break;
    default:
        color = century::perception::base::TLColor::TL_UNKNOWN_COLOR;
        break;
    }

    light->status.color = color;
    light->region.debug_roi.emplace_back(light->region.detection_roi);
    light->region.debug_roi_detect_scores.emplace_back(score);

    frame->traffic_lights.emplace_back(light);
  }

  return true;
}

std::string YoloTrafficLightDetector::Name() const {
  return "YoloTrafficLightDetector";
}

REGISTER_TRAFFIC_LIGHT_DETECTOR(YoloTrafficLightDetector);

}
}
}
