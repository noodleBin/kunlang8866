#include <string>

#include "cyber/common/log.h"
#include "modules/perception/common/sensor_manager/camera_sensor_config.h"
#include "modules/perception/lidar/lib/detector/bev_detection/bevfusion_detector.h"

namespace {

std::string EnsureTrailingSlash(const std::string& path) {
  if (!path.empty() && '/' == path.back()) {
    return path;
  }
  return path + "/";
}

}  // namespace

int main(int argc, char** argv) {
  const std::string config_dir =
      argc > 1 ? argv[1]
               : "/century/modules/perception/production/conf/perception/lidar";
  const std::string camera_yaml =
      argc > 2 ? argv[2]
               : "/century/modules/perception/data/params/camera_sensor.yaml";
  const std::string dump_dir =
      argc > 3 ? argv[3] : "/century/data/bevfusion_debug";

  auto& camera_sensor_config =
      century::perception::common::CameraSensorConfig::GetInstance();
  if (!camera_sensor_config.IsInitialized() &&
      !camera_sensor_config.Initialize(camera_yaml)) {
    AERROR << "Failed to initialize camera sensor config: " << camera_yaml;
    return 1;
  }

  century::perception::lidar::BevFusionDetector detector;
  century::perception::lidar::LidarDetectorInitOptions init_options;
  init_options.use_camera = true;
  init_options.cfg_file = config_dir;
  if (!detector.Init(init_options)) {
    AERROR << "Failed to init BevFusionDetector with config dir: "
           << config_dir;
    return 1;
  }

  if (!detector.DetectFromDump(EnsureTrailingSlash(dump_dir))) {
    AERROR << "DetectFromDump failed.";
    return 1;
  }

  return 0;
}
