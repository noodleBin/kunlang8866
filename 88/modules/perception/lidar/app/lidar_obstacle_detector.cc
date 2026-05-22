/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/lidar/app/lidar_obstacle_detector.h"

#include <chrono>

#include "modules/perception/lidar/app/proto/lidar_obstacle_detection_config.pb.h"

#include "cyber/common/file.h"
#include "modules/common/util/perf_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/lib/scene_manager/scene_manager.h"

namespace century {
namespace perception {
namespace lidar {

bool LidarObstacleDetector::Init(
    const LidarObstacleDetectionInitOptions& options) {
  auto& sensor_name = options.sensor_name;
  AINFO << "*****************Debug LidarObstacleDetector::Init, "
            "options.sensor_name: "
         << options.sensor_name;
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(config_manager->GetModelConfig(Name(), &model_config));

  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path));
  config_file = cyber::common::GetAbsolutePath(work_root, root_path);
  config_file = cyber::common::GetAbsolutePath(config_file, sensor_name);
  config_file = cyber::common::GetAbsolutePath(config_file,
                                               "lidar_obstacle_detection.conf");

  LidarObstacleDetectionConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
  use_map_manager_ = config.use_map_manager();
  use_object_filter_bank_ = config.use_object_filter_bank();
  use_object_builder_ = ("PointPillarsDetector" != config.detector());

  AINFO << "*****************Debug LidarObstacleDetector::Init, "
            "options.sensor_name: "
         << options.sensor_name << ", config.detector(): " << config.detector();

  use_map_manager_ = use_map_manager_ && options.enable_hdmap_input;

  SceneManagerInitOptions scene_manager_init_options;
  ACHECK(SceneManager::Instance().Init(scene_manager_init_options));

  if (use_map_manager_) {
    MapManagerInitOptions map_manager_init_options;
    if (!map_manager_.Init(map_manager_init_options)) {
      AINFO << "Failed to init map manager.";
      use_map_manager_ = false;
    }
  }

  BaseLidarDetector* detector =
      BaseLidarDetectorRegisterer::GetInstanceByName(config.detector());
  CHECK_NOTNULL(detector);
  detector_.reset(detector);
  LidarDetectorInitOptions detection_init_options;
  detection_init_options.sensor_name = sensor_name;
  detection_init_options.use_camera = options.use_camera;
  ACHECK(detector_->Init(detection_init_options))
      << "lidar detector init error";

  return true;
}

LidarProcessResult LidarObstacleDetector::Process(
    const LidarObstacleDetectionOptions& options,
    const std::shared_ptr<century::drivers::PointCloud const>& message,
    LidarFrame* frame) {
  LidarDetectorOptions detection_options;
  if (!detector_->Detect(detection_options, frame)) {
    return LidarProcessResult(LidarErrorCode::DetectionError,
                              "Failed to detect.");
  }

  return LidarProcessResult(LidarErrorCode::PointCloudPreprocessorError,
                            "Failed to preprocess point cloud.");
}

LidarProcessResult LidarObstacleDetector::Process(
    const LidarObstacleDetectionOptions& options, LidarFrame* frame) {
  LidarDetectorOptions detection_options;
  auto start = std::chrono::system_clock::now();
  auto ret = detector_->Detect(detection_options, frame);
  auto end = std::chrono::system_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  AINFO << "***********DNN inference time: " << duration.count();

  if (!ret) {
    return LidarProcessResult(LidarErrorCode::DetectionError,
                              "Failed to detect.");
  }

  return LidarProcessResult(LidarErrorCode::PointCloudPreprocessorError,
                            "Failed to preprocess point cloud.");
}

PERCEPTION_REGISTER_LIDAROBSTACLEDETECTION(LidarObstacleDetector);

}  // namespace lidar
}  // namespace perception
}  // namespace century
