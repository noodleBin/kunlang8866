/******************************************************************************
 * Copyright 2018 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <fstream>
#include <iomanip>
#include <memory>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "absl/strings/str_split.h"

#include "cyber/common/file.h"
#include "modules/perception/base/distortion_model.h"
#include "modules/perception/camera/app/obstacle_camera_perception.h"
#include "modules/perception/camera/lib/calibration_service/online_calibration_service/online_calibration_service.h"
#include "modules/perception/camera/lib/calibrator/laneline/laneline_calibrator.h"
#include "modules/perception/camera/lib/feature_extractor/tfe/external_feature_extractor.h"
#include "modules/perception/camera/lib/feature_extractor/tfe/project_feature.h"
#include "modules/perception/camera/lib/feature_extractor/tfe/tracking_feat_extractor.h"
#include "modules/perception/camera/lib/lane/detector/darkSCNN/darkSCNN_lane_detector.h"
#include "modules/perception/camera/lib/lane/detector/denseline/denseline_lane_detector.h"
#include "modules/perception/camera/lib/lane/postprocessor/darkSCNN/darkSCNN_lane_postprocessor.h"
#include "modules/perception/camera/lib/lane/postprocessor/denseline/denseline_lane_postprocessor.h"
#include "modules/perception/camera/lib/obstacle/detector/smoke/smoke_obstacle_detector.h"
#include "modules/perception/camera/lib/obstacle/detector/yolo/yolo_obstacle_detector.h"
#include "modules/perception/camera/lib/obstacle/postprocessor/location_refiner/location_refiner_obstacle_postprocessor.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/omt_obstacle_tracker.h"
#include "modules/perception/camera/lib/obstacle/transformer/multicue/multicue_obstacle_transformer.h"
#include "modules/perception/camera/tools/offline/transform_server.h"
#include "modules/perception/camera/tools/offline/visualizer.h"
#include "modules/perception/common/io/io_util.h"

DEFINE_string(test_list,
              "/century/modules/perception/testdata/camera/lib/obstacle/"
              "detector/yolo/img/full_test_list.txt",
              "test image list");
DEFINE_string(image_root,
              "/century/modules/perception/testdata/camera/lib/obstacle/"
              "detector/yolo/img/",
              "root directory of images");
DEFINE_string(image_ext, ".jpg", "extension of image name");
DEFINE_string(image_color, "bgr", "color space of image");
DEFINE_string(config_root,
              "/century/modules/perception/production/conf/perception/camera/",
              "config_root");
DEFINE_string(tf_file, "", "tf file");
DEFINE_string(config_file, "obstacle.pt", "config_file");
DEFINE_string(base_camera_name, "front_6mm", "camera to be projected");
DEFINE_string(sensor_name, "front_6mm,front_12mm", "camera to use");
DEFINE_string(params_dir, "/century/modules/perception/data/params",
              "params directory");
DEFINE_string(visualize_dir, "/tmp/0000", "visualize directory");
DEFINE_double(camera_fps, 15, "camera_fps");
DEFINE_bool(do_undistortion, false, "do_undistortion");
DEFINE_string(undistortion_save_dir, "./undistortion_result",
              "Directory to save undistored images.");
DEFINE_string(save_dir, "./result",
              "Directory to save result images with detections.");

namespace century {
namespace perception {
namespace camera {

using StringMapF = std::map<std::string, float>;
using StringMap3f = std::map<std::string, Eigen::Matrix3f>;
using ProviderMap = std::map<std::string, DataProvider*>;
using CameraFrameList = std::vector<CameraFrame>;
using ProviderList = std::vector<std::unique_ptr<DataProvider>>;

REGISTER_OBSTACLE_DETECTOR(YoloObstacleDetector);
REGISTER_OBSTACLE_DETECTOR(SmokeObstacleDetector);
REGISTER_OBSTACLE_TRACKER(OMTObstacleTracker);
REGISTER_FEATURE_EXTRACTOR(TrackingFeatureExtractor);
REGISTER_OBSTACLE_TRANSFORMER(MultiCueObstacleTransformer);
REGISTER_OBSTACLE_POSTPROCESSOR(LocationRefinerObstaclePostprocessor);
REGISTER_FEATURE_EXTRACTOR(ProjectFeature);
REGISTER_FEATURE_EXTRACTOR(ExternalFeatureExtractor);
REGISTER_LANE_POSTPROCESSOR(DenselineLanePostprocessor);
REGISTER_LANE_DETECTOR(DenselineLaneDetector);
REGISTER_CALIBRATOR(LaneLineCalibrator);
REGISTER_CALIBRATION_SERVICE(OnlineCalibrationService);
REGISTER_LANE_DETECTOR(DarkSCNNLaneDetector);
REGISTER_LANE_POSTPROCESSOR(DarkSCNNLanePostprocessor);

static constexpr float kDefaultPitchAngle = 0.0f;
static constexpr float kDefaultCameraHeight = 1.5f;
static constexpr int kFrameCapacity = 20;
static constexpr float kMaxPitchAngleDeg = 10.0f;
static constexpr float kMaxPitchAngleRad = kMaxPitchAngleDeg * M_PI / 180.0f;
static constexpr int kImageHeight = 1080;
static constexpr int kImageWidth = 1920;
static constexpr int kDefaultDeviceId = 0;
static constexpr int kMinTimestampThreshold = 1e-3;
static constexpr double kNanosecondsPerSecond = 1e-9;

void save_image(const std::string& path, base::Image8U& image) {  // NOLINT
  AINFO << path;
  int cv_type = base::Color::GRAY == image.type() ? CV_8UC1 : CV_8UC3;
  cv::Mat cv_img(image.rows(), image.cols(), cv_type, image.mutable_cpu_data(),
                 image.width_step());
  cv::imwrite(path, cv_img);
}

bool InitPerceptionPipeline(ObstacleCameraPerception* perception,
                            CameraPerceptionInitOptions* init_options) {
  AINFO << "config_root: " << FLAGS_config_root;
  init_options->root_dir = FLAGS_config_root;
  AINFO << "config_file: " << FLAGS_config_file;
  init_options->conf_file = FLAGS_config_file;
  init_options->lane_calibration_working_sensor_name = FLAGS_base_camera_name;
  init_options->use_cyber_work_root = true;
  return perception->Init(*init_options);
}

CameraFrameList InitFrameList() {
  CameraFrameList frames(kFrameCapacity);
  for (int i = 0; i < kFrameCapacity; ++i) {
    frames[i].track_feature_blob = std::make_shared<base::Blob<float>>();
  }
  return frames;
}

ProviderMap InitDataProviders(const std::vector<std::string>& camera_names,
                              ProviderList* providers) {
  DataProvider::InitOptions options;
  options.image_height = kImageHeight;
  options.image_width = kImageWidth;
  options.do_undistortion = FLAGS_do_undistortion;
  options.device_id = kDefaultDeviceId;

  ProviderMap provider_map;
  providers->clear();
  providers->reserve(camera_names.size());

  for (size_t i = 0; i < camera_names.size(); ++i) {
    options.sensor_name = camera_names[i];
    providers->emplace_back(std::make_unique<DataProvider>());
    DataProvider* provider = (*providers)[i].get();
    ACHECK(provider->Init(options));
    provider_map[camera_names[i]] = provider;
    AINFO << "Init data_provider for " << camera_names[i];
  }
  return provider_map;
}

StringMap3f InitIntrinsicParams(const std::vector<std::string>& camera_names) {
  StringMap3f intrinsic_map;
  auto* manager = common::SensorManager::Instance();

  for (const auto& name : camera_names) {
    auto model = manager->GetUndistortCameraModel(name);
    auto* pinhole = dynamic_cast<base::PinholeCameraModel*>(model.get());
    intrinsic_map[name] = pinhole->get_intrinsic_params();
    AINFO << "#intrinsics of " << name << ": " << intrinsic_map[name];
  }
  return intrinsic_map;
}

bool SetupCalibrationService(const std::vector<std::string>& camera_names,
                             TransformServer& transform_server,
                             StringMap3f& intrinsic_map,
                             ObstacleCameraPerception* perception) {
  StringMapF ground_height_map;
  StringMapF pitch_angle_diff_map;

  for (const auto& name : camera_names) {
    Eigen::Affine3d c2g;
    if (!transform_server.QueryTransform(name, "ground", &c2g)) {
      AINFO << "Failed to query transform from " << name << " to ground";
      return false;
    }

    float ground_height = static_cast<float>(c2g.translation()[2]);
    AINFO << name << " height: " << ground_height;
    ground_height_map[name] = ground_height;

    float pitch_diff = 0.0f;
    if (name == FLAGS_base_camera_name) {
      pitch_diff = 0.0f;
    } else {
      Eigen::Affine3d trans;
      if (!transform_server.QueryTransform(name, FLAGS_base_camera_name,
                                           &trans)) {
        AINFO << "Failed to query transform";
        return false;
      }
      Eigen::Vector3d euler = trans.linear().eulerAngles(0, 1, 2);
      pitch_diff = static_cast<float>(euler(0));
      if (kMaxPitchAngleRad < pitch_diff) {
        pitch_diff = 0.0f;
      }
    }
    pitch_angle_diff_map[name] = pitch_diff;
  }

  perception->SetCameraHeightAndPitch(ground_height_map, pitch_angle_diff_map,
                                      kDefaultPitchAngle);
  return true;
}

cv::Mat LoadAndPreprocessImage(const std::string& image_path,
                               const std::string& color_space) {
  cv::Mat image;
  if ("gray" == color_space) {
    image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    cv::cvtColor(image, image, CV_GRAY2RGB);
  } else if ("rgb" == color_space) {
    image = cv::imread(image_path, cv::IMREAD_COLOR);
    cv::cvtColor(image, image, CV_BGR2RGB);
  } else if ("bgr" == color_space) {
    image = cv::imread(image_path, cv::IMREAD_COLOR);
  } else {
    AERROR << "Invalid color: " << color_space;
  }
  return image;
}

bool SetupCameraFrame(CameraFrame* frame, int frame_id,
                      const std::string& image_name,
                      const std::string& camera_name,
                      StringMap3f& intrinsic_map,
                      TransformServer& transform_server,
                      ProviderMap& provider_map) {
  frame->frame_id = frame_id;
  std::stringstream ss(image_name);
  frame->timestamp = 0.0;
  ss >> frame->timestamp;
  frame->timestamp *= kNanosecondsPerSecond;
  if (kMinTimestampThreshold > frame->timestamp) {
    frame->timestamp = 1.0 / FLAGS_camera_fps * frame_id;
  }
  AINFO << "Timestamp: " << std::fixed << std::setprecision(10)
        << frame->timestamp;

  if (camera_name == FLAGS_base_camera_name) {
    frame->project_matrix = Eigen::Matrix3d::Identity();
  } else {
    Eigen::Affine3d trans;
    if (!transform_server.QueryTransform(camera_name, FLAGS_base_camera_name,
                                         &trans)) {
      AINFO << "Failed to query transform from " << camera_name << " to "
            << FLAGS_base_camera_name;
      return false;
    }
    frame->project_matrix =
        intrinsic_map[FLAGS_base_camera_name].cast<double>() * trans.linear() *
        intrinsic_map[camera_name].cast<double>().inverse();
  }
  AINFO << "Project Matrix: \n" << frame->project_matrix;

  frame->data_provider = provider_map[camera_name];

  Eigen::Affine3d pose;
  if (!transform_server.QueryPos(frame->timestamp, &pose)) {
    pose.setIdentity();
  }

  Eigen::Affine3d c2n;
  if (!transform_server.QueryTransform(camera_name, "novatel", &c2n)) {
    AINFO << "Failed to query transform from " << camera_name << " to novatel";
    return false;
  }
  frame->camera2world_pose = pose * c2n;

  return true;
}

void SaveUndistortedImage(const std::string& save_dir,
                          const std::string& image_name,
                          const CameraFrame& frame) {
  if (!FLAGS_do_undistortion || FLAGS_undistortion_save_dir.empty() ||
      !cyber::common::PathExists(save_dir)) {
    return;
  }

  base::Image8U image;
  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::BGR;
  frame.data_provider->GetImage(image_options, &image);
  save_image(save_dir + "/" + image_name + FLAGS_image_ext, image);
  AINFO << "Undistorted image saved to : "
        << save_dir + "/" + image_name + FLAGS_image_ext;
}

void SaveDetectionResults(const std::string& save_dir,
                          const std::string& image_name, cv::Mat* image,
                          const CameraFrame& frame) {
  if (!cyber::common::PathExists(save_dir)) {
    AERROR << "save_dir does not exist : " << save_dir;
    return;
  }

  std::ofstream myfile;
  myfile.open(save_dir + "/" + image_name + ".txt");
  std::stringstream text;
  std::map<base::ObjectSubType, std::string>& sub_type_map =
      const_cast<std::map<base::ObjectSubType, std::string>&>(
          base::kSubType2NameMap);
  for (const auto& obj : frame.detected_objects) {
    const auto& box = obj->camera_supplement.box;
    myfile << sub_type_map[obj->sub_type] << " " << static_cast<int>(box.xmin)
           << " " << static_cast<int>(box.ymin) << " "
           << static_cast<int>(box.xmax) << " " << static_cast<int>(box.ymax)
           << " " << obj->sub_type_probs[static_cast<int>(obj->sub_type)]
           << "\n";

    cv::rectangle(
        *image,
        cv::Point(static_cast<int>(box.xmin), static_cast<int>(box.ymin)),
        cv::Point(static_cast<int>(box.xmax), static_cast<int>(box.ymax)),
        cv::Scalar(0, 255, 0), 2);

    text.str("");
    text.clear();
    text << sub_type_map[obj->sub_type] << " - "
         << obj->sub_type_probs[static_cast<int>(obj->sub_type)];
    cv::putText(
        *image, text.str(),
        cv::Point(static_cast<int>(box.xmin), static_cast<int>(box.ymin)),
        cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 127, 0), 2);
  }
  cv::imwrite(save_dir + "/" + image_name + FLAGS_image_ext, *image);
  AINFO << "Result saved to : "
        << save_dir + "/" + image_name + FLAGS_image_ext;
}

int work() {
  ObstacleCameraPerception perception;
  CameraPerceptionInitOptions init_option;
  CameraPerceptionOptions options;
  if (!InitPerceptionPipeline(&perception, &init_option)) {
    AERROR << "Failed to initialize perception pipeline";
    return -1;
  }

  std::vector<CameraFrame> frame_list = InitFrameList();

  int frame_id = -1;
  std::ifstream fin(FLAGS_test_list, std::ifstream::in);
  if (!fin.is_open()) {
    AERROR << "Cannot open image list: " << FLAGS_test_list;
    return -1;
  }

  const std::vector<std::string> camera_names =
      absl::StrSplit(FLAGS_sensor_name, ',');

  ProviderList providers;
  ProviderMap provider_map = InitDataProviders(camera_names, &providers);
  StringMap3f intrinsic_map = InitIntrinsicParams(camera_names);

  TransformServer transform_server;
  ACHECK(transform_server.Init(camera_names, FLAGS_params_dir));
  transform_server.print();

  if (!FLAGS_tf_file.empty()) {
    ACHECK(transform_server.LoadFromFile(FLAGS_tf_file));
  }

  if (!SetupCalibrationService(camera_names, transform_server, intrinsic_map,
                               &perception)) {
    AERROR << "Failed to setup calibration service";
    return -1;
  }

  Visualizer visualize;
  ACHECK(visualize.Init(camera_names, &transform_server));
  visualize.SetDirectory(FLAGS_visualize_dir);

  std::string line;
  while (fin >> line) {
    const std::vector<std::string> temp_strs = absl::StrSplit(line, '/');
    if (temp_strs.size() != 2) {
      AERROR << "Invalid format in " << FLAGS_test_list;
      continue;
    }

    const std::string camera_name = temp_strs[0];
    const std::string image_name = temp_strs[1];

    AINFO << "image: " << image_name << ", camera_name: " << camera_name;

    const std::string image_path =
        FLAGS_image_root + image_name + FLAGS_image_ext;
    cv::Mat image = LoadAndPreprocessImage(image_path, FLAGS_image_color);
    if (nullptr == image.data) {
      AERROR << "Cannot read image: " << image_path;
      return -1;
    }

    frame_id++;
    CameraFrame& frame = frame_list[frame_id % kFrameCapacity];
    if (!SetupCameraFrame(&frame, frame_id, image_name, camera_name,
                          intrinsic_map, transform_server, provider_map)) {
      AERROR << "Failed to setup camera frame";
      return -1;
    }

    frame.data_provider->FillImageData(image.rows, image.cols,
                                       (const uint8_t*)(image.data), "bgr8");
    perception.GetCalibrationService(&frame.calibration_service);

    SaveUndistortedImage(FLAGS_undistortion_save_dir, image_name, frame);

    ACHECK(perception.Perception(options, &frame));
    visualize.ShowResult(image, frame);

    SaveDetectionResults(FLAGS_save_dir, image_name, &image, frame);
  }

  return 0;
}

}  // namespace camera
}  // namespace perception
}  // namespace century

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::SetUsageMessage(
      "command line brew\n"
      "Usage: camera_benchmark <args>\n");

  return century::perception::camera::work();
}
