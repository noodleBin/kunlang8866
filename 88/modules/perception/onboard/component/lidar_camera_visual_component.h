/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/perception/onboard/proto/visualizer_component_config.pb.h"
#include "modules/perception/proto/perception_obstacle_debug.pb.h"

enum class ImageType { COMPRESSED = 0, RAW = 1 };

#include "cyber/cyber.h"
#include "modules/perception/common/sensor_manager/camera_sensor_config.h"
#include "modules/perception/common/visualization/lidar_detector_lite_viz.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/lidar/lib/map_manager/map_manager.h"

#if defined(__aarch64__)
#include "modules/perception/camera/lib/nvjpeg_decoder/nvjpeg_opencv.h"
#endif

namespace century {
namespace perception {
namespace onboard {

struct CameraData {
  std::string name;
  std::string topic;
  cv::Mat image;
  std::mutex mutex;
  double timestamp = 0.0;
  Eigen::Matrix4f lidar2image = Eigen::Matrix4f::Identity();
  bool has_calibration = false;

  // Intrinsics for undistortion: K_std (before undistortion) + distortion
  // coeffs
  cv::Mat camera_matrix;  // K_std from intrinsics_8
  cv::Mat dist_coeffs;    // D from intrinsics_8
  cv::Mat map1, map2;
  bool has_undistort_map = false;

  ImageType image_type = ImageType::COMPRESSED;

  int width = 1920;
  int height = 1536;

  std::atomic<bool> updated{false};

  CameraData() = default;
  CameraData(CameraData&& other) noexcept
      : name(std::move(other.name)),
        topic(std::move(other.topic)),
        image(std::move(other.image)),
        timestamp(other.timestamp),
        lidar2image(other.lidar2image),
        has_calibration(other.has_calibration),
        camera_matrix(std::move(other.camera_matrix)),
        dist_coeffs(std::move(other.dist_coeffs)),
        map1(std::move(other.map1)),
        map2(std::move(other.map2)),
        has_undistort_map(other.has_undistort_map),
        width(other.width),
        height(other.height) {
  }
  CameraData& operator=(CameraData&& other) noexcept {
    if (this != &other) {
      name = std::move(other.name);
      topic = std::move(other.topic);
      image = std::move(other.image);
      timestamp = other.timestamp;
      lidar2image = other.lidar2image;
      has_calibration = other.has_calibration;
      camera_matrix = std::move(other.camera_matrix);
      dist_coeffs = std::move(other.dist_coeffs);
      map1 = std::move(other.map1);
      map2 = std::move(other.map2);
      has_undistort_map = other.has_undistort_map;
      width = other.width;
      height = other.height;
    }
    return *this;
  }
};

struct ProjectedObstacle {
  std::array<cv::Point2f, 8> corners_2d;
  std::array<bool, 8> corner_visible;
  int visible_count;
  base::ObjectType type;
  float confidence;
  int track_id;
  cv::Rect2f bbox2d;
};

class LidarCameraVisualComponent
    : public cyber::Component<PerceptionObstacleDebugMsg> {
 public:
  LidarCameraVisualComponent();
  virtual ~LidarCameraVisualComponent();

  bool Init() override;
  bool Proc(
      const std::shared_ptr<PerceptionObstacleDebugMsg>& message) override;

 private:
  bool InitCameras();
  bool LoadCameraCalibration();

  void OnCameraImage(int camera_idx,
                     const std::shared_ptr<drivers::CompressedImage>& image);
  void OnCameraImageRaw(int camera_idx,
                        const std::shared_ptr<drivers::Image>& image);

  void RenderCameraLoop();
  void RenderCameraGrid();

  void DrawObstaclesOnCamera(cv::Mat& image, int camera_idx,
                             const std::vector<base::ObjectPtr>& objects);
  std::vector<ProjectedObstacle> ProjectObstaclesToCamera(
      const std::vector<base::ObjectPtr>& objects, int camera_idx);
  bool ProjectLidarPointToCamera(const Eigen::Vector3d& point_lidar,
                                 const Eigen::Matrix4f& lidar2image,
                                 int image_width, int image_height,
                                 cv::Point2f& image_point);

  void DrawBboxOnImage(cv::Mat& image, const ProjectedObstacle& obstacle);
  void DrawCameraLabel(cv::Mat& image, const std::string& label);
  void DrawFPS(cv::Mat& image);
  void DrawNoSignal(cv::Mat& canvas, int row, int col, const std::string& name);
  void DrawGridLines(cv::Mat& canvas);
  cv::Scalar GetColorForObjectType(base::ObjectType type);
  std::string ObjectTypeToString(base::ObjectType type);

  void ConvertPbToObject(const PerceptionObstacle& pb_msg,
                         const base::ObjectPtr& obj);
  std::vector<Eigen::Vector3d> GetObjectCorners(const base::ObjectPtr& obj);
  void HandleKeyboard(int key);

 private:
  std::vector<CameraData> cameras_;
  int num_cameras_ = 0;

#if defined(__aarch64__)
  std::vector<std::unique_ptr<camera::NvJpegDecoder>> jpeg_decoders_;
#endif

  std::unique_ptr<common::visualizer::PangolinViewer> pangolin_viewer_;

  std::thread render_thread_;
  std::thread pangolin_thread_;
  std::atomic<bool> running_{false};

  void RunPangolinViewer();

  std::shared_ptr<PerceptionObstacleDebugMsg> latest_perception_;
  std::mutex perception_mutex_;
  Eigen::Affine3d latest_lidar2world_;
  bool has_perception_data_ = false;

  lidar::MapManager map_manager_;

  std::atomic<bool> show_images_{true};
  std::atomic<bool> show_obstacle_projection_{true};
  std::atomic<bool> show_fps_{true};

  static constexpr int kImageWidth = 640;
  static constexpr int kImageHeight = 256;
  int kGridRows = 2;
  int kGridCols = 3;
  int kWindowWidth = 1920;
  int kWindowHeight = 1024;

  bool calibration_loaded_ = false;

  bool use_map_manager_{true};
  bool use_viz_pcl_{true};
  bool use_viz_pangolin_{false};
  bool seg_viz_{true};
  bool raw_viz_{true};
  bool aeb_dis_viz_{false};
  bool grid_viz_{false};
  float vehicle_length_{0.0f};
  float vehicle_width_{0.0f};
  float vehicle_height_{0.0f};
};

CYBER_REGISTER_COMPONENT(LidarCameraVisualComponent);

}  // namespace onboard
}  // namespace perception
}  // namespace century
