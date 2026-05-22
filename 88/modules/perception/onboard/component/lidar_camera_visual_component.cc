/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
 *****************************************************************************/

#include "modules/perception/onboard/component/lidar_camera_visual_component.h"

#include <chrono>
#include <iomanip>
#include <sstream>

#include <pangolin/pangolin.h>

#include "cyber/time/clock.h"

namespace century {
namespace perception {
namespace onboard {

using ::century::cyber::Clock;
using common::CameraParms;
using common::CameraSensorConfig;

LidarCameraVisualComponent::LidarCameraVisualComponent() = default;

LidarCameraVisualComponent::~LidarCameraVisualComponent() {
  running_ = false;

  if (pangolin_viewer_) {
    pangolin::Quit();
  }

  if (render_thread_.joinable()) {
    render_thread_.join();
  }
  if (pangolin_thread_.joinable()) {
    pangolin_thread_.join();
  }
}

bool LidarCameraVisualComponent::Init() {
  if (!InitCameras()) {
    AERROR << "Failed to initialize cameras";
    return false;
  }

  if (!LoadCameraCalibration()) {
    AERROR << "Failed to load camera calibration, obstacle projection disabled";
    calibration_loaded_ = false;
  } else {
    calibration_loaded_ = true;
  }

  if (use_map_manager_) {
    lidar::MapManagerInitOptions map_manager_init_options;
    if (!map_manager_.Init(map_manager_init_options)) {
      AERROR << "Failed to init map manager.";
      use_map_manager_ = false;
    }
  }

  VisualizerComponentConfig visualizer_config;
  if (!GetProtoConfig(&visualizer_config)) {
    std::cout << "Failed to load visualizer component config file: "
              << ConfigFilePath() << std::endl;
  } else {
    use_viz_pcl_ = visualizer_config.use_viz_pcl();
    use_viz_pangolin_ = visualizer_config.use_viz_pangolin();
    seg_viz_ = visualizer_config.seg_viz();
    raw_viz_ = visualizer_config.raw_viz();
    aeb_dis_viz_ = visualizer_config.aeb_dis_viz();
    grid_viz_ = visualizer_config.grid_viz();
    vehicle_length_ = static_cast<float>(visualizer_config.vehicle_length());
    vehicle_width_ = static_cast<float>(visualizer_config.vehicle_width());
    vehicle_height_ = static_cast<float>(visualizer_config.vehicle_height());
  }

  pangolin_thread_ =
      std::thread(&LidarCameraVisualComponent::RunPangolinViewer, this);

  running_ = true;
  render_thread_ =
      std::thread(&LidarCameraVisualComponent::RenderCameraLoop, this);

  AINFO << "LidarCameraVisualComponent initialized successfully";
  return true;
}

void LidarCameraVisualComponent::RunPangolinViewer() {
  pangolin_viewer_ = std::make_unique<common::visualizer::PangolinViewer>(
      1280, 720, false, false, vehicle_length_, vehicle_width_,
      vehicle_height_);
  pangolin_viewer_->Spin();
}

bool LidarCameraVisualComponent::InitCameras() {
  auto& sensor_config = CameraSensorConfig::GetInstance();
  sensor_config.Initialize(
      "/century/modules/perception/data/params/camera_sensor.yaml");
  if (!sensor_config.IsInitialized()) {
    AERROR << "CameraSensorConfig not initialized";
    return false;
  }

  auto camera_names = sensor_config.GetCameraNames();
  if (camera_names.empty()) {
    AERROR << "No cameras configured in camera_sensor.yaml";
    return false;
  }

  cameras_.clear();

  for (const auto& name : camera_names) {
    CameraParms params;
    if (!sensor_config.GetCameraConfig(name, params)) {
      AWARN << "Failed to get config for camera: " << name;
      continue;
    }

    CameraData camera;
    camera.name = name;
    sensor_config.GetCameraChannel(name, camera.topic);
    camera.width = params.width;
    camera.height = params.height;

    // Load intrinsics_8: [fx, fy, cx, cy, k1, k2, k3, k4] (K_std + distortion)
    // Used only for fisheye undistortion map initialization
    auto intrinsics_vec = sensor_config.GetCameraIntrinsicsVector(name);
    if (intrinsics_vec.size() >= 8) {
      double fx = intrinsics_vec[0];
      double fy = intrinsics_vec[1];
      double cx = intrinsics_vec[2];
      double cy = intrinsics_vec[3];
      double k1 = intrinsics_vec[4];
      double k2 = intrinsics_vec[5];
      double k3 = intrinsics_vec[6];
      double k4 = intrinsics_vec[7];

      camera.camera_matrix =
          (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
      camera.dist_coeffs = (cv::Mat_<double>(1, 4) << k1, k2, k3, k4);

      AINFO << "Loaded intrinsics_8 for " << name << ": fx=" << fx
            << " fy=" << fy << " cx=" << cx << " cy=" << cy << " k1=" << k1
            << " k2=" << k2 << " k3=" << k3 << " k4=" << k4;
    }

    // Load lidar2image: complete projection matrix = K_new @ inv(camera2lidar)
    // This is all we need for projection; it already includes K_new
    if (params.lidar2image_vec.size() >= 4) {
      Eigen::Matrix4f lidar2image = Eigen::Matrix4f::Identity();
      for (int row = 0; row < 4 && row < params.lidar2image_vec.size(); ++row) {
        for (int col = 0; col < 4 && col < params.lidar2image_vec[row].size();
             ++col) {
          lidar2image(row, col) = params.lidar2image_vec[row][col];
        }
      }
      camera.lidar2image = lidar2image;
      camera.has_calibration = true;

      AINFO << "Loaded lidar2image projection matrix for " << name;
    }

    // Initialize fisheye undistortion map
    // Use camera_matrix (K_std) and dist_coeffs to create the mapping
    if (!camera.camera_matrix.empty() && !camera.dist_coeffs.empty()) {
      cv::Mat_<double> identity_matrix = cv::Mat_<double>::eye(3, 3);
      try {
        // Build new_K from lidar2image's K component if available
        // For now, estimate new_K using OpenCV's fisheye undistort
        cv::Mat_<double> new_camera_matrix(3, 3);
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
            camera.camera_matrix, camera.dist_coeffs,
            cv::Size(camera.width, camera.height), identity_matrix,
            new_camera_matrix, 0.0);

        cv::fisheye::initUndistortRectifyMap(
            camera.camera_matrix, camera.dist_coeffs, identity_matrix,
            new_camera_matrix, cv::Size(camera.width, camera.height), CV_16SC2,
            camera.map1, camera.map2);

        camera.has_undistort_map = true;
        AINFO << "Initialized fisheye undistort map for camera " << name;
      } catch (const cv::Exception& e) {
        AERROR << "Failed to initialize fisheye undistort for camera " << name
               << ": " << e.what();
        camera.has_undistort_map = false;
      }
    }

    int camera_idx = cameras_.size();

    if (camera.topic.find("compressed") != std::string::npos) {
      camera.image_type = ImageType::COMPRESSED;
      auto callback =
          [this,
           camera_idx](const std::shared_ptr<drivers::CompressedImage>& msg) {
            OnCameraImage(camera_idx, msg);
          };
      node_->CreateReader<drivers::CompressedImage>(camera.topic, callback);
      AINFO << "Subscribed to compressed camera: " << camera.name
            << " topic: " << camera.topic;
    } else {
      camera.image_type = ImageType::RAW;
      auto callback = [this,
                       camera_idx](const std::shared_ptr<drivers::Image>& msg) {
        OnCameraImageRaw(camera_idx, msg);
      };
      node_->CreateReader<drivers::Image>(camera.topic, callback);
      AINFO << "Subscribed to raw camera: " << camera.name
            << " topic: " << camera.topic;
    }

    cameras_.emplace_back(std::move(camera));
  }

  if (cameras_.empty()) {
    AERROR << "No cameras successfully initialized";
    return false;
  }

#if defined(__aarch64__)
  jpeg_decoders_.clear();
  for (size_t i = 0; i < cameras_.size(); ++i) {
    auto decoder = std::make_unique<camera::NvJpegDecoder>(
        cameras_[i].name, 0, cameras_[i].width, cameras_[i].height, 3, false);
    jpeg_decoders_.emplace_back(std::move(decoder));
  }
#endif

  num_cameras_ = cameras_.size();

  kGridCols = (num_cameras_ <= 3)   ? num_cameras_
              : (num_cameras_ <= 4) ? 2
              : (num_cameras_ <= 6) ? 3
              : (num_cameras_ <= 8) ? 4
                                    : 4;
  kGridRows = (num_cameras_ + kGridCols - 1) / kGridCols;

  AINFO << "Successfully initialized " << num_cameras_
        << " cameras, grid layout: " << kGridCols << "x" << kGridRows;
  return true;
}

bool LidarCameraVisualComponent::LoadCameraCalibration() {
  int calibrated_count = 0;
  for (auto& camera : cameras_) {
    if (camera.has_calibration) {
      calibrated_count++;
    }
  }

  if (0 == calibrated_count) {
    AWARN << "No cameras have calibration data loaded";
    return false;
  }

  AINFO << calibrated_count << " cameras have calibration data";
  return true;
}

void LidarCameraVisualComponent::OnCameraImageRaw(
    int camera_idx, const std::shared_ptr<drivers::Image>& image_msg) {
  if (camera_idx < 0 || camera_idx >= static_cast<int>(cameras_.size())) {
    return;
  }

  auto& camera = cameras_[camera_idx];

  uint32_t width = image_msg->width();
  uint32_t height = image_msg->height();
  const std::string& encoding = image_msg->encoding();
  const auto& data = image_msg->data();

  if (0 == width || 0 == height || data.empty()) {
    AERROR << "Invalid raw image from camera " << camera_idx
           << ": width=" << width << " height=" << height
           << " data_size=" << data.size();
    return;
  }

  cv::Mat raw_image;
  if (encoding == "rgb8") {
    raw_image = cv::Mat(
        height, width, CV_8UC3,
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
    cv::cvtColor(raw_image, raw_image, cv::COLOR_RGB2BGR);
  } else if (encoding == "bgr8") {
    raw_image = cv::Mat(
        height, width, CV_8UC3,
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
  } else if (encoding == "rgba8") {
    raw_image = cv::Mat(
        height, width, CV_8UC4,
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
    cv::cvtColor(raw_image, raw_image, cv::COLOR_RGBA2BGR);
  } else if (encoding == "bgra8") {
    raw_image = cv::Mat(
        height, width, CV_8UC4,
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
    cv::cvtColor(raw_image, raw_image, cv::COLOR_BGRA2BGR);
  } else if (encoding == "mono8") {
    raw_image = cv::Mat(
        height, width, CV_8UC1,
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
    cv::cvtColor(raw_image, raw_image, cv::COLOR_GRAY2BGR);
  } else if (encoding == "yuyv" || encoding == "YUYV" || encoding == "yuy2" ||
             encoding == "YUY2") {
    raw_image = cv::Mat(
        height, width, CV_8UC2,
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
    cv::cvtColor(raw_image, raw_image, cv::COLOR_YUV2BGR_YUYV);
  } else if (encoding == "yuv420" || encoding == "i420" || encoding == "I420") {
    // YUV420 (I420) format: Y plane + U plane + V plane
    size_t y_size = width * height;
    size_t uv_size = (width / 2) * (height / 2);
    if (data.size() >= y_size + 2 * uv_size) {
      cv::Mat y_plane(
          height, width, CV_8UC1,
          const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
      cv::Mat u_plane(height / 2, width / 2, CV_8UC1,
                      const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(
                          data.data() + y_size)));
      cv::Mat v_plane(height / 2, width / 2, CV_8UC1,
                      const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(
                          data.data() + y_size + uv_size)));

      cv::Mat u_resized, v_resized;
      cv::resize(u_plane, u_resized, cv::Size(width, height), 0, 0,
                 cv::INTER_LINEAR);
      cv::resize(v_plane, v_resized, cv::Size(width, height), 0, 0,
                 cv::INTER_LINEAR);

      std::vector<cv::Mat> yuv_planes = {y_plane, u_resized, v_resized};
      cv::Mat yuv_image;
      cv::merge(yuv_planes, yuv_image);
      cv::cvtColor(yuv_image, raw_image, cv::COLOR_YUV2BGR);
    } else {
      AERROR << "Invalid YUV420 data size: " << data.size()
             << " expected: " << (y_size + 2 * uv_size);
      return;
    }
  } else {
    AERROR << "Unsupported image encoding: " << encoding << " for camera "
           << camera_idx;
    return;
  }

  if (raw_image.empty()) {
    AERROR << "Failed to create cv::Mat from raw image for camera "
           << camera_idx;
    return;
  }

  cv::Mat processed = raw_image.clone();

  cv::Mat undistorted;
  if (1 && camera.has_undistort_map) {
    cv::remap(processed, undistorted, camera.map1, camera.map2,
              cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  } else {
    undistorted = processed;
  }

  {
    std::lock_guard<std::mutex> lock(camera.mutex);
    camera.image = undistorted.clone();
    camera.timestamp = image_msg->header().timestamp_sec();
    camera.updated = true;
  }
}

void LidarCameraVisualComponent::OnCameraImage(
    int camera_idx,
    const std::shared_ptr<drivers::CompressedImage>& image_msg) {
  if (camera_idx < 0 || camera_idx >= static_cast<int>(cameras_.size())) {
    return;
  }

  auto& camera = cameras_[camera_idx];
  cv::Mat decoded;

#if defined(__aarch64__)
  if (camera_idx < static_cast<int>(jpeg_decoders_.size()) &&
      jpeg_decoders_[camera_idx]) {
    int width = 0, height = 0;
    std::vector<uint8_t> buffer(image_msg->data().begin(),
                                image_msg->data().end());

    bool decode_ret = jpeg_decoders_[camera_idx]->Decode(
        buffer.data(), buffer.size(), &width, &height);

    if (decode_ret && width > 0 && height > 0) {
      uint8_t* host_buffer = jpeg_decoders_[camera_idx]->GetHostBuffer();
      if (host_buffer) {
        cv::Mat rgb_image(height, width, CV_8UC3, host_buffer);
        cv::Mat bgr_image;
        cv::cvtColor(rgb_image, bgr_image, cv::COLOR_RGB2BGR);
        decoded = bgr_image.clone();
      }
    }
  }
#else
  std::vector<uint8_t> buffer(image_msg->data().begin(),
                              image_msg->data().end());
  // If data is empty but format is large, proto field numbers are mismatched
  // (bag recorded with older proto where data=field3, format=field2)
  const std::string& maybe_jpeg =
      image_msg->format().size() > 100 ? image_msg->format() : std::string();
  const std::vector<uint8_t>& src =
      !maybe_jpeg.empty()
          ? std::vector<uint8_t>(maybe_jpeg.begin(), maybe_jpeg.end())
          : buffer;
  decoded = cv::imdecode(src, cv::IMREAD_COLOR);
#endif

  if (decoded.empty()) {
    return;
  }

  cv::Mat undistorted;

  if (camera.has_undistort_map) {
    cv::remap(decoded, undistorted, camera.map1, camera.map2, cv::INTER_LINEAR,
              cv::BORDER_CONSTANT);
  } else {
    undistorted = decoded;
  }

  {
    std::lock_guard<std::mutex> lock(camera.mutex);
    camera.image = undistorted.clone();
    camera.timestamp = image_msg->header().timestamp_sec();
    camera.updated = true;
  }
}

bool LidarCameraVisualComponent::Proc(
    const std::shared_ptr<PerceptionObstacleDebugMsg>& message) {
  const auto& affine = message->lidar2world();
  Eigen::Translation3d translation(affine.tx(), affine.ty(), affine.tz());
  Eigen::Quaterniond rotation(affine.qw(), affine.qx(), affine.qy(),
                              affine.qz());
  Eigen::Affine3d lidar2world_pose = translation * rotation;

  auto lidar_frame = std::make_shared<lidar::LidarFrame>();
  lidar_frame->cloud = base::PointFCloudPool::Instance().Get();
  lidar_frame->raw_cloud = base::PointFCloudPool::Instance().Get();
  lidar_frame->lidar2world_pose = lidar2world_pose;

  std::vector<common::visualizer::PointXYZI> points_all;
  for (const auto& point : message->seg_pointcloud().point()) {
    base::PointF ptf;
    ptf.x = point.x();
    ptf.y = point.y();
    ptf.z = point.z();
    ptf.intensity = point.intensity();
    lidar_frame->cloud->push_back(ptf);

    Eigen::Vector3d point_lidar(point.x(), point.y(), point.z());
    Eigen::Vector3d point_world = lidar2world_pose * point_lidar;
    Eigen::Vector3f point_local =
        (point_world - lidar2world_pose.translation()).cast<float>();

    common::visualizer::PointXYZI p;
    p.x = point_local(0);
    p.y = point_local(1);
    p.z = point_local(2);
    p.intensity = 0;
    points_all.emplace_back(p);
  }

  for (const auto& point : message->raw_pointcloud().point()) {
    base::PointF ptf;
    ptf.x = point.x();
    ptf.y = point.y();
    ptf.z = point.z();
    ptf.intensity = point.intensity();
    lidar_frame->raw_cloud->push_back(ptf);

    Eigen::Vector3d point_lidar(point.x(), point.y(), point.z());
    Eigen::Vector3d point_world = lidar2world_pose * point_lidar;
    Eigen::Vector3f point_local =
        (point_world - lidar2world_pose.translation()).cast<float>();

    common::visualizer::PointXYZI p;
    p.x = point_local(0);
    p.y = point_local(1);
    p.z = point_local(2);
    p.intensity = 1;
    points_all.emplace_back(p);
  }

  std::vector<common::visualizer::BBox> out_boxes;
  common::visualizer::PolygonVector out_polygons;
  const auto& perception_obstacles = message->obstacles().perception_obstacle();

  for (const auto& obstacle : perception_obstacles) {
    base::ObjectPtr obj = std::make_shared<base::Object>();
    ConvertPbToObject(obstacle, obj);
    lidar_frame->segmented_objects.emplace_back(obj);

    if (0 == obstacle.type()) {
      std::vector<common::visualizer::PointXYZI> obj_polygon;
      for (int i = 0; i < obstacle.polygon_point_size(); ++i) {
        const auto& pt = obstacle.polygon_point(i);
        Eigen::Vector3d point_lidar(pt.x(), pt.y(), pt.z());
        Eigen::Vector3d point_world = lidar2world_pose * point_lidar;
        Eigen::Vector3f point_local =
            (point_world - lidar2world_pose.translation()).cast<float>();

        common::visualizer::PointXYZI polygon_point;
        polygon_point.x = point_local(0);
        polygon_point.y = point_local(1);
        polygon_point.z = point_local(2);
        polygon_point.intensity = 1.0f;
        obj_polygon.emplace_back(polygon_point);
      }
      if (!obj_polygon.empty()) {
        out_polygons.emplace_back(obj_polygon);
      }
    } else {
      common::visualizer::BBox box;
      Eigen::Vector3d center_lidar(obstacle.position().x(),
                                   obstacle.position().y(),
                                   obstacle.position().z());
      Eigen::Vector3d center_world = lidar2world_pose * center_lidar;
      box.center =
          (center_world - lidar2world_pose.translation()).cast<float>();
      box.size = Eigen::Vector3f(obstacle.length(), obstacle.width(),
                                 obstacle.height());

      float yaw_rad = obstacle.theta();
      Eigen::Matrix3d R_rel;
      R_rel = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ());
      Eigen::Matrix3d R_global = lidar2world_pose.rotation() * R_rel;
      Eigen::Quaternionf q_global(R_global.cast<float>());
      q_global.normalize();
      box.q = q_global;
      box.id = static_cast<int>(obstacle.type());
      out_boxes.emplace_back(box);
    }
  }

  if (use_map_manager_) {
    lidar::MapManagerOptions map_manager_options;
    if (!map_manager_.Update(map_manager_options, lidar_frame.get())) {
      AERROR << "Failed to update map structure.";
    }
  }

  std::vector<std::vector<common::visualizer::PointXYZI>> maps_info;
  if (lidar_frame->hdmap_struct != nullptr &&
      !lidar_frame->hdmap_struct->road_polygons.empty()) {
    const auto& road_polygons = lidar_frame->hdmap_struct->road_polygons;
    for (const auto& road_polygon : road_polygons) {
      std::vector<common::visualizer::PointXYZI> polygon_vertices;
      for (const auto& pt : road_polygon) {
        Eigen::Vector3d trans_point(pt.x, pt.y, pt.z);
        Eigen::Vector3d temp_point =
            trans_point - lidar2world_pose.translation();
        common::visualizer::PointXYZI pcl_local_point;
        pcl_local_point.x = temp_point(0);
        pcl_local_point.y = temp_point(1);
        pcl_local_point.z = 0;
        pcl_local_point.intensity = 1.0f;
        polygon_vertices.emplace_back(pcl_local_point);
      }
      maps_info.emplace_back(polygon_vertices);
    }
  }

  common::visualizer::Pose pose;
  pose.t = Eigen::Vector3f::Zero();
  Eigen::Quaterniond qd(lidar2world_pose.rotation());
  Eigen::Quaternionf qf = qd.cast<float>();
  qf.normalize();
  pose.q = qf;

  if (pangolin_viewer_) {
    pangolin_viewer_->PushPointCloud(points_all);
    pangolin_viewer_->PushBBoxes(out_boxes);
    pangolin_viewer_->PushPolygons(out_polygons);
    pangolin_viewer_->PushMap(maps_info);
    pangolin_viewer_->PushPose(pose);
  }

  {
    std::lock_guard<std::mutex> lock(perception_mutex_);
    latest_perception_ = message;
    has_perception_data_ = true;
    latest_lidar2world_ = lidar2world_pose;
  }

  return true;
}

void LidarCameraVisualComponent::RenderCameraLoop() {
  cv::namedWindow("LidarCamera Visualizer", cv::WINDOW_AUTOSIZE);
  while (running_ && cyber::OK()) {
    if (show_images_) {
      RenderCameraGrid();
    }

    int key = cv::waitKey(30);
    if (key > 0) {
      HandleKeyboard(key);
    }
  }

  cv::destroyAllWindows();
}

void LidarCameraVisualComponent::RenderCameraGrid() {
  bool any_updated = false;
  int image_width = 0, image_height = 0;

  for (int i = 0; i < num_cameras_; ++i) {
    if (cameras_[i].updated.load()) {
      any_updated = true;
    }
    if (0 == image_width) {
      std::lock_guard<std::mutex> lock(cameras_[i].mutex);
      if (!cameras_[i].image.empty()) {
        image_width = cameras_[i].image.cols;
        image_height = cameras_[i].image.rows;
      }
    }
  }

  if (!any_updated || 0 == image_width || 0 == image_height) {
    return;
  }

  int cell_width = image_width / 2;
  int cell_height = image_height / 2;
  int canvas_width = cell_width * kGridCols;
  int canvas_height = cell_height * kGridRows;
  cv::Mat canvas(canvas_height, canvas_width, CV_8UC3, cv::Scalar(20, 20, 20));
  int images_drawn = 0;

  for (int i = 0; i < num_cameras_; ++i) {
    int row = i / kGridCols;
    int col = i % kGridCols;

    auto& camera = cameras_[i];
    cv::Mat image;
    bool has_image = false;

    {
      std::lock_guard<std::mutex> lock(camera.mutex);
      if (!camera.image.empty()) {
        image = camera.image.clone();
        has_image = true;
      }
      camera.updated = false;
    }

    int x = col * cell_width;
    int y = row * cell_height;
    cv::Rect roi(x, y, cell_width, cell_height);

    if (has_image) {
      if (show_obstacle_projection_ && calibration_loaded_ &&
          camera.has_calibration && has_perception_data_) {
        std::vector<base::ObjectPtr> objects;
        {
          std::lock_guard<std::mutex> lock(perception_mutex_);
          if (latest_perception_) {
            const auto& obs =
                latest_perception_->obstacles().perception_obstacle();
            for (const auto& pb_obj : obs) {
              auto obj = std::make_shared<base::Object>();
              ConvertPbToObject(pb_obj, obj);
              objects.emplace_back(obj);
            }
          }
        }
        DrawObstaclesOnCamera(image, i, objects);
      }

      cv::resize(image, image, cv::Size(cell_width, cell_height), 0, 0,
                 cv::INTER_LINEAR);

      DrawCameraLabel(image, camera.name);
      if (show_fps_) {
        DrawFPS(image);
      }

      image.copyTo(canvas(roi));
      images_drawn++;
    } else {
      DrawNoSignal(canvas, row, col, camera.name);
    }
  }

  DrawGridLines(canvas);
  cv::imshow("LidarCamera Visualizer", canvas);
}

void LidarCameraVisualComponent::DrawCameraLabel(cv::Mat& image,
                                                 const std::string& label) {
  cv::putText(image, label, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
              cv::Scalar(0, 255, 0), 2);
}

void LidarCameraVisualComponent::DrawFPS(cv::Mat& image) {
  static auto last_time = std::chrono::steady_clock::now();
  static int frame_count = 0;
  static float fps = 0.0f;

  auto now = std::chrono::steady_clock::now();
  frame_count++;

  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time)
          .count();
  if (duration > 1000) {
    fps = frame_count * 1000.0f / duration;
    frame_count = 0;
    last_time = now;
  }

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(1) << fps << " FPS";
  cv::putText(image, oss.str(), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX,
              0.6, cv::Scalar(0, 255, 255), 2);
}

void LidarCameraVisualComponent::DrawNoSignal(cv::Mat& canvas, int row, int col,
                                              const std::string& name) {
  int image_width = canvas.cols / kGridCols;
  int image_height = canvas.rows / kGridRows;

  int x = col * image_width;
  int y = row * image_height;
  cv::Rect roi(x, y, image_width, image_height);

  cv::rectangle(canvas, roi, cv::Scalar(40, 40, 40), -1);

  std::string text = name + " - No Signal";
  int baseline;
  cv::Size text_size =
      cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.8, 2, &baseline);
  cv::Point text_org(x + (image_width - text_size.width) / 2,
                     y + (image_height + text_size.height) / 2);
  cv::putText(canvas, text, text_org, cv::FONT_HERSHEY_SIMPLEX, 0.8,
              cv::Scalar(128, 128, 128), 2);
}

void LidarCameraVisualComponent::DrawGridLines(cv::Mat& canvas) {
  cv::Scalar grid_color(60, 60, 60);
  int thickness = 2;

  int image_width = canvas.cols / kGridCols;
  int image_height = canvas.rows / kGridRows;

  for (int col = 1; col < kGridCols; ++col) {
    int x = col * image_width;
    cv::line(canvas, cv::Point(x, 0), cv::Point(x, canvas.rows), grid_color,
             thickness);
  }

  for (int row = 1; row < kGridRows; ++row) {
    int y = row * image_height;
    cv::line(canvas, cv::Point(0, y), cv::Point(canvas.cols, y), grid_color,
             thickness);
  }
}

void LidarCameraVisualComponent::DrawObstaclesOnCamera(
    cv::Mat& image, int camera_idx,
    const std::vector<base::ObjectPtr>& objects) {
  auto projected = ProjectObstaclesToCamera(objects, camera_idx);

  for (const auto& obstacle : projected) {
    DrawBboxOnImage(image, obstacle);
  }
}

std::vector<ProjectedObstacle>
LidarCameraVisualComponent::ProjectObstaclesToCamera(
    const std::vector<base::ObjectPtr>& objects, int camera_idx) {
  std::vector<ProjectedObstacle> result;

  auto& camera = cameras_[camera_idx];
  const auto& lidar2image = camera.lidar2image;

  for (const auto& obj : objects) {
    auto corners_3d = GetObjectCorners(obj);

    ProjectedObstacle proj;
    proj.type = obj->type;
    proj.confidence = obj->confidence;
    proj.track_id = obj->track_id;
    proj.visible_count = 0;
    proj.corner_visible = {false};

    float min_x = camera.width, min_y = camera.height;
    float max_x = 0, max_y = 0;

    for (int i = 0; i < 8; ++i) {
      cv::Point2f img_point;
      if (ProjectLidarPointToCamera(corners_3d[i], lidar2image, camera.width,
                                    camera.height, img_point)) {
        proj.corners_2d[i] = img_point;
        proj.corner_visible[i] = true;
        proj.visible_count++;

        min_x = std::min(min_x, img_point.x);
        min_y = std::min(min_y, img_point.y);
        max_x = std::max(max_x, img_point.x);
        max_y = std::max(max_y, img_point.y);
      }
    }

    if (proj.visible_count >= 4) {
      proj.bbox2d = cv::Rect2f(min_x, min_y, max_x - min_x, max_y - min_y);
      result.emplace_back(proj);
    }
  }

  return result;
}

bool LidarCameraVisualComponent::ProjectLidarPointToCamera(
    const Eigen::Vector3d& point_lidar, const Eigen::Matrix4f& lidar2image,
    int image_width, int image_height, cv::Point2f& image_point) {
  Eigen::Vector4f point_lidar_h;
  point_lidar_h << point_lidar.x(), point_lidar.y(), point_lidar.z(), 1.0f;

  Eigen::Vector4f point_image_h = lidar2image * point_lidar_h;

  if (point_image_h(2) <= 0.1f) {
    return false;
  }

  float u = point_image_h(0) / point_image_h(2);
  float v = point_image_h(1) / point_image_h(2);

  if (u < 0 || u >= image_width || v < 0 || v >= image_height) {
    return false;
  }

  image_point.x = u;
  image_point.y = v;
  return true;
}

std::vector<Eigen::Vector3d> LidarCameraVisualComponent::GetObjectCorners(
    const base::ObjectPtr& obj) {
  std::vector<Eigen::Vector3d> corners;

  Eigen::Vector3d center(obj->center(0), obj->center(1), obj->center(2));
  Eigen::Vector3d size(obj->size(0), obj->size(1), obj->size(2));

  Eigen::AngleAxisd rotation(obj->theta, Eigen::Vector3d::UnitZ());

  for (int i = 0; i < 8; ++i) {
    double x = (i & 1) ? size(0) / 2 : -size(0) / 2;
    double y = (i & 2) ? size(1) / 2 : -size(1) / 2;
    double z = (i & 4) ? size(2) / 2 : -size(2) / 2;

    Eigen::Vector3d local(x, y, z);
    Eigen::Vector3d world = center + rotation * local;
    corners.emplace_back(world);
  }

  return corners;
}

void LidarCameraVisualComponent::DrawBboxOnImage(
    cv::Mat& image, const ProjectedObstacle& obstacle) {
  cv::Scalar color = GetColorForObjectType(obstacle.type);

  // lidar2image already projects into the actual camera pixel space —
  // no extra scaling needed here.
  const int edges[12][2] = {{0, 1}, {1, 3}, {3, 2}, {2, 0}, {4, 5}, {5, 7},
                            {7, 6}, {6, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};

  for (const auto& edge : edges) {
    int i = edge[0];
    int j = edge[1];
    if (obstacle.corner_visible[i] && obstacle.corner_visible[j]) {
      cv::line(image, obstacle.corners_2d[i], obstacle.corners_2d[j], color, 2);
    }
  }

  for (int i = 0; i < 8; ++i) {
    if (obstacle.corner_visible[i]) {
      cv::circle(image, obstacle.corners_2d[i], 3, color, -1);
    }
  }

  cv::Rect2f bbox = obstacle.bbox2d;

  bbox.x = std::max(0.0f, bbox.x);
  bbox.y = std::max(0.0f, bbox.y);
  bbox.width = std::min(bbox.width, static_cast<float>(image.cols) - bbox.x);
  bbox.height = std::min(bbox.height, static_cast<float>(image.rows) - bbox.y);

  std::string label = ObjectTypeToString(obstacle.type) + " " +
                      std::to_string(obstacle.track_id);
  int baseline;
  cv::Size text_size =
      cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
  int label_y = std::max(static_cast<int>(bbox.y), text_size.height + 4);
  cv::rectangle(image, cv::Point(bbox.x, label_y - text_size.height - 4),
                cv::Point(bbox.x + text_size.width + 4, label_y), color, -1);
  cv::putText(image, label, cv::Point(bbox.x + 2, label_y - 2),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
}

cv::Scalar LidarCameraVisualComponent::GetColorForObjectType(
    base::ObjectType type) {
  switch (type) {
    case base::ObjectType::VEHICLE:
      return cv::Scalar(0, 255, 0);
    case base::ObjectType::PEDESTRIAN:
      return cv::Scalar(0, 0, 255);
    case base::ObjectType::BICYCLE:
      return cv::Scalar(255, 0, 0);
    default:
      return cv::Scalar(255, 255, 0);
  }
}

std::string LidarCameraVisualComponent::ObjectTypeToString(
    base::ObjectType type) {
  switch (type) {
    case base::ObjectType::VEHICLE:
      return "Vehicle";
    case base::ObjectType::PEDESTRIAN:
      return "Pedestrian";
    case base::ObjectType::BICYCLE:
      return "Bicycle";
    default:
      return "Unknown";
  }
}

void LidarCameraVisualComponent::ConvertPbToObject(
    const PerceptionObstacle& pb_msg, const base::ObjectPtr& obj) {
  obj->id = pb_msg.id();
  obj->track_id = pb_msg.id();
  obj->confidence = pb_msg.confidence();
  obj->type = static_cast<base::ObjectType>(pb_msg.type());

  obj->center(0) = pb_msg.position().x();
  obj->center(1) = pb_msg.position().y();
  obj->center(2) = pb_msg.position().z();

  obj->size(0) = pb_msg.length();
  obj->size(1) = pb_msg.width();
  obj->size(2) = pb_msg.height();

  obj->theta = pb_msg.theta();

  obj->polygon.clear();
  for (int i = 0; i < pb_msg.polygon_point_size(); ++i) {
    base::PointD pt;
    pt.x = pb_msg.polygon_point(i).x();
    pt.y = pb_msg.polygon_point(i).y();
    pt.z = pb_msg.polygon_point(i).z();
    obj->polygon.push_back(pt);
  }
}

void LidarCameraVisualComponent::HandleKeyboard(int key) {
  switch (key) {
    case 'q':
    case 27:
      running_ = false;
      break;
    case 'i':
      show_images_ = !show_images_;
      break;
    case 'p':
      show_obstacle_projection_ = !show_obstacle_projection_;
      break;
    case 'f':
      show_fps_ = !show_fps_;
      break;
    default:
      break;
  }
}

}  // namespace onboard
}  // namespace perception
}  // namespace century
