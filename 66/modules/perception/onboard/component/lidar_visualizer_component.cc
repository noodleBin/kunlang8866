/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
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
#include "modules/perception/onboard/component/lidar_visualizer_component.h"

// #include "cyber/time/clock.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "sys/time.h"

#include "cyber/base/thread_pool.h"
#include "cyber/time/clock.h"
#include "cyber/time/rate.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/common/point_cloud_processing/common.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/common/timer_util.h"
#include "modules/perception/lidar/common/lidar_error_code.h"
#include "modules/perception/lidar/common/lidar_frame_pool.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/lib/scene_manager/scene_manager.h"
#include "modules/perception/onboard/common_flags/common_flags.h"
#include "modules/perception/onboard/msg_serializer/msg_serializer.h"

using ::century::cyber::Clock;

namespace century {
namespace perception {
namespace onboard {

using google::protobuf::Closure;
using google::protobuf::NewCallback;

bool LidarVisualizerComponent::Init() {
  localization_reader_ =
      node_->CreateReader<localization::LocalizationEstimate>(
          FLAGS_localization_topic);
  constexpr int kQueueSize = 10;
  merged_msg_queue_ =
      std::make_shared<lib::FixedSizeConQueue<LidarFrameMessagePtr>>(
          kQueueSize);

  if (use_map_manager_) {
    lidar::MapManagerInitOptions map_manager_init_options;
    if (!map_manager_.Init(map_manager_init_options)) {
      AINFO << "Failed to init map manager.";
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
    aeb_dis_viz_ = visualizer_config.aeb_dis_viz();;
    grid_viz_ = visualizer_config.grid_viz();;
  }

  

  roi_cloud_ = base::PointFCloudPool::Instance().Get();
  roi_filter_ =
      lidar::BaseROIFilterRegisterer::GetInstanceByName("HdmapROIFilter");
  CHECK_NOTNULL(roi_filter_);
  lidar::ROIFilterInitOptions roi_filter_init_options;
  ACHECK(roi_filter_->Init(roi_filter_init_options))
      << "Failed to init roi filter.";

  typedef std::shared_ptr<century::drivers::CompressedImage> ImageMsgType;
  std::function<void(const ImageMsgType&)> front_left_camera_callback =
      std::bind(&LidarVisualizerComponent::OnFrontLeftReceiveImage, this,
                std::placeholders::_1);
  auto front_left_camera_reader =
      node_->CreateReader("/century/sensor/camera/left_front/image/compressed",
                          front_left_camera_callback);

  std::function<void(const ImageMsgType&)> front_mid_camera_callback =
      std::bind(&LidarVisualizerComponent::OnFrontMidReceiveImage, this,
                std::placeholders::_1);
  auto front_mid_camera_reader =
      node_->CreateReader("/century/sensor/camera/front/image/compressed",
                          front_mid_camera_callback);

  std::function<void(const ImageMsgType&)> front_right_camera_callback =
      std::bind(&LidarVisualizerComponent::OnFrontRightReceiveImage, this,
                std::placeholders::_1);
  auto front_right_camera_reader =
      node_->CreateReader("/century/sensor/camera/right_front/image/compressed",
                          front_right_camera_callback);

  std::function<void(const ImageMsgType&)> rear_left_camera_callback =
      std::bind(&LidarVisualizerComponent::OnRearLeftReceiveImage, this,
                std::placeholders::_1);
  auto rear_left_camera_reader =
      node_->CreateReader("/century/sensor/camera/left_rear/image/compressed",
                          rear_left_camera_callback);

  std::function<void(const ImageMsgType&)> rear_mid_camera_callback =
      std::bind(&LidarVisualizerComponent::OnRearMidReceiveImage, this,
                std::placeholders::_1);
  auto rear_mid_camera_reader = node_->CreateReader(
      "/century/sensor/camera/rear/image/compressed", rear_mid_camera_callback);

  std::function<void(const ImageMsgType&)> rear_right_camera_callback =
      std::bind(&LidarVisualizerComponent::OnRearRightReceiveImage, this,
                std::placeholders::_1);
  auto rear_right_camera_reader =
      node_->CreateReader("/century/sensor/camera/right_rear/image/compressed",
                          rear_right_camera_callback);

  if (use_viz_pcl_) {
    visualize_thread_ = std::make_shared<std::thread>(
        std::bind(&LidarVisualizerComponent::DoVisualize, this));
  }

  if (use_viz_pangolin_) {
    panolin_thread_ = std::make_shared<std::thread>(
      std::bind(&LidarVisualizerComponent::DoVisualizePangoLin, this));
  }

  return true;
}

void LidarVisualizerComponent::DoVisualizePangoLin() noexcept {
  pango_viewer_ = std::make_unique<common::visualizer::PangolinViewer>(1280, 720, aeb_dis_viz_, grid_viz_);
  pango_viewer_->Spin();
}

bool LidarVisualizerComponent::Proc(
    const std::shared_ptr<PerceptionObstacleDebugMsg>& message) {
  // Merge synchronized messages
  auto merged_message = std::make_shared<LidarFrameMessage>();
  if (!merged_message->lidar_frame_) {
    merged_message->lidar_frame_ = lidar::LidarFramePool::Instance().Get();

    merged_message->lidar_frame_->cloud =
        base::PointFCloudPool::Instance().Get();

    merged_message->lidar_frame_->raw_cloud =
        base::PointFCloudPool::Instance().Get();
  }

  UpdateLidarPose(merged_message, message->lidar2world());

  auto seg_cloud = merged_message->lidar_frame_->cloud;
  seg_cloud->set_timestamp(message->seg_pointcloud().header().timestamp_sec());
  auto raw_cloud = merged_message->lidar_frame_->raw_cloud;
  raw_cloud->set_timestamp(message->raw_pointcloud().header().timestamp_sec());
  std::vector<common::visualizer::PointXYZI> points_all; 
  for (const auto& point : message->seg_pointcloud().point()) {
    base::PointF ptf;
    ptf.x = point.x();
    ptf.y = point.y();
    ptf.z = point.z();
    ptf.intensity = point.intensity();
    
    seg_cloud->push_back(ptf);

    Eigen::Vector3d point_seg(float(point.x()), float(point.y()), float(point.z()));
    auto point_world = merged_message->lidar_frame_->lidar2world_pose * point_seg;
    auto point_local = (point_world - merged_message->lidar_frame_->lidar2world_pose.translation()).cast<float>();
    common::visualizer::PointXYZI p;
    p.x = point_local(0);
    p.y = point_local(1);
    p.z = point_local(2);
    p.intensity = 0;
    if (seg_viz_) {
      points_all.push_back(p);
    }
  }

  // std::vector<common::visualizer::PointXYZI> points_raw; 
  for (const auto& point : message->raw_pointcloud().point()) {
    base::PointF ptf;
    ptf.x = point.x();
    ptf.y = point.y();
    ptf.z = point.z();
    ptf.intensity = point.intensity();
    raw_cloud->push_back(ptf);

    Eigen::Vector3d point_raw(float(point.x()), float(point.y()), float(point.z()));
    auto point_world = merged_message->lidar_frame_->lidar2world_pose * point_raw;
    auto point_local = (point_world - merged_message->lidar_frame_->lidar2world_pose.translation()).cast<float>();
    common::visualizer::PointXYZI p;
    p.x = point_local(0);
    p.y = point_local(1);
    p.z = point_local(2);
    p.intensity = 1;
    if (raw_viz_) {
      points_all.push_back(p);
    }
  }

  auto& perception_obstacles = message->obstacles().perception_obstacle();
  std::vector<common::visualizer::BBox> out_boxes;
  common::visualizer::PolygonVector out_polygons;
  for (int i = 0; i < perception_obstacles.size(); ++i) {
    const PerceptionObstacle& obstacle = perception_obstacles.at(i);
    base::ObjectPtr obj = std::make_shared<base::Object>();
    if (!ConvertObjectFromPb(obstacle, obj)) {
      AERROR << "ConvertObjectFromPb failed, obstacle ID: " << obstacle.id();
      return false;
    }
    merged_message->lidar_frame_->segmented_objects.push_back(obj);
    if (obj->type == base::ObjectType::UNKNOWN) {
      std::vector<common::visualizer::PointXYZI> obj_polygon;
      for (const auto& vertex : obj->polygon) {
        Eigen::Vector3d e_pt(vertex.x, vertex.y, vertex.z);

        Eigen::Vector3d point_world = merged_message->lidar_frame_->lidar2world_pose * e_pt;
        Eigen::Vector3f point_local = (point_world - merged_message->lidar_frame_->lidar2world_pose.translation()).cast<float>();
        common::visualizer::PointXYZI polygon_point;
        polygon_point.x = point_local(0);
        polygon_point.y = point_local(1);
        polygon_point.z = point_local(2);
        polygon_point.intensity = 1.0f;

        obj_polygon.push_back(polygon_point);
      }
      out_polygons.push_back(obj_polygon);
    } else {
      common::visualizer::BBox box;
      auto center_world= merged_message->lidar_frame_->lidar2world_pose * Eigen::Vector3d(obj->center(0), obj->center(1), obj->center(2));
      box.center = (center_world - 
        merged_message->lidar_frame_->lidar2world_pose.translation()).cast<float>();
      
      box.size = Eigen::Vector3f(obj->size(0), obj->size(1), obj->size(2));
      float yaw_rad = obj->theta;
      Eigen::Matrix3d R_rel;
      R_rel = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ());
      Eigen::Matrix3d R_global = merged_message->lidar_frame_->lidar2world_pose.rotation() * R_rel;
      Eigen::Quaternionf q_global(R_global.cast<float>());
      q_global.normalize(); 
      box.q = q_global;
      box.id = int(obj->type);
      out_boxes.push_back(box);
    }
  }

  common::visualizer::Pose pose;
  pose.t = Eigen::Vector3f::Zero();
  Eigen::Quaterniond qd(merged_message->lidar_frame_->lidar2world_pose.rotation());
  Eigen::Quaternionf qf = qd.cast<float>();
  qf.normalize();
  pose.q = qf;

  auto frame = merged_message->lidar_frame_.get();

  if (use_map_manager_) {
    lidar::MapManagerOptions map_manager_options;
    if (!map_manager_.Update(map_manager_options, frame)) {
      AERROR << "Failed to update map structure.";
      return false;
    }
  }
  std::vector<std::vector<common::visualizer::PointXYZI>> maps_info;
  if (frame->hdmap_struct != nullptr && !frame->hdmap_struct->road_polygons.empty()) {
    const auto& road_polygons =
    frame->hdmap_struct->road_polygons;
    
    for (const auto& road_polygon : road_polygons) {
      size_t points_size = road_polygon.size();
      std::vector<common::visualizer::PointXYZI> polygon_vertices;

      for (size_t i = 0; i < points_size; i++) {
        auto& pt = road_polygon.at(i);
        Eigen::Vector3d trans_point(pt.x, pt.y, pt.z);

        Eigen::Vector3d temp_point = trans_point - frame->lidar2world_pose.translation();
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

  if (use_viz_pangolin_ && pango_viewer_) {
    pango_viewer_->PushPointCloud(points_all);
    pango_viewer_->PushBBoxes(out_boxes);
    pango_viewer_->PushPolygons(out_polygons);
    pango_viewer_->PushPose(pose);
    pango_viewer_->PushMap(maps_info);
  }
  
  merged_msg_queue_->PushBack(merged_message);

  return true;
}

bool LidarVisualizerComponent::InitAlgorithmPlugin() { return true; }

bool LidarVisualizerComponent::InternalProc(
    const std::shared_ptr<const PerceptionObstacleDebugMsg>& in_message,
    const std::shared_ptr<LidarFrameMessage>& out_message) {
  return true;
}

bool LidarVisualizerComponent::UpdateLidarPose(
    LidarFrameMessagePtr merged_msg,
    const std::shared_ptr<century::localization::LocalizationEstimate>&
        localization_msg) noexcept {
  auto& world_pose = localization_msg->pose();
  Eigen::Affine3d lidar2world_pose = Eigen::Affine3d();
  Eigen::Quaterniond vehicle2world_quaternion(
      world_pose.orientation().qw(), world_pose.orientation().qx(),
      world_pose.orientation().qy(), world_pose.orientation().qz());
  Eigen::Vector3d vehicle2world_translate(world_pose.position().x(),
                                          world_pose.position().y(),
                                          world_pose.position().z());
  lidar2world_pose.linear() = vehicle2world_quaternion.toRotationMatrix();
  lidar2world_pose.translation() = vehicle2world_translate;
  merged_msg->lidar_frame_->lidar2world_pose = lidar2world_pose;

  return true;
}

bool LidarVisualizerComponent::UpdateLidarPose(LidarFrameMessagePtr merged_msg,
    const Affine3dProto& affine3d) noexcept {
  Eigen::Translation3d translation(affine3d.tx(), affine3d.ty(), affine3d.tz());   
  Eigen::Quaterniond rotation(affine3d.qw(), affine3d.qx(), affine3d.qy(), affine3d.qz());
  rotation.normalize(); 
  merged_msg->lidar_frame_->lidar2world_pose = translation * rotation;
  return true;
}

bool LidarVisualizerComponent::ConvertObjectFromPb(
    const PerceptionObstacle& pb_msg, const base::ObjectPtr& object_ptr) {
  if (object_ptr == nullptr) {
    return false;
  }

  object_ptr->id = pb_msg.id();
  object_ptr->confidence = pb_msg.confidence();

  object_ptr->track_id = pb_msg.id();
  object_ptr->theta = pb_msg.theta();

  object_ptr->center(0) = pb_msg.position().x();
  object_ptr->center(1) = pb_msg.position().y();
  object_ptr->center(2) = pb_msg.position().z();

  object_ptr->velocity(0) = pb_msg.velocity().x();
  object_ptr->velocity(1) = pb_msg.velocity().y();
  object_ptr->velocity(2) = pb_msg.velocity().z();

  object_ptr->acceleration(0) = pb_msg.acceleration().x();
  object_ptr->acceleration(1) = pb_msg.acceleration().y();
  object_ptr->acceleration(2) = pb_msg.acceleration().z();

  object_ptr->size(0) = pb_msg.length();
  object_ptr->size(1) = pb_msg.width();
  object_ptr->size(2) = pb_msg.height();

  object_ptr->polygon.clear();
  for (int i = 0; i < pb_msg.polygon_point_size(); ++i) {
    base::PointD pt;
    const auto& p = pb_msg.polygon_point(i);
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
    object_ptr->polygon.push_back(pt);
  }

  object_ptr->anchor_point(0) = pb_msg.anchor_point().x();
  object_ptr->anchor_point(1) = pb_msg.anchor_point().y();
  object_ptr->anchor_point(2) = pb_msg.anchor_point().z();

  const auto& bbox2d = pb_msg.bbox2d();
  base::BBox2DF& box = object_ptr->camera_supplement.box;
  box.xmin = bbox2d.xmin();
  box.ymin = bbox2d.ymin();
  box.xmax = bbox2d.xmax();
  box.ymax = bbox2d.ymax();

  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      int idx = i * 3 + j;
      object_ptr->center_uncertainty(i, j) = pb_msg.position_covariance(idx);
      object_ptr->velocity_uncertainty(i, j) = pb_msg.velocity_covariance(idx);
      object_ptr->acceleration_uncertainty(i, j) =
          pb_msg.acceleration_covariance(idx);
    }
  }

  object_ptr->tracking_time = pb_msg.tracking_time();
  object_ptr->type = static_cast<base::ObjectType>(pb_msg.type());
  object_ptr->sub_type = static_cast<base::ObjectSubType>(pb_msg.sub_type());
  object_ptr->latest_tracked_time = pb_msg.timestamp();

  if (!std::isnan(pb_msg.height_above_ground())) {
    object_ptr->lidar_supplement.height_above_ground =
        pb_msg.height_above_ground();
  } else {
    object_ptr->lidar_supplement.height_above_ground = 10.0;
  }

  if (object_ptr->type == base::ObjectType::VEHICLE &&
      pb_msg.has_light_status()) {
    const auto& light_status = pb_msg.light_status();
    auto& car_light = object_ptr->car_light;

    car_light.brake_visible = light_status.brake_visible();
    car_light.brake_switch_on = light_status.brake_switch_on();
    car_light.left_turn_visible = light_status.left_turn_visible();
    car_light.left_turn_switch_on = light_status.left_turn_switch_on();
    car_light.right_turn_visible = light_status.right_turn_visible();
    car_light.right_turn_switch_on = light_status.right_turn_switch_on();
  }

  return true;
}

void LidarVisualizerComponent::DoVisualize() noexcept {
  visualizer_ = std::make_unique<common::LidarVisualizer>(true);
  LidarFrameMessagePtr merged_msg;
  while (century::cyber::OK()) {
    auto flag = merged_msg_queue_->TryPop(&merged_msg);
    if (!flag) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      visualizer_->Visualize(merged_msg);
      continue;
    }

    visualizer_->Visualize(merged_msg);
  }
}

void LidarVisualizerComponent::OnReceiveImage(
    const std::shared_ptr<drivers::CompressedImage>& compressed_image) {}

void LidarVisualizerComponent::OnFrontLeftReceiveImage(
    const std::shared_ptr<drivers::CompressedImage>& compressed_image) {
  if (camera_index_ == static_cast<uint32_t>(CameraName::FRONT_LEFT)) {
    const std::string label = "Camera: FrontLeft";
    ShowImages(compressed_image, label);
  }
}

void LidarVisualizerComponent::OnFrontMidReceiveImage(
    const std::shared_ptr<drivers::CompressedImage>& compressed_image) {
  if (camera_index_ == static_cast<uint32_t>(CameraName::FRONT_MIDDLE)) {
    const std::string label = "Camera: FrontMiddle";
    ShowImages(compressed_image, label);
  }
}

void LidarVisualizerComponent::OnFrontRightReceiveImage(
    const std::shared_ptr<drivers::CompressedImage>& compressed_image) {
  if (camera_index_ == static_cast<uint32_t>(CameraName::FRONT_RIGHT)) {
    const std::string label = "Camera: FrontRight";
    ShowImages(compressed_image, label);
  }
}

void LidarVisualizerComponent::OnRearLeftReceiveImage(
    const std::shared_ptr<drivers::CompressedImage>& compressed_image) {
  if (camera_index_ == static_cast<uint32_t>(CameraName::BACK_LEFT)) {
    const std::string label = "Camera: RearLeft";
    ShowImages(compressed_image, label);
  }
}

void LidarVisualizerComponent::OnRearMidReceiveImage(
    const std::shared_ptr<drivers::CompressedImage>& compressed_image) {
  if (camera_index_ == static_cast<uint32_t>(CameraName::BACK_MIDDLE)) {
    const std::string label = "Camera: RearMiddle";
    ShowImages(compressed_image, label);
  }
}

void LidarVisualizerComponent::OnRearRightReceiveImage(
    const std::shared_ptr<drivers::CompressedImage>& compressed_image) {
  if (camera_index_ == static_cast<uint32_t>(CameraName::BACK_RIGHT)) {
    const std::string label = "Camera: RearRight";
    ShowImages(compressed_image, label);
  }
}

bool LidarVisualizerComponent::KeyHandler(const int key) {
  AINFO << "Pressed Key: " << key;
  if (key <= 0) {
    return false;
  }
  switch (key) {
    case '1':
      camera_index_ = static_cast<uint32_t>(CameraName::FRONT_LEFT);
      break;
    case '2':
      camera_index_ = static_cast<uint32_t>(CameraName::FRONT_MIDDLE);
      break;
    case '3':
      camera_index_ = static_cast<uint32_t>(CameraName::FRONT_RIGHT);
      break;
    case '4':
      camera_index_ = static_cast<uint32_t>(CameraName::BACK_LEFT);
      break;
    case '5':
      camera_index_ = static_cast<uint32_t>(CameraName::BACK_MIDDLE);
      break;
    case '6':
      camera_index_ = static_cast<uint32_t>(CameraName::BACK_RIGHT);
      break;

    default:
      break;
  }
  return true;
}

void LidarVisualizerComponent::ShowImages(
    const std::shared_ptr<drivers::CompressedImage>& compressed_image,
    const std::string& label) noexcept {
  std::vector<uint8_t> compressed_raw_data(compressed_image->data().begin(),
                                           compressed_image->data().end());
  cv::Mat mat_image = cv::imdecode(compressed_raw_data, cv::IMREAD_COLOR);

  float font_scale = 1.2;
  cv::putText(mat_image, label, cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX,
              font_scale, cv::Scalar(0, 255, 0), 2);

  cv::namedWindow("Century Perception Visualizer", cv::WINDOW_NORMAL);
  // cv::setWindowProperty("Century Perception Visualizer",
  // cv::WND_PROP_FULLSCREEN,
  //                       cv::WINDOW_FULLSCREEN);

  double timestamp = compressed_image->header().timestamp_sec();
  std::ostringstream timestamp_stream;
  timestamp_stream << "Timestamp: " << std::fixed << std::setprecision(6)
                   << timestamp;

  cv::putText(mat_image, timestamp_stream.str(), cv::Point(30, 100),
              cv::FONT_HERSHEY_SIMPLEX, font_scale * 0.8, cv::Scalar(255, 0, 0),
              2);

  int image_width = 1280;
  int image_height = 720;
  cv::resizeWindow("Century Perception Visualizer", image_width, image_height);
  cv::imshow("Century Perception Visualizer", mat_image);
  int key = cv::waitKey(100);
  KeyHandler(key);
}

}  // namespace onboard
}  // namespace perception
}  // namespace century
