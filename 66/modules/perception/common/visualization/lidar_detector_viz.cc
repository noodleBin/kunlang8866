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

// lidar_visualizer.cc
#include "modules/perception/common/visualization/lidar_detector_viz.h"

#include <filesystem>
#include <pcl/io/pcd_io.h>

namespace fs = std::filesystem;


namespace century {
namespace perception {
namespace common {

std::array<float, 3> GetObjectSubTypeColor(base::ObjectSubType subtype) {
  static const std::unordered_map<base::ObjectSubType, std::array<float, 3>>
      color_map = {
          {base::ObjectSubType::UNKNOWN, {0.5f, 0.5f, 0.5f}},
          {base::ObjectSubType::CAR, {0.2f, 0.8f, 0.1f}},
          {base::ObjectSubType::PEDESTRIAN, {0.0f, 1.0f, 0.0f}},
          {base::ObjectSubType::IGV_FULL, {0.0f, 0.0f, 1.0f}},
          {base::ObjectSubType::TRUCK, {0.2f, 0.65f, 0.5f}},
          {base::ObjectSubType::TRAILER_EMPTY, {0.5f, 1.0f, 0.0f}},
          {base::ObjectSubType::TRAILER_FULL, {1.0f, 0.5f, 0.8f}},
          {base::ObjectSubType::IGV_EMPTY, {0.0f, 1.0f, 1.0f}},
          {base::ObjectSubType::CRANE, {0.1f, 0.2f, 0.5f}},
          {base::ObjectSubType::OTHER_VEHICLE, {1.0f, 0.0f, 1.0f}},
          {base::ObjectSubType::CONE, {0.65f, 0.16f, 0.16f}},
          {base::ObjectSubType::CONTAINER_FORKLIFT, {0.5f, 0.0f, 0.5f}},
          {base::ObjectSubType::FORKLIFT, {0.0f, 0.5f, 0.5f}},
          {base::ObjectSubType::LORRY, {0.82f, 0.41f, 0.12f}},
          {base::ObjectSubType::CONSTRUCTION_VEHICLE, {0.18f, 0.31f, 0.31f}},
          {base::ObjectSubType::WHEELCRANE, {0.5f, 0.8f, 0.5f}},
        };

  auto it = color_map.find(subtype);
  if (it != color_map.end()) {
    return it->second;
  }
  return {0.5f, 0.5f, 0.5f};
}

LidarVisualizer::LidarVisualizer(bool use_map_manager)
    : use_map_manager_(use_map_manager), is_initialized_(false) {
  Initialize();
}

bool LidarVisualizer::Initialize() {
  // Create PCL visualizer
  viewer_.reset(
      new pcl::visualization::PCLVisualizer("Lidar Frame Visualization"));

  if (!viewer_) {
    AERROR << "Failed to create PCL visualizer";
    return false;
  }

  // Set default background color
  viewer_->setBackgroundColor(0, 0, 0);

  // // Add coordinate system
  // viewer_->addCoordinateSystem(1.0, 0, 0, 0);

  // Set default camera position
  viewer_->setCameraPosition(0, 0, 50, 0, 1, 0);

  // Register keyboard callback
  viewer_->registerKeyboardCallback<LidarVisualizer>(
      &LidarVisualizer::keyboardEventOccurred, *this);

  subtype_to_name_[base::ObjectSubType::UNKNOWN] = "UNKNOWN";
  subtype_to_name_[base::ObjectSubType::CAR] = "CAR";
  subtype_to_name_[base::ObjectSubType::PEDESTRIAN] = "PEDESTRIAN";
  subtype_to_name_[base::ObjectSubType::IGV_FULL] = "IGV_FULL";
  subtype_to_name_[base::ObjectSubType::TRUCK] = "TRUCK";
  subtype_to_name_[base::ObjectSubType::TRAILER_EMPTY] = "TRAILER_EMPTY";
  subtype_to_name_[base::ObjectSubType::TRAILER_FULL] = "TRAILER_FULL";
  subtype_to_name_[base::ObjectSubType::IGV_EMPTY] = "IGV_EMPTY";
  subtype_to_name_[base::ObjectSubType::CRANE] = "CRANE";
  subtype_to_name_[base::ObjectSubType::OTHER_VEHICLE] = "OTHER_VEHICLE";
  subtype_to_name_[base::ObjectSubType::CONE] = "CONE";
  subtype_to_name_[base::ObjectSubType::CONTAINER_FORKLIFT] =
      "CONTAINER_FORKLIFT";
  subtype_to_name_[base::ObjectSubType::FORKLIFT] = "FORKLIFT";
  subtype_to_name_[base::ObjectSubType::LORRY] = "LORRY";
  subtype_to_name_[base::ObjectSubType::CONSTRUCTION_VEHICLE] =
      "CONSTRUCTION_VEHICLE";
  subtype_to_name_[base::ObjectSubType::WHEELCRANE] =
      "WHEELCRANE";
  is_initialized_ = true;
  return true;
}

bool LidarVisualizer::Visualize(
    const std::shared_ptr<LidarFrameMessage>& in_message) noexcept {
  if (!is_initialized_) {
    AERROR << "Visualizer not initialized";
    return false;
  }

  int control_flag = show_control_flag_.load();
  if (0 == (control_flag & 0x10)) {
    SpinOnce();
    return false;
  }

  if (!in_message || !in_message->lidar_frame_ ||
      !in_message->lidar_frame_->cloud) {
    AERROR << "Invalid lidar frame message.";
    return false;
  }

  // Clear previous visualization
  if (viewer_->contains("cloud")) {
    viewer_->removePointCloud("cloud");
  }
  viewer_->removeAllShapes();
  // Remove all existing cubes
  // for (int i = 0; i < cube_ids_.size(); ++i) {
  //   std::string cube_id = cube_ids_[i];
  //   if (viewer_->contains(cube_id)) {
  //     viewer_->removeShape(cube_id);
  //   }
  // }
  cube_ids_.clear();

  // Get lidar pose information
  Eigen::Vector3d vel_location = Eigen::Vector3d::Zero();
  Eigen::Matrix3d vel_rot = Eigen::Matrix3d::Identity();

  if (use_map_manager_ && in_message->lidar_frame_->hdmap_struct &&
      !in_message->lidar_frame_->hdmap_struct->road_polygons.empty()) {
    vel_location = in_message->lidar_frame_->lidar2world_pose.translation();
    vel_rot = in_message->lidar_frame_->lidar2world_pose.linear();
  }

  if (control_flag & 0x01) {
    // Display point cloud
    DisplayPointCloud(in_message->lidar_frame_->raw_cloud, vel_location, vel_rot, "raw_cloud",
                      0, 250, 0);
  } else {
    if (viewer_->contains("raw_cloud")){
      viewer_->removePointCloud("raw_cloud");
    }
  }

  if (control_flag & 0x02) {
    DisplayPointCloud(in_message->lidar_frame_->cloud, vel_location, vel_rot, "cloud", 250, 0,
                      0);
  }

  if (control_flag & 0x04) {
    // Display segmented objects
    DisplaySegmentedObjects(in_message->lidar_frame_->segmented_objects,
                            vel_location, vel_rot);
  }

  if (control_flag & 0x08) {
    // Display road polygons if available
    if (use_map_manager_ && in_message->lidar_frame_->hdmap_struct &&
        !in_message->lidar_frame_->hdmap_struct->road_polygons.empty()) {
      DisplayRoadPolygons(in_message);
    }
  }

  help();
  drawEgo(vel_rot);
  drawPose(vel_rot);
  
  // Update the display
  SpinOnce();

  AINFO << "Lidar frame visualization updated";
  return true;
}

void LidarVisualizer::DisplayPointCloud(const base::PointFCloudPtr& cloud,
                                        const Eigen::Vector3d& location,  
                                        const Eigen::Matrix3d& rotation,
                                        const std::string& name, uint8_t r,
                                        uint8_t g, uint8_t b) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  
  Eigen::Affine3d object_pose = Eigen::Affine3d::Identity();
  object_pose.translation() = location;
  object_pose.linear() = rotation;

  for (const auto& point : cloud->points()) {
    Eigen::Vector3d e_pt(point.x, point.y, point.z);
    Eigen::Vector3d point_world = object_pose * e_pt;
    Eigen::Vector3f point_local =
        (point_world - location).cast<float>();


    pcl::PointXYZRGB pcl_point;

    pcl_point.x = static_cast<float>(point_local(0));
    pcl_point.y = static_cast<float>(point_local(1));
    pcl_point.z = static_cast<float>(point_local(2));

    pcl_cloud->points.emplace_back(pcl_point);
  }

  int flag = show_control_flag_.load();
  if (name == "raw_cloud" && (flag & 0x20)) {
    SavePointCloudToPCD(cloud);
  }

  AINFO << "Point cloud size: " << pcl_cloud->size();

  viewer_->removePointCloud(name);

  viewer_->addPointCloud<pcl::PointXYZRGB>(pcl_cloud, name);
  if ("raw_cloud" == name) {
    std::ostringstream ss;
    ss << "Timestamp: " << std::fixed << std::setprecision(6)
       << cloud->get_timestamp();
    viewer_->addText(ss.str(), 10, 10, "timestamp_text");
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, name);
  } else {
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, name);
  }
}

void LidarVisualizer::SavePointCloudToPCD(const base::PointFCloudPtr& cloud,
                                          const std::string& folder_path) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());

  for (const auto& point : cloud->points()) {
    pcl::PointXYZI pcl_point;
    pcl_point.x = point.x;
    pcl_point.y = point.y;
    pcl_point.z = point.z;
    pcl_point.intensity = point.intensity;
    pcl_cloud->points.emplace_back(std::move(pcl_point));
  }

  pcl_cloud->width = pcl_cloud->points.size();
  pcl_cloud->height = 1;
  pcl_cloud->is_dense = false;

  std::ostringstream filename;
  filename << folder_path << "cloud_" << std::fixed << std::setprecision(6)
           << cloud->get_timestamp() << ".pcd";

  try {
    if (!fs::exists(folder_path)) {
      fs::create_directories(folder_path);
    }
  } catch (const std::exception& e) {
    AERROR << "Create folder path " << folder_path << " faild, " << e.what();
    return;
  }

  if (-1 == pcl::io::savePCDFileBinary(filename.str(), *pcl_cloud)) {
    AERROR << "Failed to save PCD file: " << filename.str();
  } else {
    AINFO << "Saved point cloud to: " << filename.str();
  }
}

void LidarVisualizer::SavePointCloudToBIN(const base::PointFCloudPtr& cloud,
                                          const std::string& folder_path) {
  std::ostringstream filename;
  filename << folder_path << "cloud_" << std::fixed << std::setprecision(6)
           << cloud->get_timestamp() << ".bin";
  AERROR << std::fixed << std::setprecision(10)
         << "<<<< ts: " << cloud->get_timestamp();

  std::ofstream bin_file(filename.str(), std::ios::out | std::ios::binary);
  if (!bin_file.is_open()) {
    AERROR << "Failed to open BIN file: " << filename.str();
    return;
  }

  for (const auto& point : cloud->points()) {
    float data[4] = {static_cast<float>(point.x), static_cast<float>(point.y),
                     static_cast<float>(point.z),
                     static_cast<float>(point.intensity)};
    bin_file.write(reinterpret_cast<const char*>(data), sizeof(data));
  }

  bin_file.close();

  if (bin_file.fail()) {
    AERROR << "Failed to save BIN file: " << filename.str();
  } else {
    AINFO << "Saved point cloud to: " << filename.str();
  }
}

void LidarVisualizer::DisplaySegmentedObjects(
    const std::vector<base::ObjectPtr>& objects,
    const Eigen::Vector3d& location, const Eigen::Matrix3d& rotation) {
  int polygon_id = 0;
  int bbox_id = 0;

  AINFO << "Number of segmented objects: " << objects.size();
  Eigen::Affine3d object_pose = Eigen::Affine3d::Identity();
  object_pose.translation() = location;
  object_pose.linear() = rotation; 
  for (const auto& object : objects) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr polygon_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    
    for (const auto& vertex : object->polygon) {
      Eigen::Vector3d e_pt(vertex.x, vertex.y, vertex.z);

      auto point_world = object_pose * e_pt;
      auto point_local = (point_world - location).cast<float>();

      // float cx = vertex.x;
      // float cy = vertex.y;
      polygon_cloud->points.emplace_back(point_local(0), point_local(1), point_local(2));
    }

    if (!polygon_cloud->points.empty()) {
      viewer_->addPolygon<pcl::PointXYZ>(
          polygon_cloud, 0.0, 1.0, 1.0,
          "polygon_" + std::to_string(polygon_id));
      polygon_id++;
    }

    // if (object->sub_type == base::ObjectSubType::UNKNOWN) {
    //   continue;
    // }

    // Handle bounding box
    float yaw_rad = object->theta;
    Eigen::AngleAxisd rotation_z(yaw_rad, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rotation_z_matrix;
    rotation_z_matrix = rotation_z;
    // Eigen::Quaternionf quaternion(rotation_z);
    Eigen::Matrix3d R_global = rotation * rotation_z_matrix;
    Eigen::Quaternionf quaternion(R_global.cast<float>());
    quaternion.normalize(); 

    double width = static_cast<double>(object->size(0));
    double length = static_cast<double>(object->size(1));
    double height = static_cast<double>(object->size(2));
    double confidence = object->confidence;

    Eigen::Vector3d center_d = (object_pose * object->center - location);
    Eigen::Vector3f center = center_d.cast<float>();

    auto obj_distance = sqrt(object->center[0] * object->center[0] +
                             object->center[1] * object->center[1]);
    AINFO << "Object dimensions - width: " << width << ", length: " << length
          << ", height: " << height << ", center: " << center.transpose()
          << ", yaw: " << yaw_rad;

    std::string bbox_id_str = "bbox_" + std::to_string(bbox_id);

    if (object->sub_type != base::ObjectSubType::UNKNOWN) {
      viewer_->addCube(center, quaternion, width, length, height, bbox_id_str);
    }
    float text_height = 0.2f;
    std::string bbox_id_text = "bbox_text_" + std::to_string(bbox_id);
    pcl::PointXYZ text_position(center(0), center(1), height + text_height);
    auto subtype_color = GetObjectSubTypeColor(object->sub_type);
    auto type_name = subtype_to_name_[object->sub_type];
    if (base::ObjectSubType::CONE == object->sub_type) {
      AERROR << "CONE subtype: " << static_cast<int>(object->sub_type);
    }
    auto object_info_text = type_name + "\r\n" + std::to_string(object->id) +
                            "\r\n" + std::to_string(confidence) + "\r\n" +
                            std::to_string(obj_distance) + "\r\n";

    if (object->sub_type == base::ObjectSubType::UNKNOWN) {
      // auto pos = object->polygon.at(0);
      text_position = pcl::PointXYZ(center(0), center(1), center(2));
      viewer_->addText3D(object_info_text, text_position, 0.1, 1.0, 1.0, 1.0,
                         bbox_id_text);
    } else {
      viewer_->addText3D(object_info_text, text_position, 0.3, 1.0, 1.0, 1.0,
                         bbox_id_text);
    }

    cube_ids_.emplace_back(bbox_id_text);

    if (object->sub_type != base::ObjectSubType::UNKNOWN) {
      viewer_->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, 0.5,
          bbox_id_str);

      viewer_->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR, subtype_color[0],
          subtype_color[1], subtype_color[2], bbox_id_str);
      cube_ids_.emplace_back(bbox_id_str);
    }

    bbox_id++;

  }
}

void LidarVisualizer::DisplayRoadPolygons(
    const std::shared_ptr<LidarFrameMessage>& in_message) {
  int line_id = 0;
  int polygon_id = 0;

  const auto& road_polygons =
      in_message->lidar_frame_->hdmap_struct->road_polygons;

  for (const auto& road_polygon : road_polygons) {
    size_t points_size = road_polygon.size();
    std::vector<pcl::PointXYZ> polygon_vertices;

    // Convert points to PCL format
    for (size_t i = 0; i < points_size; i++) {
      auto pt = road_polygon.at(i);
      Eigen::Vector3d trans_point(pt.x, pt.y, pt.z);

      Eigen::Vector3d temp_point = trans_point - 
          in_message->lidar_frame_->lidar2world_pose.translation();

      pcl::PointXYZ pcl_local_point(temp_point(0), temp_point(1), 0);

      polygon_vertices.emplace_back(pcl_local_point);
    }

    // Draw lines to represent polygon
    for (size_t i = 0; i < polygon_vertices.size(); ++i) {
      size_t j = (i + 1) % polygon_vertices.size();  // Close the polygon

      std::string line_id_str =
          "line_" + std::to_string(polygon_id) + "_" + std::to_string(line_id);

      viewer_->addLine<pcl::PointXYZ>(polygon_vertices[i], polygon_vertices[j],
                                      1.0, 0.0, 0.0,  // Red color (RGB)
                                      line_id_str);

      line_id++;
    }
    polygon_id++;
  }
}

void LidarVisualizer::SpinOnce() {
  if (viewer_) {
    viewer_->spinOnce();
  }
}

bool LidarVisualizer::SaveVisualization(const std::string& filename) {
  if (!viewer_) {
    AERROR << "Visualizer not initialized";
    return false;
  }

  //   return viewer_->saveScreenshot(filename);
  return true;
}

void LidarVisualizer::SetCameraPosition(float pos_x, float pos_y, float pos_z,
                                        float view_x, float view_y,
                                        float view_z) {
  if (viewer_) {
    viewer_->setCameraPosition(pos_x, pos_y, pos_z, view_x, view_y, view_z);
  }
}

void LidarVisualizer::SetBackgroundColor(int r, int g, int b) {
  if (viewer_) {
    viewer_->setBackgroundColor(r, g, b);
  }
}

void LidarVisualizer::keyboardEventOccurred(
    const pcl::visualization::KeyboardEvent& event, void* viewer_void) {
  if (event.getKeyCode() == (unsigned char)'m' && event.keyDown()) {
    int flag = show_control_flag_.load();
    flag = flag ^ 0x01;
    show_control_flag_.store(flag);
  } else if (event.getKeyCode() == (unsigned char)'n' && event.keyDown()) {
    int flag = show_control_flag_.load();
    flag = flag ^ 0x02;
    show_control_flag_.store(flag);
  } else if (event.getKeyCode() == (unsigned char)'b' && event.keyDown()) {
    int flag = show_control_flag_.load();
    flag = flag ^ 0x04;
    show_control_flag_.store(flag);
  } else if (event.getKeyCode() == (unsigned char)'r' && event.keyDown()) {
    int flag = show_control_flag_.load();
    flag = flag ^ 0x08;
    show_control_flag_.store(flag);
  } else if (event.getKeyCode() == (unsigned char)'k' && event.keyDown()) {
    int flag = show_control_flag_.load();
    flag = flag ^ 0x10;
    show_control_flag_.store(flag);
  } else if (event.getKeyCode() == (unsigned char)'S' && event.keyDown()) {
    int flag = show_control_flag_.load();
    flag = flag ^ 0x20;
    show_control_flag_.store(flag);
  } 
}

void LidarVisualizer::drawPose(const Eigen::Matrix3d& rotation) {

  const float axis_length = 1.0f;

  Eigen::Vector3d origin(0, 0, 0);

  Eigen::Vector3d x_dir = rotation.col(0) * axis_length;
  Eigen::Vector3d y_dir = rotation.col(1) * axis_length;
  Eigen::Vector3d z_dir = rotation.col(2) * axis_length;

  viewer_->removeShape("x_axis");
  viewer_->removeShape("y_axis");
  viewer_->removeShape("z_axis");

  viewer_->addLine(
      pcl::PointXYZ(origin.x(), origin.y(), origin.z()),
      pcl::PointXYZ(x_dir.x(), x_dir.y(), x_dir.z()),
      1.0, 0.0, 0.0,
      "x_axis"
  );

  viewer_->addLine(
      pcl::PointXYZ(origin.x(), origin.y(), origin.z()),
      pcl::PointXYZ(y_dir.x(), y_dir.y(), y_dir.z()),
      0.0, 1.0, 0.0,
      "y_axis"
  );

  viewer_->addLine(
      pcl::PointXYZ(origin.x(), origin.y(), origin.z()),
      pcl::PointXYZ(z_dir.x(), z_dir.y(), z_dir.z()),
      0.0, 0.0, 1.0,
      "z_axis"
  );
}

void LidarVisualizer::drawEgo(const Eigen::Matrix3d& rotation) {
  using PointT = pcl::PointXYZ;
  auto poly = boost::make_shared<pcl::PointCloud<PointT>>();
  const float length = 15.0f;
  const float width = 3.0f;

  Eigen::Vector3d front_left(-length/2.0f, width/2.0f, 0.0f);
  Eigen::Vector3d front_right(length/2.0f, width/2.0f, 0.0f);
  Eigen::Vector3d back_right(length/2.0f, -width/2.0f, 0.0f);
  Eigen::Vector3d back_left(-length/2.0f, -width/2.0f, 0.0f);

  front_left = rotation * front_left;
  front_right = rotation * front_right;
  back_right = rotation * back_right;
  back_left = rotation * back_left;

  poly->push_back(PointT(static_cast<float>(front_left(0)), static_cast<float>(front_left(1)), 0));
  poly->push_back(PointT(static_cast<float>(front_right(0)), static_cast<float>(front_right(1)), 0));
  poly->push_back(PointT(static_cast<float>(back_right(0)), static_cast<float>(back_right(1)), 0));
  poly->push_back(PointT(static_cast<float>(back_left(0)), static_cast<float>(back_left(1)), 0)); 

  // poly->push_back(PointT(-length/2.0f, -width/2.0f, 0.0f));
  // poly->push_back(PointT( length/2.0f, -width/2.0f, 0.0f));
  // poly->push_back(PointT( length/2.0f,  width/2.0f, 0.0f));
  // poly->push_back(PointT(-length/2.0f,  width/2.0f, 0.0f));

  viewer_->addPolygon<PointT>(poly, 1.0, 0.0, 0.0, "rect_xy");
  viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
                                      3.0, "rect_xy");
}

void LidarVisualizer::help() {
  std::string help_info_text = "m: raw cloud on/off\r\n"
                               "n: no ground cloud on/off\r\n"
                               "b: segmented objects on/off\r\n"
                               "k: pause on/off\r\n"
                               "q: quit\r\n";
  viewer_->addText(help_info_text, 10, 20, 16, 1.0, 0.0, 1.0, "help_info_text");
}

}  // namespace common
}  // namespace perception
}  // namespace century