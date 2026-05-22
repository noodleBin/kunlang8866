/******************************************************************************
 * Copyright 2018 The Century Authors. All Rights Reserved.
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

#include "modules/perception/lidar/lib/roi_filter/hdmap_roi_filter/hdmap_roi_filter.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <vector>

#include "modules/perception/lidar/lib/roi_filter/hdmap_roi_filter/proto/hdmap_roi_filter.pb.h"
#include <opencv2/opencv.hpp>

#include "cyber/common/file.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_point_label.h"
#include "modules/perception/lidar/lib/roi_filter/hdmap_roi_filter/polygon_mask.h"
#include "modules/perception/lidar/lib/scene_manager/scene_manager.h"

namespace century {
namespace perception {
namespace lidar {

using DirectionMajor = Bitmap2D::DirectionMajor;
using base::PolygonDType;
using century::common::EigenVector;
using century::cyber::common::GetAbsolutePath;

template <typename T>
using Polygon = typename PolygonScanCvter<T>::Polygon;

std::string generate_filename() {
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
  return "/century/data/map_filter_data/apollo_map_" + ss.str();
}

void save_polygons(
    century::common::EigenVector<century::perception::base::PointCloud<
        century::perception::base::PointD>>& road_polygons,
    base::PointFCloudPtr points, base::PointIndices& indices) {
  std::string file_name = generate_filename();
  std::cout << file_name << std::endl;
  std::string point_file = file_name + "_point.txt";
  std::ofstream point_out_file(point_file);
  if (!point_out_file.is_open()) {
    std::cerr << "Error opening file: " << point_file << std::endl;
    return;
  }

  for (size_t i = 0; i < indices.indices.size(); ++i) {
    auto& local_pt = (points)->at(indices.indices.at(i));
    point_out_file << local_pt.x << " " << local_pt.y << "\n";
  }
  point_out_file.close();

  std::string road_file = file_name + "_road.txt";
  std::ofstream road_out_file(road_file);
  if (!road_out_file.is_open()) {
    std::cerr << "Error opening file: " << road_file << std::endl;
    return;
  }

  for (size_t i = 0; i < road_polygons.size(); ++i) {
    auto& local_pts = road_polygons[i];
    for (int j = 0; j < local_pts.size(); ++j) {
      auto& point = local_pts.at(j);
      road_out_file << point.x << " " << point.y << "\n";
    }
    road_out_file << "end\n";
  }
  road_out_file.close();

  std::cout << "Map data saved to: " << file_name << std::endl;
}

void save_bitmap(const Bitmap2D& bitmap) {
  auto bit_data = bitmap.SaveBitMap();
  const int width = 961;
  const int height = 961;
  cv::Mat img(height, width, CV_8UC1, cv::Scalar(0));

  for (int row = 0; row < height; ++row) {
    for (int block_idx = 0; block_idx < 16; ++block_idx) {
      uint64_t block = bit_data[row * 16 + block_idx];
      int start_bit = block_idx * 64;
      
      if (block_idx < 15) {
        for (int k = 0; k < 64; ++k) {
          int col = start_bit + k;
          if (block & (1ULL << k)) {
            img.at<uint8_t>(row, col) = 255; 
          }
        }
      }
      else {
        int col = start_bit;  
        if (block & 1) {
          img.at<uint8_t>(row, col) = 255;
        }
      }
    }
  }
  std::string file_name = generate_filename() + "bitmap.png";
  cv::imwrite(file_name, img);
}

void ShrinkPolygons(century::common::EigenVector<base::PolygonDType>& polygons,
                    double offset) {
  for (auto poly : polygons) {
    if (poly.size() < 3) continue;
    // std::vector<century::perception::base::PointD> new_points;
    base::PolygonDType new_points;
    auto& pts = poly.points();
    size_t n = pts.size();

    for (size_t i = 0; i < n; ++i) {
      const auto& p_prev = pts[(i + n - 1) % n];
      const auto& p_curr = pts[i];
      const auto& p_next = pts[(i + 1) % n];

      double v1x = p_curr.x - p_prev.x;
      double v1y = p_curr.y - p_prev.y;
      double v2x = p_next.x - p_curr.x;
      double v2y = p_next.y - p_curr.y;

      double len1 = std::sqrt(v1x * v1x + v1y * v1y);
      double len2 = std::sqrt(v2x * v2x + v2y * v2y);
      if (len1 < 1e-6 || len2 < 1e-6) continue;
      v1x /= len1;
      v1y /= len1;
      v2x /= len2;
      v2y /= len2;

      double n1x = v1y, n1y = -v1x;
      double n2x = v2y, n2y = -v2x;

      double nx = n1x + n2x;
      double ny = n1y + n2y;
      double len_n = std::sqrt(nx * nx + ny * ny);
      if (len_n < 1e-6) continue;
      nx /= len_n;
      ny /= len_n;

      century::perception::base::PointD new_pt;
      new_pt.x = p_curr.x + nx * offset;
      new_pt.y = p_curr.y + ny * offset;
      new_points.push_back(new_pt);
    }

    poly.SwapPointCloud(&new_points);
  }
}

bool HdmapROIFilter::Init(const ROIFilterInitOptions& options) {
  // load model config
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path));
  config_file = GetAbsolutePath(work_root, root_path);
  config_file = GetAbsolutePath(config_file, "hdmap_roi_filter.conf");
  HDMapRoiFilterConfig config;
  ACHECK(century::cyber::common::GetProtoFromFile(config_file, &config));
  range_ = config.range();
  cell_size_ = config.cell_size();
  extend_dist_ = config.extend_dist();
  no_edge_table_ = config.no_edge_table();
  set_roi_service_ = config.set_roi_service();

  // reserve mem
  const size_t KPolygonMaxNum = 100;
  polygons_world_.reserve(KPolygonMaxNum);
  polygons_local_.reserve(KPolygonMaxNum);

  // init bitmap
  Eigen::Vector2d min_range(-range_, -range_);
  Eigen::Vector2d max_range(range_, range_);
  Eigen::Vector2d cell_size(cell_size_, cell_size_);
  bitmap_.Init(min_range, max_range, cell_size);

  // output input parameters
  AINFO << " HDMap Roi Filter Parameters: "
        << " range: " << range_ << " cell_size: " << cell_size_
        << " extend_dist: " << extend_dist_
        << " no_edge_table: " << no_edge_table_
        << " set_roi_service: " << set_roi_service_;
  return true;
}

bool HdmapROIFilter::Filter(const ROIFilterOptions& options,
                            LidarFrame* frame) {
  if (frame->hdmap_struct == nullptr || frame->cloud == nullptr) {
    AERROR << " Input frame data error !";
    return false;
  }

  // get map polygon of roi
  auto& road_polygons = frame->hdmap_struct->road_polygons;
  auto& junction_polygons = frame->hdmap_struct->junction_polygons;
  size_t polygons_world_size = road_polygons.size() + junction_polygons.size();
  if (0 == polygons_world_size) {
    AINFO << " Polygon Empty.";
    return false;
  }

  polygons_world_.clear();
  polygons_world_.resize(polygons_world_size, nullptr);
  size_t i = 0;
  for (auto& polygon : road_polygons) {
    polygons_world_[i++] = &polygon;
  }
  for (auto& polygon : junction_polygons) {
    polygons_world_[i++] = &polygon;
  }

  // transform to local
  base::PointFCloudPtr cloud_local = base::PointFCloudPool::Instance().Get();
  TransformFrame(frame->cloud, frame->lidar2world_pose, polygons_world_,
                 &polygons_local_, &cloud_local);

  //  ShrinkPolygons(polygons_local_, -0.1);
  bool ret = FilterWithPolygonMask(cloud_local, polygons_local_,
                                   &(frame->roi_indices));
  // save_polygons(polygons_local_, cloud_local, frame->roi_indices);
  // set roi points label
  if (ret) {
    for (auto index : frame->roi_indices.indices) {
      frame->cloud->mutable_points_label()->at(index) =
          static_cast<uint8_t>(LidarPointLabel::ROI);
      frame->world_cloud->mutable_points_label()->at(index) =
          static_cast<uint8_t>(LidarPointLabel::ROI);
    }
  }

  // set roi service
  if (set_roi_service_) {
    auto roi_service = SceneManager::Instance().Service("ROIService");
    if (roi_service != nullptr) {
      roi_service_content_.range_ = range_;
      roi_service_content_.cell_size_ = cell_size_;
      roi_service_content_.map_size_ = bitmap_.map_size();
      roi_service_content_.bitmap_ = bitmap_.bitmap();
      roi_service_content_.major_dir_ =
          static_cast<ROIServiceContent::DirectionMajor>(bitmap_.dir_major());
      roi_service_content_.transform_ = frame->lidar2world_pose.translation();
      if (!ret) {
        std::fill(roi_service_content_.bitmap_.begin(),
                  roi_service_content_.bitmap_.end(), -1);
      }
      roi_service->UpdateServiceContent(roi_service_content_);
    } else {
      AINFO << "Failed to find roi service and cannot update.";
    }
  }
  return ret;
}

bool HdmapROIFilter::FilterWithPolygonMask(
    const base::PointFCloudPtr& cloud,
    const EigenVector<PolygonDType>& map_polygons,
    base::PointIndices* roi_indices) {
  std::vector<Polygon<double>> raw_polygons;
  // convert and obtain the major direction
  raw_polygons.resize(map_polygons.size());
  double min_x = range_;
  double max_x = -min_x;
  double min_y = min_x;
  double max_y = max_x;

  for (size_t i = 0; i < map_polygons.size(); ++i) {
    auto& raw_polygon = raw_polygons[i];
    const auto& map_polygon = map_polygons[i];
    raw_polygon.resize(map_polygon.size());
    for (size_t j = 0; j < map_polygon.size(); ++j) {
      raw_polygon[j].x() = map_polygon[j].x;
      raw_polygon[j].y() = map_polygon[j].y;
      min_x = std::min(raw_polygon[j].x(), min_x);
      max_x = std::max(raw_polygon[j].x(), max_x);
      min_y = std::min(raw_polygon[j].y(), min_y);
      max_y = std::max(raw_polygon[j].y(), max_y);
    }
  }
  min_x = std::max(min_x, -range_);
  max_x = std::min(max_x, range_);
  min_y = std::max(min_y, -range_);
  max_y = std::min(max_y, range_);

  DirectionMajor major_dir = DirectionMajor::XMAJOR;
  if ((max_y - min_y) < (max_x - min_x)) {
    major_dir = DirectionMajor::YMAJOR;
  }
  bitmap_.SetUp(major_dir);

  return DrawPolygonsMask<double>(raw_polygons, &bitmap_, extend_dist_,
                                  no_edge_table_) &&
         Bitmap2dFilter(cloud, bitmap_, roi_indices);
}

void HdmapROIFilter::TransformFrame(
    const base::PointFCloudPtr& cloud, const Eigen::Affine3d& vel_pose,
    const EigenVector<PolygonDType*>& polygons_world,
    EigenVector<PolygonDType>* polygons_local,
    base::PointFCloudPtr* cloud_local) {
  Eigen::Vector3d vel_location = vel_pose.translation();
  Eigen::Matrix3d vel_rot = vel_pose.linear();
  Eigen::Vector3d x_axis = vel_rot.row(0);
  Eigen::Vector3d y_axis = vel_rot.row(1);

  // transform polygons
  polygons_local->clear();
  polygons_local->resize(polygons_world.size());
  for (size_t i = 0; i < polygons_local->size(); ++i) {
    const auto& polygon_world = *(polygons_world[i]);
    auto& polygon_local = (*polygons_local)[i];
    polygon_local.resize(polygon_world.size());
    for (size_t j = 0; j < polygon_local.size(); ++j) {
      polygon_local[j].x = polygon_world[j].x - vel_location.x();
      polygon_local[j].y = polygon_world[j].y - vel_location.y();
    }
  }

  // transform cloud
  (*cloud_local)->clear();
  (*cloud_local)->resize(cloud->size());
  for (size_t i = 0; i < (*cloud_local)->size(); ++i) {
    const auto& pt = cloud->at(i);
    auto& local_pt = (*cloud_local)->at(i);
    Eigen::Vector3d e_pt(pt.x, pt.y, pt.z);
    local_pt.x = static_cast<float>(x_axis.dot(e_pt));
    local_pt.y = static_cast<float>(y_axis.dot(e_pt));
  }
}

bool HdmapROIFilter::Bitmap2dFilter(const base::PointFCloudPtr& in_cloud,
                                    const Bitmap2D& bitmap,
                                    base::PointIndices* roi_indices) {
  // if (!bitmap.Check(Eigen::Vector2d(0.0, 0.0))) {
  //   AWARN << " Car is not in roi!!.";
  //   return false;
  // }
  // save_bitmap(bitmap);
  roi_indices->indices.clear();
  roi_indices->indices.reserve(in_cloud->size());
  for (size_t i = 0; i < in_cloud->size(); ++i) {
    const auto& pt = in_cloud->at(i);
    Eigen::Vector2d e_pt(pt.x, pt.y);
    if (!bitmap.IsExists(e_pt)) {
      continue;
    }
    if (bitmap.Check(e_pt)) {
      roi_indices->indices.push_back(static_cast<int>(i));
    }
  }
  return true;
}

PERCEPTION_REGISTER_ROIFILTER(HdmapROIFilter);

}  // namespace lidar
}  // namespace perception
}  // namespace century
