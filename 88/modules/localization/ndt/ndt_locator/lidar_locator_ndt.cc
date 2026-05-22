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

#include "modules/localization/ndt/ndt_locator/lidar_locator_ndt.h"

#include <algorithm>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"

#include "cyber/common/log.h"
#include "modules/common/util/perf_util.h"
#include "modules/localization/common/localization_gflags.h"
#include "third_party/mmath/se3.h"

namespace century {
namespace localization {
namespace ndt {

namespace {
constexpr double HALF = 0.5;
constexpr double NUM_THREAD = 4;
}

LidarLocatorNdt::LidarLocatorNdt()
    : config_("map_ndt_v01"), map_(&config_), map_preload_node_pool_(30, 12) {
  Eigen::Translation3d trans(0, 0, 0);
  Eigen::Quaterniond quat(1, 0, 0, 0);
  velodyne_extrinsic_ = trans * quat;
  is_initialized_ = false;
  is_map_loaded_ = false;
  lidar_height_ = 1.7;
  filter_x_ = 128;
  filter_y_ = 128;
}

LidarLocatorNdt::~LidarLocatorNdt() {}

void LidarLocatorNdt::Init(const Eigen::Affine3d& init_location,
                           unsigned int resolution_id, int zone_id) {
  location_ = init_location;
  resolution_id_ = resolution_id;
  zone_id_ = zone_id;
  pre_input_location_ = location_;
  pre_estimate_location_ = location_;
  ndt_max_iterations_ = FLAGS_ndt_max_iterations;
  ndt_target_resolution_ = FLAGS_ndt_target_resolution;
  ndt_line_search_step_size_ = FLAGS_ndt_line_search_step_size;
  ndt_transformation_epsilon_ = FLAGS_ndt_transformation_epsilon;

  if (!is_map_loaded_) {
    map_preload_node_pool_.Initial(&(map_.GetMapConfig()));
    map_.InitMapNodeCaches(12, 24);
    map_.AttachMapNodePool(&map_preload_node_pool_);
    map_.LoadMapArea(location_.translation(), resolution_id_, zone_id_,
                     filter_x_, filter_y_);
    AINFO << "Locator map pre-loading is done.";
    is_map_loaded_ = true;
  }

  // set filter
  filter_x_ =
      static_cast<int>(static_cast<float>(FLAGS_ndt_filter_size_x) /
                       map_.GetMapConfig().map_resolutions_[resolution_id_]);
  filter_y_ =
      static_cast<int>(static_cast<float>(FLAGS_ndt_filter_size_y) /
                       map_.GetMapConfig().map_resolutions_[resolution_id_]);
  AINFO << "Filter size: " << filter_x_ << ", " << filter_y_;

  // set NDT
  AINFO << "----------Init NDT----------" << std::endl;
  reg_.SetNumThreads(NUM_THREAD);
  reg_.SetMaximumIterations(ndt_max_iterations_);
  reg_.SetResolution(static_cast<float>(ndt_target_resolution_));
  reg_.SetStepSize(ndt_line_search_step_size_);
  reg_.SetTransformationEpsilon(ndt_transformation_epsilon_);
  AINFO << "Threads number: " << reg_.GetNumThreads();
  AINFO << "Max iterations: " << ndt_max_iterations_;
  AINFO << "Target resolution: " << reg_.GetResolution();
  AINFO << "Line search step size: " << reg_.GetStepSize();
  AINFO << "Transformation epsilon: " << ndt_transformation_epsilon_;
  AINFO << "----------------------------" << std::endl;

  is_initialized_ = true;
}

void LidarLocatorNdt::LoadMap(const Eigen::Affine3d& init_location,
                              unsigned int resolution_id, int zone_id) {
  map_preload_node_pool_.Initial(&(map_.GetMapConfig()));
  map_.InitMapNodeCaches(12, 24);
  map_.AttachMapNodePool(&map_preload_node_pool_);
  map_.LoadMapArea(location_.translation(), resolution_id, zone_id, filter_x_,
                   filter_y_);
  AINFO << "Locator map pre-loading is done.";
  is_map_loaded_ = true;
}

void LidarLocatorNdt::SetMapFolderPath(const std::string folder_path) {
  if (!map_.SetMapFolderPath(folder_path)) {
    AERROR << "Map folder is invalid!";
  }
}

void LidarLocatorNdt::SetVelodyneExtrinsic(const Eigen::Affine3d& extrinsic) {
  velodyne_extrinsic_ = extrinsic;
}

void LidarLocatorNdt::SetLidarHeight(double height) {
  lidar_height_ = height;
  AINFO << "Set height: " << lidar_height_;
}

void LidarLocatorNdt::SetOnlineCloudResolution(const float& online_resolution) {
  proj_reslution_ = online_resolution;
  AINFO << "Proj resolution: " << proj_reslution_;
}

int LidarLocatorNdt::Update(unsigned int frame_idx, const Eigen::Affine3d& pose,
                            const LidarFrame& lidar_frame) {
  return 0;
};
int LidarLocatorNdt::Update(
    unsigned int frame_idx, const Eigen::Affine3d& pose,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& online_points) {
  // // Increasement from INSPVA
  Eigen::Vector3d trans_diff =
      pose.translation() - pre_input_location_.translation();
  // AINFO << "pre pose: " << pre_input_location_.translation()[0] - UTM_offset_x << ", " << 
  // pre_input_location_.translation()[1] - UTM_offset_y << ", " << pre_input_location_.translation()[2];
  // AINFO << "cur pose: " << pose.translation()[0] - UTM_offset_x << ", " << 
  // pose.translation()[1] - UTM_offset_y << ", " << pose.translation()[2];
  Eigen::Vector3d trans_pre_local =
      pre_estimate_location_.translation() + trans_diff;
  Eigen::Quaterniond quatd(pose.linear());
  Eigen::Translation3d transd(trans_pre_local);
  Eigen::Affine3d center_pose = transd * quatd;
  Eigen::Affine3d predict_transform = center_pose;
  pre_input_location_ = pose;
  // predict_transform.translation().z() = 0.0;


// Pre-load the map nodes
#ifdef USE_PRELOAD_MAP_NODE
  bool map_is_ready =
      map_.LoadMapArea(center_pose.translation(), resolution_id_, zone_id_,
                       filter_x_, filter_y_);
  map_.PreloadMapArea(center_pose.translation(), trans_diff, resolution_id_,
                      zone_id_);
#endif

  // Online pointcloud are projected into a ndt map node. (filtered)
  double lt_x = pose.translation()[0];
  double lt_y = pose.translation()[1];
  double map_resolution = map_.GetMapConfig().map_resolutions_[resolution_id_];
  lt_x -= (map_.GetMapConfig().map_node_size_x_ * map_resolution / 2.0);
  lt_y -= (map_.GetMapConfig().map_node_size_y_ * map_resolution / 2.0);

  // Start Ndt method
  // Convert online points to pcl pointcloud
  RuntimeCounter online_filtered_runtime;
  online_filtered_runtime.StartCounter("OnlineFiltered");
  // Filter online points
  pcl::PointCloud<pcl::PointXYZ>::Ptr online_points_filtered(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(online_points);
  sor.setLeafSize(proj_reslution_, proj_reslution_, proj_reslution_);
  sor.filter(*online_points_filtered);
  online_filtered_runtime.EndCounter("OnlineFiltered");
  // Obtain map pointcloud
  RuntimeCounter map_runtime;
  map_runtime.StartCounter("MapCreate");
  Eigen::Vector2d left_top_coord2d(lt_x, lt_y);
  ComposeMapCells(left_top_coord2d, zone_id_, resolution_id_,
                  map_.GetMapConfig().map_resolutions_[resolution_id_],
                  predict_transform.inverse());

  // Convert map pointcloud to local corrdinate
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_map_point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (unsigned int i = 0; i < cell_map_.size(); ++i) {
    Leaf& le = cell_map_[i];
    float mean_0 = static_cast<float>(le.mean_(0));
    float mean_1 = static_cast<float>(le.mean_(1));
    float mean_2 = static_cast<float>(le.mean_(2));
    pcl_map_point_cloud->push_back(pcl::PointXYZ(mean_0, mean_1, mean_2));
  }
  map_runtime.EndCounter("MapCreate");
  // Set left top corner for reg
  reg_.SetLeftTopCorner(map_left_top_corner_);
  // Ndt calculation
  reg_.SetInputTarget(cell_map_, pcl_map_point_cloud);
  reg_.SetInputSource(online_points_filtered);

  RuntimeCounter ndt_runtime;
  ndt_runtime.StartCounter("NdtAlign");
  Eigen::Matrix3d inv_R = predict_transform.inverse().linear();
  Eigen::Matrix4d init_matrix = Eigen::Matrix4d::Identity();
  init_matrix.block<3, 3>(0, 0) = inv_R.inverse();
  AINFO << "init_matrix trans: " << init_matrix(0, 3) << ", " << init_matrix(1, 3)
        << ", " << init_matrix(2, 3);
  AINFO << "init_matrix rotat: " << init_matrix.block<3, 3>(0, 0).eulerAngles(2, 1, 0).z() << ", " 
        << init_matrix.block<3, 3>(0, 0).eulerAngles(2, 1, 0).y()
        << ", " << init_matrix.block<3, 3>(0, 0).eulerAngles(2, 1, 0).x();

  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  reg_.Align(output_cloud, init_matrix.cast<float>());
  ndt_runtime.EndCounter("NdtAlign");

  fitness_score_ = reg_.GetFitnessScore(2.0);
  has_converged_ = reg_.HasConverged();
  int iteration = reg_.GetFinalNumIteration();
  Eigen::Matrix4d ndt_pose = reg_.GetFinalTransformation().cast<double>();
  AINFO << "Ndt summary:";
  AINFO << "Fitness Score: " << fitness_score_;
  AINFO << "Has_converged: " << has_converged_;
  AINFO << "Iteration: %d: " << iteration;
  AINFO << "Relative Ndt pose t: " << ndt_pose(0, 3) << ", " << ndt_pose(1, 3)
        << ", " << ndt_pose(2, 3);
  mmath::SE3 np = mmath::SE3(ndt_pose);
  AINFO << "Relative Ndt pose r: " << np.getSO3().getEulerYPR().z() << " "
        << np.getSO3().getEulerYPR().y() << " "
        << np.getSO3().getEulerYPR().x();

  // Twv
  Eigen::Affine3d lidar_location = Eigen::Affine3d::Identity();

  mmath::SE3 predict = mmath::SE3(predict_transform.matrix());
  AINFO << "hjf: predict_transform:" << predict.getTranslation().x()-UTM_offset_x << " "
        << predict.getTranslation().y()-UTM_offset_y << " "
        << predict.getTranslation().z() << " "
        << predict.getSO3().getEulerYPR().z() << " "
        << predict.getSO3().getEulerYPR().y() << " "
        << predict.getSO3().getEulerYPR().x();

  lidar_location = predict_transform.matrix() * init_matrix.inverse() * ndt_pose;

  mmath::SE3 lidar = mmath::SE3(lidar_location.matrix());
  AINFO << "hjf: lidar_location:" << lidar.getTranslation().x()-UTM_offset_x << " "
        << lidar.getTranslation().y()-UTM_offset_y << " "
        << lidar.getTranslation().z() << " "
        << lidar.getSO3().getEulerYPR().z() << " "
        << lidar.getSO3().getEulerYPR().y() << " "
        << lidar.getSO3().getEulerYPR().x();

  // Save results
  // lidar_location.translation().z() = pose.translation().z();
  location_ = lidar_location;
  pre_estimate_location_ = lidar_location;
  return map_is_ready ? 0 : -1;
}

Eigen::Affine3d LidarLocatorNdt::GetPose() const { return location_; }

Eigen::Vector3d LidarLocatorNdt::GetPredictLocation() const {
  return predict_location_.translation();
}

Eigen::Matrix3d LidarLocatorNdt::GetLocationCovariance() const {
  return location_covariance_;
}

void LidarLocatorNdt::ComposeMapCells(
    const Eigen::Vector2d& left_top_coord2d, int zone_id,
    unsigned int resolution_id, float map_pixel_resolution,
    const Eigen::Affine3d& inverse_transform) {
  century::common::util::Timer timer;
  timer.Start();

  unsigned int map_node_size_x = map_.GetMapConfig().map_node_size_x_;
  unsigned int map_node_size_y = map_.GetMapConfig().map_node_size_y_;
  unsigned int filter_size_x = filter_x_;
  unsigned int filter_size_y = filter_y_;

  // get the left top coordinate of input online pointcloud
  Eigen::Vector2d coord2d = left_top_coord2d;
  coord2d[0] -= map_pixel_resolution * static_cast<float>(filter_size_x / 2);
  coord2d[1] -= map_pixel_resolution * static_cast<float>(filter_size_y / 2);

  // get the node index of left top corner and global coordinate
  MapNodeIndex map_id = MapNodeIndex::GetMapNodeIndex(
      map_.GetMapConfig(), coord2d, resolution_id, zone_id);
  NdtMapNode* map_node_lt =
      dynamic_cast<NdtMapNode*>(map_.GetMapNodeSafe(map_id));
  assert(map_.IsMapNodeExist(map_id));
  unsigned int map_coord_x = 0;
  unsigned int map_coord_y = 0;
  map_node_lt->GetCoordinate(coord2d, &map_coord_x, &map_coord_y);

  std::vector<std::vector<int>> map_nodes_zones;

  int top_left_start_x = static_cast<int>(map_coord_x);
  int top_left_start_y = static_cast<int>(map_coord_y);
  int top_left_end_x = static_cast<int>(map_node_size_x) - 1;
  int top_left_end_y = static_cast<int>(map_node_size_y) - 1;
  std::vector<int> node_zone;
  node_zone.push_back(top_left_start_x);
  node_zone.push_back(top_left_start_y);
  node_zone.push_back(top_left_end_x);
  node_zone.push_back(top_left_end_y);
  map_nodes_zones.push_back(node_zone);

  int top_center_start_x = 0;
  int top_center_start_y = top_left_start_y;
  int top_center_end_x = static_cast<int>(map_coord_x) - 1 +
                         2 * static_cast<int>(filter_size_x / 2);
  top_center_end_x = top_center_end_x > static_cast<int>(map_node_size_x) - 1
                         ? static_cast<int>(map_node_size_x) - 1
                         : top_center_end_x;
  int top_center_end_y = static_cast<int>(map_node_size_y) - 1;
  node_zone.clear();
  node_zone.push_back(top_center_start_x);
  node_zone.push_back(top_center_start_y);
  node_zone.push_back(top_center_end_x);
  node_zone.push_back(top_center_end_y);
  map_nodes_zones.push_back(node_zone);

  int top_right_start_x = 0;
  int top_right_start_y = top_left_start_y;
  int top_right_end_x = 2 * static_cast<int>(filter_size_x / 2) -
                        (top_left_end_x - top_left_start_x + 1) - 1;
  int top_right_end_y = static_cast<int>(map_node_size_y) - 1;
  node_zone.clear();
  node_zone.push_back(top_right_start_x);
  node_zone.push_back(top_right_start_y);
  node_zone.push_back(top_right_end_x);
  node_zone.push_back(top_right_end_y);
  map_nodes_zones.push_back(node_zone);

  int middle_left_start_x = top_left_start_x;
  int middle_left_start_y = 0;
  int middle_left_end_x = top_left_end_x;
  int middle_left_end_y = static_cast<int>(map_coord_y) - 1 +
                          2 * static_cast<int>(filter_size_y / 2);
  middle_left_end_y = middle_left_end_y > static_cast<int>(map_node_size_y) - 1
                          ? static_cast<int>(map_node_size_y) - 1
                          : middle_left_end_y;
  node_zone.clear();
  node_zone.push_back(middle_left_start_x);
  node_zone.push_back(middle_left_start_y);
  node_zone.push_back(middle_left_end_x);
  node_zone.push_back(middle_left_end_y);
  map_nodes_zones.push_back(node_zone);

  int middle_center_start_x = 0;
  int middle_center_start_y = 0;
  int middle_center_end_x = top_center_end_x;
  int middle_center_end_y = middle_left_end_y;
  node_zone.clear();
  node_zone.push_back(middle_center_start_x);
  node_zone.push_back(middle_center_start_y);
  node_zone.push_back(middle_center_end_x);
  node_zone.push_back(middle_center_end_y);
  map_nodes_zones.push_back(node_zone);

  int middle_right_start_x = 0;
  int middle_right_start_y = 0;
  int middle_right_end_x = top_right_end_x;
  int middle_right_end_y = middle_center_end_y;
  node_zone.clear();
  node_zone.push_back(middle_right_start_x);
  node_zone.push_back(middle_right_start_y);
  node_zone.push_back(middle_right_end_x);
  node_zone.push_back(middle_right_end_y);
  map_nodes_zones.push_back(node_zone);

  int bottom_left_start_x = top_left_start_x;
  int bottom_left_start_y = 0;
  int bottom_left_end_x = top_left_end_x;
  int bottom_left_end_y = 2 * static_cast<int>(filter_size_y / 2) -
                          (top_left_end_y - top_left_start_y + 1) - 1;
  node_zone.clear();
  node_zone.push_back(bottom_left_start_x);
  node_zone.push_back(bottom_left_start_y);
  node_zone.push_back(bottom_left_end_x);
  node_zone.push_back(bottom_left_end_y);
  map_nodes_zones.push_back(node_zone);

  int bottom_center_start_x = middle_center_start_x;
  int bottom_center_start_y = bottom_left_start_y;
  int bottom_center_end_x = middle_center_end_x;
  int bottom_center_end_y = bottom_left_end_y;
  node_zone.clear();
  node_zone.push_back(bottom_center_start_x);
  node_zone.push_back(bottom_center_start_y);
  node_zone.push_back(bottom_center_end_x);
  node_zone.push_back(bottom_center_end_y);
  map_nodes_zones.push_back(node_zone);

  int bottom_right_start_x = middle_right_start_x;
  int bottom_right_start_y = bottom_left_start_y;
  int bottom_right_end_x = middle_right_end_x;
  int bottom_right_end_y = bottom_left_end_y;
  node_zone.clear();
  node_zone.push_back(bottom_right_start_x);
  node_zone.push_back(bottom_right_start_y);
  node_zone.push_back(bottom_right_end_x);
  node_zone.push_back(bottom_right_end_y);
  map_nodes_zones.push_back(node_zone);

  std::vector<int> start_x_indices(3, 0);
  std::vector<int> start_y_indices(3, 0);
  start_x_indices[0] = 0;
  start_x_indices[1] = top_left_end_x - top_left_start_x + 1;
  start_x_indices[2] =
      start_x_indices[1] + top_center_end_x - top_center_start_x + 1;
  start_y_indices[0] = 0;
  start_y_indices[1] = top_left_end_y - top_left_start_y + 1;
  start_y_indices[2] =
      start_y_indices[1] + middle_center_end_y - middle_center_start_y + 1;

  std::vector<std::pair<int, int>> composed_map_indices;
  for (int y = 0; y < 3; ++y) {
    for (int x = 0; x < 3; ++x) {
      composed_map_indices.push_back(
          std::make_pair(start_x_indices[x], start_y_indices[y]));
    }
  }

  Eigen::Vector2d coord2d_center =
      map_node_lt->GetCoordinate(map_node_size_x / 2, map_node_size_y / 2);

  cell_map_.clear();
  Eigen::Affine3d transform = inverse_transform.inverse();
  Eigen::Vector3d R_inv_t = -transform.translation();
  // Calculate left top corner
  map_left_top_corner_ = Eigen::Vector3d::Zero();
  map_left_top_corner_.block<2, 1>(0, 0) = map_node_lt->GetLeftTopCorner();
  map_left_top_corner_ += R_inv_t;

  for (int y = 0; y < 3; ++y) {
    for (int x = 0; x < 3; ++x) {
      if (map_nodes_zones[y * 3 + x][2] - map_nodes_zones[y * 3 + x][0] >= 0 &&
          map_nodes_zones[y * 3 + x][3] - map_nodes_zones[y * 3 + x][1] >= 0) {
        // get map node
        NdtMapNode* map_node_ptr = nullptr;
        if (x == 0 && y == 0) {
          map_node_ptr = map_node_lt;
        } else {
          Eigen::Vector2d coord2d_xy;
          coord2d_xy[0] =
              coord2d_center[0] + static_cast<double>(x * map_node_size_x) *
                                      static_cast<double>(map_pixel_resolution);
          coord2d_xy[1] =
              coord2d_center[1] + static_cast<double>(y * map_node_size_y) *
                                      static_cast<double>(map_pixel_resolution);

          MapNodeIndex map_id = MapNodeIndex::GetMapNodeIndex(
              map_.GetMapConfig(), coord2d_xy, resolution_id, zone_id);
          NdtMapNode* map_node_xy =
              dynamic_cast<NdtMapNode*>(map_.GetMapNodeSafe(map_id));
          assert(map_.IsMapNodeExist(map_id));
          map_node_ptr = map_node_xy;
        }

        // get map matrix
        NdtMapMatrix& map_cells =
            dynamic_cast<NdtMapMatrix&>(map_node_ptr->GetMapCellMatrix());

        // start obtain cells in MapNdtMatrix
        const Eigen::Vector2d& left_top_corner =
            map_node_ptr->GetLeftTopCorner();
        double resolution = map_node_ptr->GetMapResolution();
        double resolution_z = map_node_ptr->GetMapResolutionZ();
        for (int map_y = map_nodes_zones[y * 3 + x][1];
             map_y <= map_nodes_zones[y * 3 + x][3]; ++map_y) {
          for (int map_x = map_nodes_zones[y * 3 + x][0];
               map_x <= map_nodes_zones[y * 3 + x][2]; ++map_x) {
            const NdtMapCells& cell_ndt = map_cells.GetMapCell(map_y, map_x);
            if (cell_ndt.cells_.size() > 0) {
              for (auto it = cell_ndt.cells_.begin();
                   it != cell_ndt.cells_.end(); ++it) {
                unsigned int cell_count = it->second.count_;
                if (cell_count >= 6) {
                  Leaf leaf;
                  leaf.nr_points_ = static_cast<int>(cell_count);

                  Eigen::Vector3d eigen_point(Eigen::Vector3d::Zero());
                  eigen_point(0) = left_top_corner[0] + map_x * resolution +
                                   it->second.centroid_[0];
                  eigen_point(1) = left_top_corner[1] + map_y * resolution +
                                   it->second.centroid_[1];
                  eigen_point(2) =
                      resolution_z * it->first + it->second.centroid_[2];
                  leaf.mean_ = (eigen_point + R_inv_t);
                  if (it->second.is_icov_available_ == 1) {
                    leaf.icov_ = it->second.centroid_icov_.cast<double>();
                  } else {
                    leaf.nr_points_ = -1;
                  }

                  cell_map_.push_back(leaf);
                }
              }
            }
          }
        }
      }
    }
  }

  timer.End("Compose map cells.");
}

}  // namespace ndt
}  // namespace localization
}  // namespace century
