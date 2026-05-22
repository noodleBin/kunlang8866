/******************************************************************************
 * Copyright 2017 The Century Authors. All Rights Reserved.
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

/**
 * @file velodyne_utility.h
 * @brief The utilities of velodyne.
 */

#pragma once

#include <string>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include "Eigen/Geometry"

#include "modules/common/util/eigen_defs.h"

namespace {
constexpr double UTM_offset_x = 250932.851957;
constexpr double UTM_offset_y = 3987498.593868;
constexpr double UTM_offset_z = 0.0;
};  // namespace

namespace century {
namespace localization {
namespace msf {

namespace velodyne {

struct VelodyneFrame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**@brief The frame index. */
  unsigned int frame_index;
  /**@brief The time stamp. */
  double timestamp;
  /**@brief The 3D point cloud in this frame. */
  ::century::common::EigenVector3dVec pt3ds;
  /**@brief The laser reflection values in this frames. */
  std::vector<unsigned char> intensities;
  /**@brief The laser IDs. */
  std::vector<unsigned char> laser_ids;
  /**@brief The pose of the frame. */
  Eigen::Affine3d pose;
};

struct KeyFramePose {
  double timestamp;
  float x, y, z;
  float roll, pitch, yaw;
};

void LoadPcds(const std::string& file_path, const unsigned int frame_index,
              const Eigen::Affine3d& pose, VelodyneFrame* velodyne_frame,
              const bool is_global = false);

void LoadPcds(const std::string& file_path, const unsigned int frame_index,
              const Eigen::Affine3d& pose,
              ::century::common::EigenVector3dVec* pt3ds,
              std::vector<unsigned char>* intensities, bool is_global = false);

/**@brief Load the PCD poses with their timestamps. */
void LoadPcdPoses(const std::string& file_path,
                  ::century::common::EigenAffine3dVec* poses,
                  std::vector<double>* timestamps);

/**@brief Load the PCD poses with their timestamps and indices. */
void LoadPcdPoses(const std::string& file_path,
                  ::century::common::EigenAffine3dVec* poses,
                  std::vector<double>* timestamps,
                  std::vector<unsigned int>* pcd_indices);

/**@brief Load poses and stds their timestamps. */
void LoadPosesAndStds(const std::string& file_path,
                      ::century::common::EigenAffine3dVec* poses,
                      ::century::common::EigenVector3dVec* stds,
                      std::vector<double>* timestamps);

void LoadPcdPosesFromCsv(const std::string& file_path,
                         ::century::common::EigenAffine3dVec* poses,
                         std::vector<double>* timestamps);

// /**@brief Save the PCD poses with their timestamps. */
// void save_pcd_poses(std::string file_path,
//    const ::century::common::EigenAffine3dVec& poses,
//    const std::vector<double>& timestamps);

/**@brief Load the velodyne extrinsic from a YAML file. */
bool LoadExtrinsic(const std::string& file_path, Eigen::Affine3d* extrinsic);

}  // namespace velodyne
}  // namespace msf
}  // namespace localization
}  // namespace century
