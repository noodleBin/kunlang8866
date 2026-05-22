/******************************************************************************
 * Copyright 2021 The Century Authors. All Rights Reserved.
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
#pragma once

#include <string>
#include <memory>

#include "cyber/common/macros.h"
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/perception/lidar/common/lidar_frame.h"

#define POINTCLOUD_PACKED_DEFINITION

#ifdef POINTCLOUD_PACKED_DEFINITION
using PointCloudInType = century::drivers::PointCloudPacked;
#else
using PointCloudInType = century::drivers::PointXYZIRTCloud;
#endif

namespace century {
namespace perception {
namespace lidar {

struct PointCloudPreprocessorInitOptions {
  std::string sensor_name = "velodyne64";
};

struct PointCloudPreprocessorOptions {
  Eigen::Affine3d sensor2novatel_extrinsics;
  Eigen::Affine3d sensor2vehicle_extrinsics;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

class BasePointCloudPreprocessor {
 public:
  BasePointCloudPreprocessor() = default;

  virtual ~BasePointCloudPreprocessor() = default;

  virtual bool Init(const PointCloudPreprocessorInitOptions& options =
                PointCloudPreprocessorInitOptions()) = 0;

  // @brief: preprocess point cloud
  // @param [in]: options
  // @param [in]: point cloud message
  // @param [in/out]: frame
  // cloud should be filled, required,
  virtual bool Preprocess(
      const PointCloudPreprocessorOptions& options,
      const std::shared_ptr<century::drivers::PointCloud const>& message,
      LidarFrame* frame) const = 0;
  
  /**
   * @brief 
   * 
   * @param options 
   * @param message 
   * @param frame 
   * @return true 
   * @return false 
   */
  virtual bool Preprocess(const PointCloudPreprocessorOptions& options,
    const std::shared_ptr<PointCloudInType const>& message,
    LidarFrame* frame) {return true;}

  // @brief: preprocess point cloud
  // @param [in/out]: frame
  // cloud should be filled, required,
  virtual bool Preprocess(const PointCloudPreprocessorOptions& options,
                  LidarFrame* frame) const = 0;

  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BasePointCloudPreprocessor);
};  // class BasePointCloudPreprocessor

PERCEPTION_REGISTER_REGISTERER(BasePointCloudPreprocessor);
#define PERCEPTION_REGISTER_POINTCLOUDPREPROCESSOR(name) \
  PERCEPTION_REGISTER_CLASS(BasePointCloudPreprocessor, name)

}  // namespace lidar
}  // namespace perception
}  // namespace century
