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
#pragma once

#include <string>
#include <memory>
#include <vector>

#include "cyber/common/macros.h"
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/perception/lidar/common/lidar_frame.h"

namespace century {
namespace perception {
namespace lidar {

struct ClusterInitOptions {
  std::string sensor_name = "robosensor";
  bool enable_hdmap_input = true;
};

struct ClusterOptions {
  bool enable_hdmap_input = true;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

class BasePointCloudCluster {
 public:
  BasePointCloudCluster() = default;
  virtual ~BasePointCloudCluster() = default;

  virtual bool Init(const ClusterInitOptions& options =
                        ClusterInitOptions()) = 0;

  // @brief: cluster point cloud
  // @param [in]: options
  // @param [in]: frame filled point cloud
  // @param [out]: clusters result
  virtual bool Cluster(const ClusterOptions& options,
                       LidarFrame* frame) = 0;

  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BasePointCloudCluster);
};

PERCEPTION_REGISTER_REGISTERER(BasePointCloudCluster);
#define PERCEPTION_REGISTER_POINTCLOUDCLUSTERER(name) \
  PERCEPTION_REGISTER_CLASS(BasePointCloudCluster, name)

}  // namespace lidar
}  // namespace perception
}  // namespace century