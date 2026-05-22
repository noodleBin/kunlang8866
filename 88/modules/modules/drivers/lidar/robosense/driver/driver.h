/******************************************************************************
 * Copyright 2020 The Century Authors. All Rights Reserved.
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
#include <memory>
#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include "modules/drivers/lidar/proto/config.pb.h"
#include "modules/drivers/lidar/proto/robosense.pb.h"
#include "modules/drivers/lidar/proto/robosense_config.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"

#include "cyber/cyber.h"
#include "cyber/transport/shm/shm_queue.h"
#include "modules/drivers/lidar/common/driver_factory/driver_base.h"
#include "modules/drivers/lidar/robosense/decoder/decoder_16.hpp"
#include "modules/drivers/lidar/robosense/decoder/decoder_factory.hpp"
#include "modules/drivers/lidar/robosense/driver/utility.h"
#include "modules/drivers/lidar/robosense/input/input.h"

#include "modules/drivers/lidar/robosense/rs_driver/src/rs_driver/api/lidar_driver.hpp"

#ifdef ENABLE_PCL_POINTCLOUD
#include "modules/drivers/lidar/robosense/rs_driver/src/rs_driver/msg/pcl_point_cloud_msg.hpp"
#else
#include "modules/drivers/lidar/robosense/rs_driver/src/rs_driver/msg/point_cloud_msg.hpp"
#endif

#define PKT_DATA_LENGTH 1248


typedef PointXYZIRT PointT;
typedef PointCloudT<PointT> PointCloudMsg;

// 
using namespace robosense::lidar;

namespace century {
namespace drivers {
namespace robosense {
  class RobosenseDriver  : public lidar::LidarDriver {
  public:
    RobosenseDriver(const std::shared_ptr<cyber::Node> &node,
      const ::century::drivers::lidar::config &config)
  : node_(node), conf_(config.robosense()) {}  

    RobosenseDriver(const std::shared_ptr<cyber::Node> &node, const Config &conf)
        : node_(node), conf_(conf) {}
    bool Init();
    bool Start();
    void Stop();
  
  protected:
    std::shared_ptr<PointCloudMsg> DriverGetPointCloudFromCallerCallback();
    void DriverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg);
    void ProcessCloud();
    void ProcessPackedCloud();

    void ExceptionCallback(const Error& code);
    LidarType ConvertToLidarType(const std::string& lidar_type);

  private:
      void InitParam();
      void InitDecoder();
      void InitInput();
  private:
    std::unique_ptr<cyber::transport::shm_queue<PointCloudShm>> shm_queue_;
    std::shared_ptr<cyber::Writer<PointXYZIRTCloud>> pointcloud_writer_ = nullptr;
    std::shared_ptr<cyber::Writer<PointCloudPacked>> pointpackedcloud_writer_ = nullptr;
    std::shared_ptr<cyber::Node> node_ = nullptr;
    Config conf_;
    std::string name_;
    ::robosense::lidar::LidarDriver<PointCloudMsg> driver_;
    bool to_exit_process_;
    std::thread cloud_handle_thread_;
    std::atomic<uint64_t> cloud_pool_miss_count_ = {0};
    SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue_;
    SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue_;
  };
}
}
}


