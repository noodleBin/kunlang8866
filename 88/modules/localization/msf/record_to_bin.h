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

#include "modules/localization/proto/imu.pb.h"
#include "cyber/common/log.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/time/clock.h"
#include "cyber/time/rate.h"
#include "cyber/timer/timer.h"
#include <Eigen/Core>
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"

using namespace std;

namespace century {
namespace loc {

struct MyPointXYZIRT {
  uint32_t intensity;
  uint32_t ring;
  double timestamp;
  float z;
  float x;
  float y;
};

struct MyPointXYZIRTCloud {
  //vector<MyPointXYZIRT> point;
  bool is_dense;
  uint32_t width;
  double measuretime;
  uint32_t height;
};

struct MyImuData {
  double timestamp;
  double ax;
  double ay;
  double az;
  double gx;
  double gy;
  double gz;
};

struct MyGnssData {
  double timestamp;
  double latitude;
  double longitute;
  double altitude;
  float latitude_std;
  float longitute_std;
  float altitude_std;
  float heading;
  float pitch;
  float roll;
  float heading_std;
  float pitch_std;
  float roll_std;
};

class RecordToBin final
    : public cyber::Component<localization::CorrectedImu> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RecordToBin();
  ~RecordToBin();

  bool Init() override;
  bool Proc(const std::shared_ptr<localization::CorrectedImu>& msg) override;

 private:
  void LeftLidarCallback(
      const std::shared_ptr<drivers::PointCloudPacked>& cloud_msg);
  void RightLidarCallback(
      const std::shared_ptr<drivers::PointCloudPacked>& cloud_msg);
  void InsPvaCallback(const std::shared_ptr<drivers::gnss::Insx>& msg);

  std::shared_ptr<cyber::Reader<drivers::PointCloudPacked>>
      left_lidar_listener_ = nullptr;
  std::shared_ptr<cyber::Reader<drivers::PointCloudPacked>>
      right_lidar_listener_ = nullptr;
  std::shared_ptr<cyber::Reader<drivers::gnss::Insx>> ins_pva_listener_ =
      nullptr;
  std::unique_ptr<std::ofstream> left_lidar_file_;
  std::unique_ptr<std::ofstream> right_lidar_file_;
  std::unique_ptr<std::ofstream> imu_file_;
  std::unique_ptr<std::ofstream> gnss_file_;
};

CYBER_REGISTER_COMPONENT(RecordToBin);

}  // namespace loc
}  // namespace century