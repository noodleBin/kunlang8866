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

#include "modules/localization/msf/record_to_bin.h"

#include "record_to_bin.h"

namespace century {
namespace loc {
  
#pragma pack(push, 1)
struct PointPacked {
private:
    float x_;
    float y_;
    float z_;
    uint16_t intensity_;
    uint16_t ring_;
    double timestamp_;

public:
    // Default constructor
    PointPacked() = default;

    // Parameterized constructor
    PointPacked(float x, float y, float z,
                uint16_t intensity, uint16_t ring,
                double timestamp)
        : x_(x), y_(y), z_(z),
          intensity_(intensity), ring_(ring),
          timestamp_(timestamp) {}

    // ---- Accessors (Getters) ----
    inline float x() const noexcept { return x_; }
    inline float y() const noexcept { return y_; }
    inline float z() const noexcept { return z_; }
    inline uint16_t intensity() const noexcept { return intensity_; }
    inline uint16_t ring() const noexcept { return ring_; }
    inline double timestamp() const noexcept { return timestamp_; }

    // ---- Modifiers (Setters) ----
    inline void x(float val) noexcept { x_ = val; }
    inline void y(float val) noexcept { y_ = val; }
    inline void z(float val) noexcept { z_ = val; }
    inline void intensity(uint16_t val) noexcept { intensity_ = val; }
    inline void ring(uint16_t val) noexcept { ring_ = val; }
    inline void timestamp(double val) noexcept { timestamp_ = val; }
};
#pragma pack(pop)

RecordToBin::RecordToBin() {
}

RecordToBin::~RecordToBin() {
  if (left_lidar_file_ && left_lidar_file_->is_open()) {
      left_lidar_file_->close();
  }
  
  if (right_lidar_file_ && right_lidar_file_->is_open()) {
      right_lidar_file_->close();
  }

  if (imu_file_ && imu_file_->is_open()) {
      imu_file_->close();
  }
  
  if (gnss_file_ && gnss_file_->is_open()) {
      gnss_file_->close();
  }
}

bool RecordToBin::Init() {
  century::cyber::ReaderConfig l_reader_config;
  l_reader_config.pending_queue_size = 10000;
  l_reader_config.channel_name = "/lidar/helios/front_left";
  century::cyber::ReaderConfig r_reader_config;
  r_reader_config.pending_queue_size = 10000;
  r_reader_config.channel_name = "/lidar/helios/rear_right";
  century::cyber::ReaderConfig i_reader_config;
  i_reader_config.pending_queue_size = 10000;
  i_reader_config.channel_name = "/century/sensor/gnss/insx";

  left_lidar_listener_ = node_->CreateReader<drivers::PointCloudPacked>(
      l_reader_config,
      std::bind(&RecordToBin::LeftLidarCallback, this,
                std::placeholders::_1));
  right_lidar_listener_ = node_->CreateReader<drivers::PointCloudPacked>(
      r_reader_config,
      std::bind(&RecordToBin::RightLidarCallback, this,
                std::placeholders::_1));
  ins_pva_listener_ = node_->CreateReader<drivers::gnss::Insx>(
      i_reader_config,
      std::bind(&RecordToBin::InsPvaCallback, this, std::placeholders::_1));

  left_lidar_file_ = std::make_unique<std::ofstream>("/century/data/bag/record_to_bin/left_lidar.bin", std::ios::binary | std::ios::app);
  right_lidar_file_ = std::make_unique<std::ofstream>("/century/data/bag/record_to_bin/right_lidar.bin", std::ios::binary | std::ios::app);
  imu_file_ = std::make_unique<std::ofstream>("/century/data/bag/record_to_bin/imu.bin", std::ios::binary | std::ios::app);
  gnss_file_ = std::make_unique<std::ofstream>("/century/data/bag/record_to_bin/gnss.bin", std::ios::binary | std::ios::app);

  if (!left_lidar_file_->is_open()) {
      cout << "error opening left lidar file: " << endl;
  }
  if (!right_lidar_file_->is_open()) {
      cout << "error opening right lidar file: " << endl;
  }
  if (!imu_file_->is_open()) {
      cout << "error opening imu file: " << endl;
  }
  if (!gnss_file_->is_open()) {
      cout << "error opening gnss file: " << endl;
  }
  return true;
}

bool RecordToBin::Proc(
    const std::shared_ptr<localization::CorrectedImu>& msg) {
  if (nullptr == msg) {
    return false;
  }
  if (!imu_file_ || !imu_file_->is_open()) {
      return false;
  }

  MyImuData imu_data;
  imu_data.timestamp = msg->header().timestamp_sec();
  imu_data.ax = msg->imu().linear_acceleration().x();
  imu_data.ay = msg->imu().linear_acceleration().y();
  imu_data.az = msg->imu().linear_acceleration().z();
  imu_data.gx = msg->imu().angular_velocity().x();
  imu_data.gy = msg->imu().angular_velocity().y();
  imu_data.gz = msg->imu().angular_velocity().z();
  // cout << fixed << setprecision(6);
  // cout << "imu_data.timestamp:" << imu_data.timestamp << endl;
  // cout << "imu_data.ax:" << imu_data.ax << endl;
  // cout << "imu_data.ay:" << imu_data.ay << endl;
  // cout << "imu_data.az:" << imu_data.az << endl;
  // cout << "imu_data.gx:" << imu_data.gx << endl;
  // cout << "imu_data.gy:" << imu_data.gy << endl;
  // cout << "imu_data.gz:" << imu_data.gz << endl;
  imu_file_->write(reinterpret_cast<const char*>(&imu_data), sizeof(MyImuData));
  imu_file_->flush();
  return true;
}

void RecordToBin::LeftLidarCallback(
    const std::shared_ptr<drivers::PointCloudPacked> &lidar_msg) {
  if (nullptr == lidar_msg) {
    return;
  }
  if (!left_lidar_file_ || !left_lidar_file_->is_open()) {
      return;
  }

  MyPointXYZIRT my_p;
  MyPointXYZIRTCloud my_pc;
  int size = lidar_msg->point_size();
  left_lidar_file_->write(reinterpret_cast<const char*>(&size), sizeof(int));
  std::vector<PointPacked> ppd( lidar_msg->point_size());
  const std::string& raw_data = lidar_msg->data();
  std::memcpy(ppd.data(), raw_data.data(), raw_data.size());
  for (int i = 0; i < size; ++i) {
    const auto& pt = ppd[i];
    
    my_p.x = pt.x();
    my_p.y = pt.y();
    my_p.z = pt.z();
    my_p.intensity = static_cast<uint32_t>(pt.intensity());
    my_p.ring = static_cast<uint32_t>(pt.ring());
    my_p.timestamp = static_cast<double>(pt.timestamp()) * 1e-9;
    left_lidar_file_->write(reinterpret_cast<const char*>(&my_p), sizeof(MyPointXYZIRT));
  }
  my_pc.height = 1;
  my_pc.width = lidar_msg->point_size();
  my_pc.is_dense = true;
  my_pc.measuretime = lidar_msg->header().timestamp_sec();
  // cout << "1my_pc.measuretime:" << my_pc.measuretime << endl;

  left_lidar_file_->write(reinterpret_cast<const char*>(&my_pc), sizeof(MyPointXYZIRTCloud));
  left_lidar_file_->flush();

  // MyPointXYZIRT my_p;
  // MyPointXYZIRTCloud my_pc;
  // int size = lidar_msg->point().size();
  // left_lidar_file_->write(reinterpret_cast<const char*>(&size), sizeof(int));
  // for (const auto& point : lidar_msg->point()) {
  //   my_p.x = point.x();
  //   my_p.y = point.y();
  //   my_p.z = point.z();
  //   my_p.intensity = point.intensity();
  //   my_p.ring = point.ring();
  //   my_p.timestamp = point.timestamp();
  //   left_lidar_file_->write(reinterpret_cast<const char*>(&my_p), sizeof(MyPointXYZIRT));
  // }
  // my_pc.height = lidar_msg->height();
  // my_pc.width = lidar_msg->width();
  // my_pc.is_dense = lidar_msg->is_dense();
  // my_pc.measuretime = lidar_msg->measuretime();
  // // cout << fixed << setprecision(6);
  // // cout << "1my_pc.height:" << my_pc.height << endl;
  // // cout << "1my_pc.width:" << my_pc.width << endl;
  // // cout << "1my_pc.is_dense:" << my_pc.is_dense << endl;
  // // cout << "1my_pc.measuretime:" << my_pc.measuretime << endl;
  // // cout << "1size:" << size << endl;
  // // int index = -1;
  // // for (const auto& point : lidar_msg->point()) {
  // //   ++index;
  // //   if(index != 0 && index != size-1) {
  // //     continue;
  // //   }
  // //   // if (point.ring() == 8 || point.ring() == 21) {
  // //   //   continue;
  // //   // }
  // //   cout << "1point:" << point.x() << "," << point.y() << "," << point.z() << "," 
  // //   << point.intensity() << "," << point.ring() << "," << point.timestamp() << endl;
  // // }
  // left_lidar_file_->write(reinterpret_cast<const char*>(&my_pc), sizeof(MyPointXYZIRTCloud));
  // left_lidar_file_->flush();
}

void RecordToBin::RightLidarCallback(
    const std::shared_ptr<drivers::PointCloudPacked> &lidar_msg) {
  if (nullptr == lidar_msg) {
    return;
  }
  if (!right_lidar_file_ || !right_lidar_file_->is_open()) {
      return;
  }

  MyPointXYZIRT my_p;
  MyPointXYZIRTCloud my_pc;
  int size = lidar_msg->point_size();
  right_lidar_file_->write(reinterpret_cast<const char*>(&size), sizeof(int));
  std::vector<PointPacked> ppd( lidar_msg->point_size());
  const std::string& raw_data = lidar_msg->data();
  std::memcpy(ppd.data(), raw_data.data(), raw_data.size());
  for (int i = 0; i < size; ++i) {
    const auto& pt = ppd[i];
    
    my_p.x = pt.x();
    my_p.y = pt.y();
    my_p.z = pt.z();
    my_p.intensity = static_cast<uint32_t>(pt.intensity());
    my_p.ring = static_cast<uint32_t>(pt.ring());
    my_p.timestamp = static_cast<double>(pt.timestamp()) * 1e-9;
    right_lidar_file_->write(reinterpret_cast<const char*>(&my_p), sizeof(MyPointXYZIRT));
  }
  my_pc.height = 1;
  my_pc.width = lidar_msg->point_size();
  my_pc.is_dense = true;
  my_pc.measuretime = lidar_msg->header().timestamp_sec();
  // cout << "2my_pc.measuretime:" << my_pc.measuretime << endl;

  right_lidar_file_->write(reinterpret_cast<const char*>(&my_pc), sizeof(MyPointXYZIRTCloud));
  right_lidar_file_->flush();

  // MyPointXYZIRT my_p;
  // MyPointXYZIRTCloud my_pc;
  // int size = lidar_msg->point().size();
  // right_lidar_file_->write(reinterpret_cast<const char*>(&size), sizeof(int));
  // for (const auto& point : lidar_msg->point()) {
  //   my_p.x = point.x();
  //   my_p.y = point.y();
  //   my_p.z = point.z();
  //   my_p.intensity = point.intensity();
  //   my_p.ring = point.ring();
  //   my_p.timestamp = point.timestamp();
  //   right_lidar_file_->write(reinterpret_cast<const char*>(&my_p), sizeof(MyPointXYZIRT));
  // }
  // my_pc.height = lidar_msg->height();
  // my_pc.width = lidar_msg->width();
  // my_pc.is_dense = lidar_msg->is_dense();
  // my_pc.measuretime = lidar_msg->measuretime();
  // // cout << fixed << setprecision(6);
  // // cout << "2my_pc.height:" << my_pc.height << endl;
  // // cout << "2my_pc.width:" << my_pc.width << endl;
  // // cout << "2my_pc.is_dense:" << my_pc.is_dense << endl;
  // // cout << "2my_pc.measuretime:" << my_pc.measuretime << endl;
  // // cout << "2size:" << size << endl;
  // // int index = -1;
  // // for (const auto& point : lidar_msg->point()) {
  // //   ++index;
  // //   if(index != 0 && index != size-1) {
  // //     continue;
  // //   }
  // //   cout << "2point:" << point.x() << "," << point.y() << "," << point.z() << "," 
  // //   << point.intensity() << "," << point.ring() << "," << point.timestamp() << endl;
  // // }
  // right_lidar_file_->write(reinterpret_cast<const char*>(&my_pc), sizeof(MyPointXYZIRTCloud));
  // right_lidar_file_->flush();
}

void RecordToBin::InsPvaCallback(
    const std::shared_ptr<drivers::gnss::Insx>& msg) {
  if (nullptr == msg) {
    return;
  }
  if (!gnss_file_ || !gnss_file_->is_open()) {
      return;
  }

  MyGnssData gnss_data;
  gnss_data.timestamp = msg->header().timestamp_sec();
  gnss_data.longitute = msg->position().lon();
  gnss_data.latitude = msg->position().lat();
  gnss_data.altitude = msg->position().height();
  gnss_data.longitute_std = msg->position_std().x();
  gnss_data.latitude_std = msg->position_std().y();
  gnss_data.altitude_std = msg->position_std().z();
  gnss_data.roll = msg->euler_angles().x();
  gnss_data.pitch = msg->euler_angles().y();
  gnss_data.heading = msg->euler_angles().z();
  gnss_data.roll_std = msg->euler_angles_std().x();
  gnss_data.pitch_std = msg->euler_angles_std().y();
  gnss_data.heading_std = msg->euler_angles_std().z();
  // cout << fixed << setprecision(6);
  // cout << "gnss_data.timestamp:" << gnss_data.timestamp << endl;
  // cout << "gnss_data.longitute:" << gnss_data.longitute << endl;
  // cout << "gnss_data.latitude:" << gnss_data.latitude << endl;
  // cout << "gnss_data.altitude:" << gnss_data.altitude << endl;
  // cout << "gnss_data.longitute_std:" << gnss_data.longitute_std << endl;
  // cout << "gnss_data.latitude_std:" << gnss_data.latitude_std << endl;
  // cout << "gnss_data.altitude_std:" << gnss_data.altitude_std << endl;
  // cout << "gnss_data.roll:" << gnss_data.roll << endl;
  // cout << "gnss_data.pitch:" << gnss_data.pitch << endl;
  // cout << "gnss_data.heading:" << gnss_data.heading << endl;
  // cout << "gnss_data.roll_std:" << gnss_data.roll_std << endl;
  // cout << "gnss_data.pitch_std:" << gnss_data.pitch_std << endl;
  // cout << "gnss_data.heading_std:" << gnss_data.heading_std << endl;
  gnss_file_->write(reinterpret_cast<const char*>(&gnss_data), sizeof(MyGnssData));
  gnss_file_->flush();
}

}  // namespace loc
}  // namespace century
