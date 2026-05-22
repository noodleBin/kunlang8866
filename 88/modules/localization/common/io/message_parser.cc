/******************************************************************************
 * Copyright 2024 The Century Authors. All Rights Reserved.
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
#include "modules/localization/common/io/message_parser.h"

#include "cyber/time/clock.h"

namespace century {
namespace common {
namespace io {

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

bool MessageParser::CheckInsStatus(
    const std::shared_ptr<drivers::gnss::Insx>& msg, double* first_ins_ok_time,
    double ins_pos_thresh, double ins_atti_thresh) {
  static int k = 0;
  Eigen::Vector3d std_llh, std_ypr;
  std_llh << msg->position_std().x(), msg->position_std().y(),
      msg->position_std().z();
  std_ypr << msg->euler_angles_std().z(), msg->euler_angles_std().y(),
      msg->euler_angles_std().x();

  // check
  double now = msg->header().timestamp_sec();
  if ((56 == msg->pos_type()) && (std_ypr(0) < ins_atti_thresh)) {
    if (0 == *first_ins_ok_time) {
      *first_ins_ok_time = now;
    } else if (now - *first_ins_ok_time >= 5.0) {
      AINFO << "ins is OK";
      return true;
    }
  } else {
    *first_ins_ok_time = 0;
    if (0 == (++k) % 100) {
      AINFO << "waiting for ins being OK...";
    }
  }

  return false;
}

void MessageParser::ParseInsxMsg(
    const std::shared_ptr<drivers::gnss::Insx>& msg,
    const Eigen::Vector3d& mercator_origin, double* timestamp,
    mmath::SE3* Tx_Mp_ins, Eigen::Vector3d* linear_vel,
    Eigen::Vector6d* omega_acc) {
  *timestamp = msg->header().timestamp_sec();

  // parse llh
  double lon_deg, lat_deg, height;
  lon_deg = msg->position().lon();
  lat_deg = msg->position().lat();
  height = msg->position().height();

  char zone[20] = {0};
  Eigen::Vector3d pos;
  double gamma = 0.0;
  mmath::Wgs84toUtm(lon_deg, lat_deg, &pos[0], &pos[1], nullptr, &gamma, zone);
  pos[0] -= mercator_origin[0];
  pos[1] -= mercator_origin[1];
  pos[2] = height;

  mmath::SO3 rot = mmath::SO3::fromEulerYPR(
      Eigen::Vector3d(msg->euler_angles().z() + gamma * mmath::kRadToDeg,
                      msg->euler_angles().y(), msg->euler_angles().x()) *
      mmath::kDegToRad);
  *Tx_Mp_ins = mmath::SE3(rot, pos);

  // vel
  *linear_vel << msg->linear_velocity().x(), msg->linear_velocity().y(),
      msg->linear_velocity().z();

  // imu
  *omega_acc << msg->angular_velocity().x(), msg->angular_velocity().y(),
      msg->angular_velocity().z(), msg->linear_acceleration().x(),
      msg->linear_acceleration().y(), msg->linear_acceleration().z();
}

void MessageParser::ParseLidarMsg(
    const drivers::PointXYZIRTCloud& lidar_msg,
    const std::shared_ptr<loc::PointCloudXYZIRT>& pcl_cloud) {
  CHECK_NOTNULL(pcl_cloud);
  for (const auto& point : lidar_msg.point()) {
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
        !std::isfinite(point.z())) {
      continue;
    }

    loc::PointXYZIRT pt;
    pt.x = point.x();
    pt.y = point.y();
    pt.z = point.z();
    pt.intensity = point.intensity();
    pt.ring = point.ring();
    pt.timestamp = point.timestamp();
    pcl_cloud->points.emplace_back(pt);
  }
  pcl_cloud->height = 1;
  pcl_cloud->width = pcl_cloud->size();
  pcl_cloud->is_dense = true;
}

void MessageParser::ParseLidarMsg(
    const drivers::PointCloudPacked& lidar_msg,
    const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pcl_cloud) {
  CHECK_NOTNULL(pcl_cloud);
  int size = lidar_msg.point_size();
  std::vector<PointPacked> ppd( lidar_msg.point_size());
  const std::string& raw_data = lidar_msg.data();
  std::memcpy(ppd.data(), raw_data.data(), raw_data.size());
  for (int i = 0; i < size; ++i) {
    const auto& pt = ppd[i];
    if (!std::isfinite(pt.x()) || !std::isfinite(pt.y()) ||
        !std::isfinite(pt.z())) {
      continue;
    }
    
    pcl::PointXYZ tmp_pt;
    tmp_pt.x = pt.x();
    tmp_pt.y = pt.y();
    tmp_pt.z = pt.z();
    pcl_cloud->points.emplace_back(tmp_pt);
  }
  pcl_cloud->height = 1;
  pcl_cloud->width = pcl_cloud->size();
  pcl_cloud->is_dense = true;
}

void MessageParser::ParseLidarMsg(
    const drivers::PointCloudPacked& lidar_msg,
    const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& pcl_cloud) {
  CHECK_NOTNULL(pcl_cloud);
  int size = lidar_msg.point_size();
  std::vector<PointPacked> ppd( lidar_msg.point_size());
  const std::string& raw_data = lidar_msg.data();
  std::memcpy(ppd.data(), raw_data.data(), raw_data.size());
  for (int i = 0; i < size; ++i) {
    const auto& pt = ppd[i];
    if (!std::isfinite(pt.x()) || !std::isfinite(pt.y()) ||
        !std::isfinite(pt.z())) {
      continue;
    }
    
    pcl::PointXYZI tmp_pt;
    tmp_pt.x = pt.x();
    tmp_pt.y = pt.y();
    tmp_pt.z = pt.z();
    tmp_pt.intensity = static_cast<float>(pt.intensity());
    pcl_cloud->points.emplace_back(tmp_pt);
  }
  pcl_cloud->height = 1;
  pcl_cloud->width = pcl_cloud->size();
  pcl_cloud->is_dense = true;
}

void MessageParser::ParseCorrImuMsg(
    const std::shared_ptr<localization::CorrectedImu>& msg, double* timestamp,
    Eigen::Vector3d* linear_acc, Eigen::Vector3d* ang_vel) {
  if (nullptr == msg) {
    return;
  }

  *timestamp = msg->header().timestamp_sec();

  // *linear_acc << msg->imu().linear_acceleration().x(),
  //     msg->imu().linear_acceleration().y(),
  //     msg->imu().linear_acceleration().z();

  // *ang_vel << msg->imu().angular_velocity().x(),
  //     msg->imu().angular_velocity().y(), msg->imu().angular_velocity().z();

  *linear_acc << msg->imu().linear_acceleration().y(),
      -msg->imu().linear_acceleration().x(),
      msg->imu().linear_acceleration().z();

  *ang_vel << msg->imu().angular_velocity().y(),
      -msg->imu().angular_velocity().x(), msg->imu().angular_velocity().z();
}

void MessageParser::ParseFusionMsg(
    const std::shared_ptr<localization::LocalizationEstimate>& msg,
    double* timestamp, mmath::SE3* pose, Eigen::Vector3d* vel) {
  *timestamp = msg->header().timestamp_sec();

  Eigen::Quaterniond quaternion(
      msg->pose().orientation().qw(), msg->pose().orientation().qx(),
      msg->pose().orientation().qy(), msg->pose().orientation().qz());
  Eigen::Vector3d translation(msg->pose().position().x(),
                              msg->pose().position().y(),
                              msg->pose().position().z());
  *pose = mmath::SE3(quaternion, translation);
}

void MessageParser::GetLidarPoseMsg(const double& timestamp,
                                    const std::string& frame_id,
                                    const mmath::SE3& pose,
                                    const double fitness_score,
                                    const bool is_converged,
                                    localization::PoseWithCov* pose_msg) {
  auto* header_msg = pose_msg->mutable_header();
  header_msg->set_timestamp_sec(timestamp);
  header_msg->set_frame_id(frame_id);

  const Eigen::Vector3d& t = pose.getTranslation();
  pose_msg->mutable_position()->set_x(t.x());
  pose_msg->mutable_position()->set_y(t.y());
  pose_msg->mutable_position()->set_z(t.z());
  // pose_msg->mutable_position()->set_z(9.21);

  const Eigen::Quaterniond& q = pose.getSO3().getQuaternion();
  pose_msg->mutable_orientation()->set_qw(q.w());
  pose_msg->mutable_orientation()->set_qx(q.x());
  pose_msg->mutable_orientation()->set_qy(q.y());
  pose_msg->mutable_orientation()->set_qz(q.z());

  pose_msg->set_fitness_score(fitness_score);
  pose_msg->set_is_converged(is_converged);
}

void MessageParser::GetLocMsg(
    const double& timestamp, const mmath::SE3& pose,
    const Eigen::Vector6d& omega_vel, const Eigen::Vector3d& acc,
    const localization::LocalizationEstimate::StatusType loc_status,
    localization::LocalizationEstimate* loc_msg) {
  auto* header_msg = loc_msg->mutable_header();
  header_msg->set_timestamp_sec(timestamp);
  loc_msg->set_measurement_time(cyber::Clock::NowInSeconds());
  header_msg->set_frame_id("map");
  static std::atomic<uint64_t> sequence_num = {0};
  header_msg->set_sequence_num(
      static_cast<unsigned int>(sequence_num.fetch_add(1)));

  const Eigen::Vector3d& t = pose.getTranslation();
  auto mutable_pose = loc_msg->mutable_pose();
  mutable_pose->mutable_position()->set_x(t.x());
  mutable_pose->mutable_position()->set_y(t.y());
  mutable_pose->mutable_position()->set_z(t.z());
  const Eigen::Quaterniond& q = pose.getSO3().getQuaternion();
  mutable_pose->mutable_orientation()->set_qw(q.w());
  mutable_pose->mutable_orientation()->set_qx(q.x());
  mutable_pose->mutable_orientation()->set_qy(q.y());
  mutable_pose->mutable_orientation()->set_qz(q.z());

  const Eigen::Vector3d& ypr = pose.getSO3().getEulerYPR();
  mutable_pose->mutable_euler_angles()->set_x(ypr.z());
  mutable_pose->mutable_euler_angles()->set_y(ypr.y());
  mutable_pose->mutable_euler_angles()->set_z(ypr.x());
  double heading = ypr.x();
  mutable_pose->set_heading(heading);

  mutable_pose->mutable_linear_velocity()->set_x(omega_vel(3));
  mutable_pose->mutable_linear_velocity()->set_y(omega_vel(4));
  mutable_pose->mutable_linear_velocity()->set_z(omega_vel(5));

  mutable_pose->mutable_angular_velocity()->set_x(omega_vel(0));
  mutable_pose->mutable_angular_velocity()->set_y(omega_vel(1));
  mutable_pose->mutable_angular_velocity()->set_z(omega_vel(2));
  mutable_pose->mutable_angular_velocity_vrf()->set_x(omega_vel(0));
  mutable_pose->mutable_angular_velocity_vrf()->set_y(omega_vel(1));
  mutable_pose->mutable_angular_velocity_vrf()->set_z(omega_vel(2));

  mutable_pose->mutable_linear_acceleration()->set_x(acc(0));
  mutable_pose->mutable_linear_acceleration()->set_y(acc(1));
  mutable_pose->mutable_linear_acceleration()->set_z(acc(2));
  mutable_pose->mutable_linear_acceleration_vrf()->set_x(acc(0));
  mutable_pose->mutable_linear_acceleration_vrf()->set_y(acc(1));
  mutable_pose->mutable_linear_acceleration_vrf()->set_z(acc(2));

  loc_msg->set_status_type(loc_status);
}

double MessageParser::GetMessageStamp(
    const std::shared_ptr<drivers::PointCloudPacked>& lidar_msg,
    const double& sync_thresh) {
  CHECK_NOTNULL(lidar_msg);

  // check time
  // cyber::Time(lidar_msg->measuretime()).ToMicrosecond()
  // cyber::Time(lidar_msg->header().timestamp_sec()).ToMicrosecond()
  // double timestamp = lidar_msg->header().timestamp_sec();
  double measuretime = lidar_msg->header().timestamp_sec();
  // double gap_time =
  //     measuretime -
  //     static_cast<double>(lidar_msg->point()[0].timestamp()) / 1e6;
  // if (std::fabs(gap_time) > 0.12) {  // 120ms
  //   AERROR << "## lidar_msg point gap_time too large: "
  //          << std::to_string(gap_time);
  //   // continue;
  // }
  // AINFO << "@@ gap_time:" << std::to_string(gap_time);
  // AINFO << "@@ measuretime:" << std::to_string(measuretime);

  // TODO
  // double sync_diff = timestamp - measuretime;
  // if (std::fabs(sync_diff) > sync_thresh) {
  //   AERROR << "[lidar] lidar_msg point sync_diff time too large: "
  //          << std::to_string(sync_diff);
  //   AERROR << "[lidar] timestamp: " << std::to_string(timestamp)
  //          << " measuretime: " << std::to_string(measuretime);
  //   // continue;
  // }
  // AINFO << "@@ sync_diff:" << std::to_string(sync_diff);

  // TODO(nqy): lidar timestamp
  // return timestamp;
  return measuretime;
}

}  // namespace io
}  // namespace common
}  // namespace century
