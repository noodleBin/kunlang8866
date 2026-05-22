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
#include "modules/localization/common/io/config_parser.h"

namespace century {
namespace common {
namespace io {

bool ConfigParser::LoadInsConfig(const std::string& file_path,
                                 Eigen::Vector3d* gravity_acc,
                                 Eigen::Vector3d* gyro_noise,
                                 Eigen::Vector3d* acc_noise,
                                 mmath::SE3* Tx_veh_imu,
                                 mmath::SE3* Tx_imu_gnss) {
  YAML::Node config = YAML::LoadFile(file_path);

  if (config["gravity_acc"] && gravity_acc != nullptr) {
    double x = config["gravity_acc"]["x"].as<double>();
    double y = config["gravity_acc"]["y"].as<double>();
    double z = config["gravity_acc"]["z"].as<double>();
    *gravity_acc = Eigen::Vector3d(x, y, z);
  }

  if (config["gyro_noise"] && gyro_noise != nullptr) {
    double x = config["gyro_noise"]["x"].as<double>();
    double y = config["gyro_noise"]["y"].as<double>();
    double z = config["gyro_noise"]["z"].as<double>();
    *gyro_noise = Eigen::Vector3d(x, y, z);
  }

  if (config["acc_noise"] && acc_noise != nullptr) {
    double x = config["acc_noise"]["x"].as<double>();
    double y = config["acc_noise"]["y"].as<double>();
    double z = config["acc_noise"]["z"].as<double>();
    *acc_noise = Eigen::Vector3d(x, y, z);
  }

  if (config["tf_vehicle_imu"]) {
    double x = config["tf_vehicle_imu"]["translation"]["x"].as<double>();
    double y = config["tf_vehicle_imu"]["translation"]["y"].as<double>();
    double z = config["tf_vehicle_imu"]["translation"]["z"].as<double>();

    Eigen::Vector3d translation = Eigen::Vector3d(x, y, z);
    Tx_veh_imu->setTranslation(translation);

    double roll = config["tf_vehicle_imu"]["rotation"]["roll"].as<double>();
    double pitch = config["tf_vehicle_imu"]["rotation"]["pitch"].as<double>();
    double yaw = config["tf_vehicle_imu"]["rotation"]["yaw"].as<double>();
    mmath::SO3 rot = mmath::SO3::fromEulerYPR(
        Eigen::Vector3d(yaw, pitch, roll) * mmath::kDegToRad);
    Tx_veh_imu->setSO3(rot);

    AINFO << "[calib] Tx_veh_imu trans:"
          << Tx_veh_imu->getTranslation().transpose() << ", ypr:"
          << Tx_veh_imu->getSO3().getEulerYPR().transpose() * mmath::kRadToDeg;
    return true;
  }

  return false;
}

bool ConfigParser::LoadLidarConfig(const std::string& file_path,
                                   mmath::SE3* Tx_veh_lidar1,
                                   mmath::SE3* Tx_imu_lidar1,
                                   mmath::SE3* Tx_veh_lidar2,
                                   mmath::SE3* Tx_imu_lidar2) {
  YAML::Node config = YAML::LoadFile(file_path);

  if (config["lidar1"]) {
    if (config["lidar1"]["tf_vehicle_lidar"]) {
      double qw =
          config["lidar1"]["tf_vehicle_lidar"]["rotation"]["w"].as<double>();
      double qx =
          config["lidar1"]["tf_vehicle_lidar"]["rotation"]["x"].as<double>();
      double qy =
          config["lidar1"]["tf_vehicle_lidar"]["rotation"]["y"].as<double>();
      double qz =
          config["lidar1"]["tf_vehicle_lidar"]["rotation"]["z"].as<double>();

      mmath::SO3 rot(Eigen::Quaterniond(qw, qx, qy, qz));
      Tx_veh_lidar1->setSO3(rot);

      double x =
          config["lidar1"]["tf_vehicle_lidar"]["translation"]["x"].as<double>();
      double y =
          config["lidar1"]["tf_vehicle_lidar"]["translation"]["y"].as<double>();
      double z =
          config["lidar1"]["tf_vehicle_lidar"]["translation"]["z"].as<double>();
      Eigen::Vector3d translation = Eigen::Vector3d(x, y, z);
      Tx_veh_lidar1->setTranslation(translation);

      AINFO << "[calib] Tx_veh_lidar1:"
            << Tx_veh_lidar1->getTranslation().transpose() << ", qwxyz:"
            << Tx_veh_lidar1->getSO3().getQuaternionWxyz().transpose();
    } else {
      AERROR << "No lidar1 tf_vehicle_lidar in config file";
      return false;
    }
    if (config["lidar1"]["tf_lidar_imu"]) {
      double qw =
          config["lidar1"]["tf_lidar_imu"]["rotation"]["w"].as<double>();
      double qx =
          config["lidar1"]["tf_lidar_imu"]["rotation"]["x"].as<double>();
      double qy =
          config["lidar1"]["tf_lidar_imu"]["rotation"]["y"].as<double>();
      double qz =
          config["lidar1"]["tf_lidar_imu"]["rotation"]["z"].as<double>();

      mmath::SO3 rot(Eigen::Quaterniond(qw, qx, qy, qz));
      //   Tx_imu_lidar1->setSO3(rot);

      double x =
          config["lidar1"]["tf_lidar_imu"]["translation"]["x"].as<double>();
      double y =
          config["lidar1"]["tf_lidar_imu"]["translation"]["y"].as<double>();
      double z =
          config["lidar1"]["tf_lidar_imu"]["translation"]["z"].as<double>();
      Eigen::Vector3d translation = Eigen::Vector3d(x, y, z);
      *Tx_imu_lidar1 = mmath::SE3(rot, translation).inverse();
      Tx_imu_lidar1->setTranslation(translation);

      AINFO << "[calib] Tx_imu_lidar1:"
            << Tx_imu_lidar1->getTranslation().transpose() << ", qwxyz:"
            << Tx_imu_lidar1->getSO3().getQuaternionWxyz().transpose();
    } else {
      AERROR << "No lidar1 tf_lidar_imu in config file";
      return false;
    }
  }

  if (config["lidar2"]) {
    if (config["lidar2"]["tf_vehicle_lidar"]) {
      double qw =
          config["lidar2"]["tf_vehicle_lidar"]["rotation"]["w"].as<double>();
      double qx =
          config["lidar2"]["tf_vehicle_lidar"]["rotation"]["x"].as<double>();
      double qy =
          config["lidar2"]["tf_vehicle_lidar"]["rotation"]["y"].as<double>();
      double qz =
          config["lidar2"]["tf_vehicle_lidar"]["rotation"]["z"].as<double>();

      mmath::SO3 rot(Eigen::Quaterniond(qw, qx, qy, qz));
      Tx_veh_lidar2->setSO3(rot);

      double x =
          config["lidar2"]["tf_vehicle_lidar"]["translation"]["x"].as<double>();
      double y =
          config["lidar2"]["tf_vehicle_lidar"]["translation"]["y"].as<double>();
      double z =
          config["lidar2"]["tf_vehicle_lidar"]["translation"]["z"].as<double>();
      Eigen::Vector3d translation = Eigen::Vector3d(x, y, z);
      Tx_veh_lidar2->setTranslation(translation);

      AINFO << "[calib] Tx_veh_lidar2:"
            << Tx_veh_lidar2->getTranslation().transpose() << ", qwxyz:"
            << Tx_veh_lidar2->getSO3().getQuaternionWxyz().transpose();
    } else {
      AERROR << "No lidar2 tf_vehicle_lidar in config file";
      return false;
    }
    if (config["lidar2"]["tf_lidar_imu"]) {
      double qw =
          config["lidar2"]["tf_lidar_imu"]["rotation"]["w"].as<double>();
      double qx =
          config["lidar2"]["tf_lidar_imu"]["rotation"]["x"].as<double>();
      double qy =
          config["lidar2"]["tf_lidar_imu"]["rotation"]["y"].as<double>();
      double qz =
          config["lidar2"]["tf_lidar_imu"]["rotation"]["z"].as<double>();

      mmath::SO3 rot(Eigen::Quaterniond(qw, qx, qy, qz));
      //   Tx_imu_lidar2->setSO3(rot);

      double x =
          config["lidar2"]["tf_lidar_imu"]["translation"]["x"].as<double>();
      double y =
          config["lidar2"]["tf_lidar_imu"]["translation"]["y"].as<double>();
      double z =
          config["lidar2"]["tf_lidar_imu"]["translation"]["z"].as<double>();
      Eigen::Vector3d translation = Eigen::Vector3d(x, y, z);
      *Tx_imu_lidar2 = mmath::SE3(rot, translation).inverse();
      Tx_imu_lidar2->setTranslation(translation);

      AINFO << "[calib] Tx_imu_lidar2:"
            << Tx_imu_lidar2->getTranslation().transpose() << ", qwxyz:"
            << Tx_imu_lidar2->getSO3().getQuaternionWxyz().transpose();
    } else {
      AERROR << "No lidar2 tf_lidar_imu in config file";
      return false;
    }
  }
  return true;
}

bool ConfigParser::LoadNdtMapConfig(const std::string& ndt_map_path,
                                    NdtMapConfig* ndt_map_config) {
  localization::msf::pyramid_map::BaseMapConfig base_map_config;
  std::string config_path = ndt_map_path + "/config.xml";
  if (!base_map_config.Load(config_path)) return false;
  ndt_map_config->ndt_map_version = base_map_config.map_version_;
  ndt_map_config->coordinate_type = base_map_config.coordinate_type_;
  ndt_map_config->map_node_size_x = base_map_config.map_node_size_x_;
  ndt_map_config->map_node_size_y = base_map_config.map_node_size_y_;
  ndt_map_config->map_min_x = base_map_config.map_range_.GetMinX();
  ndt_map_config->map_min_y = base_map_config.map_range_.GetMinY();
  ndt_map_config->map_max_x = base_map_config.map_range_.GetMaxX();
  ndt_map_config->map_max_y = base_map_config.map_range_.GetMaxY();
  ndt_map_config->map_is_compression = base_map_config.map_is_compression_;
  ndt_map_config->map_resolution = base_map_config.map_resolutions_[0];
  ndt_map_config->map_ground_height_offset =
      base_map_config.map_ground_height_offset_;
  return true;
}
}  // namespace io
}  // namespace common
}  // namespace century
