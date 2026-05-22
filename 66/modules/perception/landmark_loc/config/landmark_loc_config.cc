#include "config/landmark_loc_config.h"
#include <sys/stat.h>
#include <assert.h>
#include <glog/logging.h>
#include <iostream>
namespace landmark_loc{

LandmarkLocConfig::LandmarkLocConfig() {
   
  struct stat buffer;
  if(stat(config_file_.c_str(), &buffer) != 0) {
    std::cout <<" config file "<< config_file_ <<" not exist ";
    return;
  }
  (void)(buffer);
  YAML::Node node = YAML::LoadFile(config_file_);

  for(const auto& iter : node["camera_type"]) {
    camera_types_.insert(iter.as<int>());
  }

  for(const auto& iter : node["lidar_type"]) {
    lidar_types_.insert(iter.as<int>());
  }

  res_folder_ = node["res_folder"].as<std::string>();
  debug_folder_ = node["debug_folder"].as<std::string>();
  max_particle_num_ =  node["max_particle_num"].as<int>();
  min_particle_num_ = node["min_particle_num"].as<int>();
  z_hit_ = node["z_hit"].as<double>();
  int i=0;
  for(const auto& iter : node["particle_cov"]) {
    particle_cov_[i++] = iter.as<double>();
  }
  z_rand_ = node["z_rand"].as<double>();
  sigma_hit_ = node["sigma_hit"].as<double>();
  segmentation_thres_ = node["segmentation_thres"].as<double>();
  fused_map_dump_ = node["fused_map_dump"].as<int>();
  visualize_ = node["visualize"].as<int>();
  recorder_amcl_ = node["recorder_amcl"].as<int>();
  recorder_predicted_ = node["recorder_predicted"].as<int>();
  enable_odom_receive_ = node["enable_odom_receive"].as<int>();
  if (node["enable_pose_safety_check"]) {
    enable_pose_safety_check_ = node["enable_pose_safety_check"].as<int>();
  }
  if (node["safety_dis"]) {
    safety_dis_ = node["safety_dis"].as<double>();
  }
  if (node["force_update"]) {
    force_update_ = node["force_update"].as<int>();
  }
  if (node["use_delayed_ekf"]) {
    use_delayed_ekf_ = node["use_delayed_ekf"].as<int>();
  }
  if (node["using_imu_chassis"]) {
    using_imu_chassis_ = node["using_imu_chassis"].as<int>();
  }
  if (node["delayed_ekf_initial_cov"]) {
    int i = 0;
    for (const auto& iter : node["delayed_ekf_initial_cov"]) {
      if (i >= static_cast<int>(delayed_ekf_initial_cov_.size())) {
        break;
      }
      delayed_ekf_initial_cov_[i++] = iter.as<double>();
    }
  }
  if (node["delayed_ekf_control_cov"]) {
    int i = 0;
    for (const auto& iter : node["delayed_ekf_control_cov"]) {
      if (i >= static_cast<int>(delayed_ekf_control_cov_.size())) {
        break;
      }
      delayed_ekf_control_cov_[i++] = iter.as<double>();
    }
  }

  LOG(INFO) << "LandmarkLocConfig: config_file=" << config_file_
            << ", camera_types=" << camera_types_.size()
            << ", lidar_types=" << lidar_types_.size()
            << ", res_folder=" << res_folder_
            << ", debug_folder=" << debug_folder_
            << ", max_particle_num=" << max_particle_num_
            << ", min_particle_num=" << min_particle_num_
            << ", z_hit=" << z_hit_
            << ", particle_cov=[" << particle_cov_[0] << ", "
            << particle_cov_[1] << ", " << particle_cov_[2] << "]"
            << ", z_rand=" << z_rand_
            << ", sigma_hit=" << sigma_hit_
            << ", segmentation_thres=" << segmentation_thres_
            << ", fused_map_dump=" << fused_map_dump_
            << ", visualize=" << visualize_
            << ", recorder_amcl=" << recorder_amcl_
            << ", recorder_predicted=" << recorder_predicted_
            << ", enable_odom_receive=" << enable_odom_receive_
            << ", enable_pose_safety_check=" << enable_pose_safety_check_
            << ", safety_dis=" << safety_dis_
            << ", force_update=" << force_update_
            << ", use_delayed_ekf=" << use_delayed_ekf_
            << ", using_imu_chassis=" << using_imu_chassis_
            << ", delayed_ekf_initial_cov=[" << delayed_ekf_initial_cov_[0]
            << ", " << delayed_ekf_initial_cov_[1] << ", "
            << delayed_ekf_initial_cov_[2] << "]"
            << ", delayed_ekf_control_cov=[" << delayed_ekf_control_cov_[0]
            << ", " << delayed_ekf_control_cov_[1] << ", "
            << delayed_ekf_control_cov_[2] << "]";



}
}
