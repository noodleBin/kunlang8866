#pragma once
#include "delayed_ekf/delayed_measurement_ekf.h"
#include "yaml-cpp/yaml.h"
#include <array>
#include <string>
#include <unordered_set>
namespace landmark_loc{
class LandmarkLocConfig {
public:
static LandmarkLocConfig* GetInstance() {
  static LandmarkLocConfig cfg;
  return &cfg;
}

std::unordered_set<int> CameraTypes() const {
  return camera_types_;
}

std::unordered_set<int> LidarTypes() const {
  return lidar_types_;
}

std::string ResFolder() const {
  return res_folder_;
}

std::string DebugFolder() const {
  return debug_folder_;
}

int MaxParticleNum() const {
  return max_particle_num_;
}

int MinParticleNum() const {
  return min_particle_num_;
}

double ZHit() const {
  return z_hit_;
}

std::array<double, 3> ParticleCov() const {
  return particle_cov_;
}

double ZRand() const {
  return z_rand_;
}

double SigmaHit() const {
  return sigma_hit_;
}

double SegmentationThres() const {
  return segmentation_thres_;
}

bool FusedMapDump() const {
  return fused_map_dump_;
}

bool Visualize() const {
  return visualize_;
}

bool RecorderAmcl() const {
  return recorder_amcl_;
}

bool RecorderPredicted() const {
  return recorder_predicted_;
}

bool EnableOdomReceive() const {
  return enable_odom_receive_;
}

bool EnablePoseSafetyCheck() const {
  return enable_pose_safety_check_;
}

double SafetyDis() const {
  return safety_dis_;
}

  bool ForceUpdate() const {
    return force_update_;
  }

  bool UsingImuChassis() const {
    return using_imu_chassis_;
  }

  delayed_ekf::DelayedMeasurementEkf::Options DelayedEkfOptions() const {
    delayed_ekf::DelayedMeasurementEkf::Options options;
    options.use_delayed_ekf = use_delayed_ekf_;
    options.default_initial_covariance =
        delayed_ekf::DelayedMeasurementEkf::Vector3(
            delayed_ekf_initial_cov_[0], delayed_ekf_initial_cov_[1],
            delayed_ekf_initial_cov_[2]);
    options.default_control_covariance =
        delayed_ekf::DelayedMeasurementEkf::Vector3(
            delayed_ekf_control_cov_[0], delayed_ekf_control_cov_[1],
            delayed_ekf_control_cov_[2]);
    return options;
  }


private:
LandmarkLocConfig();
std::string res_folder_ = "/century/data/data/";
std::string debug_folder_ = "/century/data/log/";
std::unordered_set<int> camera_types_ = {
  0, //CamFrontLeft, 
  1, //CamFrontMiddle,
  2, //CamFrontRight,
 // 3, // CamRearLeft, 
 // 4, //CamRearMiddle, 
 // 5,//CamRearRight
};
std::unordered_set<int> lidar_types_ = {
 // 0 ,//HeliosFrontLeft,
 // 1 ,//HeliosRearRight,
  2, //BpFrontLeft,
  3, //BpRearRight,
 };

int max_particle_num_ = 5000;
int min_particle_num_ = 1000;
double z_hit_ = 0.99;
std::array<double, 3> particle_cov_ = {0.5,0.5,0.07};
double z_rand_ = 0.01;
double sigma_hit_ = 0.01;
double segmentation_thres_ = 0.4;
bool fused_map_dump_ = false;
bool visualize_ = true;
bool recorder_amcl_ = false;
bool recorder_predicted_ = true;
bool enable_odom_receive_ = true;
bool enable_pose_safety_check_ = false;
double safety_dis_ = 0.25;
bool force_update_ = true;
bool use_delayed_ekf_ = false;
bool using_imu_chassis_ = false;
std::array<double, 3> delayed_ekf_initial_cov_ = {1e-4, 1e-4, 7.6e-5};
std::array<double, 3> delayed_ekf_control_cov_ = {1e-2, 1e-2, 1e-4};
const std::string config_file_ = "/century/modules/perception/landmark_loc/res/landmark_loc.yaml";
};

}
