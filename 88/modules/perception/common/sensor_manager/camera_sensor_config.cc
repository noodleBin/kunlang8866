// camera_sensor_config.cc
#include "modules/perception/common/sensor_manager/camera_sensor_config.h"

namespace century {
namespace perception {
namespace common {

std::once_flag CameraSensorConfig::init_flag_;

CameraSensorConfig& CameraSensorConfig::GetInstance() {
  static CameraSensorConfig instance;
  return instance;
}

bool CameraSensorConfig::Initialize(const std::string& yaml_file) {
  bool success = false;
  std::call_once(init_flag_, [this, &yaml_file, &success]() {
    success = TryLoadCameraConfig(yaml_file);
    if (success) {
      initialized_ = true;
      AERROR << "CameraSensorConfig initialized successfully.";
    } else {
      AERROR << "CameraSensorConfig initialization failed.";
    }
  });
  return success;
}

bool CameraSensorConfig::IsInitialized() const { return initialized_; }

bool CameraSensorConfig::GetCameraConfig(const std::string& camera_name,
                                         CameraParms& params) const {
  if (!initialized_) {
    AERROR << "CameraSensorConfig not initialized.";
    return false;
  }
  auto it = camera_configs_.find(camera_name);
  if (it == camera_configs_.end()) {
    return false;
  }

  const auto& config = it->second;
  params = config.camparams;
  return true;
}

const std::vector<std::string>& CameraSensorConfig::GetCameraNames() const {
  static const std::vector<std::string> empty;
  if (!initialized_) {
    AERROR << "CameraSensorConfig not initialized.";
    return empty;
  }
  return camera_names_;
}

std::vector<double> CameraSensorConfig::GetCameraIntrinsicsVector(
    const std::string& camera_name) {
  if (!initialized_) {
    AERROR << "CameraSensorConfig not initialized.";
    return {};
  }
  auto it = camera_configs_.find(camera_name);
  if (it == camera_configs_.end()) {
    AERROR << "Camera not found: " << camera_name;
    return {};
  }

  const CameraConfig& config = it->second;
  std::vector<double> intrinsics = {
      config.intrinsics_8.fx, config.intrinsics_8.fy, config.intrinsics_8.cx,
      config.intrinsics_8.cy, config.intrinsics_8.k1, config.intrinsics_8.k2,
      config.intrinsics_8.k3, config.intrinsics_8.k4};

  ADEBUG << "Retrieved intrinsics for camera: " << camera_name
         << ", fx=" << config.intrinsics_8.fx
         << ", fy=" << config.intrinsics_8.fy
         << ", cx=" << config.intrinsics_8.cx
         << ", cy=" << config.intrinsics_8.cy;
  return intrinsics;
}

bool CameraSensorConfig::GetCameraChannel(const std::string& camera_name,
                                          std::string& channel) const {
  if (!initialized_) {
    AERROR << "CameraSensorConfig not initialized.";
    return false;
  }
  auto it = camera_configs_.find(camera_name);
  if (it == camera_configs_.end()) {
    return false;
  }
  channel = it->second.channel;
  return true;
}

bool CameraSensorConfig::ParseMatrix4d(
    const YAML::Node& node, std::vector<std::vector<float>>& matrix) {
  if (!node.IsSequence() || node.size() != 4) {
    return false;
  }
  matrix.resize(4);
  for (std::size_t i = 0; i < 4; ++i) {
    if (!node[i].IsSequence() || node[i].size() != 4) {
      return false;
    }
    matrix[i].resize(4);
    for (std::size_t j = 0; j < 4; ++j) {
      matrix[i][j] = node[i][j].as<float>();
    }
  }
  return true;
}

bool CameraSensorConfig::ParseMatrix4d(const YAML::Node& node,
                                       Eigen::Matrix4d& matrix) {
  if (!node.IsSequence() || node.size() != 4) {
    return false;
  }
  for (std::size_t i = 0; i < 4; ++i) {
    if (!node[i].IsSequence() || node[i].size() != 4) {
      return false;
    }
    for (std::size_t j = 0; j < 4; ++j) {
      matrix(i, j) = node[i][j].as<double>();
    }
  }
  return true;
}

bool CameraSensorConfig::TryLoadCameraConfig(const std::string& yaml_file) {
  try {
    YAML::Node config_node = YAML::LoadFile(yaml_file);

    if (!config_node["cameras"] || !config_node["cameras"].IsSequence()) {
      AERROR << "Invalid camera sensor config: no 'cameras' array found";
      return false;
    }

    camera_configs_.clear();
    camera_names_.clear();

    for (const auto& camera : config_node["cameras"]) {
      std::string camera_name = camera["name"].as<std::string>();
      camera_names_.emplace_back(camera_name);

      CameraConfig config;
      config.channel = camera["channel"].as<std::string>();

      if (!ParseMatrix4d(camera["camera2lidar"],
                         config.camparams.camera2lidar_vec)) {
        AERROR << "Failed to parse camera2lidar for camera: " << camera_name;
        return false;
      }

      if (!ParseMatrix4d(camera["lidar2image"],
                         config.camparams.lidar2image_vec)) {
        AERROR << "Failed to parse lidar2image for camera: " << camera_name;
        return false;
      }

      if (!ParseMatrix4d(camera["img_aug_matrix"],
                         config.camparams.img_aug_matrix_vec)) {
        AERROR << "Failed to parse img_aug_matrix for camera: " << camera_name;
        return false;
      }

      if (!ParseMatrix4d(camera["camera_intrinsics_4x4"],
                         config.camparams.camera_intrinsics_4x4_vec)) {
        AERROR << "Failed to parse camera_intrinsics_4x4 for camera: "
               << camera_name;
        return false;
      }

      if (camera["intrinsics_8"]) {
        CameraIntrinsics& intrinsics = config.intrinsics_8;
        intrinsics.fx = camera["intrinsics_8"]["fx"].as<double>();
        intrinsics.fy = camera["intrinsics_8"]["fy"].as<double>();
        intrinsics.cx = camera["intrinsics_8"]["cx"].as<double>();
        intrinsics.cy = camera["intrinsics_8"]["cy"].as<double>();
        intrinsics.k1 = camera["intrinsics_8"]["k1"].as<double>();
        intrinsics.k2 = camera["intrinsics_8"]["k2"].as<double>();
        intrinsics.k3 = camera["intrinsics_8"]["k3"].as<double>();
        intrinsics.k4 = camera["intrinsics_8"]["k4"].as<double>();
      }

      if (camera["resolution"]) {
        config.camparams.width = camera["resolution"]["width"].as<int>();
        config.camparams.height = camera["resolution"]["height"].as<int>();
      }

      camera_configs_[camera_name] = config;
    }

    yaml_file_path_ = yaml_file;
    AINFO << "Successfully loaded camera sensor config from: " << yaml_file;
    AINFO << "Found " << camera_names_.size() << " cameras.";
    return true;
  } catch (const YAML::Exception& e) {
    AERROR << "Failed to load camera sensor config: " << e.what();
    return false;
  }
}

}  // namespace common
}  // namespace perception
}  // namespace century