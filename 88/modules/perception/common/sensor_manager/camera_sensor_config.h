// camera_sensor_config.h
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "Eigen/Dense"
#include "yaml-cpp/yaml.h"

#include "cyber/common/log.h"

namespace century {
namespace perception {
namespace common {

struct CameraIntrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double k1 = 0.0;
  double k2 = 0.0;
  double k3 = 0.0;
  double k4 = 0.0;
};

struct CameraParms {
  std::vector<std::vector<float>> camera2lidar_vec;
  std::vector<std::vector<float>> lidar2image_vec;
  std::vector<std::vector<float>> img_aug_matrix_vec;
  std::vector<std::vector<float>> camera_intrinsics_4x4_vec;
  int width = 1920;
  int height = 1080;
};

class CameraSensorConfig {
 public:
  using Ptr = std::shared_ptr<CameraSensorConfig>;

  // Disable copy and assignment.
  CameraSensorConfig(const CameraSensorConfig&) = delete;
  CameraSensorConfig& operator=(const CameraSensorConfig&) = delete;

  // Singleton accessor.
  static CameraSensorConfig& GetInstance();

  // Load the configuration file. This is expected to succeed only once.
  bool Initialize(const std::string& yaml_file);

  // Report whether initialization completed successfully.
  bool IsInitialized() const;

  // Public query interfaces. Call after successful initialization.
  bool GetCameraConfig(const std::string& camera_name,
                       CameraParms& params) const;
  const std::vector<std::string>& GetCameraNames() const;
  std::vector<double> GetCameraIntrinsicsVector(const std::string& camera_name);
  bool GetCameraChannel(const std::string& camera_name,
                        std::string& channel) const;

 private:
  // Private constructor.
  CameraSensorConfig() = default;

  // Internal configuration data.
  struct CameraConfig {
    std::string channel;
    CameraIntrinsics intrinsics_8;
    CameraParms camparams;
  };

  // Helper functions.
  bool ParseMatrix4d(const YAML::Node& node,
                     std::vector<std::vector<float>>& matrix);
  bool ParseMatrix4d(const YAML::Node& node, Eigen::Matrix4d& matrix);
  bool TryLoadCameraConfig(const std::string& yaml_file);

  // Member state.
  std::vector<std::string> camera_names_;
  std::unordered_map<std::string, CameraConfig> camera_configs_;
  std::string yaml_file_path_;
  bool initialized_ = false;
  static std::once_flag init_flag_;
};

}  // namespace common
}  // namespace perception
}  // namespace century
