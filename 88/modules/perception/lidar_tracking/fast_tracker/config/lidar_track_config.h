/****************************************************************************/
// Module:    lidar_track_config.hpp
// Description: lidar_track_config
//
// Authors: xiaxinrong
// Date: March 3th 2025
/****************************************************************************/
#pragma once
#include "modules/perception/lidar_tracking/fast_tracker/lidar_track_type.h"

namespace Track {
namespace Ekf {
  struct MultiClassObjectTrackingConfig;
}
namespace Config {
class ConfigParser {
public:
  explicit ConfigParser(const std::string& config_file);
  ConfigParser() = default;
  virtual ~ConfigParser() = default;
  const std::vector<double>& GetExtricLidarToVehicle() const;
  const std::string& GetSensorDataFolder() const;
  const Track::Ekf::MultiClassObjectTrackingConfig* GetLidarTrackConfig() const;
 
private:
  
  Track::Ekf::MultiClassObjectTrackingConfig* config_ = nullptr;
  std::string sensor_data_folder_;
  std::vector<double> extrinsic_lidar_to_vehicle_;
};
  
}
}