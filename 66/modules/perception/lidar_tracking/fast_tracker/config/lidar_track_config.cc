#include "modules/perception/lidar_tracking/fast_tracker/config/lidar_track_config.h"
#include "modules/perception/lidar_tracking/fast_tracker/ekf/lidar_track_ekf_filter.h"
#include "modules/perception/lidar_tracking/fast_tracker/util/lidar_track_util.h"
#include "yaml-cpp/yaml.h"
namespace Track {
namespace Config {

ConfigParser::ConfigParser(const std::string& config_path) {

  if(!Track::Util::FileExists(config_path)) {
    std::cout << "config file does not exist: " << config_path << std::endl;
    return ;
  }

  try {
    YAML::Node node = YAML::LoadFile(config_path);
    config_ = new Track::Ekf::MultiClassObjectTrackingConfig();

    config_->global_coord_track = node["global_coord_track"].as<int>();
    config_->output_local_coord = node["output_local_coord"].as<int>();
    config_->output_period_lidar = node["output_period_lidar"].as<int>();
    config_->output_confirmed_track = node["output_confirmed_track"].as<int>();
  
    config_->use_predefined_ref_point = node["use_predefined_ref_point"].as<int>();
    config_->reference_lat = node["reference_lat"].as<double>();
    config_->reference_lon = node["reference_lon"].as<double>();
    config_->reference_height = node["reference_height"].as<double>();

    config_->cal_detection_individual_time = node["cal_detection_individual_time"].as<int>();
    config_->lidar_rotation_period = node["lidar_rotation_period"].as<double>();
    config_->lidar_sync_scan_start = node["lidar_sync_scan_start"].as<int>();
  
    config_->max_association_dist_m = node["max_association_dist_m"].as<double>();
  
    config_->prediction_model = static_cast<Track::Ekf::PredictionModel>(node["prediction_model"].as<int>());
    
    config_->system_noise_std_xy_m = node["system_noise_std_xy_m"].as<double>();
    config_->system_noise_std_yaw_deg = node["system_noise_std_yaw_deg"].as<double>();
    config_->system_noise_std_vx_vy_ms = node["system_noise_std_vx_vy_ms"].as<double>();
  
    config_->system_noise_std_yaw_rate_degs = node["system_noise_std_yaw_rate_degs"].as<double>();
    config_->system_noise_std_ax_ay_ms2 = node["system_noise_std_ax_ay_ms2"].as<double>();

    config_->meas_noise_std_xy_m = node["meas_noise_std_xy_m"].as<double>();
    config_->meas_noise_std_yaw_deg = node["meas_noise_std_yaw_deg"].as<double>();

    config_->dimension_filter_alpha = node["dimension_filter_alpha"].as<double>();

    config_->use_kinematic_model = node["use_kinematic_model"].as<int>();
    config_->use_yaw_rate_filtering = node["use_yaw_rate_filtering"].as<int>();
    config_->max_steer_deg = node["max_steer_deg"].as<double>();
    config_->visualize_mesh = node["visualize_mesh"].as<int>();
    extrinsic_lidar_to_vehicle_ = node["extrinsic"].as<std::vector<double>>();
    sensor_data_folder_ = node["sensor_data_folder"].as<std::string>();
    
  } catch (const YAML::Exception& e) {
    std::cerr << "Error parsing YAML file: " << e.what() << ", file name: " << config_path << std::endl;
  }

  std::cout << "config is as below: "<< std::endl;
  std::cout << "global_coord_track:" << config_->global_coord_track << std::endl;
  std::cout << "output_local_coord:" << config_->output_local_coord << std::endl;
  std::cout << "config_->prediction_model:"<< config_->prediction_model << std::endl;
  std::cout << "sensor_data_folder :" << sensor_data_folder_  << std::endl;
  std::cout << "extrinsic_lidar_to_vehicle:"
            << extrinsic_lidar_to_vehicle_[0] << " "
            << extrinsic_lidar_to_vehicle_[1] << " "
            << extrinsic_lidar_to_vehicle_[2] << " "
            << extrinsic_lidar_to_vehicle_[3] << " "
            << extrinsic_lidar_to_vehicle_[4] << " "
            << extrinsic_lidar_to_vehicle_[5] << std::endl;
  std::cout << "system_noise_std_xy_m:" << config_->system_noise_std_xy_m << std::endl;
  std::cout << "system_noise_std_yaw_deg:" << config_->system_noise_std_yaw_deg << std::endl;
  std::cout << "system_noise_std_vx_vy_ms:" << config_->system_noise_std_vx_vy_ms << std::endl;
  std::cout << "system_noise_std_yaw_rate_degs:" << config_->system_noise_std_yaw_rate_degs << std::endl;
  std::cout << "meas_noise_std_xy_m:" << config_->meas_noise_std_xy_m << std::endl;
  std::cout << "meas_noise_std_yaw_deg:" << config_->meas_noise_std_yaw_deg << std::endl;
  std::cout << "dimension_filter_alpha:" << config_->dimension_filter_alpha << std::endl;
  std::cout << "max_steer_deg:" << config_->max_steer_deg << std::endl;
  std::cout << "use_yaw_rate_filtering" << config_->use_yaw_rate_filtering << std::endl;
  std::cout << "visualize_mesh" << config_->visualize_mesh << std::endl;
  std::cout << "output_period_lidar" << config_->output_period_lidar << std::endl;
  std::cout << "use_kinematic_model" << config_->use_kinematic_model << std::endl;

  return;
 }

const std::vector<double>& ConfigParser::GetExtricLidarToVehicle() const {
  return extrinsic_lidar_to_vehicle_;
 }
const std::string& ConfigParser::GetSensorDataFolder() const {
  return sensor_data_folder_;

}
const Track::Ekf::MultiClassObjectTrackingConfig* ConfigParser::GetLidarTrackConfig() const {
    
  return config_;
}

    
  }
  }