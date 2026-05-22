#include "config/semantic_mapping_config.h"
#include <sys/stat.h>
#include <assert.h>
#include <iostream>
namespace semantic_mapping{

SemanticMappingConfig::SemanticMappingConfig() {
   
  struct stat buffer;
  if(stat(config_file_.c_str(), &buffer) != 0) {
    std::cout <<" config file "<< config_file_ <<" not exist ";
    return;
  }
  (void)(buffer);
  YAML::Node node = YAML::LoadFile(config_file_);

  res_folder_ = node["res_folder"].as<std::string>();
  debug_folder_ = node["debug_folder"].as<std::string>();
  map_width_= node["map_width"].as<int>();
  key_frame_interval_ = node["key_frame_interval"].as<int>();
  key_frame_pos_thres_ = node["key_frame_pos_thres"].as<double>();
  key_frame_ang_thres_ =  node["key_frame_ang_thres"].as<double>();
  mode_ =  node["mode"].as<int>();
  seg_thres_ = node["seg_thres"].as<double>();
  loop_closing_ = node["loop_closing"].as<int>();
}
}