// Created by xiaxinrong on 2025/8/15.
#include "fused_object/fused_object.h"
#include "fused_object/data_adapter.h"
#include "semantic_mapping/mapping_2d.h"
#include "boost/filesystem.hpp"
#include "boost/program_options.hpp"
#include <glog/logging.h>
#include "config/semantic_mapping_config.h"
#include <array>
//#include "visualizer/map_show.h"
using namespace semantic_mapping;
const uint32_t name_length = 10;
std::unordered_map<std::string, semantic_mapping::CameraType> camera_name_to_type = {

  {"FrontLeft", CamFrontLeft},
  {"FrontMiddle", CamFrontMiddle},
  {"FrontRight", CamFrontRight},
  {"RearRight", CamRearRight},
  {"RearMiddle", CamRearMiddle},
  {"RearLeft", CamRearLeft}
};

int main(int argc, char** argv) {
 
  FLAGS_minloglevel = google::INFO; 
  FLAGS_alsologtostderr = true; 
  FLAGS_log_dir = SemanticMappingConfig::GetInstance()->DebugFolder();
  google::InitGoogleLogging(SemanticMappingConfig::GetInstance()->DebugFolder().c_str());
  std::string data_folder = SemanticMappingConfig::GetInstance()->ResFolder();
  int mode = SemanticMappingConfig::GetInstance()->Mode(); // 0: mapping only, 1: mapping + amcl, 2: amcl only
  std::string parameter_folder = data_folder + "/parameter/";
  std::string odom_in = data_folder + "/localization.txt";
  FusedObjectGenerator generator;
  if(!generator.Init({CamFrontLeft, CamFrontMiddle, CamFrontRight, CamRearLeft, CamRearMiddle, CamRearRight}, parameter_folder,true, false)) {
    std::cerr << "Failed to initialize FusedObjectGenerator." << std::endl;
    return -1;
  }
  auto trajactory = ReadDrpose(odom_in);
  if(trajactory.empty()) {
    std::cerr << "Failed to read trajectory from: " << odom_in << std::endl;
    return -1;
  }


  Mapping2D* mapping = nullptr;
  if(mode < 2) {
    mapping = new Mapping2D(SemanticMappingConfig::GetInstance()->KeyFramePosThes(),
                            SemanticMappingConfig::GetInstance()->KeyFrameAngThes(),
                            SemanticMappingConfig::GetInstance()->KeyFrameInterval(),
                            SemanticMappingConfig::GetInstance()->DebugFolder() + "/semantic/",
                            SemanticMappingConfig::GetInstance()->MapWidth(),
                            SemanticMappingConfig::GetInstance()->LoopClosing());
  }

  if(mapping) {
    mapping->Init();
  }
 
  std::unordered_set<std::string> ts_set;
  for (auto i : boost::filesystem::directory_iterator(data_folder +"/image/")) { 
    auto folder_name = data_folder +"/image/" + i.path().stem().string();
    if (boost::filesystem::is_directory(folder_name)) {
      if(camera_name_to_type.count(i.path().stem().string()) == 0) {
        std::cerr << "Unknown camera: " << i.path().stem().string() << std::endl;
        continue;
      }
      FileTsMap(folder_name, ts_set);
    }
  }


  if(ts_set.empty()) {
    std::cerr << "No images found in the specified folder." << std::endl;
    return -1;
  }

  std::vector<double> vts;
  for (const auto& s : ts_set) {
    vts.push_back(std::stod(s)); // string -> double
  }

  std::sort(vts.begin(), vts.end());

  LOG(INFO) << "Found " << ts_set.size() << " images." << std::endl;

  std::vector<std::string> pcl_folder;
  for (auto i : boost::filesystem::directory_iterator(data_folder +"/pcl/")) { 
    auto folder_name = data_folder +"/pcl/" + i.path().stem().string();
    if (boost::filesystem::is_directory(folder_name)) {
      pcl_folder.push_back(folder_name);
    }
   
  }
  std::vector<std::pair<double, semantic_mapping::PointCloudPtr>> fused_clouds;
  fused_clouds.reserve(vts.size());
  for(const auto& ts : vts) {
    LOG(INFO) << "Processing timestamp: "<< std::fixed << ts << std::endl;
    Eigen::Matrix4d closest_pose = Eigen::Matrix4d::Identity();
    for(const auto& dr_pose : trajactory) {
      if(std::get<0>(dr_pose) < ts) continue; // dr pose timestamp is earlier than image timestamp
      LOG(INFO) << "Found closest pose for timestamp: " << std::fixed<< std::get<0>(dr_pose) << std::endl;
      closest_pose = std::get<1>(dr_pose);
      break;
    }
    if(!semantic_mapping::InMap(closest_pose(0,3), closest_pose(1,3))) {
      LOG(INFO) <<"out of map";
      continue;
    } 
    auto lidar_cloud = ConcatPcl(pcl_folder, std::to_string(ts).substr(0,name_length));
    if((!lidar_cloud) ||(lidar_cloud->empty())) {
      std::cerr << "Failed to load lidar point cloud for timestamp: " << ts << std::endl;
      continue;
    }
    std::unordered_map<CameraType, cv::Mat> segmentations;
    for (auto i : boost::filesystem::directory_iterator(data_folder +"/image/")) { 
      auto folder_name = data_folder +"/image/" + i.path().stem().string();
      if (boost::filesystem::is_directory(folder_name)) {
        if(camera_name_to_type.count(i.path().stem().string()) == 0) {
          std::cerr << "Unknown camera: " << i.path().stem().string() << std::endl;
          continue;
        }
        const auto image_name =  data_folder +"/image/" + i.path().stem().string() + "/" + std::to_string(ts).substr(0,name_length) + ".png";
        if (!boost::filesystem::exists(image_name)) {
          LOG(INFO) << "Image file does not exist: " << image_name << std::endl;
          continue;
        }
        cv::Mat segmentation = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
        if(segmentation.empty()) {
          LOG(INFO) << "Failed to load segmentation for camera: " << camera_name_to_type.count(i.path().stem().string()) 
                    << " at timestamp: " << ts << std::endl;
          continue;
        }
        LOG(INFO) << "Successfully loaded segmentation for camera: " << i.path().stem().string() << std::endl;
        segmentations[camera_name_to_type.at(i.path().stem().string())] = segmentation;
      }
    }
    if(segmentations.empty()) {
      std::cerr << "Failed to load all segmentations for timestamp: " << ts << std::endl;
      continue;
    }
  
    auto fused_object = generator.RollOut(lidar_cloud, segmentations, std::to_string(ts).substr(0,name_length));
    if(!fused_object) {
      std::cerr << "Failed to generate fused object for timestamp: " << ts << std::endl;
      continue;
    }
    LOG(INFO) << "Received fused object at " << std::fixed << ts << " with " << fused_object->points.size() << " points.";
    fused_clouds.push_back(std::pair<double, semantic_mapping::PointCloudPtr>(ts, fused_object));
   // for(const auto& pt : fused_object->points) {
    //  LOG(INFO) << "Point: (" << pt.x << ", " << pt.y << ", " << pt.z << ") Label: " << pt.label;
   // }
   // LOG(INFO) << closest_pose.transpose() << std::endl;
    ComposedSensorData sensor_data(ts, closest_pose, fused_object);
    if(mapping) {
      mapping->ProcessScanKunlang(sensor_data);
    }
  }
  if(mapping) {
    mapping->ShowGlobalMap();
    mapping->SaveSemanticMap();
  }

  if(mode == 0) {
    delete mapping;
    LOG(INFO) << "Mapping only mode. Exiting." << std::endl;
    return 0;
  }
  
  return 0;


}