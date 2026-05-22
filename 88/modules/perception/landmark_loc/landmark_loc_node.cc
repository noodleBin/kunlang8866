// Created by xiaxinrong on 2025/8/15.
#include "sensor_data_provider/data_provider.h"
#include "publisher/amcl_pose_publisher.h"
#include "fused_object/fused_object.h"
#include <unordered_set>
#include "common/datatypes.h"
#include "amcl_worker/amcl_worker.h"
#include "config/landmark_loc_config.h"

using namespace landmark_loc;
using namespace semantic_mapping;

std::atomic<bool> stop_flag(false);
int main(int argc, char** argv) {

  static auto handler = [](int sig) {
    std::cout << "\n[Signal] Caught signal " << sig << " (Ctrl+C)." << std::endl;
    stop_flag = true;
  };

  std::signal(SIGINT, handler);
  FLAGS_minloglevel = google::INFO; 
  FLAGS_alsologtostderr = true; 
  FLAGS_log_dir = LandmarkLocConfig::GetInstance()->DebugFolder();
  std::unordered_set<CameraType> camera_types_register;
  for(const auto& iter : LandmarkLocConfig::GetInstance()->CameraTypes()) {
    camera_types_register.insert(static_cast<CameraType>(iter));
  }
  std::unordered_set<LidarType> lidar_types_register;
  for(const auto& iter : LandmarkLocConfig::GetInstance()->LidarTypes()) {
    lidar_types_register.insert(static_cast<LidarType>(iter));
  }


  DataProvider data_provider;
  data_provider.Init(LandmarkLocConfig::GetInstance()->ResFolder(), camera_types_register, lidar_types_register,
                     LandmarkLocConfig::GetInstance()->SegmentationThres(), true,
                     LandmarkLocConfig::GetInstance()->RecorderPredicted(),
                     LandmarkLocConfig::GetInstance()->EnableOdomReceive(),
                     LandmarkLocConfig::GetInstance()->UsingImuChassis());

  if(LandmarkLocConfig::GetInstance()->UsingImuChassis()) {
    century::cyber::WaitForShutdown();
    return 0;
  }
  semantic_mapping::CameraParmeter cam_parmeter = data_provider.ProposeCameraParmeter();
  cam_parmeter.Debug(); 
  FusedObjectGenerator generator;
  generator.Init(camera_types_register, cam_parmeter, true,  LandmarkLocConfig::GetInstance()->FusedMapDump());
  AmclWorker amcl_node( LandmarkLocConfig::GetInstance()->ResFolder(),
                        LandmarkLocConfig::GetInstance()->MinParticleNum(),
                        LandmarkLocConfig::GetInstance()->MaxParticleNum(),
                        LandmarkLocConfig::GetInstance()->ZHit(),
                        LandmarkLocConfig::GetInstance()->ParticleCov(),
                        LandmarkLocConfig::GetInstance()->ZRand(),
                        LandmarkLocConfig::GetInstance()->SigmaHit(),
                        LandmarkLocConfig::GetInstance()->EnablePoseSafetyCheck(),
                        LandmarkLocConfig::GetInstance()->SafetyDis(),
                        LandmarkLocConfig::GetInstance()->ForceUpdate(),
                        LandmarkLocConfig::GetInstance()->Visualize(),
                        LandmarkLocConfig::GetInstance()->RecorderAmcl());
  std::vector<std::tuple<double, Eigen::Matrix4d>> poses;
  amcl_node.Init(poses);
  amcl_node.HandleMapMessage();
  
  
  //bool inited = false;
  const std::chrono::milliseconds interval(100);
  bool is_in_map = false;
  while(!stop_flag) {
    double ts = 0.0;
    pcl::PointCloud<::pcl::PointXYZI>::Ptr cloud = nullptr;
    std::unordered_map<CameraType, cv::Mat> images;
    Eigen::Matrix4d closest_pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d closest_dr = Eigen::Matrix4d::Identity();
    if(!data_provider.FetchBundleData(ts, cloud, images, closest_pose, closest_dr)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    auto start = std::chrono::steady_clock::now();
    if(semantic_mapping::InMap(closest_pose(0,3), closest_pose(1,3))) {
      if(!is_in_map) {
        LOG(INFO) << "Current pose is in map scope, initializing AMCL.";
        amcl_node.SetInitPose(closest_pose);
        data_provider.PublishOdomReset();
        is_in_map = true;
      }
    } else {
      is_in_map = false;
      LOG(WARNING) << closest_pose(0,3)<<","<< closest_pose(1,3)<<" out of map scope, skip amcl update.";      
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
  
    auto fused_objects = generator.RollOut(cloud, images);
    if((!fused_objects) || (fused_objects->points.empty())) {
      continue;
    }
    LOG(INFO) << std::fixed << ts <<" fused point size: " << fused_objects->points.size();
    auto publish_amcl =  amcl_node.ReceiveSensorData(closest_dr, closest_pose, fused_objects , ts);
    if(publish_amcl) {
      AmclPosePublisher::GetInstance()->FeedUpAmclPose(*publish_amcl, ts);
    }
    
    
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    LOG(INFO)<< "All Processing time: " << duration.count() << " ms";
    if(duration < interval) {
      std::this_thread::sleep_for(interval - duration);
    }
   
  }
  return 0;
}
