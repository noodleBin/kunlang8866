


// #include "cyber/time/clock.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include "modules/perception/lidar/lib/detector/pointpillars_detection_nv/point_pillars_detector.h"

#include <dirent.h>
#include <vector>
#include <string>
#include <iostream>
#include <sys/stat.h>
using namespace century::perception;
std::vector<std::string> GetFilesInDirectory(const std::string& directoryPath) {
  std::vector<std::string> files;
  DIR *dir;
  struct dirent *ent;
  struct stat st{};

  if ((dir = opendir(directoryPath.c_str())) != nullptr) {
      while ((ent = readdir(dir)) != nullptr) {
          std::string filename = ent->d_name;
          std::string fullpath = directoryPath + "/" + filename;
          
          if (filename == "." || filename == "..")
              continue;
              
          if (stat(fullpath.c_str(), &st) == 0 && S_ISREG(st.st_mode)) {
              files.push_back(filename);
          }
      }
      closedir(dir);
  } else {
    AERROR <<"could not open directory: " << directoryPath << std::endl;
  }
  
  return files;
}



std::shared_ptr<lidar::LidarFrame> ExtractLidarFrameFromPcd(const std::string& pcd_file_name) {
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  std::cout << "pcd_file_name: " << pcd_file_name << std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file_name, *cloud) == -1) {
    AERROR << "Couldn't read file " << pcd_file_name << ".\n";
    return nullptr;
  }
  std::cout << "--------------------------------------------------frame->raw_cloud->size(): " << cloud->points.size() << std::endl;
  auto frame = std::shared_ptr<lidar::LidarFrame>(new lidar::LidarFrame());
  frame->raw_cloud = base::PointFCloudPool::Instance().Get();

  for(auto& point : cloud->points) {

    base::PointF cur_point;
    cur_point.x = point.x;
    cur_point.y = point.y;
    cur_point.z = point.z;
    cur_point.intensity = point.intensity;

    frame->raw_cloud->push_back(cur_point);
  }
  std::cout << "--------------------------------------------------------filter: " << frame->raw_cloud->points().size() << std::endl;
  return frame;
}

int main(int argc, char** argv) {
  if(argc < 2) {
    AERROR << "Missing input file name";
    return -1;
  }
  const std::string folderPath = argv[1];
  std::vector<std::string> fileList = GetFilesInDirectory(folderPath);
  for (size_t i = 0; i < fileList.size(); i++)
  {
    auto pcd_file_name = folderPath + "/" + fileList[i];
    auto frame = ExtractLidarFrameFromPcd(pcd_file_name);
    if(!frame) {
      AERROR << "Failed to read input file " << pcd_file_name;
      return -1;
    }
    
    century::perception::lidar::PointPillarsDetector detector;
    century::perception::lidar::LidarDetectorOptions opt;
    century::perception::lidar::LidarDetectorInitOptions option;
    option.cfg_file = "/century/modules/perception/production/conf/perception/lidar";

    detector.Init(option);
    for(uint32_t i=0; i<5; ++i) {
      detector.Detect(opt,frame.get());
    }
  }
  
  
  

}