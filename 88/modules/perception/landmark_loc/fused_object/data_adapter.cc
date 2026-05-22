// Created by xiaxinrong on 2025/8/15.
#include "data_adapter.h"
#include <unordered_set>
#include "boost/filesystem.hpp"
#include "boost/program_options.hpp"
#include <sys/stat.h>
#include <iostream>
namespace semantic_mapping {
namespace {

}
bool EnsureDirectory(const std::string& path) {

  struct stat info;
  if (stat(path.c_str(), &info) == 0) {
    if (info.st_mode & S_IFDIR) {
     // AINFO << "Folder already exists: " << path << std::endl;
      return true;
    } else {
      LOG(INFO) << "Path exists but is not a directory: " << path << std::endl;
      return false;
    }
  } else {
    if (mkdir(path.c_str(), 0755) == 0) {
      LOG(INFO) << "Folder created successfully: " << path << std::endl;
      return true;
    } else {
      LOG(INFO) << "Failed to create folder: " << path << std::endl;
      return false;
    }
  }
}

std::vector<std::tuple<double, Eigen::Matrix4d>> ReadDrpose(const std::string odom_in) {

  std::vector<std::tuple<double, Eigen::Matrix4d>> trajectory;
  std::ifstream file_if(odom_in);
  if (!file_if) {
    LOG(INFO) << " error dr pose file " << odom_in << std::endl;
    return trajectory;
  }
  std::string line;
  while (getline(file_if, line)) {

    std::string data[8];
    int i = 0;
    std::stringstream line_stream(line);
    while (std::getline(line_stream, data[i++], ' ')) {

    }

    double timestamp = stod(data[0]);
    double x = stod(data[1]);
    double y = stod(data[2]);
    double z = stod(data[3]);
    double qx = atof(data[4].c_str());
    double qy = atof(data[5].c_str());
    double qz = atof(data[6].c_str());
    double qw = atof(data[7].c_str());

    EigenPose pose(Eigen::Quaterniond(qw, qx, qy, qz).normalized(),  Eigen::Vector3d(x, y, z));
    trajectory.push_back(std::tuple<double, Eigen::Matrix4d>(
        timestamp, pose.toMatrix()));
  }
  LOG(INFO) << "read " << trajectory.size() << " dr poses from " << odom_in
            << std::endl;
  return trajectory;
}

std::map<std::string, float> readIntrinsic(const std::string &xmlFilePath) {
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(xmlFilePath.c_str())) {
    std::cerr << "FATAL ERROR: Failed to open intrinsic xml file "
              << xmlFilePath << std::endl;
    std::cerr << "the file is not found or it's not a valid xml file"
              << std::endl;
    exit(0);
  }

  tinyxml2::XMLElement *param_element = doc.FirstChildElement("param");
  if (!param_element) {
    std::cerr << "cannot find tag 'param' in xml file " << xmlFilePath
              << std::endl;
    exit(0);
  }

  std::map<std::string, float> values = {
      {"fx", 0.f},           {"fy", 0.f},         {"cx", 0.f}, {"cy", 0.f},
      {"k1", 0.f},           {"k2", 0.f},         {"k3", 0.f}, {"k4", 0.f},
      {"image_height", 0.f}, {"image_width", 0.f}};

  for (auto &p : values) {
    auto tag_name = p.first;
    tinyxml2::XMLElement *textNode =
        param_element->FirstChildElement(tag_name.c_str());
    if (!textNode) {
      std::cerr << "cannot find tag " << tag_name << " in xml file "
                << xmlFilePath << std::endl;
      exit(0);
    }
    printf("%s %s\n", tag_name.c_str(), textNode->GetText());
    p.second = atof(textNode->GetText());
  }

  return values;
}
std::vector<float> readExtrinsic(const std::string &filename) {
  std::vector<float> transformations;
  std::ifstream inputFile(filename);
  if (!inputFile.is_open()) {
    std::cerr << "Error opening file: " << filename << std::endl;
    exit(0);
  }
  LOG(INFO) << filename << std::endl;
  std::string line;
  std::vector<std::string> lines(11);
  for (size_t i = 0; i < lines.size(); i++)
    std::getline(inputFile, lines[i]); //translation:
  inputFile.close();
  for (int i: {1, 2, 3, 6, 7, 8, 9}) {
    auto line = lines[i];
    std::istringstream iss(line);
    std::string token;
    float value;
    iss >> token >> value;
    transformations.push_back(value);
    LOG(INFO) << value << std::endl;
  }

  return transformations;

}

void FileTsMap(const std::string &img, std::unordered_set<std::string> &res) {
  if (!boost::filesystem::exists(img)) {
    {};
  }
  for (auto i : boost::filesystem::directory_iterator(img)) { 
    res.insert(i.path().stem().string());
  }
  
  return;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr ConcatPcl(const std::vector<std::string>& pcl_folder, 
                                               const std::string ts_str) {
    
     
  pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  for(const auto& folder : pcl_folder) {
    if (!boost::filesystem::exists(folder)) {
      std::cerr << "Folder " << folder << " does not exist." << std::endl;
      continue;
    }
    auto pcd_file_name = folder +"/" + ts_str + ".pcd";
    if (!boost::filesystem::exists(pcd_file_name)) {
      std::cerr << "file " << pcd_file_name << " does not exist." << std::endl;
      continue;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = nullptr;
    if(result_cloud->points.empty()) {
      cloud = result_cloud;
    } else {
      cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file_name, *cloud) == -1) {
      printf("Couldn't read file %s\n", (pcd_file_name).c_str());
      return nullptr;
    }

    if(cloud.get() != result_cloud.get()) {
      // Concatenate the point clouds
      result_cloud->insert(result_cloud->end(), cloud->begin(), cloud->end());
      result_cloud->width = result_cloud->points.size();
      result_cloud->height = 1;
    }
    LOG(INFO) << "loaded " << cloud->points.size() << " points from " << pcd_file_name << std::endl;
  }
  LOG(INFO) << "result " << result_cloud->points.size() << std::endl;
  
  return result_cloud;
}
}