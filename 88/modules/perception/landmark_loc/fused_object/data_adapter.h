// Created by xiaxinrong on 2025/8/15.
#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <string>
#include <tinyxml2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "common/datatypes.h"
#include <pcl/io/pcd_io.h>
#include <unordered_set>
namespace semantic_mapping{
bool EnsureDirectory(const std::string& path);
std::vector<std::tuple<double, Eigen::Matrix4d>> ReadDrpose(const std::string odom_in);
std::map<std::string, float> readIntrinsic(const std::string &xmlFilePath);
std::vector<float> readExtrinsic(const std::string &filename);
void FileTsMap(const std::string &img, std::unordered_set<std::string> &res);
pcl::PointCloud<pcl::PointXYZI>::Ptr ConcatPcl(const std::vector<std::string>& pcl_folder, 
                                               const std::string ts_str); 
}