#include "modules/perception/lidar_tracking/io/geojson_convert.h"
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <unistd.h>
#include "cyber/common/log.h"
namespace century {
namespace perception {  
namespace lidar {

bool EnsureDirectory(const std::string& path) {

  struct stat info;
  if (stat(path.c_str(), &info) == 0) {
    if (info.st_mode & S_IFDIR) {
     // AINFO << "Folder already exists: " << path << std::endl;
      return true;
    } else {
      AERROR << "Path exists but is not a directory: " << path << std::endl;
      return false;
    }
  } else {
    if (mkdir(path.c_str(), 0755) == 0) {
      AINFO << "Folder created successfully: " << path << std::endl;
      return true;
    } else {
      AERROR << "Failed to create folder: " << path << std::endl;
      return false;
    }
  }
}
GeoJsonConvert::GeoJsonConvert(const double ts, const std::string type) {
  std::string str_ts =  std::to_string(ts);
  std::replace(str_ts.begin(), str_ts.end(), '.', '_');
  file_path_ =  FLAGS_log_dir + "/" + str_ts + "/"; 
  file_name_ =  file_path_ + type + ".geojson";

};
void GeoJsonConvert::AddPoint(const double x, const double y, const std::string& feature) {
  Point2DFeature point{x,y,feature};
  points_.push_back(std::move(point));
}

void GeoJsonConvert::AddLine(const std::vector<Point2D> line, const std::string& feature) {

  LineFeature line_feature{line,feature};
  lines_.push_back(line_feature);
}

void GeoJsonConvert::Clear() {
  points_.clear();
  lines_.clear();
}

void GeoJsonConvert::DumpTrack() {

  const std::string log_folder = FLAGS_log_dir + "/";
  if(!EnsureDirectory(log_folder)) {
    AERROR << "Failed to ensure directory: " << log_folder << std::endl;
    return;
  }

  if(!EnsureDirectory(file_path_)) {
    AERROR<< "Failed to ensure directory: " << file_path_ << std::endl;
    return;
  }

  
  json geojson;
  geojson["type"] = "FeatureCollection";
  geojson["features"] = json::array();

  for (const auto& point : points_) {
    json feature;
    feature["type"] = "Feature";
    feature["geometry"]["type"] = "Point";
    feature["geometry"]["coordinates"] = {point.x, point.y};
    feature["properties"]["description"] = point.comments; 
    geojson["features"].push_back(feature);
  }

    
  std::string geojson_str = geojson.dump(4); 

  std::ofstream out_file(file_name_);
 // AINFO << "Writing GeoJSON to file: " << file_name_ << std::endl;
  if (out_file.is_open()) {
      out_file << geojson_str;
      out_file.close();
  } else {
      AERROR << "Unable to open file for writing. " << file_name_ << std::endl;
  }

  return ;
}

void GeoJsonConvert::DumpTraj() {

  const std::string log_folder = FLAGS_log_dir + "/";
  if(!EnsureDirectory(log_folder)) {
    AERROR << "Failed to ensure directory: " << log_folder << std::endl;
    return;
  }

  if(!EnsureDirectory(file_path_)) {
    AERROR << "Failed to ensure directory: " << file_path_ << std::endl;
    return;
  }

  
  json geojson;
  geojson["type"] = "FeatureCollection";
  geojson["features"] = json::array();


  for (const auto& line : lines_) {
    json feature;
    feature["type"] = "Feature";
    feature["geometry"]["type"] = "LineString";
    feature["geometry"]["coordinates"] = json::array();
    for (const auto& point : line.point) {
      feature["geometry"]["coordinates"].push_back({point.x, point.y});
    }
    feature["properties"]["description"] = line.comments;
    geojson["features"].push_back(feature);
  }

  for (const auto& point : points_) {
    json feature;
    feature["type"] = "Feature";
    feature["geometry"]["type"] = "Point";
    feature["geometry"]["coordinates"] = {point.x, point.y};
    feature["properties"]["description"] = point.comments; 
    geojson["features"].push_back(feature);
  }
    
  std::string geojson_str = geojson.dump(4); 

  std::ofstream out_file(file_name_);
  //AINFO << "Writing GeoJSON to file: " << file_name_ << std::endl;
  if (out_file.is_open()) {
    out_file << geojson_str;
    out_file.close();
  } else {
    AERROR << "Unable to open file for writing. " << file_name_ << std::endl;
  }

  return ;
}

}
}
}