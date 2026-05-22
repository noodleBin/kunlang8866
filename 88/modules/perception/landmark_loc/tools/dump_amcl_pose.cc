#include "modules/perception/landmark_loc/proto/amcl_recorder.pb.h"
#include "cyber/cyber.h"
#include "cyber/common/file.h"
#include <boost/filesystem.hpp>
#include <iostream>
#include "glog/logging.h"
#include <nlohmann/json.hpp>
namespace fs = boost::filesystem;
namespace {

struct Line {
  double x1, y1;
  double x2, y2;
};
const double direction_length = 0.01;

std::vector<Line> ProcessFiles(const fs::path& dir) {
  std::vector<Line> lines;
  for (fs::directory_iterator it(dir), end; it != end; ++it) {
    if (!fs::is_regular_file(*it)) {
      continue;
    }
    
    century::perception::landmark_loc::AmclPredictedPoses amcl_record;
    if (!century::cyber::common::GetProtoFromBinaryFile(it->path().string(), &amcl_record)) {
      std::cout << "Failed to load amcl record from " << it->path().string() << std::endl;
      continue;
    }
    
    for (int i = 0; i < amcl_record.pose_size(); ++i) {
      const auto& pose = amcl_record.pose(i);
      Line line;
      line.x1 = std::round(pose.position().x() * 1000.0) / 1000.0;
      line.y1 = std::round(pose.position().y() * 1000.0) / 1000.0;
      line.x2 = line.x1 + direction_length * std::cos(pose.position().z());
      line.y2 = line.y1 + direction_length * std::sin(pose.position().z());
      lines.push_back(line);
    }
  }
  return lines;
}

void WriteTxt(const std::vector<Line>& lines, const std::string& out_path) {
  std::ofstream out(out_path);
  for (const auto& line : lines) {
    out << std::fixed << std::setprecision(3) << line.x1 << ", " << line.y1 << std::endl;
  }
  out.close();
}

nlohmann::json BuildGeoJson(const std::vector<Line>& lines) {
  nlohmann::json fc;
  fc["type"] = "FeatureCollection";
  fc["features"] = nlohmann::json::array();
  for (size_t i = 0; i < lines.size(); ++i) {
    nlohmann::json feature;
    feature["type"] = "Feature";
    feature["geometry"]["type"] = "LineString";
    feature["geometry"]["coordinates"] = nlohmann::json::array({
        {lines[i].x1, lines[i].y1},
        {lines[i].x2, lines[i].y2}
    });
    feature["properties"]["id"] = i;
    fc["features"].push_back(feature);
  }
  return fc;
}

}

int main(int argc, char** argv) {
  if(argc != 2) {
    std::cout << "Usage: dump_amcl_pose amcl_record_file.pb.bin" << std::endl;
    return -1;
  } 
  std::string folder = argv[1];
  std::string prefix = "/century/data/";
  fs::path dir(prefix + folder);

  if (!fs::exists(dir) || !fs::is_directory(dir)) {
    std::cerr << "Not a directory\n";
    return -1;
  }
  
  auto lines = ProcessFiles(dir);
  WriteTxt(lines, prefix + "/" + folder + ".txt");
  
  std::ofstream geojson(prefix + "/" + folder + "_direction.Geojson");
  geojson << BuildGeoJson(lines).dump(2) << std::endl;
  geojson.close();

  std::cout << "Dumped " << lines.size() << " poses." << std::endl;
  return 0;
}