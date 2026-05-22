#pragma once
#include <nlohmann/json.hpp>
#include <vector>
#include <list>
namespace century {
namespace perception {  
namespace lidar {
using namespace nlohmann; 
struct Point2D {
  double x = 0.0;
  double y = 0.0;
  Point2D(const double a, const double b) : x(a), y(b) {}
};
struct Point2DFeature {
  double x = 0.0;
  double y = 0.0;
  std::string comments;
};
struct LineFeature {
  std::vector<Point2D> point;
  std::string comments;
};
bool EnsureDirectory(const std::string& path) ;
class GeoJsonConvert {
  public:
  GeoJsonConvert() = default;
  explicit GeoJsonConvert(const double ts, const std::string type);
  ~GeoJsonConvert() = default;
  void AddPoint(const double x, const double y, const std::string& feature);
  void AddLine(const std::vector<Point2D> line, const std::string& feature);
  void DumpTraj();
  void DumpTrack();
  void Clear();
  private:
  std::vector<Point2DFeature> points_;
  std::vector<LineFeature> lines_;
  std::string file_name_;
  std::string file_path_;
  bool enable_= false;
};

}
}
}