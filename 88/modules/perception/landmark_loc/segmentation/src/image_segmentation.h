// Created by xiaxinrong on 2025/8/15.
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include "boost/filesystem.hpp"
#include "boost/program_options.hpp"

class Segmentation;
class YOLOv8SegDetector;
namespace landmark_loc {
class ImageSegmentation {
 public:
 ImageSegmentation() = default;
 ~ImageSegmentation();
 explicit ImageSegmentation(const std::string& model_path,
                            const std::string& labels_path,
                            const double conf_thresh, const bool speed_up = true);
  bool DoSegment(const cv::Mat& image, cv::Mat& result);

  bool GenerateClassMask(const std::vector<Segmentation> &results, 
                         const cv::Size &imageSize, cv::Mat& output) const;
  YOLOv8SegDetector* segmentor_ = nullptr;
  const bool is_GPU = true;
  float conf_threshold = 0.9f;
  float iou_threshold =0.7f; 
};
}