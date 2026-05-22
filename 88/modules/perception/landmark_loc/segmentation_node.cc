// Created by xiaxinrong on 2025/8/15.
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include "boost/filesystem.hpp"
#include "boost/program_options.hpp"
#include "config/semantic_mapping_config.h"

// Include the YOLOv11 Segmentation header
#include "yolo8_seg.h"
#include "cyber/cyber.h"
namespace {
const int arrow_id = 12;
const std::string labelsPath = "/century/modules/perception/landmark_loc/segmentation/onnx/models/best.names";       // Path to class labels
const std::string modelPath  = "/century/modules/perception/landmark_loc/segmentation/onnx/models/best.onnx";     // Path to YOLO11 model
//const std::string imagePath  = "/home/xr/code/YOLOs-CPP/models/001.jpg";           // Path to input image
cv::Mat GenerateClassMask(const double threshold, const std::vector<Segmentation> &results, const cv::Size &imageSize, const std::string name)
{
  cv::Mat output_mask = cv::Mat::zeros(imageSize, CV_8UC1);  
  output_mask.setTo(0);
  for (const auto &seg : results) {
    if (seg.conf < threshold || seg.mask.empty() || 
        ((seg.classId == arrow_id) && (seg.conf < 0.8)) || 
        ((seg.classId < 10) && seg.conf < 0.8)) {
      continue;
    }

    // Resize mask to match image size if necessary
    cv::Mat mask_resized;
    if (seg.mask.size() != imageSize) {
      cv::resize(seg.mask, mask_resized, imageSize, 0, 0, cv::INTER_NEAREST);
    } else {
      mask_resized = seg.mask;
    }

    cv::Mat mask_gray;
    if (mask_resized.channels() == 3) {
      cv::cvtColor(mask_resized, mask_gray, cv::COLOR_BGR2GRAY);
    } else {
      mask_gray = mask_resized.clone();
    }

    cv::Mat mask_binary;
    cv::threshold(mask_gray, mask_binary, 127, 255, cv::THRESH_BINARY);

    uchar class_gray = static_cast<uchar>(255 - seg.classId);
    LOG(INFO) <<"seg id: "<< seg.classId << " confidence: " << seg.conf << std::endl;
  
    for (int y = 0; y < output_mask.rows; ++y) {
      const uchar* mask_row = mask_binary.ptr<uchar>(y);
      uchar* output_row = output_mask.ptr<uchar>(y);
      for (int x = 0; x < output_mask.cols; ++x) {
        if (mask_row[x] > 0) {
          output_row[x] = class_gray; 
        }
      }
    }
  }
  return output_mask.clone();
}
}
int main(int argc, char** argv) {
  century::cyber::Init(argv[0]);
  FLAGS_minloglevel = google::INFO; 
  FLAGS_alsologtostderr = true; 
  FLAGS_log_dir = "/century/data/log/";
  bool isGPU = true;                          
  // Initialize the YOLO11 segmentor
  YOLOv8SegDetector segmentor(modelPath, labelsPath, isGPU, false);

  LOG(INFO) << "  Model and labels loaded successfully." << std::endl;
  // Load an image
  const std::string img = argv[1]; // Directory containing images
  if (!boost::filesystem::exists(img)) {
    return 0;
  }
  for (auto i : boost::filesystem::directory_iterator(img)) { 
    auto file_name = img + "/" + i.path().stem().string() + ".jpg";
    LOG(INFO) << file_name << std::endl;
    
  
    cv::Mat image = cv::imread(file_name);

    // Perform object segmentation to get segmentation masks and bboxs
    std::vector<Segmentation> results;

    results.clear(); // Clear previous results
    auto start = std::chrono::high_resolution_clock::now();
    double threshold = semantic_mapping::SemanticMappingConfig::GetInstance()->SegThres();
    //usleep(5000);
    results = segmentor.segment(image, threshold, threshold);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::high_resolution_clock::now() - start).count();
    LOG(INFO) << "Segmentation took " << duration << " milliseconds." << std::endl;
    cv::imwrite(img + "/" + i.path().stem().string() + ".png", 
                GenerateClassMask(threshold, results, image.size(), i.path().stem().string()));
  }
  return 0;
}