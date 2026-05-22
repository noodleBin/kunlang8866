// Created by xiaxinrong on 2025/8/15.
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include "boost/filesystem.hpp"
#include "boost/program_options.hpp"

// Include the YOLOv11 Segmentation header

#include "image_segmentation.h"
#include "glog/logging.h"
#include "yolo8_seg.h"
namespace landmark_loc {
const int arrow_id = 12;
bool ImageSegmentation::GenerateClassMask(const std::vector<Segmentation> &results, const cv::Size &imageSize, cv::Mat& output_mask) const {

  output_mask = cv::Mat::zeros(imageSize, CV_8UC1);  
  for (const auto &seg : results) {
    if (seg.conf < 0.5 || seg.mask.empty() || 
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
    //LOG(INFO) <<"seg id: "<< seg.classId << " confidence: " << seg.conf << std::endl;
    
  
    for (int y = 0; y < output_mask.rows; ++y) {
      const uchar* mask_row = mask_binary.ptr<uchar>(y);
      uchar* output_row = output_mask.ptr<uchar>(y);
      for (int x = 0; x < output_mask.cols; ++x) {
        if (mask_row[x] > 0) {
          output_row[x] = class_gray;  // 设置灰度值
        }
      }
    }
  }
  return true;
}



ImageSegmentation::ImageSegmentation(const std::string& model_path, const std::string& labels_path, const double conf_thresh, const bool speed_up) {
  // Initialize the YOLOv11 Segmentation model
  segmentor_ = new YOLOv8SegDetector(model_path, labels_path, is_GPU, speed_up);
  conf_threshold = conf_thresh;
  iou_threshold = conf_thresh; 
}

bool ImageSegmentation::DoSegment(const cv::Mat& image, cv::Mat& img_result) {
  auto seg_result = segmentor_->segment(image, conf_threshold, iou_threshold);
  GenerateClassMask(seg_result , image.size(), img_result);

  return true;

}

ImageSegmentation::~ImageSegmentation() {
  delete segmentor_;
}
}