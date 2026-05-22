/******************************************************************************
 * Copyright 2019 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "modules/perception/camera/lib/lane/common/proto/darkSCNN.pb.h"
#include "modules/perception/camera/lib/lane/postprocessor/darkSCNN/proto/darkSCNN_postprocessor.pb.h"

#include "modules/perception/base/point.h"
#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/lib/interface/base_calibration_service.h"
#include "modules/perception/camera/lib/interface/base_lane_postprocessor.h"
#include "modules/perception/camera/lib/lane/common/common_functions.h"
#include "modules/perception/lib/registerer/registerer.h"

namespace century {
namespace perception {
namespace camera {

using LanePositionType = base::LaneLinePositionType;

class DarkSCNNLanePostprocessor : public BaseLanePostprocessor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  DarkSCNNLanePostprocessor() : BaseLanePostprocessor() {}

  virtual ~DarkSCNNLanePostprocessor() {}

  bool Init(const LanePostprocessorInitOptions& options =
                LanePostprocessorInitOptions()) override;

  bool Process2D(const LanePostprocessorOptions& options,
                 CameraFrame* frame) override;
  bool Process3D(const LanePostprocessorOptions& options,
                 CameraFrame* frame) override;

  void SetIm2CarHomography(Eigen::Matrix3d homography_im2car) override {
    trans_mat_ = homography_im2car.cast<float>();
    trans_mat_inv = trans_mat_.inverse();
  }

  std::string Name() const override;

  std::vector<std::vector<LanePointInfo>> GetLanelinePointSet();
  std::vector<LanePointInfo> GetAllInferLinePointSet();

 private:
  void ConvertImagePoint2Camera(CameraFrame* frame);
  void PolyFitCameraLaneline(CameraFrame* frame);
  void SampleAndProjectLanePoints(const cv::Mat& lane_map, CameraFrame* frame);
  void PerformRansacFitting(CameraFrame* frame);
  void BuildLaneObjects(CameraFrame* frame);
  void AdjustLaneSpatialLabels(CameraFrame* frame);

  std::vector<float> CalculateLaneCurveValues();
  void SwapLanePointsForCenterLane(std::vector<float>* c0s);
  void SwapLanePointsForBoundary(std::vector<float>* c0s);
  void CreateLaneObjectsFromPoints(CameraFrame* frame,
                                   const std::vector<float>& c0s);

  static void InitializeSpatialLookups();

 private:
  int input_offset_x_ = 0;
  int input_offset_y_ = 312;
  int lane_map_width_ = 640;
  int lane_map_height_ = 480;

  int roi_height_ = 768;
  int roi_start_ = 312;
  int roi_width_ = 1920;

  size_t minNumPoints_ = 8;

  int64_t time_1 = 0;
  int64_t time_2 = 0;
  int64_t time_3 = 0;
  int time_num = 0;

  float max_longitudinal_distance_ = 300.0f;
  float min_longitudinal_distance_ = 0.0f;

  int lane_type_num_;

  lane::DarkSCNNLanePostprocessorParam lane_postprocessor_param_;

  static constexpr float kLaneYRatio = 0.9f;
  static constexpr int kStepYConstant = 6400;
  static constexpr int kMinStepYStart = 40;
  static constexpr int kMinStepYStartAlt = 45;
  static constexpr float kPolyEvalX = 3.0f;
  static constexpr int kLaneValueLeftMin = 5;
  static constexpr float kMinXThreshold = 50.0f;
  static constexpr float kMaxYThreshold = 30.0f;
  static constexpr float kHomographyEpsilon = 1e-6f;
  static constexpr int kLaneValueCurbLeft = 11;
  static constexpr int kLaneValueCurbRight = 12;
  static constexpr float kXMinThreshold = 0.0f;
  static constexpr int kRansacIterations = 200;
  static constexpr float kRansacOutlierThreshold = 0.1f;
  static constexpr float kHomographyZCoordinate = 1.0f;
  static constexpr float kYCoordinateThreshold = 1.0f;
  static constexpr int kPixelOffsetStart = 1;
  static constexpr int kPixelOffsetEnd = 1;

  static constexpr int kLaneIndexEgoLeft = 4;
  static constexpr int kLaneIndexEgoCenter = 5;
  static constexpr int kLaneIndexEgoRight = 6;
  static constexpr int kLaneIndexCurbLeft = 11;
  static constexpr int kLaneIndexCurbRight = 12;

  static constexpr int kSpatialRangeLeftStart = 1;
  static constexpr int kSpatialRangeLeftEnd = 5;
  static constexpr int kSpatialRangeRightStart = 5;
  static constexpr int kSpatialRangeRightEnd = 9;

 private:
  Eigen::Matrix<float, 3, 3> trans_mat_;
  Eigen::Matrix<float, 3, 3> trans_mat_inv;
  std::vector<std::vector<Eigen::Matrix<float, 2, 1>>> xy_points;
  std::vector<std::vector<Eigen::Matrix<float, 2, 1>>> uv_points;
  std::vector<Eigen::Matrix<float, 4, 1>> coeffs_;

  static std::vector<base::LaneLinePositionType> kSpatialPositionLUT;
  static std::map<base::LaneLinePositionType, int> kSpatialPositionIndexMap;
  static bool spatial_lut_initialized_;
};

}  // namespace camera
}  // namespace perception
}  // namespace century
