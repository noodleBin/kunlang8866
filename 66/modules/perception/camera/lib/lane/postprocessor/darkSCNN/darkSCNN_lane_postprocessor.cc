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
#include "modules/perception/camera/lib/lane/postprocessor/darkSCNN/darkSCNN_lane_postprocessor.h"

#include <algorithm>
#include <map>
#include <utility>

#include "cyber/common/file.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/camera/common/math_functions.h"
#include "modules/perception/camera/common/timer.h"

namespace century {
namespace perception {
namespace camera {

using century::cyber::common::GetAbsolutePath;
using century::cyber::common::GetProtoFromFile;

std::vector<base::LaneLinePositionType>
    DarkSCNNLanePostprocessor::kSpatialPositionLUT;
std::map<base::LaneLinePositionType, int>
    DarkSCNNLanePostprocessor::kSpatialPositionIndexMap;
bool DarkSCNNLanePostprocessor::spatial_lut_initialized_ = false;

template <typename T = float>
T GetPolyValue(T a, T b, T c, T d, T x) {
  T y = d;
  T v = x;

  y += (c * v);
  v *= x;
  y += (b * v);
  v *= x;
  y += (a * v);

  return y;
}

void DarkSCNNLanePostprocessor::InitializeSpatialLookups() {
  if (!spatial_lut_initialized_) {
    kSpatialPositionLUT = {base::LaneLinePositionType::UNKNOWN,
                           base::LaneLinePositionType::FOURTH_LEFT,
                           base::LaneLinePositionType::THIRD_LEFT,
                           base::LaneLinePositionType::ADJACENT_LEFT,
                           base::LaneLinePositionType::EGO_LEFT,
                           base::LaneLinePositionType::EGO_CENTER,
                           base::LaneLinePositionType::EGO_RIGHT,
                           base::LaneLinePositionType::ADJACENT_RIGHT,
                           base::LaneLinePositionType::THIRD_RIGHT,
                           base::LaneLinePositionType::FOURTH_RIGHT,
                           base::LaneLinePositionType::OTHER,
                           base::LaneLinePositionType::CURB_LEFT,
                           base::LaneLinePositionType::CURB_RIGHT};

    kSpatialPositionIndexMap = {{base::LaneLinePositionType::UNKNOWN, 0},
                                {base::LaneLinePositionType::FOURTH_LEFT, 1},
                                {base::LaneLinePositionType::THIRD_LEFT, 2},
                                {base::LaneLinePositionType::ADJACENT_LEFT, 3},
                                {base::LaneLinePositionType::EGO_LEFT, 4},
                                {base::LaneLinePositionType::EGO_CENTER, 5},
                                {base::LaneLinePositionType::EGO_RIGHT, 6},
                                {base::LaneLinePositionType::ADJACENT_RIGHT, 7},
                                {base::LaneLinePositionType::THIRD_RIGHT, 8},
                                {base::LaneLinePositionType::FOURTH_RIGHT, 9},
                                {base::LaneLinePositionType::OTHER, 10},
                                {base::LaneLinePositionType::CURB_LEFT, 11},
                                {base::LaneLinePositionType::CURB_RIGHT, 12}};

    spatial_lut_initialized_ = true;
  }
}

bool DarkSCNNLanePostprocessor::Init(
    const LanePostprocessorInitOptions& options) {
  InitializeSpatialLookups();

  darkSCNN::DarkSCNNParam darkscnn_param;
  const std::string& proto_path =
      GetAbsolutePath(options.detect_config_root, options.detect_config_name);
  if (!GetProtoFromFile(proto_path, &darkscnn_param)) {
    AINFO << "Failed to load proto param, root dir: " << options.root_dir;
    return false;
  }
  const auto& model_param = darkscnn_param.model_param();
  input_offset_x_ = model_param.input_offset_x();
  input_offset_y_ = model_param.input_offset_y();
  lane_map_width_ = model_param.resize_width();
  lane_map_height_ = model_param.resize_height();
  AINFO << "offset_x=" << input_offset_x_ << " offset_y=" << input_offset_y_
        << " lane_map_width=" << lane_map_width_
        << " lane_map_height_=" << lane_map_height_;
  const std::string& root_dir = options.root_dir;
  const std::string& conf_file = options.conf_file;
  const std::string& postprocessor_config =
      GetAbsolutePath(root_dir, conf_file);
  AINFO << "postprocessor_config: " << postprocessor_config;
  if (!GetProtoFromFile(postprocessor_config, &lane_postprocessor_param_)) {
    AERROR << "Failed to read config detect_param: " << postprocessor_config;
    return false;
  }
  std::string param_str;
  google::protobuf::TextFormat::PrintToString(lane_postprocessor_param_,
                                              &param_str);
  AINFO << "lane_postprocessor param: " << param_str;

  roi_height_ = lane_postprocessor_param_.roi_height();
  roi_start_ = lane_postprocessor_param_.roi_start();
  roi_width_ = lane_postprocessor_param_.roi_width();

  lane_type_num_ = static_cast<int>(kSpatialPositionIndexMap.size());
  AINFO << "lane_type_num_: " << lane_type_num_;
  return true;
}

bool DarkSCNNLanePostprocessor::Process2D(
    const LanePostprocessorOptions& options, CameraFrame* frame) {
  ADEBUG << "Begin to Process2D.";
  frame->lane_objects.clear();

  const auto start = std::chrono::high_resolution_clock::now();

  cv::Mat lane_map(lane_map_height_, lane_map_width_, CV_32FC1);
  memcpy(lane_map.data, frame->lane_detected_blob->cpu_data(),
         lane_map_width_ * lane_map_height_ * sizeof(float));

  SampleAndProjectLanePoints(lane_map, frame);

  auto elapsed_1 = std::chrono::high_resolution_clock::now() - start;
  int64_t microseconds_1 =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed_1).count();
  time_1 += microseconds_1;

  PerformRansacFitting(frame);

  auto elapsed_2 = std::chrono::high_resolution_clock::now() - start;
  int64_t microseconds_2 =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed_2).count();
  time_2 += microseconds_2 - microseconds_1;

  BuildLaneObjects(frame);

  AdjustLaneSpatialLabels(frame);

  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  int64_t microseconds =
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
  time_3 += microseconds - microseconds_2;
  ++time_num;

  ADEBUG << "frame->lane_objects.size(): " << frame->lane_objects.size();

  if (time_num > 0) {
    ADEBUG << "Avg sampling time: " << time_1 / time_num
           << " Avg fitting time: " << time_2 / time_num
           << " Avg writing time: " << time_3 / time_num;
  }
  ADEBUG << "darkSCNN lane_postprocess done!";
  return true;
}

void DarkSCNNLanePostprocessor::SampleAndProjectLanePoints(
    const cv::Mat& lane_map, CameraFrame* frame) {
  int y = static_cast<int>(lane_map.rows * kLaneYRatio - 1);
  int step_y = (y - kMinStepYStart) * (y - kMinStepYStart) / kStepYConstant + 1;

  const float scale_x = static_cast<float>(roi_width_) / lane_map.cols;
  const float scale_y = static_cast<float>(roi_height_) / lane_map.rows;

  xy_points.clear();
  xy_points.resize(lane_type_num_);
  uv_points.clear();
  uv_points.resize(lane_type_num_);

  Eigen::Matrix<float, 3, 1> img_point;
  Eigen::Matrix<float, 3, 1> xy_p;
  Eigen::Matrix<float, 2, 1> xy_point;
  Eigen::Matrix<float, 2, 1> uv_point;

  while (y > 0) {
    const float scaled_y =
        static_cast<float>(y) * scale_y + static_cast<float>(roi_start_);

    for (int x = kPixelOffsetStart; x < lane_map.cols - kPixelOffsetEnd; ++x) {
      int value = static_cast<int>(round(lane_map.at<float>(y, x)));

      if ((value > 0 && value < kLaneValueLeftMin) ||
          kLaneValueCurbLeft == value) {
        if (value != static_cast<int>(round(lane_map.at<float>(y, x + 1)))) {
          const float scaled_x = static_cast<float>(x) * scale_x;
          img_point << scaled_x, scaled_y, kHomographyZCoordinate;
          xy_p = trans_mat_ * img_point;

          if (std::fabs(xy_p(2)) < kHomographyEpsilon) {
            continue;
          }
          xy_point << xy_p(0) / xy_p(2), xy_p(1) / xy_p(2);

          if (xy_point(0) < kXMinThreshold ||
              xy_point(0) > max_longitudinal_distance_ ||
              std::abs(xy_point(1)) > kMaxYThreshold) {
            continue;
          }
          uv_point << scaled_x, scaled_y;
          if (xy_points[value].size() < minNumPoints_ ||
              xy_point(0) < kMinXThreshold ||
              std::fabs(xy_point(1) - xy_points[value].back()(1)) <
                  kYCoordinateThreshold) {
            xy_points[value].emplace_back(xy_point);
            uv_points[value].emplace_back(uv_point);
          }
        }
      } else if (value >= kLaneValueLeftMin && value < lane_type_num_) {
        if (value != static_cast<int>(round(lane_map.at<float>(y, x - 1)))) {
          const float scaled_x = static_cast<float>(x) * scale_x;
          img_point << scaled_x, scaled_y, kHomographyZCoordinate;
          xy_p = trans_mat_ * img_point;

          if (std::fabs(xy_p(2)) < kHomographyEpsilon) {
            continue;
          }
          xy_point << xy_p(0) / xy_p(2), xy_p(1) / xy_p(2);
          if (xy_point(0) < kXMinThreshold ||
              xy_point(0) > max_longitudinal_distance_ ||
              std::abs(xy_point(1)) > kMaxYThreshold) {
            continue;
          }
          uv_point << scaled_x, scaled_y;
          if (xy_points[value].size() < minNumPoints_ ||
              xy_point(0) < kMinXThreshold ||
              std::fabs(xy_point(1) - xy_points[value].back()(1)) <
                  kYCoordinateThreshold) {
            xy_points[value].emplace_back(xy_point);
            uv_points[value].emplace_back(uv_point);
          }
        } else if (value >= lane_type_num_) {
          AWARN << "Lane line value shouldn't be equal or more than: "
                << lane_type_num_;
        }
      }
    }
    step_y =
        (y - kMinStepYStartAlt) * (y - kMinStepYStartAlt) / kStepYConstant + 1;
    y -= step_y;
  }
}

void DarkSCNNLanePostprocessor::PerformRansacFitting(CameraFrame* frame) {
  std::vector<Eigen::Matrix<float, 2, 1>> selected_xy_points;
  Eigen::Matrix<float, 4, 1> coeff;
  coeffs_.resize(lane_type_num_);
  for (int i = 1; i < lane_type_num_; ++i) {
    coeffs_[i] << 0, 0, 0, 0;
    if (xy_points[i].size() < minNumPoints_) {
      continue;
    }
    if (RansacFitting<float>(xy_points[i], &selected_xy_points, &coeff,
                             kRansacIterations, static_cast<int>(minNumPoints_),
                             kRansacOutlierThreshold)) {
      coeffs_[i] = coeff;
      xy_points[i].clear();
      xy_points[i] = selected_xy_points;
    } else {
      xy_points[i].clear();
    }
  }
}

std::vector<float> DarkSCNNLanePostprocessor::CalculateLaneCurveValues() {
  std::vector<float> c0s(lane_type_num_, 0);
  for (int i = 1; i < lane_type_num_; ++i) {
    if (xy_points[i].size() < minNumPoints_) {
      continue;
    }
    c0s[i] = GetPolyValue(static_cast<float>(coeffs_[i](3)),
                          static_cast<float>(coeffs_[i](2)),
                          static_cast<float>(coeffs_[i](1)),
                          static_cast<float>(coeffs_[i](0)), kPolyEvalX);
  }
  return c0s;
}

void DarkSCNNLanePostprocessor::SwapLanePointsForCenterLane(
    std::vector<float>* c0s) {
  if (xy_points[kLaneIndexEgoLeft].size() < minNumPoints_ &&
      xy_points[kLaneIndexEgoCenter].size() >= minNumPoints_) {
    std::swap(xy_points[kLaneIndexEgoLeft], xy_points[kLaneIndexEgoCenter]);
    std::swap(uv_points[kLaneIndexEgoLeft], uv_points[kLaneIndexEgoCenter]);
    std::swap(coeffs_[kLaneIndexEgoLeft], coeffs_[kLaneIndexEgoCenter]);
    std::swap((*c0s)[kLaneIndexEgoLeft], (*c0s)[kLaneIndexEgoCenter]);
  } else if (xy_points[kLaneIndexEgoRight].size() < minNumPoints_ &&
             xy_points[kLaneIndexEgoCenter].size() >= minNumPoints_) {
    std::swap(xy_points[kLaneIndexEgoRight], xy_points[kLaneIndexEgoCenter]);
    std::swap(uv_points[kLaneIndexEgoRight], uv_points[kLaneIndexEgoCenter]);
    std::swap(coeffs_[kLaneIndexEgoRight], coeffs_[kLaneIndexEgoCenter]);
    std::swap((*c0s)[kLaneIndexEgoRight], (*c0s)[kLaneIndexEgoCenter]);
  }
}

void DarkSCNNLanePostprocessor::SwapLanePointsForBoundary(
    std::vector<float>* c0s) {
  static constexpr int kLeftLaneStartIndex = 3;
  static constexpr int kLeftLaneEndIndex = 1;
  static constexpr int kRightLaneStartIndex = 7;
  static constexpr int kRightLaneEndIndex = 9;

  if (xy_points[kLaneIndexEgoLeft].size() < minNumPoints_ &&
      xy_points[kLaneIndexCurbLeft].size() >= minNumPoints_) {
    bool use_boundary = true;
    for (int k = kLeftLaneStartIndex; k >= kLeftLaneEndIndex; --k) {
      if (xy_points[k].size() >= minNumPoints_) {
        use_boundary = false;
        break;
      }
    }
    if (use_boundary) {
      std::swap(xy_points[kLaneIndexEgoLeft], xy_points[kLaneIndexCurbLeft]);
      std::swap(uv_points[kLaneIndexEgoLeft], uv_points[kLaneIndexCurbLeft]);
      std::swap(coeffs_[kLaneIndexEgoLeft], coeffs_[kLaneIndexCurbLeft]);
      std::swap((*c0s)[kLaneIndexEgoLeft], (*c0s)[kLaneIndexCurbLeft]);
    }
  }

  if (xy_points[kLaneIndexEgoRight].size() < minNumPoints_ &&
      xy_points[kLaneIndexCurbRight].size() >= minNumPoints_) {
    bool use_boundary = true;
    for (int k = kRightLaneStartIndex; k <= kRightLaneEndIndex; ++k) {
      if (xy_points[k].size() >= minNumPoints_) {
        use_boundary = false;
        break;
      }
    }
    if (use_boundary) {
      std::swap(xy_points[kLaneIndexEgoRight], xy_points[kLaneIndexCurbRight]);
      std::swap(uv_points[kLaneIndexEgoRight], uv_points[kLaneIndexCurbRight]);
      std::swap(coeffs_[kLaneIndexEgoRight], coeffs_[kLaneIndexCurbRight]);
      std::swap((*c0s)[kLaneIndexEgoRight], (*c0s)[kLaneIndexCurbRight]);
    }
  }
}

void DarkSCNNLanePostprocessor::CreateLaneObjectsFromPoints(
    CameraFrame* frame, const std::vector<float>& c0s) {
  static constexpr int kLeftBoundaryIndex = 5;
  static constexpr int kRightBoundaryIndex = 5;
  static constexpr int kSortCount = 10;
  static constexpr int kCurbLeftIndex = 11;
  static constexpr int kCurbRightIndex = 12;
  static constexpr int kLastSortIndex = 9;
  static constexpr int kFirstSortIndex = 0;

  for (int i = 1; i < lane_type_num_; ++i) {
    base::LaneLine cur_object;
    if (xy_points[i].size() < minNumPoints_) {
      continue;
    }

    cur_object.pos_type = kSpatialPositionLUT[i];

    if ((i < kLeftBoundaryIndex && c0s[i] < c0s[i + 1]) ||
        (i > kRightBoundaryIndex && i < kSortCount && c0s[i] > c0s[i - 1])) {
      continue;
    }
    if (kCurbLeftIndex == i || kCurbRightIndex == i) {
      std::vector<float> sorted_c0s(c0s.begin(), c0s.begin() + kSortCount);
      std::sort(sorted_c0s.begin(), sorted_c0s.end());
      if ((sorted_c0s[i] > sorted_c0s[kFirstSortIndex] &&
           kCurbRightIndex == i) ||
          (sorted_c0s[i] < sorted_c0s[kLastSortIndex] && kCurbLeftIndex == i)) {
        continue;
      }
    }

    cur_object.curve_car_coord.x_start =
        static_cast<float>(xy_points[i].front()(0));
    cur_object.curve_car_coord.x_end =
        static_cast<float>(xy_points[i].back()(0));
    cur_object.curve_car_coord.a = static_cast<float>(coeffs_[i](3));
    cur_object.curve_car_coord.b = static_cast<float>(coeffs_[i](2));
    cur_object.curve_car_coord.c = static_cast<float>(coeffs_[i](1));
    cur_object.curve_car_coord.d = static_cast<float>(coeffs_[i](0));

    cur_object.curve_car_coord_point_set.clear();
    base::Point2DF p_j;
    for (size_t j = 0; j < xy_points[i].size(); ++j) {
      p_j.x = static_cast<float>(xy_points[i][j](0));
      p_j.y = static_cast<float>(xy_points[i][j](1));
      cur_object.curve_car_coord_point_set.emplace_back(p_j);
    }

    cur_object.curve_image_point_set.clear();
    for (size_t j = 0; j < uv_points[i].size(); ++j) {
      p_j.x = static_cast<float>(uv_points[i][j](0));
      p_j.y = static_cast<float>(uv_points[i][j](1));
      cur_object.curve_image_point_set.emplace_back(p_j);
    }

    cur_object.confidence = 1.0f;
    frame->lane_objects.emplace_back(cur_object);
  }
}

void DarkSCNNLanePostprocessor::BuildLaneObjects(CameraFrame* frame) {
  std::vector<float> c0s = CalculateLaneCurveValues();
  SwapLanePointsForCenterLane(&c0s);
  SwapLanePointsForBoundary(&c0s);
  CreateLaneObjectsFromPoints(frame, c0s);
}

void DarkSCNNLanePostprocessor::AdjustLaneSpatialLabels(CameraFrame* frame) {
  int has_center_ = 0;
  for (auto lane_ : frame->lane_objects) {
    if (base::LaneLinePositionType::EGO_CENTER == lane_.pos_type) {
      if (lane_.curve_car_coord.d >= 0) {
        has_center_ = 1;
      } else if (lane_.curve_car_coord.d < 0) {
        has_center_ = 2;
      }
      break;
    }
  }

  if (1 == has_center_) {
    for (auto& lane_ : frame->lane_objects) {
      int spatial_id = kSpatialPositionIndexMap[lane_.pos_type];
      if (spatial_id >= kSpatialRangeLeftStart &&
          spatial_id <= kSpatialRangeLeftEnd) {
        lane_.pos_type = kSpatialPositionLUT[spatial_id - 1];
      }
    }
  } else if (2 == has_center_) {
    for (auto& lane_ : frame->lane_objects) {
      int spatial_id = kSpatialPositionIndexMap[lane_.pos_type];
      if (spatial_id >= kSpatialRangeRightStart &&
          spatial_id <= kSpatialRangeRightEnd) {
        lane_.pos_type = kSpatialPositionLUT[spatial_id + 1];
      }
    }
  }
}

bool DarkSCNNLanePostprocessor::Process3D(
    const LanePostprocessorOptions& options, CameraFrame* frame) {
  ConvertImagePoint2Camera(frame);
  PolyFitCameraLaneline(frame);
  return true;
}

void DarkSCNNLanePostprocessor::ConvertImagePoint2Camera(CameraFrame* frame) {
  float pitch_angle = frame->calibration_service->QueryPitchAngle();
  float camera_ground_height =
      frame->calibration_service->QueryCameraToGroundHeight();
  const Eigen::Matrix3f& intrinsic_params = frame->camera_k_matrix;
  const Eigen::Matrix3f& intrinsic_params_inverse = intrinsic_params.inverse();
  std::vector<base::LaneLine>& lane_objects = frame->lane_objects;
  int laneline_num = static_cast<int>(lane_objects.size());
  base::Point3DF camera_point;
  Eigen::Vector3d camera_point3d;
  for (int line_index = 0; line_index < laneline_num; ++line_index) {
    std::vector<base::Point2DF>& image_point_set =
        lane_objects[line_index].curve_image_point_set;
    std::vector<base::Point3DF>& camera_point_set =
        lane_objects[line_index].curve_camera_point_set;
    for (int i = 0; i < static_cast<int>(image_point_set.size()); i++) {
      const base::Point2DF& image_point = image_point_set[i];
      ImagePoint2Camera(image_point, pitch_angle, camera_ground_height,
                        intrinsic_params_inverse, &camera_point3d);
      camera_point.x = static_cast<float>(camera_point3d(0));
      camera_point.y = static_cast<float>(camera_point3d(1));
      camera_point.z = static_cast<float>(camera_point3d(2));
      camera_point_set.emplace_back(camera_point);
    }
  }
}

void DarkSCNNLanePostprocessor::PolyFitCameraLaneline(CameraFrame* frame) {
  std::vector<base::LaneLine>& lane_objects = frame->lane_objects;
  int laneline_num = static_cast<int>(lane_objects.size());
  Eigen::Matrix<float, max_poly_order + 1, 1> camera_coeff;
  Eigen::Matrix<float, 2, 1> camera_pos;
  for (int line_index = 0; line_index < laneline_num; ++line_index) {
    const std::vector<base::Point3DF>& camera_point_set =
        lane_objects[line_index].curve_camera_point_set;
    float x_start = camera_point_set[0].z;
    float x_end = 0.0f;
    std::vector<Eigen::Matrix<float, 2, 1>> camera_pos_vec;
    camera_pos_vec.reserve(camera_point_set.size());
    for (int i = 0; i < static_cast<int>(camera_point_set.size()); ++i) {
      x_end = std::max(camera_point_set[i].z, x_end);
      x_start = std::min(camera_point_set[i].z, x_start);
      camera_pos << camera_point_set[i].z, camera_point_set[i].x;
      camera_pos_vec.emplace_back(camera_pos);
    }

    bool is_x_axis = true;
    bool fit_flag =
        PolyFit(camera_pos_vec, max_poly_order, &camera_coeff, is_x_axis);
    if (!fit_flag) {
      continue;
    }
    lane_objects[line_index].curve_camera_coord.a = camera_coeff(3, 0);
    lane_objects[line_index].curve_camera_coord.b = camera_coeff(2, 0);
    lane_objects[line_index].curve_camera_coord.c = camera_coeff(1, 0);
    lane_objects[line_index].curve_camera_coord.d = camera_coeff(0, 0);
    lane_objects[line_index].curve_camera_coord.x_start = x_start;
    lane_objects[line_index].curve_camera_coord.x_end = x_end;
    lane_objects[line_index].use_type = base::LaneLineUseType::REAL;
  }
}

std::string DarkSCNNLanePostprocessor::Name() const {
  return "DarkSCNNLanePostprocessor";
}

REGISTER_LANE_POSTPROCESSOR(DarkSCNNLanePostprocessor);

}  // namespace camera
}  // namespace perception
}  // namespace century
