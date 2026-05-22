/******************************************************************************
 * Copyright 2019 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/tool/benchmark/lidar/eval/lidar_option.h"

#include <map>
#include <set>
#include <string>

#include "modules/perception/tool/benchmark/lidar/base/point_cloud_frame.h"
#include "modules/perception/tool/benchmark/lidar/eval/frame_statistics.h"
#include "modules/perception/tool/benchmark/lidar/eval/meta_statistics.h"
#include "modules/perception/tool/benchmark/lidar/eval/position_metric.h"

namespace century {
namespace perception {
namespace benchmark {

namespace {

bool HasSingleValue(const std::set<std::string>& values) {
  return (1 == values.size());
}

void SetJaccardOption(
    const std::map<std::string, std::set<std::string>>& options) {
  auto iter = options.find("JACCARD");
  if (options.end() == iter) {
    return;
  }
  if (HasSingleValue(iter->second)) {
    double jaccard_threshold = std::stof(*iter->second.begin());
    FrameStatistics::set_jaccard_index_threshold(jaccard_threshold);
  }
}

void SetJaccardPercentileOption(
    const std::map<std::string, std::set<std::string>>& options) {
  auto iter = options.find("JACCARD_PERCENTILE");
  if (options.end() == iter) {
    return;
  }
  if (HasSingleValue(iter->second)) {
    double jaccard_percentile = std::stof(*iter->second.begin());
    FrameStatistics::set_jaccard_index_percentile(jaccard_percentile);
  }
}

void SetRangeOption(
    const std::map<std::string, std::set<std::string>>& options) {
  auto iter = options.find("RANGE");
  if (options.end() == iter) {
    return;
  }
  if (!HasSingleValue(iter->second)) {
    return;
  }
  const std::string& range_type = *iter->second.begin();
  if ("view" == range_type) {
    MetaStatistics::set_range_type(VIEW);
  } else if ("box" == range_type) {
    MetaStatistics::set_range_type(BOX);
  } else if ("roi" == range_type) {
    MetaStatistics::set_range_type(ROI);
  } else {
    MetaStatistics::set_range_type(DISTANCE);
  }
}

void SetIgnoreOutsideRoiOption(
    const std::map<std::string, std::set<std::string>>& options) {
  auto iter = options.find("IGNORE_OUTSIDE_ROI");
  if (options.end() == iter) {
    return;
  }
  if (!HasSingleValue(iter->second)) {
    return;
  }
  if ("true" == *iter->second.begin()) {
    RoiDistanceBasedRangeInterface::set_ignore_roi_outside(true);
  } else {
    RoiDistanceBasedRangeInterface::set_ignore_roi_outside(false);
  }
}

void SetLabelBlackListOption(
    const std::map<std::string, std::set<std::string>>& options) {
  auto iter = options.find("LABEL_BLACK_LIST");
  if (options.end() == iter) {
    return;
  }
  Frame::set_black_list(iter->second);
}

void SetOverallDistanceOption(
    const std::map<std::string, std::set<std::string>>& options) {
  auto iter = options.find("OVERALL_DISTANCE");
  if (options.end() == iter) {
    return;
  }
  if (HasSingleValue(iter->second)) {
    double overall_distance = std::stof(*iter->second.begin());
    DistanceBasedRangeInterface::set_distance(overall_distance);
  }
}

void SetFrontAngleOption(
    const std::map<std::string, std::set<std::string>>& options) {
  auto iter = options.find("FRONT_ANGLE");
  if (options.end() == iter) {
    return;
  }
  if (HasSingleValue(iter->second)) {
    double front_angle = std::stof(*iter->second.begin());
    ViewBasedRangeInterface::set_front_view_angle(front_angle);
  }
}

void SetRearAngleOption(
    const std::map<std::string, std::set<std::string>>& options) {
  auto iter = options.find("REAR_ANGLE");
  if (options.end() == iter) {
    return;
  }
  if (HasSingleValue(iter->second)) {
    double rear_angle = std::stof(*iter->second.begin());
    ViewBasedRangeInterface::set_rear_view_angle(rear_angle);
  }
}

void SetFrontDistanceOption(
    const std::map<std::string, std::set<std::string>>& options) {
  auto iter = options.find("FRONT_DISTANCE");
  if (options.end() == iter) {
    return;
  }
  if (HasSingleValue(iter->second)) {
    double front_distance = std::stof(*iter->second.begin());
    ViewBasedRangeInterface::set_front_view_distance(front_distance);
    BoxBasedRangeInterface::set_front_box_distance(front_distance);
  }
}

void SetRearDistanceOption(
    const std::map<std::string, std::set<std::string>>& options) {
  auto iter = options.find("REAR_DISTANCE");
  if (options.end() == iter) {
    return;
  }
  if (HasSingleValue(iter->second)) {
    double rear_distance = std::stof(*iter->second.begin());
    ViewBasedRangeInterface::set_rear_view_distance(rear_distance);
    BoxBasedRangeInterface::set_rear_box_distance(rear_distance);
  }
}

void SetSideDistanceOption(
    const std::map<std::string, std::set<std::string>>& options) {
  auto iter = options.find("SIDE_DISTANCE");
  if (options.end() == iter) {
    return;
  }
  if (HasSingleValue(iter->second)) {
    double side_distance = std::stof(*iter->second.begin());
    BoxBasedRangeInterface::set_side_box_distance(side_distance);
  }
}

void SetCloudTypeOption(
    const std::map<std::string, std::set<std::string>>& options) {
  auto iter = options.find("CLOUD_TYPE");
  if (options.end() == iter) {
    return;
  }
  if (HasSingleValue(iter->second)) {
    std::string cloud_type = *iter->second.begin();
    PointCloudFrame::set_cloud_type(cloud_type);
  }
}

void SetPenalizePiOption(
    const std::map<std::string, std::set<std::string>>& options) {
  auto iter = options.find("PENALIZE_PI");
  if (options.end() == iter) {
    return;
  }
  if (!HasSingleValue(iter->second)) {
    return;
  }
  const std::string& is_penalize_pi = *iter->second.begin();
  if ("true" == is_penalize_pi) {
    OrientationSimilarityMetric::penalize_pi = true;
  } else if ("false" == is_penalize_pi) {
    OrientationSimilarityMetric::penalize_pi = false;
  }
}

void SetRecallDimOption(
    const std::map<std::string, std::set<std::string>>& options) {
  auto iter = options.find("RECALL_DIM");
  if (options.end() == iter) {
    return;
  }
  if (HasSingleValue(iter->second)) {
    unsigned int recall_dim = std::stoi(*iter->second.begin());
    MetaStatistics::set_recall_dim(recall_dim);
  }
}

void SetVisibleOption(
    const std::map<std::string, std::set<std::string>>& options) {
  auto iter = options.find("VISIBLE");
  if (options.end() == iter) {
    return;
  }
  if (HasSingleValue(iter->second)) {
    float visible_threshold = std::stof(*iter->second.begin());
    Frame::set_visible_threshold(visible_threshold);
  }
}

void SetConfidenceOption(
    const std::map<std::string, std::set<std::string>>& options) {
  auto iter = options.find("CONFIDENCE");
  if (options.end() == iter) {
    return;
  }
  if (HasSingleValue(iter->second)) {
    float min_confidence = std::stof(*iter->second.begin());
    Frame::set_min_confidence(min_confidence);
  }
}

void SetRoiTypeOption(
    const std::map<std::string, std::set<std::string>>& options) {
  auto iter = options.find("ROI_TYPE");
  if (options.end() == iter) {
    return;
  }
  if (HasSingleValue(iter->second)) {
    bool is_main_lanes = ("LANE" == *iter->second.begin());
    FrameStatistics::set_roi_is_main_lanes(is_main_lanes);
  }
}

}  // namespace

bool LidarOption::set_options() const {
  SetJaccardOption(_options);
  SetJaccardPercentileOption(_options);
  SetRangeOption(_options);
  SetIgnoreOutsideRoiOption(_options);
  SetLabelBlackListOption(_options);
  SetOverallDistanceOption(_options);
  SetFrontAngleOption(_options);
  SetRearAngleOption(_options);
  SetFrontDistanceOption(_options);
  SetRearDistanceOption(_options);
  SetSideDistanceOption(_options);
  SetCloudTypeOption(_options);
  SetPenalizePiOption(_options);
  SetRecallDimOption(_options);
  SetVisibleOption(_options);
  SetConfidenceOption(_options);
  SetRoiTypeOption(_options);
  return true;
}

}  // namespace benchmark
}  // namespace perception
}  // namespace century
