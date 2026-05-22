/******************************************************************************
 * Copyright 2022 The Century Authors. All Rights Reserved.
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

/**
 * @file obstacle_heading_history.h
 **/

#pragma once

#include <deque>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path/frenet_frame_path.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/proto/common_config/historical_tracking_config.pb.h"

namespace century {
namespace planning {

class ObstacleHeadingHistory {
 private:
  struct ObstacleInfo {
    ObstacleInfo(uint32_t seq_num, double heading, double timestamp)
        : last_seq_num(seq_num) {
      values.emplace_back(heading, timestamp);
    }
    uint32_t last_seq_num;
    std::deque<std::pair<double, double>> values;
  };
  enum ObstaclePosition {
    LEFT_POSITION = 0,
    RIGHT_POSITION,
    LAT_OVERLAP,
  };
  using ObsContainer = std::unordered_map<std::string, ObstacleInfo>;

 public:
  explicit ObstacleHeadingHistory(size_t capacity);
  virtual ~ObstacleHeadingHistory() {}

  bool Update();
  static void ResetUpdateState() { has_updated_ = false; }
  void SetCapacity(size_t capacity) { capacity_ = capacity; }
  size_t GetElementsNum() { return obstacle_container_.size(); }
  size_t GetCapacitySize() { return capacity_; }
  static void SetSequenceNum(uint32_t seq_num) { sequence_num_ = seq_num; }
  bool GetIsSafeSidepass(const Obstacle& obs);
  void SetReferenceLine(const ReferenceLineInfo* reference_line_info) {
    reference_line_info_ = reference_line_info;
  }

  static void SetFrenetPath(const FrenetFramePath& frenet_path) {
    last_frame_frenet_path_.clear();
    last_frame_frenet_path_ = frenet_path;
    ADEBUG << "last_frame_frenet_path_ = " << last_frame_frenet_path_.size();
  }

 private:
  void UpdateObstacleInfo();
  bool GetIsSafeSidepassFromHeading(const Obstacle& obs,
                                    double obs_cur_heading);
  bool GetIsSafeSidepassFromHeadingRate(const Obstacle& obs,
                                        const size_t values_size,
                                        ObsContainer::iterator it);
  ObstaclePosition GetObstaclePosition(const Obstacle& obs);
  size_t ClearObstacleObsoleteElements();
  bool IsObstacleInScope(const Obstacle& obstacle);
  bool GetScopeState(const Obstacle& obstacle, double left_scope,
                     double right_scope, double front_scope, double rear_scope);
  void GetScopeParams(const bool is_unknown, double* left_scope,
                      double* right_scope, double* front_scope,
                      double* rear_scope);
  double ComputeCrossDistance(const common::math::Vec2d& adc_pos_xy,
                              double adc_heading,
                              const common::math::Vec2d& obs_attention_xy,
                              double obs_cur_heading);
  bool CaculSafeSidepassState(const Obstacle& obs,
                              const double cur_diff_heading,
                              const double ave_heading_rate);

 private:
  common_config::ObstacleHeadingHistoryConfig config_;
  ObsContainer obstacle_container_;
  std::unordered_map<std::string, bool> vehicle_lat_status_;
  static FrenetFramePath last_frame_frenet_path_;
  static uint32_t sequence_num_;
  const ReferenceLineInfo* reference_line_info_;
  size_t capacity_;
  static bool has_updated_;
};

}  // namespace planning
}  // namespace century
