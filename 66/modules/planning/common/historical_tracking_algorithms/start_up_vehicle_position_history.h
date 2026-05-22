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
 * @file start_up_vehicle_position_history.h
 **/

#pragma once

#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/planning/proto/common_config/historical_tracking_config.pb.h"

#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path/frenet_frame_path.h"
#include "modules/planning/common/reference_line_info.h"

namespace century {
namespace planning {

class StartUpVehiclePositionHistory {
 private:
  struct ObstacleInfo {
    ObstacleInfo(uint32_t seq_num, common::math::Vec2d position,
                 double timestamp)
        : last_seq_num(seq_num) {
      values.emplace_back(position, timestamp);
    }
    uint32_t last_seq_num;
    std::deque<std::pair<common::math::Vec2d, double>> values;
  };

 public:
  explicit StartUpVehiclePositionHistory(size_t capacity);
  virtual ~StartUpVehiclePositionHistory() {}

  bool Update();
  static void ResetUpdateState() { has_updated_ = false; }
  void SetCapacity(size_t capacity) { capacity_ = capacity; }
  size_t GetElementsNum() { return vehicle_container_.size(); }
  size_t GetCapacitySize() { return capacity_; }
  static void SetSequenceNum(uint32_t seq_num) { sequence_num_ = seq_num; }
  bool GetVehicleIsStartUp(const Obstacle& obs);
  void SetReferenceLine(const ReferenceLineInfo* reference_line_info) {
    reference_line_info_ = reference_line_info;
  }

  static void SetFrenetPath(const FrenetFramePath& frenet_path) {
    last_frame_frenet_path_.clear();
    last_frame_frenet_path_ = frenet_path;
  }

 private:
  void UpdateUnknownObstacleInfo();
  void UpdateVehicleInfo();
  bool AddToSplitUnknownObs(const Obstacle& obstacle,
                            const std::string vehicle_id);
  void FuseUnknownObsToVehicle();
  size_t ClearVehicleObsoleteElements();
  size_t ClearUnknownObsoleteElements();
  bool IsVehicleObstacleInScope(const Obstacle& obstacle);
  bool IsUnknownObstacleInScope(const Obstacle& obstacle);
  bool GetScopeState(const Obstacle& obstacle, double left_scope,
                     double right_scope, double front_scope, double rear_scope);
  void GetScopeParams(const bool is_unknown, double* left_scope,
                      double* right_scope, double* front_scope,
                      double* rear_scope);
  double CountLargeValueTimes(
      const std::vector<std::pair<double, double>>& ave_speed,
      double comp_large_val, double comp_small_val);
  bool IsStartUpRules(double large_times, size_t ave_speed_size);
  bool IsObsOnTurnPath(const Obstacle& obs);
  double GetPathPointThetaByS(const double ref_s);

 private:
  common_config::StartUpVehicleHistoryConfig config_;
  std::unordered_map<std::string, ObstacleInfo> vehicle_container_;
  std::unordered_map<std::string, std::pair<uint32_t, std::string>>
      split_unknown_obs_container_;
  std::unordered_map<std::string, bool> vehicle_lat_status_;
  static FrenetFramePath last_frame_frenet_path_;
  static uint32_t sequence_num_;
  const ReferenceLineInfo* reference_line_info_;
  size_t capacity_;
  static bool has_updated_;
};

}  // namespace planning
}  // namespace century
