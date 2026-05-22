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
 * @file start_up_vehicle_position_history.cc
 **/

#include "modules/planning/common/historical_tracking_algorithms/start_up_vehicle_position_history.h"

#include <algorithm>
#include <array>
#include <iomanip>
#include <limits>
#include <string>

#include "cyber/common/file.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/sl_boundary.pb.h"

namespace century {
namespace planning {

namespace {
constexpr uint32_t kLowNumObsoleteSeqNum = 5U;
constexpr uint32_t kHighNumObsoleteSeqNum = 2U;
constexpr uint32_t kMinValuesSize = 5U;
constexpr size_t kAverageNumber = 3UL;
constexpr size_t kStepSize = 3UL;
constexpr size_t kFilterSize = 1UL;

constexpr double kStartUpThresholdOne = 1.0;
constexpr double kStartUpThresholdTwo = 1.5;
constexpr double kStartUpThresholdThree = 2.0;
constexpr double kStartUpThresholdRatioMore = 0.75;
constexpr double kEpsilon = 1.0e-10;

}  // namespace

uint32_t StartUpVehiclePositionHistory::sequence_num_ = 0;
FrenetFramePath StartUpVehiclePositionHistory::last_frame_frenet_path_;
bool StartUpVehiclePositionHistory::has_updated_ = false;

StartUpVehiclePositionHistory::StartUpVehiclePositionHistory(size_t capacity)
    : capacity_(capacity) {
  vehicle_container_.clear();
  const std::string config_file =
      "/century/modules/planning/conf/common_config/"
      "historical_tracking_config.pb.txt";
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config_))
      << "Read config failed: " << config_file;
}

bool StartUpVehiclePositionHistory::Update() {
  UpdateVehicleInfo();
  UpdateUnknownObstacleInfo();
  FuseUnknownObsToVehicle();
  has_updated_ = true;
  return has_updated_;
}

double StartUpVehiclePositionHistory::CountLargeValueTimes(
    const std::vector<std::pair<double, double>>& ave_speed,
    double comp_large_val, double comp_small_val) {
  double ret = 0.0;
  for (uint32_t i = 0U; i < ave_speed.size(); ++i) {
    if (ave_speed[i].second > comp_large_val) {
      ret += (1.0 + kEpsilon);
    } else if (ave_speed[i].second > comp_small_val) {
      ret += (0.5 + kEpsilon);
    }
  }
  return ret;
}

bool StartUpVehiclePositionHistory::IsStartUpRules(double large_times,
                                                   size_t ave_speed_size) {
  bool is_start_up = false;
  switch (ave_speed_size) {
    case 0UL:
      is_start_up = false;
      break;
    case 1UL:
      is_start_up = large_times > kStartUpThresholdOne;
      break;
    case 2UL:
      is_start_up = large_times > kStartUpThresholdTwo;
      break;
    case 3UL:
      is_start_up = large_times > kStartUpThresholdThree;
      break;
    default:
      is_start_up = large_times > kStartUpThresholdRatioMore *
                                      static_cast<double>(ave_speed_size);
      break;
  }
  return is_start_up;
}

double StartUpVehiclePositionHistory::GetPathPointThetaByS(const double ref_s) {
  const auto frenet_point = last_frame_frenet_path_.EvaluateByS(ref_s);
  const ReferencePoint ref_point =
      reference_line_info_->reference_line().GetReferencePoint(
          frenet_point.s());
  const double path_point_theta =
      common::math::CartesianFrenetConverter::CalculateTheta(
          ref_point.heading(), ref_point.kappa(), frenet_point.l(),
          frenet_point.dl());
  return path_point_theta;
}

bool StartUpVehiclePositionHistory::IsObsOnTurnPath(const Obstacle& obs) {
  const auto& obs_sl = obs.PerceptionSLBoundary();
  const double obs_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
  const double path_point_theta = GetPathPointThetaByS(obs_s);
  const double front_path_point_theta =
      GetPathPointThetaByS(obs_s + config_.turn_path_front_buffer());
  const double rear_path_point_theta =
      GetPathPointThetaByS(obs_s - config_.turn_path_rear_buffer());

  const auto& adc_pos = reference_line_info_->vehicle_state();
  double diff_heading =
      std::abs(common::math::AngleDiff(adc_pos.heading(), path_point_theta)) *
      180.0 / M_PI;
  double diff_obs_heading = (std::abs(common::math::AngleDiff(
                                 front_path_point_theta, path_point_theta)) +
                             std::abs(common::math::AngleDiff(
                                 rear_path_point_theta, path_point_theta))) *
                            180.0 / M_PI;
  bool is_on_turn_path = diff_heading > config_.adc_turn_path_threshold() ||
                         diff_obs_heading > config_.obs_turn_path_threshold();
  ADEBUG << "is_on_turn_path = " << is_on_turn_path
         << ", obstacle id: " << obs.Id();
  return is_on_turn_path;
}

bool StartUpVehiclePositionHistory::GetVehicleIsStartUp(const Obstacle& obs) {
  if (!has_updated_) {
    AERROR << "Please Update the checking start up function.";
    return false;
  }
  const std::string id = std::to_string(obs.PerceptionId());
  auto it = vehicle_container_.find(id);
  auto is_left_state = vehicle_lat_status_.find(id);
  if (vehicle_container_.end() == it ||
      vehicle_lat_status_.end() == is_left_state) {
    ADEBUG << "No suitable vehicle.";
    return false;
  }
  const size_t values_size = it->second.values.size();
  if (values_size >= kAverageNumber + kStepSize + kFilterSize) {
    bool is_left_obs = vehicle_lat_status_[id];
    std::vector<std::pair<double, common::SLPoint>> ave_pos_info;
    size_t step_num =
        1UL + (values_size - kFilterSize - kAverageNumber) / kStepSize;
    for (size_t step = 0UL; step < step_num; ++step) {
      common::math::Vec2d ave_point(0.0, 0.0);
      common::SLPoint ave_point_sl;
      double ave_time = 0.0;
      for (auto value = it->second.values.rbegin() + step * kStepSize;
           value !=
           it->second.values.rbegin() + step * kStepSize + kAverageNumber;
           ++value) {
        ave_point += value->first;
        ave_time += value->second;
      }
      ave_point /= static_cast<double>(kAverageNumber);
      reference_line_info_->reference_line().XYToSL(ave_point, &ave_point_sl);
      ave_time /= static_cast<double>(kAverageNumber);
      ave_pos_info.emplace_back(ave_time, ave_point_sl);
    }
    ADEBUG << "is_left_obs: " << is_left_obs;
    std::vector<std::pair<double, double>> ave_speed;
    for (size_t index = 0UL; index < ave_pos_info.size() - 1UL; ++index) {
      const auto& curr_pos = ave_pos_info[index];
      const auto& prev_pos = ave_pos_info[index + 1UL];
      double lon_average_speed = (curr_pos.second.s() - prev_pos.second.s()) /
                                 (curr_pos.first - prev_pos.first + kEpsilon);
      double lat_average_speed = (curr_pos.second.l() - prev_pos.second.l()) /
                                 (curr_pos.first - prev_pos.first + kEpsilon);
      if (is_left_obs) {
        lat_average_speed = -lat_average_speed;
      }
      ave_speed.emplace_back(lon_average_speed, lat_average_speed);
      ADEBUG << "lat_average_speed: " << lat_average_speed;
      // ADEBUG << "lon_average_speed: " << lon_average_speed;
    }
    // ADEBUG << "obstacle id: " << obs.Id();
    // uint32_t i = 0U;
    // for (auto rit_value = it->second.values.rbegin();
    //      rit_value != it->second.values.rend(); ++rit_value, ++i) {
    //   common::SLPoint sl_point;
    //   reference_line_info_->reference_line().XYToSL(rit_value->first,
    //                                                 &sl_point);
    //   ADEBUG << "values(" << i << "): time[" << std::fixed
    //         << std::setprecision(3) << rit_value->second << "], pos["
    //         << sl_point.s() << ", " << sl_point.l() << "]";
    // }
    double threshold_level_one = config_.small_start_up_lat_speed();
    double threshold_level_two = config_.large_start_up_lat_speed();
    if (IsObsOnTurnPath(obs)) {
      threshold_level_one = config_.small_start_up_lat_speed_for_turn_path();
      threshold_level_two = config_.large_start_up_lat_speed_for_turn_path();
    }
    if (ave_speed[0].second < threshold_level_one) {
      return false;
    }
    double large_times = CountLargeValueTimes(ave_speed, threshold_level_two,
                                              threshold_level_one);
    bool is_start_up = IsStartUpRules(large_times, ave_speed.size());
    ADEBUG << "is_start_up:" << is_start_up << ", obstacle id: " << obs.Id();
    return is_start_up;
  }
  return false;
}

void StartUpVehiclePositionHistory::GetScopeParams(const bool is_unknown,
                                                   double* left_scope,
                                                   double* right_scope,
                                                   double* front_scope,
                                                   double* rear_scope) {
  CHECK_NOTNULL(left_scope);
  CHECK_NOTNULL(right_scope);
  CHECK_NOTNULL(front_scope);
  CHECK_NOTNULL(rear_scope);
  if (is_unknown) {
    *left_scope = config_.unknown_scope_range().left_scope_dis();
    *right_scope = config_.unknown_scope_range().right_scope_dis();
    *front_scope = config_.unknown_scope_range().front_scope_dis();
    *rear_scope = config_.unknown_scope_range().rear_scope_dis();
  } else {
    *left_scope = config_.vehicle_scope_range().left_scope_dis();
    *right_scope = config_.vehicle_scope_range().right_scope_dis();
    *front_scope = config_.vehicle_scope_range().front_scope_dis();
    *rear_scope = config_.vehicle_scope_range().rear_scope_dis();
  }
}
bool StartUpVehiclePositionHistory::GetScopeState(const Obstacle& obstacle,
                                                  double left_scope,
                                                  double right_scope,
                                                  double front_scope,
                                                  double rear_scope) {
  const auto& obs_sl = obstacle.PerceptionSLBoundary();
  const SLBoundary& adc_sl = reference_line_info_->AdcSlBoundary();
  const double obs_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
  if (last_frame_frenet_path_.size() < 5UL) {
    ADEBUG << "last_frame_frenet_path_.size() = "
           << last_frame_frenet_path_.size();
    return false;
  }
  const auto frenet_point = last_frame_frenet_path_.EvaluateByS(obs_s);
  const double adc_half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  bool is_lateral_in_scope =
      !(obs_sl.start_l() > frenet_point.l() + adc_half_width + left_scope ||
        obs_sl.end_l() < frenet_point.l() - adc_half_width - right_scope);
  bool is_longitudinal_in_scope =
      obs_sl.start_s() < adc_sl.end_s() + front_scope &&
      obs_sl.end_s() > adc_sl.start_s() - rear_scope;
  bool ret = false;
  if (is_lateral_in_scope && is_longitudinal_in_scope) {
    ret = true;
  }
  ADEBUG << "is_lateral_in_scope: " << is_lateral_in_scope;
  ADEBUG << "is_longitudinal_in_scope: " << is_longitudinal_in_scope;
  ADEBUG << "obs l: (" << obs_sl.start_l() << ", " << obs_sl.end_l() << ")";
  ADEBUG << "frenet point l : " << frenet_point.l();
  ADEBUG << "obs s: (" << obs_sl.start_s() << ", " << obs_sl.end_s() << ")";
  ADEBUG << "adc s: (" << adc_sl.start_s() << ", " << adc_sl.end_s() << ")";
  return ret;
}

bool StartUpVehiclePositionHistory::IsVehicleObstacleInScope(
    const Obstacle& obstacle) {
  bool ret = false;

  double left_scope = 0.0, right_scope = 0.0, front_scope = 0.0,
         rear_scope = 0.0;
  GetScopeParams(false, &left_scope, &right_scope, &front_scope, &rear_scope);

  ret =
      GetScopeState(obstacle, left_scope, right_scope, front_scope, rear_scope);
  ADEBUG << "IsVehicleObstacleInScope: " << ret;
  return ret;
}

bool StartUpVehiclePositionHistory::IsUnknownObstacleInScope(
    const Obstacle& obstacle) {
  bool ret = false;

  double left_scope = 0.0, right_scope = 0.0, front_scope = 0.0,
         rear_scope = 0.0;
  GetScopeParams(true, &left_scope, &right_scope, &front_scope, &rear_scope);

  ret =
      GetScopeState(obstacle, left_scope, right_scope, front_scope, rear_scope);
  ADEBUG << "IsUnknownObstacleInScope: " << ret;
  return ret;
}

void StartUpVehiclePositionHistory::UpdateVehicleInfo() {
  const auto& obstacles = reference_line_info_->path_decision().obstacles();
  vehicle_lat_status_.clear();
  for (const auto* obstacle : obstacles.Items()) {
    if (last_frame_frenet_path_.size() < 5UL) {
      break;
    }
    ADEBUG << "obstacle Id: " << obstacle->Id();
    const double curr_timestamp = obstacle->Perception().timestamp();
    if (curr_timestamp < kEpsilon) {
      ADEBUG << "Error Obstacle's Time Stamp.";
      continue;
    }
    bool is_vehicle = perception::PerceptionObstacle::VEHICLE ==
                          obstacle->Perception().type() ||
                      perception::PerceptionObstacle::BICYCLE ==
                          obstacle->Perception().type();
    if (!is_vehicle || !IsVehicleObstacleInScope(*obstacle)) {
      continue;
    }
    const double obs_s = (obstacle->PerceptionSLBoundary().start_s() +
                          obstacle->PerceptionSLBoundary().end_s()) *
                         0.5;
    const double obs_l = (obstacle->PerceptionSLBoundary().start_l() +
                          obstacle->PerceptionSLBoundary().end_l()) *
                         0.5;
    const auto frenet_point = last_frame_frenet_path_.EvaluateByS(obs_s);
    const bool is_left_obs = obs_l > frenet_point.l();
    // Get the id of the obstacle.
    const std::string id = std::to_string(obstacle->PerceptionId());
    vehicle_lat_status_.emplace(id, is_left_obs);
    common::SLPoint attention_sl;
    common::math::Vec2d attention_xy(0.0, 0.0);
    attention_sl.set_s(obstacle->PerceptionSLBoundary().start_s());
    if (is_left_obs) {
      attention_sl.set_l(obstacle->PerceptionSLBoundary().start_l());
    } else {
      attention_sl.set_l(obstacle->PerceptionSLBoundary().end_l());
    }
    reference_line_info_->reference_line().SLToXY(attention_sl, &attention_xy);
    // Check if the container has the obstacle.
    auto it = vehicle_container_.find(id);
    if (it != vehicle_container_.end()) {
      if (it->second.last_seq_num != sequence_num_) {
        // Add the new value to the values of the obstacle.
        it->second.values.emplace_back(attention_xy, curr_timestamp);
        // Check if the number of values is greater than the maximum record
        // times for start up.
        if (it->second.values.size() >
            config_.max_record_times() + kFilterSize) {
          // Remove the first element.
          it->second.values.pop_front();
        }
        it->second.last_seq_num = sequence_num_;
      }
    } else {
      // Clear the obsolete elements.
      auto erased_num = ClearVehicleObsoleteElements();
      ADEBUG << "erased number:" << erased_num;
      // Create a new element.
      ObstacleInfo obs_info(sequence_num_, attention_xy, curr_timestamp);
      vehicle_container_.emplace(id, obs_info);
    }
  }
}

bool StartUpVehiclePositionHistory::AddToSplitUnknownObs(
    const Obstacle& obstacle, const std::string vehicle_id) {
  const std::string id = std::to_string(obstacle.PerceptionId());
  auto split_unknown_info = split_unknown_obs_container_.find(id);
  if (split_unknown_obs_container_.end() != split_unknown_info) {
    split_unknown_info->second.first = sequence_num_;
    split_unknown_info->second.second = vehicle_id;
  } else {
    ClearUnknownObsoleteElements();
    split_unknown_obs_container_[id] = {sequence_num_, vehicle_id};
  }
  return true;
}

void StartUpVehiclePositionHistory::FuseUnknownObsToVehicle() {
  for (auto unknown_obs_info : split_unknown_obs_container_) {
    ADEBUG << "split_unknown_obs id(" << unknown_obs_info.first
           << "): passed sequence number"
           << sequence_num_ - unknown_obs_info.second.first
           << ", relative vehicle id: " << unknown_obs_info.second.second;
    if (unknown_obs_info.second.first != sequence_num_) {
      continue;
    }
    const auto& obstacles = reference_line_info_->path_decision().obstacles();
    const auto* unknown_obs = obstacles.Find(unknown_obs_info.first);
    // const auto* vehicle_obs_info = obstacles.Find();
    const std::string vehicle_id = unknown_obs_info.second.second;
    auto vehicle_obs_info = vehicle_container_.find(vehicle_id);
    auto is_left_state = vehicle_lat_status_.find(vehicle_id);
    if (nullptr == unknown_obs ||
        vehicle_container_.end() == vehicle_obs_info ||
        vehicle_lat_status_.end() == is_left_state) {
      continue;
    }
    bool is_left_obs = vehicle_lat_status_[vehicle_id];
    common::SLPoint unknown_attention_sl;
    common::SLPoint vehicle_attention_sl;
    // common::math::Vec2d vehicle_attention_xy;
    unknown_attention_sl.set_s(unknown_obs->PerceptionSLBoundary().start_s());
    if (is_left_obs) {
      unknown_attention_sl.set_l(unknown_obs->PerceptionSLBoundary().start_l());
    } else {
      unknown_attention_sl.set_l(unknown_obs->PerceptionSLBoundary().end_l());
    }
    common::math::Vec2d vehicle_attention_xy =
        vehicle_obs_info->second.values.back().first;
    reference_line_info_->reference_line().XYToSL(vehicle_attention_xy,
                                                  &vehicle_attention_sl);
    bool need_modify = false;
    if (unknown_attention_sl.s() < vehicle_attention_sl.s()) {
      vehicle_attention_sl.set_s(unknown_attention_sl.s());
      need_modify = true;
    }
    if (is_left_obs && unknown_attention_sl.l() < vehicle_attention_sl.l()) {
      vehicle_attention_sl.set_l(unknown_attention_sl.l());
      need_modify = true;
    }
    if (!is_left_obs && unknown_attention_sl.l() > vehicle_attention_sl.l()) {
      vehicle_attention_sl.set_l(unknown_attention_sl.l());
      need_modify = true;
    }
    if (need_modify) {
      common::math::Vec2d vehicle_attention_xy_modify(0.0, 0.0);
      reference_line_info_->reference_line().SLToXY(
          vehicle_attention_sl, &vehicle_attention_xy_modify);
      vehicle_obs_info->second.values.back().first =
          vehicle_attention_xy_modify;
    }
  }
  for (auto vehicle_info : vehicle_container_) {
    ADEBUG << "vehicle id(" << vehicle_info.first << "): passed sequence number"
           << sequence_num_ - vehicle_info.second.last_seq_num;
  }
}

void StartUpVehiclePositionHistory::UpdateUnknownObstacleInfo() {
  const auto& obstacles = reference_line_info_->path_decision().obstacles();
  for (const auto* obstacle : obstacles.Items()) {
    ADEBUG << "obstacle Id: " << obstacle->Id();
    bool is_unknown = perception::PerceptionObstacle::UNKNOWN ==
                          obstacle->Perception().type() ||
                      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE ==
                          obstacle->Perception().type() ||
                      perception::PerceptionObstacle::UNKNOWN_MOVABLE ==
                          obstacle->Perception().type();
    if (!is_unknown || !IsUnknownObstacleInScope(*obstacle)) {
      continue;
    }
    bool has_attributed_to_vehicle = false;
    const std::string id = std::to_string(obstacle->PerceptionId());
    auto split_unknown_info = split_unknown_obs_container_.find(id);
    if (split_unknown_obs_container_.end() != split_unknown_info) {
      const auto vehicle_info =
          vehicle_container_.find(split_unknown_info->second.second);
      if (vehicle_container_.end() != vehicle_info &&
          vehicle_info->second.last_seq_num == sequence_num_) {
        has_attributed_to_vehicle = true;
      }
    }
    if (has_attributed_to_vehicle) {
      has_attributed_to_vehicle = false;
      const auto* obs_vehicle =
          obstacles.Find(split_unknown_info->second.second);
      if (nullptr == obs_vehicle) {
        continue;
      }
      double dis = obs_vehicle->PerceptionPolygon().DistanceTo(
          obstacle->PerceptionPolygon());
      if (dis < config_.max_fusion_distance()) {
        split_unknown_info->second.first = sequence_num_;
        has_attributed_to_vehicle = true;
      }
    }
    if (!has_attributed_to_vehicle) {
      std::string min_dis_vehicle_id;
      double min_cost_dis = std::numeric_limits<double>::max();
      for (auto item : vehicle_container_) {
        if (item.second.last_seq_num != sequence_num_) {
          continue;
        }
        const auto* obs_vehicle = obstacles.Find(item.first);
        if (nullptr == obs_vehicle) {
          continue;
        }
        double dis = obs_vehicle->PerceptionPolygon().DistanceTo(
            obstacle->PerceptionPolygon());
        double overlap_area = 0.0;
        if (dis < kEpsilon) {
          common::math::Polygon2d overlap_polygon;
          obs_vehicle->PerceptionPolygon().ComputeOverlap(
              obstacle->PerceptionPolygon(), &overlap_polygon);
          overlap_area = overlap_polygon.area();
        }
        double cost_dis = dis - overlap_area;

        if (dis < config_.max_fusion_distance() && cost_dis < min_cost_dis) {
          min_cost_dis = cost_dis;
          min_dis_vehicle_id = item.first;
        }
      }
      if (min_cost_dis < config_.max_fusion_distance()) {
        AddToSplitUnknownObs(*obstacle, min_dis_vehicle_id);
      }
    }
  }
}

size_t StartUpVehiclePositionHistory::ClearVehicleObsoleteElements() {
  double obsolete_seq_num = kLowNumObsoleteSeqNum;
  if (vehicle_container_.size() < capacity_ / 2) {
    ADEBUG << "the container capacity is enough, so deleting nothing.";
    return 0UL;
  } else if (vehicle_container_.size() < 3 * capacity_ / 4) {
    obsolete_seq_num = kLowNumObsoleteSeqNum;
  } else {
    obsolete_seq_num = kHighNumObsoleteSeqNum;
  }
  size_t erase_num = 0;

  std::vector<std::string> to_remove;
  for (const auto& item : vehicle_container_) {
    if (sequence_num_ - item.second.last_seq_num > obsolete_seq_num) {
      to_remove.emplace_back(item.first);
    }
  }
  for (const auto& key : to_remove) {
    erase_num += vehicle_container_.erase(key);
  }
  return erase_num;
}

size_t StartUpVehiclePositionHistory::ClearUnknownObsoleteElements() {
  double obsolete_seq_num = kLowNumObsoleteSeqNum;
  if (split_unknown_obs_container_.size() < capacity_ / 2) {
    ADEBUG << "the container capacity is enough, so deleting nothing.";
    return 0UL;
  } else if (split_unknown_obs_container_.size() < 3 * capacity_ / 4) {
    obsolete_seq_num = kLowNumObsoleteSeqNum;
  } else {
    obsolete_seq_num = kHighNumObsoleteSeqNum;
  }
  size_t erase_num = 0;

  std::vector<std::string> to_remove;
  for (const auto& item : split_unknown_obs_container_) {
    if (sequence_num_ - item.second.first > obsolete_seq_num) {
      to_remove.emplace_back(item.first);
    }
  }
  for (const auto& key : to_remove) {
    erase_num += split_unknown_obs_container_.erase(key);
  }
  return erase_num;
}

}  // namespace planning
}  // namespace century
