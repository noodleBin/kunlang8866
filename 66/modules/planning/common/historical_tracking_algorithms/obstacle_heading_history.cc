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
 * @file obstacle_heading_history.cc
 **/

#include "modules/planning/common/historical_tracking_algorithms/obstacle_heading_history.h"

#include <algorithm>
#include <array>
#include <iomanip>
#include <limits>
#include <string>

#include "cyber/common/file.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/sl_boundary.pb.h"

namespace century {
namespace planning {

namespace {
constexpr uint32_t kLowNumObsoleteSeqNum = 5U;
constexpr uint32_t kHighNumObsoleteSeqNum = 2U;
constexpr size_t kAverageNumber = 3UL;
constexpr size_t kStepSize = 3UL;
constexpr size_t kFilterSize = 1UL;

constexpr double kAngle45 = 45.0;
constexpr double kAngle135 = 135.0;
constexpr double kAngleRateThreshold = 3.0;
constexpr double kEpsilon = 1.0e-10;
}  // namespace

uint32_t ObstacleHeadingHistory::sequence_num_ = 0;
FrenetFramePath ObstacleHeadingHistory::last_frame_frenet_path_;
bool ObstacleHeadingHistory::has_updated_ = false;

ObstacleHeadingHistory::ObstacleHeadingHistory(size_t capacity)
    : capacity_(capacity) {
  obstacle_container_.clear();
  const std::string config_file =
      "/century/modules/planning/conf/common_config/"
      "obstacle_heading_history_config.pb.txt";
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config_))
      << "Read config failed: " << config_file;
}

bool ObstacleHeadingHistory::Update() {
  UpdateObstacleInfo();
  has_updated_ = true;
  return has_updated_;
}

ObstacleHeadingHistory::ObstaclePosition
ObstacleHeadingHistory::GetObstaclePosition(const Obstacle& obs) {
  ObstaclePosition result_pos = LAT_OVERLAP;
  const auto& obs_sl = obs.PerceptionSLBoundary();
  const double obs_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
  const double obs_l = (obs_sl.start_l() + obs_sl.end_l()) * 0.5;
  const auto frenet_point = last_frame_frenet_path_.EvaluateByS(obs_s);
  const double adc_half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() * 0.5;
  bool is_lateral_overlap =
      !(obs_sl.start_l() >
            frenet_point.l() + adc_half_width + config_.left_buffer() ||
        obs_sl.end_l() <
            frenet_point.l() - adc_half_width - config_.right_buffer());
  if (is_lateral_overlap) {
    result_pos = LAT_OVERLAP;
  } else if (obs_l > frenet_point.l()) {
    result_pos = LEFT_POSITION;
  } else {
    result_pos = RIGHT_POSITION;
  }
  return result_pos;
}

// determine whether the line and the ray have an intersection point, and
// calculate the directed distance from the intersection point to ADC point.
double ObstacleHeadingHistory::ComputeCrossDistance(
    const common::math::Vec2d& adc_pos_xy, double adc_heading,
    const common::math::Vec2d& obs_attention_xy, double obs_cur_heading) {
  // the direction vector of a straight line
  common::math::Vec2d dA = common::math::Vec2d::CreateUnitVec2d(adc_heading);
  // The direction vector of the ray
  common::math::Vec2d dB =
      common::math::Vec2d::CreateUnitVec2d(obs_cur_heading);

  common::math::Vec2d AB = obs_attention_xy - adc_pos_xy;

  double cross_dA_dB = dA.CrossProd(dB);
  if (std::abs(cross_dA_dB) < std::numeric_limits<double>::epsilon()) {
    // If the cross product is close to zero, the line and ray are parallel or
    // collinear
    return std::numeric_limits<double>::lowest();
  }

  // Calculates the directed distance from the intersection to adc point A
  double t = AB.CrossProd(dB) / cross_dA_dB;

  return t;
}

bool ObstacleHeadingHistory::GetIsSafeSidepassFromHeading(
    const Obstacle& obs, double obs_cur_heading) {
  common::math::Vec2d obs_attention_xy(obs.Perception().position().x(),
                                       obs.Perception().position().y());

  const auto& adc_pos = reference_line_info_->vehicle_state();
  common::math::Vec2d adc_pos_xy(adc_pos.x(), adc_pos.y());
  double adc_heading = adc_pos.heading();
  double cross_dis = ComputeCrossDistance(adc_pos_xy, adc_heading,
                                          obs_attention_xy, obs_cur_heading);
  ADEBUG << "adc_heading = " << adc_heading * 180.0 / M_PI;
  ADEBUG << "obs_cur_heading = " << obs_cur_heading * 180.0 / M_PI;
  ADEBUG << "cross_dis = " << cross_dis;
  if (cross_dis < -config_.cross_dis_buffer()) {
    return true;
  }
  return false;
}

bool ObstacleHeadingHistory::CaculSafeSidepassState(
    const Obstacle& obs, const double cur_diff_heading,
    const double ave_heading_rate) {
  ObstaclePosition obs_pos = GetObstaclePosition(obs);
  bool is_safe_sidepass = false;
  switch (obs_pos) {
    case LEFT_POSITION:
      is_safe_sidepass = true;
      if (cur_diff_heading >= -kAngle135 && cur_diff_heading <= -kAngle45) {
        is_safe_sidepass = false;
      } else if (cur_diff_heading >= 0.0 && cur_diff_heading < kAngle45) {
        is_safe_sidepass = ave_heading_rate > 0.0;
      } else if (cur_diff_heading > -kAngle45 && cur_diff_heading < 0.0) {
        is_safe_sidepass = ave_heading_rate > config_.angle_rate_threshold();
      } else if (cur_diff_heading > kAngle135) {
        is_safe_sidepass = ave_heading_rate < 0.0;
      } else if (cur_diff_heading < -kAngle135) {
        is_safe_sidepass = ave_heading_rate < -config_.angle_rate_threshold();
      }
      break;
    case RIGHT_POSITION:
      is_safe_sidepass = false;
      if (cur_diff_heading >= -kAngle135 && cur_diff_heading <= -kAngle45) {
        is_safe_sidepass = true;
      } else if (cur_diff_heading >= 0.0 && cur_diff_heading < kAngle45) {
        is_safe_sidepass = ave_heading_rate < -config_.angle_rate_threshold();
      } else if (cur_diff_heading > -kAngle45 && cur_diff_heading < 0.0) {
        is_safe_sidepass = ave_heading_rate < 0.0;
      } else if (cur_diff_heading > kAngle135) {
        is_safe_sidepass = ave_heading_rate > config_.angle_rate_threshold();
      } else if (cur_diff_heading < -kAngle135) {
        is_safe_sidepass = ave_heading_rate > 0.0;
      }
      break;
    default:
      break;
  }
  ADEBUG << "obs_pos = " << obs_pos;
  ADEBUG << "cur_diff_heading = " << cur_diff_heading;
  ADEBUG << "ave_heading_rate = " << ave_heading_rate;
  ADEBUG << "is_safe_sidepass:" << is_safe_sidepass
         << ", obstacle id: " << obs.Id();
  return is_safe_sidepass;
}

bool ObstacleHeadingHistory::GetIsSafeSidepassFromHeadingRate(
    const Obstacle& obs, const size_t values_size, ObsContainer::iterator it) {
  std::vector<std::pair<double, double>> ave_heading_info;
  size_t step_num =
      1UL + (values_size - kFilterSize - kAverageNumber) / kStepSize;
  for (size_t step = 0UL; step < step_num; ++step) {
    double ave_heading = 0.0;
    double ave_time = 0.0;
    for (auto value = it->second.values.rbegin() + step * kStepSize;
         value !=
         it->second.values.rbegin() + step * kStepSize + kAverageNumber;
         ++value) {
      double cur_heading = value->first;
      if (cur_heading < 0.0) {
        cur_heading += (2.0 * M_PI);
      }
      ave_heading += cur_heading;
      ave_time += value->second;
    }
    ave_heading /= static_cast<double>(kAverageNumber);
    ave_heading = common::math::NormalizeAngle(ave_heading);
    ave_time /= static_cast<double>(kAverageNumber);
    ave_heading_info.emplace_back(ave_time, ave_heading);
  }
  double ave_heading_rate = 0.0;
  for (size_t index = 0UL; index < ave_heading_info.size() - 1UL; ++index) {
    const auto& curr_pos = ave_heading_info[index];
    const auto& prev_pos = ave_heading_info[index + 1UL];
    ave_heading_rate +=
        common::math::AngleDiff(prev_pos.second, curr_pos.second) /
        (curr_pos.first - prev_pos.first + kEpsilon);
  }
  ave_heading_rate /= (ave_heading_info.size() - 1UL);
  ave_heading_rate *= (180.0 / M_PI);

  double adc_heading = reference_line_info_->vehicle_state().heading();
  double cur_diff_heading =
      common::math::AngleDiff(adc_heading, ave_heading_info.front().second) *
      180.0 / M_PI;
  double is_safe_sidepass =
      CaculSafeSidepassState(obs, cur_diff_heading, ave_heading_rate);

  return is_safe_sidepass;
}

bool ObstacleHeadingHistory::GetIsSafeSidepass(const Obstacle& obs) {
  if (!has_updated_) {
    AERROR << "Please Update the checking start up function.";
    return false;
  }
  const std::string id = std::to_string(obs.PerceptionId());
  auto it = obstacle_container_.find(id);
  if (obstacle_container_.end() == it) {
    ADEBUG << "No suitable vehicle.";
    return false;
  }
  ADEBUG << "HeadingHistory obstacle id = " << id;
  // if (std::string::npos != id.find("113224")) {
  //   uint32_t i = 0U;
  //   for (auto rit_value = it->second.values.rbegin();
  //        rit_value != it->second.values.rend(); ++rit_value, ++i) {
  //     ADEBUG << "values(" << i << "): time[" << std::fixed
  //            << std::setprecision(3) << rit_value->second << "], heading["
  //            << rit_value->first << "]";
  //   }
  // }
  const size_t values_size = it->second.values.size();
  if (FLAGS_enable_cancel_stop_decision_from_heading &&
      values_size >= kFilterSize &&
      GetIsSafeSidepassFromHeading(obs, it->second.values.back().first)) {
    return true;
  }

  if (FLAGS_enable_cancel_stop_decision_from_heading_rate &&
      values_size >= kAverageNumber + kStepSize + kFilterSize &&
      GetIsSafeSidepassFromHeadingRate(obs, values_size, it)) {
    return true;
  }
  return false;
}

void ObstacleHeadingHistory::GetScopeParams(const bool is_unknown,
                                            double* left_scope,
                                            double* right_scope,
                                            double* front_scope,
                                            double* rear_scope) {
  CHECK_NOTNULL(left_scope);
  CHECK_NOTNULL(right_scope);
  CHECK_NOTNULL(front_scope);
  CHECK_NOTNULL(rear_scope);
  *left_scope = config_.obstacle_scope_range().left_scope_dis();
  *right_scope = config_.obstacle_scope_range().right_scope_dis();
  *front_scope = config_.obstacle_scope_range().front_scope_dis();
  *rear_scope = config_.obstacle_scope_range().rear_scope_dis();
}
bool ObstacleHeadingHistory::GetScopeState(const Obstacle& obstacle,
                                           double left_scope,
                                           double right_scope,
                                           double front_scope,
                                           double rear_scope) {
  const auto& obs_sl = obstacle.PerceptionSLBoundary();
  const SLBoundary& adc_sl = reference_line_info_->AdcSlBoundary();
  const double obs_s = (obs_sl.start_s() + obs_sl.end_s()) * 0.5;
  if (last_frame_frenet_path_.size() < 5UL) {
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

bool ObstacleHeadingHistory::IsObstacleInScope(const Obstacle& obstacle) {
  bool ret = false;

  double left_scope = 0.0, right_scope = 0.0, front_scope = 0.0,
         rear_scope = 0.0;
  GetScopeParams(false, &left_scope, &right_scope, &front_scope, &rear_scope);

  ret =
      GetScopeState(obstacle, left_scope, right_scope, front_scope, rear_scope);
  ADEBUG << "IsObstacleInScope: " << ret << "obstacle id: " << obstacle.Id();
  return ret;
}

void ObstacleHeadingHistory::UpdateObstacleInfo() {
  const auto& obstacles = reference_line_info_->path_decision().obstacles();
  ADEBUG << "UpdateObstacleInfo";
  for (const auto* obstacle : obstacles.Items()) {
    if (last_frame_frenet_path_.size() < 5UL) {
      AINFO << "last_frame_frenet_path_.size() = "
            << last_frame_frenet_path_.size();
      break;
    }
    const double curr_timestamp = obstacle->Perception().timestamp();
    if (curr_timestamp < kEpsilon) {
      AINFO << "Error Obstacle's Time Stamp.";
      continue;
    }
    bool is_unknown = perception::PerceptionObstacle::UNKNOWN ==
                          obstacle->Perception().type() ||
                      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE ==
                          obstacle->Perception().type() ||
                      perception::PerceptionObstacle::UNKNOWN_MOVABLE ==
                          obstacle->Perception().type();
    if (is_unknown || !IsObstacleInScope(*obstacle)) {
      continue;
    }
    const std::string id = std::to_string(obstacle->PerceptionId());
    auto it = obstacle_container_.find(id);

    if (it != obstacle_container_.end()) {
      if (it->second.last_seq_num != sequence_num_) {
        // Add the new value to the values of the obstacle.
        it->second.values.emplace_back(obstacle->SpeedHeading(),
                                       curr_timestamp);
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
      auto erased_num = ClearObstacleObsoleteElements();
      ADEBUG << "erased number:" << erased_num;
      // Create a new element.
      ObstacleInfo obs_info(sequence_num_, obstacle->SpeedHeading(),
                            curr_timestamp);
      obstacle_container_.emplace(id, obs_info);
    }
  }
}

size_t ObstacleHeadingHistory::ClearObstacleObsoleteElements() {
  double obsolete_seq_num = kLowNumObsoleteSeqNum;
  if (obstacle_container_.size() < capacity_ / 2) {
    ADEBUG << "the container capacity is enough, so deleting nothing.";
    return 0UL;
  } else if (obstacle_container_.size() < 3 * capacity_ / 4) {
    obsolete_seq_num = kLowNumObsoleteSeqNum;
  } else {
    obsolete_seq_num = kHighNumObsoleteSeqNum;
  }
  size_t erase_num = 0;

  std::vector<std::string> to_remove;
  for (const auto& item : obstacle_container_) {
    if (sequence_num_ - item.second.last_seq_num > obsolete_seq_num) {
      to_remove.emplace_back(item.first);
    }
  }
  for (const auto& key : to_remove) {
    erase_num += obstacle_container_.erase(key);
  }
  return erase_num;
}

}  // namespace planning
}  // namespace century
