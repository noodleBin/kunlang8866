/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
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
 * @file aeb_planner.cpp
 **/

#include "aeb_planner.h"

namespace {
constexpr int kFallbackTrackId = -1;
static int s_high_warning_frames_left = 0;
// for filter
static std::deque<::century::planning::AebWarningLevel> s_warning_level_history;
}  // namespace

namespace century {
namespace planning {

AebPlanner::AebPlanner(const AebConfig& aeb_config) : aeb_config_(aeb_config) {
  // vehicle parameter
  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
  average_wheel_angle_ = 0.0;
  is_turning_= false;
  has_obstacle_counter_ = 0;
  is_auto_enable_aeb_ = false;
  overlap_obstacle_map_ = {{INT_MAX, false}};
  turn_detector_.Reset(
      aeb_config_.enter_turn_threshold(), aeb_config_.exit_turn_threshold(),
      aeb_config_.enter_hold_frames(), aeb_config_.exit_hold_frames());

  ObstacleTracker::Config tracker_config;
  tracker_config.match_distance_threshold =
      aeb_config.match_distance_threshold();
  tracker_config.track_timeout = aeb_config.track_timeout();
  obstacle_tracker_.reset(new ObstacleTracker(tracker_config));
  high_warning_frames_map_.clear();
  track_obstacles_.clear();
  first_time_in_high_level_map_.clear();
}

bool AebPlanner::init() {
  ADEBUG << "aeb init !!!!";
  aeb_injector_ = std::make_shared<DependencyInjector>();
  if (!aeb_injector_) {
    ADEBUG << "create aeb injector failed, return false.";
    return false;
  }
  is_stop_by_closest_obstacle_ = false;
  calc_dis_is_overlap_ = false;
  calc_dis_is_both_overlap_ = false;
  is_auto_enable_aeb_ = false;
  std::pair<float, std::string> min_dis_obstacle_info_ =
      std::make_pair(std::numeric_limits<float>::max(), "");
  min_dis_obstacle_position_ = {std::numeric_limits<float>::max(),
                                std::numeric_limits<float>::max(),
                                std::numeric_limits<float>::max()};
  ttc_infos_ = std::make_tuple(
      std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  seq_num_ = 0;
  has_obstacle_counter_ = 0;
  v_lateral_ = 0.0;
  process_time_ = 0;
  average_wheel_angle_ = 0.0;
  std::string error_messages_ = "";
  speed_history_map_.clear();
  aeb_selected_obstacle_.Clear();
  original_distance_info_ =
          std::make_tuple(std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max(), nullptr);
  return true;
}

void AebPlanner::UpdateMapInfo() {
  UpdateHighWarningFramesMapInfo();
  UpdateFirstTimeInHighLevelMapInfo();
  return;
}

void AebPlanner::UpdateHighWarningFramesMapInfo() {
  for (auto it = high_warning_frames_map_.begin();
       it != high_warning_frames_map_.end();) {
    if (!obstacle_tracker_->HasTrack(it->first)) {
      it = high_warning_frames_map_.erase(it);
    } else {
      ++it;
    }
  }
  return;
}

void AebPlanner::UpdateFirstTimeInHighLevelMapInfo() {
  for (auto it = first_time_in_high_level_map_.begin();
       it != first_time_in_high_level_map_.end();) {
    if (!obstacle_tracker_->HasTrack(it->first)) {
      it = first_time_in_high_level_map_.erase(it);
    } else {
      ++it;
    }
  }
  return;
}

AebPlanner::AebTurnType AebPlanner::GetTurningResult(
    const LocalView& local_view) {
  return AebTurnType::NO_TURN;
}

::century::planning::AebWarningLevel AebPlanner::DowngradeFirstHighFrame(
    const int& track_id, ::century::planning::AebWarningLevel candidate_level,
    ::century::planning::AebWarningLevel target_downgrade_level) {
  using ::century::planning::AebWarningLevel;
  if (!aeb_config_.enable_downgrade_high_warning_level()) {
    return candidate_level;
  }
  const int threshold =
      aeb_config_.not_high_level_counter_threshold();  // e.g. 3
  auto& downgrade_infos = first_time_in_high_level_map_[track_id];
  auto& is_first_time_in_high_level = downgrade_infos.is_first_time_in_high_level;
  auto& not_high_level_counter = downgrade_infos.not_high_level_counter;
  auto& downgrade_counter = downgrade_infos.downgrade_counter;
  auto& has_downgrade = downgrade_infos.has_downgrade;
  has_downgrade = false;

  if (candidate_level != AebWarningLevel::WARNING_LEVEL_HIGH) {
    // if current level is not high level, reset downgrade counter to 0
    ++not_high_level_counter;
    AINFO << "not_high_level_counter: " << not_high_level_counter;
    if (not_high_level_counter >= threshold) {
      if (!is_first_time_in_high_level) {
        AERROR << "Reset first-high flag after " << not_high_level_counter
               << " non-HIGH frames.";
      }
      is_first_time_in_high_level = true;
      not_high_level_counter = 0;  // reset for next cycle
      downgrade_counter = 0;
      AINFO << "reset counter and first enter flag for next cycle.";
    }
    return candidate_level;
  }

  // if calculate result is high， need reset counter
  not_high_level_counter = 0;
  if (is_first_time_in_high_level) {
    downgrade_counter++;
    AERROR << "track_id[" << track_id << "], " << downgrade_counter
           << " HIGH frame detected, downgrade warning level.";
    if (downgrade_counter >= aeb_config_.downgrade_counter_threshold()) {
      is_first_time_in_high_level = false;
      AINFO << "reset downgrade_counter.";
      downgrade_counter = 0;
    }
    has_downgrade = true;
    return target_downgrade_level;
  }
  return candidate_level;
}

::century::planning::AebWarningLevel
AebPlanner::CheckHighLevelWithConsecutiveFrames(
    const perception::PerceptionObstacle& obstacle,
    const int& track_id,
    ::century::planning::AebWarningLevel current_level) {
  AINFO << __func__ << ", current_level: " << current_level;

  // get per-obstacle consecutive counter
  int& consecutive_high_frames = high_warning_frames_map_[track_id];
  // update consecutive count: high level or has downgrade flag
  if (current_level ==
          ::century::planning::AebWarningLevel::WARNING_LEVEL_HIGH ||
      first_time_in_high_level_map_[track_id].has_downgrade) {
    consecutive_high_frames++;
  } else {
    consecutive_high_frames = 0;
  }
  AINFO << "matched_id: " << track_id << ", obstacle_id: " << obstacle.id()
        << ", level: " << current_level
        << ", Consecutive high frames: " << consecutive_high_frames
        << ", threshold: "
        << aeb_config_.warning_level_high_consecutive_frames_thr();

  // if high level holds for 3 continuous， then send HIGH, otherwise send NONE
  if (consecutive_high_frames >=
      aeb_config_.warning_level_high_consecutive_frames_thr()) {
    return ::century::planning::AebWarningLevel::WARNING_LEVEL_HIGH;
  }
  return current_level;
}

::century::planning::AebWarningLevel
AebPlanner::CheckHighLevelWithHistoryFrames(
    ::century::planning::AebWarningLevel current_level) {
  AINFO << __func__ << ", current_level: " << current_level;
  // update history
  s_warning_level_history.push_back(current_level);
  AINFO << "s_warning_level_history.size: " << s_warning_level_history.size();
  for (const auto& level : s_warning_level_history) {
    AINFO << "level: " << level;
  }

  if (s_warning_level_history.size() >
      aeb_config_.warning_level_history_frame_count()) {
    s_warning_level_history.pop_front();
  }

  // if current warnin level is not HIGH, return current warning level
  if (current_level !=
      ::century::planning::AebWarningLevel::WARNING_LEVEL_HIGH) {
    return current_level;
  }

  // count the number of HIGH level frames in the history
  int high_count = 0;
  for (const auto& level : s_warning_level_history) {
    if (level == ::century::planning::AebWarningLevel::WARNING_LEVEL_HIGH) {
      high_count++;
      if (high_count >= aeb_config_.warning_level_high_threshold()) {
        break;
      }
    }
  }

  AINFO << "High level history check: " << high_count << ", HIGH frames in last "
        << s_warning_level_history.size() << " frames";

  // if 5 frames in history, 3 frames are HIGH, then send HIGH, otherwise send
  // NONE
  if (high_count >= aeb_config_.warning_level_high_threshold()) {
    AINFO << "finally send high warning level, high_count: " << high_count
          << ", threshold: " << aeb_config_.warning_level_high_threshold();
    return ::century::planning::AebWarningLevel::WARNING_LEVEL_HIGH;
  } else {
    return ::century::planning::AebWarningLevel::WARNING_LEVEL_NONE;
  }
  return current_level;
}

void AebPlanner::SetWarningLevelWithMinFrames(
    const perception::PerceptionObstacle& obstacle,
    const ::century::planning::AebWarningLevel candidate_level,
    ::century::planning::AebResult* aeb_result) {
  // match aeb obstacle to track
  int track_id = obstacle_tracker_->MatchTrack(obstacle);
  if (track_id < 0) {
    AERROR << "No valid track for obstacle.";
    track_id = kFallbackTrackId;
  }
  ::century::planning::AebWarningLevel filtered_level = DowngradeFirstHighFrame(
      track_id, candidate_level,
      ::century::planning::AebWarningLevel::WARNING_LEVEL_MEDIUM);
  AINFO << __func__ << ", candidate_level: " << candidate_level
        << ", filtered_level: " << filtered_level;
  // Apply frame-based filtering if enabled
  if (aeb_config_.enable_check_with_history_frames()) {
    filtered_level = CheckHighLevelWithHistoryFrames(filtered_level);
  } else if (aeb_config_.enable_check_with_consecutive_frames()) {
    filtered_level =
        CheckHighLevelWithConsecutiveFrames(obstacle, track_id, filtered_level);
  }
  AINFO << "history_or_consecutive_frames_filtered_level: " << filtered_level;

  if (!aeb_config_.enable_set_with_min_frames()) {
    aeb_result->set_warning_level(filtered_level);
    return;
  }
  if (!is_auto_mode_ && !aeb_config_.enable_check_with_consecutive_frames() &&
      !aeb_config_.enable_check_with_history_frames()) {
    // if not in auto mode && enable muti-frames check, set warning level
    // normally.
    aeb_result->set_warning_level(filtered_level);
    return;
  }
  if (filtered_level ==
      ::century::planning::AebWarningLevel::WARNING_LEVEL_HIGH) {
    // warning high level min times: min_high_warning_frames
    s_high_warning_frames_left = std::max(
        s_high_warning_frames_left, aeb_config_.min_high_warning_frames());
    aeb_result->set_warning_level(
        ::century::planning::AebWarningLevel::WARNING_LEVEL_HIGH);
  } else {
    if (s_high_warning_frames_left > 0) {
      // still send HIGH until the counter is exhausted
      aeb_result->set_warning_level(
          ::century::planning::AebWarningLevel::WARNING_LEVEL_HIGH);
      --s_high_warning_frames_left;
      AINFO << "Still send HIGH WARNING LEVEL until the counter is exhausted, "
               "s_high_warning_frames_left: "
            << s_high_warning_frames_left;
    } else {
      aeb_result->set_warning_level(filtered_level);
    }
  }
  return;
}

bool AebPlanner::IsTurnScene(const double& average_wheel_angle) {
  turn_detector_.Update(average_wheel_angle);
  AINFO << "IsTurnScene: " << turn_detector_.IsTurning();
  if (turn_detector_.IsTurning()) {
    return true;
  }
  return false;
}

bool AebPlanner::IsFarAwayToTheSide(
    const perception::PerceptionObstacle& obstacle,
    const std::pair<double, double>& dis_info) {
  const auto& min_distance = vehicle_param_.width() / 2.0 +
                             aeb_config_.lateral_far_away_threshold();
  // second : x
  // first: y
  if (std::fabs(dis_info.first) <= aeb_config_.lateral_far_away_threshold()) {
    AERROR << "id: [" << obstacle.id()
           << "], calculate nearest point lateral distance[" << dis_info.first
           << "] is too close, "
              "threshold:["
           << aeb_config_.lateral_far_away_threshold() << "], no need ignore.";
    return false;
  }

  const auto& points = obstacle.polygon_point();
  bool all_points_far = std::all_of(
      points.begin(), points.end(),
      [&](const auto& point) { return std::fabs(point.y()) > min_distance; });
  if (all_points_far) {
    AERROR << "obstacle[" << obstacle.id()
           << "], is far away from ego, need ignore."
           << obstacle.ShortDebugString();
    return true;
  }
  return false;
}

bool AebPlanner::IgnoreLongitTTC(const perception::PerceptionObstacle& obstacle,
                                 const std::pair<double, double>& dis_info) {
  if (IsFarAwayToTheSide(obstacle, dis_info) && !is_turning_) {
    AERROR << "far away to the side, ignore.";
    return true;
  }
  return false;
}

bool AebPlanner::IsAllPointsBehindEgoByMotionDirection(
    const perception::PerceptionObstacle& obstacle) {
  if (!aeb_injector_ || !aeb_injector_->vehicle_state() ||
      obstacle.polygon_point().empty()) {
    return false;
  }

  const auto& vehicle_state = aeb_injector_->vehicle_state()->vehicle_state();
  const auto gear = vehicle_state.gear();
  double motion_direction_x = 0.0;
  if (gear == canbus::Chassis::GEAR_DRIVE) {
    motion_direction_x = 1.0;
  } else if (gear == canbus::Chassis::GEAR_REVERSE) {
    motion_direction_x = -1.0;
  } else {
    return false;
  }

  const double x_threshold = -0.5 * vehicle_param_.length();
  const bool all_points_behind = std::all_of(
      obstacle.polygon_point().begin(), obstacle.polygon_point().end(),
      [&](const auto& point) {
        return motion_direction_x * point.x() <= x_threshold;
      });
  return all_points_behind;
}

double AebPlanner::CalcMinDistanceFromPointToEgo(
    const common::math::Vec2d& point,
    const std::vector<common::math::Vec2d>& ego_points) const {
  if (ego_points.empty()) {
    return std::numeric_limits<double>::max();
  }
  if (ego_points.size() < 2) {
    return ego_points.front().DistanceTo(point);
  }
  if (ego_points.size() < 3) {
    common::math::LineSegment2d ego_segment(ego_points.front(),
                                            ego_points.back());
    return ego_segment.DistanceTo(point);
  }
  common::math::Polygon2d ego_polygon(ego_points);
  return ego_polygon.DistanceTo(point);
}

bool AebPlanner::AebNearestPointObstacleFilter(
    const LocalView& local_view, const perception::PerceptionObstacle& obstacle,
    const common::math::Vec2d nearest_point,
    const std::vector<common::math::Vec2d>& ego_points) {
  if ((common::util::IsZero(nearest_point.x(), common::util::kMathEpsilon) &&
       common::util::IsZero(nearest_point.y(), common::util::kMathEpsilon)) ||
      ego_points.empty()) {
    AERROR << "calculate nearest point failed or ego points is empty, return "
              "false.";
    return false;
  }
  // calculate min distance
  const auto& speed =
      aeb_injector_->vehicle_state()->vehicle_state().linear_velocity();

  double yaw_rate = 0.0;
  double v_lon = 0.0;
  double v_lat = 0.0;
  CalcSpeed(speed, average_wheel_angle_ * common::util::ANG2RAD, v_lon, v_lat);
  if (aeb_config_.enable_cal_accurate_speed()) {
    CalcAccurateSpeed(local_view, v_lon, v_lat, yaw_rate);
  }
  const double ttc_speed = std::max(v_lon, v_lat);
  const double min_distance =
      CalcMinDistanceFromPointToEgo(nearest_point, ego_points);
  AINFO << "nearest point distance to ego polygon: " << min_distance;
  const double min_ttc_threshold =
      aeb_config_.min_ttc_buffer_for_far_away_point() +
      FLAGS_long_ttc_low_threshold;
  if (overlap_obstacle_map_[obstacle.id()] &&
      std::fabs(min_distance) > std::fabs(ttc_speed) * min_ttc_threshold) {
    AERROR << "id: [" << obstacle.id()
           << ", overlap condition, need filter, min_distance(" << min_distance
           << ") > v * t, v: " << ttc_speed << ", t: " << min_ttc_threshold
           << ", v * t: " << ttc_speed * min_ttc_threshold;
    return false;
  }
  AINFO << "average_wheel_angle: " << average_wheel_angle_
        << ", lat_speed: " << v_lat << ", lon_speed: " << v_lon;
  double min_turn_ttc_threshold = aeb_config_.min_ttc_for_turn_scene();
  min_turn_ttc_threshold =
      std::clamp(min_turn_ttc_threshold, FLAGS_lateral_ttc_high_threshold,
                 FLAGS_long_ttc_low_threshold);
  if (IsTurnScene(average_wheel_angle_ * common::util::ANG2RAD) &&
      std::fabs(min_distance) > std::fabs(ttc_speed) * min_turn_ttc_threshold) {
    AINFO << "turn scene, need filter far away obstacle, min_distance("
          << min_distance << ") > v * t, v: " << ttc_speed
          << ", t: " << min_turn_ttc_threshold
          << ", v * t: " << ttc_speed * min_turn_ttc_threshold;
    return false;
  }

  if (IsAllPointsBehindEgoByMotionDirection(obstacle) &&
      min_distance > aeb_config_.min_distance_threshold_for_nearest_point()) {
    AERROR << "obstacle all polygon points are behind ego in motion direction,"
              " and far away from ego point, need filter.";
    return false;
  }
  return true;
}

bool AebPlanner::FindTargetObstacleTypeInRange(
    const perception::PerceptionObstacles& perceptions,
    const perception::PerceptionObstacle::Type target_type) {
  bool has_stacker_in_range = false;
  common::math::Vec2d adc_point(0.0, 0.0);
  const double adc_heading = 0.0;
  common::math::Box2d adc_box = util::ConstructionEgoBox(adc_point, adc_heading);
  double threshold = aeb_config_.aeb_stacker_in_range_distance_threshold();

  for (const auto& obstacle : perceptions.perception_obstacle()) {
    if (obstacle.type() != target_type) {
      continue;
    }
    std::vector<common::math::Vec2d> obs_points;
    obs_points.reserve(obstacle.polygon_point_size());
    for (const auto& point : obstacle.polygon_point()) {
      obs_points.emplace_back(point.x(), point.y());
    }
    if (obs_points.size() < 3) {
      continue;
    }
    common::math::Polygon2d ob_polygon(obs_points);
    double dist = ob_polygon.DistanceTo(adc_box);
    AINFO << "dist_to stacker: " << dist;
    if (dist < threshold) {
      AINFO << "Found target obstacle in range! ID: " << obstacle.id()
            << ", Type: "
            << perception::PerceptionObstacle::Type_Name(target_type)
            << ", Distance: " << dist;
      has_stacker_in_range = true;
      break;
    }
  }
  AINFO << "has_stacker_in_range: " << has_stacker_in_range;
  return has_stacker_in_range;
}

bool AebPlanner::AebObstacleHeightFilterForStacker(
    const perception::PerceptionObstacle& obstacle,
    const bool& has_stacker_in_range) {
  const double adc_height =
      vehicle_param_.height() + aeb_config_.vehicle_height_buffer();
  if (!has_stacker_in_range) {
    // only has stacker in range, enable to filter obstacle which height is
    // upper than threshold
    AINFO << "No stacker in range, do not need filter.";
    return false;
  }
  // height filter
  for (const auto& polygon : obstacle.polygon_point()) {
    if (polygon.z() < adc_height) {
      continue;
    }
    AINFO << "has stacker and obstacle all polygon points height > adc height("
          << adc_height << "),  need filter.";
    return true;
  }
  return false;
}

bool AebPlanner::AebObstacleHeightFilterInJunction(
    const perception::PerceptionObstacle& obstacle) {
  common::PointENU adc_point_enu;
  adc_point_enu.set_x(aeb_injector_->vehicle_state()->vehicle_state().x());
  adc_point_enu.set_y(aeb_injector_->vehicle_state()->vehicle_state().y());
  if (util::IsGateJunctionContainAdc(
          adc_point_enu, aeb_injector_->vehicle_state()->vehicle_state())) {
    for (const auto& corner : obstacle.polygon_point()) {
      if (corner.z() >
          aeb_config_.aeb_obstacle_height_threshold_in_junction()) {
        special_junction_info_.set_special_junction(
            SpecialJunction::DONGJIAZHEN_GATE);
        special_junction_info_.set_obstacle_id(obstacle.id());
        special_junction_info_.mutable_filter_point()->CopyFrom(corner);
        special_junction_info_.mutable_position()->CopyFrom(
            obstacle.position());
        return true;
      }
    }
  }
  return false;
}

bool AebPlanner::AebObstacleDistanceFilter(
    const perception::PerceptionObstacle& obstacle,
    const double& aeb_obstacle_distance_threshold, double& min_distance) {
  double length = vehicle_param_.length() + FLAGS_aeb_obstacle_length_buffer;
  double width = vehicle_param_.width() + FLAGS_aeb_obstacle_width_buffer;
  double min_distance_from_ego_to_obstacle = std::numeric_limits<double>::max();
  common::math::Box2d ego_box({0.0, 0.0}, 0.0, length, width);
  for (auto& corner : obstacle.polygon_point()) {
    common::math::Vec2d pt(corner.x(), corner.y());
    min_distance_from_ego_to_obstacle =
        std::min(ego_box.DistanceTo(pt), min_distance_from_ego_to_obstacle);
  }
  if (min_distance_from_ego_to_obstacle > aeb_obstacle_distance_threshold) {
    return false;
  }
  min_distance = min_distance_from_ego_to_obstacle;
  return true;
}

bool AebPlanner::HasOverlapWithBuffer(
    const perception::PerceptionObstacle& obstacle, const double& length_buffer,
    double& width_buffer, const bool is_reverse) {
  if (!is_auto_mode_) {
    width_buffer = aeb_config_.manual_aeb_obstacle_overlap_width_buffer();
  }
  double length = vehicle_param_.length() + length_buffer;
  double width = vehicle_param_.width() + width_buffer;
  common::math::Box2d ego_box({0.0, 0.0}, (is_reverse ? M_PI : 0.0), length,
                              width);
  common::math::Polygon2d ego_polygon(ego_box);
  std::vector<common::math::Vec2d> points;
  points.clear();
  for (const auto& corner : obstacle.polygon_point()) {
    common::math::Vec2d temp_pt(corner.x(), corner.y());
    points.emplace_back(temp_pt);
  }
  // ADEBUG << "capacity: " << points.capacity() << ", size: " << points.size();
  points.shrink_to_fit();
  // ADEBUG << "after shrink, capacity: " << points.capacity()
  //        << ", size: " << points.size();
  common::math::Polygon2d polygon_ob(points);
  if (polygon_ob.HasOverlap(ego_polygon)) {
    ADEBUG << "has_overlap_obstacle: " << obstacle.DebugString();
    return true;
  }
  return false;
}

bool AebPlanner::AebObstaclePositionFilter(
    const perception::PerceptionObstacle& obstacle) {
  ADEBUG << __func__;
  // whaterver auto mode or not, need to filter the obstacle behind adc, 0819
  // if (!is_auto_mode_) {
  //   return true;
  // }
  bool is_behind = false;
  bool d_gear_points_behind = true;
  bool r_gear_points_behind = true;
  const auto gear = aeb_injector_->vehicle_state()->vehicle_state().gear();
  ADEBUG << "gear: " << gear;
  for (const auto& point : obstacle.polygon_point()) {
    if (gear == canbus::Chassis::GEAR_DRIVE) {
      if (point.x() > (- 0.5 * vehicle_param_.length())) {
        d_gear_points_behind = false;
        break;
      }
    } else if (gear == canbus::Chassis::GEAR_REVERSE) {
      if (point.x() < 0.5 * vehicle_param_.length()) {
        r_gear_points_behind = false;
        break;
      }
    }
  }
  ADEBUG << "d_gear_points_behind: " << d_gear_points_behind
         << ", r_gear_points_behind: " << r_gear_points_behind
         << ", obstacle_info: " << obstacle.DebugString();
  if (gear == canbus::Chassis::GEAR_DRIVE && d_gear_points_behind /*&&
      !HasOverlapWithBuffer(obstacle, FLAGS_behind_ego_length_buffer,
                            FLAGS_behind_ego_width_buffer)*/) {
    is_behind = true;
    ADEBUG << "gear D, behind adc";
  } else if (gear == canbus::Chassis::GEAR_REVERSE && r_gear_points_behind /*&&
             !HasOverlapWithBuffer(obstacle, FLAGS_behind_ego_length_buffer,
                                   FLAGS_behind_ego_width_buffer, true)*/) {
    is_behind = true;
    ADEBUG << "gear R, behind adc";
  }
  if (is_behind || gear == canbus::Chassis::GEAR_PARKING ||
      gear == canbus::Chassis::GEAR_NEUTRAL) {
    return false;
  }
  return true;
}

bool AebPlanner::AebObstacleSpeedFilter(
    const perception::PerceptionObstacle& obstacle) {
  if (!FLAGS_enable_aeb_speed_filter) {
    return true;
  }
  if (!aeb_injector_->vehicle_state()) {
    const std::string msg = "Vehicle state not available, return false";
    error_messages_ += "\n";
    error_messages_ += msg;
    AERROR << msg;
    return false;
  }

  const auto& vehicle_state = aeb_injector_->vehicle_state()->vehicle_state();
  double ego_speed = vehicle_state.linear_velocity();
  const auto gear = vehicle_state.gear();

  // get obstacle velocity(flu)
  double obstacle_vx = obstacle.velocity().x();
  // double obstacle_vy = obstacle.velocity().y();

  // calculate relative speed
  double relative_speed = 0.0;
  if (gear == canbus::Chassis::GEAR_DRIVE) {
    relative_speed = ego_speed - obstacle_vx;
  } else if (gear == canbus::Chassis::GEAR_REVERSE) {
    relative_speed = -ego_speed - obstacle_vx;
  } else {
    return true;  // invaild gear, do not ignore
  }

  // update speed history
  auto& history = speed_history_map_[obstacle.id()];
  history.relative_speeds.push_back(relative_speed);
  if (history.relative_speeds.size() > kSpeedHistorySize) {
    history.relative_speeds.pop_front();
  }
  history.history_size = history.relative_speeds.size();

  // calculate average relative speed
  double sum_speed = 0.0;
  for (double speed : history.relative_speeds) {
    sum_speed += speed;
  }
  double avg_relative_speed = history.history_size > 0
                                  ? sum_speed / history.history_size
                                  : relative_speed;

  // stability check: make sure the speed variation is within a reasonable range
  bool is_stable = true;
  if (history.history_size > 1) {
    double max_variation = *std::max_element(history.relative_speeds.begin(),
                                             history.relative_speeds.end());
    double min_variation = *std::min_element(history.relative_speeds.begin(),
                                             history.relative_speeds.end());
    double variation_range = max_variation - min_variation;
    // when speed is low, use a fixed threshold
    if (std::abs(ego_speed) < 0.5) {
      if (variation_range > 1.0) {  // fluctuation threshold of 1 m/s
        is_stable = false;
      }
    } else {
      if (variation_range > (kSpeedFilterThreshold * std::abs(ego_speed))) {
        is_stable = false;
      }
    }
  }

  // filter condition: positive relative speed means obstacle is slower than adc
  return (avg_relative_speed < 0) && is_stable;
}

AebPlanner::AebObstacleInfo AebPlanner::CreateAebObstacles(
    const perception::PerceptionObstacles& perceptions) {
  AebObstacleInfo filtered_obstacles_info;
  double min_dis = std::numeric_limits<double>::max();
  std::pair<double, perception::PerceptionObstacle> aeb_ob;

  // clear out the history of obstacles that no longer appear
  std::unordered_set<int> current_ids;
  for (const auto& ob : perceptions.perception_obstacle()) {
    current_ids.insert(ob.id());
  }

  for (auto it = speed_history_map_.begin(); it != speed_history_map_.end();) {
    if (current_ids.find(it->first) == current_ids.end()) {
      it = speed_history_map_.erase(it);
    } else {
      ++it;
    }
  }

  const perception::PerceptionObstacle::Type target_type =
      perception::PerceptionObstacle::STACKER;
  bool has_stacker_in_range =
      FindTargetObstacleTypeInRange(perceptions, target_type);

  for (const auto& perception_obstacle : perceptions.perception_obstacle()) {
    if (!Obstacle::IsValidPerceptionObstacle(perception_obstacle)) {
      ADEBUG << "Invalid perception obstacle: "
             << perception_obstacle.DebugString();
      continue;
    }
    if (AebObstacleHeightFilterForStacker(perception_obstacle,
                                          has_stacker_in_range)) {
      ADEBUG << "stacker around obstacle height exceeds threshold, invalid "
                "perception obstacle: "
             << perception_obstacle.DebugString() << ", continue";
      continue;
    }
    if (AebObstacleHeightFilterInJunction(perception_obstacle)) {
      ADEBUG << "height exceeds threshold, invalid perception obstacle: "
             << perception_obstacle.DebugString() << ", continue";
      continue;
    }
    if (!AebObstacleDistanceFilter(perception_obstacle,
                                   FLAGS_aeb_obstacle_distance_threshold,
                                   min_dis)) {
      ADEBUG << "min_distance: " << min_dis;
      ADEBUG << "Distance invalid perception obstacle: "
             << perception_obstacle.DebugString() << ", continue";
      continue;
    }
    // behind ego obstacles filter, select front obstacles to calculate ttc
    if (!AebObstaclePositionFilter(perception_obstacle)) {
      ADEBUG << "Direction invalid perception obstacle: "
             << perception_obstacle.DebugString() << ", continue";
      continue;
    }
    bool is_reverse = aeb_injector_->vehicle_state()->vehicle_state().gear() ==
                      canbus::Chassis::GEAR_REVERSE;
    if (!HasOverlapWithBuffer(
            perception_obstacle, FLAGS_aeb_obstacle_overlap_length_buffer,
            FLAGS_aeb_obstacle_overlap_width_buffer, is_reverse)) {
      ADEBUG << "no overlap perception obstacle: "
             << perception_obstacle.DebugString() << ", continue";
      continue;
    }
    if (!AebObstacleSpeedFilter(perception_obstacle)) {
      ADEBUG << "higher speed than adc, invalid perception obstacle: "
             << perception_obstacle.DebugString() << ", continue";
      continue;
    }
    min_dis_obstacle_info_.first = min_dis;
    min_dis_obstacle_info_.second = std::to_string(perception_obstacle.id());
    min_dis_obstacle_position_ = {
        static_cast<float>(perception_obstacle.position().x()),
        static_cast<float>(perception_obstacle.position().y()),
        static_cast<float>(perception_obstacle.position().z())};
    aeb_ob = std::make_pair(min_dis, perception_obstacle);
    filtered_obstacles_info.emplace_back(aeb_ob);
  }
  ADEBUG << __func__ << ", filtered aeb obstacles from perception size is : "
         << filtered_obstacles_info.size();
  return filtered_obstacles_info;
}

// for electric fence
void AebPlanner::CalculateLateralSpeed(const LocalView& local_view,
                                       const double& ego_speed, double& v_lon,
                                       double& v_lat,
                                       const bool enable_use_yaw_rate) {
  double yaw_rate = 0.0;
  auto average_wheel_angle =
      0.5 * (local_view.chassis->bridge_1_left_wheel_angle() +
             local_view.chassis->bridge_1_right_wheel_angle());
  if (local_view.chassis.get()->gear_location() ==
      canbus::Chassis::GEAR_REVERSE) {
    average_wheel_angle =
        0.5 * (local_view.chassis->bridge_4_left_wheel_angle() +
               local_view.chassis->bridge_4_right_wheel_angle());
  }
  CalcSpeed(local_view.chassis->speed_mps(),
            average_wheel_angle * common::util::ANG2RAD, v_lon, v_lat);
  if (enable_use_yaw_rate) {
    CalcAccurateSpeed(local_view, v_lon, v_lat, yaw_rate);
  }
  return;
}

void AebPlanner::CalcSpeed(const double& speed, const double& wheel_angle,
                           double& v_lon, double& v_lat) {
  v_lon = speed * std::cos(wheel_angle);
  v_lat = speed * std::sin(wheel_angle);
  AINFO << "CalcSpeed, v_lat: " << v_lat << ", v_lon: " << v_lon
        << ",wheel_angle:" << wheel_angle * common::util::RAD2ANG;
  return;
}

void AebPlanner::CalcAccurateSpeed(const LocalView& local_view, double& v_lon,
                                   double& v_lat, double& yaw_rate) {
  if (!local_view.chassis) {
    v_lon = 0.0;
    v_lat = 0.0;
    yaw_rate = 0.0;
    return;
  }

  double fl = local_view.chassis->bridge_1_left_wheel_angle();
  double fr = local_view.chassis->bridge_1_right_wheel_angle();
  double rl = local_view.chassis->bridge_4_left_wheel_angle();
  double rr = local_view.chassis->bridge_4_right_wheel_angle();
  const double v = local_view.chassis->speed_mps();
  const double L = vehicle_param_.wheel_base();
  double delta_f = 0.5 * (fl + fr) * common::util::ANG2RAD;
  double delta_r = 0.5 * (rl + rr) * common::util::ANG2RAD;

  // yaw_rate， + turn left, - turn right
  if (std::abs(delta_f - delta_r) < 1e-6) {
    yaw_rate = 0.0;
  } else {
    yaw_rate = v * (std::tan(delta_f) - std::tan(delta_r)) / L;
  }
  const double half_length = yaw_rate < common::util::kMathEpsilon
                                 ? vehicle_param_.length() / 2.0
                                 : -vehicle_param_.length() / 2.0;

  AINFO << "[4WS Accurate Speed] current_v_lat: " << v_lat
        << ", delta_v_lat=" << yaw_rate * half_length;

  v_lat = std::fabs(v_lat + yaw_rate * half_length);

  AINFO << "[4WS Accurate Speed] v_lat=" << v_lat << ", v_lon=" << v_lon
        << ", yaw_rate=" << yaw_rate;
  return;
}

// overload for aeb
void AebPlanner::CalcAccurateSpeed(const LocalView& local_view, double& v_lon,
                                   double& v_lat, double& yaw_rate,
                                   const double& yaw_rate_coff) {
  if (!local_view.chassis) {
    v_lon = 0.0;
    v_lat = 0.0;
    yaw_rate = 0.0;
    return;
  }

  const auto& chassis = *local_view.chassis;
  const double fl = chassis.bridge_1_left_wheel_angle();
  const double fr = chassis.bridge_1_right_wheel_angle();
  const double rl = chassis.bridge_4_left_wheel_angle();
  const double rr = chassis.bridge_4_right_wheel_angle();
  const double v = chassis.speed_mps();
  const double L = vehicle_param_.wheel_base();
  const double delta_f = 0.5 * (fl + fr) * common::util::ANG2RAD;
  const double delta_r = 0.5 * (rl + rr) * common::util::ANG2RAD;
  const double delta_diff = delta_f - delta_r;

  // yaw_rate， + turn left, - turn right
  if (std::abs(delta_diff) < 1e-6) {
    yaw_rate = 0.0;
  } else {
    yaw_rate = v * (std::tan(delta_f) - std::tan(delta_r)) / L;
  }
  double sign_gear = 1.0;
  if (canbus::Chassis::GEAR_REVERSE == chassis.gear_location()) {
    sign_gear = -1.0;
  }
  const auto& vehicle_state = aeb_injector_->vehicle_state()->vehicle_state();
  AINFO << "[4WS Accurate Speed] speed_mps: " << v << ", linear_velocity: "
        << vehicle_state.linear_velocity()
        << ", raw yaw_rate: " << yaw_rate << ", sign_gear: " << sign_gear;
  const double half_length = vehicle_param_.length() / 2.0;
  yaw_rate = common::util::InverseInterpolationLookUp(
      average_wheel_angle_, aeb_config_.min_turn_wheel_angle(),
      aeb_config_.max_turn_wheel_angle(), yaw_rate * yaw_rate_coff, yaw_rate);
  const double delta_v_lat = yaw_rate * sign_gear * half_length;

  AINFO << "[4WS Accurate Speed] current_v_lat: " << v_lat
        << ", delta_v_lat=" << delta_v_lat
        << ", yaw_rate: " << yaw_rate
        << ", ave_wheel_angle: " << average_wheel_angle_;

  v_lat = std::fabs(v_lat + delta_v_lat);
  return;
}

void AebPlanner::UpdateOverlapDistanceInfo(
    const perception::PerceptionObstacle& obstacle,
    const std::pair<double, double> dis_info,
    std::pair<double, double>& updated_distance_info) {
  // NOTES: only for adc static, disable aeb function condition
  // need consider vehicle half length and half width
  updated_distance_info.second =
      dis_info.second - 0.5 * vehicle_param_.length();
  updated_distance_info.first = dis_info.first - 0.5 * vehicle_param_.width();
  double overlap_length_buffer = aeb_config_.overlap_length_buffer();
  double overlap_width_buffer = aeb_config_.overlap_width_buffer();
  double vehicle_min_width_buffer = 0.175;

  common::PointENU adc_point_enu;
  adc_point_enu.set_x(aeb_injector_->vehicle_state()->vehicle_state().x());
  adc_point_enu.set_y(aeb_injector_->vehicle_state()->vehicle_state().y());
  if (util::IsGateJunctionContainAdc(
          adc_point_enu, aeb_injector_->vehicle_state()->vehicle_state())) {
    AINFO << __func__ << ", in dongjiazhen gate, reduce aeb buffer and vehicle width.";
    overlap_length_buffer = aeb_config_.overlap_min_length_buffer();
    overlap_width_buffer = aeb_config_.overlap_min_width_buffer();
    updated_distance_info.first += vehicle_min_width_buffer;
  }
  AINFO << "-= vehicle param, dis_info, y: " << updated_distance_info.first
        << ", x: " << updated_distance_info.second;
  // second : x
  // first: y
  if (IgnoreLongitTTC(obstacle, updated_distance_info)) {
    AERROR << "disable longitudinal ttc because of ignore";
    updated_distance_info.second = std::numeric_limits<double>::max();
    return;
  }

  if (updated_distance_info.second < overlap_length_buffer &&
      updated_distance_info.first < overlap_width_buffer) {
    // lat and long overlap, continue calculate ttc, only adc static used!!!!
    AINFO << "Calculate ttc normally, it must be high warning level.";
    calc_dis_is_both_overlap_ = true;
  } else if (updated_distance_info.second < overlap_length_buffer) {
    // long overlap, disable longitudinal ttc
    AERROR << "id:[" << obstacle.id() << "]" << "disable longitudinal ttc";
    calc_dis_is_overlap_ = true;
    overlap_obstacle_map_[obstacle.id()] = calc_dis_is_overlap_;
    updated_distance_info.second = std::numeric_limits<double>::max();
  } else if (updated_distance_info.first < overlap_width_buffer) {
    // lat overlap, disable lateral ttc
    AERROR << "id:[" << obstacle.id() << "]" << "disable lateral ttc";
    calc_dis_is_overlap_ = true;
    overlap_obstacle_map_[obstacle.id()] = calc_dis_is_overlap_;
    updated_distance_info.first = std::numeric_limits<double>::max();
  } else {
    AINFO << "No overlap, dis info do not need update.";
    calc_dis_is_overlap_ = false;
    overlap_obstacle_map_[obstacle.id()] = calc_dis_is_overlap_;
    updated_distance_info = std::make_pair(dis_info.first, dis_info.second);
  }
  return;
}

bool AebPlanner::CalcCollisionDistance(
    const LocalView& local_view, const perception::PerceptionObstacle& obstacle,
    std::pair<double, double>& distance_info) {
  if (obstacle.polygon_point().empty()) {
    const std::string msg = "obstacle polygon pt empty, return.";
    error_messages_ += "\n";
    error_messages_ += msg;
    AERROR << msg;
    return false;
  }
  double length = vehicle_param_.length();
  double width = vehicle_param_.width();
  ADEBUG << "adc_length: " << length << ", width: " << width;
  // adc box and all points
  common::math::Box2d ego_box({0.0, 0.0}, 0.0, length, width);
  auto all_points = ego_box.GetAllCorners();
  if (all_points.size() < 2 || obstacle.polygon_point_size() < 2) {
    const std::string msg =
        "insufficient polygon points when calculate collision distance.";
    error_messages_ += "\n";
    error_messages_ += msg;
    AERROR << msg;
    return false;
  }

  std::vector<common::math::LineSegment2d> ego_linesegments;
  std::vector<common::math::LineSegment2d> obstacle_linesegments;
  std::vector<common::math::Vec2d> obstacle_points;
  ego_linesegments.clear();
  obstacle_linesegments.clear();
  obstacle_points.clear();
  double min_dis = std::numeric_limits<double>::max();
  common::math::Vec2d nearest_pt(0, 0);
  common::math::Vec2d temp_nearest_pt(0, 0);
  for (const auto& obstacle_vertice : obstacle.polygon_point()) {
    common::math::Vec2d vertice(obstacle_vertice.x(), obstacle_vertice.y());
    obstacle_points.emplace_back(vertice);
  }
  ConvertPointsToLineSegments(all_points, ego_linesegments);
  ego_linesegments.emplace_back(
      common::math::LineSegment2d(all_points.back(), all_points.front()));
  ConvertPointsToLineSegments(obstacle_points, obstacle_linesegments);
  obstacle_linesegments.emplace_back(common::math::LineSegment2d(
      obstacle_points.back(), obstacle_points.front()));

  // direction-1: ego corners to obstacle edges.
  for (const auto& adc_vertice : all_points) {
    for (const auto& obstacle_linesegment : obstacle_linesegments) {
      double distance =
          obstacle_linesegment.DistanceTo(adc_vertice, &temp_nearest_pt);
      if (distance < min_dis) {
        nearest_pt = temp_nearest_pt;
        min_dis = distance;
      }
    }
  }

  // direction-2: obstacle corners to ego edges.
  // nearest point on obstacle is the obstacle corner itself.
  for (const auto& obstacle_vertice : obstacle_points) {
    for (const auto& ego_linesegment : ego_linesegments) {
      double distance =
          ego_linesegment.DistanceTo(obstacle_vertice, &temp_nearest_pt);
      if (distance < min_dis) {
        nearest_pt = obstacle_vertice;
        min_dis = distance;
      }
    }
  }
  AINFO << "min_dis: " << min_dis
         << ", nearest_pt.x: " << nearest_pt.x()
         << ", nearest_pt.y: " << nearest_pt.y();
  std::pair<double, double> temp_distance_info =
      std::make_pair(std::fabs(nearest_pt.y()), std::fabs(nearest_pt.x()));
  std::get<0>(original_distance_info_) = temp_distance_info.first;  // y
  std::get<1>(original_distance_info_) = temp_distance_info.second;  // x
  std::get<2>(original_distance_info_) =
      const_cast<perception::PerceptionObstacle*>(&obstacle);
  AINFO << "original distance_info, y: " << temp_distance_info.first
        << ", x: " << temp_distance_info.second;
  UpdateOverlapDistanceInfo(obstacle, temp_distance_info, distance_info);
  // nearest collision point distance filter
  if (!AebNearestPointObstacleFilter(local_view, obstacle, nearest_pt,
                                     all_points)) {
    AINFO << "nearest point of obstacle is behind, and distance > "
             "threshold("
          << aeb_config_.min_distance_threshold_for_nearest_point()
          << "), need filter, return false.";
    return false;
  }
  return true;
}

bool AebPlanner::CalcTTCFromEgoToAebObstacle(
    const LocalView& local_view, const perception::PerceptionObstacle& obstacle,
    const double& ego_speed, AebResult* aeb_result) {
  if (!local_view.chassis) {
    const std::string msg = "local view chassis is nullptr. return false";
    error_messages_ += "\n";
    error_messages_ += msg;
    AERROR << msg;
    return false;
  }
  ADEBUG << __func__;
  ADEBUG << "already selected perception obstacle info: "
         << obstacle.DebugString();

  // lateral, longitudinal
  std::pair<double, double> distance_info = std::make_pair(
      std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  std::pair<double, double> speed_info;
  std::pair<double, double> ttc_info;
  double min_ttc = std::numeric_limits<double>::max();
  const auto gear = aeb_injector_->vehicle_state()->vehicle_state().gear();
  ADEBUG << "bridge_1_left_wheel_angle: "
         << local_view.chassis->bridge_1_left_wheel_angle()
         << ", bridge_1_right_wheel_angle: "
         << local_view.chassis->bridge_1_right_wheel_angle();
  ADEBUG << "bridge_4_left_wheel_angle: "
         << local_view.chassis->bridge_4_left_wheel_angle()
         << ",  bridge_4_right_wheel_angle: "
         << local_view.chassis->bridge_4_right_wheel_angle();

  auto average_wheel_angle =
      0.5 * (local_view.chassis->bridge_1_left_wheel_angle() +
             local_view.chassis->bridge_1_right_wheel_angle());
  if (gear == canbus::Chassis::GEAR_REVERSE) {
    average_wheel_angle =
        0.5 * (local_view.chassis->bridge_4_left_wheel_angle() +
               local_view.chassis->bridge_4_right_wheel_angle());
  }
  average_wheel_angle_ = average_wheel_angle;
  is_turning_ = IsTurnScene(average_wheel_angle_ * common::util::ANG2RAD);
  AINFO << "average_wheel_angle(rad): "
        << average_wheel_angle * common::util::ANG2RAD
        << ", angle: " << average_wheel_angle
        << ", is_turning: " << is_turning_;
  if (!CalcCollisionDistance(local_view, obstacle, distance_info)) {
    const std::string msg = "Calculate collision distance failed.";
    error_messages_ += "\n";
    error_messages_ += msg;
    AERROR << msg;
    aeb_result->set_warning_level(
        ::century::planning::AebWarningLevel::WARNING_LEVEL_NONE);
    return false;
  }
  AINFO << "after consider overlap, distance_info, y: " << distance_info.first
        << ", x: " << distance_info.second;
  // turn left -> +, turn right -> -, unit: degree
  AINFO << "ego_speed,m/s: " << ego_speed << ", km/h: " << ego_speed * 3.6;
  CalcSpeed(ego_speed, average_wheel_angle * common::util::ANG2RAD,
            speed_info.first, speed_info.second);
  if (aeb_config_.enable_cal_accurate_speed()) {
    double yaw_rate = 0.0;
    CalcAccurateSpeed(local_view, speed_info.first, speed_info.second,
                      yaw_rate, aeb_config_.yaw_rate_coff());
  }
  v_lateral_ = speed_info.second;

  AINFO << "speed_info, v_lat: " << speed_info.second
        << ", v_lon: " << speed_info.first;

  // lateral
  ttc_info.first =
      distance_info.first /
      (std::fabs(speed_info.second) + std::numeric_limits<double>::epsilon());
  // longitudinal
  ttc_info.second =
      distance_info.second /
      (std::fabs(speed_info.first) + std::numeric_limits<double>::epsilon());

  std::get<0>(ttc_infos_) = distance_info.second;
  std::get<1>(ttc_infos_) = distance_info.first;
  std::get<2>(ttc_infos_) = speed_info.first;
  std::get<3>(ttc_infos_) = speed_info.second;
  std::get<4>(ttc_infos_) = ttc_info.first;
  std::get<5>(ttc_infos_) = ttc_info.second;
  // set ttc info for sort
  auto* mutable_min_ttc_info = aeb_result->mutable_min_ttc_info();
  mutable_min_ttc_info->set_lateral_ttc(RoundToFixed(ttc_info.first, 3));
  mutable_min_ttc_info->set_longitudinal_ttc(RoundToFixed(ttc_info.second, 3));

  AINFO << "min_ttc: " << min_ttc
        << ", ttc_info, lateral_ttc: " << ttc_info.first
        << ", longitudinal_ttc: " << ttc_info.second;

  if (GetEmergencyResult(local_view, original_distance_info_)) {
    // special case, emergency warning level when obstacle is too close, even if
    // obstacle is static
    AINFO << "enter emergency case, set high level.";
    SetWarningLevelWithMinFrames(
        obstacle, ::century::planning::AebWarningLevel::WARNING_LEVEL_HIGH,
        aeb_result);
    AINFO << "after emergency check, aeb_result: "
          << aeb_result->warning_level();
  } else if (ttc_info.second < FLAGS_long_ttc_high_threshold ||
             ttc_info.first < FLAGS_lateral_ttc_high_threshold) {
    SetWarningLevelWithMinFrames(
        obstacle, ::century::planning::AebWarningLevel::WARNING_LEVEL_HIGH,
        aeb_result);
  } else if (ttc_info.second < FLAGS_long_ttc_medium_threshold ||
             ttc_info.first < FLAGS_lateral_ttc_medium_threshold) {
    SetWarningLevelWithMinFrames(
        obstacle, ::century::planning::AebWarningLevel::WARNING_LEVEL_MEDIUM,
        aeb_result);
  } else if (ttc_info.second < FLAGS_long_ttc_low_threshold ||
             ttc_info.first < FLAGS_lateral_ttc_low_threshold) {
    SetWarningLevelWithMinFrames(
        obstacle, ::century::planning::AebWarningLevel::WARNING_LEVEL_LOW,
        aeb_result);
  } else {
    SetWarningLevelWithMinFrames(
        obstacle, ::century::planning::AebWarningLevel::WARNING_LEVEL_NONE,
        aeb_result);
  }
  AINFO << "after calculate, aeb_result: " << aeb_result->warning_level()
        << ", ob_id: " << obstacle.id()
        << ", obstacle_position_debug_string: " << obstacle.DebugString();
  return true;
}

bool AebPlanner::GetEmergencyResult(
    const LocalView& local_view,
    const std::tuple<double, double, century::perception::PerceptionObstacle*>
        origin_distance_info) {
  double aeb_emergency_length_distance = aeb_config_.aeb_emergency_length_distance();
  double aeb_emergency_width_distance = aeb_config_.aeb_emergency_width_distance();
  if (!aeb_config_.enable_closest_obstacle_aeb()) {
    AERROR << "aeb closest obstacle given high warning level is disabled, "
              "return false.";
    return false;
  }
  if (!std::get<2>(origin_distance_info)) {
    ADEBUG << "emergency obstacle is nullptr, need to notice.";
  }
  if (std::fabs(local_view.chassis->speed_mps()) <
          aeb_config_.aeb_low_speed_threshold() &&
      !aeb_config_.enable_adc_static_closest_obstacle_aeb()) {
    // only used for moving status
    AINFO << "closest obstacle enable aeb only used for moving status, adc "
             "speed < aeb_low_speed_threshold (m/s), return false.";
    is_stop_by_closest_obstacle_ = false;
    return false;
  }
  const auto gear = aeb_injector_->vehicle_state()->vehicle_state().gear();
  const auto& [y, x, emergency_ob] = origin_distance_info;
  // Dongjiazhen junction, reduce buffer
  common::PointENU adc_point_enu;
  adc_point_enu.set_x(aeb_injector_->vehicle_state()->vehicle_state().x());
  adc_point_enu.set_y(aeb_injector_->vehicle_state()->vehicle_state().y());
  if (util::IsGateJunctionContainAdc(
          adc_point_enu, aeb_injector_->vehicle_state()->vehicle_state())) {
    AINFO << "special case: in dongjiazhen gate, reduce aeb buffer.";
    aeb_emergency_length_distance =
        aeb_config_.aeb_emergency_length_min_distance();
    aeb_emergency_width_distance =
        aeb_config_.aeb_emergency_width_min_distance();
  }
  ADEBUG << "current_gear: " << gear << ", nearest_dis_fabs_y: " << std::fabs(y)
         << ", y_threshold: "
         << (0.5 * vehicle_param_.width() + aeb_emergency_width_distance)
         << ", fabs_x: " << std::fabs(x) << ", x_threshold: "
         << (0.5 * vehicle_param_.length() + aeb_emergency_length_distance);

  // around adc, set high warning level
  if (std::fabs(y) <
          (0.5 * vehicle_param_.width() + aeb_emergency_width_distance) &&
      std::fabs(x) <
          (0.5 * vehicle_param_.length() + aeb_emergency_length_distance)) {
    AINFO << "find emergency obstacle, set high level.";
    is_stop_by_closest_obstacle_ = true;
    // emergency_obstacle_.CopyFrom(*emergency_ob);
    AINFO << "emergency obstacle_id: " << emergency_ob->id()
          << ", emergency_dis_x: " << x << ", y: " << y;
    return true;
  }
  return false;
}

bool AebPlanner::getAebResult(const double start_time,
                              const LocalView& local_view,
                              const double& ego_speed,
                              AebObstacleInfo aeb_obstacles_info,
                              ADCTrajectory* adc_trajectory_pb,
                              AebResult* aeb_result) {
  std::vector<std::pair<AebResult, century::perception::PerceptionObstacle>>
      aeb_result_vec_info;
  AebResult aeb_res;
  aeb_res.clear_warning_level();
  aeb_result_vec_info.clear();
  for (auto& info : aeb_obstacles_info) {
    auto& aeb_obstacle = info.second;
    CalcTTCFromEgoToAebObstacle(local_view, aeb_obstacle, ego_speed, &aeb_res);
    aeb_result_vec_info.emplace_back(std::make_pair(aeb_res, aeb_obstacle));
  }
  aeb_result_vec_info.shrink_to_fit();
  AINFO << "aeb_result_vec_info.size: " << aeb_result_vec_info.size();
  if (aeb_result_vec_info.empty()) {
    const std::string msg = "aeb_result_vec is empty. return false";
    error_messages_ += "\n";
    error_messages_ += msg;
    AERROR << msg;
    return false;
  }

  std::pair<AebResult, century::perception::PerceptionObstacle>
      aeb_final_result_pair = *std::max_element(
          aeb_result_vec_info.begin(), aeb_result_vec_info.end(),
          [this](const auto& a, const auto& b) {
            // warning level maybe inaccurate because of frame counting
            // strategy, so we must add other sort strategy warning level is
            // must important
            if (static_cast<int>(a.first.warning_level()) !=
                static_cast<int>(b.first.warning_level())) {
              return static_cast<int>(a.first.warning_level()) <
                     static_cast<int>(b.first.warning_level());
            }
            // secondly: using ttc
            const double a_min_ttc =
                CalcEffectiveMinTTC(a.first.min_ttc_info().lateral_ttc(),
                                    a.first.min_ttc_info().longitudinal_ttc());
            const double b_min_ttc =
                CalcEffectiveMinTTC(b.first.min_ttc_info().lateral_ttc(),
                                    b.first.min_ttc_info().longitudinal_ttc());
            const bool a_finite = std::isfinite(a_min_ttc);
            const bool b_finite = std::isfinite(b_min_ttc);
            if (a_finite != b_finite) {
              // Valid TTC takes precedence over invalid TTC
              // if a is invaild, means a < b
              return !a_finite;
            }
            if (a_finite && b_finite && a_min_ttc != b_min_ttc) {
              // small ttc is more dangerous
              return a_min_ttc > b_min_ttc;
            }
            // thirdly: using distance
            const auto& obs_a = a.second;
            const auto& obs_b = b.second;
            double dist_a_sq =
                std::hypot(obs_a.position().x(), obs_a.position().y());
            double dist_b_sq =
                std::hypot(obs_b.position().x(), obs_b.position().y());
            return dist_a_sq > dist_b_sq;
          });
  // output
  *aeb_result = aeb_final_result_pair.first;
  aeb_selected_obstacle_.CopyFrom(aeb_final_result_pair.second);
  ADEBUG << "corresponding_obstacle: "
         << aeb_final_result_pair.second.DebugString();
  if (std::fabs(local_view.chassis->speed_mps()) <
          aeb_config_.aeb_low_speed_threshold() &&
      !aeb_config_.enable_adc_static_closest_obstacle_aeb()) {
    // adc is stop, do not send any warning level
    aeb_result->set_warning_level(
        ::century::planning::AebWarningLevel::WARNING_LEVEL_NONE);
    AINFO << "### disabled aeb, aeb_result->warning_level: "
          << aeb_result->warning_level();
    return false;
  }
  AINFO << "final after select, aeb_result->warning_level: "
         << aeb_result->warning_level() << ", gear: "
         << aeb_injector_->vehicle_state()->vehicle_state().gear();
  if (aeb_result->warning_level() ==
          ::century::planning::AebWarningLevel::WARNING_LEVEL_LOW ||
      aeb_result->warning_level() ==
          ::century::planning::AebWarningLevel::WARNING_LEVEL_MEDIUM ||
      aeb_result->warning_level() ==
          ::century::planning::AebWarningLevel::WARNING_LEVEL_HIGH) {
    ADEBUG << "aeb_result_pb.debug_string: " << aeb_result->DebugString();
    return true;
  }
  return false;
}

bool AebPlanner::GetAebResultFromPerceptionObstacles(
    const double start_time, LocalView& local_view,
    ADCTrajectory* adc_trajectory_pb, AebResult* aeb_result) {
  ADEBUG << __func__;
  error_messages_.clear();
  if (!local_view.perception_aeb_obstacles) {
    const std::string msg =
        "local_view.perception_aeb_obstacles is nullptr, return false. ";
    error_messages_ += msg;
    AERROR << msg;
    return false;
  }
  if (!local_view.localization_estimate || !local_view.chassis) {
    const std::string msg =
        "local_view.localization_estimate or chassis is nullptr, return false. ";
    error_messages_ += msg;
    AERROR << msg;
    return false;
  }
  if (aeb_config_.enable_print_perception_ego_obs_info()) {
    // for debug input obstacle infos
    ADEBUG << "*** start printing perception around ego obstacle info ***";
    for (auto& obstacle :
         local_view.perception_aeb_obstacles->perception_obstacle()) {
      ADEBUG << "perception around ego obstacle id: " << obstacle.id();
      for (const auto polygon_pt : obstacle.polygon_point()) {
        ADEBUG << "perception around ego obstacle polygon point: "
               << polygon_pt.DebugString();
      }
      // AINFO << "perception around ego obstacle debugstring: " <<
      // obstacle.DebugString();
    }
    ADEBUG << "--- end of printing perception around ego obstacle info ---";
  }

  is_auto_mode_ = local_view.chassis.get()->driving_mode() ==
                  canbus::Chassis::COMPLETE_AUTO_DRIVE;
  if (!is_auto_mode_) {
    // if not in auto mode, reset min warning level frames counter
    s_high_warning_frames_left = 0;
  }
  AINFO << "is_auto_mode_: " << is_auto_mode_
        << ", min_frames: " << s_high_warning_frames_left;
  century::common::Status status = aeb_injector_->vehicle_state()->Update(
      *local_view.localization_estimate, *local_view.chassis, false);
  if (!status.ok() || !util::IsVehicleStateValid(
                          aeb_injector_->vehicle_state()->vehicle_state())) {
    error_messages_ += "\n";
    error_messages_ += status.code();
    AERROR << "Failed to update vehicle state: " << status.code();
    const std::string msg =
        "Update VehicleStateProvider failed in AEB"
        "or the vehicle state is out dated.";
    error_messages_ += ", ";
    error_messages_ += msg;
    AERROR << msg;
  }
  AINFO << "input aeb obstacles size: "
         << local_view.perception_aeb_obstacles->perception_obstacle().size();
  auto vehicle_state = aeb_injector_->vehicle_state();
  auto aeb_obstacles = CreateAebObstacles(*local_view.perception_aeb_obstacles);
  AINFO << "after filter, aeb obstacles size: " << aeb_obstacles.size();

  std::vector<perception::PerceptionObstacle> filter_obstacles;
  for (const auto& obstacle_info : aeb_obstacles) {
    filter_obstacles.emplace_back(obstacle_info.second);
  }
  track_obstacles_ =
      obstacle_tracker_->Track(filter_obstacles, Clock::NowInSeconds());

  // need to comfirm that input speed is positive or negative
  AINFO << "aeb_seq: " << seq_num_;
  if (ReadyToEnableAeb(local_view)) {
    aeb_result->set_ready_to_enable_aeb(true);
  }
  AINFO << "ready_to_enable_aeb: " << aeb_result->ready_to_enable_aeb();

  if (getAebResult(start_time, local_view, vehicle_state->linear_velocity(),
                   aeb_obstacles, adc_trajectory_pb, aeb_result)) {
    // if needed to brake, return true
    AINFO << "AEB[" << aeb_result->warning_level()
          << "] is started, return true";
    return true;
  }
  AINFO << "final, do not send aeb, aeb_result: "
        << aeb_result->warning_level();
  return false;
}

bool AebPlanner::HasObstacleAroundAdc(const LocalView& local_view,
                                      const double& length_buffer,
                                      const double& width_buffer) {
  const double ego_length = vehicle_param_.length() + length_buffer;
  const double ego_width = vehicle_param_.width() + width_buffer;
  double min_dis = std::numeric_limits<double>::max();
  common::math::Box2d ego_box({0.0, 0.0}, 0.0, ego_length, ego_width);
  common::math::Polygon2d ego_polygon(ego_box);

  for (const auto& perception_obstacle :
       local_view.perception_aeb_obstacles->perception_obstacle()) {
    if (!Obstacle::IsValidPerceptionObstacle(perception_obstacle)) {
      continue;
    }
    if (!AebObstacleDistanceFilter(perception_obstacle,
                                   FLAGS_aeb_obstacle_distance_threshold,
                                   min_dis)) {
      // adc to obstacle distance > aeb_obstacle_distance_threshold(25m), need
      // filter
      continue;
    }
    std::vector<common::math::Vec2d> polygon_points;
    polygon_points.reserve(perception_obstacle.polygon_point_size());
    for (const auto& pt : perception_obstacle.polygon_point()) {
      polygon_points.emplace_back(pt.x(), pt.y());
    }

    if (polygon_points.size() < 3) {
      continue;
    }
    common::math::Polygon2d obs_polygon(polygon_points);
    // double min_distance = ego_box.DistanceTo(obs_polygon);
    if (ego_polygon.HasOverlap(obs_polygon)) {
      AINFO << "id: " << perception_obstacle.id()
            << " has overlap with adc box, obstacle_info: "
            << perception_obstacle.ShortDebugString();
      return true;
    }
  }
  return false;
}

// true: no obstacle around, enable aeb, false: has obstacle, do not enable aeb
bool AebPlanner::ReadyToEnableAeb(const LocalView& local_view) {
  AINFO << __func__;
  if (!local_view.fas_aeb_result) {
    const std::string msg = "fas_aeb_result topic is empty.";
    error_messages_ += "\n";
    error_messages_ += msg;
    return false;
  }
  AINFO << "fas_aeb_switch: " << local_view.fas_aeb_result->fas_aeb_switch()
        << ", ego_speed: " << local_view.chassis->speed_mps();
  is_auto_enable_aeb_ = false;
  if (!aeb_config_.enable_auto_aeb()) {
    AERROR << "auto enable aeb config is disabled.";
    const std::string msg = "auto enable aeb config is false.";
    error_messages_ += "\n";
    error_messages_ += msg;
    return false;
  }
  if (local_view.fas_aeb_result->fas_aeb_switch()) {
    // if aeb aready enabled, return false
    has_obstacle_counter_ = 0;
    AERROR << "aeb already enable, do not need to enable aeb, reset counter.";
    const std::string msg = "aeb already enable, do not need to enable aeb, reset counter.";
    error_messages_ += "\n";
    error_messages_ += msg;
    return false;
  }
  if (std::fabs(local_view.chassis->speed_mps()) <
      aeb_config_.aeb_low_speed_threshold()) {
    // if adc is static, do not ready to enable aeb
    has_obstacle_counter_ = 0;
    AINFO << "adc is static, do not need to enable aeb, reset counter.";
    const std::string msg =
        "adc is static, do not need to enable aeb, reset counter.";
    error_messages_ += "\n";
    error_messages_ += msg;
    return false;
  }
  if (HasObstacleAroundAdc(local_view,
                           aeb_config_.aeb_emergency_length_distance(),
                           aeb_config_.aeb_emergency_width_distance())) {
    AINFO << "obstacle close to adc, must enable aeb.";
    is_auto_enable_aeb_ = true;
    return true;
  }

  if (!HasObstacleAroundAdc(local_view,
                            aeb_config_.auto_enable_aeb_longit_buffer(),
                            aeb_config_.auto_enable_aeb_lat_buffer())) {
    // no obstacle for several frames, ready to enable aeb
    has_obstacle_counter_ = (has_obstacle_counter_ < SIZE_MAX)
                                ? (has_obstacle_counter_ + 1)
                                : has_obstacle_counter_;
    AINFO << "counter: " << has_obstacle_counter_;
    if (has_obstacle_counter_ >= aeb_config_.auto_enable_aeb_frames()) {
      is_auto_enable_aeb_ = true;
      AINFO << "No obstacle around for " << has_obstacle_counter_
            << " frames, need start enable fas aeb switch.";
      return true;
    }
  } else {
    // otherwise, reset counter for 0, next time need to recounter
    has_obstacle_counter_ = 0;
    AINFO << "has around obstacle, reset counter for 0.";
  }
  return false;
}

void AebPlanner::GenerateStopTrajectory(
    const common::VehicleStateProvider vehicle_state_input,
    const bool enable_generate_stop_trajectory,
    ADCTrajectory* ptr_trajectory_pb) {
  if (!enable_generate_stop_trajectory) {
    return;
  }
  ptr_trajectory_pb->clear_trajectory_point();
  const auto& vehicle_state = vehicle_state_input.vehicle_state();
  const double max_t = FLAGS_fallback_total_time;  // 3.0 s
  const double unit_t = FLAGS_fallback_time_unit;  // 0.1 s

  common::TrajectoryPoint tp;
  auto* path_point = tp.mutable_path_point();
  path_point->set_x(vehicle_state.x());
  path_point->set_y(vehicle_state.y());
  path_point->set_theta(vehicle_state.heading());
  path_point->set_s(0.0);
  tp.set_v(0.0);
  tp.set_a(0.0);
  for (double t = 0.0; t < max_t; t += unit_t) {
    tp.set_relative_time(t);
    auto next_point = ptr_trajectory_pb->add_trajectory_point();
    next_point->CopyFrom(tp);
  }
  return;
}

double AebPlanner::MakeTTCEffective(double ttc) {
  if (std::isnan(ttc)) {
    return std::numeric_limits<double>::infinity();
  }
  if (!std::isfinite(ttc)) {
    return std::numeric_limits<double>::infinity();
  }
  if (ttc > common::util::kMathEpsilon) {
    return std::numeric_limits<double>::infinity();
  }
  return ttc;
}

double AebPlanner::CalcEffectiveMinTTC(double lat_ttc, double lon_ttc) {
  const double lat = MakeTTCEffective(lat_ttc);
  const double lon = MakeTTCEffective(lon_ttc);

  if (std::isinf(lat) && std::isinf(lon)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::min(lat, lon);
}

double AebPlanner::RoundToFixed(double val, int precision) {
    double multiplier = std::pow(10.0, precision);
    return std::round(val * multiplier) / multiplier;
}

void AebPlanner::RecordDebugData(const LocalView& local_view,
                                 const AebResult& aeb_result,
                                 planning::AEBDebug* debug) {
  AINFO << "***** record aeb_debug data !!!!";
  if (!debug) {
    AERROR << "aeb planner debug data is nullptr, Skip record input into debug";
    return;
  }
  // obstacle debug
  bool enable_record_input_obstacles_info = false;
  auto* input_obstacles = debug->mutable_input_obstacles();
  auto& obstacles = local_view.perception_aeb_obstacles->perception_obstacle();
  if (enable_record_input_obstacles_info) {
    for (auto& obstacle : obstacles) {
      perception::PerceptionObstacle* obstacle_debug = input_obstacles->Add();
      obstacle_debug->set_id(obstacle.id());
      obstacle_debug->set_theta(obstacle.theta());
      obstacle_debug->set_length(obstacle.length());
      obstacle_debug->set_width(obstacle.width());
      obstacle_debug->set_type(obstacle.type());
      // TODO: check if this is correct
      obstacle_debug->mutable_polygon_point()->CopyFrom(
          obstacle.polygon_point());
      obstacle_debug->mutable_velocity()->CopyFrom(obstacle.velocity());
      obstacle_debug->mutable_position()->CopyFrom(obstacle.position());
    }
  }
  // fill special junction debug infos
  auto* special_junction_info = debug->mutable_special_junction_info();
  special_junction_info->set_special_junction(
      special_junction_info_.special_junction());
  special_junction_info->set_obstacle_id(special_junction_info_.obstacle_id());
  special_junction_info->mutable_filter_point()->CopyFrom(
      special_junction_info_.filter_point());
  special_junction_info->mutable_position()->CopyFrom(
      special_junction_info_.position());
  AINFO << "special_junction_info: ";
  AINFO << "  junction_name: " << special_junction_info_.special_junction()
        << ", ib_id: " << special_junction_info_.obstacle_id()
        << ", filter_point: (" << special_junction_info_.filter_point().x()
        << ", " << special_junction_info_.filter_point().y() << ", "
        << special_junction_info_.filter_point().z() << "); " << "position: ("
        << special_junction_info_.position().x() << ", "
        << special_junction_info_.position().y() << ", "
        << special_junction_info_.position().z() << ")";

  // fill closest obstacle debug infos
  auto* closest_obstacle_info = debug->mutable_closest_obstacle_info();
  // auto* closest_ob = std::get<2>(original_distance_info_);
  closest_obstacle_info->mutable_closest_distance()->set_x(
      (std::fabs(std::get<1>(original_distance_info_) -
                 std::numeric_limits<double>::max()) <
       common::util::kMathEpsilon)
          ? 99999
          : std::get<1>(original_distance_info_));
  closest_obstacle_info->mutable_closest_distance()->set_y(
      (std::fabs(std::get<1>(original_distance_info_) -
                 std::numeric_limits<double>::max()) <
       common::util::kMathEpsilon)
          ? 99999
          : std::get<0>(original_distance_info_));
  closest_obstacle_info->set_is_stop_by_closest_obstacle(
      is_stop_by_closest_obstacle_);
  AINFO << "closest_ob.x: " << std::get<1>(original_distance_info_)
        << ", y: " << std::get<0>(original_distance_info_);
  // if (is_stop_by_closest_obstacle_ && closest_ob) {
  //   closest_obstacle_info->set_id(closest_ob->id());
  //   closest_obstacle_info->mutable_position()->set_x(
  //       closest_ob->position().x());
  //   closest_obstacle_info->mutable_position()->set_y(
  //       closest_ob->position().y());
  //   closest_obstacle_info->mutable_position()->set_z(
  //       closest_ob->position().z());
  //   closest_obstacle_info->set_theta(closest_ob->theta());
  //   closest_obstacle_info->mutable_velocity()->set_x(
  //       closest_ob->velocity().x());
  //   closest_obstacle_info->mutable_velocity()->set_y(
  //       closest_ob->velocity().y());
  //   closest_obstacle_info->mutable_velocity()->set_z(
  //       closest_ob->velocity().z());
  //   closest_obstacle_info->set_length(closest_ob->length());
  //   closest_obstacle_info->set_width(closest_ob->width());
  //   closest_obstacle_info->set_height(closest_ob->height());
  //   AINFO << "closest_ob->polygon_point().size(): "
  //          << closest_ob->polygon_point_size();
  // }
  auto* selected_obstacle_info = debug->mutable_selected_obstacles();
  selected_obstacle_info->mutable_min_distance_obstacle_info()->set_obstacle_id(
      min_dis_obstacle_info_.second);
  selected_obstacle_info->mutable_min_distance_obstacle_info()
      ->set_min_distance(
          (min_dis_obstacle_info_.first - std::numeric_limits<float>::max() <
           common::util::kMathEpsilon)
              ? 99999
              : min_dis_obstacle_info_.first);
  if (min_dis_obstacle_position_.size() > 1) {
    selected_obstacle_info->mutable_min_distance_obstacle_info()
        ->mutable_position()
        ->set_x((min_dis_obstacle_position_.at(0) -
                     std::numeric_limits<float>::max() <
                 common::util::kMathEpsilon)
                    ? 99999
                    : min_dis_obstacle_position_.at(0));
    selected_obstacle_info->mutable_min_distance_obstacle_info()
        ->mutable_position()
        ->set_y((min_dis_obstacle_position_.at(1) -
                     std::numeric_limits<float>::max() <
                 common::util::kMathEpsilon)
                    ? 99999
                    : min_dis_obstacle_position_.at(1));
  }
  debug->mutable_selected_obstacles()->mutable_stop_obstacle()->CopyFrom(
      aeb_selected_obstacle_);

  debug->mutable_ttc_infos()->set_d_lon(
      (std::fabs(std::get<0>(ttc_infos_) - std::numeric_limits<float>::max()) <
       common::util::kMathEpsilon)
          ? 99999
          : std::get<0>(ttc_infos_));
  debug->mutable_ttc_infos()->set_d_lat(
      (std::fabs(std::get<0>(ttc_infos_) - std::numeric_limits<float>::max()) <
       common::util::kMathEpsilon)
          ? 99999
          : std::get<1>(ttc_infos_));
  debug->mutable_ttc_infos()->set_v_lon(
      (std::fabs(
           (std::get<0>(ttc_infos_) - std::numeric_limits<float>::max())) <
       common::util::kMathEpsilon)
          ? 99999
          : std::get<2>(ttc_infos_));
  debug->mutable_ttc_infos()->set_v_lat(
      (std::fabs(std::get<0>(ttc_infos_) - std::numeric_limits<float>::max()) <
       common::util::kMathEpsilon)
          ? 99999
          : std::get<3>(ttc_infos_));
  debug->mutable_ttc_infos()->set_min_lateral_ttc(
      (std::fabs(std::get<0>(ttc_infos_) - std::numeric_limits<float>::max()) <
       common::util::kMathEpsilon)
          ? 99999
          : std::get<4>(ttc_infos_));
  debug->mutable_ttc_infos()->set_min_long_ttc(
      (std::fabs(std::get<0>(ttc_infos_) - std::numeric_limits<float>::max()) <
       common::util::kMathEpsilon)
          ? 99999
          : std::get<5>(ttc_infos_));
  debug->set_calc_dis_is_overlap(calc_dis_is_overlap_);
  debug->set_calc_dis_is_both_overlap(calc_dis_is_both_overlap_);
  debug->set_error_info(error_messages_);

  // auto enable aeb debug
  debug->mutable_auto_enable_aeb_info()->set_ready_to_enable_aeb(
      is_auto_enable_aeb_);
  debug->mutable_auto_enable_aeb_info()->set_has_not_obs_around_counter(
      has_obstacle_counter_);

  AINFO << "is_overlap: " << calc_dis_is_overlap_
        << ", is_both_overlap: " << calc_dis_is_both_overlap_;
  AINFO << "debug_input_obstacles: " << input_obstacles->size();
  AINFO << "debug_selected_obstacle_info: "
         << selected_obstacle_info->DebugString();
  AINFO << "is_stop_by_closest_obstacle: " << is_stop_by_closest_obstacle_;
  AINFO << "debug_min_lateral_ttc_: " << std::get<4>(ttc_infos_)
        << ", min_long_ttc_:" << std::get<5>(ttc_infos_);
  AINFO << "d_lon: " << debug->ttc_infos().d_lon()
        << ", d_lat: " << debug->ttc_infos().d_lat()
        << ", v_lon: " << debug->ttc_infos().v_lon()
        << ", v_lat: " << debug->ttc_infos().v_lat()
        << ", min_ttc: " << debug->ttc_infos().min_ttc();
  AINFO << "error_messages_debug_string: " << error_messages_;
  AINFO << "is_auto_enable_aeb: " << is_auto_enable_aeb_
        << ", counter: " << has_obstacle_counter_;

  // aeb debug
  // debug->mutable_aeb_result()->CopyFrom(aeb_result);

  return;
}

}  // namespace planning
}  // namespace century
