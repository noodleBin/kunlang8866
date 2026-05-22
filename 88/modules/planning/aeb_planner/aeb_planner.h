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

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <climits>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/monitor/proto/system_status.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_aeb.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/status/status.h"
#include "modules/common/util/normal_util.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/aeb_obstacle_track/aeb_obstacle_track.h"
#include "modules/planning/common/local_view.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/common/util/util.h"

namespace century {
namespace planning {

using century::cyber::Clock;

class AebPlanner {
 public:
  AebPlanner(const AebConfig& aeb_config);
  ~AebPlanner() = default;

 public:
  bool init();
  void UpdateMapInfo();
  void set_aeb_config(AebConfig aeb_config) { aeb_config_ = aeb_config; };

  struct DowngradeHighLevelInfo {
    int downgrade_counter = 0;
    int not_high_level_counter = 0;
    bool is_first_time_in_high_level = true;
    bool has_downgrade = false;
  };

  enum class AebTurnType {
    NO_TURN = 0,
    LEFT_TURN = 1,
    RIGHT_TURN = 2,
  };
  using AebObstacleInfo =
      std::vector<std::pair<double, perception::PerceptionObstacle>>;

 public:
  void setSeqNum(size_t seq_num) { seq_num_ = seq_num; };

  size_t getSeqNum() { return seq_num_; };

  double GetLateralSpeed() { return v_lateral_; };

  template <typename T>
  void ConvertPointsToLineSegments(
      const std::vector<std::vector<T>>& points_vec,
      std::vector<std::vector<common::math::LineSegment2d>>& linesegments_vec) {
    std::vector<common::math::LineSegment2d> temp_linesegments;
    temp_linesegments.clear();
    for (const auto& points : points_vec) {
      ConvertPointsToLineSegments(points, temp_linesegments);
      linesegments_vec.emplace_back(temp_linesegments);
    }
    return;
  }

  template <typename T>
  void ConvertPointsToLineSegments(
      const std::vector<T>& points,
      std::vector<common::math::LineSegment2d>& linesegments) {
    size_t vertices_num = points.size();
    for (size_t i = 0; i + 1 < vertices_num; ++i) {
      common::math::LineSegment2d line_segment =
          common::math::LineSegment2d(points[i], points[i + 1]);
      linesegments.emplace_back(line_segment);
    }
    return;
  }

  /**
   * 2025-12-25 requirement
   * @brief downgrade first high frame into medium or low level
   * @param candidate_level current warning level after calculation
   * @return final output warning level when downgrade
   */
  ::century::planning::AebWarningLevel DowngradeFirstHighFrame(
      const int& track_id, ::century::planning::AebWarningLevel candidate_level,
      ::century::planning::AebWarningLevel target_downgrade_level);

  /**
   * 2025-12-05 requirement
   * @brief check high level with history frames
   * @param current_level current warning level after calculation
   * @return final output warning level
   */
  ::century::planning::AebWarningLevel CheckHighLevelWithHistoryFrames(
      ::century::planning::AebWarningLevel current_level);
  /**
   * 2025-12-05 requirement
   * @brief check whether the current level is high level for consecutive frames
   * @param current_level current warning level after calculation
   * @return final output warning level
   */
  ::century::planning::AebWarningLevel CheckHighLevelWithConsecutiveFrames(
      const perception::PerceptionObstacle& obstacle, const int& track_id,
      ::century::planning::AebWarningLevel current_level);

  bool IsFarAwayToTheSide(const perception::PerceptionObstacle& obstacle,
                          const std::pair<double, double>& dis_info);

  bool IsTurnScene(const double& average_wheel_angle);

  void SetWarningLevelWithMinFrames(
      const perception::PerceptionObstacle& obstacle,
      ::century::planning::AebWarningLevel candidate_level,
      ::century::planning::AebResult* aeb_result);

  void UpdateHighWarningFramesMapInfo();

  void UpdateFirstTimeInHighLevelMapInfo();

  AebTurnType GetTurningResult(const LocalView& local_view);

  /// @brief Calculate the lateral speed of ego
  /// @param local_view local view
  /// @param ego_speed ego speed
  /// @param v_lon lontitudinal speed
  /// @param v_lat lateral speed
  /// @param enable_use_yaw_rate flag to enable use yaw rate
  void CalculateLateralSpeed(const LocalView& local_view,
                             const double& ego_speed, double& v_lon,
                             double& v_lat,
                             const bool enable_use_yaw_rate = true);

  void CalcAccurateSpeed(const LocalView& local_view, double& v_lon,
                         double& v_lat, double& yaw_rate);

  void CalcAccurateSpeed(const LocalView& local_view, double& v_lon,
                         double& v_lat, double& yaw_rate,
                         const double& yaw_rate_coff);

  void CalcSpeed(const double& speed, const double& wheel_angle, double& v_lon,
                 double& v_lat);

  bool CalcCollisionDistance(const LocalView& local_view,
                             const perception::PerceptionObstacle& obstacle,
                             std::pair<double, double>& distance_info);

  bool HasOverlapWithBuffer(const perception::PerceptionObstacle& obstacle,
                            const double& length_buffer, double& width_buffer,
                            const bool is_reverse = false);

  bool FindTargetObstacleTypeInRange(
      const perception::PerceptionObstacles& perceptions,
      const perception::PerceptionObstacle::Type target_type);

  bool AebObstacleHeightFilterForStacker(
      const perception::PerceptionObstacle& obstacle,
      const bool& has_stacker_in_range);

  bool AebObstacleSpeedFilter(const perception::PerceptionObstacle& obstacle);

  bool AebObstaclePositionFilter(
      const perception::PerceptionObstacle& obstacle);

  bool IgnoreLongitTTC(const perception::PerceptionObstacle& obstacle,
                       const std::pair<double, double>& dis_info);

  bool AebNearestPointObstacleFilter(
      const LocalView& local_view,
      const perception::PerceptionObstacle& obstacle,
      const common::math::Vec2d nearest_point,
      const std::vector<common::math::Vec2d>& ego_points);

  bool AebObstacleHeightFilterInJunction(
      const perception::PerceptionObstacle& obstacle);

  bool AebObstacleDistanceFilter(const perception::PerceptionObstacle& obstacle,
                                 const double& aeb_obstacle_distance_threshold,
                                 double& min_distance);

  AebObstacleInfo CreateAebObstacles(
      const perception::PerceptionObstacles& perceptions);

  void UpdateOverlapDistanceInfo(
      const perception::PerceptionObstacle& obstacle,
      const std::pair<double, double> dis_info,
      std::pair<double, double>& updated_distance_info);

  bool CalcTTCFromEgoToAebObstacle(
      const LocalView& local_view,
      const perception::PerceptionObstacle& obstacle, const double& ego_speed,
      AebResult* aeb_result);

  bool GetEmergencyResult(
      const LocalView& local_view,
      const std::tuple<double, double, century::perception::PerceptionObstacle*>
          distance_info);

  bool getAebResult(const double start_time, const LocalView& local_view,
                    const double& ego_speed, AebObstacleInfo aeb_obstacles_info,
                    ADCTrajectory* adc_trajectory_pb, AebResult* aeb_result);

  bool GetAebResultFromPerceptionObstacles(const double start_time,
                                           LocalView& local_view,
                                           ADCTrajectory* adc_trajectory_pb,
                                           AebResult* aeb_result);

  bool HasObstacleAroundAdc(const LocalView& local_view,
                            const double& length_buffer,
                            const double& width_buffer);

  bool ReadyToEnableAeb(const LocalView& local_view);

  void GenerateStopTrajectory(
      const common::VehicleStateProvider vehicle_state_input,
      const bool enable_generate_stop_trajectory,
      ADCTrajectory* ptr_trajectory_pb);

  double CalcEffectiveMinTTC(double lat_ttc, double lon_ttc);

  double MakeTTCEffective(double ttc);

  double RoundToFixed(double val, int precision);

  void RecordDebugData(const LocalView& local_view, const AebResult& aeb_result,
                       planning::AEBDebug* debug);

 private:
  double CalcMinDistanceFromPointToEgo(
      const common::math::Vec2d& point,
      const std::vector<common::math::Vec2d>& ego_points) const;
  bool IsAllPointsBehindEgoByMotionDirection(
      const perception::PerceptionObstacle& obstacle);

  std::shared_ptr<DependencyInjector> aeb_injector_;

  // debug
  bool is_auto_enable_aeb_ = false;
  SpecialJunctionInfo special_junction_info_;
  bool is_stop_by_closest_obstacle_ = false;
  bool calc_dis_is_both_overlap_ = false;
  bool calc_dis_is_overlap_ = false;
  std::pair<float, std::string> min_dis_obstacle_info_ =
      std::make_pair(std::numeric_limits<float>::max(), "");
  std::array<float, 3> min_dis_obstacle_position_ = {
      std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max()};
  std::tuple<float, float, float, float, float, float> ttc_infos_ =
      std::make_tuple(std::numeric_limits<double>::max(),
                      std::numeric_limits<double>::max(),
                      std::numeric_limits<double>::max(),
                      std::numeric_limits<double>::max(),
                      std::numeric_limits<double>::max(),
                      std::numeric_limits<double>::max());
  century::perception::PerceptionObstacle aeb_selected_obstacle_;
  std::unique_ptr<ObstacleTracker> obstacle_tracker_;

  bool is_auto_mode_ = false;
  size_t has_obstacle_counter_ = 0;
  double v_lateral_ = 0.0;
  size_t seq_num_ = 0;
  double process_time_ = 0;
  std::string error_messages_ = "";
  century::common::VehicleParam vehicle_param_;
  AebConfig aeb_config_;

  double average_wheel_angle_ = 0.0;
  double is_turning_ = false;
  bool has_downgrade_ = false;
  int downgrade_counter_ = 0;
  int not_high_level_counter_ = 0;
  std::unordered_map<int, bool> overlap_obstacle_map_;
  // track_obstacle_id, downgrade infos
  std::unordered_map<int, DowngradeHighLevelInfo> first_time_in_high_level_map_;
  // track_obstacle_id, frames_left
  std::unordered_map<int, int> high_warning_frames_map_;
  std::vector<century::planning::TrackResult> track_obstacles_;
  // y, x, obstacle info
  std::tuple<double, double, perception::PerceptionObstacle*>
      original_distance_info_ =
          std::make_tuple(std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max(), nullptr);

 private:
  common::util::TurnStateDetector turn_detector_{0.12, 0.08, 3, 5};

 private:
  // in order to record the speed history of speed filter
  struct SpeedHistory {
    std::deque<double> relative_speeds;
    int history_size = 0;
  };
  std::unordered_map<int, SpeedHistory> speed_history_map_;
  const int kSpeedHistorySize = 5;  // used 5 frames of history data
  const double kSpeedFilterThreshold =
      0.3;  // 30% of speed fluctuation tolerance
};

}  // namespace planning
}  // namespace century
