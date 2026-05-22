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
 * @file
 **/

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/ml/ml.hpp>
#include <opencv2/opencv.hpp>

#include "modules/planning/common/frame.h"
#include "modules/planning/common/historical_tracking_algorithms/obstacle_history_diff_value.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/svm_feature_data.pb.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace century {
namespace planning {

class SpeedBoundsDecider : public Decider {
 public:
  SpeedBoundsDecider(const TaskConfig& config,
                     const std::shared_ptr<DependencyInjector>& injector);

 private:
  struct ObstacleInfo {
    ObstacleInfo(uint32_t stop_times, uint32_t speed_limit_times,
                 uint32_t seq_num)
        : delay_stop_times(stop_times),
          delay_speed_limit_times(speed_limit_times),
          last_seq_num(seq_num) {}
    uint32_t delay_stop_times;
    uint32_t delay_speed_limit_times;
    uint32_t last_seq_num;
  };
  using DangerousObsContainer = std::unordered_map<std::string, ObstacleInfo>;
  using DangerousObsForSpeedLimitContainer =
      std::unordered_map<std::string, size_t>;
  enum ObsLongiPos {
    UNKNOWN_POS = 0,
    BEHIND_ADC_TOTALLY,
    BEHIND_ADC_WITH_OVERLAP,
    FRONT_OF_ADC_WITH_OVERLAP,
    FRONT_OF_ADC_TOTALLY
  };

 private:
  common::Status Process(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info) override;

  double SetSpeedFallbackDistance(PathDecision* const path_decision);

  void RecordSTGraphDebug(
      const StGraphData& st_graph_data,
      planning_internal::STGraphDebug* st_graph_debug) const;
  common::Status GetSpeedLimit(ReferenceLineInfo* const reference_line_info);

  void MultiDecisionsBeforePrioriSpeedBounds(PathDecision* const path_decision);

  void MakeDangerousStartUpDecision(PathDecision* const path_decision);
  void MakeVehicleStartUpDecision(PathDecision* const path_decision);

  bool IsMoveNearToPathByObsHeading(
      const Obstacle& obstacle, const common::FrenetFramePoint& frenet_point,
      const double path_point_heading, const bool is_start_up);
  bool IsMoveAwayFromPathByObsHeading(
      const Obstacle& obstacle, const common::FrenetFramePoint& frenet_point,
      const double path_point_heading);
  bool GetScopeState(const Obstacle& obstacle,
                     const common::FrenetFramePoint& frenet_point,
                     double left_scope, double right_scope, double front_scope,
                     double rear_scope);
  bool GetScopeStateForVehicleStartUp(
      const Obstacle& obstacle, const common::FrenetFramePoint& frenet_point,
      const double left_scope, const double right_scope,
      const double front_scope, const double rear_scope);
  bool IsObstacleInScopeForVehicleStartUp(
      const Obstacle& obstacle, const common::FrenetFramePoint& frenet_point);
  bool GetScopeStateWithLatOverlap(const Obstacle& obstacle,
                                   const common::FrenetFramePoint& frenet_point,
                                   double left_scope, double right_scope,
                                   double front_scope, double rear_scope);
  void GetScopeParams(const bool is_start_up, const bool is_unknown,
                      const bool is_large_scope, double* left_scope,
                      double* right_scope, double* front_scope,
                      double* rear_scope);
  void GetScopeParamsForSidepass(const bool is_large_scope, double* left_scope,
                                 double* right_scope, double* front_scope,
                                 double* rear_scope);
  double GetStopDistanceForDangerousObs(const Obstacle& obstacle,
                                        const double path_point_heading);
  bool IsObstacleInScope(const Obstacle& obstacle,
                         const common::FrenetFramePoint& frenet_point,
                         const bool is_start_up);
  bool IsObstacleInLargeScope(const Obstacle& obstacle,
                              const common::FrenetFramePoint& frenet_point,
                              const bool is_start_up);
  bool IsMoveAwayFromSegmentPathByObsHeading(const Obstacle& obstacle,
                                             const PathData& path_data);
  bool IsObstacleInScopeForSidepass(
      const Obstacle& obstacle, const common::FrenetFramePoint& frenet_point,
      const bool is_large_scope);
  bool GetStartUpStatus();
  bool IsMoveTowardRearAdc(const Obstacle& obstacle);
  void UpdateContainerItem(const Obstacle& obstacle, double frenet_point_l,
                           const DangerousObsContainer::iterator& it);
  void UpdateVehicleContainerItem(const Obstacle& obstacle,
                                  double frenet_point_l,
                                  const DangerousObsContainer::iterator& it);
  void SetSpeedLimitsForKeepApproachingObs(PathDecision* const path_decision);
  void KinematicSlowlyDownForApproachingObs(PathDecision* const path_decision);
  void UpdateSpeedLimitContainerItem(
      const Obstacle& obstacle,
      const DangerousObsForSpeedLimitContainer::iterator& it);
  ObsLongiPos GetObstacleLongiPosition(const Obstacle& obstacle);
  bool IsObstacleNearPath(const Obstacle& obstacle);
  bool IsLatOverlapWithBuffer(const Obstacle& obstacle,
                              const double lat_buffer);
  bool IsSameDirectionWithNearPathPoint(const Obstacle& obstacle);
  void LatDistanceAndSpeedOfNearAdc(
      const Obstacle& obstacle, const common::FrenetFramePoint& frenet_point,
      double* lateral_dis, double* lateral_speed, double* longitude_speed);
  ObstacleHistoryDiffValue::MovingState GetObstacleMoveState(
      const Obstacle& obstacle, double* relative_speed);
  bool IsObstacleFasterMoving(
      const ObstacleHistoryDiffValue::MovingState& move_state);
  bool IsObstacleSlowerMoving(
      const ObstacleHistoryDiffValue::MovingState& move_state);
  bool IsObstacleStableMoving(
      const ObstacleHistoryDiffValue::MovingState& move_state);
  void MakeObstacleSidepassDecision(PathDecision* const path_decision);
  void MakeIgnoreOrBrakeDecisionByObstacleState(
      const Obstacle& obstacle, const common::FrenetFramePoint& frenet_point,
      const ObstacleHistoryDiffValue::MovingState move_state,
      const double relative_speed, const bool is_obs_near_to_path,
      PathDecision* const path_decision, double* target_speed,
      double* target_accel);
  void MakeIgnoreDecisionForBehindOverlapObstacle(
      const Obstacle& obstacle,
      const ObstacleHistoryDiffValue::MovingState move_state,
      const bool is_move_away_obs, const double relative_speed,
      PathDecision* const path_decision, double* target_speed,
      double* target_accel);
  bool SlowBreaking(double* min_stop_s, std::string* slow_breking_id);
  bool SlowBreakingForLowRoadRightLane(double* min_stop_s,
                                       double* min_stop_decel);
  bool IsSameDirection(const Obstacle& obstacle, double adc_theta) const;
  bool CheckObsOverlap(const common::PathPoint& path_point,
                       const planning::SLBoundary& obs_sl_boundary,
                       const common::math::Polygon2d& obs_polygon) const;
  bool GetAdcCornerPoint(const common::PathPoint& path_point,
                         common::math::Vec2d* ptr_left_front) const;
  bool IsMergeArea(const double s,
                   const hdmap::LaneInfoConstPtr& ptr_left_neighbor_lane) const;
  bool IsNearMergeAreaWithNeighborLane(
      const double s, hdmap::LaneInfoConstPtr* ptr_left_neighbor_lane) const;
  bool IsNearMergeAreaWithLaneAttributes(
      const double s, hdmap::LaneInfoConstPtr* ptr_left_neighbor_lane) const;
  bool IsAbleToStop(const double stop_s, double* extend_s);
  double FindStopS(const std::vector<common::PathPoint>& path_points,
                   const hdmap::LaneInfoConstPtr& ptr_left_neighbor_lane,
                   double lat_buf, double* extend_l);
  bool FindStopSForLaneBorrow(double* stop_s, double* extend_l);
  bool IsChangingToLeftLaneByDiffHeading();
  void RadicalChangeLaneInLaneBorrow(PathDecision* const path_decision);
  void RadicalChangeLaneInMergeArea(PathDecision* const path_decision);
  void SlowBreakingForAbnormalPrediction();
  void SlowBreakingOnIntersection();
  void SlowBreakingReachTurn();
  void UpdateSlowBreakingInLowSpeed();
  void SlowBreakingForApproachingObs();
  void SlowBreakingForInitCollisionInFastCutin();
  void BuildStopWallForDecisionInSlowBreaking(Frame* const frame);
  bool IsGoStraight();
  void SlowBreakingForLargeTtc();
  bool IsVulnerableGroups(const Obstacle& ptr_obstacle);

  void GetBoundaries(std::vector<const STBoundary*>* boundaries);

  void GetMonitorObstacles(std::vector<std::string>* range_obs_ids);
  void GetSlowBreakingAccBasedLateralDistanceForApproachingObs(
      const planning::SLBoundary& adc_sl, const planning::SLBoundary& obs_sl,
      double obstacle_lon_speed, double adc_speed, double* decel_temp);
  void GetSlowBreakingAccBasedLateralDistanceForApproachingObsNew(
      const Obstacle& obstacle, const planning::SLBoundary& adc_sl,
      const planning::SLBoundary& obs_sl, double obstacle_lon_speed,
      double adc_speed, double* decel_temp);
  void GetSlowBreakingAccBasedLateralDistanceForLowRoadRight(
      const double diff_latetal_distance, double* decel);
  bool CheckSlowBreakingForObsInForLowRoadRight(
      double obstacle_lon_speed, double obstacle_lateral_speed,
      const planning::SLBoundary& adc_sl, const planning::SLBoundary& obs_sl,
      bool is_same_direction, bool is_face_path,
      const double diff_latetal_distance, Obstacle* ptr_obstacle,
      double* decel);
  void GetSlowBreakingAccBasedSpeedDiff(bool is_front_obs,
                                        double obstacle_lon_speed,
                                        double adc_speed, double* decel_temp);
  void GetSlowBreakingAccBasedSpeedDiffNew(const Obstacle& obstacle,
                                           bool is_front_obs,
                                           double obstacle_lon_speed,
                                           double adc_speed,
                                           double* decel_temp);
  void GetObsLateralAndLonSpeed(const Obstacle& obstacle,
                                const planning::SLBoundary& adc_sl,
                                double* obstacle_lon_speed,
                                double* obstacle_lateral_speed);
  bool IsBackSideObs(const planning::SLBoundary& adc_sl,
                     const planning::SLBoundary& obs_sl);
  bool IsOverlapDangerousObs(const planning::SLBoundary& adc_sl,
                             const planning::SLBoundary& obs_sl,
                             bool is_same_direction, bool is_face_path);
  bool IsNearToSlowBreaking(double lateral_diff, double obstacle_lateral_speed,
                            const planning::SLBoundary& obs_sl,
                            const planning::SLBoundary& adc_sl,
                            bool is_need_to_turn, Obstacle* mutable_obstacle);
  bool IsNeedToUpdateSlowBreakingInLowSpeed();
  bool IsCanSlowBreakingForAbnormalPrediction();
  bool IsOverlapAndAwayFromPath(bool is_same_direction, bool is_face_path,
                                const planning::SLBoundary& obs_sl,
                                const planning::SLBoundary& adc_sl);
  void UpdateDpDataInlowSpeed(const double& min_stop_decel,
                              const double& adc_v);
  bool NoConsiderObs(const planning::SLBoundary& obs_sl,
                     const planning::SLBoundary& adc_sl,
                     Obstacle* mutable_obstacle);
  void GetSlowBreakingDecel(const planning::SLBoundary& obs_sl,
                            const planning::SLBoundary& adc_sl,
                            double obstacle_lon_speed, double lateral_diff,
                            double* decel_temp);
  void GetSlowBreakingDecelForFrontObs(const Obstacle& obstacle,
                                       const planning::SLBoundary& obs_sl,
                                       const planning::SLBoundary& adc_sl,
                                       double obstacle_lon_speed,
                                       double lateral_diff, double* decel_temp);
  bool MakeIgnoreDecision(const std::string obs_id,
                          const std::string decision_tag,
                          PathDecision* const path_decision);
  bool IsAdcExceedBackObs(const Obstacle& obstacle,
                          double obstacle_lateral_speed);
  void CreatStopWallForReverseObs(Frame* const frame);
  bool EnableSlowBreakingForCutin();
  void UpdateDecel(double decel);
  void GetDecelInLowSpeedUseLateralDiff(const double lateral_diff,
                                        double* tem_decel);
  bool GetTargetSpeedForApproachingObs(const Obstacle* approaching_obstacle,
                                       const SLBoundary& adc_sl,
                                       double* obs_target_speed);
  void RecordToLearningData(Frame* const frame);
  void GetStaticObs(double diff_lateral, const SLBoundary& adc_sl_boundary,
                    const SLBoundary& obs_sl, bool is_unknown,
                    const Obstacle& obstacle,
                    std::array<std::vector<std::string>, 20>* consider_obs);
  void GetDynamicObs(double diff_lateral, const SLBoundary& adc_sl_boundary,
                     const SLBoundary& obs_sl, bool is_unknown,
                     const Obstacle& obstacle,
                     std::array<std::vector<std::string>, 20>* consider_obs);
  void AddFeature(double time_stamp,
                  const std::array<std::vector<std::string>, 20>& consider_obs);
  double GetReachTime(double diff_lateral, const SLBoundary& adc_sl_boundary,
                      const SLBoundary& obs_sl, const Obstacle& obstacle);
  void GetTrainingData();
  void GetSVMPredictionData(Frame* const frame);
  void GetLastTimeStamp(double* last_time_stamp);
  bool IsSlowBreakingObs(bool is_in_commonjunction, bool is_same_direction,
                         bool at_left_turn, bool is_face_path,
                         bool is_right_obs);
  bool IsFaceToPath(const Obstacle& obstacle, double obstacle_lateral_speed,
                    const planning::SLBoundary& obs_sl,
                    const planning::SLBoundary& adc_sl);
  bool IsSlowBreakingApproachingObs(const bool is_too_close,
                                    const bool is_face_to_path,
                                    const bool is_near_to_path_loose_constraint,
                                    const bool is_in_slow_breaking_scope,
                                    const bool is_near_to_path);
  void IsConsiderAwayFromPathObs(const planning::SLBoundary& obs_sl,
                                 const planning::SLBoundary& adc_sl,
                                 const double obstacle_lateral_speed,
                                 bool* is_near_to_path);
  size_t ClearObsoleteElements(DangerousObsContainer* container,
                               size_t capacity);
  bool IsInLongitudinalRange(const planning::SLBoundary& obs_sl,
                             const planning::SLBoundary& adc_sl);
  bool IsSlowBreakingForBackObs(const bool is_direct_turn,
                                const bool is_in_commonjunction,
                                const bool at_left_turn,
                                const bool at_right_turn,
                                const planning::SLBoundary& obs_sl,
                                const planning::SLBoundary& adc_sl,
                                Obstacle* ptr_obstacle, double* decel);

 private:
  SpeedBoundsDeciderConfig speed_bounds_config_;
  SpeedLimitDeciderConfig speed_limit_config_;
  std::vector<std::string> slow_breaking_obs_ids_;
  planning::SvmFeatureData svm_feature_data_;
  static DangerousObsContainer obs_container_;
  static DangerousObsContainer vehicle_container_;
  static DangerousObsForSpeedLimitContainer speed_limit_obs_container_;
  static size_t start_up_count_;
  static size_t start_up_extra_times_for_slower_;
  bool is_adc_start_up_ = false;
  cv::Ptr<cv::ml::SVM> model_;
  bool is_near_merge_area_ = false;
};

}  // namespace planning
}  // namespace century
