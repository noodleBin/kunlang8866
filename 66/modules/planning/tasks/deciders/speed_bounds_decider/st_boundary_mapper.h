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
 *   @file
 **/

#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/task_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/reference_line/reference_line.h"

namespace century {
namespace planning {

class STBoundaryMapper {
 private:
  enum OverlapStatus { NEED_CONTINUE = 0, NEED_BREAK, FIND_OVERLAP };
  bool IsFindOvelap(OverlapStatus stat) const { return FIND_OVERLAP == stat; }

 public:
  STBoundaryMapper(const SLBoundary& adc_sl_boundary,
                   const SpeedBoundsDeciderConfig& config,
                   const ReferenceLine& reference_line,
                   const PathData& path_data, const SpeedLimit& speed_limit,
                   const double planning_distance, const double planning_time,
                   const std::shared_ptr<DependencyInjector>& injector,
                   ReferenceLineInfo* reference_line_info, Frame* const frame);

  virtual ~STBoundaryMapper() = default;

  common::Status ComputeSTBoundary(PathDecision* path_decision);

 private:
  FRIEND_TEST(StBoundaryMapperTest, check_overlap_test);
  FRIEND_TEST(StBoundaryMapperTest, get_overlap_boundary_points);

  common::Status PreCheckForComputeSTBoundary(PathDecision* path_decision);

  /** @brief Calls GetOverlapBoundaryPoints to get upper and lower points
   * for a given obstacle, and then formulate STBoundary based on that.
   * It also labels boundary type based on previously documented decisions.
   */
  void ComputeSTBoundary(Obstacle* obstacle);

  /** @brief Map the given obstacle onto the ST-Graph. The boundary is
   * represented as upper and lower points for every s of interests.
   * Note that upper_points.size() = lower_points.size()
   */
  bool GetOverlapBoundaryPoints(
      const std::vector<common::PathPoint>& path_points,
      const Obstacle& obstacle,
      std::vector<size_t>* begin_overlap_position_in_st_points,
      std::vector<size_t>* end_overlap_position_in_st_points,
      std::vector<STPoint>* upper_points, std::vector<STPoint>* lower_points);

  bool GetOverlapMultiBoundaryPoints(
      const std::vector<common::PathPoint>& path_points,
      const Obstacle& obstacle, std::vector<std::vector<STPoint>>* upper_points,
      std::vector<std::vector<STPoint>>* lower_points);

  /** @brief Given a path-point and an obstacle bounding box, check if the
   *        ADC, when at that path-point, will collide with the obstacle.
   * @param The path-point of the center of rear-axis for ADC.
   * @param The bounding box of the obstacle.
   * @param The extra lateral buffer for our ADC.
   */
  bool CheckOverlap(const common::PathPoint& path_point,
                    const common::math::Box2d& obs_box) const;

  /** @brief Given a path-point and an obstacle footprint, check if the
   *        ADC, when at that path-point, will collide with the obstacle.
   * @param The path-point of the center of rear-axis for ADC.
   * @param The footprint of the obstacle.
   * @param The extra lateral buffer for our ADC.
   */
  bool CheckOverlap(const common::PathPoint& path_point,
                    const common::math::Polygon2d& obs_polygon) const;
  bool CheckUnknownObsOverlap(const common::PathPoint& path_point,
                              const planning::SLBoundary& obs_sl_boundary,
                              const common::math::Polygon2d& obs_polygon) const;
  bool CheckStackerObsOverlap(const common::PathPoint& path_point,
                              const planning::SLBoundary& obs_sl_boundary,
                              const common::math::Polygon2d& obs_polygon) const;

  /** @brief Maps the closest STOP decision onto the ST-graph. This STOP
   * decision can be stopping for blocking obstacles, or can be due to
   * traffic rules, etc.
   */
  bool MapStopDecision(Obstacle* stop_obstacle,
                       const ObjectDecisionType& decision) const;

  /** @brief Fine-tune the boundary for yielding or overtaking obstacles.
   * Increase boundary on the s-dimension or set the boundary type, etc.,
   * when necessary.
   */
  void ComputeSTBoundaryWithDecision(Obstacle* obstacle,
                                     const ObjectDecisionType& decision);

  /**
   * @brief Get the closest stop decison.
   *
   * @param current_stop_obstacle current obstacle with a stop decision
   * @param current_stop_decision current stop decision
   * @param closest_stop_obstacle returned closest stop obstacle
   * @param closest_stop_decision returned closest stop decision
   * @param min_stop_s returned minimum stop distance
   */
  void GetClosestStopDecision(Obstacle* current_stop_obstacle,
                              const ObjectDecisionType& current_stop_decision,
                              Obstacle** const closest_stop_obstacle,
                              ObjectDecisionType* const closest_stop_decision,
                              double* const min_stop_s) const;
  common::Status MakeCloseStopObstacleDecision(
      const ObjectDecisionType& closest_stop_decision,
      Obstacle* const closest_stop_obstacle);

  void CalcuMaxCollisionPathLength();
  void CalcuLBufferForOverlapCheck(const Obstacle& obstacle);
  bool IsPathOnHighRight() const;
  bool IsNeedToTurn(const bool is_path_onlane);
  bool IsCanSkipObstacle(const Obstacle& obstacle,
                         const bool& is_adc_high_road_right_beginning) const;
  bool GetRoadRightRelativeObstacle(const Obstacle& obstacle,
                                    const bool is_left_side_obs,
                                    const bool is_right_side_obs,
                                    const bool is_high_right) const;
  bool IsSameDirection(const Obstacle& obstacle, double path_point_theta) const;
  bool JudgeCarInTrafficeLightInnerJunction(
      std::vector<hdmap::JunctionInfoConstPtr>* junctions,
      const common::math::Vec2d& dead_end_point,
      hdmap::JunctionInfoConstPtr* target_junction) const;

  OverlapStatus DetectOverlapByObstacleTrajectory(
      const Obstacle& obstacle, const common::TrajectoryPoint& trajectory_point,
      const bool is_need_break, STPoint* const lower_points,
      STPoint* const upper_points) const;
  void SetInnerJunctionStatus() const;
  void GetObstacleReverseAndLateralSpeed(const Obstacle& obstacle,
                                         const double adc_heading);
  bool GetCrossObstacleStateForOverlapCheck(const Obstacle& obstacle,
                                            bool* is_can_skip);
  bool GetReverseObstacleStateForOverlapCheck(const Obstacle& obstacle);
  void OverlapCheckForStaticObstacle(
      const std::vector<common::PathPoint>& path_points,
      const Obstacle& obstacle, std::vector<STPoint>* upper_points,
      std::vector<STPoint>* lower_points);
  void SubsamplePathPoint(const std::vector<common::PathPoint>& path_points);
  uint32_t GetObsStepForOverlapCheck(const Obstacle& obstacle);
  void RoughSearchOverlap(
      const Obstacle& obstacle, const uint32_t step, const bool is_need_break,
      std::vector<int>* const begin_overlap_index,
      std::vector<int>* const end_overlap_index,
      std::vector<size_t>* const begin_overlap_position_in_st_points,
      std::vector<size_t>* const end_overlap_position_in_st_points,
      std::vector<STPoint>* upper_points, std::vector<STPoint>* lower_points);
  void FineSearchOverlap(
      const Obstacle& obstacle, const uint32_t step, const bool is_need_break,
      const std::vector<int>& begin_overlap_index,
      const std::vector<int>& end_overlap_index,
      std::vector<size_t>* const begin_overlap_position_in_st_points,
      std::vector<size_t>* const end_overlap_position_in_st_points,
      std::vector<STPoint>* upper_points, std::vector<STPoint>* lower_points);
  Obstacle* AddReadyBoundaryObstacle(const Obstacle& obstacle, size_t idx);
  bool SelectStPoints(const std::string& obs_id,
                      std::vector<std::vector<STPoint>>* upper_points_list,
                      std::vector<std::vector<STPoint>>* lower_points_list,
                      std::vector<STPoint>* upper_points,
                      std::vector<STPoint>* lower_points);
  bool IsNeedToIgnoreObstacles(const Obstacle& obstacle,
                               const bool is_adc_high_road_right_beginning,
                               const bool is_high_right_in_intersection,
                               const bool is_need_to_turn);
  bool CheckHighRightOnIntersection(bool* at_right_merage_lane,
                                    bool* at_left_merage_lane,
                                    bool* is_direct_turn);
  bool IgnoreFrontHighSpeedObstacle(
      const Obstacle& obstacle,
      const util::MovingObstacleType moving_obstacle_type);
  double GetOvertakeSBuffer(const Obstacle& obstacle,
                            const bool is_minway_entry_boundary);
  OverlapStatus FoundFineOverlap(const Obstacle& obstacle,
                                 const common::math::Polygon2d& obs_polygon,
                                 const common::math::Box2d& obs_box,
                                 const double trajectory_point_time,
                                 const double path_s, const double step_length,
                                 STPoint* const ptr_lower_point,
                                 STPoint* const ptr_upper_point) const;
  void GetLaneHeading(const std::vector<hdmap::LaneSegment>& lane_segments,
                      const hdmap::LaneInfoConstPtr& adc_lane,
                      const int adc_lane_precessor_index,
                      const int adc_lane_sucessor_index,
                      double* precessor_lane_heading,
                      double* sucessor_lane_heading,
                      common::math::Vec2d* sucessor_lane_first_point,
                      common::math::Vec2d* sucessor_lane_second_point);
  bool IsReachTurnLane(bool* reach_right_lane, bool* reach_left_lane);
  void IsCanSkipForlinearTrajectory(const Obstacle& obstacle,
                                    bool* is_can_skip);
  // for radical decision
  bool IgnoreObstacleForRadicalDecision(const Obstacle& obstacle);
  void IsCanSkipForAdcExceedCollisionPoint(
      const Obstacle& obstacle, bool is_adc_in_common_junction,
      bool is_adc_exceed_cross_obs, bool is_face_to_path_obs, bool is_cross_obs,
      bool is_cross_path_relative_time, bool* is_can_skip);
  bool CheckRoadRightInTurn(const Obstacle& obstacle, bool is_left_side_obs,
                            bool is_right_side_obs, double hy_diff_heading,
                            bool is_high_right_of_way_compare_obs) const;
  void SkipObsForExceedCollisionPoint(Obstacle* obstacle);
  void SetRoadRightInfo(const bool is_high_right_in_intersection,
                        const bool at_left_merage_lane,
                        const bool at_right_merage_lane, bool* is_direct_turn,
                        bool* is_need_to_turn);
  void GetDangerousHighSpeedBicycle(PathDecision* path_decision);
  bool IsReachRightTurn(double* first_turn_s);
  bool GetStopS(const Obstacle& obstacle, double* first_stop_s);
  bool GetNeastStopS(PathDecision* path_decision,
                     std::vector<std::string> dangerous_high_speed_bicycle_ids,
                     double* first_stop_s);
  void StopForRightTurn(PathDecision* path_decision);
  void CanSkipObstaclesInLaneborrowReturn(PathDecision* path_decision);
  bool IsRerutnReferenceLine(const bool is_left_borrow,
                             const bool is_right_borrow);

 private:
  const SLBoundary& adc_sl_boundary_;
  const SpeedBoundsDeciderConfig& speed_bounds_config_;
  const ReferenceLine& reference_line_;
  const PathData& path_data_;
  DiscretizedPath subsample_discretized_path_;
  double max_collision_path_len_ = std::numeric_limits<double>::max();
  std::vector<Obstacle> ready_to_add_obstalces_;
  const SpeedLimit& speed_limit_;
  const common::VehicleParam& vehicle_param_;
  const double planning_max_distance_;
  const double planning_max_time_;
  double adc_lane_left_width_;
  double adc_lane_right_width_;
  bool path_turn_right_ = false;
  bool path_turn_left_ = false;
  bool at_right_turn_ = false;
  bool at_left_turn_ = false;
  std::shared_ptr<DependencyInjector> injector_;
  ReferenceLineInfo* reference_line_info_;
  double l_buffer_ = 0.0;
  double obs_reverse_speed_ = 0.0;
  double obs_lateral_speed_ = 0.0;
  double obs_lon_speed_ratio_ = 0.0;
  double obs_lat_speed_ratio_ = 0.0;
  bool is_vertical_obs_ = false;
  bool is_opposite_obs_ = false;
  bool at_turn_area_in_junction_ = false;
  bool at_left_turn_area_in_junction_ = false;
  bool at_right_turn_area_in_junction_ = false;
  double successor_lane_heading_diff_ = 0.0;
  double adc_lane_heading_diff_ = 0.0;
  double first_turn_point_s_ = 0.0;
  std::vector<std::string> exceed_collision_obs_ids_;
  std::vector<std::string> skip_high_speed_bicycle_ids_;
  std::vector<std::string> dangerous_high_speed_bicycle_ids_;
  std::vector<std::string> skip_back_laneborrow_obs_ids_;
  Frame* const frame_;
  bool is_in_merage_ = false;
  double diagonal_heading_ = 0.0;
};

}  // namespace planning
}  // namespace century
