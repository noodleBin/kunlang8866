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

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/proto/decision.pb.h"
#include "modules/planning/proto/sl_boundary.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/indexed_list.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/reference_line/reference_line.h"

namespace century {
namespace planning {

/**
 * @class Obstacle
 * @brief This is the class that associates an Obstacle with its path
 * properties. An obstacle's path properties relative to a path.
 * The `s` and `l` values are examples of path properties.
 * The decision of an obstacle is also associated with a path.
 *
 * The decisions have two categories: lateral decision and longitudinal
 * decision.
 * Lateral decision includes: nudge, ignore.
 * Lateral decision safety priority: nudge > ignore.
 * Longitudinal decision includes: stop, yield, follow, overtake, ignore.
 * Decision safety priorities order: stop > yield >= follow > overtake > ignore
 *
 * Ignore decision belongs to both lateral decision and longitudinal decision,
 * and it has the lowest priority.
 */
class Obstacle {
 public:
  Obstacle() = default;
  Obstacle(const std::string& id,
           const perception::PerceptionObstacle& perception_obstacle,
           const prediction::ObstaclePriority::Priority& obstacle_priority,
           const bool is_static, const double theta = 0.0);
  Obstacle(const std::string& id,
           const perception::PerceptionObstacle& perception_obstacle,
           const prediction::Trajectory& trajectory,
           const prediction::ObstaclePriority::Priority& obstacle_priority,
           const bool is_static, const double theta = 0.0);

  const std::string& Id() const { return id_; }
  void SetId(const std::string& id) { id_ = id; }

  double speed() const { return speed_; }
  double SpeedHeading() const { return speed_heading_; }
  double acceleration() const { return acceleration_; }

  int32_t PerceptionId() const { return perception_id_; }
  void SetPerceptionId(double perception_id) { perception_id_ = perception_id; }

  bool IsStatic() const { return is_static_; }
  bool IsVirtual() const { return is_virtual_; }
  bool IsCaution() const { return is_caution_; }
  void SetIsCaution(bool is_caution) { is_caution_ = is_caution; }
  bool IsCross() const { return is_cross_; }
  void SetIsCross(bool is_cross) { is_cross_ = is_cross; }
  bool IsModifiedVelocity() const { return is_modified_velocity_; }
  void SetIsModifiedVelocity(bool is_modified_velocity) {
    is_modified_velocity_ = is_modified_velocity;
  }
  double ModifiedVelocity() const { return modified_velocity_; }
  void SetModifiedVelocity(double modified_velocity) {
    modified_velocity_ = modified_velocity;
  }
  bool IsReverse() const { return is_reverse_; }
  void SetIsReverse(bool is_reverse) { is_reverse_ = is_reverse; }
  void SetPerceptionLateralSpeed(const double lateral_speed) {
    lateral_speed_ = lateral_speed;
  }
  void SetPerceptionLongitudinalSpeed(const double lon_speed) {
    lon_speed_ = lon_speed;
  }
  double LateralSpeed() const { return lateral_speed_; }
  double LongitudinalSpeed() const { return lon_speed_; }
  void SetPerceptionLongitudianlAcc(const double lon_acc) {
    lon_acc_ = lon_acc;
  }
  double LongitudianlAcc() const { return lon_acc_; }

  common::TrajectoryPoint GetPointAtTime(const double time) const;

  common::math::Box2d GetBoundingBox(
      const common::TrajectoryPoint& point) const;

  common::math::Polygon2d GetPolygon(
      const common::TrajectoryPoint& point) const;

  const common::math::Box2d& PerceptionBoundingBox() const {
    return perception_bounding_box_;
  }
  const common::math::Polygon2d& PerceptionPolygon() const {
    return perception_polygon_;
  }
  common::math::Polygon2d& no_const_PerceptionPolygon() {
    return perception_polygon_;
  }
  const prediction::Trajectory& Trajectory() const { return trajectory_; }
  common::TrajectoryPoint* AddTrajectoryPoint() {
    return trajectory_.add_trajectory_point();
  }
  bool HasTrajectory() const {
    return !(trajectory_.trajectory_point().empty());
  }

  const perception::PerceptionObstacle& Perception() const {
    return perception_obstacle_;
  }

  /**
   * @brief This is a helper function that can create obstacles from prediction
   * data.  The original prediction may have multiple trajectories for each
   * obstacle. But this function will create one obstacle for each trajectory.
   * @param predictions The prediction results
   * @return obstacles The output obstacles saved in a list of unique_ptr.
   */
  static std::list<std::unique_ptr<Obstacle>> CreateObstacles(
      const prediction::PredictionObstacles& predictions);

  static std::unique_ptr<Obstacle> CreateStaticVirtualObstacles(
      const std::string& id, const common::math::Box2d& obstacle_box);
  static std::unique_ptr<Obstacle> CreateStaticObstacles(
    const std::string& id, const common::math::Box2d& obstacle_box);
  static std::unique_ptr<Obstacle> CreateStackerObstacles(
      const std::string& id, const common::math::Box2d& obstacle_box);
  static std::unique_ptr<Obstacle> CreateStackerObstaclesWithID(
      const std::string& id, const common::math::Box2d& obstacle_box,
      const double& speed);
  static std::unique_ptr<Obstacle> CreateWheelCraneObstacles(
      const std::string& id, const common::math::Box2d& obstacle_box);
  static bool IsValidPerceptionObstacle(
      const perception::PerceptionObstacle& obstacle);

  static bool IsValidTrajectoryPoint(const common::TrajectoryPoint& point);

  inline bool IsCautionLevelObstacle() const {
    return is_caution_level_obstacle_;
  }

  // const Obstacle* obstacle() const;

  /**
   * return the merged lateral decision
   * Lateral decision is one of {Nudge, Ignore}
   **/
  const ObjectDecisionType& LateralDecision() const;

  /**
   * @brief return the merged longitudinal decision
   * Longitudinal decision is one of {Stop, Yield, Follow, Overtake, Ignore}
   **/
  const ObjectDecisionType& LongitudinalDecision() const;

  std::string DebugString() const;

  const SLBoundary& PerceptionSLBoundary() const;

  const STBoundary& reference_line_st_boundary() const;

  const STBoundary& path_st_boundary() const;

  const std::vector<std::string>& decider_tags() const;

  const std::vector<ObjectDecisionType>& decisions() const;

  void AddLongitudinalDecision(const std::string& decider_tag,
                               const ObjectDecisionType& decision);

  void AddLateralDecision(const std::string& decider_tag,
                          const ObjectDecisionType& decision);
  bool HasLateralDecision() const;

  void set_path_st_boundary(const STBoundary& boundary);

  bool is_path_st_boundary_initialized() const {
    return path_st_boundary_initialized_;
  }

  void SetStBoundaryType(const STBoundary::BoundaryType type);

  void EraseStBoundary();

  void SetReferenceLineStBoundary(const STBoundary& boundary);

  void SetReferenceLineStBoundaryType(const STBoundary::BoundaryType type);

  void EraseReferenceLineStBoundary();

  bool HasLongitudinalDecision() const;

  bool HasNonIgnoreDecision() const;

  void clear_yield_decision() {
    if (HasLongitudinalDecision() && longitudinal_decision_.has_yield()) {
      longitudinal_decision_.clear_yield();
    }
  }

  /**
   * @brief Calculate stop distance with the obstacle using the ADC's minimum
   * turning radius
   */
  double MinRadiusStopDistance(const common::VehicleParam& vehicle_param,
                               const double adc_l) const;

  /**
   * @brief Check if this object can be safely ignored.
   * The object will be ignored if the lateral decision is ignore and the
   * longitudinal decision is ignore
   *  return longitudinal_decision_ == ignore && lateral_decision == ignore.
   */
  bool IsIgnore() const;
  bool IsLongitudinalIgnore() const;
  bool IsLateralIgnore() const;

  void BuildReferenceLineStBoundary(const ReferenceLine& reference_line,
                                    const double adc_start_s);

  void SetPerceptionSlBoundary(const SLBoundary& sl_boundary);

  /**
   * @brief check if an ObjectDecisionType is a longitudinal decision.
   **/
  static bool IsLongitudinalDecision(const ObjectDecisionType& decision);

  /**
   * @brief check if an ObjectDecisionType is a lateral decision.
   **/
  static bool IsLateralDecision(const ObjectDecisionType& decision);

  void SetBlockingObstacle(bool blocking) { is_blocking_obstacle_ = blocking; }
  bool IsBlockingObstacle() const { return is_blocking_obstacle_; }

  void SetSlowBreakingObstacle(bool slow_breaking) {
    is_slow_breaking_obstacle_ = slow_breaking;
  }
  bool IsSlowBreakingObstacle() const { return is_slow_breaking_obstacle_; }

  void SetSlowBreakingTag(std::string slow_breaking_tag) {
    slow_breaking_tag_ = slow_breaking_tag;
  }
  const std::string& SlowBreakingTag() const { return slow_breaking_tag_; }

  /*
   * @brief IsLaneBlocking is only meaningful when IsStatic() == true.
   */
  bool IsLaneBlocking() const { return is_lane_blocking_; }
  void CheckLaneBlocking(const ReferenceLine& reference_line);
  bool IsLaneChangeBlocking() const { return is_lane_change_blocking_; }
  void SetLaneChangeBlocking(const bool is_distance_clear);
  const std::pair<double, double>& GetLaneWidthBaseOnCenter(
      const ReferenceLine& reference_line);

  const bool IsCanPass() const { return is_can_pass_; }
  void SetCanPass(const bool is_can_pass) { is_can_pass_ = is_can_pass; }

  const bool IsIgv() const { return is_igv_; }
  void SetIsIgv(const bool is_igv) { is_igv_ = is_igv; }
  const std::string GetIgvVehicleId() const { return igv_vehicle_id_; }
  void SetIgvVehicleId(const std::string& igv_vehicle_id) {
    igv_vehicle_id_ = igv_vehicle_id;
  }
  const bool IsHigherObs() const { return is_higher_obs_; }
  void SetIsHigherObs(const bool is_higher_obs) { is_higher_obs_ = is_higher_obs; }
  const bool IsSlowCanPass() const { return is_slow_can_pass_; }
  void SetSlowCanPass(const bool is_slow_can_pass) {
    is_slow_can_pass_ = is_slow_can_pass;
  }

  const bool IsNeedShift() const { return is_need_shift_; }
  void SetNeedShift(const bool is_need_shift) {
    is_need_shift_ = is_need_shift;
  }

  bool IsStaticToDynamic() const { return is_static_to_dynamic_; }
  void SetStaticToDynamic(const bool static_to_dynamic) {
    is_static_to_dynamic_ = static_to_dynamic;
  }

  double HeadingDiffWithLane(const ReferenceLine& reference_line) const;
  bool IsLargeVehicle(const double check_length) const;
  bool IsSmallUnknown(const double check_width) const;
  bool GetStartUpState() const { return is_start_up_; }
  bool GetSidepassState() const { return is_safe_sidepass_; }

  void SetStartUpState(bool start_up_state) { is_start_up_ = start_up_state; }
  void SetSidepassState(bool sidepass_state) {
    is_safe_sidepass_ = sidepass_state;
  }

  void SetOutOfOpenSpaceROI(const bool out_of_openspace_roi) {
    is_out_openspace_roi_ = out_of_openspace_roi;
  }
  bool IsOutOfOpenSpaceROI() const { return is_out_openspace_roi_; }

 private:
  FRIEND_TEST(MergeLongitudinalDecision, AllDecisions);
  static ObjectDecisionType MergeLongitudinalDecision(
      const ObjectDecisionType& lhs, const ObjectDecisionType& rhs);
  FRIEND_TEST(MergeLateralDecision, AllDecisions);
  static ObjectDecisionType MergeLateralDecision(const ObjectDecisionType& lhs,
                                                 const ObjectDecisionType& rhs);

  bool BuildTrajectoryStBoundary(const ReferenceLine& reference_line,
                                 const double adc_start_s,
                                 STBoundary* const st_boundary);
  bool BuildTrajectoryStPolygonPoints(
      const ReferenceLine& reference_line, const double adc_start_s,
      std::vector<std::pair<STPoint, STPoint>>* const polygon_points);
  bool BuildStPolygonPointsFromSLBoundary(
      const ReferenceLine& reference_line, const double adc_start_s,
      const common::TrajectoryPoint& first_traj_point,
      const common::TrajectoryPoint& second_traj_point,
      const common::math::Box2d& object_moving_box,
      const SLBoundary& object_boundary,
      std::vector<std::pair<STPoint, STPoint>>* const polygon_points);

  bool IsValidObstacle(
      const perception::PerceptionObstacle& perception_obstacle);
  void CalculateSpeedHeading();

 private:
  std::string id_;
  int32_t perception_id_ = 0;
  bool is_static_ = false;
  bool is_virtual_ = false;
  bool is_caution_ = false;
  bool is_cross_ = false;
  bool is_modified_velocity_ = false;
  double modified_velocity_ = 0.0;
  bool is_reverse_ = false;
  double speed_heading_ = 0.0;  // only being set when obstacle has speed.
  double speed_ = 0.0;
  double acceleration_ = 0.0;

  common::VehicleParam adc_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double adc_length_ = adc_param_.length();
  double adc_half_length_ = adc_length_ * 0.5;
  double adc_width_ = adc_param_.width();

  bool path_st_boundary_initialized_ = false;

  prediction::Trajectory trajectory_;
  perception::PerceptionObstacle perception_obstacle_;
  common::math::Box2d perception_bounding_box_;
  common::math::Polygon2d perception_polygon_;

  std::vector<ObjectDecisionType> decisions_;
  std::vector<std::string> decider_tags_;
  SLBoundary sl_boundary_;
  double lateral_speed_ = 0.0;
  double lon_speed_ = 0.0;
  double lon_acc_ = 0.0;

  STBoundary reference_line_st_boundary_;
  STBoundary path_st_boundary_;

  ObjectDecisionType lateral_decision_;
  ObjectDecisionType longitudinal_decision_;

  // for keep_clear usage only
  bool is_blocking_obstacle_ = false;

  bool is_lane_blocking_ = false;

  bool is_lane_change_blocking_ = false;

  bool is_caution_level_obstacle_ = false;

  double min_radius_stop_distance_ = -1.0;

  bool is_can_pass_ = true;
  bool is_igv_ = false;
  std::string igv_vehicle_id_;
  bool is_higher_obs_ = false;
  bool is_slow_can_pass_ = false;

  bool is_static_to_dynamic_ = false;

  bool is_need_shift_ = true;

  bool is_slow_breaking_obstacle_ = false;

  bool is_start_up_ = false;
  bool is_safe_sidepass_ = false;
  bool is_out_openspace_roi_ = false;

  std::string slow_breaking_tag_ = "no";

  struct ObjectTagCaseHash {
    size_t operator()(
        const planning::ObjectDecisionType::ObjectTagCase tag) const {
      return static_cast<size_t>(tag);
    }
  };  // namespace planning

  /**
   * @brief land width in (obstacle_start_s + obstacle_end_s)*0.5
   * first is left width, second is right width
   */
  std::pair<double, double> lane_width_base_on_obs_center_ = {0.0, 0.0};
  bool has_set_lane_width_ = false;

  static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                                  ObjectTagCaseHash>
      s_lateral_decision_safety_sorter_;
  static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                                  ObjectTagCaseHash>
      s_longitudinal_decision_safety_sorter_;
};  // namespace century

typedef IndexedList<std::string, Obstacle> IndexedObstacles;
typedef ThreadSafeIndexedList<std::string, Obstacle> ThreadSafeIndexedObstacles;

}  // namespace planning
}  // namespace century
