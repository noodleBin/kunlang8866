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

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/canbus/proto/chassis.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/status/status.h"
#include "modules/common/util/normal_util.h"
#include "modules/planning/common/smoothers/bspline.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/math/curve_math.h"
#include "modules/planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"
#include "modules/planning/tasks/utils/openspace_util.h"
#include "modules/planning/open_space/openspace_common/openspace_common.h"
#include "modules/planning/tasks/deciders/teb_planner_decider/teb_pre_observation_decider/teb_tar_fsm_common.h"
#include "modules/planning/tasks/optimizers/trajectory_optimizer.h"

namespace century {
namespace planning {
class PathPoint2D {
 public:
  PathPoint2D(double x, double y) : x_(x), y_(y) {}
  bool operator!=(const PathPoint2D& other) const {
    return (std::fabs(x_ - other.x_) >
            std::numeric_limits<double>::epsilon()) ||
           (std::fabs(y_ - other.y_) > std::numeric_limits<double>::epsilon());
  }
  bool operator==(const PathPoint2D& other) const {
    return (std::fabs(x_ - other.x_) <=
            std::numeric_limits<double>::epsilon()) &&
           (std::fabs(y_ - other.y_) <= std::numeric_limits<double>::epsilon());
  }
  void SetX(double value) { x_ = value; }
  void SetY(double value) { y_ = value; }
  const double GetConstX() const { return x_; }
  const double GetConstY() const { return y_; }
  double GetX() { return x_; }
  double GetY() { return y_; }

 private:
  double x_;
  double y_;
};
class TEBTrajectoryPartition : public TrajectoryOptimizer {
 public:
  TEBTrajectoryPartition(const TaskConfig& config,
                         const std::shared_ptr<DependencyInjector>& injector);

  ~TEBTrajectoryPartition() = default;

  void Restart();

 private:
  common::Status Process() override;

  void InterpolateTrajectory(
      const DiscretizedTrajectory& stitched_trajectory_result,
      DiscretizedTrajectory* interpolated_trajectory);

  Eigen::VectorXd Polyfit(const std::vector<PathPoint2D>& in_point, int n);
  bool PolyfitTrajectory(const bool is_reverse, const double step,
                         DiscretizedTrajectory* polyfit_trajectory_ptr,
                         DiscretizedTrajectory** current_trajectory_ptr);

  void FillTrajectoryTime(const double& start_time, const double& end_time,
                          DiscretizedTrajectory* trajectory);

  void InterpolateCurrentTrajectory(
      const double step, DiscretizedTrajectory* interpolated_trajectory_ptr,
      DiscretizedTrajectory** current_trajectory_ptr);

  void UpdateVehicleInfo();

  bool EncodeTrajectory(const DiscretizedTrajectory& trajectory,
                        std::string* const encoding);

  bool CheckTrajTraversed(const std::string& trajectory_encoding_to_check);

  void UpdateTrajHistory(const std::string& chosen_trajectory_encoding);

  void PartitionTrajectory(const DiscretizedTrajectory& trajectory,
                           std::vector<TrajGearPair>* partitioned_trajectories);

  void LoadTrajectoryPoint(const common::TrajectoryPoint& trajectory_point,
                           const bool is_trajectory_last_point,
                           const canbus::Chassis::GearPosition& gear,
                           common::math::Vec2d* last_pos_vec,
                           double* distance_s,
                           DiscretizedTrajectory* current_trajectory);

  bool CheckReachTrajectoryEnd(const DiscretizedTrajectory& trajectory,
                               const canbus::Chassis::GearPosition& gear,
                               const size_t trajectories_size,
                               const size_t trajectories_index,
                               size_t* current_trajectory_index,
                               size_t* current_trajectory_point_index);

  bool NewCheckReachTrajectoryEnd(const DiscretizedTrajectory& trajectory,
                                  const canbus::Chassis::GearPosition& gear,
                                  const size_t trajectories_size,
                                  const size_t trajectories_index,
                                  size_t* current_trajectory_index,
                                  size_t* current_trajectory_point_index);

  bool CalRemainDis(const DiscretizedTrajectory& current_trajectory,
                    const common::math::Vec2d adc_position,
                    size_t& nearest_index, double& remain_dis);

  bool CalSwitchToNextTrace(const DiscretizedTrajectory& current_trajectory,
                            const canbus::Chassis::GearPosition& current_gear,
                            const common::VehicleStateProvider* vehicle_state);

  bool UseFailSafeSearch(
      const std::vector<TrajGearPair>& partitioned_trajectories,
      const std::vector<std::string>& trajectories_encodings,
      size_t* current_trajectory_index, size_t* current_trajectory_point_index);

  bool InsertGearShiftTrajectory(
      const bool flag_change_to_next, const size_t current_trajectory_index,
      const std::vector<TrajGearPair>& partitioned_trajectories,
      TrajGearPair* gear_switch_idle_time_trajectory);

  void GenerateGearShiftTrajectory(
      const canbus::Chassis::GearPosition& gear_position,
      TrajGearPair* gear_switch_idle_time_trajectory);

  void NewAdjustRelativeTimeAndS(
      const std::vector<TrajGearPair>& partitioned_trajectories,
      const size_t current_trajectory_index,
      const size_t closest_trajectory_point_index,
      DiscretizedTrajectory* stitched_trajectory_result,
      TrajGearPair* current_partitioned_trajectory);

  bool CheckFinishInitPosition(
      const std::vector<TrajGearPair>& partitioned_trajectories,
      TrajGearPair* current_partitioned_trajectory);

  bool NewCheckFinishInitPosition(
      const std::vector<TrajGearPair>& partitioned_trajectories,
      TrajGearPair* current_partitioned_trajectory);

  Vec2d ComputeProjection(const Vec2d& a, const Vec2d& b, const Vec2d& p);

  double ComputeSideLonDistanceToProjection(const Vec2d& a, const Vec2d& b,
                                            const Vec2d& projection_point);

  double GetObstacleDist(const common::math::Vec2d& point,
                         const double& heading);

  double CalFrenetSpeedSlowDown(
      century::planning::DiscretizedTrajectory* trajectory, const size_t& i,
      const double& adc_s, century::canbus::Chassis::GearPosition* const gear,
      century::common::math::Vec2d* path_point_vec, const double path_point_theta,
      bool* is_need_speed_limit);

  double CalTraceCurveSpeed(century::planning::DiscretizedTrajectory* trajectory,
                            const size_t& i);

  double CalNearEndDisSpeed(century::planning::DiscretizedTrajectory* trajectory,
                            const size_t& i, const double& s_shift);

  double GetFluObsToVehicleSpeed(const common::math::Vec2d& vehicle_point,
                                 const double& vehicle_heading);

  void BsplineSmooth(const bool is_reverse, DiscretizedTrajectory* trajectory);
  void CalCurve(const bool is_reverse, DiscretizedTrajectory* trajectory);
  void CalTheta(const bool is_reverse, DiscretizedTrajectory* trajectory);
  void PullOverRoutingEndExtendLength(
      const size_t& current_trajectory_index,
      const size_t& partitioned_trajectories_size, const bool& is_reverse,
      DiscretizedTrajectory* trajectory);
  void RoutingEndExtendLength(const bool is_reverse,
                              DiscretizedTrajectory* trajectory);
  void TraceInterpolatePolyfitSmooth(const bool& is_reverse,
                                     DiscretizedTrajectory* trajectory);
  bool JudgeStopTrace(const DiscretizedTrajectory* trajectory);

  void OptimizeTrajectory(
      const std::vector<TrajGearPair>& partitioned_trajectories,
      const size_t current_trajectory_index,
      const size_t closest_trajectory_point_index,
      DiscretizedTrajectory* unpartitioned_trajectory_result,
      TrajGearPair* current_partitioned_trajectory);
  void ShrunkTrajectory(common::TrajectoryPoint* start_point,
                        TrajGearPair* current_partitioned_trajectory,
                        DiscretizedTrajectory* trajectory);
  double GetTotalTime(double total_length, double adc_speed, double target_v,
                      double* begin_break_s,
                      std::vector<std::pair<double, double>>* ref_s_list,
                      bool* is_get_total_time);
  double GetTotalTimeForNoComfortStop(
      double total_length, double adc_speed, double target_v,
      double* begin_break_s,
      std::vector<std::pair<double, double>>* ref_s_list);
  double GetTotalTimeForUniformSpeedComfortStop(
      double total_length, double adc_speed, double target_v,
      double comfort_decel, double comfort_decel_distance,
      double* begin_break_s,
      std::vector<std::pair<double, double>>* ref_s_list);
  double GetTotalTimeForACCAndUniformSpeedAndComfortStop(
      double s_rest, double t_rampup, double total_length, double adc_speed,
      double target_v, double comfort_decel, double comfort_acc,
      double comfort_decel_distance, double* begin_break_s,
      std::vector<std::pair<double, double>>* ref_s_list);
  double GetTotalTimeForACCAndComfortStop(
      double s_rest, double t_rampup, double total_length, double adc_speed,
      double target_v, double comfort_decel, double comfort_acc,
      double comfort_decel_distance, double* begin_break_s,
      std::vector<std::pair<double, double>>* ref_s_list);
  void OptimizeReproscess(double delta_t,
                          PiecewiseJerkSpeedProblem* piecewise_jerk_problem,
                          int num_of_knots, DiscretizedTrajectory* trajectory);

  bool CalcuNeastTrajectoryPoint(const Obstacle& obstacle,
                                 const DiscretizedTrajectory& trajectory,
                                 const bool is_reverse_traj,
                                 bool* is_vehicle_box_valid,
                                 std::vector<common::math::Box2d>* vehicle_boxs,
                                 std::pair<size_t, double>* neast_traj_info);
  void CalcuSpeedLimit(const TrajGearPair& current_partitioned_trajectory);

  double GetObstacleAndPathPointDistance(const Obstacle& obstacle,
                                         const common::PathPoint& path_point);

  void ReconstructSpeedLimitByKinematic(
      const double min_v_upper_bound, double init_speed, double init_acc,
      std::vector<std::pair<double, double>>* s_dot_bounds);

  void SetProblemStatus(const int num_of_knots, const double adc_speed,
                        const double total_length,
                        PiecewiseJerkSpeedProblem* piecewise_jerk_problem);
  bool GetNewStopS(const std::vector<const Obstacle*>& obstacles,
                   const size_t* current_index,
                   DiscretizedTrajectory* trajectory, double* stop_s);
  bool CheckObstacleCollision(const double relative_s,
                              const common::math::Box2d& ego_box,
                              const std::vector<const Obstacle*>& obstacles);
  void ConvertCostmap();
  void UpdateTrajectory(double stop_s, DiscretizedTrajectory* trajectory);
  bool GetStartPoint(TrajGearPair* current_partitioned_trajectory,
                     common::TrajectoryPoint* start_point);
  void CreateStopTrajectory(DiscretizedTrajectory* trajectory);
  void CalculateReverseTrajectory(
      const std::vector<TrajGearPair>& partitioned_trajectories,
      const size_t current_trajectory_index,
      const size_t closest_trajectory_point_index,
      DiscretizedTrajectory* unpartitioned_trajectory_result,
      TrajGearPair* current_partitioned_trajectory);
  bool GetNewStopSForReverse(const std::vector<const Obstacle*>& obstacles,
                             const size_t* current_index,
                             DiscretizedTrajectory* trajectory, double* stop_s);
  void UpdateTrajectoryForReverse(double stop_s,
                                  DiscretizedTrajectory* trajectory);
  double GetTotalTimeForReverse(
      double total_length, double adc_speed, double target_v,
      double* begin_break_s, std::vector<std::pair<double, double>>* ref_s_list,
      std::vector<std::pair<double, double>>* ref_v_list,
      bool* is_get_total_time);
  void OptimizeReproscessForReverse(
      double delta_t, const std::vector<std::pair<double, double>>& ref_s_list,
      int num_of_knots, DiscretizedTrajectory* trajectory);
  void ShrunkTrajectoryForReverse(common::TrajectoryPoint* start_point,
                                  TrajGearPair* current_partitioned_trajectory,
                                  DiscretizedTrajectory* trajectory);
  void IsDepartureTrajectory(
      century::planning::DiscretizedTrajectory* trajectory);
  void CollisionCheckForTebSpeed(const size_t current_trajectory_index,
                                 DiscretizedTrajectory* trajectory,
                                 bool* is_collision);
  bool StartupForReachDestination(bool is_collision,
                                  DiscretizedTrajectory* trajectory);
  void CheckTrajectory(DiscretizedTrajectory* trajectory);
  void CheckBackupPoints(std::vector<common::TrajectoryPoint> backup_points,
                         DiscretizedTrajectory* trajectory);
  void GetStopBackupPoints(std::vector<common::TrajectoryPoint>* backup_points);
  void SetSpeedLimitForQp(
      int num_of_knots, bool is_get_total_time, double total_length,
      double adc_speed,
      const std::vector<std::pair<double, double>>& ref_s_list,
      const std::array<double, 3>& init_s, std::vector<double>* x_ref,
      std::vector<std::pair<double, double>>* s_dot_bounds,
      PiecewiseJerkSpeedProblem* piecewise_jerk_problem);
  void AddLastPoint(const common::TrajectoryPoint& back_point,
                    DiscretizedTrajectory* trajectory);

 private:
  std::shared_ptr<OpenspaceUtil> openspace_util_ =
      std::make_shared<OpenspaceUtil>();
  std::shared_ptr<OpenspaceCommon> openspace_common_ =
      std::make_shared<OpenspaceCommon>();
  std::vector<common::math::Polygon2d> costmap_polygons_;
  const TEBTrajectoryPartitionConfig& teb_trajectory_partition_config_;
  double heading_search_range_ = 0.0;
  double heading_track_range_ = 0.0;
  double distance_search_range_ = 0.0;
  double heading_offset_to_midpoint_ = 0.0;
  double lateral_offset_to_midpoint_ = 0.0;
  double longitudinal_offset_to_midpoint_ = 0.0;
  double vehicle_box_iou_threshold_to_midpoint_ = 0.0;
  double linear_velocity_threshold_on_ego_ = 0.0;
  double up_dist_threshold_on_ego_ = 2.5;
  double down_dist_threshold_on_ego_ = 0.1;
  double slow_down_min_speed_ = 0.2;
  bool use_new_trajectory_partition_ = false;
  bool use_new_trajectory_speed_cal_ = false;
  double switch_trace_thr_s_ = 0.2;
  double switch_trace_thr_dis_ = 0.3;
  double teb_trajectory_near_end_ = 2.0;
  double teb_max_speed_limit_ = 2.0;
  double teb_min_speed_limit_ = 0.5;
  double teb_min_curve_thr_ = 0.06;
  double teb_max_curve_thr_ = 0.26;
  double teb_max_curve_speed_limit_ = 1.0;
  double flu_up_dist_threshold_on_ego_ = 1.0;

  std::vector<std::pair<double, double>> ref_v_list_;

  common::VehicleParam vehicle_param_;
  double front_to_center_ = 0.0;
  double back_to_center_ = 0.0;
  double ego_length_ = 0.0;
  double ego_width_ = 0.0;
  double shift_distance_ = 0.0;

  double ego_theta_ = 0.0;
  double ego_x_ = 0.0;
  double ego_y_ = 0.0;
  double ego_v_ = 0.0;
  common::math::Box2d ego_box_;
  double vehicle_moving_direction_ = 0.0;
  std::unordered_map<std::string, std::pair<size_t, double>>
      obstacle_trajectory_info_;
  SpeedLimit speed_limit_;

  double pullover_end_heading_ = 0.0;

  std::vector<century::planning::TrajGearPair> last_partitioned_trajectories_;
  size_t last_current_trajectory_index_ = 0;
  size_t last_current_trajectory_point_index_ = 0;
  size_t last_trajectories_size_ = 0;
  bool replanning_ = false;
  bool replanned_ = false;
  size_t start_c_id_ = 0;
  size_t end_c_id_ = 0;
  bool is_null_traj_ = false;
  bool extent_trace_for_routing_end_ = false;
  bool is_stop_trajectory_ = false;
  struct pair_comp_ {
    bool operator()(
        const std::pair<std::pair<size_t, size_t>, double>& left,
        const std::pair<std::pair<size_t, size_t>, double>& right) const {
      return left.second <= right.second;
    }
  };
  struct comp_ {
    bool operator()(const std::pair<size_t, double>& left,
                    const std::pair<size_t, double>& right) {
      return left.second <= right.second;
    }
  };
};
}  // namespace planning
}  // namespace century
