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

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/monitor/proto/system_status.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "cyber/threadlib/threadlib.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/smoothers/smoother.h"
#include "modules/planning/planner/on_lane_planner_dispatcher.h"
#include "modules/planning/planning_base.h"
#include "modules/planning/open_space/openspace_common/openspace_common.h"

/**
 * @namespace century::planning
 * @brief century::planning
 */
namespace century {
namespace planning {
using century::cyber::Clock;
using century::cyber::ComponentBase;
enum FallBackStatus {
  REGULAR_TRAJECTORY = 0u,
  ROUTING_NOT_READY = 1u,
  NO_LAST_FRAME = 2u,
  NO_FRAME = 3u,
  NO_PATH_DATA = 4u,
  NO_SPEED_DATA = 5u,
  COMBINE_PATH_SPEED_FAILED = 6u,
  COMPLETED_FALL_BACK_TRACTORY = 7u,
  FALLBACK_SUCCESS = 8u,
};
/**
 * @class planning
 *
 * @brief Planning module main class. It processes GPS and IMU as input,
 * to generate planning info.
 */
class OnLanePlanning : public PlanningBase {
 public:
  explicit OnLanePlanning(const std::shared_ptr<DependencyInjector>& injector)
      : PlanningBase(injector) {
    planner_dispatcher_ = std::make_unique<OnLanePlannerDispatcher>();
  }
  virtual ~OnLanePlanning();

  /**
   * @brief Planning name.
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  common::Status Init(const PlanningConfig& config) override;

  void FallbackStop();
  /**
   * @brief main logic of the planning module, runs periodically triggered by
   * timer.
   */
  void RunOnce(const LocalView& local_view,
               ADCTrajectory* const ptr_trajectory_pb) override;

  common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const trajectory) override;

 private:
 void CheckStopForWheelcrane(ADCTrajectory* const ptr_trajectory_pb) ;
  void CheckEnableBorrow(ADCTrajectory* const ptr_trajectory_pb);
  void CheckOutRouting(ADCTrajectory* const ptr_trajectory_pb);
  void CheckIsNeedToReroutingForBlock();
  void CheckUseReverseTrajectory(
      const century::common::TrajectoryPoint& start_point,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const ptr_trajectory_pb);
  void CheckNeedToShrinkCollisionBuffer(ADCTrajectory* ptr_trajectory_pb);
  double GetReverseTrajectoryDistance(
      const century::common::TrajectoryPoint& start_point,
      planning::ReferenceLineInfo* best_ref_info,
      ADCTrajectory* const ptr_trajectory_pb);
  void GetReverseSpeedData(planning::ReferenceLineInfo* best_ref_info,
                          ADCTrajectory* const ptr_trajectory_pb,
                          double distance_to_stop_point);
  void GenerateReversePathData(
      const century::common::TrajectoryPoint& start_point,
      planning::ReferenceLineInfo* best_ref_info,
      double* distance_to_stop_point);
  double GetNewReverseDistance(planning::ReferenceLineInfo* best_ref_info);
  common::Status IsAdcInRoad(ReferenceLineInfo* reference_line_info);
  void IsAdcInCommonJunction(ReferenceLineInfo* reference_line_info);
  common::Status IsAdcDeviateLaneDirection(
      const ReferenceLineInfo* reference_line_info,
      ADCTrajectory* const ptr_trajectory_pb);
  bool IsJunctionContainAdc(const common::VehicleState& vehicle_state,
                            const hdmap::JunctionInfo& junction_info,
                            bool use_half_width = false) const;
  bool IsGateJunctionContainAdc() const;
  bool IsVehInSpecificJunction(int junction_type);
  bool IsExpresswayJunctionWithoutLaneChange();
  common::Status InitFrame(const uint32_t sequence_num,
                           const common::TrajectoryPoint& planning_start_point,
                           const common::VehicleState& vehicle_state);

  common::VehicleState AlignTimeStamp(const common::VehicleState& vehicle_state,
                                      const double curr_timestamp) const;

  void ExportReferenceLineDebug(planning_internal::Debug* debug);
  bool CheckPlanningConfig(const PlanningConfig& config);
  bool HandleOppositeDirectionRouting(
      common::Status& status, const double start_timestamp,
      const routing::RoutingResponse* routing_candidate,
      const bool is_different_routing,
      const common::VehicleState& previous_vehicle_state,
      const common::VehicleState& vehicle_state,
      ADCTrajectory* const ptr_trajectory_pb);
  bool IsSameOppositeDirectionRouting(
      const routing::RoutingResponse& routing) const;
  bool IsOppositeDirectionNewRouting(
      const common::VehicleState& vehicle_state,
      const routing::RoutingResponse& new_routing) const;
  bool ExpandRoutingSegmentsToVehicleState(
      const common::VehicleState& vehicle_state,
      routing::RoutingResponse* routing) const;
  void GenerateStopTrajectory(ADCTrajectory* ptr_trajectory_pb);
  void GenerateSlowBreakingTrajectory(ADCTrajectory* ptr_trajectory_pb);
  void ExportFailedLaneChangeSTChart(const planning_internal::Debug& debug_info,
                                     planning_internal::Debug* debug_chart);
  void ExportOnLaneChart(const planning_internal::Debug& debug_info,
                         planning_internal::Debug* debug_chart);
  void ExportOpenSpaceChart(const planning_internal::Debug& debug_info,
                            const ADCTrajectory& trajectory_pb,
                            planning_internal::Debug* debug_chart);
  void PathPointNormalizing(double rotate_angle,
                            const common::math::Vec2d& translate_origin,
                            double* x, double* y, double* phi);
  void AddOpenSpaceOptimizerResult(const planning_internal::Debug& debug_info,
                                   planning_internal::Debug* debug_chart);
  void AddOpenSpaceObstacleDebugInfo(const planning_internal::Debug& debug_info,
                                     planning_internal::Debug* debug_chart);
  void AddOpenSpaceSmoothLineDebugInfo(
      const planning_internal::Debug& debug_info,
      planning_internal::Debug* debug_chart);
  void AddOpenSpaceWarmStartDebugInfo(
      const planning_internal::Debug& debug_info,
      planning_internal::Debug* debug_chart);
  void AddOpenSpaceStartEndPointDebugInfo(
      const planning_internal::Debug& debug_info,
      planning_internal::Debug* debug_chart);
  void AddOpenSpaceROILineDebugInfo(const planning_internal::Debug& debug_info,
                                    planning_internal::Debug* debug_chart);
  void AddOpenSpaceEndHeadingLineDebugInfo(
      const planning_internal::Debug& debug_info,
      planning_internal::Debug* debug_chart);

  void AddPartitionedTrajectory(const planning_internal::Debug& debug_info,
                                planning_internal::Debug* debug_chart);
  void DrawOpenSpaceChosenTrajectory(const planning_internal::Debug& debug_info,
                                     planning_internal::Debug* debug_chart);
  void DrawOpenSpaceTrajectoryStitchPoint(
      const planning_internal::Debug& debug_info,
      planning_internal::Debug* debug_chart);
  void DrawOpenSpaceFallbackTrajectory(
      const planning_internal::Debug& debug_info,
      planning_internal::Debug* debug_chart);

  void AddStitchSpeedProfile(planning_internal::Debug* debug_chart);

  void AddPublishedSpeed(const ADCTrajectory& trajectory_pb,
                         planning_internal::Debug* debug_chart);

  void AddPublishedAcceleration(const ADCTrajectory& trajectory_pb,
                                planning_internal::Debug* debug);

  void AddFallbackTrajectory(const planning_internal::Debug& debug_info,
                             planning_internal::Debug* debug_chart);

  bool DeadEndHandle(const common::PointENU& dead_end_point,
                     const common::VehicleState& vehicle_state);

  bool JudgeCarInDeadEndJunction(
      std::vector<hdmap::JunctionInfoConstPtr>* junctions,
      const common::math::Vec2d& dead_end_point,
      hdmap::JunctionInfoConstPtr* target_junction);

  bool CheckAndPubStopTrajectory(const double start_timestamp,
                                 ReferenceLineInfo* const reference_line_info,
                                 ADCTrajectory* const ptr_trajectory_pb);
  /**
   * @brief 1: planning error 20s
   * 2: stop too long(need distinguish stop line ? ) 90s.
   * 3: virtual traffic light.
   * 4: system monitor fault data.
   * 5: reserve.
   */
  void RemoteDecider(ADCTrajectory* const ptr_trajectory_pb);
  void PubRemoteRequest(const bool lane_borrow_request,
                        ADCTrajectory* const ptr_trajectory_pb);
  void CancelRemoteRequest(ADCTrajectory* const ptr_trajectory_pb);
  void FallbackPlanningThread();

  // Generate preferred lane information
  void GeneratePreferredLane();

  // Generate route lane information
  void GenerateRouteLaneInfo();
  void AddPassageLaneInfo(
      std::vector<std::vector<RouteLaneInfo>>* route_lane_info,
      const std::vector<RouteLaneInfo>& passage_lane);
  void CheckOffLaneDeparture();

  // check and set whether the route destination is reached
  bool JudgeReachTargetPoint(ADCTrajectory* const ptr_trajectory_pb);
  void CheckOpenSpaceReachTargetPoint(ADCTrajectory* const ptr_trajectory_pb);
  void CheckPublicRoadReachTargetPoint(ADCTrajectory* const ptr_trajectory_pb);
  bool CheckHumanShapedDriverRerouting(
      ADCTrajectory* const ptr_trajectory_pb,
      bool near_routing_point_distance,
      bool near_point_distance);
  void CheckDestinationReachedRerouting(
      ADCTrajectory* const ptr_trajectory_pb,
      bool near_target_distance,
      bool near_target_angle,
      bool huaman_shaped_driver);
  void CheckLoopRunningRerouting(ADCTrajectory* const ptr_trajectory_pb,
                                bool near_routing_point_distance);
  bool JudgeArrivedStationImmediately(ADCTrajectory* const ptr_trajectory_pb);
  void ClearRoutingHeader();
  void CheckReroutingInSpecificScenarios(ADCTrajectory* const ptr_trajectory_pb);
  double CalcRemainDistance(const ReferenceLineInfo* reference_line_info);
  void CalcEgoPassageDistance(const ReferenceLine& reference_line, const int& i,
                              const SLBoundary& adc_sl_boundary,
                              const common::PointENU& ego_pose,
                              double& remain_distance);
  void TryReloadPlanningConfIfChanged();
  bool UpdateTopNullFlagsRuntime(bool is_top_null, int action_type,
                                 double reverse_distance);
  bool IsEgoPassage(const std::string& ego_lane_id,
                    const routing::RoadSegment& road_segment);

  void UpdateTrajectory(const ReferenceLineInfo* best_ref_info,
                        const ReferenceLineInfo* target_ref_info,
                        ADCTrajectory* const ptr_trajectory_pb);
  bool CheckFallbackContinue();

  void SetOpenSpaceTrajectory(ADCTrajectory* const ptr_trajectory_pb);
  void SetPublicRoadTrajectory(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const ptr_trajectory_pb);
  void AddPlanningRecordDebug(const ReferenceLineInfo* best_ref_info,
                              ADCTrajectory* const ptr_trajectory_pb);
  bool GetLastFramePoint(
      double* const last_time_stamp, common::SLPoint* const last_sl_point,
      common::SLPoint* const sl_point_of_match_point_in_last_frame);
  void GetLastFrameInfo(
      std::vector<common::FrenetFramePoint>* const last_frenet_frame_points,
      std::vector<common::PathPoint>* const last_discretized_path_points,
      std::vector<Obstacle>* const path_obstacles,
      std::vector<Obstacle>* const obstacles,
      SLBoundary* const adc_sl_boundary);

  bool GetReferenceTrajectoryFromLastFrame(
      const std::vector<common::FrenetFramePoint>& last_frenet_frame_points,
      const std::vector<common::PathPoint>& last_discretized_path_points,
      const std::vector<Obstacle>& path_obstacles,
      const std::vector<Obstacle>& obstacles, const SLBoundary& adc_sl_boundary,
      const double& last_time_stamp, const common::SLPoint& last_sl_point,
      const common::SLPoint& sl_point_of_match_point_in_last_frame,
      DiscretizedTrajectory* const reference_trajectory);
  void AddModifiedObstacle(
      ReferenceLineInfo* const reference_line_info,
      const std::vector<std::pair<std::string, double>>& corrected_obstacles);
  void CreateStackerObstacle(ReferenceLineInfo* const reference_line_info);
  void CreateStackerObstacleWithID(ReferenceLineInfo* const reference_line_info);
  void AddStackerObstacle(ReferenceLineInfo* const reference_line_info);
  void AddFlowerBedObstacle(ReferenceLineInfo* const reference_line_info);
  void FindUnreasonableSpeedObstacles(
      ReferenceLineInfo* ref_line_info,
      std::vector<std::pair<std::string, double>>* corrected_obstacles);
  bool IsObstacleInScope(const Obstacle& obstacle,
                         const ReferenceLineInfo* ref_line_info);
  bool IsObstacleInLargeScope(const Obstacle& obstacle,
                              const ReferenceLineInfo* ref_line_info);
  bool GetScopeState(const Obstacle& obstacle,
                     const ReferenceLineInfo* ref_line_info,
                     const bool is_large_scope, const double left_scope,
                     const double right_scope, const double front_scope,
                     const double rear_scope);
  void GetScopeParams(const bool is_large_scope, double* left_scope,
                      double* right_scope, double* front_scope,
                      double* rear_scope);
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
  void CalcuHonkingStats(ADCTrajectory* const ptr_trajectory_pb);
  void CalcuDisplayType(ADCTrajectory* const ptr_trajectory_pb);
  void CalcuObstaclesDisplay(ReferenceLineInfo* const best_ref_info,
                             ADCTrajectory* const ptr_trajectory_pb);
  bool IsObstacleFarAway(planning::ReferenceLineInfo* const best_ref_info);
  bool IsTaskPointBlockingScenario(
      planning::ReferenceLineInfo* const best_ref_info);
  bool CalReverseObstacleAvoidancePoint(
      planning::ReferenceLineInfo* best_ref_info,
      ADCTrajectory* const ptr_trajectory_pb);
  bool CalTaskStartReverseObstacleAvoidancePoint(
      planning::ReferenceLineInfo* best_ref_info);
  bool CalRoutingRequestReversePoint();
  bool CheckTopBullRerouting();
  bool CheckAndCleanupTopBullReverseState(
      ADCTrajectory* const ptr_trajectory_pb);
  bool CalTopBullReversePoint(ADCTrajectory* const ptr_trajectory_pb);
  bool CalTopBullReverseBorrowPoint(ADCTrajectory* const ptr_trajectory_pb);
  bool IsStackerSafeLonDistance(
      planning::ReferenceLineInfo* const best_ref_info);
  void SetIsAutoDrive(ADCTrajectory* const ptr_trajectory_pb,
                                    bool is_auto_drive);
  void CreateWheelCraneObstacle(ReferenceLineInfo* const reference_line_info);
  void CreateWheelCraneObstacleWithPerception(
      ReferenceLineInfo* const reference_line_info);
  bool IsNeedStopForTrain();

 private:
  std::shared_ptr<cyber::Writer<ADCTrajectory>> planning_writer_;
  routing::RoutingResponse last_routing_;
  std::unique_ptr<ReferenceLineProvider> reference_line_provider_;
  Smoother planning_smoother_;
  bool is_mixed_traffic_ = false;
  bool has_reached_station_ = false;
  double reached_station_time_;
  bool is_dangerous_road_ = false;
  bool is_railway_crossing_ = false;
  int enable_borrow_request_count_ = 0;
  int no_borrow_request_count_ = 0;
  bool last_trajectory_has_borrow_request_ = false;
  std::string last_block_obs_id_ = "";
  bool has_pullover_finished_flag_ = false;
  double pullover_finished_start_time_ = 0.0;
  century::canbus::Chassis_GearPosition vehicle_gear_ =
      century::canbus::Chassis::GEAR_DRIVE;
  // ------------Just For DeadEnd Scenario-------------------------
  // Whether it has entered the Junction area of DeadEnd
  bool wait_flag_ = false;
  // Not used at present
  bool routing_in_flag_ = true;
  // Route destination,  Not the end of the dead end road
  common::PointENU dead_end_point_;
  // ------------Just For DeadEnd Scenario-------------------------
  double planning_error_start_time_;
  double stop_start_time_;
  bool is_stopped_;
  int cnt_obs_human_shaped_ = 0;
  int cnt_block_human_shaped_ = 0;
  std::unique_ptr<cyber::Thread> fallback_planning_thread_ = nullptr;
  bool is_fallback_planning_thread_stop_ = false;
  double start_timestamp_ = Clock::NowInSeconds();
  bool routing_is_ready_ = false;
  std::mutex frame_mutex_;
  common::VehicleState vehicle_state_fallback_;
  bool is_auto_state_ = false;
  bool is_sim_control_ = false;
  bool stop_for_gate_block_ = false;
  bool is_new_stacker_response_ = false;
  century::routing::LaneWaypoint route_end_;
  std::unordered_set<std::string> all_lane_ids_;
  bool west_in_ = false;
  bool west_out_ = false;
  bool east_in_ = false;
  bool east_out_ = false;
  routing::RoutingResponse pending_routing_;
  bool has_pending_routing_ = false;
  bool is_opposite_direction_routing_braking_ = false;
  bool is_opposite_direction_routing_brake_done_ = false;
  uint32_t opposite_direction_routing_seq_num_ = 0;
  double opposite_direction_routing_timestamp_ = 0.0;
  // ------------Remote Control Report-------------------------
  bool request_remote_for_traffic_light_ = false;
  bool is_near_traffic_light_stop_line_ = false;
  bool is_planning_error_ = true;
  bool lane_borrow_failed_ = false;
  bool is_need_backward_for_turn_blocking_ = false;
  int need_backward_count_ = 0;
  int auto_borrow_count_ = 0;
  int disappear_boundary_obs_count_ = 0;
  std::mutex mutex_;
  // ------------Reverse trajectory-------------------------
  std::vector<std::pair<double, double>> ref_v_list_;
  std::pair<std::string, int> last_message_id_;
  std::pair<std::string, int> last_wheelcrane_message_id_;
  bool is_in_passing_ = false;
  int no_stacker_notice_count_ = 0;
  int stacker_id_change_count_ = 0;
  int enable_auto_drive_count_ = 0;
  int replan_count_ = 0;
  bool need_reset_borrow_state_ = false;
  bool top_null_runtime_initialized_ = false;
  bool last_top_null_is_top_null_ = false;
  int last_top_null_action_type_ = 0;
  double last_top_null_reverse_distance_ = 0.0;
  double last_planning_conf_check_time_sec_ = 0.0;
  int64_t last_planning_conf_mtime_sec_ = -1;
  bool is_first_enter_top_bull_ = false;
};

}  // namespace planning
}  // namespace century
