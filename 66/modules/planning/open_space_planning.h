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
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/monitor/proto/system_status.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "cyber/cyber.h"
#include "cyber/threadlib/threadlib.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/smoothers/smoother.h"
#include "modules/planning/planner/open_space_planner_dispatcher.h"
#include "modules/planning/planning_base.h"
#include "modules/planning/scenarios/rescue_teb/stage_teb.h"

/**
 * @namespace century::planning
 * @brief century::planning
 */
namespace century {
namespace planning {
using century::cyber::Clock;
using century::cyber::ComponentBase;

struct TrajStatusTEB {
  double stamp;
  bool has_traj;
  bool has_traj_for_recovery;
};
/**
 * @class planning
 *
 * @brief Planning module main class. It processes GPS and IMU as input,
 * to generate planning info.
 */
class OpenSpacePlanning : public PlanningBase {
 public:
  explicit OpenSpacePlanning(
      const std::shared_ptr<DependencyInjector>& injector)
      : PlanningBase(injector) {
    planner_dispatcher_ = std::make_unique<OpenSpacePlannerDispatcher>();
  }
  virtual ~OpenSpacePlanning();

  /**
   * @brief Planning name.
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  common::Status Init(const PlanningConfig& config) override;

  void RunOnce(const LocalView& local_view,
               ADCTrajectory* const ptr_trajectory_pb) override;
  //   void RunOnce(const LocalView& local_view, bool* finish_status,
  //                bool* calculation_result);
  common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const trajectory) override;

 private:
  bool PrepareAndUpdateVehicleState(
      const LocalView& local_view, ADCTrajectory* const ptr_trajectory_pb,
      common::VehicleState* vehicle_state,
      std::vector<common::TrajectoryPoint>* stitching_trajectory,
      std::string* replan_reason, double* start_timestamp,
      double* start_system_timestamp, uint32_t* frame_num);

  bool BuildFrameAndTrafficProcess(
      ADCTrajectory* const ptr_trajectory_pb,
      const common::VehicleState& vehicle_state,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      uint32_t frame_num, double start_timestamp);

  bool FinalizePlanningResult(
      ADCTrajectory* const ptr_trajectory_pb,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      const std::string& replan_reason, const double start_timestamp,
      const double start_system_timestamp, common::Status& status);

  ADCTrajectory GetTrajectory() { return adc_trajectory_pb_thread_; }
  void FallbackStop();
  /**
   * @brief main logic of the planning module, runs periodically triggered by
   * timer.
   */
  common::Status IsAdcInRoad(ReferenceLineInfo* reference_line_info);
  void IsAdcInCommonJunction(ReferenceLineInfo* reference_line_info);
  common::Status IsAdcDeviateLaneDirection(
      const ReferenceLineInfo* reference_line_info,
      ADCTrajectory* const ptr_trajectory_pb);
  bool IsJunctionContainAdc(const common::VehicleState& vehicle_state,
                            const hdmap::JunctionInfo& junction_info) const;
  common::Status InitFrame(const uint32_t sequence_num,
                           const common::TrajectoryPoint& planning_start_point,
                           const common::VehicleState& vehicle_state);
  common::Status InitTEBFrame(
      const uint32_t sequence_num,
      const common::TrajectoryPoint& planning_start_point,
      const common::VehicleState& vehicle_state);

  common::VehicleState AlignTimeStamp(const common::VehicleState& vehicle_state,
                                      const double curr_timestamp) const;

  void ExportReferenceLineDebug(planning_internal::Debug* debug);
  bool CheckPlanningConfig(const PlanningConfig& config);
  void GenerateStopTrajectory(ADCTrajectory* ptr_trajectory_pb);
  void ExportFailedLaneChangeSTChart(const planning_internal::Debug& debug_info,
                                     planning_internal::Debug* debug_chart);
  void ExportOnLaneChart(const planning_internal::Debug& debug_info,
                         planning_internal::Debug* debug_chart);
  void ExportOpenSpaceChart(const planning_internal::Debug& debug_info,
                            const ADCTrajectory& trajectory_pb,
                            planning_internal::Debug* debug_chart);

  void ExportOpenSpaceChartThread(const planning_internal::Debug& debug_info,
                                  const ADCTrajectory& trajectory_pb,
                                  planning_internal::Debug* debug_chart);
  void PathPointNormalizing(double rotate_angle,
                            const common::math::Vec2d& translate_origin,
                            double* x, double* y, double* phi);
  void AddOpenSpaceOptimizerResult(const planning_internal::Debug& debug_info,
                                   planning_internal::Debug* debug_chart);
  void AddOpenSpaceOptimizerResultThread(
      const planning_internal::Debug& debug_info,
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
  void AddPartitionedTrajectoryThread(
      const planning_internal::Debug& debug_info,
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
  void AddStitchSpeedProfileThread(planning_internal::Debug* debug_chart);

  void AddPublishedSpeed(const ADCTrajectory& trajectory_pb,
                         planning_internal::Debug* debug_chart);
  void AddPublishedSpeedThread(const ADCTrajectory& trajectory_pb,
                               planning_internal::Debug* debug_chart);

  void AddPublishedAcceleration(const ADCTrajectory& trajectory_pb,
                                planning_internal::Debug* debug);
  void AddPublishedAccelerationThread(const ADCTrajectory& trajectory_pb,
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
  bool JudgeArrivedStationImmediately(ADCTrajectory* const ptr_trajectory_pb);

  double CalcRemainDistance(const ReferenceLineInfo* reference_line_info);
  bool IsEgoPassage(const std::string& ego_lane_id,
                    const routing::RoadSegment& road_segment);

  void UpdateTrajectory(const ReferenceLineInfo* best_ref_info,
                        const ReferenceLineInfo* target_ref_info,
                        ADCTrajectory* const ptr_trajectory_pb);
  bool CheckFallbackContinue();

  void SetOpenSpaceTrajectory(ADCTrajectory* const ptr_trajectory_pb);
  void SetOpenSpaceTrajectoryThread(ADCTrajectory* const ptr_trajectory_pb);
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

  bool GenerateTEBThread(
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const ptr_trajectory_pb);
  void UpdateTEBOriginInfo(century::planning::Frame* frame_teb);

  bool CalculationTEBTrajectory(
      century::planning::Frame* frame_teb, double* openspace_success_time,
      scenario::rescue::RescueStageTeb* rescue_stage_teb);
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
  void UpdateTEBOpenSpaceInfo(
      ADCTrajectory* const adc_trajectory_pb_thread_temp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory);
  void UpdateRescueStatusInfo(Frame* frame_teb);
  void UpdateTEBFrameInfo();
  bool CheckDrivingModeEnableRescue();
  bool CheckIsEnableRescue(ADCTrajectory* const adc_trajectory);
  void CalVehicleInCommonJunction();
  void CalPrpValidTrace(ADCTrajectory* const adc_trajectory);
  bool EnableRescue();
  void CalVehicleInPlayStreet();
  void CollectLaneDrivingTrajInfo(ADCTrajectory* const adc_trajectory);
  bool CheckRescueLaneSafe();
  bool CheckIsYield();
  bool IsCanPlanTebOnPublicRoad(const ReferenceLineInfo& reference_line_info);
  bool IsAdcTrajectoryValid(const ADCTrajectory& adc_trajectory);

 private:
  std::shared_ptr<cyber::Writer<ADCTrajectory>> planning_writer_;
  routing::RoutingResponse last_routing_;
  std::unique_ptr<ReferenceLineProvider> reference_line_provider_;
  Smoother planning_smoother_;
  bool is_mixed_traffic_ = false;
  bool has_reached_station_ = false;
  double reached_station_time_;
  bool is_dangerous_road_ = false;

  bool has_pullover_finished_flag_ = false;
  double pullover_finished_start_time_ = 0.0;
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
  std::unique_ptr<cyber::Thread> fallback_planning_thread_ = nullptr;
  bool is_fallback_planning_thread_stop_ = false;
  double start_timestamp_ = Clock::NowInSeconds();
  bool routing_is_ready_ = false;
  std::mutex frame_mutex_;
  std::mutex frame_teb_mutex_;
  common::VehicleState vehicle_state_fallback_;
  bool is_auto_state_ = false;
  bool is_sim_control_ = false;
  century::routing::LaneWaypoint route_end_;
  // ------------Remote Control Report-------------------------
  bool request_remote_for_traffic_light_ = false;
  bool is_near_traffic_light_stop_line_ = false;
  bool is_planning_error_ = true;
  bool lane_borrow_failed_ = false;
  std::mutex mutex_;
  ADCTrajectory adc_trajectory_pb_thread_;
  // ------------Remote Control Report-------------------------
};

}  // namespace planning
}  // namespace century
