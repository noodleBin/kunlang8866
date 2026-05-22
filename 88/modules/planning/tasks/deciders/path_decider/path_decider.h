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

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/planning/tasks/task.h"

namespace century {
namespace planning {

class PathDecider : public Task {
 public:
  PathDecider(const TaskConfig &config,
              const std::shared_ptr<DependencyInjector> &injector);

  century::common::Status Execute(
      Frame *frame, ReferenceLineInfo *reference_line_info) override;

 private:
  century::common::Status Process(ReferenceLineInfo *reference_line_info,
                                  const PathData &path_data,
                                  PathDecision *const path_decision);

  century::common::Status MakeObjectDecision(
      const PathData &path_data, const std::string &blocking_obstacle_id,
      PathDecision *const path_decision);

  century::common::Status MakeStaticObstacleAndElectricFenceDecision(
      const PathData &path_data, const std::string &blocking_obstacle_id,
      PathDecision *const path_decision);

  void CheckHeadbangPath(Frame *frame,
                         ReferenceLineInfo *const reference_line_info);

  void TrimLastPath(Frame *frame, ReferenceLineInfo *const reference_line_info);

  void GetKappaAndCentripetalAcceleration(
      ReferenceLineInfo *const reference_line_info, double *max_kappa_diff,
      double *max_kappa, double *max_centripetal_acceleration);

  double GetStopDistance();

  double GetObstacleStopDistance(const Obstacle &obstacle,
                                 const double stop_distance);

  bool IsNeedToSkipDecisionMaking(const PathData &path_data,
                                  const std::string &blocking_obstacle_id,
                                  const Obstacle &obstacle,
                                  const double stop_distance,
                                  PathDecision *const path_decision);

  void MakeObstacleLateralDecision(const PathData &path_data,
                                   PathDecision *const path_decision,
                                   const Obstacle &obstacle,
                                   const double stop_distance);

  bool IsCollisionWithObstacle(const PathData &path_data,
                               const Obstacle &obstacle,
                               const double adc_lateral_safe_buffer,
                               double *const stop_limit_s);

  bool IsInDrivableArea(const PathData &path_data, double &collision_distance, std::string &msg);

  void DumpCollisionDebug(const PathData &path_data,
                          const common::PathPoint &path_point,
                          double heading, int iter, double collision_distance);

  bool IsCollisionWithElectricFence(const double &x, const double &y, const double &heading, bool is_path_point = true);

  bool IsPointInJunction(const double &x, const double &y, double search_radius, int junction_type);
  double GetObstacleNearestPointS(const PathData &path_data,
                                  const Obstacle &obstacle);

  void MakePathDecision(const PathData &path_data,
                        ReferenceLineInfo *reference_line_info,
                        PathDecision *const path_decision);

  void MakeFallbackPathDecision(const PathData &path_data,
                                ReferenceLineInfo *reference_line_info);
  void MakeNaearstStackerObjectDecision(const std::string& stacker_id);
  void MakeWheelCraneObjectDecision(const PathData &path_data,
                                    ReferenceLineInfo *reference_line_info,
                                    PathDecision *const path_decision);
  void MakeWeighingDecision(const PathData& path_data,
                            ReferenceLineInfo* reference_line_info,
                            PathDecision* const path_decision);
  void MakeBarrierDecision(const PathData& path_data,
                           ReferenceLineInfo* reference_line_info,
                           PathDecision* const path_decision);
  void MakeBarrierBlockDecision(const PathData& path_data,
                                ReferenceLineInfo* reference_line_info,
                                PathDecision* const path_decision);
  void MakeTemporaryParkingDecision(const PathData& path_data,
                                    ReferenceLineInfo* reference_line_info,
                                    PathDecision* const path_decision);
  void MakeStackerObjectDecision(const PathData &path_data,
                                 ReferenceLineInfo *reference_line_info,
                                 PathDecision *const path_decision);
  void MakeStackerObjectDecisionUsePerception(
      const PathData &path_data, ReferenceLineInfo *reference_line_info,
      PathDecision *const path_decision);
  void MakeJunctionDecision(const PathData &path_data,
                                 ReferenceLineInfo *reference_line_info,
                                 PathDecision *const path_decision);
  // Handle TopBull waiting action: build stop wall and mark completion
  void HandleTopBullWaiting(const PathData &path_data,
                            ReferenceLineInfo *reference_line_info,
                            PathDecision *const path_decision);
  bool IsInLaneTurn(const common::PathPoint &path_point);
  bool CheckStartPonitorEndPointNearStackerForStacker(const std::string& stacker_id,
    Frame *frame, ReferenceLineInfo *const reference_line_info);
      bool CheckStartPonitorEndPointNearStacker(
    Frame *frame, ReferenceLineInfo *const reference_line_info);
  bool CheckIsWorkingStacker(const std::string &stacker_id);
  bool IsJunctionContainStacker(const common::PathPoint &stacker_point,
                                const hdmap::JunctionInfo &junction_info);
  void GetStackerPoseArea(ReferenceLineInfo *reference_line_info,
                          const common::PathPoint &stacker_point,
                          bool *stacker_in_J1, bool *stacker_in_J23);
  void GetAdcPoseArea(bool *adc_in_J1, bool *adc_in_J23);
  bool IsJunctionContainAdc(const common::VehicleState& vehicle_state,
                            const hdmap::JunctionInfo& junction_info);
  bool IsDZoneToYuan2(const PathData &path_data);
  void GetPathPointInTurnLaneRange(const PathData &path_data);
  void CheckForDynamicStacker(const std::string& stacker_id,
                              bool* is_dynamic_stacker,
                              double* nearst_stacker_heading);
  void SetPassingNotice(const std::string& stacker_id);

 private:
  ReferenceLineInfo *reference_line_info_;
  int dynamic_still_count_ = 0;
  int static_still_count_ = 0;
  int no_wheelcrane_count_ = 0;
  bool is_auto_pass_stacker_ = false;
  int adc_stop_times_ = 0;
  bool is_can_pass_weighing_ = false;
};

}  // namespace planning
}  // namespace century
