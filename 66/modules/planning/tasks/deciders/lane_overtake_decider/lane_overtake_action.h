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

#include <memory>
#include <string>

#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_status.pb.h"

#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/tasks/deciders/lane_overtake_decider/lane_overtake_fsm_common.h"

namespace century {
namespace planning {

class FsmAction {
 public:
  enum CheckDirection : uint8_t {
    NO_CHECK = 0,
    CHECK_LEFT = 1,
    CHECK_RIGHT = 2,
    CHECK_BOTH = 3,
    CHECK_SELF = 4,
  };

 public:
  explicit FsmAction(const TaskConfig& config,
                     const std::shared_ptr<DependencyInjector>& injector);

  void Default2Default(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  void Default2Prepare(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  void Prepare2Default(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  void Prepare2Turn(Frame* const frame,
                    ReferenceLineInfo* const reference_line_info);
  void Prepare2Prepare(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  void Turn2Return(Frame* const frame,
                   ReferenceLineInfo* const reference_line_info);
  void Turn2Overtake(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  void Turn2Turn(Frame* const frame,
                 ReferenceLineInfo* const reference_line_info);
  void Overtake2Fail(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  void Overtake2Finish(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  void Overtake2Overtake(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info);
  void Overtake2Return(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  void Return2Finish(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  void Return2Hold(Frame* const frame,
                   ReferenceLineInfo* const reference_line_info);
  void Return2Return(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  void Hold2Fail(Frame* const frame,
                 ReferenceLineInfo* const reference_line_info);
  void Hold2Return(Frame* const frame,
                   ReferenceLineInfo* const reference_line_info);
  void Hold2Hold(Frame* const frame,
                 ReferenceLineInfo* const reference_line_info);
  void Finish2Default(Frame* const frame,
                      ReferenceLineInfo* const reference_line_info);
  void Finish2Finish(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  void Fail2Default(Frame* const frame,
                    ReferenceLineInfo* const reference_line_info);
  void Fail2Fail(Frame* const frame,
                 ReferenceLineInfo* const reference_line_info);

 private:
  void InitParamInfo(Frame* const frame);
  bool IsRoadSupportOvertake(ReferenceLineInfo* const reference_line_info);
  bool IsObstacleNeedToBeOvertaken(Frame* const frame,
                                   ReferenceLineInfo* const reference_line_info,
                                   const Obstacle* obstacle) const;
  bool ObstacleIsFarFromAdc(ReferenceLineInfo* const reference_line_info,
                            const Obstacle* obstacle) const;
  bool ObstacleIsTooFast(ReferenceLineInfo* const reference_line_info,
                         const Obstacle* obstacle) const;
  bool ObstacleIsBlocking(Frame* const frame, const Obstacle* obstacle) const;
  double GetMinOvertakeNeededDistance(
      ReferenceLineInfo* const reference_line_info,
      const Obstacle* obstacle) const;
  bool IsSafeToOvertake(ReferenceLineInfo* const reference_line_info,
                        const bool& is_turn_left,
                        const ReferenceLineInfo::CheckLevel& check_level);
  bool GetNeighborLaneWidth(ReferenceLineInfo* const reference_line_info,
                            const bool& is_turn_left,
                            double* const neighbor_lane_width);
  bool CheckIsForwoardMovingObstacle(
      ReferenceLineInfo* const reference_line_info,
      const Obstacle* obstacle) const;
  void ResetOvertakeInfo(Frame* const frame);
  bool CheckDistanceToOverlap(ReferenceLineInfo* const reference_line_info);
  bool CheckDistanceToDestination(ReferenceLineInfo* const reference_line_info);
  bool CheckSolidLine(ReferenceLineInfo* const reference_line_info,
                      const CheckDirection& check_direction);
  bool LaneBoundaryTypeCheckOK(
      ReferenceLineInfo* const reference_line_info,
      const hdmap::LaneBoundaryType::Type& lane_boundary_type,
      const double check_s);
  bool CheckRoutingEnd(ReferenceLineInfo* const reference_line_info,
                       const CheckDirection& check_direction);
  bool HasOvertakePassTheObstacle(ReferenceLineInfo* const reference_line_info,
                                  const Obstacle* obstacle);
  bool AdcWillPassMergeLane(ReferenceLineInfo* const reference_line_info);
  bool NeighborLaneIsMerge(ReferenceLineInfo* const reference_line_info,
                           const double check_s);
  bool NeedCancelLaneChangeInMergeLane(
      Frame* const frame, ReferenceLineInfo* const reference_line_info);
  void CheckIsNeedBuildLaneChangeStop(
      ReferenceLineInfo* const reference_line_info, bool* const need_stop,
      double* const stop_s);

 private:
  TaskConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
  std::string origin_lane_id_ = "";
  double lane_change_remain_dis_;
  double routing_end_remain_dis_;
  bool find_solid_start_ = false;
  bool need_give_up_overtake_ = false;
  common::math::Vec2d solid_start_point_;
};

}  // namespace planning
}  // namespace century
