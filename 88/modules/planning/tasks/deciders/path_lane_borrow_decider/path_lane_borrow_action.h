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

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_status.pb.h"

#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_fsm_common.h"

namespace century {
namespace planning {

class LaneBorrowFsmAction {
 public:
  explicit LaneBorrowFsmAction(
      const TaskConfig& config,
      const std::shared_ptr<DependencyInjector>& injector);

  void Default2Default(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  void Default2Prepare(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  void Prepare2Default(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  void Prepare2LeftBorrow(Frame* const frame,
                          ReferenceLineInfo* const reference_line_info);
  void Prepare2RightBorrow(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info);
  void Prepare2Prepare(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  void LeftBorrow2Return(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info);
  void LeftBorrow2LeftBorrow(Frame* const frame,
                             ReferenceLineInfo* const reference_line_info);
  void RightBorrow2Return(Frame* const frame,
                          ReferenceLineInfo* const reference_line_info);
  void RightBorrow2RightBorrow(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info);
  void Return2LeftBorrow(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info);
  void Return2RightBorrow(Frame* const frame,
                          ReferenceLineInfo* const reference_line_info);
  void Return2Finish(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  void Return2Return(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  void Finish2Default(Frame* const frame,
                      ReferenceLineInfo* const reference_line_info);
  void Finish2Finish(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);

 private:
  bool FindLaneBorrowObstacle(ReferenceLineInfo* const reference_line_info);
  bool IsObsDisappearOrFarAway(ReferenceLineInfo* const reference_line_info);
  bool IsNecessaryToLaneBorrow(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info);
  bool HasSingleReferenceLine(const Frame& frame);
  bool IsWithinSidePassingSpeedADC();
  bool IsBlockingObstacleWithinDestination(
      const ReferenceLineInfo& reference_line_info);
  bool IsBlockingObstacleFarFromIntersection(
      Frame* const frame, ReferenceLineInfo* const reference_line_info);
  bool IsSidePassableObstacle(const ReferenceLineInfo& reference_line_info);
  void CheckBothLaneBorrowable(const ReferenceLineInfo& reference_line_info,
                               bool* const is_left_borrowable,
                               bool* const is_right_borrowable);
  void CheckLeftLaneBorrowable(const ReferenceLineInfo& reference_line_info,
                               bool* const is_left_borrowable);
  void CheckRightLaneBorrowable(const ReferenceLineInfo& reference_line_info,
                                bool* const is_right_borrowable);
  bool IsLaneTypeSupportLaneBorrow(
      const ReferenceLineInfo& reference_line_info);
  bool IsLaneBoundaryTypeBorrowable(
      const hdmap::LaneBoundaryType::Type& lane_boundary_type);
  bool IsNeighbourLaneEmpty(
      const hdmap::LaneWaypoint& waypoint,
      const LaneborrowStatus::LaneborrowDirection& direction);
  bool IsLeftNeighbourForwardLaneEmpty(
      ReferenceLineInfo* const reference_line_info);
  bool IsNearObstacle(const ReferenceLineInfo& reference_line_info);
  void CheckIsSafeToLaneBorrow(const ReferenceLineInfo& reference_line_info);
  double GetForwardCheckDistance(const ReferenceLineInfo& reference_line_info);
  bool AbleToUseSelfLane(const ReferenceLineInfo& reference_line_info);
  bool IsBorrowNeedRemoteRequest(ReferenceLineInfo* const reference_line_info);
  void PrintDebugInfo(
      const planning_internal::LaneBorrowDebug::LaneBorrowFsmStatus&
          lane_borrow_status,
      ReferenceLineInfo* const reference_line_info);
  void CheckLaneBorrowInPlayStreet(
      ReferenceLineInfo* const reference_line_info);
  void CheckLaneBorrowInAuxiliaryRoad(
      Frame* const frame, ReferenceLineInfo* const reference_line_info);
  void CheckLaneBorrowInfo(ReferenceLineInfo* const reference_line_info);
  double CheckBorrowRemainDistance(
      ReferenceLineInfo* const reference_line_info);
  void CheckAdcDistance(ReferenceLineInfo* const reference_line_info);
  void CheckNearJunctionExitLaneBorrow(
      Frame* const frame, ReferenceLineInfo* const reference_line_info);
  bool IsForcedToExitLaneBorrow(ReferenceLineInfo* const reference_line_info);
  void CheckIsNeedGobackReferenceLane(
      Frame* const frame, ReferenceLineInfo* const reference_line_info);
  void CheckIsNeedGotoLeftNeighborLane(
      Frame* const frame, ReferenceLineInfo* const reference_line_info);
  void CheckIsNeedPassTruck(ReferenceLineInfo* const reference_line_info);
  std::pair<bool, bool> CheckIsObsInFrontOfAdc(
      ReferenceLineInfo* const reference_line_info);
  bool IsBlockingObstacleTruck(ReferenceLineInfo* const reference_line_info);
  int32_t GetKeepCheckCountLimit(ReferenceLineInfo* const reference_line_info);
  void CheckObsNearJunction(const double distance,
                            const Obstacle* const block_obs,
                            bool* const is_obs_near_junction,
                            bool* const has_truck_near_junction);
  void CheckVehicleObsInFrontOfAdc(
      ReferenceLineInfo* const reference_line_info);
  void CheckTrafficStatus(ReferenceLineInfo* const reference_line_info);
  int32_t GetKeepCheckCountLimit(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info);
  void CheckAuxiliaryRoadStatus(ReferenceLineInfo* const reference_line_info);
  void CheckAuxiliaryRoadExitLaneBorrow(
      Frame* const frame, ReferenceLineInfo* const reference_line_info);
  /*
   * @brief Calculate additional safety time based on lateral position
   * @param reference_line_info   reference line
   * @param direction   lane borrow direction
   * */
  double GetExtraSafeCheckTime(const ReferenceLineInfo& reference_line_info,
                               const LaneborrowStatus::LaneborrowDirection direction);

 private:
  TaskConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
  double closest_stop_sign_start_s_ = std::numeric_limits<double>::max();
  bool not_borrow_reverse_ = false;
  bool has_checked_obs_in_front_ = false;
  std::pair<bool, bool> is_obs_in_front_ = {false, false};
  // left neighour lane vehicle
  std::vector<const Obstacle*> left_lane_veh_;
  // current lane vehicle
  std::vector<const Obstacle*> ref_lane_veh_;
  // right neighbour lane vehicle
  std::vector<const Obstacle*> right_lane_veh_;
  // current lane static obstacle
  std::vector<const Obstacle*> ref_static_obs_;
  std::vector<const Obstacle*> ref_lane_obs_;
  // left neighour lane static obstacle
  std::vector<const Obstacle*> left_static_obs_;
  // right neighour lane static obstacle
  std::vector<const Obstacle*> right_static_obs_;
  int32_t last_obs_keep_check_threshold_ = 0;
  bool has_checked_red_light_ = false;
};

}  // namespace planning
}  // namespace century
