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

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/planning/tasks/deciders/decider.h"
#include "modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_fsm.h"

namespace century {
namespace planning {

class PathLaneBorrowDecider : public Decider {
 public:
  PathLaneBorrowDecider(const TaskConfig& config,
                        const std::shared_ptr<DependencyInjector>& injector);

 private:
  common::Status Process(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;

  bool IsNecessaryToBorrowLane(const Frame& frame,
                               const ReferenceLineInfo& reference_line_info);

  bool CheckAdcPostureIsCorrect();

  bool HasSingleReferenceLine(const Frame& frame);

  bool IsWithinSidePassingSpeedADC();

  bool IsWithinSuitableDistance(const ReferenceLineInfo& reference_line_info);

  bool IsBlockingObstacleWithinDestination(
      const ReferenceLineInfo& reference_line_info);

  bool IsBlockingObstacleFarFromIntersection(
      const ReferenceLineInfo& reference_line_info);

  bool IsSidePassableObstacle(const ReferenceLineInfo& reference_line_info);

  void CheckLaneBorrow(const ReferenceLineInfo& reference_line_info,
                       bool* const left_neighbor_lane_borrowable,
                       bool* const right_neighbor_lane_borrowable);
  bool IsNearObstacle(const ReferenceLineInfo& reference_line_info);

  bool CheckIsNeedExitLaneBorrow(const ReferenceLineInfo& reference_line_info);

  bool CheckIsCouldEnterLaneBorrow(
      const Frame& frame, const ReferenceLineInfo& reference_line_info);

  bool IsLaneTypeSupportBorrow(const ReferenceLineInfo& reference_line_info);

  double GetForwardCheckDistance(const ReferenceLineInfo& reference_line_info);

  void CheckLaneBoundaryTypeIsBorrowable(
      const ReferencePoint& ref_point, const double check_s,
      bool* const left_neighbor_lane_borrowable,
      bool* const right_neighbor_lane_borrowable);

  bool LaneBoundTypeIsUnPassable(
      const hdmap::LaneBoundaryType::Type& lane_boundary_type);

  bool IsBorrowNeedRemoteRequest(const ReferenceLineInfo& reference_line_info);

  bool IsStopNearIntersectionLongTime(
      const ReferenceLineInfo& reference_line_info);

  void CheckIsSafeToLaneBorrow(const ReferenceLineInfo& reference_line_info);

  double CheckBorrowRemainDistance();

  void CheckAdcDistance(const ReferenceLineInfo& reference_line_info);

  void CheckNearJunctionExitLaneBorrow(
      const ReferenceLineInfo& reference_line_info);

  void CheckIsNeedGotoLeftNeighborLane(
      const ReferenceLineInfo& reference_line_info);

  void CheckIsNeedGobackReferenceLane(
      const ReferenceLineInfo& reference_line_info);

  void CheckIsNeedPassTruck(const ReferenceLineInfo& reference_line_info);

  bool IsForcedToExitLaneBorrow(const ReferenceLineInfo& reference_line_info);

  bool IsAdcInLeftNeighborLane(const ReferenceLineInfo& reference_line_info);

  std::pair<bool, bool> CheckIsObsInFrontOfAdc(
      const ReferenceLineInfo& reference_line_info);

  void CheckObsNearJunction(const double distance,
                            const Obstacle* const block_obs,
                            bool* const is_obs_near_junction,
                            bool* const has_truck_near_junction);

  void CheckVehicleObsInFrontOfAdc(
      const ReferenceLineInfo& reference_line_info,
      std::vector<const Obstacle*>* const left_lane_veh,
      std::vector<const Obstacle*>* const ref_lane_veh);

  bool IsBlockingObstacleTruck(const ReferenceLineInfo& reference_line_info);

  int32_t GetKeepCheckCountLimit(const ReferenceLineInfo& reference_line_info);

  void CheckTrafficStatus(const ReferenceLineInfo& reference_line_info);

  FRIEND_TEST(PathLaneBorrowDeciderTest,
              IsNecessaryToBorrowLane_PlayStreet_OnLane);
  FRIEND_TEST(PathLaneBorrowDeciderTest,
              IsNecessaryToBorrowLane_PlayStreet_OffLane);
  FRIEND_TEST(PathLaneBorrowDeciderTest, IsNecessaryToBorrowLane_TrueScenario1);
  FRIEND_TEST(PathLaneBorrowDeciderTest, IsNecessaryToBorrowLane_TrueScenario2);
  FRIEND_TEST(PathLaneBorrowDeciderTest, IsNecessaryToBorrowLane_TrueScenario3);

 private:
  double stop_start_time_ = std::numeric_limits<double>::max();
  double closest_stop_sign_start_s_ = std::numeric_limits<double>::max();
  bool not_borrow_reverse_ = false;
  bool has_checked_obs_in_front_ = false;
  std::pair<bool, bool> is_obs_in_front_ = {false, false};
  std::shared_ptr<LaneBorrowFsm> lane_borrow_fsm_;
};

}  // namespace planning
}  // namespace century
