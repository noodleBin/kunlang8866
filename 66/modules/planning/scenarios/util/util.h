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

#include <string>

#include "modules/common/util/normal_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/reference_line_info.h"

namespace century {
namespace planning {
namespace scenario {
namespace util {

enum PullOverStatus {
  UNKNOWN = 0,
  APPROACHING = 1,
  PARK_COMPLETE = 2,
  PARK_FAIL = 3,
  PASS_DESTINATION = 4,
};

hdmap::PathOverlap* GetOverlapOnReferenceLine(
    const ReferenceLineInfo& reference_line_info, const std::string& overlap_id,
    const ReferenceLineInfo::OverlapType& overlap_type);

PullOverStatus CheckADCPullOver(
    const common::VehicleStateProvider* vehicle_state_provider,
    const ReferenceLineInfo& reference_line_info,
    const ScenarioPullOverConfig& scenario_config,
    const PlanningContext* planning_context);

PullOverStatus CheckADCPullOverPathPoint(
    const ReferenceLineInfo& reference_line_info,
    const ScenarioPullOverConfig& scenario_config,
    const common::PathPoint& path_point,
    const PlanningContext* planning_context);

bool CheckPullOverPositionBySL(const ReferenceLineInfo& reference_line_info,
                               const ScenarioPullOverConfig& scenario_config,
                               const common::math::Vec2d& adc_position,
                               const double adc_theta,
                               const common::math::Vec2d& target_position,
                               const double target_theta, const bool check_s);

/*
 * @brief check if it is possible to start cruising
 * @param vehicle_state_provider     vehicle state
 * @param frame                      data frame
 * @param scenario_config            configure
 */
bool CheckADCReadyToCruise(
    const common::VehicleStateProvider* vehicle_state_provider, Frame* frame,
    const ScenarioParkAndGoConfig& scenario_config);

/*
 * @brief check whether has obstacle in front of vehicle
 * @param adc_position                  vehicle position
 * @param adc_heading                   vehicle heading
 * @param frame                         data frame
 * @param front_obstacle_buffer         obstacle distance threshold
 */
bool CheckADCSurroundObstacles(const common::math::Vec2d adc_position,
                               const double adc_heading, Frame* frame,
                               const double front_obstacle_buffer);

/*
 * @brief check heading error
 * @param adc_position            vehicle position
 * @param adc_heading             vehicle heading
 * @param reference_line_info     reference line
 * @param heading_buffer          heading error threshold
 */
bool CheckADCHeading(const common::math::Vec2d adc_position,
                     const double adc_heading,
                     const ReferenceLineInfo& reference_line_info,
                     const double heading_buffer);

bool CheckADCReadyToCruise(
    const common::VehicleStateProvider* vehicle_state_provider, Frame* frame,
    const double front_obstacle_buffer, const double ref_heading_buffer);

bool CheckRefReadyToCruise(
    const common::VehicleStateProvider* vehicle_state_provider, Frame* frame,
    century::planning::RescueStatus* rescue_status,
    const double front_obstacle_buffer, const double ref_heading_buffer);

bool GetEarlyExitRescueState(
    const Frame* frame,
    const century::common::VehicleStateProvider* vehicle_status,
    const century::planning::OpenspaceReason& reason,
    const ScenarioRescueConfig& rescue_config);

}  // namespace util
}  // namespace scenario
}  // namespace planning
}  // namespace century
