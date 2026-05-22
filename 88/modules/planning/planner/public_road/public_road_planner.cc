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

#include "modules/planning/planner/public_road/public_road_planner.h"

#include "modules/planning/common/planning_gflags.h"

namespace century {
namespace planning {

using century::common::Status;
using century::common::TrajectoryPoint;

namespace {
constexpr double kEStopSpeed = 0.15;
}  // namespace

Status PublicRoadPlanner::Init(const PlanningConfig& config) {
  config_ = config;
  scenario_manager_.Init(config);
  return Status::OK();
}

Status PublicRoadPlanner::Plan(const TrajectoryPoint& planning_start_point,
                               Frame* frame,
                               ADCTrajectory* ptr_computed_trajectory) {
  scenario_manager_.Update(planning_start_point, *frame);
  scenario_ = scenario_manager_.mutable_scenario();
  auto result = scenario_->Process(planning_start_point, frame);

  if (FLAGS_enable_record_debug) {
    auto scenario_debug = ptr_computed_trajectory->mutable_debug()
                              ->mutable_planning_data()
                              ->mutable_scenario();
    scenario_debug->set_scenario_type(scenario_->scenario_type());
    scenario_debug->set_stage_type(scenario_->GetStage());
    scenario_debug->set_msg(scenario_->GetMsg());
  }

  Status ret_status = Status::OK();
  if (result == scenario::Scenario::STATUS_DONE) {
    // only updates scenario manager when previous scenario's status is
    // STATUS_DONE
    scenario_manager_.Update(planning_start_point, *frame);
  } else if (result == scenario::Scenario::STATUS_UNKNOWN) {
    if (nullptr != frame->local_view().routing &&
        frame->local_view().routing->has_header() &&
        std::abs(Clock::NowInSeconds() -
                 frame->local_view().routing->header().timestamp_sec()) <
            FLAGS_signal_expire_time_sec &&
        frame->vehicle_state().linear_velocity() < kEStopSpeed) {
      ret_status = Status(common::PLANNING_ERROR_NEED_RESTART,
                          "scenario returned unknown");
    } else {
      ret_status = Status(common::PLANNING_ERROR, "scenario returned unknown");
    }
    ret_status.merge_error_message(scenario_->GetCommonStatus());
    return ret_status;
  }
  ret_status.merge_error_message(scenario_->GetCommonStatus());
  return ret_status;
}

}  // namespace planning
}  // namespace century
