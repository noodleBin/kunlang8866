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

#include "modules/planning/scenarios/rescue/stage_adjust.h"

#include "cyber/common/log.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/scenarios/util/util.h"

namespace century {
namespace planning {
namespace scenario {
namespace rescue {

namespace {
constexpr int kCheckTime = 10;
constexpr int kRescueCountDecrease = 2;
constexpr double kMaxConstantOpenspaceTime = 2147483647;
constexpr double kStopSpeed = 0.1;
constexpr double kReachDist = 0.75;
constexpr double kDistanceToDestination = 10;
}  // namespace

using century::common::TrajectoryPoint;
using century::cyber::Clock;

Stage::StageStatus RescueStageAdjust::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  CHECK_NOTNULL(frame);
  static int ready_count = 0;
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  const auto& history_frame = injector_->frame_history()->Latest();
  const bool is_ready_to_cruise = scenario::util::CheckADCReadyToCruise(
      injector_->vehicle_state(), frame,
      scenario_config_.front_obstacle_buffer(),
      scenario_config_.heading_buffer());
  AINFO << "is_ready_to_cruise: " << is_ready_to_cruise
        << ", stage_adjust, is on openspace trajectory: "
        << history_frame->open_space_info().is_on_open_space_trajectory();
  auto his_end_pose = history_frame->open_space_info().open_space_end_pose();
  if (!history_frame->open_space_info().is_on_open_space_trajectory()) {
    openspace_success_time_ = Clock::NowInSeconds();
  }

  if (Clock::NowInSeconds() - openspace_success_time_ > FLAGS_rescue_max_time) {
    ready_count = 0;
    AINFO << "adjust overtime "
          << Clock::NowInSeconds() - openspace_success_time_ << " s";
    return FinishStage();
  }

  const auto* rescue_status = injector_->planning_context()
                                  ->mutable_planning_status()
                                  ->mutable_rescue();
  double dx =
      rescue_status->adc_init_position().x() - injector_->vehicle_state()->x();
  double dy =
      rescue_status->adc_init_position().y() - injector_->vehicle_state()->y();

  double pass_s = std::hypot(dx, dy);

  if (is_ready_to_cruise) {
    if (ready_count < kCheckTime) {
      ++ready_count;
    }
  } else {
    ready_count = ready_count > 0 ? ready_count - kRescueCountDecrease : 0;
  }

  /*
    kCheckTime: 10.0
    rescue_min_dist： 3.0
  */
  bool is_new_routing =
      planning::util::IsNewRouting(injector_->planning_context());
  AINFO << "is_new_routing " << is_new_routing;
  if ((ready_count >= kCheckTime && pass_s > FLAGS_rescue_min_dist) ||
      scenario::util::GetEarlyExitRescueState(
          history_frame, injector_->vehicle_state(),
          injector_->openspace_reason(), scenario_config_) ||
      is_new_routing) {
    AINFO << "is ready to cruise, count is " << ready_count;
    ready_count = 0;
    return FinishStage();
  }

  if (!ExecuteTaskOnOpenSpace(frame)) {
    return StageStatus::ERROR;
  } else {
    openspace_success_time_ = Clock::NowInSeconds();
  }

  bool is_end_of_trajectory = false;
  if (history_frame) {
    const auto& trajectory_points =
        history_frame->open_space_info().stitched_trajectory_result();
    if (history_frame->open_space_info().open_space_provider_success() &&
        trajectory_points.size() > 1) {
      is_end_of_trajectory = CheckReachTrajectoryEnd(trajectory_points.back());
    }
  }
  AINFO << " is_ready_to_cruise " << is_ready_to_cruise
        << " is_end_of_trajectory " << is_end_of_trajectory << " ready_count "
        << ready_count << "pass_s " << pass_s;

  // final use
  if (is_end_of_trajectory) {
    return FinishStage();
  }

  return StageStatus::RUNNING;
}

Stage::StageStatus RescueStageAdjust::FinishStage() {
  const auto vehicle_status = injector_->vehicle_state();
  AINFO << " steering_percentage " << vehicle_status->steering_percentage();
  if (std::fabs(vehicle_status->steering_percentage()) <
      scenario_config_.max_steering_percentage_when_cruise()) {
    // directly lane follow
    next_stage_ = ScenarioConfig::NO_STAGE;
    injector_->set_need_to_rescue(false);
  } else {
    return StageStatus::RUNNING;
  }
  return Stage::FINISHED;
}

void RescueStageAdjust::ResetInitPostion() {
  auto* rescue_status = injector_->planning_context()
                            ->mutable_planning_status()
                            ->mutable_rescue();
  rescue_status->mutable_adc_init_position()->set_x(
      injector_->vehicle_state()->x());
  rescue_status->mutable_adc_init_position()->set_y(
      injector_->vehicle_state()->y());
  rescue_status->mutable_adc_init_position()->set_z(0.0);
  rescue_status->set_adc_init_heading(injector_->vehicle_state()->heading());
}

bool RescueStageAdjust::CheckReachTrajectoryEnd(
    const common::TrajectoryPoint& planning_init_point) {
  const auto& path_point = planning_init_point.path_point();
  double dist =
      std::sqrt((path_point.x() - injector_->vehicle_state()->x()) *
                    (path_point.x() - injector_->vehicle_state()->x()) +
                (path_point.y() - injector_->vehicle_state()->y()) *
                    (path_point.y() - injector_->vehicle_state()->y()));
  double adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();
  ADEBUG << "dist: " << dist << ", adc_speed1: " << adc_speed;
  return (std::fabs(adc_speed) < kStopSpeed && dist < kReachDist);
}

}  // namespace rescue
}  // namespace scenario
}  // namespace planning
}  // namespace century
