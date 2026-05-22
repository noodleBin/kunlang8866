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

#include "modules/planning/scenarios/uturn_teb/stage_uturn.h"

#include <algorithm>
#include <utility>

#include "cyber/common/log.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/scenarios/util/util.h"
namespace century {
namespace planning {
namespace scenario {
namespace uturn {

namespace {
constexpr int kCheckTime = 4;
constexpr double kStopSpeed = 0.1;
constexpr double kReachDist = 0.75;
constexpr double kDistanceToDestination = 10.0;
constexpr size_t kMaxKeepCount = 100;
}  // namespace

using century::common::TrajectoryPoint;
using century::cyber::Clock;

UturnStageTeb::UturnStageTeb(
    const ScenarioConfig::StageConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Stage(config, injector) {
  auto* common_status = injector_->planning_context()
                            ->mutable_planning_status()
                            ->mutable_teb_common();

  common_status->mutable_adc_init_position()->set_x(
      injector_->vehicle_state()->x());
  common_status->mutable_adc_init_position()->set_y(
      injector_->vehicle_state()->y());
  common_status->mutable_adc_init_position()->set_z(0.0);
  common_status->set_adc_init_heading(injector_->vehicle_state()->heading());
}

Stage::StageStatus UturnStageTeb::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  AINFO << "enter uturn ";
  CHECK_NOTNULL(frame);
  frame_ = frame;
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);

  const auto& end_point = frame->local_view()
                              .routing->routing_request()
                              .waypoint()
                              .rbegin()
                              ->pose();
  const double dx = injector_->vehicle_state()->x() - end_point.x();
  const double dy = injector_->vehicle_state()->y() - end_point.y();
  const auto& distance = std::sqrt(dx * dx + dy * dy);
  if (distance < kDistanceToDestination) {
    AINFO << "adc is near goal , not enter uturn mode ";
    return FinishStage();
  }

  bool plan_ok = ExecuteTaskOnOpenSpace(frame);

  static size_t keep_count = 0;
  if (!plan_ok) {
    if (keep_count < kMaxKeepCount && GenerateStopTrajectory()) {
      ++keep_count;
      return StageStatus::RUNNING;
    }
    return StageStatus::ERROR;
  }
  keep_count = 0;

  const double front_obstacle_buffer = scenario_config_.front_obstacle_buffer();
  const double heading_buffer = scenario_config_.heading_buffer();
  const bool is_ready_to_cruise = scenario::util::CheckADCReadyToCruise(
      injector_->vehicle_state(), frame, front_obstacle_buffer, heading_buffer);

  bool reached_destination =
      frame->mutable_open_space_info()->destination_reached();
  if (reached_destination || is_ready_to_cruise) {
    injector_->use_teb_default_bound_ = false;
    AERROR << "----reached_destination-----";
    return FinishStage();
  }

  return StageStatus::RUNNING;
}

Stage::StageStatus UturnStageTeb::FinishStage() {
  const auto vehicle_status = injector_->vehicle_state();

  if (std::fabs(vehicle_status->steering_percentage()) <
      scenario_config_.max_steering_percentage_when_cruise()) {
    // directly lane follow
    next_stage_ = ScenarioConfig::NO_STAGE;
    AERROR << " directly lane follow ";
  } else {
    return StageStatus::RUNNING;
  }
  injector_->is_need_to_uturn_ = false;
  injector_->use_teb_default_bound_ = false;
  AERROR << " FinishStage: directly lane follow ";
  return Stage::FINISHED;
}

bool UturnStageTeb::CheckReachTrajectoryEnd(
    const common::TrajectoryPoint& planning_init_point) {
  const auto& path_point = planning_init_point.path_point();
  double dist =
      std::sqrt((path_point.x() - injector_->vehicle_state()->x()) *
                    (path_point.x() - injector_->vehicle_state()->x()) +
                (path_point.y() - injector_->vehicle_state()->y()) *
                    (path_point.y() - injector_->vehicle_state()->y()));
  double adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();
  AINFO << "---dist " << dist << " --- adc_speed " << adc_speed;
  return (std::fabs(adc_speed) < kStopSpeed && dist < kReachDist);
}

bool UturnStageTeb::GenerateStopTrajectory() {
  auto* previous_frame = injector_->frame_history()->Latest();
  if (nullptr == previous_frame) {
    return false;
  }

  auto* publishable_traj_and_gear =
      frame_->mutable_open_space_info()->mutable_publishable_trajectory_data();

  *publishable_traj_and_gear =
      previous_frame->open_space_info().publishable_trajectory_data();

  auto& trajectory = publishable_traj_and_gear->first;

  for (size_t i = 0; i < trajectory.size(); ++i) {
    auto& point = trajectory[i];
    point.set_v(0.0);
    point.set_a(0);
  }
  return true;
}

}  // namespace uturn
}  // namespace scenario
}  // namespace planning
}  // namespace century
