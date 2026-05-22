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

#include "modules/planning/scenarios/traffic_light/protected_teb/stage_adjust_teb.h"

#include <string>
#include <utility>

#include "cyber/common/log.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/util/util.h"

namespace century {
namespace planning {
namespace scenario {
namespace traffic_light_teb {

namespace {
constexpr double kStopSpeed = 0.05;  // m/s
constexpr size_t kStopTrajectorySize = 10;
constexpr double kRelativetime = 0.1;
}  // namespace

using century::common::TrajectoryPoint;
using century::hdmap::PathOverlap;
using century::perception::TrafficLight;

TrafficLightProtectedStageAdjustTeb::TrafficLightProtectedStageAdjustTeb(
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

Stage::StageStatus TrafficLightProtectedStageAdjustTeb::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  CHECK_NOTNULL(frame);
  scenario_config_.CopyFrom(GetContext()->scenario_config);
  frame_ = frame;
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  bool plan_ok = ExecuteTaskOnOpenSpace(frame);
  if (!plan_ok) {
    AERROR << "TrafficLigth Adjust planning error";
  }
  // end state is the same with origin approach ,
  // approach is consider to enter adjust
  const auto& reference_line_info = frame->reference_line_info().front();

  // to get from frame
  std::string traffic_id = frame->GetCurrentTrafficLightId();
  PathOverlap* current_traffic_light_overlap =
      scenario::util::GetOverlapOnReferenceLine(reference_line_info, traffic_id,
                                                ReferenceLineInfo::SIGNAL);
  if (!current_traffic_light_overlap) {
    AERROR << "!current_traffic_light_overlap"
           << "traffic_id is " << traffic_id;
    return FinishScenario();
  }
  // set right_of_way_status
  reference_line_info.SetJunctionRightOfWay(
      current_traffic_light_overlap->start_s, false);
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double distance_adc_to_stop_line =
      current_traffic_light_overlap->start_s - adc_front_edge_s;
  perception::TrafficLight traffic_light;
  frame->GetSignal(traffic_id, &traffic_light);

  if (distance_adc_to_stop_line >
      scenario_config_.start_traffic_light_scenario_distance()) {
    AINFO << "dist too far,the value is "
          << scenario_config_.start_traffic_light_scenario_distance();
    return FinishScenario();
  }

  const auto& signal_color = traffic_light.color();
  AINFO << "traffic_light_overlap_id[" << traffic_id << "] start_s["
        << current_traffic_light_overlap->start_s
        << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line
        << "] color[" << signal_color << "]";
  // || parking finisher ,return FinishScenario
  if (TrafficLight::GREEN == signal_color) {
    return FinishStage();
  }

  return Stage::RUNNING;
}

Stage::StageStatus TrafficLightProtectedStageAdjustTeb::FinishStage() {
  if (std::fabs(injector_->vehicle_state()->steering_percentage()) >
      FLAGS_teb_return_angle_limit) {
    double adc_speed = std::fabs(
        injector_->vehicle_state()->vehicle_state().linear_velocity());
    if (adc_speed > kStopSpeed) {
      return Stage::RUNNING;
    } else {
      AINFO << "need send empty traj for steering_percentage < limit";
      GenerateStopTrajectory();
      return StageStatus::ERROR;
    }
  }
  auto* traffic_light = injector_->planning_context()
                            ->mutable_planning_status()
                            ->mutable_traffic_light();
  traffic_light->clear_done_traffic_light_overlap_id();
  for (const auto& traffic_light_overlap_id :
       GetContext()->current_traffic_light_overlap_ids) {
    traffic_light->add_done_traffic_light_overlap_id(traffic_light_overlap_id);
  }

  next_stage_ = ScenarioConfig::TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE;
  return Stage::FINISHED;
}

void TrafficLightProtectedStageAdjustTeb::GenerateStopTrajectory() {
  TrajGearPair fallback_trajectory_pair_candidate =
      frame_->open_space_info().fallback_trajectory();
  fallback_trajectory_pair_candidate.first.clear();
  double relative_time = 0.0;
  for (size_t i = 0; i < kStopTrajectorySize; ++i) {
    TrajectoryPoint point;
    point.mutable_path_point()->set_x(frame_->vehicle_state().x());
    point.mutable_path_point()->set_y(frame_->vehicle_state().y());
    point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(0.0);
    point.set_relative_time(relative_time);
    point.set_v(0.0);
    point.set_a(0.0);
    fallback_trajectory_pair_candidate.first.AppendTrajectoryPoint(point);
    relative_time += kRelativetime;
  }
  auto& trajectory = frame_->open_space_info().fallback_trajectory().first;
  auto& gear = frame_->open_space_info().fallback_trajectory().second;
  PublishableTrajectory publishable_trajectory(Clock::NowInSeconds(),
                                               trajectory);
  auto publishable_traj_and_gear =
      std::make_pair(std::move(publishable_trajectory), gear);

  *(frame_->mutable_open_space_info()->mutable_publishable_trajectory_data()) =
      std::move(publishable_traj_and_gear);
}

}  // namespace traffic_light_teb
}  // namespace scenario
}  // namespace planning
}  // namespace century
