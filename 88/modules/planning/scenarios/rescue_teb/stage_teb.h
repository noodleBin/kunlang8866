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

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/planning/scenarios/rescue_teb/rescue_teb_scenario.h"
#include "modules/planning/scenarios/stage.h"

namespace century {
namespace planning {
namespace scenario {
namespace rescue {
struct CheckData {
  double stamp;
  bool has_speed;
};
class RescueStageTeb : public Stage {
 public:
  RescueStageTeb(const ScenarioConfig::StageConfig& config,
                 const std::shared_ptr<DependencyInjector>& injector);

  Stage::StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                             Frame* frame) override;

  RescueTebContext* GetContext() {
    return Stage::GetContextAs<RescueTebContext>();
  }

  Stage::StageStatus FinishStage();

  bool CheckReachTrajectoryEnd(
      const common::TrajectoryPoint& planning_init_point);

  void GenerateStopTrajectory();

  int GetFallBackCount();

  bool FirstInProcessLogic();

  bool CheckReferenceLineBlock();

  bool InGateOrReachGoalOrOverTime();

  bool PullOverRunning();

  bool CheckReachGoal();

  bool Ready2Cruise();

  bool CheckVehicleLongStop(Frame* frame);
  bool StopLongTimeReportAndExit(Frame* frame);
  bool TaskFailedReportAndExit(const bool& task_failed, Frame* frame);

  bool CalVehicleIsOutRoad(Frame* frame);
  void ClearDataThread();
  void CalculateFirstIntoTEB();
  void SetFrame(Frame* frame) { frame_ = frame; }
  void SetInjector(const std::shared_ptr<DependencyInjector>& injector) {
    injector_ = injector;
  }
  void SetsScenarioConfig(ScenarioRescueConfig scenario_config) {
    scenario_config_ = scenario_config;
  }

 private:
  void ResetInitPostion();
  ScenarioRescueConfig scenario_config_;
  double openspace_success_time_;
  century::planning::RescueStatus* rescue_status_;
  century::planning::Frame* frame_;

  int count_ = 0;
  int ready_count_ = 0;
  bool is_back_traj_ = false;
  std::deque<CheckData> vehicle_speed_deque_;
  int error_cnt_ = 0;  // error count for the rescue, used for remote driving
};

}  // namespace rescue
}  // namespace scenario
}  // namespace planning
}  // namespace century
