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

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/planning/scenarios/rescue/rescue_scenario.h"
#include "modules/planning/scenarios/stage.h"

namespace century {
namespace planning {
namespace scenario {
namespace rescue {

struct RescueContext;

class RescueStagePreCruise : public Stage {
 public:
  RescueStagePreCruise(const ScenarioConfig::StageConfig& config,
                       const std::shared_ptr<DependencyInjector>& injector)
      : Stage(config, injector) {}

  Stage::StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                             Frame* frame) override;

  RescueContext* GetContext() { return Stage::GetContextAs<RescueContext>(); }

  Stage::StageStatus FinishStage();

 private:
  ScenarioRescueConfig scenario_config_;
};

}  // namespace rescue
}  // namespace scenario
}  // namespace planning
}  // namespace century
