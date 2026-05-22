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

#include "modules/planning/scenarios/rescue/rescue_scenario.h"

#include "cyber/common/log.h"
#include "modules/planning/scenarios/rescue/stage_adjust.h"
#include "modules/planning/scenarios/rescue/stage_cruise.h"
#include "modules/planning/scenarios/rescue/stage_pre_cruise.h"

namespace century {
namespace planning {
namespace scenario {
namespace rescue {

century::common::util::Factory<
    ScenarioConfig::StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config,
               const std::shared_ptr<DependencyInjector>& injector)>
    RescueScenario::s_stage_factory_;

void RescueScenario::Init() {
  if (init_) {
    return;
  }

  Scenario::Init();

  if (!GetScenarioConfig()) {
    AERROR << "fail to get scenario specific config";
    return;
  }

  init_ = true;
}

void RescueScenario::RegisterStages() {
  if (!s_stage_factory_.Empty()) {
    s_stage_factory_.Clear();
  }
  s_stage_factory_.Register(
      ScenarioConfig::RESCUE_ADJUST,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new RescueStageAdjust(config, injector);
      });
  s_stage_factory_.Register(
      ScenarioConfig::RESCUE_PRE_CRUISE,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new RescueStagePreCruise(config, injector);
      });
  s_stage_factory_.Register(
      ScenarioConfig::RESCUE_CRUISE,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new RescueStageCruise(config, injector);
      });
}

std::unique_ptr<Stage> RescueScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (s_stage_factory_.Empty()) {
    RegisterStages();
  }
  auto ptr = s_stage_factory_.CreateObjectOrNull(stage_config.stage_type(),
                                                 stage_config, injector);
  if (ptr) {
    ptr->SetContext(&context_);
  }
  return ptr;
}

bool RescueScenario::GetScenarioConfig() {
  if (!config_.has_rescue_config()) {
    AERROR << "miss scenario specific config";
    return false;
  }
  context_.scenario_config.CopyFrom(config_.rescue_config());
  return true;
}

}  // namespace rescue
}  // namespace scenario
}  // namespace planning
}  // namespace century
