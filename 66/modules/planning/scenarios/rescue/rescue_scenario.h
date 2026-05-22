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
 * @file rescue_scenario.h
 */
#pragma once

#include <memory>

#include "modules/planning/scenarios/scenario.h"

namespace century {
namespace planning {
namespace scenario {
namespace rescue {

// stage context
struct RescueContext {
  ScenarioRescueConfig scenario_config;
  // rescue pose and status
  common::math::Vec2d rescue_origin_point;
  double rescue_origin_heading = 0.0;
  common::math::Vec2d rescue_end_point;
  double rescue_end_heading = 0.0;
  bool has_end_pose_flag = false;
  bool replan_flag = false;
  int32_t replan_count = 60;
};

class RescueScenario : public Scenario {
 public:
  RescueScenario(const ScenarioConfig& config, const ScenarioContext* context,
                 const std::shared_ptr<DependencyInjector>& injector)
      : Scenario(config, context, injector) {}

  void Init() override;

  std::unique_ptr<Stage> CreateStage(
      const ScenarioConfig::StageConfig& stage_config,
      const std::shared_ptr<DependencyInjector>& injector) override;

  RescueContext* GetContext() { return &context_; }

 private:
  static void RegisterStages();
  bool GetScenarioConfig();

 private:
  static century::common::util::Factory<
      ScenarioConfig::StageType, Stage,
      Stage* (*)(const ScenarioConfig::StageConfig& stage_config,
                 const std::shared_ptr<DependencyInjector>& injector)>
      s_stage_factory_;
  bool init_ = false;
  RescueContext context_;
};

}  // namespace rescue
}  // namespace scenario
}  // namespace planning
}  // namespace century
