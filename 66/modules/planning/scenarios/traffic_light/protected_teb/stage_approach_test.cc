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

#include "modules/planning/scenarios/traffic_light/protected_teb/stage_approach.h"

#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace century {
namespace planning {
namespace scenario {
namespace traffic_light_teb {

class TrafficLightProtectedStageApproachTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_stage_type(ScenarioConfig::TRAFFIC_LIGHT_PROTECTED_APPROACH);
    injector_ = std::make_shared<DependencyInjector>();
  }

 protected:
  ScenarioConfig::StageConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
};

TEST_F(TrafficLightProtectedStageApproachTest, Init) {
  TrafficLightProtectedStageApproach traffic_light_protected_stage_approach(
      config_, injector_);
  EXPECT_EQ(traffic_light_protected_stage_approach.Name(),
            ScenarioConfig::StageType_Name(
                ScenarioConfig::TRAFFIC_LIGHT_PROTECTED_APPROACH));
}

}  // namespace traffic_light_teb
}  // namespace scenario
}  // namespace planning
}  // namespace century
