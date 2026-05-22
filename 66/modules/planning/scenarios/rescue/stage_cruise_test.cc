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
 * @file stage_cruise_test.cc
 */

#include "modules/planning/scenarios/rescue/stage_cruise.h"

#include "gtest/gtest.h"

#include "modules/planning/proto/planning_config.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace century {
namespace planning {
namespace scenario {
namespace rescue {

class RescueStageCruiseTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_stage_type(ScenarioConfig::RESCUE_CRUISE);
    injector_ = std::make_shared<DependencyInjector>();
  }

 protected:
  ScenarioConfig::StageConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
};

TEST_F(RescueStageCruiseTest, Init) {
  RescueStageCruise rescue_stage_cruise(config_, injector_);
  EXPECT_EQ(rescue_stage_cruise.Name(),
            ScenarioConfig::StageType_Name(ScenarioConfig::RESCUE_CRUISE));
}

}  // namespace rescue
}  // namespace scenario
}  // namespace planning
}  // namespace century
