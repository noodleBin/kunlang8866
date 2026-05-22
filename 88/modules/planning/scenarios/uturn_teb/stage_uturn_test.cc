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
#include "modules/planning/scenarios/uturn_teb/stage_uturn.h"

#include "gtest/gtest.h"

#include "modules/planning/proto/planning_config.pb.h"

namespace century {
namespace planning {
namespace scenario {
namespace uturn {

class UturnStageTebTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_stage_type(ScenarioConfig::UTURN_TEB_ADJUST);
    injector_ = std::make_shared<DependencyInjector>();
  }

 protected:
  ScenarioConfig::StageConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
};

TEST_F(UturnStageTebTest, Init) {
  UturnStageTeb uturn_stage_adjust(config_, injector_);
  EXPECT_EQ(uturn_stage_adjust.Name(),
            ScenarioConfig::StageType_Name(ScenarioConfig::UTURN_TEB_ADJUST));
}

}  // namespace uturn
}  // namespace scenario
}  // namespace planning
}  // namespace century
