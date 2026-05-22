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

#include "modules/planning/scenarios/rescue_teb/rescue_teb_scenario.h"

#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace century {
namespace planning {
namespace scenario {
namespace rescue {

class RescueTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  std::unique_ptr<RescueTebScenario> scenario_;
};

TEST_F(RescueTest, VerifyConf) {
  ScenarioConfig config;
  EXPECT_TRUE(century::cyber::common::GetProtoFromFile(
      FLAGS_scenario_rescue_teb_config_file, &config));
}

}  // namespace rescue
}  // namespace scenario
}  // namespace planning
}  // namespace century
