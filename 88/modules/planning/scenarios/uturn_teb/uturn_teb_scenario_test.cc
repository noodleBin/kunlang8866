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

#include "modules/planning/scenarios/uturn_teb/uturn_teb_scenario.h"

#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace century {
namespace planning {
namespace scenario {
namespace uturn {

class UturnTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  std::unique_ptr<UturnTebScenario> scenario_;
};

TEST_F(UturnTest, VerifyConf) {
  ScenarioConfig config;
  EXPECT_TRUE(century::cyber::common::GetProtoFromFile(
      FLAGS_scenario_uturn_teb_config_file, &config));
}

}  // namespace uturn
}  // namespace scenario
}  // namespace planning
}  // namespace century
