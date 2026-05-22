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

#include "modules/planning/tasks/deciders/speed_bounds_decider/speed_bounds_decider.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "modules/planning/proto/planning_config.pb.h"

#include "cyber/common/file.h"
namespace century {
namespace planning {

class SpeedBoundsDeciderTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    injector_ = std::make_shared<DependencyInjector>();

    PlanningConfig planning_config;
    const std::string planning_config_file =
        "/century/modules/planning/conf/planning_config.pb.txt";
    cyber::common::GetProtoFromFile(planning_config_file, &planning_config);

    for (const auto& cfg : planning_config.default_task_config()) {
      if (cfg.task_type() == TaskConfig::SPEED_BOUNDS_PRIORI_DECIDER) {
        task_config_.set_task_type(TaskConfig::SPEED_BOUNDS_PRIORI_DECIDER);
        auto* mutable_conf = task_config_.mutable_speed_bounds_decider_config();
        *mutable_conf = cfg.speed_bounds_decider_config();
        config_ = cfg.speed_bounds_decider_config();
        break;
      }
    }
  }

 protected:
  std::shared_ptr<DependencyInjector> injector_;
  SpeedBoundsDeciderConfig config_;
  TaskConfig task_config_;
};

TEST_F(SpeedBoundsDeciderTest, Init) {
  SpeedBoundsDecider speed_bounds_decider(task_config_, injector_);

  EXPECT_EQ(speed_bounds_decider.Name(),
            TaskConfig::TaskType_Name(task_config_.task_type()));
}

}  // namespace planning
}  // namespace century
