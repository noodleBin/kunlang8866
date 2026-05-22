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

#include "modules/planning/tasks/optimizers/piecewise_jerk_speed/piecewise_jerk_speed_optimizer.h"

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

class PiecewiseJerkSpeedOptimizerTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    injector_ = std::make_shared<DependencyInjector>();

    PlanningConfig planning_config;
    const std::string planning_config_file =
        "/century/modules/planning/conf/planning_config.pb.txt";
    cyber::common::GetProtoFromFile(planning_config_file, &planning_config);

    for (const auto& cfg : planning_config.default_task_config()) {
      if (cfg.task_type() == TaskConfig::PIECEWISE_JERK_SPEED_OPTIMIZER) {
        task_config_.set_task_type(TaskConfig::PIECEWISE_JERK_SPEED_OPTIMIZER);
        auto* mutable_conf =
            task_config_.mutable_piecewise_jerk_speed_optimizer_config();
        *mutable_conf = cfg.piecewise_jerk_speed_optimizer_config();
        break;
      }
    }
  }

 protected:
  std::shared_ptr<DependencyInjector> injector_;
  TaskConfig task_config_;
};

TEST_F(PiecewiseJerkSpeedOptimizerTest, Init) {
  PiecewiseJerkSpeedOptimizer piecewise_jerk_speed_optimizer(task_config_,
                                                             injector_);

  EXPECT_EQ(piecewise_jerk_speed_optimizer.Name(),
            TaskConfig::TaskType_Name(task_config_.task_type()));
}

}  // namespace planning
}  // namespace century
