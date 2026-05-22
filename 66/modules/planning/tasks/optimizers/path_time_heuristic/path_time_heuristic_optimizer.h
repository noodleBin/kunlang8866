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
 * @file path_time_heuristic_optimizer.h
 **/

#pragma once

#include <memory>
#include <string>

#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/planning/proto/task_config.pb.h"

#include "modules/planning/tasks/optimizers/speed_optimizer.h"

namespace century {
namespace planning {

/**
 * @class PathTimeHeuristicOptimizer
 * @brief PathTimeHeuristicOptimizer does ST graph speed planning with dynamic
 * programming algorithm.
 */
class PathTimeHeuristicOptimizer : public SpeedOptimizer {
 public:
  PathTimeHeuristicOptimizer(
      const TaskConfig& config,
      const std::shared_ptr<DependencyInjector>& injector);

 private:
  common::Status Process(const PathData& path_data,
                         const common::TrajectoryPoint& init_point,
                         SpeedData* const speed_data) override;

  common::Status SearchPathTimeGraph(SpeedData* speed_data) const;

 private:
  common::TrajectoryPoint init_point_;
  SLBoundary adc_sl_boundary_;
  SpeedHeuristicOptimizerConfig speed_heuristic_optimizer_config_;
};

}  // namespace planning
}  // namespace century
