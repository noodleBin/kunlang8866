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
 * @file path_time_heuristic_optimizer.cc
 **/

#include "modules/planning/tasks/optimizers/path_time_heuristic/path_time_heuristic_optimizer.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/tasks/optimizers/path_time_heuristic/gridded_path_time_graph.h"

namespace century {
namespace planning {

using century::common::ErrorCode;
using century::common::Status;

namespace {
constexpr double kMinDecel = -6.0;
}  // namespace

PathTimeHeuristicOptimizer::PathTimeHeuristicOptimizer(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : SpeedOptimizer(config, injector) {
  ACHECK(config.has_speed_heuristic_optimizer_config());
  speed_heuristic_optimizer_config_ = config.speed_heuristic_optimizer_config();
}

Status PathTimeHeuristicOptimizer::SearchPathTimeGraph(
    SpeedData* speed_data) const {
  const auto& dp_st_speed_optimizer_config =
      reference_line_info_->IsChangeLanePath()
          ? speed_heuristic_optimizer_config_.lane_change_speed_config()
          : speed_heuristic_optimizer_config_.default_speed_config();

  GriddedPathTimeGraph st_graph(
      reference_line_info_->st_graph_data(), dp_st_speed_optimizer_config,
      *reference_line_info_->path_decision(), init_point_, injector_);
  double min_stop_decel = kMinDecel;
  double min_stop_s = 0.0;
  bool is_need_slow_breaking =
      reference_line_info_->IsNeedSlowBreaking(&min_stop_decel, &min_stop_s);
  return st_graph.Search(is_need_slow_breaking, speed_data);
}

Status PathTimeHeuristicOptimizer::Process(
    const PathData& path_data, const common::TrajectoryPoint& init_point,
    SpeedData* const speed_data) {
  init_point_ = init_point;

  if (path_data.discretized_path().empty()) {
    const std::string msg = "Empty path data";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  const Status ret = SearchPathTimeGraph(speed_data);
  if (!ret.ok()) {
    const std::string msg = absl::StrCat(
        Name(), ": Failed to search graph with dynamic programming.");
    AERROR << msg;
    RecordDebugInfo(*speed_data, reference_line_info_->mutable_st_graph_data()
                                     ->mutable_st_graph_debug());
    return Status(ErrorCode::PLANNING_ERROR, ret.error_message());
  }
  RecordDebugInfo(
      *speed_data,
      reference_line_info_->mutable_st_graph_data()->mutable_st_graph_debug());
  return Status::OK();
}

}  // namespace planning
}  // namespace century
