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

#include "modules/planning/tasks/optimizers/piecewise_jerk_path/piecewise_jerk_path_optimizer.h"

#include "gtest/gtest.h"

#include "modules/planning/proto/planning_config.pb.h"

namespace century {
namespace planning {

class PiecewiseJerkPathOptimizerTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_task_type(TaskConfig::PIECEWISE_JERK_PATH_OPTIMIZER);
    config_.mutable_piecewise_jerk_path_optimizer_config();
    injector_ = std::make_shared<DependencyInjector>();
    hdmap_ = std::make_shared<hdmap::HDMap>();

    const std::string map_file =
        "/century/modules/planning/testdata/garage_map/base_map.txt";
    hdmap_->LoadMapFromFile(map_file);
    const std::string lane_id = "1_-1";
    hdmap::LaneInfoConstPtr lane_info_ptr =
        hdmap_->GetLaneById(hdmap::MakeMapId(lane_id));
    if (!lane_info_ptr) {
      AERROR << "failed to find lane " << lane_id << " from map " << map_file;
      return;
    }

    std::vector<ReferencePoint> ref_points;
    const auto& points = lane_info_ptr->points();
    const auto& headings = lane_info_ptr->headings();
    const auto& accumulate_s = lane_info_ptr->accumulate_s();
    for (size_t i = 0; i < points.size(); ++i) {
      std::vector<hdmap::LaneWaypoint> waypoint;
      waypoint.emplace_back(lane_info_ptr, accumulate_s[i]);
      hdmap::MapPathPoint map_path_point(points[i], headings[i], waypoint);
      ref_points.emplace_back(map_path_point, 0.0, 0.0);
    }
    std::shared_ptr<ReferenceLine> reference_line =
        std::make_unique<ReferenceLine>(ref_points);

    common::math::Vec2d vehicle_position;
    vehicle_position = points[50];
    ADEBUG << "vehicle_position X: " << setiosflags(std::ios::fixed)
           << vehicle_position.x();
    ADEBUG << "vehicle_position Y: " << setiosflags(std::ios::fixed)
           << vehicle_position.y();

    common::TrajectoryPoint start_point;
    start_point.mutable_path_point()->set_x(vehicle_position.x());
    start_point.mutable_path_point()->set_y(vehicle_position.y());

    common::PointENU point;
    point.set_x(vehicle_position.x());
    point.set_y(vehicle_position.y());

    common::VehicleState state;
    state.set_x(point.x());
    state.set_y(point.y());
    state.set_z(point.z());

    LocalView local_view;
    frame_ =
        std::make_shared<Frame>(1, local_view, start_point, state, nullptr);

    hdmap::RouteSegments segments;
    reference_line_info_ = std::make_shared<ReferenceLineInfo>(
        state, start_point, *reference_line, segments);
  }

  virtual void TearDown() {}

 protected:
  TaskConfig config_;
  std::shared_ptr<DependencyInjector> injector_;

  std::shared_ptr<hdmap::HDMap> hdmap_ = nullptr;
  std::shared_ptr<Frame> frame_ = nullptr;
  std::shared_ptr<ReferenceLineInfo> reference_line_info_ = nullptr;
};

TEST_F(PiecewiseJerkPathOptimizerTest, Name) {
  PiecewiseJerkPathOptimizer piecewise_jerk_path_optimizer(config_, injector_);
  EXPECT_EQ(piecewise_jerk_path_optimizer.Name(),
            TaskConfig::TaskType_Name(config_.task_type()));
}

}  // namespace planning
}  // namespace century
