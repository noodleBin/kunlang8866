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
#include "modules/planning/tasks/deciders/lane_change_decider/lane_change_decider.h"

#include "gtest/gtest.h"

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/util/util.h"

namespace century {

namespace planning {

class LaneChangeDeciderTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_task_type(TaskConfig::LANE_CHANGE_DECIDER);
    config_.mutable_lane_change_decider_config();
    injector_ = std::make_shared<DependencyInjector>();
    hdmap.LoadMapFromFile(map_file);
    const std::string lane_id = "1_-1";
    lane_info_ptr = hdmap.GetLaneById(hdmap::MakeMapId(lane_id));
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
    reference_line = std::make_unique<ReferenceLine>(ref_points);
    vehicle_position = points[50];
    ADEBUG << "vehicle_position X: " << setiosflags(std::ios::fixed)
           << vehicle_position.x();
    ADEBUG << "vehicle_position Y: " << setiosflags(std::ios::fixed)
           << vehicle_position.y();
    start_point.mutable_path_point()->set_x(vehicle_position.x());
    start_point.mutable_path_point()->set_y(vehicle_position.y());

    common::PointENU point;
    point.set_x(vehicle_position.x());
    point.set_y(vehicle_position.y());

    state.set_x(point.x());
    state.set_y(point.y());
    state.set_z(point.z());
  }
  virtual void TearDown() {}

 public:
  common::VehicleState state;
  common::TrajectoryPoint start_point;
  PlanningContext* context_;
  PathDecision path_decision_;
  TaskConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
  hdmap::RouteSegments segments;
  std::unique_ptr<ReferenceLineInfo> reference_line_info_ = nullptr;
  const std::string map_file =
      "/century/modules/planning/testdata/garage_map/base_map.txt";
  hdmap::HDMap hdmap;
  hdmap::LaneInfoConstPtr lane_info_ptr = nullptr;
  std::unique_ptr<ReferenceLine> reference_line;
  common::math::Vec2d vehicle_position;
};

TEST_F(LaneChangeDeciderTest, Init) {
  LaneChangeDecider lane_change_decider(config_, injector_);
  EXPECT_EQ(lane_change_decider.Name(),
            TaskConfig::TaskType_Name(config_.task_type()));
}

TEST_F(LaneChangeDeciderTest, IsChangeLanePath_sc1) {
  reference_line_info_ = std::make_unique<ReferenceLineInfo>(
      state, start_point, *reference_line, segments);
  EXPECT_EQ(true, reference_line_info_->IsChangeLanePath());
}

}  // namespace planning
}  // namespace century
