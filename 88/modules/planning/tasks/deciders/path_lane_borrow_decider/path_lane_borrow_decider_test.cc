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
#include "modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_decider.h"

#include "gtest/gtest.h"

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/planning/common/obstacle.h"

namespace century {
namespace planning {

class PathLaneBorrowDeciderTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_task_type(TaskConfig::PATH_LANE_BORROW_DECIDER);
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

    localization.mutable_pose()->mutable_position()->set_x(
        vehicle_position.x());
    localization.mutable_pose()->mutable_position()->set_y(
        vehicle_position.y());
    localization.mutable_pose()->mutable_angular_velocity()->set_x(0.0);
    localization.mutable_pose()->mutable_angular_velocity()->set_y(0.0);
    localization.mutable_pose()->mutable_angular_velocity()->set_z(0.0);
    localization.mutable_pose()->mutable_linear_acceleration()->set_x(0.0);
    localization.mutable_pose()->mutable_linear_acceleration()->set_y(0.0);
    localization.mutable_pose()->mutable_linear_acceleration()->set_z(0.0);
    injector_->vehicle_state()->Update(localization, chassis);

    state.set_x(point.x());
    state.set_y(point.y());
    state.set_z(point.z());

    obstacles_.clear();

    m_path_lane_borrow_decider =
        std::make_unique<PathLaneBorrowDecider>(config_, injector_);
  }

  virtual void TearDown() {}

 protected:
  common::TrajectoryPoint start_point;
  hdmap::RouteSegments segments;
  common::VehicleState state;
  LocalView local_view;
  canbus::Chassis chassis;
  localization::LocalizationEstimate localization;
  hdmap::HDMap hdmap;
  common::math::Vec2d vehicle_position;

  TaskConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
  std::unique_ptr<Frame> frame_ = nullptr;
  std::unique_ptr<ReferenceLineInfo> reference_line_info_ = nullptr;
  std::vector<const Obstacle*> obstacles_;

  std::unique_ptr<ReferenceLine> reference_line;
  hdmap::LaneInfoConstPtr lane_info_ptr = nullptr;
  const std::string map_file =
      "/century/modules/planning/testdata/garage_map/base_map.txt";

  std::unique_ptr<PathLaneBorrowDecider> m_path_lane_borrow_decider;
};

TEST_F(PathLaneBorrowDeciderTest, Init) {
  EXPECT_EQ(m_path_lane_borrow_decider->Name(),
            TaskConfig::TaskType_Name(config_.task_type()));
}

TEST_F(PathLaneBorrowDeciderTest, IsNecessaryToBorrowLane_PlayStreet_OnLane) {
  /// start of Setting reference_line_info_, frame_, injector_
  reference_line_info_ = std::make_unique<ReferenceLineInfo>(
      state, start_point, *reference_line, segments);
  reference_line_info_->Init(obstacles_);
  frame_ = std::make_unique<Frame>(1, local_view, start_point, state, nullptr);
  // injector_->vehicle_state()->Update(localization, chassis);
  /// end of Setting reference_line_info_, frame_, injector_

  auto* mutable_path_decider_status =
      m_path_lane_borrow_decider->injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_path_decider();
  reference_line_info_->setLaneType(hdmap::Lane::PLAY_STREET);
  mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
  mutable_path_decider_status->clear_decided_side_pass_direction();
  mutable_path_decider_status->add_decided_side_pass_direction(
      PathDeciderStatus::LEFT_BORROW);

  auto result = m_path_lane_borrow_decider->IsNecessaryToBorrowLane(
      *frame_, *reference_line_info_);
  EXPECT_EQ(1,
            mutable_path_decider_status->decided_side_pass_direction().size());
  EXPECT_EQ(true, result);
  EXPECT_EQ(true,
            mutable_path_decider_status->is_in_path_lane_borrow_scenario());
}

TEST_F(PathLaneBorrowDeciderTest, IsNecessaryToBorrowLane_PlayStreet_OffLane) {
  /// start of Setting reference_line_info_, frame_, injector_
  auto VehPos = common::math::Vec2d(vehicle_position.x() + 3.0,
                                    vehicle_position.y() + 3.0);
  start_point.mutable_path_point()->set_x(VehPos.x());
  start_point.mutable_path_point()->set_y(VehPos.y());
  state.set_x(VehPos.x());
  state.set_y(VehPos.x());
  localization.mutable_pose()->mutable_position()->set_x(VehPos.x());
  localization.mutable_pose()->mutable_position()->set_y(VehPos.y());

  reference_line_info_ = std::make_unique<ReferenceLineInfo>(
      state, start_point, *reference_line, segments);
  reference_line_info_->Init(obstacles_);
  frame_ = std::make_unique<Frame>(1, local_view, start_point, state, nullptr);
  injector_->vehicle_state()->Update(localization, chassis);
  /// end of Setting reference_line_info_, frame_, injector_

  auto* mutable_path_decider_status =
      m_path_lane_borrow_decider->injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_path_decider();

  reference_line_info_->setLaneType(hdmap::Lane::PLAY_STREET);
  mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
  mutable_path_decider_status->clear_decided_side_pass_direction();

  auto result = m_path_lane_borrow_decider->IsNecessaryToBorrowLane(
      *frame_, *reference_line_info_);
  EXPECT_EQ(1,
            mutable_path_decider_status->decided_side_pass_direction().size());
  EXPECT_EQ(true, result);
  EXPECT_EQ(true,
            mutable_path_decider_status->is_in_path_lane_borrow_scenario());
}

TEST_F(PathLaneBorrowDeciderTest, IsNecessaryToBorrowLane_TrueScenario1) {
  /// start of Setting reference_line_info_, frame_, injector_
  reference_line_info_ = std::make_unique<ReferenceLineInfo>(
      state, start_point, *reference_line, segments);
  reference_line_info_->Init(obstacles_);
  frame_ = std::make_unique<Frame>(1, local_view, start_point, state, nullptr);
  injector_->vehicle_state()->Update(localization, chassis);
  /// end of Setting reference_line_info_, frame_, injector_

  auto* mutable_path_decider_status =
      m_path_lane_borrow_decider->injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_path_decider();
  reference_line_info_->setLaneType(hdmap::Lane::CITY_DRIVING);
  mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
  mutable_path_decider_status->clear_decided_side_pass_direction();
  mutable_path_decider_status->add_decided_side_pass_direction(
      PathDeciderStatus::LEFT_BORROW);
  mutable_path_decider_status->add_decided_side_pass_direction(
      PathDeciderStatus::RIGHT_BORROW);

  auto result = m_path_lane_borrow_decider->IsNecessaryToBorrowLane(
      *frame_, *reference_line_info_);
//   EXPECT_EQ(2,
//             mutable_path_decider_status->decided_side_pass_direction().size());
  EXPECT_EQ(true, result);
  EXPECT_EQ(true,
            mutable_path_decider_status->is_in_path_lane_borrow_scenario());
}

TEST_F(PathLaneBorrowDeciderTest, IsNecessaryToBorrowLane_TrueScenario2) {
  /// start of Setting reference_line_info_, frame_, injector_
  reference_line_info_ = std::make_unique<ReferenceLineInfo>(
      state, start_point, *reference_line, segments);
  reference_line_info_->Init(obstacles_);
  frame_ = std::make_unique<Frame>(1, local_view, start_point, state, nullptr);
  injector_->vehicle_state()->Update(localization, chassis);
  /// end of Setting reference_line_info_, frame_, injector_

  auto* mutable_path_decider_status =
      m_path_lane_borrow_decider->injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_path_decider();
  reference_line_info_->setLaneType(hdmap::Lane::CITY_DRIVING);
  mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
  mutable_path_decider_status->clear_decided_side_pass_direction();
  mutable_path_decider_status->add_decided_side_pass_direction(
      PathDeciderStatus::LEFT_BORROW);
  mutable_path_decider_status->add_decided_side_pass_direction(
      PathDeciderStatus::RIGHT_BORROW);

  auto result = m_path_lane_borrow_decider->IsNecessaryToBorrowLane(
      *frame_, *reference_line_info_);
//   EXPECT_EQ(0,
//             mutable_path_decider_status->decided_side_pass_direction().size());
  EXPECT_EQ(true, result);
  EXPECT_EQ(true,
            mutable_path_decider_status->is_in_path_lane_borrow_scenario());
}

TEST_F(PathLaneBorrowDeciderTest, IsNecessaryToBorrowLane_TrueScenario3) {
  /// start of Setting reference_line_info_, frame_, injector_
  reference_line_info_ = std::make_unique<ReferenceLineInfo>(
      state, start_point, *reference_line, segments);
  reference_line_info_->Init(obstacles_);
  frame_ = std::make_unique<Frame>(1, local_view, start_point, state, nullptr);
  injector_->vehicle_state()->Update(localization, chassis);
  /// end of Setting reference_line_info_, frame_, injector_

  auto* mutable_path_decider_status =
      m_path_lane_borrow_decider->injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_path_decider();
  reference_line_info_->setLaneType(hdmap::Lane::CITY_DRIVING);
  mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
  mutable_path_decider_status->clear_decided_side_pass_direction();
  mutable_path_decider_status->add_decided_side_pass_direction(
      PathDeciderStatus::LEFT_BORROW);
  mutable_path_decider_status->add_decided_side_pass_direction(
      PathDeciderStatus::RIGHT_BORROW);
  mutable_path_decider_status->set_able_to_use_self_lane_counter(6);

  auto result = m_path_lane_borrow_decider->IsNecessaryToBorrowLane(
      *frame_, *reference_line_info_);
  EXPECT_EQ(0,
            mutable_path_decider_status->decided_side_pass_direction().size());
  EXPECT_EQ(false, result);
  EXPECT_EQ(false,
            mutable_path_decider_status->is_in_path_lane_borrow_scenario());
}

}  // namespace planning
}  // namespace century
