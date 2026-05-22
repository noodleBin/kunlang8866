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

#include "modules/planning/tasks/deciders/speed_bounds_decider/st_boundary_mapper.h"

#include <array>

#include "gmock/gmock.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/reference_line/qp_spline_reference_line_smoother.h"
#include "modules/planning/tasks/deciders/speed_bounds_decider/speed_limit_decider.h"

namespace century {
namespace planning {
using century::common::VehicleConfigHelper;
using century::common::math::Box2d;
using century::common::math::Vec2d;
using century::planning_internal::PathPointsForTest;
using century::planning_internal::STPointsForTest;

namespace {
constexpr double kPlanningDistance = 70.0;
constexpr double kDifferBuffer = 1.0e-3;
constexpr double kStBoundaryEpsilon = 1e-9;
constexpr double kMinDeltaT = 1e-6;
}  // namespace
class StBoundaryMapperTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    hdmap_.LoadMapFromFile(map_file);
    const std::string lane_id = "1_-1";
    lane_info_ptr = hdmap_.GetLaneById(hdmap::MakeMapId(lane_id));
    if (!lane_info_ptr) {
      AERROR << "failed to find lane " << lane_id << " from map " << map_file;
      return;
    }
    ReferenceLineSmootherConfig config;

    injector_ = std::make_shared<DependencyInjector>();

    auto* planning_status = injector_->planning_context()
                                ->mutable_planning_status()
                                ->mutable_change_lane();
    planning_status->set_status(ChangeLaneStatus::CHANGE_LANE_FINISHED);
    injector_->is_in_play_street = false;
    injector_->vehicle_state()->set_linear_velocity(1.0);
    injector_->is_adc_in_junction_ = false;

    vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();

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
    reference_line_.reset(new ReferenceLine(ref_points));
    vehicle_position_ = points[0];

    path_data_.SetReferenceLine(reference_line_.get());

    std::vector<common::FrenetFramePoint> ff_points;
    for (int i = 0; i < 100; ++i) {
      common::FrenetFramePoint ff_point;
      ff_point.set_s(i * 1.0);
      ff_point.set_l(0.1);
      ff_points.push_back(std::move(ff_point));
    }
    frenet_frame_path_ = FrenetFramePath(std::move(ff_points));
    path_data_.SetFrenetPath(std::move(frenet_frame_path_));
    double adc_center_point_s = 30.0;
    double adc_center_point_l = 0.0;
    adc_sl_boundary_.set_start_s(adc_center_point_s -
                                 vehicle_param_.back_edge_to_center());
    adc_sl_boundary_.set_end_s(adc_center_point_s +
                               vehicle_param_.front_edge_to_center());
    adc_sl_boundary_.set_start_l(adc_center_point_l -
                                 vehicle_param_.right_edge_to_center());
    adc_sl_boundary_.set_end_l(adc_center_point_l +
                               vehicle_param_.left_edge_to_center());

    PlanningConfig planning_config;
    const std::string planning_config_file =
        "/century/modules/planning/conf/planning_config.pb.txt";
    cyber::common::GetProtoFromFile(planning_config_file, &planning_config);

    for (const auto& cfg : planning_config.default_task_config()) {
      if (cfg.task_type() == TaskConfig::SPEED_BOUNDS_PRIORI_DECIDER) {
        config_ = cfg.speed_bounds_decider_config();
        break;
      }
    }
  }

 protected:
  const std::string map_file =
      "modules/planning/testdata/garage_map/base_map.txt";
  hdmap::HDMap hdmap_;
  common::math::Vec2d vehicle_position_;
  std::unique_ptr<ReferenceLine> reference_line_;
  hdmap::LaneInfoConstPtr lane_info_ptr = nullptr;
  PathData path_data_;
  FrenetFramePath frenet_frame_path_;
  SpeedLimit speed_limit_;
  std::shared_ptr<DependencyInjector> injector_;
  SLBoundary adc_sl_boundary_;
  common::VehicleParam vehicle_param_;
  SpeedBoundsDeciderConfig config_;
  ReferenceLineInfo* reference_line_info_;
};

TEST_F(StBoundaryMapperTest, check_overlap_test) {
  common::PathPoint path_point;
  path_point.set_x(1.0);
  path_point.set_y(1.0);
  common::math::Box2d box(common::math::Vec2d(1.0, 1.0), 0.0, 5.0, 3.0);
  Frame* frame_ = nullptr;
  STBoundaryMapper mapper(adc_sl_boundary_, config_, *reference_line_,
                          path_data_, speed_limit_, kPlanningDistance,
                          config_.total_time(), injector_, reference_line_info_,
                          frame_);
  EXPECT_TRUE(mapper.CheckOverlap(path_point, box));
}

TEST_F(StBoundaryMapperTest, get_overlap_boundary_points) {
  PathPointsForTest path_points_data;
  prediction::PredictionObstacles prediction_obs_data;
  std::vector<STPoint> lower_points, upper_points, lower_points_ref,
      upper_points_ref;
  STPointsForTest st_points_lower, st_points_upper;
  Frame* frame_ = nullptr;
  STBoundaryMapper mapper(adc_sl_boundary_, config_, *reference_line_,
                          path_data_, speed_limit_, kPlanningDistance,
                          config_.total_time(), injector_, reference_line_info_,
                          frame_);
  std::vector<std::string> test_case_names = {"twice_collision_0",
                                              "twice_collision_1"};
  const std::string dir =
      "/century/modules/planning/testdata/task/speed_bounds_decider/";
  const std::string test_data = "overlap_test_data/",
                    test_result = "overlap_test_result/", suffix = ".pb.txt";

  for (const auto& case_name : test_case_names) {
    const std::string path_points_name =
        dir + test_data + "path_points_" + case_name + suffix;
    const std::string prediction_obstacle_name =
        dir + test_data + "prediction_obstacle_" + case_name + suffix;
    const std::string lower_points_name =
        dir + test_result + "lower_points_" + case_name + suffix;
    const std::string upper_points_name =
        dir + test_result + "upper_points_" + case_name + suffix;
    ASSERT_TRUE(
        cyber::common::GetProtoFromFile(path_points_name, &path_points_data))
        << "failed to read: " << path_points_name;
    ASSERT_TRUE(cyber::common::GetProtoFromFile(prediction_obstacle_name,
                                                &prediction_obs_data))
        << "failed to read: " << prediction_obstacle_name;
    ASSERT_TRUE(
        cyber::common::GetProtoFromFile(lower_points_name, &st_points_lower))
        << "failed to read: " << lower_points_name;
    ASSERT_TRUE(
        cyber::common::GetProtoFromFile(upper_points_name, &st_points_upper))
        << "failed to read: " << upper_points_name;
    ASSERT_EQ(st_points_upper.st_point_size(), st_points_lower.st_point_size())
        << "failed case name: " << case_name;
    std::vector<century::common::PathPoint> path_points;
    for (const auto& item : path_points_data.path_point()) {
      path_points.push_back(std::move(item));
    }
    auto prediction_obstacles = Obstacle::CreateObstacles(prediction_obs_data);
    std::unique_ptr<Obstacle>& obstacle = prediction_obstacles.front();
    lower_points.clear();
    upper_points.clear();
    lower_points_ref.clear();
    upper_points_ref.clear();
    std::vector<size_t> begin_overlap_position_in_st_points;
    std::vector<size_t> end_overlap_position_in_st_points;
    mapper.GetOverlapBoundaryPoints(
        path_points, *obstacle, &begin_overlap_position_in_st_points,
        &end_overlap_position_in_st_points, &upper_points, &lower_points);
    ASSERT_EQ(lower_points.size(), upper_points.size());

    for (const auto& item : st_points_lower.st_point()) {
      lower_points_ref.emplace_back(item.y(), item.x());
    }
    for (const auto& item : st_points_upper.st_point()) {
      upper_points_ref.emplace_back(item.y(), item.x());
    }
    const auto boundary_real =
        STBoundary::CreateInstance(lower_points, upper_points);
    const auto boundary_ref =
        STBoundary::CreateInstance(lower_points_ref, upper_points_ref);

    const std::array<std::string, 4UL> points_name = {
        "bottom_left_point", "upper_left_point", "bottom_right_point",
        "upper_right_point"};
    const std::array<double, 4UL> point_s_real = {
        boundary_real.bottom_left_point().s(),
        boundary_real.upper_left_point().s(),
        boundary_real.bottom_right_point().s(),
        boundary_real.upper_right_point().s()};
    const std::array<double, 4UL> point_s_ref = {
        boundary_ref.bottom_left_point().s(),
        boundary_ref.upper_left_point().s(),
        boundary_ref.bottom_right_point().s(),
        boundary_ref.upper_right_point().s()};
    const std::array<double, 4UL> point_t_real = {
        boundary_real.bottom_left_point().t(),
        boundary_real.upper_left_point().t(),
        boundary_real.bottom_right_point().t(),
        boundary_real.upper_right_point().t()};
    const std::array<double, 4UL> point_t_ref = {
        boundary_ref.bottom_left_point().t(),
        boundary_ref.upper_left_point().t(),
        boundary_ref.bottom_right_point().t(),
        boundary_ref.upper_right_point().t()};

    double diff_s = 0.0;
    double diff_t = 0.0;
    for (size_t i = 0; i < 4UL; ++i) {
      diff_s = std::abs(point_s_real[i] - point_s_ref[i]);
      EXPECT_LE(diff_s, kDifferBuffer)
          << "case name:" << case_name << ", " << points_name[i]
          << " S are differ" << std::endl;
      diff_t = std::abs(point_t_real[i] - point_t_ref[i]);
      EXPECT_LE(diff_t, kDifferBuffer)
          << "case name:" << case_name << ", " << points_name[i]
          << " T are differ" << std::endl;
    }
  }
}

}  // namespace planning
}  // namespace century
