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

#include "modules/planning/tasks/optimizers/teb_trajectory_generation/teb_trajectory_provider.h"

#include <algorithm>
#include <memory>
#include <string>

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"

#include "cyber/task/task.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/common/trajectory_stitcher.h"

namespace century {
namespace planning {
namespace {
constexpr double kEndPoseChangeDist = 5.0;
constexpr double kAdcInitPlanningErrorDist = 2.0;
constexpr double kEpsilon = 0.05;
constexpr int kValidTrajNum = 11;
constexpr double kValidGoalChangeS = 1.0;
constexpr int kCountNum = 5;
constexpr int kStitchPointNum = 2;
constexpr int32_t kSleepTime = 20;  // milliseconds
constexpr size_t kMaxTrajectoryErrorNum = 1000;
constexpr double kBackEnoughPath = 2.0;
constexpr int kIsEmptyIntValue = 0;  // int size

constexpr bool kTempProcessThreadFlages = false;
constexpr double kPreviewTime = 0.2;
constexpr double kFollowWellThr = 0.3;
}  // namespace
using century::common::ErrorCode;
using century::common::Status;
using century::common::TrajectoryPoint;
using century::common::math::Vec2d;
using century::cyber::Clock;

TEBTrajectoryProvider::TEBTrajectoryProvider(
    const TaskConfig &config,
    const std::shared_ptr<DependencyInjector> &injector)
    : TrajectoryOptimizer(config, injector) {
  rescue_status_ = injector_->planning_context()
                       ->mutable_planning_status()
                       ->mutable_rescue();

  teb_trajectory_optimizer_ = std::make_unique<TEBTrajectoryOptimizer>(
      config.teb_trajectory_provider_config().teb_trajectory_optimizer_config(),
      rescue_status_);
}

TEBTrajectoryProvider::~TEBTrajectoryProvider() {
  if (FLAGS_enable_teb_planner_thread || kTempProcessThreadFlages) {
    Stop();
  }
}

void TEBTrajectoryProvider::Stop() {
  if (FLAGS_enable_teb_planner_thread || kTempProcessThreadFlages) {
    is_generation_thread_stop_.store(true);
    if (thread_init_flag_) {
      task_future_.get();
    }
    trajectory_updated_.store(false);
    trajectory_error_.store(false);
    trajectory_skipped_.store(false);
    optimizer_thread_counter_ = 0;
  }
}

void TEBTrajectoryProvider::Restart() {
  if (FLAGS_enable_teb_planner_thread || kTempProcessThreadFlages) {
    is_generation_thread_stop_.store(true);
    if (thread_init_flag_) {
      task_future_.get();
    }
    is_generation_thread_stop_.store(false);
    thread_init_flag_ = false;
    trajectory_updated_.store(false);
    trajectory_error_.store(false);
    trajectory_skipped_.store(false);
    optimizer_thread_counter_ = 0;
  }
}

void TEBTrajectoryProvider::UpdateScenario() {
  const auto scenario_type = injector_->planning_context()
                                 ->planning_status()
                                 .scenario()
                                 .scenario_type();
  if (ScenarioConfig::RESCUE_TEB == scenario_type &&
      injector_->is_personlike_blocked_) {
    teb_trajectory_optimizer_->SetUseKappaContraint(true);
  } else {
    teb_trajectory_optimizer_->SetUseKappaContraint(false);
  }
}

Status TEBTrajectoryProvider::Process() {
  // process with thread
  if (((FLAGS_enable_teb_planner_thread && !injector_->is_in_near_goal_) &&
       config_.open_space_trajectory_provider_config()
           .open_space_trajectory_optimizer_config()
           .planner_open_space_config()
           .use_hybrid_thread()) ||
      kTempProcessThreadFlages) {
    AERROR << "TEBTrajectoryProvider::ProcessThread()";
    return ProcessThread();
  } else {
    AERROR << "TEBTrajectoryProvider::ProcessNormal()";
    return ProcessNormal();
  }
}

// WYQ_Mark_Thread
Status TEBTrajectoryProvider::ProcessThread() {
  // Start thread when getting in Process() for the first time
  if (!thread_init_flag_) {
    task_future_ =
        cyber::Async(&TEBTrajectoryProvider::GenerateTrajectoryThread, this);
    thread_init_flag_ = true;
    AINFO << "First enter teb, init trajectory thread.";
    is_first_enter_openspace_ = true;
  }

  // Final trajectory data to be filled
  auto trajectory_data =
      frame_->mutable_open_space_info()->mutable_stitched_trajectory_result();

  // Get stitching trajectory from last frame
  const common::VehicleState vehicle_state = frame_->vehicle_state();
  auto *previous_frame = injector_->use_thread_in_play_street()
                             ? injector_->frame_teb_history()->Latest()
                             : injector_->frame_history()->Latest();
  // The previous frame was not an open space,
  // indicating that the scene has just entered
  if (!previous_frame->open_space_info().is_on_open_space_trajectory()) {
    AINFO << "Just come from lane follow.";
    is_first_enter_openspace_ = true;
  }

  const double adc_speed = vehicle_state.linear_velocity();
  double adc_heading = vehicle_state.heading();
  // Use complete raw trajectory from last frame for stitching purpose
  std::vector<TrajectoryPoint> stitching_trajectory;
  common::TrajectoryPoint planning_init_point;
  planning_init_point.mutable_path_point()->set_x(vehicle_state.x());
  planning_init_point.mutable_path_point()->set_y(vehicle_state.y());
  planning_init_point.mutable_path_point()->set_theta(vehicle_state.heading());
  stitching_trajectory.emplace_back(planning_init_point);

  // Get open_space_info from current frame
  const auto &open_space_info = frame_->open_space_info();

  Vec2d start_point(stitching_trajectory.back().path_point().x(),
                    stitching_trajectory.back().path_point().y());

  if (previous_frame->open_space_info()
          .chosen_partitioned_trajectory()
          .first.size() > 0) {
    RecomputeStartPoint(vehicle_state, &start_point, &adc_heading);
  }

  // AINFO << "world2local | world_start_point " << start_point.DebugString();
  start_point -= open_space_info.origin_point();
  start_point.SelfRotate(-open_space_info.origin_heading());

  std::vector<double> start_pose;
  start_pose.clear();
  start_pose.emplace_back(start_point.x());
  start_pose.emplace_back(start_point.y());
  start_pose.emplace_back(adc_heading);
  start_pose.emplace_back(adc_speed);

  const auto &end_pose = open_space_info.open_space_end_pose();
  if (kIsEmptyIntValue == static_cast<int>(end_pose.size())) {
    AERROR << "Teb end_pose is empty.";
    return Status(ErrorCode::PLANNING_ERROR, "Teb end_pose is empty");
  }

  AINFO << "teb plan in multi-threads mode.";
  // Check vehicle state
  if (IsVehicleNearDestination(
          vehicle_state, open_space_info.open_space_end_pose(),
          open_space_info.origin_heading(), open_space_info.origin_point())) {
    GenerateStopTrajectory(trajectory_data);
    AINFO << "Vehicle is near to destination.";
    return Status(ErrorCode::OK, "Vehicle is near to destination");
  }

  if (is_generation_thread_stop_) {
    GenerateStopTrajectory(trajectory_data);
    AINFO << "Teb thread is stop.";
    return Status(ErrorCode::OK, "Teb Stage is finished.");
  }

  // Check if trajectory updated
  if (trajectory_updated_) {
    AINFO << "Trajectory is updated.";
    {
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      LoadResult(previous_frame, trajectory_data);
      data_ready_.store(false);
      start_plan_task_.store(false);
      trajectory_updated_.store(false);
    }

    if (FLAGS_enable_openspace_record_debug) {
      auto *ptr_debug = frame_->mutable_open_space_info()->mutable_debug();
      teb_trajectory_optimizer_->UpdateDebugInfo(
          ptr_debug->mutable_planning_data()->mutable_open_space());
    }

    // Plan succeed, so reset some flag
    is_first_enter_openspace_ = false;
    injector_->set_rescue_replan(false);
    rescue_status_->set_is_first_init(false);
    auto *open_space_status = injector_->planning_context()
                                  ->mutable_planning_status()
                                  ->mutable_open_space();
    AINFO << "Plan succeed, set_position_init false";
    open_space_status->set_position_init(false);
    return Status::OK();
  }

  // WYQ_Mark_Thread
  // data ready
  {
    std::lock_guard<std::mutex> lock(open_space_mutex_);
    if (previous_frame->open_space_info()
            .chosen_partitioned_trajectory()
            .first.size() > 0) {
      thread_data_.last_trace = previous_frame->open_space_info()
                                    .chosen_partitioned_trajectory()
                                    .first;
      thread_data_.last_gear = previous_frame->open_space_info()
                                   .chosen_partitioned_trajectory()
                                   .second;
    }
    thread_data_.world_obstacles_vertices_vec =
        open_space_info.boundary_vertices_vec();
    thread_data_.stitching_trajectory = stitching_trajectory;
    thread_data_.start_pose = start_pose;
    thread_data_.end_pose = open_space_info.open_space_end_pose();
    thread_data_.XYbounds = open_space_info.ROI_xy_boundary();
    thread_data_.rotate_angle = open_space_info.origin_heading();
    thread_data_.translate_origin = open_space_info.origin_point();
    thread_data_.obstacles_edges_num = open_space_info.obstacles_edges_num();
    thread_data_.obstacles_A = open_space_info.obstacles_A();
    thread_data_.obstacles_b = open_space_info.obstacles_b();
    thread_data_.obstacles_vertices_vec =
        open_space_info.obstacles_vertices_vec();

    thread_data_.obstacles = open_space_info.interest_obstacles();
    thread_data_.boundary_vertices_vec =
        open_space_info.boundary_vertices_vec();
    thread_data_.costmap_obstacles = frame_->static_area_polygon();
    data_ready_.store(true);
  }

  // Is Need Replan Now
  bool is_need_plan_now = false;
  is_need_plan_now = injector_->rescue_replan();
  AINFO << "WYQ is_need_plan_now " << is_need_plan_now;
  if (is_first_enter_openspace_ || is_need_plan_now) {
    start_plan_task_.store(true);
  }

  AINFO << "is_first_enter_openspace_ " << is_first_enter_openspace_
        << " ,is_need_plan_now " << is_need_plan_now << " ,start_plan_task_ "
        << start_plan_task_.load() << " ,last open_space_provider_success "
        << previous_frame->open_space_info().open_space_provider_success();

  // Successfully planned, reusing previous results
  if (previous_frame->open_space_info().open_space_provider_success() &&
      (!injector_->first_into_rescue() && !injector_->first_into_pullover())) {
    ReuseLastFrameResult(previous_frame, trajectory_data);
    AINFO << "Successfully planned, reusing previous results. size is "
          << trajectory_data->size();
    if (FLAGS_enable_openspace_record_debug) {
      // copy previous debug to current frame
      ReuseLastFrameDebug(previous_frame);
    }
    // reuse last frame debug when use last frame traj
    return Status(ErrorCode::OK,
                  "Successfully planned, reusing previous results.");
  } else {
    GenerateStopTrajectory(trajectory_data);
    AINFO << "Stop due to computation not finished.";
    return Status(ErrorCode::OK, "Stop due to computation not finished.");
  }
}

Status TEBTrajectoryProvider::ProcessNormal() {
  auto trajectory_data =
      frame_->mutable_open_space_info()->mutable_stitched_trajectory_result();
  auto *previous_frame = injector_->use_thread_in_play_street()
                             ? injector_->frame_teb_history()->Latest()
                             : injector_->frame_history()->Latest();
  const common::VehicleState vehicle_state = frame_->vehicle_state();
  // Use complete raw trajectory from last frame for stitching purpose
  std::vector<TrajectoryPoint> stitching_trajectory;
  const double adc_speed = vehicle_state.linear_velocity();
  const double adc_heading = vehicle_state.heading();
  const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                            injector_->vehicle_state()->y()};
  bool is_first_plan_success =
      previous_frame->open_space_info().open_space_provider_success();
  const auto &trajectory =
      previous_frame->open_space_info().chosen_partitioned_trajectory().first;

  bool enable_func = false;
  if (is_first_plan_success && enable_func && trajectory.size() > 1) {
    size_t current_index = trajectory.QueryNearestPoint(adc_position);
    size_t start_id = current_index + kStitchPointNum >= trajectory.size()
                          ? trajectory.size() - 1
                          : current_index + kStitchPointNum;
    stitching_trajectory.emplace_back(
        trajectory.TrajectoryPointAt(current_index));
    size_t s_id = current_index > 1 ? current_index - 1 : 0;
    stitching_trajectory.clear();
    for (size_t i = s_id; i <= start_id; ++i) {
      stitching_trajectory.emplace_back(trajectory.TrajectoryPointAt(i));
    }
  } else {
    common::TrajectoryPoint planning_init_point;
    planning_init_point.mutable_path_point()->set_x(vehicle_state.x());
    planning_init_point.mutable_path_point()->set_y(vehicle_state.y());
    planning_init_point.mutable_path_point()->set_theta(
        vehicle_state.heading());
    stitching_trajectory.emplace_back(planning_init_point);
  }
  const auto &open_space_info = frame_->open_space_info();
  Vec2d start_point(stitching_trajectory.back().path_point().x(),
                    stitching_trajectory.back().path_point().y());
  start_point -= open_space_info.origin_point();
  start_point.SelfRotate(-open_space_info.origin_heading());

  std::vector<double> start_pose;
  start_pose.clear();
  start_pose.emplace_back(start_point.x());
  start_pose.emplace_back(start_point.y());
  start_pose.emplace_back(adc_heading);
  start_pose.emplace_back(adc_speed);

  const auto &end_pose = open_space_info.open_space_end_pose();
  if (kIsEmptyIntValue == static_cast<int>(end_pose.size())) {
    AERROR << "Teb end_pose is empty";
    return Status(ErrorCode::PLANNING_ERROR, "Teb end_pose is empty");
  }
  const auto &rotate_angle = open_space_info.origin_heading();
  const auto &translate_origin = open_space_info.origin_point();
  const auto &obstacles_edges_num = open_space_info.obstacles_edges_num();
  const auto &obstacles_A = open_space_info.obstacles_A();
  const auto &obstacles_b = open_space_info.obstacles_b();
  const auto &obstacles_vertices_vec =
      open_space_info.obstacles_vertices_vec();  // All obstacle point sets
  const auto &XYbounds = open_space_info.ROI_xy_boundary();
  if (IsVehicleNearDestination(vehicle_state, end_pose, rotate_angle,
                               translate_origin) &&
      frame_->open_space_info().is_rescue_mode() &&
      !injector_->is_in_near_goal_) {
    has_warm_start_ = false;
    teb_trajectory_optimizer_->ClearPlanner();
    GenerateStopTrajectory(trajectory_data);
    return Status(ErrorCode::OK, "");
  }

  // constanst rescue astar is not ok in this mode
  const auto last_end_pose =
      previous_frame->open_space_info().open_space_end_pose();
  if (last_end_pose.size() < 3) {
    has_warm_start_ = false;
  } else {
    double dx = end_pose[0] - last_end_pose[0];
    double dy = end_pose[1] - last_end_pose[1];
    double ds = std::sqrt(dx * dx + dy * dy);
    if (ds > kValidGoalChangeS) {
      has_warm_start_ = false;
    }
  }

  if (rescue_status_->is_first_init() &&
      frame_->open_space_info().is_rescue_mode()) {
    has_warm_start_ = false;
  }

  Status status = Status(ErrorCode::PLANNING_ERROR, "init set not ok");
  for (const auto &it : openspace_common_ptr_->GetHADebugStatus()) {
    AERROR << "  last astar reason " << static_cast<int>(it);
  }
  auto *open_space_status = injector_->planning_context()
                                ->mutable_planning_status()
                                ->mutable_open_space();
  if (FLAGS_enable_rescue_back_use_teb &&
      rescue_status_->is_select_back_pose()) {
    if (!has_warm_start_) {
      teb_trajectory_optimizer_->ClearPlanner();
    }
    status = teb_trajectory_optimizer_->UpdateTEB(
        stitching_trajectory, start_pose, end_pose, XYbounds, rotate_angle,
        translate_origin, obstacles_edges_num, obstacles_A, obstacles_b,
        obstacles_vertices_vec, open_space_info.interest_obstacles(),
        open_space_info.boundary_vertices_vec(), frame_->static_area_polygon());
  } else {
    if (!has_warm_start_) {
      UpdateScenario();
      teb_trajectory_optimizer_->ClearPlanner();
      const auto start_timestamp = std::chrono::system_clock::now();
      const std::vector<std::vector<Vec2d>> world_obstacles_vertices_vec =
          open_space_info.boundary_vertices_vec();
      status = teb_trajectory_optimizer_->Plan(
          world_obstacles_vertices_vec, stitching_trajectory, start_pose,
          end_pose, XYbounds, rotate_angle, translate_origin,
          obstacles_edges_num, obstacles_A, obstacles_b, obstacles_vertices_vec,
          open_space_info.interest_obstacles(),
          open_space_info.boundary_vertices_vec(),
          frame_->static_area_polygon(), !injector_->use_teb_default_bound_,
          &injector_->start_collision_flag_);
      const auto end_timestamp = std::chrono::system_clock::now();
      std::chrono::duration<double> diff = end_timestamp - start_timestamp;
      ADEBUG << "open space trajectory optimizer total time: "
             << diff.count() * 1000.0 << " ms.";
      double time_latency = diff.count() * 1000.0;
      frame_->mutable_open_space_info()->set_time_latency(time_latency);
      injector_->shrink_end_s_ = false;
      // only triger once
      // warm start error, relax the contraints
      if (status != Status::OK() && !status.ok()) {
        for (const auto &reason : openspace_common_ptr_->GetHADebugStatus()) {
          if (OpenspaceCommon::HybridAStarDebugStatus::START_COLLISION ==
              reason) {
            injector_->deal_start_block_ = true;
          }
          if ((reason != OpenspaceCommon::HybridAStarDebugStatus::NORMAL) &&
              !injector_->use_teb_default_bound_) {
            injector_->use_teb_default_bound_ = true;
          }
        }
        injector_->shrink_end_s_ = true;
      } else {
        AINFO << "Plan succeed, set_position_init false";
        open_space_status->set_position_init(false);
      }
    } else if ((previous_frame->open_space_info()
                    .open_space_provider_success() &&
                !open_space_status->position_init()) ||
               FLAGS_enable_reuse_teb_plan || injector_->is_in_near_goal_) {
      AINFO << "Use_Plan_Last";
      ReuseLastFrameResult(previous_frame, trajectory_data);
      if (FLAGS_enable_openspace_record_debug) {
        ReuseLastFrameDebug(previous_frame);
      }
      return Status(ErrorCode::OK, "Reuse last frame result.");
    } else {
      status = teb_trajectory_optimizer_->UpdateTEB(
          stitching_trajectory, start_pose, end_pose, XYbounds, rotate_angle,
          translate_origin, obstacles_edges_num, obstacles_A, obstacles_b,
          obstacles_vertices_vec, open_space_info.interest_obstacles(),
          open_space_info.boundary_vertices_vec(),
          frame_->static_area_polygon());
    }
  }

  if (Status::OK() == status) {
    has_warm_start_ = true;
    rescue_status_->set_is_first_init(false);
    LoadResult(previous_frame, trajectory_data);

    if (FLAGS_enable_openspace_record_debug) {
      auto *ptr_debug = frame_->mutable_open_space_info()->mutable_debug();
      teb_trajectory_optimizer_->UpdateDebugInfo(
          ptr_debug->mutable_planning_data()->mutable_open_space());
    }
    return status;
  } else if (previous_frame->open_space_info().open_space_provider_success()) {
    ReuseLastFrameResult(previous_frame, trajectory_data);
    if (FLAGS_enable_openspace_record_debug) {
      ReuseLastFrameDebug(previous_frame);
    }
    return status;
  } else {
    if (FLAGS_enable_openspace_record_debug) {
      auto *ptr_debug = frame_->mutable_open_space_info()->mutable_debug();
      teb_trajectory_optimizer_->UpdateDebugInfo(
          ptr_debug->mutable_planning_data()->mutable_open_space());
    }
  }
  return Status(ErrorCode::PLANNING_ERROR);
}

// WYQ_Mark_Thread
void TEBTrajectoryProvider::GenerateTrajectoryThread() {
  while (!is_generation_thread_stop_.load()) {
    cyber::SleepFor(std::chrono::milliseconds(kSleepTime));
    if (trajectory_updated_.load() || !data_ready_.load() ||
        !start_plan_task_.load()) {
      continue;
    }

    TEBTrajectoryThreadData thread_data;
    {
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      thread_data = thread_data_;
    }
    double start_timestamp = ::century::cyber::Clock::NowInSeconds();
    Status status;
    teb_trajectory_optimizer_->ClearPlanner();
    if (FLAGS_enable_rescue_back_use_teb &&
        rescue_status_->is_select_back_pose()) {
      status = teb_trajectory_optimizer_->UpdateTEB(
          thread_data.stitching_trajectory, thread_data.start_pose,
          thread_data.end_pose, thread_data.XYbounds, thread_data.rotate_angle,
          thread_data.translate_origin, thread_data.obstacles_edges_num,
          thread_data.obstacles_A, thread_data.obstacles_b,
          thread_data.obstacles_vertices_vec, thread_data.obstacles,
          thread_data.boundary_vertices_vec, thread_data.costmap_obstacles);
    } else {
      status = teb_trajectory_optimizer_->Plan(
          thread_data.world_obstacles_vertices_vec,
          thread_data.stitching_trajectory, thread_data.start_pose,
          thread_data.end_pose, thread_data.XYbounds, thread_data.rotate_angle,
          thread_data.translate_origin, thread_data.obstacles_edges_num,
          thread_data.obstacles_A, thread_data.obstacles_b,
          thread_data.obstacles_vertices_vec, thread_data.obstacles,
          thread_data.boundary_vertices_vec, thread_data.costmap_obstacles,
          !injector_->use_teb_default_bound_,
          &injector_->start_collision_flag_);
    }

    double end_timestamp = ::century::cyber::Clock::NowInSeconds();
    double total_timestamp = (end_timestamp - start_timestamp) * 1000;

    AINFO << "Thread cost time is " << total_timestamp << " ms.";
    frame_->mutable_open_space_info()->set_time_latency(total_timestamp);
    if (Status::OK() == status) {
      trajectory_updated_.store(true);
      data_ready_.store(false);
      is_first_enter_openspace_ = false;
      optimizer_thread_counter_ = 0;
      start_plan_task_.store(false);
      injector_->set_rescue_replan(false);
      AINFO << "Plan succeed, reset some flags.";
      continue;
    }

    if (status.ok()) {
      AERROR << "Vehicle is near to destination Or Teb Thread is stop.";
      trajectory_skipped_.store(true);
      continue;
    }

    trajectory_error_.store(true);
    AERROR << "Plan failed, do some things. shrink_end_s_";

    rescue_status_->set_is_first_init(true);
    injector_->shrink_end_s_ = true;
    for (const auto& reason : openspace_common_ptr_->GetHADebugStatus()) {
      if (OpenspaceCommon::HybridAStarDebugStatus::START_COLLISION == reason) {
        injector_->deal_start_block_ = true;
        AERROR << "Plan start collision, deal_start_block.";
      }
      if ((reason != OpenspaceCommon::HybridAStarDebugStatus::NORMAL) &&
          !injector_->use_teb_default_bound_) {
        AERROR << "Plan failed, use_teb_default_bound_";
        injector_->use_teb_default_bound_ = true;
      }
    }
  }
}

bool TEBTrajectoryProvider::IsVehicleNearDestination(
    const common::VehicleState &vehicle_state,
    const std::vector<double> &end_pose, double rotate_angle,
    const Vec2d &translate_origin) {
  CHECK_EQ(end_pose.size(), 4U);

  if (injector_->pullover_using_) {
    return false;
  }

  Vec2d end_pose_to_world_frame(end_pose[0], end_pose[1]);

  end_pose_to_world_frame.SelfRotate(rotate_angle);
  end_pose_to_world_frame += translate_origin;
  double distance_to_vehicle2 =
      std::sqrt((vehicle_state.x() - end_pose_to_world_frame.x()) *
                    (vehicle_state.x() - end_pose_to_world_frame.x()) +
                (vehicle_state.y() - end_pose_to_world_frame.y()) *
                    (vehicle_state.y() - end_pose_to_world_frame.y()));
  double end_theta_to_world_frame = end_pose[2];
  end_theta_to_world_frame += rotate_angle;
  double distance_to_vehicle1 = std::sqrt(
      (vehicle_state.x() - end_pose[0]) * (vehicle_state.x() - end_pose[0]) +
      (vehicle_state.y() - end_pose[1]) * (vehicle_state.y() - end_pose[1]));
  double distance_to_vehicle =
      std::sqrt((vehicle_state.x() - end_pose_to_world_frame.x()) *
                    (vehicle_state.x() - end_pose_to_world_frame.x()) +
                (vehicle_state.y() - end_pose_to_world_frame.y()) *
                    (vehicle_state.y() - end_pose_to_world_frame.y()));
  double theta_to_vehicle = std::abs(common::math::AngleDiff(
      vehicle_state.heading(), end_theta_to_world_frame));
  // in same frame
  ADEBUG << "distance_to_vehicle2 is: " << distance_to_vehicle2;

  distance_to_vehicle =
      std::min(std::min(distance_to_vehicle, distance_to_vehicle1),
               distance_to_vehicle2);
  theta_to_vehicle =
      std::min(theta_to_vehicle, std::abs(vehicle_state.heading()));
  const double adc_speed = vehicle_state.linear_velocity();

  AINFO << "to_local_end_dis: " << distance_to_vehicle
        << "theta_diff: " << theta_to_vehicle;
  double reached_thr = config_.teb_trajectory_provider_config()
                           .teb_trajectory_optimizer_config()
                           .planner_open_space_config()
                           .is_near_destination_threshold();

  if (distance_to_vehicle < reached_thr &&
      theta_to_vehicle < config_.teb_trajectory_provider_config()
                             .teb_trajectory_optimizer_config()
                             .planner_open_space_config()
                             .is_near_destination_theta_threshold() &&
      std::abs(adc_speed) < kEpsilon) {
    AINFO << "vehicle reach end_pose";
    frame_->mutable_open_space_info()->set_destination_reached(true);
    return true;
  }
  return false;
}

bool TEBTrajectoryProvider::IsVehicleStopDueToFallBack(
    const bool is_on_fallback, const common::VehicleState &vehicle_state) {
  if (!is_on_fallback) {
    return false;
  }
  const double adc_speed = vehicle_state.linear_velocity();

  if (std::abs(adc_speed) < kEpsilon) {
    AERROR << "ADC stops due to fallback trajectory";
    return true;
  }
  return false;
}

void TEBTrajectoryProvider::GenerateStopTrajectory(
    DiscretizedTrajectory *const trajectory_data) {
  double relative_time = 0.0;
  // TODO(Jinyun) Move to conf
  AINFO << "stop!!!!";
  static constexpr int stop_trajectory_length = 10;
  static constexpr double relative_stop_time = 0.1;
  static constexpr double vEpsilon = 0.00001;
  double standstill_acceleration =
      frame_->vehicle_state().linear_velocity() >= -vEpsilon
          ? -FLAGS_open_space_standstill_acceleration
          : FLAGS_open_space_standstill_acceleration;
  trajectory_data->clear();
  for (size_t i = 0; i < stop_trajectory_length; i++) {
    TrajectoryPoint point;
    point.mutable_path_point()->set_x(frame_->vehicle_state().x());
    point.mutable_path_point()->set_y(frame_->vehicle_state().y());
    point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(0.0);
    point.set_relative_time(relative_time);
    point.set_v(0.0);
    point.set_a(standstill_acceleration);
    trajectory_data->emplace_back(point);
    relative_time += relative_stop_time;
  }
}

void TEBTrajectoryProvider::LoadResult(
    const Frame *last_frame, DiscretizedTrajectory *const trajectory_data) {
  // Load unstitched two trajectories into frame for debug
  auto optimizer_trajectory_ptr =
      frame_->mutable_open_space_info()->mutable_optimizer_trajectory_data();
  auto stitching_trajectory_ptr =
      frame_->mutable_open_space_info()->mutable_stitching_trajectory_data();

  teb_trajectory_optimizer_->GetOptimizedTrajectory(optimizer_trajectory_ptr);

  // origin planning once
  teb_trajectory_optimizer_->GetStitchingTrajectory(stitching_trajectory_ptr);
  size_t optimizer_trajectory_size = optimizer_trajectory_ptr->size();
  trajectory_data->clear();

  if (optimizer_trajectory_size > 1) {
    bool is_front_traj = optimizer_trajectory_ptr->at(1).v() > 0;
    double adc_speed = frame_->vehicle_state().linear_velocity();
    if (!is_front_traj && adc_speed > kEpsilon &&
        last_frame->open_space_info().open_space_provider_success()) {
      AINFO << "ReuseLastFrameResult";
      ReuseLastFrameResult(last_frame, trajectory_data);
      return;
    }
  }

  // Stitch two trajectories and load back to trajectory_data from frame
  // teb just plan once
  // double stitching_point_relative_time = 0;
  if (stitching_trajectory_ptr->size()) {
    double stitching_point_relative_time =
        stitching_trajectory_ptr->back().relative_time();
    double stitching_point_relative_s =
        stitching_trajectory_ptr->back().path_point().s();
    for (size_t i = 0; i < optimizer_trajectory_size; ++i) {
      optimizer_trajectory_ptr->at(i).set_relative_time(
          optimizer_trajectory_ptr->at(i).relative_time() +
          stitching_point_relative_time);
      optimizer_trajectory_ptr->at(i).mutable_path_point()->set_s(
          optimizer_trajectory_ptr->at(i).path_point().s() +
          stitching_point_relative_s);
    }
  }
  *(trajectory_data) = *(optimizer_trajectory_ptr);

  static int last_size = trajectory_data->size();
  int s = trajectory_data->size();
  if (s != last_size) {
    AINFO << "size changed, notice ";
    last_size = s;
  }
  AINFO << "optimizer_trajectory_ptr size " << optimizer_trajectory_size
        << ", stitch size" << stitching_trajectory_ptr->size()
        << ", trajectory_data.size load " << trajectory_data->size();

  if ((!rescue_status_->is_select_back_pose() &&
           FLAGS_enable_teb_planner_thread ||
       kTempProcessThreadFlages) &&
      frame_->open_space_info().stitching_trajectory_data().size()) {
    frame_->mutable_open_space_info()
        ->mutable_stitching_trajectory_data()
        ->pop_back();

    size_t prev_trajectory_size = stitching_trajectory_ptr->size();
    if (prev_trajectory_size) {
      DiscretizedTrajectory prev_trajectory =
          DiscretizedTrajectory(*stitching_trajectory_ptr);
      const common::VehicleState vehicle_state = frame_->vehicle_state();
      size_t position_matched_index =
          prev_trajectory.QueryNearestPointWithBuffer(
              {vehicle_state.x(), vehicle_state.y()}, kEpsilon);
      AERROR << "position_matched_index " << position_matched_index;
      AERROR << "prev_trajectory_size " << prev_trajectory_size;
      if (position_matched_index >= prev_trajectory_size) {
        position_matched_index = prev_trajectory_size - 1;
      } else {
        const size_t preserved_points_num =
            rescue_status_->trajectory_stitching_preserved_points_num();
        position_matched_index = std::max(
            static_cast<int>(0),
            static_cast<int>(position_matched_index - preserved_points_num));
      }
      AERROR << "after position_matched_index " << position_matched_index;
      stitching_trajectory_ptr->assign(
          stitching_trajectory_ptr->begin() + position_matched_index,
          stitching_trajectory_ptr->begin() + prev_trajectory_size);

      trajectory_data->PrependTrajectoryPoints(
          frame_->open_space_info().stitching_trajectory_data());
    }
  }

  ADEBUG << "end trajectory_data.size " << trajectory_data->size();
  frame_->mutable_open_space_info()->set_open_space_provider_success(true);
}

void TEBTrajectoryProvider::ReuseLastFrameResult(
    const Frame *last_frame, DiscretizedTrajectory *const trajectory_data) {
  *(trajectory_data) =
      last_frame->open_space_info().stitched_trajectory_result();
  frame_->mutable_open_space_info()->set_open_space_provider_success(true);

  *(frame_->mutable_open_space_info()->mutable_rescue_first_end_point()) =
      last_frame->open_space_info().rescue_first_end_point();
}

void TEBTrajectoryProvider::ReuseLastFrameDebug(const Frame *last_frame) {
  // reuse last frame's instance
  auto *ptr_debug = frame_->mutable_open_space_info()->mutable_debug_instance();
  ptr_debug->mutable_planning_data()->mutable_open_space()->MergeFrom(
      last_frame->open_space_info()
          .debug_instance()
          .planning_data()
          .open_space());
}

double TEBTrajectoryProvider::GetDistanceToEndPose(
    const common::TrajectoryPoint &planning_init_point,
    const std::vector<double> &end_pose, double rotate_angle,
    const Vec2d &translate_origin) {
  CHECK_EQ(end_pose.size(), 4U);
  Vec2d end_pose_to_world_frame(end_pose[0], end_pose[1]);

  end_pose_to_world_frame.SelfRotate(rotate_angle);
  end_pose_to_world_frame += translate_origin;

  const auto &path_point = planning_init_point.path_point();
  double distance_to_init_point =
      std::sqrt((path_point.x() - end_pose_to_world_frame.x()) *
                    (path_point.x() - end_pose_to_world_frame.x()) +
                (path_point.y() - end_pose_to_world_frame.y()) *
                    (path_point.y() - end_pose_to_world_frame.y()));
  double distance_adc_to_init_point =
      std::sqrt((path_point.x() - frame_->vehicle_state().x()) *
                    (path_point.x() - frame_->vehicle_state().x()) +
                (path_point.y() - frame_->vehicle_state().y()) *
                    (path_point.y() - frame_->vehicle_state().y()));
  AINFO << "distance_adc_to_init_point " << distance_adc_to_init_point
        << " distance_to_init_point " << distance_to_init_point
        << " end_pose[0]" << end_pose[0];
  return distance_adc_to_init_point;
}

void TEBTrajectoryProvider::RecomputeStartPoint(
    const common::VehicleState& vehicle_state, common::math::Vec2d* start_point,
    double* adc_heading) {
  const auto last_gear = thread_data_.last_gear;
  const auto last_trace = thread_data_.last_trace;
  if (last_trace.empty()) {
    return;
  }
  Vec2d adc_point(vehicle_state.x(), vehicle_state.y());
  size_t adc_index = last_trace.QueryNearestPoint(adc_point);
  cyber::SleepFor(std::chrono::milliseconds(kSleepTime));
  Vec2d nearest_point(last_trace[adc_index].path_point().x(),
                      last_trace[adc_index].path_point().y());

  if (start_point->DistanceTo(nearest_point) > kFollowWellThr) {
    return;
  }

  double current_s = last_trace[adc_index].path_point().s();
  double preview_dis =
      kPreviewTime * std::fabs(vehicle_state.linear_velocity());
  if (preview_dis <= 0.0) {
    return;
  }
  bool forward = (canbus::Chassis::GEAR_DRIVE == last_gear);
  double target_s =
      forward ? (current_s + preview_dis) : (current_s - preview_dis);
  for (size_t i = adc_index; i < last_trace.size(); ++i) {
    start_point->set_x(last_trace[i].path_point().x());
    start_point->set_y(last_trace[i].path_point().y());
    *adc_heading = last_trace[i].path_point().theta();
    double s_val = last_trace[i].path_point().s();
    if ((forward && s_val >= target_s) || (!forward && s_val <= target_s)) {
      break;
    }
  }
}
}  // namespace planning
}  // namespace century
