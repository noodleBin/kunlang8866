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

#include "modules/planning/tasks/optimizers/open_space_trajectory_generation/open_space_trajectory_provider.h"

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
constexpr double kEpsilon = 1.0e-1;
constexpr double kGoodTrajDist = 5.0;
constexpr int kValidTrajNum = 11;
constexpr int kSmoothSpeedPointNum = 25;
constexpr int kCountNum = 5;
constexpr int kStitchPointNum = 3;
}  // namespace
using century::common::ErrorCode;
using century::common::Status;
using century::common::TrajectoryPoint;
using century::common::math::Vec2d;
using century::cyber::Clock;

// constructor
OpenSpaceTrajectoryProvider::OpenSpaceTrajectoryProvider(
    const TaskConfig &config,
    const std::shared_ptr<DependencyInjector> &injector)
    : TrajectoryOptimizer(config, injector) {

  open_space_trajectory_optimizer_.reset(new OpenSpaceTrajectoryOptimizer(
      config.open_space_trajectory_provider_config()
          .open_space_trajectory_optimizer_config()));

  last_plan_traj_ptr_.reset(new DiscretizedTrajectory());
}

// destructor
OpenSpaceTrajectoryProvider::~OpenSpaceTrajectoryProvider() {
  if (FLAGS_enable_open_space_planner_thread) {
    Stop();
  }
}

void OpenSpaceTrajectoryProvider::Stop() {
  if (FLAGS_enable_open_space_planner_thread) {
    is_generation_thread_stop_.store(true);
    if (thread_init_flag_) {
      task_future_.get();
    }
    trajectory_updated_.store(false);
    trajectory_error_.store(false);
    trajectory_skipped_.store(false);
    optimizer_thread_counter = 0;
  }
}

void OpenSpaceTrajectoryProvider::Restart() {
  if (FLAGS_enable_open_space_planner_thread) {
    is_generation_thread_stop_.store(true);
    if (thread_init_flag_) {
      task_future_.get();
    }
    is_generation_thread_stop_.store(false);
    thread_init_flag_ = false;
    trajectory_updated_.store(false);
    trajectory_error_.store(false);
    trajectory_skipped_.store(false);
    optimizer_thread_counter = 0;
  }
}

Status OpenSpaceTrajectoryProvider::Process() {
  ADEBUG << "OpenSpaceTrajectoryProvider Process";
  auto trajectory_data =
      frame_->mutable_open_space_info()->mutable_stitched_trajectory_result();
  // generate stop trajectory at park_and_go check_stage
  if (injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_park_and_go()
          ->in_check_stage()) {
    ADEBUG << "ParkAndGo Stage Check.";
    GenerateStopTrajectory(trajectory_data);
    return Status::OK();
  }

  // Start thread when getting in Process() for the first time
  if (FLAGS_enable_open_space_planner_thread && !thread_init_flag_) {
    task_future_ = cyber::Async(
        &OpenSpaceTrajectoryProvider::GenerateTrajectoryThread, this);
    thread_init_flag_ = true;
    is_first_enter_openspace_ = true;
  }

  auto *previous_frame = injector_->frame_history()->Latest();
  if (nullptr == previous_frame ||
      !previous_frame->open_space_info().is_on_open_space_trajectory()) {
    is_first_enter_openspace_ = true;
  }

  const common::VehicleState vehicle_state = frame_->vehicle_state();
  const auto &gear =
      previous_frame->open_space_info().chosen_partitioned_trajectory().second;

  // new a thread to plan trajectory
  if (FLAGS_enable_open_space_planner_thread) {
    // Check if trajectory updated
    if (trajectory_updated_.load()) {          // new trajectory generated
      if (frame_->open_space_info().is_ready_to_second_plan() &&
          previous_frame->open_space_info().open_space_provider_success()) {
        // lwt: If you want ride comfort, this gear check protection can be
        // turned off; Currently, check is enabled for security
        if (canbus::Chassis::GEAR_REVERSE == gear) {      // reverse gear
          AERROR << "second_plan must need gear == drive, return false";
          return Status(ErrorCode::PLANNING_ERROR,
                        "second_plan must need gear == drive, return false");
        }
        frame_->mutable_open_space_info()
            ->set_open_space_second_provider_success(true);
      }

      {
        std::lock_guard<std::mutex> lock(open_space_mutex_);
        LoadResult(previous_frame, trajectory_data);
      }

      //
      if (FLAGS_enable_openspace_record_debug) {
        auto *ptr_debug = frame_->mutable_open_space_info()->mutable_debug();
        {
          std::lock_guard<std::mutex> lock(open_space_mutex_);
          open_space_trajectory_optimizer_->UpdateDebugInfo(
              ptr_debug->mutable_planning_data()->mutable_open_space());
        }
        frame_->mutable_open_space_info()->sync_debug_instance();
      }
      is_first_enter_openspace_ = false;
      ready_count_ = kCountNum;
      data_ready_.store(false);
      trajectory_updated_.store(false);
      return Status::OK();
    }
  }
  // Use complete raw trajectory from last frame for stitching purpose
  std::vector<TrajectoryPoint> stitching_trajectory;
  const double adc_speed = vehicle_state.linear_velocity();
  bool is_stop = (std::abs(adc_speed) < common::VehicleConfigHelper::Instance()
                      ->GetConfig().vehicle_param().max_abs_speed_when_stopped());
  const auto &trajectory =
      previous_frame->open_space_info().chosen_partitioned_trajectory().first;

  common::TrajectoryPoint planning_init_point;

  const Vec2d adc_position = {injector_->vehicle_state()->x(),
                              injector_->vehicle_state()->y()};
  bool enable_func = false;
  bool is_first_plan_success =
      previous_frame->open_space_info().open_space_provider_success();
  if (is_first_plan_success && enable_func && trajectory.size() > 1) {
    stitching_trajectory.clear();
    bool is_first_plan_success =
        previous_frame->open_space_info().open_space_provider_success();
    if (is_first_plan_success) {
      const auto &trajectory = previous_frame->open_space_info()
                                   .chosen_partitioned_trajectory()
                                   .first;
      size_t current_index = trajectory.QueryNearestPoint(adc_position);
      size_t start_id = current_index + kStitchPointNum >= trajectory.size()
                            ? trajectory.size() - 1
                            : current_index + kStitchPointNum;
      stitching_trajectory.emplace_back(
          trajectory.TrajectoryPointAt(current_index));
      if (start_id >= current_index + 1) {
        stitching_trajectory.emplace_back(
            trajectory.TrajectoryPointAt(start_id));
      }
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
  double distance = GetDistanceToEndPose(
      stitching_trajectory.back(), open_space_info.open_space_end_pose(),
      open_space_info.origin_heading(), open_space_info.origin_point());
  bool is_need_first_plan =
      (distance < kAdcInitPlanningErrorDist &&
       !previous_frame->open_space_info().open_space_provider_success() &&
       is_stop);

  if (is_first_enter_openspace_ || is_need_first_plan) {
    start_plan_task_.store(true);
  } else {
    start_plan_task_.store(false);
  }

  if (FLAGS_enable_open_space_planner_thread) {
    if (IsVehicleNearDestination(
            vehicle_state, open_space_info.open_space_end_pose(),
            open_space_info.origin_heading(), open_space_info.origin_point())) {
      GenerateStopTrajectory(trajectory_data);
      return Status(ErrorCode::OK, "Vehicle is near to destination");
    }

    if (is_generation_thread_stop_) {
      GenerateStopTrajectory(trajectory_data);
      return Status(ErrorCode::OK, "Parking finished");
    }
    ready_count_ = ready_count_ > 0 ? ready_count_ - 1 : 0;
    if (ready_count_ == 0) {          // wait kCountNum and replan
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      thread_data_.stitching_trajectory = stitching_trajectory;
      Vec2d init_point(stitching_trajectory.back().path_point().x(),
                       stitching_trajectory.back().path_point().y());
      thread_data_.end_pose = open_space_info.open_space_end_pose();
      thread_data_.rotate_angle = open_space_info.origin_heading();
      thread_data_.translate_origin = open_space_info.origin_point();
      thread_data_.obstacles_edges_num = open_space_info.obstacles_edges_num();
      thread_data_.obstacles_A = open_space_info.obstacles_A();
      thread_data_.obstacles_b = open_space_info.obstacles_b();
      thread_data_.obstacles_vertices_vec =
          open_space_info.obstacles_vertices_vec();
      thread_data_.XYbounds = open_space_info.ROI_xy_boundary();
      thread_data_.nearby_path = frame_->reference_line_info().front()
                                     .reference_line().GetMapPath();
      data_ready_.store(true);
    }

    if (trajectory_error_ &&
        !previous_frame->open_space_info().open_space_provider_success()) {
      ++optimizer_thread_counter;
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      trajectory_error_.store(false);
      // TODO(Jinyun) Use other fallback mechanism when last iteration
      // smoothing result has out of bound pathpoint which is not allowed for
      // next iteration hybrid astar algorithm which requires start position
      // to be strictly in bound
      if (optimizer_thread_counter > 1000) {
        return Status(ErrorCode::PLANNING_ERROR,
                      "open_space_optimizer failed too many times");
      }
    }

    // previous frame plan succeed
    if (previous_frame->open_space_info().open_space_provider_success()) {
      // reuse last frame result
      ReuseLastFrameResult(previous_frame, trajectory_data);
      if (FLAGS_enable_openspace_record_debug) {
        // copy previous debug to current frame
        ReuseLastFrameDebug(previous_frame);
      }
      // reuse last frame debug when use last frame traj
      return Status(ErrorCode::OK,
                    "Waiting for open_space_trajectory_optimizer in "
                    "open_space_trajectory_provider");
    } else {
      GenerateStopTrajectory(trajectory_data);
      AERROR << "Stop due to computation not finished";
      return Status(ErrorCode::OK, "Stop due to computation not finished");
    }
  } else {
    return Process(previous_frame, stitching_trajectory);
  }
  return Status::OK();
}

Status OpenSpaceTrajectoryProvider::Process(
    const century::planning::Frame* pre_frame,
    std::vector<TrajectoryPoint>& stitching_trajectory) {
  auto trajectory_data =
      frame_->mutable_open_space_info()->mutable_stitched_trajectory_result();
  const common::VehicleState vehicle_state = frame_->vehicle_state();
  const auto& open_space_info = frame_->open_space_info();
  const auto& end_pose = open_space_info.open_space_end_pose();
  const auto& rotate_angle = open_space_info.origin_heading();
  const auto& translate_origin = open_space_info.origin_point();
  const auto& obstacles_edges_num = open_space_info.obstacles_edges_num();
  const auto& obstacles_A = open_space_info.obstacles_A();
  const auto& obstacles_b = open_space_info.obstacles_b();
  const auto& obstacles_vertices_vec = open_space_info.obstacles_vertices_vec();
  const auto& XYbounds = open_space_info.ROI_xy_boundary();
  const auto& nearby_path =
      frame_->reference_line_info().front().reference_line().GetMapPath();

  // Check vehicle state
  if (IsVehicleNearDestination(vehicle_state, end_pose, rotate_angle,
                               translate_origin)) {
    GenerateStopTrajectory(trajectory_data);
    return Status(ErrorCode::OK, "Vehicle is near to destination");
  }
  const auto& roi_type =
      config_.open_space_trajectory_provider_config().roi_type();
  bool use_pure_astar = false;
  if (FLAGS_enable_pure_astar) {
    use_pure_astar =
        OpenSpaceTrajectoryProviderConfig::TRAFFIC_LIGHT == roi_type ? true
                                                                     : false;
  }
  double time_latency;
  // constanst rescue astar is not ok in this mode
  Status status = open_space_trajectory_optimizer_->Plan(
      stitching_trajectory, end_pose, XYbounds, rotate_angle, translate_origin,
      obstacles_edges_num, obstacles_A, obstacles_b, obstacles_vertices_vec,
      &time_latency, use_pure_astar,
      (roi_type == OpenSpaceTrajectoryProviderConfig::PARK_AND_GO ? &nearby_path
                                                                  : nullptr));
  frame_->mutable_open_space_info()->set_time_latency(time_latency);
  if (status == Status::OK()) {
    LoadResult(pre_frame, trajectory_data);
    if (FLAGS_enable_openspace_record_debug) {
      // call merge debug ptr, teb_trajectory_optimizer_
      auto* ptr_debug = frame_->mutable_open_space_info()->mutable_debug();
      open_space_trajectory_optimizer_->UpdateDebugInfo(
          ptr_debug->mutable_planning_data()->mutable_open_space());
      frame_->mutable_open_space_info()->sync_debug_instance();
    }
    return status;
  } else if (pre_frame->open_space_info().open_space_provider_success()) {
    ReuseLastFrameResult(pre_frame, trajectory_data);
    if (FLAGS_enable_openspace_record_debug) {
      // copy previous debug to current frame
      ReuseLastFrameDebug(pre_frame);
    }
    // reuse last frame debug when use last frame traj
    return Status(ErrorCode::OK,
                  "use last frame status "
                  "open_space_trajectory_provider");
  }

  AINFO << "reuse last frame , plan result"
        << pre_frame->open_space_info().open_space_provider_success();
  ADEBUG << "trajectory_data.size " << trajectory_data->size();

  return Status(ErrorCode::PLANNING_ERROR);
}

// new a thread for plan
void OpenSpaceTrajectoryProvider::GenerateTrajectoryThread() {
  const auto &roi_type = config_.open_space_trajectory_provider_config().roi_type();
  bool use_pure_astar = false;
  if (FLAGS_enable_pure_astar) {
    use_pure_astar =
        OpenSpaceTrajectoryProviderConfig::TRAFFIC_LIGHT == roi_type ? true : false;
  }

  while (!is_generation_thread_stop_.load()) {          // thread stop flag
    // data ready
    if (!trajectory_updated_.load() && data_ready_.load() &&
        start_plan_task_.load()) {

      // get input data
      OpenSpaceTrajectoryThreadData thread_data;
      {
        std::lock_guard<std::mutex> lock(open_space_mutex_);
        thread_data = thread_data_;
      }

      // start plan
      double time_latency;
      Status status = open_space_trajectory_optimizer_->Plan(
          thread_data.stitching_trajectory, thread_data.end_pose,
          thread_data.XYbounds, thread_data.rotate_angle,
          thread_data.translate_origin, thread_data.obstacles_edges_num,
          thread_data.obstacles_A, thread_data.obstacles_b,
          thread_data.obstacles_vertices_vec, &time_latency, use_pure_astar,
          (roi_type == OpenSpaceTrajectoryProviderConfig::PARK_AND_GO ?
               &thread_data.nearby_path : nullptr));

      frame_->mutable_open_space_info()->set_time_latency(time_latency);
      if (status == Status::OK()) {            // plan success
        trajectory_updated_.store(true);
        data_ready_.store(false);
        start_plan_task_.store(false);
        // update end pose
        auto* end_pose =
            frame_->mutable_open_space_info()->mutable_open_space_end_pose();
        *end_pose = open_space_trajectory_optimizer_->GetPlannedEndPose();
      } else {
        if (status.ok()) {
          trajectory_skipped_.store(true);
          cyber::SleepFor(std::chrono::milliseconds(20));
        } else {
          AERROR << "trajectory failed";
          trajectory_error_.store(true);
        }
      }
    } else {
      cyber::SleepFor(std::chrono::milliseconds(20));
    }
  }
}

// check vehicle is near destination
bool OpenSpaceTrajectoryProvider::IsVehicleNearDestination(
    const common::VehicleState &vehicle_state,
    const std::vector<double> &end_pose, double rotate_angle,
    const Vec2d &translate_origin) {
  CHECK_EQ(end_pose.size(), 4U);
  // end pose  origin --> world
  Vec2d end_pose_to_world_frame(end_pose[0], end_pose[1]);
  end_pose_to_world_frame.SelfRotate(rotate_angle);
  end_pose_to_world_frame += translate_origin;
  double end_theta_to_world_frame = end_pose[2];
  end_theta_to_world_frame += rotate_angle;

  double distance_to_vehicle =
      std::hypot(vehicle_state.x() - end_pose_to_world_frame.x(),
                 vehicle_state.y() - end_pose_to_world_frame.y());
  double theta_to_vehicle = std::abs(common::math::AngleDiff(
      vehicle_state.heading(), end_theta_to_world_frame));

  ADEBUG << "distance_to_vehicle is: " << distance_to_vehicle;
  ADEBUG << "is_near_destination_threshold"
         << config_.open_space_trajectory_provider_config()
                .open_space_trajectory_optimizer_config()
                .planner_open_space_config()
                .is_near_destination_threshold();
  ADEBUG << "is_near_destination_theta_threshold"
         << config_.open_space_trajectory_provider_config()
                .open_space_trajectory_optimizer_config()
                .planner_open_space_config()
                .is_near_destination_theta_threshold();

  const double adc_speed = vehicle_state.linear_velocity();

  if (distance_to_vehicle < config_.open_space_trajectory_provider_config()
                                .open_space_trajectory_optimizer_config()
                                .planner_open_space_config()
                                .is_near_destination_threshold() &&
      theta_to_vehicle < config_.open_space_trajectory_provider_config()
                             .open_space_trajectory_optimizer_config()
                             .planner_open_space_config()
                             .is_near_destination_theta_threshold() &&
      std::abs(adc_speed) < common::VehicleConfigHelper::Instance()->GetConfig()
          .vehicle_param().max_abs_speed_when_stopped()) {
    AERROR << "vehicle reach end_pose";
    frame_->mutable_open_space_info()->set_destination_reached(true);
    return true;
  }
  return false;
}

bool OpenSpaceTrajectoryProvider::IsVehicleStopDueToFallBack(
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

// generate stop trajectory
void OpenSpaceTrajectoryProvider::GenerateStopTrajectory(
    DiscretizedTrajectory *const trajectory_data) {
  double relative_time = 0.0;
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

// get trajectory_data
void OpenSpaceTrajectoryProvider::LoadResult(
    const Frame *last_frame, DiscretizedTrajectory *const trajectory_data) {
  // Load unstitched two trajectories into frame for debug
  trajectory_data->clear();
  auto optimizer_trajectory_ptr =
      frame_->mutable_open_space_info()->mutable_optimizer_trajectory_data();
  open_space_trajectory_optimizer_->GetOptimizedTrajectory(
      optimizer_trajectory_ptr);

  auto stitching_trajectory_ptr =
      frame_->mutable_open_space_info()->mutable_stitching_trajectory_data();
  if (frame_->open_space_info().is_ready_to_second_plan()) {
    open_space_trajectory_optimizer_->GetStitchingTrajectory(
        stitching_trajectory_ptr);
  } else {
    open_space_trajectory_optimizer_->GetStitchingTrajectory(
        stitching_trajectory_ptr);

    auto *open_space_status = injector_->planning_context()
                                  ->mutable_planning_status()
                                  ->mutable_open_space();
    open_space_status->set_position_init(false);
  }

  // Stitch two trajectories and load back to trajectory_data from frame
  size_t optimizer_trajectory_size = optimizer_trajectory_ptr->size();
  double stitching_point_relative_time =
      stitching_trajectory_ptr->back().relative_time();
  double stitching_point_relative_s =
      stitching_trajectory_ptr->back().path_point().s();
  // update optimizer_trajectory_ptr relative_time and s
  for (size_t i = 0; i < optimizer_trajectory_size; ++i) {
    optimizer_trajectory_ptr->at(i).set_relative_time(
        optimizer_trajectory_ptr->at(i).relative_time() +
        stitching_point_relative_time);
    optimizer_trajectory_ptr->at(i).mutable_path_point()->set_s(
        optimizer_trajectory_ptr->at(i).path_point().s() +
        stitching_point_relative_s);
  }
  *(trajectory_data) = *(optimizer_trajectory_ptr);
  *last_plan_traj_ptr_ = *(optimizer_trajectory_ptr);

  // when is is_ready_to_second_plan ,Last point in stitching trajectory is
  // already in optimized trajectory, so it is deleted。
  // lwt: when only once plan , i think use optimizer_trajectory_ptr alone is
  // also ok
  if (frame_->open_space_info().is_ready_to_second_plan()) {
    frame_->mutable_open_space_info()
        ->mutable_stitching_trajectory_data()
        ->pop_back();

    // trajectory_data = stitching_trajectory + optimizer_trajectory
    trajectory_data->PrependTrajectoryPoints(
        frame_->open_space_info().stitching_trajectory_data());
  }

  if (trajectory_data->size() <= 2) {
    AERROR << "trajectory_data.size too low " << trajectory_data->size();
    return;
  }

  //---------------------------------------------
  frame_->mutable_open_space_info()->set_open_space_provider_success(true);

  if (!optimizer_trajectory_ptr->empty()) {
    size_t points_num = optimizer_trajectory_ptr->NumOfPoints();
    const auto &last_plan_first_point =
        optimizer_trajectory_ptr->TrajectoryPointAt(0).path_point();
    frame_->mutable_open_space_info()->mutable_rescue_first_end_point()->set_x(
        last_plan_first_point.x());
    frame_->mutable_open_space_info()->mutable_rescue_first_end_point()->set_y(
        last_plan_first_point.y());
    const auto &last_plan_second_point =
        optimizer_trajectory_ptr->TrajectoryPointAt(points_num - 1)
            .path_point();
    frame_->mutable_open_space_info()->mutable_rescue_second_end_point()->set_x(
        last_plan_second_point.x());
    frame_->mutable_open_space_info()->mutable_rescue_second_end_point()->set_y(
        last_plan_second_point.y());
    ADEBUG << "last_plan_second_point.x() " << last_plan_second_point.x()
          << "last_plan_second_point.y() " << last_plan_second_point.y();
  }
}

// reuse last frame result
void OpenSpaceTrajectoryProvider::ReuseLastFrameResult(
    const Frame *last_frame, DiscretizedTrajectory *const trajectory_data) {
  *(trajectory_data) =
      last_frame->open_space_info().stitched_trajectory_result();
  frame_->mutable_open_space_info()->set_open_space_provider_success(true);
  if (frame_->open_space_info().open_space_second_provider_success()) {
    // is start replan in roi decider
    frame_->mutable_open_space_info()->set_open_space_second_provider_success(
        false);
  } else {
    frame_->mutable_open_space_info()->set_open_space_second_provider_success(
        last_frame->open_space_info().open_space_second_provider_success());
  }

  *(frame_->mutable_open_space_info()->mutable_rescue_first_end_point()) =
      last_frame->open_space_info().rescue_first_end_point();

  *(frame_->mutable_open_space_info()->mutable_rescue_second_end_point()) =
      last_frame->open_space_info().rescue_second_end_point();
}

void OpenSpaceTrajectoryProvider::ReuseLastFrameDebug(const Frame *last_frame) {
  // reuse last frame's instance
  auto *ptr_debug = frame_->mutable_open_space_info()->mutable_debug_instance();
  ptr_debug->mutable_planning_data()->mutable_open_space()->MergeFrom(
      last_frame->open_space_info()
          .debug_instance()
          .planning_data()
          .open_space());
}

// the distance between planning_init_point and end_pose
double OpenSpaceTrajectoryProvider::GetDistanceToEndPose(
    const common::TrajectoryPoint &planning_init_point,
    const std::vector<double> &end_pose, double rotate_angle,
    const Vec2d &translate_origin) {
  CHECK_EQ(end_pose.size(), 4U);

  // end_pose  origin --> world
  Vec2d end_pose_to_world_frame(end_pose[0], end_pose[1]);
  end_pose_to_world_frame.SelfRotate(rotate_angle);
  end_pose_to_world_frame += translate_origin;

  const auto &path_point = planning_init_point.path_point();
  double distance_to_end_point =
      std::hypot(path_point.x() - end_pose_to_world_frame.x(),
                 path_point.y() - end_pose_to_world_frame.y());
  return distance_to_end_point;
}

}  // namespace planning
}  // namespace century
