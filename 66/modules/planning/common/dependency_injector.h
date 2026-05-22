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

#pragma once
#include <limits>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/planning/proto/lane_borrow_response.pb.h"
#include "modules/planning/proto/pass_stacker_response.pb.h"
#include "modules/planning/proto/stackers_info.pb.h"
#include "modules/planning/proto/v2x_info.pb.h"
#include "modules/planning/proto/planning_aeb.pb.h"

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/kalman_filter_vechicle_status.h"
#include "modules/planning/common/learning_based_data.h"
#include "modules/planning/common/planning_context.h"

namespace century {
namespace planning {
using BorrowObstacle = std::pair<int, Obstacle>;
using ObstacleId = std::pair<std::string, int32_t>;
using TargetStacker = std::pair<std::string, century::hdmap::Polygon>;
// AppearCounts of obstacle contains:
// 1. the number of consecutive occurrences,
// 2. the SequenceNumber of the latest occurrence,
// 3. whether there has ever been a non-UNKNOWN type obstacle.
using AppearCounts = std::tuple<size_t, uint32_t, bool>;
using AppearCountList = std::unordered_map<std::string, AppearCounts>;
using century::common::math::Vec2d;

enum class OpenspaceReason {
  NO_OPENSPACE = 0,
  OBSTACLE_STOPPING,
  PLANNING_ERROR,
  PLANNING_FAILED,
  ROUTING_FAILED,
  OPERATION,
};

enum AdcSpeedStatus {
  SPEED_NONE = 0,
  SPEED_UPPER = 1,
  SPEED_LOWER = 2,
};

enum VaildPathLable {
  DEFAULT = 0,
  FALLBACK,
  SELF,
  LEFT_BORROW,
  RIGHT_BORROW,
  LANE_CHANGE,
};

enum ReverseTrajectoryType {
  FORWARD_DRIVING = 0,
  OBSTACLE_AVOIDANCE = 1,
  TASK_POINT = 2,
  TASK_POINT_REVERSE_END = 3,
  ROUTING_REQUEST = 4,
};

class DependencyInjector {
 public:
  DependencyInjector() = default;
  ~DependencyInjector() = default;

  PlanningContext* planning_context() { return &planning_context_; }
  FrameHistory* frame_history() { return &frame_history_; }
  FrameHistory* frame_teb_history() { return &frame_teb_history_; }
  History* history() { return &history_; }
  EgoInfo* ego_info() { return &ego_info_; }
  century::common::VehicleStateProvider* vehicle_state() {
    return &vehicle_state_;
  }
  LearningBasedData* learning_based_data() { return &learning_based_data_; }
  const PathData& last_path_data() const { return last_path_data_; }
  PathData* mutable_last_path_data() { return &last_path_data_; }
  bool need_to_rescue() { return need_to_rescue_; }

  void set_need_to_rescue(bool need_to_rescue) {
    need_to_rescue_ = need_to_rescue;
  }
  bool need_to_rescue_thread() { return need_to_rescue_thread_; }

  void set_need_to_rescue_thread(bool need_to_rescue_thread) {
    need_to_rescue_thread_ = need_to_rescue_thread;
  }

  bool first_into_rescue() { return first_into_rescue_; }

  void set_first_into_rescue(bool first_into_rescue) {
    first_into_rescue_ = first_into_rescue;
  }

  void set_openspace_reason(OpenspaceReason reason) {
    openspace_reason_ = reason;
  }

  OpenspaceReason openspace_reason() { return openspace_reason_; }

  bool use_reverse_trajectory() { return is_use_reverse_trajectory_; }

  void set_use_reverse_trajectory(bool is_use_reverse_trajectory) {
    is_use_reverse_trajectory_ = is_use_reverse_trajectory;
  }

  void set_use_reverse_type(ReverseTrajectoryType reverse_type) {
    use_reverse_type_ = reverse_type;
  }
  void set_is_new_routing(bool is_new_routing) {
    is_new_routing_ = is_new_routing;
  }
  bool is_new_routing() { return is_new_routing_; }
  ReverseTrajectoryType use_reverse_type() { return use_reverse_type_; }
  bool need_request_borrow() { return need_request_borrow_; }

  void set_need_request_borrow(bool need_request_borrow) {
    need_request_borrow_ = need_request_borrow;
  }
  planning::BorrowResponse borrow_response() { return borrow_response_; }

  void set_borrow_response(planning::BorrowResponse borrow_response) {
    borrow_response_ = borrow_response;
  }

  planning::AebResult aeb_result() { return aeb_result_; }

  void set_aeb_result(planning::AebResult aeb_result) {
    aeb_result_ = aeb_result;
  }

  planning::V2xInfo v2x_info() { return v2x_info_; }

  void set_v2x_info(planning::V2xInfo v2x_info) { v2x_info_ = v2x_info; }
  planning::StackersInfo stackers_info() { return stackers_info_; }
  void set_stackers_info(planning::StackersInfo stackers_info) {
    stackers_info_ = stackers_info;
  }
  planning::PassStackerResponse pass_stacker_response() {
    return pass_stacker_response_;
  }

  void set_pass_stacker_response(
      planning::PassStackerResponse pass_stacker_response) {
    pass_stacker_response_ = pass_stacker_response;
  }

  void set_adc_lane_turn(hdmap::Lane_LaneTurn adc_lane_turn) {
    adc_lane_turn_ = adc_lane_turn;
  }

  hdmap::Lane_LaneTurn get_adc_lane_turn() { return adc_lane_turn_; }

  common::PointENU reverse_stop_point() { return reverse_stop_point_; }
  void set_reverse_stop_point(const common::PointENU& reverse_stop_point) {
    reverse_stop_point_.set_x(reverse_stop_point.x());
    reverse_stop_point_.set_y(reverse_stop_point.y());
    reverse_stop_point_.set_z(reverse_stop_point.z());
  }

  bool use_thread_in_play_street() { return use_thread_in_play_street_; }

  void set_use_thread_in_play_street(bool use_thread_in_play_street) {
    use_thread_in_play_street_ = use_thread_in_play_street;
  }

  bool first_into_pullover() { return first_into_pullover_; }

  void set_first_into_pullover(bool first_into_pullover) {
    first_into_pullover_ = first_into_pullover;
  }

  bool enable_rescue_pullover() { return enable_rescue_pullover_; }

  void set_enable_rescue_pullover(bool enable_rescue_pullover) {
    enable_rescue_pullover_ = enable_rescue_pullover;
  }

  bool on_lane_planning_result() { return on_lane_planning_result_; }

  void set_on_lane_planning_result(bool on_lane_planning_result) {
    on_lane_planning_result_ = on_lane_planning_result;
  }

  bool rescue_replan() { return rescue_replan_; }
  void set_rescue_replan(bool rescue_replan) { rescue_replan_ = rescue_replan; }

  bool rescue_crowded() { return rescue_crowded_; }
  void set_rescue_crowded(bool rescue_crowded) {
    rescue_crowded_ = rescue_crowded;
  }

  void set_lateral_diff(const double& lateral_diff) {
    lateral_diff_ = lateral_diff;
  }
  double get_lateral_diff() { return lateral_diff_; }
  void ResetTaskFailureInfo() {
    task_failure_count_ = 0;
    task_failure_name_ = "";
  }
  void SetTaskFailureCount(size_t count) { task_failure_count_ = count; }
  void AddTaskFailureCount() {
    if (std::numeric_limits<size_t>::max() > task_failure_count_) {
      ++task_failure_count_;
    }
  }
  size_t GetTaskFailureCount() const { return task_failure_count_; }

  void SetTaskFailureName(const std::string& name) {
    task_failure_name_ = name;
  }
  const std::string& GetTaskFailureName() const { return task_failure_name_; }

  void SetPoorStatusOfTaskFailure() { is_poor_status_of_task_failure_ = true; }
  void ClearPoorStatusOfTaskFailure() {
    is_poor_status_of_task_failure_ = false;
  }
  bool IsPoorStatusOfTaskFailure() const {
    return is_poor_status_of_task_failure_;
  }

  void SetSequenceNum(uint32_t seq_num) { sequence_num_ = seq_num; }
  uint32_t GetSequenceNum() { return sequence_num_; }

  void SetLatestSeqNumForSlowerDecel() {
    latest_sequence_num_for_slower_decel_ = sequence_num_;
  }
  uint32_t GetLatestSeqNumForSlowerDecel() {
    return latest_sequence_num_for_slower_decel_;
  }

  void SetLatestSeqNumForTargetSpeedSlowerDecel() {
    latest_sequence_num_for_target_speed_slower_decel_ = sequence_num_;
  }
  uint32_t GetLatestSeqNumForTargetSpeedSlowerDecel() {
    return latest_sequence_num_for_target_speed_slower_decel_;
  }

  void SetLatestSeqNumForApproachObsSlowerDecel() {
    latest_sequence_num_for_approach_obs_slower_decel_ = sequence_num_;
  }
  uint32_t GetLatestSeqNumForApproachObsSlowerDecel() {
    return latest_sequence_num_for_approach_obs_slower_decel_;
  }

  std::vector<BorrowObstacle>& GetBorrowObstacles() { return obs_borrow_; }
  void AddBorrowObstacle(const BorrowObstacle& obs) {
    obs_borrow_.push_back(obs);
  }
  void ClearBorrowObstacles() { obs_borrow_.clear(); }
  const std::vector<BorrowObstacle>& GetDisappearObstacles() {
    return obs_disappear_;
  }
  void AddDisappearedObstacle(const BorrowObstacle& obs) {
    obs_disappear_.push_back(obs);
  }
  void ClearDisappearedObstacles() { obs_disappear_.clear(); }
  const AppearCountList& GetAppearedObstacles() {
    return obstacle_appear_counts_;
  }
  AppearCountList* GetPtrAppearedObstacles() {
    return &obstacle_appear_counts_;
  }
  std::map<int32_t, BorrowObstacle>& GetStartMovingObstacles() {
    return obs_start_moving_;
  }
  void AddStartMovingObstacle(const int32_t obs_id, const BorrowObstacle& obs) {
    obs_start_moving_.insert({obs_id, obs});
  }
  void ClearStartMovingObstacles() { obs_start_moving_.clear(); }

  KalmanFilterForVehicleStatus* GetVehicleStateKalman() {
    return &vehicle_state_kalman_;
  }
  double DiffTimeStamp(double now_time) {
    if (now_time <= last_time_kalman_) {
      AERROR << "time reversal!";
      return 0.0;
    }
    if (0.0 == last_time_kalman_) {
      return 0.0;
    }
    double diff_time = now_time - last_time_kalman_;
    last_time_kalman_ = now_time;
    return diff_time;
  }
  double GetDrivingDistance() { return driving_distance_; }
  void UpdateDrivingDistance(double vehicle_x, double vehicle_y,
                             bool is_forward) {
    double distance_coeff = is_forward ? 1.0 : -1.0;
    if (0.0 == last_vehicle_x_ || 0.0 == last_vehicle_y_) {
      driving_distance_ = 0.0;
    } else {
      double delta_distance = std::sqrt(
          (vehicle_x - last_vehicle_x_) * (vehicle_x - last_vehicle_x_) +
          (vehicle_y - last_vehicle_y_) * (vehicle_y - last_vehicle_y_));
      driving_distance_ += (distance_coeff * delta_distance);
    }
    last_vehicle_x_ = vehicle_x;
    last_vehicle_y_ = vehicle_y;
  }

  const AdcSpeedStatus& GetAdcSpeedStatus() { return adc_speed_status_; }
  void UpdateAdcSpeedStatus(const double& adc_speed) {
    if (AdcSpeedStatus::SPEED_LOWER == adc_speed_status_) {
      if (adc_speed > FLAGS_lane_borrow_max_speed +
                          FLAGS_adc_speed_hysteresis_upper_limit) {
        adc_speed_status_ = AdcSpeedStatus::SPEED_UPPER;
      }
    } else if (AdcSpeedStatus::SPEED_UPPER == adc_speed_status_) {
      if (adc_speed < FLAGS_lane_borrow_max_speed -
                          FLAGS_adc_speed_hysteresis_lower_limit) {
        adc_speed_status_ = AdcSpeedStatus::SPEED_LOWER;
      }
    } else {
      if (adc_speed < FLAGS_lane_borrow_max_speed) {
        adc_speed_status_ = AdcSpeedStatus::SPEED_LOWER;
      } else {
        adc_speed_status_ = AdcSpeedStatus::SPEED_UPPER;
      }
    }
  }

  void SetReinitStartPoint() {
    is_need_to_reinit_stitching_trajectory_ = true;
    sequence_num_last_reinit_ = sequence_num_;
  }

  void ResetReinitStartPoint() {
    is_need_to_reinit_stitching_trajectory_ = false;
  }

  bool NeedReinitStartPoint() {
    return is_need_to_reinit_stitching_trajectory_;
  }

  void SetReplanState(const bool is_replan) { is_replan_ = is_replan; }

  bool GetReplanState() const { return is_replan_; }

  void SetNeedToReplan(const bool is_replan) { is_need_to_replan_ = is_replan; }

  bool NeedToReplan() const { return is_need_to_replan_; }
  void SetLastFrameNeedDiagonal(const bool last_frame_need_diagonal) {
    last_frame_need_diagonal_ = last_frame_need_diagonal;
  }

  bool LastFrameNeedDiagonal() const { return last_frame_need_diagonal_; }
  bool IsVehCollisionElectricFence() const {
    return is_veh_collision_electric_fence_;
  }
  void SetIsVehCollisionElectricFence(const bool is_veh_collision_electric_fence) {
    is_veh_collision_electric_fence_ = is_veh_collision_electric_fence;
  }
  uint32_t GetDiffSeqNumFromLastReinit() {
    return (sequence_num_ - sequence_num_last_reinit_);
  }

  void SetReroutinForHuman(const bool is_rerouting) { is_enter_human_rerouting_ = is_rerouting; }

  bool GetReroutinForHuman() const { return is_enter_human_rerouting_; }

  bool pullover_using_ = false;
  bool pullover_end_trace_ = false;
  bool pullover_finished = false;
  bool teb_adc_is_out_lane_ = false;
  bool is_in_common_junction_ = false;
  bool is_in_play_street = false;
  bool can_borrow_pedestrian_ = false;
  double last_using_lateral_ = 0.0;
  double fallback_endstate = 0.0;
  double left_laneborrow_endstate = 0.0;
  double right_laneborrow_endstate = 0.0;
  double selfborrow_endstate = 0.0;
  VaildPathLable last_path_label_ = DEFAULT;
  bool is_need_to_keep_right_ = false;
  double nearst_obs_s_ = 0.0;
  ObstacleId reverse_obj_ = {"\0", 0};
  double reverse_obstacle_start_l_ = 0.0;
  bool near_traffic_line_ = false;
  bool is_adc_in_junction_ = false;
  bool is_multi_reference_line_ = false;
  bool use_teb_default_bound_ = false;
  bool start_collision_flag_ = false;
  bool deal_start_block_ = false;
  bool shrink_end_s_ = false;
  bool is_personlike_blocked_ = false;
  bool is_adc_out_lane_ = false;
  bool is_adc_deviate_lane_direction_ = false;
  bool is_need_to_uturn_ = false;
  common::math::Vec2d last_signal_overlap_end_xy_ = Vec2d(0.0, 0.0);
  double is_can_enter_mixed_flow_ = false;
  bool is_reach_search_end_point_ = false;
  bool is_in_near_goal_ = false;
  bool is_reach_goal_ = false;
  bool is_uturn_blocked_ = false;
  bool is_in_traffic_light_junction_ = false;
  uint32_t near_junction_borrow_check_times_ = 0;
  bool near_junction_keep_straight_ = false;
  double adc_precessor_lane_heading_ = 0.0;
  bool is_turn_right_ = false;
  bool is_teb_overtime_ = false;
  bool is_auxiliary_road_ = false;
  bool is_single_lane_ = false;
  bool is_use_reverse_trajectory_ = false;
  bool need_request_borrow_ = false;
  int self_borrow_check_times_ = 0;
  int back_ward_check_times_ = 0;
  bool need_back_ward_ = false;
  double back_ward_distance_ = 0.0;
  bool is_auto_state_ = false;
  bool is_last_auto_state_ = false;
  int auto_state_count_ = 0;
  std::pair<bool, TargetStacker> target_stacker_info_;
  int target_stacker_check_times_ = 0;  
  int stacker_change_times_ = 0;
  int stacker_disappear_times_ = 0;
  int stacker_discover_times_ = 0; 
  int working_stacker_count_times_ = 0;
  std::string borro_obs_name_ = "";
  double borrow_obs_consider_l_ = 0.0;
  int stacker_static_to_dynamic_times_ = 0;
  bool is_manual_pass_stacker_ = false;
  std::string pass_stacker_id_ = "";
  bool is_new_routing_for_replan_ = false;
  bool is_need_rerouting_for_block_ = false;
  bool is_pass_dynamic_stacker_ = false;
  int path_block_count_ = 0;

  int end_state_shake_check_times_ = 0;
  int end_state_return_check_times_ = 0;
  bool enable_self_borrow_ = false;
  planning::BorrowResponse borrow_response_;
  planning::PassStackerResponse pass_stacker_response_;
  PassStackerRequest wheelcrane_notice_;
  planning::StackersInfo stackers_info_;
  planning::V2xInfo v2x_info_;
  planning::AebResult aeb_result_;
  common::math::Vec2d stacker_point_ = Vec2d(0.0, 0.0);
  int pass_stacker_count_ = 0;
  int adc_stop_count_ = 0;
  bool enable_shrink_collision_buffer_ = false;
  std::string shrink_collision_id_ = "";
  int adc_block_count_ = 0;
  int adc_stable_stop_count_ = 0;
  int start_up_safe_check_count_ = 0;
  double start_up_safe_check_s_ = 0.0;
  common::PointENU reverse_stop_point_;
  ReverseTrajectoryType use_reverse_type_ =
      ReverseTrajectoryType::FORWARD_DRIVING;
  bool is_new_routing_ = false;
  // adc_in_junction_info_.first is current frame info,
  // adc_in_junction_info_.second is last frame info.
  std::pair<bool, bool> adc_in_junction_info_ = {false, false};
  bool is_off_lane_depart_ = false;
  std::atomic<bool> exit_from_teb_ = {false};
  bool is_adc_in_expressway_junction_ = false;
  bool is_adc_in_gate_junction_ = false;

 private:
  PlanningContext planning_context_;
  FrameHistory frame_history_;
  FrameHistory frame_teb_history_;
  History history_;
  EgoInfo ego_info_;
  century::common::VehicleStateProvider vehicle_state_;
  LearningBasedData learning_based_data_;
  double lateral_diff_ = 0.0;
  // ------------Just For Rescue Scenario-------------------------
  bool need_to_rescue_ = false;
  bool need_to_rescue_thread_ = false;
  bool enable_rescue_pullover_ = false;
  bool first_into_rescue_ = false;
  bool use_thread_in_play_street_ = false;

  bool first_into_pullover_ = false;

  bool on_lane_planning_result_ = false;
  bool rescue_replan_ = false;
  bool rescue_crowded_ = false;
  OpenspaceReason openspace_reason_ = OpenspaceReason::NO_OPENSPACE;
  // ------------Just For Rescue Scenario-------------------------
  size_t task_failure_count_ = 0;
  bool is_poor_status_of_task_failure_ = false;
  std::string task_failure_name_ = "";
  std::vector<BorrowObstacle> obs_borrow_;
  std::vector<BorrowObstacle> obs_disappear_;
  std::map<int32_t, BorrowObstacle> obs_start_moving_;
  AppearCountList obstacle_appear_counts_;
  uint32_t sequence_num_ = 0U;
  uint32_t latest_sequence_num_for_slower_decel_ = 0U;
  uint32_t latest_sequence_num_for_target_speed_slower_decel_ = 0U;
  uint32_t latest_sequence_num_for_approach_obs_slower_decel_ = 0U;
  uint32_t sequence_num_last_reinit_ = 0U;
  KalmanFilterForVehicleStatus vehicle_state_kalman_;
  double last_time_kalman_ = 0.0;
  double driving_distance_ = 0.0;
  double last_vehicle_x_ = 0.0;
  double last_vehicle_y_ = 0.0;
  PathData last_path_data_;
  AdcSpeedStatus adc_speed_status_ = AdcSpeedStatus::SPEED_NONE;
  bool is_need_to_reinit_stitching_trajectory_ = false;
  bool is_replan_ = false;
  bool is_need_to_replan_ = false;
  bool last_frame_need_diagonal_ = false;
  bool is_veh_collision_electric_fence_ = false;
  bool is_enter_human_rerouting_ = false;
  hdmap::Lane_LaneTurn adc_lane_turn_ =
      hdmap::Lane_LaneTurn::Lane_LaneTurn_NO_TURN;
};

}  // namespace planning
}  // namespace century
