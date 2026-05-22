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

#include "gflags/gflags.h"

DECLARE_bool(planning_test_mode);

DECLARE_int32(history_max_record_num);
DECLARE_int32(max_frame_history_num);

// scenarios related
DECLARE_string(scenario_bare_intersection_unprotected_config_file);
DECLARE_string(scenario_emergency_pull_over_config_file);
DECLARE_string(scenario_emergency_stop_config_file);
DECLARE_string(scenario_lane_follow_config_file);
DECLARE_string(scenario_lane_follow_hybrid_config_file);
DECLARE_string(scenario_learning_model_sample_config_file);
DECLARE_string(scenario_narrow_street_u_turn_config_file);
DECLARE_string(scenario_park_and_go_config_file);
DECLARE_string(scenario_pull_over_config_file);
DECLARE_string(scenario_stop_sign_unprotected_config_file);
DECLARE_string(scenario_traffic_light_protected_config_file);
DECLARE_string(scenario_traffic_light_protected_teb_config_file);
DECLARE_string(scenario_traffic_light_unprotected_left_turn_config_file);
DECLARE_string(scenario_traffic_light_unprotected_right_turn_config_file);
DECLARE_string(scenario_valet_parking_config_file);
DECLARE_string(scenario_deadend_turnaround_config_file);
DECLARE_string(scenario_yield_sign_config_file);
DECLARE_string(scenario_rescue_config_file);
DECLARE_string(scenario_rescue_teb_config_file);
DECLARE_string(scenario_uturn_teb_config_file);

DECLARE_string(rescue_status_config_file);

DECLARE_bool(enable_stitching_with_prediction);
DECLARE_bool(enable_scenario_bare_intersection);
DECLARE_bool(enable_scenario_emergency_pull_over);
DECLARE_bool(enable_scenario_emergency_stop);
DECLARE_bool(enable_scenario_park_and_go);
DECLARE_bool(enable_scenario_pull_over);
DECLARE_bool(enable_scenario_stop_sign);
DECLARE_bool(enable_scenario_traffic_light);
DECLARE_bool(enable_scenario_yield_sign);

DECLARE_bool(enable_scenario_traffic_teb_instead);

DECLARE_bool(enable_scenario_side_pass_multiple_parked_obstacles);
DECLARE_bool(enable_force_pull_over_open_space_parking_test);
DECLARE_bool(enable_use_lane_as_boundary);

DECLARE_string(traffic_rule_config_filename);
DECLARE_string(smoother_config_filename);
DECLARE_int32(planning_loop_rate);
DECLARE_string(rtk_trajectory_filename);
DECLARE_uint64(rtk_trajectory_forward);
DECLARE_double(rtk_trajectory_resolution);
DECLARE_bool(enable_reference_line_stitching);
DECLARE_double(look_forward_extend_distance);
DECLARE_double(reference_line_stitch_overlap_distance);

DECLARE_bool(enable_smooth_reference_line);

DECLARE_bool(enable_canbus_info_conver);

DECLARE_bool(prioritize_change_lane);
DECLARE_double(change_lane_min_length);

DECLARE_bool(publish_estop);
DECLARE_bool(enable_trajectory_stitcher);
DECLARE_bool(enable_use_steer);
DECLARE_bool(enable_use_minimum_turning_radius_to_get_kappa);
DECLARE_bool(enable_acc_start);

DECLARE_double(look_backward_distance);
DECLARE_double(look_forward_short_distance);
DECLARE_double(look_forward_long_distance);

// parameters for trajectory stitching and reinit planning starting point.
DECLARE_double(replan_lateral_distance_threshold);
DECLARE_double(replan_longitudinal_distance_threshold);

// parameter for reference line
DECLARE_bool(enable_reference_line_provider_thread);
DECLARE_bool(enable_reverse_trajectory);
DECLARE_bool(enable_rerouting_for_block);
DECLARE_bool(enable_backward_for_obs_block);
DECLARE_bool(enable_backward_for_routing_request);
DECLARE_bool(enable_backward_for_task_start_blocking);
DECLARE_double(task_point_reverse_length);
DECLARE_double(min_brorrow_length);
DECLARE_double(default_reference_line_width);
DECLARE_double(smoothed_reference_line_max_diff);
DECLARE_double(max_centripetal_acceleration_threshold);

// parameter for aeb planning
DECLARE_bool(enable_aeb_planning);
DECLARE_bool(enable_aeb_speed_filter);
DECLARE_double(aeb_obstacle_distance_threshold);
DECLARE_double(aeb_obstacle_length_buffer);
DECLARE_double(aeb_obstacle_width_buffer);
DECLARE_double(aeb_obstacle_overlap_length_buffer);
DECLARE_double(aeb_obstacle_overlap_width_buffer);
DECLARE_double(behind_ego_length_buffer);
DECLARE_double(behind_ego_width_buffer);
DECLARE_double(lateral_ttc_low_threshold);
DECLARE_double(long_ttc_low_threshold);
DECLARE_double(lateral_ttc_medium_threshold);
DECLARE_double(long_ttc_medium_threshold);
DECLARE_double(lateral_ttc_high_threshold);
DECLARE_double(long_ttc_high_threshold);

// parameters for trajectory planning
DECLARE_double(planning_upper_speed_limit);
DECLARE_double(play_street_speed_limit);
DECLARE_double(overtake_upper_speed_limit);
DECLARE_double(auxiliary_road_limit_speed);
DECLARE_double(overtake_speed_up_ratio);
DECLARE_bool(enable_overtake_speed_up);
DECLARE_double(trajectory_time_length);
DECLARE_double(trajectory_time_min_interval);
DECLARE_double(trajectory_time_max_interval);
DECLARE_double(trajectory_time_high_density_period);

// parameters for trajectory sanity check
DECLARE_bool(enable_trajectory_check);
DECLARE_double(speed_lower_bound);
DECLARE_double(speed_upper_bound);

DECLARE_double(longitudinal_acceleration_lower_bound);
DECLARE_double(longitudinal_acceleration_upper_bound);

DECLARE_double(longitudinal_jerk_lower_bound);
DECLARE_double(longitudinal_jerk_upper_bound);
DECLARE_double(longitudinal_jerk_upper_steady);
DECLARE_double(lateral_jerk_bound);
DECLARE_double(min_jerk_for_const_accel_speed_plan);
DECLARE_double(max_jerk_for_const_accel_speed_plan);

DECLARE_double(kappa_bound);

DECLARE_bool(enable_longitudinal_accel_and_jerk_constraint);

// STBoundary
DECLARE_double(st_max_s);
DECLARE_double(st_max_t);

// Decision Part
DECLARE_bool(enable_nudge_slowdown);
DECLARE_bool(enable_follow_slowdown);
DECLARE_bool(enable_yield_slowdown);
DECLARE_bool(enable_stop_slowdown);
DECLARE_bool(enable_pedestrian_slowdown);
DECLARE_bool(enable_bicycle_slowdown);
DECLARE_double(obs_cross_angle_degree);
DECLARE_double(hy_buffer_of_cross_angle);

DECLARE_double(static_obstacle_nudge_l_buffer);
DECLARE_double(nonstatic_obstacle_nudge_l_buffer);

DECLARE_double(lateral_ignore_buffer);
DECLARE_double(lateral_check_buffer);
DECLARE_double(longitudinal_check_buffer);
DECLARE_double(min_stop_distance_obstacle);
DECLARE_double(max_stop_distance_obstacle);
DECLARE_double(follow_min_distance);
DECLARE_double(max_yield_buffer);
DECLARE_double(follow_min_obs_lateral_distance);
DECLARE_double(yield_distance);
DECLARE_double(weighing_stop_distance);
DECLARE_double(follow_time_buffer);
DECLARE_double(follow_min_time_sec);
DECLARE_double(signal_expire_time_sec);
DECLARE_double(prediction_expire_time_sec);
DECLARE_double(max_stop_speed);
DECLARE_double(stop_speed_buffer);
DECLARE_double(stop_distance_buffer);
DECLARE_double(buffer_degrees);
DECLARE_double(forward_buffer_degrees);
DECLARE_double(ignore_obstacle_l_buffer);
DECLARE_double(ignore_obstacle_speed_coeff);
DECLARE_double(ignore_obstacle_speed_coeff_radical);
DECLARE_double(ignore_obstacle_acceleration_buffer);
DECLARE_double(min_dynamic_obstacle_speed);
DECLARE_double(wheelcrane_consider_distance);
// electric fence
DECLARE_bool(enable_electric_fence_drivable_area);
DECLARE_bool(enable_not_auto_electric_fence_drivable_area);
DECLARE_double(electric_fence_max_deceleration);
DECLARE_double(electric_fence_min_lat_velocity);
DECLARE_double(electric_fence_min_lon_velocity);

// Path Deciders
DECLARE_bool(enable_speed_limit_for_obstacle);
DECLARE_bool(enable_skip_path_tasks);
DECLARE_double(stop_distance_to_obstacle);
DECLARE_double(stop_distance_to_stacker);
DECLARE_double(license_plate_recognition_distance);
DECLARE_double(stop_distance_to_obstacle_far);
DECLARE_bool(enable_borrow_request);
DECLARE_bool(enable_self_borrow);
DECLARE_bool(enable_check_self_path_has_turn);
DECLARE_bool(enable_check_times_for_borrow_request);
DECLARE_bool(enable_modify_stop_distance);
DECLARE_bool(only_use_one_trajectory);

DECLARE_bool(enable_trim_unknown_obstacle);
DECLARE_double(trim_polygon_check_step);

DECLARE_double(adc_speed_low_threshold_public_road);
DECLARE_double(obstacle_max_lat_buffer_public_road);
DECLARE_double(obstacle_min_lat_buffer_public_road);

DECLARE_double(static_obstacle_speed_threshold);
DECLARE_double(static_unknown_obstacle_speed_threshold);
DECLARE_double(static_obstacle_speed_hysteresis_relative_lower_limit);
DECLARE_double(static_obstacle_speed_hysteresis_relative_upper_limit);
DECLARE_double(block_obstacle_lat_dis_hysteresis_width);
DECLARE_double(lane_borrow_max_speed);
DECLARE_double(adc_speed_hysteresis_lower_limit);
DECLARE_double(adc_speed_hysteresis_upper_limit);
DECLARE_double(lane_borrow_ttc_time);
DECLARE_double(lane_borrow_distance_pedestrian);
DECLARE_double(lane_borrow_distance_car);
DECLARE_double(lane_borrow_distance_unknown);

DECLARE_string(destination_obstacle_id);
DECLARE_double(destination_check_distance);

DECLARE_double(virtual_stop_wall_length);
DECLARE_double(virtual_stop_wall_height);

DECLARE_double(prediction_total_time);
DECLARE_bool(align_prediction_time);
DECLARE_bool(enable_right_borrow);
DECLARE_int32(trajectory_point_num_for_debug);
DECLARE_double(lane_change_prepare_length);
DECLARE_double(min_lane_change_prepare_length);
DECLARE_double(allowed_lane_change_failure_time);
DECLARE_bool(enable_smarter_lane_change);

DECLARE_double(turn_signal_distance);

DECLARE_double(min_step_end_state_l);
DECLARE_double(max_step_end_state_l);
DECLARE_double(min_speed_step_end_state_l);
DECLARE_double(max_speed_step_end_state_l);

DECLARE_string(path_label_is_fallback);
DECLARE_string(path_label_is_self);
DECLARE_string(path_label_is_left_borrow);
DECLARE_string(path_label_is_right_borrow);
DECLARE_string(path_label_is_lane_change);

// QpSt optimizer
DECLARE_double(slowdown_profile_deceleration);
DECLARE_double(max_centric_acceleration_limit);

DECLARE_bool(enable_sqp_solver);

// Kinematic speed optimizer
DECLARE_bool(enable_kinematic_speed_optimizer);
DECLARE_bool(enable_add_obs);

/// thread pool
DECLARE_bool(use_multi_thread_to_add_obstacles);
DECLARE_bool(enable_multi_thread_in_dp_st_graph);

DECLARE_double(numerical_epsilon);
DECLARE_double(default_cruise_speed);

DECLARE_double(trajectory_time_resolution);
DECLARE_double(trajectory_space_resolution);
DECLARE_double(lateral_acceleration_bound);
DECLARE_double(speed_lon_decision_horizon);
DECLARE_uint64(num_velocity_sample);
DECLARE_bool(enable_backup_trajectory);
DECLARE_double(backup_trajectory_cost);
DECLARE_double(min_velocity_sample_gap);
DECLARE_double(lon_collision_buffer);
DECLARE_double(lat_collision_buffer);
DECLARE_uint64(num_sample_follow_per_timestamp);

DECLARE_bool(lateral_optimization);
DECLARE_double(weight_lateral_offset);
DECLARE_double(weight_lateral_derivative);
DECLARE_double(weight_lateral_second_order_derivative);
DECLARE_double(weight_lateral_third_order_derivative);
DECLARE_double(weight_lateral_obstacle_distance);
DECLARE_double(lateral_third_order_derivative_max);

// Lattice Evaluate Parameters
DECLARE_double(weight_lon_objective);
DECLARE_double(weight_lon_jerk);
DECLARE_double(weight_lon_collision);
DECLARE_double(weight_lat_offset);
DECLARE_double(weight_lat_comfort);
DECLARE_double(weight_centripetal_acceleration);
DECLARE_double(cost_non_priority_reference_line);
DECLARE_double(weight_same_side_offset);
DECLARE_double(weight_opposite_side_offset);
DECLARE_double(weight_dist_travelled);
DECLARE_double(weight_target_speed);
DECLARE_double(lat_offset_bound);
DECLARE_double(lon_collision_yield_buffer);
DECLARE_double(lon_collision_overtake_buffer);
DECLARE_double(lon_collision_cost_std);
DECLARE_double(default_lon_buffer);
DECLARE_double(time_min_density);
DECLARE_double(comfort_acceleration_factor);
DECLARE_double(polynomial_minimal_param);
DECLARE_double(lattice_stop_buffer);
DECLARE_double(max_s_lateral_optimization);
DECLARE_double(default_delta_s_lateral_optimization);
DECLARE_double(bound_buffer);
DECLARE_double(nudge_buffer);

DECLARE_double(fallback_deceleration);
DECLARE_double(fallback_total_time);
DECLARE_double(fallback_time_unit);

DECLARE_double(speed_bump_speed_limit);
DECLARE_double(default_city_road_speed_limit);
DECLARE_double(default_highway_speed_limit);

// correct obstacle speed
DECLARE_bool(enable_correct_obstacle_speed);
DECLARE_double(left_scope_dis_for_correct_speed);
DECLARE_double(right_scope_dis_for_correct_speed);
DECLARE_double(front_scope_dis_for_correct_speed);
DECLARE_double(rear_scope_dis_for_correct_speed);
DECLARE_double(aspect_range_ratio_for_correct_speed);
DECLARE_double(speed_correct_threshold);

// navigation mode
DECLARE_bool(enable_planning_pad_msg);

// open space planner
DECLARE_string(planner_open_space_config_filename);
DECLARE_double(open_space_planning_period);
DECLARE_double(open_space_prediction_time_horizon);
DECLARE_bool(enable_perception_obstacles);
DECLARE_bool(enable_open_space_planner_thread);
DECLARE_bool(enable_teb_planner_thread);
DECLARE_bool(use_dual_variable_warm_start);
DECLARE_bool(use_gear_shift_trajectory);
DECLARE_uint64(open_space_trajectory_stitching_preserved_length);
DECLARE_bool(enable_smoother_failsafe);
DECLARE_bool(use_s_curve_speed_smooth);
DECLARE_bool(use_iterative_anchoring_smoother);
DECLARE_bool(enable_parallel_trajectory_smoothing);

DECLARE_bool(enable_osqp_debug);
DECLARE_bool(export_chart);
DECLARE_bool(enable_record_debug);
DECLARE_bool(enable_openspace_record_debug);

DECLARE_double(default_front_clear_distance);

DECLARE_double(max_trajectory_len);
DECLARE_bool(enable_rss_fallback);
DECLARE_bool(enable_rss_info);
DECLARE_double(rss_max_front_obstacle_distance);

DECLARE_bool(enable_planning_smoother);
DECLARE_double(smoother_stop_distance);

DECLARE_double(side_pass_driving_width_l_buffer);

DECLARE_bool(enable_parallel_hybrid_a);
DECLARE_bool(enable_pure_astar);

DECLARE_double(open_space_standstill_acceleration);

DECLARE_bool(enable_dp_reference_speed);

DECLARE_double(message_latency_threshold);
DECLARE_bool(enable_lane_change_urgency_checking);
DECLARE_double(short_path_length_threshold);

DECLARE_uint64(trajectory_stitching_preserved_length);

DECLARE_bool(use_st_drivable_boundary);

DECLARE_bool(use_smoothed_dp_guide_line);

DECLARE_bool(use_soft_bound_in_nonlinear_speed_opt);

DECLARE_bool(use_front_axe_center_in_path_planning);

DECLARE_bool(use_road_boundary_from_map);

DECLARE_double(threshold_distance_for_destination);

DECLARE_double(open_space_threshold_distance_for_destination);

DECLARE_double(buffer_in_routing);

DECLARE_double(buffer_out_routing);

// learning related
DECLARE_bool(planning_offline_learning);
DECLARE_string(planning_data_dir);
DECLARE_string(planning_offline_bags);
DECLARE_int32(learning_data_obstacle_history_time_sec);
DECLARE_int32(learning_data_frame_num_per_file);
DECLARE_string(planning_birdview_img_feature_renderer_config_file);
DECLARE_int32(min_past_history_points_len);

// hybrid model
DECLARE_bool(skip_path_reference_in_side_pass);
DECLARE_bool(skip_path_reference_in_change_lane);

// routing change lane
DECLARE_double(brake_buffer_for_lane_change);
DECLARE_double(limit_stop_wall_for_lane_change);
DECLARE_double(preview_brake_distance_for_lane_change);
DECLARE_double(soft_deceleration_for_lane_change_stop);
DECLARE_double(min_stop_distance_for_lane_change_wall);
DECLARE_int32(prediction_trajectory_relative_time_index);

// fallback planning
DECLARE_bool(enable_fallback_planning_thread);
DECLARE_bool(enable_TEB_thread);

DECLARE_bool(enable_skip_motion_obstacle);
DECLARE_bool(enable_skip_back_obstacle_in_the_same_line);
DECLARE_bool(enable_slowdown_for_obstacle);
DECLARE_bool(enable_shrink_reference_line);
DECLARE_bool(enable_intelligent_projection);
DECLARE_bool(enable_pre_build_sl_boundary);
DECLARE_bool(enable_skip_back_obstacles);
DECLARE_bool(enable_yield_for_min_lateral_obs);
DECLARE_bool(enable_add_adc_buffer);
DECLARE_bool(enable_check_back_side_obstacle);
DECLARE_bool(enable_use_lastpassage);
DECLARE_bool(enable_shrink_yield_distance);
DECLARE_bool(enable_shrink_stop_distance_for_pedestrian);
DECLARE_bool(enable_use_radical_decision);
DECLARE_bool(enable_slow_breaking);
DECLARE_bool(enable_slow_breaking_for_cutin);
DECLARE_bool(enable_slow_breaking_for_abnormal_prediction);
DECLARE_bool(enable_slow_breaking_for_large_ttc);
DECLARE_bool(enable_slow_breaking_for_reverse_obs);
DECLARE_bool(enable_yield_for_high_speed_bicycle);
DECLARE_bool(enable_use_new_method_to_get_decel_for_approaching_obs);
DECLARE_bool(enable_use_new_method_to_get_decel_for_large_speed_obs);
DECLARE_bool(enable_no_emergency_break_reverse_obs);
DECLARE_bool(enable_use_adc_lane_width_for_reverse_obs);
DECLARE_bool(enable_skip_back_side_and_has_overlap_obs);
DECLARE_bool(enable_skip_back_side_in_laneborrow_return);
DECLARE_bool(enable_use_boundary_check_obs_on_lane);
DECLARE_bool(enable_use_boundary_check_adc_on_lane);
DECLARE_bool(enable_slow_down_for_cross_obstacle);
DECLARE_bool(enable_slow_down_for_reverse_obstacle);
DECLARE_bool(enable_no_slow_down_for_overtake_obstacle);
DECLARE_bool(enable_enter_mixed_flow_mode);
DECLARE_bool(enable_high_speed);
DECLARE_bool(enable_use_reference_v_for_speed_optimizer);
DECLARE_bool(enable_no_shrink_upper_bound);
DECLARE_double(max_overtake_longitude_buffer);
DECLARE_double(min_overtake_longitude_buffer);
DECLARE_bool(enable_replan_for_smaller_start_point);
DECLARE_bool(enable_replan_for_diagobal_change_to_normal);
DECLARE_bool(enable_dynamic_modify_piecewise_jerk_weight);

DECLARE_double(stop_distance_in_common_junction);
DECLARE_double(lower_speed_in_public_road);
DECLARE_bool(enable_make_dangerous_start_up);
DECLARE_bool(enable_vehicle_start_up);
DECLARE_bool(enable_radical_change_lane_in_merge_area);
DECLARE_bool(enable_speed_limit_for_approaching_obs);
DECLARE_bool(enable_kinematic_speed_slowly_down);
DECLARE_bool(enable_obstacle_sidepass_decision);
DECLARE_bool(enable_cancel_stop_decision_from_heading);
DECLARE_bool(enable_cancel_stop_decision_from_heading_rate);
DECLARE_uint32(max_record_times_for_correct_speed);
DECLARE_uint32(max_record_times_for_start_up);
DECLARE_uint32(max_record_times_for_diff_s);
DECLARE_double(stable_speed_buffer);
DECLARE_double(average_step_value);
DECLARE_double(four_times_step_value);
DECLARE_double(three_times_step_value);
DECLARE_double(average_step_value_for_already_stop);
DECLARE_double(average_step_distance_for_already_stop);
DECLARE_double(first_level_average_step_distance);
DECLARE_double(second_level_average_step_distance);
DECLARE_double(average_step_distance_for_already_stop_loose_constraint);
DECLARE_double(first_level_average_step_distance_loose_constraint);
DECLARE_double(second_level_average_step_distance_loose_constraint);
DECLARE_double(average_step_value_tightly);
DECLARE_double(four_times_step_value_tightly);
DECLARE_double(three_times_step_value_tightly);
DECLARE_double(average_step_value_for_already_stop_tightly);
DECLARE_double(average_step_value_slower_near);
DECLARE_double(four_times_step_value_slower_near);
DECLARE_double(three_times_step_value_slower_near);
DECLARE_double(average_step_value_for_already_stop_slower_near);
DECLARE_uint32(lost_keep_move_near_times);

DECLARE_bool(enable_emergency_speed_fallback);
DECLARE_double(lower_speed_for_speed_fallback);
DECLARE_double(lower_kappa_for_speed_fallback);

DECLARE_double(max_diff_angle_for_stoped_collision_check);
DECLARE_double(hy_buffer_lower_for_stoped_collision_check);
DECLARE_double(hy_buffer_upper_for_stoped_collision_check);

DECLARE_bool(enable_slower_deceleration_after_qp_failure);

DECLARE_bool(enable_use_drive_area);
// planning learning
DECLARE_bool(enable_record_to_learning_data_for_svm);
DECLARE_bool(enable_txt_to_bin_for_svm);
DECLARE_bool(svm_switch);
DECLARE_bool(enable_use_svm_model);
DECLARE_bool(enable_change_acc_weight_for_svm);

// lane_borrow decider
DECLARE_bool(consider_obstacle_blocked);
DECLARE_double(static_obstacle_buffer);
DECLARE_double(block_adc_center_view_buffer);
DECLARE_bool(enable_near_junction_laneborrow);
DECLARE_double(borrow_slow_obstacle_velocity_threshold);
DECLARE_double(routing_lane_length_threshold);
DECLARE_bool(enable_extend_path_bound_base_adc_posture);
DECLARE_double(adc_posture_correct_check_heading_diff);
DECLARE_bool(enable_separate_auxiliary_road_borrow);
DECLARE_bool(enable_efficiency_shift_borrow);
DECLARE_bool(enable_auto_borrow);
DECLARE_bool(enable_auto_lane_borrow);

DECLARE_bool(allow_lane_borrow_fsm);
DECLARE_bool(allow_smi_diagonal);
DECLARE_bool(allow_narrow_pass);
DECLARE_bool(allow_skip_higher_obs);

// change lane distance
DECLARE_bool(enable_overtake_cross_junction);
DECLARE_bool(allow_lane_change_pass_overlap);
DECLARE_double(lane_change_total_time);
DECLARE_double(overtake_mindis_to_stopsign_threshold);
DECLARE_double(overtake_obstacle_l_buffer);
DECLARE_int32(lc_safe_check_times);
DECLARE_double(check_lane_width_ratio);
DECLARE_double(overtake_end_state_l_ratio);
DECLARE_double(usable_route_lc_min_remain_distance);
DECLARE_double(back_safe_check_distance);
DECLARE_double(front_safe_check_distance);
DECLARE_double(block_obstacle_lateral_buffer);
DECLARE_double(obstacle_cutin_check_lat_buffer);
DECLARE_double(unknown_obstacle_cutin_check_lat_buffer);

DECLARE_bool(enable_anchor_lane_change_path);
DECLARE_bool(enable_use_rescue_mode);
DECLARE_bool(enable_use_deadend_mode);
DECLARE_bool(enable_use_pullover_mode);
DECLARE_bool(enable_pullover_use_hd_map);
DECLARE_bool(enable_noborrow_nearobstacle);
DECLARE_bool(enable_openspace_skip_search);

DECLARE_bool(enable_record_to_learning_data_for_svm);
// obstacle confidence
DECLARE_double(max_valid_perception_distance);
DECLARE_double(min_extremely_accurate_perception_distance);
DECLARE_double(half_confidence_level_perception_distance);

// st_boundary_mapper
DECLARE_double(car_type_lateral_buffer);

// path
DECLARE_double(same_heading);
DECLARE_double(lateral_error);
DECLARE_bool(enable_diagonal_path);
DECLARE_bool(enable_diagonal_road_check);
DECLARE_bool(enable_v2x);
DECLARE_double(distance_to_igv);
DECLARE_bool(enable_change_routing_end);
DECLARE_double(change_routing_end_distance);
DECLARE_bool(enable_backward_path);
DECLARE_bool(enable_self_borrow_near_turn);
DECLARE_double(slef_borrow_buffer);
DECLARE_double(slef_borrow_buffer_min);
DECLARE_double(slef_borrow_buffer_unknown);
DECLARE_double(distance_to_turnlane);
DECLARE_double(distance_self_borrow_extend_boudary);
DECLARE_double(distance_borrow_return);
DECLARE_double(pass_stacker_consider_lateral_range);
DECLARE_double(pass_stacker_stop_distance);
DECLARE_double(pass_stacker_wait_times);
DECLARE_bool(enable_auto_borrow_for_one_obs);
DECLARE_bool(enable_use_pass_stacker);
DECLARE_bool(enable_use_pass_stacker_with_perception);
DECLARE_bool(enable_use_planning_aeb);
DECLARE_bool(enable_use_huamn_in_junction);

// peed
DECLARE_double(diagonal_peed_limit);
DECLARE_bool(enable_check_start_up_safe);
DECLARE_double(check_start_up_distance);

// safty
DECLARE_bool(enable_backward_in_turn);
DECLARE_double(borrow_lateral_buffer_in_turn);
DECLARE_double(stacker_borrow_lateral_distance);

// ------------ Rescue Enable Flag-------------------------
DECLARE_bool(enable_use_ref_lane_roadboundary);
DECLARE_bool(enable_rescue_replan_reason1);
DECLARE_bool(enable_rescue_replan_reason2);
DECLARE_bool(enable_rescue_second_plan);
DECLARE_bool(enable_openspace_function_test);
DECLARE_bool(enable_rescue_surround_cost);

// not use hd map road/lane as roi boundary, just set by costmap and youself
// This was decided by Lei Wang and YiQiang Wang
// If you have any questions, please consult them
DECLARE_bool(enable_not_use_map_as_boundary);

// ------------Just For Rescue Scenario-------------------------
// resce enable flag
DECLARE_bool(enable_rescue);
// hybird a star end ignore distance
DECLARE_double(rescue_hybird_ingore_distance);
DECLARE_double(rescue_hybird_ingore_safe_distance);
// hybird a star lat sample interval
DECLARE_double(rescue_hybird_lat_sample_interval);
// hybird a star goal lat buffer
DECLARE_double(rescue_hybird_lat_buffer);

// ------------Just For Rescue Scenario-------------------------
DECLARE_int32(rescue_failed_report_threshold);
DECLARE_int32(rescue_warring_report_threshold);
DECLARE_double(rescue_vehicle_stop_threshold);
DECLARE_int32(rescue_stop_check_time);

DECLARE_double(astar_first_long_buffer);
DECLARE_double(astar_first_lat_buffer);
DECLARE_bool(enable_openspace_use_polygon_plan);
DECLARE_bool(enable_astar_fallback_buffer);
DECLARE_bool(enable_use_teb);
DECLARE_bool(enable_always_teb);
DECLARE_bool(enable_use_qp_for_teb_speed);
DECLARE_bool(enable_optimized_reverse_trajectory_for_teb_speed);
DECLARE_bool(enable_stop_for_departure_trajectory);
DECLARE_bool(enable_collision_check_for_teb_speed);
DECLARE_bool(start_collision_buff_adjustment);
DECLARE_bool(enable_add_last_point_for_teb);
DECLARE_bool(enable_change_collision_buffer);
DECLARE_bool(enable_startup_for_reach_destination);
DECLARE_bool(enable_teb_speed_limit);
DECLARE_bool(enable_use_costmap);
DECLARE_bool(disenable_play_street_common_junction);
DECLARE_bool(enable_use_origin_obstacle);
DECLARE_bool(enable_use_costmap_boundary);
DECLARE_bool(use_ego_l_cost);

DECLARE_double(rescue_extra_time);
DECLARE_double(rescue_max_time);
DECLARE_double(rescue_min_dist);
DECLARE_bool(enable_create_sl_boundary_using_polygon);

DECLARE_double(astar_smooth_coff);
DECLARE_bool(enable_rescue_pre_endpose_cost);
DECLARE_bool(enable_openspace_ingore_dynamic_obstacle);

DECLARE_double(fallback_collsion_reduction_time);

DECLARE_uint64(fail_back_num);

// ------------For openspace-------------------------
DECLARE_double(teb_back_speed);
DECLARE_double(teb_min_front_speed);
DECLARE_double(teb_max_front_speed);
DECLARE_double(teb_accel_front);
DECLARE_double(fallback_collsion_preview_time);
DECLARE_bool(enable_dynamic_collsion_check);
DECLARE_bool(enable_directly_stop);

DECLARE_bool(enable_reuse_teb_plan);

DECLARE_bool(enable_public_road_teb);
DECLARE_bool(enable_new_teb_test_func);
DECLARE_bool(enable_new_teb_test_func2);
DECLARE_bool(enable_bspline_smooth);
DECLARE_double(teb_obs_prediction_time);
DECLARE_double(teb_static_obs_ttc);
DECLARE_bool(enable_ignore_teb_fallback_check);
DECLARE_double(teb_return_angle_limit);
DECLARE_double(tl_headingerror_limit);

DECLARE_bool(enable_teb_plan_ingore_dynamic_obs);

DECLARE_bool(enable_teb_plan_with_dynamic_adjust_kappa);

// ------------For openspace turn around generate reference---------------
DECLARE_bool(enable_generate_reference_line_with_dis);

// ------------For kalman filter of vehicle state----------
DECLARE_double(transform_distance_variance);
DECLARE_double(transform_speed_variance);
DECLARE_double(transform_acceleration_variance);
DECLARE_double(observation_distance_variance);
DECLARE_double(observation_speed_variance);
DECLARE_double(observation_acceleration_variance);

DECLARE_bool(enable_stable_ststic_obs);
DECLARE_bool(enable_rescue_back_use_teb);

DECLARE_double(openspace_junction_search_radius);
DECLARE_double(cal_speed_filter_too_far_obs);
DECLARE_bool(enable_speed_limit_max_kappa);

// human shaped
DECLARE_double(rerouting_human_shape_distance);

DECLARE_double(expressway_lance_change_distance);
DECLARE_bool(enable_expressway_priority);

// openspace config
DECLARE_string(openspace_config_file);
// default init values for planning_status.top_null
DECLARE_bool(top_bull_default_is_top_bull);
DECLARE_int32(top_bull_default_action_type);
DECLARE_double(top_bull_default_reverse_distance);

DECLARE_bool(enable_dump_collision_debug);
