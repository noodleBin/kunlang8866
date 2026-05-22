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

#include "modules/planning/common/planning_gflags.h"

#include <limits>

DEFINE_bool(planning_test_mode, false, "Enable planning test mode.");

DEFINE_int32(planning_loop_rate, 10, "Loop rate for planning node");

DEFINE_int32(history_max_record_num, 5,
             "the number of planning history frame to keep");
DEFINE_int32(max_frame_history_num, 1, "The maximum history frame number");

// scenario related
DEFINE_string(scenario_bare_intersection_unprotected_config_file,
              "/century/modules/planning/conf/"
              "scenario/bare_intersection_unprotected_config.pb.txt",
              "The bare_intersection_unprotected scenario configuration file");
DEFINE_string(scenario_lane_follow_config_file,
              "/century/modules/planning/conf/"
              "scenario/lane_follow_config.pb.txt",
              "The lane_follow scenario configuration file");
DEFINE_string(scenario_lane_follow_hybrid_config_file,
              "/century/modules/planning/conf/"
              "scenario/lane_follow_hybrid_config.pb.txt",
              "The lane_follow scenario configuration file for HYBRID");
DEFINE_string(scenario_learning_model_sample_config_file,
              "/century/modules/planning/conf/"
              "scenario/learning_model_sample_config.pb.txt",
              "learning_model_sample scenario config file");
DEFINE_string(scenario_narrow_street_u_turn_config_file,
              "/century/modules/planning/conf/"
              "scenario/narrow_street_u_turn_config.pb.txt",
              "narrow_street_u_turn scenario config file");
DEFINE_string(scenario_park_and_go_config_file,
              "/century/modules/planning/conf/"
              "scenario/park_and_go_config.pb.txt",
              "park_and_go scenario config file");
DEFINE_string(scenario_pull_over_config_file,
              "/century/modules/planning/conf/"
              "scenario/pull_over_config.pb.txt",
              "The pull_over scenario configuration file");
DEFINE_string(scenario_emergency_pull_over_config_file,
              "/century/modules/planning/conf/"
              "scenario/emergency_pull_over_config.pb.txt",
              "The emergency_pull_over scenario configuration file");
DEFINE_string(scenario_emergency_stop_config_file,
              "/century/modules/planning/conf/"
              "scenario/emergency_stop_config.pb.txt",
              "The emergency_stop scenario configuration file");
DEFINE_string(scenario_stop_sign_unprotected_config_file,
              "/century/modules/planning/conf/"
              "scenario/stop_sign_unprotected_config.pb.txt",
              "stop_sign_unprotected scenario configuration file");
DEFINE_string(scenario_traffic_light_protected_config_file,
              "/century/modules/planning/conf/"
              "scenario/traffic_light_protected_config.pb.txt",
              "traffic_light_protected scenario config file");
DEFINE_string(scenario_traffic_light_protected_teb_config_file,
              "/century/modules/planning/conf/"
              "scenario/traffic_light_protected_teb_config.pb.txt",
              "traffic_light_protected_teb scenario config file");
DEFINE_string(scenario_traffic_light_unprotected_left_turn_config_file,
              "/century/modules/planning/conf/"
              "scenario/traffic_light_unprotected_left_turn_config.pb.txt",
              "traffic_light_unprotected_left_turn scenario config file");
DEFINE_string(scenario_traffic_light_unprotected_right_turn_config_file,
              "/century/modules/planning/conf/"
              "scenario/traffic_light_unprotected_right_turn_config.pb.txt",
              "traffic_light_unprotected_right_turn scenario config file");
DEFINE_string(scenario_valet_parking_config_file,
              "/century/modules/planning/conf/"
              "scenario/valet_parking_config.pb.txt",
              "valet_parking scenario config file");
DEFINE_string(scenario_deadend_turnaround_config_file,
              "/century/modules/planning/conf/"
              "scenario/deadend_turnaround_config.pb.txt",
              "deadend_turnaround scenario config file");
DEFINE_string(scenario_yield_sign_config_file,
              "/century/modules/planning/conf/"
              "scenario/yield_sign_config.pb.txt",
              "yield_sign scenario config file");
DEFINE_string(scenario_rescue_config_file,
              "/century/modules/planning/conf/"
              "scenario/rescue_config.pb.txt",
              "rescue scenario config file");
DEFINE_string(scenario_rescue_teb_config_file,
              "/century/modules/planning/conf/"
              "scenario/rescue_teb_config.pb.txt",
              "rescue_teb scenario config file");

DEFINE_string(scenario_uturn_teb_config_file,
              "/century/modules/planning/conf/scenario/uturn_teb_config.pb.txt",
              "uturn_teb scenario config file");

DEFINE_string(rescue_status_config_file,
              "/century/modules/planning/conf/status/rescue_status.pb.txt",
              "rescue status config file");

DEFINE_bool(enable_stitching_with_prediction, true,
            "enable trajectory stitching with predicted vehicle state");
DEFINE_bool(enable_scenario_bare_intersection, true,
            "enable bare_intersection scenarios in planning");

DEFINE_bool(enable_scenario_park_and_go, true,
            "enable park-and-go scenario in planning");

DEFINE_bool(enable_scenario_pull_over, false,
            "enable pull-over scenario in planning");

DEFINE_bool(enable_scenario_emergency_pull_over, true,
            "enable emergency-pull-over scenario in planning");

DEFINE_bool(enable_scenario_emergency_stop, true,
            "enable emergency-stop scenario in planning");

DEFINE_bool(enable_scenario_side_pass_multiple_parked_obstacles, true,
            "enable ADC to side-pass multiple parked obstacles without"
            "worrying if the obstacles are blocked by others.");

DEFINE_bool(enable_scenario_stop_sign, true,
            "enable stop_sign scenarios in planning");

DEFINE_bool(enable_scenario_traffic_light, true,
            "enable traffic_light scenarios in planning");

DEFINE_bool(enable_scenario_traffic_teb_instead, true,
            "enable traffic_light_teb scenarios instead openspace traffic "
            "light in planning");

DEFINE_bool(enable_scenario_yield_sign, true,
            "enable yield_sign scenarios in planning");

DEFINE_bool(enable_force_pull_over_open_space_parking_test, false,
            "enable force_pull_over_open_space_parking_test");

DEFINE_bool(enable_use_lane_as_boundary, true,
            "enable use_lane_as_boundary, because no road");

DEFINE_string(traffic_rule_config_filename,
              "/century/modules/planning/conf/traffic_rule_config.pb.txt",
              "Traffic rule config filename");

DEFINE_string(smoother_config_filename,
              "/century/modules/planning/conf/qp_spline_smoother_config.pb.txt",
              "The configuration file for qp_spline smoother");

DEFINE_string(rtk_trajectory_filename, "modules/planning/data/garage.csv",
              "Loop rate for planning node");

DEFINE_uint64(rtk_trajectory_forward, 800,
              "The number of points to be included in RTK trajectory "
              "after the matched point");

DEFINE_double(rtk_trajectory_resolution, 0.01,
              "The time resolution of output trajectory for rtk planner.");

DEFINE_bool(publish_estop, false, "publish estop decision in planning");
DEFINE_bool(enable_trajectory_stitcher, true, "enable stitching trajectory");

DEFINE_double(
    look_backward_distance, 30,
    "look backward this distance when creating reference line from routing");

DEFINE_double(look_forward_short_distance, 100,
              "short look forward this distance when creating reference line "
              "from routing when ADC is slow");
DEFINE_double(
    look_forward_long_distance, 150,
    "look forward this distance when creating reference line from routing");

DEFINE_bool(enable_reference_line_stitching, true,
            "Enable stitching reference line, which can reducing computing "
            "time and improve stability");
DEFINE_double(look_forward_extend_distance, 50,
              "The step size when extending reference line.");
DEFINE_double(reference_line_stitch_overlap_distance, 20,
              "The overlap distance with the existing reference line when "
              "stitching the existing reference line");

DEFINE_bool(enable_smooth_reference_line, true,
            "enable smooth the map reference line");

DEFINE_bool(enable_canbus_info_conver, true, "enable canbus info conver");

DEFINE_bool(prioritize_change_lane, false,
            "change lane strategy has higher priority, always use a valid "
            "change lane path if such path exists");
DEFINE_bool(enable_use_steer, false,
            "using steering wheel angle calculate init_kappa.");
DEFINE_bool(enable_use_minimum_turning_radius_to_get_kappa, false,
            "using minimum turing radius calculate kappa.");
DEFINE_bool(enable_acc_start, false, "enable start fast.");
DEFINE_double(change_lane_min_length, 30.0,
              "meters. If the change lane target has longer length than this "
              "threshold, it can shortcut the default lane.");

DEFINE_double(replan_lateral_distance_threshold, 0.5,
              "The lateral distance threshold of replan");
DEFINE_double(replan_longitudinal_distance_threshold, 1.5,
              "The longitudinal distance threshold of replan");

DEFINE_bool(enable_reference_line_provider_thread, true,
            "Enable reference line provider thread.");

DEFINE_bool(enable_reverse_trajectory, false, "enable_reverse_trajectory.");
DEFINE_bool(enable_rerouting_for_block, false, "enable_rerouting_for_block.");
DEFINE_bool(enable_backward_for_obs_block, false,
            "enable_backward_for_obs_block.");
DEFINE_bool(enable_backward_for_routing_request, false,
            "enable_backward_for_routing_request.");
DEFINE_bool(enable_backward_for_task_start_blocking, false,
            "enable_backward_for_task_start_blocking.");
DEFINE_double(task_point_reverse_length, 30.0,
                "Task point blockage, reverse obstacle avoidance length");
DEFINE_double(min_brorrow_length, 25.0,
                    "Minimum length for reversing obstacle avoidance");

DEFINE_double(default_reference_line_width, 4.0,
              "Default reference line width");

DEFINE_double(smoothed_reference_line_max_diff, 5.0,
              "Maximum position difference between the smoothed and the raw "
              "reference lines.");

DEFINE_double(
    max_centripetal_acceleration_threshold, 0.35,
    "Maximum centripetal acceleration threshold for check is headbang path.");

// aeb planning
DEFINE_bool(enable_aeb_planning, true, "enable AEB planning.");

DEFINE_bool(enable_aeb_speed_filter, false, "enable AEB speed filter.");

DEFINE_double(aeb_obstacle_distance_threshold, 25.0,
              "aeb obstacle distance threshold.");

DEFINE_double(aeb_obstacle_length_buffer, 0.5, "aeb obstacle length buffer.");

DEFINE_double(aeb_obstacle_width_buffer, 0.5, "aeb obstacle width buffer.");

DEFINE_double(aeb_obstacle_overlap_length_buffer, 50.0,
              "aeb obstacle overlap length buffer.");

DEFINE_double(aeb_obstacle_overlap_width_buffer, 1.2,
              "aeb obstacle overlap width buffer.");

DEFINE_double(behind_ego_length_buffer, 10.0, "behind ego length buffer.");

DEFINE_double(behind_ego_width_buffer, 6.0, "behind ego width buffer.");

DEFINE_double(lateral_ttc_low_threshold, 7.0, "lateral ttc low threshold.");

DEFINE_double(long_ttc_low_threshold, 7.0, "longitudinal ttc low threshold.");

DEFINE_double(lateral_ttc_medium_threshold, 5.0,
              "lateral ttc medium threshold.");

DEFINE_double(long_ttc_medium_threshold, 5.0,
              "longitudinal ttc medium threshold.");

DEFINE_double(lateral_ttc_high_threshold, 3.5, "lateral ttc high threshold.");

DEFINE_double(long_ttc_high_threshold, 3.5, "longitudinal ttc high threshold.");

// parameters for trajectory planning
DEFINE_double(planning_upper_speed_limit, 31.3,
              "Maximum speed (m/s) in planning.");

DEFINE_double(play_street_speed_limit, 2.78,
              "Play street speed (m/s) in planning. Default 10 Km/h");

DEFINE_double(overtake_upper_speed_limit, 8.33,
              "Maximum speed (m/s) when overtaking.");

DEFINE_double(auxiliary_road_limit_speed, 5.56,
              "Maximum speed (m/s) when in auxoliary road.");

DEFINE_double(overtake_speed_up_ratio, 1.2, "speed up ratio when overtaking.");

DEFINE_bool(enable_overtake_speed_up, false,
            "true to enable speed up function when overtaking.");

DEFINE_double(trajectory_time_length, 8.0, "Trajectory time length");

DEFINE_double(threshold_distance_for_destination, 0.01,
              "threshold distance for destination");

DEFINE_double(open_space_threshold_distance_for_destination, 1.0,
              "open space threshold distance for destination");

DEFINE_double(buffer_in_routing, 0.0, "buffer for select in lane for boundary");

DEFINE_double(buffer_out_routing, -7.0,
              "buffer for select out lane for boundary");
// planning trajectory output time density control
DEFINE_double(
    trajectory_time_min_interval, 0.02,
    "(seconds) Trajectory time interval when publish. The is the min value.");
DEFINE_double(
    trajectory_time_max_interval, 0.1,
    "(seconds) Trajectory time interval when publish. The is the max value.");
DEFINE_double(
    trajectory_time_high_density_period, 1.0,
    "(seconds) Keep high density in the next this amount of seconds. ");

DEFINE_bool(enable_trajectory_check, false,
            "Enable sanity check for planning trajectory.");

DEFINE_double(speed_lower_bound, -0.1, "The lowest speed allowed.");
DEFINE_double(speed_upper_bound, 40.0, "The highest speed allowed.");

DEFINE_double(longitudinal_acceleration_lower_bound, -6.0,
              "The lowest longitudinal acceleration allowed.");
DEFINE_double(longitudinal_acceleration_upper_bound, 4.0,
              "The highest longitudinal acceleration allowed.");
DEFINE_double(lateral_acceleration_bound, 4.0,
              "Bound of lateral acceleration; symmetric for left and right");

DEFINE_double(longitudinal_jerk_lower_bound, -4.0,
              "The lower bound of longitudinal jerk.");
DEFINE_double(longitudinal_jerk_upper_bound, 1.0,
              "The upper bound of longitudinal jerk.");
DEFINE_double(longitudinal_jerk_upper_steady, 1.0,
              "The steady upper vaule of longitudinal jerk.");
DEFINE_double(lateral_jerk_bound, 4.0,
              "Bound of lateral jerk; symmetric for left and right");
DEFINE_double(min_jerk_for_const_accel_speed_plan, -4.0,
              "The lower bound of longitudinal jerk.");
DEFINE_double(max_jerk_for_const_accel_speed_plan, 2.0,
              "The upper bound of longitudinal jerk.");

DEFINE_double(kappa_bound, 0.1979, "The bound for trajectory curvature");

DEFINE_bool(enable_longitudinal_accel_and_jerk_constraint, true,
            "enable longitudinal accel and jerk constraint");

// ST Boundary
DEFINE_double(st_max_s, 100, "the maximum s of st boundary");
DEFINE_double(st_max_t, 8, "the maximum t of st boundary");

// Decision Part
DEFINE_bool(enable_nudge_slowdown, true,
            "True to slow down when nudge obstacles.");
DEFINE_bool(enable_follow_slowdown, true,
            "True to slow down when follow obstacles.");
DEFINE_bool(enable_yield_slowdown, true,
            "True to slow down when yield obstacles.");
DEFINE_bool(enable_stop_slowdown, true,
            "True to slow down when stop obstacles.");
DEFINE_bool(enable_pedestrian_slowdown, true,
            "True to slow down when passing near pedestrians.");
DEFINE_bool(enable_bicycle_slowdown, true,
            "True to slow down when passing near pedestrians.");
DEFINE_double(obs_cross_angle_degree, 45.0,
              "Degree angle to judge the obstacle cross road.");
DEFINE_double(hy_buffer_of_cross_angle, 15.0,
              "hysteresis buffer of obs_cross_angle_degree.");

DEFINE_double(static_obstacle_nudge_l_buffer, 0.3,
              "minimum l-distance to nudge a static obstacle (meters)");
DEFINE_double(nonstatic_obstacle_nudge_l_buffer, 0.4,
              "minimum l-distance to nudge a non-static obstacle (meters)");

DEFINE_double(lateral_ignore_buffer, 3.0,
              "If an obstacle's lateral distance is further away than this "
              "distance, ignore it");
DEFINE_double(lateral_check_buffer, 0.5,
              "If an obstacle's lateral distance is further away than this "
              "distance, check is collision or not");
DEFINE_double(longitudinal_check_buffer, 2.0,
              "If an obstacle's longitudinal distance is further away than this "
              "distance, check is collision or not");
DEFINE_double(max_stop_distance_obstacle, 10.0,
              "max stop distance from in-lane obstacle (meters)");
DEFINE_double(min_stop_distance_obstacle, 6.0,
              "min stop distance from in-lane obstacle (meters)");
DEFINE_double(follow_min_distance, 3.0,
              "min follow distance for vehicles/bicycles/moving objects");
DEFINE_double(max_yield_buffer, 5.0, "extend yield boundary buffer.");
DEFINE_double(follow_min_obs_lateral_distance, 2.5,
              "obstacle min lateral distance to follow");
DEFINE_double(yield_distance, 5.0,
              "min yield distance for vehicles/moving objects "
              "other than pedestrians/bicycles");
DEFINE_double(weighing_stop_distance, 5.0, "weighing point stop distance.");
DEFINE_double(follow_time_buffer, 2.5,
              "time buffer in second to calculate the following distance.");
DEFINE_double(follow_min_time_sec, 2.0,
              "min follow time in st region before considering a valid follow,"
              " this is to differentiate a moving obstacle cross adc's"
              " current lane and move to a different direction");
DEFINE_double(signal_expire_time_sec, 5.0,
              "traffic light signal info read expire time in sec");
DEFINE_double(prediction_expire_time_sec, 0.5,
              "prediction info read expire time in sec");
DEFINE_string(destination_obstacle_id, "DEST",
              "obstacle id for converting destination to an obstacle");
DEFINE_double(destination_check_distance, 5.0,
              "if the distance between destination and ADC is less than this,"
              " it is considered to reach destination");

DEFINE_double(virtual_stop_wall_length, 0.1,
              "virtual stop wall length (meters)");
DEFINE_double(virtual_stop_wall_height, 2.0,
              "virtual stop wall height (meters)");

DEFINE_double(stop_speed_buffer, 2.0,
              "After ADC stops, when the speed of the obstacle ahead is less "
              "than this value, ADC does not start.");
DEFINE_double(stop_distance_buffer, 1.5,
              "After ADC stops, when the distance between ADC and the obstacle "
              "ahead is less than this value, ADC does not start.");
DEFINE_double(max_stop_speed, 0.2, "max speed(m/s) to be considered as a stop");
DEFINE_double(
    buffer_degrees, 30.0,
    "The angle buffer (in degrees) is used to judge whether an obstacle is "
    "crossing the road and moving in the opposite direction.");
DEFINE_double(
    forward_buffer_degrees, 15.0,
    "The angle buffer (in degrees) is used to judge whether an obstacle is "
    "froward moving in the same direction.");

DEFINE_double(ignore_obstacle_l_buffer, 1.0,
              "ignore obstacle l buffer in cut in scenario.");
DEFINE_double(
    ignore_obstacle_speed_coeff, 1.5,
    "ignore obstacle speed coeff in follow speed limit and emergency stopped");
DEFINE_double(
    ignore_obstacle_speed_coeff_radical, 1.1,
    "ignore obstacle speed coeff in follow speed limit and emergency stopped");
DEFINE_double(ignore_obstacle_acceleration_buffer, 0.1,
              "ignore obstacle acceleration buffer in follow speed limit and "
              "emergency stopped");
DEFINE_double(min_dynamic_obstacle_speed, 0.1,
              "minimum dynamic obstacle speed");
DEFINE_double(wheelcrane_consider_distance, 100.0,
              "wheelcrane consider distance.");

// electric fence
DEFINE_bool(enable_electric_fence_drivable_area, false,
            "enable electric fence drivable area.");
DEFINE_bool(enable_not_auto_electric_fence_drivable_area, false,
            "enable not auto driving electric fence drivable area.");
DEFINE_double(electric_fence_max_deceleration, 2.0,
              "electric_fence_max_deceleration.");
DEFINE_double(electric_fence_min_lat_velocity, 0.4,
              "electric_fence_min_lat_velocity.");
DEFINE_double(electric_fence_min_lon_velocity, 0.4,
              "electric_fence_min_lon_velocity.");
// Path Deciders
DEFINE_bool(enable_speed_limit_for_obstacle, false,
            "enable speed limit for obstacle.");
DEFINE_bool(enable_skip_path_tasks, false,
            "skip all path tasks and use trimmed previous path");
DEFINE_bool(enable_borrow_request, false, "enable borrow request.");
DEFINE_bool(enable_self_borrow, false, "enable self borrow.");
DEFINE_bool(enable_check_self_path_has_turn, false, "enable check self path has turn.");
DEFINE_bool(enable_check_times_for_borrow_request, false, "enable check times for borrow request.");
DEFINE_bool(enable_modify_stop_distance, false, "enable modify stop distance.");
DEFINE_bool(only_use_one_trajectory, false, "only_use_one_trajectory.");
DEFINE_double(stop_distance_to_obstacle, 6.0, "stop_distance_to_obstacle");
DEFINE_double(stop_distance_to_stacker, 20.0, "stop_distance_to_stacker");
DEFINE_double(license_plate_recognition_distance, 5.0, "License plate recognition distance");
DEFINE_double(stop_distance_to_obstacle_far, 5.0, "stop_distance_to_obstacle when obs is behind path end point");
DEFINE_bool(enable_trim_unknown_obstacle, true,
            "trim out the part of the unknown obstacle which outside the lane");

DEFINE_double(trim_polygon_check_step, 0.3,
              "trim unknown obstacle polygon, check step for xytosl.");

// PublicRoad obstacle buffer
DEFINE_double(adc_speed_low_threshold_public_road, 0,
              "the low threshold value of adc speed in public road");
DEFINE_double(obstacle_max_lat_buffer_public_road, 0.4,
              "obstacle max lateral buffer (meters) for deciding path "
              "boundaries in public road");
DEFINE_double(obstacle_min_lat_buffer_public_road, 0.4,
              "obstacle min lateral buffer (meters) for deciding path "
              "boundaries in public road");

DEFINE_double(static_obstacle_speed_threshold, 1.5,
              "The speed threshold to decide whether an obstacle is static "
              "or not.");
DEFINE_double(
    static_unknown_obstacle_speed_threshold, 1.5,
    "The speed threshold to decide whether an unknown obstacle is static "
    "or not.");
DEFINE_double(
    static_obstacle_speed_hysteresis_relative_lower_limit, -0.25,
    "The hysteresis speed lower limit to decide whether an obstacle is "
    "static or not.");
DEFINE_double(
    static_obstacle_speed_hysteresis_relative_upper_limit, 0.5,
    "The hysteresis speed upper limit to decide whether an obstacle is "
    "static or not.");
DEFINE_double(block_obstacle_lat_dis_hysteresis_width, 1.0,
              "The hysteresis interval width of obstacle lateral distance.");
DEFINE_double(lane_borrow_max_speed, 5.0,
              "The speed threshold for lane-borrow");
DEFINE_double(adc_speed_hysteresis_lower_limit, 0.3,
              "The hysteresis speed lower limit to decide adc speed status.");
DEFINE_double(adc_speed_hysteresis_upper_limit, 1.0,
              "The hysteresis speed lower limit to decide adc speed status.");
DEFINE_double(lane_borrow_ttc_time, 7.0,
              "The ttc time for lane borrow reasonable distance.");
DEFINE_double(lane_borrow_distance_pedestrian, 20.0,
              "Stable tracking distance for pedestrian or bicycle in the "
              "perception module.");
DEFINE_double(lane_borrow_distance_car, 30.0,
              "Stable tracking distance for car in the perception module.");
DEFINE_double(
    lane_borrow_distance_unknown, 20.0,
    "Stable tracking distance for unknown obstacle  in the perception module.");

// Prediction Part
DEFINE_double(prediction_total_time, 5.0, "Total prediction time");
DEFINE_bool(align_prediction_time, false,
            "enable align prediction data based planning time");
DEFINE_bool(enable_right_borrow, false, "Enable right lane-borrow");

// Trajectory

// according to DMV's rule, turn signal should be on within 200 ft from
// intersection.
DEFINE_double(
    turn_signal_distance, 30.00,
    "In meters. If there is a turn within this distance, use turn signal");

DEFINE_int32(trajectory_point_num_for_debug, 10,
             "number of output trajectory points for debugging");

DEFINE_double(lane_change_prepare_length, 80.0,
              "The distance of lane-change preparation on current lane.");

DEFINE_double(min_lane_change_prepare_length, 10.0,
              "The minimal distance needed of lane-change on current lane.");

DEFINE_double(allowed_lane_change_failure_time, 2.0,
              "The time allowed for lane-change failure before updating"
              "preparation distance.");

DEFINE_bool(enable_smarter_lane_change, false,
            "enable smarter lane change with longer preparation distance.");

DEFINE_double(min_step_end_state_l, 0.06, "min step end state l");
DEFINE_double(max_step_end_state_l, 0.13, "max step end state l");
DEFINE_double(min_speed_step_end_state_l, 4.5, "min speed step end state l");
DEFINE_double(max_speed_step_end_state_l, 8.34, "max speed step end state l");

DEFINE_string(path_label_is_fallback, "fallback", "path label is fallback");
DEFINE_string(path_label_is_self, "self", "path label is self");
DEFINE_string(path_label_is_left_borrow, "left borrow",
              "path label is left borrow");
DEFINE_string(path_label_is_right_borrow, "right borrow",
              "path label is right borrow");
DEFINE_string(path_label_is_lane_change, "lane change",
              "path label is lanechange");

// QpSt optimizer
DEFINE_double(slowdown_profile_deceleration, -4.0,
              "The deceleration to generate slowdown profile. unit: m/s^2.");
DEFINE_double(max_centric_acceleration_limit, 0.5, "centric acc limit.");

// SQP solver
DEFINE_bool(enable_sqp_solver, true, "True to enable SQP solver.");

// Kinematic speed optimizer
DEFINE_bool(enable_kinematic_speed_optimizer, true,
            "True to enable kinematic speed optimizer.");
DEFINE_bool(enable_add_obs, true, "True to enable enable_add_obs.");

/// thread pool
DEFINE_bool(use_multi_thread_to_add_obstacles, false,
            "use multiple thread to add obstacles.");
DEFINE_bool(enable_multi_thread_in_dp_st_graph, false,
            "Enable multiple thread to calculation curve cost in dp_st_graph.");

/// Lattice Planner
DEFINE_double(numerical_epsilon, 1e-6, "Epsilon in lattice planner.");
DEFINE_double(default_cruise_speed, 5.0, "default cruise speed");
DEFINE_double(trajectory_time_resolution, 0.1,
              "Trajectory time resolution in planning");
DEFINE_double(trajectory_space_resolution, 1.0,
              "Trajectory space resolution in planning");
DEFINE_double(speed_lon_decision_horizon, 70.0,
              "Longitudinal horizon for speed decision making (meter)");
DEFINE_uint64(num_velocity_sample, 6,
              "The number of velocity samples in end condition sampler.");
DEFINE_bool(enable_backup_trajectory, true,
            "If generate backup trajectory when planning fail");
DEFINE_double(backup_trajectory_cost, 1000.0,
              "Default cost of backup trajectory");
DEFINE_double(min_velocity_sample_gap, 1.0,
              "Minimal sampling gap for velocity");
DEFINE_double(lon_collision_buffer, 2.0,
              "The longitudinal buffer to keep distance to other vehicles");
DEFINE_double(lat_collision_buffer, 0.1,
              "The lateral buffer to keep distance to other vehicles");
DEFINE_uint64(num_sample_follow_per_timestamp, 3,
              "The number of sample points for each timestamp to follow");

// Lattice Evaluate Parameters
DEFINE_double(weight_lon_objective, 10.0, "Weight of longitudinal travel cost");
DEFINE_double(weight_lon_jerk, 1.0, "Weight of longitudinal jerk cost");
DEFINE_double(weight_lon_collision, 5.0,
              "Weight of longitudinal collision cost");
DEFINE_double(weight_lat_offset, 2.0, "Weight of lateral offset cost");
DEFINE_double(weight_lat_comfort, 10.0, "Weight of lateral comfort cost");
DEFINE_double(weight_centripetal_acceleration, 1.5,
              "Weight of centripetal acceleration");
DEFINE_double(cost_non_priority_reference_line, 5.0,
              "The cost of planning on non-priority reference line.");
DEFINE_double(weight_same_side_offset, 1.0,
              "Weight of same side lateral offset cost");
DEFINE_double(weight_opposite_side_offset, 10.0,
              "Weight of opposite side lateral offset cost");
DEFINE_double(weight_dist_travelled, 10.0, "Weight of travelled distance cost");
DEFINE_double(weight_target_speed, 1.0, "Weight of target speed cost");
DEFINE_double(lat_offset_bound, 3.0, "The bound of lateral offset");
DEFINE_double(lon_collision_yield_buffer, 1.0,
              "Longitudinal collision buffer for yield");
DEFINE_double(lon_collision_overtake_buffer, 5.0,
              "Longitudinal collision buffer for overtake");
DEFINE_double(lon_collision_cost_std, 0.5,
              "The standard deviation of longitudinal collision cost function");
DEFINE_double(default_lon_buffer, 5.0,
              "Default longitudinal buffer to sample path-time points.");
DEFINE_double(time_min_density, 1.0,
              "Minimal time density to search sample points.");
DEFINE_double(comfort_acceleration_factor, 0.5,
              "Factor for comfort acceleration.");
DEFINE_double(polynomial_minimal_param, 0.01,
              "Minimal time parameter in polynomials.");
DEFINE_double(lattice_stop_buffer, 0.02,
              "The buffer before the stop s to check trajectories.");

DEFINE_bool(lateral_optimization, true,
            "whether using optimization for lateral trajectory generation");
DEFINE_double(weight_lateral_offset, 1.0,
              "weight for lateral offset "
              "in lateral trajectory optimization");
DEFINE_double(weight_lateral_derivative, 500.0,
              "weight for lateral derivative "
              "in lateral trajectory optimization");
DEFINE_double(weight_lateral_second_order_derivative, 1000.0,
              "weight for lateral second order derivative "
              "in lateral trajectory optimization");
DEFINE_double(weight_lateral_third_order_derivative, 1000.0,
              "weight for lateral third order derivative "
              "in lateral trajectory optimization");
DEFINE_double(
    weight_lateral_obstacle_distance, 0.0,
    "weight for lateral obstacle distance in lateral trajectory optimization");
DEFINE_double(lateral_third_order_derivative_max, 0.1,
              "the maximal allowance for lateral third order derivative");
DEFINE_double(max_s_lateral_optimization, 60.0,
              "The maximal s for lateral optimization.");
DEFINE_double(default_delta_s_lateral_optimization, 1.0,
              "The default delta s for lateral optimization.");
DEFINE_double(bound_buffer, 0.1, "buffer to boundary for lateral optimization");
DEFINE_double(nudge_buffer, 0.3, "buffer to nudge for lateral optimization");

DEFINE_double(fallback_deceleration, -1.0, "fallback deceleration");
DEFINE_double(fallback_total_time, 3.0, "total fallback trajectory time");
DEFINE_double(fallback_time_unit, 0.1,
              "fallback trajectory unit time in seconds");

DEFINE_double(speed_bump_speed_limit, 4.4704,
              "the speed limit when passing a speed bump, m/s. The default "
              "speed limit is 10 mph.");
DEFINE_double(default_city_road_speed_limit, 8.33,
              "default speed limit (m/s) for city road. 35 mph.");
DEFINE_double(default_highway_speed_limit, 8.33,
              "default speed limit (m/s) for highway. 65 mph.");

// correct obstacle speed
DEFINE_bool(enable_correct_obstacle_speed, true,
            "enable correct obstacle speed.");
DEFINE_double(left_scope_dis_for_correct_speed, 2.0,
              "left scope distance for correct speed.");
DEFINE_double(right_scope_dis_for_correct_speed, 2.0,
              "right scope distance for correct speed.");
DEFINE_double(front_scope_dis_for_correct_speed, 20.0,
              "front scope distance for correct speed.");
DEFINE_double(rear_scope_dis_for_correct_speed, 0.0,
              "rear scope distance for correct speed.");
DEFINE_double(aspect_range_ratio_for_correct_speed, 1.0,
              "aspect range ratio for correct speed.");
DEFINE_double(speed_correct_threshold, 3.0, "speed_correct_threshold.");

// navigation mode
DEFINE_bool(enable_planning_pad_msg, false,
            "To control whether to enable planning pad message.");

// TODO(all): open space planner, merge with planning conf
DEFINE_string(planner_open_space_config_filename,
              "/century/modules/planning/conf/planner_open_space_config.pb.txt",
              "The open space planner configuration file");

DEFINE_double(open_space_planning_period, 4.0,
              "estimated time for open space planner planning period");

DEFINE_double(open_space_prediction_time_horizon, 2.0,
              "the time in second we use from the trajectory of obstacles "
              "given by prediction");

DEFINE_bool(enable_perception_obstacles, true,
            "enable the open space planner to take perception obstacles into "
            "consideration");

DEFINE_bool(enable_open_space_planner_thread, true,
            "Enable thread in open space planner for trajectory publish.");

DEFINE_bool(enable_teb_planner_thread, true,
            "Enable thread in teb planner for trajectory publish.");

DEFINE_bool(use_dual_variable_warm_start, true,
            "whether or not enable dual variable warm start ");

DEFINE_bool(use_gear_shift_trajectory, false,
            "allow some time for the vehicle to shift gear");

DEFINE_uint64(open_space_trajectory_stitching_preserved_length,
              std::numeric_limits<uint32_t>::infinity(),
              "preserved points number in trajectory stitching for open space "
              "trajectory");
DEFINE_bool(
    enable_smoother_failsafe, false,
    "whether to use warm start result as final output when smoother fails");

DEFINE_bool(use_s_curve_speed_smooth, false,
            "Whether use s-curve (piecewise_jerk) for smoothing Hybrid Astar "
            "speed/acceleration.");

DEFINE_bool(
    use_iterative_anchoring_smoother, false,
    "Whether use iterative_anchoring_smoother for open space planning ");

DEFINE_bool(
    enable_parallel_trajectory_smoothing, false,
    "Whether to partition the trajectory first and do smoothing in parallel");

DEFINE_bool(enable_osqp_debug, false,
            "True to turn on OSQP verbose debug output in log.");

DEFINE_bool(export_chart, false, "export chart in planning");
DEFINE_bool(enable_record_debug, true,
            "True to enable record debug info in chart format");

DEFINE_bool(enable_openspace_record_debug, false,
            "True to enable openspace record debug info in chart format");

DEFINE_double(
    default_front_clear_distance, 300.0,
    "default front clear distance value in case there is no obstacle around.");

DEFINE_double(max_trajectory_len, 1000.0,
              "(unit: meter) max possible trajectory length.");
DEFINE_bool(enable_rss_fallback, false, "trigger rss fallback");
DEFINE_bool(enable_rss_info, true, "enable rss_info in trajectory_pb");
DEFINE_double(rss_max_front_obstacle_distance, 3000.0,
              "(unit: meter) for max front obstacle distance.");

DEFINE_bool(
    enable_planning_smoother, false,
    "True to enable planning smoother among different planning cycles.");
DEFINE_double(smoother_stop_distance, 10.0,
              "(unit: meter) for ADC stop, if it is close to the stop point "
              "within this threshold, current planning will be smoothed.");

DEFINE_bool(enable_parallel_hybrid_a, false,
            "True to enable hybrid a* parallel implementation.");

DEFINE_bool(enable_pure_astar, false,
            "True to enable pure hybrid a* instead rs sheep.");

DEFINE_double(open_space_standstill_acceleration, 0.0,
              "(unit: meter/sec^2) for open space stand still at destination");

DEFINE_bool(enable_dp_reference_speed, true,
            "True to penalize dp result towards default cruise speed");

DEFINE_double(message_latency_threshold, 0.02, "Threshold for message delay");
DEFINE_bool(enable_lane_change_urgency_checking, false,
            "True to check the urgency of lane changing");
DEFINE_double(short_path_length_threshold, 20.0,
              "Threshold for too short path length");

DEFINE_uint64(trajectory_stitching_preserved_length, 20,
              "preserved points number in trajectory stitching");

DEFINE_double(side_pass_driving_width_l_buffer, 0.1,
              "(unit: meter) for side pass driving width l buffer");

DEFINE_bool(use_st_drivable_boundary, false,
            "True to use st_drivable boundary in speed planning");

DEFINE_bool(
    use_smoothed_dp_guide_line, false,
    "True to penalize speed optimization result to be close to dp guide line");

DEFINE_bool(use_soft_bound_in_nonlinear_speed_opt, true,
            "False to disallow soft bound in nonlinear speed opt");

DEFINE_bool(use_front_axe_center_in_path_planning, false,
            "If using front axe center in path planning, the path can be "
            "more agile.");

DEFINE_bool(use_road_boundary_from_map, false, "get road boundary from HD map");

DEFINE_bool(planning_offline_learning, false,
            "offline learning. read record files and dump learning_data");
DEFINE_string(planning_data_dir, "/century/modules/planning/data/",
              "Prefix of files to store feature data");
DEFINE_string(planning_offline_bags, "",
              "a list of source files or directories for offline mode. "
              "The items need to be separated by colon ':'. ");
DEFINE_int32(learning_data_obstacle_history_time_sec, 3.0,
             "time sec (second) of history trajectory points for a obstacle");
DEFINE_int32(learning_data_frame_num_per_file, 100,
             "number of learning_data_frame to write out in one data file.");
DEFINE_string(
    planning_birdview_img_feature_renderer_config_file,
    "/century/modules/planning/conf/planning_semantic_map_config.pb.txt",
    "config file for renderer singleton");

DEFINE_bool(
    skip_path_reference_in_side_pass, false,
    "skipping using learning model output as path reference in side pass");
DEFINE_bool(
    skip_path_reference_in_change_lane, true,
    "skipping using learning model output as path reference in change lane");
DEFINE_double(brake_buffer_for_lane_change, 22.5,
              "lane change stop wall buffer");
DEFINE_double(limit_stop_wall_for_lane_change, 20.0,
              "lane change stop wall buffer");
DEFINE_double(preview_brake_distance_for_lane_change, 52.5,
              "lane change stop wall buffer");
DEFINE_double(soft_deceleration_for_lane_change_stop, 1.0,
              "soft deceleration stop before lane change wall");
DEFINE_double(min_stop_distance_for_lane_change_wall, 1.2,
              "min stop distance for lane change stop wall");
DEFINE_int32(prediction_trajectory_relative_time_index, 2.0,
             "prediction trajectory relative time index.");

DEFINE_int32(min_past_history_points_len, 0,
             "minimun past history points length for trainsition from "
             "rule-based planning to learning-based planning");
DEFINE_bool(enable_fallback_planning_thread, false,
            "enable to use fallback planning thread");
DEFINE_bool(enable_TEB_thread, false, "enable to use teb planning thread");
DEFINE_bool(enable_skip_motion_obstacle, false,
            "enable to skip motino obstacle when create stgraph.");
DEFINE_bool(enable_skip_back_obstacle_in_the_same_line, false,
            "enable skip back obstacle in the same line.");
DEFINE_bool(enable_slowdown_for_obstacle, false,
            "enable to slowdown for obstacle.");
DEFINE_bool(enable_shrink_reference_line, false,
            "enable shrink reference line.");
DEFINE_bool(enable_intelligent_projection, false,
            "enable intelligent projection.");
DEFINE_bool(enable_pre_build_sl_boundary, false,
            "enable pre build sl boundary.");
DEFINE_bool(enable_skip_back_obstacles, false,
            "enable to skip back obstacles.");
DEFINE_bool(enable_yield_for_min_lateral_obs, false,
            "enable yield for min lateral obstacle.");
DEFINE_bool(enable_add_adc_buffer, false,
            "enable to add adc lateral buffer when check collision.");
DEFINE_bool(enable_check_back_side_obstacle, false,
            "enable to check obstacles are back side or nor.");
DEFINE_bool(enable_use_lastpassage, false,
            "enable to use last passage when no lanechange.");
DEFINE_bool(enable_shrink_yield_distance, false,
            "enable to shrink yield distance when adc behind the stop line in "
            "traffic red.");
DEFINE_bool(enable_shrink_stop_distance_for_pedestrian, false,
            "enable to shrink stop distance when adc follow or yied or stop in "
            "playstreet.");
DEFINE_bool(enable_use_radical_decision, false,
            "enable use radical yield decision.");
DEFINE_bool(enable_slow_breaking, false, "enable slow breaking.");
DEFINE_bool(enable_slow_breaking_for_cutin, false, "enable slow breaking.");
DEFINE_bool(enable_slow_breaking_for_abnormal_prediction, false,
            "enable slow breaking.");
DEFINE_bool(enable_slow_breaking_for_large_ttc, false, "enable slow breaking.");
DEFINE_bool(enable_slow_breaking_for_reverse_obs, false,
            "enable slow breaking.");
DEFINE_bool(enable_yield_for_high_speed_bicycle, false,
            "enable yield for high speed bicycle.");
DEFINE_bool(enable_use_new_method_to_get_decel_for_approaching_obs, false,
            "enable_use_new_method_to_get_decel_for_approaching_obs.");
DEFINE_bool(enable_use_new_method_to_get_decel_for_large_speed_obs, false,
            "enable_use_new_method_to_get_decel_for_large_speed_obs.");
DEFINE_bool(enable_no_emergency_break_reverse_obs, false,
            "enable no emergency breaking.");
DEFINE_bool(enable_use_adc_lane_width_for_reverse_obs, false,
            "enable use adc lane width for reverse obs.");
DEFINE_bool(enable_skip_back_side_and_has_overlap_obs, false,
            "enable skip back side and has overlap obs.");
DEFINE_bool(enable_skip_back_side_in_laneborrow_return, false,
            "enable skip back side in laneborrow return.");
DEFINE_bool(enable_use_boundary_check_obs_on_lane, false,
            "enable use boundary check is obstacle on lane.");
DEFINE_bool(enable_use_boundary_check_adc_on_lane, false,
            "enable use boundary check is adc on lane.");
DEFINE_bool(enable_slow_down_for_cross_obstacle, false,
            "enable slow down for cross obstacle.");
DEFINE_bool(enable_slow_down_for_reverse_obstacle, false,
            "enable slow down for reverse obstacle.");
DEFINE_bool(enable_no_slow_down_for_overtake_obstacle, false,
            "enable no slow down for overtake obstacle.");
DEFINE_bool(enable_enter_mixed_flow_mode, false,
            "enable enter mixed flow  mode.");
DEFINE_bool(enable_high_speed, false, "enable high speed.");
DEFINE_bool(enable_use_reference_v_for_speed_optimizer, false,
            "enable use reference_v for speed_optimizer.");
DEFINE_bool(enable_dynamic_modify_piecewise_jerk_weight, false,
            "enable dynamic modify piecewise jerk weight.");
DEFINE_bool(enable_no_shrink_upper_bound, false,
            "enable no shrink upper bound.");
DEFINE_double(
    max_overtake_longitude_buffer, 10.0,
    "meters. Max distance to overtake decision lon collision buffer.");
DEFINE_double(
    min_overtake_longitude_buffer, 2.0,
    "meters. Min distance to overtake decision lon collision buffer.");
DEFINE_bool(enable_replan_for_smaller_start_point, false,
            "enable replan for smaller start point.");
DEFINE_bool(enable_replan_for_diagobal_change_to_normal, false,
            "enable replan for diagobal change to normal.");
DEFINE_bool(enable_near_junction_laneborrow, false,
            "allow to laneborrow when near junction.");
DEFINE_double(routing_lane_length_threshold, 32.0,
              "left routing lane length shorter than threshold, stop near "
              "junction laneborrow");
DEFINE_bool(enable_extend_path_bound_base_adc_posture, false,
            "enable extend path bound when adc out of lane.");
DEFINE_double(adc_posture_correct_check_heading_diff, 0.5236,
              "this value is radian, check adc heading diff with lane, if too "
              "large, need consider "
              "use neighbor lane");
DEFINE_bool(enable_separate_auxiliary_road_borrow, false,
            "enable separate non driveway laneborrow and selfborrow.");
DEFINE_bool(enable_efficiency_shift_borrow, false,
            "enable efficiency shift borrow for slow dynamic obs.");
DEFINE_bool(enable_auto_borrow, false, "enable auto borrow.");
DEFINE_bool(allow_skip_higher_obs, false, "allow skip higher obs.");
DEFINE_bool(enable_auto_lane_borrow, false, "enable auto  lane borrow.");
DEFINE_bool(allow_lane_borrow_fsm, false, "enable fsm laneborrow.");
DEFINE_bool(allow_smi_diagonal, false, "allow smi diagonal.");
DEFINE_bool(allow_narrow_pass, false, "allow narrow pass.");

DEFINE_double(borrow_slow_obstacle_velocity_threshold, 1.2,
              "borrow slow obstacles which velocity less than this value in "
              "lane borrow status.");

DEFINE_bool(enable_overtake_cross_junction, false,
            "allow to laneborrow when near junction.");
DEFINE_double(static_obstacle_buffer, 0.3,
              "lateral safe buffer for static obstacle");
DEFINE_double(
    block_adc_center_view_buffer, 0.3,
    "near junction obstacle block the center view of adc lateral buffer");

DEFINE_double(stop_distance_in_common_junction, 2.0,
              "Stop distance in COMMON_JUNCTION area.");

DEFINE_double(lower_speed_in_public_road, 4.0, "Lower speed in public road.");

DEFINE_bool(enable_make_dangerous_start_up, false,
            "enable make dangerous start up.");
DEFINE_bool(enable_vehicle_start_up, false, "enable vehicle start up.");

DEFINE_bool(enable_radical_change_lane_in_merge_area, false,
            "enable radical change lane in merge area.");
DEFINE_bool(enable_speed_limit_for_approaching_obs, false,
            "enable speed limit for approaching obstacle.");
DEFINE_bool(enable_kinematic_speed_slowly_down, false,
            "enable kinematic speed slowly down for approaching obstacle.");
DEFINE_bool(enable_obstacle_sidepass_decision, false,
            "enable obstacle sidepass decision.");

DEFINE_bool(enable_cancel_stop_decision_from_heading, false,
            "enable cancel stop decision from heading.");
DEFINE_bool(enable_cancel_stop_decision_from_heading_rate, false,
            "enable cancel stop decision from heading rate.");

DEFINE_uint32(max_record_times_for_correct_speed, 5,
              "Maximum record times for correct speed.");

DEFINE_uint32(max_record_times_for_start_up, 5,
              "Maximum record times for start up.");

DEFINE_uint32(max_record_times_for_diff_s, 10,
              "Maximum record times for side pass.");
DEFINE_double(stable_speed_buffer, 1.5, "stable speed buffer.");

DEFINE_double(average_step_value, 0.2, "average step value.");

DEFINE_double(four_times_step_value, 0.08, "four times step value.");

DEFINE_double(three_times_step_value, 0.12, "three times step value.");

DEFINE_double(average_step_value_for_already_stop, 0.05,
              "average step value for already stoped obstacle.");

DEFINE_double(average_step_distance_for_already_stop, 0.03,
              "average step value for already stoped obstacle.");
DEFINE_double(first_level_average_step_distance, 0.05,
              "average step value for already stoped obstacle.");
DEFINE_double(second_level_average_step_distance, 0.08,
              "average step value for already stoped obstacle.");
DEFINE_double(average_step_distance_for_already_stop_loose_constraint, 0.02,
              "average step value for already stoped obstacle.");
DEFINE_double(first_level_average_step_distance_loose_constraint, 0.03,
              "average step value for already stoped obstacle.");
DEFINE_double(second_level_average_step_distance_loose_constraint, 0.05,
              "average step value for already stoped obstacle.");
DEFINE_double(average_step_value_tightly, 0.2, "tightly average step value.");

DEFINE_double(four_times_step_value_tightly, 0.08,
              "tightly four times step value.");

DEFINE_double(three_times_step_value_tightly, 0.12,
              "tightly three times step value.");

DEFINE_double(average_step_value_for_already_stop_tightly, 0.05,
              "tightly average step value for already stoped obstacle.");

DEFINE_double(average_step_value_slower_near, 0.2,
              "average step value for slower near obstacle.");

DEFINE_double(four_times_step_value_slower_near, 0.08,
              "four times step value for slower near obstacle.");

DEFINE_double(three_times_step_value_slower_near, 0.12,
              "three times step value for slower near obstacle.");

DEFINE_double(average_step_value_for_already_stop_slower_near, 0.05,
              "average step value for already stoped slower near obstacle.");

DEFINE_uint32(lost_keep_move_near_times, 30, "lost keep move near times.");

DEFINE_bool(enable_emergency_speed_fallback, false,
            "enable emergency speed fallback.");

DEFINE_double(lower_speed_for_speed_fallback, 4.0,
              "Lower speed for speed fallback.");

DEFINE_double(lower_kappa_for_speed_fallback, 1e-4,
              "Lower speed for speed fallback.");

DEFINE_double(max_diff_angle_for_stoped_collision_check, 60.0,
              "max diff angle for stoped collision check.");
DEFINE_double(hy_buffer_lower_for_stoped_collision_check, -5.0,
              "hysteresis buffer lower limit for stoped collision check.");
DEFINE_double(hy_buffer_upper_for_stoped_collision_check, 10.0,
              "hysteresis buffer upper limit for stoped collision check.");

DEFINE_bool(enable_slower_deceleration_after_qp_failure, false,
            "enable slower deceleration after qp failure.");

DEFINE_bool(consider_obstacle_blocked, false,
            "consider target obstacle blocked by anothers");
DEFINE_bool(allow_lane_change_pass_overlap, false,
            "allow adc has lane change behavior in crosswalk area");
DEFINE_double(lane_change_total_time, 6.0, "lane change spend total time");
DEFINE_double(overtake_mindis_to_stopsign_threshold, 150.0,
              "meters. mininum distance to stop sign to change lane.");
DEFINE_double(overtake_obstacle_l_buffer, 0.3,
              "minimum l-distance to pass a low speed obstacle (meters).");

DEFINE_bool(enable_anchor_lane_change_path, false,
            "True to enable use anchor lane change path");
DEFINE_bool(enable_use_rescue_mode, false, "True to enable use rescue mode.");
DEFINE_bool(enable_use_deadend_mode, false, "True to enable use deadend mode.");
DEFINE_bool(enable_use_pullover_mode, false,
            "True to enable use pullover mode.");
DEFINE_bool(enable_pullover_use_hd_map, false,
            "True to enable pullover use ha_map.");
DEFINE_bool(enable_openspace_skip_search, true,
            "True to enable openspace skip search.");
DEFINE_int32(lc_safe_check_times, 2,
             "number of continus check times for overtake function");
DEFINE_double(check_lane_width_ratio, 0.3,
              "check how much the adc takes in ratio * lane_width");
DEFINE_double(overtake_end_state_l_ratio, 0.4,
              "overtake end state l percentage relative to current adc l");
DEFINE_double(usable_route_lc_min_remain_distance, 80.0,
              "the min usable distance for route lane change");
DEFINE_double(front_safe_check_distance, 180.0,
              "the front distance for lane change safe check");
DEFINE_double(back_safe_check_distance, -100.0,
              "the back distance for lane change safe check");
DEFINE_double(block_obstacle_lateral_buffer, -0.2,
              "when check the obstacle is block by it's front one, add the"
              "lateral buffer");
DEFINE_double(
    obstacle_cutin_check_lat_buffer, 0.4,
    "obstacle latral cut in lane will be considered as risk for lane change");
DEFINE_double(unknown_obstacle_cutin_check_lat_buffer, 0.6,
              "unknown obstacle latral cut in lane will be considered as risk "
              "for lane change");
// ------------ Rescue Enable Flag-------------------------

DEFINE_bool(enable_use_ref_lane_roadboundary, true,
            "True to use lane in  referenceline roadboundary.");
DEFINE_bool(enable_rescue_replan_reason1, false,
            "True to enable refresh rescue goal.");
DEFINE_bool(enable_rescue_replan_reason2, false,
            "True to enable refresh rescue goal.");
DEFINE_bool(enable_rescue_second_plan, false,
            "True to plan constantly in rescue.");

// not use hd map road/lane as roi boundary, just set by costmap and youself
// This was decided by Lei Wang and YiQiang Wang
// If you have any questions, please consult them
DEFINE_bool(enable_not_use_map_as_boundary, false,
            "True to not use map as boundary.");
DEFINE_bool(enable_noborrow_nearobstacle, false,
            "True to noborrow when near obstacle.");
DEFINE_bool(enable_openspace_function_test, false,
            "True to test some openspace function.");
DEFINE_bool(enable_rescue_surround_cost, false, "True to use surround cost.");

// ------------Just For Rescue Scenario-------------------------
// resce enable flag
DEFINE_bool(enable_rescue, false, "rescue enable flag.");
// hybird a star end ignore distance
DEFINE_double(rescue_hybird_ingore_distance, 18.0,
              "meters. hybird a star end ignore distance.");
DEFINE_double(rescue_hybird_ingore_safe_distance, 0.2,
              "meters. hybird a star end ignore safe distance.");
// hybird a star lat sample interval
DEFINE_double(rescue_hybird_lat_sample_interval, 0.3,
              "meters. hybird a star lat sample interval.");
// hybird a star lat buffer
DEFINE_double(rescue_hybird_lat_buffer, 0.1,
              "meters. hybird a star lat buffer");
// ------------Just For Rescue Scenario-------------------------
DEFINE_int32(rescue_failed_report_threshold, 4,
             "rescue_failed_report_threshold.");
DEFINE_int32(rescue_warring_report_threshold, 2,
             "rescue_warring_report_threshold.");
DEFINE_double(rescue_vehicle_stop_threshold, 0.05,
              "meters. rescue_vehicle_stop_threshold.");
DEFINE_int32(rescue_stop_check_time, 9000,
             "meters. rescue_stop_check_time(ms).");

DEFINE_double(astar_first_long_buffer, 0.5,
              "meters. hybird a star long buffer.");
DEFINE_double(astar_first_lat_buffer, 1.1, "meters. hybird a star lat buffer.");

DEFINE_bool(enable_openspace_use_polygon_plan, true,
            "true to enable openspace use  polygon  plan.");

DEFINE_bool(enable_astar_fallback_buffer, true,
            "true to enable fallback decider add adc buffer.");

DEFINE_bool(enable_use_teb, true, "true to enable use teb planner.");
DEFINE_bool(enable_always_teb, false, "true to always use teb planner.");
DEFINE_bool(enable_use_qp_for_teb_speed, false, "enable use qp for teb speed.");
DEFINE_bool(enable_optimized_reverse_trajectory_for_teb_speed, false,
            "enable optimized reverse trajectory for teb_speed.");
DEFINE_bool(enable_stop_for_departure_trajectory, false,
            "enable stop for departure trajectory.");
DEFINE_bool(enable_collision_check_for_teb_speed, false,
            "enable collision check for teb speed.");
DEFINE_bool(start_collision_buff_adjustment, false,
            "start collision buff adjustment.");

DEFINE_bool(enable_add_last_point_for_teb, false,
            "enable add last point for teb.");
DEFINE_bool(enable_change_collision_buffer, false,
            "enable change collision buffer.");
DEFINE_bool(enable_startup_for_reach_destination, false,
            "enable startup for reach destination.");
DEFINE_bool(enable_teb_speed_limit, false, "enable teb speed limit function.");
DEFINE_bool(enable_use_costmap, false,
            "true to enable use costmap as obstacle.");
DEFINE_bool(enable_use_drive_area, false, "true to enable use drive area.");
DEFINE_bool(enable_use_costmap_boundary, false,
            "true to enable use costmap boundary.");
DEFINE_bool(use_ego_l_cost, false, "true to enable use_ego_l_cost.");
DEFINE_bool(
    disenable_play_street_common_junction, false,
    "true to disenable plan outside road in play_street_common_junction.");
DEFINE_bool(enable_use_origin_obstacle, false,
            "true to enable use origin obstacle in costmap planner.");

DEFINE_double(rescue_extra_time, 7.0, "rescue extra time.");

DEFINE_double(rescue_max_time, 90.0,
              "seconds. hybird astar scenario max time .");

DEFINE_double(rescue_min_dist, 0.0,
              "meters. hybird astar must pass min distance.");

DEFINE_bool(enable_create_sl_boundary_using_polygon, true,
            "only use polygon to create SL boundary.");

DEFINE_double(
    fallback_collsion_reduction_time, 0.5,
    "%. hybird astar reduce collsion check time for static obj or pedestrian.");

DEFINE_double(fallback_collsion_preview_time, 1.2,
              " openspace dynamic_collsion_check use preview time.");

DEFINE_bool(enable_dynamic_collsion_check, true,
            " openspace also use dynamic_collsion_check.");

DEFINE_bool(enable_directly_stop, true,
            " openspace fallback directly stop when check collision.");

DEFINE_double(astar_smooth_coff, 0.8,
              "%. hybird astar reduce radius, 1 is no smooth,between 0.3 ~ 1.");
DEFINE_bool(enable_rescue_pre_endpose_cost, false,
            "enable_rescue_pre_endpose_cost.");

DEFINE_bool(enable_openspace_ingore_dynamic_obstacle, false,
            "enable use dynamic_obstacle in fallback planner.");

DEFINE_uint64(fail_back_num, 0, "---------00000---------.");
// ------------------For ST boundary-----------------------------
DEFINE_double(teb_back_speed, -0.5, "m/s. the  speed for teb back.");
DEFINE_double(teb_min_front_speed, 0.65, "m/s. the min speed for teb front.");
DEFINE_double(teb_max_front_speed, 1.3, "m/s. the max speed for teb front.");
DEFINE_double(teb_accel_front, 0.02, "m/s. the accel for teb front.");

DEFINE_bool(enable_reuse_teb_plan, false, "enable reuse teb plan.");

DEFINE_double(teb_obs_prediction_time, 3.00, "s. prediction time for teb use.");
DEFINE_double(teb_static_obs_ttc, 1.50,
              "s. ttc time use for teb static obs collision check.");
DEFINE_bool(enable_ignore_teb_fallback_check, false,
            "enable_ignore_teb_fallback_check.");
DEFINE_double(teb_return_angle_limit, 25.0,
              "%. steer angle need in limit before return prp .");
DEFINE_double(tl_headingerror_limit, 18.0,
              "deg. traffic light heading error > limit, then enter adjust;0 "
              "is close function.");
DEFINE_bool(enable_public_road_teb, false,
            "enable a new method for dynamic obstacle in teb plan.");
DEFINE_bool(enable_new_teb_test_func, false, "teb function test temp use.");
DEFINE_bool(enable_new_teb_test_func2, false, "teb functionc2 test temp use.");
DEFINE_bool(enable_bspline_smooth, false, "teb enable use bspline smooth.");

DEFINE_bool(enable_teb_plan_ingore_dynamic_obs, false,
            "enable teb plan ingore dynamic obstacle.");

DEFINE_bool(enable_teb_plan_with_dynamic_adjust_kappa, false,
            "enable teb plan with dynamic adjust kappa.");

// ------------For openspace turn around generate
DEFINE_bool(enable_generate_reference_line_with_dis, false,
            "openspace turn around generate reference must close with heading "
            "change dis.");

// ------------For kalman filter of vehicle state----------
DEFINE_double(transform_distance_variance, 0.01,
              "the distance variance of the state transition noise");
DEFINE_double(transform_speed_variance, 0.01,
              "the speed variance of the state transition noise");
DEFINE_double(transform_acceleration_variance, 0.05,
              "the acceleration variance of the state transition noise");
DEFINE_double(observation_distance_variance, 0.1,
              "the distance variance of the observation noise");
DEFINE_double(observation_speed_variance, 0.01,
              "the speed variance of the observation noise");
DEFINE_double(observation_acceleration_variance, 10.0,
              "the acceleration variance of the observation noise");

DEFINE_bool(enable_stable_ststic_obs, false, "enable_stable_ststic_obs.");
DEFINE_bool(enable_rescue_back_use_teb, false, "enable_rescue_back_use_teb.");

// ------------For SVM-------------------------
DEFINE_bool(enable_record_to_learning_data_for_svm, false,
            "enable record to learning data for svm.");
DEFINE_bool(enable_txt_to_bin_for_svm, false, "enable txt to bin for svm.");
DEFINE_bool(svm_switch, false, "svm switch.");
DEFINE_bool(enable_use_svm_model, false, "enable use svm model.");
DEFINE_bool(enable_change_acc_weight_for_svm, false,
            "enable change acc weight for svm.");

// st_boundary_mapper
DEFINE_double(car_type_lateral_buffer, 0.3,
              "vehicle type obs need large lateral collision buffer.");

// path
DEFINE_double(same_heading, 0.02, "same heading.");
DEFINE_double(lateral_error, 0.1, "lateral error.");
DEFINE_bool(enable_diagonal_path, false, "enable diagonal path.");
DEFINE_bool(enable_diagonal_road_check, false, "enable diagonal road check.");
DEFINE_bool(enable_v2x, false, "enable v2x.");
DEFINE_double(distance_to_igv, 10.0, "distance to igv.");
DEFINE_bool(enable_change_routing_end, false, "enable change routing end.");
DEFINE_double(change_routing_end_distance, 20.0, "change routing end distance.");
DEFINE_bool(enable_backward_path, false, "enable backward path.");
DEFINE_bool(enable_self_borrow_near_turn, false, "enable self borrow near turn.");
DEFINE_double(slef_borrow_buffer, 0.5, "slef borrow buffer.");
DEFINE_double(slef_borrow_buffer_min, 0.5, "min slef borrow buffer.");
DEFINE_double(slef_borrow_buffer_unknown, 0.5, "slef borrow buffer.");
DEFINE_double(distance_to_turnlane, 30.0, "distance to turnlane.");
DEFINE_double(distance_self_borrow_extend_boudary, 1.6, "distance to extend for self borrow.");
DEFINE_double(distance_borrow_return, 5.0, "distance borrow return.");
DEFINE_double(pass_stacker_consider_lateral_range, 20.0, "pass stacker consider lateral range.");
DEFINE_double(pass_stacker_stop_distance, 25.0, "pass stacker stop distance.");
DEFINE_double(pass_stacker_wait_times, 1800.0, "pass_stacker_wait_times : 180/60 = 3min.");
DEFINE_bool(enable_auto_borrow_for_one_obs, false, "enable auto borrow for one obs.");
DEFINE_bool(enable_use_pass_stacker, false, "enable use pass stacker.");
DEFINE_bool(enable_use_pass_stacker_with_perception, false, "enable use pass stacker with_perception.");
DEFINE_bool(enable_use_planning_aeb, false, "enable use planning aeb.");
DEFINE_bool(enable_use_huamn_in_junction, false, "enable_use_huamn_in_junction.");

// speed
DEFINE_double(diagonal_peed_limit, 3.0, "diagonal peed limit.");
DEFINE_bool(enable_check_start_up_safe, false, "enable check start up safe.");
DEFINE_double(check_start_up_distance, 3.0, "check start up distance.");

// safty
DEFINE_bool(enable_backward_in_turn, false, "enable backward in turn.");
DEFINE_double(borrow_lateral_buffer_in_turn, 1.0,
              "borrow_lateral_buffer_in_turn");
DEFINE_double(stacker_borrow_lateral_distance, 2.5,
              "stacker borrow lateral distance.");
// ------------For obstacle confidence----------
DEFINE_double(max_valid_perception_distance, 40.0,
              "Beyond this distance, the perception accuracy is unreliable");
DEFINE_double(min_extremely_accurate_perception_distance, 8.0,
              "Within this distance, the perception accuracy is beyond 90%");
DEFINE_double(half_confidence_level_perception_distance, 20.0,
              "Within this distance, the perception accuracy is beyond 50%");
DEFINE_double(openspace_junction_search_radius, 1.0,
              "openspace junction search radius");
DEFINE_double(cal_speed_filter_too_far_obs, 10.0, "cal_speed_filter_obs");
DEFINE_bool(enable_speed_limit_max_kappa, false, "enable speed limit use max kappa.");

// human shape
DEFINE_double(rerouting_human_shape_distance, 30.0, "rerouting rerouting_human_shape_distance");


DEFINE_double(expressway_lance_change_distance, 50.0,
              "expressway_lance_change_distance");
DEFINE_bool(enable_expressway_priority, false, "enable_expressway_priority.");

// openspace config
DEFINE_string(
    openspace_config_file,
    "/century/modules/planning/conf/openspace_config/openspace_config.pb.txt",
    "openspace common all configs file.");
DEFINE_bool(top_bull_default_is_top_bull, false,
            "default init value for planning_status.top_bull.is_top_bull");
DEFINE_int32(top_bull_default_action_type, 0,
             "default init value for planning_status.top_bull.action_type");
DEFINE_double(
    top_bull_default_reverse_distance, 0.0,
    "default init value for planning_status.top_bull.reverse_distance");

DEFINE_bool(enable_dump_collision_debug, false,
            "dump collision debug data to file when electric fence collision");
