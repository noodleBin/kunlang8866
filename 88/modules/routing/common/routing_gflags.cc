/******************************************************************************
 * Copyright 2017 The Century Authors. All Rights Reserved.
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

#include "modules/routing/common/routing_gflags.h"

DEFINE_string(project_scenario, "qingdaogang",
              "project_scenario qingdaogang dongjiazhen");

DEFINE_string(routing_conf_file,
              "/century/modules/routing/conf/routing_config.pb.txt",
              "default routing conf data file");

DEFINE_string(routing_point_file,
              "/century/modules/routing/conf/routing_point.pb.txt",
              "default routing point data file");

DEFINE_string(routing_node_name, "routing", "the name for this node");

DEFINE_double(min_length_for_lane_change, 1.0,
              "meters, which is 100 feet.  Minimum distance needs to travel on "
              "a lane before making a lane change. Recommended by "
              "https://www.oregonlaws.org/ors/811.375");

DEFINE_bool(enable_change_lane_in_result, true,
            "contain change lane operator in result");

DEFINE_uint32(routing_response_history_interval_ms, 1000,
              "ms, emit routing resposne for this time interval");

DEFINE_uint32(routing_request_interval_ms, 1000,
              "ms, emit routing request for this time interval");

DEFINE_bool(enable_extend_passage, false, "enable extend passage.");

DEFINE_bool(enable_extend_passage_junction, false,
            "enable extend passage in junction.");

DEFINE_bool(enable_routing_turn_around_flag, false,
            "openspace turn around routing");

DEFINE_bool(enable_search_more_routing_flag, false,
            "enable search more routing");

DEFINE_bool(enable_full_path_twoway_driving, false,
            "enable full path twoway driving");
DEFINE_bool(enable_loop_running, false, "enable loop running");
DEFINE_bool(enable_add_blacklisted_lanes, false,
            "enable_add_blacklisted_lanes");
DEFINE_bool(enable_default_shortest_path_output, false,
            "enable_default_shortest_path_output");
DEFINE_uint32(max_number_responses_output, 3,
              "Maximum number of responses output");
DEFINE_bool(enable_only_u_turn, false, "enable_only_u_turn");

DEFINE_bool(enable_lane_change_length_determination, false, "enable_lane_change_length_determination");

DEFINE_bool(enable_huaman_shaped_driver, false, "enable_huaman_shaped_driver");

DEFINE_bool(enable_multi_mode_tiny_adjustment, false,
             "enable_multi_mode_tiny_adjustment");
DEFINE_bool(enable_tiny_along_lane, false,
             "enable_tiny_along_lane");
DEFINE_bool(enable_prefer_lane_change, false, "enable prefer lane change");

DEFINE_bool(enable_all_prefer_lane_change, false,
            "enable all prefer lane change");

DEFINE_bool(enable_shortest_path_non_replan, false,
            "enable the comparison function of the shortest path between re "
            "planning and non re planning in special areas");

DEFINE_double(min_length_for_extend_passage_lane_change, 100,
              "meters.  Minimum distance needs to travel on a lane before "
              "making a lane change of extend passage.");

DEFINE_double(max_nearest_lane_l_buffer, 3.0,
              "meters.  max nearest lane l buffer.");

DEFINE_double(max_lane_search_distance, 30.0,
              "meters. Maximum distance for incremental lane search when default radius fails.");

DEFINE_double(human_shape_distance, 50.0,
              "meters. the distance between the nearest point and the "
              "human_shape passing frist point.");

DEFINE_double(human_shape_midlle_distance, 32.0,
              "meters. distance from the first waypoint to the second waypoint "
              "(human_shape).");
DEFINE_double(human_shape_max_distance, 100.0,
              "meters. beyond this distance, do not proceed with human shape route");    
DEFINE_bool(enable_yard_static_wait_distance, false, "enable_yard_static_wait_distance");
DEFINE_double(yard_static_wait_distance, 30.0, "yard_static_wait_distance");
DEFINE_double(dual_box_exit_distance, 20.0, "dual_box_exit_distance");
DEFINE_bool(enable_lane_change_close_distance_check, false,
            "enable close distance check for lane change routing");
DEFINE_double(max_stacker_position_distance, 20.0,
              "meters. maximum distance between first and last waypoint to "
              "consider stacker position reliable");
DEFINE_double(reach_stacker_projection_distance_threshold, 1.0,
              "meters. threshold to decide which waypoint to keep "
              "for reach stacker routing");
DEFINE_bool(enable_reach_stacker_end_projection, true,
            "enable end point projection calculation for reach stacker");
// dongjiazhen
DEFINE_bool(enable_add_djz_blacklisted_lanes, false,
            "enable_add_dognjiazhen_blacklisted_lanes");
DEFINE_bool(enable_djz_handing_dead_end, false, "enable_djz_handing_dead_end");
DEFINE_bool(enable_DJZ_demonstrate, false, "enable_DJZ_demonstrate");
DEFINE_uint32(DJZ_demonstrate_scheme, 0, "DJZ_demonstrate_scheme");
DEFINE_double(DJZ_demonstrate_move_disatnce, 60.0,
              "DJZ_demonstrate_move_disatnce");
DEFINE_double(DJZ_demonstrate_max_lane_change_disatnce, 15.0,
              "DJZ_demonstrate_max_lane_change_disatnce");
DEFINE_double(DJZ_demonstrate_min_lane_change_disatnce, 20.0,
              "DJZ_demonstrate_min_lane_change_disatnce");

// routing constants (formerly hardcoded in routing.cc and routing_math.cc)
DEFINE_double(routing_lane_search_radius, 0.3,
              "meters. radius for nearest lane search");
DEFINE_int32(lane_search_loop_count, 20,
             "number of radius expansion loops for lane search");
DEFINE_double(lane_search_increment, 1.0,
              "meters. increment step for lane search expansion");
DEFINE_double(adc_lane_range, 3.5,
              "meters. range threshold for adc lane matching");
DEFINE_double(min_adc_lane_range, 1.0,
              "meters. minimum range threshold for adc lane matching");
DEFINE_double(adc_black_lane_range, 1.5,
              "meters. range threshold for adc blacklisted lane matching");
DEFINE_double(adc_lane_heading_diff, 0.5,
              "radians. max heading difference between adc and lane");
DEFINE_double(adc_advance_distance, 30.0,
              "meters. distance for adc advance projection");
DEFINE_double(vehicle_length, 15.0,
              "meters. vehicle length");
DEFINE_double(min_route_distance_diff, 3.5,
              "meters. min tolerance for route distance comparison");
DEFINE_double(max_route_distance_diff, 5.0,
              "meters. max tolerance for route distance comparison");
DEFINE_double(lane_change_distance, 35.0,
              "meters. distance threshold for lane change");
DEFINE_double(junction_two_point_distance, 50.0,
              "meters. distance from junction to two-point waypoint");
DEFINE_double(adc_two_point_distance, 2.0,
              "meters. distance from adc to two-point waypoint");
DEFINE_double(j4_east_fixed_heading, -0.18,
              "radians. fixed heading for J4 east direction");
DEFINE_double(rerouting_distance, 60.0,
              "meters. distance threshold for rerouting trigger");
DEFINE_int32(max_rerouting_times, 3,
             "maximum number of rerouting attempts");
DEFINE_int32(index_offset, 100,
             "offset for route index calculation");
DEFINE_double(max_blacklisted_lane_distance, 65.0,
              "meters. max distance for blacklisted lane");
DEFINE_double(min_blacklisted_lane_distance, 50.0,
              "meters. min distance for blacklisted lane");
DEFINE_double(djz_max_blacklisted_lane_distance, 65.0,
              "meters. max distance for DJZ blacklisted lane");
DEFINE_double(d7j4w_blacklisted_lane_distance, 40.0,
              "meters. distance for D7J4W blacklisted lane");
DEFINE_double(demo_heading, -0.08,
              "radians. heading for demo mode");
DEFINE_double(two_point_distance, 75.0,
              "meters. distance for two-point routing");
DEFINE_double(reverse_type_cost, 100.0,
              "cost penalty for reverse routing type");
DEFINE_double(rerouting_length_diff, 100.0,
              "meters. length difference for rerouting preference");
DEFINE_double(min_lane_change_length, 10.0,
              "meters. minimum lane change length");
DEFINE_double(lane_change_length, 15.0,
              "meters. default lane change length");
DEFINE_double(routing_junction_search_radius, 0.5,
              "meters. radius for junction search");
DEFINE_double(max_heading_diff, 2.5,
              "radians. max heading difference threshold");
DEFINE_double(min_heading_diff, 0.2,
              "radians. min heading difference threshold");
DEFINE_double(j4_west_fixed_heading, 2.94,
              "radians. fixed heading for J4 west direction");