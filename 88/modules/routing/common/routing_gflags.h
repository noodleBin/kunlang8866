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

#pragma once

#include "gflags/gflags.h"

DECLARE_string(project_scenario);

DECLARE_string(routing_conf_file);
DECLARE_string(routing_point_file);
DECLARE_string(routing_node_name);

DECLARE_double(min_length_for_lane_change);
DECLARE_bool(enable_change_lane_in_result);
DECLARE_uint32(routing_response_history_interval_ms);

DECLARE_uint32(routing_request_interval_ms);
DECLARE_bool(enable_extend_passage);
DECLARE_bool(enable_extend_passage_junction);

DECLARE_bool(enable_routing_turn_around_flag);
DECLARE_bool(enable_search_more_routing_flag);
DECLARE_bool(enable_prefer_lane_change);
DECLARE_bool(enable_all_prefer_lane_change);
DECLARE_bool(enable_shortest_path_non_replan);
DECLARE_bool(enable_loop_running);
DECLARE_bool(enable_full_path_twoway_driving);
DECLARE_bool(enable_add_blacklisted_lanes);
DECLARE_bool(enable_default_shortest_path_output);
DECLARE_uint32(max_number_responses_output);
DECLARE_bool(enable_huaman_shaped_driver);
DECLARE_bool(enable_multi_mode_tiny_adjustment);
DECLARE_bool(enable_tiny_along_lane);
DECLARE_bool(enable_only_u_turn);
DECLARE_bool(enable_lane_change_length_determination);
DECLARE_double(min_length_for_extend_passage_lane_change);
DECLARE_double(max_nearest_lane_l_buffer);
DECLARE_double(max_lane_search_distance);
DECLARE_double(human_shape_distance);
DECLARE_double(human_shape_midlle_distance);
DECLARE_double(human_shape_max_distance);
DECLARE_bool(enable_yard_static_wait_distance);
DECLARE_double(yard_static_wait_distance);
DECLARE_double(dual_box_exit_distance);
DECLARE_bool(enable_lane_change_close_distance_check);
DECLARE_double(max_stacker_position_distance);
DECLARE_double(reach_stacker_projection_distance_threshold);
DECLARE_bool(enable_reach_stacker_end_projection);
// dongjiazhen
DECLARE_bool(enable_add_djz_blacklisted_lanes);
DECLARE_bool(enable_djz_handing_dead_end);
DECLARE_bool(enable_DJZ_demonstrate);
DECLARE_uint32(DJZ_demonstrate_scheme);
DECLARE_double(DJZ_demonstrate_move_disatnce);
DECLARE_double(DJZ_demonstrate_max_lane_change_disatnce);
DECLARE_double(DJZ_demonstrate_min_lane_change_disatnce);

// routing constants
DECLARE_double(routing_lane_search_radius);
DECLARE_int32(lane_search_loop_count);
DECLARE_double(lane_search_increment);
DECLARE_double(adc_lane_range);
DECLARE_double(min_adc_lane_range);
DECLARE_double(adc_black_lane_range);
DECLARE_double(adc_lane_heading_diff);
DECLARE_double(adc_advance_distance);
DECLARE_double(vehicle_length);
DECLARE_double(min_route_distance_diff);
DECLARE_double(max_route_distance_diff);
DECLARE_double(lane_change_distance);
DECLARE_double(junction_two_point_distance);
DECLARE_double(adc_two_point_distance);
DECLARE_double(j4_east_fixed_heading);
DECLARE_double(rerouting_distance);
DECLARE_int32(max_rerouting_times);
DECLARE_int32(index_offset);
DECLARE_double(max_blacklisted_lane_distance);
DECLARE_double(min_blacklisted_lane_distance);
DECLARE_double(djz_max_blacklisted_lane_distance);
DECLARE_double(d7j4w_blacklisted_lane_distance);
DECLARE_double(demo_heading);
DECLARE_double(two_point_distance);
DECLARE_double(reverse_type_cost);
DECLARE_double(rerouting_length_diff);
DECLARE_double(min_lane_change_length);
DECLARE_double(lane_change_length);
DECLARE_double(routing_junction_search_radius);
DECLARE_double(max_heading_diff);
DECLARE_double(min_heading_diff);
DECLARE_double(j4_west_fixed_heading);