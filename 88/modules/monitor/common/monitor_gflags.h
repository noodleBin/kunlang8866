/******************************************************************************
 * Copyright 2023 The century Authors. All Rights Reserved.
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

DECLARE_double(cputhreshold);
DECLARE_double(memthreshold);
DECLARE_double(diskthreshold);
DECLARE_double(diskwarningthreshold);
DECLARE_double(lowfreqthreshold);
DECLARE_double(kill_ros_sdk_time_diff);
DECLARE_double(kill_process_time_diff);
DECLARE_uint32(rs_sdk_status_zero_count);
DECLARE_double(kill_planning_time_diff);
DECLARE_double(teb_planning_required_frq);
DECLARE_double(prediction_block_diff);
DECLARE_double(kill_prediction_time_diff);
DECLARE_double(kill_centerpoint_diff);
DECLARE_double(loc_abnormal_status_time);
DECLARE_string(rs_module_name);
DECLARE_bool(turn_on_monitor);
DECLARE_int32(monitorlevel);
DECLARE_bool(treat_low_freq_as_error);
DECLARE_bool(chassis_topic_monitor_switch);
DECLARE_bool(control_topic_monitor_switch);
DECLARE_bool(planning_topic_monitor_switch);
DECLARE_bool(perceptionObstacle_topic_monitor_switch);
DECLARE_bool(prediction_topic_monitor_switch);
DECLARE_bool(trafficLigth_topic_monitor_switch);
DECLARE_bool(localization_topic_monitor_switch);
DECLARE_bool(loc_topic_monitor_switch);
DECLARE_bool(mems_topic_monitor_switch);
DECLARE_bool(radar_topic_monitor_switch);
DECLARE_bool(rs_topic_monitor_switch);
DECLARE_bool(ultrasonic_topic_monitor_switch);
DECLARE_bool(camera_topic_monitor_switch);
DECLARE_bool(cubtek_radar_eol_switch);
DECLARE_bool(mcloud_topic_monitor_switch);
DECLARE_bool(front_12mm_status_topic_monitor_switch);
DECLARE_bool(front_3mm_status_topic_monitor_switch);
DECLARE_bool(back_3mm_status_topic_monitor_switch);
DECLARE_bool(mems_status_topic_monitor_switch);
DECLARE_bool(udas_ultrasonic_eol_monitor_switch);
DECLARE_bool(imu_raw_topic_monitor_switch);
DECLARE_string(outside_address);
DECLARE_string(tbox_address);
DECLARE_string(other_machine_address);
DECLARE_string(local_address);
DECLARE_string(adsens_version_file);
DECLARE_string(adplan_version_file);
DECLARE_string(monitor_data_addition_topic);
DECLARE_string(common_config_path);
DECLARE_string(localization_init_check_command);
DECLARE_string(localization_init_check_key_command);
DECLARE_int32(local_init_keep_times);
DECLARE_double(transmission_latency_diff);
DECLARE_bool(transmission_latency_switch);
DECLARE_double(planning_speed_limit);
DECLARE_double(check_network_freq);
DECLARE_double(localization_diff_horizontal);
DECLARE_double(localization_diff_site);
