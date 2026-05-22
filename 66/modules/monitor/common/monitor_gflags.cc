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

#include "modules/monitor/common/monitor_gflags.h"
namespace {
constexpr char kCheckKeyCommand[] =
    R"(tail -100 /century/data/log/robosense_sdk_orin.INFO \
    | grep -E 'KEY_NOT_FOUND' \
    | sed 's/\x1B\[[0-9;]*[a-zA-Z]//g' \
    | sort \
    | uniq)";
}  // namespace

DEFINE_double(cputhreshold, 0.95, "CPU max usag");
DEFINE_double(memthreshold, 0.95, "memory max usag");
DEFINE_double(diskthreshold, 0.95, "disk max usag");
DEFINE_double(diskwarningthreshold, 0.85, "disk max-warning threshold usag");
DEFINE_double(lowfreqthreshold, 0.8, "the threshold of low channels-freq");
DEFINE_double(kill_ros_sdk_time_diff, 20.0,
              "during time of rs status time is zero");
DEFINE_double(kill_process_time_diff, 180.0,
              "time since the last killing process");
DEFINE_uint32(
    rs_sdk_status_zero_count, 3,
    "the count an alarm is required if the rs_sdk_status continues to be 0");
DEFINE_double(kill_planning_time_diff, 3.0,
              "time since the last killing planning");
DEFINE_double(teb_planning_required_frq, 0.2,
              "the required freq of teb planning");
DEFINE_double(prediction_block_diff, 2.5,
              "the time prediction continue to block");
DEFINE_double(kill_prediction_time_diff, 20.0,
              "time since the last killing prediction");
DEFINE_double(kill_centerpoint_diff, 60.0,
              "time since the last killing centerpoint");
DEFINE_double(loc_abnormal_status_time, 5.0,
              "time since the last killing rs sdk demo");
DEFINE_string(rs_module_name, "centerpoint.dag",
              "/century/track/rs module name");
DEFINE_bool(turn_on_monitor, true, "turn on the monitor function");
DEFINE_int32(monitorlevel, 1, "the lowest level which will be monitored");
DEFINE_bool(treat_low_freq_as_error, false, "turn on the monitor function");
DEFINE_bool(chassis_topic_monitor_switch, true,
            "turn on the monitor chassis topic");
DEFINE_bool(control_topic_monitor_switch, true,
            "turn on the monitor control topic");
DEFINE_bool(planning_topic_monitor_switch, true,
            "turn on the monitor planning topic");
DEFINE_bool(perceptionObstacle_topic_monitor_switch, true,
            "turn on the monitor perceptionObstacle topic");
DEFINE_bool(prediction_topic_monitor_switch, true,
            "turn on the monitor prediction topic");
DEFINE_bool(trafficLigth_topic_monitor_switch, true,
            "turn on the monitor trafficLigth topic");
DEFINE_bool(localization_topic_monitor_switch, true,
            "turn on the monitor localization topic");
DEFINE_bool(loc_topic_monitor_switch, true,
            "turn on the monitor localization topic");
DEFINE_bool(mems_topic_monitor_switch, true, "turn on the monitor mems topic");
DEFINE_bool(radar_topic_monitor_switch, true,
            "turn on the monitor radar topic");
DEFINE_bool(rs_topic_monitor_switch, true, "turn on the monitor rs topic");
DEFINE_bool(ultrasonic_topic_monitor_switch, true,
            "turn on the monitor ultrasonic topic");
DEFINE_bool(camera_topic_monitor_switch, true,
            "turn on the monitor camera topic");
DEFINE_bool(cubtek_radar_eol_switch, false,
            "turn off the cubtek radar eol topic");
DEFINE_bool(mcloud_topic_monitor_switch, true, "turn on the mcloud topic");
DEFINE_bool(front_12mm_status_topic_monitor_switch, false,
            "turn off the front 12mm status topic");
DEFINE_bool(front_3mm_status_topic_monitor_switch, false,
            "turn off the front 3mm status topic");
DEFINE_bool(back_3mm_status_topic_monitor_switch, false,
            "turn off the back 3mm status topic");
DEFINE_bool(mems_status_topic_monitor_switch, false,
            "turn off the mems status topic");
DEFINE_bool(udas_ultrasonic_eol_monitor_switch, false,
            "turn off the udas ultrasonic eol topic");
DEFINE_bool(imu_raw_topic_monitor_switch, true,
            "turn on the  imu raw topic monitor topic");
DEFINE_string(outside_address, "www.baidu.com", "outside address");
DEFINE_string(tbox_address, "192.168.1.120", "tbox address");
DEFINE_string(other_machine_address, "192.168.1.102", "other machine address");
DEFINE_string(local_address, "192.168.1.138", "local address");
DEFINE_string(adsens_version_file, "/century/adsens_version",
              "version file of the software package on sens controller");
DEFINE_string(adplan_version_file, "/century/adplan_version",
              "version file of the software package on plan controller");
DEFINE_string(monitor_data_addition_topic,
              "/century/monitor/monitor_data_addition",
              "monitor_data_addition topic name");
DEFINE_string(common_config_path,
              "/century/modules/common/data/global_flagfile.txt",
              "common config path");
DEFINE_string(localization_init_check_command,
              "tail -100  /century/data/log/robosense_sdk_orin.INFO |grep -E "
              "'RS-LiDAR|GPS/RTK|IMU|Odometry' |grep -E 'Waiting...'|sort "
              "|uniq |awk '{print $2}'",
              "localization init check command");

DEFINE_string(
    localization_init_check_key_command, kCheckKeyCommand,
    "localization init check key command(removed text color and format)");
DEFINE_int32(local_init_keep_times, 4, "local_init_keep_times");
DEFINE_double(transmission_latency_diff, 0.1,
              "the time transmission_latency_diff");
DEFINE_bool(transmission_latency_switch, true,
            "turn on transmission latency faultdata");
DEFINE_double(planning_speed_limit, 1.5, "planning speed limit");

DEFINE_double(check_network_freq, 4000, "check network freq");

DEFINE_double(localization_diff_horizontal, 0.25, "localization diff horizontal");
DEFINE_double(localization_diff_site, 0.3, "localization diff site");