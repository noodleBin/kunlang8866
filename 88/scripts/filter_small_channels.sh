#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Century Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
#
# Use after "century.sh build":
#    ./filter_small_channels.sh <input_record> <output_record>
#
# The output_record will only contain channels with small data.

INPUT_RECORD=$1
OUTPUT_RECORD=$2

source "$(dirname "${BASH_SOURCE[0]}")/century_base.sh"

mkdir -p "$(dirname "${OUTPUT_RECORD}")"

cyber_recorder split -f "${INPUT_RECORD}" -o "${OUTPUT_RECORD}" \
  -c "/century/canbus/chassis" \
  -c "/century/canbus/chassis_detail" \
  -c "/century/control" \
  -c "/century/control/pad" \
  -c "/century/drive_event" \
  -c "/century/guardian" \
  -c "/century/localization/pose" \
  -c "/century/localization/msf_gnss" \
  -c "/century/localization/msf_lidar" \
  -c "/century/localization/msf_status" \
  -c "/century/hmi/status" \
  -c "/century/monitor" \
  -c "/century/monitor/system_status" \
  -c "/century/navigation" \
  -c "/century/perception/obstacles" \
  -c "/century/perception/traffic_light" \
  -c "/century/planning" \
  -c "/century/prediction" \
  -c "/century/relative_map" \
  -c "/century/routing_request" \
  -c "/century/routing_response" \
  -c "/century/routing_response_history" \
  -c "/century/sensor/conti_radar" \
  -c "/century/sensor/delphi_esr" \
  -c "/century/sensor/gnss/best_pose" \
  -c "/century/sensor/gnss/corrected_imu" \
  -c "/century/sensor/gnss/gnss_status" \
  -c "/century/sensor/gnss/imu" \
  -c "/century/sensor/gnss/ins_stat" \
  -c "/century/sensor/gnss/odometry" \
  -c "/century/sensor/gnss/raw_data" \
  -c "/century/sensor/gnss/rtk_eph" \
  -c "/century/sensor/gnss/rtk_obs" \
  -c "/century/sensor/mobileye" \
  -c "/tf" \
  -c "/tf_static"
