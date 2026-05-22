#! /usr/bin/env bash

###############################################################################
# Copyright 2022 The Century Authors. All Rights Reserved.
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

if [[ ! -d "data/bag" ]]; then
    mkdir -p "data/bag"
fi

source cyber/setup.bash
cd data/bag/

##delete mpi record files  --zheqiang.wu  20231129
# At present, MPI records packages for test, and this script is still used to record packages
# so in this script, delete the mpi's record files for save disk
# In the future, if only MPI is used to record packets,
# this script will not start and then the deletion action will not be executed
if [ -d "/century/data/bag/mpi" ]; then
    find /century/data/bag/mpi -type f -mmin +600 -delete
fi

# avoid repeated record process
process_num=$(ps -ef | grep cyber_recorder | grep -w record | wc -l)
if [ ${process_num} -gt 0 ]; then
    process_id=$(ps -ef | grep cyber_recorder | grep -w record | awk '{print $2}')
    kill -9 ${process_id}
    echo "killed process_num_id:"${process_id}
fi

nohup cyber_recorder record -i 60 -m 2048 \
    -c /century/canbus/chassis \
    -c /century/control \
    -c /century/localization/pose \
    -c /century/perception/obstacles \
    -c /century/robosense/obstacles \
    -c /century/signal_request \
    -c /century/signal_response \
    -c /century/mcloud/super_traffic_light \
    -c /century/planning \
    -c /century/prediction \
    -c /century/routing_request \
    -c /century/routing_response \
    -c /century/routing_result \
    -c /century/tracker/rs \
    -c /century/tracker/mems \
    -c /century/tracker/camera \
    -c /century/tracker/camera_front_3mm \
    -c /century/camera/front_12mm/status \
    -c /century/camera/front_3mm/status \
    -c /century/tracker/radar \
    -c /century/tracker/ultrasonic \
    -c /century/sensor/udas_ultrasonic \
    -c /century/sensor/radar \
    -c /century/monitor/monitor_data_x86 \
    -c /century/monitor/monitor_data_aarch \
    -c /century/mcloud \
    -c imu_raw \
    -c /tf \
    -c /tf_static