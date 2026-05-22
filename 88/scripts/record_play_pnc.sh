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

source "$(dirname "${BASH_SOURCE[0]}")/century_base.sh"

if [ $# -lt 1 ]; then
  echo "$0 record_file"
  exit
fi

cyber_recorder play \
  -c /century/perception/obstacles \
  -c /century/control \
  -c /century/canbus/chassis \
  -c /century/localization/pose \
  -c /century/routing_request \
  -c /century/routing_response \
  -c /century/prediction \
  -c /century/planning \
  -c /century/canbus/chassis \
  -c /century/guardian \
  -c /century/perception/traffic_light \
  -c /century/monitor/system_status \
  -c /tf_static \
  -c /century/control/pad \
  -c /century/drive_event \
  -c /century/monitor \
  -c /tf \
  -c /century/sensor/gnss/best_pose \
  -f $*
