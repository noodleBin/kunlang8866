#!/usr/bin/env bash

###############################################################################
# Copyright 2017-2021 The Century Authors. All Rights Reserved.
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

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DREAMVIEW_URL="http://localhost:8888"

cd "${DIR}/.."

# Make sure supervisord has correct coredump file limit.
ulimit -c unlimited

source "${DIR}/century_base.sh"

function start() {

  # echo "--static_file_dir=/century/modules/dreamview/frontend/dist" | sudo tee -a <目标文件路径>
  DIST_LITE_WEB_PATH="/century/modules/dreamview/frontend/build"
  DIST_FULL_WEB_PATH="/century/modules/dreamview/frontend/dist"

  CONF_FILE="/century/modules/dreamview/conf/dreamview.conf"
  FLAG_NAME="static_file_dir"

  case "$1" in
    lite)
      echo "--${FLAG_NAME}=${DIST_LITE_WEB_PATH}" | tee -a "${CONF_FILE}"
      ;;
    full)
      echo "--${FLAG_NAME}=${DIST_FULL_WEB_PATH}" | tee -a "${CONF_FILE}"
      ;;
    *)
      echo "--${FLAG_NAME}=${DIST_LITE_WEB_PATH}" | tee -a "${CONF_FILE}"
      ;;
  esac

  for mod in ${CENTURY_BOOTSTRAP_EXTRA_MODULES}; do
    echo "Starting ${mod}"
    nohup cyber_launch start ${mod} &
  done
  ./scripts/monitor.sh start
  ./scripts/dreamview.sh start
  if [ $? -eq 0 ]; then
    sleep 2 # wait for some time before starting to check
    http_status="$(curl -o /dev/null -x '' -I -L -s -w '%{http_code}' ${DREAMVIEW_URL})"
    if [ $http_status -eq 200 ]; then
      echo "Dreamview is running at" $DREAMVIEW_URL
    else
      echo "Failed to start Dreamview. Please check /century/data/log or /century/data/core for more information"
    fi
  fi
}

function stop() {
  ./scripts/dreamview.sh stop
  ./scripts/monitor.sh stop
  for mod in ${CENTURY_BOOTSTRAP_EXTRA_MODULES}; do
    echo "Stopping ${mod}"
    nohup cyber_launch stop ${mod}
  done
}

case $1 in
  start)
    start $2
    ;;
  stop)
    stop
    ;;
  restart)
    stop
    start $2
    ;;
  *)
    start
    ;;
esac
