#!/bin/bash

log_dir="/century/data/log"
mkdir -p "$log_dir"

log_file="$log_dir/process_guard_$(date +%Y%m%d).log"

declare -A processes=(
  ["planning"]="mainboard -d /century/modules/planning/dag/planning.dag"
  ["control"]="mainboard -d /century/modules/control/dag/control.dag -p control_sched"
  ["routing"]="mainboard -d /century/modules/routing/dag/routing.dag"
  ["monitor.dag"]="mainboard -d /century/modules/monitor/dag/monitor.dag"
  ["mcloud"]="mainboard -d /century/modules/mcloud/dag/mcloud.dag"
  ["led"]="mainboard -d /century/modules/led_monitor/dag/led_monitor.dag"
  ["gnss"]="mainboard -d /century/modules/drivers/gnss/dag/gnss.dag -s drivers"
  ["ins_loc"]="mainboard -d /century/modules/localization/dag/ins_loc.dag -p localization_sched"
  ["transform"]="mainboard -d /century/modules/transform/dag/static_transform.dag"
  ["perception"]="mainboard -d /century/modules/perception/production/dag/dag_streaming_kl_perception_lidar.dag -p perception_sched "
  ["lidar_tracking"]="/century/bazel-bin/modules/perception/lidar_tracking/sample_lidar_tracking"
  ["rs_rear"]="mainboard -d /century/modules/drivers/lidar/robosense/dag/rs32_rear.dag -s drivers"
  ["rs_front"]="mainboard -d /century/modules/drivers/lidar/robosense/dag/rs32_front.dag -s drivers"
  ["rsbp_rear"]="mainboard -d /century/modules/drivers/lidar/robosense/dag/rsbp_rear.dag -s drivers"
  ["rsbp_front"]="mainboard -d /century/modules/drivers/lidar/robosense/dag/rsbp_front.dag -s drivers"
  ["fas_aeb_backend"]="mainboard -d /century/modules/fas_aeb_backend/dag/fas_aeb_backend.dag"
)

declare -A special_processes=(
  ["canbus"]="mainboard -d /century/modules/canbus/dag/canbus.dag"
  ["cyber_recorder"]="cyber_recorder record -a"
  ["camera"]="mainboard -d /century/modules/drivers/camera/dag/camera.dag -s drivers"
)

declare -A special_processes_run=(
  ["canbus"]="bash /century/autoset.sh"
  ["cyber_recorder"]="bash /century/record.sh"
  ["camera"]="bash /century/launch_camera.sh"
)

function log {
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$log_file"
}

function remove_process {
    local process_name=$1
    unset processes["$process_name"]
    log "remove process: $process_name"
}

function remove_special_process {
    local process_name=$1
    unset special_processes["$process_name"]
    unset special_processes_run["$process_name"]
    log "remove special process: $process_name"
}

function check_platform_diff {
  if [ ! -f /usr/autowin/version ]; then
    remove_process "transform"
    remove_process "perception"
    remove_process "lidar_tracking"
    return 1
  fi

  remove_process "bridge_receiver_localization"
  remove_process "bridge_receiver_perception"
  remove_process "bridge_receiver_prediction"
  log ${!processes[@]}
  DomainController=`cat /usr/autowin/version | grep 'DomainController:' | awk '{print $2}'`

  if [[ ${DomainController} == "master" ]]; then
    remove_process "gnss"
    remove_process "ins_loc"
    remove_process "transform"
    remove_process "perception"
    remove_process "lidar_tracking"
    remove_process "rs_rear"
    remove_process "rs_front"
    remove_process "rsbp_rear"
    remove_process "rsbp_front"
    remove_special_process "camera"
  else
    remove_special_process "canbus"
    remove_process "monitor"
    remove_process "led"
    remove_process "planning"
    remove_process "control"
    remove_process "routing"
    remove_process "mcloud"
    remove_process "fas_aeb_backend"
  fi
  for name in "${!processes[@]}"; do
    echo "name: $name"
  done

}

function check_and_start {
  local name=$1
  local cmd="${processes[$name]}"

  if ! pgrep -f "$cmd" > /dev/null; then
    log "Restarting $name..."
    nohup $cmd 2>&1 &
  fi
}

function check_and_start_special {
  local name=$1
  local cmd="${special_processes[$name]}"
  local run="${special_processes_run[$name]}"
  if ! pgrep -f "$cmd" > /dev/null; then
    log "Restarting $name..."
    nohup $run 2>&1 &
  fi
}

function guard_processes {
  source /century/cyber/setup.bash
  check_platform_diff

  echo "Special Processes:"
  for name in "${!special_processes[@]}"; do
    echo "Key: $name, Value: ${special_processes[$name]}"
  done
  
  echo -e "\nRegular Processes:"
  for name in "${!processes[@]}"; do
    echo "Key: $name, Value: ${processes[$name]}"
  done

  while true; do
    for name in "${!special_processes[@]}"; do
      check_and_start_special "$name"
    done

    for name in "${!processes[@]}"; do
      check_and_start "$name"
    done
    sleep 5
  done
}

guard_processes
