#!/bin/bash
 
platform=`uname -m`

if [[ ! -f /century/data ]]; then
  mkdir -p /century/data/log
  mkdir -p /century/data/bag
  mkdir -p /century/data/core
fi


echo ${platform}

bash /century/stop.sh

bash /century/disk_clear.sh

bash /century/create_link.sh

bash -c 'echo -1 > /proc/sys/kernel/sched_rt_runtime_us'

function run_corage() {
  {
    sleep 10
    nohup bash /century/autoset.sh > /dev/null 2>&1 &
  } &
  nohup mainboard -d /century/modules/drivers/gnss/dag/gnss.dag -s drivers > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/localization/dag/ins_loc.dag > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/planning/dag/planning.dag > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/control/dag/control.dag > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/routing/dag/routing.dag > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/bridge/dag/bridge_receiver_localization.dag > /dev/null 2>&1 &
  sleep 1
  nohup  mainboard -d /century/modules/bridge/dag/bridge_receiver_perception.dag > /dev/null 2>&1 &
  sleep 1
  nohup  mainboard -d /century/modules/bridge/dag/bridge_receiver_prediction.dag > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/mcloud/dag/mcloud.dag > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/monitor/dag/monitor.dag > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/led_monitor/dag/led_monitor.dag > /dev/null 2>&1 &
  nohup bash /century/run_nc.sh > /dev/null 2>&1 &
  nohup bash /century/node_monitor.sh &
  nohup bash /century/monitor_corage.sh &
}

function run_nvidia_slave() {
  nohup mainboard -d /century/modules/drivers/gnss/dag/gnss.dag -s drivers > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/localization/dag/ins_loc.dag > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/transform/dag/static_transform.dag > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/perception/production/dag/dag_streaming_kl_perception_lidar.dag > /dev/null 2>&1 &
  sleep 1
  nohup /century/bazel-bin/modules/perception/lidar_tracking/sample_lidar_tracking > /dev/null 2>&1 &
  sleep 1
  bash /century/launch_lidar.sh
  sleep 1
  nohup bash /century/launch_camera.sh > /dev/null 2>&1 &
  sleep 1
  nohup bash /century/run_nc.sh > /dev/null 2>&1 &
  nohup bash /century/node_monitor.sh &
  nohup /century/bazel-bin/modules/dreamview/backend/hmi/map_change_listener > /dev/null 2>&1 &
  nohup /century/bin/bag_backup_daemon > /dev/null 2>&1 &
  nohup /century/bin/monitor_node > /dev/null 2>&1 &
}

function run_nvidia_master() {
  nohup bash /century/autoset.sh > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/planning/dag/planning.dag > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/control/dag/control.dag -p control_sched > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/routing/dag/routing.dag > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/mcloud/dag/mcloud.dag > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/monitor/dag/monitor.dag > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/fas_aeb_backend/dag/fas_aeb_backend.dag > /dev/null 2>&1 &
  sleep 1

  bash /century/launch_dreamview.sh

  nohup bash /century/run_nc.sh > /dev/null 2>&1 &
  nohup bash /century/node_monitor.sh &
}

function main() {
  source /century/cyber/setup.bash
  bash /century/record.sh

  if [ -f /usr/autowin/version ]; then
    DomainController=`cat /usr/autowin/version | grep 'DomainController:' | awk '{print $2}'`
    case "${DomainController}" in
        "slave")  run_nvidia_slave ;;
        "master") run_nvidia_master;;
    esac
  else
    echo "corage"
    run_corage
  fi
}

main

