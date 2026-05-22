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

if [ "$(cat /proc/sys/kernel/sched_rt_runtime_us)" != "-1" ]; then
  echo -1 | sudo tee /proc/sys/kernel/sched_rt_runtime_us > /dev/null
fi

function run_nvidia_slave() {
  nohup mainboard -d /century/modules/drivers/gnss/dag/gnss.dag -p drivers -s drivers > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/localization/dag/ins_loc.dag -p localization_sched > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/transform/dag/static_transform.dag -p transform_sched > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/perception/production/dag/dag_streaming_kl_perception_lidar.dag -p perception_sched > /dev/null 2>&1 &    
  sleep 1
  nohup taskset -c 6 /century/bazel-bin/modules/perception/lidar_tracking/sample_lidar_tracking > /dev/null 2>&1 &
  sleep 1
  bash /century/launch_lidar_bak.sh
  sleep 1
  nohup bash /century/launch_camera_bak.sh > /dev/null 2>&1 &
  sleep 1
  nohup taskset -c 7 bash /century/run_nc_bak.sh > /dev/null 2>&1 &
  nohup taskset -c 7 bash /century/node_monitor_bak.sh &
  nohup taskset -c 7 /century/bazel-bin/modules/dreamview/backend/hmi/map_change_listener > /dev/null 2>&1 &
  nohup taskset -c 7 /century/bin/bag_backup_daemon > /dev/null 2>&1 &
  nohup taskset -c 7 /century/bin/monitor_node > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century//modules/camera_monitor/dag/camera_monitor.dag -p camera_monitor_sched > /dev/null 2>&1 &
}

function run_nvidia_master() {
  nohup bash /century/autoset.sh > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/planning/dag/planning.dag -p planning_sched > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/control/dag/control.dag -p control_sched > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/routing/dag/routing.dag -p routing_sched > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/mcloud/dag/mcloud.dag -p mcloud_sched > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/monitor/dag/monitor.dag -p monitor_sched  > /dev/null 2>&1 &
  sleep 1
  nohup mainboard -d /century/modules/fas_aeb_backend/dag/fas_aeb_backend.dag -p fas_aeb_backend_sched > /dev/null 2>&1 &
  sleep 1

  bash /century/launch_dreamview_bak.sh

  nohup taskset -c 7 bash /century/run_nc_bak.sh > /dev/null 2>&1 &
  nohup taskset -c 7 bash /century/node_monitor_bak.sh &
}

function main() {
  source /century/cyber/setup.bash
  bash /century/record_bak.sh

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
