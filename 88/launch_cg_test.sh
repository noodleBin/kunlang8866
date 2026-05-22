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

# ── slave cgroup cpuset 划分 ───────────────────────────────────────────────────
# cpu0-1: century_sys        recorder(34%) + IRQ + 后台工具
# cpu2:   century_loc        gnss(10%) + ins_loc(12%) + static_transform(5%)
# cpu3-4: century_lidar      6x lidar driver dag(~10%×6=58%，2核各~29%)
# cpu5:   century_camera     camera(57%) + camera_monitor(7%)
# cpu6-7: century_perception perception dag(89%) + lidar_tracking(10%)

function setup_slave_cgroups() {
  for g in century_sys century_loc century_lidar century_camera century_perception; do
    mkdir -p /sys/fs/cgroup/cpuset/$g
  done

  printf "0-1" > /sys/fs/cgroup/cpuset/century_sys/cpuset.cpus
  printf "0"   > /sys/fs/cgroup/cpuset/century_sys/cpuset.mems

  printf "2"   > /sys/fs/cgroup/cpuset/century_loc/cpuset.cpus
  printf "0"   > /sys/fs/cgroup/cpuset/century_loc/cpuset.mems

  printf "3-4" > /sys/fs/cgroup/cpuset/century_lidar/cpuset.cpus
  printf "0"   > /sys/fs/cgroup/cpuset/century_lidar/cpuset.mems

  printf "5"   > /sys/fs/cgroup/cpuset/century_camera/cpuset.cpus
  printf "0"   > /sys/fs/cgroup/cpuset/century_camera/cpuset.mems

  printf "6-7" > /sys/fs/cgroup/cpuset/century_perception/cpuset.cpus
  printf "0"   > /sys/fs/cgroup/cpuset/century_perception/cpuset.mems

  # cpu6-7 的 per-CPU kworker 单独放一个 cgroup，使 root cpuset 可安全排除 cpu6-7
  mkdir -p /sys/fs/cgroup/cpuset/century_kworker67
  printf "6-7" > /sys/fs/cgroup/cpuset/century_kworker67/cpuset.cpus
  printf "0"   > /sys/fs/cgroup/cpuset/century_kworker67/cpuset.mems
  for pid in $(pgrep kworker 2>/dev/null); do
    name=$(cat /proc/$pid/comm 2>/dev/null)
    if echo "$name" | grep -qE '^kworker/[67]:'; then
      echo "$pid" > /sys/fs/cgroup/cpuset/century_kworker67/cgroup.procs 2>/dev/null
    fi
  done

  # root cpuset 限制到 cpu0-5，未分组的用户态进程无法漂入 cpu6-7
  printf "0-5" > /sys/fs/cgroup/cpuset/cpuset.cpus 2>/dev/null

  # 硬件中断亲和性绑到 cpu0-1（bitmap 0x3），softirq 随之也只在 cpu0-1 处理
  for f in /proc/irq/*/smp_affinity; do
    printf "3" > "$f" 2>/dev/null
  done
  # 网卡 IRQ 单独绑到 cpu1（bitmap 0x2），避免和 recorder 争 cpu0
  for irq_dir in /proc/irq/*/; do
    if grep -qE 'eth|eno|enp|mlx|ixgbe' "${irq_dir}actions" 2>/dev/null; then
      printf "2" > "${irq_dir}smp_affinity" 2>/dev/null
    fi
  done

  # 只限制 unbound kworker 到 cpu0-3，跳过 per-CPU bound kworker（kworker/N:x）
  # 强迁 per-CPU bound kworker 会导致其绑定核的 work queue 永远无法处理 → 系统死锁
  for pid in $(pgrep kworker 2>/dev/null); do
    name=$(cat /proc/$pid/comm 2>/dev/null)
    if echo "$name" | grep -qE '^kworker/[4-9][0-9]*:'; then
      continue
    fi
    taskset -p 0xf "$pid" 2>/dev/null
  done

  # 声明感知核心 cpu6-7 独占，防止其他 cgroup 的用户态进程漂入
  printf "1" > /sys/fs/cgroup/cpuset/century_perception/cpuset.cpu_exclusive 2>/dev/null
}

# 将整个进程组（含所有线程）移入指定 cpuset cgroup
function cg_assign() {
  echo "$1" > /sys/fs/cgroup/cpuset/$2/cgroup.procs 2>/dev/null
}

# ──────────────────────────────────────────────────────────────────────────────

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
  setup_slave_cgroups

  # recorder 由 record.sh 在 main() 里提前启动，在此归入 century_sys
  for pid in $(pgrep -f "cyber_recorder record"); do
    cg_assign $pid century_sys
  done

  # 定位组 → cpu2
  nohup mainboard -d /century/modules/drivers/gnss/dag/gnss.dag -s drivers > /dev/null 2>&1 &
  cg_assign $! century_loc
  sleep 1

  nohup mainboard -d /century/modules/localization/dag/ins_loc.dag > /dev/null 2>&1 &
  cg_assign $! century_loc
  sleep 1

  nohup mainboard -d /century/modules/transform/dag/static_transform.dag > /dev/null 2>&1 &
  cg_assign $! century_loc
  sleep 1

  # 感知组 → cpu6-7，FIFO 优先级 50（高于 lidar 的 30）
  CYBER_SCHED_PERF=1 CYBER_RECV_LATENCY=1 \
  nohup mainboard -d /century/modules/perception/production/dag/dag_streaming_kl_perception_lidar.dag > /dev/null 2>&1 &
  PERC_PID=$!
  cg_assign $PERC_PID century_perception
  sleep 1
  chrt -f -p 50 "$PERC_PID" 2>/dev/null
  for tid in /proc/$PERC_PID/task/*/; do
    chrt -f -p 50 "$(basename $tid)" 2>/dev/null
  done

  nohup /century/bazel-bin/modules/perception/lidar_tracking/sample_lidar_tracking > /dev/null 2>&1 &
  TRACK_PID=$!
  cg_assign $TRACK_PID century_perception
  sleep 1
  chrt -f -p 50 "$TRACK_PID" 2>/dev/null
  for tid in /proc/$TRACK_PID/task/*/; do
    chrt -f -p 50 "$(basename $tid)" 2>/dev/null
  done

  # lidar 驱动组 → cpu3-4（launch_lidar.sh 同步执行，返回后批量 assign）
  bash /century/launch_lidar.sh
  sleep 2
  for pid in $(pgrep -f "robosense/dag"); do
    cg_assign $pid century_lidar
    chrt -f -p 30 "$pid" 2>/dev/null
    # 对进程内所有线程同样设置 FIFO
    for tid in /proc/$pid/task/*/; do
      chrt -f -p 30 "$(basename $tid)" 2>/dev/null
    done
  done
  sleep 1

  # camera 组 → cpu5
  nohup bash /century/launch_camera.sh > /dev/null 2>&1 &
  sleep 2
  nohup mainboard -d /century/modules/camera_monitor/dag/camera_monitor.dag > /dev/null 2>&1 &
  cg_assign $! century_camera
  sleep 1

  # 后台工具 → cpu0-1
  nohup bash /century/run_nc.sh > /dev/null 2>&1 &
  cg_assign $! century_sys

  nohup bash /century/node_monitor.sh &
  cg_assign $! century_sys

  nohup /century/bazel-bin/modules/dreamview/backend/hmi/map_change_listener > /dev/null 2>&1 &
  cg_assign $! century_sys

  nohup /century/bin/bag_backup_daemon > /dev/null 2>&1 &
  cg_assign $! century_sys

  nohup /century/bin/monitor_node > /dev/null 2>&1 &
  cg_assign $! century_sys
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

