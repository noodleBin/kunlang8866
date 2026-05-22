#!/bin/bash 

function run_slave() {
  if command -v tztek-jetson-tool-eeprom-info >/dev/null 2>&1; then
    echo "on slave and tztek dcu, launch camera."
    # ====== RT 调度准备 ======
    # 提升当前 shell 的 rlimit,让 mainboard 进程能用 SCHED_RR
    prlimit --pid $$ --rtprio=99 2>/dev/null || echo "warn: failed to raise rtprio limit"
    prlimit --pid $$ --memlock=unlimited 2>/dev/null || true
    nohup mainboard -d /century/modules/drivers/camera/dag/camera.dag -s drivers -p drivers > /dev/null 2>&1 &
  else
    echo "on slave and corage dcu, do not launch camera."
  fi
}

function run_master() {
    echo "on master dcu, do not launch camera."
}

function run_null() {
    echo "on corage dcu, do not launch camera."
}

function main() {
  source /century/cyber/setup.bash

  if [ -f /usr/autowin/version ]; then
    DomainController=`cat /usr/autowin/version | grep 'DomainController:' | awk '{print $2}'`
    case "${DomainController}" in
        "slave")  run_slave ;;
        "master") run_master;;
    esac
  else
    echo "corage"
    run_null
  fi
}


main
