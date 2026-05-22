#! /usr/bin/env bash
TOP_DIR="$(cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd -P)"
source ${TOP_DIR}/scripts/century.bashrc

export CYBER_PATH="${CENTURY_ROOT_DIR}/cyber"

pathprepend "${TOP_DIR}/bin"

export CYBER_DOMAIN_ID=80

platform=`uname -m`
export CYBER_IP=192.168.1.66
if [[ ${platform} == "aarch64" ]]; then
  ip_segment=$([ -d /sys/class/net/eth_xfi0 ] && echo "1" || echo "11")

  if [ -f /usr/autowin/version ]; then
    DomainController=`cat /usr/autowin/version | grep 'DomainController:' | awk '{print $2}'`
    case "${DomainController}" in
        "slave")   CYBER_IP=192.168.$ip_segment.88 ;;
        "master") CYBER_IP=192.168.$ip_segment.66  ;;
    esac
  fi
fi

export PYTHONPATH=/century/bazel-bin/cyber/python:$PYTHONPATH

export GLOG_log_dir="${CENTURY_ROOT_DIR}/data/log"
export GLOG_alsologtostderr=0
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0

export sysmo_start=0

# for DEBUG log
#export GLOG_v=4
