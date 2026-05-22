#! /usr/bin/env bash
TOP_DIR="$(cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd -P)"
source ${TOP_DIR}/scripts/century.bashrc

export CENTURY_BAZEL_DIST_DIR="${CENTURY_CACHE_DIR}/distdir"
export CYBER_PATH="${CENTURY_ROOT_DIR}/cyber"

bazel_bin_path="${CENTURY_ROOT_DIR}/bazel-bin"
mainboard_path="${bazel_bin_path}/cyber/mainboard"
cyber_tool_path="${bazel_bin_path}/cyber/tools"
recorder_path="${cyber_tool_path}/cyber_recorder"
launch_path="${cyber_tool_path}/cyber_launch"
channel_path="${cyber_tool_path}/cyber_channel"
node_path="${cyber_tool_path}/cyber_node"
service_path="${cyber_tool_path}/cyber_service"
monitor_path="${cyber_tool_path}/cyber_monitor"
visualizer_path="${bazel_bin_path}/modules/tools/visualizer"

# TODO(all): place all these in one place and pathprepend
for entry in "${mainboard_path}" \
    "${recorder_path}" "${monitor_path}"  \
    "${channel_path}" "${node_path}" \
    "${service_path}" \
    "${launch_path}" \
    "${visualizer_path}" ; do
    pathprepend "${entry}"
done

pathprepend ${bazel_bin_path}/cyber/python/internal PYTHONPATH

export CYBER_DOMAIN_ID=80

platform=`uname -m`
export CYBER_IP=127.0.0.1

if [[ ${platform} == "aarch64" ]]; then
    export CYBER_IP=192.168.1.66
fi

if [[ ${platform} == "aarch64" ]]; then
  ip_segment=$([ -d /sys/class/net/eth_xfi0 ] && echo "1" || echo "11")

  if [ -f /usr/autowin/version ]; then
    DomainController=`cat /usr/autowin/version | grep 'DomainController:' | awk '{print $2}'`
    case "${DomainController}" in
  "slave")
		CYBER_IP=192.168.$ip_segment.88   
		export CYBER_RTPS_WHITELIST_ENABLED=1
	  ;;        
	"master") CYBER_IP=192.168.$ip_segment.66  ;;
    esac
  fi
fi

export GLOG_log_dir="${CENTURY_ROOT_DIR}/data/log"
export GLOG_alsologtostderr=0
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0
export GLOG_max_log_size=100

export sysmo_start=0
export PATH="/usr/local/cuda/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH"
# for DEBUG log
#export GLOG_v=4

source ${CYBER_PATH}/tools/cyber_tools_auto_complete.bash
