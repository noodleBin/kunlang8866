#!/bin/bash

# ======== Configurable Parameters ========
JOBS="${JOBS:-8}"
CPU_RESOURCE="${CPU_RESOURCE:-HOST_CPUS*0.9}"
RAM_RESOURCE="${RAM_RESOURCE:-HOST_RAM*0.9}"
COPT_FLAGS="${COPT_FLAGS:---copt=-fPIC}"
HOST_COPT_FLAGS="${HOST_COPT_FLAGS:---host_copt=-fPIC}"
BAZEL_OPTS="${BAZEL_OPTS:-}"

# WORKER_MEMORY="$((RAM_RESOURCE / JOBS))MB"

# ======== Default target lists ========
ALL_TARGETS=(
  "//cyber/tools/..."
  "//cyber/mainboard/..."
  "//modules/dreamview:dreamview"
  "//modules/planning:libplanning_component.so"
  "//modules/routing:librouting_component.so"
  "//modules/control:libcontrol_component.so"
  "//modules/monitor:libmonitor.so"
  "//modules/bridge:libudp_bridge_receiver_component.so"
  "//modules/mcloud:libmcloud.so"
  "//modules/canbus:libcanbus_component.so"
  "//modules/localization/ins:libins_loc_component.so"
  "//modules/drivers/gnss:libgnss_component.so"
  "//modules/perception/onboard/component:libperception_component_lidar.so"
  "//modules/perception/lidar_tracking:shell_component"
  "//modules/perception/lidar_tracking:sample_lidar_tracking"
  "//modules/drivers/lidar/robosense:librobosense_driver_component.so"
  "//modules/led_monitor:libled_monitor_component.so"
  "//modules/drivers/camera:libcamera_component.so"
  "//modules/fas_aeb_backend:lib_fas_aeb_backend.so"
)

MASTER_TARGETS=(
  "//cyber/tools/..."
  "//cyber/mainboard/..."  
  "//modules/dreamview:dreamview"
  "//modules/planning:libplanning_component.so"
  "//modules/routing:librouting_component.so"
  "//modules/control:libcontrol_component.so"
  "//modules/canbus:libcanbus_component.so"
  "//modules/monitor:libmonitor.so"
  "//modules/mcloud:libmcloud.so"
  "//modules/led_monitor:libled_monitor_component.so"
)

SLAVE_TARGETS=(
  "//cyber/tools/..."
  "//cyber/mainboard/..."  
  "//modules/localization/ins:libins_loc_component.so"
  "//modules/drivers/gnss:libgnss_component.so"
  "//modules/perception/onboard/component:libperception_component_lidar.so"
  "//modules/perception/lidar_tracking:shell_component"
  "//modules/perception/lidar_tracking:sample_lidar_tracking"
  "//modules/transform:libstatic_transform_component.so"
  "//modules/drivers/lidar/robosense:librobosense_driver_component.so"
  "//modules/led_monitor:libled_monitor_component.so"
  "//modules/localization/msf:liblidar_loc_component.so"
  "//modules/localization/msf:libmsf_loc_component.so"
  "//modules/drivers/camera:libcamera_component.so"
)

if [ -z "$1" ]; then
  if [ -f /usr/autowin/version ]; then
    DomainController=`cat /usr/autowin/version | grep 'DomainController:' | awk '{print $2}'`
    case "${DomainController}" in
        "slave")  TARGETS=("${SLAVE_TARGETS[@]}") ;;
        "master") TARGETS=("${MASTER_TARGETS[@]}");;
    esac
  else 
    TARGETS=("${ALL_TARGETS[@]}")
  fi
else
  case "$1" in
    66)
      TARGETS=("${MASTER_TARGETS[@]}")
      echo ">>> Using MASTER-specific targets"
      ;;
    88)
      TARGETS=("${SLAVE_TARGETS[@]}")
      echo ">>> Using SLAVE-specific targets"
      ;;
    *)
      TARGETS=()
      keywords=("$@") 
      
      for t in "${ALL_TARGETS[@]}"; do
        for keyword in "${keywords[@]}"; do
            [[ "$t" == *"$keyword"* ]] && { TARGETS+=("$t"); break; }
        done
      done
      
      if [ ${#TARGETS[@]} -eq 0 ]; then
          echo "❌ No matching target found for keyword: $1"
          exit 1
      fi
      ;;
  esac
fi

# ======== Print configuration ========
echo ">>> Targets to build:"
for tgt in "${TARGETS[@]}"; do
  echo "    $tgt"
done
echo ">>> Jobs: $JOBS"
echo ">>> CPU: $CPU_RESOURCE | RAM: $RAM_RESOURCE"
echo ">>> Cache: $CACHE_DIR"
echo ">>> COPT: $COPT_FLAGS | HOST_COPT: $HOST_COPT_FLAGS"
echo ">>> Extra Bazel options: $BAZEL_OPTS"
echo ">>> Starting build..."

function century_env_setup() {
    CENTURY_ENV="${CENTURY_ENV} STAGE=${STAGE}"
    CENTURY_ENV="${CENTURY_ENV} USE_ESD_CAN=${USE_ESD_CAN}"

    if [[ -z "${CENTURY_BAZEL_DIST_DIR}" ]]; then
        source "${TOP_DIR}/cyber/setup.bash"
    fi

    if [[ ! -d "${CENTURY_BAZEL_DIST_DIR}" ]]; then
        mkdir -p "${CENTURY_BAZEL_DIST_DIR}"
    fi

    if [ ! -f "${CENTURY_ROOT_DIR}/.century.bazelrc" ]; then
        env ${CENTURY_ENV} bash "${CENTURY_ROOT_DIR}/scripts/century_config.sh" --noninteractive
    fi
}

century_env_setup

# ======== Run Bazel ========
bazel build \
  --config=gpu \
  --config=opt \
  --jobs="$JOBS" \
  --local_cpu_resources="$CPU_RESOURCE" \
  --local_ram_resources="$RAM_RESOURCE" \
  --spawn_strategy=local \
  $COPT_FLAGS \
  $HOST_COPT_FLAGS \
  --keep_going \
  $BAZEL_OPTS \
  "${TARGETS[@]}"
