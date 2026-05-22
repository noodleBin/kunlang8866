#!/bin/bash
# set -x
cd /century/data/bag/
current_date=$(date +"%Y%m%d")
# seven_days_ago=$(date -d "7 days ago" +"%Y%m%d")

# for dir in $(ls -d */ | grep -E "^[0-9]{8}/$" | sed 's#/##'); do
#     if [[ "$dir" < "$seven_days_ago" ]]; then
#         echo "delete dir: $dir"
#         rm -r "$dir"
#     fi
# done

if [[ ! -d ${current_date} ]]; then
  mkdir ${current_date}
fi
cd  ${current_date}

LIDAR_POINTCLOUD_EXCLUDES=(
  /lidar/bp/front_left
  /lidar/bp/rear_right
  /lidar/bp/front_right
  /lidar/bp/rear_left
  /lidar/helios/front_left
  /lidar/helios/rear_right
  /lidar/m1/front
  /lidar/m1/rear
)

LIDAR_EXCLUDE_ARGS=()
for topic in "${LIDAR_POINTCLOUD_EXCLUDES[@]}"; do
  LIDAR_EXCLUDE_ARGS+=("-k" "${topic}")
done

if [ -f /usr/autowin/version ]; then
  DomainController=`cat /usr/autowin/version | grep 'DomainController:' | awk '{print $2}'`
  case "${DomainController}" in
      "slave")  nohup cyber_recorder record -a -i 30 -m 8192 "${LIDAR_EXCLUDE_ARGS[@]}" > /dev/null 2>&1 & ;;
      "master") nohup cyber_recorder record -a -i 60 -m 8192 "${LIDAR_EXCLUDE_ARGS[@]}" -k /century/debug/perception/obstacles > /dev/null 2>&1 &;;
  esac
fi
