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
  /century/sensor/camera/front/image
  /century/sensor/camera/left_front/image
  /century/sensor/camera/left_rear/image
  /century/sensor/camera/rear/image
  /century/sensor/camera/right_front/image
  /century/sensor/camera/right_rear/image
)
  #/lidar/bp/front_left
  #/lidar/bp/front_right
  #/lidar/bp/rear_left
  #/lidar/bp/rear_right
  #/lidar/helios/front_left
  #/lidar/helios/rear_right
  #/century/perception/inner/lidar_bp_front_left_segmentation/debug/seg_pointcloud
  #/century/perception/inner/lidar_bp_front_right_segmentation/debug/seg_pointcloud
  #/century/perception/inner/lidar_bp_rear_left_segmentation/debug/seg_pointcloud
  #/century/perception/inner/lidar_bp_rear_right_segmentation/debug/seg_pointcloud
  #/century/perception/inner/lidar_helios_front_left_segmentation/debug/seg_pointcloud
  #/century/perception/inner/lidar_helios_rear_right_segmentation/debug/seg_pointcloud


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
