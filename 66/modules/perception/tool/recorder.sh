cyber_recorder record -c /lidar/bp/front_left \
                         /lidar/bp/rear_right \
                         /lidar/bp/front_right \
                         /lidar/bp/rear_left \
                         /lidar/helios/front_left \
                         /lidar/helios/rear_right \
                         /tf_static \
                         /century/sensor/camera/front/image/compressed \
                         /century/sensor/camera/left_front/image/compressed \
                         /century/sensor/camera/left_rear/image/compressed \
                         /century/sensor/camera/rear/image/compressed \
                         /century/sensor/camera/right_front/image/compressed \
                         /century/sensor/camera/right_rear/image/compressed \
                         /century/loc/pose \



nohup cyber_recorder record -c /lidar/bp/front_left /lidar/bp/rear_right /lidar/bp/front_right /lidar/bp/rear_left /lidar/helios/front_left /lidar/helios/rear_right /tf_static /century/sensor/camera/front/image/compressed /century/sensor/camera/left_front/image/compressed /century/sensor/camera/left_rear/image/compressed /century/sensor/camera/rear/image/compressed /century/sensor/camera/right_front/image/compressed /century/sensor/camera/right_rear/image/compressed /lidar/m1/front /lidar/m1/rear /century/loc/pose -i 60 -m 16384 > /dev/null 2>&1 &
