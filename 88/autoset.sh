#!/bin/bash

# start can driver
cd /century;
echo "nvidia" | sudo -S bash /century/cansetup.sh;
# start canbus
# you should killall -9 mainboard in other .sh before cyber_launch start
# so better use bash launch_century_system.sh directly
{
    canbus=$(ps -ef | grep "canbus" | grep -v "grep")
    
    if [ -z "${canbus}" ]; then
        echo "No canbus module needs to be stopped."
    fi

    if [ "$#" -eq 0 ]; then
        echo "${canbus}" | awk '{print $2}' | xargs kill -s 9
        echo "canbus have been stopped."
    fi
   # ps -ef|grep canbus|grep -v grep|awk '{print $2}'|xargs kill -9
   sleep 0.1
   source cyber/setup.bash;
   mainboard -d /century/modules/canbus/dag/canbus.dag -p canbus_sched -s CYBER_DEFAULT
}&
