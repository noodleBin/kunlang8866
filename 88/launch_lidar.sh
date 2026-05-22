#!/bin/bash 

nohup mainboard -d /century/modules/drivers/lidar/robosense/dag/rs32_rear.dag -s drivers > /dev/null 2>&1 &
sleep 0.5
nohup mainboard -d /century/modules/drivers/lidar/robosense/dag/rs32_front.dag -s drivers > /dev/null 2>&1 &
sleep 0.5
nohup mainboard -d /century/modules/drivers/lidar/robosense/dag/rsbp_rear.dag -s drivers > /dev/null 2>&1 &
sleep 0.5
nohup mainboard -d /century/modules/drivers/lidar/robosense/dag/rsbp_front.dag -s drivers > /dev/null 2>&1 &
sleep 0.5
nohup mainboard -d /century/modules/drivers/lidar/robosense/dag/rsbp_rear_left.dag -s drivers > /dev/null 2>&1 &
sleep 0.5
nohup mainboard -d /century/modules/drivers/lidar/robosense/dag/rsbp_front_right.dag -s drivers > /dev/null 2>&1 &

# nohup mainboard -d /century/modules/drivers/lidar/robosense/dag/rsm1_rear.dag -s drivers > /dev/null 2>&1 &
# nohup mainboard -d /century/modules/drivers/lidar/robosense/dag/rsm1_front.dag -s drivers > /dev/null 2>&1 &

