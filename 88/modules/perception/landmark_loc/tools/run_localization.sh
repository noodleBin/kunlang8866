#!/bin/bash

source cyber/setup.bash
for file in /century/data/bag/1104/*.record; do
    sleep 1
    base=$(basename "$file" .record)
    /century/bazel-bin/modules/perception/lidar_tracking/tools/colllect_dr_camer_data $base &
    sleep 1
    cyber_recorder play -f $file
    kill -9 $(pgrep -f colllect_dr_camer_data)
done

