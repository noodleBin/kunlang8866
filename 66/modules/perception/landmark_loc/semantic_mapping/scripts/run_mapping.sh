#!/bin/bash

source cyber/setup.bash
pids=$(pgrep -f colllect_dr_camer_data)
if [ -z "$pids" ]; then
    echo "no colllect_dr_camer_data exit"
else
    kill -9 $(pgrep -f colllect_dr_camer_data)
fi
if [ -d "/century/data/log" ]; then
    rm -rf /century/data/log/*
else
    mkdir -p /century/data/log/
fi
/century/bazel-bin/modules/perception/landmark_loc/tools/colllect_dr_camer_data &
sleep 3
for file in /mnt/disk6/bag/now/*.record; do
    cyber_recorder play -f $file
done
kill -9 $(pgrep -f colllect_dr_camer_data)

if [ -d "/century/data/data/image" ]; then
    rm -rf /century/data/data/image/*
else
    mkdir -p /century/data/data/image/
fi
if [ -d "/century/data/data/pcl" ]; then
    rm -rf /century/data/data/pcl/*
else
    mkdir -p /century/data/data/pcl/
fi
if [ -d "/century/data/data/parameter" ]; then
    rm -rf /century/data/data/parameter/*
else
    mkdir -p /century/data/data/parameter/
fi
cp  /century/modules/perception/landmark_loc/parameter/*  /century/data/data/parameter/
cp -rf /century/data/log/FrontLeft data/data/image/
rm /century/data/log/FrontLeft/*
cp -rf /century/data/log/FrontMiddle data/data/image/
rm /century/data/log/FrontMiddle/*
cp -rf /century/data/log/FrontRight data/data/image/
rm /century/data/log/FrontRight/*
#cp -rf /century/data/log/RearLeft data/data/image/
#cp -rf /century/data/log/RearMiddle data/data/image/
#cp -rf /century/data/log/RearRight data/data/image/
cp -rf /century/data/log/*_pcl data/data/pcl
cp /century/data/log/localization.txt data/data/
rm -rf /century/data/log/*_pcl
#rm /century/data/log/RearLeft/*
#rm /century/data/log/RearMiddle/*
#rm /century/data/log/RearRight/*
export LD_LIBRARY_PATH=/century/modules/perception/landmark_loc/segmentation/onnx/lib/x86
bazel-bin/modules/perception/landmark_loc/segmentation_node /century/data/data/image/FrontMiddle/
bazel-bin/modules/perception/landmark_loc/segmentation_node /century/data/data/image/FrontLeft/
bazel-bin/modules/perception/landmark_loc/segmentation_node /century/data/data/image/FrontRight/
#bazel-bin/modules/perception/landmark_loc/segmentation_node /century/data/data/image/RearMiddle/
#bazel-bin/modules/perception/landmark_loc/segmentation_node /century/data/data/image/RearLeft/
#bazel-bin/modules/perception/landmark_loc/segmentation_node /century/data/data/image/RearRight/
bazel-bin/modules/perception/landmark_loc/mapping_node
cp /century/data/log/semantic/amcl_map.pb.bin /century/data/data/
