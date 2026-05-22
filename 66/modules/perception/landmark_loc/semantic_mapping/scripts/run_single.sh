#!/bin/bash

source cyber/setup.bash
pids=$(pgrep -f colllect_dr_camer_data)
if [ -z "$pids" ]; then
    echo "no colllect_dr_camer_data exit"
else
    kill -9 $(pgrep -f colllect_dr_camer_data)
fi

for file in /century/data/bag/training/test/*.record; do
  if [ -d "/century/data/log" ]; then
    rm -rf /century/data/log/*
  else
    mkdir -p /century/data/log/
  fi
  base=$(basename "$file" .record)
  /century/bazel-bin/modules/perception/lidar_tracking/tools/colllect_dr_camer_data &
  sleep 1
  cyber_recorder play -f $file

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
  export LD_LIBRARY_PATH=/century/modules/perception/landmark_loc/segmentation/onnx/lib/
  bazel-bin/modules/perception/landmark_loc/segmentation_node /century/data/data/image/FrontMiddle/
  bazel-bin/modules/perception/landmark_loc/segmentation_node /century/data/data/image/FrontLeft/
  bazel-bin/modules/perception/landmark_loc/segmentation_node /century/data/data/image/FrontRight/
  #bazel-bin/modules/perception/landmark_loc/segmentation_node /century/data/data/image/RearMiddle/
  #bazel-bin/modules/perception/landmark_loc/segmentation_node /century/data/data/image/RearLeft/
  #bazel-bin/modules/perception/landmark_loc/segmentation_node /century/data/data/image/RearRight/
  bazel-bin/modules/perception/landmark_loc/mapping_node
  cp /century/data/log/semantic/global.png /century/data/core/
  mv /century/data/core/global.png /century/data/core/$base.png
done
