#!/bin/bash

TARGET_PERCEPTION_CONF_DIR="/century/modules/perception/production/conf"
SOURCE_PERCEPTION_CONF_DIR="/home/nvidia/workspace/vehicle_config/perception/conf"

if [ ! -L "$TARGET_PERCEPTION_CONF_DIR" ] && [ -e "$SOURCE_PERCEPTION_CONF_DIR" ]; then
    rm -rf "$TARGET_PERCEPTION_CONF_DIR"
    ln -s "$SOURCE_PERCEPTION_CONF_DIR" "$TARGET_PERCEPTION_CONF_DIR"
    echo "Created symbolic link from $SOURCE_PERCEPTION_CONF_DIR to $TARGET_PERCEPTION_CONF_DIR"
else
    echo "Symbolic link already exists at $TARGET_PERCEPTION_CONF_DIR or source missing"
fi


TARGET_PERCEPTION_MODELS_DIR="/century/modules/perception/production/data/perception/lidar"
SOURCE_PERCEPTION_MODELS_DIR="/home/nvidia/workspace/vehicle_config/perception/lidar"

if [ ! -L "$TARGET_PERCEPTION_MODELS_DIR" ] && [ -e "$SOURCE_PERCEPTION_MODELS_DIR" ]; then
    rm -rf "$TARGET_PERCEPTION_MODELS_DIR"
    ln -s "$SOURCE_PERCEPTION_MODELS_DIR" "$TARGET_PERCEPTION_MODELS_DIR"
    echo "Created symbolic link from $SOURCE_PERCEPTION_MODELS_DIR to $TARGET_PERCEPTION_MODELS_DIR"
else
    echo "Symbolic link already exists at $TARGET_PERCEPTION_MODELS_DIR or source missing"
fi

LIDAR_DRIVER_CONF_DIR="/century/modules/drivers/lidar/robosense/conf"
SOURCE_LIDAR_DRIVER_CONF_DIR="/home/nvidia/workspace/vehicle_config/lidar_driver/robosense/conf"

if [ ! -L "$LIDAR_DRIVER_CONF_DIR" ] && [ -e "$SOURCE_LIDAR_DRIVER_CONF_DIR" ]; then
    rm -rf "$LIDAR_DRIVER_CONF_DIR"
    ln -s "$SOURCE_LIDAR_DRIVER_CONF_DIR" "$LIDAR_DRIVER_CONF_DIR"
    echo "Created symbolic link from $SOURCE_LIDAR_DRIVER_CONF_DIR to $LIDAR_DRIVER_CONF_DIR"
else
    echo "Symbolic link already exists at $LIDAR_DRIVER_CONF_DIR or source missing"
fi

echo "Creation of symbolic links completed."