/******************************************************************************
 * Copyright 2017 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/common/adapters/adapter_gflags.h"

DEFINE_bool(enable_adapter_dump, false,
            "Whether enable dumping the messages to "
            "/tmp/adapters/<topic_name>/<seq_num>.txt for debugging purposes.");
DEFINE_string(gps_topic, "/century/sensor/gnss/odometry", "GPS topic name");
DEFINE_string(imu_topic, "/century/sensor/gnss/corrected_imu",
              "IMU topic name");
DEFINE_string(raw_imu_topic, "/century/sensor/gnss/imu", "Raw IMU topic name");
DEFINE_string(audio_detection_topic, "/century/audio_detection",
              "audio detection topic name");
DEFINE_string(chassis_topic, "/century/canbus/chassis", "chassis topic name");
DEFINE_string(chassis_detail_topic, "/century/canbus/chassis_detail",
              "chassis detail topic name");
DEFINE_string(localization_topic, "/century/loc/pose",
              "localization topic name");
DEFINE_string(loc_topic, "/century/localization/pose",
              "loc topic name");
DEFINE_string(planning_learning_data_topic, "/century/planning/learning_data",
              "planning learning data");
DEFINE_string(planning_trajectory_topic, "/century/planning",
              "planning trajectory topic name");
DEFINE_string(planning_aeb_topic, "/century/planning/planning_aeb",
              "planning aeb topic name");
DEFINE_string(planning_pad_topic, "/century/planning/pad",
              "planning pad topic name");
DEFINE_string(monitor_topic, "/century/monitor", "Monitor");
DEFINE_string(pad_topic, "/century/control/pad",
              "control pad message topic name");
DEFINE_string(control_command_topic, "/century/control",
              "control command topic name");
DEFINE_string(control_preprocessor_topic, "/century/control/preprocessor",
              "control preprocessor topic name");
DEFINE_string(control_local_view_topic, "/century/control/localview",
              "control local view topic name");
DEFINE_string(control_core_command_topic, "/century/control/controlcore",
              "control command core algorithm topic name");
DEFINE_string(pointcloud_topic,
              "/century/sensor/lidar128/compensator/PointCloud2",
              "pointcloud topic name");
DEFINE_string(pointcloud_16_topic,
              "/century/sensor/lidar16/compensator/PointCloud2",
              "16 beam Lidar pointcloud topic name");
DEFINE_string(pointcloud_16_raw_topic, "/century/sensor/lidar16/PointCloud2",
              "16 beam Lidar raw pointcloud topic name");
DEFINE_string(pointcloud_16_front_up_topic,
              "/century/sensor/lidar16/front/up/compensator/PointCloud2",
              "Front up 16 beam Lidar pointcloud topic name");
DEFINE_string(pointcloud_64_topic,
              "/century/sensor/velodyne64/compensator/PointCloud2",
              "pointcloud topic name");
DEFINE_string(pointcloud_128_topic,
              "/century/sensor/lidar128/compensator/PointCloud2",
              "pointcloud topic name for 128 beam lidar");
DEFINE_string(pointcloud_hesai_40p_topic,
              "/century/sensor/hesai40/compensator/PointCloud2",
              "pointcloud topic name for hesai40p lidar");
DEFINE_string(pointcloud_raw_topic, "/century/sensor/velodyne64/PointCloud2",
              "pointcloud raw topic name");
DEFINE_string(velodyne_raw_topic,
              "/century/sensor/velodyne64/VelodyneScanUnified",
              "velodyne64 raw data topic name");
DEFINE_string(pointcloud_fusion_topic,
              "/century/sensor/velodyne64/fusion/PointCloud2",
              "pointcloud fusion topic name");
DEFINE_string(vlp16_pointcloud_topic,
              "/century/sensor/velodyne16/compensator/PointCloud2",
              "16 beam Lidar pointcloud topic name");
DEFINE_string(lidar_16_front_center_topic,
              "/century/sensor/lidar16/front/center/PointCloud2",
              "front center 16 beam lidar topic name");
DEFINE_string(lidar_16_front_up_topic,
              "/century/sensor/lidar16/front/up/PointCloud2",
              "front up 16 beam lidar topic name");
DEFINE_string(lidar_16_rear_left_topic,
              "/century/sensor/lidar16/rear/left/PointCloud2",
              "rear left 16 beam lidar topic name");
DEFINE_string(lidar_16_rear_right_topic,
              "/century/sensor/lidar16/rear/right/PointCloud2",
              "rear right 16 beam lidar topic name");
DEFINE_string(lidar_16_fusion_topic,
              "/century/sensor/lidar16/fusion/PointCloud2",
              "16 beam lidar fusion topic name");
DEFINE_string(lidar_16_fusion_compensator_topic,
              "/century/sensor/lidar16/fusion/compensator/PointCloud2",
              "16 beam lidar fusion compensator topic name");
DEFINE_string(lidar_128_topic, "/century/sensor/lidar128/PointCloud2",
              "128 beam lidar topic name");
DEFINE_string(prediction_topic, "/century/prediction", "prediction topic name");
DEFINE_string(prediction_container_topic, "/century/prediction_container",
              "prediction container submodule topic name");
DEFINE_string(perception_around_ego_obstacle_topic,
              "/century/perception/around_ego/obstalces",
              "perception around_ego obstacle topic name");
DEFINE_string(perception_obstacle_topic, "/century/perception/obstacles",
              "perception obstacle topic name");
DEFINE_string(drive_event_topic, "/century/drive_event",
              "drive event topic name");
DEFINE_string(traffic_light_detection_topic,
              "/century/perception/traffic_light",
              "traffic light detection topic name");
DEFINE_string(perception_lane_mask_segmentation_topic,
              "/century/perception/lane_mask",
              "lane mask segmentation topic name");
DEFINE_string(routing_request_topic, "/century/routing_request",
              "routing request topic name");
DEFINE_string(routing_response_topic, "/century/routing_response",
              "routing response topic name");
DEFINE_string(routing_response_history_topic,
              "/century/routing_response_history",
              "routing response history topic name");
DEFINE_string(relative_odometry_topic, "/century/calibration/relative_odometry",
              "relative odometry topic name");
DEFINE_string(ins_stat_topic, "/century/sensor/gnss/ins_stat",
              "ins stat topic name");
DEFINE_string(ins_status_topic, "/century/sensor/gnss/ins_status",
              "ins status topic name");
DEFINE_string(gnss_status_topic, "/century/sensor/gnss/gnss_status",
              "gnss status topic name");
DEFINE_string(system_status_topic, "/century/monitor/system_status",
              "System status topic name");
DEFINE_string(static_info_topic, "/century/monitor/static_info",
              "Static info topic name");
DEFINE_string(mobileye_topic, "/century/sensor/mobileye",
              "mobileye topic name");
DEFINE_string(smartereye_obstacles_topic,
              "/century/sensor/smartereye/obstacles",
              "smartereye obstacles topic name");
DEFINE_string(smartereye_lanemark_topic, "/century/sensor/smartereye/lanemark",
              "smartereye lanemark topic name");
DEFINE_string(smartereye_image_topic, "/century/sensor/smartereye/image",
              "smartereye image topic name");
DEFINE_string(delphi_esr_topic, "/century/sensor/delphi_esr",
              "delphi esr radar topic name");
DEFINE_string(conti_radar_topic, "/century/sensor/conti_radar",
              "continental radar topic name");
DEFINE_string(racobit_radar_topic, "/century/sensor/racobit_radar",
              "racobit radar topic name");
DEFINE_string(ultrasonic_radar_topic, "/century/sensor/ultrasonic_radar",
              "ultrasonic esr radar topic name");
DEFINE_string(front_radar_topic, "/century/sensor/radar/front",
              "front radar topic name");
DEFINE_string(rear_radar_topic, "/century/sensor/radar/rear",
              "rear radar topic name");
// TODO(Authors): Change the topic name
DEFINE_string(compressed_image_topic, "camera/image_raw",
              "CompressedImage topic name");
DEFINE_string(image_front_topic, "/century/sensor/camera/front_6mm/image",
              "front camera image topic name for obstacles from camera");
DEFINE_string(image_short_topic,
              "/century/sensor/camera/front_6mm/image/compressed",
              "short camera image topic name");
DEFINE_string(image_long_topic, "/century/sensor/camera/traffic/image_long",
              "long camera image topic name");
DEFINE_string(image_usb_cam_topic, "/century/sensor/camera/image_usb_cam",
              "USB camera image topic name");
DEFINE_string(camera_image_long_topic, "/century/sensor/camera/image_long",
              "long camera image topic name");
DEFINE_string(camera_image_short_topic, "/century/sensor/camera/image_short",
              "short camera image topic name");
DEFINE_string(camera_front_6mm_topic, "/century/sensor/camera/front_6mm/image",
              "front 6mm camera topic name");
DEFINE_string(camera_front_6mm_2_topic,
              "/century/sensor/camera/front_6mm_2/image",
              "front 6mm camera topic name 2");
DEFINE_string(camera_front_12mm_topic,
              "/century/sensor/camera/front_12mm/image",
              "front 12mm camera topic name");
DEFINE_string(camera_front_6mm_compressed_topic,
              "/century/sensor/camera/front_6mm/image/compressed",
              "front 6mm camera compressed topic name");
DEFINE_string(camera_front_12mm_compressed_topic,
              "/century/sensor/camera/front_12mm/image/compressed",
              "front 12mm camera compressed topic name");
DEFINE_string(camera_left_fisheye_compressed_topic,
              "/century/sensor/camera/left_fisheye/image/compressed",
              "left fisheye camera compressed topic name");
DEFINE_string(camera_right_fisheye_compressed_topic,
              "/century/sensor/camera/right_fisheye/image/compressed",
              "right fisheye camera compressed topic name");
DEFINE_string(camera_rear_6mm_compressed_topic,
              "/century/sensor/camera/rear_6mm/image/compressed",
              "front 6mm camera compressed topic name");
DEFINE_string(camera_front_6mm_video_compressed_topic,
              "/century/sensor/camera/front_6mm/video/compressed",
              "front 6mm camera video compressed topic name");
DEFINE_string(camera_front_12mm_video_compressed_topic,
              "/century/sensor/camera/front_12mm/video/compressed",
              "front 12mm camera video compressed topic name");
DEFINE_string(camera_left_fisheye_video_compressed_topic,
              "/century/sensor/camera/left_fisheye/video/compressed",
              "left fisheye camera video compressed topic name");
DEFINE_string(camera_right_fisheye_video_compressed_topic,
              "/century/sensor/camera/right_fisheye/video/compressed",
              "right fisheye camera video compressed topic name");
DEFINE_string(camera_rear_6mm_video_compressed_topic,
              "/century/sensor/camera/rear_6mm/video/compressed",
              "front 6mm camera video compressed topic name");
DEFINE_string(gnss_rtk_obs_topic, "/century/sensor/gnss/rtk_obs",
              "Gnss rtk observation topic name");
DEFINE_string(gnss_rtk_eph_topic, "/century/sensor/gnss/rtk_eph",
              "Gnss rtk ephemeris topic name");
DEFINE_string(insx_topic, "/century/sensor/gnss/insx", "Gnss insx");
DEFINE_string(gnss_best_pose_topic, "/century/sensor/gnss/best_pose",
              "Gnss rtk best gnss pose");
DEFINE_string(localization_gnss_topic, "/century/localization/msf_gnss",
              "Gnss localization measurement topic name");
DEFINE_string(localization_lidar_topic, "/century/localization/msf_lidar",
              "Lidar localization measurement topic name");
DEFINE_string(localization_ndt_topic, "/century/localization/ndt_lidar",
              "NDT localization lidar measurement topic name");
DEFINE_string(localization_sins_pva_topic, "/century/localization/msf_sins_pva",
              "Localization sins pva topic name");
DEFINE_string(localization_msf_status, "/century/localization/msf_status",
              "msf localization status");
DEFINE_string(relative_map_topic, "/century/relative_map", "relative map");
DEFINE_string(navigation_topic, "/century/navigation", "navigation");
DEFINE_string(hmi_status_topic, "/century/hmi/status",
              "HMI status topic name.");
DEFINE_string(audio_capture_topic, "/century/hmi/audio_capture",
              "HMI audio capture topic name.");
DEFINE_string(v2x_obu_traffic_light_topic,
              "/century/v2x/obu/internal/traffic_light",
              "v2x obu traffic_light topic name");
DEFINE_string(v2x_internal_obstacle_topic,
              "/century/v2x/obu/internal/obstacles",
              "v2x internal obstacles topic name");
DEFINE_string(v2x_obstacle_topic, "/century/v2x/obstacles",
              "v2x obstacles topic name");
DEFINE_string(v2x_traffic_light_topic, "/century/v2x/traffic_light",
              "v2x traffic light topic name");
DEFINE_string(v2x_traffic_light_for_hmi_topic,
              "/century/v2x/traffic_light/for_hmi",
              "v2x traffic light topic name for hmi");
DEFINE_string(v2x_rsi_topic, "/century/v2x/rsi", "v2x rsi topic name");

DEFINE_string(storytelling_topic, "/century/storytelling",
              "Storytelling topic.");
DEFINE_string(audio_event_topic, "/century/audio_event", "Audio event topic.");

DEFINE_string(guardian_topic, "/century/guardian", "Guardian topic.");
DEFINE_string(gnss_raw_data_topic, "/century/sensor/gnss/raw_data",
              "gnss raw data topic name");
DEFINE_string(stream_status_topic, "/century/sensor/gnss/stream_status",
              "gnss stream status topic name");
DEFINE_string(heading_topic, "/century/sensor/gnss/heading",
              "gnss heading topic name");
DEFINE_string(rtcm_data_topic, "/century/sensor/gnss/rtcm_data",
              "gnss rtcm data topic name");
DEFINE_string(tf_topic, "/tf", "Transform topic.");
DEFINE_string(tf_static_topic, "/tf_static", "Transform static topic.");
DEFINE_string(recorder_status_topic, "/century/data/recorder/status",
              "Recorder status topic.");
DEFINE_string(latency_recording_topic, "/century/common/latency_records",
              "Latency recording topic.");
DEFINE_string(latency_reporting_topic, "/century/common/latency_reports",
              "Latency reporting topic.");
DEFINE_string(task_topic, "/century/task_manager", "task manager topic name");
// value: velodyne128, velodyne64, velodyne16
DEFINE_string(lidar_model_version, "",
              "It determins which lidar model(16 ,64 or 128) to load, "
              "if not to set, the model will be loaded by the sensor name.");
DEFINE_string(system_monitor_x86_topic, "/century/monitor/monitor_data_x86",
              "System monitor topic name in x86");

DEFINE_string(mcloud_info_topic, "/century/mcloud", "mcloud info topic name");
DEFINE_string(imu_raw_topic, "imu_raw", "Raw IMU data topic");
DEFINE_string(front_12mm_status_topic, "/century/camera/front_12mm/status",
              "Front 12 mm camera status topic name");

DEFINE_string(system_monitor_aarch_topic, "/century/monitor/monitor_data_aarch",
              "System monitor topic name in aarch");

DEFINE_string(udas_ultrasonic_topic, "/century/sensor/udas_ultrasonic",
              "udas ultrasonic topic name");
DEFINE_string(udas_ultrasonic_eol_topic, "/century/sensor/udas_ultrasonic_eol",
              "udas ultrasonic eol topic name");
DEFINE_string(cubtek_radar_topic, "/century/sensor/cubtek_radar",
              "cubtek radar topic name");
DEFINE_string(cubtek_radar_eol_topic, "/century/sensor/radar_eol",
              "cubtek radar eol topic name");
DEFINE_string(lgrx_radar_topic, "/century/sensor/lgrx_radar",
              "lgrx radar topic name");
DEFINE_string(movex_radar_topic, "/century/sensor/radar",
              "century radar topic name");
DEFINE_string(microbrain_radar_topic, "/century/sensor/microbrain_radar",
              "microbrain radar topic name");

DEFINE_string(mems_status_topic, "/century/sensor/zvision_ml30sa1/status",
              "MEMS status topic name");

DEFINE_string(tracker_mems_topic, "/century/tracker/mems", "Mems");
DEFINE_string(tracker_radar_topic, "/century/tracker/radar", "Radar");
DEFINE_string(tracker_rs_topic, "/century/tracker/rs", "Rs");
DEFINE_string(tracker_ultrasonic_topic, "/century/tracker/ultrasonic",
              "Ultrasonic");
DEFINE_string(tracker_camera_topic, "/century/tracker/camera", "Camera");

DEFINE_string(front_3mm_status_topic, "/century/camera/front_3mm/status",
              "Front 3 mm camera status topic name");
DEFINE_string(back_3mm_status_topic, "/century/camera/back_3mm/status",
              "Back 3 mm camera status topic name");

DEFINE_string(fas_aeb_info_topic, "/century/fas_aeb_info",
              "Back 3 mm camera status topic name");
DEFINE_string(emergency_stop_topic, "/century/mcloud/emergency_stop",
              "mcoud emergency stop topic name");
