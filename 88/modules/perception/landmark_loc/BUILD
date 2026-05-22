load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")
cc_library(
    name = "semantic_mapping",
    srcs = glob([
        "semantic_mapping/mapping_2d.cc",
        "semantic_mapping/likelihood_field.cc",
        "semantic_mapping/*.h",
        "semantic_mapping/amcl_map/*.h",
        "semantic_mapping/amcl_map/*.cc",
    ]),
    includes = [
        ".",
    ],
    visibility = ["//visibility:public"],
    linkopts = ["-lm"],
    copts = ["-g"],
    deps = [
        "//modules/perception/landmark_loc/g2o:g2o",
        "@com_github_google_glog//:glog",
        "@opencv//:highgui",
        "@opencv//:imgproc",
        "@opencv//:core",
        "@opencv//:imgcodecs",
        "@local_config_pcl//:pcl",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "//modules/perception/landmark_loc/proto:amcl_map_cc_proto",
        "//cyber/common:file",
        "@boost",
    ],
)

cc_library(
    name = "fused_object",
    srcs = glob([
        "fused_object/*.cc",
        "fused_object/*.h",
    ]),
    includes = [
        ".",
        "amcl/include",
    ],
    visibility = ["//visibility:public"],
    linkopts = ["-lm"],
    copts = ["-g"],
    deps = [
        "@eigen",
        "@com_github_google_glog//:glog",
        "@opencv//:highgui",
        "@opencv//:imgproc",
        "@opencv//:core",
        "@opencv//:imgcodecs",
        "@opencv//:calib3d",
        "@local_config_pcl//:pcl",
        "@tinyxml2",
        "@boost",
    ],
)

cc_binary(
    name = "mapping_node",
    srcs = [
        "mapping_node.cc",
        "config/semantic_mapping_config.cc",
        "config/semantic_mapping_config.h",
    ],
    includes = [
        ".",
        "amcl/src/include/",
        "amcl/include/"
    ],
   # linkopts = ["-lm"], 
    copts = ["-g"],
   # linkopts = ["-L/usr/lib/x86_64-linux-gnu", "-ltbb"],
    deps = [
        ":semantic_mapping",
        ":fused_object",
        "//modules/perception/landmark_loc/amcl:amcl_loc",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    ])


cc_binary(
    name = "landmark_loc_node",
    srcs = [
        "landmark_loc_node.cc",
        "sensor_data_provider/data_provider.cc",
        "sensor_data_provider/data_provider.h",
        "amcl_worker/amcl_worker.h",
        "amcl_worker/amcl_worker.cc",
        "amcl_worker/amcl_pose_safety_checker.h",
        "amcl_worker/amcl_pose_safety_checker.cc",
        "amcl_worker/recorder/amcl_recorder.h",
        "amcl_worker/recorder/amcl_recorder.cc",
        "config/landmark_loc_config.cc",
        "config/landmark_loc_config.h",
        "amcl_worker/visualizer/map_show.cc",
        "amcl_worker/visualizer/map_show.h",
        "publisher/amcl_pose_publisher.cc",
        "publisher/amcl_pose_publisher.h"
    ],
    includes = [
        ".",
        "amcl/src/include/",
        "amcl/include/"
    ],

    copts = ["-g"],
    deps = [
      "//modules/perception/landmark_loc/amcl:amcl_loc",
      "//modules/perception/landmark_loc/segmentation:segmentation",
      "//modules/perception/landmark_loc/delayed_ekf:delayed_measurement_ekf",
      "//modules/perception/landmark_loc/imu_fusion:delayed_measurement_eskf",
      ":semantic_mapping",
      ":fused_object",
      "//cyber:cyber",
      "@opencv//:imgcodecs",
      "@opencv//:highgui",
      "@local_config_pcl//:pcl",
      "@eigen",
      "@boost",
      "@tinyxml2",
      "@com_github_jbeder_yaml_cpp//:yaml-cpp",
      "//modules/perception/onboard/transform_wrapper",
      "//modules/drivers/proto:sensor_image_cc_proto",
      "//modules/drivers/proto:pointcloud_cc_proto",
      "//modules/drivers/camera/proto:config_cc_proto",
      "//modules/drivers/lidar/proto:config_proto",
      "//modules/perception/onboard/proto:lidar_component_config_cc_proto",
      "//modules/perception/base:base",
      "//modules/localization/proto:localization_cc_proto",
      "//modules/perception/landmark_loc/proto:amcl_recorder_cc_proto",
      "//modules/localization/proto:msf_reset_cc_proto",
      "//modules/localization/proto:imu_cc_proto",
      "//modules/canbus/proto:chassis_cc_proto",

    ])

cc_binary(
    name = "segmentation_node",
    srcs = [
      "segmentation_node.cc",
      "config/semantic_mapping_config.cc",
      "config/semantic_mapping_config.h",
    ],
    includes = [
      ".",
      "segmentation/src/"
    ],
    copts = ["-g"],
    deps = [
      "//modules/perception/landmark_loc/segmentation:segmentation",
      "@com_github_jbeder_yaml_cpp//:yaml-cpp",
      "//cyber:cyber",
    ])
