load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PERCEPTION_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "app",
    deps = [
        ":lidar_obstacle_detector",
        ":lidar_obstacle_detection",
    #    ":lidar_obstacle_tracking",
    ],
)

cc_library(
    name = "lidar_obstacle_detector",
    srcs = ["lidar_obstacle_detector.cc"],
    hdrs = ["lidar_obstacle_detector.h"],
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber",
        "//modules/common/util:perf_util",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/common:lidar_error_code",
        "//modules/perception/lidar/app/proto:lidar_obstacle_detection_config_cc_proto",
        "//modules/perception/lidar/lib/map_manager",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/object_filter_bank",
        "//modules/perception/lidar/lib/interface:base_lidar_detector",
        "//modules/perception/lidar/lib/interface:base_pointcloud_preprocessor",
        "//modules/perception/lidar/lib/interface:base_lidar_obstacle_detection",
        "//modules/perception/lidar/lib/pointcloud_preprocessor:pointcloud_preprocessor",
        "//modules/perception/lidar/lib/detector/pointpillars_detection_nv:point_pillars_detector",
        "//modules/perception/lidar/lib/detector/center_point_detection:centerpoint_detector", 
        "//modules/perception/lidar/lib/detector/bev_detection:bevfusion_detector", 
        #"//modules/perception/lidar/lib/detector/cnn_segmentation:cnn_segmentation",
        #"//modules/perception/lidar/lib/detector/mask_pillars_detection:mask_pillars_detection",
        #"//modules/perception/lidar/lib/detector/ncut_segmentation:ncut_segmentation",
        #"//modules/perception/lidar/lib/scene_manager",
        "@eigen",
    ],
    alwayslink = True,
)

cc_library(
    name = "lidar_obstacle_detection",
    srcs = ["lidar_obstacle_detection.cc"],
    hdrs = ["lidar_obstacle_detection.h"],
    deps = [
        "//cyber",
        "//modules/common/util:perf_util",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/common:lidar_error_code",
        "//modules/perception/lidar/app/proto:lidar_obstacle_detection_config_cc_proto",
        "//modules/perception/lidar/lib/map_manager",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/object_filter_bank",
        "//modules/perception/lidar/lib/interface:base_lidar_detector",
        "//modules/perception/lidar/lib/interface:base_pointcloud_preprocessor",
        "//modules/perception/lidar/lib/interface:base_lidar_obstacle_detection",
        "//modules/perception/lidar/lib/pointcloud_preprocessor:pointcloud_preprocessor",
        "//modules/perception/lidar/lib/scene_manager",
        "@eigen",
    ],
    alwayslink = True,
)


cpplint()
