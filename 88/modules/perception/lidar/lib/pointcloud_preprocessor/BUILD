load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PERCEPTION_COPTS")
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pointcloud_preprocessor",
    srcs = [
        "pointcloud_preprocessor.cc",
    ],
    hdrs = [
        "pointcloud_preprocessor.h",
    ],
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/perception/base",
        "//modules/perception/common/geometry:convex_hull_2d",
        "//modules/perception/common/i_lib",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface:base_pointcloud_preprocessor",
        "//modules/perception/lidar/lib/pointcloud_preprocessor/proto:pointcloud_preprocessor_config_cc_proto",
        "//modules/perception/lidar/lib/roi_filter/ego_service_filter",
        "//modules/perception/lidar/lib/interface:base_lidar_detector",
        "//modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector",
        "//modules/perception/lidar/lib/ground_detector/spatio_grid_ground_detector",
        "//modules/perception/lidar/lib/ground_detector/segmentation_ground_detector",
        "//modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector_v2",
        "//modules/perception/common:perception_timer_util",
        "@eigen",
    ],
    alwayslink = True,
)

cpplint()
