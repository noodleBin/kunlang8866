load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PERCEPTION_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "spatio_grid_ground_detector",
    srcs = [
        "spatio_grid_ground_detector.cc",
        "pointcloud_ransac_plane.cc",
    ],
    hdrs = [
        "spatio_grid_ground_detector.h",
        "pointcloud_ransac_plane.h",
    ],
    copts = select({
        "@platforms//cpu:x86_64": [
            "-msse4.1",
        ],
        "@platforms//cpu:aarch64": [],
        "//conditions:default": [],
    }) + PERCEPTION_COPTS,
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/common:perception_timer_util",
        "//modules/perception/lib/registerer",
        "@eigen",
        #"//modules/perception/lib/io:protobuf_util",
        "//modules/perception/common/i_lib",
        "//modules/perception/common/i_lib/pc:i_ground",
        "//modules/perception/common/i_lib/pc:i_struct_s",
        "//modules/perception/common/i_lib/pc:i_util",
        "//modules/perception/common/point_cloud_processing",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/ground_detector/spatio_grid_ground_detector/proto:spatio_grid_ground_detector_config_cc_proto",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/scene_manager",
        "//modules/perception/lidar/lib/scene_manager/ground_service",
        "//modules/perception/proto:perception_config_schema_cc_proto",
    ],
    alwayslink = True,
)

cpplint()
