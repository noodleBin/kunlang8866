load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:variables.bzl", "PERCEPTION_COPTS")
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bev_occupancy_cluster",
    srcs = [
        "bev_occupancy_cluster.cc",
        "island_segment.cc",
        "occ_map_generator.cc"
    ],
    hdrs = [
        "bev_occupancy_cluster.h",
        "island_segment.h",
        "occ_map_generator.h"
    ],
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/perception/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/pointcloud_cluster/proto:pointcloud_cluster_config_cc_proto",
        "//modules/perception/lidar/lib/interface:base_pointcloud_cluster",
        "//modules/perception/lidar/lib/interface:base_roi_filter",
        "//modules/perception/lidar/lib/map_manager",
        "//modules/perception/common/point_cloud_processing:point_cloud_processing",
        "//modules/perception/base:singleton",
        "@eigen",
    ],
    alwayslink = True,
)

cc_binary(
    name = "test_bev_occupancy_cluster",
    srcs = [
        "test_bev_occupancy_cluster.cc"
    ],
    deps = [
        "//modules/perception/lidar/lib/pointcloud_cluster/bev_occupancy:bev_occupancy_cluster",
    ],
)

cpplint()
