load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PERCEPTION_COPTS")
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pointcloud_cluster",
    srcs = ["pointcloud_cluster.cc", "dbscan_cluster.cc", "euclidean_cluster.cc", "graph_segmentation_cluster.cc"],
    hdrs = ["pointcloud_cluster.h", "dbscan_cluster.h", "euclidean_cluster.h", "graph_segmentation_cluster.h"],
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
        "//modules/perception/lidar/lib/pointcloud_cluster/bev_occupancy:bev_occupancy_cluster",
        "//modules/perception/lidar/lib/interface:base_roi_filter",
        "//modules/perception/lidar/lib/map_manager",
        "//modules/perception/common/point_cloud_processing:point_cloud_processing",
        "@eigen",
    ],
    alwayslink = True,
)

cpplint()
