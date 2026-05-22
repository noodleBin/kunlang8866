load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "colllect_dr_camer_data",
    srcs = ["colllect_dr_camer_data.cc"
           ],
    deps = [
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/perception/lidar_tracking:liblidar_tracking_component",
        "//modules/perception/common/visualization:lidar_detector_viz",
        "//modules/perception/lidar/lib/pointcloud_preprocessor:pointcloud_preprocessor",
        "//cyber:cyber",
        "@opencv//:imgcodecs",
        "@opencv//:highgui",
        "@local_config_pcl//:pcl",

    ]
)