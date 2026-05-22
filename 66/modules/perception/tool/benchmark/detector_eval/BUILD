load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PERCEPTION_COPTS")

package(default_visibility = ["//visibility:public"])


cc_binary(
    name = "detector_eval",
    srcs = ["detector_eval.cc"],
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber/time:clock",
        "//modules/perception/base",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/app:lidar_obstacle_detector",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/detector/bev_detection:bevfusion_detector",
        "//modules/perception/base:singleton",
        "@eigen",
        "@boost",
    ],
)

cpplint()