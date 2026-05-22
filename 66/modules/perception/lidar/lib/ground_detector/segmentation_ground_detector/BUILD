load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PERCEPTION_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "segmentation_ground_detector",
    srcs = ["segmentation_ground_detector.cc"],
    hdrs = ["segmentation_ground_detector.h"],
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/ground_detector/segmentation_ground_detector/proto:segmentation_ground_detector_config_cc_proto",
        "//modules/perception/lidar/lib/interface",
        "@eigen",
    ],
    alwayslink = True,
)

cc_test(
    name = "segmentation_ground_detector_test",
    size = "small",
    srcs = ["segmentation_ground_detector_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":segmentation_ground_detector",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
