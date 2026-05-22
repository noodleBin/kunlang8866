load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "lidar_detector_viz",
    srcs = ["lidar_detector_viz.cc",
            "lidar_detector_lite_viz.cc"],
    hdrs = ["lidar_detector_viz.h",
            "lidar_detector_lite_viz.h"],
    deps = [
        "//cyber",
        "//modules/perception/base:base_type",
        "//modules/perception/common/io:io_util",
        "//cyber/time:clock",
        "//modules/common/util:string_util",
        "//modules/perception/base",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/map_manager",
        "//modules/perception/onboard/component:lidar_inner_component_messages",
        "//modules/perception/onboard/inner_component_messages:inner_component_messages",
        "//third_party/pangolin:pangolin",
        "@local_config_pcl//:pcl",
        "@local_config_vtk//:vtk",
        "@eigen",
    ],
    copts = ["-DHAVE_GLEW",
            "-DGLEW_NO_GLU"],
)


cpplint()
