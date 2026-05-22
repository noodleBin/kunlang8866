load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PERCEPTION_COPTS")
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cone_filter",
    srcs = ["cone_filter.cc"],
    hdrs = ["cone_filter.h"],
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber",
        "//modules/perception/common/algorithm:data_structures_grid",
        "//cyber/common",
        "//modules/common/math:math_utils",
        "//modules/perception/base",
        "//modules/perception/lidar/common",
        "//modules/perception/lib/config_manager",
        "//modules/perception/base:point_cloud",
        "//modules/perception/common/geometry:common",
        "//modules/perception/lidar/lib/interface:base_object_filter",
        "//modules/perception/lidar/lib/object_filter_bank/cone_filter/proto:cone_filter_config_cc_proto",
        "@com_google_googletest//:gtest",
        "@eigen",
    ],
    alwayslink = True,
)

cpplint()
