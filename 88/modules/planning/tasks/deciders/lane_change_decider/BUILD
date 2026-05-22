load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_change_decider",
    srcs = ["lane_change_decider.cc"],
    hdrs = ["lane_change_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/map/pnc_map:route_segments",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning/tasks/deciders:decider_base",
    ],
)

cc_test(
    name = "lane_change_decider_test",
    size = "small",
    srcs = ["lane_change_decider_test.cc"],
    deps = [
        "lane_change_decider",
        "@com_google_googletest//:gtest_main",
    ],
)



cpplint()
