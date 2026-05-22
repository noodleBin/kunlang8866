load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rule_based_stop_decider",
    srcs = ["rule_based_stop_decider.cc"],
    hdrs = ["rule_based_stop_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/status",
        "//modules/map/pnc_map:route_segments",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/tasks/deciders/lane_change_decider",
        "//modules/routing/proto:routing_cc_proto",
    ],
)

cc_test(
    name = "rule_based_stop_decider_test",
    size = "small",
    srcs = ["rule_based_stop_decider_test.cc"],
    deps = [
        "rule_based_stop_decider",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
