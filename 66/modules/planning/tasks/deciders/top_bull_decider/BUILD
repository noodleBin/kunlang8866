load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "top_bull_decider",
    srcs = ["top_bull_decider.cc"],
    hdrs = ["top_bull_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/proto:planning_status_cc_proto",
        "//modules/planning/proto:task_config_cc_proto",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/common/util:common_lib",
    ],
)

cc_test(
    name = "top_bull_decider_test",
    size = "small",
    srcs = ["top_bull_decider_test.cc"],
    deps = [
        "top_bull_decider",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()