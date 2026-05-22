load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_overtake_fsm_common",
    hdrs = ["lane_overtake_fsm_common.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/proto:planning_cc_proto",
    ],
)

cc_library(
    name = "lane_overtake_action",
    srcs = ["lane_overtake_action.cc"],
    hdrs = ["lane_overtake_action.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/proto:planning_status_cc_proto",
        "//modules/planning/tasks/deciders/lane_overtake_decider:lane_overtake_fsm_common",
    ],
)

cc_library(
    name = "lane_overtake_condition",
    srcs = ["lane_overtake_condition.cc"],
    hdrs = ["lane_overtake_condition.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/tasks/deciders/lane_overtake_decider:lane_overtake_fsm_common",
    ],
)

cc_library(
    name = "lane_overtake_fsm",
    srcs = ["lane_overtake_fsm.cc"],
    hdrs = ["lane_overtake_fsm.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/tasks/deciders/lane_overtake_decider:lane_overtake_action",
        "//modules/planning/tasks/deciders/lane_overtake_decider:lane_overtake_condition",
    ],
)

cc_library(
    name = "lane_overtake_decider",
    srcs = ["lane_overtake_decider.cc"],
    hdrs = ["lane_overtake_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/tasks/deciders/lane_overtake_decider:lane_overtake_fsm",
    ],
)


cc_test(
    name = "lane_overtake_fsm_test",
    size = "small",
    srcs = ["lane_overtake_fsm_test.cc"],
    deps = [
        "lane_overtake_fsm",
        "@com_google_googletest//:gtest_main",
    ],
)


cc_test(
    name = "lane_overtake_condition_test",
    size = "small",
    srcs = ["lane_overtake_condition_test.cc"],
    deps = [
        "lane_overtake_condition",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lane_overtake_action_test",
    size = "small",
    srcs = ["lane_overtake_action_test.cc"],
    deps = [
        "lane_overtake_action",
        "@com_google_googletest//:gtest_main",
    ],
)
cpplint()
