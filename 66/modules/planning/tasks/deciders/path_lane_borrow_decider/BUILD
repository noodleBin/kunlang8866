load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path_lane_borrow_fsm_common",
    hdrs = ["path_lane_borrow_fsm_common.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/proto:planning_cc_proto",
    ],
)

cc_library(
    name = "path_lane_borrow_action",
    srcs = ["path_lane_borrow_action.cc"],
    hdrs = ["path_lane_borrow_action.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:frame",
        "//modules/planning/common:obstacle_blocking_analyzer",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/proto:planning_status_cc_proto",
        "//modules/planning/tasks/deciders/path_lane_borrow_decider:path_lane_borrow_fsm_common",
    ],
)

cc_library(
    name = "path_lane_borrow_condition",
    srcs = ["path_lane_borrow_condition.cc"],
    hdrs = ["path_lane_borrow_condition.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/tasks/deciders/path_lane_borrow_decider:path_lane_borrow_fsm_common",
    ],
)

cc_library(
    name = "path_lane_borrow_fsm",
    srcs = ["path_lane_borrow_fsm.cc"],
    hdrs = ["path_lane_borrow_fsm.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/tasks/deciders/path_lane_borrow_decider:path_lane_borrow_action",
        "//modules/planning/tasks/deciders/path_lane_borrow_decider:path_lane_borrow_condition",
    ],
)

cc_library(
    name = "path_lane_borrow_decider",
    srcs = ["path_lane_borrow_decider.cc"],
    hdrs = ["path_lane_borrow_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:obstacle_blocking_analyzer",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/tasks/deciders/path_lane_borrow_decider:path_lane_borrow_fsm",
    ],
)

#cc_test(
#    name = "path_lane_borrow_decider_test",
#    size = "small",
#    srcs = ["path_lane_borrow_decider_test.cc"],
#    deps = [
#        "path_lane_borrow_decider",
#        "@com_google_googletest//:gtest_main",
#    ],
#)

cpplint()
