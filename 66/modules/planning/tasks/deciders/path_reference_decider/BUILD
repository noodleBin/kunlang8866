load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path_reference_decider",
    srcs = ["path_reference_decider.cc"],
    hdrs = ["path_reference_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:path_boundary",
        "//modules/planning/common/path:discretized_path",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/tasks:task",
    ],
)

cpplint()
