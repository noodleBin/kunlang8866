load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path_decider",
    srcs = ["path_decider.cc"],
    hdrs = ["path_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/tasks:task",
        "//modules/map/proto:map_electric_fence_cc_proto",
        "//modules/map/hdmap:hdmap_util",
    ],
)

cpplint()
