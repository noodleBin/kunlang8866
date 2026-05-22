load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "aeb_obstacle_track",
    srcs = ["aeb_obstacle_track.cc"],
    hdrs = ["aeb_obstacle_track.h"],
    deps = [
        "//cyber/common:log",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common/math:math_utils",
        "//modules/common/util:normal_util",
        "//modules/common/math:geometry",
        "//modules/perception/proto:perception_obstacle_cc_proto",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:local_view",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/common/util:math_util_lib",
        "//modules/common/proto:pnc_point_cc_proto",
    ],
)
cpplint()