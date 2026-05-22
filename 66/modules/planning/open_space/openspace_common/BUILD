load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "openspace_common",
    srcs = ["openspace_common.cc"],
    hdrs = ["openspace_common.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber/time:clock",
        "//cyber/common:log",
        "//modules/common/util:point_factory",
        "//modules/map/pnc_map:path",
        "//modules/map/hdmap:hdmap",
        "//cyber/proto:run_mode_conf_cc_proto",
        "//modules/planning/proto/openspace_config:openspace_common_cc_proto",
        "//modules/common/math",
        "//modules/common/util:normal_util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/tasks/utils:openspace_util",
    ],
)

cpplint()