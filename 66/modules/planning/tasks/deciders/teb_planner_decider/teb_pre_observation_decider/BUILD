load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "teb_tar_fsm_common",
    hdrs = ["teb_tar_fsm_common.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/proto:planning_cc_proto",
        "//cyber/time:clock",
    ],
)

cc_library(
    name = "tar_condition",
    hdrs = ["tar_condition.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:frame",
        "//modules/planning/tasks/deciders/teb_planner_decider/teb_pre_observation_decider:teb_tar_fsm_common",
    ],
)

cc_library(
    name = "tar_action",
    hdrs = ["tar_action.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:frame",
    ],
)

cc_library(
    name = "teb_tar_decider_fsm",
    srcs = ["teb_tar_decider_fsm.cc"],
    hdrs = ["teb_tar_decider_fsm.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/tasks/deciders/teb_planner_decider/teb_pre_observation_decider:tar_condition",
        "//modules/planning/tasks/deciders/teb_planner_decider/teb_pre_observation_decider:tar_action",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:frame",
        "//modules/planning/tasks/deciders/teb_planner_decider/teb_pre_observation_decider:teb_tar_fsm_common",
        "//modules/planning/common:dependency_injector",

    ],
    
)

cc_library(
    name = "teb_pre_observation_decider",
    srcs = ["teb_pre_observation_decider.cc"],
    hdrs = ["teb_pre_observation_decider.h"],
    copts = PLANNING_COPTS,
        linkopts = [
            "-lm",
            "-lgomp",
        ],
    deps = [
        "//modules/dreamview/backend/map:map_service",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/open_space/teb:teb_planner",
        "//modules/planning/open_space/teb:teb_timed_elastic_band",
        "//modules/planning/open_space/teb:teb_visualization",
        "//modules/planning/tasks/deciders:decider_base",
        "//third_party/teb/g2o:g2o_header",
        "//third_party/teb/g2o:g2o_teb",
        "//modules/planning/open_space/teb/g2o_types",
        "//modules/planning/open_space/teb/utils:g2o_teb_types",
        "//modules/planning/tasks/deciders/teb_planner_decider/teb_pre_observation_decider:teb_tar_decider_fsm"
    ],
    
)

cc_test(
    name = "teb_pre_observation_decider_test",
    size = "small",
    srcs = ["teb_pre_observation_decider_test.cc"],
    data = ["//modules/planning:planning_conf"],
    linkopts = ["-lgomp"],
    deps = [
        ":teb_pre_observation_decider",
        "@com_google_googletest//:gtest_main",
    ],
)
cpplint()
