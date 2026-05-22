load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "teb_planner_decider",
    srcs = ["teb_planner_decider.cc"],
    hdrs = ["teb_planner_decider.h"],
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
        "//modules/planning/tasks/deciders/teb_planner_decider/teb_pre_observation_decider:teb_tar_fsm_common",
    ],
)

cc_library(
   name = "teb_fallback_decider",
   srcs = ["teb_fallback_decider.cc"],
   hdrs = ["teb_fallback_decider.h"],
   copts = PLANNING_COPTS,
   deps = [
       "//modules/planning/common:dependency_injector",
       "//modules/planning/common:planning_context",
       "//modules/planning/common:planning_gflags",
       "//modules/planning/tasks/deciders:decider_base",
       "//modules/planning/tasks/deciders/teb_planner_decider/teb_pre_observation_decider:teb_tar_fsm_common",
   ],
)

cc_test(
   name = "teb_fallback_decider_test",
   size = "small",
   srcs = ["teb_fallback_decider_test.cc"],
   deps = [
       ":teb_fallback_decider",
       "//modules/planning/common:dependency_injector",
       "@com_google_googletest//:gtest_main",
   ],
)

cpplint()
