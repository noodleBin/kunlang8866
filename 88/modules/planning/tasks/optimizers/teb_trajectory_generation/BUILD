load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_gpu")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])
FOPENMP_COPTS = ["-fopenmp"]

cc_library(
    name = "teb_trajectory_provider",
    srcs = ["teb_trajectory_provider.cc"],
    hdrs = ["teb_trajectory_provider.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        ":teb_trajectory_optimizer",
        "//modules/common/status",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:trajectory_stitcher",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks/optimizers:trajectory_optimizer",
        "//modules/planning/open_space/openspace_common:openspace_common",
       ] + if_gpu(["@local_config_cuda//cuda:cudart"]),
)

cc_library(
    name = "teb_trajectory_optimizer",
    srcs = ["teb_trajectory_optimizer.cc"],
    hdrs = ["teb_trajectory_optimizer.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:frame",
        "//modules/planning/open_space/openspace_common:openspace_common",
        "//modules/planning/open_space/coarse_trajectory_generator:hybrid_a_star",
        "//modules/planning/open_space/trajectory_smoother:distance_approach_problem",
        "//modules/planning/open_space/trajectory_smoother:dual_variable_warm_start_problem",
        "//modules/planning/open_space/trajectory_smoother:iterative_anchoring_smoother",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/open_space/teb:teb_planner",
        "//modules/planning/open_space/teb:teb_timed_elastic_band",
        "//modules/planning/open_space/teb:teb_visualization",
        "//third_party/teb/g2o:g2o_header",
        "//third_party/teb/g2o:g2o_teb",
        "//modules/planning/open_space/teb/g2o_types",
        "//modules/planning/open_space/teb/utils:g2o_teb_types",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

cc_test(
    name = "teb_trajectory_provider_test",
    size = "small",
    srcs = ["teb_trajectory_provider_test.cc"],
    copts = FOPENMP_COPTS,
    linkopts = ["-lgomp"],
    deps = [
        ":teb_trajectory_provider",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "teb_trajectory_optimizer_test",
    size = "small",
    srcs = ["teb_trajectory_optimizer_test.cc"],
    copts = FOPENMP_COPTS,
    linkopts = ["-lgomp"],
    deps = [
        ":teb_trajectory_optimizer",
        "@com_google_googletest//:gtest_main",
    ] + if_gpu(["@local_config_cuda//cuda:cudart"]),
)

cpplint()
