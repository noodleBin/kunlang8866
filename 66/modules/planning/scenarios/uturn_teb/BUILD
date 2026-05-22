load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_gpu")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "uturn_teb_scenario",
    srcs = [
        "uturn_teb_scenario.cc",
        "stage_uturn.cc",
    ],
    hdrs = [
        "uturn_teb_scenario.h",
        "stage_uturn.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber/common:log",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/scenarios/util:scenario_util_lib",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_test(
    name = "uturn_teb_scenario_test",
    size = "small",
    srcs = ["uturn_teb_scenario_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":uturn_teb_scenario",
        "@com_google_googletest//:gtest_main",
    ] + if_gpu(["@local_config_cuda//cuda:cudart"]),
)

cc_test(
    name = "uturn_stages_test",
    size = "small",
    srcs = [
        "stage_uturn_test.cc",
    ],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":uturn_teb_scenario",
        "@com_google_googletest//:gtest_main",
    ] + if_gpu(["@local_config_cuda//cuda:cudart"]),
)

cpplint()
