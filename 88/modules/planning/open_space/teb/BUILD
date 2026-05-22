load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "teb_planner",
    srcs = ["optimal_planner.cc"],
    hdrs =
        [
            "optimal_planner.h",
            "planner_interface.h",
        ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    includes = [
        ".",
    ],
    deps = [
        "//cyber",
        "//cyber/common:log",
        "//modules/planning/open_space/teb/g2o_types",
        "//modules/planning/open_space/teb/utils:g2o_teb_types",
        "//modules/planning/open_space/teb/utils:g2o_utils",
        "//third_party/teb/g2o:g2o_header",
        "//third_party/teb/g2o:g2o_teb",
        "@boost",
        "@eigen",
    ],
)

cc_library(
    name = "teb_timed_elastic_band",
    srcs = ["timed_elastic_band.cc"],
    hdrs =
        ["timed_elastic_band.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    includes = [
        ".",
    ],
    deps = [
        "//cyber",
        "//cyber/common:log",
        "//modules/planning/open_space/teb/g2o_types",
        "//modules/planning/open_space/teb/utils:g2o_teb_types",
        "//modules/planning/open_space/teb/utils:g2o_utils",
        "//third_party/teb/g2o:g2o_header",
        "//third_party/teb/g2o:g2o_teb",
        "@boost",
        "@eigen",
    ],
)

cc_library(
    name = "teb_visualization",
    srcs = ["visualization.cc"],
    hdrs =
        ["visualization.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    includes = [
        ".",
    ],
    deps = [
        "//cyber",
        "//cyber/common:log",
        "//modules/planning/open_space/teb/g2o_types",
        "//modules/planning/open_space/teb/utils:g2o_teb_types",
        "//modules/planning/open_space/teb/utils:g2o_utils",
        "//third_party/teb/g2o:g2o_header",
        "//third_party/teb/g2o:g2o_teb",
        "@boost",
        "@eigen",
    ],
)

cpplint()
