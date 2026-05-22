load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "g2o_misc",
    hdrs = [
        "misc.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//third_party/teb/g2o:g2o_header",
        "//third_party/teb/g2o:g2o_teb",
        "@boost",
        "@eigen",
    ],
)

cc_library(
    name = "g2o_distance_calculations",
    hdrs = [
        "distance_calculations.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        ":g2o_misc",
        "//third_party/teb/g2o:g2o_header",
        "//third_party/teb/g2o:g2o_teb",
        "@boost",
        "@eigen",
    ],
)

cc_library(
    name = "g2o_teb_types",
    hdrs = [
        "teb_types.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//third_party/teb/g2o:g2o_header",
        "//third_party/teb/g2o:g2o_teb",
        "@boost",
        "@eigen",
    ],
)

cc_library(
    name = "g2o_pose_se2",
    hdrs = [
        "pose_se2.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//third_party/teb/g2o:g2o_header",
        "//third_party/teb/g2o:g2o_teb",
        "@boost",
        "@eigen",
    ],
)

cc_library(
    name = "g2o_utils",
    srcs = ["teb_config.cc"],
    hdrs = [
        "distance_calculations.h",
        "equivalence_relations.h",
        "misc.h",
        "pose_se2.h",
        "robot_footprint_model.h",
        "teb_config.h",
        "teb_types.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber/common:log",
        "//third_party/teb/g2o:g2o_header",
        "//third_party/teb/g2o:g2o_teb",
        "@boost",
        "@eigen",
    ],
)

cpplint()