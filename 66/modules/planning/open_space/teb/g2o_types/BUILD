load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "g2o_types",
    srcs = ["teb_obstacles.cc"],
    hdrs = [
        "base_teb_edges.h",
        "edge_acceleration.h",
        "edge_dynamic_obstacle.h",
        "edge_kinematics.h",
        "edge_obstacle.h",
        "edge_prefer_rotdir.h",
        "edge_shortest_path.h",
        "edge_time_optimal.h",
        "edge_velocity.h",
        "edge_via_point.h",
        "penalties.h",
        "teb_obstacles.h",
        "vertex_pose.h",
        "vertex_timediff.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber/common:log",
        "//modules/planning/open_space/teb/utils:g2o_distance_calculations",
        "//modules/planning/open_space/teb/utils:g2o_teb_types",
        "//third_party/teb/g2o:g2o_header",
        "//third_party/teb/g2o:g2o_teb",
        "@boost",
        "@eigen",
    ],
)

cpplint()
