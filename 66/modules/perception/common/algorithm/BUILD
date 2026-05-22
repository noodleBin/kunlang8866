load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_aarch64", "if_x86_64")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "century_perception_common_algorithm",
    srcs = [
        "geometry/camera_homography.cc",
        "geometry/roi_filter.cc",
        "graph/connected_component_analysis.cc",
        "graph/disjoint_set.cc",
        "graph/graph_segmentor.cc",
        "i_lib/pc/i_ground.cc",
        "image_processing/hough_transfer.cc",
        "io/io_util.cc",
        "sensor_manager/sensor_manager.cc",
    ],
    hdrs = [
        "geometry/basic.h",
        "geometry/camera_homography.h",
        "geometry/common.h",
        "geometry/convex_hull_2d.h",
        "geometry/roi_filter.h",
        "graph/conditional_clustering.h",
        "graph/connected_component_analysis.h",
        "graph/disjoint_set.h",
        "graph/gated_hungarian_bigraph_matcher.h",
        "graph/graph_segmentor.h",
        "graph/hungarian_optimizer.h",
        "graph/secure_matrix.h",
        "i_lib/algorithm/i_sort.h",
        "i_lib/core/i_alloc.h",
        "i_lib/core/i_basic.h",
        "i_lib/core/i_blas.h",
        "i_lib/core/i_constant.h",
        "i_lib/core/i_rand.h",
        "i_lib/da/i_ransac.h",
        "i_lib/geometry/i_line.h",
        "i_lib/geometry/i_plane.h",
        "i_lib/geometry/i_util.h",
        "i_lib/pc/i_ground.h",
        "i_lib/pc/i_struct_s.h",
        "i_lib/pc/i_util.h",
        "image_processing/hough_transfer.h",
        "io/io_util.h",
        "point_cloud_processing/common.h",
        "point_cloud_processing/downsampling.h",
        "sensor_manager/sensor_manager.h",
    ] + if_aarch64(["i_lib/pc/sse2neon.h",]),
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/base:base",
        "//modules/perception/common/proto:sensor_meta_schema_cc_proto",
        "@boost",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

cc_library(
    name = "data_structures_grid",
    hdrs = ["data_structures/grid.h"],
    deps = [
        "//cyber",
    ],
)

cc_test(
    name = "grid_test",
    srcs = ["data_structures/grid_test.cc"],
    size = "small",
    deps = [
      ":data_structures_grid",
      "@com_google_googletest//:gtest_main"],
)



# century_cc_test(
#     name = "io_util_test",
#     size = "small",
#     srcs = ["io/io_util_test.cc"],
#     copts = ["-fno-access-control"],
#     data = [
#         "//modules/perception/data:perception_testdata",
#     ],
#     deps = [
#         ":century_perception_common_algorithm",
#         "@com_google_googletest//:gtest_main",
#     ],
# )



cpplint()