load("@rules_cc//cc:defs.bzl", "cc_library")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_gpu")
load("//tools:variables.bzl", "PERCEPTION_COPTS")

package(default_visibility = ["//visibility:public"])

cuda_library(
    name = "freespace_mask_cuda",
    srcs = ["freespace_mask_cuda.cu"],
    hdrs = ["freespace_mask_cuda.h"],
    deps = [
        "@local_config_cuda//cuda:cudart",
    ],
    tags = ["manual"],
)

cc_library(
    name = "freespace_mask",
    srcs = [
        "freespace_mask.cc",
    ] + if_gpu([], ["freespace_mask_cuda_stub.cc"]),
    hdrs = [
        "freespace_mask.h",
        "freespace_mask_cuda.h",
    ],
    copts = PERCEPTION_COPTS,
    deps = if_gpu([
        ":freespace_mask_cuda",
    ], []) + [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/perception/base",
        "//modules/perception/common/geometry:common",
        "//modules/perception/common/i_lib/core",
        "//modules/perception/lidar/common:lidar_frame",
        "//modules/perception/lidar/common:lidar_log",
        "//modules/perception/proto:perception_obstacle_cc_proto",
        "//modules/perception/proto:perception_obstacle_debug_cc_proto",
        "@eigen",
    ],
)

cpplint()
