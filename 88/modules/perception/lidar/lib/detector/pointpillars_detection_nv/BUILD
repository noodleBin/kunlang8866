load("@rules_cc//cc:defs.bzl", "cc_library")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PERCEPTION_COPTS")

package(default_visibility = ["//visibility:public"])

cuda_library(
    name = "lidar-backbone",
    srcs = ["lidar-backbone.cu"],
    hdrs = ["lidar-backbone.hpp"],
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber/common",
        "//modules/perception/lidar/lib/detector/dnn_common/common",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cuda_library(
    name = "lidar-postprocess",
    srcs = ["lidar-postprocess.cu"],
    hdrs = ["lidar-postprocess.hpp"],
    deps = [
        "//modules/perception/common:cuda_math_compat",
        "//modules/perception/lidar/lib/detector/dnn_common/common",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cuda_library(
    name = "lidar-voxelization",
    srcs = ["lidar-voxelization.cu"],
    hdrs = ["lidar-voxelization.hpp"],
    deps = [
        "//modules/perception/lidar/lib/detector/dnn_common/common",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cuda_library(
    name = "pillarscatter-kernel",
    srcs = ["pillarscatter-kernel.cu"],
    hdrs = ["pillarscatter-kernel.hpp"],
    deps = [
        "//modules/perception/lidar/lib/detector/dnn_common/common",
        "@local_config_cuda//cuda:cudart",
    ],
)

cc_library(
    name = "pointpillar-scatter",
    srcs = [
        "pointpillar-scatter.cpp",
    ],
    hdrs = [
        "pointpillar-scatter.hpp",
    ],
    deps = [
        ":pillarscatter-kernel",
        "//modules/perception/lidar/lib/detector/dnn_common/common",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
    ],
    alwayslink = True,
)

cc_library(
    name = "pointpillar",
    srcs = [
        "pointpillar.cpp",
    ],
    hdrs = [
        "pointpillar.hpp",
    ],
    deps = [
        ":lidar-backbone",
        ":lidar-postprocess",
        ":lidar-voxelization",
        ":pointpillar-scatter",
        "//modules/perception/lidar/lib/detector/dnn_common/common",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
    ],
    alwayslink = True,
)

cc_library(
    name = "point_pillars_detector",
    srcs = [
        "point_pillars_detector.cpp",
    ],
    hdrs = [
        "lidar-backbone.hpp",
        "lidar-postprocess.hpp",
        "lidar-voxelization.hpp",
        "point_pillars_detector.h",
        "pointpillar.hpp",
    ],
    copts = PERCEPTION_COPTS,
    deps = [
        ":lidar-backbone",
        ":lidar-postprocess",
        ":lidar-voxelization",
        ":pointpillar",
        ":pointpillar-scatter",
        "//cyber/common",
        "//modules/perception/base",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/lib/thread",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/detector/dnn_common/common",
        "//modules/perception/lidar/lib/interface:base_lidar_detector",
        "//modules/perception/lidar/lib/detector/pointpillars_detection_nv/proto:pointpillars_detection_config_cc_proto",
        "//modules/perception/lib/config_manager",
        "//modules/perception/base:singleton",
        "@eigen",
        "@local_config_cuda//cuda:cudart",
        "@local_config_pcl//:pcl",
        "@local_config_tensorrt//:tensorrt",
    ],
    alwayslink = True,
)

cc_binary(
    name = "generate_plan",
    srcs = ["generate_plan.cpp"],
    deps = [
        ":lidar-backbone",
        ":lidar-postprocess",
        ":lidar-voxelization",
        ":pointpillar",
        ":pointpillar-scatter",
        "@eigen",
        "@local_config_cuda//cuda:cudart",
        "@local_config_pcl//:pcl",
        "@local_config_tensorrt//:tensorrt",
    ],
    linkstatic = True,
)

cpplint()
