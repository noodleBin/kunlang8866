load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PERCEPTION_COPTS")

package(default_visibility = ["//visibility:public"])


# cc_library(
#     name = "bev_spconv_lib",
#     srcs = select({
#       "@platforms//cpu:x86_64": ["spconv/lib/x86_64_cuda11.4/libbevspconv.so"],
#       "@platforms//cpu:aarch64": ["spconv/lib/aarch64_cuda11.4/libbevspconv.so"],
#       "//conditions:default": [],
#     }),
#     hdrs = glob(["spconv/include/*.hpp"]),
#     includes = ["spconv/include"],
#     visibility = ["//visibility:public"],
#     linkstatic = 0,
# )

cuda_library(
    name = "custom_layernorm",
    srcs = glob(["plugins/*.cu"]),
    hdrs = glob(["plugins/*.h"]),
    copts = PERCEPTION_COPTS,
    deps = [
      "@local_config_cuda//cuda:cudart",
      "@local_config_tensorrt//:tensorrt",
      "@eigen",
    ],
    alwayslink = 1,
)

cc_library(
    name = "bev_core_lib",
    # srcs = ["generate_plan.cpp"],
    deps = [
        "//modules/perception/lidar/lib/detector/dnn_common:spconv_lib",
        # ":custom_layernorm",
        "@eigen",
        "@local_config_cuda//cuda:cudart",
        "@local_config_pcl//:pcl",
        "@local_config_tensorrt//:tensorrt",
    ],
    linkstatic = True,
)

cc_library(
    name = "bev_head_types",
    hdrs = ["bevfusion/head-transbbox.hpp"],
    copts = PERCEPTION_COPTS,
    deps = [
        "//modules/perception/lidar/lib/detector/dnn_common/common",
    ],
)

cc_library(
    name = "bev_debug_utils",
    srcs = ["bev_debug_utils.cpp"],
    hdrs = ["bev_debug_utils.h"],
    copts = PERCEPTION_COPTS,
    deps = [
        ":bev_head_types",
        "//cyber/common",
        "//modules/perception/lidar/lib/detector/dnn_common/common",
        "@local_config_cuda//cuda:cudart",
    ],
    alwayslink = True,
)

cuda_library(
    name = "bev_fusion_cuda",
    srcs = glob(["bevfusion/*.cu"]),
    hdrs = glob(["bevfusion/*.h", "bevfusion/*.hpp"]),
    copts = PERCEPTION_COPTS,
    deps = [
      ":bev_debug_utils",
      ":custom_layernorm",
      "//modules/perception/common:cuda_math_compat",
      "@local_config_cuda//cuda:cudart",
      "@local_config_tensorrt//:tensorrt",
      "//modules/perception/lidar/lib/detector/dnn_common/common",
      "//modules/perception/lidar/lib/detector/dnn_common/proto:onnx-operators-ml_cc_proto",
      "//modules/perception/lidar/lib/detector/dnn_common/proto:centerpoint_detection_config_cc_proto",
      "//modules/perception/lidar/lib/detector/proto:object_subtype_mapping_config_cc_proto",
      "//modules/perception/lidar/lib/detector/dnn_common/proto:bev_fusion_detection_config_cc_proto",
      "//modules/perception/lidar/common",
      "//modules/perception/common:perception_gflags",
      ":bev_core_lib",
      "@com_github_jbeder_yaml_cpp//:yaml-cpp",
      "@local_config_pcl//:pcl",
      "@eigen",
    ],
)

cc_library(
    name = "bev_fusion_core",
    srcs = glob(["bevfusion/*.cpp"]),
    hdrs = glob(["bevfusion/*.h", "bevfusion/*.hpp"]),
    copts = PERCEPTION_COPTS,
    deps = [
      ":bev_fusion_cuda",
      ":bev_debug_utils",
      "//modules/perception/common:cuda_math_compat",
      "@local_config_cuda//cuda:cudart",
      "@local_config_tensorrt//:tensorrt",
      "//modules/perception/lidar/lib/detector/dnn_common/common",
      "//modules/perception/lidar/lib/detector/dnn_common/proto:onnx-operators-ml_cc_proto",
      "//modules/perception/lidar/lib/detector/dnn_common/proto:centerpoint_detection_config_cc_proto",
      "//modules/perception/lidar/lib/detector/proto:object_subtype_mapping_config_cc_proto",
      "//modules/perception/lidar/lib/detector/dnn_common/proto:bev_fusion_detection_config_cc_proto",
      "//modules/perception/lidar/common",
      "//modules/perception/common:perception_gflags",
      ":bev_core_lib",
      "@com_github_jbeder_yaml_cpp//:yaml-cpp",
      "@local_config_pcl//:pcl",
      "@eigen",
    ],
)



cc_library(
    name = "bevfusion_detector",
    srcs = ["bevfusion_detector.cpp"],
    hdrs = ["bevfusion_detector.h"],
    copts = PERCEPTION_COPTS,
    deps = [
        ":bev_fusion_core",
        ":bev_debug_utils",
        "//cyber/common",
        "//modules/perception/base",
        "//modules/perception/lib/thread",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface:base_lidar_detector",
        "//modules/perception/lib/config_manager",
        "//modules/perception/base:singleton",
        "//modules/perception/common/sensor_manager:camera_sensor_config",
    ],
    alwayslink = True,
)

cc_binary(
    name = "bevfusion_debug_runner",
    srcs = ["bevfusion_debug_runner.cpp"],
    copts = PERCEPTION_COPTS,
    deps = [
        ":bevfusion_detector",
        "//cyber/common",
        "//modules/perception/common/sensor_manager:camera_sensor_config",
    ],
)

cpplint()
