load("@rules_cc//cc:defs.bzl", "cc_library")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PERCEPTION_COPTS")

package(default_visibility = ["//visibility:public"])

# cc_library(
#     name = "spconv_lib",
#     srcs = select({
#       "@platforms//cpu:x86_64": ["spconv/lib/x86_64_cuda11.4/libspconv.so"],
#       "@platforms//cpu:aarch64": ["spconv/lib/aarch64_cuda11.4/libspconv.so"],
#       "//conditions:default": [],
#     }),
#     hdrs = glob(["spconv/*hpp"]),
#     includes = ["spconv"],
#     visibility = ["//visibility:public"],
#     linkstatic = 0,
# )

cuda_library(
    name = "center_core",
    srcs = glob(["src/*.cc", "src/*.cu", "src/*.cpp"]),
    hdrs = glob(["src/*.h", "src/*.hpp"]),
    copts = PERCEPTION_COPTS,
    deps = [
      "@local_config_cuda//cuda:cudart",
      "@local_config_tensorrt//:tensorrt",
      "//modules/perception/lidar/lib/detector/dnn_common/proto:onnx-operators-ml_cc_proto",
      "//modules/perception/lidar/lib/detector/dnn_common/proto:centerpoint_detection_config_cc_proto",
      "//modules/perception/lidar/lib/detector/dnn_common/common",
      "//modules/perception/lidar/common",
      "//modules/perception/common:perception_gflags",
      "//modules/perception/lidar/lib/detector/dnn_common:spconv_lib",
      "@com_github_jbeder_yaml_cpp//:yaml-cpp",
      "@local_config_pcl//:pcl",
      "@eigen",
    ],
)

# cc_binary(
#     name = "center_test",
#     srcs = ["test.cpp"],
#     deps = [
#         ":center-test",
#     ],
# )


cc_binary(
    name = "generate_head_plan",
    srcs = ["generate_head_plan.cpp"],
    deps = [
        "@eigen",
        "@local_config_cuda//cuda:cudart",
        "@local_config_pcl//:pcl",
        "@local_config_tensorrt//:tensorrt",
    ],
    linkstatic = True,
)

cc_library(
    name = "centerpoint_detector",
    srcs = ["centerpoint_detector.cpp"],
    hdrs = ["centerpoint_detector.h"],
    copts = PERCEPTION_COPTS,
    deps = [
      ":center_core",
      "//cyber/common",
      "//modules/perception/base",
      "//modules/perception/lib/thread",
      "//modules/perception/lidar/common",
      "//modules/perception/lidar/lib/interface:base_lidar_detector",
      "//modules/perception/lib/config_manager",
      "//modules/perception/base:singleton",
    ],
    alwayslink = True,
)
cpplint()
