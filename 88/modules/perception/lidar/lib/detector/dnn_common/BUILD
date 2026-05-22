load("@rules_cc//cc:defs.bzl", "cc_library")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PERCEPTION_COPTS")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "spconv_lib",
    srcs = select({
      "@platforms//cpu:x86_64": ["spconv/lib/x86_64_cuda11.4/libbevspconv.so"],
      "@platforms//cpu:aarch64": ["spconv/lib/aarch64_cuda11.4/libbevspconv.so"],
      "//conditions:default": [],
    }) + ["spconv/parser/onnx-parser.cpp", "spconv/parser/onnx-parser.hpp"],
    hdrs = glob(["spconv/include/*.hpp"]),
    includes = ["spconv/include"],
    deps = ["//modules/perception/lidar/lib/detector/dnn_common/proto:onnx-operators-ml_cc_proto",
            "//modules/perception/lidar/lib/detector/dnn_common/proto:onnx-ml_cc_proto"],
    visibility = ["//visibility:public"],
    linkstatic = 0,
)








cpplint()