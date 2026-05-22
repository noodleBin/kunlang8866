load("@rules_cc//cc:defs.bzl", "cc_library")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cuda_library(
    name = "common",
    deps = [
        ":check",
        ":dtype",
        ":launch",
        ":tensor",
        ":tensorrt",
        ":timer",
    ],
)

cc_library(
    name = "check",
    hdrs = ["check.hpp"],
    deps = [
        "@local_config_cuda//cuda:cudart",
    ],
)

cc_library(
    name = "dtype",
    hdrs = ["dtype.hpp"],
)

cuda_library(
    name = "launch",
    hdrs = ["launch.cuh"],
    deps = [
        ":check",
        "@local_config_cuda//cuda:cudart",
    ],
)

cuda_library(
    name = "tensor",
    hdrs = ["tensor.hpp"],
    srcs = ["tensor.cu"],
    deps = [
        ":launch",
        "@local_config_cuda//cuda:cudart",
    ],
)

cc_library(
    name = "timer",
    hdrs = ["timer.hpp"],
    deps = [
        ":check",
        "//cyber/common",
    ],
)

cc_library(
    name = "tensorrt",
    srcs = ["tensorrt.cpp"],
    hdrs = ["tensorrt.hpp"],
    deps = [
        ":check",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
        "//cyber/common",

    ],
)

cc_binary(
    name = "trt_builder",
    srcs = ["trt_builder.cpp"],
    deps = [
        "@eigen",
        "@local_config_cuda//cuda:cudart",
        "@local_config_pcl//:pcl",
        "@local_config_tensorrt//:tensorrt",
    ],
    linkstatic = True,
    linkopts = ["-ldl"],
)


# cc_binary(
#     name = "trt_builder",
#     srcs = glob(["common/*.cpp", "trtexec/*.cpp", "common/*.h"]),
#     includes = ["common"],
#     deps = [
#         "@eigen",
#         "@local_config_cuda//cuda:cudart",
#         "@local_config_pcl//:pcl",
#         "@local_config_tensorrt//:tensorrt",
#     ],
#     linkstatic = True,
#     linkopts = ["-lm"], 
# )


cpplint()
