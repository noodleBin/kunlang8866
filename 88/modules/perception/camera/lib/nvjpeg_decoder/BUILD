load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools/platform:build_defs.bzl", "if_x86_64")

package(default_visibility = ["//visibility:public"])


config_setting(
    name = "is_x86_64",
    constraint_values = [
        "@platforms//cpu:x86_64",
    ],
)

config_setting(
    name = "is_aarch64",
    constraint_values = [
        "@platforms//cpu:aarch64",
    ],
)


cuda_library(
    name = "fisheye_cuda_lib",
    srcs = [
        "fisheye_cuda.cu",
    ],
    hdrs = [
        "fisheye_cuda.h",
    ],
    deps = [
        "@local_config_cuda//cuda:cudart",
    ]
)

# TODO
# (If JPEG decompression is needed on Orin, this function is required.)
# cc_library(
#     name = "nvjpeg_decoder_lib",
#     srcs = select({
#         ":is_aarch64": ["nvjpeg_decoder_jeston.cc"],
#         ":is_x86_64": ["nvjpeg_decoder.cc"],
#         "//conditions:default": [],
#     }),
#     hdrs = select({
#         ":is_aarch64": ["nvjpeg_decoder_jeston.h"],
#         ":is_x86_64": ["nvjpeg_decoder.h"],
#         "//conditions:default": [],
#     }),
#     # Hide libjpeg symbols to prevent conflicts with OpenCV
#     linkopts = select({
#         "@platforms//cpu:aarch64": [
#             "-Wl,--exclude-libs,libjpeg.a",
#             "-Wl,--exclude-libs,libjpeg.so",
#             "-Wl,--exclude-libs,libjpeg.so.8",
#         ],
#         "//conditions:default": [],
#     }),
#     deps = [
#             ":fisheye_cuda_lib",
#             "//cyber/common",
#         ] + select({
#             ":is_x86_64": ["@libnvjpeg"],
#             ":is_aarch64": ["@libjestonjpeg"],
#             "//conditions:default": [],
#         }),
# )

cc_library(
    name = "nvjpeg_decoder_lib",
    srcs = select({
        ":is_aarch64": ["nvjpeg_opencv.cc"],
        ":is_x86_64": ["nvjpeg_decoder.cc"],
        "//conditions:default": [],
    }),
    hdrs = select({
        ":is_aarch64": ["nvjpeg_opencv.h"],
        ":is_x86_64": ["nvjpeg_decoder.h"],
        "//conditions:default": [],
    }),
    deps = [
            ":fisheye_cuda_lib",
            "//cyber/common",
        ] + select({
            ":is_x86_64": ["@libnvjpeg"],
            ":is_aarch64": [ "@opencv//:core",
                            "@opencv//:imgcodecs",
                            "@opencv//:imgproc"],
            "//conditions:default": [],
        }),
)


cpplint()
