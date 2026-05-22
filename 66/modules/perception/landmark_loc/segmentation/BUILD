load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")
config_setting(
    name = "is_orin",
    values = {"cpu": "aarch64"},
)

cc_import(
    name = "onnxruntime_core",
    shared_library = select({
        ":is_orin": "onnx/lib/orin/libonnxruntime.so.1.16.3",
        "//conditions:default": "onnx/lib/x86/libonnxruntime.so.1.16.3",
    }),
    visibility = ["//visibility:public"],
)


cc_library(
    name = "segmentation",
    srcs = [
        "src/image_segmentation.cpp",
    ],
    hdrs = [
        "src/image_segmentation.h",
        "src/yolo8_seg.h",
    ],
    includes = [
        "./include",
        "./onnx/include",
    ],
    visibility = ["//visibility:public"],
    linkopts = ["-lm", "-L/cenutry/modules/perception/landmark_loc/segmentation/onnx/lib/"],
    copts = ["-g"],
    deps = [
        "@com_github_google_glog//:glog",
        "@opencv//:highgui",
        "@opencv//:imgproc",
        "@opencv//:core",
        "@opencv//:imgcodecs",
        "@boost",
        ":onnxruntime_core",
    ],
)

cc_binary(
    name = "hfnet_onnx",
    srcs = [
      "hfnet_onnx.cc",
    ],
    includes = [
      ".",
      "segmentation/src/"
    ],
    copts = ["-g"],
    deps = [
      "//modules/perception/landmark_loc/segmentation:segmentation",
    ])





