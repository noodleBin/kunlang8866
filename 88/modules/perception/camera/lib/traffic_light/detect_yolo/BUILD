load("@rules_cc//cc:defs.bzl", "cc_library")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PERCEPTION_COPTS")

package(default_visibility = ["//visibility:public"])

cuda_library(
    name = "yolo_backend",
    srcs = glob([
        "core/*.cpp",
        "infer/*.cpp",
        "infer/*.cu",
        "utils/*.cpp",
        "model.cpp",
    ]),
    hdrs = glob([
        "core/*.hpp",
        "infer/*.hpp",
        "utils/*.hpp",
        "model.hpp",
        "option.hpp",
        "result.hpp",
        "core/*.h", 
    ]),
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
        "@opencv//:core",
        "@opencv//:imgcodecs",
        "@opencv//:highgui",
        "@opencv//:imgproc",
    ],
)

cc_library(
    name = "detection",
    srcs = ["yolo_traffic_light_detector.cpp"],
    hdrs = ["yolo_traffic_light_detector.h"],
    copts = PERCEPTION_COPTS,
    deps = [
        ":yolo_backend",
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/traffic_light/proto:detection_cc_proto",
    ],
    alwayslink = True, 
)

cc_library(
    name = "recognition",
    srcs = ["recognition.cc"],
    hdrs = ["recognition.h"],
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/lib/interface",
    ],
    alwayslink = True,
)

cpplint()