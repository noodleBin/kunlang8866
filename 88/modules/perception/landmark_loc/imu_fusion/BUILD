load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "delayed_measurement_eskf",
    srcs = [
        "async_delayed_measurement_eskf.cc",
        "delayed_measurement_eskf.cc",
    ],
    hdrs = [
        "async_delayed_measurement_eskf.h",
        "delayed_measurement_eskf.h",
    ],
    includes = ["."],
    include_prefix = "modules/perception/landmark_loc/imu_fusion",
    strip_include_prefix = ".",
    visibility = ["//visibility:public"],
    deps = [
        "//cyber/common:log",
        "@eigen",
        "@com_github_google_glog//:glog",
    ],
)

