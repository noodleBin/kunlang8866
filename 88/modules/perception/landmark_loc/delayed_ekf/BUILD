load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "delayed_measurement_ekf",
    srcs = [
        "async_delayed_measurement_ekf.cc",
        "delayed_measurement_ekf.cc",
    ],
    hdrs = [
        "async_delayed_measurement_ekf.h",
        "delayed_measurement_ekf.h",
    ],
    includes = ["."],
    include_prefix = "modules/perception/landmark_loc/delayed_ekf",
    strip_include_prefix = ".",
    visibility = ["//visibility:public"],
    deps = [
        "@eigen",
    ],
)

cc_test(
    name = "delayed_measurement_ekf_test",
    size = "small",
    srcs = ["delayed_measurement_ekf_test.cc"],
    deps = [
        ":delayed_measurement_ekf",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "async_delayed_measurement_ekf_test",
    size = "small",
    srcs = ["async_delayed_measurement_ekf_test.cc"],
    deps = [
        ":delayed_measurement_ekf",
        "@com_google_googletest//:gtest_main",
    ],
)
