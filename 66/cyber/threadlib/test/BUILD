load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "thread_name_compile_test",
    srcs = ["thread_name_compile_test.cc"],
    deps = [
        "//cyber/threadlib:threadlib",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "thread_function_test",
    srcs = ["thread_function_test.cc"],
    deps = [
        "//cyber/threadlib:threadlib",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
