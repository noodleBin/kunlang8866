load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")
cc_library(
    name = "g2o",
    srcs = glob([
        "g2o/core/*.cpp",
        "g2o/types/*.cpp",
        "g2o/solvers/*.cpp",   # 使用 eigen 解算器
        "g2o/stuff/*.cpp",
    ]),
    hdrs = glob([
        "g2o/core/*.h",
        "g2o/types/*.h",
        "g2o/solvers/*.h",
        "g2o/stuff/*.h",
        "*.h",
    ]),
    includes = [
        ".",
        "g2o/core",
        "g2o/types",
        "g2o/solvers",
        "g2o/stuff",
    ],
    visibility = ["//visibility:public"],
    strip_include_prefix = ".",
    copts = [
        "-Wno-error=reorder",  # 允许 reorder 警告，不当错误
    ],
    deps = [
        "@eigen",  # 只需要 eigen，不再依赖 csparse
    ],
)

