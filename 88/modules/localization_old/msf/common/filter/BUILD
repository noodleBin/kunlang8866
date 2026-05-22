load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

# cc_library(
#     name = "msf_filter",
#     srcs = glob([
#         "*.cc",
#     ]),
#     hdrs = glob([
#         "*.h",
#     ]),
#     copts = [
#         "-DMODULE_NAME=\\\"loc\\\"",
#       ],
#     deps = [
#         "@eigen",
#         "//cyber",
#         "//third_party/mmath",
#         "//modules/common/util:eigen_defs",
#     ],
# )

cc_library(
    name = "kalman_filter",
    srcs = glob([
        "*.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//cyber",
        "@eigen",
        "@com_github_google_glog//:glog",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "//third_party/mmath",
        "//third_party/manif",
    ],
    copts = [
        "-DMODULE_NAME=\\\"loc\\\"",
    ],
    # include_prefix = "modules/localization/common/",
    # includes = ["include/"],
    # strip_include_prefix = "include/",
)

cpplint()
