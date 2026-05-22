load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "monitor_node",
    srcs = ["monitor_node.cc"],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
        "@com_github_nlohmann_json//:json",
    ],
)

cpplint()
