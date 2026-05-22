load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "virtual_chassis_writer",
    srcs = ["virtual_chassis_writer.cc"],
    deps = [
        "//cyber",
        "//modules/canbus/proto:chassis_cc_proto",
        "//cyber/service_discovery:topology_manager",
    ],
)

cpplint()
