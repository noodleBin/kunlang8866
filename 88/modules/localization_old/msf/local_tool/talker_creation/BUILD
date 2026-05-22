load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "msf_reset_talker",
    srcs = ["msf_reset_talker.cc"],
    deps = [
        "//cyber",
        "//cyber/time:clock",
        "//cyber/time:rate",
        "//cyber/timer:timer",
        "//modules/localization/proto:msf_reset_cc_proto",
    ],
)

install(
    name = "install",
    targets = [
        ":msf_reset_talker",
    ],
    runtime_dest = "modules/localization/msf/local_tool/talker_creation",
)