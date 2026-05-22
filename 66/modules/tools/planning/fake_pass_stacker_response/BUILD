load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libfake_pass_stacker_response_component.so",
    srcs = ["fake_pass_stacker_response.cc"],
    linkshared = True,
    linkstatic = False,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/planning/proto:pass_stacker_response_cc_proto",
    ],
)

cpplint()
