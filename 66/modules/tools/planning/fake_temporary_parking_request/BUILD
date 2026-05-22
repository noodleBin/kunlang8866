load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libfake_temporary_parking_request_component.so",
    srcs = ["fake_temporary_parking_request.cc"],
    linkshared = True,
    linkstatic = False,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/planning/proto:temporary_parking_request_cc_proto",
    ],
)

cpplint()
