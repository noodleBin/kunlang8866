load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

FAS_AEB_BACKEND_COPTS = [
    '-DMODULE_NAME=\\"led_monitor\\"',
    '-std=c++17',
]

cc_binary(
    name = "lib_fas_aeb_backend.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":fas_aeb_backend_lib",
    ],
)



cc_library(
    name = "fas_aeb_backend_lib",
    srcs = ["fas_aeb_backend.cc"],
    hdrs = ["fas_aeb_backend.h" ],
    copts = FAS_AEB_BACKEND_COPTS,
    deps = [
        "//cyber",
        "@com_google_absl//absl/strings",
        "//modules/common/kv_db",
        "//modules/fas_aeb_backend/proto:fas_aeb_backend_cc_proto",
        "//modules/planning/proto:planning_aeb_cc_proto",
        "//modules/common/util",
        "//modules/common/util:message_util",
        "@com_github_nlohmann_json//:json",
    ],
)

# AEB Result Sender Demo
cc_binary(
    name = "aeb_result_sender",
    srcs = ["aeb_result_sender.cc"],
    copts = FAS_AEB_BACKEND_COPTS,
    deps = [
        "//cyber",
        "//modules/planning/proto:planning_aeb_cc_proto",
        "//modules/common/util:message_util",
    ],
)



# install(
#     name = "install",
#     data = [
#         ":runtime_data",
#     ],
#     targets = [
#         ":libled_monitor_component.so",
#     ],
#     deps = [
#         "//cyber:install",
#     ],
# )

filegroup(
    name = "runtime_data",
    srcs = glob([
       "dag/*.dag",
       "conf/*.json",
    ]),
)

install(
    name = "install",
    targets = [
        ":lib_fas_aeb_backend.so",
    ],
    data = [
        ":runtime_data",
    ],
    deps = [
        "//cyber:install",
    ],
)


cpplint()
