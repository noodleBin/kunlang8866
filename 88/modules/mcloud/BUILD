load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MCLOUD_COPTS = [
    '-DMODULE_NAME=\\"mcloud\\"',
    '-std=c++11',
]

cc_binary(
    name = "libmcloud.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":mcloud_lib",
    ],
)

# cc_binary(
#     name = "virutal_routing",
#     srcs = ["virutal_routing.cpp"],
#     deps = [
#         ":mcloud_lib",
#         "@com_github_nlohmann_json//:json",
#     ],
# )

cc_library(
    name = "mcloud_lib",
    srcs = glob([
        "*.cc", 
    ]),
    hdrs = glob([
        "*.h", 
    ]),
    copts = MCLOUD_COPTS,
    visibility = ["//visibility:private"],
    deps = [
        "//cyber",
        "@proj",
        "@com_google_absl//absl/strings",
        "@eigen",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "//modules/mcloud/common:cloud_gflags",
        "//modules/mcloud/cloud:cloud",
        "//modules/planning/proto:barrier_cc_proto",
        "//modules/routing/proto:routing_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)


filegroup(
    name = "runtime_data",
    srcs = glob([
       "dag/*.dag",
       "launch/*.launch",
       "conf/*.conf",
    ]),
)

install(
    name = "install",
    targets = [
        ":libmcloud.so",
    ],
    data = [
        ":runtime_data",
    ],
    deps = [
        "//cyber:install",
    ],
)

cpplint()
