load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MCLOUD_COPTS = ['-DMODULE_NAME=\\"mcloud\\"',"-Werror=reorder"]


cc_library(
    name = "network",
    hdrs = [
      "client.hpp",
      "data_handler.h"
    ],
    copts = MCLOUD_COPTS,
    deps = [
        "//cyber",
        "@proj",
        "@com_google_absl//absl/strings",
        "@eigen",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "//modules/mcloud/common:cloud_gflags",
        "//modules/routing/proto:routing_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()