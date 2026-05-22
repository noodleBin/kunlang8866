load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

COLLECTOR_COPTS = ["-DMODULE_NAME=\\\"message_collector\\\""]

cc_binary(
    name = "message_collector",
    srcs = ["main.cc"],
    copts = COLLECTOR_COPTS,
    deps = [
        "//cyber",
        "//cyber/time:clock",
        "//modules/perception/proto:perception_obstacle_cc_proto",
        "//modules/common/proto:header_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/localization/proto:pose_cc_proto",
        "//modules/localization/proto:localization_status_cc_proto",
        "//modules/prediction/common:message_process",
        "//modules/prediction/evaluator:evaluator_manager",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/common/virtual_prediction/proto:prediction_virtual_obstacles_cc_proto",
        "//modules/prediction/util:data_extraction",
        "@eigen",
    ],
)


cpplint()
