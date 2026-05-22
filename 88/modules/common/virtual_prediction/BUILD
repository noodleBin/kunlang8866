load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library",)
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PREDICTION_COPTS = ["-DMODULE_NAME=\\\"prediction\\\""]

cc_binary(
    name = "libprediction_virtual_obstacles_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":inner_prediction_virtual_obstacles_component"],
)

cc_library(
    name = "inner_prediction_virtual_obstacles_component",
    srcs = ["prediction_virtual_obstacles_component.cc"],
    hdrs = ["prediction_virtual_obstacles_component.h"],
    copts = PREDICTION_COPTS,
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
        "//modules/common/virtual_prediction/proto:prediction_virtual_obstacles_cc_proto",
        "//modules/prediction/submodules:predictor_submodule_lib",
        "//modules/prediction/util:data_extraction",
        "@eigen",
    ],
)


cpplint()
