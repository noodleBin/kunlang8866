load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "test_duplicated_object_filter",
    srcs = ["test_duplicated_filter.cc",
            "duplicated_object_filter.cc",
            "duplicated_object_filter.h",
          ],
    deps = [
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//cyber:cyber",
        "@eigen",
    ]
)

cc_library(
    name = "lidar_tracking_io",
    srcs = ["duplicated_object_filter.cc",
            "duplicated_object_filter.h",
            "lidar_frame_generator.cc",
            "lidar_frame_generator.h",
            "lidar_predictor_free_move.cc",
            "lidar_predictor_free_move.h",
            "prediction_frame_generator.cc",
            "prediction_frame_generator.h",
            "geojson_convert.cc",
            "geojson_convert.h",
            "dump_perception.h",
            "bbx_build.cc",
            "bbx_build.h",
            "velocity_filter.h"
          ],
    hdrs = ["duplicated_object_filter.h",
            "lidar_frame_generator.h",
            "lidar_predictor_free_move.h",
            "prediction_frame_generator.h",
            "geojson_convert.h",
            "dump_perception.h",
            "bbx_build.h",
            "velocity_filter.h"
          ],
    deps = [
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//cyber:cyber",
        "@eigen",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest_main",
        "//modules/prediction/predictor/free_move:free_move_predictor",
        "//modules/perception/onboard/msg_serializer:msg_serializer",
        "//modules/perception/onboard/inner_component_messages:inner_component_messages",
        "//modules/perception/onboard/component:lidar_inner_component_messages",
        "//modules/perception/lidar/common:lidar_frame",
        "@com_github_nlohmann_json//:json",
    ],
    includes = ["."],
    visibility = ["//visibility:public"],
)