load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MCLOUD_COPTS = [
    '-DMODULE_NAME=\\"mcloud\\"',
    '-std=c++17',
]

cc_library(
    name = "cloud",
    srcs = [
        "cloud.cc",
        "cloud_mqtt.cc",
    ],
    hdrs = [
        "cloud.h",
        "cloud_mqtt.h",
        "convert.h",
    ],
    copts = MCLOUD_COPTS,
    deps = [
        "//cyber",
        "@proj",
        "@com_google_absl//absl/strings",
        "@eigen",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "//modules/common/configs:config_gflags",
        "//modules/mcloud/common:cloud_gflags",
        "//modules/routing/proto:routing_cc_proto",
        "//modules/mcloud/network:network",
        "//modules/mcloud/mqtt:mqtt_client",
        "//modules/mcloud/parser:vehicle_message",
        "@com_github_gflags_gflags//:gflags",
        "//modules/common/adapters:adapter_gflags",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/mcloud/proto:mcloud_info_cc_proto",
        "//modules/mcloud/proto:road_event_code_config_cc_proto",
        "//modules/common/util",
        "//modules/common/util:message_util",
        "//modules/planning/proto:blocking_area_response_cc_proto",
        "@com_github_nlohmann_json//:json",
        "//modules/planning/proto:v2x_info_cc_proto",
        "//modules/planning/proto:lane_borrow_response_cc_proto",
        "//modules/planning/proto:pass_stacker_response_cc_proto",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/control/proto:control_cmd_cc_proto",
        "//modules/planning/proto:stackers_info_cc_proto",
        "//modules/planning/proto:barrier_cc_proto",
        "//modules/monitor/common:monitor_reader",
        "//modules/monitor/proto:system_status_cc_proto",
        "//modules/fas_aeb_backend/proto:fas_aeb_backend_cc_proto",
        "//modules/planning/proto:temporary_parking_request_cc_proto",
        "//modules/mcloud/proto:emergency_stop_cc_proto",
        "//modules/dreamview/proto:background_music_cc_proto",
        "//modules/dreamview/proto:barrier_command_cc_proto",
    ],
)

cpplint()
