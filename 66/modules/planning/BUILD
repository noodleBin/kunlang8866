load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:variables.bzl", "PLANNING_COPTS")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libplanning_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":planning_component_lib"],
)

cc_library(
    name = "planning_component_lib",
    srcs = ["planning_component.cc"],
    hdrs = ["planning_component.h"],
    copts = PLANNING_COPTS,
    deps = [
        # ":navi_planning",
        ":on_lane_planning",
        ":open_space_planning",
        "//modules/planning/aeb_planner:aeb_planner",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:normal_util",
        "//modules/common/util:message_util",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/map/relative_map/proto:navigation_cc_proto",
        "//modules/perception/proto:traffic_light_detection_cc_proto",
        "//modules/planning/common:history",
        "//modules/planning/common:danger_stay_away_util",
        "//modules/planning/common:message_process",
        "//modules/planning/common:count_appeared_obstacles",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/fas_aeb_backend/proto:fas_aeb_backend_cc_proto",
        "//modules/storytelling/proto:story_cc_proto",
        "//modules/mcloud/proto:super_traffic_light_cc_proto",
        "//cyber/service:client",
        "//modules/planning/proto:lane_borrow_response_cc_proto",
        "//modules/planning/proto:pass_stacker_response_cc_proto",
        "//modules/planning/proto:stackers_info_cc_proto",
        "//modules/planning/proto:v2x_info_cc_proto",
        "//modules/planning/proto:blocking_area_response_cc_proto",
        "//modules/planning/proto:planning_aeb_cc_proto",
        "//modules/planning/proto:barrier_cc_proto",
        "//modules/dreamview/proto:background_music_cc_proto",
    ],
)

cc_library(
    name = "planning_base",
    srcs = ["planning_base.cc"],
    hdrs = ["planning_base.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/math:quaternion",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/common/util:future",
        "//modules/common/util:message_util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/proto:traffic_light_detection_cc_proto",
        "//modules/planning/common:history",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:trajectory_stitcher",
        "//modules/planning/common/smoothers:smoother",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/planner",
        "//modules/planning/aeb_planner",
        "//modules/planning/planner:planner_dispatcher",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/proto:traffic_rule_config_cc_proto",
        "//modules/planning/tasks:task_factory",
        "//modules/planning/traffic_rules:traffic_decider",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/planning/open_space/openspace_common:openspace_common",
    ],
)

# cc_library(
#    name = "navi_planning",
#    srcs = ["navi_planning.cc"],
#    hdrs = ["navi_planning.h"],
#    copts = [
#        "-DMODULE_NAME=\\\"planning\\\"",
#        "-fopenmp",
#    ],
#    deps = [
#        ":planning_base",
#    ],
# )

cc_library(
    name = "on_lane_planning",
    srcs = ["on_lane_planning.cc"],
    hdrs = ["on_lane_planning.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        "//cyber",
        ":planning_base",
        "//cyber/common:log",
        "//cyber/threadlib:threadlib",
        "//modules/planning/common/historical_tracking_algorithms:hysteresis_interval",
        "//modules/planning/common/historical_tracking_algorithms:obstacle_stabilization_for_teb_speed",
        "//modules/planning/common/historical_tracking_algorithms:obstacle_speed_distance_history",
        "//modules/planning/common/historical_tracking_algorithms:obstacle_heading_history",
        "//modules/planning/common:danger_stay_away_util",
        "//modules/planning/common/historical_tracking_algorithms:start_up_vehicle_position_history",
        # "//modules/planning/learning_based/img_feature_renderer:birdview_img_feature_renderer",
        "//modules/monitor/proto:system_status_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/planning/scenarios/rescue_teb:rescue_teb_scenario",
    ],
)

cc_library(
    name = "open_space_planning",
    srcs = ["open_space_planning.cc"],
    hdrs = ["open_space_planning.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        "//cyber",
        ":planning_base",
        "//cyber/common:log",
        "//cyber/threadlib:threadlib",
        "//modules/planning/common/historical_tracking_algorithms:hysteresis_interval",
        "//modules/planning/common/historical_tracking_algorithms:obstacle_stabilization_for_teb_speed",
        "//modules/planning/common/historical_tracking_algorithms:obstacle_speed_distance_history",
        "//modules/planning/common/historical_tracking_algorithms:start_up_vehicle_position_history",
        # "//modules/planning/learning_based/img_feature_renderer:birdview_img_feature_renderer",
        "//modules/monitor/proto:system_status_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/planning/scenarios/rescue_teb:rescue_teb_scenario",
    ],
)

filegroup(
    name = "planning_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

filegroup(
    name = "planning_conf",
    srcs = glob([
        "conf/**",
        "conf/**/**",
        "learning_based/start_up_learning_tool/start_up.xml",
    ]),
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "dag/*.dag",
        "launch/*.launch",
    ]) + [":planning_conf"],
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libplanning_component.so",
    ],
    deps = [
        ":pb_planning",
        "//cyber:install",
        "//modules/planning/learning_based/start_up_learning_tool:install",
    ],
)

install_files(
    name = "pb_planning",
    dest = "modules/planning",
    files = [
        "//modules/planning/proto:auto_tuning_raw_feature_py_pb2",
        "//modules/planning/proto:decision_py_pb2",
        "//modules/planning/proto:ipopt_return_status_py_pb2",
        "//modules/planning/proto:lattice_structure_py_pb2",
        "//modules/planning/proto:learning_data_py_pb2",
        "//modules/planning/proto:open_space_task_config_py_pb2",
        "//modules/planning/proto:pad_msg_py_pb2",
        "//modules/planning/proto:planner_open_space_config_py_pb2",
        "//modules/planning/proto:planning_config_py_pb2",
        "//modules/planning/proto:planning_internal_py_pb2",
        "//modules/planning/proto:planning_py_pb2",
        "//modules/planning/proto:planning_semantic_map_config_py_pb2",
        "//modules/planning/proto:planning_stats_py_pb2",
        "//modules/planning/proto:planning_status_py_pb2",
        "//modules/planning/proto:reference_line_smoother_config_py_pb2",
        "//modules/planning/proto:sl_boundary_py_pb2",
        "//modules/planning/proto:st_drivable_boundary_py_pb2",
        "//modules/planning/proto:task_config_py_pb2",
        "//modules/planning/proto:traffic_rule_config_py_pb2",
        "//modules/planning/proto/math:cos_theta_smoother_config_py_pb2",
        "//modules/planning/proto/math:fem_pos_deviation_smoother_config_py_pb2",
        "//modules/planning/proto/math:qp_problem_py_pb2",
    ],
)

cpplint()
