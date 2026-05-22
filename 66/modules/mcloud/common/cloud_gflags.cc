/******************************************************************************
 * Copyright 2017 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/mcloud/common/cloud_gflags.h"

DEFINE_string(cloud_server_ip,
              "192.168.1.168",
              "default cloud server ip");
DEFINE_int32(cloud_server_port, 50018,
             "default cloud server port");
DEFINE_int32(cloud_connect_timeout_ms, 3000,
             "max time in ms for establishing tcp connection");
DEFINE_int32(cloud_response_timeout_ms, 5000,
             "max time in ms for waiting tcp response data");
DEFINE_int32(cloud_report_interval_ms, 1000,
             "interval in ms for reporting system info to cloud");
DEFINE_int32(cloud_max_reconnect_attempts, 5,
             "max attempts for reconnecting to cloud server");
DEFINE_int32(cloud_reconnect_delay_ms, 2000,
             "delay in ms between cloud server reconnect attempts");
DEFINE_int32(cloud_buffer_block_max_size, 1024,
             "max size in bytes for one read/write buffer block");
DEFINE_int32(cloud_rw_block_size, 1024,
             "actual size in bytes for each read/write operation");
DEFINE_int32(cloud_circular_buffer_block_count, 8,
             "number of blocks used by the circular buffer");
DEFINE_string(cloud_mqtt_server_ip,
              "192.168.1.168",
              "default cloud mqtt server ip");
             
DEFINE_int32(cloud_mqtt_server_port, 50019,
             "default cloud mqtt server port");             
DEFINE_string(cloud_mqtt_username, "", "mqtt username");
DEFINE_string(cloud_mqtt_password, "", "mqtt password");
DEFINE_int32(cloud_mqtt_sub_qos_default, 1,
             "default mqtt subscribe qos, valid: 0/1/2");
DEFINE_int32(cloud_mqtt_pub_qos_default, 1,
             "default mqtt publish qos, valid: 0/1/2");
DEFINE_string(cloud_mqtt_sub_qos_overrides, "{}",
              "mqtt subscribe qos overrides in json map, topic->qos");
DEFINE_string(cloud_mqtt_pub_qos_overrides, "{}",
              "mqtt publish qos overrides in json map, topic->qos");

DEFINE_string(cloud_server_conf,
              "/century/modules/mcloud/conf/mcloud.conf",
              "conf file path");
DEFINE_string(road_event_code_config,
              "/century/modules/mcloud/conf/road_event_code.pb.txt",
              "road event cloud error code config file path");
DEFINE_string(vehicle_unique_id,
              "LK1100000PS000002",
              "vehicle unique id");

DEFINE_string(vehicle_id,
              "KL001",
              "vehicle id");

DEFINE_double(temporary_parking_release_delay_s, 10.0,
              "Delay seconds before sending need_stop=false after temporary parking");

DEFINE_double(offset_x, 250932.85, "The offset x from utm to map");
DEFINE_double(offset_y, 3987498.59, "The offset y from utm to map");
DEFINE_string(zone_id, "51S", "The utm zone id");
DEFINE_string(cloud_module_name, "mcloud", "The cloud module name");
