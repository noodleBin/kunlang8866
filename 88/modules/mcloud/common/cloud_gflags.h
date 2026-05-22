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

#pragma once

#include "gflags/gflags.h"


DECLARE_string(cloud_server_ip);
DECLARE_int32(cloud_server_port);
DECLARE_string(cloud_mqtt_server_ip);
DECLARE_int32(cloud_mqtt_server_port);
DECLARE_string(cloud_mqtt_username);
DECLARE_string(cloud_mqtt_password);
DECLARE_int32(cloud_mqtt_sub_qos_default);
DECLARE_int32(cloud_mqtt_pub_qos_default);
DECLARE_string(cloud_mqtt_sub_qos_overrides);
DECLARE_string(cloud_mqtt_pub_qos_overrides);
DECLARE_string(cloud_server_conf);
DECLARE_string(vehicle_unique_id);
DECLARE_string(vehicle_id);
DECLARE_double(temporary_parking_release_delay_s);

DECLARE_double(offset_x);
DECLARE_double(offset_y);
DECLARE_string(zone_id);
DECLARE_string(cloud_module_name);
