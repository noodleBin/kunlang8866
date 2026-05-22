/*
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file century_ad_ipc_msg.h
 * @brief century_ad_ipc_msg class
 * @Author: songlf
 * @Date Created: 10/10/2022
 */

#ifndef CENTURY_AD_IPC_MSG_H_
#define CENTURY_AD_IPC_MSG_H_

#include <iostream>

namespace century {
namespace ad {
namespace msg {

typedef struct _st_linear_acceleration {
  float x;
  float y;
  float z;
} st_linear_acceleration;

typedef struct _st_angular_velocity {
  float x;
  float y;
  float z;
} st_angular_velocity;

typedef struct _st_orientation {
  float x;
  float y;
  float z;
  float w;
} st_orientation;

typedef struct _st_ad_ipc_msg_imu {
  st_orientation orientation;
  st_angular_velocity angular_velocity;
  st_linear_acceleration linear_acceleration;
  uint64_t ts;
} st_ad_ipc_msg_imu;

typedef struct _st_ad_ipc_msg_imu_status {
  uint8_t mode;
  uint32_t error;
  uint64_t ts;
} st_ad_ipc_msg_imu_status;

typedef struct _st_ad_ipc_msg_vehicle_can {
  // 0x18FF1082
  // 1 Bridge left wheel steering Angle
  float bridge_1_left_wheel_angle;
  // 2 Bridge left wheel steering Angle
  float bridge_2_left_wheel_angle;
  // 3 Bridge left wheel steering Angle
  float bridge_3_left_wheel_angle;
  // 4 Bridge left wheel steering Angle
  float bridge_4_left_wheel_angle;
  // 0x18FF1084
  // 1 Bridge right wheel steering Angle
  float bridge_1_right_wheel_angle;
  // 2 Bridge right wheel steering Angle
  float bridge_2_right_wheel_angle;
  // 3 Bridge right wheel steering Angle
  float bridge_3_right_wheel_angle;
  // 4 Bridge right wheel steering Angle
  float bridge_4_right_wheel_angle;
  // 0x1883EFF3
  // Charge times
  uint32_t b2v_st1_rechrg_cycels;
  // Main negative relay status
  uint8_t b2v_st1_main_neg_relay_st;
  // Dc charging relay 1 status
  uint8_t b2V_st1_DC_chrg_pos1_relay_st;
  // Dc negative charging relay 1 status
  uint8_t b2v_st1_DC_chrg_neg1_relay_st;
  // Dc charging relay 2 status
  uint8_t b2V_st1_DC_chrg_pos2_relay_st;
  // Dc negative charging relay 2 status
  uint8_t b2v_st1_DC_chrg_neg2_relay_st;
  // Collector bow positive relay status
  uint8_t b2v_st1_panto_chrg_pos_relay_st;
  // Pantograph negative relay status
  uint8_t b2v_st1_panto_chrg_neg_relay_st;
  // bms Indicates the current high voltage status
  uint8_t bms_hv_state;
  // Dc charging gun connection status
  uint8_t b2v_st1_DC_chrg_connect_st;
  // Charging state
  uint8_t b2v_st1_chrg_status;
  // Connection status of the pantograph charging gun
  uint8_t b2v_st1_panto_chrg_connect_st;
  // Ac charging gun connection status
  uint8_t b2v_st1_ac_chrg_connect_st;
  // 0x1884EFF3
  // The total charge of the battery pack is positive and the discharge point is negative
  float b2v_pack_current;
  // Voltage inside the battery pack
  float b2v_inside_high_voltage;
  // Battery pack soc
  float b2v_soc;
  // Battery pack soh
  float b2v_soh;
  // Trouble code
  int8_t st2_fault_code;
  // BMS life signal
  int8_t st2_life_couter;
  //The current highest fault level
  uint8_t b2v_fault_level;
  //High voltage request under BMS
  uint8_t b2v_require_hv_power_off;
  // 0x18FF8510
  // Vehicle fault code
  uint16_t whole_errcode;
  // MCU1 Fault code
  uint8_t mcu1_err_code;
  // MCU2 Fault code
  uint8_t mcu2_err_code;
  // PDU fault code
  uint8_t dcdc_err_code;
  // Input and output fault codes
  uint8_t io_can_err_code;
  // Tire pressure failure code
  uint8_t tpms_err_code;
  // Vehicle fault level
  uint8_t whole_err_level;
  // 0x18EFD014
  // 2 Bridge left wheel speed pulse input
  float wheel_speed_0;
  // 2 Bridge right wheel speed pulse input
  float wheel_speed_1;
  // 3 Bridge left wheel speed pulse input
  float wheel_speed_2;
  // 3 Bridge right wheel speed pulse input
  float wheel_speed_3;

  // 0x18FF1081
  // Remote control desired speed/service brake
  float remote_target_speed;
  // Remote target steering Angle
  float remote_target_angle;
  // Remote transmitter corresponding number
  uint8_t remote_trans_number;
  // Driving pattern
  uint8_t remote_turn_mode;
  // Mode switching (vacancy, remote control, navigation)
  uint8_t remote_state_change;
  // Fault reset
  uint8_t remote_5_byte_0_error_reset;
  // High voltage power-on and power-off
  uint8_t remote_hv_up;
  // Emergency stop
  uint8_t remote_estop;
  // Angle table zero
  uint8_t remote_angle_preset;

  uint8_t remote_remotenumber;
  uint8_t rmotCtrl_5_Byte_5_Gear;
  uint8_t rmotCtrl_5_Byte_1_Double_Flash;
  uint8_t rmotCtrl_5_Byte_Clearance_Light;
  uint8_t rmotCtrl_5_Byte_4_Horn;
  uint8_t rmotCtrl_6_Byte_4_Driving_Ligh;
  uint8_t rmotCtrl_6_Byte_5_Park_Light;
  uint8_t rmotCtrl_6_Byte_5_Left_Light;
  uint8_t rmotCtrl_6_Byte_5_Right_Light;

  // Heavy-duty signal
  int8_t remote_heavy_hight_state;

  // 0x18
  // mileage
  int64_t total_klimeters;

  // 0x18FF8010
  // Current gear status
  uint8_t vcu1_gear;
  // Loop check
  uint8_t vcu1_roling_coutnter;
  // Current steering mode
  uint8_t current_steering_mode;
  // Current working mode
  uint8_t current_working_mode;
  // Current walking mode
  uint8_t current_walking_mode;
  // Position mode in place feedback
  uint8_t position_mode_feedback;
  // Precharge state
  uint8_t precharge_state;
  // 4-in-1 pre-charge state
  uint8_t four_in_one_precharge_state;
  // DCDC contactor status
  uint8_t DCDC;
  // Oil pump contactor status
  uint8_t oil_pump_cont_actor;
  // High pressure condition
  uint8_t high_pressure_state;
  // Skid condition
  uint8_t dahua_state;
  // Remote control Can state
  uint8_t remote_can;
  // Navigation Can state
  uint8_t guider_can;
  // BMS Can status
  uint8_t bms_can;
  // Can state before driving the motor
  uint8_t motor_front_can;
  // an state after driving the electric C machine
  uint8_t motor_back_can;
  // Pump motor Can state
  uint8_t pump_motor_can;
  // Pre-charging Can status
  uint8_t precharge_can;
  // 4 in 1Can state
  uint8_t four_in_one_can;
  // The steering valve is in Can state
  uint8_t steer_can;
  // Tire pressure Can
  uint8_t tire_can;
  // 5G remote control Can status
  int8_t yk_5g_can_err;
  // 1 The bridge responds slowly
  uint8_t brige1_act_delay;
  // 2 The bridge responds slowly
  uint8_t brige2_act_delay;
  // 3 The bridge responds slowly
  uint8_t brige3_act_delay;
  // 4 The bridge responds slowly
  uint8_t brige4_act_delay;

  uint8_t yuankong_lock;
  uint8_t vcu1_f_low_beamlight;
  uint8_t vcu1_f_high_beamlight;
  uint8_t vcu1_r_low_beamlight;
  uint8_t vcu1_r_lhigh_beamlight;
  uint8_t vcu1_fog_ligh;
  uint8_t vcu1_gu_low_Powlloss;
  uint8_t vcu1_day_light;

  // Monitor HMICan status
  int8_t hmi_can_err;
  // ACCU communication status
  int8_t up_motor_run;

  // 0x18FF8110
  // Front drive wheel speed
  float front_drive_wheel_speed;
  // Rear drive wheel speed
  float back_drive_wheel_speed;
  // Front motor actual torque
  float front_motor_torque;
  // Actual torque after motor
  float back_motor_torque;

  // 0x19
  // Cumulative high pressure time
  uint32_t totol_time;
  // The vehicle weight
  uint8_t total_weight;
  // Total number of pump emptying times
  uint8_t xikong_leiji;
  // Suction times of grade pump station
  uint8_t xikong_dangci;

  // 0x18EFD011
  // Pump station main pressure
  float pump_station_main_pressure;
  // Y1 1 Proportional unloading valve
  uint8_t proportional_unloading_valve;
  // Y2 12 bridge brake proportional electromagnet
  uint8_t y2_12_brake_proportional_electromagnet;
  //Y3 34 bridge brake proportional electromagnet
  uint8_t y3_34_brake_proportional_electromagnet;
  // Brake oil block pressure
  float brake_oil_block_pressure;
  // Parking brake pressure
  float parking_brake_pressure;
  // Front drive service brake pressure
  float front_drive_service_brake_pressure;
  // Rear-drive service brake pressure
  float back_drive_service_brake_pressure;

  //0x18FF1280
  uint8_t yuankong_mode=0;

  //0xB
  uint8_t tire_pressure_11;
  uint8_t tire_pressure_14;
  uint8_t tire_pressure_21;
  uint8_t tire_pressure_24;
  uint8_t tire_pressure_31;
  uint8_t tire_pressure_34;
  uint8_t tire_pressure_41;
  uint8_t tire_pressure_44;
  uint64_t ts;
} st_ad_ipc_msg_vehicle_can;

typedef struct _st_ad_ipc_msg_vehicle_can_command {
  // Brake setting (0~250)
  uint8_t guide1_brake;
  // Target distance (cm, it is sent without change until it is in place, after it is in place, it needs to send 0 first and then send a new position)
  uint8_t guide1_target_distance;
  // Driving mode (0: speed mode 1: torque mode 2: position mode)
  uint8_t guide1_xingzou_mode;
  uint8_t guide1_zhuanxiang_mode;
  // Navigation manufacturer selection (1: Main line 2: West well currently used West well)
  uint8_t guide1_compny;
  // Target gear (0: P gear 1: R gear 2: N gear 3: D gear)
  uint8_t guide1_gear;
  // double_flash
  uint8_t guide1_double_flash;
  // left_light
  uint8_t guide1_left_light;
  // right_light
  uint8_t guide1_right_light;
  // horn
  uint8_t guide1_horn;
  // estop
  uint8_t guide1_estop;
  // driving_light
  uint8_t guide1_driving_light;
  // clearance_light
  uint8_t guide1_clearance_light;
  // Navigation standby mode
  uint8_t guide1_navigation_standby_mode;
  // High voltage power-on and power-off instructions
  uint8_t guide1_HVC;
  // Navigation fault reset
  uint8_t guide1_reset;

  uint8_t guide1_f_low_beam_light;
  uint8_t guide1_f_lhigh_beam_light;
  uint8_t guide1_r_low_beam_light;
  uint8_t guide1_r_high_beam_light;
  uint8_t guide1_fog_light;
  uint8_t guide1_unload_lamp;
  uint8_t guide1_loading_lamp;
  uint8_t guide1_loaded_lamp;

  // Navigation request Low power mode
  uint8_t guide1_lowpower_mode;
  // 2-axis target speed (m/s, torque Nm when torque mode is selected)
  float guide1_target_torque_2_axis;
  // 1 axis center equivalent target Angle (degree)
  float guide1_target_steering_angle_1axis;
  // 3-axis target speed (m/s, torque Nm when torque mode is selected)
  float guide1_target_torque_3_axis;
  // 4 axis center equivalent target Angle (degree)
  float guide1_target_steering_angle_4_axis;

	int8_t guide1_voice_num; /* scaling 1.0, offset 0.0, units none  */
	int8_t guide1_voice_volue; /* scaling 1.0, offset 0.0, units none  */

  uint8_t AEB_Enable;

  uint64_t ts;
} st_ad_ipc_msg_vehicle_can_command;

typedef struct _st_ad_ipc_msg_vehicle_can_status {
  uint64_t flag;
  uint64_t ts;
} st_ad_ipc_msg_vehicle_can_status;

typedef struct _st_ad_ipc_payload {
  uint8_t type;
  uint32_t size;
  // max(msg size)
  char data[sizeof(st_ad_ipc_msg_vehicle_can) + 56];
} st_ad_ipc_payload;

}  // namespace msg
}  // namespace ad
}  // namespace century

#endif  // century_AD_IPC_MSG_H_
