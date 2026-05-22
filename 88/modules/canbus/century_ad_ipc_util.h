

/**
 * @file century_ad_ipc_util.h
 * @brief century_ad_ipc_util class
 * @Author: songlf
 * @Date Created: 10/10/2022
 */

#ifndef CENTURY_AD_IPC_UTIL_H_
#define CENTURY_AD_IPC_UTIL_H_

#include <iostream>
#include <string>

namespace century {
namespace ad {
namespace util {

// ip
#define century_AD_IPC_IP_IMU ("192.168.11.20")
#define century_AD_IPC_IP_VEHICLE_CAN_RX ("192.168.11.20")
#define century_AD_IPC_IP_VEHICLE_CAN_TX ("192.168.11.20")
#define century_AD_IPC_IP_DEMO_A ("192.168.1.138")
#define century_AD_IPC_IP_DEMO_B ("192.168.11.20")
#define century_AD_IPC_IP_DEMO_C ("192.168.11.20")
#define century_AD_IPC_IP_DEMO_D ("192.168.11.20")

// port
#define century_AD_IPC_PORT_IMU (23651)
#define century_AD_IPC_PORT_VEHICLE_CAN_RX (century_AD_IPC_PORT_IMU + 1)
#define century_AD_IPC_PORT_VEHICLE_CAN_TX (century_AD_IPC_PORT_VEHICLE_CAN_RX + 1)
#define century_AD_IPC_PORT_DEMO_A (century_AD_IPC_PORT_VEHICLE_CAN_TX + 1)
#define century_AD_IPC_PORT_DEMO_B (century_AD_IPC_PORT_DEMO_A + 1)
#define century_AD_IPC_PORT_DEMO_C (century_AD_IPC_PORT_DEMO_B + 1)
#define century_AD_IPC_PORT_DEMO_D (century_AD_IPC_PORT_DEMO_C + 1)

typedef void (*callback)(void *);

typedef enum _en_century_ad_ipc_type {
  en_ad_ipc_type_none,
  en_ad_ipc_type_imu,
  en_ad_ipc_type_imu_status,
  en_ad_ipc_type_vehicle_can_rx,
  en_ad_ipc_type_vehicle_can_rx_status,
  en_ad_ipc_type_vehicle_can_tx_status,
  en_ad_ipc_type_vehicle_can_command,
  // ...
  en_ad_ipc_type_max
} en_century_ad_ipc_type;

typedef enum _en_century_ad_ipc_error {
  en_ad_ipc_error_none,
  en_ad_ipc_error_ng,
  en_ad_ipc_error_ok,
  // ...
  en_ad_ipc_error_max
} en_century_ad_ipc_error;

typedef struct _st_ad_ipc_url {
  uint16_t port{};
  std::string ip{};
} st_ad_ipc_url;

typedef struct _st_ad_ipc_sub {
  callback func{nullptr};
  st_ad_ipc_url url{};
  en_century_ad_ipc_type type{};
} st_ad_ipc_sub;

}  // namespace util
}  // namespace ad
}  // namespace century

#endif  // century_AD_IPC_UTIL_H_
