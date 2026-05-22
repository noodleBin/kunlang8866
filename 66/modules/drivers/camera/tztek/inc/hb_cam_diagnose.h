/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2022 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef INC_HB_CAM_DIAGNOSE_H_
#define INC_HB_CAM_DIAGNOSE_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "../utility/hb_queue.h"
#include "../utility/camera_diag/inc/camera_diag_utility.h"

#ifndef BIT
#define BIT(i)	(1 << (i))
#endif
#define MAX_USER	(3)
#define PIN_COUNT (2)  // errb, lock
#define MAX_NAME_LENGTH (128)
#define MON_PIN_NUM (SERDES_NUMBER*PIN_COUNT)

/* camera diag module id & event id */
typedef enum {
	SERDES_UNLOCK_MODULE			= 0xC010,
	SERDES_DEC_ERR_MODULE			= 0xC011,
	SERDES_IDLE_ERR_MODULE			= 0xC012,
	SERDES_MAX_RT_FLAG_MODULE		= 0xC013,
	SERDES_RT_CNT_FLAG_MODULE		= 0XC014,
	SERDES_VID_PXL_CRC_ERR_MODULE	= 0xC015,
	SERDES_LCRC_ERR_MODULE			= 0xC019,
	SERDES_MEM_ECC_ERR2_MODULE		= 0xC01A,
	SERDES_LFLT_INT_MODULE			= 0xC01B,
	SERDES_VDDBAD_INT_MODULE		= 0xC01C,
	SERDES_PORZ_INT_MODULE			= 0xC01D,
	SERDES_VDDCMP_INT_MODULE		= 0xC01E,
	SERDES_LMO_MODULE				= 0xC01F,
} serdes_module_id;

// 0x  0 	0/1 	0/1/2/3 	1
//       	|	  	|			|
//       	j5a/b 	rx0/1/2/3	linka/b/c/d
//
// note, solo ~ j5a

typedef enum {
	RX0_LINKA		= 0X0001,
	RX0_LINKB		= 0X0002,
	RX0_LINKC		= 0X0003,
	RX0_LINKD		= 0X0004,
	RX0_LINK_ALL	= 0X000f,
} rx0_serdes_event_id;

typedef enum {
	RX1_LINKA		= 0X0011,
	RX1_LINKB		= 0X0012,
	RX1_LINKC		= 0X0013,
	RX1_LINKD		= 0X0014,
	RX1_LINK_ALL	= 0X001f,
} rx1_serdes_event_id;

typedef enum {
	RX2_LINKA		= 0X0021,
	RX2_LINKB		= 0X0022,
	RX2_LINKC		= 0X0023,
	RX2_LINKD		= 0X0024,
	RX2_LINK_ALL	= 0X002f,
} rx2_serdes_event_id;

typedef enum {
	RX3_LINKA		= 0X0031,
	RX3_LINKB		= 0X0032,
	RX3_LINKC		= 0X0033,
	RX3_LINKD		= 0X0034,
	RX3_LINK_ALL	= 0X003f,
} rx3_serdes_event_id;

typedef enum {
    NO_USER		= 0,
	USER_NUM	= 1,
} lisener_id;

typedef struct ccd_pin_info_s {
	uint16_t pin_num;
	uint16_t deserial_no;
	char *pin_name;
} ccd_pin_info_t;

typedef struct mon_pin_info_s {
	int fd;
	char pin_name[MAX_NAME_LENGTH];
	int32_t value;
	uint16_t pin_num;
} mon_pin_info_t;

typedef struct camera_component_diagnose {
	pthread_t mon_pid;
	pthread_t poll_pid;
	mon_pin_info_t mon_pin[SERDES_NUMBER][PIN_COUNT];
	int mon_thread_status;
	int poll_thread_status;
	int epfd;
} camera_component_diagnose_t;

typedef struct fault_statistics_s
{
	uint8_t value;
	uint32_t cnt;
	struct timespec start;
	struct timespec end;
} fault_statistics_t;

typedef struct register_bit_info_s {
	uint8_t mark;
	uint8_t fault;
	uint8_t confirm_ret;
	uint8_t target_fail_val;
	uint16_t fault_clear_reg;
	uint16_t fault_clear_reg1;
	uint32_t module_id;
	uint32_t event_id;
	int64_t fault_clear_time;  // ms
	int64_t fault_confirm_time;  // ms
	fault_statistics_t fault_bit;
	fault_statistics_t better_bit;
} register_bit_info_t;

typedef int8_t (*CLEAR_FAULT_FUNCTION)(void *des, void *data, uint8_t reg_val);
typedef int8_t (*HANDLE_FAULT_FUNCTION)(void *data, uint8_t reg_val);
typedef int8_t (*REPORT_FAULT_FUNCTION)(
										void *data,
										uint8_t reg_val,
										uint8_t valid_num,
										int8_t (*diag_api)(uint16_t module_id, uint16_t event_id, uint8_t ret));
typedef struct register_general_fun_s {
	uint16_t reg;
	uint8_t val;
	HANDLE_FAULT_FUNCTION handle_fault_fun;
	REPORT_FAULT_FUNCTION report_fault_fun;
	CLEAR_FAULT_FUNCTION clear_fault_fun;
	register_bit_info_t *reg_bit_info;
} register_general_fun_t;
#ifdef __cplusplus
}
#endif

#endif  // INC_HB_CAM_DIAGNOSE_H_
