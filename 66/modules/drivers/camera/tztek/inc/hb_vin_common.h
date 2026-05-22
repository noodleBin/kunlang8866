/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef INC_HB_VIN_COMMON_H_
#define INC_HB_VIN_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


#define JSON_FILE_CAMERA_PARAMETER_CHECK

#ifdef JSON_FILE_CAMERA_PARAMETER_CHECK
/* Check the parameters of the vin json file */
#define JSON_SENSOR_DEV_PORT_MIN -1
#define JSON_SENSOR_DEV_PORT_MAX CAM_MAX_NUM
#define JSON_SENSOR_ENTRY_NUM_MIN 0
#define JSON_SENSOR_ENTRY_NUM_MAX 64
#define JSON_SENSOR_SENSOR_MODE_MIN 0
#define JSON_SENSOR_SENSOR_MODE_MAX 5
#define JSON_SENSOR_FPS_MIN 1
#define JSON_SENSOR_FPS_MAX 240
#define JSON_SENSOR_RESOLUTION_MIN 32
/* Check the parameters of the mipi json file */
#define JSON_MIPI_BASE_LANE_MIN 1U
#define JSON_MIPI_BASE_LANE_MAX 4U
#define JSON_MIPI_BASE_DATATYPE_MIN 0x00U
#define JSON_MIPI_BASE_DATATYPE_MAX 0x13FU
#define JSON_MIPI_BASE_FPS_MIN 1U
#define JSON_MIPI_BASE_FPS_MAX 120U
#define JSON_MIPI_BASE_MIPICLK_MIN 1U
#define JSON_MIPI_BASE_MIPICLK_MAX 10000U
#define JSON_MIPI_BASE_SETTLE_MIN 0U
#define JSON_MIPI_BASE_SETTLE_MAX 127U
#define JSON_MIPI_HOST_HSATIME_MIN 0U
#define JSON_MIPI_HOST_HSATIME_MAX 4095U
#define JSON_MIPI_HOST_HBPTIME_MIN 0U
#define JSON_MIPI_HOST_HBPTIME_MAX 4095U
#define JSON_MIPI_HOST_HSDTIME_MIN 0U
#define JSON_MIPI_HOST_HSDTIME_MAX 4095U
#define JSON_MIPI_HOST_CHANNELNUM_MIN 0U
#define JSON_MIPI_HOST_CHANNELNUM_MAX 4U
#define JSON_MIPI_DEV_CHANNELNUM_MIN 0U
#define JSON_MIPI_DEV_CHANNELNUM_MAX 4U
#define JSON_MIPI_DEV_DATATYPE_MAX 0x3FU
#define JSON_MIPI_CONFIG_DEFAULT_MAX 0xFFFFU

#endif

#define HB_CAM_PARSE_BOARD_CFG_FAIL (200)
#define HB_CAM_PARSE_MIPI_CFG_FAIL	(201)
#define HB_CAM_DLOPEN_LIBRARY_FAIL	(202)
#define HB_CAM_INIT_FAIL			(203)
#define HB_CAM_DEINIT_FAIL			(204)
#define HB_CAM_START_FAIL			(205)
#define HB_CAM_STOP_FAIL			(206)
#define HB_CAM_I2C_WRITE_FAIL		(207)
#define HB_CAM_I2C_WRITE_BYTE_FAIL	(208)
#define HB_CAM_I2C_WRITE_BLOCK_FAIL	(209)
#define HB_CAM_I2C_READ_BLOCK_FAIL		(210)
#define HB_CAM_DYNAMIC_SWITCH_FAIL		(211)
#define HB_CAM_DYNAMIC_SWITCH_FPS_FAIL	(212)
#define HB_CAM_SERDES_POWERON_FAIL		(213)
#define HB_CAM_SERDES_CONFIG_FAIL		(214)
#define HB_CAM_SERDES_STREAM_ON_FAIL	(215)
#define HB_CAM_SERDES_STREAM_OFF_FAIL	(216)
#define HB_CAM_SENSOR_POWERON_FAIL		(217)
#define HB_CAM_SENSOR_POWEROFF_FAIL		(218)
#define HB_CAM_START_PHYSICAL_FAIL		(219)
#define HB_CAM_SPI_WRITE_BLOCK_FAIL		(220)
#define HB_CAM_SPI_READ_BLOCK_FAIL		(221)
#define HB_CAM_INVALID_PARAM		(222)
#define HB_CAM_SET_EX_GAIN_FAIL		(223)
#define HB_CAM_SET_AWB_FAIL			(224)
#define HB_CAM_I2C_READ_FAIL		(225)
#define HB_CAM_I2C_READ_BYTE_FAIL	(226)
#define HB_CAM_RESET_FAIL			(227)
#define HB_CAM_OPS_NOT_SUPPORT		(228)
#define HB_CAM_ISP_POWERON_FAIL		(229)
#define HB_CAM_ISP_POWEROFF_FAIL	(230)
#define HB_CAM_ISP_RESET_FAIL		(231)
#define HB_CAM_ENABLE_CLK_FAIL		(232)
#define HB_CAM_DISABLE_CLK_FAIL		(233)
#define HB_CAM_SET_CLK_FAIL			(234)
#define HB_CAM_DYNAMIC_SWITCH_MODE_FAIL	(235)
#define HB_CAM_INVALID_OPERATION		(236)
#define HB_CAM_IPI_RESET_FAIL			(237)
#define HB_CAM_SERDES_LIB_OPEN_FAIL		(238)
#define HB_CAM_SERDES_LIB_OPS_FAIL		(239)
#define HB_CAM_SERDES_INFO_FAIL		(240)
#define HB_CAM_EMB_INFO_FAIL		(241)

#define HB_VIN_SENSOR_LIB_OPEN_FAIL			(1000)
#define HB_VIN_SENSOR_LIB_OPS_FAIL			(1001)
#define HB_VIN_SENSOR_LIB_INIT_FAIL			(1002)
#define HB_VIN_SENSOR_LIB_START_FAIL		(1003)
#define HB_VIN_SENSOR_LIB_STOP_FAIL			(1004)
#define HB_VIN_SENSOR_LIB_DEINIT_FAIL		(1005)
#define HB_VIN_SENSOR_LIB_AE_INIT_FAIL		(1006)
#define HB_VIN_SENSOR_LIB_CTRL_FAIL			(1007)
#define HB_VIN_SENSOR_IOCTL_LOCK_FAIL		(1008)
#define HB_VIN_SENSOR_IOCTL_UNLOCK_FAIL		(1009)
#define HB_VIN_SENSOR_IOCTL_CNT_FAIL		(1010)
#define HB_VIN_SENSOR_IOCTL_AE_FAIL			(1011)

#ifdef JSON_FILE_CAMERA_PARAMETER_CHECK
#define HB_VIO_J5DEV_INVALID_CONFIG			(1012)
#endif

#define HB_VIN_LPWM_NODE_OPEN_FAIL			(1100)
#define HB_VIN_LPWM_PARAM_FAIL				(1101)
#define HB_VIN_LPWM_INIT_FAIL				(1102)
#define HB_VIN_LPWM_CONFIG_FAIL				(1103)
#define HB_VIN_LPWM_CONFIG_OFFSET_FAIL		(1104)
#define HB_VIN_LPWM_ENABLE_SINGLE_FAIL		(1105)
#define HB_VIN_LPWM_DISABLE_SINGLE_FAIL		(1106)
#define HB_VIN_LPWM_ENABLE_ALL_FAIL			(1107)
#define HB_VIN_LPWM_DISABLE_ALL_FAIL		(1108)
#define HB_VIN_LPWM_SET_MODE_FAIL			(1109)

#define HB_VIN_MIPI_HOST_START_FAIL		(2000)
#define HB_VIN_MIPI_HOST_STOP_FAIL		(2001)
#define HB_VIN_MIPI_HOST_INIT_FAIL		(2002)
#define HB_VIN_MIPI_HOST_PARSER_FAIL	(2003)
#define HB_VIN_MIPI_HOST_NOT_ENABLE		(2004)
#define HB_VIN_MIPI_HOST_SNRCLK_SET_EN_FAIL   	(2005)
#define HB_VIN_MIPI_HOST_SNRCLK_SET_FREQ_FAIL 	(2006)
#define HB_VIN_MIPI_HOST_PPE_INIT_REQUEST_FAIL  (2007)
#define HB_VIN_MIPI_HOST_PRE_START_REQUEST_FAIL (2008)
#define HB_VIN_MIPI_HOST_PRE_INIT_RESULT_FAIL   (2009)
#define HB_VIN_MIPI_HOST_PRE_START_RESULT_FAIL  (2010)
#define HB_VIN_MIPI_HOST_IPI_RESET_FAIL		(2011)
#define HB_VIN_MIPI_HOST_IPI_FATAL_FAIL		(2012)
#define HB_VIN_MIPI_HOST_IPI_GET_INFO_FAIL	(2013)
#define HB_VIN_MIPI_HOST_IPI_SET_INFO_FAIL	(2014)
#define HB_VIN_MIPI_HOST_PPE_UNSUPPORT_FAIL (2015)

#define HB_VIN_MIPI_DEV_START_FAIL		(3000)
#define HB_VIN_MIPI_DEV_STOP_FAIL		(3001)
#define HB_VIN_MIPI_DEV_INIT_FAIL		(3002)
#define HB_VIN_MIPI_DEV_PARSER_FAIL		(3003)
#define HB_VIN_MIPI_DEV_NOT_ENABLE		(3004)
#define HB_VIN_MIPI_DEV_IPI_FATAL_FAIL		(3005)
#define HB_VIN_MIPI_DEV_IPI_GET_INFO_FAIL	(3006)
#define HB_VIN_MIPI_DEV_IPI_SET_INFO_FAIL	(3007)

/* CIM ERR INFO */
#define HB_VIN_CIM_OPEN_DEV_FAIL       (4000)
#define HB_VIN_CIM_INIT_FAIL           (4001)
#define HB_VIN_CIM_DEINIT_FAIL         (4002)
#define HB_VIN_CIM_BYPASS_FAIL         (4003)
#define HB_VIN_CIM_STOP_FAIL           (4004)
#define HB_VIN_CIM_START_FAIL          (4005)
#define HB_VIN_CIM_PARSER_FAIL         (4006)
#define HB_VIN_CIM_EPOLL_CREATE_FAIL   (4007)
#define HB_VIN_CIM_EPOLL_CTL_FAIL      (4008)
#define HB_VIN_CIM_EPOLL_WAIT_FAIL     (4009)
#define HB_VIN_CIM_STOP_WORKING        (4010)
#define HB_VIN_CIM_BAD_VALUE           (4011)
#define HB_VIN_CIM_TIME_OUT            (4012)
#define HB_VIN_CIM_INVALID_OPERATION   (4013)
#define HB_VIN_CIM_INVALID_CONFIG      (4014)
#define HB_VIN_CIM_NULL_POINT          (4015)
#define HB_VIN_CIM_BUF_MGR_FAIL        (4016)
#define HB_VIN_CIM_BIND_GROUP_FAIL     (4017)
#define HB_VIN_CIM_EOF_FAIL            (4018)
#define HB_VIN_CIM_RESET_FAIL          (4019)
#define HB_VIN_CIM_REQBUF_FAIL         (4020)
#define HB_VIN_CIM_MMAP_FAIL           (4021)
#define HB_VIN_CIM_DYNAMIC_FPS_FAIL    (4022)
#define HB_VIN_CIM_EMB_NOT_WORKING     (4023)

#ifndef ENTRY_NUM
#define ENTRY_NUM 4u
#endif
#ifndef RET_OK
#define RET_OK 0
#endif
#ifndef RET_ERROR
#define RET_ERROR 1
#endif

#define SHIFT_16 16

#define HB_VIN_MIPI_HOST_PATH	"/dev/mipi_host"
#define HB_VIN_MIPI_DEV_PATH	"/dev/mipi_dev"

#define MIPIDEV_CHANNEL_NUM  (4)
#define MIPIDEV_CHANNEL_0    (0)
#define MIPIDEV_CHANNEL_1    (1)
#define MIPIDEV_CHANNEL_2    (2)
#define MIPIDEV_CHANNEL_3    (3)

typedef struct _mipi_dev_cfg_t {
	uint16_t  lane;
	uint16_t  datatype;
	uint16_t  fps;
	uint16_t  mclk;
	uint16_t  mipiclk;
	uint16_t  width;
	uint16_t  height;
	uint16_t  linelenth;
	uint16_t  framelenth;
	uint16_t  settle;
	uint16_t  vpg;
	uint16_t  ipi_lines;
	uint16_t  channel_num;
	uint16_t  channel_sel[MIPIDEV_CHANNEL_NUM];
} mipi_dev_cfg_t;

typedef struct _mipi_dev_ipi_info_t {
	uint16_t index;
	uint16_t fatal;
	uint16_t mode;
	uint16_t vc;
	uint16_t datatype;
	uint16_t maxfnum;
	uint32_t pixels;
	uint32_t lines;
} mipi_dev_ipi_info_t;

#define MIPIHOST_CHANNEL_NUM (4)
#define MIPIHOST_CHANNEL_0   (0)
#define MIPIHOST_CHANNEL_1   (1)
#define MIPIHOST_CHANNEL_2   (2)
#define MIPIHOST_CHANNEL_3   (3)

typedef struct _mipi_host_cfg_t {
	uint16_t  video_index;
	uint16_t  lane;
	uint16_t  datatype;
	uint16_t  fps;
	uint16_t  mclk;
	uint16_t  mipiclk;
	uint16_t  width;
	uint16_t  height;
	uint16_t  linelenth;
	uint16_t  framelenth;
	uint16_t  emb_data;
	uint16_t  settle;
	uint16_t  hsaTime;
	uint16_t  hbpTime;
	uint16_t  hsdTime;
	uint16_t  channel_num;
	uint16_t  channel_sel[MIPIHOST_CHANNEL_NUM];
} mipi_host_cfg_t;

typedef struct mipi_host_ipi_reset_s {
	uint16_t mask;
	uint16_t enable;
} mipi_host_ipi_reset_t;

#define MIPI_IPI_MASK_ALL	(0xffffu)

typedef struct mipi_host_ipi_info_s {
	uint8_t index;
	uint8_t vc;
	uint16_t fatal;
	uint16_t datatype;
	uint16_t hsa;
	uint16_t hbp;
	uint16_t hsd;
	uint32_t adv;
	uint32_t mode;
} mipi_host_ipi_info_t;

#define MIPI_DEV_PATH_LEN	(128)
#define MIPI_CFG_FILE_LEN	(256)
#define MIPI_PARAM_NAME_LEN	(32)
#define MIPI_PARAM_MAX		(10)
#define MIPI_SYSFS_PATH_PRE		"/sys/class/vps"
#define MIPI_SYSFS_DIR_PARAM	"param"
#define MIPI_SYSFS_DIR_CHAR_SEP	'/'
#define MIPI_SYSFS_DIR_CHAR_COM	'#'

typedef struct _mipi_param_t {
	char name[MIPI_PARAM_NAME_LEN];
	int32_t value;
} mipi_param_t;

typedef struct entry_s {
	mipi_host_cfg_t mipi_host_cfg;
	mipi_dev_cfg_t  mipi_dev_cfg;
	uint32_t entry_num;
	int32_t host_fd;
	int32_t dev_fd;
	int32_t host_enable;
	int32_t dev_enable;
	int32_t init_state;
	int32_t start_state;
	char host_path[MIPI_DEV_PATH_LEN];
	char dev_path[MIPI_DEV_PATH_LEN];
	mipi_param_t host_params[MIPI_PARAM_MAX];
	mipi_param_t dev_params[MIPI_PARAM_MAX];
}entry_t;

#ifndef NOSIF
#include <fcntl.h>
#include <sys/ioctl.h>

#define HB_VIN_SIF_PATH	"/dev/sif_capture"

#define SIF_IOC_MAGIC ((uint32_t)'x')
#define SIF_IOC_BYPASS _IOW(SIF_IOC_MAGIC, 7, int32_t)

#define HB_VIN_SIF_OPEN_DEV_FAIL	700
#define HB_VIN_SIF_BYPASS_FAIL		717

typedef struct sif_input_bypass {
	uint32_t enable_bypass;
	uint32_t enable_frame_id;
	uint32_t init_frame_id;
	uint32_t set_bypass_channels;
} sif_input_bypass_t;
#endif

enum {
	VIN_DEINIT,
	VIN_INIT,
	VIN_START,
	VIN_STOP
};

#ifdef __cplusplus
}
#endif

#endif  // INC_HB_VIN_COMMON_H_
