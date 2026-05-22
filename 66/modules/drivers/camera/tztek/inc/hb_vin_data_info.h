/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2020 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef __HB_VIN_DATA_INFO_H__
#define __HB_VIN_DATA_INFO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <sys/time.h>

typedef enum CAM_DATA_TYPE_S {
	HB_CAM_RAW_DATA = 0,
	HB_CAM_YUV_DATA = 1,
	HB_CAM_EMB_DATA = 2,
	HB_CAM_EMB_INFO = 3,
} CAM_DATA_TYPE_E;

typedef struct AWB_DATA {
	uint16_t WBG_R;
	uint16_t WBG_GR;
	uint16_t WBG_GB;
	uint16_t WBG_B;
}AWB_DATA_s;

typedef struct img_addr_info_s {
	uint16_t width;
	uint16_t height;
	uint16_t stride;
	uint64_t y_paddr;
	uint64_t c_paddr;
	uint64_t y_vaddr;
	uint64_t c_vaddr;
} img_addr_info_t;

typedef struct cam_img_info_s {
	int32_t g_id;
	int32_t slot_id;
	int32_t cam_id;
	int32_t frame_id;
	int64_t timestamp;
	img_addr_info_t img_addr;
} cam_img_info_t;

typedef struct {
	uint32_t frame_length;
	uint32_t line_length;
	uint32_t width;
	uint32_t height;
	float    fps;
	uint32_t pclk;
	uint32_t exp_num;
	uint32_t lines_per_second;
	char     version[10];
} sensor_parameter_t;

typedef struct {
	uint8_t major_version;
	uint8_t minor_version;
	uint16_t vendor_id;
	uint16_t module_id;
	uint32_t module_serial;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t cam_type;
	uint8_t module_falg;
	uint8_t efl_flag;
	uint8_t cod_flag;
	uint8_t pp_flag;
	uint8_t distortion_flag;
	uint16_t image_height;
	uint16_t image_width;
	uint32_t crc32_1;
	uint32_t crc_group1;
	uint8_t distort_params;
	uint8_t distort_model_type;
	uint8_t serial_num[40];	//camera serial number
	double pp_x;
	double pp_y;
	double cam_skew; 	
	double focal_u;
	double focal_v;
	double center_u;
	double center_v;
	double hfov;
	double k1;
	double k2;
	double p1;
	double p2;
	double k3;
	double k4;
	double k5;
	double k6;
	double k7;
	double k8;
	double k9;
	double k10;
	double k11;
	double k12;
	double k13;
	double k14;
	double focal_u_2;
	double focal_v_2;
	double center_u_2;
	double center_v_2;
	double k1_2;
	double k2_2;
	double k3_2;
	double k4_2;
} sensor_intrinsic_parameter_t;

typedef struct {
	sensor_parameter_t sns_param;
	sensor_intrinsic_parameter_t sns_intrinsic_param;
} cam_parameter_t;

#define CIM_FS		0u
#define CIM_FE		1u
#define ISP_FS		2u
#define ISP_FE		3u
#define PYM_FS		4u
#define PYM_FE		5u
#define GDC_FS		6u
#define GDC_FE		7u
#define STC_FS		8u
#define STC_FE		9u
#define CIM_QB		10u
#define CIM_DQB		11u
#define PYM_QB		12u
#define PYM_DQB		13u
#define ISP_QB		14u
#define ISP_DQB		15u
#define STAT_NUM	16u

#define MAX_DELAY_FRAMES 100u

struct statinfo {
	uint32_t framid;
	uint64_t g_tv_sec;
	uint64_t g_tv_usec;
};

struct vio_statinfo {
	struct statinfo stat[MAX_DELAY_FRAMES][STAT_NUM];
};

#define MAX_COMBINE_FRAME		(4)
#define MAX_TEMPERATURE_NUM		(2)

typedef struct embed_data_cust_info_s {
	const char *name;
	uint32_t size;
	void *info;
} embed_data_cust_info_t;

typedef struct embed_data_info_s {
	uint32_t port;
	uint32_t dev_port;
	uint32_t frame_count;
	uint32_t line[MAX_COMBINE_FRAME];
#if 0
	uint32_t again[MAX_COMBINE_FRAME];
	uint32_t dgain[MAX_COMBINE_FRAME];
	uint32_t rgain[MAX_COMBINE_FRAME];
	uint32_t bgain[MAX_COMBINE_FRAME];
	uint32_t grgain[MAX_COMBINE_FRAME];
	uint32_t gbgain[MAX_COMBINE_FRAME];
	uint32_t vts;
	uint32_t hts;
#endif
	uint32_t height;
	uint32_t width;
	uint32_t exposure;
	int32_t  temperature;
	uint32_t check_state;

	uint32_t num_exp;
	uint32_t exposures[MAX_COMBINE_FRAME];
	uint32_t num_temp;
	int32_t  temperatures[MAX_TEMPERATURE_NUM];

	uint32_t frame_id;
	uint64_t time_stamp;//HW time stamp
	struct timeval tv;//system time of hal get buf

	int32_t  reserved[16];
	struct embed_data_cust_info_s cust;
} embed_data_info_t;

typedef enum CAM_EVENT_TYPE_S {
	HB_CAM_EVENT_DIAG = 0,
} CAM_EVENT_TYPE_E;

typedef struct hb_cam_event_s {
    uint32_t port;
    uint32_t status;
    uint32_t event_id;
    uint32_t module_id;
    uint32_t event_type;
} cam_event_t;
#ifdef __cplusplus
}
#endif

#endif
