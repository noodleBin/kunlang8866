/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2020 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef __HB_VIN_INTERFACE_H__
#define __HB_VIN_INTERFACE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "hb_vin_data_info.h"
#include "hb_vin_common.h"

#define VIN_REG_HIGH_BITS 16u
#define VIN_SHARE_AE_FLAG 0xA0u
#define VIN_PARSE_LENGTH 28

/* vin init and start status*/
typedef enum vin_device_type_s {
	SENSOR_DEVICE,
	ISP_DEVICE,
	EEPROM_DEVICE,
	IMU_DEVICE,
	DEVICE_INVALID
} hb_vin_device_type_e;

//---------------------------------------------------------
/**
 * initialization of vin     return port /dev/video#
 * @param[in] cfg_index      configuration index in camera json file
 * @param[in] cfg_file       camera configuration file
 * @return 0 if success
 */
extern uint32_t hb_port_mapping(uint32_t port ,uint32_t* pVideoIndex,uint32_t *pWidth,uint32_t *pHeight,uint32_t *pFps,uint32_t *pFormat);

//---------------------------------------------------------
/**
 * initialization of vin     init cam, include cmos, mipi, cim/cimdma.
 * @param[in] cfg_index      configuration index in camera json file
 * @param[in] cfg_file       camera configuration file
 * @return 0 if success
 */
extern int32_t hb_vin_init(uint32_t cfg_index, const char *cfg_file);

//---------------------------------------------------------
/**
 * destroy vin               deinit cam, include cmos, mipi, cim/cimdma
 * @param[in] cfg_index      configuration index in camera json file
 * @return 0 if success
 */
extern int32_t hb_vin_deinit(uint32_t cfg_index);

//---------------------------------------------------------
/**
 * vin_start       	start cam, include cmos, mipi, cim/cimdma
 * @param[in] port 	which cam you want to choose;
 * @return 0 if success
 */
extern int32_t hb_vin_start(uint32_t port);
//---------------------------------------------------------
/**
 * hb_vin_stop :        deinit cam, include cmos, mipi, cim/cimdma.
 * @param[in] port:     camera port, which cam you want to choose;
 */
extern int32_t hb_vin_stop(uint32_t port);
//---------------------------------------------------------
/**
 * hb_cam_start_all : start all cam which you init(in config index), include cmos, mipi, cim/cimdma.
 */
extern int32_t hb_vin_start_all();
//---------------------------------------------------------
/**
 * hb_cam_stop_all : stop all cam which you init(in config index), include cmos, mipi, cim/cimdma.
 */
extern int32_t hb_vin_stop_all();
//---------------------------------------------------------
/**
 * hb_vin_reset :               reset cam, include cmos, mipi, cim/cimdma.
 * @param[in] port              which cam you want to choose;
 * @return 0 if success
 */
extern int32_t hb_vin_reset(uint32_t port);
//---------------------------------------------------------
/**
 * hb_vin_power_on :           power on cmos.
 * @param[in] port             which cam you want to choose;
 * @return 0 if success
 */
extern int32_t hb_vin_power_on(uint32_t port);
//---------------------------------------------------------
/**
 * hb_vin_power_off :                  power off cmos.
 * @param[in] port              which cam you want to choose;
 * @return 0 if success
 */
extern int32_t hb_vin_power_off(uint32_t port);
//---------------------------------------------------------
/**
 * hb_vin_get_fps : deinit cam, include cmos, mipi, cim/cimdma.
 * @param[in] port: cam port, which cam you want to choose;
 * @param[out] fps: current frame rate
 * @return 0 if success
 */
extern int32_t hb_vin_get_fps(uint32_t port, uint32_t *fps);
//---------------------------------------------------------
/**
 * hb_vin_i2c_read : for i2c read.
 * @param[in] port   which cam you want to choose;
 * @param[in] reg_addr: sensor register address
 * @return value
 */
extern int32_t hb_vin_i2c_read(uint32_t port, uint32_t reg_addr);
//---------------------------------------------------------
/**
 * hb_vin_i2c_read_byte : for i2c read.
 * @param[in] port: cam port, which cam you want to choose;
 * @param[in] reg_addr: which addr you want to read;
 * @return value
 */
extern int32_t hb_vin_i2c_read_byte(uint32_t port, uint32_t reg_addr);
//---------------------------------------------------------
/**
 * hb_vin_i2c_write : for i2c write.
 * param[in] port: cam port, which cam you want to choose;
 * param[in] reg_addr: which addr you want to read;
 * param[in] value: what you write to addr;
 * @return 0 if success
 */
extern int32_t hb_vin_i2c_write(uint32_t port, uint32_t reg_addr, uint16_t value);
//---------------------------------------------------------
/**
 * hb_vin_i2c_write_byte : for i2c write.
 * param[in]port: cam port, which cam you want to choose;
 * param[in]reg_addr: which addr you want to read;
 * param[in]value: what you write to addr;
 * @return 0 if success
 */
extern int32_t hb_vin_i2c_write_byte(uint32_t port, uint32_t reg_addr, uint8_t value);
//---------------------------------------------------------
/**
 * hb_vin_i2c_block_write : for i2c write.
 * param[in]port: cam port, which cam you want to choose;
 * param[in]subdev: sub device;
 * param[in]reg_addr: your block's first addr;
 * param[in]buffer: save what you write;
 * param[in]size: block size;
 * @return 0 if success
 */
extern int32_t hb_vin_i2c_block_write(uint32_t port, hb_vin_device_type_e subdev, uint32_t reg_addr, char *buffer, uint32_t size);
//---------------------------------------------------------
/**
 * hb_vin_i2c_block_read : for i2c read.
 * param[in] port: cam port, which cam you want to choose;
 * param[in] subdev: sub device;
 * param[in] reg_addr: your block's first addr;
 * param[in] buffer: save what you read;
 * param[in] size: block size;
 * param[out]buffer: value
 * @return 0 if success
 */
extern int32_t hb_vin_i2c_block_read(uint32_t port, hb_vin_device_type_e subdev, uint32_t reg_addr, char *buffer, uint32_t size);
//---------------------------------------------------------
/**
 * hb_vin_dynamic_switch : switch coms setting.
 * param[in]port: cam port, which cam you want to choose;
 * param[in]fps: switch cmos fps setting;
 * param[in]resolution: switch cmos resolution setting;
 * @return 0 if success
 */
extern int32_t hb_vin_dynamic_switch(uint32_t port, uint32_t fps, uint32_t resolution);
//---------------------------------------------------------
/**
 * hb_vin_dynamic_switch_fps : switch coms setting.
 * param[in] port: cmos port, which cmos you want to choose;
 * param[in] fps: switch cmos fps setting;
 * @return 0 if success
 */
extern int32_t hb_vin_dynamic_switch_fps(uint32_t port, uint32_t fps);
//---------------------------------------------------------
/**
 * hb_vin_get_img : get cam img info(only support in xj2).
 * param[out] cam_img_info: output parameters, inlcude vaddr/paddr/w/h etc;
 * @return 0 if success
 */
extern int32_t hb_vin_get_img(cam_img_info_t *cam_img_info);
//---------------------------------------------------------
/**
 * hb_vin_free_img : free cam info(only support in xj2).
 * param[in]cam_img_info: output parameters, inlcude vaddr/paddr/w/h etc.
 * @return 0 if success
 */
extern int32_t hb_vin_free_img(cam_img_info_t *cam_img_info);
//---------------------------------------------------------
/**
 * hb_vin_get_data : get cam data info.
 * param[in] port: cam port, which cam you want to choose;
 * param[in] data_type: data type, raw/yuv etc.
 * param[out] data: cam image(from cim/cimdma)
 * @return 0 if success
 */
extern int32_t hb_vin_get_data(uint32_t port, CAM_DATA_TYPE_E data_type, void* data);
//---------------------------------------------------------
/**
 * hb_vin_free_data : free cam data info.
 * param[in] port: cam port, which cam you want to choose;
 * param[in] data_type: data type, raw/yuv etc.
 * param[in] data: cam image(from cim/cimdma)
 * @return 0 if success
 */
extern int32_t hb_vin_free_data(uint32_t port, CAM_DATA_TYPE_E data_type, void* data);
//---------------------------------------------------------
/**
 * hb_vin_parse_embed_data: parse embed raw data to info struct.
 * param[in] port: cam port, which cam you want to choose;
 * param[in] embed_raw: embed raw data(from cim/cimdma).
 * param[out] embed_info: embed info struct to parsed
 * @return 0 if success
 */
extern int32_t hb_vin_parse_embed_data(uint32_t port, char* embed_raw, struct embed_data_info_s *embed_info);
//---------------------------------------------------------
/**
 * hb_vin_clean_img : clean cam info(only support in xj2).
 * param[in] cam_img_info: output parameters, inlcude vaddr/paddr/w/h etc.
 * @return 0 if success
 */
extern int32_t hb_vin_clean_img(cam_img_info_t *cam_img_info);
//---------------------------------------------------------
/**
 * hb_vin_extern_isp_reset :  reset independent external isp.
 * param[in] port: cam port, which cam you want to choose;
 * @return 0 if success
 */
extern int32_t hb_vin_extern_isp_reset(uint32_t port);
//---------------------------------------------------------
/**
 * hb_vin_extern_isp_poweroff : poweroff independent external isp.
 * param[in] port: cam port, which cam you want to choose;
 * @return 0 if success
 */
extern int32_t hb_vin_extern_isp_poweroff(uint32_t port);
//---------------------------------------------------------
/**
 * hb_vin_extern_isp_poweron : poweron independent external isp.
 * param[in] port: cam port, which cam you want to choose;
 * @return 0 if success
 */
extern int32_t hb_vin_extern_isp_poweron(uint32_t port);
//---------------------------------------------------------
/**
 * hb_vin_spi_block_write : for spi write.
 * param[in] port: cam port, which cam you want to choose;
 * param[in] subdev: sub device;
 * param[in] reg_addr: your block's first addr;
 * param[in] buffer: save what you write;
 * param[in] size: block size;
 * @return 0 if success
 */
extern int32_t hb_vin_spi_block_write(uint32_t port, hb_vin_device_type_e subdev, uint32_t reg_addr, char *buffer, uint32_t size);
//---------------------------------------------------------
/**
 * hb_vin_spi_block_read : for spi read.
 * param[in] port: cam port, which cam you want to choose;
 * param[in] subdev: sub device;
 * param[in] reg_addr: your block's first addr;
 * param[out] buffer: save what you read;
 * param[in] size: block size;
 * @return 0 if success
 */
extern int32_t hb_vin_spi_block_read(uint32_t port, hb_vin_device_type_e subdev, uint32_t reg_addr, char *buffer, uint32_t size);
//---------------------------------------------------------
/**
 * hb_vin_set_mclk : set sensor mclk(only xj3 support).
 * param[in] entry_num: which mipi host you choose;
 * param[in] mclk:  main clk;
 * @return 0 if success
 */
extern int32_t hb_vin_set_mclk(uint32_t entry_num, uint32_t mclk);
//---------------------------------------------------------
/**
 * hb_vin_enable_mclk : enable sensor mclk(only xj3 support).
 * param[in] entry_num: which mipi host you choose;
 * @return 0 if success
 */
extern int32_t hb_vin_enable_mclk(uint32_t entry_num);
//---------------------------------------------------------
/**
 * hb_vin_disable_mclk : disable sensor mclk(only xj3 support).
 * param[in] entry_num:  which mipi host you choose;
 * @return 0 if success
 */
extern int32_t hb_vin_disable_mclk(uint32_t entry_num);
//---------------------------------------------------------
/**
 * hb_vin_dynamic_switch_mode : switch coms setting.
 * param[in] port: 				cam port, which cam you want to choose;
 * param[in] mode: 				switch cmos mode setting;
 * @return 0 if success
 */
extern int32_t hb_vin_dynamic_switch_mode(uint32_t port, uint32_t mode);
//---------------------------------------------------------
/**
 * hb_vin_share_ae : shared ae.
 * param[in] sharer_dev_port: share port
 * param[in] user_dev_port:  user port
 * @return 0 if success
 */
extern int32_t hb_vin_share_ae(uint32_t sharer_dev_port, uint32_t user_dev_port);
//---------------------------------------------------------
/**
 * hb_vin_get_sns_info : get information about sensor
 * param[in] dev_port:   cam port, which cam you want to choose;
 * param[out] sp:        information about sensor or eeprom
 * param[in] type:       0: read sensor parameters
 *                       1: read eeprom parameters
 *                       3: read parameters form sensor and eeprom
 * @return 0 if success
 */
extern int32_t hb_vin_get_sns_info(uint32_t dev_port, cam_parameter_t *sp, uint8_t type);
//---------------------------------------------------------
/**
 * hb_vin_bypass_enable : deinit cam, include cmos, mipi, cim/cimdma.
 * param[in] port:   	  cam port, which cam you want to choose;
 * param[in] enable: 	  1 enable; 0 disable;
 * @return 0 if success
 */
extern int32_t hb_vin_bypass_enable(uint32_t port, int32_t enable);
//---------------------------------------------------------
/**
 * hb_vin_set_fps_ctrl : skip frame in cim/cimdma side dynamically.
 * param[in] port: cam port: which cam port you want to choose;
 * param[in] skip_frame: 1 enable; 0 disable;
 * param[in] in_fps: input fps
 * param[in] out_fps: output fps
 * @return 0 if success
 */
extern int32_t hb_vin_set_fps_ctrl(uint32_t port, uint32_t skip_frame, uint32_t in_fps, uint32_t out_fps);
extern int32_t hb_vin_ipi_reset(uint32_t entry_num, uint32_t ipi_index, uint32_t enable);
extern int32_t hb_vin_get_stat_info(uint32_t port, struct vio_statinfo *statinfo);

extern int32_t hb_vin_set_event_callback(uint32_t port, void (*event_callback)(cam_event_t* fault_info));

#ifdef __cplusplus
}
#endif

#endif
