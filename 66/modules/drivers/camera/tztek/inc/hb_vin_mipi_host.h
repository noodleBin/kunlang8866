/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
/**
 * @file     mipi_host.h
 * @brief    MIPI HOST Common define
 * @author   tarryzhang (tianyu.zhang@hobot.cc)
 * @date     2017/7/6
 * @version  V1.0
 * @par      Horizon Robotics
 */
#ifndef INC_HB_VIN_MIPI_HOST_H_
#define INC_HB_VIN_MIPI_HOST_H_

#ifdef __cplusplus
extern "C"
{
#endif


#include <linux/types.h>
#include "hb_vin_common.h"

int32_t hb_vin_mipi_host_start(const entry_t *e);
int32_t hb_vin_mipi_host_stop(const entry_t *e);
int32_t hb_vin_mipi_host_init(entry_t *e);
int32_t hb_vin_mipi_host_init_port_do(int32_t video_index ,int32_t width ,int32_t height);
int32_t hb_vin_mipi_host_deinit(entry_t *e);
int32_t hb_vin_mipi_host_parser_config(const void *root, entry_t *e);
int32_t hb_vin_mipi_host_snrclk_set_en(entry_t *e, uint32_t enable);
int32_t hb_vin_mipi_host_snrclk_set_freq(entry_t *e, uint32_t freq);
int32_t hb_vin_mipi_host_pre_init_request(entry_t *e, uint32_t timeout);
int32_t hb_vin_mipi_host_pre_start_request(entry_t *e, uint32_t timeout);
int32_t hb_vin_mipi_host_pre_init_result(entry_t *e, uint32_t result);
int32_t hb_vin_mipi_host_pre_start_result(entry_t *e, uint32_t result);
int32_t hb_vin_mipi_host_ipi_reset(entry_t *e, int32_t ipi, uint32_t enable);
int32_t hb_vin_mipi_host_ipi_fatal(entry_t *e, int32_t ipi);
int32_t hb_vin_mipi_host_ipi_get_info(entry_t *e, int32_t ipi, mipi_host_ipi_info_t *info);
int32_t hb_vin_mipi_host_ipi_set_info(entry_t *e, int32_t ipi, const mipi_host_ipi_info_t *info);

#ifdef __cplusplus
}
#endif

#endif  // INC_HB_VIN_MIPI_HOST_H_
