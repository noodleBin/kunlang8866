/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef INC_HB_VIN_H_
#define INC_HB_VIN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern int32_t hb_vin_mipi_parse_cfg(const char *filename, int32_t fps, int32_t resolution, uint32_t entry_num);
extern int32_t hb_vin_mipi_init(uint32_t entry_num);
extern int32_t hb_vin_mipi_stop(uint32_t entry_num);
extern int32_t hb_vin_mipi_start(uint32_t entry_num);
extern int32_t hb_vin_mipi_deinit(uint32_t entry_num);
extern int32_t hb_vin_mipi_reset(uint32_t entry_num);
extern int32_t hb_vin_mipi_snrclk_set_en(uint32_t entry_num, uint32_t enable);
extern int32_t hb_vin_mipi_snrclk_set_freq(uint32_t entry_num, uint32_t freq);
extern int32_t hb_vin_mipi_chn_bypass(uint32_t port, uint32_t enable, uint32_t mux_sel, uint32_t chn_mask);
extern int32_t hb_vin_mipi_iar_bypass(uint32_t port, uint32_t enable, uint32_t enable_frame_id, uint32_t init_frame_id);
extern int32_t hb_vin_mipi_set_bypass(uint32_t port, uint32_t enable);
extern int32_t hb_vin_mipi_pre_request(uint32_t entry_num, uint32_t type, uint32_t timeout);
extern int32_t hb_vin_mipi_pre_result(uint32_t entry_num, uint32_t type, uint32_t result);
extern int32_t hb_vin_mipi_ipi_reset(uint32_t entry_num, int32_t ipi, uint32_t enable);
extern int32_t hb_vin_mipi_ipi_fatal(uint32_t entry_num, int32_t ipi);
extern int32_t hb_vin_mipi_ipi_get_info(uint32_t entry_num, int32_t ipi, void *info);
extern int32_t hb_vin_mipi_ipi_set_info(uint32_t entry_num, int32_t ipi, const void *info);

enum {
	HB_MIPI_PRE_TYPE_INIT,
	HB_MIPI_PRE_TYPE_START,
};

#ifdef __cplusplus
}
#endif

#endif  // INC_HB_VIN_H_
