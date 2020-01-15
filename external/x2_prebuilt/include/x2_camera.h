/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef __HB_CAMERA_H__
#define __HB_CAMERA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define X2_CAM_DEBUG (1)
#define cam_err(format, ...) printf("[%s]%s[%d] E: "format"\n",__TIME__, __func__, __LINE__, ##__VA_ARGS__)
#define cam_log(format, ...) printf("[%s]%s[%d] W: "format"\n",__TIME__, __func__, __LINE__, ##__VA_ARGS__)
#if X2_CAM_DEBUG
#define cam_dbg(format, ...) printf("[%s]%s[%d] D: "format"\n",__TIME__, __func__, __LINE__, ##__VA_ARGS__)
#else
#define cam_dbg(format, ...)
#endif

enum BLOCK_ACCESS_DEVICE_TYPE{
	SENSOR_DEVICE,
	ISP_DEVICE,
	EEPROM_DEVICE,
	IMU_DEVICE
};
	
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
	int g_id;
	int slot_id;
	int cam_id;
	int64_t timestamp;
	img_addr_info_t img_addr;
} cam_img_info_t;

extern int hb_cam_init(uint32_t cfg_index, const char *cfg_file);
extern int hb_cam_deinit(uint32_t cfg_index);
extern int hb_cam_start(uint32_t port);
extern int hb_cam_stop(uint32_t port);
extern int hb_cam_reset(uint32_t port);
extern int hb_cam_i2c_read(uint32_t port, int reg_addr);
extern int hb_cam_i2c_read_byte(uint32_t port, int reg_addr);
extern int hb_cam_i2c_write(uint32_t port, int reg_addr, uint16_t value);
extern int hb_cam_i2c_write_byte(uint32_t port, int reg_addr, uint8_t value);
extern int hb_cam_i2c_block_write(uint32_t port, int subdev, int reg_addr, char *buffer, int size);
extern int hb_cam_i2c_block_read(uint32_t port, int subdev, int reg_addr, char *buffer, int size);
extern int hb_cam_control_isp(uint32_t port, uint32_t enable);
extern int hb_cam_dynamic_switch(uint32_t port, uint32_t fps, uint32_t resolution);
extern int hb_cam_dynamic_switch_fps(uint32_t port, uint32_t fps);
extern int hb_cam_set_ex_gain(uint32_t port, uint32_t exposure_setting, uint32_t gain_setting_0, uint16_t gain_setting_1);
extern int hb_cam_set_awb_data(uint32_t port, AWB_DATA_s awb_data, float rg_gain, float b_gain);
extern int hb_cam_get_img(cam_img_info_t *cam_img_info);
extern int hb_cam_free_img(cam_img_info_t *cam_img_info);
#ifdef __cplusplus
}
#endif

#endif
