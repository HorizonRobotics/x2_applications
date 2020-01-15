/*************************************************************************
 *                     COPYRIGHT NOTICE
 *            Copyright 2016-2018 Horizon Robotics, Inc.
 *                   All rights reserved.
 *************************************************************************/
#ifndef __HB_BPU_INTF_H__
#define __HB_BPU_INTF_H__
#ifdef __cplusplus
extern "C" {
#endif
#include <inttypes.h>

/* recived msg id */
#define BPU_MSG_DONE	0x1
#define BPU_MSG_STEP	0x2
/* send msg id */
#define BPU_MSG_ENQUEUE		0x3
#define BPU_MSG_STEP_CFG	0x4

/* message queue message */
struct hb_bpu_msg {
	uint32_t msg_id;
	unsigned int magic_id;
	void *buf;
};

struct image_info {
	uint32_t y_height;
	uint32_t y_width;
	uint32_t uv_height;
	uint32_t uv_width;
};

struct image_stride {
	uint32_t y_stride;
	uint32_t uv_stride;
};

struct roi_region {
	int32_t left;
	int32_t top;
	int32_t right;
	int32_t bottom;
};

struct process_step_cfg {
	unsigned int process_cancel;
};

struct process_output {
	void *output_addr;
	void *output_phy_addr;
	uint32_t output_size;
	int errno;
	int occupy;
};

struct hb_bpu_process_cbs {
	/*
	 * after the data process done by bpu service, the function will be called.
	 * the function must not block.
	 */
	void (*process_done_cb)(unsigned int magic_id, struct process_output *output);

	/*
	 * callback after one segment done by bpu, the function will be called.
	 * the function must not block.
	 * if set to NULL, the data process have no sig until the end of
	 * model process.
	 */
	void (*process_step_cb)(unsigned int magic_id, struct process_step_cfg *step_config, int step);
};

extern int hb_bpu_init(char* libname);
extern void hb_bpu_exit(void);

/* 
 * return < 0 error, = 0 data to process buffer
 * if process_cbs set to NULL, message will be used. 
 */
int hb_bpu_process_resizer_data(
		char *model_name, int to_run_core,
		void *y_addr, void *uv_addr, unsigned int magic_id,
		struct image_info *info, struct image_stride *stride, struct roi_region *roi,
		void *output_addr, int output_range,
		struct hb_bpu_process_cbs *process_cbs);

int hb_bpu_process_resizer_data_phy(
		char *model_name, int to_run_core,
		void *y_phy_addr, void *uv_phy_addr, unsigned int magic_id,
		struct image_info *info, struct image_stride *stride, struct roi_region *roi,
		void *output_phy_addr, int output_range,
		struct hb_bpu_process_cbs *process_cbs);

int hb_bpu_process_pyramid_data(
		char *model_name, int to_run_core,
		void *y_addr, void *uv_addr, unsigned int magic_id,
		struct image_info *info, struct image_stride *stride,
		void *output_addr, int output_range,
		struct hb_bpu_process_cbs *process_cbs);

int hb_bpu_process_pyramid_data_phy(
		char *model_name, int to_run_core,
		void *y_phy_addr, void *uv_phy_addr, unsigned int magic_id,
		struct image_info *info, struct image_stride *stride,
		void *output_phy_addr, int output_range,
		struct hb_bpu_process_cbs *process_cbs);

int hb_bpu_process_feature_data(
		char *model_name, int to_run_core, 
		void *input_addr, uint32_t input_size, unsigned int magic_id,
		void *output_addr, int output_range,
		struct hb_bpu_process_cbs *process_cbs);

int hb_bpu_process_feature_data_phy(
		char *model_name, int to_run_core, 
		void *input_phy_addr, uint32_t input_size, unsigned int magic_id,
		void *output_phy_addr, int output_range,
		struct hb_bpu_process_cbs *process_cbs);

/*
 * return < 0 error, = 0 success
 * if success(*output_addr) is output addr(bpu addr type)
 * (*output_size) is the output size
 */
int hb_bpu_process_resizer_data_blocked(
		char *model_name, int to_run_core,
		void *y_addr, void *uv_addr, 
		struct image_info *info, struct image_stride *stride, struct roi_region *roi,
		void **output_addr, uint32_t *output_size);

int hb_bpu_process_pyramid_data_blocked(
		char *model_name, int to_run_core,
		void *y_addr, void *uv_addr,
		struct image_info *info, struct image_stride *stride,
		void **output_addr, uint32_t *output_size);

int hb_bpu_process_feature_data_blocked(
		char *model_name, int to_run_core,
		void *input_addr, uint32_t input_size,
		void **output_addr, uint32_t *output_size);

#ifdef __cplusplus
}
#endif
#endif
