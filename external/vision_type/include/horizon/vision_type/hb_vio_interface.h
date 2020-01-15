/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2019 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/

#ifndef __HB_VIO_INTERFACE_H__
#define __HB_VIO_INTERFACE_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <linux/types.h>

/* for get info type */
#define HB_VIO_SRC_INFO			1
#define HB_VIO_PYM_INFO			2
#define HB_VIO_SIF_INFO			3
#define HB_VIO_IPU_STATE_INFO	4
#define HB_VIO_FRAME_START_INFO	5
#define HB_VIO_PYM_MULT_INFO	6
#define HB_VIO_SRC_MULT_INFO	7
#define HB_VIO_FEEDBACK_SRC_INFO 8
#define HB_VIO_FEEDBACK_FLUSH 9
#define HB_VIO_FEEDBACK_SRC_MULT_INFO 10
#define HB_VIO_PYM_INFO_CONDITIONAL 11

/* for get info type */

/* for set info type */
#define HB_VIO_BYPASS_CTRL_INFO	1
#define HB_VIO_SIF_UPDATE_INFO	2
#define HB_VIO_IPU_UPDATE_INFO	3
/* for set info type */

/* for ds|us */
#define DOWN_SCALE_MAIN_MAX 	6
#define DOWN_SCALE_MAX	24
#define UP_SCALE_MAX	6
/* for ds|us */

/* for mult info */
#define SRC_MAX			4
#define PYM_MAX			4
/* for mult info */

/* for sif motion detection*/
#define SIF_MOT_DET_WIDTH_MAX	2048
#define SIF_MOT_DET_HEIGHT_MAX	2048
/* for sif motion detection*/

/* for mipi bypass */
typedef struct vio_bypass_ctl_s{
	int enable;
	int channel;
}vio_bypass_ctl_t;
/* for mipi bypass */

/*
vio channel
channel 0 mean not support
channel 1 mean support channel 1
channel 2 mean support channel 2
channel 3 mean support channel 1 & 2
*/
typedef struct vio_channel_s{
	int ipu;
	int sif;
	int isp;
	int iar;
}vio_channel;

/* img_format */
enum Formate{
	HB_RGB,
	HB_RAW,
    HB_YUV422,
    HB_YUV420SP // only support yuv420sp
};

typedef struct addr_info_s{
	uint16_t width;
	uint16_t height;
	uint16_t step;
	uint8_t *y_paddr;
	uint8_t *c_paddr;
	uint8_t *y_vaddr;
	uint8_t *c_vaddr;
}addr_info_t;

typedef struct src_img_info_s{
	int cam_id;
	int slot_id;
	int img_format;
	int frame_id;
	int64_t timestamp;
	addr_info_t src_img;
	addr_info_t scaler_img;
}src_img_info_t;

typedef struct img_info_s{
	int slot_id;					// getted slot buff
	int frame_id;					// for x2 may be 0 - 0xFFFF or 0x7FFF
	int64_t timestamp;				// BT from Hisi; mipi & dvp from kernel time
	int img_format;					// now only support yuv420sp
	int ds_pym_layer;				// get down scale layers
	int us_pym_layer;				// get up scale layers
	addr_info_t src_img;			// for x2 src img = crop img
	addr_info_t down_scale[DOWN_SCALE_MAX];
	addr_info_t up_scale[UP_SCALE_MAX];
	addr_info_t down_scale_main[DOWN_SCALE_MAIN_MAX];
	int cam_id;
}img_info_t;

typedef struct mult_src_info_s{
	int src_num;
	src_img_info_t src_img_info[SRC_MAX];
}mult_src_info_t;

typedef struct mult_img_info_s{
	int img_num;
	img_info_t img_info[PYM_MAX];
}mult_img_info_t;


/* add multi-process api */
int hb_vio_connect(int cam_port, int img_type);
int hb_vio_disconnect(void);
/* add multi-process api */

int hb_vio_start(void);
int hb_vio_stop(void);
int hb_vio_init(const char *cfg_file);
int hb_vio_deinit(void);
int hb_vio_get_info(uint32_t info_type, void *data);
int hb_vio_set_info(uint32_t info_type, void *data);
int hb_vio_free_info(uint32_t info_type, void *data);
int hb_vio_get_info_conditional(uint32_t info_type, void *data, int time);

/* update api */
int hb_vio_free(img_info_t *img_info);
/* update api */

/* new api */
int hb_vio_pym_process(src_img_info_t *src_img_info);
int hb_vio_mult_pym_process(mult_src_info_t *mult_src_info);
int hb_vio_mult_free(mult_img_info_t *mult_img_info);
int hb_vio_src_free(src_img_info_t *src_img_info);
int hb_vio_mult_src_free(mult_src_info_t *mult_src_info);
/* new api */

/*use for debug*/
int hb_vio_test_info(char *file_name);
uint32_t hb_vio_meminfo(uint32_t *paddr);
void* hb_vio_mmap(uint64_t offset, uint32_t lenth);
void hb_vio_munmap(void *addr, uint32_t lenth);
/*use for debug*/

void hb_vio_enable_3a(void);
void hb_vio_disenable_3a(void);

/*sif mot_det config*/
int hb_vio_sif_mot_det_config(int left, int top, int right,
	int bottom, int threshold);
int hb_vio_sif_mot_det_enable(int mot_det_en);
/*sif mot_det config*/

/*suspend to idle*/
int enter_freeze_state(void);
/*suspend to idle*/

#ifdef __cplusplus
}
#endif

#endif //__IPS_INTERFACE_H__
