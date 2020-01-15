/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2019 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/

#ifndef __HB_VIO_COMMON_H__
#define __HB_VIO_COMMON_H__

#ifdef __cplusplus
extern "C"
{
#endif


#define X2_VIO_DEBUG (0)

#define vio_err(format, ...) printf("[%s]%s[%d] E: "format"\n",__TIME__, __func__, __LINE__, ##__VA_ARGS__)
#define vio_log(format, ...) printf("[%s]%s[%d] W: "format"\n",__TIME__, __func__, __LINE__, ##__VA_ARGS__)
#if X2_VIO_DEBUG
#define vio_dbg(format, ...) printf("[%s]%s[%d] D: "format"\n",__TIME__, __func__, __LINE__, ##__VA_ARGS__)
#else
#define vio_dbg(format, ...)
#endif

#define HB_VIO_PAESER_FAIL				001
#define HB_VIO_OPEN_CFG_FAIL			002
#define HB_VIO_INIT_FAIL				003
#define HB_VIO_START_FAIL				004
#define HB_VIO_STOP_FAIL				005
#define HB_VIO_PYM_IS_BUSY			    407

#define HB_VIO_SIF_OPEN_DEV_FAIL		700
#define HB_VIO_SIF_INIT_FAIL			701
#define HB_VIO_SIF_UPDATE_FAIL			702
#define HB_VIO_SIF_STOP_FAIL			703
#define HB_VIO_SIF_START_FAIL			704
#define HB_VIO_SIF_PARSER_FAIL			705
#define HB_VIO_SIF_EPOLL_CREATE_FAIL	706
#define HB_VIO_SIF_EPOLL_CTL_FAIL		707
#define HB_VIO_SIF_EPOLL_WAIT_FAIL		708
#define HB_VIO_SIF_STOP_WORKING			709
#define HB_VIO_SIF_MOT_DET_CFG_FAIL		710
#define HB_VIO_SIF_MOT_DET_EN_FAIL		711

#define HB_VIO_IPU_INIT_FAIL			800
#define HB_VIO_IPU_DEINIT_FAIL			801
#define HB_VIO_IPU_START_FAIL			802
#define HB_VIO_IPU_STOP_FAIL			803
#define HB_VIO_IPU_PARSER_FAIL			804
#define HB_VIO_IPU_EPOLL_CREATE_FAIL	805
#define HB_VIO_IPU_EPOLL_CTL_FAIL		806
#define HB_VIO_IPU_EPOLL_WAIT_FAIL		807
#define HB_VIO_IPU_STOP_WORKING			808

#define MOT_DET						(1 << 28)
#define ISP_HIST_FRAME_DONE			(1 << 27)
#define ISP_RSUM_FRAME_DONE			(1 << 26)
#define ISP_GRID_FRAME_DONE			(1 << 25)
#define ISP_TILE_FRAME_DONE			(1 << 24)
#define ISP_HIST_FRAME_DROP			(1 << 23)
#define ISP_RSUM_FRAME_DROP			(1 << 22)
#define ISP_GRID_FRAME_DROP			(1 << 21)
#define ISP_TILE_FRAME_DROP			(1 << 20)
#define ISP_RCCB_FRAME_FINISH		(1 << 19)
#define SIF_SIZE_ERR1				(1 << 18)
#define SIF_SIZE_ERR0				(1 << 17)
#define ISP_STF_FRAME_DROP			(1 << 16)
#define ISP_HMP_FRAME_DROP			(1 << 15)
#define ISP_STF_FRAME_FINISH		(1 << 14)
#define ISP_HMP_FRAME_FINISH		(1 << 13)
#define ISP_FRAME_START				(1 << 12)
#define PYM_US_FRAME_DROP			(1 << 11)
#define PYM_DS_FRAME_DROP			(1 << 10)
#define PYM_FRAME_DONE				(1 << 9)
#define PYM_FRAME_START				(1 << 8)
#define IPU_FRAME_DONE				(1 << 7)
#define IPU_FRAME_START				(1 << 6)
#define IPU_BUS23_TRANSMIT_ERRORS   (1 << 5)
#define IPU_BUS01_TRANSMIT_ERRORS   (1 << 4)
#define RESERVED					(1 << 3)
#define SIF_SOFT_DROP				(1 << 2)
#define SIF_FRAME_END_INTERRUPT		(1 << 1)
#define SIF_FRAME_START_INTERRUPT   (1 << 0)

#define ISP_INT_BITS    (ISP_RSUM_FRAME_DONE|ISP_GRID_FRAME_DONE|ISP_TILE_FRAME_DONE|ISP_HIST_FRAME_DROP|ISP_RSUM_FRAME_DROP|ISP_GRID_FRAME_DROP|ISP_TILE_FRAME_DROP|ISP_RCCB_FRAME_FINISH)
#define SIF_INT_BITS    (SIF_SIZE_ERR1|SIF_SIZE_ERR0|SIF_SOFT_DROP|SIF_FRAME_END_INTERRUPT|SIF_FRAME_START_INTERRUPT|MOT_DET)
#define IPU_INT_BITS    (IPU_FRAME_DONE|IPU_FRAME_START|IPU_BUS23_TRANSMIT_ERRORS|IPU_BUS01_TRANSMIT_ERRORS|PYM_FRAME_START|PYM_FRAME_DONE|PYM_DS_FRAME_DROP|PYM_US_FRAME_DROP)



#ifdef __cplusplus
}
#endif

#endif
