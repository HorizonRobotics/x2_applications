/*
 *  Copyright (c) 2019 by Horizon
 * \file bpu_io.h
 * \brief BPU io API for Horizon BPU Platform.
 */

#ifndef BPU_IO_H_
#define BPU_IO_H_

#include "bpu_predict.h"

typedef void* BPUPyramidHandle;
/*
 *
 */
int BPU_createPyramid(const char *vio_config_file_name,
                      const char* camera_config_file_name,
                      int cam_cfg_index,
                      BPUPyramidHandle *handle);

/*
 *
 */
int BPU_getPyramidResult(BPUPyramidHandle handle, BPUPyramidBuffer *buffer);

/*
 *
 */
void BPU_printPyramidBufferInfo(BPUPyramidBuffer buffer);

typedef void(*BPU_pyramidCallBack)(void *yuv_ptr, void *uv_ptr,
                                   int height, int width, void *cb_data_ptr);

void BPU_processPyramidResult(BPUPyramidBuffer pyr_buffer, int pyr_level,
                              BPU_pyramidCallBack cb, void *cb_data_ptr);

/*
 *
 */
int64_t BPU_getPyramidTimestamp(BPUPyramidBuffer buffer);

/*
 *
 */
int BPU_getPyramidFrameID(BPUPyramidBuffer buffer);

/*
 *
 */
int BPU_releasePyramidResult(BPUPyramidHandle handle, BPUPyramidBuffer buffer);

/*
 *
 */
int BPU_releasePyramid(BPUPyramidHandle handle);

typedef void* BPUFakeImageHandle;

/*
 * \brief create a fake image handle
 */
int BPU_createFakeImageHandle(int height, int width, BPUFakeImageHandle *handle);

/*
 * \brief get one image data from current fake image handle
 */
BPUFakeImage *BPU_getFakeImage(BPUFakeImageHandle handle, uint8_t* yuv_nv12_ptr, int img_len);

/*
 * \brief release one frame buffer than come from fake camera
 */
int BPU_releaseFakeImage(BPUFakeImageHandle handle, BPUFakeImage* image_ptr);

/*
 * \brief release fake camera handle
 */
int BPU_releaseFakeImageHandle(BPUFakeImageHandle handle);

typedef void *BPUFeedbackHandle;

/*
 *
 */
int BPU_createFeedback(const char *fb_config_file, BPUFeedbackHandle *handle);

/*
 *
 */
int BPU_getFeedbackResult(BPUFeedbackHandle handle,
                          void *data_ptr, int data_len,
                          BPUPyramidBuffer *pyr_buffer);

/*
 *
 */
int BPU_releaseFeedbackResult(BPUFeedbackHandle handle,
                              BPUPyramidBuffer pyr_buffer);

/*
 *
 */
int BPU_releaseFeedbackHandle(BPUFeedbackHandle handle);

/*
 * for double-camera
 */
struct BPUMultiPyramidBuffer {
    int data_num;
      BPUPyramidBuffer data[4];
};

typedef void *BPUMultiPyramidHandle;

/*
 *
 */
int BPU_createMultiPyramid(const char *vio_config_file_name,
                           const char *camera_config_file_name,
                           int cam_cfg_index,
                           BPUMultiPyramidHandle *handle);

/*
 *
 */
int BPU_getMultiPyramidResult(BPUMultiPyramidHandle handle,
                              BPUMultiPyramidBuffer *buffer);

/*
 *
 */
int BPU_releaseMultiPyramidResult(BPUMultiPyramidHandle handle,
                                  BPUMultiPyramidBuffer *buffer);

/*
 *
 */
int BPU_releaseMultiPyramid(BPUMultiPyramidHandle handle);

typedef void *BPUMultiFeedbackHandle;

/**
 * @brief BPU_createMultiFeedback
 * @param multi_fb_config_file [in] - config file for multi feedback
 * @param handle [in & out] - BPUMultiFeedbackHandle pointer
 * @return 0 if sucess, others means error
 */
int BPU_createMultiFeedback(const char *multi_fb_config_file,
                            BPUMultiFeedbackHandle *handle);

/**
 * @brief BPU_getMultiFeedbackResult
 * @param handle [in] - BPUMultiFeedbackHandle
 * @param data_ptr [in] - feedback image data array pointer
 * @param data_num [in] - feedback image number
 * @param data_len [in] - length of one feedback image
 * @param pyr_buffer [in & out] - BPUMultiPyramidBuffer
 * @return 0 if sucess, others means error
 */
int BPU_getMultiFeedbackResult(BPUMultiFeedbackHandle handle,
                               uint8_t *data_ptr[],
                               int data_num, int data_len,
                               BPUMultiPyramidBuffer *pyr_buffer);

/**
 * @brief BPU_releaseMultiFeedbackResult
 * @param handle [in] - BPUMultiFeedbackHandle
 * @param pyr_buffer [in] - BPUMultiPyramidBuffer
 * @return 0 if sucess, others means error
 */
int BPU_releaseMultiFeedbackResult(BPUMultiFeedbackHandle handle,
                                   BPUMultiPyramidBuffer *pyr_buffer);

/**
 * @brief BPU_releaseMultiFeedbackHandle
 * @param handle [in] - BPUMultiFeedbackHandle
 * @return 0 if sucess, others means error
 */
int BPU_releaseMultiFeedbackHandle(BPUMultiFeedbackHandle handle);

typedef void *BPUCameraBypassCtrlHandle;
/**
 * @brief BPU_createCameraBypassCtrl
 * @param handle [in & out] - BPUCameraBypassCtrlHandle
 * @return 0 if sucess, others means error
 */
int BPU_createCameraBypassCtrl(BPUCameraBypassCtrlHandle *handle);

/**
 * @brief BPU_setCameraBypassCtrlInfo
 * @param handle [in] - BPUCameraBypassCtrlHandle
 * @param port [in] - camera index, 0 or 1 when two cameras
 * @param enable [in] - whether enable camera bypass feature
 * @return 0 if sucess, others means error
 */
int BPU_setCameraBypassCtrlInfo(BPUCameraBypassCtrlHandle handle,
                                uint32_t port, bool enable);

/**
 * @brief BPU_releaseCameraBypassCtrl
 * @param handle [in] - BPUCameraBypassCtrlHandle
 * @return 0 if sucess, others means error
 */
int BPU_releaseCameraBypassCtrl(BPUCameraBypassCtrlHandle handle);

#endif // BPU_IO_H_
