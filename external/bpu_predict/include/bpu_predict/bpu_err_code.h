/*
 *  Copyright (c) 2019 by Horizon
 * \file bpu_error.h
 * \brief BPU error code
 */


#ifndef BPU_ERROR_CODE_H_
#define BPU_ERROR_CODE_H_

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

// Note: The return value of function is negative when error happened,
//       0 when success

// sucess
#define BPU_OK                      0

#define BPU_ERR_BASE                (-1000)
// bpu common error
#define BPU_ERR_INVALID_HANDLE                            (BPU_ERR_BASE - 1)
#define BPU_ERR_NOT_INITED                                (BPU_ERR_BASE - 2)

// bpu io error
#define BPU_ERR_CAM_HAS_STARTED                           (BPU_ERR_BASE - 10)
#define BPU_ERR_CREATE_FAKE_IMAGE_FAILED                  (BPU_ERR_BASE - 11)
#define BPU_ERR_FEEDBACK_SIZE_MISMATCH                    (BPU_ERR_BASE - 12)
#define BPU_ERR_PYM_RESULT_NULLPTR                        (BPU_ERR_BASE - 13)

// bpu predict error when init instance
#define BPU_ERR_CONFIG_ERR                                (BPU_ERR_BASE - 100)
#define BPU_ERR_LOAD_HBM_FAILED                           (BPU_ERR_BASE - 101)
#define BPU_ERR_SET_LOG_LEVEL_FAILED                      (BPU_ERR_BASE - 102)
#define BPU_ERR_SET_GLOBAL_CONFIG_FAILED                  (BPU_ERR_BASE - 103)
#define BPU_ERR_GET_MODEL_NUM_FAILED                      (BPU_ERR_BASE - 104)
#define BPU_ERR_GET_MODEL_NAMES_FAILED                    (BPU_ERR_BASE - 105)
#define BPU_ERR_GET_MODEL_HANDLE_FAILED                   (BPU_ERR_BASE - 106)
#define BPU_ERR_GET_MODEL_CORE_NUM_FAILED                 (BPU_ERR_BASE - 107)
#define BPU_ERR_CREATE_RUNTASK_MEMPOOL_FAILED             (BPU_ERR_BASE - 108)
#define BPU_ERR_CREATE_BUFFER_MANAGER_FAILED              (BPU_ERR_BASE - 109)
#define BPU_ERR_ENGINE_INIT_FAILED                        (BPU_ERR_BASE - 110)

// bpu predict error when run model
#define BPU_ERR_INVALID_MODEL_NAME                        (BPU_ERR_BASE - 111)
#define BPU_ERR_INVALID_CORE_ID                           (BPU_ERR_BASE - 112)
#define BPU_ERR_RUNTASK_ALLOC_FAILED                      (BPU_ERR_BASE - 200)
#define BPU_ERR_RUNTASK_INIT_FAILED                       (BPU_ERR_BASE - 201)
#define BPU_ERR_RUNTASK_ADD_FAILED                        (BPU_ERR_BASE - 202)
#define BPU_ERR_RUNTASK_NO_BBOX_FOR_RESIZER               (BPU_ERR_BASE - 203)

#define BPU_ERR_RUNTASK_SET_INPUT_PYM_LAYER_ERR           (BPU_ERR_BASE - 204)
#define BPU_ERR_RUNTASK_SET_INPUT_PYM_SHAPE_MISMATCH      (BPU_ERR_BASE - 205)
#define BPU_ERR_RUNTASK_SET_INPUT_PYM_STEP_MISMATCH       (BPU_ERR_BASE - 206)
#define BPU_ERR_RUNTASK_SET_INPUT_EXTRA_SIZE_ERR          (BPU_ERR_BASE - 207)
#define BPU_ERR_RUNTASK_SET_INPUT_ALLOC_CNNMEM_ERR        (BPU_ERR_BASE - 208)
#define BPU_ERR_RUNTASK_SET_INPUT_INVALID_STARTXY         (BPU_ERR_BASE - 209)
#define BPU_ERR_RUNTASK_SET_INPUT_ROI_EXCEED              (BPU_ERR_BASE - 210)
#define BPU_ERR_RUNTASK_SET_INPUT_ROI_NOT_ALIGNED         (BPU_ERR_BASE - 211)
#define BPU_ERR_RUNTASK_SET_INPUT_PYMBUFF_NULLPTR         (BPU_ERR_BASE - 212)
#define BPU_ERR_RUNTASK_SET_INPUT_GET_NORM_PARAM_FAILED   (BPU_ERR_BASE - 213)
#define BPU_ERR_RUNTASK_SET_INPUT_FAKEIMG_NULLPTR         (BPU_ERR_BASE - 214)
#define BPU_ERR_RUNTASK_SET_OUTPUT_NUM_MISMATCH           (BPU_ERR_BASE - 215)
#define BPU_ERR_RUNTASK_SET_OUTPUT_ALLOC_CNNMEM_ERR       (BPU_ERR_BASE - 216)

#define BPU_ERR_RUNTASK_INVALID_TASKID                    (BPU_ERR_BASE - 217)
#define BPU_ERR_RUNTASK_INVALID_INPUT_SOURCE              (BPU_ERR_BASE - 218)
#define BPU_ERR_RUNTASK_MODEL_INPUT_ISNOT_RESIZER         (BPU_ERR_BASE - 219)
#define BPU_ERR_RUNTASK_RESIZER_HBRT_RISTART_FAILED       (BPU_ERR_BASE - 220)
#define BPU_ERR_RUNTASK_HBRT_RI_START_FAILED              (BPU_ERR_BASE - 221)
#define BPU_ERR_RUNTASK_HBRT_RI_GET_OUTPUT_FAILED         (BPU_ERR_BASE - 222)
#define BPU_ERR_RUNTASK_SET_FC_FAILED                     (BPU_ERR_BASE - 223)
#define BPU_ERR_RUNTASK_PRE_READ_OUTPUT_FAILED            (BPU_ERR_BASE - 224)
#define BPU_ERR_RUNTASK_NOT_DONE                          (BPU_ERR_BASE - 225)
#define BPU_ERR_RUNTASK_RELEASE_FAILED                    (BPU_ERR_BASE - 226)

// vio related error in bpu io
#define BPU_ERR_VIO_BASE                               (-2000)

#define BPU_ERR_VIO_PAESER_FAIL                        (BPU_ERR_VIO_BASE - 1)
#define BPU_ERR_VIO_OPEN_CFG_FAIL                      (BPU_ERR_VIO_BASE - 2)
#define BPU_ERR_VIO_INIT_FAIL                          (BPU_ERR_VIO_BASE - 3)
#define BPU_ERR_VIO_START_FAIL                         (BPU_ERR_VIO_BASE - 4)
#define BPU_ERR_VIO_STOP_FAIL                          (BPU_ERR_VIO_BASE - 5)
#define BPU_ERR_VIO_PYM_IS_BUSY                        (BPU_ERR_VIO_BASE - 6)

#define BPU_ERR_VIO_SIF_OPEN_DEV_FAIL                  (BPU_ERR_VIO_BASE - 7)
#define BPU_ERR_VIO_SIF_INIT_FAIL                      (BPU_ERR_VIO_BASE - 8)
#define BPU_ERR_VIO_SIF_UPDATE_FAIL                    (BPU_ERR_VIO_BASE - 9)
#define BPU_ERR_VIO_SIF_STOP_FAIL                      (BPU_ERR_VIO_BASE - 10)
#define BPU_ERR_VIO_SIF_START_FAIL                     (BPU_ERR_VIO_BASE - 11)
#define BPU_ERR_VIO_SIF_PARSER_FAIL                    (BPU_ERR_VIO_BASE - 12)
#define BPU_ERR_VIO_SIF_EPOLL_CREATE_FAIL              (BPU_ERR_VIO_BASE - 13)
#define BPU_ERR_VIO_SIF_EPOLL_CTL_FAIL                 (BPU_ERR_VIO_BASE - 14)
#define BPU_ERR_VIO_SIF_EPOLL_WAIT_FAIL                (BPU_ERR_VIO_BASE - 15)
#define BPU_ERR_VIO_SIF_STOP_WORKING                   (BPU_ERR_VIO_BASE - 16)

#define BPU_ERR_VIO_IPU_INIT_FAIL                      (BPU_ERR_VIO_BASE - 17)
#define BPU_ERR_VIO_IPU_DEINIT_FAIL                    (BPU_ERR_VIO_BASE - 18)
#define BPU_ERR_VIO_IPU_START_FAIL                     (BPU_ERR_VIO_BASE - 19)
#define BPU_ERR_VIO_IPU_STOP_FAIL                      (BPU_ERR_VIO_BASE - 20)
#define BPU_ERR_VIO_IPU_PARSER_FAIL                    (BPU_ERR_VIO_BASE - 21)
#define BPU_ERR_VIO_IPU_EPOLL_CREATE_FAIL              (BPU_ERR_VIO_BASE - 22)
#define BPU_ERR_VIO_IPU_EPOLL_CTL_FAIL                 (BPU_ERR_VIO_BASE - 23)
#define BPU_ERR_VIO_IPU_EPOLL_WAIT_FAIL                (BPU_ERR_VIO_BASE - 24)
#define BPU_ERR_VIO_IPU_STOP_WORKING                   (BPU_ERR_VIO_BASE - 25)


static inline const char* BPU_getErrorName(int error) {
  switch (error)
  {
    case BPU_OK:
      return "bpu success";
    case BPU_ERR_INVALID_HANDLE:
      return "bpu input handle is invalid";
    case BPU_ERR_NOT_INITED:
      return "bpu instance is not inited";
    case BPU_ERR_CAM_HAS_STARTED:
      return "camera has started";
    case BPU_ERR_CREATE_FAKE_IMAGE_FAILED:
      return "create fake image failed";
    case BPU_ERR_FEEDBACK_SIZE_MISMATCH:
      return "feedback input size is not same with pym";
    case BPU_ERR_PYM_RESULT_NULLPTR:
      return "pym result buffer is nullptr";
    case BPU_ERR_CONFIG_ERR:
      return "bpu config error";
    case BPU_ERR_LOAD_HBM_FAILED:
      return "bpu load hbm file failed";
    case BPU_ERR_SET_LOG_LEVEL_FAILED:
      return "bpu set hbrt log level failed";
    case BPU_ERR_SET_GLOBAL_CONFIG_FAILED:
      return "bpu set hbrt global config failed";
    case BPU_ERR_GET_MODEL_NUM_FAILED:
      return "bpu hbrt get model num failed";
    case BPU_ERR_GET_MODEL_NAMES_FAILED:
      return "bpu hbrt get model names failed";
    case BPU_ERR_GET_MODEL_HANDLE_FAILED:
      return "bpu hbrt get model handle failed";
    case BPU_ERR_GET_MODEL_CORE_NUM_FAILED:
      return "bpu hbrt get model core num failed";
    case BPU_ERR_CREATE_RUNTASK_MEMPOOL_FAILED:
      return "bpu create model runtask mem pool failed";
    case BPU_ERR_CREATE_BUFFER_MANAGER_FAILED:
      return "bpu create buffer manager failed";
    case BPU_ERR_ENGINE_INIT_FAILED:
      return "bpu engine init failed";
    case BPU_ERR_INVALID_MODEL_NAME:
      return "bpu input model name is invalid";
    case BPU_ERR_INVALID_CORE_ID:
      return "bpu input core id is invalid";
    case BPU_ERR_RUNTASK_ALLOC_FAILED:
      return "bpu model runtask alloc failed";
    case BPU_ERR_RUNTASK_INIT_FAILED:
      return "bpu model runtask init failed";
    case BPU_ERR_RUNTASK_ADD_FAILED:
      return "bpu model runtask add failed";
    case BPU_ERR_RUNTASK_NO_BBOX_FOR_RESIZER:
      return "bpu model runtask has no bbox for resizer";
    case BPU_ERR_RUNTASK_SET_INPUT_PYM_LAYER_ERR:
      return "bpu model runtask set input pym layer error";
    case BPU_ERR_RUNTASK_SET_INPUT_PYM_SHAPE_MISMATCH:
      return "bpu model runtask set input pym shape mismatch";
    case BPU_ERR_RUNTASK_SET_INPUT_PYM_STEP_MISMATCH:
      return "bpu model runtask set input pym step mismatch";
    case BPU_ERR_RUNTASK_SET_INPUT_EXTRA_SIZE_ERR:
      return "bpu model runtask set input extra data size error";
    case BPU_ERR_RUNTASK_SET_INPUT_ALLOC_CNNMEM_ERR:
      return "bpu model runtask set input alloc cnn mem error";
    case BPU_ERR_RUNTASK_SET_INPUT_INVALID_STARTXY:
      return "bpu model runtask set input invalid start X or Y";
    case BPU_ERR_RUNTASK_SET_INPUT_ROI_EXCEED:
      return "bpu model runtask set input roi exceed origin image";
    case BPU_ERR_RUNTASK_SET_INPUT_ROI_NOT_ALIGNED:
      return "bpu model runtask set input roi address is nog aligned";
    case BPU_ERR_RUNTASK_SET_INPUT_PYMBUFF_NULLPTR:
      return "bpu model runtask set input pym buffer is nullptr";
    case BPU_ERR_RUNTASK_SET_INPUT_GET_NORM_PARAM_FAILED:
      return "bpu model runtask set input get norm param failed";
    case BPU_ERR_RUNTASK_SET_INPUT_FAKEIMG_NULLPTR:
      return "bpu model runtask set input fake image buffer is nullptr";
    case BPU_ERR_RUNTASK_SET_OUTPUT_NUM_MISMATCH:
      return "bpu model runtask set output num mismatch";
    case BPU_ERR_RUNTASK_SET_OUTPUT_ALLOC_CNNMEM_ERR:
      return "bpu model runtask set output alloc cnn mem error";
    case BPU_ERR_RUNTASK_INVALID_TASKID:
      return "bpu model runtask task id is invalid";
    case BPU_ERR_RUNTASK_INVALID_INPUT_SOURCE:
      return "bpu model runtask input source is invalid";
    case BPU_ERR_RUNTASK_MODEL_INPUT_ISNOT_RESIZER:
      return "bpu model runtask input is not for resizer";
    case BPU_ERR_RUNTASK_RESIZER_HBRT_RISTART_FAILED:
      return "bpu model runtask hbrt resizer ri start failed";
    case BPU_ERR_RUNTASK_HBRT_RI_START_FAILED:
      return "bpu model runtask hbrt ri start failed";
    case BPU_ERR_RUNTASK_HBRT_RI_GET_OUTPUT_FAILED:
      return "bpu model runtask hbrt fi get output failed";
    case BPU_ERR_RUNTASK_SET_FC_FAILED:
      return "bpu model runtask set function call failed";
    case BPU_ERR_RUNTASK_PRE_READ_OUTPUT_FAILED:
      return "bpu model runtask pre read output failed";
    case BPU_ERR_RUNTASK_NOT_DONE:
      return "bpu model runtask is not done";
    case BPU_ERR_RUNTASK_RELEASE_FAILED:
      return "bpu model runtask release failed";
    case BPU_ERR_VIO_BASE:
      return "vio base error";
    case BPU_ERR_VIO_PAESER_FAIL:
      return "vio parse config failed";
    case BPU_ERR_VIO_OPEN_CFG_FAIL:
      return "vio open config file failed";
    case BPU_ERR_VIO_INIT_FAIL:
      return "vio init failed";
    case BPU_ERR_VIO_START_FAIL:
      return "vio start failed";
    case BPU_ERR_VIO_STOP_FAIL:
      return "vio stop failed";
    case BPU_ERR_VIO_PYM_IS_BUSY:
      return "vio pym is busy";
    case BPU_ERR_VIO_SIF_OPEN_DEV_FAIL:
      return "vio sif open device failed";
    case BPU_ERR_VIO_SIF_INIT_FAIL:
      return "vio sif init failed";
    case BPU_ERR_VIO_SIF_UPDATE_FAIL:
      return "vio sif update failed";
    case BPU_ERR_VIO_SIF_STOP_FAIL:
      return "vio sif stop failed";
    case BPU_ERR_VIO_SIF_START_FAIL:
      return "vio sif start failed";
    case BPU_ERR_VIO_SIF_PARSER_FAIL:
      return "vio sif parser config failed";
    case BPU_ERR_VIO_SIF_EPOLL_CREATE_FAIL:
      return "vio sif epoll create failed";
    case BPU_ERR_VIO_SIF_EPOLL_CTL_FAIL:
      return "vio sif epoll ctl failed";
    case BPU_ERR_VIO_SIF_EPOLL_WAIT_FAIL:
      return "vio sif epoll wait failed";
    case BPU_ERR_VIO_SIF_STOP_WORKING:
      return "vio sif stop working";
    case BPU_ERR_VIO_IPU_INIT_FAIL:
      return "vio ipu init failed";
    case BPU_ERR_VIO_IPU_DEINIT_FAIL:
      return "vio ipu deinit failed";
    case BPU_ERR_VIO_IPU_START_FAIL:
      return "vio ipu start failed";
    case BPU_ERR_VIO_IPU_STOP_FAIL:
      return "vio ipu stop failed";
    case BPU_ERR_VIO_IPU_PARSER_FAIL:
      return "vio ipu parse config failed";
    case BPU_ERR_VIO_IPU_EPOLL_CREATE_FAIL:
      return "vio ipu epoll create failed";
    case BPU_ERR_VIO_IPU_EPOLL_CTL_FAIL:
      return "vio ipu epoll ctl failed";
    case BPU_ERR_VIO_IPU_EPOLL_WAIT_FAIL:
      return "vio ipu epoll wait failed";
    case BPU_ERR_VIO_IPU_STOP_WORKING:
      return "vio ipu stop working";
  }
  return "invalid bpu error code";
}

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // BPU_ERROR_CODE_H_


