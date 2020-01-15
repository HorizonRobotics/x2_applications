/*
 *  Copyright (c) 2019 by Horizon
 * \file bpu_predict.h
 * \brief BPU predict API for Horizon BPU Platform.
 */

#ifndef BPU_INDEX_CONSIST_H_
#define BPU_INDEX_CONSIST_H_

#include <stdint.h>
#include "bpu_predict.h"

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

int BPU_runModelForIDXConsistency(BPUHandle handle, const char *model_name,
                                  BPUPyramidBuffer input, BPUBBox *bbox,
                                  int nBox, int *resizable_cnt,
                                  BPU_Buffer_Handle output[], int nOutput,
                                  BPUModelHandle *model_handle,
                                  bool do_normalize);

int BPU_getHBMhandleFromBPUhandle(BPUHandle handle, uint64_t * hbm_handle);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // BPU_INDEX_CONSIST_H_
