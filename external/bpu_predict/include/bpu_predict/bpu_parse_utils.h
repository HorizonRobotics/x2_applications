/*
 *  Copyright (c) 2019 by Horizon
 * \file bpu_parse_utils.h
 * \brief BPU parse output utils API for Horizon BPU Platform.
 */

#ifndef BPU_PARSE_UTILS_H_
#define BPU_PARSE_UTILS_H_

#include "bpu_predict.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*
 * \brief parse det thresh result, get bbox
 */
int BPU_parseDetThreshResult(BPUHandle handle, const char* model_name, BPU_Buffer_Handle output[], int nOutput, BPUBBox **bbox, int *nBox, const char** cls_names, int nCls);

/*
 * \brief parse channel max result, get classify index
 */
int BPU_parseChannelMaxResult(BPUHandle handle, const char* model_name, BPU_Buffer_Handle output[], int nOutput, int *result, int nRes);

/*
 * \brief parse rpp op output result, get bbox
 */
int BPU_parseRPPResult(BPUHandle handle, const char *model_name, BPU_Buffer_Handle output[], int output_index, BPUBBox **bbox, int *nBox);

/*
 * \brief parse dpp op output result, get bbox
 */
int BPU_parseDPPResult(BPUHandle handle, const char *model_name, BPU_Buffer_Handle output[], int nOutput, BPUBBox **bbox, int *nBox);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // end of BPU_PARSE_UTILS_H_

