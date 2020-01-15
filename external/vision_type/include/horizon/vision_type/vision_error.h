/**
* Copyright (c) 2019 Horizon Robotics. All rights reserved.
* @file vision_error.h
* \~Chinese @brief 错误码
* \~Chinese @details 小于-1：指定错误码; 等于-1：通用错误码; 等于0：通用成功码;大于0：指定成功码;每个模块单独有100个>0的error code，也有100个<0的error code可分配
* @date 2019/4/1
*/

#ifndef VISION_TYPE_VISION_ERROR_H_
#define VISION_TYPE_VISION_ERROR_H_
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#define kHorizonVisionSuccess 0
#define kHorizonVisionFailure -1
#define kHorizonVisionErrorParam -2
#define kHorizonVisionErrorNoImpl -3
#define kHorizonVisionErrorNoMem -4
#define kHorizonVisionInitFail  -5
#define kHorizonVisionOpenFileFail  -6

static inline const char *GetHorizonVisionError(int32_t code) {
  switch (code) {
    case kHorizonVisionSuccess: return "Run OK";
    case kHorizonVisionFailure: return "Run Failed";
    case kHorizonVisionErrorParam: return "Wrong Parameters";
    case kHorizonVisionErrorNoImpl: return "The method is not implemented yet";
    case kHorizonVisionErrorNoMem: return "Out of memory";
    case kHorizonVisionInitFail: return "Init Failed";
    case kHorizonVisionOpenFileFail: return "Failed to open file";
  }
  return "";
}

/// \~Chinese xic >0 error code offset，xic 相关的只能是 > 1000 或 < -1000
#define kHorizonVisionXicOffset 1000
/// \~Chinese XPerson 专属错误码
#define kHorizonVisionXPersonOffset 1100
/// \~Chinese X2Solution 专属错误码，预留200
#define kHorizonVisionX2SolutionOffset 1200
#ifdef __cplusplus
}
#endif
#endif  // VISION_TYPE_VISION_ERROR_H_
