/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides xroc framework C interface
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.4
 */

#ifndef HOBOTXSDK_XROC_CAPI_TYPE_H_
#define HOBOTXSDK_XROC_CAPI_TYPE_H_

#include "stddef.h"
#include "stdint.h"

#if !defined(HOBOT_EXPORT)
#ifdef HR_WIN
#ifdef HOBOT_FACEPOST_DLL_EXPORTS
#ifdef HOBOT_EXPORTS
#define HOBOT_EXPORT __declspec(dllexport)
#else
#define HOBOT_EXPORT __declspec(dllimport)
#endif  // #ifdef HOBOT_EXPORTS
#else
#define HOBOT_EXPORT
#endif  // #ifdef HOBOT_FACEPOST_DLL_EXPORTS
#else
#define HOBOT_EXPORT
#endif  // #ifdef HR_WIN
#endif  // #if !defined(HOBOT_EXPORT)

#ifdef __cplusplus
extern "C" {
#endif

#define DECLEAR_DEFINE_CAPI_FREE_PARENT(type)                               \
  HOBOT_EXPORT                                                            \
  inline void HobotXRocCapi##type##FreeParent(HobotXRocCapiData** parent) { \
    HobotXRocCapi##type##Free((HobotXRocCapiBaseDataVector**)(parent));     \
  }

typedef struct HobotXRocCapiBaseData_ {
  const char* type_;
  const char* name_;
  int error_code_;
  const char* error_detail_;
  void* context_;  //< 用来存储SDK内部上下文，在free HobotXRocCapiData
                   // 时通知SDK释放context_指向的资源
} HobotXRocCapiBaseData;

typedef HobotXRocCapiBaseData HobotXRocCapiData;

/**
 * @brief 释放DataList
 *
 * @param datalist [in, out] 释放datalist并把指针置空
 */
HOBOT_EXPORT
void HobotXRocCapiDataFree(HobotXRocCapiData** data);

/**
 * @brief HobotXRocCapiData数组结构
 *
 */
typedef struct HobotXRocCapiDataList_ {
  size_t datas_size_;
  HobotXRocCapiData* datas_[];
} HobotXRocCapiDataList;

typedef HobotXRocCapiDataList HobotXRocCapiInputList;
typedef HobotXRocCapiDataList HobotXRocCapiOutputList;

/**
 * @brief HobotXRocCapiBaseDataVector数组结构
 *
 */
typedef struct HobotXRocCapiBaseDataVector_ {
  HobotXRocCapiData parent_;
  HobotXRocCapiDataList* datas_;
} HobotXRocCapiBaseDataVector;

/**
 * @brief HobotXRocCapiBaseDataVector构造
 *
 * @param length
 * @return HobotXRocCapiBaseDataVector*
 */
HOBOT_EXPORT
HobotXRocCapiBaseDataVector* HobotXRocCapiBaseDataVectorAlloc(size_t length);

/**
 * @brief HobotXRocCapiBaseDataVector释放
 *
 * @param data_vector
 */
HOBOT_EXPORT
void HobotXRocCapiBaseDataVectorFree(HobotXRocCapiBaseDataVector** data_vector);

/**
 * @brief 定义一个根据BaseDataVector.parent释放BaseDataVector的函数
 *
 */
DECLEAR_DEFINE_CAPI_FREE_PARENT(BaseDataVector);

/**
 * @brief 申请DataList
 *
 */
HOBOT_EXPORT
HobotXRocCapiDataList* HobotXRocCapiDataListAlloc(size_t length);

/**
 * @brief 释放DataList
 *
 * @param datalist [in, out] 释放datalist并把指针置空
 */
HOBOT_EXPORT
void HobotXRocCapiDataListFree(HobotXRocCapiDataList** datalist);

/**
 * @brief 异步callback返回的的数据结构定义
 *
 */
typedef struct HobotXRocCapiCallbackData_ {
  int64_t sequence_id_;
  int error_code_;
  char* error_detail_;              //< 释放callbackdata时会被释放
  HobotXRocCapiInputList* inputs_;  //< 是异步输入的数据指针，SDK不会主动释放
  HobotXRocCapiDataList* outputs_;  //< SDK不会主动释放，需要用户使用完后主动调用 HobotXRocCapiDataListFree
} HobotXRocCapiCallbackData;

#ifdef __cplusplus
}
#endif

#endif  // HOBOTXSDK_XROC_CAPI_TYPE_H_
