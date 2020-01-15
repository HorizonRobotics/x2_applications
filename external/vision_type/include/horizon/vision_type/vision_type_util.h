/*!
 * Copyright (c) 2019 Horizon Robotics
 * \file vision_type_util.h
 * \~Chinese \brief C数据结构常见操作
 * \author sunshuhuan
 */

#ifndef VISION_TYPE_VISION_TYPE_UTIL_H_
#define VISION_TYPE_VISION_TYPE_UTIL_H_

#include <stdint.h>
#include <stdbool.h>

#include "vision_msg.h"
#include "vision_type.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \~Chinese @brief 复制字符串
 *
 * \~Chinese @param str [in] 源字符串
 * \~Chinese @return char* 失败则返回NULL
 * \~Chinese @note 返回的字符串资源需手动调用free释放
 */
char *HorizonVisionStrDup(const char *str);

/**
 * \~Chinese @brief 复制字符串
 *
 * \~Chinese @param str [in] 源字符串
 * \~Chinese @return char* 失败则返回NULL
 * \~Chinese @note 返回的字符串资源需手动调用free释放
 */
char *HorizonVisionMemDup(const char *src_addr, size_t size);

/**
 * \~Chinese @brief 构造landmarks结构
 *
 * \~Chinese @param pnew_lmks [out] 保存新landmarks指针的地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @points资源需单独申请
 */
int HorizonVisionAllocLandmarks(HorizonVisionLandmarks **pnew_lmks);

/**
 * \~Chinese @brief 复制landmarks结构，复制数据到另一个结构体，相当于赋值操作
 *
 * \~Chinese @param lmks [in] 源landmarks
 * \~Chinese @param new_lmks [out] 新的landmarks
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionCopyLandmarks(HorizonVisionLandmarks *lmks,
                               HorizonVisionLandmarks *new_lmks);

/**
 * \~Chinese @brief 复制landmarks结构，构造新的结构体并复制内容过来，然后返回新的结构体指针
 *
 * \~Chinese @param lmks [in] 源landmarks
 * \~Chinese @param pnew_lmks [out] 用于保存新的landmarks的地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionDupLandmarks(HorizonVisionLandmarks *lmks,
                              HorizonVisionLandmarks **pnew_lmks);

/**
 * \~Chinese @brief 清空landmarks内部指针数据，但保留结构体本身
 *
 * \~Chinese @param lmks [in] 要清空的结构体指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果points不为空，则会一并释放该资源
 */
int HorizonVisionCleanLandmarks(HorizonVisionLandmarks *lmks);

/**
 * \~Chinese @brief 释放landmarks
 *
 * \~Chinese @param lmks [in] 要释放的指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果points不为空，则会一并释放该资源
 */
int HorizonVisionFreeLandmarks(HorizonVisionLandmarks *lmks);

/**
 * \~Chinese @brief 申请HorizonVisionCharArray
 *
 * \~Chinese @param pnew_array [out] 用来存储新申请结构体指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note HorizonVisionCharArray.values资源需单独申请
 */
int HorizonVisionAllocCharArray(HorizonVisionCharArray **pnew_array);

/**
 * \~Chinese @brief 复制HorizonVisionCharArray内容，复制数据到另一个结构体，相当于赋值操作
 *
 * \~Chinese @param array [in] 源HorizonVisionCharArray
 * \~Chinese @param new_array [out] 新的HorizonVisionCharArray
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionCopyCharArray(HorizonVisionCharArray *array,
                               HorizonVisionCharArray *new_array);

/**
 * \~Chinese @brief 复制HorizonVisionCharArray，构造新的结构体并复制内容过来，然后返回新的结构体指针
 *
 * \~Chinese @param array [in] 源HorizonVisionCharArray
 * \~Chinese @param pnew_array [out] 新的HorizonVisionCharArray
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionDupCharArray(HorizonVisionCharArray *array,
                              HorizonVisionCharArray **pnew_array);

/**
 * \~Chinese @brief 清理HorizonVisionCharArray内部指针数据，但保留结构体本身
 *
 * \~Chinese @param array [in] 要清理的结构体指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果HorizonVisionCharArray.values不为空，则会一并释放该资源
 */
int HorizonVisionCleanCharArray(HorizonVisionCharArray *array);

/**
 * \~Chinese @brief 释放CharArray
 *
 * \~Chinese @param farray [in] 要释放的指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果HorizonVisionCharArray.values不为空，则会一并释放该资源
 */
int HorizonVisionFreeCharArray(HorizonVisionCharArray *farray);

/**
 * \~Chinese @brief 申请FloatArray
 *
 * \~Chinese @param pnew_farray [out] 用来存储新申请结构体指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note values资源需单独申请
 */
int HorizonVisionAllocFloatArray(HorizonVisionFloatArray **pnew_farray);

/**
 * \~Chinese @brief 复制FloatArray内容，复制数据到另一个结构体，相当于赋值操作
 *
 * \~Chinese @param farray [in] 源Float Array
 * \~Chinese @param new_farray [out] 新的Float Array
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionCopyFloatArray(HorizonVisionFloatArray *farray,
                                HorizonVisionFloatArray *new_farray);

/**
 * \~Chinese @brief 复制FloatArray，构造新的结构体并复制内容过来，然后返回新的结构体指针
 *
 * \~Chinese @param farray [in] 源FloatArrray
 * \~Chinese @param pnew_farray [out] 新的FloatArray
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionDupFloatArray(HorizonVisionFloatArray *farray,
                               HorizonVisionFloatArray **pnew_farray);

/**
 * \~Chinese @brief 清理FloatArray内部指针数据，但保留结构体本身
 *
 * \~Chinese @param farray [in] 要清理的结构体指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果values不为空，则会一并释放该资源
 */
int HorizonVisionCleanFloatArray(HorizonVisionFloatArray *farray);

/**
 * \~Chinese @brief 释放FloatArray
 *
 * \~Chinese @param farray [in] 要释放的指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果values不为空，则会一并释放该资源
 */
int HorizonVisionFreeFloatArray(HorizonVisionFloatArray *farray);

/**
 * \~Chinese @brief 申请分割结构体
 *
 * \~Chinese @param pnew_farray [out] 用来保存新构造的结构体指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note values资源需单独申请
 */
int HorizonVisionAllocSegmentation(HorizonVisionSegmentation **pnew_farray);

/**
 * \~Chinese @brief 复制分割结构体，复制数据到另一个结构体，相当于赋值操作
 *
 * \~Chinese @param farray [in] 源结构体指针
 * \~Chinese @param new_farray [out] 新结构体的指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionCopySegmentation(HorizonVisionSegmentation *farray,
                                  HorizonVisionSegmentation *new_farray);

/**
 * \~Chinese @brief 复制分割结构体，构造新的结构体并复制内容过来，然后返回新的结构体指针
 *
 * \~Chinese @param farray [in] 源结构体指针
 * \~Chinese @param pnew_farray [out] 用来存储新构建的结构体指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionDupSegmentation(HorizonVisionSegmentation *farray,
                                 HorizonVisionSegmentation **pnew_farray);

/**
 * \~Chinese @brief 清空分割结构体内部指针数据，但保留结构体本身
 *
 * \~Chinese @param farray [in] 要清空的数据结构体指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果values不为空，则会一并释放该资源
 */
int HorizonVisionCleanSegmentation(HorizonVisionSegmentation *farray);

/**
 * \~Chinese @brief 释放分割结构体
 *
 * \~Chinese @param farray [in] 结构体指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionFreeSegmentation(HorizonVisionSegmentation *farray);

/**
 * \~Chinese @brief 申请smart data
 *
 * \~Chinese @param psmart [out] 用来存储新申请的smart data指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note landmarks 及 feature 需单独申请
 */
int HorizonVisionAllocFaceSmartData(HorizonVisionFaceSmartData **psmart);

/**
 * \~Chinese @brief 复制 smart data，复制数据到另一个结构体，相当于赋值操作
 *
 * \~Chinese @param smart [in] smart data 指针存储地址
 * \~Chinese @param new_smart [out] 新的smart data的地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionCopyFaceSmartData(HorizonVisionFaceSmartData *smart_data,
                                   HorizonVisionFaceSmartData *new_smart);

/**
 * \~Chinese @brief 复制 smart data
 *
 * \~Chinese @param smart [in] smart data 指针存储地址
 * \~Chinese @param pnew_smart [out] 用于存储新的smart data的地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionDupFaceSmartData(HorizonVisionFaceSmartData *smart_data,
                                  HorizonVisionFaceSmartData **pnew_smart);

/**
 * \~Chinese @brief 释放smart data内部指针数据，但保留结构体本身
 *
 * \~Chinese @param smart [in] smart data指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果landmarks 或 feature不为空，则会一并释放该资源
 */
int HorizonVisionCleanFaceSmartData(HorizonVisionFaceSmartData *smart);

/**
 * \~Chinese @brief 释放smart data
 *
 * \~Chinese @param smart [in] smart data指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果landmarks 或 feature不为空，则会一并释放该资源
 */
int HorizonVisionFreeFaceSmartData(HorizonVisionFaceSmartData *smart);

/**
 * \~Chinese @brief 申请smart data
 *
 * \~Chinese @param psmart [out] 用来存储新申请的smart data指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note skeleton 及 segmentation 资源需单独申请
 */
int HorizonVisionAllocBodySmartData(HorizonVisionBodySmartData **psmart);

/**
 * \~Chinese @brief 复制 smart data，复制数据到另一个结构体，相当于赋值操作
 *
 * \~Chinese @param smart [in] smart data 指针存储地址
 * \~Chinese @param new_smart [out] 新的smart data的地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionCopyBodySmartData(HorizonVisionBodySmartData *smart_data,
                                   HorizonVisionBodySmartData *new_smart);

/**
 * \~Chinese @brief 复制 smart data，构造新的结构体并复制内容过来，然后返回新的结构体指针
 *
 * \~Chinese @param smart [in] smart data 指针存储地址
 * \~Chinese @param pnew_smart [out] 用于存储新的smart data的地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionDupBodySmartData(HorizonVisionBodySmartData *smart_data,
                                  HorizonVisionBodySmartData **pnew_smart);

/**
 * \~Chinese @brief 清空smart data内部数据，但保留smart data本身
 *
 * \~Chinese @param smart [in] smart data指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果skeleton 或 segmentation不为空，则会一并释放该资源
 */
int HorizonVisionCleanBodySmartData(HorizonVisionBodySmartData *smart);

/**
 * \~Chinese @brief 释放smart data
 *
 * \~Chinese @param smart [in] smart data指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionFreeBodySmartData(HorizonVisionBodySmartData *smart);

/**
 * \~Chinese @brief 申请smart data
 *
 * \~Chinese @param psmart [out] 用来存储新申请的smart data指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note face 及 body 资源需单独申请
 */
int HorizonVisionAllocSmartData(HorizonVisionSmartData **psmart, int num);

/**
 * \~Chinese @brief 复制 smart data，复制数据到另一个结构体，相当于赋值操作
 *
 * \~Chinese @param smart [in] smart data 指针存储地址
 * \~Chinese @param new_smart [out] 目标smart data
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionCopySmartData(HorizonVisionSmartData *smart_data,
                               HorizonVisionSmartData *new_smart);

/**
 * \~Chinese @brief 复制 smart data，构造新的结构体并复制内容过来，然后返回新的结构体指针
 *
 * \~Chinese @param smart [in] smart data 指针存储地址
 * \~Chinese @param pnew_smart [out] 用于存储新的smart data的地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionDupSmartData(HorizonVisionSmartData *smart_data,
                              HorizonVisionSmartData **pnew_smart);

/**
 * \~Chinese @brief 清空smart data内部数据，但保留smart data本身
 *
 * \~Chinese @param smart [in] smart data指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果face 或 body不为空，则会一并释放该资源
 */
int HorizonVisionCleanSmartData(HorizonVisionSmartData *smart);

/**
 * \~Chinese @brief 释放smart data
 *
 * \~Chinese @param smart [in] smart data指针
 * \~Chinese @param num [in] smart data数量
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果face 或 body不为空，则会一并释放该资源
 */
int HorizonVisionFreeSmartData(HorizonVisionSmartData *smart, int num);

/**
 * \~Chinese @brief 创建抓拍数组
 *
 * \~Chinese @param psnaps [in] 指向抓拍数组地址的指针
 * \~Chinese @param num  抓拍图数组大小
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 每个抓拍的croped_image和smart_data需单独申请
 */
int HorizonVisionAllocSnapshot(HorizonVisionSnapshot **psnaps, int num);

/**
 * \~Chinese @brief 复制 snapshot，复制数据到另一个结构体，相当于赋值操作
 *
 * \~Chinese @param snapshot [in] snapshot指针存储地址
 * \~Chinese @param new_snapshot [out] 目标snapshot
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionCopySnapshot(HorizonVisionSnapshot *snapshot,
                              HorizonVisionSnapshot *new_snapshot);

/**
 * \~Chinese @brief 复制 smart snapshot，构造新的结构体并复制内容过来，然后返回新的结构体指针
 * **此处会复制抓拍图像帧数据
 *
 * \~Chinese @param snapshot [in] snapshot指针存储地址
 * \~Chinese @param pnew_snapshot [out] new_snapshot的地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionDupSnapshot(HorizonVisionSnapshot *snapshot,
                             HorizonVisionSnapshot **pnew_snapshot);

/**
 * \~Chinese @brief 清空单个snapshot 内部数据，但保留snapshot本身
 * **此处会复制抓拍图像帧数据
 *
 * \~Chinese @param snap [in] 抓拍指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果croped_image或smart_data不为空，则会一并释放该资源
 */
int HorizonVisionCleanSnapshot(HorizonVisionSnapshot *snap);

/**
 * \~Chinese @brief 释放抓拍数组
 *
 * \~Chinese @param snap [in] 抓拍指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果croped_image或smart_data不为空，则会一并释放该资源
 */
int HorizonVisionFreeSnapshot(HorizonVisionSnapshot *snap, int num);

/**
 * \~Chinese @brief 申请创建图像
 * \~Chinese @param pimg [out] 用来保存图像指针的地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note data字段需单独申请
 */
int HorizonVisionAllocImage(HorizonVisionImage **pimg);

/**
 * \~Chinese @brief 复制 image，复制数据到另一个结构体，相当于赋值操作
 *
 * \~Chinese @param image [in] image
 * \~Chinese @param copy_image_data [in] 是否拷贝图像数据
 * \~Chinese @param new_image [out] 目标image
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionCopyImage(HorizonVisionImage *image,
                           bool copy_image_data,
                           HorizonVisionImage *new_image);

/**
 * \~Chinese @brief 复制 image，构造新的结构体并复制内容过来，然后返回新的结构体指针
 *
 * \~Chinese @param image [in] image指针存储地址
 * \~Chinese @param dup_image_data [in] 是否拷贝图像数据
 * \~Chinese @param pnew_image [out] image的地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionDupImage(HorizonVisionImage *image,
                          bool dup_image_data,
                          HorizonVisionImage **pnew_image);

/**
 * \~Chinese @brief 清空图像中的数据
 *
 * \~Chinese @param img [in] 指向要清空的图像
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果data字段不为空，则会一并释放该资源
 */
int HorizonVisionCleanImage(HorizonVisionImage *img);

/**
 * \~Chinese @brief 清空图像中的数据,但是不释放图像的data
 *
 * \~Chinese @param img [in] 指向要清空的图像
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果data字段不为空，也不会释放该内存
 */
int HorizonVisionCleanImageWithoutData(HorizonVisionImage *img);

/**
 * \~Chinese @brief 释放图像指针
 *
 * \~Chinese @param img [in] 图像指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果data字段不为空，则会一并释放该资源
 */
int HorizonVisionFreeImage(HorizonVisionImage *img);

/**
 * \~Chinese @brief 释放图像指针，但是不释放图像的data
 *
 * \~Chinese @param img [in] 图像指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果data字段不为空，也不会释放该内存
 */
int HorizonVisionFreeImageWithoutData(HorizonVisionImage *img);

/**
 * \~Chinese @brief 申请创建1个图像帧
 *
 * \~Chinese @param pimage [in] image frame 指针存储地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionAllocImageFrame(HorizonVisionImageFrame **pimage);
/**
 * \~Chinese @brief 申请创建连续的多个图像帧
 *
 * \~Chinese @param pimage [in] image frame 指针数组的存储地址
 * \~Chinese @param frame_num [in] 图像帧数目
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionAllocImageFrames(HorizonVisionImageFrame ***pimage,
                                  uint32_t frame_num);
/**
 * \~Chinese @brief 复制 image frame，复制数据到另一个结构体，相当于赋值操作
 *
 * \~Chinese @param image_frame [in] image_frame
 * \~Chinese @param copy_image_data [in] 是否拷贝图像数据
 * \~Chinese @param new_image_frame [out] 目标image_frame
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionCopyImageFrame(HorizonVisionImageFrame *image_frame,
                                bool copy_image_data,
                                HorizonVisionImageFrame *new_image_frame);

/**
 * \~Chinese @brief 复制一个image frame，构造新的结构体并复制内容过来，然后返回新的结构体指针
 *
 * \~Chinese @param image_frame [in] image_frame指针存储地址
 * \~Chinese @param dup_image_data [in] 是否拷贝图像数据
 * \~Chinese @param pnew_image_frame [out] image_frame的地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionDupImageFrame(HorizonVisionImageFrame *image_frame,
                               bool dup_image_data,
                               HorizonVisionImageFrame **pnew_image_frame);
/**
 * \~Chinese @brief 复制多个image frame，内部对图像数据为深拷贝
 * \~Chinese @param image_frame [in] image_frame指针数组
 * \~Chinese @param pnew_image_frame [out] 指向新的image_frame指针数组的地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionDupImageFrames(HorizonVisionImageFrame **image_frame,
                                uint32_t image_num,
                                HorizonVisionImageFrame ***pnew_image_frame);
/**
 * \~Chinese @brief 复制多个image frame，内部对图像数据为浅拷贝
 * \~Chinese @param image_frame [in] image_frame指针数组
 * \~Chinese @param pnew_image_frame [out] 指向新的image_frame指针数组的地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 * */
int HorizonVisionDupImageFramesWithoutData(HorizonVisionImageFrame **image_frame,
                                           uint32_t image_num,
                                           HorizonVisionImageFrame ***pnew_image_frame);
/**
 * \~Chinese @brief 释放图像帧资源
 *
 * \~Chinese @param image [in] image frame 指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionFreeImageFrame(HorizonVisionImageFrame *image);

/**
 * \~Chinese @brief 释放多个图像帧资源
 *
 * \~Chinese @param image [in] image frame 指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionFreeImageFrames(HorizonVisionImageFrame **images,
                                 uint32_t image_num);

/**
 * \~Chinese @brief 释放多个图像帧资源,但不释放data。
 *
 * \~Chinese @param image [in] image frame 指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionFreeImageFramesWithoutData(HorizonVisionImageFrame **images,
                                            uint32_t image_num);

/**
 * \~Chinese @brief 释放图像帧资源，但是不释放图片data本身
 *
 * \~Chinese @param image [in] image frame 指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionFreeImageFrameWithoutData(HorizonVisionImageFrame *image);

/**
 * \~Chinese @brief 申请创建智能帧
 *
 * \~Chinese @param psmart [in] 智能帧指针存储地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note smart_data_list 资源需单独申请
 */
int HorizonVisionAllocSmartFrame(HorizonVisionSmartFrame **psmart);

/**
 * \~Chinese @brief 复制 smart frame，复制数据到另一个结构体，相当于赋值操作
 * **此处会深拷贝原始图像帧数据
 *
 * \~Chinese @param smart_frame [in] smart_frame指针存储地址
 * \~Chinese @param new_smart_frame [out] 目标smart_frame
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionCopySmartFrame(HorizonVisionSmartFrame *smart_frame,
                                HorizonVisionSmartFrame *new_smart_frame);

/**
 * \~Chinese @brief 复制 smart frame，复制数据到另一个结构体，相当于赋值操作
 * **此处不会深拷贝原始图像帧数据
 *
 * \~Chinese @param smart_frame [in] smart_frame指针存储地址
 * \~Chinese @param new_smart_frame [out] 目标smart_frame
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionCopySmartFrameWithoutData(HorizonVisionSmartFrame *smart_frame,
                                           HorizonVisionSmartFrame *new_smart_frame);

/**
 * \~Chinese @brief 复制 smart frame，构造新的结构体并复制内容过来，然后返回新的结构体指针
 * **此处会深拷贝原始图像帧数据
 *
 * \~Chinese @param smart_frame [in] smart_frame指针存储地址
 * \~Chinese @param pnew_smart_frame [out] smart_frame的地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionDupSmartFrame(HorizonVisionSmartFrame *smart_frame,
                               HorizonVisionSmartFrame **pnew_smart_frame);

/**
 * \~Chinese @brief 复制 smart frame，构造新的结构体并复制内容过来，然后返回新的结构体指针
 * **此处不会深拷贝原始图像帧数据
 *
 * \~Chinese @param smart_frame [in] smart_frame指针存储地址
 * \~Chinese @param pnew_smart_frame [out] smart_frame的地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionDupSmartFrameWithoutData(HorizonVisionSmartFrame *smart_frame,
                                          HorizonVisionSmartFrame **pnew_smart_frame);

/**
 * \~Chinese @brief 释放智能帧资源及图像帧data
 *
 * \~Chinese @param smart [in] 指向智能帧资源的指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果smart_data_list不为空，会一并释放该资源
 * \~Chinese @note 如果image_frame不为空，不会释放该资源
 */
int HorizonVisionFreeSmartFrame(HorizonVisionSmartFrame *smart);

/**
 * \~Chinese @brief 释放智能帧资源，并且释放image_frame，但是不释放图像帧data
 *
 * \~Chinese @param smart [in] 指向智能帧资源的指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果smart_data_list不为空，会一并释放该资源
 * \~Chinese @note 如果image_frame不为空，会释放该资源
 * \~Chinese @note 如果image_frame[i]不为空，会释放该资源
 * \~Chinese @note 如果image_frame[i]->image.data不为空，不会释放该资源
 */
int HorizonVisionFreeSmartFrameWithoutData(HorizonVisionSmartFrame *smart);

/**
 * \~Chinese @brief 申请创建抓拍目标列表
 *
 * \~Chinese @param ptargets [in] 指向HorizonVisionSnapshotTarget列表地址的指针
 * \~Chinese @param num [in] 抓拍目标数量
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 每个抓拍目标的snapshots资源需单独申请
 */
int HorizonVisionAllocSnapshotTarget(HorizonVisionSnapshotTarget **ptargets,
                                     int num);

/**
 * \~Chinese @brief 复制 snapshot target，复制数据到另一个结构体，相当于赋值操作
 * **此处会复制抓拍图像帧数据
 *
 * \~Chinese @param target [in] snapshot target 指针存储地址
 * \~Chinese @param new_target [out] 目标snapshot target
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionCopySnapshotTarget(HorizonVisionSnapshotTarget *target,
                                    HorizonVisionSnapshotTarget *new_target);

/**
 * \~Chinese @brief 复制 snapshot target，构造新的结构体并复制内容过来，然后返回新的结构体指针
 * **此处会复制抓拍图像帧数据
 *
 * \~Chinese @param target [in] snapshot target 指针存储地址
 * \~Chinese @param pnew_target [out] 用于存储新的snapshot target的地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionDupSnapshotTarget(HorizonVisionSnapshotTarget *target,
                                   HorizonVisionSnapshotTarget **pnew_target);

/**
 * \~Chinese @brief 清空单个抓拍目标，但不释放HorizonVisionSnapshotTarget本身
 *
 * \~Chinese @param targets [in] HorizonVisionSnapshotTarget地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果该目标snapshots不为空，则会一并释放抓拍资源
 */
int HorizonVisionCleanSnapshotTarget(HorizonVisionSnapshotTarget *targets);

/**
 * \~Chinese @brief 释放抓拍目标列表
 *
 * \~Chinese @param targets [in] HorizonVisionSnapshotTarget列表地址
 * \~Chinese @param num [in] 抓拍目标数量
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 对于每个抓拍目标，如果snapshots不为空，则会一并释放抓拍资源
 */
int HorizonVisionFreeSnapshotTarget(HorizonVisionSnapshotTarget *targets,
                                    int num);

/**
 * \~Chinese @brief 申请创建抓拍帧资源
 *
 * \~Chinese @param psnapshot [in] snapshot list 指针存储地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note targets资源需单独申请
 */
int HorizonVisionAllocSnapshotFrame(HorizonVisionSnapshotFrame **psnapshots);

/**
 * \~Chinese @brief 复制 snapshot frame，复制数据到另一个结构体，相当于赋值操作
 * **此处会复制抓拍图像帧数据
 *
 * \~Chinese @param snapshots [in] snapshot frame 指针存储地址
 * \~Chinese @param new_snapshots [out] 目标snapshot frame
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionCopySnapshotFrame(HorizonVisionSnapshotFrame *snapshots,
                                   HorizonVisionSnapshotFrame *new_snapshots);

/**
 * \~Chinese @brief 复制 snapshot frame，构造新的结构体并复制内容过来，然后返回新的结构体指针
 * **此处会复制抓拍图像帧数据
 *
 * \~Chinese @param snapshots [in] snapshot frame 指针存储地址
 * \~Chinese @param pnew_snapshots [out] 用于存储新的snapshot frame的地址
 * \~Chinese @return int 0表示成功，<0则为错误码
 */
int HorizonVisionDupSnapshotFrame(HorizonVisionSnapshotFrame *snapshots,
                                  HorizonVisionSnapshotFrame **pnew_snapshots);

/**
 * \~Chinese @brief 清空抓拍帧内容，但不释放抓拍帧本身
 *
 * \~Chinese @param snapshots [in] snapshot list 指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果targets不为空，会一并释放该资源
 */
int HorizonVisionCleanSnapshotFrame(HorizonVisionSnapshotFrame *snapshots);

/**
 * \~Chinese @brief 释放 snapshot 列表
 *
 * \~Chinese @param snapshots [in] snapshot list 指针
 * \~Chinese @return int 0表示成功，<0则为错误码
 * \~Chinese @note 如果targets不为空，会一并释放该资源
 */
int HorizonVisionFreeSnapshotFrame(HorizonVisionSnapshotFrame *snapshots);

#ifdef __cplusplus
}
#endif

#endif  // VISION_TYPE_VISION_TYPE_UTIL_H_
