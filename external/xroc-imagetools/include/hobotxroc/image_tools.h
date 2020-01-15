/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     提供了c接口版本的图像处理接口头文件，具体包括：图像解码
 *            图像格式转换、图像缩放、抠图、padding、镜像翻转、中心旋转等功能
 * @author    hangjun.yang
 * @email     hangjun.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.07
 */
#ifndef INCLUDE_HOBOTXROC_IMAGE_TOOLS_H_
#define INCLUDE_HOBOTXROC_IMAGE_TOOLS_H_
#include <stdint.h>

#ifdef __GNUC__
#define IMAGETOOLS_EXPORT __attribute__ ((visibility("default")))
#endif

#ifdef __cplusplus
#define XROC_API  extern "C" IMAGETOOLS_EXPORT
#else
#define XROC_API IMAGETOOLS_EXPORT
#endif

enum HobotXRocImageToolsPixelFormat {
  IMAGE_TOOLS_RAW_NONE,
  IMAGE_TOOLS_RAW_RGB,
  IMAGE_TOOLS_RAW_BGR,
  IMAGE_TOOLS_RAW_GRAY,
  IMAGE_TOOLS_RAW_YUV_NV21,
  IMAGE_TOOLS_RAW_YUV_NV12,
  IMAGE_TOOLS_RAW_YUV_I420,
  IMAGE_TOOLS_RAW_YUV_444
};

struct HobotXRocImageToolsResizeInfo {
  int src_width_;            // 图像原始宽度
  int src_height_;
  // dst_width_ = src_width_ * width_ratio_ + padding_right_
  int dst_width_;            // 缩放之后的最终大小
  int dst_height_;
  float width_ratio_;        // 原始图像缩放的尺寸
  float height_ratio_;
  int padding_right_;        // 图片右边加了多少padding区域
  int padding_bottom_;
  int fix_aspect_ratio_;    // 宽高比是否固定
};

/**
 * @brief 分配图像内存区域
 * @param image_size 图像区域的大小， 要求image_size > 0
 * @param output_image 分配的图像区域，要求output_image != NULl
 * @return: 成功返回0，失败返回-1
 */
XROC_API int HobotXRocAllocImage(const int image_size, uint8_t** output_image);

/**
 * @brief 释放图像接口返回的内存
 * @param input: 指针，由HobotXRocDecodeImage/HobotXRocConvertImage
 *                /HobotXRocResizeImage
 *                /HobotXRocCropImage
 *                /HobotXRocPadImage等接口返回的输出数据的地址。
 * @return: 成功返回0，失败返回-1
 */
XROC_API int HobotXRocFreeImage(const uint8_t* input);

/**
 * @brief 解码图片至dst_fmt格式。接口内部分配输出的内存空间\
 *        由调用者调用HobotXRocFreeImage接口释放.
 * 注意：若原图的宽与高存在奇数，且要求解码后的图像是YUV420系列，则解码会失败!
 * @param input: 输入图片二进制数据，支持jpg/png/bmp
 * @param input_size: 输入图片二进制数据长度
 * @param dst_fmt： 解码后的图像格式
 * @param output: 解码后的图片二进制数据地址
 * @param output_size: 解码后的图片二进制数据长度
 * @param width: 图像宽度
 * @param height: 图像高度
 * @param first_stride: rgb/bgr/gray/y的stride
 * @param second_stride: uv的stride
 * @return: 成功返回0，失败返回-1
 */
XROC_API int HobotXRocDecodeImage(\
                            const uint8_t *input,
                            const int input_size,
                            const enum HobotXRocImageToolsPixelFormat dst_fmt,
                            uint8_t **output,
                            int *output_size,
                            int *width,
                            int *height,
                            int *first_stride,
                            int *second_stride);

/**
 * @brief 图像转码。接口内部分配输出的内存空间\
 *        由调用者调用HobotXRocFreeImage接口释放.图像分辨率不变
 *  注意：若转换成YUV420系列，则要求图片分辨率的高与宽均为偶数，否则转码失败
 * @param input: 输入图片二进制数据
 * @param input_size: 输入图片二进制数据长度
 * @param width: 图像宽度
 * @param height: 图像高度
 * @param input_first_stride: rgb/bgr/gray/y的stride
 * @param input_second_stride: uv的stride
 * @param input_fmt: 输入图像的格式
 * @param output_fmt: 转码后图像的格式
 * @param output: 转码后的图片二进制数据地址
 * @param output_size: 转码后的图片二进制数据长度
 * @param output_first_stride: rgb/bgr/gray/y的stride
 * @param output_second_stride: uv的stride
 * @return: 成功返回0，失败返回-1
 */
XROC_API int HobotXRocConvertImage(\
                          const uint8_t *input,
                          const int input_size,
                          const int width,
                          const int height,
                          const int input_first_stride,
                          const int input_second_stride,
                          const enum HobotXRocImageToolsPixelFormat input_fmt,
                          const enum HobotXRocImageToolsPixelFormat output_fmt,
                          uint8_t **output,
                          int *output_size,
                          int *output_first_stride,
                          int *output_second_stride);

/**
 * @brief 根据目标尺寸缩放图像\
 *        由调用者调用HobotXRocFreeImage接口释放.图像格式不变
 *  注意：若图像格式为YUV420系列，则要求缩放前后图像分辨率的高与宽均为偶数，
 *       否则缩放失败
 * @param input: 输入图片二进制数据
 * @param input_size: 输入图片二进制数据长度
 * @param width: 图像宽度
 * @param height: 图像高度
 * @param input_first_stride: rgb/bgr/gray/y的stride
 * @param input_second_stride: uv的stride
 * @param format: 图像的格式
 * @param fix_aspect_ratio: 1为保持宽高比。0为直接缩放
 *              当保持宽高比时，会在图像右侧与下方填充黑色。
 * @param dst_width: 目标图片的宽
 * @param dst_height: 目标图片的高
 * @param output: 缩放后的图片二进制数据地址
 * @param output_size: 缩放后的图片二进制数据长度
 * @param output_first_stride: rgb/bgr/gray/y的stride
 * @param output_second_stride: uv的stride
 * @param resize_info: 缩放参数
 * @return: 成功返回0，失败返回-1
 */
XROC_API int HobotXRocResizeImage(\
                         const uint8_t *input,
                         const int input_size,
                         const int input_width,
                         const int input_height,
                         const int input_first_stride,
                         const int input_second_stride,
                         const enum HobotXRocImageToolsPixelFormat format,
                         const int fix_aspect_ratio,
                         const int dst_width,
                         const int dst_height,
                         uint8_t **output,
                         int *output_size,
                         int *output_first_stride,
                         int *output_second_stride,
                         struct HobotXRocImageToolsResizeInfo *resize_info);

/**
 * @brief 根据缩放因子缩放图像\
 *        由调用者调用HobotXRocFreeImage接口释放.图像格式不变
 *  注意：若图像格式为YUV420系列，则要求缩放前后图像分辨率的高与宽均为偶数，
 *       否则缩放失败
 * @param input: 输入图片二进制数据
 * @param input_size: 输入图片二进制数据长度
 * @param width: 图像宽度
 * @param height: 图像高度
 * @param input_first_stride: rgb/bgr/gray/y的stride
 * @param input_second_stride: uv的stride
 * @param format: 图像的格式
 * @param fix_aspect_ratio: 1为保持宽高比。0为直接缩放
 *              当保持宽高比时，会在图像右侧与下方填充黑色。
 * @param width_ratio: 宽的缩放因子, width_ratio = dst_width / input_width
 * @param height_ratio: 
 * @param output: 缩放后的图片二进制数据地址
 * @param output_size: 缩放后的图片二进制数据长度
 * @param output_first_stride: rgb/bgr/gray/y的stride
 * @param output_second_stride: uv的stride
 * @param resize_info: 缩放参数
 * @return: 成功返回0，失败返回-1
 */
XROC_API int HobotXRocResizeImageByRatio(\
                         const uint8_t *input,
                         const int input_size,
                         const int input_width,
                         const int input_height,
                         const int input_first_stride,
                         const int input_second_stride,
                         const enum HobotXRocImageToolsPixelFormat format,
                         const int fix_aspect_ratio,
                         const float width_ratio,
                         const float height_ratio,
                         uint8_t **output,
                         int *output_size,
                         int *output_first_stride,
                         int *output_second_stride,
                         struct HobotXRocImageToolsResizeInfo *resize_info);

/**
 * @brief 抠图\
 *        由调用者调用HobotXRocFreeImage接口释放.图像格式不变
 *  注意：若图像格式为YUV420系列，则要求抠图区域的宽与高均为偶数，
 *       否则抠图失败
 * @param input: 输入图片二进制数据
 * @param input_size: 输入图片二进制数据长度
 * @param input_width: 输入图片的宽
 * @param input_height: 输入图片的高
 * @param input_first_stride: rgb/bgr/gray/y的stride
 * @param input_second_stride: uv的stride
 * @param format: 图像的格式
 * @param top_left_x: 要求>= 0
 * @param top_left_y: 要求>= 0
 * @param bottom_right_x: 要求>= 0
 * @param bottom_right_y: 要求>= 0
 * @param output: 抠图的图片二进制数据地址
 * @param output_size: 抠图的图片二进制数据长度
 * @param output_width: 抠图的宽
 * @param output_height: 抠图的高
 * @param output_first_stride: rgb/bgr/gray/y的stride
 * @param output_second_stride: uv的stride
 * @return: 成功返回0，失败返回-1
 */
XROC_API int HobotXRocCropImage(\
                       const uint8_t *input,
                       const int input_size,
                       const int input_width,
                       const int input_height,
                       const int input_first_stride,
                       const int input_second_stride,
                       const enum HobotXRocImageToolsPixelFormat format,
                       const int top_left_x,
                       const int top_left_y,
                       const int bottom_right_x,
                       const int bottom_right_y,
                       uint8_t **output,
                       int *output_size,
                       int *output_width,
                       int *output_height,
                       int *output_first_stride,
                       int *output_second_stride);

/**
 * @brief 抠图\
 *        由调用者调用HobotXRocFreeImage接口释放.图像格式不变
 *        若抠图区域超过图像边界，则超过区域补黑色
 *  注意：若图像格式为YUV420系列，则要求抠图区域的宽与高均为偶数，且yuv存储空间需连续
 *       否则抠图失败
 * @param input: 输入图片二进制数据
 * @param input_size: 输入图片二进制数据长度
 * @param input_width: 输入图片的宽
 * @param input_height: 输入图片的高
 * @param input_first_stride: rgb/bgr/gray/y的stride
 * @param input_second_stride: uv的stride
 * @param format: 图像的格式
 * @param top_left_x: 
 * @param top_left_y: 
 * @param bottom_right_x: 要求> top_left_x
 * @param bottom_right_y: 要求> top_left_y
 * @param output: 抠图的图片二进制数据地址
 * @param output_size: 抠图的图片二进制数据长度
 * @param output_width: 抠图的宽
 * @param output_height: 抠图的高
 * @param output_first_stride: rgb/bgr/gray/y的stride
 * @param output_second_stride: uv的stride
 * @return: 成功返回0，失败返回-1
 */
XROC_API int HobotXRocCropImageWithPaddingBlack(\
                       const uint8_t *input,
                       const int input_size,
                       const int input_width,
                       const int input_height,
                       const int input_first_stride,
                       const int input_second_stride,
                       const enum HobotXRocImageToolsPixelFormat format,
                       const int top_left_x,
                       const int top_left_y,
                       const int bottom_right_x,
                       const int bottom_right_y,
                       uint8_t **output,
                       int *output_size,
                       int *output_width,
                       int *output_height,
                       int *output_first_stride,
                       int *output_second_stride);

/**
 * @brief 对YUV420图像抠图（y、u、v存储空间可能不连续的情况）\
 *        由调用者调用HobotXRocFreeImage接口释放.图像格式不变
 *        若抠图区域超过图像边界，则超过区域补黑色
 *        要求抠图区域的宽与高均为偶数，否则抠图失败
 * @param input_yuv_data: 输入图片二进制数据，[0]固定为y分量的地址，
 *                        对于nv12/nv21，[1]为uv的地址，[2]无效
 *                        对于i420，[1]固定为u的地址，[2]固定为v的地址
 * @param input_yuv_size: 输入图片二进制数据长度，[0]固定为y分量的长度，
 *                        对于nv12/nv21，[1]为uv分量的长度，[2]无效
 *                        对于i420，[1]固定为u的长度，[2]固定为v的长度
 * @param input_width: 输入图片的宽
 * @param input_height: 输入图片的高
 * @param input_first_stride: y的stride
 * @param input_second_stride: uv的stride
 * @param format: 图像的格式
 * @param top_left_x: 
 * @param top_left_y: 
 * @param bottom_right_x: 要求> top_left_x
 * @param bottom_right_y: 要求> top_left_y
 * @param output: 抠图的图片二进制数据地址
 * @param output_size: 抠图的图片二进制数据长度
 * @param output_width: 抠图的宽
 * @param output_height: 抠图的高
 * @param output_first_stride: y的stride
 * @param output_second_stride: uv的stride
 * @return: 成功返回0，失败返回-1
 */
XROC_API int HobotXRocCropYuvImageWithPaddingBlack(\
                       const uint8_t *input_yuv_data[3],
                       const int input_yuv_size[3],
                       const int input_width,
                       const int input_height,
                       const int input_first_stride,
                       const int input_second_stride,
                       const enum HobotXRocImageToolsPixelFormat format,
                       const int top_left_x,
                       const int top_left_y,
                       const int bottom_right_x,
                       const int bottom_right_y,
                       uint8_t **output,
                       int *output_size,
                       int *output_width,
                       int *output_height,
                       int *output_first_stride,
                       int *output_second_stride);

/**
 * @brief 对vision_type中c++版本的ImageFrame进行抠图
 *        由调用者调用HobotXRocFreeImage接口释放.图像格式不变
 *        若抠图区域超过图像边界，则超过区域补黑色
 *        要求抠图区域的宽与高均为偶数，否则抠图失败
 * @param input: hobot::vision::ImageFrame
 * @param top_left_x: 
 * @param top_left_y: 
 * @param bottom_right_x: 要求> top_left_x
 * @param bottom_right_y: 要求> top_left_y
 * @param output: 抠图的图片二进制数据地址
 * @param output_size: 抠图的图片二进制数据长度
 * @param output_width: 抠图的宽
 * @param output_height: 抠图的高
 * @param output_first_stride:
 * @param output_second_stride:
 * @return: 成功返回0，失败返回-1
 */
XROC_API int HobotXRocCropImageFrameWithPaddingBlack(\
                       void *input,
                       const int top_left_x,
                       const int top_left_y,
                       const int bottom_right_x,
                       const int bottom_right_y,
                       enum HobotXRocImageToolsPixelFormat *output_format,
                       uint8_t **output,
                       int *output_size,
                       int *output_width,
                       int *output_height,
                       int *output_first_stride,
                       int *output_second_stride);

/**
 * @brief 图像四周填充元素，会导致图像分辨率变大。\
 *          由调用者调用HobotXRocFreeImage接口释放.图像格式不变
 *  注意：若图像格式为YUV420系列，则要求原图的宽、高、padding_left_width、
 *       padding_right_width、padding_top_height、padding_bottom_height
 *       均为偶数或者0，否则paddiing失败
 * @param input: 输入图片二进制数据
 * @param input_size: 输入图片二进制数据长度
 * @param input_width: 输入图片的宽
 * @param input_height: 输入图片的高
 * @param input_first_stride: rgb/bgr/gray/y的stride
 * @param input_second_stride: uv的stride
 * @param format: 图像的格式
 * @param padding_left_width: 要求 >= 0
 * @param padding_right_width:  要求 >= 0
 * @param padding_top_height: 要求 >= 0
 * @param padding_bottom_height: 要求 >= 0
 * @param padding_value: 填充的像素值,长度必须为3
 *    RGB/BGR格式，则padding_value统一为[r, g, b]
 *    GRay格式，则padding_value = [gray, xx, xx]
 *    I420/NV12/NV21，则padding_value统一为[y, u, v]
 * @param output: padding后图片二进制数据地址
 * @param output_size: padding后的图片二进制数据长度
* @param output_width: padding后的图片的宽
 * @param output_height: padding后的图片的高
 * @param output_first_stride: rgb/bgr/gray/y的stride
 * @param output_second_stride: uv的stride
 * @return: 成功返回0，失败返回-1
 */
XROC_API int HobotXRocPadImage(\
                      const uint8_t *input,
                      const int input_size,
                      const int input_width,
                      const int input_height,
                      const int input_first_stride,
                      const int input_second_stride,
                      const enum HobotXRocImageToolsPixelFormat format,
                      const int padding_left_width,
                      const int padding_right_width,
                      const int padding_top_height,
                      const int padding_bottom_height,
                      const uint8_t padding_value[3],
                      uint8_t **output,
                      int *output_size,
                      int *output_width,
                      int *output_height,
                      int *output_first_stride,
                      int *output_second_stride);

/**
 * @brief 对图像顺时针进行90/180/270度的旋转
 *        由调用者调用HobotXRocFreeImage接口释放.图像格式不变
 * @param input: 输入图片二进制数据
 * @param input_size: 输入图片二进制数据长度
 * @param width: 图像宽度
 * @param height: 图像高度
 * @param input_first_stride: rgb/bgr/gray/y的stride
 * @param input_second_stride: uv的stride
 * @param format: 图像的格式
 * @param degree: 图像旋转角度：90 180 270
 * @param output: 缩放后的图片二进制数据地址
 * @param output_size: 缩放后的图片二进制数据长度
 * @param output_width: 目标图片的宽
 * @param output_height: 目标图片的高
 * @param output_first_stride: rgb/bgr/gray/y的stride
 * @param output_second_stride: uv的stride
 * @param resize_info: 缩放参数
 * @return: 成功返回0，失败返回-1
 */
XROC_API int HobotXRocRotateImage(\
                         const uint8_t *input,
                         const int input_size,
                         const int input_width,
                         const int input_height,
                         const int input_first_stride,
                         const int input_second_stride,
                         const enum HobotXRocImageToolsPixelFormat format,
                         const int degree,
                         uint8_t **output,
                         int *output_size,
                         int *output_width,
                         int *output_height,
                         int *output_first_stride,
                         int *output_second_stride);

#endif  // INCLUDE_HOBOTXROC_IMAGE_TOOLS_H_


