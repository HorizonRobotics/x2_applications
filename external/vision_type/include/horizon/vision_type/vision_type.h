/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file vision_type.h
 * \~English @brief this c header file defines the vision related data structure that are
 * used in IOT, including face, head
 * @date 2019/4/3
 */

#ifndef VISION_TYPE_VISION_TYPE_H_
#define VISION_TYPE_VISION_TYPE_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include "horizon/vision_type/vision_type_common.h"

typedef uint32_t HorizonVisionTrackID;
typedef uint64_t HorizonVisionTimeStamp;
typedef uint64_t HorizonVisionFrameID;

/**
 * \~Chinese @brief 图片
 */
typedef struct HorizonVisionImage_ {
  /// \~Chinese 图片编码方式
  HorizonVisionPixelFormat pixel_format;
  /// \~Chinese 图片
  uint8_t *data;
  /// \~Chinese 图片大小
  uint32_t data_size;
  /// \~Chinese 宽度
  uint32_t width;
  /// \~Chinese 高度
  uint32_t height;
  /// \~Chinese 长度
  uint32_t stride;
  /// \~Chinese uv长度
  uint32_t stride_uv;
} HorizonVisionImage;
/**
 * \~Chinese @brief 图片帧
 */
typedef struct HorizonVisionImageFrame_ {
  /// \~Chinese 图片
  HorizonVisionImage image;
  /// \~Chinese 通道号
  uint32_t channel_id;
  /// \~Chinese 时间戳
  HorizonVisionTimeStamp time_stamp;
  /// \~Chinese 帧号
  HorizonVisionFrameID frame_id;
} HorizonVisionImageFrame;
/**
 * \~Chinese @brief 2D坐标点
 */
typedef struct HorizonVisionPoint_ {
  /// \~Chinese x坐标
  float x;
  /// \~Chinese y坐标
  float y;
  /// \~Chinese 置信度
  float score;
} HorizonVisionPoint;
/**
 * \~Chinese @brief 3D 坐标点
 */
typedef struct HorizonVisionPoint3d_ {
  /// \~Chinese x坐标
  float x;
  /// \~Chinese y坐标
  float y;
  /// \~Chinese z坐标
  float z;
  /// \~Chinese 置信度
  float score;
} HorizonVisionPoint3d;
/**
 * \~Chinese @brief 2D 坐标点集合
 */
typedef struct HorizonVisionPoints_ {
  /// \~Chinese 点数目
  size_t num;
  /// \~Chinese 指向2D 坐标点集合的指针
  HorizonVisionPoint *points;
  /// \~Chinese 置信度
  float score;
} HorizonVisionPoints;

/// \~Chinese 关键点
typedef HorizonVisionPoints HorizonVisionLandmarks;
/**
 * \~Chinese @brief 检测框
 */
typedef struct HorizonVisionBBox_ {
  /// \~Chinese 左上点x坐标
  float x1;
  /// \~Chinese 左上点y坐标
  float y1;
  /// \~Chinese 右下点x坐标
  float x2;
  /// \~Chinese 右下点y坐标
  float y2;
  /// \~Chinese 置信度
  float score;
  /// \~Chinese ID 号
  HorizonVisionTrackID id;
} HorizonVisionBBox;
/**
 * \~Chinese @brief 单精度浮点数组
 */
typedef struct HorizonVisionFloatArray_ {
  /// \~Chinese 数目
  size_t num;
  /// \~Chinese 指向单精度浮点数组数组的指针
  float *values;
} HorizonVisionFloatArray;

/// \~Chinese 人脸特征
typedef HorizonVisionFloatArray HorizonVisionFeature;

/**
 * \~Chinese @brief 字符数组，可用于存储加密后的特征值
 */
typedef struct HorizonVisionCharArray_ {
  /// \~Chinese 数目
  size_t num;
  /// \~Chinese 指向字符数组的指针
  char *values;
} HorizonVisionCharArray;

/// \~Chinese 人脸特征
typedef HorizonVisionCharArray HorizonVisionEncryptedFeature;

/**
 * \~Chinese @brief 人体分割
 */
typedef struct HorizonVisionSegmentation {
  /// \~Chinese 数目
  size_t num;
  /// \~Chinese 指向float数组的指针
  float *values;
  /// \~Chinese 区域宽度
  int32_t width;
  /// \~Chinese 区域高度
  int32_t height;
} HorizonVisionSegmentation;
/**
 * \~Chinese @brief 人脸姿态
 */
typedef struct HorizonVisionPose3D_ {
  /// \~Chinese 俯仰角度
  float pitch;
  /// \~Chinese 左右摇头角度
  float yaw;
  /// \~Chinese 侧头角度
  float roll;
  /// \~Chinese 置信度
  float score;
} HorizonVisionPose3D;
/**
 * \~Chinese @brief 年龄
 */
typedef struct HorizonVisionAge_ {
  /// \~Chinese 年龄分类
  int32_t value;
  /// \~Chinese 年龄段下限
  int32_t min;
  /// \~Chinese 年龄段上限
  int32_t max;
  /// \~Chinese 置信度
  float score;
} HorizonVisionAge;
/**
 * \~Chinese @brief 属性类检测结果
 */
typedef struct HorizonVisionAttribute_ {
  /// \~Chinese 值
  int32_t value;
  /// \~Chinese 置信度
  float score;
} HorizonVisionAttribute;

/// \~Chinese 年龄
typedef HorizonVisionAttribute HorizonVisionGender;
/// \~Chinese 眼镜
typedef HorizonVisionAttribute HorizonVisionGlass;
/// \~Chinese 口罩
typedef HorizonVisionAttribute HorizonVisionBreathingMask;
/// \~Chinese 图片质量
typedef HorizonVisionAttribute HorizonVisionQuality;
/// \~Chinese 活体
typedef HorizonVisionAttribute HorizonVisionAntiSpoofing;
/**
* \~English @brief HorizonVisionFaceQuality
* \~Chinese @brief 人脸质量
*/

typedef struct HorizonVisionFaceQuality_ {
  /// \~Chinese 人脸清晰度
  HorizonVisionQuality blur;
  /// \~Chinese 人脸亮度
  HorizonVisionQuality brightness;
  /// \~Chinese 眼睛表情
  HorizonVisionQuality eye_abnormalities;
  /// \~Chinese 嘴部
  HorizonVisionQuality mouth_abnormal;
  /// \~Chinese 左眼
  HorizonVisionQuality left_eye;
  /// \~Chinese 右眼
  HorizonVisionQuality right_eye;
  /// \~Chinese 左眉毛
  HorizonVisionQuality left_brow;
  /// \~Chinese 右眉毛
  HorizonVisionQuality right_brow;
  /// \~Chinese 额头
  HorizonVisionQuality forehead;
  /// \~Chinese 左脸颊
  HorizonVisionQuality left_cheek;
  /// \~Chinese 右脸颊
  HorizonVisionQuality right_cheek;
  /// \~Chinese 鼻子
  HorizonVisionQuality nose;
  /// \~Chinese 嘴部
  HorizonVisionQuality mouse;
  /// \~Chinese 下巴
  HorizonVisionQuality jaw;
} HorizonVisionFaceQuality;

#ifdef __cplusplus
}
#endif
#endif  // VISION_TYPE_VISION_TYPE_H_
