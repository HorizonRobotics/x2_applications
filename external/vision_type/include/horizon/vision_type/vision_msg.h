/**
* Copyright (c) 2019 Horizon Robotics. All rights reserved.
* @file vision_msg.h
* \~English @brief this c header file defines the vision related smart and snapshot message that are used in
 * IOT, including face, head
* @date 2019/4/3
*/

#ifndef VISION_TYPE_VISION_MSG_H_
#define VISION_TYPE_VISION_MSG_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "horizon/vision_type/vision_type.h"

/**
 * \~English @brief FaceSmartData
 * \~Chinese @brief 人脸智能信息
 */

typedef struct HorizonVisionFaceSmartData_ {
  /// \~Chinese 跟踪ID
  HorizonVisionTrackID track_id;
  /// \~Chinese 人脸框
  HorizonVisionBBox face_rect;
  /// \~Chinese 人头框
  HorizonVisionBBox head_rect;
  /// \~Chinese 人脸姿态
  HorizonVisionPose3D pose3d;
  /// \~Chinese 人脸关键点
  HorizonVisionLandmarks *landmarks;
  /// \~Chinese 年龄
  HorizonVisionAge age;
  /// \~Chinese 性别
  HorizonVisionGender gender;
  /// \~Chinese 眼镜
  HorizonVisionGlass glass;
  /// \~Chinese 口罩
  HorizonVisionBreathingMask mask;
  /// \~Chinese 活体信息
  HorizonVisionAntiSpoofing anti_spoofing;
  /// \~Chinese 人脸质量
  HorizonVisionFaceQuality quality;
  /// \~Chinese 人脸特征
  HorizonVisionFeature *feature;
  /// \~Chinese 加密后的人脸特征
  HorizonVisionEncryptedFeature *encrypted_feature;
} HorizonVisionFaceSmartData;
/**
 * \~Chinese @brief 人体智能信息
 */
typedef struct HorizonVisionBodySmartData_ {
  /// \~Chinese 跟踪ID
  HorizonVisionTrackID track_id;
  /// \~Chinese 人体框
  HorizonVisionBBox body_rect;
  /// \~Chinese 人体分割
  HorizonVisionSegmentation *segmentation;
  /// \~Chinese 骨骼点
  HorizonVisionLandmarks *skeleton;
} HorizonVisionBodySmartData;

/**
 *\~Chinese @brief 每一个目标额外的人脸调试用辅助信息
 */

typedef struct {
  /// \~Chinese RGB活体信息
  HorizonVisionAntiSpoofing rgb_anti_spf_;
  /// \~Chinese 红外活体信息
  HorizonVisionAntiSpoofing nir_anti_spf_;
  /// \~Chinese 红外检测框
  HorizonVisionBBox nir_box_;
  /// \~Chinese 红外外扩检测框
  HorizonVisionBBox nir_norm_box_;
  /// \~Chinese RGB检测框
  HorizonVisionBBox rgb_box_;
  /// \~Chinese RGB外扩检测框
  HorizonVisionBBox rgb_norm_box_;
} HorizonVisionFaceExtraInfo;

/**
 * \~Chinese @brief 单个跟踪目标智能信息，包含人脸智能信息、人体智能信息等
 */

typedef struct HorizonVisionSmartData_ {
  /// \~Chinese 目标类型 0：正常，1:已过滤 2:已消失
  uint32_t type;
  /// \~Chinese 跟踪ID
  HorizonVisionTrackID track_id;
  /// \~Chinese 人脸智能信息
  HorizonVisionFaceSmartData *face;
  /// \~Chinese 人体智能信息
  HorizonVisionBodySmartData *body;
  /// \~Chinese 人脸额外的辅助信息，默认为空
  HorizonVisionFaceExtraInfo *face_extra;
} HorizonVisionSmartData;
/**
 * \~Chinese @brief 智能帧
 */
typedef struct HorizonVisionSmartFrame_ {
  /// \~Chinese 图像帧个数
  uint32_t image_num;
  /// \~Chinese 当前智能帧对应图像帧列表
  HorizonVisionImageFrame **image_frame;
  /// \~Chinese 智能信息列表个数
  uint32_t smart_data_list_num;
  /// \~Chinese 指向智能信息列表
  HorizonVisionSmartData *smart_data_list;
  /// \~Chinese 时间戳
  HorizonVisionTimeStamp time_stamp;
  /// \~Chinese 帧ID
  HorizonVisionFrameID frame_id;
} HorizonVisionSmartFrame;
/**
 * \~Chinese @brief 单个抓拍图
 */
typedef struct HorizonVisionSnapshot_ {
  /// \~Chinese 时间戳
  HorizonVisionTimeStamp time_stamp;
  /// \~Chinese 抓拍图的智能信息，如：跟踪ID、人脸特征
  HorizonVisionSmartData *smart_data;
  /// \~Chinese 抓拍图
  HorizonVisionImage *croped_image;
} HorizonVisionSnapshot;

/**
 * \~Chinese @brief 单个抓拍目标，内含多个抓拍图
 */
typedef struct HorizonVisionSnapshotTarget_ {
  /// \~Chinese 抓拍类型，0为消失触发的抓拍，1为达到优选数量触发的抓拍
  int32_t type;
  /// \~Chinese 跟踪ID
  HorizonVisionTrackID track_id;
  /// \~Chinese 抓拍图数量
  uint32_t snapshots_num;
  /// \~Chinese 指向抓拍图列表
  HorizonVisionSnapshot *snapshots;
} HorizonVisionSnapshotTarget;

/**
 * \~Chinese @brief 抓拍帧
 */
typedef struct HorizonVisionSnapshotFrame_ {
  /// \~Chinese 抓拍目标数目
  uint32_t targets_num;
  /// \~Chinese 指向抓拍目标列表
  HorizonVisionSnapshotTarget *targets;
} HorizonVisionSnapshotFrame;

#ifdef __cplusplus
}
#endif
#endif  // VISION_TYPE_VISION_MSG_H_
