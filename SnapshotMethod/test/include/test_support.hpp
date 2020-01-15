/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     dump helper header
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2019.05.23
 */


#ifndef TEST_SUPPORT_HPP_
#define TEST_SUPPORT_HPP_

#include <string>
#include <memory>
#include "hobotxsdk/xroc_sdk.h"
#include "horizon/vision_type/vision_type.hpp"
#include "opencv2/opencv.hpp"

typedef std::shared_ptr<hobot::vision::ImageFrame> ImageFramePtr;
typedef hobot::vision::SnapshotInfo<HobotXRoc::BaseDataPtr> SnapshotInfoXRocBaseData;
typedef std::shared_ptr<SnapshotInfoXRocBaseData> SnapshotInfoXRocBaseDataPtr;
typedef HobotXRoc::XRocData<SnapshotInfoXRocBaseDataPtr> XRocSnapshotInfo;
typedef std::shared_ptr<XRocSnapshotInfo> XRocSnapshotInfoPtr;

typedef HobotXRoc::XRocData<ImageFramePtr> XRocImageFrame;
typedef std::shared_ptr<XRocImageFrame> XRocImageFramePtr;
typedef std::shared_ptr<hobot::vision::CVImageFrame> CVImageFramePtr;
typedef HobotXRoc::XRocData<hobot::vision::BBox> XRocBBox;
typedef HobotXRoc::XRocData<uint32_t> XRocUint32;
typedef HobotXRoc::XRocData<float> XRocFloat;

int WriteLog(const XRocSnapshotInfoPtr &snapshot_info);

static int SaveImg(const ImageFramePtr &img_ptr, const std::string &path);

int DumpSnap(const XRocSnapshotInfoPtr &snapshot_info, std::string dir = ".");

int ConstructInput(const std::string &smart_frame,
                   const std::string &video_path,
                   HobotXRoc::InputDataPtr &input,
                   const std::string &img_format,
                   bool filter);

int ConstructInputInvalidUserdata(
                   const std::string &smart_frame,
                   const std::string &video_path,
                   HobotXRoc::InputDataPtr &input,
                   const std::string &img_format,
                   bool filter);

#endif //  TEST_SUPPORT_HPP_
