//
// Created by yaoyao.sun on 2019-04-23.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#include <assert.h>
#include <stdint.h>
#include <algorithm>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "hobot_vision/bpumodel_manager.hpp"
#include "bpu_predict/bpu_io.h"
#include "bpu_predict/bpu_predict.h"
#include "bpu_predict/bpu_internal.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxroc/profiler.h"
#include "hobotxsdk/xroc_data.h"
#include "json/json.h"
#include "opencv2/imgproc.hpp"
#include "hobotxroc/profiler.h"
#include "horizon/vision_type/vision_type.hpp"
#include "hobotxsdk/xroc_data.h"
#include "hobotlog/hobotlog.hpp"
#include "hbdk/hbdk_hbrt.h"
#include "FasterRCNNMethod/FasterRCNNMethod.h"
#include "FasterRCNNMethod/config.h"
#include "FasterRCNNMethod/faster_rcnn_imp.h"
#include "FasterRCNNMethod/result.h"
#include "FasterRCNNMethod/util.h"
#include "FasterRCNNMethod/yuv_utils.h"
#include "common/common.h"

#define DMA_ALIGN_SIZE 64
#define BPU_CEIL_ALIGN(len) \
  ((((len) + DMA_ALIGN_SIZE - 1) / DMA_ALIGN_SIZE) * DMA_ALIGN_SIZE)

using hobot::vision::Point;
using hobot::vision::Pose3D;
using hobot::vision::BBox;
using hobot::vision::Landmarks;
using hobot::vision::Segmentation;
using hobot::vision::Attribute;
using hobot::vision::PymImageFrame;
using hobot::vision::CVImageFrame;

using HobotXRoc::BaseDataPtr;
using HobotXRoc::BaseDataVector;
using HobotXRoc::XRocData;
using HobotXRoc::InputParamPtr;
using HobotXRoc::FasterRCNNParam;

using ImageFramePtr = std::shared_ptr<hobot::vision::ImageFrame>;

namespace faster_rcnn_method {
// used to decide output info for each layer of model. first we should use hbcc
// interface to get model's info,
// and get each layer's output info, then config it in config file.
static std::map<std::string, FasterRCNNBranchOutType> str2faster_rcnn_out_type =
    {{"bbox", FasterRCNNBranchOutType::BBOX},
     {"kps", FasterRCNNBranchOutType::KPS},
     {"mask", FasterRCNNBranchOutType::MASK},
     {"reid", FasterRCNNBranchOutType::REID},
     {"lmks2_label", FasterRCNNBranchOutType::LMKS2_LABEL},
     {"lmks2_offset", FasterRCNNBranchOutType::LMKS2_OFFSET},
     {"lmks1", FasterRCNNBranchOutType::LMKS1},
     {"3d_pose", FasterRCNNBranchOutType::POSE_3D},
     {"plate_color", FasterRCNNBranchOutType::PLATE_COLOR},
     {"plate_row", FasterRCNNBranchOutType::PLATE_ROW}};


class ModelOutputBuffer {
 public:
  ModelOutputBuffer(const BPUModelInfo &output_info,
                    std::vector<BPU_Buffer_Handle> &output_buf)
      : output_buf_(output_buf) {
    // alloc output buffer.
    for (int i = 0; i < output_info.num; ++i) {
      BPU_Buffer_Handle out_handle = BPU_createEmptyBPUBuffer();
      output_buf_.push_back(out_handle);
    }
    LOGD << "create bpu buffer success.";
  }
  ~ModelOutputBuffer() {
    // release output buffer.
    for (auto &buf : output_buf_) {
      BPU_freeBPUBuffer(buf);
    }
    output_buf_.clear();
    LOGD << "release bpu buffer success.";
  }
 private:
  std::vector<BPU_Buffer_Handle> &output_buf_;
};

void FasterRCNNImp::ParseConfig(const std::string &config_file) {
  FR_Config cfg_jv;
  std::ifstream infile(config_file.c_str());
  infile >> cfg_jv;
  config_.reset(new Config(cfg_jv));

  auto net_info = config_->GetSubConfig("net_info");
  model_name_ = net_info->GetSTDStringValue("model_name");
  model_version_ = net_info->GetSTDStringValue("model_version", "1.0.0");
  std::vector<std::shared_ptr<Config>> model_out_sequence =
      net_info->GetSubConfigArray("model_out_sequence");

  LOGD << "rcnn branch out type:";
  for (size_t i = 0; i < model_out_sequence.size(); ++i) {
    std::string type_str = model_out_sequence[i]->GetSTDStringValue("type");
    HOBOT_CHECK(!type_str.empty());
    LOGD << type_str;
    if (str2faster_rcnn_out_type.find(type_str) ==
        str2faster_rcnn_out_type.end()) {
      out_level2rcnn_branch_info_[i].type = FasterRCNNBranchOutType::INVALID;
    } else {
      out_level2rcnn_branch_info_[i].type = str2faster_rcnn_out_type[type_str];
      out_level2rcnn_branch_info_[i].name =
          model_out_sequence[i]->GetSTDStringValue("name");
      out_level2rcnn_branch_info_[i].box_name =
          model_out_sequence[i]->GetSTDStringValue("box_name");
    }
  }

  model_input_width_ = net_info->GetIntValue("model_input_width", 960);
  model_input_height_ = net_info->GetIntValue("model_input_height", 540);
  pyramid_layer_ = net_info->GetIntValue("pyramid_layer", 4);

  kps_pos_distance_ = net_info->GetFloatValue("kps_pos_distance", 0.1);
  kps_feat_width_ = net_info->GetIntValue("kps_feat_width", 16);
  kps_feat_height_ = net_info->GetIntValue("kps_feat_height", 16);
  kps_points_number_ = net_info->GetIntValue("kps_points_number", 17);

  lmk_feat_height_ = net_info->GetIntValue("lmk_feat_height", 8);
  lmk_feat_width_ = net_info->GetIntValue("lmk_feat_width", 8);
  lmk_feat_stride_ = net_info->GetIntValue("lmk_feat_stride", 16);
  lmk_points_number_ = net_info->GetIntValue("lmk_points_number", 5);
  lmk_pos_distance_ = net_info->GetFloatValue("lmk_pos_distance", 12);
  face_pose_number_ = net_info->GetIntValue("3d_pose_number", 3);

  plate_color_num_ = net_info->GetIntValue("plate_color_num", 6);
  plate_row_num_ = net_info->GetIntValue("plate_row_num", 2);

  method_outs_ = config_->GetSTDStringArray("method_outs");
  LOGD << "method out type:";
  for (const auto &method_out : method_outs_) {
    LOGD << method_out;
  }

  std::string parent_path = GetParentPath(config_file);
  bpu_config_path_ =
      parent_path + config_->GetSTDStringValue("bpu_config_path");
  model_file_path_ =
      parent_path + config_->GetSTDStringValue("model_file_path");
  LOGD << "config file parent path: " << parent_path
       << " bpu_config_path: " << bpu_config_path_
       << " model_file_path: " << model_file_path_;
}

void FasterRCNNImp::GetModelInfo(const std::string &model_name) {
  const hbrt_feature_handle_t *feature_info;
  uint32_t output_layer_num = 0;
  hbrt_hbm_handle_t hbm_handle_;
  int ret = BPU_getHBMhandleFromBPUhandle(bpu_handle_, &hbm_handle_.handle);
  HOBOT_CHECK(ret == 0) << "Load bpu model failed: "
                        << BPU_getLastError(bpu_handle_);
  hbrt_model_handle_t model_handle_;
  CHECK_HBRT_ERROR(hbrtGetModelHandle(&model_handle_, hbm_handle_,
                      model_name.c_str()));
  CHECK_HBRT_ERROR(hbrtGetOutputFeatureNumber(&output_layer_num,
                     model_handle_));
  CHECK_HBRT_ERROR(hbrtGetOutputFeatureHandles(&feature_info, model_handle_));

  for (size_t i = 0; i < output_layer_num; ++i) {
    const auto &branch_info = out_level2rcnn_branch_info_[i];
    auto out_type = branch_info.type;
    // get shifts
    const uint8_t *shift_value;
    CHECK_HBRT_ERROR(hbrtGetFeatureShiftValues(&shift_value, feature_info[i]));
    hbrt_layout_type_t layout_type;
    CHECK_HBRT_ERROR(hbrtGetFeatureLayoutType(&layout_type, feature_info[i]));
    hbrt_dimension_t aligned_dim;
    CHECK_HBRT_ERROR(hbrtGetFeatureAlignedDimension(&aligned_dim,
    feature_info[i]));
    bool is_big_endian;
    CHECK_HBRT_ERROR(hbrtFeatureIsBigEndian(&is_big_endian, feature_info[i]));
    hbrt_element_type_t element_type;
    CHECK_HBRT_ERROR(hbrtGetFeatureElementType(&element_type, feature_info[i]));
    switch (out_type) {
      case FasterRCNNBranchOutType::INVALID:
        break;
      case FasterRCNNBranchOutType::KPS:
        // TODO(yaoyao.sun) pack into a function
        kps_shift_ = shift_value[0];
        kps_layout_type_ = layout_type;
        aligned_kps_dim = aligned_dim;
        aligned_kps_dim.n = 1;
        kps_is_big_endian = is_big_endian;
        kps_element_type = element_type;
        break;
      case FasterRCNNBranchOutType::MASK:
        mask_shift_ = shift_value[0];
        mask_layout_type_ = layout_type;
        aligned_mask_dim = aligned_dim;
        aligned_mask_dim.n = 1;
        mask_is_big_endian = is_big_endian;
        mask_element_type = element_type;
        break;
      case FasterRCNNBranchOutType::REID:
        reid_shift_ = shift_value[0];
        reid_layout_type_ = layout_type;
        aligned_reid_dim = aligned_dim;
        aligned_reid_dim.n = 1;
        reid_is_big_endian = is_big_endian;
        reid_element_type = element_type;
        break;
      case FasterRCNNBranchOutType::LMKS2_LABEL:
        lmks2_label_shift_ = shift_value[0];
        lmks2_label_layout_type_ = layout_type;
        aligned_lmks2_label_dim = aligned_dim;
        aligned_lmks2_label_dim.n = 1;
        lmk2_label_is_big_endian = is_big_endian;
        lmk2_label_element_type = element_type;
        break;
      case FasterRCNNBranchOutType::LMKS2_OFFSET:
        lmks2_offset_shift_ = shift_value[0];
        lmks2_offset_layout_type_ = layout_type;
        aligned_lmks2_offset_dim = aligned_dim;
        aligned_lmks2_offset_dim.n = 1;
        lmk2_offset_is_big_endian = is_big_endian;
        lmk2_offet_element_type = element_type;
        break;
      case FasterRCNNBranchOutType::LMKS1:
        lmks1_shift_ = shift_value[0];
        lmks1_layout_type_ = layout_type;
        aligned_lmks1_dim = aligned_dim;
        aligned_lmks1_dim.n = 1;
        lmk1_is_big_endian = is_big_endian;
        lmk1_element_type = element_type;
        break;
      case FasterRCNNBranchOutType::POSE_3D:
        face_pose_shift_ = shift_value[0];
        face_pose_layout_type_ = layout_type;
        aligned_face_pose_dim = aligned_dim;
        aligned_face_pose_dim.n = 1;
        face_pose_is_big_endian = is_big_endian;
        face_pose_element_type = element_type;
        break;
      case FasterRCNNBranchOutType::PLATE_COLOR:
        plate_color_shift_ = shift_value[0];
        plate_color_layout_type_ = layout_type;
        aligned_plate_color_dim = aligned_dim;
        aligned_plate_color_dim.n = 1;
        plate_color_is_big_endian = is_big_endian;
        plate_color_element_type = element_type;
        HOBOT_CHECK(plate_color_num_ <= aligned_plate_color_dim.c);
        break;
      case FasterRCNNBranchOutType::PLATE_ROW:
        plate_row_shift_ = shift_value[0];
        plate_row_layout_type_ = layout_type;
        aligned_plate_row_dim = aligned_dim;
        aligned_plate_row_dim.n = 1;
        plate_row_is_big_endian = is_big_endian;
        plate_row_element_type = element_type;
        HOBOT_CHECK(plate_row_num_ <= aligned_plate_row_dim.c);
        break;
      default:
        break;
    }
  }
}

int FasterRCNNImp::Init(const std::string &config_file) {
  faster_rcnn_param_ = nullptr;
  // parse config file.
  ParseConfig(config_file);
  CHECK_HBRT_ERROR(hbrtIsCompatibleHeader());
  bpu_handle_ = hobot::vision::BPUModelManager::Get().GetBpuHandle(
    model_file_path_, bpu_config_path_);
  int ret = BPU_getModelOutputInfo(bpu_handle_, model_name_.c_str(),
      &output_info_);
  HOBOT_CHECK(ret == 0) << "Get model " << model_name_
                        << " output info failed: "
                        << BPU_getLastError(bpu_handle_);
  LOGD << "BPU_getModelOutputInfo success.";
  GetModelInfo(model_name_);
  return 0;
}

void FasterRCNNImp::RunSingleFrame(const std::vector<BaseDataPtr> &frame_input,
                                   std::vector<BaseDataPtr> &frame_output) {
  // only one input slot -> PyramidImage or CVImage
  HOBOT_CHECK(frame_input.size() == 1);
  const auto frame_img_ = frame_input[0];

  for (size_t out_index = 0; out_index < method_outs_.size(); ++out_index) {
    frame_output.push_back(std::make_shared<HobotXRoc::BaseDataVector>());
  }

  auto xroc_img = std::static_pointer_cast<XRocData<ImageFramePtr>>(frame_img_);

  std::string img_type = xroc_img->value->type;
  LOGD << "image type: " << img_type << std::endl;

  BPUModelHandle model_handle;
  int ret = 0;

  int src_img_width = 0;
  int src_img_height = 0;

  ModelOutputBuffer output_buf(output_info_, out_buf_);
  {
    RUN_PROCESS_TIME_PROFILER("FasterRCNN RunModelFromPyramid");
    RUN_FPS_PROFILER("FasterRCNN RunModelFromPyramid");

    if (img_type == kPyramidImage) {
      auto pyramid_image =
          std::static_pointer_cast<PymImageFrame>(xroc_img->value);
      src_img_height = pyramid_image->img.src_img.height;
      src_img_width = pyramid_image->img.src_img.width;
      ret = BPU_runModelFromPyramid(bpu_handle_, model_name_.c_str(),
                                    static_cast<void *>(&(pyramid_image->img)),
                                    pyramid_layer_, out_buf_.data(),
                                    out_buf_.size(), &model_handle);

    } else if (img_type == kCVImageFrame) {
      auto cv_image = std::static_pointer_cast<CVImageFrame>(xroc_img->value);

      HOBOT_CHECK(cv_image->pixel_format ==
                  HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawBGR);

      src_img_height = cv_image->Height();
      src_img_width = cv_image->Width();

      auto img_mat = cv_image->img;
      cv::Mat resized_mat = img_mat;
      if (src_img_height != model_input_height_ &&
          src_img_width != model_input_width_) {
        LOGD << "need resize.";
        cv::resize(img_mat, resized_mat,
                   cv::Size(model_input_width_, model_input_height_));
      }

      cv::Mat img_nv12;
      uint8_t *bgr_data = resized_mat.ptr<uint8_t>();
      bgr_to_nv12(bgr_data, model_input_height_, model_input_width_, img_nv12);
      uint8_t *nv12_data = img_nv12.ptr<uint8_t>();

      // construct BPUFakeImage
      BPUFakeImageHandle fake_img_handle;

      int img_len = model_input_width_ * model_input_height_ * 3 / 2;
      LOGD << "nv12 img_len: " << img_len;

      ret = BPU_createFakeImageHandle(model_input_height_, model_input_width_,
                                      &fake_img_handle);
      HOBOT_CHECK(ret == 0) << "create fake image handle failed";

      BPUFakeImage *fake_img_ptr = nullptr;
      fake_img_ptr = BPU_getFakeImage(fake_img_handle, nv12_data, img_len);
      HOBOT_CHECK(fake_img_ptr != nullptr) << "get fake image failed";
      ret = BPU_runModelFromImage(bpu_handle_, model_name_.c_str(),
                                  fake_img_ptr, out_buf_.data(),
                                  out_buf_.size(), &model_handle);

      BPU_releaseFakeImage(fake_img_handle, fake_img_ptr);

      BPU_releaseFakeImageHandle(fake_img_handle);

    } else {
      HOBOT_CHECK(0) << "Not support this input type: " << img_type;
    }

    LOGD << "image height: " << src_img_height << "width: " << src_img_width;
    if (ret != 0) {
      LOGE << "Run model failed: " << BPU_getLastError(bpu_handle_);
      BPU_releaseModelHandle(bpu_handle_, model_handle);
      return;
    }
    ret = BPU_getModelOutput(bpu_handle_, model_handle);
    BPU_releaseModelHandle(bpu_handle_, model_handle);
    if (ret != 0) {
      LOGE << "Get model out put failed: " << BPU_getLastError(bpu_handle_);
      return;
    }
  }

  RUN_PROCESS_TIME_PROFILER("FasterRCNN PostProcess");
  RUN_FPS_PROFILER("FasterRCNN PostProcess");

  // Post process
  GetFrameOutput(src_img_width, src_img_height, frame_output);
}

void FasterRCNNImp::GetFrameOutput(int src_img_width, int src_img_height,
                                   std::vector<BaseDataPtr> &frame_output) {
  FasterRCNNOutMsg det_result;
  PostProcess(det_result);
  CoordinateTransform(det_result, src_img_width, src_img_height,
                      model_input_width_, model_input_height_);
  for (auto &boxes : det_result.boxes) {
    LOGD << boxes.first << ", num: " << boxes.second.size();
    for (auto &box : boxes.second) {
      LOGD << box;
    }
  }

  // TODO(yaoyao.sun) Packaged into a function, GetResultMsg()
  // convert FasterRCNNOutMsg to XROC data structure
  std::map<std::string, std::shared_ptr<BaseDataVector>> xroc_det_result;

  // get landmark by landmark2 and landmark1.
  if (det_result.landmarks.find("landmark2") != det_result.landmarks.end() &&
      det_result.landmarks.find("landmark1") != det_result.landmarks.end()) {
    std::vector<Landmarks> vec_landmarks;
    auto &landmarks2 = det_result.landmarks["landmark2"];
    auto &landmarks1 = det_result.landmarks["landmark1"];
    HOBOT_CHECK(landmarks2.size() == landmarks1.size())
        << "landmarks2's size not equal to landmarks1's size.";
    for (size_t lmk_index = 0; lmk_index < landmarks2.size(); ++lmk_index) {
      Landmarks landmarks;
      auto &landmark2 = landmarks2[lmk_index];
      auto &landmark1 = landmarks1[lmk_index];
      HOBOT_CHECK(landmark2.values.size() == landmark1.values.size())
          << "landmark2's size not equal to landmark1's size.";
      for (size_t point_index = 0; point_index < landmark2.values.size();
           ++point_index) {
        auto &landmark2_point = landmark2.values[point_index];
        auto &landmark1_point = landmark1.values[point_index];
        if (landmark2_point.score > 0.5) {
          landmarks.values.push_back(std::move(landmark2_point));
        } else {
          landmarks.values.push_back(std::move(landmark1_point));
        }
      }
      vec_landmarks.push_back(landmarks);
    }
    det_result.landmarks["landmark"] = vec_landmarks;
    det_result.landmarks.erase("landmark2");
    det_result.landmarks.erase("landmark1");
  }

  // box
  for (const auto &boxes : det_result.boxes) {
    xroc_det_result[boxes.first] =
        std::make_shared<HobotXRoc::BaseDataVector>();
    xroc_det_result[boxes.first]->name_ = "rcnn_" + boxes.first;
    for (auto &box : boxes.second) {
      auto xroc_box = std::make_shared<XRocData<BBox>>();
      xroc_box->value = std::move(box);
      xroc_det_result[boxes.first]->datas_.push_back(xroc_box);
    }
  }

  // landmark
  for (const auto &landmarks_vec : det_result.landmarks) {
    xroc_det_result[landmarks_vec.first] =
        std::make_shared<HobotXRoc::BaseDataVector>();
    xroc_det_result[landmarks_vec.first]->name_ = "rcnn_" + landmarks_vec.first;
    for (auto &landmarks : landmarks_vec.second) {
      LOGD << "lmk point: [";
      for (const auto &point : landmarks.values) {
        LOGD << point.x << "," << point.y << "," << point.score;
      }
      LOGD << "]";
      auto xroc_landmark = std::make_shared<XRocData<Landmarks>>();
      xroc_landmark->value = std::move(landmarks);
      xroc_det_result[landmarks_vec.first]->datas_.push_back(xroc_landmark);
    }
  }

  // feature
  for (const auto &feature_vec : det_result.features) {
    xroc_det_result[feature_vec.first] =
        std::make_shared<HobotXRoc::BaseDataVector>();
    xroc_det_result[feature_vec.first]->name_ = "rcnn_" + feature_vec.first;
    for (auto &feature : feature_vec.second) {
      auto xroc_feature = std::make_shared<XRocData<Feature>>();
      xroc_feature->value = std::move(feature);
      xroc_det_result[feature_vec.first]->datas_.push_back(xroc_feature);
    }
  }

  // segmentations
  for (const auto &segmentation_vec : det_result.segmentations) {
    xroc_det_result[segmentation_vec.first] =
        std::make_shared<HobotXRoc::BaseDataVector>();
    xroc_det_result[segmentation_vec.first]->name_ =
        "rcnn_" + segmentation_vec.first;
    for (auto &segmentation : segmentation_vec.second) {
      auto xroc_segmentation = std::make_shared<XRocData<Segmentation>>();
      xroc_segmentation->value = std::move(segmentation);
      xroc_det_result[segmentation_vec.first]->datas_.push_back(
          xroc_segmentation);
    }
  }

  // poses
  for (const auto &pose_vec : det_result.poses) {
    xroc_det_result[pose_vec.first] =
        std::make_shared<HobotXRoc::BaseDataVector>();
    xroc_det_result[pose_vec.first]->name_ = "rcnn_" + pose_vec.first;
    for (auto &pose : pose_vec.second) {
      auto xroc_pose = std::make_shared<XRocData<Pose3D>>();
      xroc_pose->value = std::move(pose);
      xroc_det_result[pose_vec.first]->datas_.push_back(xroc_pose);
    }
  }

  // attributes
  for (const auto &attribute_vec : det_result.attributes) {
    xroc_det_result[attribute_vec.first] =
        std::make_shared<HobotXRoc::BaseDataVector>();
    xroc_det_result[attribute_vec.first]->name_ = "rcnn_" + attribute_vec.first;
    for (auto &attribute : attribute_vec.second) {
      auto xroc_attribute = std::make_shared<XRocData<Attribute<int>>>();
      xroc_attribute->value = std::move(attribute);
      xroc_det_result[attribute_vec.first]->datas_.push_back(xroc_attribute);
    }
  }

  for (size_t out_index = 0; out_index < method_outs_.size(); ++out_index) {
    if (xroc_det_result[method_outs_[out_index]]) {
      frame_output[out_index] = xroc_det_result[method_outs_[out_index]];
    }
  }
}

void FasterRCNNImp::PostProcess(FasterRCNNOutMsg &det_result) {
  BPU_Buffer_Handle lmk2_label_out_put = nullptr;
  BPU_Buffer_Handle lmk2_offset_out_put = nullptr;

  for (size_t out_level = 0; out_level < out_buf_.size(); ++out_level) {
    const auto &branch_info = out_level2rcnn_branch_info_[out_level];
    auto out_type = branch_info.type;
    switch (out_type) {
      case FasterRCNNBranchOutType::INVALID:
        break;
      case FasterRCNNBranchOutType::BBOX:
        GetRppRects(det_result.boxes[branch_info.name], out_buf_[out_level]);
        break;
      case FasterRCNNBranchOutType::LMKS2_LABEL:
        lmk2_label_out_put = out_buf_[out_level];
        break;
      case FasterRCNNBranchOutType::LMKS2_OFFSET:
        lmk2_offset_out_put = out_buf_[out_level];
        break;
      default:
        break;
    }
  }

  for (size_t out_level = 0; out_level < out_buf_.size(); ++out_level) {
    const auto &branch_info = out_level2rcnn_branch_info_[out_level];
    auto out_type = branch_info.type;
    switch (out_type) {
      case FasterRCNNBranchOutType::INVALID:
        break;
      case FasterRCNNBranchOutType::KPS:
        GetKps(det_result.landmarks[branch_info.name], out_buf_[out_level],
               det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::MASK:
        GetMask(det_result.segmentations[branch_info.name], out_buf_[out_level],
                det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::REID:
        GetReid(det_result.features[branch_info.name], out_buf_[out_level],
                det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::LMKS2_LABEL:
        GetLMKS2(det_result.landmarks["landmark2"], lmk2_label_out_put,
                 lmk2_offset_out_put, det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::LMKS1:
        GetLMKS1(det_result.landmarks["landmark1"], out_buf_[out_level],
                 det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::POSE_3D:
        GetPose(det_result.poses[branch_info.name], out_buf_[out_level],
                det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::PLATE_COLOR:
        GetPlateColor(&(det_result.attributes[branch_info.name]),
                      out_buf_[out_level],
                      det_result.boxes[branch_info.box_name]);
        break;
      case FasterRCNNBranchOutType::PLATE_ROW:
        GetPlateRow(&(det_result.attributes[branch_info.name]),
                    out_buf_[out_level],
                    det_result.boxes[branch_info.box_name]);
        break;
      default:
        break;
    }
  }
}

void FasterRCNNImp::GetRects(std::vector<BBox> &boxes,
                             BPU_Buffer_Handle output) {
  std::vector<float> rects;
  void *feature_map = BPU_getRawBufferPtr(output);
  // TODO(yaoyao.sun) use feature_info's element_size to replace unit8_t
  uint8_t *box_infos = reinterpret_cast<uint8_t *>(feature_map);
  uint32_t int_out_sz = *(reinterpret_cast<uint32_t *>(box_infos));
  LOGD << "int_out_sz: " << int_out_sz;
  box_infos += BPU_CEIL_ALIGN(int_out_sz + 16);
  uint32_t fp_out_sz = *(reinterpret_cast<uint32_t *>(box_infos));
  LOGD << "fp_out_sz: " << fp_out_sz;
  box_infos += 16;
  int box_num = fp_out_sz / (sizeof(float) * 6);

  rects.resize(box_num * 6);
  memcpy(rects.data(), reinterpret_cast<float *>(box_infos),
         rects.size() * sizeof(float));

  // add boxes
  for (size_t j = 0; j < rects.size(); j += 6) {
    hobot::vision::BBox box;
    box.x1 = rects[j];
    box.y1 = rects[j + 1];
    box.x2 = rects[j + 2];
    box.y2 = rects[j + 3];
    box.score = rects[j + 4];
    boxes.push_back(std::move(box));
  }
}

void FasterRCNNImp::GetRppRects(std::vector<BBox> &boxes,
                                BPU_Buffer_Handle output) {
  void *feature_map = BPU_getRawBufferPtr(output);
  auto item_size = sizeof(cpu_op_rcnn_post_process_bbox_float_type_t);
  float output_by_size = *(reinterpret_cast<float*>(feature_map));
  uint32_t box_num = (uint32_t)output_by_size / item_size;
  cpu_op_rcnn_post_process_bbox_float_type_t *p_box =
          reinterpret_cast<cpu_op_rcnn_post_process_bbox_float_type_t *>(
                     reinterpret_cast<uintptr_t>(feature_map) + item_size);
  for (uint32_t i = 0; i < box_num; ++i) {
    hobot::vision::BBox box;
    box.x1 = p_box[i].left;
    box.y1 = p_box[i].top;
    box.x2 = p_box[i].right;
    box.y2 = p_box[i].bottom;
    box.score = p_box[i].score;
    boxes.push_back(std::move(box));
  }
}

void FasterRCNNImp::GetKps(std::vector<Landmarks> &kpss,
                           BPU_Buffer_Handle output,
                           const std::vector<BBox> &body_boxes) {
  int32_t *kps_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(output));

  float pos_distance = kps_pos_distance_ * kps_feat_width_;
  size_t body_box_num = body_boxes.size();
  int feature_size =
      aligned_kps_dim.h * aligned_kps_dim.w * aligned_kps_dim.c;

  std::vector<int32_t> mxnet_out_for_one_point(
      kps_feat_height_ * kps_feat_width_, 0);
  for (size_t box_id = 0; box_id < body_box_num; ++box_id) {
    const auto &body_box = body_boxes[box_id];
    float x1 = body_box.x1;
    float y1 = body_box.y1;
    float x2 = body_box.x2;
    float y2 = body_box.y2;
    float w = x2 - x1 + 1;
    float h = y2 - y1 + 1;

    float scale_x = kps_feat_width_ / w;
    float scale_y = kps_feat_height_ / h;

    Landmarks skeleton;
    skeleton.values.resize(kps_points_number_);

    for (int kps_id = 0; kps_id < kps_points_number_; ++kps_id) {
      mxnet_out_for_one_point.assign(kps_feat_height_ * kps_feat_width_, 0);
      unsigned best_index = 0;
      CHECK_HBRT_ERROR(hbrtConvertLayoutToNative1HW1(
                                              mxnet_out_for_one_point.data(),
                                         kps_feature + feature_size * box_id,
                                          kps_layout_type_, kps_element_type,
                                              aligned_kps_dim,
                                              kps_is_big_endian, 0, kps_id));

      // find the best position
      for (int i = 0; i < kps_feat_width_ * kps_feat_height_; ++i) {
        if (mxnet_out_for_one_point[i] > mxnet_out_for_one_point[best_index]) {
          best_index = i;
        }
      }
      float max_score =
          GetFloatByInt(mxnet_out_for_one_point[best_index], kps_shift_);
      // best_index = max_h * kps_feat_width + max_w
      unsigned max_w = best_index % kps_feat_width_;
      unsigned max_h = best_index / kps_feat_width_;

      // get delta
      mxnet_out_for_one_point.assign(kps_feat_width_ * kps_feat_height_, 0);
      CHECK_HBRT_ERROR(hbrtConvertLayoutToNative1HW1(
                                        mxnet_out_for_one_point.data(),
                                        kps_feature + feature_size * box_id,
                                        kps_layout_type_, kps_element_type,
                                        aligned_kps_dim, kps_is_big_endian,
                                        0, 2 * kps_id + kps_points_number_));
      const auto x_delta = mxnet_out_for_one_point[best_index];
      float fp_delta_x = GetFloatByInt(x_delta, kps_shift_) * pos_distance;

      mxnet_out_for_one_point.assign(kps_feat_width_*kps_feat_height_, 0);
      CHECK_HBRT_ERROR(hbrtConvertLayoutToNative1HW1(
                                         mxnet_out_for_one_point.data(),
                                         kps_feature + feature_size * box_id,
                                         kps_layout_type_, kps_element_type,
                                         aligned_kps_dim, kps_is_big_endian, 0,
                                         2 * kps_id + kps_points_number_ + 1));
      const auto y_delta = mxnet_out_for_one_point[best_index];
      float fp_delta_y = GetFloatByInt(y_delta, kps_shift_) * pos_distance;

      Point point;
      point.x = (max_w + fp_delta_x + 0.46875) / scale_x + x1;
      point.y = (max_h + fp_delta_y + 0.46875) / scale_y + y1;
      point.score = max_score;
      skeleton.values[kps_id] = point;
    }
    kpss.push_back(std::move(skeleton));
  }
}

void FasterRCNNImp::GetMask(std::vector<Segmentation> &masks,
                            BPU_Buffer_Handle output,
                            const std::vector<BBox> &body_boxes) {
  size_t body_box_num = body_boxes.size();
  int32_t *mask_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(output));

  std::vector<int32_t> mxnet_out_for_one_box(
      aligned_mask_dim.h * aligned_mask_dim.w, 0);
  int feature_size =
      aligned_mask_dim.h * aligned_mask_dim.w * aligned_mask_dim.c;
  for (size_t box_id = 0; box_id < body_box_num; ++box_id) {
    mxnet_out_for_one_box.assign(
                        aligned_mask_dim.h * aligned_mask_dim.w, 0);
    CHECK_HBRT_ERROR(hbrtConvertLayoutToNative1HW1(
                                mxnet_out_for_one_box.data(),
                                mask_feature + feature_size * box_id,
                                mask_layout_type_, mask_element_type,
                                aligned_mask_dim, mask_is_big_endian,
                                0, 0));
    float fp_for_this_mask;
    Segmentation mask;
    for (int32_t mask_i = 0;
         mask_i < aligned_mask_dim.h * aligned_mask_dim.w; ++mask_i) {
      fp_for_this_mask =
          GetFloatByInt(mxnet_out_for_one_box[mask_i], mask_shift_);
      mask.values.push_back(fp_for_this_mask);
    }
    mask.height = aligned_mask_dim.h;
    mask.width = aligned_mask_dim.w;
    masks.push_back(std::move(mask));
  }
}

void FasterRCNNImp::GetReid(std::vector<Feature> &reids,
                            BPU_Buffer_Handle output,
                            const std::vector<BBox> &body_boxes) {
  size_t body_box_num = body_boxes.size();
  int32_t *reid_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(output));

  int feature_size =
      aligned_reid_dim.h * aligned_reid_dim.w * aligned_reid_dim.c;
  std::vector<int32_t> mxnet_out_for_one_box(aligned_reid_dim.c, 0);

  for (size_t box_id = 0; box_id < body_box_num; ++box_id) {
    mxnet_out_for_one_box.assign(aligned_reid_dim.c, 0);
    CHECK_HBRT_ERROR(hbrtConvertLayoutToNative111C(
                                      mxnet_out_for_one_box.data(),
                              reid_feature + feature_size * box_id,
                               reid_layout_type_, reid_element_type,
                               aligned_reid_dim, reid_is_big_endian,
                               0, 0, 0));
    float fp_for_this_reid;
    Feature reid;
    for (int32_t reid_i = 0; reid_i < aligned_reid_dim.c; ++reid_i) {
      fp_for_this_reid =
          GetFloatByInt(mxnet_out_for_one_box[reid_i], reid_shift_);
      reid.values.push_back(fp_for_this_reid);
    }
    // l2norm
    l2_norm(reid);
    reids.push_back(std::move(reid));
  }
}

void FasterRCNNImp::GetLMKS2(std::vector<Landmarks> &landmarks,
                             BPU_Buffer_Handle lmks2_label_output,
                             BPU_Buffer_Handle lmks2_offset_output,
                             const std::vector<BBox> &face_boxes) {
  int32_t *label_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(lmks2_label_output));
  int32_t *offset_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(lmks2_offset_output));

  int input_height = lmk_feat_height_ * lmk_feat_stride_;
  int input_width = lmk_feat_width_ * lmk_feat_stride_;
  float base_center = (lmk_feat_stride_ - 1) / 2.0;
  size_t face_box_num = face_boxes.size();

  int label_feature_size = aligned_lmks2_label_dim.h *
                           aligned_lmks2_label_dim.w *
                           aligned_lmks2_label_dim.c;
  int offset_feature_size = aligned_lmks2_offset_dim.h *
                            aligned_lmks2_offset_dim.w *
                            aligned_lmks2_offset_dim.c;

  std::vector<int32_t> mxnet_out_for_one_point(
      lmk_feat_height_ * lmk_feat_width_, 0);

  for (size_t box_id = 0; box_id < face_box_num; ++box_id) {
    const auto &face_box = face_boxes[box_id];
    float x1 = face_box.x1;
    float y1 = face_box.y1;
    float x2 = face_box.x2;
    float y2 = face_box.y2;
    float w = x2 - x1 + 1;
    float h = y2 - y1 + 1;

    assert(input_width != 0 && input_height != 0);
    float scale_x = w / input_width;
    float scale_y = h / input_height;
    float scale_pos_x = lmk_pos_distance_ * scale_x;
    float scale_pos_y = lmk_pos_distance_ * scale_y;

    Landmarks landmark;
    landmark.values.resize(lmk_points_number_);
    for (int kps_id = 0; kps_id < lmk_points_number_; ++kps_id) {
      mxnet_out_for_one_point.assign(
                             lmk_feat_height_ * lmk_feat_width_, 0);
      unsigned best_index = 0;
      CHECK_HBRT_ERROR(hbrtConvertLayoutToNative1HW1(
                     mxnet_out_for_one_point.data(),
                     label_feature + label_feature_size * box_id,
                     lmks2_label_layout_type_, lmk2_label_element_type,
                     aligned_lmks2_label_dim, lmk2_label_is_big_endian,
                     0, kps_id));

      // find the best position
      for (int i = 0; i < lmk_feat_width_ * lmk_feat_height_; ++i) {
        if (mxnet_out_for_one_point[i] > mxnet_out_for_one_point[best_index]) {
          best_index = i;
        }
      }
      float max_score = GetFloatByInt(mxnet_out_for_one_point[best_index],
                                      lmks2_label_shift_);
      // best_index = max_h * lmk_feat_width_ + max_w
      unsigned max_w = best_index % lmk_feat_width_;
      unsigned max_h = best_index / lmk_feat_width_;

      float base_x = (max_w * lmk_feat_stride_ + base_center) * scale_x + x1;
      float base_y = (max_h * lmk_feat_stride_ + base_center) * scale_y + y1;

      mxnet_out_for_one_point.assign(lmk_feat_height_ * lmk_feat_width_, 0);
      // get delta
      CHECK_HBRT_ERROR(hbrtConvertLayoutToNative1HW1(
                             mxnet_out_for_one_point.data(),
                             offset_feature + offset_feature_size * box_id,
                             lmks2_offset_layout_type_,
                             lmk2_offet_element_type, aligned_lmks2_offset_dim,
                             lmk2_offset_is_big_endian, 0, 2*kps_id));
      auto x_delta = mxnet_out_for_one_point[best_index];
      float fp_delta_x = GetFloatByInt(x_delta, lmks2_offset_shift_);
      mxnet_out_for_one_point.assign(lmk_feat_height_ * lmk_feat_width_, 0);
      CHECK_HBRT_ERROR(hbrtConvertLayoutToNative1HW1(
                             mxnet_out_for_one_point.data(),
                             offset_feature + offset_feature_size * box_id,
                             lmks2_offset_layout_type_,
                             lmk2_offet_element_type, aligned_lmks2_offset_dim,
                             lmk2_offset_is_big_endian, 0, 2*kps_id + 1));
      auto y_delta = mxnet_out_for_one_point[best_index];
      float fp_delta_y = GetFloatByInt(y_delta, lmks2_offset_shift_);
      Point point;
      point.x = base_x + fp_delta_x * scale_pos_x;
      point.y = base_y + fp_delta_y * scale_pos_y;
      point.score = SigMoid(max_score);
      landmark.values[kps_id] = point;
    }
    landmarks.push_back(std::move(landmark));
  }
}

void FasterRCNNImp::GetLMKS1(std::vector<Landmarks> &landmarks,
                             BPU_Buffer_Handle output,
                             const std::vector<BBox> &face_boxes) {
  size_t face_box_num = face_boxes.size();
  int32_t *lmks1_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(output));

  int feature_size =
      aligned_lmks1_dim.h * aligned_lmks1_dim.w * aligned_lmks1_dim.c;
  std::vector<int32_t> mxnet_out_for_this_box(aligned_lmks1_dim.c, 0);
  for (size_t box_id = 0; box_id < face_box_num; ++box_id) {
    const auto &face_box = face_boxes[box_id];
    float x1 = face_box.x1;
    float y1 = face_box.y1;
    float x2 = face_box.x2;
    float y2 = face_box.y2;
    float w = x2 - x1 + 1;
    float h = y2 - y1 + 1;

    mxnet_out_for_this_box.assign(aligned_lmks1_dim.c, 0);
    CHECK_HBRT_ERROR(hbrtConvertLayoutToNative111C(
       mxnet_out_for_this_box.data(), lmks1_feature + feature_size * box_id,
       lmks1_layout_type_, lmk1_element_type, aligned_lmks1_dim,
       lmk1_is_big_endian, 0, 0, 0));

    hobot::vision::Landmarks landmark;
    for (int i = 0; i < lmk_points_number_; ++i) {
      float x =
         GetFloatByInt(mxnet_out_for_this_box[2 * i], lmks1_shift_) * w + x1;
      float y =
         GetFloatByInt(mxnet_out_for_this_box[2 * i + 1], lmks1_shift_) * h +
         y1;
      landmark.values.push_back(Point(x, y));
    }
    landmarks.push_back(std::move(landmark));
  }
}

void FasterRCNNImp::GetPose(std::vector<Pose3D> &face_pose,
                            BPU_Buffer_Handle output,
                            const std::vector<BBox> &face_boxes) {
  size_t face_box_num = face_boxes.size();
  int32_t *pose_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(output));

  int feature_size = aligned_face_pose_dim.h * aligned_face_pose_dim.w *
                     aligned_face_pose_dim.c;
  std::vector<int32_t> mxnet_out_for_one_box(aligned_face_pose_dim.c, 0);
  for (size_t box_id = 0; box_id < face_box_num; ++box_id) {
    mxnet_out_for_one_box.assign(aligned_face_pose_dim.c, 0);
    CHECK_HBRT_ERROR(hbrtConvertLayoutToNative111C(
       mxnet_out_for_one_box.data(), pose_feature + feature_size * box_id,
       face_pose_layout_type_, face_pose_element_type, aligned_face_pose_dim,
       face_pose_is_big_endian, 0, 0, 0));
    hobot::vision::Pose3D pose;
    pose.yaw = GetFloatByInt(mxnet_out_for_one_box[0], face_pose_shift_) * 90.0;
    pose.pitch =
        GetFloatByInt(mxnet_out_for_one_box[1], face_pose_shift_) * 90.0;
    pose.roll =
        GetFloatByInt(mxnet_out_for_one_box[2], face_pose_shift_) * 90.0;
    face_pose.push_back(pose);
  }
}

void FasterRCNNImp::GetPlateColor(
    std::vector<hobot::vision::Attribute<int>> *plates_color,
    BPU_Buffer_Handle output,
    const std::vector<hobot::vision::BBox> &plate_boxes) {
  size_t plate_box_num = plate_boxes.size();
  int32_t *plate_color_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(output));

  int feature_size = aligned_plate_color_dim.h * aligned_plate_color_dim.w *
                     aligned_plate_color_dim.c;
  std::vector<int32_t> mxnet_out_for_one_box(aligned_plate_color_dim.c, 0);

  LOGD << "plate color: ";
  for (size_t box_id = 0; box_id < plate_box_num; ++box_id) {
    mxnet_out_for_one_box.assign(aligned_plate_color_dim.c, 0);
    CHECK_HBRT_ERROR(hbrtConvertLayoutToNative111C(
                          mxnet_out_for_one_box.data(),
                          plate_color_feature + feature_size * box_id,
                          plate_color_layout_type_, plate_color_element_type,
                          aligned_plate_color_dim, plate_color_is_big_endian,
                          0, 0, 0));

    hobot::vision::Attribute<int> one_plate_color;
    int max_index = 0;
    float max_score = -1000;
    for (int32_t color_index = 0; color_index < plate_color_num_;
         ++color_index) {
      float color_score =
          GetFloatByInt(mxnet_out_for_one_box[color_index], plate_color_shift_);
      if (color_score > max_score) {
        max_score = color_score;
        max_index = color_index;
      }
    }
    one_plate_color.value = max_index;
    one_plate_color.score = max_score;
    LOGD << "value: " << one_plate_color.value
         << ", score: " << one_plate_color.score << "\n";
    plates_color->push_back(one_plate_color);
  }
}

void FasterRCNNImp::GetPlateRow(
    std::vector<hobot::vision::Attribute<int>> *plates_row,
    BPU_Buffer_Handle output,
    const std::vector<hobot::vision::BBox> &plate_boxes) {
  size_t plate_box_num = plate_boxes.size();
  int32_t *plate_row_feature =
      reinterpret_cast<int32_t *>(BPU_getRawBufferPtr(output));

  int feature_size = aligned_plate_row_dim.h * aligned_plate_row_dim.w *
                     aligned_plate_row_dim.c;
  std::vector<int32_t> mxnet_out_for_one_box(aligned_plate_row_dim.c, 0);

  LOGD << "plate row: ";
  for (size_t box_id = 0; box_id < plate_box_num; ++box_id) {
    mxnet_out_for_one_box.assign(aligned_plate_row_dim.c, 0);
    CHECK_HBRT_ERROR(hbrtConvertLayoutToNative111C(
                            mxnet_out_for_one_box.data(),
                            plate_row_feature + feature_size * box_id,
                            plate_row_layout_type_, plate_row_element_type,
                            aligned_plate_row_dim, plate_row_is_big_endian,
                            0, 0, 0));

    hobot::vision::Attribute<int> one_plate_row;

    int max_index = 0;
    float max_score = -1000;
    for (int32_t row_index = 0; row_index < plate_row_num_; ++row_index) {
      float row_score =
          GetFloatByInt(mxnet_out_for_one_box[row_index], plate_row_shift_);
      if (row_score > max_score) {
        max_score = row_score;
        max_index = row_index;
      }
    }
    one_plate_row.value = max_index;
    one_plate_row.score = max_score;
    LOGD << "value: " << one_plate_row.value
         << ", score: " << one_plate_row.score << "\n";
    plates_row->push_back(one_plate_row);
  }
}

void FasterRCNNImp::Finalize() {
  hobot::vision::BPUModelManager::Get().ReleaseBpuHandle(model_file_path_);
  LOGD << "release " << model_file_path_ << "\n";
}

}  // namespace faster_rcnn_method
