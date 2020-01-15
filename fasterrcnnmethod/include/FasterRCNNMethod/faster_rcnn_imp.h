//
// Created by yaoyao.sun on 2019-04-23.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#ifndef INCLUDE_FASTERRCNNMETHOD_FASTER_RCNN_IMP_H_
#define INCLUDE_FASTERRCNNMETHOD_FASTER_RCNN_IMP_H_

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <mutex>
#include <unordered_map>
#include <atomic>
#include <utility>
#include "hobotxsdk/xroc_data.h"
#include "horizon/vision_type/vision_type.hpp"
#include "hobotxroc/method.h"
#include "bpu_predict/bpu_predict.h"
#include "hbdk/hbdk_layout.h"
#include "hbdk/hbdk_hbrt.h"
#include "hobotxroc/method.h"
#include "3rd_party_lib/plat_cnn.h"
#include "result.h"
#include "config.h"
#include "hobot_vision/bpu_handle_manager.hpp"

namespace faster_rcnn_method {

enum class FasterRCNNBranchOutType {
  BBOX,
  KPS,
  MASK,
  REID,
  LMKS2_LABEL,
  LMKS2_OFFSET,
  LMKS1,
  POSE_3D,
  PLATE_COLOR,
  PLATE_ROW,
  INVALID
};

struct FasterRCNNBranchInfo {
  FasterRCNNBranchOutType type;
  std::string name;
  std::string box_name;
};

class FasterRCNNImp {
 public:
  FasterRCNNImp() {}

  virtual ~FasterRCNNImp() {}

  // return 0 for successed, -1 for failed.
  int Init(const std::string &config_file);

  // return 0 for successed,  else failed.
  int UpdateParameter(HobotXRoc::InputParamPtr ptr) {
    faster_rcnn_param_ = ptr;
    return 0;
  }

  HobotXRoc::InputParamPtr GetParameter() const { return faster_rcnn_param_; }

  std::string GetVersion() const { return model_version_; }

  void RunSingleFrame(const std::vector<HobotXRoc::BaseDataPtr> &frame_input,
                      std::vector<HobotXRoc::BaseDataPtr> &frame_output);
  // xroc framework will call this function to do something, like release
  // resources.
  void Finalize();

 private:
  void ParseConfig(const std::string &config_file);

  void GetModelInfo(const std::string &model_name);

  void GetFrameOutput(int src_img_width, int src_img_height,
                      std::vector<HobotXRoc::BaseDataPtr> &frame_output);

  void PostProcess(FasterRCNNOutMsg &det_result);

  // get face, head or body boxes.
  void GetRects(std::vector<hobot::vision::BBox> &boxes,
                BPU_Buffer_Handle output);

  // get face, head or body boxes from rpp.

  void GetRppRects(std::vector<hobot::vision::BBox> &boxes,
                   BPU_Buffer_Handle output);
  // get body skeleton.
  void GetKps(std::vector<hobot::vision::Landmarks> &kpss,
              BPU_Buffer_Handle output,
              const std::vector<hobot::vision::BBox> &body_boxes);
  // get body segmentation
  void GetMask(std::vector<hobot::vision::Segmentation> &masks,
               BPU_Buffer_Handle output,
               const std::vector<hobot::vision::BBox> &body_boxes);
  void GetReid(std::vector<hobot::vision::Feature> &reids,
               BPU_Buffer_Handle output,
               const std::vector<hobot::vision::BBox> &body_boxes);
  // get face lmk2
  void GetLMKS2(std::vector<hobot::vision::Landmarks> &landmarks,
                BPU_Buffer_Handle lmks2_label_output,
                BPU_Buffer_Handle lmks2_offset_output,
                const std::vector<hobot::vision::BBox> &face_boxes);
  // get face lmk1, lmk1 means regression lmk, used to supplement lmk2.
  void GetLMKS1(std::vector<hobot::vision::Landmarks> &landmarks,
                BPU_Buffer_Handle output,
                const std::vector<hobot::vision::BBox> &face_boxes);
  // get face 3dpose.
  void GetPose(std::vector<hobot::vision::Pose3D> &face_pose,
               BPU_Buffer_Handle output,
               const std::vector<hobot::vision::BBox> &face_boxes);
  // get vehicle plate color.
  void GetPlateColor(std::vector<hobot::vision::Attribute<int>> *plate_color,
                     BPU_Buffer_Handle output,
                     const std::vector<hobot::vision::BBox> &plate_box);

  // get vehicle plate row.
  void GetPlateRow(std::vector<hobot::vision::Attribute<int>> *plate_row,
                   BPU_Buffer_Handle output,
                   const std::vector<hobot::vision::BBox> &plate_box);

  std::string bpu_config_path_;
  std::string model_file_path_;
  std::string model_name_;
  std::string model_version_;
  int pyramid_layer_;
  BPUHandle bpu_handle_;
  std::vector<BPU_Buffer_Handle> out_buf_;
  BPUModelInfo output_info_;
  std::map<int, FasterRCNNBranchInfo> out_level2rcnn_branch_info_;
  std::vector<std::string> method_outs_;

  std::shared_ptr<Config> config_;

  int model_input_width_;
  int model_input_height_;

  float kps_pos_distance_;
  int kps_feat_width_;
  int kps_feat_height_;
  int kps_points_number_;

  float lmk_pos_distance_;
  int lmk_feat_width_;
  int lmk_feat_height_;
  int lmk_feat_stride_;
  int lmk_points_number_;
  int face_pose_number_;

  int32_t plate_color_num_;
  int32_t plate_row_num_;

  // TODO(yaoyao.sun) use vector<uint32_t>
  uint32_t kps_shift_ = 0;
  uint32_t mask_shift_ = 0;
  uint32_t reid_shift_ = 0;
  uint32_t lmks2_label_shift_ = 0;
  uint32_t lmks2_offset_shift_ = 0;
  uint32_t lmks1_shift_ = 0;
  uint32_t face_pose_shift_ = 0;
  uint32_t plate_color_shift_ = 0;
  uint32_t plate_row_shift_ = 0;

  hbrt_layout_type_t kps_layout_type_;
  hbrt_layout_type_t mask_layout_type_;
  hbrt_layout_type_t reid_layout_type_;
  hbrt_layout_type_t lmks2_label_layout_type_;
  hbrt_layout_type_t lmks2_offset_layout_type_;
  hbrt_layout_type_t lmks1_layout_type_;
  hbrt_layout_type_t face_pose_layout_type_;
  hbrt_layout_type_t plate_color_layout_type_;
  hbrt_layout_type_t plate_row_layout_type_;

  hbrt_dimension_t aligned_reid_dim;
  hbrt_dimension_t aligned_mask_dim;
  hbrt_dimension_t aligned_kps_dim;
  hbrt_dimension_t aligned_lmks2_label_dim;
  hbrt_dimension_t aligned_lmks2_offset_dim;
  hbrt_dimension_t aligned_lmks1_dim;
  hbrt_dimension_t aligned_face_pose_dim;
  hbrt_dimension_t aligned_plate_color_dim;
  hbrt_dimension_t aligned_plate_row_dim;

  bool  reid_is_big_endian;
  bool  mask_is_big_endian;
  bool  kps_is_big_endian;
  bool  lmk1_is_big_endian;
  bool  lmk2_label_is_big_endian;
  bool  lmk2_offset_is_big_endian;
  bool  face_pose_is_big_endian;
  bool  plate_color_is_big_endian;
  bool  plate_row_is_big_endian;

  hbrt_element_type_t reid_element_type;
  hbrt_element_type_t mask_element_type;
  hbrt_element_type_t kps_element_type;
  hbrt_element_type_t lmk1_element_type;
  hbrt_element_type_t lmk2_label_element_type;
  hbrt_element_type_t lmk2_offet_element_type;
  hbrt_element_type_t face_pose_element_type;
  hbrt_element_type_t plate_color_element_type;
  hbrt_element_type_t plate_row_element_type;

  HobotXRoc::InputParamPtr faster_rcnn_param_;
};

}  // namespace faster_rcnn_method

#endif // INCLUDE_FASTERRCNNMETHOD_FASTER_RCNN_IMP_H_
