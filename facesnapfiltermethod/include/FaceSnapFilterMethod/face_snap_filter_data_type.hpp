/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @brief     face snap filter data type header
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2019.05.27
 */

#ifndef FACESNAPFILTERMETHOD_FACE_SNAP_FILTER_DATA_TYPE_HPP_
#define FACESNAPFILTERMETHOD_FACE_SNAP_FILTER_DATA_TYPE_HPP_

#include <string>

namespace HobotXRoc {

#define SET_FACE_SNAP_FILTER_ERROR_CODE(err_desp, key)                       \
        if (err_desp.isMember(#key) && err_desp[#key].isInt()) {             \
          key##_err_code = err_desp[#key].asInt();                           \
          LOGV << #key << "_err_code: " << key##_err_code;                   \
        }

#define SET_FACE_SNAP_FILTER_METHOD_PARAM(json_cfg, type, key)               \
        if (json_cfg.isMember(#key) && json_cfg[#key].is##type()) {          \
          key = json_cfg[#key].as##type();                                   \
          config_jv_all_[#key] =  key;                                       \
          LOGV << #key << ": " << key;                                       \
        }

struct FaceSnapFilterParam : public HobotXRoc::InputParam {
  explicit FaceSnapFilterParam(const std::string &content = "")
      : InputParam("FaceSnapFilterMethod") {
    is_enable_this_method_ = true;
    is_json_format_ = true;
    method_name_ = "FaceSnapFilterMethod";
    UpdateParameter(content);
  }
  virtual int UpdateParameter(const std::string &content) {
    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    JSONCPP_STRING error;
    std::shared_ptr<Json::CharReader> json_reader(builder.newCharReader());
    try {
      bool ret = json_reader->parse(content.c_str(), content.c_str()
          + content.size(), &config_jv, &error);
      SetFilterParam();
      SetFilterErrCode();
      float black_area_iou_thr = 0.5;
      auto jv_data = config_jv["black_area_iou_thr"];
      if (!jv_data.isNull()) {
        black_area_iou_thr = jv_data.asFloat();
      }
      jv_data = config_jv["black_area_list"];
      if (!jv_data.isNull()) {
        black_list_module.ClearBlackAreaList();
        int black_area_size = jv_data.size();
        for (int i = 0; i < black_area_size; ++i) {
          auto one_area = jv_data[i];
          if (one_area.isNull()) {
            continue;
          }
          int data_size = one_area.size();
          assert(data_size == 4 || data_size == 5);
          int x1 = one_area[0].asInt();
          int y1 = one_area[1].asInt();
          int x2 = one_area[2].asInt();
          int y2 = one_area[3].asInt();
          float iou_thr = black_area_iou_thr;
          if (data_size == 5) {
            iou_thr = one_area[4].asFloat();
          }
          black_list_module.AddBlackList(x1, y1, x2, y2, iou_thr);
        }
      }
      jv_data = config_jv["white_area_list"];
      if (!jv_data.isNull()) {
        white_list_module.ClearWhiteAreaList();
        int white_area_size = jv_data.size();
        for (int i = 0; i < white_area_size; ++i) {
          auto one_area = jv_data[i];
          if (one_area.isNull()) {
            continue;
          }
          int data_size = one_area.size();
          assert(data_size == 4);
          int x1 = one_area[0].asInt();
          int y1 = one_area[1].asInt();
          int x2 = one_area[2].asInt();
          int y2 = one_area[3].asInt();
          white_list_module.AddWhiteList(x1, y1, x2, y2);
        }
      }
      if (ret) {
        return 0;
      } else {
        return -1;
      }
    } catch (std::exception &e) {
      return -1;
    }
  }

  void SetOccludeErrCode() {
    auto &desp = config_jv["err_description"];
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, left_eye_occluded_thr);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, right_eye_occluded_thr);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, left_brow_occluded_thr);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, right_brow_occluded_thr);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, forehead_occluded_thr);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, left_cheek_occluded_thr);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, right_cheek_occluded_thr);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, nose_occluded_thr);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, mouth_occluded_thr);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, jaw_occluded_thr);
  }

  void SetFilterErrCode() {
    auto &desp = config_jv["err_description"];
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, passed);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, snap_area);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, snap_size_thr);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, frontal_thr);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, quality_thr);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, lmk_thr);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, pv_thr);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, black_list);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, big_face);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, brightness);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, abnormal_thr);
    SET_FACE_SNAP_FILTER_ERROR_CODE(desp, expand_thr);
    SetOccludeErrCode();
  }

  void SetOccludeParam() {
    SET_FACE_SNAP_FILTER_METHOD_PARAM(
        config_jv, Double, left_eye_occluded_thr);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(
        config_jv, Double, right_eye_occluded_thr);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(
        config_jv, Double, left_brow_occluded_thr);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(
        config_jv, Double, right_brow_occluded_thr);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(
        config_jv, Double, forehead_occluded_thr);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(
        config_jv, Double, left_cheek_occluded_thr);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(
        config_jv, Double, right_cheek_occluded_thr);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(
        config_jv, Double, nose_occluded_thr);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(
        config_jv, Double, mouth_occluded_thr);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(
        config_jv, Double, jaw_occluded_thr);
  }

  void SetFilterParam() {
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Int, image_width);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Int, image_height);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Int, snap_size_thr);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Double, frontal_pitch_thr);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Double, frontal_yaw_thr);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Double, frontal_roll_thr);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Double, frontal_thr);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Bool, filter_with_frontal_thr);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Double, quality_thr);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Double, lmk_thr);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Int, lmk_filter_num);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Double, pv_thr);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Int, bound_thr_w);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Int, bound_thr_h);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Int, max_box_counts);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Int, brightness_min);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Int, brightness_max);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Double, abnormal_thr);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Double, expand_scale);
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, Int, filter_status);
    SetOccludeParam();
    std::string norm_method;
    SET_FACE_SNAP_FILTER_METHOD_PARAM(config_jv, String, norm_method);
    if (norm_method.empty()) {
      LOGW << "not set norm_method";
    } else {
      auto iter = norm_method_map_.find(norm_method);
      if (iter == norm_method_map_.end()) {
        LOGW << "unknown norm_method:" << norm_method;
      } else {
        e_norm_method = iter->second;
      }
    }
    LOGV << "norm_method:" << norm_method;

    if (filter_with_frontal_thr)
      LOGD << "pose filter with frontal_thr, thr is " << frontal_thr;
    else
      LOGD << "pose filter with pitch: " << frontal_pitch_thr
           << " yaw: " << frontal_yaw_thr
           << " roll: " << frontal_roll_thr;
  }

  std::string Format() override {
    return config_jv_all_.toStyledString();
  };

  Json::Value config_jv_all_;          // contain all params
  Json::Value config_jv;              // contain param updated
  int image_width = 960;              // the width of image frame
  int image_height = 540;             // the height of image frame
  int snap_size_thr = 100;            // the threshold of snap size
  float frontal_pitch_thr = 0;   // the threshold of pitch value in frontal area
  float frontal_yaw_thr = 0;     // the threshold of yaw value in frontal area
  float frontal_roll_thr = 0;    // the threshold of roll value in frontal area
  float frontal_thr = 0;    // the threshold of overall value in frontal area
  bool filter_with_frontal_thr = false;    // the switch of frontal filter type
  float quality_thr = 0.0;            // the threshold of snap quality
  float lmk_thr = 2.0;                // the threshold of snap landmarks score
  int lmk_filter_num = 0;             // the num of snap landmarks filter
  float pv_thr = 0.5;                 // the threshold of snap face rect score
  int bound_thr_w = 100;              // the boundary of crop area
  int bound_thr_h = 100;
  int filter_status = 1;
  BlackListsModule
      black_list_module;  // we will filter the snaps in the blacklist area
  WhiteListsModule
      white_list_module; // we will allow the snaps in the whitelist area
  int max_box_counts = 0;            // the max count of boxes
  int brightness_min = 0;
  int brightness_max = 4;
  float left_eye_occluded_thr = 0.5;
  float right_eye_occluded_thr = 0.5;
  float left_brow_occluded_thr = 0.5;
  float right_brow_occluded_thr = 0.5;
  float forehead_occluded_thr = 0.5;
  float left_cheek_occluded_thr = 0.5;
  float right_cheek_occluded_thr = 0.5;
  float nose_occluded_thr = 0.5;
  float mouth_occluded_thr = 0.5;
  float jaw_occluded_thr = 0.5;
  float abnormal_thr = 1.0;
  float expand_scale = 1.0;
  NormMethod e_norm_method = NormMethod::BPU_MODEL_NORM_BY_NOTHING;
  int passed_err_code = -1;
  int snap_area_err_code = -1;
  int snap_size_thr_err_code = -1;
  int frontal_thr_err_code = -1;
  int quality_thr_err_code = -1;
  int lmk_thr_err_code = -1;
  int pv_thr_err_code = -1;
  int black_list_err_code = -1;
  int white_list_err_code = -1;
  int big_face_err_code = -1;
  int brightness_err_code = -1;
  int abnormal_thr_err_code = -1;
  int expand_thr_err_code = -1;
  int left_eye_occluded_thr_err_code = -1;
  int right_eye_occluded_thr_err_code = -1;
  int left_brow_occluded_thr_err_code = -1;
  int right_brow_occluded_thr_err_code = -1;
  int forehead_occluded_thr_err_code = -1;
  int left_cheek_occluded_thr_err_code = -1;
  int right_cheek_occluded_thr_err_code = -1;
  int nose_occluded_thr_err_code = -1;
  int mouth_occluded_thr_err_code = -1;
  int jaw_occluded_thr_err_code = -1;
  static const std::map<std::string, NormMethod> norm_method_map_;
};
const std::map<std::string, NormMethod>
        FaceSnapFilterParam::norm_method_map_ = {
    {"norm_by_width_length", NormMethod::BPU_MODEL_NORM_BY_WIDTH_LENGTH},
    {"norm_by_width_ratio", NormMethod::BPU_MODEL_NORM_BY_WIDTH_RATIO},
    {"norm_by_height_rario", NormMethod::BPU_MODEL_NORM_BY_HEIGHT_RATIO},
    {"norm_by_lside_ratio", NormMethod::BPU_MODEL_NORM_BY_LSIDE_RATIO},
    {"norm_by_height_length", NormMethod::BPU_MODEL_NORM_BY_HEIGHT_LENGTH},
    {"norm_by_lside_length", NormMethod::BPU_MODEL_NORM_BY_LSIDE_LENGTH},
    {"norm_by_lside_square", NormMethod::BPU_MODEL_NORM_BY_LSIDE_SQUARE},
    {"norm_by_diagonal_square", NormMethod::BPU_MODEL_NORM_BY_DIAGONAL_SQUARE},
    {"norm_by_nothing", NormMethod::BPU_MODEL_NORM_BY_NOTHING}
  };

}  // namespace HobotXRoc


#endif // FACESNAPFILTERMETHOD_FACE_SNAP_FILTER_DATA_TYPE_HPP_
