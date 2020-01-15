/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @brief     the interface of FaceSnapFilterMethod
 * @author    hangjun.yang, chao.yang
 * @email     hangjun.yang@horizon.ai, chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2019.01.11
 */

#ifndef FACESNAPFILTERMETHOD_FACESNAPFILTERMETHOD_H_
#define FACESNAPFILTERMETHOD_FACESNAPFILTERMETHOD_H_

#include <map>
#include <cassert>
#include <memory>
#include <vector>
#include <string>
#include <utility>
#include <list>
#include <algorithm>
#include "hobotxroc/method.h"
#include "horizon/vision_type/vision_type.hpp"

namespace HobotXRoc {

struct BlackArea {
  float x1_ = 0;
  float y1_ = 0;
  float x2_ = 0;
  float y2_ = 0;
  float iou_threshold_ = 0.f;
};
enum class NormMethod {
  BPU_MODEL_NORM_BY_WIDTH_LENGTH,
  BPU_MODEL_NORM_BY_WIDTH_RATIO,
  BPU_MODEL_NORM_BY_HEIGHT_RATIO,
  BPU_MODEL_NORM_BY_LSIDE_RATIO,
  BPU_MODEL_NORM_BY_HEIGHT_LENGTH,
  BPU_MODEL_NORM_BY_LSIDE_LENGTH,
  BPU_MODEL_NORM_BY_LSIDE_SQUARE,
  BPU_MODEL_NORM_BY_DIAGONAL_SQUARE,
  BPU_MODEL_NORM_BY_NOTHING
};
class BlackListsModule {
 public:
  BlackListsModule() = default;

  std::list<BlackArea>& GetBlackAreaList() {
    return black_area_list_;
  }

  void ClearBlackAreaList() {
    black_area_list_.clear();
  }

  void AddBlackList(BlackArea black_area) {
    black_area_list_.push_back(black_area);
  }

  void AddBlackList(float x1, float y1, float x2, float y2, float threshold) {
    BlackArea black_area;
    black_area.x1_ = x1;
    black_area.y1_ = y1;
    black_area.x2_ = x2;
    black_area.y2_ = y2;
    black_area.iou_threshold_ = threshold;
    black_area_list_.push_back(black_area);
  }

  bool IsSameBBox(
      float x1, float y1, float x2, float y2, BlackArea black_area) {
    float black_area_x1 = black_area.x1_;
    float black_area_y1 = black_area.y1_;
    float black_area_x2 = black_area.x2_;
    float black_area_y2 = black_area.y2_;

    float l_inter = std::max(x1, black_area_x1);
    float r_inter = std::min(x2, black_area_x2);
    if (l_inter >= r_inter) {
      return false;
    }
    float t_inter = std::max(y1, black_area_y1);
    float b_inter = std::min(y2, black_area_y2);
    if (t_inter >= b_inter) {
      return false;
    }
    float w_inter = r_inter - l_inter;
    float h_inter = b_inter - t_inter;
    float area_inter = w_inter * h_inter;
    float area_box1 = (x2 - x1) * (y2 - y1);
    float iou = area_inter / area_box1;
    return iou >= black_area.iou_threshold_;
  }

 private:
  std::list<BlackArea> black_area_list_;
};

struct WhiteArea {
  float x1_ = 0;
  float y1_ = 0;
  float x2_ = 0;
  float y2_ = 0;
};

class WhiteListsModule {
 public:
  WhiteListsModule() = default;

  const std::list<WhiteArea>& GetWhiteAreaList() const {
    return white_area_list_;
  }

  void ClearWhiteAreaList() {
    white_area_list_.clear();
  }

  void AddWhiteList(float x1, float y1, float x2, float y2) {
    WhiteArea white_area;
    white_area.x1_ = x1;
    white_area.y1_ = y1;
    white_area.x2_ = x2;
    white_area.y2_ = y2;
    white_area_list_.emplace_back(white_area);
  }

  static bool IsInZone(
          float x1, float y1, float x2, float y2,
          const WhiteArea& white_area) {
    float c_x = x1 + (x2 - x1) * 0.5;
    float c_y = y1 + (y2 - y1) * 0.5;
    if ((c_x >= white_area.x1_)
        && (c_x <= white_area.x2_)
        && (c_y >= white_area.y1_)
        && (c_y <= white_area.y2_)) {
      return true;
    } else {
      return false;
    }
  }

  bool IsInZone(
          float x1, float y1, float x2, float y2) {
    if (white_area_list_.empty()) return true;

    float c_x = x1 + (x2 - x1) * 0.5;
    float c_y = y1 + (y2 - y1) * 0.5;
    for (const auto& white_area : white_area_list_) {
      if ((c_x >= white_area.x1_)
          && (c_x <= white_area.x2_)
          && (c_y >= white_area.y1_)
          && (c_y <= white_area.y2_)) {
        return true;
      }
    }
    return false;
  }

 private:
  std::list<WhiteArea> white_area_list_;
};

struct FaceSnapFilterParam;

typedef std::shared_ptr<FaceSnapFilterParam> FaceSnapFilterParamPtr;
typedef std::shared_ptr<BaseDataVector> BaseDataVectorPtr;

class FaceSnapFilterMethod : public Method {
 public:
  int Init(const std::string &config_file_path) override;

  /**
   * input[0] : [face_box(BBox), pose(Pose3D), landmark(Landmark), quality(Quality), age(Age), gender(Gender)]
   *       required params: face_box/pose/landmark
   *       optional params: quality, age, gender
   * output[0]: [face_box, pose, landmark, quality, age, gender]
   *       optional params: quality, age, gender same as inputs
   * */
  std::vector<std::vector<BaseDataPtr>> DoProcess(
      const std::vector<std::vector<BaseDataPtr>> &input,
      const std::vector<HobotXRoc::InputParamPtr> &param) override;

  void Finalize() override;

  int UpdateParameter(InputParamPtr ptr) override;

  InputParamPtr GetParameter() const override;

  std::string GetVersion() const override { return "0.0.15"; }

  void OnProfilerChanged(bool on) override { }

 private:
  typedef XRocData<hobot::vision::ImageFrame> XRocImageFrame;
  typedef XRocData<hobot::vision::BBox> XRocBBox;
  typedef XRocData<hobot::vision::Pose3D> XRocPose3D;
  typedef XRocData<hobot::vision::Landmarks> XRocLandmarks;
  typedef XRocData<hobot::vision::Quality> XRocQuality;
  typedef XRocData<hobot::vision::Age> XRocAge;
  typedef XRocData<hobot::vision::Gender> XRocGender;
  typedef XRocData<hobot::vision::Attribute<int32_t>> XRocAttribute;
  typedef XRocData<int32_t> XRocFilterDescription;

  /**
   * filter by 3d pose
   * */
  bool ReachPoseThreshold(const float &pitch,
                          const float &yaw,
                          const float &roll = 0);
  /**
   * filter by size
   * */
  bool ReachSizeThreshold(const float &x1,
                          const float &y1,
                          const float &x2,
                          const float &y2);
  /**
   * filter by quality
   * */
  bool ReachQualityThreshold(const float &quality = 0);

  /**
   * filter by brightness
   * */
  bool ValidBrightness(const int &brightness = 0);

  /**
   * filter by occlude
   * */
  bool IsOccluded(const float &val = 0, const float &thr = 0);

  /**
 * filter by abnormal condition
 * */
  bool IsAbnormal(const float &val = 0);

  /**
   * filter by face rect score
   * */
  bool PassPostVerification(const float &bbox_score);

  /**
   * filter by face landmark score
   * */
  bool LmkVerification(const std::shared_ptr<XRocLandmarks> &lmk);

  /**
   * if not in the snap area, return false
   * */
  bool IsWithinSnapArea(const float &x1,
                        const float &y1,
                        const float &x2,
                        const float &y2,
                        const int &image_width,
                        const int &image_height);
  /**
   * if not in the blacklist area, return false
   * */
  bool IsWithinBlackListArea(const float &x1,
                             const float &y1,
                             const float &x2,
                             const float &y2);
  /**
   * if not in the whitelist area, return false
   * */
  bool IsWithinWhiteListArea(const float &x1,
                             const float &y1,
                             const float &x2,
                             const float &y2);
  /**
   * filter by expand
   * */
  bool ExpandThreshold(hobot::vision::BBox *box);

  BaseDataVectorPtr ConstructFilterOutputSlot0(const size_t &num);

  template<typename T>
  BaseDataVectorPtr CopyAttribute(const BaseDataVectorPtr &attr) {
    BaseDataVectorPtr ret = std::make_shared<BaseDataVector>();
    int attr_size = attr->datas_.size();
    if (0 == attr_size) {
      return ret;
    }
    for (auto data : attr->datas_) {
      auto actual_data = std::static_pointer_cast<T>(data);
      std::shared_ptr<T> copy_data = std::make_shared<T>();
      *copy_data = *actual_data;
      ret->datas_.push_back(std::static_pointer_cast<BaseData>(copy_data));
    }
    return ret;
  }

  BaseDataVectorPtr CopyAttribute(\
              const BaseDataVectorPtr &attr,
              const std::string &data_type);


 private:
  FaceSnapFilterParamPtr filter_param_ = nullptr;
  // todo: read from config file
  std::vector<std::string> input_data_types_ = {"bbox",
                                                "pose3D",
                                                "landmark",
                                                "blur",
                                                "brightness",
                                                "eye_abnormalities",
                                                "mouth_abnormal",
                                                "left_eye",
                                                "right_eye",
                                                "left_brow",
                                                "right_brow",
                                                "forehead",
                                                "left_cheek",
                                                "right_cheek",
                                                "nose",
                                                "mouth",
                                                "jaw"};

  void Copy2Output(const std::vector<BaseDataVectorPtr> &input_slot,
                   std::vector<BaseDataVectorPtr> *p_output_slot,
                   bool pass_through);

  int isValid(const std::vector<BaseDataVectorPtr> &input_slot,
               int face_idx);

  void ProcessOneBatch(const std::vector<BaseDataPtr> &batch_i,
                       std::vector<BaseDataPtr> *p_frame_output,
                       const std::shared_ptr<InputParam> &param_i);

  void BigFaceFilter(int frame_input_size, int face_size,
      std::vector<BaseDataVectorPtr> *p_output_slot) const;

  void AttributeFilter(int frame_input_size,
                       int face_size,
                       const std::vector<BaseDataVectorPtr> &input_slot,
                       const std::vector<BaseDataVectorPtr> &output_slot);

  int NormalizeRoi(hobot::vision::BBox *src,
                   float norm_ratio,
                   NormMethod norm_method,
                   uint32_t total_w,
                   uint32_t total_h);

  float GetOccludeVal(const std::string &name);

  int GetOccludeErrCode(const std::string &name);

  template<typename T>
  T ValueNorm(const T &min, const T &max, const T &val) {
    if (val > max)  return max;
    if (val < min)  return min;
    return val;
  }
};
}  // namespace HobotXRoc
#endif  // FACESNAPFILTERMETHOD_FACESNAPFILTERMETHOD_H_
