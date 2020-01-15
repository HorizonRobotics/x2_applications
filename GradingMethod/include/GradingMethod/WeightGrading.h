/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     WeightGrading Method
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.25
 */

#ifndef GRADINGMETHOD_WEIGHTGRADING_H_
#define GRADINGMETHOD_WEIGHTGRADING_H_

#include <vector>
#include <string>

#include "GradingMethod.h"
#include "horizon/vision_type/vision_type.hpp"
#include "json/json.h"
#include "grading_method_data_type.hpp"

namespace HobotXRoc {

struct WeightGradingParam : GradingParam {
 public:
  explicit WeightGradingParam(const std::string &content)
      : GradingParam(content) {
    UpdateParameter(content);
  }
  int UpdateParameter(const std::string &content) override;
  float size_min = 0;
  float size_max = 0;
  float size_inflexion = 0;
  float frontal_thr = 0;
  float size_weight = 0;
  float pose_weight = 0;
  float lmk_weight = 0;
  float quality_weight = 0;
  float normalize_lmk_divisor = 75;
};


class WeightGrading : public Grading {
 public:
  int GradingInit(const std::string &config_file_path) override;

  int ProcessFrame(const std::vector<BaseDataPtr> &in,
                   const InputParamPtr &param,
                   std::vector<BaseDataPtr> &out) override;

  void GradingFinalize() override;

  InputParamPtr GetParameter() override;

  int UpdateParameter(const std::string &content) override;

 private:
  typedef XRocData<hobot::vision::BBox> XRocBBox;

  typedef XRocData<hobot::vision::Pose3D> XRocPose3D;

  typedef XRocData<hobot::vision::Landmarks> XRocLandmarks;

  typedef XRocData<hobot::vision::Quality> XRocQuality;

  typedef XRocData<float> XRocFloat;

  float ProcessRect(XRocBBox &bbox);

  float ProcessPose3d(XRocPose3D &pose3d);

  float ProcessLmk(XRocLandmarks &lmk);

  float ProcessQuality(XRocQuality &quality);

  std::shared_ptr<WeightGradingParam> config_param_;
};
} // namespace HobotXRoc

#endif // GRADINGMETHOD_WEIGHTGRADING_H_
