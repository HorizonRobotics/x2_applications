/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: CNNMethod.h
 * @Brief: declaration of the CNNMethod
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-04-15 14:18:28
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-04-15 16:01:54
 */

#ifndef INCLUDE_CNNMETHOD_CNNMETHOD_H_
#define INCLUDE_CNNMETHOD_CNNMETHOD_H_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "hobotxroc/method.h"

namespace HobotXRoc {
class PostPredictor;
class Predictor;
class CNNMethodConfig;

class CNNMethod : public Method {
 public:
  CNNMethod() {}
  virtual ~CNNMethod() {}

  virtual int Init(const std::string &cfg_path);
  virtual void Finalize();

  virtual std::vector<std::vector<BaseDataPtr> >
  DoProcess(const std::vector<std::vector<BaseDataPtr> > &input,
            const std::vector<HobotXRoc::InputParamPtr> &param);
  virtual int UpdateParameter(HobotXRoc::InputParamPtr ptr);
  virtual InputParamPtr GetParameter() const;
  virtual std::string GetVersion() const;
  virtual void OnProfilerChanged(bool on) {}

 private:
  std::shared_ptr<Predictor> predictor_;
  std::shared_ptr<PostPredictor> post_predict_;

  std::shared_ptr<CNNMethodConfig> config_;
  static std::mutex init_mutex_;
};
}  // namespace HobotXRoc
#endif  // INCLUDE_CNNMETHOD_CNNMETHOD_H_
