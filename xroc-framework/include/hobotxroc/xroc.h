/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides xroc framework interface
 * @file      xroc.h
 * @author    chuanyi.yang
 * @email     chuanyi.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */
#ifndef HOBOTXROC_XROC_H_
#define HOBOTXROC_XROC_H_

#include <mutex>
#include <string>
#include <vector>
#include "hobotxroc/scheduler.h"
#include "hobotxsdk/xroc_sdk.h"

namespace HobotXRoc {
/// 数据流提供的接口
class XRocFlow : public XRocSDK {
 public:
  XRocFlow();
  virtual ~XRocFlow();

 public:
  virtual int SetConfig(
      const std::string &key,
      const std::string &value);  // 设置授权路径、模型路径等等
  int SetCallback(XRocCallback callback,
                  const std::string &name) override;  // 设置回调
  virtual int UpdateConfig(const std::string &method_name,
                           InputParamPtr param_ptr);
  InputParamPtr GetConfig(
      const std::string &method_name) const override;
  std::string GetVersion(const std::string &method_name) const override;
  virtual int Init();
  // 同步接口，单路输出
  OutputDataPtr SyncPredict(InputDataPtr input) override;
  // 同步接口，多路输出
  std::vector<OutputDataPtr> SyncPredict2(InputDataPtr input) override;
  // 异步接口
  int64_t AsyncPredict(InputDataPtr input) override;

 private:
  OutputDataPtr OnError(int64_t error_code, const std::string &error_detail);
 private:
  SchedulerPtr scheduler_;
  XRocCallback callback_;
  std::string config_file_;
  std::mutex mutex_;
  bool is_initial_;
  std::unordered_map<std::string, std::string> param_dict_;
};

}  // namespace HobotXRoc

#endif  // HOBOTXROC_XROC_H_
