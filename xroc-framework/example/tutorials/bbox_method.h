/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2020-01-01 21:19:52
 * @Version: v0.0.1
 * @Brief: BBoxFilter Method declarition.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2020-01-01 21:21:31
 */

#ifndef XROC_FRAMEWORK_EXAMPLE_TUTORIALS_METHODS_BBOX_METHOD_H_
#define XROC_FRAMEWORK_EXAMPLE_TUTORIALS_METHODS_BBOX_METHOD_H_

#include <atomic>
#include <string>
#include <vector>

#include "hobotxroc/method.h"

namespace HobotXRoc {

class BBoxFilter : public Method {
 public:
  /// load json配置参数，完成method初始化
  int Init(const std::string &config_file_path) override;
  // 数据处理函数，第一个参数是输入数据（双重vector，外层vector表示batch是多帧的输入
  // 内层的vector表示单帧的数据列表），
  // Note：由于目前XRoc框架接口并没有支持Batch模式，外层的vector恒等于1
  std::vector<std::vector<BaseDataPtr>> DoProcess(
      const std::vector<std::vector<BaseDataPtr>> &input,
      const std::vector<HobotXRoc::InputParamPtr> &param) override;
  /// 析构
  void Finalize() override {}
  /// 动态改变Method运行参数配置
  int UpdateParameter(InputParamPtr ptr) override;
  /// 获取Method运行参数配置
  InputParamPtr GetParameter() const override;
  /// 获取Method版本号，比如 metric_v0.4.0 或者 MD112 等
  std::string GetVersion() const override { return "BBoxFilter_test_v0.0.1"; }
  // 当workflow的profile状态发生变化时，调用该函数.
  void OnProfilerChanged(bool on) override {}

 private:
  /// 过滤bbox的面积阈值
  std::atomic<float> area_threshold_;
};
}  // namespace HobotXRoc

#endif  // XROC_FRAMEWORK_EXAMPLE_TUTORIALS_METHODS_BBOX_METHOD_H_