/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     Method interface of xroc framework
 * @file method.h
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */
#ifndef HOBOTXROC_METHOD_H_
#define HOBOTXROC_METHOD_H_

#include <memory>
#include <string>
#include <vector>
#include "hobotxsdk/xroc_data.h"

namespace HobotXRoc {
/// Method基本信息
struct MethodInfo {
  /// 是否线程安全
  bool is_thread_safe_ = false;
  /// 是否需要做reorder，也就是让每一帧结果的返回顺序同请求顺序。
  bool is_need_reorder = false;
  /// 是否对输入源有前后文依赖 source context dependent
  bool is_src_ctx_dept = false;
};

class Method {
 public:
  virtual ~Method();
  /// 初始化
  virtual int Init(const std::string &config_file_path) = 0;
  /// 动态改变Method运行参数配置
  virtual int UpdateParameter(InputParamPtr ptr) = 0;
  // 数据处理函数，第一个参数是输入数据（双重vector，外层vector表示batch是多帧的输入
  // 内层的vector表示单帧的数据列表），
  // 内层vector对应workflow的"inputs"输入列表
  virtual std::vector<std::vector<BaseDataPtr>> DoProcess(
      const std::vector<std::vector<BaseDataPtr>> &input,
      const std::vector<InputParamPtr> &param) = 0;
  /// 获取Method运行参数配置
  virtual InputParamPtr GetParameter() const = 0;
  /// 获取Method版本号，比如 metric_v0.4.0 或者 MD112 等
  virtual std::string GetVersion() const = 0;
  /// 析构
  virtual void Finalize() = 0;
  /// 获取Method基本信息
  virtual MethodInfo GetMethodInfo();
  /// 用于告知Method整个SDK的Profiler状态更改
  virtual void OnProfilerChanged(bool on) = 0;
};

typedef std::shared_ptr<Method> MethodPtr;

}  // namespace HobotXRoc

#endif  // HOBOTXROC_METHOD_H_
