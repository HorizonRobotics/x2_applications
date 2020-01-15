/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides xroc framework interface
 * @file      xroc_sdk.h
 * @author    chuanyi.yang
 * @email     chuanyi.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */
#ifndef HOBOTXSDK_XROC_SDK_H_
#define HOBOTXSDK_XROC_SDK_H_

#include <string>
#include <vector>

#include "xroc_data.h"

namespace HobotXRoc {

/**
 * 典型使用
 * HobotXRoc::XRocSDK *flow = HobotXRoc::XRocSDK::CreateSDK();
 * flow->SetConfig("config_file", config);
 * flow->Init();
 * InputDataPtr inputdata(new InputData());
 * // ... 构造输入数据
 * auto out = flow->SyncPredict(inputdata);
 * // PrintOut(out);
 * // ... 处理输出结果
 * delete flow;
 */

/// 数据流提供的接口
class XRocSDK {
 public:
  /// 因为构造出来的实例是XRocSDK接口的子类
  virtual ~XRocSDK() {}
  /// 通过此方法构造SDK实例
  static XRocSDK *CreateSDK();

  /// key：config_file，用来设置workflow配置文件路径
  virtual int SetConfig(
      const std::string &key,
      const std::string &value) = 0;  // 设置授权路径、模型路径等等
  /// 初始化workflow
  virtual int Init() = 0;
  /// 针对method node更新配置参数
  virtual int UpdateConfig(const std::string &method_name,
                           InputParamPtr ptr) = 0;
  /// 获取method当前配置
  virtual InputParamPtr GetConfig(const std::string &method_name) const = 0;
  /// 获取method的版本号
  virtual std::string GetVersion(const std::string &method_name) const = 0;
  /// 同步预测接口，单路输出
  virtual OutputDataPtr SyncPredict(InputDataPtr input) = 0;
  /// 同步预测接口，多路输出
  virtual std::vector<OutputDataPtr> SyncPredict2(InputDataPtr input) = 0;
  /**
   *  异步接口的callback设置接口
   *
   * 需要在Init()后执行，否则name不为空时无法得到结果
   * @param callback [in], 回调函数
   * @param name [in], workflow 节点
   * unique_name，当使用默认参数时，callback为全局回调，
   *    只有当这一帧数据全部计算结束才会回调报告结果；如果设置了unique_name，则在异步调用中就会
   *    上报当前节点的输出，但同步调用中不会回调。
   */
  virtual int SetCallback(XRocCallback callback,
                          const std::string &name = "") = 0;  // 设置回调
  /// 异步预测接口
  virtual int64_t AsyncPredict(InputDataPtr input) = 0;  // 异步接口
};

}  // namespace HobotXRoc

#endif  // HOBOTXSDK_XROC_SDK_H_
