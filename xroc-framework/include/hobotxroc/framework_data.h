/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief provides data structure for xroc framework
 * @file framework_data.h
 * @author    chuanyi.yang
 * @email     chuanyi.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */
#ifndef HOBOTXROC_FRAMEWORK_DATA_H_
#define HOBOTXROC_FRAMEWORK_DATA_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "hobotxsdk/xroc_data.h"

namespace HobotXRoc {
/// 框架数据状态
enum BaseDataState {
  DataState_None = 0,
  DataState_Doing,
  DataState_Ready,
  DataState_Error,
};
/// FrameworkData状态
enum FrameworkDataState {
  FrameworkDataState_None = 0,
  FrameworkDataState_Doing,
  FrameworkDataState_Ready,
  FrameworkDataState_Error,
};
/// 在框架中流转的数据结构，简称框架数据
struct FrameworkData {
  /// 每个slot的数据
  std::vector<BaseDataPtr> datas_;
  /// 每个slot的数据状态
  std::vector<BaseDataState> datas_state_;
  /// 每个method的输入参数
  std::unordered_map<std::string, InputParamPtr> method_param_;
  /// 已上报的单路output
  std::vector<std::string> output_type_isout_;
  /// 每个slot数据已驱动Node的数量
  std::vector<int> driven_nodes_nums_;
  /// SDK Input透传数据
  const void *context_ = nullptr;
  /// 用于SDK等待同步结果的辅助字段
  void *sync_context_ = nullptr;
  /// 时间戳
  uint64_t timestamp_;
  /// 用来做reorder
  uint64_t sequence_id_;
  /// 数据源 id 用于多路输入时区分输入源,单一源情况赋值为 0
  uint32_t source_id_ = 0;
  uint32_t golbal_squence_id_;
  /// 当前帧状态
  FrameworkDataState state_ = FrameworkDataState_None;
};
typedef std::shared_ptr<FrameworkData> FrameworkDataPtr;

/// 一次Batch批量操作
struct FrameworkDataBatch {
  /// 多帧数据
  std::vector<FrameworkDataPtr> datas_;
  /// 时间戳
  int64_t timestamp_;
};
typedef std::shared_ptr<FrameworkDataBatch> FrameworkDataBatchPtr;

}  // namespace HobotXRoc

#endif  // HOBOTXROC_FRAMEWORK_DATA_H_
