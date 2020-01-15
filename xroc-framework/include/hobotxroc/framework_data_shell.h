/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     Shell of FrameworkData in xsoul framework
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */
#ifndef HOBOTXROC_FRAMEWORK_DATA_SHELL_H_
#define HOBOTXROC_FRAMEWORK_DATA_SHELL_H_

#include <chrono>
#include <memory>
#include <vector>

#include "hobotxroc/framework_data.h"
#include "hobotxsdk/xroc_data.h"

namespace HobotXRoc {

struct ExtraInfoWithinNode {
  virtual ~ExtraInfoWithinNode() {}
};

typedef std::shared_ptr<ExtraInfoWithinNode> ExtraInfoWithinNodePtr;

struct FrameworkDataShell {
  enum class ShellType { TIMER, METHOD };

  FrameworkDataShell(const FrameworkDataBatchPtr &batchData, ShellType type,
                     ExtraInfoWithinNodePtr user_data)
      : datas_(batchData), type_(type), shared_user_data_(user_data) {}

  void SetResult(const std::vector<std::vector<BaseDataPtr>> &result) {
    result_ = result;
  }

  std::vector<std::vector<BaseDataPtr>> &GetResult() { return result_; }

  FrameworkDataBatchPtr datas_;
  ShellType type_;
  std::shared_ptr<ExtraInfoWithinNode> shared_user_data_;

  std::vector<std::vector<BaseDataPtr>> result_;
};

typedef std::shared_ptr<FrameworkDataShell> FrameworkDataShellPtr;

}  // namespace HobotXRoc
#endif  // HOBOTXROC_FRAMEWORK_DATA_SHELL_H_
