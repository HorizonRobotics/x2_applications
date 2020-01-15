/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-12-23 04:03:28
 * @Version: v0.0.1
 * @Brief: bpu model manager for ensuring only loading once.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-12-23 04:21:25
 */
#ifndef HOBOT_VISION_BPUMODEL_MANAGER_HPP_
#define HOBOT_VISION_BPUMODEL_MANAGER_HPP_

#include <map>
#include <mutex>
#include <string>
#include "bpu_predict/bpu_predict.h"
#include "hobotlog/hobotlog.hpp"

namespace hobot {
namespace vision {
class BPUModelManager {
 public:
  static BPUModelManager &Get() {
    static BPUModelManager inst;
    return inst;
  }

  BPUHandle GetBpuHandle(const std::string &model_path,
                         const std::string &bpu_config_path) {
    std::lock_guard<std::mutex> lck(mutex_);
    if (model_map_.count(model_path)) {
      LOGI << "Get bpu handle for " << model_path;
      model_map_[model_path].second++;
      return model_map_[model_path].first;
    }

    BPUHandle bpu_handle;
    int ret = BPU_loadModel(model_path.c_str(), &bpu_handle,
                            bpu_config_path.c_str());
    HOBOT_CHECK(ret == 0) << "Load bpu model failed: "
                          << BPU_getLastError(bpu_handle);
    model_map_[model_path] = std::make_pair(bpu_handle, 1);
    LOGI << "load bpu model " << model_path;
    return bpu_handle;
  }

  void ReleaseBpuHandle(const std::string &model_path) {
    std::lock_guard<std::mutex> lck(mutex_);
    if (model_map_.count(model_path)) {
      model_map_[model_path].second--;
      if (model_map_[model_path].second == 0) {
        auto bpu_handle = model_map_[model_path].first;
        BPU_release(bpu_handle);
        model_map_.erase(model_path);
      }
    }
  }
 private:
  std::mutex mutex_;
  std::map<std::string, std::pair<BPUHandle, int>> model_map_;
};
}  // namespace vision
}  // namespace hobot
#endif  // HOBOT_VISION_BPUMODEL_MANAGER_HPP_