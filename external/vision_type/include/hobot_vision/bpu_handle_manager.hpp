/*!
 * Copyright (c) 2019 by Contributors
 * \file bpu_handle_manager.hpp
 * \brief
 * \author zezhou.gan
 */

#ifndef HOBOT_VISION_BPU_HANDLE_MANAGER_HPP_
#define HOBOT_VISION_BPU_HANDLE_MANAGER_HPP_

#include <string>
#include <unordered_map>
#include <mutex>
#include "hobotlog/hobotlog.hpp"
#include "bpu_predict/bpu_predict.h"

class BpuHandleManager {
public:
    BpuHandleManager() = default;

    BpuHandleManager(const BpuHandleManager &) = delete;

    BpuHandleManager &operator=(const BpuHandleManager &) = delete;

    BpuHandleManager(BpuHandleManager &&) = delete;

    BpuHandleManager &operator=(BpuHandleManager &&) = delete;

    BPUHandle getBpuHandle(const std::string &model_path) {
        if (bpu_handle_map_.find(model_path) == bpu_handle_map_.end()) {
            return nullptr;
        } else {
            bpu_handle_map_[model_path].second++;
            LOGD << model_path << " already loaded, refcnt:"
                 << bpu_handle_map_[model_path].second;
            return bpu_handle_map_[model_path].first;
        }
    }

    BPUHandle getBpuHandle_safe(const std::string &model_path) {
        std::lock_guard <std::mutex> lk(map_mutex_);
        return getBpuHandle(model_path);
    }

    void addBpuHandle(const std::string &model_path, const BPUHandle bpuhandle) {
        HOBOT_CHECK(bpu_handle_map_.find(model_path) == bpu_handle_map_.end())
                << "ERROR, invalid operation to add model more than once:" << model_path;
        bpu_handle_map_[model_path] = std::make_pair(bpuhandle, 1);
        LOGD << model_path << " first loaded;";
    }

    void addBpuHandle_safe(const std::string &model_path, const BPUHandle bpuhandle) {
        std::lock_guard <std::mutex> lk(map_mutex_);
        addBpuHandle(model_path, bpuhandle);
    }

    int releaseBpuHandle(const std::string &model_path) {
        std::lock_guard <std::mutex> lk(map_mutex_);
        HOBOT_CHECK(bpu_handle_map_.find(model_path) != bpu_handle_map_.end())
                << "ERROR, try to release not exist model-path:" << model_path;
        int ref = --bpu_handle_map_[model_path].second;
        if (ref == 0) {
            bpu_handle_map_.erase(model_path);
        }
        LOGD << "releaseBpuHandle: " << model_path << "refcnt left:" << ref;
        HOBOT_CHECK(ref >= 0) << "error negtive model refcnt found:" << model_path
                              << "refcnt:" << ref;
        return ref;
    }

private:
    std::unordered_map <std::string, std::pair<BPUHandle, int>> bpu_handle_map_;
    std::mutex map_mutex_;
};

#endif  // HOBOT_VISION_BPU_HANDLE_MANAGER_HPP_

