/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: Predictor.h
 * @Brief: declaration of the Predictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-16 14:52:31
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-16 16:22:44
 */

#ifndef INCLUDE_CNNMETHOD_PREDICTOR_PREDICTOR_H_
#define INCLUDE_CNNMETHOD_PREDICTOR_PREDICTOR_H_

#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <unordered_map>
#include <utility>
#include "CNNMethod/CNNConst.h"
#include "CNNMethod/util/CNNMethodConfig.h"
#include "CNNMethod/util/CNNMethodData.h"
#include "CNNMethod/util/ModelInfo.h"
#include "bpu_predict/bpu_io.h"
#include "bpu_predict/bpu_predict.h"
#include "horizon/vision_type/vision_type.hpp"
#include "horizon/vision_type/vision_type_common.h"
#include "hobot_vision/bpu_handle_manager.hpp"

namespace HobotXRoc {

class ModelOutputBuffer {
 public:
  ModelOutputBuffer() = delete;
  ModelOutputBuffer(const ModelInfo &model_info, int target_num) {
    for (int i = 0; i < target_num; ++i) {
      for (auto n : model_info.output_layer_size_) {
        BPU_Buffer_Handle out_handle = BPU_createEmptyBPUBuffer();
        out_bufs_.push_back(out_handle);
      }
    }
  }
  ~ModelOutputBuffer() {
    for (auto &out_buf : out_bufs_) {
      BPU_freeBPUBuffer(out_buf);
    }
    out_bufs_.clear();
  }

 public:
  std::vector<BPU_Buffer_Handle> out_bufs_;
};

class Predictor {
 public:
  Predictor() {}
  virtual ~Predictor() { Finalize(); }

  virtual int32_t Init(std::shared_ptr<CNNMethodConfig> config);
  virtual void Finalize();
  virtual void Do(CNNMethodRunData *run_data) = 0;
  virtual void UpdateParam(std::shared_ptr<CNNMethodConfig> config);
  virtual std::string GetVersion() const { return model_version_; }

 protected:
  int RunModelFromImage(uint8_t *data,
                        int data_size,
                        BPU_Buffer_Handle *output_buf,
                        int output_size);
  int RunModelFromResizer(BPUPyramidBuffer input,
                          BPUBBox *box,
                          int box_num,
                          int *resizable_cnt,
                          BPU_Buffer_Handle *output_buf,
                          int output_size);
  // void RunModelFromDDR();

  void ConvertOutputToMXNet(void *src_ptr, void *dest_ptr, int layer_idx);

  int NormalizeRoi(hobot::vision::BBox *src,
                   hobot::vision::BBox *dst,
                   float norm_ratio,
                   NormMethod norm_method,
                   uint32_t total_w,
                   uint32_t total_h,
                   FilterMethod filter_method = FilterMethod::OUT_OF_RANGE);

 protected:
  std::string model_name_;
  std::string model_version_;

  BPUHandle bpu_handle_ = nullptr;
  BPUFakeImageHandle fake_img_handle_ = nullptr;

  ModelInfo model_info_;
  std::string model_path_;
  std::vector<std::vector<int8_t>> feature_bufs_;
  int32_t max_handle_num_ = -1;  // Less than 0 means unlimited
 private:
  int FilterRoi(hobot::vision::BBox *src,
                hobot::vision::BBox *dst,
                int src_w,
                int src_h,
                FilterMethod filter_method);
};
}  // namespace HobotXRoc
#endif  // INCLUDE_CNNMETHOD_PREDICTOR_PREDICTOR_H_
