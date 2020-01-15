/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      b_box_filter.cc
 * @brief     BBoxFilter class implementation
 * @author    Qingpeng Liu (qingpeng.liu@horizon.ai)
 * @version   0.0.0.1
 * @date      2020-01-03
 */

#include "b_box_filter.h"
#include <cassert>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <thread>
#include "hobotxroc/profiler.h"
#include "hobotxsdk/xroc_data.h"
#include "json/json.h"
#include "b_box.h"

namespace HobotXRoc {

int BBoxFilter::Init(const std::string &config_file_path) {
  std::cout << "BBoxFilter::Init " << config_file_path << std::endl;
  std::ifstream infile(config_file_path);
  Json::Value cfg_jv;
  infile >> cfg_jv;
  infile.close();
  area_threshold_ = cfg_jv["threshold"].asFloat();
  std::cout << "BBoxFilter::Init area_thres:" << area_threshold_ << std::endl;
  return 0;
}

int BBoxFilter::UpdateParameter(InputParamPtr ptr) {
  auto real_ptr = dynamic_cast<HobotXRoc::FilterParam *>(ptr.get());
  if (real_ptr->HasThreshold()) {
    area_threshold_ = real_ptr->GetThreshold();
  }
  return 0;
}

InputParamPtr BBoxFilter::GetParameter() const {
  // method name is useless here
  auto param = std::make_shared<FilterParam>("");
  param->SetThreshold(area_threshold_);
  return param;
}

std::vector<std::vector<BaseDataPtr>> BBoxFilter::DoProcess(
    const std::vector<std::vector<BaseDataPtr>> &input,
    const std::vector<InputParamPtr> &param) {
  std::cout << "BBoxFilter::DoProcess" << std::endl;
  std::vector<std::vector<BaseDataPtr>> output;
  output.resize(input.size());
  for (size_t i = 0; i < input.size(); ++i) {
    // 当前不支持batch模式，batch恒等于1
    assert(i <= 1);
    auto &in_batch_i = input[i];
    auto &out_batch_i = output[i];
    out_batch_i.resize(in_batch_i.size());
    std::cout << "input size: " << in_batch_i.size() << std::endl;
    // 只支持n个输入，输入格式是BBox的数组
    for (size_t j = 0; j < in_batch_i.size(); ++j) {
      auto in_rects = std::static_pointer_cast<BaseDataVector>(in_batch_i[j]);
      assert("BaseDataVector" == in_rects->type_);
      auto out_rects = std::make_shared<BaseDataVector>();
      out_batch_i[j] = std::static_pointer_cast<BaseData>(out_rects);
      for (auto &in_rect : in_rects->datas_) {
        auto bbox = std::static_pointer_cast<HobotXRoc::BBox>(in_rect);
        // 因为BBoxFilter_A和BBoxFilter_B使用智能指针指向同一份输入数据，为避免两个Filter在一个处理完成后修改State，
        // 影响另一个Filter处理输入数据，这里会将原来的输入数据copy一份
        auto out_rect = BaseDataPtr(new HobotXRoc::BBox(bbox->value));
        out_rect->type_ = bbox->type_;
        // 如果已经被之前的模块过滤掉，直接传递到输出。
        if (in_rect->state_ == DataState::FILTERED) {
          out_rects->datas_.push_back(out_rect);
          continue;
        }
        assert("BBox" == out_rect->type_);
        if (bbox->value.Width() * bbox->value.Height() > area_threshold_) {
          out_rects->datas_.push_back(out_rect);
        } else {
          std::cout << "B filter: " << bbox->value.x1 << "," << bbox->value.y1
                    << "," << bbox->value.x2 << "," << bbox->value.y2
                    << std::endl;
          // 设置过滤状态，输出通过该状态过滤
          out_rect->state_ = DataState::FILTERED;
          out_rects->datas_.push_back(out_rect);
        }
      }
    }
  }
  return output;
}

void BBoxFilter::Finalize() {
  std::cout << "BBoxFilter::Finalize" << std::endl;
}

std::string BBoxFilter::GetVersion() const { return "test_only"; }

void BBoxFilter::OnProfilerChanged(bool on) {}

}  // namespace HobotXRoc
