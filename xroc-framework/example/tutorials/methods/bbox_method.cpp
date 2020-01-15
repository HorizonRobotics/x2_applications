/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2020-01-01 21:52:19
 * @Version: v0.0.1
 * @Brief: BBoxFilter Method Impl.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2020-01-01 21:52:19
 */

#include <vector>
#include <string>
#include <string>
#include <memory>

#include "bbox_method.h"

namespace HobotXRoc {
std::vector<std::vector<BaseDataPtr>> BBoxFilter::DoProcess(
    const std::vector<std::vector<BaseDataPtr>> &input,
    const std::vector<InputParamPtr> &param) {

  std::cout << "BBoxFilter::DoProcess" << std::endl;
  std::vector<std::vector<BaseDataPtr>> output;
  output.resize(input.size());
  for (size_t i = 0; i < input.size(); ++i) {
    // 当前不支持batch模式，batch恒等于1.
    assert(i <= 1);
    auto &in_batch_i = input[i];
    auto &out_batch_i = output[i];
    out_batch_i.resize(in_batch_i.size());
    std::cout << "input size: " << in_batch_i.size() << std::endl;
    // 输入格式是BBox的数组
    for (size_t j = 0; j < in_batch_i.size(); ++j) {
      auto in_rects = std::static_pointer_cast<BaseDataVector>(in_batch_i[j]);
      assert("BaseDataVector" == in_rects->type_);
      auto out_rects = std::make_shared<BaseDataVector>();
      out_batch_i[j] = std::static_pointer_cast<BaseData>(out_rects);
      for (auto &in_rect : in_rects->datas_) {
        /// 如果已经被之前的模块过滤掉，直接传递到输出.
        if (in_rect->state_ == DataState::FILTERED) {
          out_rects->datas_.push_back(in_rect);
          continue;
        }
        assert("BBox" == in_rect->type_);
        auto bbox = std::static_pointer_cast<HobotXRoc::BBox>(in_rect);
        if (bbox->value.Width() * bbox->value.Height() > area_threshold_) {
          out_rects->datas_.push_back(in_rect);
        } else {
          std::cout << "filter out: "
                    << bbox->value.x1 << ","
                    << bbox->value.y1 << ","
                    << bbox->value.x2 << ","
                    << bbox->value.y2 << std::endl;
          // 设置过滤状态，输出通过该状态过滤
          in_rect->state_ = DataState::FILTERED;
          out_rects->datas_.push_back(in_rect);
        }
      }
    }
  }

  return output;
}
}  // namespace HobotXRoc