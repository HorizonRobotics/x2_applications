/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      main.cc
 * @brief     main function
 * @author    Qingpeng Liu (qingpeng.liu@horizon.ai)
 * @version   0.0.0.1
 * @date      2020-01-08
 */

#ifndef XROC_TUTORIALS_STAGE1_CALLBACK_H_
#define XROC_TUTORIALS_STAGE1_CALLBACK_H_

#include "method/b_box.h"

namespace Stage1Async {

class Callback {
public:
  void OnCallback(HobotXRoc::OutputDataPtr output) { ParseOutput(output); }

private:
  void ParseOutput(HobotXRoc::OutputDataPtr output) {
    using HobotXRoc::BaseDataVector;
    std::cout << "======================" << std::endl;
    std::cout << "seq: " << output->sequence_id_ << std::endl;
    std::cout << "output_type: " << output->output_type_ << std::endl;
    std::cout << "method_name: " << output->method_name_ << std::endl;
    std::cout << "error_code: " << output->error_code_ << std::endl;
    std::cout << "error_detail_: " << output->error_detail_ << std::endl;
    std::cout << "datas_ size: " << output->datas_.size() << std::endl;

    for (auto data : output->datas_) {
      if (data->error_code_ < 0) {
        std::cout << "data error: " << data->error_code_ << std::endl;
        continue;
      }
      std::cout << "data type_name : " << data->type_ << " " << data->name_
                << std::endl;
      BaseDataVector *pdata = reinterpret_cast<BaseDataVector *>(data.get());
      std::cout << "pdata size: " << pdata->datas_.size() << std::endl;
      std::cout << "Output BBox " << pdata->name_ << ":" << std::endl;
      for (size_t i = 0; i < pdata->datas_.size(); ++i) {
        auto xroc_box =
            std::static_pointer_cast<HobotXRoc::XRocData<hobot::vision::BBox>>(
                pdata->datas_[i]);
        if (xroc_box->state_ == HobotXRoc::DataState::VALID) {
          std::cout << "[" << xroc_box->value.x1 << "," << xroc_box->value.y1
                    << "," << xroc_box->value.x2 << "," << xroc_box->value.y2
                    << "]" << std::endl;
        } else {
          printf("pdata->datas_[%d]: state_:%d\n", i, xroc_box->state_);
        }
      }
    }
  }
};
} // namespace Stage1Async

#endif // XROC_TUTORIALS_STAGE1_CALLBACK_H_
