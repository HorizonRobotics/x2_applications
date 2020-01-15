/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief provides base data struct for xsoul framework
 * @author    chuanyi.yang
 * @email     chuanyi.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */

#include "hobotxsdk/xroc_data.h"

#include "hobotxsdk/xroc_capi_type_helper.h"

namespace HobotXRoc {
BaseDataVector::BaseDataVector() {
  type_ = "BaseDataVector";
  XROC_BASE_CPP_CONTEXT_INIT(BaseDataVector, c_data_);
}

BaseData::BaseData() {
  type_ = "BaseData";
  c_data_.reset(new HobotXRoc::CContext());
  XROC_BASE_CPP_CONTEXT_INIT(BaseData, c_data_);
}

BaseData::~BaseData() {
}
}  // namespace HobotXRoc
