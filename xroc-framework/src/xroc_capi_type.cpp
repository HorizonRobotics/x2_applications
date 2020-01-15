/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides xsoul c framework interface
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.14
 */


#include <string>

#include "hobotxsdk/xroc_capi_type.h"
#include "hobotxsdk/xroc_capi_type_helper.h"
#include "hobotxsdk/xroc_data.h"
#include "hobotxsdk/xroc_capi.h"


void HobotXRocCapiDataFree(HobotXRocCapiData** data) {
  if (data && *data) {
    auto context = reinterpret_cast<HobotXRoc::CppContext*>((*data)->context_);
    context->cfree_method_(data);
  }
}


HobotXRocCapiDataList* HobotXRocCapiDataListAlloc(size_t length) {
  auto datalist = reinterpret_cast<HobotXRocCapiDataList*>(
      std::calloc(1, sizeof(HobotXRocCapiDataList) + sizeof(HobotXRocCapiData*) * length));
  // std::calloc函数会保证申请的内存全为0
  datalist->datas_size_ = length;
  return datalist;
}


void HobotXRocCapiDataListFree(HobotXRocCapiDataList** datalist) {
  if (datalist && *datalist) {
    for (size_t i = 0; i < (*datalist)->datas_size_; ++i) {
      HobotXRocCapiDataFree(&(*datalist)->datas_[i]);
    }
    free(*datalist);
    *datalist = nullptr;
  }
}


HobotXRocCapiBaseDataVector* HobotXRocCapiBaseDataVectorAlloc(size_t length) {
  XROC_CAPI_BASE_ALLOC(BaseDataVector, data_vector);
  data_vector->datas_ = HobotXRocCapiDataListAlloc(length);
  return data_vector;
}


void HobotXRocCapiBaseDataVectorFree(HobotXRocCapiBaseDataVector** data_vector) {
  if (data_vector && *data_vector) {
    XROC_CAPI_BASE_FREE(BaseDataVector, data_vector);
    HobotXRocCapiDataListFree(&((*data_vector)->datas_));
    free(*data_vector);
    *data_vector = nullptr;
  }
}

namespace HobotXRoc {

XROC_DEFINE_2C_TRANSFER_FUNC(BaseDataVector, cpp_data) {
  XROC_BASE_2C_TRANSFER_PROCESS(BaseDataVector, cpp_data, c_data, cpp_data->datas_.size());

  auto& in = cpp_data;
  auto& out = c_data;
  for (size_t i = 0; i < in->datas_.size(); ++i) {
    auto& in_data_i = in->datas_[i];
    out->datas_->datas_[i] = (in_data_i->c_data_)->cpp2c_method_(in_data_i);
  }

  return out;
}

XROC_DEFINE_2CPP_TRANSFER_FUNC(BaseDataVector, c) {
  XROC_BASE_2CPP_TRANSFER_PROCESS(BaseDataVector, c, cpp);
  auto& in = c;
  auto& out = cpp;
  out->datas_.resize(in->datas_->datas_size_);
  for (size_t i = 0; i < in->datas_->datas_size_; ++i) {
    auto& in_data_i = in->datas_->datas_[i];
    out->datas_[i] = reinterpret_cast<CppContext*>(in_data_i->context_)->c2cpp_method_(in_data_i);
  }
  return out;
}
}  // namespace HobotXRoc
