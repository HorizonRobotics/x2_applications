/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     first_num_best implementation
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.10
 * @date      2019.05.24
 */

#include <chrono>
#include <fstream>
#include <ostream>
#include <iostream>
#include <memory>
#include <cstring>

#include "SnapShotMethod/error_code.h"
#include "SnapShotMethod/strategy/crop.h"
#include "hobotlog/hobotlog.hpp"
#include "json/json.h"

namespace HobotXRoc {

using std::chrono::high_resolution_clock;
using std::chrono::duration;

typedef HobotXRoc::XRocData<ImageFramePtr> XRocImageFrame;

int Crop::Init(std::shared_ptr<SnapShotParam> config) {
  snapshot_config_param_ = config;
  return XROC_SNAPSHOT_OK;
}

std::vector<BaseDataPtr> Crop::ProcessFrame(
    const std::vector<BaseDataPtr> &in, const InputParamPtr &param) {
  HOBOT_CHECK(!in.empty());
  if (param) {
    if (param->is_json_format_) {
      auto content = param->Format();
      int ret = UpdateParameter(content);
      HOBOT_CHECK(ret == XROC_SNAPSHOT_OK) << "param error";
    }
  }
  return CropAndPost(in);
}

std::vector<BaseDataPtr> Crop::CropAndPost(const std::vector<BaseDataPtr> &in) {
  std::vector<BaseDataPtr> out;
  HOBOT_CHECK(in.size() >= 2);
  auto img_frame = std::static_pointer_cast<BaseData>(in[0]);
  auto box_list = std::static_pointer_cast<BaseDataVector>(in[1]);

  HOBOT_CHECK(img_frame) << "Lost image frame";
  HOBOT_CHECK(box_list) << "Lost face boxes";
  HOBOT_CHECK("ImageFrame" == img_frame->type_);
  HOBOT_CHECK("BaseDataVector" == box_list->type_);

  size_t item_size = box_list->datas_.size();
  LOGI << "data size:" << item_size;

  std::vector<BaseDataVectorPtr> user_datas;
  for (unsigned i = 4; i < in.size(); i++) {
    auto &puser_data = in[i];
    if (puser_data->state_ == HobotXRoc::DataState::INVALID) {
      BaseDataVectorPtr empty_data_list(new BaseDataVector());
      for (auto j = 0; j < item_size; j++) {
        empty_data_list->datas_.emplace_back(puser_data);
      }
      user_datas.push_back(empty_data_list);
    } else {
      auto user_data = std::static_pointer_cast<BaseDataVector>(puser_data);
      HOBOT_CHECK(item_size == user_data->datas_.size());
      HOBOT_CHECK("BaseDataVector" == user_data->type_);
      user_datas.push_back(user_data);
    }
  }

  auto pframe = std::static_pointer_cast<XRocImageFrame>(img_frame);
  auto snapshot_info_list = std::make_shared<BaseDataVector>();
  out.push_back(snapshot_info_list);
  for (size_t i = 0; i < item_size; i++) {
    auto pbbox = std::static_pointer_cast<XRocBBox>(box_list->datas_[i]);
    if (pbbox->state_ == DataState::VALID) {
      std::vector<BaseDataPtr> onetrack_user_datas;
      for (auto &data : user_datas) {
        auto &user_data = data->datas_[i];
        onetrack_user_datas.push_back(user_data);
      }
      std::vector<SelectSnapShotInfoPtr> snaps_info;
      snaps_info.push_back(
          SnapShotInfo::GetSnapShotInfo(pframe->value,
                                        0,
                                        pbbox,
                                        snapshot_config_param_.get(),
                                        onetrack_user_datas));
      HOBOT_CHECK(!snaps_info.empty());
      auto one_target =
          SnapShotInfo::GenerateSnapshotInfo(snaps_info, READY_POST_TYPE);
      HOBOT_CHECK(!one_target->datas_.empty());
      snapshot_info_list->datas_.push_back(one_target);
    }
  }

  if (snapshot_config_param_->snapshot_state_enable) {
    auto snapshot_state_list = std::make_shared<BaseDataVector>();
    out.push_back(snapshot_state_list);
  }

  return out;
}

int Crop::UpdateParameter(const std::string &content) {
  int ret = snapshot_config_param_->UpdateParameter(content);
  return ret;
}

void Crop::Finalize() {}

}  // namespace HobotXRoc
