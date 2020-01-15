/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     first_num_best implementation
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.10
 * @date      2019.04.18
 */

#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <ostream>
#include "hobotxroc/profiler.h"
#include "json/json.h"
#include "SnapShotMethod/error_code.h"
#include "hobotlog/hobotlog.hpp"
#include "SnapShotMethod/strategy/first_num_best.h"

namespace HobotXRoc {

using std::chrono::duration;
using std::chrono::high_resolution_clock;

typedef HobotXRoc::XRocData<ImageFramePtr> XRocImageFrame;

int FirstNumBest::Init(std::shared_ptr<SnapShotParam> config) {
  snapshot_config_param_ = config;
  return XROC_SNAPSHOT_OK;
}

std::vector<BaseDataPtr> FirstNumBest::ProcessFrame(
    const std::vector<BaseDataPtr> &in, const InputParamPtr &param) {
  HOBOT_CHECK(!in.empty());
  if (param) {
    if (param->is_json_format_) {
      std::string content = param->Format();
      int ret = UpdateParameter(content);
      HOBOT_CHECK(ret == XROC_SNAPSHOT_OK) << "param error";
    }
  }
  LOGI << "SnapShot Mode";
  return SelectAndPost(in);
}

std::vector<BaseDataPtr> FirstNumBest::SelectAndPost(
    const std::vector<BaseDataPtr> &in) {
  std::vector<BaseDataPtr> out;
  {
    RUN_PROCESS_TIME_PROFILER("select")
    auto ret = Select(in);
    HOBOT_CHECK(ret == XROC_SNAPSHOT_OK) << "select error";
  }
  {
    RUN_PROCESS_TIME_PROFILER("post")
    auto ret = Post(in, out);
    HOBOT_CHECK(ret == XROC_SNAPSHOT_OK) << "post error";
  }
  return out;
}

int FirstNumBest::Select(const std::vector<BaseDataPtr> &in) {
  HOBOT_CHECK(in.size() >= 3);
  auto img_frame = std::static_pointer_cast<BaseData>(in[0]);
  auto box_list = std::static_pointer_cast<BaseDataVector>(in[1]);
  auto select_score_list = std::static_pointer_cast<BaseDataVector>(in[2]);

  HOBOT_CHECK_EQ("ImageFrame", img_frame->type_);
  HOBOT_CHECK_EQ("BaseDataVector", box_list->type_);
  HOBOT_CHECK_EQ("BaseDataVector", select_score_list->type_);

  size_t item_size = box_list->datas_.size();
  HOBOT_CHECK(item_size == select_score_list->datas_.size());
  LOGD << "input data size:" << item_size;

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

  auto frame = std::static_pointer_cast<XRocImageFrame>(img_frame);
  unsigned crop_count = 0;
  int ret = 0;
  auto *snap_order_map = GetSnapOrder(box_list);
  auto config_param = GetConfig();
  for (size_t i = 0; i < item_size; i++) {
    auto bbox =
        std::static_pointer_cast<XRocBBox>(box_list->datas_[snap_order_map[i]]);
    auto select_score = std::static_pointer_cast<XRocFloat>(
        select_score_list->datas_[snap_order_map[i]]);
    std::vector<BaseDataPtr> one_track_user_datas;
    for (auto &data : user_datas) {
      auto &user_data = data->datas_[snap_order_map[i]];
      one_track_user_datas.push_back(user_data);
    }
    if (select_states_.size() < config_param->max_tracks) {
      ret = UpdateState(frame->value, crop_count, bbox, select_score->value,
                        one_track_user_datas);
      if (ret != XROC_SNAPSHOT_OK) return ret;
    } else {
      LOGI << "select slot is full and drop the id: " << bbox->value.id;
    }
  }
  ret = UpdateCropHistory(crop_count);
  delete[] snap_order_map;
  return ret;
}

int FirstNumBest::Post(const std::vector<BaseDataPtr> &in,
                       std::vector<BaseDataPtr> &out) {
  auto snap_list = std::make_shared<BaseDataVector>();
  out.push_back(snap_list);

  HOBOT_CHECK(in.size() >= 2);
  auto img_frame = std::static_pointer_cast<BaseData>(in[0]);
  auto bbox_list = std::static_pointer_cast<BaseDataVector>(in[1]);
  HOBOT_CHECK("ImageFrame" == img_frame->type_);
  HOBOT_CHECK("BaseDataVector" == bbox_list->type_);
  int ret = XROC_SNAPSHOT_OK;

  auto frame = std::static_pointer_cast<XRocImageFrame>(img_frame);
  auto &frame_id = frame->value->frame_id;
  auto config_param = GetConfig();

  if (in.size() >= 4) {  // there is disappeared list
    auto flush_id_list = std::static_pointer_cast<BaseDataVector>(in[3]);
    HOBOT_CHECK("BaseDataVector" == flush_id_list->type_);
    //  post the flushed track
    FlushTheTrackState(snap_list, flush_id_list);
  }

  //  post the track whose snap count is reach to the begin_post_frame threshold
  PostSnapshot(snap_list, bbox_list, frame_id);

  // if satisfied re-snap condition, delete the track state
  UpdateResnapState(frame_id);

  LOGI << "Post Track Size:" << snap_list->datas_.size();

  if (config_param->snapshot_state_enable) {
    auto state_list = std::make_shared<BaseDataVector>();
    out.push_back(state_list);
    for (auto &item : snapshot_state_) {
      XRocSnapshotStatePtr StatePtr(new XRocSnapshotState());
      StatePtr->type_ = "Snap State";
      StatePtr->value = item.second;
      state_list->datas_.push_back(StatePtr);
    }
    LOGI << "Post State Size:" << state_list->datas_.size();
    snapshot_state_.clear();
  }

  return ret;
}

void FirstNumBest::FlushTheTrackState(const BaseDataVectorPtr &snap_list,
                                      const BaseDataVectorPtr &flush_id_list) {
  auto config_param = GetConfig();
  for (const auto &data : flush_id_list->datas_) {
    auto track_id = std::static_pointer_cast<XRocUint32>(data);
    auto &id = track_id->value;
    auto id_itr = select_states_.find(id);
    if (config_param->IsVanishPostEnabled()) {
      if (id_itr != select_states_.end()) {
        auto &track_state = select_states_[id];
        if ((config_param->repeat_post_flag ||
             (!config_param->repeat_post_flag && !track_state->finish_)) &&
            (track_state->snaps_.size() >= config_param->snaps_per_track)) {
          auto &snaps = select_states_[id]->snaps_;
          HOBOT_CHECK(!snaps.empty())
              << "report_flushed_track_flag: "
              << config_param->report_flushed_track_flag
              << " out_date_target_post_flag: "
              << config_param->out_date_target_post_flag
              << " snaps_per_track: " << config_param->snaps_per_track;
          auto one_target =
              SnapShotInfo::GenerateSnapshotInfo(snaps, FLUSH_POST_TYPE);
          HOBOT_CHECK(!one_target->datas_.empty());
          snap_list->datas_.push_back(one_target);
          select_states_.erase(id);
          continue;
        }
      }
      if (config_param->allow_empty_snapshot) {
        snap_list->datas_.push_back(SnapShotInfo::GenerateWithoutSnapshot(id));
      }
    }
    if (id_itr != select_states_.end()) {
      select_states_.erase(id);
    }
  }
  if (config_param->IsVanishPostEnabled() &&
      config_param->allow_empty_snapshot) {
    HOBOT_CHECK_GE(snap_list->datas_.size(), flush_id_list->datas_.size());
  }
}

void FirstNumBest::PostSnapshot(const BaseDataVectorPtr &snap_list,
                                const BaseDataVectorPtr &bbox_list,
                                const uint64_t &frame_id) {
  auto config_param = GetConfig();
  for (const auto &data : bbox_list->datas_) {
    auto bbox = std::static_pointer_cast<XRocBBox>(data);
    auto &id = bbox->value.id;
    if (!select_states_.count(id)) {
      LOGI << "id: " << id << " is not in select_states";
      continue;
    }

    auto &track_state = select_states_[id];
    if (!track_state->finish_ && ReadyToPost(track_state) &&
        track_state->snaps_.size() >= config_param->snaps_per_track) {
      if (config_param->out_date_target_post_flag ||
          (!config_param->out_date_target_post_flag &&
           data->state_ == DataState::VALID)) {
        auto &snaps = select_states_[id]->snaps_;
        HOBOT_CHECK(!snaps.empty())
            << "snaps_per_track: " << config_param->snaps_per_track;
        auto one_target =
            SnapShotInfo::GenerateSnapshotInfo(snaps, READY_POST_TYPE);
        HOBOT_CHECK(!one_target->datas_.empty());
        snap_list->datas_.push_back(one_target);
      }
      if (config_param->snapshot_state_enable) {
        if (snapshot_state_.find(id) == snapshot_state_.end()) {
          SnapshotStatePtr SnapState(new SnapshotState());
          SnapState->id = id;
          SnapState->snap_stop = 1;
          snapshot_state_[id] = SnapState;
        } else {
          auto &SnapState = snapshot_state_[id];
          SnapState->snap_stop = 1;
        }
      }
      track_state->finish_ = true;
    } else {
      LOGV << "track " << id << " finish status is " << track_state->finish_
           << " ready status is " << ReadyToPost(track_state)
           << " snaps size is " << track_state->snaps_.size()
           << " config snaps per track is " << config_param->snaps_per_track;
    }
  }
}

void FirstNumBest::UpdateResnapState(const uint64_t &frame_id) {
  auto config_param = GetConfig();
  auto iter = select_states_.begin();
  while (iter != select_states_.end()) {
    auto id = iter->first;
    auto &track_state = iter->second;
    if (NeedReSnap(frame_id, track_state)) {
      if (config_param->snapshot_state_enable) {
        if (snapshot_state_.find(id) == snapshot_state_.end()) {
          SnapshotStatePtr SnapState(new SnapshotState());
          SnapState->id = id;
          SnapState->snap_repeat = 1;
          snapshot_state_[id] = SnapState;
          LOGD << std::dec << "Snap id:" << SnapState->id
               << " Snap repeat:" << (SnapState->snap_repeat? 1 : 0);
        } else {
          auto &SnapState = snapshot_state_[id];
          SnapState->snap_repeat = 1;
          LOGD << std::dec << "Snap id:" << SnapState->id
               << " Snap repeat:" << (SnapState->snap_repeat? 1 : 0);
        }
      }
      iter = select_states_.erase(iter);
    } else {
      iter++;
    }
  }
}

unsigned *FirstNumBest::GetSnapOrder(const BaseDataVectorPtr &bbox_list) {
  auto *snap_order_map = new unsigned[bbox_list->datas_.size()];
  auto *new_resps_order_map = new unsigned[bbox_list->datas_.size()];
  auto *old_resps_order_map = new unsigned[bbox_list->datas_.size()];
  int new_resps_count = 0;
  unsigned old_resps_count = 0;
  for (unsigned i = 0; i < bbox_list->datas_.size(); i++) {
    auto bbox = std::static_pointer_cast<XRocBBox>(bbox_list->datas_[i]);
    auto id = static_cast<unsigned>(bbox->value.id);
    if (select_states_.find(id) != select_states_.end()) {
      old_resps_order_map[old_resps_count] = i;
      for (int j = old_resps_count; j > 0; j--) {
        auto bbox1 = std::static_pointer_cast<XRocBBox>(
            bbox_list->datas_[old_resps_order_map[j]]);
        auto bbox2 = std::static_pointer_cast<XRocBBox>(
            bbox_list->datas_[old_resps_order_map[j - 1]]);
        if (bbox1->value.y2 > bbox2->value.y2) {
          auto tmp = old_resps_order_map[j];
          old_resps_order_map[j] = old_resps_order_map[j - 1];
          old_resps_order_map[j - 1] = tmp;
        }
      }
      old_resps_count++;
    } else {
      new_resps_order_map[new_resps_count] = i;
      for (int j = new_resps_count; j > 0; j--) {
        auto bbox1 = std::static_pointer_cast<XRocBBox>(
            bbox_list->datas_[new_resps_order_map[j]]);
        auto bbox2 = std::static_pointer_cast<XRocBBox>(
            bbox_list->datas_[new_resps_order_map[j - 1]]);
        if (bbox1->value.y2 > bbox2->value.y2) {
          auto tmp = new_resps_order_map[j];
          new_resps_order_map[j] = new_resps_order_map[j - 1];
          new_resps_order_map[j - 1] = tmp;
        }
      }
      new_resps_count++;
    }
  }
  memcpy(snap_order_map, new_resps_order_map,
         sizeof(unsigned) * new_resps_count);
  memcpy(snap_order_map + new_resps_count, old_resps_order_map,
         sizeof(unsigned) * old_resps_count);
  delete[] new_resps_order_map;
  delete[] old_resps_order_map;
  return snap_order_map;
}

int FirstNumBest::UpdateState(const ImageFramePtr &frame, uint32_t &crop_count,
                              const XRocBBoxPtr &pbbox,
                              const float &select_score,
                              const std::vector<BaseDataPtr> &userdatas) {
  int ret = 0;
  unsigned crop_history_sum = GetPreviousCropCount();
  auto &bbox = pbbox->value;
  auto &id = bbox.id;

  if ((pbbox->state_ == DataState::VALID ||
       pbbox->state_ == DataState::FILTERED)) {
    if (select_states_.find(id) == select_states_.end()) {
      ret = AddNewTrackState(id, frame->frame_id);
      if (ret != XROC_SNAPSHOT_OK) return ret;
    }
    auto &state = select_states_[id];
    state->count_++;
    auto config_param = GetConfig();
    if (crop_count < config_param->max_crop_num_per_frame &&
        (crop_count + crop_history_sum <
         config_param->smoothing_frame_range *
             config_param->avg_crop_num_per_frame)) {
      if (pbbox->state_ == DataState::VALID) {
        if (state->snaps_.empty() ||
            state->snaps_.size() < config_param->snaps_per_track) {
          select_states_[id]->snaps_.push_back(SnapShotInfo::GetSnapShotInfo(
              frame, select_score, pbbox, config_param.get(), userdatas));
          if (config_param->snapshot_state_enable
              && !select_states_[id]->finish_) {
            SnapshotStatePtr SnapState(new SnapshotState());
            SnapState->id = id;
            SnapState->box = pbbox->value;
            SnapState->select_index = state->snaps_.size() - 1;
            SnapState->overall_quality = select_score;
            SnapState->snap_en = 1;
            SnapState->snap_stop = 0;
            SnapState->snap_repeat = 0;
            snapshot_state_[id] = SnapState;
          }
          crop_count++;
        } else {
          HOBOT_CHECK(!select_states_[id]->snaps_.empty());
          auto &snaps = select_states_[id]->snaps_;
          auto min_select_value = snaps[0]->select_value;
          size_t min_select_value_index = 0;
          for (size_t i = 0; i < snaps.size(); i++) {
            if (snaps[i]->select_value < min_select_value) {
              min_select_value = snaps[i]->select_value;
              min_select_value_index = i;
            }
          }
          if (min_select_value + config_param->update_steps < select_score) {
            snaps[min_select_value_index] = SnapShotInfo::GetSnapShotInfo(
                frame, select_score, pbbox, config_param.get(), userdatas);
            if (config_param->snapshot_state_enable
                && !select_states_[id]->finish_) {
              SnapshotStatePtr SnapState(new SnapshotState());
              SnapState->id = id;
              SnapState->box = pbbox->value;
              SnapState->select_index = min_select_value_index;
              SnapState->overall_quality = select_score;
              SnapState->snap_en = 1;
              SnapState->snap_stop = 0;
              SnapState->snap_repeat = 0;
              snapshot_state_[id] = SnapState;
            }
            crop_count++;
          }
        }
      }
    }
  }
  return ret;
}

uint32_t FirstNumBest::GetPreviousCropCount() {
  unsigned count = 0;
  for (auto crop_count : crop_history_) count += crop_count;
  return count;
}

int FirstNumBest::UpdateCropHistory(uint32_t crop_count) {
  auto config_param = GetConfig();
  if (crop_history_.size() >= config_param->smoothing_frame_range)
    crop_history_.pop_front();
  crop_history_.push_back(crop_count);
  return XROC_SNAPSHOT_OK;
}

int FirstNumBest::AddNewTrackState(const int64_t &track_id,
                                   const uint64_t &frame_id) {
  StatePtr state(new State());
  state->start_ = frame_id;
  state->count_ = 0;
  state->finish_ = false;
  select_states_[track_id] = state;
  return XROC_SNAPSHOT_OK;
}

bool FirstNumBest::NeedReSnap(const float &frame_id,
                              const StatePtr &track_state) {
  auto config_param = GetConfig();
  if (config_param->resnap_value <= config_param->begin_post_frame_thr)
    return false;
  if (frame_id >= track_state->start_)
    return (frame_id - track_state->start_ >= config_param->resnap_value - 1);
  else
    return false;
}

bool FirstNumBest::ReadyToPost(StatePtr &trackstate) {
  auto config_param = GetConfig();
  return (trackstate->count_ >= config_param->begin_post_frame_thr);
}

void FirstNumBest::Finalize() {
  auto iter = select_states_.begin();
  while (iter != select_states_.end()) {
    iter = select_states_.erase(iter);
  }
}

int FirstNumBest::UpdateParameter(const std::string &content) {
  return snapshot_config_param_->UpdateParameter(content);
}

std::shared_ptr<FirstNumBestParam> FirstNumBest::GetConfig() {
  auto select_config =
      std::static_pointer_cast<FirstNumBestParam>(snapshot_config_param_);
  return select_config;
}

void FirstNumBest::Reset() {
  crop_history_.clear();
  select_states_.clear();
  snapshot_state_.clear();
  LOGI << "Clear SnapShot states!";
}

int FirstNumBestParam::UpdateParameter(const std::string &content) {
  LOGD << "FirstNumBestParam update config: " << this;
  int ret = SnapShotParam::UpdateParameter(content);
  if (ret != XROC_SNAPSHOT_OK) return ret;
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  JSONCPP_STRING error;
  std::shared_ptr<Json::CharReader> json_reader(builder.newCharReader());
  try {
    Json::Value json_var;
    ret = json_reader->parse(content.c_str(), content.c_str() + content.size(),
                             &json_var, &error);
    SET_SNAPSHOT_METHOD_PARAM(json_var, UInt, update_steps);
    SET_SNAPSHOT_METHOD_PARAM(json_var, UInt, snaps_per_track);
    SET_SNAPSHOT_METHOD_PARAM(json_var, UInt, max_tracks);
    SET_SNAPSHOT_METHOD_PARAM(json_var, UInt, max_crop_num_per_frame);
    SET_SNAPSHOT_METHOD_PARAM(json_var, UInt, smoothing_frame_range);
    SET_SNAPSHOT_METHOD_PARAM(json_var, UInt, avg_crop_num_per_frame);
    SET_SNAPSHOT_METHOD_PARAM(json_var, UInt64, begin_post_frame_thr);
    SET_SNAPSHOT_METHOD_PARAM(json_var, UInt64, resnap_value);
    SET_SNAPSHOT_METHOD_PARAM(json_var, Bool, report_flushed_track_flag);
    SET_SNAPSHOT_METHOD_PARAM(json_var, Bool, out_date_target_post_flag);
    SET_SNAPSHOT_METHOD_PARAM(json_var, Bool, repeat_post_flag);
    SET_SNAPSHOT_METHOD_PARAM(json_var, Bool, allow_empty_snapshot);
    LOGD << "update_steps: " << update_steps;
    LOGD << "snaps_per_track: " << snaps_per_track;
    LOGD << "max_tracks: " << max_tracks;
    LOGD << "max_crop_num_per_frame: " << max_crop_num_per_frame;
    LOGD << "smoothing_frame_range: " << smoothing_frame_range;
    LOGD << "avg_crop_num_per_frame: " << avg_crop_num_per_frame;
    LOGD << "begin_post_frame_thr: " << begin_post_frame_thr;
    LOGD << "resnap_value: " << resnap_value;
    LOGD << "report_flushed_track_flag: " << report_flushed_track_flag;
    LOGD << "out_date_target_post_flag: " << out_date_target_post_flag;
    LOGD << "repeat_post_flag: " << repeat_post_flag;
    LOGD << "allow_empty_snapshot: " << allow_empty_snapshot;
    if (ret) {
      return XROC_SNAPSHOT_OK;
    } else {
      return XROC_SNAPSHOT_ERR_PARAM;
    }
  } catch (std::exception &e) {
    return XROC_SNAPSHOT_ERR_PARAM;
  }
}

}  // namespace HobotXRoc
