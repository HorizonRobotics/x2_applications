/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     first_num_best header
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.16
 * @date      2019.05.22
 */

#ifndef SNAPSHOTMETHOD_STRATEGY_FIRST_NUM_BEST_H_
#define SNAPSHOTMETHOD_STRATEGY_FIRST_NUM_BEST_H_

#include <string>
#include <vector>
#include <list>
#include <map>
#include <memory>

#include "horizon/vision_type/vision_type.hpp"
#include "SnapShotMethod/SnapShotMethod.h"
#include "SnapShotMethod/image_utils/image_utils.hpp"
#include "SnapShotMethod/snapshot_data_type/snapshot_data_type.hpp"

namespace HobotXRoc {

struct FirstNumBestParam : SnapShotParam {
 public:
  explicit FirstNumBestParam(const std::string &content = "")
      : SnapShotParam(content) {
    UpdateParameter(content);
  }
  bool IsVanishPostEnabled() const {
    return report_flushed_track_flag && out_date_target_post_flag;
  }
  int UpdateParameter(const std::string &content) override;
  float update_steps = 0;
  unsigned snaps_per_track = 0;
  unsigned max_tracks = 0;
  unsigned max_crop_num_per_frame = 0;
  unsigned smoothing_frame_range = 0;
  unsigned avg_crop_num_per_frame = 0;
  uint64_t begin_post_frame_thr = 0;
  uint64_t resnap_value = 0;
  bool report_flushed_track_flag = false;
  bool out_date_target_post_flag = false;
  bool repeat_post_flag = false;
  bool allow_empty_snapshot = false;
};

class FirstNumBest : public SnapShot {
 public:
  int Init(std::shared_ptr<SnapShotParam> config) override;

  std::vector<BaseDataPtr> ProcessFrame(const std::vector<BaseDataPtr> &in,
                                        const InputParamPtr &param) override;

  void Finalize() override;

  int UpdateParameter(const std::string &content) override;

 private:
  struct State {
    uint64_t start_ = 0;
    uint64_t count_ = 0;
    bool finish_ = false;
    std::vector<SelectSnapShotInfoPtr> snaps_;
  };

  typedef std::shared_ptr<State> StatePtr;

  std::vector<BaseDataPtr> SelectAndPost(const std::vector<BaseDataPtr> &in);

  std::shared_ptr<FirstNumBestParam> GetConfig();

  int Select(const std::vector<BaseDataPtr> &in);

  int Post(const std::vector<BaseDataPtr> &in, std::vector<BaseDataPtr> &out);

  int AddNewTrackState(const int64_t &track_id, const uint64_t &frame_id_);

  uint32_t GetPreviousCropCount();

  int UpdateCropHistory(uint32_t crop_count);

  unsigned* GetSnapOrder(const BaseDataVectorPtr &bbox_list);

  bool NeedReSnap(const float &frame_id, const StatePtr &track_state);

  bool ReadyToPost(StatePtr &trackinfo);

  int UpdateState(const ImageFramePtr &frame,
                  uint32_t &crop_count,
                  const XRocBBoxPtr &pbbox,
                  const float &select_score,
                  const std::vector<BaseDataPtr> &userdatas);

  std::map <int32_t, StatePtr> select_states_;

  std::list<unsigned> crop_history_;

  void UpdateResnapState(const uint64_t &frame_id);

  void PostSnapshot(const BaseDataVectorPtr &snap_list,
                    const BaseDataVectorPtr &bbox_list,
                    const uint64_t &frame_id);

  void FlushTheTrackState(const BaseDataVectorPtr &snap_list,
                          const BaseDataVectorPtr &flush_id_list);

  void Reset() override;

  std::map<int32_t, SnapshotStatePtr> snapshot_state_;
};
} // namespace HobotXRoc

#endif // SNAPSHOTMETHOD_STRATEGY_FIRST_NUM_BEST_H_
