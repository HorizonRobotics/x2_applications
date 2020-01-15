/*
 * @Description: tracking algorithm implementation
 * @Author: chang.huang@horizon.ai
 * @Date: 2016-10-8 16:00:20
 * @Author: chao01.yang@horizon.ai
 * @Date: 2019-12-14 16:00:20
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-12-17 12:07:42
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */

#ifndef TRACKING_MODULE_H_
#define TRACKING_MODULE_H_

#include "hobot/hobot.h"
#include "horizon/vision_type/vision_msg.hpp"
#include "iou_tracking/tracking_data_types.h"
#include "tracking.h"

/* tracking module declare */

struct tracking_module_data;
struct fc_batch_s;

namespace hobot {
namespace vision {

class MultiObjTrackingModuleIPC : public hobot::Module {
 public:
  MultiObjTrackingModuleIPC();
  ~MultiObjTrackingModuleIPC() override;

  int Init(RunContext *context) override;

  void Reset() override;

  FORWARD_DECLARE(MultiObjTrackingModuleIPC, 0);

  FORWARD_DECLARE(MultiObjTrackingModuleIPC, 1);

  struct IPCPostvMessage : public hobot::Message {
    void *cnn_output_base;
    void *parse_base;
    fc_batch_t fcb[FC_BATCH_CNT];
    uint32_t time_stamp[4];
    frame_status_t frame_status;
  };
  struct IPCTracksMessage : public hobot::Message {
    /* these members used for bif-module, the same size as bif_module_message_t
     */
    /* we play a trick here, we can transform this message type to
     * bif_module_message_t type through type cast */
    void *dummy;
    void *track_info_addr;
    int length;

    /* these members used for postv-module */
    void *tracking_target_base;
    int track_cnt[PREDICT_MODEL_NUM];

    frame_status_t frame_status;
    uint8_t postv_score[256];
  };
  typedef std::shared_ptr<IPCPostvMessage> spIPCPostvMessage;
  typedef std::shared_ptr<IPCTracksMessage> spIPCTracksMessage;

 private:
  int tensor_to_box(box_s *box, fc_batch_s *fcb, int total_num);
  int rects_to_box(box_s *box, hobot::vision::RectsMessage *rects_msg,
                   int total_num);
  int track_transfer(tracking_module_data *tracking_module_data,
                     IPCTracksMessage *tracking_message);
  int track_to_rects(tracking_module_data *tracking_module_data,
                     hobot::vision::RectsMessage *rects_msg);

  struct tracking_module_data *tracking_module_data = nullptr;
  bool original_bbox = false;
  uint32_t disappeared_frame_num = 50;

#ifdef TRACK_POST_FUNCTION
  ipc_context_t *ipc_ctx;
#endif
};
}  // namespace vision
}  // namespace hobot

#endif
