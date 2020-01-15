/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     snapshot_data_type implementation
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.10
 * @date      2019.04.22
 */

#include <fstream>
#include <vector>
#include "json/json.h"
#include "hobotlog/hobotlog.hpp"
#include "SnapShotMethod/error_code.h"
#include "SnapShotMethod/snapshot_data_type/snapshot_data_type.hpp"

namespace HobotXRoc {

Points SelectSnapShotInfo::PointsToSnap(const Points &in) {
  Points out = Points();
  for (auto &point : in.values) {
    Point p;
    p.x = (floor(point.x) - snap_base_point.x) * wide_scale;
    p.y = (floor(point.y) - snap_base_point.y) * height_scale;
    p.score = point.score;
    out.values.push_back(p);
  }
  return out;
}

int SnapShotParam::UpdateParameter(const std::string &content) {
  LOGD << "SnapShotParam update config: " << this;
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  JSONCPP_STRING error;
  std::shared_ptr<Json::CharReader> json_reader(builder.newCharReader());
  try {
    Json::Value json_var;
    bool ret = json_reader->parse(content.c_str(), content.c_str()
        + content.size(), &json_var, &error);
    SET_SNAPSHOT_METHOD_PARAM(json_var, Double, scale_rate);
    SET_SNAPSHOT_METHOD_PARAM(json_var, Bool, need_resize);
    SET_SNAPSHOT_METHOD_PARAM(json_var, UInt, output_width);
    SET_SNAPSHOT_METHOD_PARAM(json_var, UInt, output_height);
    SET_SNAPSHOT_METHOD_PARAM(json_var, Bool, save_original_image_frame);
    SET_SNAPSHOT_METHOD_PARAM(json_var, Bool, snapshot_state_enable);

    LOGD << "scale_rate: " << scale_rate;
    LOGD << "need_resize: " << need_resize;
    LOGD << "output_width: " << output_width;
    LOGD << "output_height: " << output_height;
    LOGD << "save_original_image_frame: "
         << save_original_image_frame;

    if (ret) {
      return XROC_SNAPSHOT_OK;
    } else {
      return XROC_SNAPSHOT_ERR_PARAM;
    }
  } catch (std::exception &e) {
    return XROC_SNAPSHOT_ERR_PARAM;
  }
}

std::string SnapShotParam::Format() {
  return config_jv.toStyledString();
}

SelectSnapShotInfoPtr SnapShotInfo::GetSnapShotInfo(const ImageFramePtr &frame,
                                      const float &select_score,
                                      const XRocBBoxPtr &pbbox,
                                      SnapShotParam* param,
                                      std::vector<BaseDataPtr> userdatas) {
  SelectSnapShotInfoPtr snapshot_info(new SelectSnapShotInfo());
  snapshot_info->track_id = pbbox->value.id;
  snapshot_info->select_value = select_score;
  if (param->save_original_image_frame) {
    snapshot_info->origin_image_frame = frame;
  } else {
    ImageFramePtr image_frame(new hobot::vision::CVImageFrame());
    image_frame->frame_id = frame->frame_id;
    image_frame->time_stamp = frame->time_stamp;
    image_frame->pixel_format = frame->pixel_format;
    image_frame->type = frame->type;
    image_frame->channel_id = frame->channel_id;
    snapshot_info->origin_image_frame = image_frame;
  }
  auto &bbox = pbbox->value;
  auto ad_bbox = ImageUtils::AdjustSnapRect(
      frame->Width(), frame->Height(), bbox, param->scale_rate);
  snapshot_info->snap_base_point.x = ad_bbox.x1;
  snapshot_info->snap_base_point.y = ad_bbox.y1;
  auto snap_width = ad_bbox.Width() + 1;
  auto snap_height = ad_bbox.Height() + 1;
  if (param->need_resize) {
    snapshot_info->wide_scale =
        static_cast<float>(param->output_width) / snap_width;
    snapshot_info->height_scale =
        static_cast<float>(param->output_height) / snap_height;
  } else {
    snapshot_info->wide_scale = 1;
    snapshot_info->height_scale = 1;
  }

  snapshot_info->snap =
      ImageUtils::DoFaceCrop(frame, ad_bbox,
          param->output_width, param->output_height, param->need_resize);
  snapshot_info->userdata = std::move(userdatas);
  return snapshot_info;
}


typedef HobotXRoc::XRocData<SnapshotInfoXRocBaseDataPtr> XRocSnapshotInfo;
typedef std::shared_ptr<XRocSnapshotInfo> XRocSnapshotInfoPtr;

SelectSnapShotInfoPtr CopySelectSnapShotInfo(
    const SelectSnapShotInfoPtr &input) {
  SelectSnapShotInfoPtr cp_info(new SelectSnapShotInfo());
  cp_info->type = input->type;
  cp_info->height_scale = input->height_scale;
  cp_info->wide_scale = input->wide_scale;
  cp_info->snap_base_point = input->snap_base_point;
  cp_info->snap = input->snap;
  cp_info->origin_image_frame = input->origin_image_frame;
  cp_info->select_value = input->select_value;
  cp_info->track_id = input->track_id;
  cp_info->userdata = input->userdata;
  return cp_info;
}

BaseDataVectorPtr SnapShotInfo::GenerateSnapshotInfo(
    const std::vector<SelectSnapShotInfoPtr> &snap_infos,
    const int32_t &type) {
  BaseDataVectorPtr snap_list(new BaseDataVector());
  for (auto &snap_info : snap_infos) {
    XRocSnapshotInfoPtr xroc_snapshot_info(new XRocSnapshotInfo());
    xroc_snapshot_info->value = CopySelectSnapShotInfo(snap_info);
    xroc_snapshot_info->value->type = type;
    xroc_snapshot_info->type_ = "SnapshotInfo";
    snap_list->datas_.emplace_back(xroc_snapshot_info);
  }
  return snap_list;
}

BaseDataPtr SnapShotInfo::GenerateWithoutSnapshot(
    const int32_t id) {
  XRocSnapshotInfoPtr xroc_snapshot_info(new XRocSnapshotInfo());
  auto info = static_cast<SelectSnapShotInfo *>(
      std::calloc(1, sizeof(SelectSnapShotInfo)));
  SelectSnapShotInfoPtr cp_info(info);
  cp_info->type = FLUSH_POST_TYPE;
  cp_info->track_id = id;
  xroc_snapshot_info->value = cp_info;
  xroc_snapshot_info->type_ = "SnapshotInfo";
  return xroc_snapshot_info;
}

} // namespace HobotXRoc
