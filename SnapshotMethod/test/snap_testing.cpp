/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     SelectMethod Example
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2019.01.07
 */

#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <cassert>
#include <map>
#include <set>
#include <unistd.h>
#include <sys/stat.h>

#include "hobotxroc/data_types/bbox.h"
#include "hobotxroc/data_types/pose_3d.h"
#include "hobotxroc/data_types/landmark.h"
#include "hobotxroc/data_types/quality.h"
#include "hobotxroc/data_types/number.h"
#include "hobotxroc/data_types/image_frame.h"
#include "hobotxroc/data_types/snapshot_info.h"
#include "hobotxsdk/xroc_sdk.h"
#include "json/json.h"

struct RectInfo : HobotXRoc::BaseData {
  uint64_t track_id_ = 0;
  float size_ = 0;
  float pitch_ = 0;
  float yaw_ = 0;
  float roll_ = 0;
  float pose_ = 0;
  float quality_ = 0;
  int lmk0_ = 0;
  int lmk1_ = 0;
  int lmk2_ = 0;
  int lmk3_ = 0;
  int lmk4_ = 0;
};

struct TrackState {
  uint64_t start = 0;
  uint64_t last = 0;
};

class Callback {
 public:
  Callback() = default;

  ~Callback() = default;

  std::ofstream json_ofs_;

  std::map<int, std::string> id_map;

  int WriteLog(HobotXRoc::SnapshotInfo *snapshot_info) {
    FILE *plog = nullptr;
    plog = fopen("snap_log.txt", "a+");
    fprintf(plog, "track_id: %li, frame_id: %li\n",
            snapshot_info->track_id_, snapshot_info->origin_image_frame_->frame_id_);
    fclose(plog);
    return 0;
  }

  void write_json(HobotXRoc::SnapshotInfo *snapshot_info) {
    char filename[50];
    snprintf(filename, sizeof(filename), "FaceSnap%li_%li_%li.jpeg",
             snapshot_info->origin_image_frame_->time_stamp_,
             snapshot_info->track_id_, snapshot_info->origin_image_frame_->frame_id_);

    auto info = std::static_pointer_cast<RectInfo>(snapshot_info->userdata_[0]);

    int id = static_cast<int>(snapshot_info->track_id_);
    if (id_map.find(id) == id_map.end())
      return;

    Json::Value snap;
    Json::Value attributes;

    snap["track_id"] = info->track_id_;
    snap["name"] = id_map[info->track_id_] + ".bmp";
    attributes["size"] = info->size_;
    attributes["pitch"] = info->pitch_;
    attributes["yaw"] = info->yaw_;
    attributes["roll"] = info->roll_;
    attributes["pose"] = info->pose_;
    attributes["quality"] = info->quality_;
    attributes["lmk0"] = info->lmk0_;
    attributes["lmk1"] = info->lmk1_;
    attributes["lmk2"] = info->lmk2_;
    attributes["lmk3"] = info->lmk3_;
    attributes["lmk4"] = info->lmk4_;
    snap["filename"] = filename;
    snap["frame"] = snapshot_info->origin_image_frame_->frame_id_;
    snap["frame_timestamp"] = snapshot_info->origin_image_frame_->time_stamp_;
    snap["attributes"] = attributes;

    Json::StreamWriterBuilder builder;
    builder.settings_["indentation"] = "";
    std::unique_ptr<Json::StreamWriter> write(builder.newStreamWriter());
    std::ostringstream os;
    write->write(snap, &os);
    json_ofs_ << os.str() << std::endl;
  }

  int DumpSnap(HobotXRoc::SnapshotInfo *snapshot_info) {
    char filename[50];
    int id = static_cast<int>(snapshot_info->track_id_);
    if (id_map.find(id) == id_map.end())
      return 0;

    if (access("snaps", 2) != 0)
      mkdir("snaps", 00700);
    snprintf(filename, sizeof(filename), "./snaps/FaceSnap%li_%li_%li.yuv",
             snapshot_info->origin_image_frame_->time_stamp_,
             snapshot_info->track_id_, snapshot_info->origin_image_frame_->frame_id_);

    FILE *pfile = nullptr;
    pfile = fopen(filename, "w");
    if (!pfile) {
      std::cerr << "open file " << filename << " failed" << std::endl;
      return -1;
    }
    if (!fwrite(snapshot_info->snap_->data_, snapshot_info->snap_->data_size_, 1, pfile)) {
      std::cout << "fwrite data to " << filename << " failed" << std::endl;
      fclose(pfile);
    }
    fclose(pfile);
    return 0;
  }


  void OnCallback(const HobotXRoc::OutputDataPtr &output) {
    using HobotXRoc::BaseDataVector;
    for (const auto &data : output->datas_) {
      if (data->error_code_ < 0) {
        std::cout << "data error: " << data->error_code_ << std::endl;
        continue;
      }
      auto *pdata = dynamic_cast<BaseDataVector *>(data.get());
      if (!pdata->datas_.empty()) {
        for (const auto &item : pdata->datas_) {
          assert("SnapshotInfo" == item->type_);
          //  get the item score
          auto snapshot_info = std::static_pointer_cast<HobotXRoc::SnapshotInfo>(item);
          WriteLog(snapshot_info.get());
          DumpSnap(snapshot_info.get());
          write_json(snapshot_info.get());
          auto userdata = snapshot_info->userdata_;
          std::cout << "track_id:" << snapshot_info->track_id_
                    << " time_stamp_:" << snapshot_info->origin_image_frame_->frame_id_
                    << " select_score:" << snapshot_info->select_value_
                    << " basedata:" << userdata[0]->name_
                    << std::endl;
        }
      }
    }
  }
};

struct TrackState {
  uint64_t start = 0;
  uint64_t last = 0;
};

int BuildIdMap_dh5c(std::map<int, std::string> &map) {
  map[202] = "jzw";
  map[207] = "jzw";
  map[220] = "lxd";
  map[263] = "yhx";
  map[352] = "tf";
  map[327] = "tf";
  map[297] = "tf";
  map[339] = "zcy";
  map[236] = "swq";
  map[353] = "fjf";
  map[262] = "sj";
  map[264] = "dy";
  map[444] = "zxd";
  map[442] = "zxd";
  map[374] = "cpf";
  map[541] = "cpf";
  map[501] = "cpf";
  map[175] = "cpf";
  map[558] = "lsy";
  map[510] = "lsy";
  map[394] = "syc";
  map[539] = "mxd";
  map[565] = "gss";
  map[592] = "xxx";
  map[610] = "why";
  map[277] = "yhx";
  return 0;
}


int BuildIdMap_gsmk(std::map<int, std::string> &map) {
  map[48] = "xxx";
  map[105] = "yhx";
  map[117] = "zxd";
  map[126] = "lsy";
  map[133] = "lsy";
  map[171] = "sj";
  map[189] = "lxd";
  map[211] = "swq";
  map[221] = "dy";
  map[260] = "cpf";
  map[17] = "syc";
  map[29] = "why";
  map[215] = "why";
  map[324] = "why";
  map[277] = "jzw";
  map[139] = "gss";
  map[160] = "gss";
  map[201] = "zyd";
  map[204] = "tf";
  map[197] = "zcy";
  map[284] = "fjf";
  map[193] = "mls";
  map[198] = "mxd";
  return 0;
}

int BuildIdMap_xqmk(std::map<int, std::string> &map) {
  map[60] = "zyd";
  map[52] = "xxx";
  map[312] = "yml";
  map[77] = "lsy";
  map[316] = "jzw";
  map[175] = "th";
  map[152] = "zxw";
  map[371] = "zgq";
  map[190] = "wgh";
  map[392] = "fjf";
  map[56] = "dy";
  map[61] = "lxd";
  map[132] = "zxd";
  map[359] = "zxd";
  map[91] = "syc";
  map[422] = "syc";
  map[431] = "zcy";
  map[153] = "tf";
  map[434] = "mxd";
  map[194] = "swq";
  map[378] = "swq";
  map[138] = "lxj";
  map[344] = "lxj";
  map[342] = "cm";
  map[13] = "mls";
  map[396] = "why";

  return 0;
}


int ConstructInput(const std::string &smart_frame,
                   const std::string &video_path,
                   HobotXRoc::InputDataPtr &input) {
  using HobotXRoc::BaseData;
  using HobotXRoc::BaseDataPtr;
  using HobotXRoc::BaseDataVector;
  using HobotXRoc::InputData;
  using HobotXRoc::InputDataPtr;
  using HobotXRoc::BBox;
  using HobotXRoc::Number;
  using HobotXRoc::ImageFrame;
  using HobotXRoc::DataState;
  using HobotXRoc::PixelFormat;

  static std::map<unsigned, TrackState> track_map;

  unsigned id = 0, md_id = 0;
  float x0 = 0, x1 = 0, y0 = 0, y1 = 0, pose = 0, pitch = 0, yaw = 0, roll = 0, quality = 0, over_quality = 0;
  int lmk0 = 0, lmk1 = 0, lmk2 = 0, lmk3 = 0, lmk4 = 0;
  uint64_t timestamp, frame_num = 0;

  std::istringstream ss(smart_frame);
  ss >> frame_num >> timestamp;

  unsigned frame_width = 1920, frame_height = 1080;
  std::shared_ptr<ImageFrame> img_frame(new ImageFrame(frame_width * frame_height, nullptr));
  img_frame->name_ = "img_frame";
  img_frame->pixel_format_ = PixelFormat::RAW_GRAY;
  img_frame->height_ = frame_height;
  img_frame->width_ = frame_width;
  img_frame->stride_ = frame_width;
  img_frame->data_size_ = frame_width * frame_height;
  img_frame->time_stamp_ = timestamp;
  img_frame->frame_id_ = frame_num;
  input->datas_.push_back(BaseDataPtr(img_frame));
  if (!video_path.empty()) {
    FILE *pfile = nullptr;
    pfile = fopen(video_path.data(), "r");
    if (!pfile) {
      std::cout << "Open video_ failed" << std::endl;
      return -1;
    }
    if (!fread(img_frame->data_, frame_width * frame_height, 1, pfile)) {
      std::cout << "fread video_.yuv error" << std::endl;
      fclose(pfile);
      return -1;
    }
    fclose(pfile);
  }

  std::shared_ptr<BaseDataVector> track_id_list(new BaseDataVector());
  track_id_list->name_ = "track_id_list";
  input->datas_.push_back(BaseDataPtr(track_id_list));

  std::shared_ptr<BaseDataVector> select_score_list(new BaseDataVector());
  select_score_list->name_ = "select_score_list";
  input->datas_.push_back(BaseDataPtr(select_score_list));


  std::shared_ptr<BaseDataVector> box_list(new BaseDataVector());
  box_list->name_ = "box_list";
  input->datas_.push_back(BaseDataPtr(box_list));

  std::shared_ptr<BaseDataVector> userdata_list(new BaseDataVector());
  userdata_list->name_ = "box_userdata_list";
  input->datas_.push_back(BaseDataPtr(userdata_list));

  std::shared_ptr<BaseDataVector> disappeared_track_id_list(new BaseDataVector());
  disappeared_track_id_list->name_ = "disappeared_track_id_list";
  input->datas_.push_back(BaseDataPtr(disappeared_track_id_list));

  std::set<uint64_t> id_set;

  while (ss >> id) {
    ss >> md_id >> x0 >> y0 >> x1 >> y1;
    ss >> pose;
    ss >> pitch >> yaw >> roll;
    ss >> quality >> over_quality;
    ss >> lmk0 >> lmk1 >> lmk2 >> lmk3 >> lmk4;

    std::shared_ptr<BBox> bbox(new BBox());
    std::shared_ptr<Number> track_id(new Number());
    std::shared_ptr<Number> score(new Number());
    std::shared_ptr<BaseData> data(new HobotXRoc::BaseData);

    track_id->value_ = id;

    if (md_id == 0) {  // is fase
      id_set.insert(id);
      if (track_map.find(id) == track_map.end()) {
        TrackState state;
        state.start = frame_num;
        state.last = frame_num;
        track_map[id] = state;
      } else {
        track_map[id].last = frame_num;
      }
      data->name_ = std::to_string(timestamp) + "_" + std::to_string(id) + "_FaceData";
      float resp_width = x1 - x0;
      float resp_height = y1 - y0;
      float size = std::min(resp_width, resp_height);
      bbox->values_ = { x0, y0, x1, y1 };
      if (pose > 1000 && size > 40 && x0 > 10 && x1 < 1910 && y0 > 10 && y1 < 1070) {
        bbox->state_ = DataState::VALID;
      } else {
        bbox->state_ = DataState::FILTERED;
      }
      if (bbox->state_ == DataState::VALID) {
        //  grading for snaps
        unsigned size_min = 40, size_max = 200, size_inflexion = 80;
        unsigned pose_thr = 1000, pose_max = 2000;

        float frontalPos_weight_ = 0.3, size_weight_ = 0.2, blur_weight_ = 0.2, lmk_weight_ = 0.3;

        float normalized_frontal_pos = (pose- pose_thr) / (pose_max - pose_thr);
        float normalized_size = 0;
        if (size < size_min) {
          normalized_size = 0;
        } else if (size > size_max) {
          normalized_size = 1;
        } else {
          if (size > size_inflexion)
            normalized_size = (size - size_inflexion) / (size_max - size_inflexion) * 0.5f + 0.5f;
          else
            normalized_size = (size - size_min) / (size_inflexion - size_min) * 0.5f;
        }
        float normalized_blur = 0;
        if (quality < -200)
          normalized_blur = 1;
        else if (quality > 200)
          normalized_blur = 0;
        else
          normalized_blur = -(quality - 200) / 400;

        float normalized_lmk = 0;
        normalized_lmk += lmk0;
        normalized_lmk += lmk1;
        normalized_lmk += lmk2;
        normalized_lmk += lmk3;
        normalized_lmk += lmk4;
        normalized_lmk /= 75;
        if (normalized_lmk < 0)
          normalized_lmk = 0;
        if (normalized_lmk > 1)
          normalized_lmk = 1;

        float res = frontalPos_weight_ * normalized_frontal_pos +
            size_weight_ * normalized_size +
            blur_weight_ * normalized_blur +
            lmk_weight_ * normalized_lmk;

        score->value_ = 2000 * res;
      }
      box_list->datas_.push_back(BaseDataPtr(bbox));
      track_id_list->datas_.push_back(BaseDataPtr(track_id));
      select_score_list->datas_.push_back(BaseDataPtr(score));
      userdata_list->datas_.push_back(BaseDataPtr(data));
    } else { // head
      if (track_map.find(id) != track_map.end()) {
        if (id_set.find(id) == id_set.end()) {
          track_map[id].last = frame_num;
        }
      }
    }
  }

  auto iter = track_map.begin();
  while (iter != track_map.end()) {
    std::shared_ptr<Number> track_id(new Number());
    track_id->value_ = iter->first;
    if (frame_num - iter->second.last == 0) {
      track_id->state_ = DataState::DISAPPEARED;
      disappeared_track_id_list->datas_.push_back(BaseDataPtr(track_id));
      iter = track_map.erase(iter);
    } else {
      iter++;
    }
  }


//  while (ss >> id) {
//    ss >> md_id >> x0 >> y0 >> x1 >> y1;
//    ss >> pose;
//    ss >> pitch >> yaw >> roll;
//    ss >> quality >> over_quality;
//    ss >> lmk0 >> lmk1 >> lmk2 >> lmk3 >> lmk4;
//
//    std::shared_ptr<BBox> bbox(new BBox());
//    std::shared_ptr<Number> track_id(new Number());
//    std::shared_ptr<Number> score(new Number());
//
//    track_id->value_ = id;
//    score->value_ = over_quality;
//
//    if (md_id == 0) {
//      id_set.insert(id);
//      if (track_map.find(id) == track_map.end()) {
//        TrackState state;
//        state.start = frame_num;
//        state.last = frame_num;
//        track_map[id] = state;
//      } else {
//        track_map[id].last = frame_num;
//      }
//      float resp_width = x1 - x0;
//      float resp_height = y1 - y0;
//      float size = std::min(resp_width, resp_height);
//      bbox->values_ = { x0, y0, x1, y1 };
//      if (pose > 1000 && size > 40 && x0 > 10 && x1 < 1910 && y0 > 10 && y1 < 1070) {
//        bbox->state_ = DataState::VALID;
//      } else {
//        bbox->state_ = DataState::FILTERED;
//      }
//      std::shared_ptr<RectInfo> info(new RectInfo);
//      info->track_id_ = id;
//      info->pitch_ = pitch;
//      info->yaw_ = yaw;
//      info->roll_ = roll;
//      info->pose_ = pose;
//      info->size_ = size;
//      info->quality_ = quality;
//      info->lmk0_ = lmk0;
//      info->lmk1_ = lmk1;
//      info->lmk2_ = lmk2;
//      info->lmk3_ = lmk3;
//      info->lmk4_ = lmk4;
//
//      box_list->datas_.push_back(BaseDataPtr(bbox));
//      track_id_list->datas_.push_back(BaseDataPtr(track_id));
//      select_score_list->datas_.push_back(BaseDataPtr(score));
//      userdata_list->datas_.push_back(BaseDataPtr(info));
//    }
//  }
//
//  auto iter = track_map.begin();
//  while (iter != track_map.end()) {
//    std::shared_ptr<BBox> bbox(new BBox());
//    std::shared_ptr<Number> track_id(new Number());
//    std::shared_ptr<Number> score(new Number());
//    std::shared_ptr<BaseData> data(new HobotXRoc::BaseData);
//    track_id->value_ = iter->first;
//    if (frame_num - iter->second.last == 0) {
//      track_id->state_ = DataState::DISAPPEARED;
//      iter = track_map.erase(iter);
//    } else {
//      if (id_set.find(iter->first) == id_set.end()) {
//        track_id->state_ = DataState::VALID;
//        bbox->state_ = DataState::DISAPPEARED;
//      }
//      iter++;
//    }
//    box_list->datas_.push_back(BaseDataPtr(bbox));
//    track_id_list->datas_.push_back(BaseDataPtr(track_id));
//    select_score_list->datas_.push_back(BaseDataPtr(score));
//    userdata_list->datas_.push_back(BaseDataPtr(data));
//  }


  return 0;
}


int main(int argc, char const *argv[]) {
  // init the SDK
  HobotXRoc::XRocSDK* flow = HobotXRoc::XRocSDK::CreateSDK();
  Callback callback;
  flow->SetCallback(std::bind(&Callback::OnCallback, &callback, std::placeholders::_1));
  flow->SetConfig("config_file", "../../test/testing_config/snapshot.json");
  flow->Init();

  callback.json_ofs_.open("./snap_res.json");

//  BuildIdMap_dh5c(callback.id_map);
//  std::string algo_res = "/media/psf/Home/Data/dh5c/logdh5cdump.log";
//  std::string img_list_path = "/media/psf/Home/Data/dh5c/img_list.txt";


  BuildIdMap_gsmk(callback.id_map);
  std::string algo_res = "/media/psf/Home/Data/gsmk/loggsmkdump.log";
  std::string img_list_path = "/media/psf/Home/Data/gsmk/img_list.txt";


//  BuildIdMap_xqmk(callback.id_map);
//  std::string algo_res = "/media/psf/Home/Data/xqmk/logxqmkdump.log";
//  std::string img_list_path = "/media/psf/Home/Data/xqmk/img_list.txt";

  std::ifstream fin(algo_res.data(), std::ios::binary);
  if (fin.fail()) {
    std::cout << "Open track_result failed" << std::endl;
    return -1;
  }
  FILE *image_list = nullptr;
  if (!img_list_path.empty())
    image_list = fopen(img_list_path.data(), "r");
  std::string smart_frame;

  while (getline(fin, smart_frame)) {
    char img_fn[1024] = "";
    if (!img_list_path.empty() && EOF == fscanf(image_list, "%s\n", img_fn))
      break;
    HobotXRoc::InputDataPtr inputdata(new HobotXRoc::InputData());
    int ret = ConstructInput(smart_frame, img_fn, inputdata);
    if (ret == 0) {
      // sync Grading
      auto out = flow->SyncPredict(inputdata);
      callback.OnCallback(out);
    }
  }

  delete flow;
  return 0;
}
