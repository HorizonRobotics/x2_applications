/*
 * @Description: implement of vioplugin
 * @Author: fei.cheng@horizon.ai
 * @Date: 2019-08-26 16:17:25
 * @Author: songshan.gong@horizon.ai
 * @Date: 2019-09-26 16:17:25
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-10-16 15:41:38
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */

#include <unistd.h>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <unordered_map>

#include "hobotlog/hobotlog.hpp"
#include "opencv2/opencv.hpp"
#include "utils/executor.h"

#include "vioplugin/viomessage.h"
#include "vioplugin/vioprocess.h"
#include "vioplugin/vioproduce.h"

#define CHECK_NULL(p) \
  if (nullptr == p) return kHorizonVisionErrorParam;

namespace horizon {
namespace vision {
namespace xpluginflow {
namespace vioplugin {

const std::unordered_map<std::string, VioProduce::TSTYPE>
VioProduce::str2ts_type_ = {
  {"raw_ts", TSTYPE::RAW_TS},
  {"frame_id", TSTYPE::FRAME_ID},
  {"input_coded", TSTYPE::INPUT_CODED}
};

VioConfig *VioConfig::config_ = nullptr;

std::string VioConfig::GetValue(const std::string &key) const {
  std::lock_guard<std::mutex> lk(mutex_);
  if (json_[key].empty()) {
    LOGW << "Can not find key: " << key;
    return "";
  }

  return json_[key].asString();
}

Json::Value VioConfig::GetJson() const { return this->json_; }

VioConfig *VioConfig::GetConfig() {
  if (config_ != nullptr) {
    return config_;
  } else {
    return nullptr;
  }
}

bool VioConfig::SetConfig(VioConfig *config) {
  if (config != nullptr) {
    config_ = config;
    return true;
  } else {
    return false;
  }
}

int VioCamera::read_time_stamp(void *addr, uint64_t *timestamp) {
  LOGI << "read time stamp";
  uint8_t *addrp = reinterpret_cast<uint8_t *>(addr);
  uint8_t *datap = reinterpret_cast<uint8_t *>(timestamp);
  int i = 0;
  for (i = 15; i >= 0; i--) {
    if (i % 2)
      datap[(15 - i) / 2] |= (addrp[i] & 0x0f);
    else
      datap[(15 - i) / 2] |= ((addrp[i] & 0x0f) << 4);
  }

  return 0;
}

std::shared_ptr<VioProduce> VioProduce::CreateVioProduce(
    const std::string &data_source) {
  auto config = VioConfig::GetConfig();
  HOBOT_CHECK(config);
  auto json = config->GetJson();
  std::shared_ptr<VioProduce> Vio_Produce;
  if ("jpeg_image_list" == data_source) {
    Vio_Produce = std::make_shared<JpegImageList>(
        json["vio_cfg_file"]["jpeg_image_list"].asCString());
  } else if ("nv12_image_list" == data_source) {
    Vio_Produce = std::make_shared<Nv12ImageList>(
        json["vio_cfg_file"]["nv12_image_list"].asCString());
  } else if ("panel_camera" == data_source) {
    Vio_Produce = std::make_shared<PanelCamera>(
        json["vio_cfg_file"]["panel_camera"].asString());
  } else if ("ipc_camera" == data_source) {
    Vio_Produce = std::make_shared<IpcCamera>(
        json["vio_cfg_file"]["ipc_camera"].asString());
  } else if ("image" == data_source) {
    Vio_Produce = std::make_shared<PanelCamera>(
        json["vio_cfg_file"]["image"].asCString());
  } else {
    std::cout << "data source " << data_source << " is unsupported";
  }
  Vio_Produce->cam_type_ = json["cam_type"].asString();
  Vio_Produce->ts_type_ =
    str2ts_type_.find(json["ts_type"].asString())->second;
  return Vio_Produce;
}

void VioProduce::WaitUntilAllDone() {
  LOGW << "consumed_vio_buffers_=" << consumed_vio_buffers_;
  while (consumed_vio_buffers_ > 0) {
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
}

bool VioProduce::AllocBuffer() {
  LOGV << "AllocBuffer()";
  LOGV << "count: " << consumed_vio_buffers_;
  if (consumed_vio_buffers_ < max_vio_buffer_) {
    consumed_vio_buffers_++;
    return true;
  }
  return false;
}

void VioProduce::FreeBuffer() {
  LOGV << "FreeBuffer()";
  if (0 == consumed_vio_buffers_) return;
  consumed_vio_buffers_--;
}

int VioProduce::SetConfig(VioConfig *config) {
  config_ = config;
  return kHorizonVisionSuccess;
}

int VioProduce::SetListener(const Listener &callback) {
  push_data_cb_ = callback;
  return kHorizonVisionSuccess;
}

int VioProduce::Finish() {
  if (is_running_) {
    is_running_ = false;
  }
  WaitUntilAllDone();
  return kHorizonVisionSuccess;
}

// 将指定路径的图像转换为HorizonVisionImageFrame格式
HorizonVisionImageFrame *VioProduce::GetImageFrame(const std::string &path) {
  HorizonVisionImage *bgr_img = nullptr;
  std::string image_path = path;
  // avoid windows system line break
  if (!image_path.empty() && image_path.back() == '\r') {
    image_path.erase(image_path.length() - 1);
  }
  auto res = HorizonFillFromFile(path.c_str(), &bgr_img);
  if (res != 0) {
    LOGE << "Failed to load image " << path << ", error code is " << res;
    return nullptr;
  }
  HOBOT_CHECK(bgr_img);
  static uint64_t frame_id = 0;
  HorizonVisionImageFrame *frame = nullptr;
  HorizonVisionAllocImageFrame(&frame);
  frame->channel_id = 0;
  frame->frame_id = frame_id++;
  frame->time_stamp = static_cast<uint64_t>(std::time(nullptr));
  // 转换图像数据
  HorizonConvertImage(bgr_img, &frame->image, kHorizonVisionPixelFormatRawBGR);
  HorizonVisionFreeImage(bgr_img);
  return frame;
}

// 补全图像，保证图像按照规定分辨率输入
int VioProduce::PadImage(HorizonVisionImage *img, uint32_t dst_width,
                         uint32_t dst_height) {
  if (!img) {
    return kHorizonVisionErrorParam;
  }
  if (img->height == dst_height && img->width == dst_width) {
    return kHorizonVisionSuccess;
  }
  cv::Mat in_img(img->height, img->width, CV_8UC3);
  memcpy(in_img.data, img->data, img->data_size);
  HOBOT_CHECK(!in_img.empty());
  uint32_t dst_data_size = dst_width * dst_height * 3;
  cv::Mat out_img = cv::Mat(dst_height, dst_width, CV_8UC3, cv::Scalar::all(0));
  if (img->width > dst_width || img->height > dst_height) {
    auto src_width = static_cast<float>(img->width);
    auto src_height = static_cast<float>(img->height);
    auto aspect_ratio = src_width / src_height;
    auto dst_ratio = static_cast<float>(dst_width) / dst_height;
    uint32_t resized_width = -1;
    uint32_t resized_height = -1;
    // 等比缩放
    if (aspect_ratio >= dst_ratio) {
      resized_width = dst_width;
      resized_height =
          static_cast<uint32_t>(src_height * dst_width / src_width);
    } else {
      resized_width =
          static_cast<uint32_t>(src_width * dst_height / src_height);
      resized_height = dst_height;
    }
    cv::resize(in_img, in_img, cv::Size(resized_width, resized_height));
  }

  // 复制到目标图像中间
  in_img.copyTo(out_img(cv::Rect((dst_width - in_img.cols) / 2,
                                 (dst_height - in_img.rows) / 2, in_img.cols,
                                 in_img.rows)));
  HorizonVisionCleanImage(img);
  img->data =
      reinterpret_cast<uint8_t *>(std::calloc(dst_data_size, sizeof(uint8_t)));
  memcpy(img->data, out_img.data, dst_data_size);
  img->data_size = dst_data_size;
  img->width = dst_width;
  img->height = dst_height;
  img->stride = dst_width;
  img->stride_uv = dst_width;
  return kHorizonVisionSuccess;
}

int VioProduce::Start() {
  auto func = std::bind(&VioProduce::Run, this);
  task_future_ = Executor::GetInstance()->AddTask(func);
  return 0;
}

int VioProduce::Stop() {
  this->Finish();
  LOGD << "wait task to finish";
  task_future_.get();
  LOGD << "task done";
  #if 0
  Executor::GetInstance()->Stop();
  LOGD << "Stop Executor";
  #endif
  return 0;
}

std::shared_ptr<ImageVioMessage> VioProduce::Image2ImageMessageInput(
    const HorizonVisionImage *image) {
  auto *pvio_image =
      reinterpret_cast<img_info_t *>(std::calloc(1, sizeof(img_info_t)));
  int ret = RawImage::FillVIOImageByVisionImage(pvio_image, image);
  if (kHorizonVisionSuccess != ret) {
    std::free(pvio_image);
    return nullptr;
  }
  auto **images = new HorizonVisionImageFrame *[1];
  HorizonVisionImageFrame *image_frame = nullptr;
  HorizonVisionAllocImageFrame(&image_frame);
  image_frame->channel_id = 0;
  image_frame->frame_id = 0;
  image_frame->time_stamp = pvio_image->timestamp;
  image_frame->image.height = pvio_image->src_img.height;
  image_frame->image.width = pvio_image->src_img.width;
  image_frame->image.pixel_format = kHorizonVisionPixelFormatX2PYM;
  image_frame->image.stride = pvio_image->src_img.width;
  image_frame->image.stride_uv = pvio_image->src_img.width;
  image_frame->image.data = reinterpret_cast<uint8_t *>(pvio_image);
  image_frame->image.data_size = sizeof(img_info_t);
  images[0] = image_frame;
  std::shared_ptr<ImageVioMessage> input(new ImageVioMessage(images, 1),
                                         [&](ImageVioMessage *p) {
                                           if (p) {
                                             p->FreeImage();
                                             delete p;
                                           }
                                           p = nullptr;
                                         });
  return input;
}

bool GetPyramidInfo(img_info_t *pvio_image, char *data, int len) {
  src_img_info_t src_img_info;
  auto ret = hb_vio_get_info(HB_VIO_FEEDBACK_SRC_INFO, &src_img_info);
  if (ret < 0) {
    LOGE << "hb_vio_get_info failed";
    return false;
  }
  std::memcpy(src_img_info.src_img.y_vaddr, data, len);
  ret = hb_vio_set_info(HB_VIO_FEEDBACK_FLUSH, &src_img_info);
  if (ret < 0) {
    LOGE << "hb_vio_feedback_flush failed";
    return false;
  }
  ret = hb_vio_pym_process(&src_img_info);
  if (ret < 0) {
    LOGE << "hb_vio_pym_process failed";
    return false;
  }
  ret = hb_vio_get_info(HB_VIO_PYM_INFO, pvio_image);
  if (ret < 0) {
    LOGE << "hb_vio_pyramid_info failed";
    return false;
  }
  return true;
}

//通过多个金字塔获取输入图像数据的输出图像数据
bool GetPyramidInfo(mult_img_info_t *pvio_image, char *data, int len) {
  mult_src_info_t mult_src_info;
  auto ret = hb_vio_get_info(HB_VIO_FEEDBACK_SRC_MULT_INFO, &mult_src_info);
  if (ret < 0) {
    LOGE << "hb_vio_get_info failed";
    return false;
  }
  std::memcpy(mult_src_info.src_img_info[0].src_img.y_vaddr, data, len);
  std::memcpy(mult_src_info.src_img_info[1].src_img.y_vaddr, data, len);
  ret = hb_vio_mult_pym_process(&mult_src_info);
  if (ret < 0) {
    LOGE << "hb_vio_mult_pym_process failed";
    return false;
  }
  ret = hb_vio_get_info(HB_VIO_PYM_MULT_INFO, pvio_image);
  if (ret < 0) {
    LOGE << "hb_vio_pyramid_info failed";
    return false;
  }
  return true;
}

static int DumpPyramidImage(const addr_info_t &vio_image,
                            const std::string &path) {
  auto height = vio_image.height;
  auto width = vio_image.width;
  if (height <= 0 || width <= 0) {
    LOGE << "pyrmid: " << width << "x" << height;
    return -1;
  }
  cv::Mat nv12(height * 3 / 2, width, CV_8UC1, vio_image.y_vaddr);
  cv::Mat bgr;
  cv::cvtColor(nv12, bgr, CV_YUV2BGR_NV12);
  cv::imwrite(path.c_str(), bgr);
  LOGD << "saved path: " << path;
  return 0;
}

static void DumpPyramidImage(const img_info_t &vio_info, const int pyr_index,
                             const std::string &path) {
  LOGD << "DumpPyramidImage";
  addr_info_t addr_info;
  if (-1 == pyr_index) {
    addr_info = vio_info.src_img;
  } else {
    addr_info = vio_info.down_scale[pyr_index];
  }
  DumpPyramidImage(addr_info, path);
}

int VioCamera::Run() {
  bool check_timestamp = false;
  auto check_timestamp_str = getenv("check_timestamp");
  if (check_timestamp_str && !strcmp(check_timestamp_str, "ON")) {
    check_timestamp = true;
  }
  if (is_running_) return kHorizonVioErrorAlreadyStart;
  static uint64_t frame_id = 0;
  static int64_t last_timestamp = 0;
  is_running_ = true;
  while (is_running_) {
    uint32_t img_num = 1;
    if (cam_type_ == "mono") {
      auto *pvio_image =
          reinterpret_cast<img_info_t *>(std::calloc(1, sizeof(img_info_t)));
      auto res = hb_vio_get_info(HB_VIO_PYM_INFO, pvio_image);

      uint64_t img_time = 0;

      if (ts_type_ == TSTYPE::INPUT_CODED && check_timestamp && res == 0 &&
         pvio_image != nullptr && pvio_image->src_img.y_vaddr != nullptr) {
        read_time_stamp(pvio_image->src_img.y_vaddr, &img_time);
        LOGI << "src img ts:  " << img_time;

        if (pvio_image->timestamp != img_time) {
          LOGE << "timestamp is different!!! "
               << "image info ts: " << pvio_image->timestamp;
        }
        pvio_image->timestamp = img_time;
      } else if (ts_type_ == TSTYPE::FRAME_ID) {
        pvio_image->timestamp = pvio_image->frame_id;
      }
      if (res != 0 ||
          (check_timestamp && pvio_image->timestamp == last_timestamp)) {
        LOGW << "hb_vio_get_info: " << res;
        hb_vio_free(pvio_image);
        std::free(pvio_image);
        continue;
      }
#ifdef DEBUG
      auto dump_pyramid_level_env = getenv("dump_pyramid_level");
      if (dump_pyramid_level_env) {
        int dump_pyr_level = std::stoi(dump_pyramid_level_env);
        std::string name =
            "pyr_images/vio_" + std::to_string(frame_id) + ".jpg";
        DumpPyramidImage(*pvio_image, dump_pyr_level, name);
      }
#endif  // DEBUG
      if (check_timestamp && last_timestamp != 0) {
        HOBOT_CHECK(pvio_image->timestamp > last_timestamp)
            << pvio_image->timestamp << " <= " << last_timestamp;
      }
      LOGI << "Vio TimeStamp: " << pvio_image->timestamp;
      last_timestamp = pvio_image->timestamp;
      if (frame_id % sample_freq_ == 0 && AllocBuffer()) {
        if (!is_running_) {
          LOGW << "stop vio job";
          hb_vio_free(pvio_image);
          std::free(pvio_image);
          FreeBuffer();
          break;
        }
        auto **images = new HorizonVisionImageFrame *[img_num];
        HorizonVisionImageFrame *image_frame = nullptr;
        HorizonVisionAllocImageFrame(&image_frame);
        image_frame->channel_id = 0;
        image_frame->frame_id = frame_id;
        image_frame->time_stamp = pvio_image->timestamp;
        image_frame->image.height = pvio_image->src_img.height;
        image_frame->image.width = pvio_image->src_img.width;
        image_frame->image.pixel_format = kHorizonVisionPixelFormatX2PYM;
        image_frame->image.stride = pvio_image->src_img.width;
        image_frame->image.stride_uv = pvio_image->src_img.width;
        image_frame->image.data = reinterpret_cast<uint8_t *>(pvio_image);
        image_frame->image.data_size = sizeof(img_info_t);
        images[0] = image_frame;
        std::shared_ptr<VioMessage> input(new ImageVioMessage(images, img_num),
                                          [&](ImageVioMessage *p) {
                                            if (p) {
                                              p->FreeImage();
                                              FreeBuffer();
                                              delete (p);
                                            }
                                            p = nullptr;
                                          });
        HOBOT_CHECK(push_data_cb_);
        if (push_data_cb_) {
          push_data_cb_(input);
          LOGI << "Push Image message!!!";
        }
      } else {
        LOGV << "NO VIO BUFFER ";
        auto input = std::make_shared<DropVioMessage>(
          static_cast<uint64_t>(pvio_image->timestamp), frame_id);
        if (push_data_cb_) push_data_cb_(input);
        LOGI << "Push Drop message!!!";
        hb_vio_free(pvio_image);
        std::free(pvio_image);
      }
    } else if (cam_type_ == "dual") {
      auto *pvio_image = reinterpret_cast<mult_img_info_t *>(
          std::calloc(1, sizeof(mult_img_info_t)));
      auto res = hb_vio_get_info(HB_VIO_PYM_MULT_INFO, pvio_image);
      uint64_t img_time = 0;

      if (ts_type_ == TSTYPE::INPUT_CODED && check_timestamp && res == 0 &&
         pvio_image != nullptr &&
         pvio_image->img_info[0].src_img.y_vaddr != nullptr) {
        read_time_stamp(pvio_image->img_info[0].src_img.y_vaddr, &img_time);
        LOGE << "src img ts:  " << img_time;

        if (pvio_image->img_info[0].timestamp != img_time) {
          LOGE << "timestamp is different!!! "
               << "image info ts: " << pvio_image->img_info[0].timestamp;
        }
        pvio_image->img_info[0].timestamp = img_time;
      } else if (ts_type_ == TSTYPE::FRAME_ID) {
        pvio_image->img_info[0].timestamp =
          pvio_image->img_info[0].frame_id;
      }
      if (res != 0 || pvio_image->img_info[0].timestamp == last_timestamp) {
        std::free(pvio_image);
        continue;
      }
      if (check_timestamp && last_timestamp != 0) {
        HOBOT_CHECK(pvio_image->img_info[0].timestamp > last_timestamp)
            << pvio_image->img_info[0].timestamp << " <= " << last_timestamp;
      }
      last_timestamp = pvio_image->img_info[0].timestamp;

      if (frame_id % sample_freq_ == 0 && AllocBuffer()) {
        if (!is_running_) {
          LOGF << "stop vio job";
          hb_vio_mult_free(pvio_image);
          std::free(pvio_image);
          FreeBuffer();
          break;
        }
        auto **images = new HorizonVisionImageFrame *[pvio_image->img_num];
        for (int i = 0; i < pvio_image->img_num; ++i) {
          HorizonVisionImageFrame *image_frame = nullptr;
          HorizonVisionAllocImageFrame(&image_frame);
          image_frame->channel_id = 0;
          image_frame->frame_id = frame_id;
          image_frame->time_stamp = pvio_image->img_info[i].timestamp;
          image_frame->image.pixel_format = kHorizonVisionPixelFormatX2PYM;
          image_frame->image.data =
              reinterpret_cast<uint8_t *>(&pvio_image->img_info[i]);
          image_frame->image.height = pvio_image->img_info[i].src_img.height;
          image_frame->image.width = pvio_image->img_info[i].src_img.width;
          images[i] = image_frame;
        }

        std::shared_ptr<VioMessage> input(
            new ImageVioMessage(images, pvio_image->img_num, true, pvio_image),
            [&](ImageVioMessage *p) {
              if (p) {
                p->FreeImage();
                FreeBuffer();
                delete p;
              }
              p = nullptr;
            });
        HOBOT_CHECK(push_data_cb_);
        if (push_data_cb_) {
          push_data_cb_(input);
        }
      } else {
        LOGV << "NO VIO BUFFER ";
        auto input = std::make_shared<DropVioMessage>(
          static_cast<uint64_t>(pvio_image[0].img_info->timestamp), frame_id);
        if (push_data_cb_) push_data_cb_(input);
        hb_vio_mult_free(pvio_image);
        std::free(pvio_image);
      }
    } else {
      LOGF << "Don't support type: " << cam_type_;
    }
    ++frame_id;
  }

  return kHorizonVisionSuccess;
}

PanelCamera::PanelCamera(const std::string &cfg_file) {
  std::ifstream if_cfg(cfg_file);
  HOBOT_CHECK(if_cfg.is_open()) << "config file load failed!!";
  std::stringstream oss_config;
  oss_config << if_cfg.rdbuf();
  if_cfg.close();
  Json::Value config_jv;
  oss_config >> config_jv;
  cam_type_ = config_jv["type"].asString();
  std::string cam_cfg_file, vio_cfg_file;

  if (cam_type_ == "mono") {
    cam_cfg_file = config_jv["mono_cam_cfg_file"].asString();
    vio_cfg_file = config_jv["mono_vio_cfg_file"].asString();
  } else if (cam_type_ == "dual") {
    cam_cfg_file = config_jv["dual_cam_cfg_file"].asString();
    vio_cfg_file = config_jv["dual_vio_cfg_file"].asString();
  }
  camera_index_ = config_jv["cam_index"].asInt();
  hb_vio_init(vio_cfg_file.c_str());
  hb_cam_init(camera_index_, cam_cfg_file.c_str());
  hb_cam_start(camera_index_);
  hb_vio_start();
}

PanelCamera::~PanelCamera() {
  hb_vio_stop();
  hb_cam_stop(camera_index_);
  hb_cam_deinit(camera_index_);
  hb_vio_deinit();
}

IpcCamera::IpcCamera(const std::string &vio_cfg_file) {
  hb_vio_init(vio_cfg_file.c_str());
  hb_vio_start();
}

IpcCamera::~IpcCamera() {
  hb_vio_stop();
  hb_vio_deinit();
}

ImageList::ImageList(const char *vio_cfg_file) : VioProduce() {
  auto ret = hb_vio_init(vio_cfg_file);
  HOBOT_CHECK_LE(ret, 0) << "vio init failed";
  ret = hb_vio_start();
  HOBOT_CHECK_LE(ret, 0) << "vio start failed";
}

ImageList::~ImageList() {
  hb_vio_stop();
  hb_vio_deinit();
}

int ImageList::Run() {
  if (is_running_) return kHorizonVioErrorAlreadyStart;
  static uint64_t frame_id = 0;

  auto file_path = config_->GetValue("file_path");
  std::ifstream ifs(file_path);
  if (!ifs.good()) {
    LOGF << "Open file failed: " << file_path;
    return kHorizonVisionErrorParam;
  }
  std::string image_path;
  is_running_ = true;
  std::vector<std::string> image_path_list;
  while (std::getline(ifs, image_path)) {
    // trim the spaces in the beginning
    image_path.erase(0, image_path.find_first_not_of(' '));
    // trim the spaces in the end
    image_path.erase(image_path.find_last_not_of(' ') + 1, std::string::npos);
    image_path_list.emplace_back(image_path);
  }
  ifs.close();
  LOGI << "Finish importing images";
  uint32_t img_count = 0;
  auto image_num = image_path_list.size();
  while (is_running_) {
    if (img_count < image_num) {
      if (!AllocBuffer()) {
        LOGV << "NO VIO_FB_BUFFER";
        std::this_thread::sleep_for(std::chrono::microseconds(1));
        continue;
      }
      image_path = image_path_list[img_count++];
      if (cam_type_ == "mono") {
        auto *pvio_image =
            reinterpret_cast<img_info_t *>(std::calloc(1, sizeof(img_info_t)));
        auto ret = FillVIOImageByImagePath(pvio_image, image_path);
        auto **images = new HorizonVisionImageFrame *[1];
        HorizonVisionImageFrame *image_frame = nullptr;
        HorizonVisionAllocImageFrame(&image_frame);
        if (ret) {
          image_frame->channel_id = 0;
          image_frame->frame_id = frame_id++;
          image_frame->time_stamp = pvio_image->timestamp;
          image_frame->image.height = pvio_image->src_img.height;
          image_frame->image.width = pvio_image->src_img.width;
          image_frame->image.pixel_format = kHorizonVisionPixelFormatX2PYM;
          image_frame->image.stride = pvio_image->src_img.width;
          image_frame->image.stride_uv = pvio_image->src_img.width;
          image_frame->image.data = reinterpret_cast<uint8_t *>(pvio_image);
          image_frame->image.data_size = sizeof(img_info_t);
        } else {
          std::free(pvio_image);
        }
        images[0] = image_frame;
        std::shared_ptr<VioMessage> input(new ImageVioMessage(images, 1, ret),
                                          [&](ImageVioMessage *p) {
                                            if (p) {
                                              p->FreeImage();
                                              FreeBuffer();
                                              delete p;
                                            }
                                            p = nullptr;
                                          });
        if (push_data_cb_) push_data_cb_(input);
        LOGD << "Push Image message!!!";
      } else if (cam_type_ == "dual") {
        auto *pvio_image = reinterpret_cast<mult_img_info_t *>(
            std::calloc(1, sizeof(mult_img_info_t)));
        auto ret = FillVIOImageByImagePath(pvio_image, image_path);
        auto **images = new HorizonVisionImageFrame *[2];
        if (ret) {
          for (int i = 0; i < pvio_image->img_num; ++i) {
            HorizonVisionImageFrame *image_frame = nullptr;
            HorizonVisionAllocImageFrame(&image_frame);
            auto pvio = pvio_image->img_info[i];
            image_frame->channel_id = 0;
            image_frame->frame_id = frame_id;
            image_frame->time_stamp = pvio.timestamp;
            image_frame->image.height = pvio.src_img.height;
            image_frame->image.width = pvio.src_img.width;
            image_frame->image.pixel_format = kHorizonVisionPixelFormatX2PYM;
            image_frame->image.stride = pvio.src_img.width;
            image_frame->image.stride_uv = pvio.src_img.width;
            image_frame->image.data = reinterpret_cast<uint8_t *>(&pvio);
            image_frame->image.data_size = sizeof(img_info_t);
            images[i] = image_frame;
          }
        } else {
          std::free(pvio_image);
        }
        ++frame_id;
        std::shared_ptr<VioMessage> input(
            new ImageVioMessage(images, pvio_image->img_num, ret, pvio_image),
            [&](ImageVioMessage *p) {
              if (p) {
                p->FreeImage();
                FreeBuffer();
                delete p;
              }
              p = nullptr;
            });
        if (push_data_cb_) push_data_cb_(input);
        LOGD << "Push Image message!!!";
      } else {
        LOGF << "Don't support type: " << cam_type_;
      }
    } else {
      LOGI << "No data left";
      break;
    }
  }  // no running
  is_running_ = false;
  return kHorizonVisionSuccess;
}

RawImage::RawImage(const char *vio_cfg_file) : VioProduce() {
  auto ret = hb_vio_init(vio_cfg_file);
  HOBOT_CHECK_LE(ret, 0) << "vio init failed";
  ret = hb_vio_start();
  HOBOT_CHECK_LE(ret, 0) << "vio start failed";
}

RawImage::~RawImage() {
  is_running_ = false;
  hb_vio_stop();
  hb_vio_deinit();
}

int RawImage::Run() {
  if (is_running_) return kHorizonVioErrorAlreadyStart;
  is_running_ = true;
  return kHorizonVisionSuccess;
}

// 通过horizonvisionimage输入获取金字塔数据
int RawImage::FillVIOImageByVisionImage(img_info_t *pvio_image,
                                        const HorizonVisionImage *image) {
  if (!image) {
    LOGF << "Failed to load image";
    return kHorizonVisionErrorParam;
  }
  HorizonVisionImage *cp_image;
  HorizonVisionAllocImage(&cp_image);
  HorizonVisionCopyImage(const_cast<HorizonVisionImage *>(image), true,
                         cp_image);
  auto ret = VioProduce::PadImage(cp_image, 1920, 1080);
  if (ret != kHorizonVisionSuccess) {
    LOGF << "Failed to pad image, error code is " << ret;
    HorizonVisionFreeImage(cp_image);
    return ret;
  }
  HorizonVisionImage *nv12;
  HorizonVisionAllocImage(&nv12);
  HorizonConvertImage(cp_image, nv12, kHorizonVisionPixelFormatRawNV12);
  HorizonVisionFreeImage(cp_image);
  if (!GetPyramidInfo(pvio_image, reinterpret_cast<char *>(nv12->data),
                      nv12->height * nv12->width * 3 / 2)) {
    HorizonVisionFreeImage(nv12);
    return kHorizonVisionErrorParam;
  }
  HorizonVisionFreeImage(nv12);
  return ret;
}

// 此函数用于通过路径将nv12/jpepg等格式回灌
template <typename T>
bool ImageList::FillVIOImageByImagePath(T *pvio_image,
                                        const std::string &img_name) {
  auto data_source = config_->GetValue("data_source");
  if (data_source == "jpeg_image_list") {
    auto image = GetImageFrame(img_name);
    auto pad_width = std::stoi(config_->GetValue("pad_width"));
    auto pad_height = std::stoi(config_->GetValue("pad_height"));
    auto res = PadImage(&image->image, pad_width, pad_height);
    if (res != 0) {
      LOGF << "Failed to pad image " << img_name << ", error code is " << res;
      return false;
    }
    HorizonVisionImage *nv12;
    HorizonVisionAllocImage(&nv12);
    HorizonConvertImage(&image->image, nv12, kHorizonVisionPixelFormatRawNV12);
    auto ret = GetPyramidInfo(pvio_image, reinterpret_cast<char *>(nv12->data),
                              nv12->height * nv12->width * 3 / 2);
    HorizonVisionFreeImage(nv12);
    HorizonVisionFreeImageFrame(image);
    return ret;
  } else if (data_source == "nv12_image_list") {
    if (access(img_name.c_str(), F_OK) != 0) {
      LOGE << "File not exist: " << img_name;
      return false;
    }
    std::ifstream ifs(img_name, std::ios::in | std::ios::binary);
    if (!ifs) {
      LOGE << "Failed load " << img_name;
    }
    ifs.seekg(0, std::ios::end);
    int len = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    char *data = new char[len];
    ifs.read(data, len);
    auto ret = GetPyramidInfo(pvio_image, data, len);
    delete[] data;
    ifs.close();
    return ret;
  } else {
    LOGF << "Don't support data source: " << data_source;
    return false;
  }
}

}  // namespace vioplugin
}  // namespace xpluginflow
}  // namespace vision
}  // namespace horizon
