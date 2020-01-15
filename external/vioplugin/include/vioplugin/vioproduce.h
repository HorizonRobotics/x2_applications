/*
 * @Description: implement of vioplugin
 * @Author: fei.cheng@horizon.ai
 * @Date: 2019-08-26 16:17:25
 * @Author: songshan.gong@horizon.ai
 * @Date: 2019-09-26 16:17:25
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-10-16 15:41:22
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */

#ifndef INCLUDE_VIOPRODUCE_VIOPRODUCE_H_
#define INCLUDE_VIOPRODUCE_VIOPRODUCE_H_
#include <atomic>
#include <cstddef>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "json/json.h"

#include "vioplugin/viomessage.h"

#define kHorizonVisionXppOffset 1200

namespace horizon {
namespace vision {
namespace xpluginflow {
namespace vioplugin {

enum HorizonVioError {
  kHorizonNoInternalFree = -kHorizonVisionXppOffset - 1,
  kHorizonVioErrorAlreadyStart = -kHorizonVisionXppOffset - 2,
  kHorizonVioErrorNotStart = -kHorizonVisionXppOffset - 3,
  kHorizonVioErrorModeSwitch = -kHorizonVisionXppOffset - 5,
  kHorizonVioErrorTooShortCharArray = -kHorizonVisionXppOffset - 6,
};

class VioConfig {
 public:
  VioConfig() = default;
  explicit VioConfig(const std::string &path, const Json::Value &json)
      : path_(path), json_(json) {}
  std::string GetValue(const std::string &key) const;
  Json::Value GetJson() const;
  static VioConfig *GetConfig();
  bool SetConfig(VioConfig *config);

 private:
  std::string path_;
  Json::Value json_;
  static VioConfig *config_;
  mutable std::mutex mutex_;
};

class VioProduce : public std::enable_shared_from_this<VioProduce> {
 public:
  //根据data_source不同创建不同的图像对象，/camera/jpeg/nv12等
  static std::shared_ptr<VioProduce> CreateVioProduce(
      const std::string &data_source);
  virtual ~VioProduce() {}

  // called by vioplugin, add a run job to executor, async invoke, thread pool
  int Start();
  int Stop();
  // produce inputs, subclass must be implement
  virtual int Run() = 0;
  // finish producing inputs，common use
  virtual int Finish();
  // set VioConfig
  int SetConfig(VioConfig *config);
  // viomessage can be base class
  using Listener = std::function<int(const std::shared_ptr<VioMessage> &input)>;
  // set callback function
  int SetListener(const Listener &callback);
  // static member function
  static int PadImage(HorizonVisionImage *img, uint32_t dst_width = 1920,
                      uint32_t dst_height = 1080);
  // feedback path
  HorizonVisionImageFrame *GetImageFrame(const std::string &path);
  std::shared_ptr<ImageVioMessage> Image2ImageMessageInput(
      const HorizonVisionImage *image);
  virtual void FreeBuffer();
  bool AllocBuffer();
  virtual void WaitUntilAllDone();

 protected:
  VioProduce() : is_running_{false} {
    auto config = VioConfig::GetConfig();
    auto json = config->GetJson();
    max_vio_buffer_ = json["max_vio_buffer"].asUInt();
    std::atomic_init(&consumed_vio_buffers_, 0);
  }
  explicit VioProduce(int max_vio_buffer) : is_running_{false} {
    max_vio_buffer_ = max_vio_buffer;
    std::atomic_init(&consumed_vio_buffers_, 0);
  }

 protected:
  VioConfig *config_ = nullptr;
  std::function<int(const std::shared_ptr<VioMessage> &input)> push_data_cb_ =
      nullptr;
  std::atomic<bool> is_running_;
  std::atomic<int> consumed_vio_buffers_;
  int max_vio_buffer_ = 0;
  std::future<bool> task_future_;
  std::string cam_type_ = "mono";
  enum class TSTYPE {
    RAW_TS,  // 读取pvio_image->timestamp
    FRAME_ID,  // 读取pvio_image->frame_id
    INPUT_CODED  // 解析金字塔0层y图的前16个字节，其中编码了timestamp。
  };
  TSTYPE ts_type_ = TSTYPE::RAW_TS;
  static const
  std::unordered_map<std::string, TSTYPE> str2ts_type_;
};

class VioCamera : public VioProduce {
 public:
  int Run() override;

 protected:
  VioCamera() = default;
  virtual ~VioCamera() = default;

 protected:
  uint32_t sample_freq_ = 1;

 private:
  int read_time_stamp(void *addr, uint64_t *timestamp);
};

class PanelCamera : public VioCamera {
 public:
  explicit PanelCamera(const std::string &vio_cfg_file);
  virtual ~PanelCamera();

 private:
  int camera_index_ = -1;
};

class IpcCamera : public VioCamera {
 public:
  explicit IpcCamera(const std::string &vio_cfg_file);
  virtual ~IpcCamera();
};

class RawImage : public VioProduce {
 public:
  RawImage() = delete;
  explicit RawImage(const char *vio_cfg_file);
  static int FillVIOImageByVisionImage(img_info_t *pvio_image,
                                       const HorizonVisionImage *image);
  virtual ~RawImage();
  int Run() override;
};

class ImageList : public VioProduce {
 public:
  ImageList() = delete;
  explicit ImageList(const char *vio_cfg_file);
  template <typename T>
  bool FillVIOImageByImagePath(T *pvio_image, const std::string &img_name);
  virtual ~ImageList();
  int Run() override;
};

class JpegImageList : public ImageList {
 public:
  JpegImageList() = delete;
  explicit JpegImageList(const char *vio_cfg_file) : ImageList(vio_cfg_file) {}
  virtual ~JpegImageList() {}
};

class Nv12ImageList : public ImageList {
 public:
  Nv12ImageList() = delete;
  explicit Nv12ImageList(const char *vio_cfg_file) : ImageList(vio_cfg_file) {}
  virtual ~Nv12ImageList() {}
};

}  // namespace vioplugin
}  // namespace xpluginflow
}  // namespace vision
}  // namespace horizon
#endif  // XPLUGINFLOW_INCLUDE_XPLUGINFLOW_PLUGIN_XPLUGIN_H_
