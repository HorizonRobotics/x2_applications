# VioPlugin 使用文档
## 介绍
VioPlugin负责获取、转换图像数据，并将图像数据或丢帧消息推送给消息总线。
如果VioPlugin内部没有为当前视频帧分配到buffer, 就会产生一个丢帧消息并推送到总线.   
图像可以通过摄像头或者回灌<sup>*1*</sup>的方式获取.

## 使用
### 配置文件
```json
{
  "cam_type": "mono",
  "data_source": "ipc_camera",
  "max_vio_buffer": 3,
  "ts_type": "input_coded",
  "file_path": "name.list",
  "pad_width": 1920,
  "pad_height": 1080,
  "vio_cfg_file": {
    "ipc_camera": "configs/vio/hb_vio.json",
    "panel_camera": "configs/vio/panel_camera.json",
    "jpeg_image_list": "configs/vio/vio_onsemi0230_fb.json",
    "nv12_image_list": "configs/vio/vio_onsemi0230_fb.json",
    "image": "configs/vio/vio_onsemi0230_fb.json"
  }
}
```
各个字段的含义:  
+ **cam_type** : 镜头类型，单目(mono)或双目(dual)
+ **data_source** : 输入源类型:
  + *ipc_camera* : IPC等后接场景，输入通常为bt1120
  + *panel_camera* : 面板机等前接场景，输入通常为mipi
  + *jpeg_image_list* : jpeg格式的回灌图片
  + *nv12_image_list* : nv12格式回灌图片
+ **max_vio_buffer** : 控制Vio pending帧数上限，最大缓存数量  
  如果递交给SmartPlugin处理的视频帧(Pending状态)大于该数量, 则新的视频帧会丢弃, 并产生丢帧消息
+ **ts_type** : vio时间戳类型. 该时间戳会填充到视频帧消息中
  + *input_coded* : 通过y图的前16个字节的编码获得时间戳，通常用于ipc等后接场景
  + *frame_id* : 读取vio数据结构的frame_id字段作为时间戳，96board等使用该配置
  + *raw_ts* : 读取vio数据结构中的timestamp字段作为时间戳，面板机standalone方案使用该类型
+ **file_path** : 回灌图片的name list
+ **pad_width** : jpeg回灌时图片对齐参数
+ **pad_height** : jpeg回灌时图片对齐参数
+ **vio_cfg_file** : 对应每种输入源的详细配置文件

### 生产消息类型
+ `XPLUGIN_IMAGE_MESSAGE` : 图像帧类型的消息. 
    ```c++
    // 图像帧消息对应的结构体
    struct ImageVioMessage : VioMessage {
        public:
        ImageVioMessage() = delete;
        explicit ImageVioMessage(HorizonVisionImageFrame **image_frame,
                                uint32_t img_num, bool is_valid = true,
                                mult_img_info_t *info = nullptr);
        ~ImageVioMessage(){};

        // serialize proto
        std::string Serialize() { return "No need serialize"; };

        void FreeImage();
    };
    ```
+ `XPLUGIN_DROP_MESSAGE` : 丢帧消息
    ```c++
    // 丢帧消息对应的结构体
    struct DropVioMessage : VioMessage {
        public:
        DropVioMessage() = delete;
        explicit DropVioMessage(uint64_t timestamp, uint64_t seq_id);
        ~DropVioMessage(){};

        // serialize proto
        std::string Serialize() override;
    };
    ```

视频帧消息和丢帧消息的共同继承于`VioMessage`结构体.
```c++
struct VioMessage : public XPluginFlowMessage {
 public:
  VioMessage() { type_ = TYPE_IMAGE_MESSAGE; };
  virtual ~VioMessage() = default;

  // image frames number
  uint32_t num_ = 0;
  // sequence id, would increment automatically
  uint64_t sequence_id_ = 0;
  // time stamp
  uint64_t time_stamp_ = 0;
  // is valid uri
  bool is_valid_uri_ = true;
  // image frames
  HorizonVisionImageFrame **image_ = nullptr;
  // free source image
  void FreeImage();
  // serialize proto
  std::string Serialize() override { return "Default vio message"; };
  // multi
  mult_img_info_t *multi_info_ = nullptr;
};
```

## 接口
### 构造函数
#### 定义
#include "hbipcplugin/hbipcplugin.h"

**VioPlugin::VioPlugin(const std::string &*path*);**

#### 参数
+ const std::string &*path*: 配置文件路径

#### 说明
构造VioPlugin类对象, 并指定该类对象使用的配置信息.

### 析构函数
#### 定义
#include "hbipcplugin/hbipcplugin.h"

**VioPlugin::~VioPlugin();**

#### 参数
无

#### 说明
析构VioPlugin类对象.

### 初始化Plugin
#### 定义
#include "vioplugin/vioplugin.h"

**int VioPlugin::Init() override;**

#### 参数
无

#### 返回值
+ 0: 成功
+ 非0: 失败

#### 说明
初始化Plugin.

### 启动Plugin
#### 定义
#include "vioplugin/vioplugin.h"

**int VioPlugin::Start() override;**

#### 参数
无

#### 返回值
+ 0: 成功
+ 非0: 失败

#### 说明
启动Plugin.

### 停止Plugin
#### 定义
#include "vioplugin/vioplugin.h"

**int VioPlugin::Stop() override;**

#### 参数
无

#### 返回值
+ 0: 成功
+ 非0: 失败

#### 说明
停止Plugin.

### 获得Plugin名称
#### 定义
#include "vioplugin/vioplugin.h"

**std::string VioPlugin::desc() const;**

#### 参数
无

#### 返回值
Plugin描述字符串

#### 说明
获取Plugin描述字符串.

----
#### 术语表
1. **回灌** : 从一组图片文件中获取视频帧的一种方式  
