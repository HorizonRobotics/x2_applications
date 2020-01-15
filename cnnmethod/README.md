# cnnmethod

## Intro

cnn预测Method。目前支持人脸特征、3dpose、lmk、双目活体、人脸质量、年龄、性别等模型。

## Build

目前repo支持aarch32和aarch64

```shell
# cp build.properties.local.aarch32 build.properties.local
# mkdir build && cd build
# cmake .. release
```

## Usage
### Example
example的编译和运行：

```shell
# sh cicd/build_linux.sh
# scp -r build/bin username@x2_pad_ip:/run/path
# ssh username@x2_pad_ip
# cd /run/path
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./
# ./CNNMethod_example get_model_info config/models/PanelBoard.hbm config/configs/bpu_config.json
# ./CNNMethod_example do_fb_det_cnn pose_lmk config/det_cnn_pose_lmk.json config/vio_config/vio_onsemi0230_fb.json data/det_cnn/det_cnn_list.txt data/det_cnn/det_cnn_out.txt
# ./CNNMethod_example do_fb_rect_cnn anti_spf  config/anti_spf.json config/vio_config/vio_onsemi0230_fb.json data/rect_cnn/anti_spf/img_lst.txt data/rect_cnn/anti_spf/anti_spf_out.txt
```

### 输入/输出

#### 输入

| slot | 内容    | 备注               |
| ---- | ------- | ------------------ |
| 0    | rois    | 需要做预测的检测框 |
| 1    | pyramid | 图像的金字塔数据   |

#### 输出

活体模型

| slot | 内容     | 备注                       |
| ---- | -------- | -------------------------- |
| 0    | anti_spf | 活体值，包括value和score   |
| 1    | norm_roi | 经过norm_method处理后的roi |

人脸特征值

| slot | 内容         | 备注       |
| ---- | ------------ | ---------- |
| 0    | face_feature | 人脸特征值 |

人脸质量（详细说明见：http://wiki.hobot.cc/pages/viewpage.action?pageId=73945188）

| slot | 内容              | 备注           |
| ---- | ----------------- | -------------- |
| 0    | Blur              | 清晰度         |
| 1    | Brightness        | 亮度           |
| 2    | Eye_Abnormalities | 眼睛表情       |
| 3    | Mouth_Abnormal    | 嘴部表情       |
| 4    | Left_Eye          | 左眼可见区域   |
| 5    | Right_Eye         | 右眼可见区域   |
| 6    | Left_Brow         | 左眉毛可见区域 |
| 7    | Right_Brow        | 右眉毛可见区域 |
| 8    | ForeHead          | 额头可见区域   |
| 9    | Left_Cheek        | 左脸颊可见区域 |
| 10   | Right_Cheek       | 右脸颊可见区域 |
| 11   | Nose              | 鼻子可见区域   |
| 12   | Mouth             | 嘴部可见区域   |
| 13   | Jaw               | 下巴可见区域   |

pose+lmk

| slot | 内容 | 备注 |
| ---- | ---- | ---- |
| 0    | lmk  |      |
| 1    | pose |      |

### 配置文件

```json
{
  "model_name": "faceAntiSpfRGB",
  "model_version": "x2.1.0.11",
  "model_file_path": "../models/PanelBoard.hbm",
  "bpu_config_path": "../bpu_config/bpu_config.json",
  "in_msg_type": "img",
  "norm_method": "norm_by_lside_square",
  "filter_method": "no_filter",
  "expand_scale": 1.5,
  "post_fn": "antispoofing",
  "threshold": 0.1,
  "max_handle_num": -1,
  "output_size": 2
}
```

| 配置名          | 说明                                 | 备注                                                         |
| --------------- | ------------------------------------ | ------------------------------------------------------------ |
| model_name      | 编译模型时指定的模型名字             |                                                              |
| model_version   | 模型版本号                           |                                                              |
| model_file_path | 模型文件地址                         |                                                              |
| bpu_config_path | bpu的配置文件地址                    |                                                              |
| in_msg_type     | 模型的处理方式（resizer或者pyramid） | rect/img(resizer/pyramid)                                    |
| norm_method     | pyramid方式必填                      | norm_by_width_length<br />norm_by_width_ratio<br />norm_by_height_rario<br />norm_by_lside_ratio<br />norm_by_height_length<br />norm_by_lside_length<br />norm_by_lside_square<br />norm_by_diagonal_square<br />norm_by_nothing |
| filter_method   | pyramid方式必填                      | out_of_range<br />no_filter                                  |
| expand_scale    | pyramid方式必填                      | 外扩系数                                                     |
| post_fn         | 后处理方式                           | face_feature<br />antispoofing<br />lmk_pose<br />age_gender<br />face_quality |
| threshold       | 阈值                                 |                                                              |
| max_handle_num       | 最大处理数量             |    负数表示无限制                                                 |
| output_size     | 输出槽的个数                         |                                                              |

