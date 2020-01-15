# FaceSnapFilterMethod

过滤Method，用于检测框的过滤，未通过过滤条件的bbox会被标记为FIlTERED状态。
具体介绍可参考：http://wiki.hobot.cc/display/ICSPDT/03_FaceSnapFilterMethod

# 输入

|Slot |内容 |备注 |
|:---:|:---------------:|:--------------:|
0 | XRocBBox_list | 必要项
1 | XRocPose3D_list | 可选项
2 | XRocLandmarks_list | 可选项
3 | blur | 可选项
4 | brightness | 可选项
5 | eye_abnormalities | 可选项
6 | mouth_abnormal | 可选项
7 | left_eye | 可选项
8 | right_eye | 可选项
9 | left_brow | 可选项
10 | right_brow | 可选项
11 | forehead | 可选项
12 | left_cheek | 可选项
13 | right_cheek | 可选项
14 | nose | 可选项
15 | mouth | 可选项
16| jaw | 可选项


# 输出

|Slot |内容 |备注 |
|:---:|:--------------------:|:---------------------------:|
0 | filter_info | 过滤错误码
1 | XRocBBox_list | 必要项
2 | XRocPose3D_list | 可选项
3 | XRocLandmarks_list | 可选项
4 | blur | 可选项
5 | brightness | 可选项
6 | eye_abnormalities | 可选项
7 | mouth_abnormal | 可选项
8 | left_eye | 可选项
9 | right_eye | 可选项
10 | left_brow | 可选项
11 | right_brow | 可选项
12 | forehead | 可选项
13 | left_cheek | 可选项
14 | right_cheek | 可选项
15 | nose | 可选项
16 | mouth | 可选项
17| jaw | 可选项


# 补充说明
+ 内部无状态机
+ 该Method支持workflow多实例，method_info.is_thread_safe_ = true，method_info.is_need_reorder = false。

# Update History

|Date      | Ver. |Change Log |
|:-------:|:-----:|:----------:|
20191105 |N/A    | 初始版本 |

# 配置文件参数

|字段      |描述     |默认值     |
|:-----------------------:|:-----------------------------------------------------:|:------:|
image_width|视频帧宽度|1920
image_height|视频帧高度|1080
snap_size_thr|检测框大小阈值|72
frontal_pitch_thr|正侧椭球pitch阈值|30
frontal_yaw_thr|正侧椭球yaw阈值|40
frontal_roll_thr|正侧椭球roll阈值|0
pv_thr|人脸置信度阈值|0.98
quality_thr|清晰度阈值，越小越好|0.5
bound_thr_w|视频帧宽边界|10
bound_thr_h|视频帧高边界|10
black_area_iou_thr|黑名单区域iou阈值|0.5
black_area_list|黑名单区域，例如可配置为[[10, 10, 30, 30], [40, 40, 50, 50]]即为在两个黑名单区域被过滤|[]
max_box_counts|最大检测框数，设置为0不对检测框数目作过滤|0
brightness_min|亮度过滤最小值|0
brightness_max|亮度过滤最大值|4
left_eye_occluded_thr|左眼遮挡阈值，越小遮挡程度越轻|0.5
right_brow_occluded_thr|右眼遮挡阈值，越小遮挡程度越轻|0.5
forehead_occluded_thr|前额遮挡阈值，越小遮挡程度越轻|0.5
left_cheek_occluded_thr|左脸遮挡阈值，越小遮挡程度越轻|0.5
right_cheek_occluded_thr|右脸遮挡阈值，越小遮挡程度越轻|0.5
nose_occluded_thr|鼻子遮挡阈值，越小遮挡程度越轻|0.5
mouth_occluded_thr|嘴巴遮挡阈值，越小遮挡程度越轻|0.5
jaw_occluded_thr|下巴遮挡阈值，越小遮挡程度越轻|0.5
abnormal_thr|行为异常遮挡阈值，越小表示行为较为正常|0.5
expand_scale|外扩系数，用以过滤外扩出边界的检测框|1.0
norm_method|检测框归一化方式|"norm_by_lside_square"
err_description|错误码描述，对应字段设置为多少filter_info输出对应的错误码｜{
                                                       "passed": 0,
                                                       "snap_area": -1,
                                                       "snap_size_thr": -2,
                                                       "expand_thr": -3,
                                                       "frontal_thr": -4,
                                                       "pv_thr": -5,
                                                       "quality_thr": -6,
                                                       "lmk_thr": -7,
                                                       "black_list": -8,
                                                       "big_face": -9,
                                                       "brightness": -10,
                                                       "abnormal_thr": -11,
                                                       "left_eye_occluded_thr": -12,
                                                       "right_eye_occluded_thr": -13,
                                                       "left_brow_occluded_thr": -14,
                                                       "right_brow_occluded_thr": -15,
                                                       "forehead_occluded_thr": -16,
                                                       "left_cheek_occluded_thr": -17,
                                                       "right_cheek_occluded_thr": -18,
                                                       "nose_occluded_thr": -19,
                                                       "mouth_occluded_thr": -20,
                                                       "jaw_occluded_thr": -21
                                                     }
