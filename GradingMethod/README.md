# GradingMethod

优选打分Method，用于检测框的打分评价。
具体介绍可参考：http://wiki.hobot.cc/display/ICSPDT/05_GradingMethod

# 输入

|Slot |内容 |备注 |
|:---:|:--------------------:|:--------------:|
0 | XRocBBox_list | 必要项
1 | XRocPose3D_list | 必要项
2 | XRocLandmarks_list | 必要项
3 | XRocQualityBlur_list | 可选项

# 输出

|Slot |内容 |备注 |
|:---:|:--------------------:|:---------------------------:|
0 | select_score_list | int32_t的分数列表

# 补充说明
+ 内部无状态机
+ 该Method支持workflow多实例，method_info.is_thread_safe_ = true，method_info.is_need_reorder = false。

# Update History

|Date      | Ver. |Change Log |
|:-------:|:-----:|:----------:|
20191104 |N/A    | 初始版本 |

# 配置文件参数

|字段      |描述     |默认值     |
|:--------------------:|:-----------------------------------------------:|:------:|
grading_type|打分工作模式，目前仅支持weight打分|weight_grading
size_min|检测框最小尺寸|40
size_max|检测框最大尺寸|200
size_inflexion|size一次线性拐点，用以激励小尺寸检测框得分|80
size_weight|检测框大小权值|0.3
pose_weight|正侧权值|0.2
lmk_weight|关键点权值|0.3
quality_weight|质量权值|0.2
