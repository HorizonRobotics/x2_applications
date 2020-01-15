# SnapshotMethod

抓拍Method，用于人脸优选、抠图。
具体介绍可参考：http://wiki.hobot.cc/display/ICSPDT/04_SnapshotMethod

# Update History

|Date      | Ver. |Change Log |
|:-------:|:-----:|:----------:|
20191012 |N/A    | 增加了抓拍上报类型，修改配置文件vanish_post_flag字段为report_flushed_track_flag，增加了上报非该帧目标的选项 |
20200106 |N/A    | 修复抓拍状态上报问题 |

# 输入

|Slot |内容 |备注 |
|:---:|:---------------------------:|:--------------:|
0 | img_frame | 必要项
1 | box_list | 必要项
2 | select_score_list | 必要项
3 | disappeared_track_id_list | 必要项
4 | userdata_list1 | 可选项
5 | userdata_list2 | 可选项
6 | userdata_list... | 可选项

从slot4开始，可输入userdata_list，要求list的数目要和slot1中box_list数目对齐，用以透传用户数据，位于snap_list中

# 输出

|Slot |内容 |备注 |
|:---:|:--------------------:|:---------------------------:|
0 | snap_list | 带track_id

# 补充说明
+ 内部有状态机来存储每个track的抓拍信息
+ 该Method支持workflow多实例，method_info.is_thread_safe_ = false，method_info.is_need_reorder = true。

# 配置文件参数

|字段      |描述     |默认值     |
|:-------:|:-----:|:----------:|
snapshot_type|抓拍工作模式，first_num_best为最优图抓拍，crop为抠图抓拍|first_num_best
scale_rate|外扩系数|1.6
need_resize|是否需要对抓拍图resize：置为true根据output_width和output_height输出外扩后的resize图像，置为false输出快扩后的原图|false
output_width|抓拍图输出宽度，仅当need_resize为true生效|128
output_height|抓拍图输出高度，仅当need_resize为true生效|128
update_steps|更新步长，新抓拍得分 - update_steps > 状态机抓拍图得分才更新状态机抓拍图|50
snaps_per_track|每个track抓拍张数|1
max_tracks|状态机保留最大track数|256
max_crop_num_per_frame|每帧最大抠图数|4
smoothing_frame_range|avg_crop_num_per_frame计算帧数|10
avg_crop_num_per_frame|平均每帧抠图数|2
begin_post_frame_thr|开始抓拍帧数阈值|1
reshape_value|重抓拍数，当reshape_value > begin_post_frame_thr才会开启重抓拍，默认关闭|0
save_original_image_frame|是否保持原始图像帧数据：置为true，抓拍图里的origin_image_frame会被赋值原始图像帧引用，置为false会重新构造一个未包含原始图像帧数据的ImageFrame|false
report_flushed_track_flag|是否在外部flush track时触发抓拍|true
out_date_target_post_flag|是否允许上报非该帧目标|false
repeat_post_flag|同一个track是否希望多次被不同触发条件上报|false

out_date_target_post_flag:
false: 只上报当前帧 && 通过过滤的框（面板机场景采用该方案）
true: 上报所有触发抓拍的目标，该目标可能在当前帧，但是被过滤掉（X1的IPC已经采用该方案）
