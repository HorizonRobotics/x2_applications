/*
 * @Description: tracking algorithm implementation
 * @Author: chang.huang@horizon.ai
 * @Date: 2016-10-8 16:00:20
 * @Author: chao01.yang@horizon.ai
 * @Date: 2019-12-14 16:00:20
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-12-17 12:07:49
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */

#ifndef __TRACKING_H__
#define __TRACKING_H__

#include <stdint.h>

#include "iou_tracking/tracking_data_types.h"

void tracking_data_alloc(tracking_module_data_t *data);

void tracking_data_free(tracking_module_data *data);

int multi_track(tracking_module_data *data, box_s *box, int box_num,
                uint32_t time_gap);

int multi_track_notarget(tracking_module_data *data, int model_id,
                         uint32_t time_gap);

#endif
