
/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     Method interface of xsoul framework
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.22
 */

#include <stdio.h>

#include "hobotxroc/c_data_types/bbox.h"
#include "hobotxsdk/xroc_capi.h"
#include "hobotxsdk/xroc_error.h"

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    printf("Usage : ./c_bbox_filter_main config\n");
    printf("Example : ./c_bbox_filter_main ./config/filter.json\n");
    return -1;
  }
  const char *config = argv[1];
  printf("sdk version: %s\n", HobotXRocCapiGetVersion());
  HobotXRocCapiSetLicensePath("../config/");
  printf("sdk license info: %s\n", HobotXRocCapiGetLicenseInfo());

  HobotXRocCapiSetGlobalConfig("config_file", config);
  // 初始化sdk
  HobotXRocCapiHandle handle;
  int ret = HobotXRocCapiInit(&handle);
  if (ret != HOBOTXROC_ERROR_CODE_OK) {
    return ret;
  }
  // 准备inputs
  HobotXRocCapiInputList *inputs = HobotXRocCapiDataListAlloc(1);
  // 填充BBox
  HobotXRocCapiBaseDataVector *rects = HobotXRocCapiBaseDataVectorAlloc(2);
  rects->parent_.name_ = "face_head_box";
  inputs->datas_[0] = &(rects->parent_);
  HobotXRocCapiBBox *bbox1 = HobotXRocCapiBBoxAlloc();
  bbox1->values_[0] = 0;
  bbox1->values_[1] = 0;
  bbox1->values_[2] = 1000;
  bbox1->values_[3] = 1000;
  rects->datas_->datas_[0] = &(bbox1->parent_);
  HobotXRocCapiBBox *bbox2 = HobotXRocCapiBBoxAlloc();
  bbox2->values_[0] = 0;
  bbox2->values_[1] = 0;
  bbox2->values_[2] = 10;
  bbox2->values_[3] = 10;
  rects->datas_->datas_[1] = &(bbox2->parent_);
  // Hobot
  // 送给sdk计算
  HobotXRocCapiDataList *outputs = NULL;
  ret = HobotXRocCapiProcessSync(handle, inputs, &outputs);
  if (ret == HOBOTXROC_ERROR_CODE_OK && outputs != NULL) {
    // 解析sdk的结果
    if (outputs && 1 == outputs->datas_size_) {
      HobotXRocCapiDataList *rect_list =
          ((HobotXRocCapiBaseDataVector *) (outputs->datas_[0]))->datas_;
      // 解析BBox
      int i;
      for (i = 0; i < rect_list->datas_size_; ++i) {
        HobotXRocCapiBBox *bbox = (HobotXRocCapiBBox *) (rect_list->datas_[i]);
        printf("%d : [%f, %f, %f, %f]\n",
               i,
               bbox->values_[0],
               bbox->values_[1],
               bbox->values_[2],
               bbox->values_[3]);
      }
    } else {
      puts("Error: outputs data size check failed");
    }
    // 释放sdk生成的结果
    HobotXRocCapiDataListFree(&outputs);
  } else {
    printf("HobotXRocCapiProcessSync error: %d\n", ret);
  }
  // 释放sdk
  HobotXRocCapiDataListFree(&inputs);
  HobotXRocCapiFinalize(handle);
  return 0;
}
