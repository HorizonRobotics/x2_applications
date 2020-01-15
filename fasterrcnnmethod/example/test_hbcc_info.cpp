//
// Created by yaoyao.sun on 2019-05-18.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#include <iostream>
#include "hbdk/hbdk_hbrt.h"


int TestHBCCInfo(int argc, char **argv) {
  const char *model_file_path = "./models/faceMultitask.hbm";
  hbrt_hbm_handle_t hbm_handle;
  CHECK_HBRT_ERROR(hbrtLoadHBMFromFile(&hbm_handle, model_file_path));
  const char **model_names;
  CHECK_HBRT_ERROR(hbrtGetModelNamesInHBM(&model_names, hbm_handle));
  uint32_t model_num;
  CHECK_HBRT_ERROR(hbrtGetModelNumberInHBM(&model_num, hbm_handle));
  uint32_t i = 0;
  for (i = 0; i < model_num; i++) {
     std::cout << "model name:" << model_names[i] << std::endl;
  }
  CHECK_HBRT_ERROR(hbrtOffloadHBM(hbm_handle));
  return 0;
}

