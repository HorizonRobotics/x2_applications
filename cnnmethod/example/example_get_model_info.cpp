/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: example_get_model_info.cpp
 * @Brief:
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-04-15 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-04-15 15:18:10
 */
#include <stdint.h>
#include <cstddef>
#include <iostream>
#include <string>
#include <vector>
#include "CNNMethod/util/ModelInfo.h"
#include "bpu_predict/bpu_predict.h"

static void Usage() {
  std::cout << "./example get_model_info model_file bpu_config" << std::endl;
}
int GetModelInfo(int argc, char** argv) {
  if (argc < 3) {
    Usage();
    return -1;
  }
  std::string model_file = argv[1];
  std::string bpu_config = argv[2];
  BPUHandle bpu_handle;
  int ret = BPU_loadModel(model_file.c_str(), &bpu_handle, bpu_config.c_str());
  if (ret != 0) {
    std::cout << "here load bpu model failed" << std::endl;
    return 1;
  }
  std::cout << "here load bpu model OK" << std::endl;

  // get bpu version
  const char* version = BPU_getVersion(bpu_handle);
  if (version == nullptr) {
    std::cout << "here get bpu version failed: " << BPU_getLastError(bpu_handle)
              << std::endl;
    return 1;
  }
  std::cout << "here get bpu version: " << version << std::endl;

  // get model names
  const char** name_list;
  int name_cnt;
  ret = BPU_getModelNameList(bpu_handle, &name_list, &name_cnt);
  if (ret != 0) {
    std::cout << "here get name list failed: " << BPU_getLastError(bpu_handle)
              << std::endl;
    return 1;
  }

  std::vector<BPUModelInfo> model_output_infos(name_cnt);
  std::vector<BPUModelInfo> model_input_infos(name_cnt);
  for (int i = 0; i < name_cnt; i++) {
    BPU_getModelOutputInfo(bpu_handle, name_list[i], &model_output_infos[i]);
    BPU_getModelInputInfo(bpu_handle, name_list[i], &model_input_infos[i]);
    HobotXRoc::ModelInfo model_info;
    model_info.Init(
    bpu_handle, name_list[i], &model_input_infos[i], &model_output_infos[i]);
    std::cout << model_info;
  }

  return 0;
}
