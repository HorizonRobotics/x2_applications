/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: example_do_pyramid.cpp
 * @Brief:
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-04-15 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-04-15 15:18:10
 */
#include <fstream>
#include <iostream>
#include <string>
#include "bpu_predict/bpu_io.h"
#include "bpu_predict/bpu_predict.h"

int DoPyramid(int argc, char **argv) {
  const char *vio_config = "./config/hb_vio.json";
  std::string camera_cfg_file(argv[1]);
  int camera_id = atoi(argv[2]);

  // do pyramid
  BPUPyramidHandle pyr_handle;
  int ret = BPU_createPyramid(
      vio_config, camera_cfg_file.c_str(), camera_id, &pyr_handle);
  if (ret != 0) {
    std::cout << "create pyramid handle failed." << std::endl;
    return -1;
  }
  std::cout << "create pyramid done." << std::endl;

  BPUPyramidBuffer pyr_buffer;
  ret = BPU_getPyramidResult(pyr_handle, &pyr_buffer);
  if (ret != 0) {
    std::cout << "get pyramid result failed" << std::endl;
    return -1;
  }

  BPU_printPyramidBufferInfo(pyr_buffer);
  BPU_releasePyramidResult(pyr_handle, pyr_buffer);
  std::cout << "dump all pyramid image to file done." << std::endl;
  BPU_releasePyramid(pyr_handle);
  return 0;
}
