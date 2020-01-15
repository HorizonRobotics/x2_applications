/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @author    yaoyao.sun
 * @date      2019.04.12
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>

#include <assert.h>
#include <stdlib.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <string>
#include <fstream>
#include <atomic>

#include "json/json.h"

#include "yuv_utils.h"
#include "FasterRCNNMethod/dump.h"
#include "hobotxsdk/xroc_sdk.h"
#include "opencv2/opencv.hpp"
#include "hobotlog/hobotlog.hpp"


#include "FasterRCNNMethod.h"
#include "horizon/vision_type/vision_type.hpp"

#include "./block_queue.h"


using HobotXRoc::BaseData;
using HobotXRoc::BaseDataPtr;
using HobotXRoc::BaseDataVector;
using HobotXRoc::InputData;
using HobotXRoc::InputDataPtr;
using HobotXRoc::XRocData;

using HobotXRoc::FasterRCNNMethod;
using hobot::vision::ImageFrame;
using hobot::vision::PymImageFrame;

static void Usage() {
  std::cout << "./FasterRCNNMethod_example two_faster_rcnn "
               "model_config_file vio_config_file image_list\n";
}

BlockQueue<img_info_t*> g_pyramid_img_queue;
std::atomic<long long> frame_cnt(0);

void ThreadFun(FasterRCNNMethod *faster_rcnn, std::string model_config_file) {
  faster_rcnn->Init(model_config_file);
  while(1) {
    img_info_t *pyramid_img_ptr = g_pyramid_img_queue.Take();

    auto py_image_frame_ptr = std::make_shared<PymImageFrame>();

    py_image_frame_ptr->img = *pyramid_img_ptr;

    auto xroc_pyramid = std::make_shared<XRocData<std::shared_ptr<ImageFrame>>>();
    xroc_pyramid->value = py_image_frame_ptr;

    std::vector<std::vector<BaseDataPtr>> input;
    std::vector<HobotXRoc::InputParamPtr> param;
    input.resize(1);
    input[0].push_back(xroc_pyramid);

    auto xroc_output = faster_rcnn->DoProcess(input, param);

    assert(xroc_output.size() == 1);
    auto faster_rcnn_out = xroc_output[0];
    delete pyramid_img_ptr;
    std::cout << "predict success: " << frame_cnt++ << std::endl;
  }

}

int TestTwoFasterRCNN(int argc, char **argv) {
  if (argc < 4) {
    Usage();
    return -1;
  }
  FasterRCNNMethod faster_rcnn_method1;
  FasterRCNNMethod faster_rcnn_method2;
  std::string model_config_file = argv[1];
  std::string vio_config_file = argv[2];
  std::string image_list = argv[3];

  std::thread t1(ThreadFun, &faster_rcnn_method1, model_config_file);
  std::thread t2(ThreadFun, &faster_rcnn_method2, model_config_file);

  std::cout << "fasterrcnn init success." << std::endl;

  int ret = 0;
  ret = hb_vio_init(vio_config_file.c_str());
  if (ret < 0) {
    std::cout << "vio init fail\n";
    return -1;
  }
  ret = hb_vio_start();
  if (ret < 0) {
    std::cout << "vio start fail\n";
    return -1;
  }

  std::ifstream ifs(image_list);
  if (!ifs.is_open()) {
    std::cout << "open image list file failed." << std::endl;
    return -1;
  }

  std::string input_image;
  src_img_info_t data;
  while (getline(ifs, input_image)) {
    /* 获取内存地址 */
    ret = hb_vio_get_info(HB_VIO_FEEDBACK_SRC_INFO, &data);
    if (ret < 0) {
      std::cout << "get fb src fail!!!" << std::endl;
      return -1;
    }

    /* 灌图片到内存 */
    cv::Mat bgr_img = cv::imread(input_image);
    int width = bgr_img.cols;
    int height = bgr_img.rows;
    cv::Mat img_nv12;
    bgr_to_nv12(bgr_img.data, height, width, img_nv12);
    memcpy(reinterpret_cast<uint8_t *>(data.src_img.y_vaddr), img_nv12.data,
           height * width * 3 / 2);

    /* 处理灌图片 */
    ret = hb_vio_pym_process(&data);
    if (ret < 0) {
      std::cout << "fb process fail!!!" << std::endl;
      return -1;
    }
    /* 得到灌图片 */
    img_info_t *fb_img = new img_info_t();
    ret = hb_vio_get_info(HB_VIO_PYM_INFO, fb_img);
    if (ret < 0) {
      std::cout << "hb_vio_get_info 2 err" << std::endl;
      return -1;
    }

    g_pyramid_img_queue.Push(fb_img);

    printf("down_scale[0] info: witdh:%d, height:%d, step:%d\n",
           fb_img->down_scale[0].width,
           fb_img->down_scale[0].height,
           fb_img->down_scale[0].step);



    /* 释放回灌buff */
    ret = hb_vio_free(fb_img);
    if (ret < 0) {
      std::cout << "hb_vio_free 2 err" << std::endl;
      return -1;
    }
  }
  t1.join();
  t2.join();
  hb_vio_stop();
  hb_vio_deinit();
  faster_rcnn_method1.Finalize();
  faster_rcnn_method2.Finalize();
  std::this_thread::sleep_for(std::chrono::seconds(10));
  return 0;
}
