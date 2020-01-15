/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: CNNMethodPredictor.cpp
 * @Brief: definition of the CNNMethodPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-04-15 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-05-06 16:23:27
 */
#include <iostream>
#include <map>
#include <string>
#include "hobotlog/hobotlog.hpp"

typedef int (*example_fn)(int, char **);

extern int GetModelInfo(int, char **);
extern int DoPyramid(int, char **);
extern int DoFbFeature(int argc, char **argv);
extern int DoVerFeature(int argc, char **argv);
extern int DoFB(int argc, char **argv);
extern int DoFbRectCnn(int argc, char **argv);
extern int DoFbDetCNN(int argc, char **argv);
extern int DoDet(int argc, char **argv);
extern int DoVerRectPyd(int argc, char **argv);
extern int DoFbImg(int argc, char **argv);
extern int DetFromCamera(int argc, char **argv);

int PrintUsage() {
  std::cout
      << "Usage: example [command] [args]\n"
      << "\n"
      << "Command:\n"
      << "         get_model_info model_file bpu_config\n"
      << "         do_pyramid\n"
      << "         do_fb_det_cnn [pose_lmk|age_gender|anti_spf] xroc_cfg_file "
         "fb_cfg img_list out_file\n"
      << "         do_fb_feature xroc_cfg_file img_lmk_list out_file\n"
      << "         do_fb_img xroc_cfg_file img_list out_file\n"
      << "         do_fb hb_cfg_file file_name\n"
      << "         do_fb_rect_cnn [pose_lmk|age_gender|anti_spf|face_quality] "
         "xroc_cfg_file fb_cfg img_list out_file\n"
      << "         do_det xroc_cfg_file fb_cfg img_list\n"
      << "         ver_feature model_file bpu_config "
         "nv12_after_affine_list.txt out_file\n"
      << "         ver_rect_pyd method_cfg_file hb_vio_cfg_file gt.txt "
         "out_file\n"
      << "         det_from_camera xroc_config vio_config_file "
         "camera_config_file output\n";
}

static std::map<std::string, example_fn> examples = {
    {"get_model_info", GetModelInfo},
    {"do_pyramid", DoPyramid},
    {"do_fb_feature", DoFbFeature},
    {"ver_feature", DoVerFeature},
    {"do_fb_det_cnn", DoFbDetCNN},
    {"do_fb", DoFB},
    {"do_fb_rect_cnn", DoFbRectCnn},
    {"do_det", DoDet},
    {"ver_rect_pyd", DoVerRectPyd},
    {"do_fb_img", DoFbImg},
    {"det_from_camera", DetFromCamera},
};

int main(int argc, char **argv) {
  SetLogLevel(HOBOT_LOG_DEBUG);
  if (argc > 1 && examples.count(argv[1])) {
    examples[argv[1]](argc - 1, argv + 1);
    std::cout << "main thread exit" << std::endl;
  } else {
    PrintUsage();
  }
  return 0;
}
