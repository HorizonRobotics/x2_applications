/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: face_id_verification.cpp
 * @Brief: Identify the nv12 images after affine
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-05-07 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-05-07 15:18:10
 */

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include "3rd_party_lib/plat_cnn.h"
#include "CNNMethod/util/ModelInfo.h"
#include "CNNMethod/util/util.h"
#include "bpu_predict/bpu_io.h"
#include "bpu_predict/bpu_predict.h"
#include "hb_vio_interface.h"
#include "hbdk/hbdk_layout.h"
#include "hobotlog/hobotlog.hpp"

static void Usage() {
  std::cout << "./example ver_feature model_file bpu_config nv12_after_affine_list.txt out_file\n";
}

int DoVerFeature(int argc, char **argv) {
  if (argc < 5) {
    Usage();
    return -1;
  }
  constexpr int img_len = 112 * 112 * 3 / 2;
  auto nv12_data = new uint8_t[img_len];  // NV12 after affine!!!
  std::string model_file = argv[1];
  std::string bpu_config = argv[2];

  BPUHandle bpu_handle = nullptr;
  int ret = BPU_loadModel(model_file.c_str(), &bpu_handle, bpu_config.c_str());
  HOBOT_CHECK(ret == 0) << "BPU_loadModel failed";

  BPUModelInfo output_model_info, input_model_info;
  BPU_getModelOutputInfo(bpu_handle, "faceID", &output_model_info);
  BPU_getModelInputInfo(bpu_handle, "faceID", &input_model_info);

  HobotXRoc::ModelInfo model_info;
  model_info.Init(bpu_handle, "faceID", &input_model_info, &output_model_info);

  // alloc bpu_buffer
  std::vector<BPU_Buffer_Handle> out_bufs;
  for (auto n : model_info.output_layer_size_) {
    void *output_data = malloc(n);
    BPU_Buffer_Handle out_handle = BPU_createBPUBuffer(output_data, n);
    out_bufs.push_back(out_handle);
  }
  BPUFakeImageHandle fake_img_handle;
  ret = BPU_createFakeImageHandle(112, 112, &fake_img_handle);
  HOBOT_CHECK(ret == 0) << "create fake image handle failed";

  std::string img_nv12_list = argv[3];
  std::string output_file = argv[4];
  std::ofstream output(output_file, std::ios::out);
  std::ifstream img_list_file(img_nv12_list);
  std::string img_path;
  while (getline(img_list_file, img_path)) {
    std::ifstream input(img_path, std::ios::in | std::ios::binary);
    input.read(reinterpret_cast<char *>(nv12_data), img_len);

    BPUFakeImage *fake_img_ptr = nullptr;
    fake_img_ptr = BPU_getFakeImage(fake_img_handle, nv12_data, img_len);
    HOBOT_CHECK(fake_img_ptr != nullptr) << "get fake image failed";

    BPUModelHandle model_handle;
    ret = BPU_runModelFromImage(bpu_handle,
                                "faceID",
                                fake_img_ptr,
                                out_bufs.data(),
                                out_bufs.size(),
                                &model_handle);
    if (ret != 0) {
      LOGE << "BPU_runModelFromImage failed:" << BPU_getLastError(bpu_handle);
      return -1;
    }
    BPU_getModelOutput(bpu_handle, model_handle);
    BPU_releaseFakeImage(fake_img_handle, fake_img_ptr);
    BPU_releaseModelHandle(bpu_handle, model_handle);

    int layer_num = model_info.output_layer_size_.size();
    std::vector<std::vector<int8_t>> mxnet_outputs(layer_num);
    for (int j = 0; j < layer_num; j++) {
      mxnet_outputs[j].resize(model_info.output_layer_size_[j]);
      hbrtConvertLayout(mxnet_outputs[j].data(), LAYOUT_NHWC_NATIVE,
                 BPU_getRawBufferPtr(out_bufs[j]), model_info.layout_flag_[j],
                 model_info.element_type_[j], model_info.align_dim_[j],
                 model_info.convert_endianness_[j]);
    }
    // post
    static const int kFeatureCnt = 128;
    std::vector<float> post_rlt(kFeatureCnt);
    auto mxnet_rlt = reinterpret_cast<int32_t *>(mxnet_outputs[0].data());
    for (int i = 0; i < kFeatureCnt; i++) {
      post_rlt[i] =
          mxnet_rlt[i] / static_cast<float>(1 << model_info.all_shift_[0][i]);
    }
    // HobotXRoc::l2_norm(post_rlt, kFeatureCnt);
    for (const auto &mx_value : post_rlt) {
      output << std::fixed << std::setprecision(5) << mx_value << " ";
    }
    output << std::endl;
  }

  // release
  BPU_releaseFakeImageHandle(fake_img_handle);
  for (auto &out_buf : out_bufs) {
    void *out_data = BPU_getRawBufferPtr(out_buf);
    free(out_data);
    BPU_freeBPUBuffer(out_buf);
  }
  BPU_release(bpu_handle);
  delete[] nv12_data;
  return 0;
}
