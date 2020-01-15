/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @author    yaoyao.sun
 * @date      2019.04.12
 */

#include <chrono>
#include <iostream>
#include <thread>
#include <string>
#include <vector>

#include "bpu_predict/bpu_predict.h"
#include "bpu_predict/bpu_internal.h"
#include "hb_vio_interface.h"
#include "bpu_predict/bpu_io.h"
#include "3rd_party_lib/plat_cnn.h"
#include "hbdk/hbdk_hbrt.h"

static void Usage() {
  std::cout << "./FasterRCNNMethod_example model_info model_name ipc/panel\n";
}

int TestModelInfo(int argc, char **argv) {
  if (argc < 3) {
    Usage();
    return -1;
  }
  std::cout << "core num: " << cnn_core_num() << std::endl;
  std::string model = argv[2];
  std::string model_file_path;
  if (model == "ipc") {
    model_file_path = "./models/IPCModel.hbm";
  } else if (model == "panel") {
    model_file_path = "./models/faceMultitask.hbm";
  } else {
    std::cout << "not support this model " << model << "\n";
  }
  CHECK_HBRT_ERROR(hbrtIsCompatibleHeader());
  const char *bpu_config_path = "./configs/bpu_config.json";
  BPUHandle bpu_handle;
  int ret = BPU_loadModel(model_file_path.c_str(),
                          &bpu_handle, bpu_config_path);
  if (ret != 0) {
    std::cout << "here load bpu model failed: "
              << BPU_getLastError(bpu_handle) << std::endl;
    return 1;
  }
  std::cout << "here load bpu model OK" << std::endl;

  // get bpu version
  const char* version = BPU_getVersion(bpu_handle);
  if (version == nullptr) {
    std::cout << "here get bpu version failed: "
              << BPU_getLastError(bpu_handle) << std::endl;
    return 1;
  }
  std::cout << "here get bpu version: " << version << std::endl;

  // get model names
  const char** name_list;
  int name_cnt;
  ret = BPU_getModelNameList(bpu_handle, &name_list, &name_cnt);
  if (ret != 0) {
    std::cout << "here get name list failed: "
              << BPU_getLastError(bpu_handle) << std::endl;
    return 1;
  }

  // print all name list
  std::cout << "here get all name list: " << name_cnt
            << " list below: " << std::endl;
  for (int i = 0; i < name_cnt; ++i) {
    std::cout << name_list[i] << std::endl;
  }

  // const char* model_name = "personMultiTask";
  const char* model_name = argv[1];
  // get input info
  BPUModelInfo input_info;
  ret = BPU_getModelInputInfo(bpu_handle, model_name, &input_info);
  if (ret != 0) {
    std::cout << "here get model: " << model_name
              << " input info failed: " << BPU_getLastError(bpu_handle) << "\n";
    return 1;
  }

  // print input info
  std::cout << "model: " << model_name
            << " has input: " << input_info.num << "\n";
  for (int i = 0; i < input_info.num; ++i) {
    std::cout << "(";
    for (int j = input_info.ndim_array[i];
         j < input_info.ndim_array[i + 1]; ++j) {
      std::cout << input_info.valid_shape_array[j] << ",";
    }
    std::cout << ")" << std::endl;
  }

  // get output info
  BPUModelInfo output_info;
  ret = BPU_getModelOutputInfo(bpu_handle, model_name, &output_info);
  if (ret != 0) {
    std::cout << "here get model: " << model_name << " output info failed: "
              << BPU_getLastError(bpu_handle) << std::endl;
    return 1;
  }

  // print output info
  std::cout << "model: " << model_name << " has output: "
            << output_info.num << std::endl;

  hbrt_hbm_handle_t hbm_handle;
  ret = BPU_getHBMhandleFromBPUhandle(bpu_handle, &hbm_handle.handle);
  if (ret != 0) {
     std::cout << "Load bpu model failed: " << BPU_getLastError(bpu_handle);
     return 1;
  }
  hbrt_model_handle_t model_handle_;
  CHECK_HBRT_ERROR(hbrtGetModelHandle(&model_handle_, hbm_handle, model_name));
  uint32_t output_layer_num = 0;
  CHECK_HBRT_ERROR(hbrtGetOutputFeatureNumber(&output_layer_num,
                   model_handle_));
  const hbrt_feature_handle_t *feature_info;
  CHECK_HBRT_ERROR(hbrtGetOutputFeatureHandles(&feature_info, model_handle_));

  for (size_t i = 0; i < output_layer_num; ++i) {
    const uint8_t *shift_value;
    CHECK_HBRT_ERROR(hbrtGetFeatureShiftValues(&shift_value, feature_info[i]));
    hbrt_dimension_t aligned_dim;
    CHECK_HBRT_ERROR(hbrtGetFeatureAlignedDimension(
    &aligned_dim, feature_info[i]));
    hbrt_dimension_t valid_dim;
    CHECK_HBRT_ERROR(hbrtGetFeatureValidDimension(&valid_dim, feature_info[i]));
    const char *feature_name;
    CHECK_HBRT_ERROR(hbrtGetFeatureName(&feature_name, feature_info[i]));
    int alligned_channel_num = aligned_dim.c;
    int padding_channel_num = aligned_dim.c - valid_dim.c;
    std::cout << "layer num: " << i << " aligned channel num: "
              <<  alligned_channel_num << " padding channel num: "
              << padding_channel_num << "\n";
    std::cout << " shift: [";
    for (int j = 0; j < alligned_channel_num - padding_channel_num; ++j) {
      std::cout << static_cast<uint32_t>(shift_value[j]) << ",";
    }
    std::cout << "]\n";
    std::cout << "feature name: " << feature_name << std::endl;
  }
//  CHECK_HBRT_ERROR(hbrtOffloadHBM(hbm_handle));
  int out_dtype_size = 0;
  std::vector<BPU_Buffer_Handle> out_buf;
  for (int i = 0; i < output_info.num; ++i) {
    int model_out_size = 1;
    std::cout << "(";
    for (int j = output_info.ndim_array[i];
         j < output_info.ndim_array[i+1]; ++j) {
      std::cout << output_info.aligned_shape_array[j] << ",";
      model_out_size *= output_info.aligned_shape_array[j];
    }
    std::cout << ")" << std::endl;
    // get dtype
    if (output_info.dtype_array[i] == BPU_DTYPE_FLOAT32) {
      out_dtype_size = sizeof(float);
    } else {
      out_dtype_size = sizeof(int8_t);
    }

    void *output_data = malloc(model_out_size * out_dtype_size);
    BPU_Buffer_Handle out_handle =
        BPU_createBPUBuffer(output_data, model_out_size * out_dtype_size);

    out_buf.push_back(out_handle);
  }
  return 0;
}

