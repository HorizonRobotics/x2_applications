//
// Created by yaoyao.sun on 2019-08-13.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#ifndef EXAMPLE_INPUT_UTIL_VIO_H_
#define EXAMPLE_INPUT_UTIL_VIO_H_

#include <string>

#include "./hb_vio_common.h"
#include "./hb_vio_interface.h"

namespace vio_debug {
inline void print_info(const img_info_t &data) {
  printf("data slot_id = %d\n", data.slot_id);
  printf("data frame_id = %d\n", data.frame_id);
  // printf("data1 timestamp = %llx\n", data.timestamp);
  // printf("data2 timestamp = %lld\n", data.timestamp);
  printf("data img_format = %d\n", data.img_format);
  printf("data ds_pym_layer = %d\n", data.ds_pym_layer);
  printf("data us_pym_layer = %d\n", data.us_pym_layer);
  printf("s w = %d\n", data.src_img.width);
  printf("s h = %d\n", data.src_img.height);
  printf("s s = %d\n", data.src_img.step);
  printf("s y_p = %p\n", reinterpret_cast<uint8_t *>(data.src_img.y_paddr));
  printf("s c_p = %p\n", reinterpret_cast<uint8_t *>(data.src_img.c_paddr));
  printf("s y_v = %p\n", reinterpret_cast<uint8_t *>(data.src_img.y_vaddr));
  printf("s c_v = %p\n", reinterpret_cast<uint8_t *>(data.src_img.c_vaddr));
  int i;
  for (i = 0; i < data.ds_pym_layer; i++) {
    printf("ds[%d] w = %d\n", i, data.down_scale[i].width);
    printf("ds[%d] h = %d\n", i, data.down_scale[i].height);
    printf("ds[%d] s = %d\n", i, data.down_scale[i].step);
    printf("ds[%d] y_p = %p\n", i,
           reinterpret_cast<uint8_t *>(data.down_scale[i].y_paddr));
    printf("ds[%d] c_p = %p\n", i,
           reinterpret_cast<uint8_t *>(data.down_scale[i].c_paddr));
    printf("ds[%d] y_v = %p\n", i,
           reinterpret_cast<uint8_t *>(data.down_scale[i].y_vaddr));
    printf("ds[%d] c_v = %p\n", i,
           reinterpret_cast<uint8_t *>(data.down_scale[i].c_vaddr));
  }
}
}  // namespace vio_debug

class VIO {
 public:
  int Init(const std::string &vio_config_file) {
    return hb_vio_init(vio_config_file.c_str());
  }
  int Start() { return hb_vio_start(); }
  int GetImage(img_info_t *data) {
    auto ret = hb_vio_get_info(HB_VIO_PYM_INFO, data);
    if (ret < 0) {
      std::cout << "get image failed!!!\n";
      return -1;
    }
    return 0;
  }
  int GetMultiImage(mult_img_info_t *data) {
    auto ret = hb_vio_get_info(HB_VIO_PYM_MULT_INFO, data);
    if (ret < 0) {
      std::cout << "get multi images failed!!!\n";
      return -1;
    }
    return 0;
  }
  int GetFBImage(img_info_t *data, uint8_t *nv12_data, size_t nv12_data_len) {
    src_img_info_t src_data;
    auto ret = hb_vio_get_info(HB_VIO_FEEDBACK_SRC_INFO, &src_data);
    if (ret < 0) {
      std::cout << "get fb src failed!!!\n";
      return -1;
    }
    memcpy(reinterpret_cast<uint8_t *>(src_data.src_img.y_vaddr), nv12_data,
           nv12_data_len);

    /* flush ddr */
    hb_vio_set_info(HB_VIO_FEEDBACK_FLUSH, &src_data);
    /* 处理灌图片 */
    ret = hb_vio_pym_process(&src_data);
    if (ret < 0) {
      std::cout << "fb process failed!!!" << std::endl;
      return -1;
    }
    /* 得到灌图片 */
    ret = hb_vio_get_info(HB_VIO_PYM_INFO, data);
    if (ret < 0) {
      std::cout << "get fb image failed!!!" << std::endl;
      return -1;
    }
    return 0;
  }
  // return value, 0 success, others failed.
  int Free(img_info_t *data) { return hb_vio_free(data); }
  // return value, 0 success, others failed.
  int Free(mult_img_info_t *data) { return hb_vio_mult_free(data); }
  void Stop() {
    hb_vio_stop();
    hb_vio_deinit();
  }
  ~VIO() {}
};

#endif  // EXAMPLE_INPUT_UTIL_VIO_H_
