/*
 * @Description: UT
 * @Author: fei.cheng@horizon.ai
 * @Date: 2019-11-05 19:40:56
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-11-05 19:41:44
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */
#include <sys/utsname.h>
#include "gtest/gtest.h"
#include "hobotlog/hobotlog.hpp"
#include "vioplugin/vioplugin.h"

using namespace horizon::vision::xpluginflow::vioplugin;
using VioPluginPtr = std::shared_ptr<VioPlugin>;

class VioPluginTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  void TearDown() override {}

  VioPluginPtr vioplugin = nullptr;
  std::mutex g_mtx_;
};

TEST_F(VioPluginTest, vio_camera) {
  int ret;
  SetLogLevel(INFO);

  vioplugin = std::make_shared<VioPlugin>("configs/ipc_camera.json");
  if (vioplugin == NULL) {
    std::cout << "vioplugin instance create failed" << std::endl;
    return;
  }

  ret = vioplugin->Init();
  EXPECT_EQ(ret, 0);

  ret = vioplugin->Start();
  EXPECT_EQ(ret, 0);

  sleep(600);

  ret = vioplugin->Stop();
  EXPECT_EQ(ret, 0);
  vioplugin.reset();
}

TEST_F(VioPluginTest, vio_nv12) {
  int ret;
  SetLogLevel(INFO);

  vioplugin = std::make_shared<VioPlugin>("configs/nv12_image.json");
  if (vioplugin == NULL) {
    std::cout << "vioplugin instance create failed" << std::endl;
    return;
  }

  ret = vioplugin->Init();
  EXPECT_EQ(ret, 0);

  ret = vioplugin->Start();
  EXPECT_EQ(ret, 0);

  sleep(600);

  ret = vioplugin->Stop();
  EXPECT_EQ(ret, 0);
  vioplugin.reset();
}

TEST_F(VioPluginTest, vio_jpeg) {
  int ret;
  SetLogLevel(INFO);

  vioplugin = std::make_shared<VioPlugin>("configs/jpeg_image.json");
  if (vioplugin == NULL) {
    std::cout << "vioplugin instance create failed" << std::endl;
    return;
  }

  ret = vioplugin->Init();
  EXPECT_EQ(ret, 0);

  ret = vioplugin->Start();
  EXPECT_EQ(ret, 0);

  sleep(600);

  ret = vioplugin->Stop();
  EXPECT_EQ(ret, 0);
  vioplugin.reset();
}