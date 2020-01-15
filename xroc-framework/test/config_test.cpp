/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: ronghui zhang
 * @Mail: zhangronghui@horizon.ai
 * @Date: 2019-11-30 01:15:22
 * @Version: v0.0.1
 * @Brief: test thread safe
 */

#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <random>
#include <set>
#include "hobotlog/hobotlog.hpp"
#include "hobotxroc/xroc_config.h"
#include "hobotxsdk/xroc_error.h"
#include "hobotxsdk/xroc_sdk.h"

TEST(ConfigTest, input)
{
    LOGD << "test input" << std::endl;
    HobotXRoc::XRocSDK *flow = HobotXRoc::XRocSDK::CreateSDK();
    flow->SetConfig("config_file", "./test/configs/config_input.json");
    EXPECT_EQ(HobotXRoc::INPUT_UNFEED_ERROR, flow->Init());
}

TEST(ConfigTest, flow_input)
{
    LOGD << "test flow input" << std::endl;
    HobotXRoc::XRocSDK *flow = HobotXRoc::XRocSDK::CreateSDK();
    flow->SetConfig("config_file", "./test/configs/config_flow_input.json");
    EXPECT_EQ(HobotXRoc::INPUT_UNFEED_ERROR, flow->Init());
}

TEST(ConfigTest, input_error)
{
    LOGD << "test input error" << std::endl;
    HobotXRoc::XRocSDK *flow = HobotXRoc::XRocSDK::CreateSDK();
    flow->SetConfig("config_file", "./test/configs/config_input_error.json");
    EXPECT_EQ(HobotXRoc::INPUT_UNFEED_ERROR, flow->Init());
}

TEST(ConfigTest, nodeName)
{
    LOGD << "test nodeName" << std::endl;
    HobotXRoc::XRocSDK *flow = HobotXRoc::XRocSDK::CreateSDK();
    flow->SetConfig("config_file", "./test/configs/config_nodeName.json");
    EXPECT_EQ(HobotXRoc::NODE_NAME_ERROR, flow->Init());
}

TEST(ConfigTest, isCircle)
{
    LOGD << "test circle" << std::endl;
    HobotXRoc::XRocSDK *flow = HobotXRoc::XRocSDK::CreateSDK();
    flow->SetConfig("config_file", "./test/configs/config_circle.json");
    EXPECT_EQ(HobotXRoc::WORKFLOW_CIRCLE_ERROR, flow->Init());
}

TEST(ConfigTest, isRepeatedOutput)
{
    LOGD << "test repeated output" << std::endl;
    HobotXRoc::XRocSDK *flow = HobotXRoc::XRocSDK::CreateSDK();
    flow->SetConfig("config_file",
    "./test/configs/config_repeated_output.json");
    EXPECT_EQ(HobotXRoc::OUTPUT_REPEATED_ERROR, flow->Init());
}

TEST(ConfigTest, isOK)
{
    LOGD << "test correct workflow" << std::endl;
    HobotXRoc::XRocSDK *flow = HobotXRoc::XRocSDK::CreateSDK();
    flow->SetConfig("config_file", "./test/configs/config_ok.json");
    EXPECT_EQ(HobotXRoc::CONFIG_OK, flow->Init());
}
