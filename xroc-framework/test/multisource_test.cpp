/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: guoqian.sun
 * @Mail: guoqian.sun@horizon.ai
 * @Date: 2019-12-05
 * @Version: v0.0.0
 * @Brief: Test Multi source input
 */

#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <random>
#include <fstream>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <vector>
#include <map>

#include "hobotlog/hobotlog.hpp"
#include "hobotxroc/data_types/orderdata.h"
#include "hobotxroc/data_types/filter_param.h"
#include "hobotxroc/xroc_config.h"
#include "hobotxsdk/xroc_error.h"
#include "hobotxsdk/xroc_sdk.h"
#include "method/bbox_filter.h"
#include "MultiSourceTestMethod.h"
#include "hobotxroc/json_key.h"


namespace MultisourceTest {
class Callback {
 public:
  Callback() {}

  ~Callback() {}

  void OnCallback(HobotXRoc::OutputDataPtr output) {
    using HobotXRoc::MulSrcTestOutput;
    using HobotXRoc::MulSrcTestOutputPtr;
    auto out_test =
      std::static_pointer_cast<MulSrcTestOutput>(output->datas_[0]);
    LOGD << "[MultisourceTest]:source_id "<< output->source_id_
         << " sequence_id " << output->sequence_id_
         << " frame_id " << out_test->value.frame_id_
         << " method_id " << std::dec << out_test->value.method_id_
         << " thread_hash " << std::hex << out_test->value.thread_hash_;
    resbuffer.push_back(out_test);
  }

  int ExamResult(HobotXRoc::MethodInfo methodinfo,
    size_t source_num, size_t thread_num) {
    std::unordered_set<size_t> method_list;
    std::unordered_set<size_t> thread_list;
    std::unordered_set<size_t> source_list;
    // std::vector<std::set<size_t>> framecheck;
    std::unordered_map<size_t, std::unordered_set<size_t>> thread_to_source;
    std::unordered_map<size_t, std::unordered_set<size_t>> thread_to_method;
    std::unordered_map<size_t, std::unordered_set<size_t>> source_to_method;
    std::unordered_set<size_t> templist;
    // framecheck.resize(source_num);
    // 处理收到的所有结果，并建立各种映射表
    for (auto outdata : resbuffer) {
      auto method_id = outdata->value.method_id_;
      // auto frame_id = outdata->value.frame_id_;
      auto thread_hash = outdata->value.thread_hash_;
      auto source_id = outdata->value.source_id_;
      if (method_list.find(outdata->value.method_id_) == method_list.end())
        method_list.insert(outdata->value.method_id_);
      if (thread_list.find(outdata->value.thread_hash_) == thread_list.end())
        thread_list.insert(outdata->value.thread_hash_);
      if (source_list.find(outdata->value.source_id_) == thread_list.end())
        source_list.insert(outdata->value.source_id_);
      // framecheck[outdata->value.source_id_].insert(outdata->value.frame_id_);
      // thread 和source id的绑定关系
      if (thread_to_source.find(thread_hash)== thread_to_source.end()) {
          thread_to_source[thread_hash] = std::unordered_set<size_t>();
      }
      if (thread_to_source[thread_hash].find(source_id)
        == thread_to_source[thread_hash].end())
        thread_to_source[thread_hash].insert(source_id);
      // thread 和 method 的绑定关系
      if (thread_to_method.find(thread_hash)== thread_to_method.end()) {
          thread_to_method[thread_hash] = std::unordered_set<size_t>();
      }

      if (thread_to_method[thread_hash].find(method_id)
        == thread_to_method[thread_hash].end())
        thread_to_method[thread_hash].insert(method_id);
      // source id 和 method id的绑定关系
      if (source_to_method.find(source_id)== source_to_method.end()) {
          source_to_method[source_id] = std::unordered_set<size_t>();
      }

      if (source_to_method[source_id].find(method_id)
        == source_to_method[source_id].end())
        source_to_method[source_id].insert(method_id);
    }

    // 检查收到的所有数据是否包含所有sourceid
    if (source_num > 0 && source_list.size() != source_num) {
      std::cout << "expect number " << source_list.size()
        <<" of soure_id equal to source_num " << source_num << std::endl;
      return -1;
    }
    // 检查收到数据是否包含所有thread
    if (thread_num > 0 && thread_list.size() != thread_num) {
      std::cout << "expect number" << thread_list.size()
        <<" of thread equal to thread_num" << thread_num << std::endl;
      return -1;
    }

    // 当thread safe 为false， method仅有可能绑定一个thread，
    if (methodinfo.is_thread_safe_ == false) {
      templist.clear();
      for (auto methodmap : thread_to_method) {
        for (auto methodid : methodmap.second) {
          if (templist.find(methodid) != templist.end()) {
            // method id 出现超过一次也就是mehtod 绑定了多个thread
            std::cout << "is_thread_safe_ == false, methodid "
            << methodid << " binded more than one thread"
            << std::endl;
            return -1;
          }
          templist.insert(methodid);
        }
      }
    } else if (thread_num !=1) {
      // thread safe 为true情况，遍历所有thread to method 的map，
      // 只要某个method出现过超过一次，即可证明method可以在任何一个thread上运行。
      // 除非thread数为1

      templist.clear();
      bool is_binded_twice = false;
      for (auto methodmap : thread_to_method) {
        for (auto methodid : methodmap.second) {
          if (templist.find(methodid) != templist.end()) {
            // method id 出现超过一次也就是mehtod 绑定了多个thread
            is_binded_twice = true;
            break;
          }
          templist.insert(methodid);
        }
        if (is_binded_twice)
          break;
      }
      if (is_binded_twice == false) {
        std::cout << "is_thread_safe_ = true, method only binded one thread"
          <<std::endl;
          return -1;
      }
    }



    if (methodinfo.is_src_ctx_dept == true) {
      if (method_list.size() != source_num) {
        std::cout<< "is_src_ctx_dept==true, total method istance "
        << method_list.size() << " should be equal to source num "
        << source_num << std::endl;
        return -1;
      }
      if (thread_list.size() > source_list.size()) {
        std::cout << "is_src_ctx_dept==true, thread num " << thread_list.size()
        << " can not be larger than source_num " << source_list.size();
      }
      // 检查source id和method绑定关系
      templist.clear();
      for (auto sourceid : source_list) {
        if (source_to_method[sourceid].size() != 1) {
          std::cout << "is_src_ctx_dept==true, source id "
          << sourceid << " binded more than one method" << std::endl;
          return -1;
        }

        if (templist.find(*source_to_method[sourceid].begin())
          != templist.end()) {
          // 有method id 重复 绑定source id
          std::cout <<  "is_src_ctx_dept==true, method id  "
            << (*source_to_method[sourceid].begin())
            << "binded to more than one source id" << std::endl;
          return -1;
        }
        templist.insert(*source_to_method[sourceid].begin());
      }
      templist.clear();
      if (methodinfo.is_thread_safe_ == false) {
        // 所有thread平分method id
        size_t avg = source_num / thread_num;
        for (auto methodlist : thread_to_method) {
          if (methodlist.second.size() != avg
          && methodlist.second.size() != avg +1) {
            std::cout << "is_thread_safe_ = false，is_src_ctx_dept==true，"
            << "thread hash " << methodlist.first
            << " binded wrong number of method, "
            << methodlist.second.size() << std::endl;
            return -1;
          }
        }
      }   // is_thread_safe_ true 的情况 thread 绑定所有的method。
    } else {
      // is_src_ctx_dept == false
      if (methodinfo.is_thread_safe_ == false
        && method_list.size() != thread_num) {
          std::cout << "When is_src_ctx_dept = false, is_thread_safe_ = false"
            << "method number " << method_list.size() << "shall equal to"
            << "thread_num " << thread_num << std::endl;
          return -1;
        }
      if (methodinfo.is_thread_safe_ == true
          && method_list.size() != 1) {
          std::cout << "When is_src_ctx_dept = false, is_thread_safe_ = true"
            << "method number " << method_list.size() << "shall equal to 1"
            << std::endl;
          return -1;
      }
    }

    return 0;
  }
  size_t GetResultNum() const {
    return resbuffer.size();
  }

 private:
  std::vector<HobotXRoc::MulSrcTestOutputPtr> resbuffer;
};

int AdjustConfigFile(const std::string& inconfig,
                     const std::string& outconfig,
                     int32_t source_num,
                     int32_t thread_num) {
  std::ifstream infile(inconfig);
  std::ofstream outfile(outconfig);
  Json::Value config;
  infile >> config;
  if (source_num <= 0) {
    config.removeMember(HobotXRoc::kSourceNum);
  } else {
    config[HobotXRoc::kSourceNum] = source_num;
  }
  auto &workflow = config[HobotXRoc::kWorkflow];
  int node_size = workflow.size();
  for (int i = 0; i < node_size; ++i) {
    auto node = workflow[i];
    auto method_name = node[HobotXRoc::kMethodType].asString();
    if (method_name == "MultiSourceTest") {
      workflow[i][HobotXRoc::kThreadNum] = thread_num > 0? thread_num:1;
      break;
    }
  }
  outfile << config;
  return 0;
}
const size_t k_framerounds = 20;
int DoTest(const std::string &config_template,
           const std::string &config_temp,
           const  HobotXRoc::MethodInfo &methodinfo,
           int source_num,
           int thread_count) {
  using HobotXRoc::BaseData;
  using HobotXRoc::BaseDataPtr;
  using HobotXRoc::BaseDataVector;
  using HobotXRoc::InputData;
  using HobotXRoc::InputDataPtr;
  using HobotXRoc::MulSrcTestInput;

  AdjustConfigFile(config_template, config_temp, source_num, thread_count);

  HobotXRoc::XRocSDK *flow = HobotXRoc::XRocSDK::CreateSDK();
  Callback callback;
  source_num = std::max(source_num, 1);
  thread_count = std::max(thread_count, 1);

  flow->SetCallback(std::bind(&Callback::OnCallback,
                    &callback,
                    std::placeholders::_1));
  flow->SetConfig("config_file", config_temp);
  flow->Init();

  LOGD << "multisource test Method Version : "
            << flow->GetVersion("multisource_node");

  InputDataPtr inputdata(new InputData());
  std::shared_ptr<MulSrcTestInput> test_input =
                   std::make_shared<MulSrcTestInput>();
  inputdata->datas_.resize(1);
  std::vector<size_t> shuffled_sourceid;

  for (size_t source_id = 0; source_id < (size_t)source_num; source_id ++) {
    for (size_t i = 0; i < k_framerounds; i ++)
      shuffled_sourceid.push_back(source_id);
  }

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::shuffle(shuffled_sourceid.begin(), shuffled_sourceid.end(),
    std::default_random_engine(seed));

  std::vector<size_t> frame_id;
  frame_id.resize(source_num, 0);

  for (auto source_id : shuffled_sourceid) {
      inputdata->source_id_ = source_id;
      test_input = std::make_shared<MulSrcTestInput>();
      test_input->name_ = "test_input";
      test_input->value.source_id_ = source_id;
      test_input->value.frame_id_ = frame_id[source_id]++;
      inputdata->datas_[0] = BaseDataPtr(test_input);
      flow->AsyncPredict(inputdata);
    }

  // waiting for async function done
  while (callback.GetResultNum() != shuffled_sourceid.size())
    std::this_thread::sleep_for(std::chrono::seconds(1));

  int ret = callback.ExamResult(methodinfo, source_num, thread_count);

  delete flow;
  // 初始化sdk
  return ret;
}

}  // namespace MultisourceTest

static const char config_template[] =
  "./test/configs/multiSource_test.json";
static const char config_temp[] = "./temp.json";
static HobotXRoc::MethodInfo methodinfo;
#if 1
TEST(MultiSourceTestMethod, BackwardCompatible1) {
  using  MultisourceTest::DoTest;

  SetLogLevel(HOBOT_LOG_ERROR);
  // Test 1.1
  HobotXRoc::MethodInfo methodinfo;
  methodinfo.is_need_reorder = true;
  methodinfo.is_src_ctx_dept = false;
  methodinfo.is_thread_safe_ = false;
  int sourcenum = 1;
  HobotXRoc::MultiSourceTestMethod::SetMethodInfo(methodinfo);
  // source_num = -1 thread_count = -1;
  EXPECT_EQ(0, DoTest(config_template, config_temp, methodinfo, sourcenum, -1));
  // source_num = 1 thread_count = 3;
  EXPECT_EQ(0, DoTest(config_template, config_temp, methodinfo, sourcenum, 3));
}

TEST(MultiSourceTestMethod, BackwardCompatible2) {
  using  MultisourceTest::DoTest;
  SetLogLevel(HOBOT_LOG_ERROR);
  // Test 1.2
  int sourcenum = 1;
  methodinfo.is_need_reorder = true;
  methodinfo.is_src_ctx_dept = false;
  methodinfo.is_thread_safe_ = true;
  HobotXRoc::MultiSourceTestMethod::SetMethodInfo(methodinfo);

  // source_num = 1 thread_count = 1;
  EXPECT_EQ(0, DoTest(config_template, config_temp, methodinfo, sourcenum, 1));
  // source_num = 1 thread_count = 3;
  EXPECT_EQ(0, DoTest(config_template, config_temp, methodinfo, sourcenum, 3));
}


TEST(MultiSourceTestMethod, MultiSource1) {
  using  MultisourceTest::DoTest;
  SetLogLevel(HOBOT_LOG_ERROR);
  // Test 2.1
  int sourcenum = 4;
  HobotXRoc::MethodInfo methodinfo;
  methodinfo.is_need_reorder = true;
  methodinfo.is_thread_safe_ = false;
  methodinfo.is_src_ctx_dept = false;
  HobotXRoc::MultiSourceTestMethod::SetMethodInfo(methodinfo);

  // source_num = 4 thread_count = -1;
  EXPECT_EQ(0, DoTest(config_template, config_temp, methodinfo, sourcenum, -1));
  // source_num = 4 thread_count = 2;
  EXPECT_EQ(0, DoTest(config_template, config_temp, methodinfo, sourcenum, 2));
  // source_num = 4 thread_count = 5;
  EXPECT_EQ(0, DoTest(config_template, config_temp, methodinfo, sourcenum, 5));
}

TEST(MultiSourceTestMethod, MultiSource2) {
  using  MultisourceTest::DoTest;
  SetLogLevel(HOBOT_LOG_ERROR);
  int sourcenum = 4;
  // Test 2.2
  methodinfo.is_need_reorder = true;
  methodinfo.is_thread_safe_ = false;
  methodinfo.is_src_ctx_dept = true;
  HobotXRoc::MultiSourceTestMethod::SetMethodInfo(methodinfo);

  // source_num = 4 thread_count = -1;
  EXPECT_EQ(0, DoTest(config_template, config_temp, methodinfo, sourcenum, -1));
  // source_num = 4 thread_count = 3;
  EXPECT_EQ(0, DoTest(config_template, config_temp, methodinfo, sourcenum, 3));
  // source_num = 4 thread_count = 4;
  EXPECT_EQ(0, DoTest(config_template, config_temp, methodinfo, sourcenum, 4));
}

TEST(MultiSourceTestMethod, MultiSource3) {
  using  MultisourceTest::DoTest;
  SetLogLevel(HOBOT_LOG_ERROR);
  int sourcenum = 4;
  // Test 2.3
  methodinfo.is_need_reorder = true;
  methodinfo.is_thread_safe_ = true;
  methodinfo.is_src_ctx_dept = false;
  HobotXRoc::MultiSourceTestMethod::SetMethodInfo(methodinfo);

  // source_num = 4 thread_count = -1;
  EXPECT_EQ(0, DoTest(config_template, config_temp, methodinfo, sourcenum, -1));
  // source_num = 4 thread_count = 3;
  EXPECT_EQ(0, DoTest(config_template, config_temp, methodinfo, sourcenum, 3));
  // source_num = 4 thread_count = 4;
  EXPECT_EQ(0, DoTest(config_template, config_temp, methodinfo, sourcenum, 4));
  // source_num = 4 thread_count = 5;
  EXPECT_EQ(0, DoTest(config_template, config_temp, methodinfo, sourcenum, 5));
}

TEST(MultiSourceTestMethod, MultiSource4) {
  using  MultisourceTest::DoTest;
  SetLogLevel(HOBOT_LOG_ERROR);
  int sourcenum = 4;
  // Test 2.4
  methodinfo.is_need_reorder = true;
  methodinfo.is_src_ctx_dept = true;
  methodinfo.is_thread_safe_ = true;
  HobotXRoc::MultiSourceTestMethod::SetMethodInfo(methodinfo);

  // source_num = 4 thread_count = -1;
  EXPECT_EQ(0, DoTest(config_template, config_temp, methodinfo, sourcenum, -1));
  // source_num = 4 thread_count = 3;
  EXPECT_EQ(0, DoTest(config_template, config_temp, methodinfo, sourcenum, 3));

  // source_num = 4 thread_count = 4;
  EXPECT_EQ(0, DoTest(config_template, config_temp, methodinfo, sourcenum, 4));
}
#endif
