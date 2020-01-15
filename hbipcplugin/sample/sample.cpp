/*
 * @Description: sample
 * @Author: songhan.gong@horizon.ai
 * @Date: 2019-09-24 15:33:49
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-10-14 21:06:18
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */

#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <sstream>

#include "xpluginflow/message/pluginflow/flowmsg.h"
#include "xpluginflow/message/pluginflow/msg_registry.h"
#include "xpluginflow/plugin/xpluginasync.h"

#include "hbipcplugin/hbipcplugin.h"

#include "hobotlog/hobotlog.hpp"

/*
 * 添加自定义plugin分为三步：
 * 1.使用宏XPLUGIN_REGISTER_MSG_TYPE,自定义消息类型，每个plugin名字唯一；
 * 2.在生成消息的Plugin部分定义新的Message，新的Message需要继承XPluginFlowMessage;
 * 3.继承XPluginAsync实现自定义Plugin：
 *   a.实现消息处理函数；
 *   b.覆盖Init函数，在其中完成监听消息注册及其他初始化工作，并在函数返回前调用父plugin的Init方法。
 */

using horizon::vision::xpluginflow::XPluginAsync;
using horizon::vision::xpluginflow::XPluginFlowMessage;
using horizon::vision::xpluginflow::XPluginFlowMessagePtr;

using horizon::vision::xpluginflow::hbipcplugin::CustomHbipcMessage;
using horizon::vision::xpluginflow::hbipcplugin::HbipcPlugin;

using std::chrono::milliseconds;

class HbipcProducerPlugin : public XPluginAsync {
 public:
  HbipcProducerPlugin() { total_cnt_ = 100; }
  ~HbipcProducerPlugin() = default;

  std::string desc() const { return "HbipcProducerPlugin"; }
  int Start() {
    LOGI << "total_cnt=" << total_cnt_;
    prd_thread_ = new std::thread([&]() {
      for (int i = 0; i < total_cnt_ && !prd_stop_; i++) {
        auto np_msg = std::make_shared<CustomHbipcMessage>("Hello X2!");
        PushMsg(np_msg);
        std::this_thread::sleep_for(milliseconds(40));
      }
      LOGI << desc() << " prd exit";
    });
  }
  int Stop() {
    prd_stop_ = true;
    prd_thread_->join();
  }

 private:
  uint32_t total_cnt_;
  std::thread *prd_thread_;
  bool prd_stop_{false};
};

struct SmartContext {
  bool exit;
  SmartContext() : exit(false) {}
};

SmartContext g_ctx;

static void signal_handle(int param) {
  std::cout << "recv signal " << param << ", stop" << std::endl;
  if (param == SIGINT) {
    g_ctx.exit = true;
  }
}

int main() {
  SetLogLevel(HOBOT_LOG_DEBUG);

  signal(SIGINT, signal_handle);
  signal(SIGPIPE, signal_handle);
  signal(SIGSEGV, signal_handle);

  auto sc_plg = std::make_shared<HbipcPlugin>("./configs/hbipc_config.json");

  std::cout << "step 1" << std::endl;
  sc_plg->Init();

  std::cout << "step 2" << std::endl;
  sc_plg->Start();

  while (!g_ctx.exit) {
    std::this_thread::sleep_for(milliseconds(40));
  }

  std::cout << "step 3" << std::endl;
  sc_plg->Stop();

  std::cout << "step 4" << std::endl;
  sc_plg->Deinit();

  return 0;
}
