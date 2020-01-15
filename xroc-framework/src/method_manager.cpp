/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     MethodManager interface of xroc framework
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */
#include "hobotxroc/method_manager.h"
#include <algorithm>
#include <chrono>
#include <unordered_map>

#include "hobotxroc/method_factory.h"
#include "hobotxroc/json_key.h"
#include "hobotlog/hobotlog.hpp"

namespace HobotXRoc {
/// MethodManager
MethodManager::~MethodManager() {
  // release resources
  for (auto &method : methods_) {
    method->Finalize();
  }
}

void MethodManager::Init(const Config &config,
                         const XRocSharedConfig &shared_config,
                         ThreadManager *engine) {
  LOGD << "MethodManager Init ";
  config_ = config;
  shared_config_ = shared_config;
  method_type_ = config_[kMethodType].asString();
  method_name_ = config_[kMethodName].asString();
  auto temp_method = MethodFactory::CreateMethod(method_type_);
  auto methodinfo = temp_method->GetMethodInfo();
  size_t thread_count = 0;

  if (!config_.isMember(kThreadNum) && !config_.isMember(kTheadList)) {
    // 如果没有thread num的信息或者 threadlist信息。default 认为thread count为1
    // 创建 source_num_个线程,否则创建1个线程。
    thread_count = 1;
  } else {
      // 至少一个thread setting存在
      HOBOT_CHECK(!(config_.isMember(kThreadNum)
      && config_.isMember(kTheadList)))
      << "Thread setting can choose only one from thread_list and thread_count";
      // 向前兼容，以thread list 为主
      thread_count =  config_.isMember(kTheadList) ?
                      config_[kTheadList].size() :
                      config_[kThreadNum].asUInt();
  }
  // 根据threadcount和其他配置生成线程
  if (config_.isMember(kTheadList)) {
    auto thread_list = config_[kTheadList];
    HOBOT_CHECK(thread_count > 0)
    << "node "<< method_name_ << "thread list is none";
    // TODO(SONGSHAN) :判断同一列表的thread_idx唯一
    for (uint i = 0; i < thread_list.size(); i++) {
      uint32_t thread_idx = thread_list[i].asUInt();
      threads_.push_back(engine->CreateThread(thread_idx));
    }
  } else {
    // no "thread_list", create auto-produce thread
    for (size_t i = 0; i < thread_count; i++) {
      threads_.push_back(engine->CreateAutoThread());
    }
  }
    // 根据配置计算method实例总数
  int32_t method_instance_count = 1;
  if (methodinfo.is_src_ctx_dept) {
    // 检查 thread count与 source num的关系是否合法
    HOBOT_CHECK(thread_count <=
        static_cast<size_t>(shared_config_.source_num_))
        << "When method is source context dependent， thread count"
        << thread_count << " can not be larger than source number "
        << shared_config_.source_num_;
    // 当method跟输入源的上下文有关的时候，method的实例个数必须跟source个数一致
    method_instance_count = shared_config_.source_num_;
  } else {
    if (!methodinfo.is_thread_safe_) {
        // 一个method实例能且仅能在一个线程上执行。为向前兼容，
        // method instant count与thread count 保持一致
        method_instance_count = thread_count;
    } else {
      // 为向前兼容，当不支持多输入的时候，method 实例数为1
      method_instance_count = 1;
    }
  }
  // 根据配置创建method，放到到methods_保存
  methods_.reserve(method_instance_count);
  methods_.push_back(temp_method);
  MethodManagerContextState state = MethodManagerContextState::NONE;
  if (method_instance_count == 1) {
    if (0 == methods_[0]->Init(config_[kMethodCfgPath].asString())) {
      state = MethodManagerContextState::INITIALIZED;
    } else {
      LOGE << " methods init failed";
      exit(1);
    }
  } else {
    state = MethodManagerContextState::NONE;
    size_t extra_instance_count = method_instance_count - 1;
    for (size_t i = 0; i < extra_instance_count; ++i) {
      methods_.push_back(MethodFactory::CreateMethod(method_type_));
    }
  }

  // 创建context实例用于保存method实例上下文，个数等于thread个数
  contexts_.resize(thread_count);
  std::vector<void *> params;
  params.resize(thread_count);
  std::vector<WrapperFunctionTask> prepares;
  std::function<void(void *)> prepare =
    std::bind(&MethodManager::InitMethod, this, std::placeholders::_1);
  for (size_t i = 0; i < thread_count; ++i) {
    contexts_[i] = std::make_shared<MethodManagerContext>();
    contexts_[i]->state_ = state;
    params[i] = contexts_[i].get();
    prepares.push_back(prepare);
  }

  // 在thread中绑定相应的method实例，规则见 http://jira.hobot.cc/browse/XPPXROC-4
  if (methodinfo.is_thread_safe_) {
    // thread safe为true，任意method实例都能在任意线程都上运行
    // methods_.size()一定大于1，在分配前因此对所有的method先做初始化处理
    if (state !=  MethodManagerContextState::INITIALIZED) {
      for (size_t i = 0; i < methods_.size(); ++i) {
        if (0 != methods_[i]->Init(config_[kMethodCfgPath].asString())) {
          LOGE << "methods " << i << " initial failed!";
          exit(1);
        }
      }
    }
      // 因此任意一个thread都可以运行所有的method， 当src_dpt == true
      // key：value对为sourceid 与method实例，当 src_dpt = false
      //
      std::unordered_map<int32_t, MethodPtr> temp_methodlist;
      for (int32_t i = 0; i < (int32_t)methods_.size(); i ++)
        temp_methodlist.insert({i, methods_[i]});
      for (size_t i = 0; i < thread_count; ++i) {
        // context绑定所有method
        contexts_[i]->method_list_ = temp_methodlist;
        // 所有method已经被初始化，不需要用prepare运行了
        contexts_[i]->state_ = MethodManagerContextState::INITIALIZED;
    }
  } else {
    // 线程不安全
    // 当src_dpt == true, method个数为sourcenum，将method在thread中平均分配。
    // context 中的key和method的对应为source id和method一一对应
    if (methodinfo.is_src_ctx_dept) {
      for (size_t i = 0; i < methods_.size(); i++) {
        contexts_[i % thread_count]->method_list_.insert({i, methods_[i]});
      }
    } else {
      // 当src_dpt == flase且线程不安全的时候，method个数与thread个数一致，
      // 每个thread仅绑定一个method。因此key和method的只有一对。key的值为0.
      // 不用methodid作为key，是由于roundrobin情况下，postasynctask的调用
      // 者，无法预先得知那个thread会得到执行。
       for (size_t i = 0; i < methods_.size(); i++) {
        contexts_[i % thread_count]->method_list_.insert(
            {0, methods_[i]});
      }
    }
  }


  thread_pool_.reset(new XThreadPool(method_name_,
                                    threads_,
                                    prepares,
                                    params),
                    [](XThreadPool *thp) {
                      thp->Stop();
                      delete thp;
                    });

  if (methodinfo.is_src_ctx_dept && !methodinfo.is_thread_safe_) {
      // 如果method对输入源上下文依赖, 且对线程不安全，
      // 通过source id获取threads对象
    thread_pool_->SetPostStrategy(XThreadPool::PostStrategy::KEY_MATCHING);
    thread_pool_->SetKeyMatchingFunc(
        [](const void* key, const std::vector<void*>& contexts)->int {
          if (key == nullptr)
            return -1;
          size_t source_id = *(static_cast<const size_t *> (key));
          return static_cast<int>(source_id % contexts.size());
        });
  }

  WaitUntilInit();

  if (config_.isMember(kThreadPriority)) {
    auto thread_priority = config_[kThreadPriority];
    auto policy = thread_priority[kPolicy].asString();
    auto priority = thread_priority[kPriority].asInt();
    thread_pool_->SetPriority(policy, priority);
  }
}

void MethodManager::WaitUntilInit() {
  while (true) {
    decltype(contexts_)::size_type initialized_count = 0;
    for (auto &ctx : contexts_) {
      if (ctx->state_ == MethodManagerContextState::INITIALIZED) {
        initialized_count++;
      }
    }
    if (initialized_count == contexts_.size()) {
      break;
    }
    std::chrono::milliseconds dur(5);
    std::this_thread::sleep_for(dur);
  }
}

int MethodManager::UpdateParameter(InputParamPtr ptr) {
//  if (ptr->method_type_ != method_type_) {
//    LOG_ERROR("[MethodManager::UpdateParameter] type mismatch");
//    return 1;
//  }
  WriteLockGuard guard(&lock_);
  for (auto method : methods_) {
    auto ret = method->UpdateParameter(ptr);
    if (0 != ret) {
      LOGE
      << "Failed to update parameter for " << ptr->method_name_ << " with code "
      << ret;
      return -1;
    }
  }
  return 0;
}

InputParamPtr MethodManager::GetParameter() const {
  if (methods_.empty()) {
    return nullptr;
  }
  return methods_[0]->GetParameter();
}

std::string MethodManager::GetVersion() const {
  if (methods_.empty()) {
    return std::string{};
  }
  return methods_[0]->GetVersion();
}

bool MethodManager::IsNeedReorder() {
  return methods_[0]->GetMethodInfo().is_need_reorder;
}


int MethodManager::ProcessAsyncTask(
    const std::vector<std::vector<BaseDataPtr>> &inputs,
    const std::vector<InputParamPtr> &params,
    ResultCallback methodCallback,
    size_t source_id) {

  uint32_t method_key = GenMethodKey(inputs, params, source_id);
  // 创建线程池可以处理的函数对象
  auto task = std::bind(&MethodManager::Process, this, inputs, params,
                        methodCallback, method_key, std::placeholders::_1);
  // 把task丢到method线程池队列
  thread_pool_->PostAsyncTask(task, &source_id);
  return 0;
}

void MethodManager::InitMethod(void *ptr) {
  MethodManagerContext *ctx = reinterpret_cast<MethodManagerContext *>(ptr);
  if (MethodManagerContextState::INITIALIZED == ctx->state_) {
    return;
  }

  // initail the method list
  for (auto method : ctx->method_list_) {
    if (0 != method.second->Init(config_[kMethodCfgPath].asString())) {
    exit(1);
      LOGE << "method Init failed";
    }
  }
  ctx->state_ = MethodManagerContextState::INITIALIZED;
}

// 把method和对应查找的key通过函数封装起来，如果未来有更复杂的映射，只需要修改本函数
uint32_t MethodManager::GenMethodKey(
    const std::vector<std::vector<BaseDataPtr>> &inputs,
    const std::vector<InputParamPtr> &params,
    size_t source_id) {
  MethodInfo methodinfo = methods_[0]->GetMethodInfo();
  if (methodinfo.is_src_ctx_dept) {
    return source_id;
  } else {
    return 0;
  }
}

// 需要线程池支持指定thread停止，并且停止时执行一段传进来的代码
void MethodManager::Process(const std::vector<std::vector<BaseDataPtr>> &inputs,
                            const std::vector<InputParamPtr> &params,
                            ResultCallback method_callback,
                            uint32_t method_key, void *context) {
  auto c = static_cast<MethodManagerContext *>(context);
  if (c->method_list_.find(method_key) == c->method_list_.end()) {
        LOGE << "can not found method for given method key " << method_key;
        exit(1);
  }
  HobotXRoc::MethodPtr method = c->method_list_[method_key];

  switch (c->state_) {
    case MethodManagerContextState::INITIALIZED: {
      RUN_PROCESS_TIME_PROFILER_WITH_TAG(method_name_, "Framework Time")
      std::vector<std::vector<BaseDataPtr>> res;
      ReadLockGuard guard(&lock_);
      res = method->DoProcess(inputs, params);
      method_callback(res);
    }
      break;
    case MethodManagerContextState::FINALIZED:
      // TODO(jet) handle this case
      break;
    default:
      // TODO(jet) add warning
      break;
  }
}
/**
 * \brief callback function when profiler status is changed
 * @param on
 */
void MethodManager::OnProfilerChanged(bool on) {
  for (auto &method:methods_) {
    method->OnProfilerChanged(on);
  }
}

}  // namespace HobotXRoc
