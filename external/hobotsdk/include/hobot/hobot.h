//
// Copyright 2016 Horizon Robotics.
// Created by Lisen Mu on 7/20/16.
//

#ifndef HOBOT_HOBOT_H_
#define HOBOT_HOBOT_H_

#include <stdint.h>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <deque>
#include <tuple>
#include <utility>
#include <mutex>

#ifdef HR_WIN
#ifdef HOBOT_DLL_EXPORTS
#ifdef HOBOT_EXPORTS
#define HOBOT_EXPORT __declspec(dllexport)
#else
#define HOBOT_EXPORT __declspec(dllimport)
#endif
#else
#define HOBOT_EXPORT
#endif
#elif HR_POSIX
#define HOBOT_EXPORT
#endif

#ifndef INT32_MAX
#define INT32_MAX 2147483647
#endif

#define FORWARD_FUNCTION_SIGNATURE_(module_name, forward_index) \
  void Forward##forward_index(const hobot::MessageLists &input, \
  hobot::Workflow *workflow, hobot::spRunContext context)

#define WRAPPER_CLASS_(module_name, forward_index) \
  module_name##_##Forward##forward_index##_Wrapper

#define WRAPPER_INSTANCE_(module_name, forward_index) \
  module_name##_##Forward##forward_index##_Wrapper_Instance

/**
 * forward declare macro
 */
#define FORWARD_DEFAULT_THREAD_STACK_SIZE 0x200000

#define FORWARD_DECLARE(module_name, forward_index) \
        FORWARD_DECLARE_STACK_SIZE(module_name, forward_index, \
                                   FORWARD_DEFAULT_THREAD_STACK_SIZE)

#define FORWARD_DECLARE_STACK_SIZE(module_name, forward_index, \
                                                stack_size_default) \
  friend class WRAPPER_CLASS_(module_name, forward_index); \
  class WRAPPER_CLASS_(module_name, forward_index) \
  : public hobot::ForwardWrapper {  \
    public:                                                 \
      WRAPPER_CLASS_(module_name, forward_index)(module_name *module, \
                                      size_t stack_size = stack_size_default) \
       : module(module) { \
           stack_size_ = stack_size; \
           module->RegisterForwardWrapper(this, forward_index);    \
      }  \
      void Forward(const hobot::MessageLists &input, \
                     hobot::Workflow *workflow, \
                     hobot::spRunContext context) { \
          module->Forward##forward_index(input, workflow, context) ; \
              }                            \
    private: \
      module_name *module;                                        \
  };\
  WRAPPER_CLASS_(module_name, forward_index) \
     WRAPPER_INSTANCE_(module_name, forward_index) =  \
          WRAPPER_CLASS_(module_name, forward_index)(this);  \
  FORWARD_FUNCTION_SIGNATURE_(module_name, forward_index)


/**
 * forward define macro
 */
#define FORWARD_DEFINE(module_name, forward_index) \
     void module_name::Forward##forward_index \
        (const hobot::MessageLists &input, \
        hobot::Workflow *workflow, hobot::spRunContext context)


/**
 * config update listener define macro
 */
#define LISTENING_CLASS_(module_name, key_name) \
  module_name##_##key_name##_Wrapper

#define LISTENING_INSTANCE_(module_name, key_name) \
  module_name##_##key_name##_Wrapper_Instance

#define LISTENING_FUNCTION_SIGNATURE_(module_name, key_name) \
  void On##key_name##Update(const std::string& key)

/**
 * config update listener declare macro
 * Recommend way is to place this at the end inside your class declaration.
 * This is because the "private" keyword inside, which would cause members
 * after this macro to be private. User should modify your code accordingly,
 * if this is placed in the midle of yoru class declaration.
 */
#define LISTENING_CONFIG_DECLARE(module_name, key_name) \
  friend class LISTENING_CLASS_(module_name, key_name); \
  class LISTENING_CLASS_(module_name, key_name)  { \
  public:    \
      LISTENING_CLASS_(module_name, key_name)(module_name* module) { \
        module->configUpdateFns_[#key_name] = [module] \
          (const std::string &key) { \
          module->On##key_name##Update(key); \
        }; \
      } \
  }; \
  private: \
    LISTENING_CLASS_(module_name, key_name) \
      LISTENING_INSTANCE_(module_name, key_name) =  \
        LISTENING_CLASS_(module_name, key_name)(this); \
    LISTENING_FUNCTION_SIGNATURE_(module_name, key_name)

namespace hobot {

class Config;

class Module;

class ForwardWrapper;

class InputModule;

class Message;

class Expression;

class Workflow;

class LinkBuilder;

class Link;

class RunContext;

class ConfigExt;

typedef std::shared_ptr<LinkBuilder> spLinkBuilder;

typedef std::shared_ptr<Message> spMessage;

typedef std::shared_ptr<RunContext> spRunContext;

typedef std::deque<spMessage> MessageList;

typedef std::vector<MessageList *> MessageLists;

typedef std::shared_ptr<Config> spConfig;

typedef std::shared_ptr<ConfigExt> spConfigExt;

class HOBOT_EXPORT Engine {
 public:
  /**
   * creates a new Engine.
   * @return created engine
   */
  static Engine *NewInstance();

  /**
  * creates a new Thread Save Engine.
  * If workflow is fixed(no dynamic workflow change), NewInstance() is enough.
  * Otherwise, Thread Safe engine is Required.
  * @return created engine
  */
  static Engine *NewInstanceThreadSafe();

  /**
   * creates a new workflow to be executed on this engine.
   * @return created workflow
   */
  virtual Workflow *NewWorkflow() = 0;

  /**
   * specify forward runs on which thread.
   * forward{index} may run on the thread which forward{random index} runs on by default
   * Engine maintains a inner threadpool on which forward would run.
   */
  virtual bool ExecuteOnThread(Module *module,
                               int forward_index,
                               int thread_index) = 0;

  /**
   * Sets the cpu affinity of thread.
   */
  virtual bool SetAffinity(int thread_index, int core_id) = 0;

  virtual int GetThreadIdx(Module *module, int forward_index) = 0;

  virtual bool ThreadRun(int thread_index) = 0;

  virtual void ThreadStop(int thread_index) = 0;


  virtual ~Engine() { }
};

class RunObserver;

/**
 * A Directed Graph specifying executing dependencies of Modules,
 * Forming a more complicated functionality than a single Module.
 */
class HOBOT_EXPORT Workflow {
 public:
  /**
   * create a LinkBuilder object to create links from module `src`.
   * workflow->From(a)->To(b) to construct a link a -> b.
   * @param src source Module of the links to be created
   * @param output_slot Module's output slot index
   * @return LinkBuilder to build links
   */
  virtual spLinkBuilder From(Module *src, int output_slot = 0) = 0;


  /**
  * Generate Task .
  * @param outputs collection of Modules whose output would be gathered by RunObserver
  * @param ob callback handler to gather output for outputs
  * @param message
  */
  virtual spRunContext Run(std::vector<std::pair<Module *, int>> outputs,
                           RunObserver *ob = nullptr,
                           Message *message = nullptr) = 0;

  /**
   * Run the graph. once. function `Run` above is strongly recommended
   * @param outputs collection of Modules whose output would be gathered by RunObserver
   * @param inputs input messages, map from InputModule to Message
   * @param ob callback handler to gather output for outputs
   * @code return init code
   * @module return the module whitch init failed
   * @param message
   */
  virtual spRunContext Run(std::vector<std::pair<Module *, int>> outputs,
                           std::vector<std::tuple<Module *,
                                                  int,
                                                  spMessage>> inputs,
                           RunObserver *ob = nullptr,
                           int *code = nullptr,
                           Module **module = nullptr,
                           Message *message = nullptr) = 0;

  /**
   * Feed and run the graph.
   * @param run_context
   * @param module
   * @param forward_index
   * @param message
   */
  virtual void Feed(spRunContext run_context,
                    Module *module,
                    int forward_index,
                    spMessage message) = 0;

  /**
  * Feed and run the graph.
  * @param run_context
  * @param module
  * @param forward_index
  * @param intput slot index
  * @param message
  */
  virtual void Feed(spRunContext run_context,
                    Module *module,
                    int forward_index,
                    int input_slot_index,
                    spMessage message) = 0;

  /**
   * Reset , clear all messages in the flow and reset modules
   */
  virtual void Reset() = 0;


  /**
   * get run condition of module in current workflow.
   * @param module module to get
   * @param forward_index forward index
   * @return the run condition of module
   */
  virtual Expression *ConditionOf(Module *module, int forward_index) = 0;

  /**
   * set run condition of module in current workflow.
   * Default run condition is AllExp:
   * every input has at least 1 message;
   * each input will fetch 1 message.
   * @param module module
   * @param condition condition
   * @param forward_index forward index
   */
  virtual void SetCondition(Module *module,
                            Expression *condition,
                            int forward_index) = 0;

  /**
   * set io flusher of module in current workflow.
   * Default io flusher is inactive.
   * @param module module
   * @param millisec io flusher will flush all input data every millisec
   * @param forward_index forward index
   */
  virtual void SetFlusher(Module *module,
                            int millisec,
                            int forward_index) = 0;

  /**
  * set module forward execution timeout in millisecond.
  * Observer->OnTimeout() will be triggered if timeout
  * Default timeout behavior is inactive.
  * @param module module
  * @param millisecond for timeout
  * @param forward_index forward index
  */
  virtual void SetExecTimeout(Module *module,
                              int millisec,
                              int forward_index) = 0;

  /**
   * should be called within Module's Forward(),
   * to produce Module's output message
   * @param from module
   * @param output_slot Module's output slot index
   * @param output message produced
   * @param context the RunContext
   */
  virtual void Return(Module *from,
                      int output_slot,
                      spMessage output,
                      spRunContext context) = 0;


  /**
   * may be called within Module's Forward or Init on  error condition
   * @param from
   * @param forward_index
   * @param err error message produced
   * @param context the RunContext
   */
  virtual void Error(Module *from,
                     int forward_index,
                     spMessage err,
                     spRunContext context) = 0;

  /**
  * may be called within Module's Forward or Init on error condition
  * and not blocking any thread
  * @param from
  * @param forward_index
  * @param err error message produced
  * @param context the RunContext
  */
  virtual void ErrorNoBlock(Module *from,
                            int forward_index,
                            spMessage err,
                            spRunContext context) = 0;

  /**
   * could be called within Module's Forward(),
   * notifying another Forward() in the future
   * @param from module
   * @param forward_index forward index
   * @param input input data of next Forward()
   * @param context the RunContext
   * @param millisec delay of next Forward(), in milliseconds, from now
   */
  virtual void Reschedule(Module *from, int forward_index,
                          const MessageLists &input,
                          const spRunContext &context, int millisec) = 0;

  virtual ~Workflow() { }

  /**
  * could be called within Module's Forward(),
  * notifying another Forward() in the future
  * @param module module ptr
  * @param forward_index forward index
  * @param input_slot module's input slot of specific forward index
   * @return if Module's input_slot@forward_index is linked
  */
  virtual bool CheckInputLink(Module *module,
                              int forward_index, int input_slot) = 0;

  /**
  * could be called within Module's Forward(),
  * notifying another Forward() in the future
  * @param module module ptr
  * @param forward_index forward index
  * @param output_slot module's output slot
  * @return if Module's output_slot is linked
  */
  virtual bool CheckOutputLink(Module *module, int output_slot) = 0;


  /**
  * manually consume link limit counter
  * @param module module ptr
  * @param forward_index forward index
  * @param input_slot module's input slot
  * @param size limit size
  */
  virtual void ConsumeLinkLimit(spRunContext run_context, Module *module,
                                int forward_index, int input_slot,
                                int size) = 0;

  /**
  * manually release link limit counter
  * @param module module ptr
  * @param forward_index forward index
  * @param input_slot module's input slot
  * @param size limit size
  */
  virtual void ReleaseLinkLimit(spRunContext run_context, Module *module,
                                int forward_index, int input_slot,
                                int size) = 0;
  /**
  * set output slots and forward index binding
  * @param module module ptr
  * @param forward_index forward index
  * @param output_slots bindings(unbind if output_slots is empty)
  */
  virtual void SetOutputSlotBinding(Module *module, int forward_index,
                                    const std::vector<int> &output_slots) = 0;
};

/**
 * Callback interface of Workflow::Run()
 * If user wishes to receive results of Workflow::Run(),
 * they should provide a instance of subclass of RunObserver to Run().
 */
class HOBOT_EXPORT  RunObserver {
 public:
  /**
   * call back to receive output from run
   * @param from
   * @param forward_index
   * @param output the returned message, same order specified
   * with Run()
   */
  virtual void OnResult(Module *from, int forward_index, spMessage output) = 0;

  /**
   *  callback to handle error from run ( Init and Forward included )
   *  @param from
   *  @param forward_index
   *  @param err
   */
  virtual void OnError(Module *from, int forward_index, spMessage err) { }

  /**
  *  callback to handle timeout module's forward execution
  *  @param from
  *  @param forward_index
  */
  virtual void OnExecTimeout(Module *from, int forward_index, int run_time) { }

  virtual ~RunObserver() { }
};

class HOBOT_EXPORT  LinkBuilder {
 public:
  /**
   * Link links module A's output to module B's input slot
   * Every module has exact one output;
   * can have zero or more input slots, indexing from 0 to N.
   * @param dest destination module
   * @param index index if input slot of `dest`
   * @param forward_index forward index
   */
  virtual Link *To(Module *dest,
                   int input_index = 0,
                   int forward_index = 0) = 0;
  // do we need another start_size to control
  // minimum requirement for output buffer?

  virtual ~LinkBuilder() { }
};

class HOBOT_EXPORT  Link {
 public:
  /**
   * Limit link output buffer size
   */
  virtual void Limit(int buffer_size = INT32_MAX) = 0;

  virtual ~Link() { }
};

typedef union HOBOT_EXPORT _Variant32 {
  _Variant32(): valueint(0) {}
  _Variant32(int _value): valueint(_value) {}
  _Variant32(float _value): valuefloat(_value) {}
  _Variant32(bool _value): valuebool(_value) {}
  _Variant32(const _Variant32 &v): valueint(v.valueint) {}

  // char int long
  int valueint;
  float valuefloat;
  bool valuebool;
} Variant32;

typedef union HOBOT_EXPORT _Variant64 {
  _Variant64() {}
  _Variant64(int64_t _value): valuelong(_value) {}
  _Variant64(double _value): valuedouble(_value) {}

  // char int long
  int64_t valuelong;
  double valuedouble;
} Variant64;

class HOBOT_EXPORT Config {
 public:
  static spConfig LoadConfig(const std::string &filename);
  static spConfig LoadStringConfig(const std::string &config_string);
  Config() { }

  template<typename T>
  void SetParams(const std::string &key, T value);

  int GetIntValue(const std::string &key, int default_value = 0);

  float GetFloatValue(const std::string &key, float default_value = 0.0f);

  bool GetBoolValue(const std::string &key, bool default_value = false);

  int64_t GetLongValue(const std::string &key, int64_t default_value = 0);

  double GetDoubleValue(const std::string &key, double default_value = 0.0);

  const std::string & GetSTDStringValue(const std::string &key,
                                     const std::string &default_value = "");

  const char * GetStringValue(const std::string &key,
                              const char * default_value = "");

  // will insert a new empty sub config if sub_configs_[name] do not exist
  Config *GetSubConfig(const std::string &name);

  void AddSubConfig(const std::string &name, spConfig config);

  Config &operator=(const Config &other);

 private:
  template<typename T>
  const T &GetParamValue(const std::string &key, const T &default_vaule);

  typedef std::map<std::string, Variant32> MapVariant32;
  typedef std::map<std::string, Variant64> MapVariant64;
  typedef std::map<std::string, std::string> MapString;
  typedef std::map<std::string, spConfig> MapSpConfig;

  MapVariant32 values32_;

  MapVariant64 values64_;

  MapString values_string_;

  MapSpConfig sub_configs_;

  std::mutex config_mutex_;
};

class HOBOT_EXPORT ConfigExt {
public:
  ConfigExt() { };

  ConfigExt(spMessage handle);

  bool LoadConfig(const std::string &filename);

  bool LoadStringConfig(const std::string &config_string);

  bool Release();

  bool Save2File(std::string another_file = "");

  std::string Save2String() const;

  int GetIntValue(const std::string &key, int default_value = 0);

  float GetFloatValue(const std::string &key, float default_value = 0.0f);

  bool GetBoolValue(const std::string &key, bool default_value = false);

  int64_t GetLongValue(const std::string &key, int64_t default_value = 0);

  double GetDoubleValue(const std::string &key, double default_value = 0.0);

  const std::string GetSTDStringValue(const std::string &key, const std::string &default_value = "");

  spConfigExt GetSubConfig(const std::string &name);

  spConfigExt GetArray(const std::string &arrayname);

  int GetArraySize();

  bool SetIntValue(const std::string& key, int value);

  bool SetFloatValue(const std::string& key, float value);

  bool SetBoolValue(const std::string& key, bool value);

  bool SetLongValue(const std::string& key, int64_t value);

  bool SetDoubleValue(const std::string& key, double value);

  bool SetSTDStringValue(const std::string& key, std::string& value);

  bool DelItem(const std::string& key);

  spConfigExt operator[](int index);

private:
  spMessage jhanle_;
};

class ForwardWrapper {
 public:
  virtual void Forward(const MessageLists &input,
                       Workflow *workflow,
                       spRunContext context) = 0;

  virtual ~ForwardWrapper() { }

  size_t GetStackSize() {
    return stack_size_;
  }

 protected:
  size_t stack_size_;
};

typedef std::function<void(const std::string &)> fnConfigUpdate;
class HOBOT_EXPORT Module {
 public:
  explicit Module(std::string instance_name = "", std::string class_name = "")
      : instance_name_(instance_name),
        class_name_(class_name),
        inited_(false) { }

  virtual std::string GetFullClassName();

  /**
   *
   * @return the Full Name of this instance; format/content of the full name
   * is determined by implementation, but usually would contain Class Name and
   * Instance ID.
   */
  virtual std::string GetFullInstanceName();

  virtual void SetInstanceName(const std::string &instance_name) {
    this->instance_name_ = instance_name;
  }

  /**
   * Init will be called exactly once, for this module,
   * in each <engine, workflow> composition
   */
  virtual int
      Init(RunContext *context) = 0;

  void UpdateConfig(spConfig config);

  void UpdateConfig(const Config &config);

  template<typename T>
  void UpdateConfig(const std::string &key, T value);

  template<typename T>
  void UpdateConfig(const std::vector<std::pair<std::string, T>>& params);

  Config *GetConfig();

  virtual void OnConfigUpdate();

  virtual void Reset() = 0;

  size_t GetForwardCount() {
    return forward_wrappers_.size();
  }

  ForwardWrapper *GetForwardWrapper(int forward_index) {
    return forward_wrappers_[forward_index];
  }

  virtual ~Module() { }

 protected:
  void RegisterForwardWrapper(ForwardWrapper *forward_wrapper,
                              size_t forward_index) {
    if (forward_wrappers_.size() < forward_index + 1) {
      forward_wrappers_.resize(forward_index + 1);
    }
    forward_wrappers_[forward_index] = forward_wrapper;
  }

 public:
  std::map<std::string, fnConfigUpdate> configUpdateFns_;
  bool inited_;

 private:
  std::string class_name_;

  std::string instance_name_;

  Config config_;

  std::vector<ForwardWrapper *> forward_wrappers_;
};

class HOBOT_EXPORT InputModule: public Module {
 public:
  explicit InputModule(std::string instance_name = "")
      : Module(instance_name, "Input") {
  }

  FORWARD_DECLARE(InputModule, 0);

  int
  Init(RunContext *context) override {
    return 0;
  }

  void Reset() override {
  }
};

class GroupModuleImp;
class HOBOT_EXPORT GroupModule : public Module {
 public:
    explicit GroupModule(std::string instance_name = "")
        : Module(instance_name, "GroupModule") {
    }
    virtual ~GroupModule();

    FORWARD_DECLARE(GroupModule, 0);

    /**
    * bind the group module with all child modules.
    * group must add the forward 1 of itself into the outputs modules, otherwise it can't work.
    * @param outputs collection of Modules whose output would be gathered by RunObserver, .
    */
    bool SetModules(const std::vector<Module*>& modules,
        int forward_index, Workflow *workflow,
        std::vector<std::pair<Module *, int>> *outputs);

    int Init(RunContext *context) override;
    void Reset() override;

 protected:
    GroupModuleImp * imp_ = nullptr;
};

/**
 * context object associated to one Workflow::Run() invocation.
 */
class HOBOT_EXPORT RunContext {
 public:
  virtual Engine *GetEngine() = 0;
  virtual Workflow *GetWorkflow() = 0;
  virtual Message *GetGlobalMessage() = 0;

  /**
   * Init reasonable modules in order
   * @param module return the moudle which init failed
   * @param init_order
   * @param verbose verbose mode
   */
  virtual int Init(Module **module = nullptr,
                   std::vector<Module*> *init_order = nullptr,
                   bool verbose = false) = 0;

  virtual ~RunContext() { }
};

/**
 * Envelop base class for all data objects input/output of Module.
 */
class HOBOT_EXPORT Message {
 public:
  virtual ~Message();
};

enum {
  ReqAll = -1
};

class HOBOT_EXPORT Expression {
 public:
  virtual ~Expression() { }

  /**
   * returns an AND expression: `a && b`.
   * AND expression's FetchAndEvaluate() fetches `a` and `b`
   * IFF `a` and `b` are both true; otherwise `data` is left unchanged.
   * @param a expression
   * @param b expression
   * @return AND expression
   */
  static Expression *And(Expression *a, Expression *b);

  /**
   * returns an OR expression: `a || b`.
   * OR expression's FetchAndEvaluate() fetches `a` if `a` evaluates to true;
   * if not, fetches `b` if `b` evaluates to true;
   * otherwise `data` is left unchanged.
   * @param a
   * @param b
   * @return OR expression
   */
  static Expression *Or(Expression *a, Expression *b);

  /**
   * returns an Require expression.
   * Require expression FetchAndEvaluate() success
   * if the data[index] has at least `count` messages.
   * if `count` == hobot::ReqAll, then this `Require` always evaluates to true,
   * and all messages in the `index`'th of `data` are fetched.
   * @param index index of data
   * @param count count of messages in data[index]
   * @param pass_if_no_link
            if input_slot[index] is NULL or index is out of range
            evaluate returns this value
   * @return Require expression
   */
  static Expression *Require(int index, int count = 1,
                             bool pass_if_no_link = true);

  /**
   * return an expression: `this && b`
   * @param b another expression
   * @return the AND expression
   */
  virtual Expression *And(Expression *b) = 0;

  /**
   * return an expression: `this || b`
   * @param b another expression
   * @return the OR expression
   */
  virtual Expression *Or(Expression *b) = 0;

  /**
   * Evaluate this expression;
   * fetch data into buffer if expression evaluates to true
   * @param data data to evaluate & fetch from
   * @param buffer empty vector to fetch into
   * @return true if evaluate passes and fetch success
   */
  virtual bool EvaluateAndFetch
      (const MessageLists &data, const MessageLists &buffer) = 0;

  /**
   * Evaluate this expression only
   * @param data data to evaluate
   * @return true if evaluate passes
   */
  virtual bool Evaluate(const MessageLists &data) = 0;

  /**
   * Fetch data into buffer this expression only
   * @param data data to fetch from
   * @param buffer empty vector to fetch into
   * @return true if fetch success
   */
  virtual bool Fetch(const MessageLists &data,
                     const MessageLists &buffer) = 0;


  /**
  * Evaluate this expression;
  * fetch data into buffer if expression evaluates to true
  * using links to skip input slots that not linked
  * @param data data to evaluate & fetch from
  * @param buffer empty vector to fetch into
  * @return true if evaluate passes and fetch success
  */
  virtual bool EvaluateAndFetch(const std::vector<Link *> &links,
                                const MessageLists &data,
                                const MessageLists &buffer) = 0;

  /**
  * Evaluate this expression only
  * using links to skip input slots that not linked
  * @param data data to evaluate
  * @return true if evaluate passes
  */
  virtual bool Evaluate(const std::vector<Link *> &links,
                        const MessageLists &data) = 0;

  /**
  * Fetch data into buffer this expression only
  * using links to skip input slots that not linked
  * @param data data to fetch from
  * @param buffer empty vector to fetch into
  * @return true if fetch success
  */
  virtual bool Fetch(const std::vector<Link *> &links,
                     const MessageLists &data,
                     const MessageLists &buffer) = 0;
};

}  // namespace hobot

#endif  // HOBOT_HOBOT_H_
