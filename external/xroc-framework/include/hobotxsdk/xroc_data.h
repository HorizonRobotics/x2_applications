/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief provides base data struct for xroc framework
 * @file xroc_data.h
 * @author    chuanyi.yang
 * @email     chuanyi.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */

#ifndef HOBOTXSDK_XROC_DATA_H_
#define HOBOTXSDK_XROC_DATA_H_

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace HobotXRoc {

class CppContext;
class CContext;

/// 数据状态
enum class DataState {
  /// 有效
      VALID = 0,
  /// 被过滤掉
      FILTERED = 1,
  /// 不可见
      INVISIBLE = 2,
  /// 消失
      DISAPPEARED = 3,
  /// 无效
      INVALID = 4,
};

/// 数据结构基类，框，lmk等等的基类
struct BaseData {
  BaseData();
  virtual ~BaseData();
  /// 类型
  std::string type_ = "";
  /// 名称
  std::string name_ = "";
  /// 错误码
  int error_code_ = 0;
  /// 错误信息
  std::string error_detail_ = "";
  /// C数据结构上下文
  std::shared_ptr<CContext> c_data_;
  /// 状态
  DataState state_ = DataState::VALID;
};

typedef std::shared_ptr<BaseData> BaseDataPtr;

struct BaseDataVector : public BaseData {
  BaseDataVector();

  std::vector<BaseDataPtr> datas_;
};

/// 类模板，value可以为任意类型的数据，比如vision_type里定义的各种基础数据
template<typename Dtype>
struct XRocData : public BaseData {
  Dtype value;
  XRocData() {}
  explicit XRocData(const Dtype& val) {
    value = val;
  }
};

/// 输入数据的自定义参数
class InputParam {
 public:
  explicit InputParam(const std::string &method_name) {
    method_name_ = method_name;
    is_json_format_ = false;
    is_enable_this_method_ = true;
  }
  virtual ~InputParam() = default;
  virtual std::string Format() = 0;

 public:
  bool is_json_format_;
  bool is_enable_this_method_;
  std::string method_name_;
};

typedef std::shared_ptr<InputParam> InputParamPtr;

/**
 * \brief 用于关闭Method的自定义参数
 */
class DisableParam : public InputParam {
 public:
  enum class Mode {
    /// 透传输入数据到输出，要求输入数据大小与输出一致
        PassThrough,
    /// 拷贝先验数据到输出，要求先验数据大小与输出大小一致
        UsePreDefine,
    /// 令每个输出都是INVALID的BaseData
        Invalid,
    /// 按顺序逐个透传输入数据到输出
    /// 如果输入数据大小多于输出，则只透传前面的slot。
    /// 如果输入数据大小少于输出，则多余的输出slot为Invalid的BaseData。
        BestEffortPassThrough
  };
  explicit DisableParam(const std::string &
  method_name, Mode mode = Mode::Invalid)
      : InputParam(method_name), mode_{mode} {
    is_enable_this_method_ = false;
  }
  virtual ~DisableParam() = default;
  virtual std::string Format() {
    return method_name_ + " : disabled";
  }
  Mode mode_;
  /// 先验数据，用于填充Method输出
  std::vector<BaseDataPtr> pre_datas_;
};

typedef std::shared_ptr<DisableParam> DisableParamPtr;

/// json格式的SDK输入参数
class SdkCommParam : public InputParam {
 public:
  SdkCommParam(const std::string &method_name, const std::string &param)
      : InputParam(
      method_name) {
    param_ = param;
    is_json_format_ = true;
  }
  virtual std::string Format() { return param_; }
  virtual ~SdkCommParam() = default;

 public:
  std::string param_;   // json格式
};
typedef std::shared_ptr<SdkCommParam> CommParamPtr;

/// 输入数据类型
struct InputData {
  /// 用户输入的数据，比如图片channel、时间戳、框等等
  std::vector<BaseDataPtr> datas_;
  /// 当前请求自定义的参数
  std::vector<InputParamPtr> params_;
  /// 数据源 id 用于多路输入时区分输入源,单一源情况赋值为 0
  uint32_t source_id_ = 0;
  /// 透传的数据，该数据会透传到OutputData::context_ 字段
  const void *context_ = nullptr;
};
typedef std::shared_ptr<InputData> InputDataPtr;

/// 输出数据类型
struct OutputData {
  /// 错误码
  int error_code_ = 0;
  /// 错误信息
  std::string error_detail_ = "";
  /// 当该OutputData为给某个Method的定向回调结果时，该字段用于指示Method名称
  std::string method_name_ = "";
  /// 多路输出结果名称
  std::string output_type_ = "";
  /// 输出结果
  std::vector<BaseDataPtr> datas_;
  /// 从InputData透传过来的数据
  const void *context_ = nullptr;
  /// 该结果的序列号
  int64_t sequence_id_ = 0;
  /// 该结果是属于那个输入源产生的结果
  uint32_t source_id_ = 0;
  uint64_t global_sequence_id_ = 0;
};
typedef std::shared_ptr<OutputData> OutputDataPtr;

/// 回调函数类型
typedef std::function<void(OutputDataPtr)> XRocCallback;

}  // namespace HobotXRoc

#endif  // HOBOTXSDK_XROC_DATA_H_
