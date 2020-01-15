/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: CNNMethodConfig.h
 * @Brief: declaration of the CNNMethodConfig
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-04-15 14:18:28
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-04-15 16:01:54
 */

#ifndef INCLUDE_CNNMETHOD_UTIL_CNNMETHODCONFIG_H_
#define INCLUDE_CNNMETHOD_UTIL_CNNMETHODCONFIG_H_

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "hobotxsdk/xroc_data.h"
#include "json/json.h"

namespace HobotXRoc {

class CNNMethodConfig : public InputParam {
 public:
  explicit CNNMethodConfig(const std::string &json_str = "{}")
      : HobotXRoc::InputParam("CNNMethod") {
    is_enable_this_method_ = true;
    is_json_format_ = true;

    Json::CharReaderBuilder readerBuilder;
    JSONCPP_STRING errs;

    std::unique_ptr<Json::CharReader> jsonReader(readerBuilder.newCharReader());
    jsonReader->parse(
        json_str.c_str(), json_str.c_str() + json_str.length(), &config, &errs);
  }

  explicit CNNMethodConfig(Json::Value &cf)
      : config(cf), HobotXRoc::InputParam("CNNMethod") {
    is_enable_this_method_ = true;
    is_json_format_ = true;
  }

  virtual std::string Format() { return std::move(config.toStyledString()); }

  int32_t GetIntValue(std::string key, int default_value = 0) {
    auto value_js = config[key];
    return value_js.isNull() ? default_value : value_js.asInt();
  }

  bool GetBoolValue(std::string key, bool default_value = false) {
    auto value_int = GetIntValue(key, default_value);
    return value_int == 0 ? false : true;
  }

  float GetFloatValue(std::string key, float default_value = 0.0) {
    auto value_js = config[key];
    return value_js.isNull() ? default_value : value_js.asFloat();
  }

  std::string GetSTDStringValue(std::string key,
                                std::string default_value = "") {
    auto value_js = config[key];
    return value_js.isNull() ? default_value : value_js.asString();
  }

  std::string SetSTDStringValue(std::string key, std::string value) {
    std::string old_value = std::move(GetSTDStringValue(key));
    config[key] = Json::Value(value);
    return old_value;
  }

  std::vector<std::string> GetSTDStringArray(std::string key) {
    auto value_js = config[key];
    std::vector<std::string> ret;
    if (value_js.isNull()) {
      return ret;
    }
    ret.resize(value_js.size());
    for (int i = 0; i < value_js.size(); ++i) {
      ret[i] = value_js[i].asString();
    }
    return ret;
  }

  bool KeyExist(std::string key) {
    auto value_js = config[key];
    return !value_js.isNull();
  }

  CNNMethodConfig GetSubConfig(std::string key) {
    auto value_js = config[key];
    if (value_js.isNull()) {
      return std::move(CNNMethodConfig("{}"));
    }
    return std::move(CNNMethodConfig(value_js));
  }

 public:
  Json::Value config;
};

static void UpdateParams(Json::Value &new_jc, Json::Value &old_jc) {
  Json::Value::Members old_members = old_jc.getMemberNames();
  Json::Value::Members new_members = new_jc.getMemberNames();

  typedef Json::Value::Members::iterator MI;
  for (MI iter = new_members.begin(); iter != new_members.end(); iter++) {
    auto &key = *iter;
    if (old_jc.isMember(key)) {
      if (old_jc[key].type() == Json::ValueType::objectValue
          && new_jc[key].type() == Json::ValueType::objectValue) {
        UpdateParams(new_jc[key], old_jc[key]);
      } else {
        old_jc[key] = new_jc[key];
      }
    } else {
      old_jc[key] = new_jc[key];
    }
  }
}

}  // namespace HobotXRoc
#endif  // INCLUDE_CNNMETHOD_UTIL_CNNMETHODCONFIG_H_
