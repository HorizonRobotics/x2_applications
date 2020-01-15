/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-22 20:00:29
 * @Version: v0.0.2
 * @Brief: load smart json config file
 * @Note: get from fasterrcnn config.h
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-08-22 23:45:43
 */

#ifndef INCLUDE_SMARTPLUGIN_SMART_CONFIG_H_
#define INCLUDE_SMARTPLUGIN_SMART_CONFIG_H_
#include <string.h>
#include <memory>
#include <string>
#include <vector>
#include "json/json.h"

namespace horizon {
namespace vision {
namespace xpluginflow {
namespace smartplugin {

class JsonConfigWrapper {
 public:
  explicit JsonConfigWrapper(Json::Value config) : config_(config) {}

  int GetIntValue(std::string key, int default_value = 0) {
    auto value_js = config_[key.c_str()];
    if (value_js.isNull()) {
      return default_value;
    }
    return value_js.asInt();
  }

  bool GetBoolValue(std::string key, bool default_value = false) {
    auto value_int = GetIntValue(key, default_value);
    return value_int == 0 ? false : true;
  }

  float GetFloatValue(std::string key, float default_value = 0.0) {
    auto value_js = config_[key.c_str()];
    if (value_js.isNull()) {
      return default_value;
    }
    return value_js.asFloat();
  }

  std::string GetSTDStringValue(std::string key,
                                std::string default_value = "") {
    auto value_js = config_[key.c_str()];
    if (value_js.isNull()) {
      return default_value;
    }
    return value_js.asString();
  }

  std::vector<std::string> GetSTDStringArray(std::string key) {
    auto value_js = config_[key.c_str()];
    std::vector<std::string> ret;
    if (value_js.isNull()) {
      return ret;
    }
    ret.resize(value_js.size());
    for (Json::ArrayIndex i = 0; i < value_js.size(); ++i) {
      ret[i] = value_js[i].asString();
    }
    return ret;
  }

  std::shared_ptr<JsonConfigWrapper> GetSubConfig(std::string key) {
    auto value_js = config_[key.c_str()];
    if (value_js.isNull()) {
      return nullptr;
    }
    return std::shared_ptr<JsonConfigWrapper>(new JsonConfigWrapper(value_js));
  }

 protected:
  Json::Value config_;
};
}  // namespace smartplugin
}  // namespace xpluginflow
}  // namespace vision
}  // namespace horizon
#endif  // INCLUDE_SMARTPLUGIN_SMART_CONFIG_H_
