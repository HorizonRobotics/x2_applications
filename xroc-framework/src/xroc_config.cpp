/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides config for xroc
 * @author    chuanyi.yang
 * @email     chuanyi.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */
#include "hobotxroc/xroc_config.h"
#include <assert.h>
#include <algorithm>
#include <fstream>
#include <map>
#include <string>
#include <vector>
#include "common/com_func.h"
#include "hobotlog/hobotlog.hpp"
#include "hobotxroc/json_key.h"

namespace HobotXRoc {

int XRocConfig::LoadFile(const std::string& file_path) {
  folder_path_ = hobotcommon::get_parent_path(file_path);
  std::ifstream infile(file_path);
  HOBOT_CHECK(infile.good())
      << "error config file path '" << file_path << "', please check it";
  infile >> cfg_jv_;
  infile.close();
  // version
  if (cfg_jv_.isMember("version")) {  // parse config
    Config out;
    parse_config_.folder_path_ = folder_path_;
    if (parse_config_.GetWorkflows(cfg_jv_, out)) {
      return -1;
    } else {
      cfg_jv_ = out;
    }
  }
  return 0;
}

SchedulerConfig::SchedulerConfig(XRocConfigPtr config) {
  config_ = config;
  auto max_count_value = config_->cfg_jv_[kMaxRunCount];
  if (!max_count_value.isNull()) {
    max_running_count_ = max_count_value.asInt();
  }
  optional_config_ = config_->cfg_jv_[kOptional];
  // check if source number config exist
  auto source_number_value = config_->cfg_jv_[kSourceNum];
  if (!source_number_value.isNull() && source_number_value.isInt()) {
    source_num_ =
        static_cast<uint32_t>(std::max(source_number_value.asInt(), 1));
  }
  auto inputs = config_->cfg_jv_[kInputs];
  auto outputs = config_->cfg_jv_[kOutputs];  // json数组
  HOBOT_CHECK(!inputs.isNull() && inputs.size())
      << "sdk input error, please check config input";
  HOBOT_CHECK(!outputs.isNull() && outputs.size())
      << "sdk output error, please check config output";
  for (uint i = 0; i < inputs.size(); ++i) {
    flow_inputs_.push_back(inputs[i].asString());
    // LOG_INFO("flow input:{}", inputs[i].asString().c_str());
  }

  // 判断单路or多路输出
  HOBOT_CHECK(outputs[0].isObject() || outputs[0].isString())
      << "sdk output error, please check config output";
  bool is_multipul_output = true;
  if (outputs[0].isObject()) {  // 判断是否多路输出
    is_multipul_output = true;
  } else if (outputs[0].isString()) {
    is_multipul_output = false;
  }

  // 多路输出flow_outputs_
  if (is_multipul_output) {
    for (uint i = 0; i < outputs.size(); ++i) {
      Json::Value output = outputs[i];
      std::vector<std::string> single_outputs;
      auto single_outputs_type = output[kOutputName].asString();
      HOBOT_CHECK(!single_outputs_type.empty())
          << " output_type for " << i << " output is empty";

      for (uint j = 0; j < output[kOutputs].size(); ++j) {
        std::string output_name = output[kOutputs][j].asString();
        HOBOT_CHECK(!output_name.empty()) << "output slot " << j << " for "
                                          << single_outputs_type << " is empty";
        single_outputs.emplace_back(output_name);

        // 判断是否已有该outputname
        if (std::find(flow_outputs_union_.begin(), flow_outputs_union_.end(),
                      output_name) == flow_outputs_union_.end()) {
          flow_outputs_union_.push_back(output_name);
        }
      }
      flow_outputs_[single_outputs_type] = single_outputs;
    }
  } else {  // 兼容单路输出
    std::vector<std::string> single_outputs;
    for (uint i = 0; i < outputs.size(); ++i) {
      single_outputs.push_back(outputs[i].asString());
      flow_outputs_union_.push_back(outputs[i].asString());
    }
    flow_outputs_[kSingleOutputName] = single_outputs;
  }
  int node_size = config_->cfg_jv_[kWorkflow].size();
  HOBOT_CHECK(node_size) << "sdk node size = 0, please check config";

  for (int i = 0; i < node_size; ++i) {
    Json::Value one_node = config_->cfg_jv_[kWorkflow][i];
    auto node_name = one_node[kMethodName].asString();
    HOBOT_CHECK(!node_name.empty())
        << " unique_name for " << i << " node is empty";
    nodes_names_.push_back(node_name);
    // adjust method config path
    if (one_node.isMember(kMethodCfgPath) &&
        one_node[kMethodCfgPath].isString()) {
      auto new_path =
          config_->folder_path_ + one_node[kMethodCfgPath].asString();
      one_node[kMethodCfgPath] = new_path;
    }
    nodes_config_[node_name] = one_node;

    auto node_inputs_json = one_node[kInputs];
    auto node_outputs_json = one_node[kOutputs];
    HOBOT_CHECK(!node_inputs_json.isNull() && node_inputs_json.size())
        << node_name << " input error, please check config input";
    HOBOT_CHECK(!node_outputs_json.isNull() && node_outputs_json.size())
        << node_name << " input error, please check config input";
    std::vector<std::string> node_inputs;
    std::vector<std::string> node_outputs;
    for (uint j = 0; j < node_inputs_json.size(); ++j) {
      auto input_name = node_inputs_json[j].asString();
      HOBOT_CHECK(!input_name.empty())
          << " input slot " << j << " for " << node_name << " is empty";
      node_inputs.emplace_back(input_name);
    }

    for (uint j = 0; j < node_outputs_json.size(); ++j) {
      auto output_name = node_outputs_json[j].asString();
      HOBOT_CHECK(!output_name.empty())
          << "output slot " << j << " for " << node_name << " is empty";
      node_outputs.emplace_back(output_name);
    }

    nodes_inputs_[node_name] = node_inputs;
    nodes_outputs_[node_name] = node_outputs;
  }
}

int SchedulerConfig::CheckInputValid() {
  //  检查node的input是否得到feed
  std::vector<std::string>::iterator it;
  std::vector<std::string>::iterator pos;
  for (it = nodes_names_.begin(); it != nodes_names_.end(); it++) {
    std::vector<std::string>::iterator input_name;
    for (input_name = nodes_inputs_[*it].begin();
         input_name != nodes_inputs_[*it].end(); input_name++) {
      std::vector<std::string>::iterator it2;
      bool find_flag = false;
      for (it2 = nodes_names_.begin(); it2 != it; it2++) {
        pos = find(nodes_outputs_[*it2].begin(), nodes_outputs_[*it2].end(),
                   *input_name);
        if (pos != nodes_outputs_[*it2].end()) {
          find_flag = true;
          break;
        }
      }
      if ((pos = find(flow_inputs_.begin(), flow_inputs_.end(), *input_name)) !=
          flow_inputs_.end()) {
        find_flag = true;
      }
      if (find_flag != true) {
        LOGE << "input name " << *input_name << " find failed" << std::endl;
        return -1;
      }
    }
  }
  return 0;
}

//  debug
/*
int SchedulerConfig::CheckOutputRedundant() {
  //  检查是否有冗余output,只提示warning
  //  通过检查nodex之后的node是否以nodex的output作为input
  std::map<std::string, std::vector<std::string>>::iterator it;
  for (it = nodes_outputs_.begin(); it != nodes_outputs_.end(); it++) {
      std::vector<std::string>::iterator output_name;
      for (output_name = it->second.begin();
        output_name != it->second.end(); output_name++) {
        std::map<std::string, std::vector<std::string>>::iterator it2;
        bool find_flag = false;
        it2 = nodes_inputs_.begin();
        while (it2->first != it->first) {
            it2++;
        }
        it2++;
        for ( ; it2 != nodes_inputs_.end(); it2++) {
           std::vector<std::string>::iterator pos = find(
                    it2->second.begin(),
                    it2->second.end(), *output_name);
                if (pos != it2->second.end()) {
                   find_flag = true;
                   break;
            }
        }
        if (find_flag != true) {
            LOGE << "output name "
            << *output_name << " find failed" << std::endl;
            return -1;
        }
      }
  }
  return 0;
}
*/

int SchedulerConfig::CheckUniqueName() {
  //  检查是否有重复node name
  std::unordered_set<std::string> flow_method_name;
  std::unordered_set<std::string>::iterator set_iter;
  std::vector<std::string>::iterator vect_iter;

  for (vect_iter = nodes_names_.begin(); vect_iter != nodes_names_.end();
       vect_iter++) {
    set_iter = flow_method_name.find(*vect_iter);
    if (set_iter != flow_method_name.end()) {
      LOGE << "node name " << *vect_iter << " repeated!" << std::endl;
      return -1;
    } else {
      flow_method_name.insert(*vect_iter);
    }
  }
  return 0;
}

int SchedulerConfig::CheckIsCircle() {
  //  检查workflow是否有环路
  std::vector<std::string>::iterator output_node_name;
  std::vector<std::string>::iterator input_node_name;
  for (output_node_name = nodes_names_.begin();
       output_node_name != nodes_names_.end(); output_node_name++) {
    std::vector<std::string>::iterator output_data;
    for (output_data = nodes_outputs_[*output_node_name].begin();
         output_data != nodes_outputs_[*output_node_name].end();
         output_data++) {
      for (input_node_name = nodes_names_.begin();
           input_node_name != output_node_name; input_node_name++) {
        std::vector<std::string>::iterator pos =
            find(nodes_inputs_[*input_node_name].begin(),
                 nodes_inputs_[*input_node_name].end(), *output_data);
        if (pos != nodes_inputs_[*input_node_name].end()) {
          LOGE << "node " << *input_node_name << " and node "
               << *output_node_name << " is circle!!" << std::endl;
          return -1;
        }
      }
    }
  }
  return 0;
}

int SchedulerConfig::CheckRepeatedOutput() {
  //  检查多个node输出到相同的slot
  std::map<std::string, std::vector<std::string>>::iterator output_iter;
  std::unordered_set<std::string> total_output_names;
  std::unordered_set<std::string>::iterator set_iter;

  for (output_iter = nodes_outputs_.begin();
       output_iter != nodes_outputs_.end(); output_iter++) {
    std::vector<std::string>::iterator output_name;
    for (output_name = output_iter->second.begin();
         output_name != output_iter->second.end(); output_name++) {
      set_iter = total_output_names.find(*output_name);
      if (set_iter != total_output_names.end()) {
        LOGE << "output " << *output_name << " is repeated!!" << std::endl;
        return -1;
      } else {
        total_output_names.insert(*output_name);
      }
    }
  }
  return 0;
}

// function: load json file's content, used in @include
int ParseConfig::LoadContent(const std::string& file_path,
                             Json::Value& config) {
  std::ifstream infile(file_path);
  if (!infile.good()) {
    LOGE << "error config file path '" << file_path << "', please check";
    return PARSE_CONFIG_ERROR::FILE_PATH_ERROR;
  }
  infile >> config;
  return 0;
}
/* Example From
 * {
 *   "type": "workflow",
 *   "inputs": ["${image}", "${bbox}"],
 *   "outputs": ["${cnn_out0}", "${cnn_out1}"],
 *   ...
 * } To:
 * {
 *   "type": "workflow",
 *   "inputs": ["${image}", "${bbox}"],
 *   "__arg_inputs__": ["image", "bbox"],
 *   "outputs": ["${cnn_out0}", "${cnn_out1}"],
 *   "__arg_outputs__": ["cnn_out0", "cnn_out1"],
 *   ...
 * }
 */
int ParseConfig::GenerateWorkFlowTplInternalArgList(
    Json::Value& template_conf) {
  Json::Value arg_inputs, arg_outputs;
  arg_inputs.append(Json::Value::null);
  arg_inputs.clear();
  arg_outputs.append(Json::Value::null);
  arg_outputs.clear();

  for (auto input : template_conf["inputs"]) {
    // 剥离出 input name
    std::string name = input.asString().substr(2);  // ${name} ->name}
    name = name.substr(0, name.size() - 1);         // name} -> name
    arg_inputs.append(name);
  }
  template_conf["__arg_inputs__"] = arg_inputs;

  for (auto output : template_conf["outputs"]) {
    // 剥离出 output name
    std::string name = output.asString().substr(2);  // ${name} ->name}
    name = name.substr(0, name.size() - 1);          // name} -> name
    arg_outputs.append(name);
  }
  template_conf["__arg_outputs__"] = arg_outputs;

  return 0;
}

bool ParseConfig::IsWorkflowObject(const Json::Value& config) {
  if (config.isObject() &&
      config.isMember("type") &&
      config["type"].asString() == "workflow" &&
      config.isMember("workflow") &&
      config["workflow"].isArray() &&
      config.isMember("inputs") &&
      config["inputs"].isArray() &&
      config.isMember("outputs") &&
      config["outputs"].isArray() &&
      config.isMember("name") &&
      config["name"].isString()) {
    LOGI << "workflow:" << config["name"].asString() << std::endl;
    return true;
  }
  return false;
}

// function：template_ref, template -> template_instance
int ParseConfig::GetTemplateInstance(Json::Value& template_conf,
                                     const Json::Value& template_ref_conf) {
  int ret = 0;  // return code
  switch (template_conf.type()) {
    case Json::objectValue:  // json
    {
      if (IsWorkflowObject(template_conf)) {
        GenerateWorkFlowTplInternalArgList(template_conf);
      }
      Json::Value::Members mem = template_conf.getMemberNames();
      for (auto iter = mem.begin(); iter != mem.end(); iter++) {
        // // 1.1 key是inputs, outputs,且需要拼接的
        // if ((*iter == "inputs" || *iter == "outputs") &&
        //     template_conf[*iter].isArray()) {  // value是[]类型才需要拼接
        //   for (uint i = 0; i < template_conf[*iter].size(); i++) {
        //     template_conf[*iter][i] = pre_name +
        //                               template_conf[*iter][i].asString();
        //   }
        // }
        // 1.2 递归替换param
        if ((ret = GetTemplateInstance(template_conf[*iter],
                                       template_ref_conf)) != 0)
          return ret;
      }
      break;
    }
    case Json::arrayValue:  // array
      for (uint i = 0; i < template_conf.size(); i++) {
        if ((ret = GetTemplateInstance(template_conf[i], template_ref_conf)) !=
            0)
          return ret;
      }
      break;
    case Json::stringValue: {
      std::string value = template_conf.asString();
      if (value.size() > 3 && value[0] == '$') {  // 需转换的参数
        // 替换parameter
        std::string name = value.substr(2);      // ${name} ->name}
        name = name.substr(0, name.size() - 1);  // name} -> name
        // 判断该参数是否存在
        if (!template_ref_conf["parameters"].isObject() ||
            !template_ref_conf["parameters"].isMember(name)) {
          LOGE << "parameter: " << name << " not exits in template_ref"
               << std::endl;
          return PARSE_CONFIG_ERROR::PARAMETER_NAME_NOT_EXIST_ERROR;
        }
        template_conf = template_ref_conf["parameters"][name];
      }
      break;
    }
    default:
      break;
  }
  return 0;
}

// function: check parameters of template_ref and template
bool ParseConfig::CheckTemplateParameters(
    const Json::Value& template_conf, const Json::Value& template_ref_conf) {
  // 0. 检查template和template_ref格式是否正确
  if (!template_conf.isObject() || !template_conf.isMember("type") ||
      !template_conf.isMember("template_name") ||
      !template_conf.isMember("parameters") ||
      !template_conf.isMember("template") || !template_ref_conf.isObject() ||
      !template_conf.isMember("type") ||
      !template_conf.isMember("template_name") ||
      !template_conf.isMember("parameters")) {
    return false;
  }
  // 1. 检查parameters的size是否一致
  if (template_conf["parameters"].size() !=
      template_ref_conf["parameters"].size()) {
    LOGE << "Parameters'size incorrect" << std::endl;
    return false;
  }
  // 2. 检查parameters的type是否正确
  if (template_conf["parameters"].type() != Json::arrayValue ||
      template_ref_conf["parameters"].type() != Json::objectValue) {
    LOGE << "parameters'format incorrect" << std::endl;
    return false;
  }
  // 3. 检查parameters的name是否一致
  for (uint i = 0; i < template_conf["parameters"].size(); i++) {
    if (!template_conf["parameters"][i].isString()) {
      LOGE << "parameters'type incorrect" << std::endl;
      break;
    }
    std::string parameter_name = template_conf["parameters"][i].asString();
    if (!template_ref_conf["parameters"].isMember(parameter_name)) {
      LOGE << "parameters'name incorrect" << std::endl;
      break;
    }
    return true;
  }
  return false;
}

// function: create map of templates
int ParseConfig::GetTemplatesMap(
    Json::Value& templates_array,
    std::map<std::string, Json::Value>& templates) {
  int ret = 0;
  if (templates_array.isNull())  // no templates in root
    return 0;
  if (!templates_array.isArray()) {
    LOGE << "templates'format incorrect, please check" << std::endl;
    return PARSE_CONFIG_ERROR::TEMPLATES_FORMAT_ERROR;
  }
  // 1. @include展开
  for (uint i = 0; i < templates_array.size(); i++) {
    if (templates_array[i].isString()) {
      std::string value = templates_array[i].asString();
      if (!(value.size() > 9 && value.substr(0, 9) == "@include:")) {
        LOGE << "invalid format for include template-definition file";
        return PARSE_CONFIG_ERROR::TEMPLATES_FORMAT_ERROR;
      }
      std::string file_path = value.substr(9);
      {
        // 去除路径前后空格
        if (file_path.empty()) {
          LOGE << "No path after @include" << std::endl;
          return PARSE_CONFIG_ERROR::INCLUDE_EMPTY_FILEPATH_ERROR;
        }
        file_path.erase(0, file_path.find_first_not_of(" "));
        file_path.erase(file_path.find_last_not_of(" ") + 1);
        // 拼接folder路径
        file_path = folder_path_ + file_path;
      }
      // 替换该参数
      Json::Value file_content;
      if ((ret = LoadContent(file_path, file_content)) != 0) {
          LOGE << "Load content of " << file_path
               << " failed, please check file_path" << std::endl;
          return ret;
        }
      templates_array[i] = file_content;
    }
  }

  // 2. 检查是否是template模板
  for (uint i = 0; i < templates_array.size(); i++) {
    if (!templates_array[i].isObject() ||
        !templates_array[i].isMember("type") ||
        !templates_array[i].isMember("template_name") ||
        !templates_array[i].isMember("parameters") ||
        !templates_array[i].isMember("template")) {  // 检查规定的对象
      LOGE << "templates'format incorrect" << std::endl;
      return PARSE_CONFIG_ERROR::TEMPLATES_FORMAT_ERROR;
    }
    if (!templates_array[i]["type"].isString() ||
        templates_array[i]["type"].asString() != "template") {
      LOGE << "The \"type\" in templates incorrect" << std::endl;
      return PARSE_CONFIG_ERROR::TEMPLATES_FORMAT_ERROR;
    }
    if (!templates_array[i]["template_name"].isString() ||
        templates.count(templates_array[i]["template_name"].asString()) > 0) {
      LOGE << "The \"template_name\" in templates repeat" << std::endl;
      return PARSE_CONFIG_ERROR::TEMPLATES_NAME_REPEATED_ERROR;
    }
    std::string tpl_name = templates_array[i]["template_name"].asString();
    templates[tpl_name] = templates_array[i];
  }
  return 0;
}

bool ParseConfig::IsTemplateRefObject(const Json::Value& config) {
    if (config.isObject() && config.isMember("type") &&
        config["type"].asString() == "template_ref" &&
        config.isMember("template_name") &&
        config["template_name"].isString() &&
        config.isMember("parameters") &&
        config["parameters"].isObject()
  ) {
    LOGI << "template_ref:" << config["template_ref"].asString()
              << std::endl;
    return true;
  }
  return false;
}
// function：parse workflows
int ParseConfig::ParseWorkflows(Json::Value& workflows,
                                std::map<std::string, Json::Value>& templates,
                                bool& repeat_parsing) {
  int ret = 0;
  switch (workflows.type()) {
    case Json::objectValue:  // json
    {
      if (IsTemplateRefObject(workflows)) {
        std::string template_key = workflows["template_name"].asString();
        if (templates.count(template_key) == 0) {
          LOGE << "Cannot find template: " << template_key << std::endl;
          return PARSE_CONFIG_ERROR::TEMPLATES_NAME_INVALID_ERROR;
        }
        Json::Value template_tmp = templates[template_key];
        if (!CheckTemplateParameters(template_tmp, workflows)) {
          return PARSE_CONFIG_ERROR::TEMPLATEREF_NOT_CORRESPOND_ERROR;
        }
        if ((ret = GetTemplateInstance(template_tmp, workflows)) != 0) {
          return ret;
        }
        // replace template_ref为template_instance
        workflows = template_tmp["template"];
        repeat_parsing = true;
        return 0;
      }
      // not template_ref object,traverse.
      Json::Value::Members mem = workflows.getMemberNames();  // 广度优先搜索
      for (auto iter = mem.begin(); iter != mem.end(); iter++) {
        if ((ret = ParseWorkflows(workflows[*iter],
                                  templates,
                                  repeat_parsing)) != 0)
        return ret;
      }
      break;
    }
    case Json::arrayValue: {  // array
      for (uint i = 0; i < workflows.size(); i++) {
        if ((ret = ParseWorkflows(workflows[i],
                                  templates,
                                  repeat_parsing)) != 0) return ret;
      }
      break;
    }
    case Json::stringValue: {
      std::string value = workflows.asString();
      if (value.size() > 9 && value.substr(0, 9) == "@include:") {  // @include:
        std::string file_path = value.substr(9);
        {
          // 去除路径前后空格
          if (file_path.empty()) {
            LOGE << "No path after @include" << std::endl;
            return PARSE_CONFIG_ERROR::INCLUDE_EMPTY_FILEPATH_ERROR;
          }
          file_path.erase(0, file_path.find_first_not_of(" "));
          file_path.erase(file_path.find_last_not_of(" ") + 1);
          // 拼接folder路径
          file_path = folder_path_ + file_path;
        }
        // 替换该参数
        Json::Value file_content;
        if ((ret = LoadContent(file_path, file_content)) != 0) {
          LOGE << "Load content of " << file_path
               << " failed, please check file_path" << std::endl;
          return ret;
        }
        workflows = file_content;
        repeat_parsing = true;
      }
      break;
    }
    default:
      break;
  }
  return 0;
}

void ParseConfig::UpdateSlotsName(const Json::Value& workflow_slots,
                                  const Json::Value& workflow_argslots,
                                  std::string& path, Json::Value& node_slots) {
  uint i = 0, j = 0;
  for (i = 0; i < node_slots.size(); i++) {
    std::string node_slot = node_slots[i].asString();
    bool is_inner_slot = true;
    for (j = 0; j < workflow_argslots.size(); j++) {
      std::string workflow_slot = workflow_argslots[j].asString();
      if (node_slot == workflow_slot) {
        is_inner_slot = false;
        break;
      }
    }  // end workflow for
    if (is_inner_slot) {
      node_slots[i] = path + "_" + node_slot;
    } else {
      node_slots[i] = workflow_slots[j];
    }
  }  // end node for
}

void ParseConfig::GetSubNodes(Json::Value& root, Json::Value& out,
                              std::string path,
                              const Json::Value& workflow_inputs,
                              const Json::Value& workflow_outputs,
                              const Json::Value& workflow_arginputs,
                              const Json::Value& workflow_argoutputs) {
  int size = root["workflow"].size();

  for (int i = 0; i < size; i++) {
    Json::Value& sub_workflow = root["workflow"][i];
    std::string type = sub_workflow["type"].asString();
    if (type.compare("workflow") == 0) {
      std::string name = sub_workflow["name"].asString();
      std::string workflow_path("");
      workflow_path = path + "_" + name;
      Json::Value& node_inputs = sub_workflow["inputs"];
      Json::Value& node_outputs = sub_workflow["outputs"];
      UpdateSlotsName(workflow_inputs, workflow_arginputs, path, node_inputs);
      UpdateSlotsName(workflow_outputs, workflow_argoutputs, path,
                      node_outputs);
      GetSubNodes(sub_workflow, out, workflow_path, sub_workflow["inputs"],
                  sub_workflow["outputs"], sub_workflow["__arg_inputs__"],
                  sub_workflow["__arg_outputs__"]);

    } else if (type.compare("node") == 0) {
      std::string unique_name = sub_workflow["unique_name"].asString();
      std::string name = path + "_" + unique_name;
      sub_workflow["unique_name"] = name;
      sub_workflow.removeMember("type");

      // 拼接内部slots
      Json::Value& node_inputs = sub_workflow["inputs"];
      Json::Value& node_outputs = sub_workflow["outputs"];
      UpdateSlotsName(workflow_inputs, workflow_arginputs, path, node_inputs);
      UpdateSlotsName(workflow_outputs, workflow_argoutputs, path,
                      node_outputs);

      out.append(sub_workflow);
    }
  }
}

void ParseConfig::GetNodes(Json::Value& root, Json::Value& output) {
  int size = root["workflows"].size();
  Json::Value nodes;
  for (int i = 0; i < size; i++) {
    std::string name = root["workflows"][i]["name"].asString();
    if (name != "main") {
      continue;
    }
    Json::Value& root_workflow = root["workflows"][i];
    std::string type = root["workflows"][i]["type"].asString();
    HOBOT_CHECK(type == "workflow" && name == "main");
    for (int j = 0; j < root_workflow["workflow"].size(); j++) {
      Json::Value& sub_workflow = root_workflow["workflow"][j];

      std::string sub_type = sub_workflow["type"].asString();
      if (sub_type == "workflow") {
        std::string sub_name = sub_workflow["name"].asString();
        GetSubNodes(sub_workflow, nodes, sub_name,
          sub_workflow["inputs"], sub_workflow["outputs"],
          sub_workflow["__arg_inputs__"], sub_workflow["__arg_outputs__"]);
      } else if (sub_type == "node") {
        sub_workflow.removeMember("type");
        nodes.append(sub_workflow);
      }
    }
    root_workflow.removeMember("type");
    root_workflow["workflow"] = nodes;
    output = root_workflow;
  }
}

int ParseConfig::GetWorkflows(Json::Value& config, Json::Value& out) {
  int ret = 0;
  // 获取templates map
  std::map<std::string, Json::Value> templates_map;
  if ((ret = GetTemplatesMap(config["templates"], templates_map)) != 0)
    return ret;

  // 获取遍历workflows
  bool repeat_parsing = true;
  for (int level = 0; repeat_parsing && level < 10; level++) {
    LOGI << "parsing workflow level:" << level << std::endl;
    repeat_parsing = false;
    if ((ret = ParseWorkflows(config["workflows"],
                              templates_map,
                              repeat_parsing)) != 0) {
      return ret;
    }
  }
  if (repeat_parsing) {
    LOGE << "parsing workflow 10 times, still not ending" << std::endl;
    return PARSE_CONFIG_ERROR::EXCEED_MAX_NESTED_LAYER_ERROR;
  }

  // 获取Node
  GetNodes(config, out);
  return 0;
}
}  // namespace HobotXRoc
