/*
 * @Description: implement of hbipcservice.cpp
 * @Author: yingmin.li@horizon.ai
 * @Date: 2019-08-29 19:24:31
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-10-14 16:17:32
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <utility>

#include "hbipcplugin//hbipcsession.h"

#include "hobotlog/hobotlog.hpp"

#include "xpluginflow/utils/time_helper.h"

namespace horizon {
namespace vision {
namespace xpluginflow {
namespace hbipcplugin {

using std::chrono::milliseconds;

HbipcSession::~HbipcSession() {
  if (is_init_) {
    if (SDeinitConnection() != ERROR_HBIPC_OK) {
      LOGE << "[HbipcPlugin] deinit HbipcSession, error code = "
           << SGetInitErrorCode();
    }
  }
}

std::string HbipcSession::GetStrValue(const std::string &key) const {
  std::lock_guard<std::mutex> lk(mutex_);
  if (x2_cfg_[key].empty()) {
    LOGE << "Can not find key: " << key;
    return "";
  }
  LOGD << "The domain update: " << x2_cfg_[key].asString();
  return x2_cfg_[key].asString();
}

int HbipcSession::GetArray(const std::string &key, int num) {
  std::lock_guard<std::mutex> lk(mutex_);
  int sz = x2_cfg_[key].size();
  if (sz != num) {
    LOGE << "The server id sz is wrong";
    return ERROR_HBIPC_PARSE;
  }
  LOGI << "Server id is ";
  for (int i = 0; i < num; i++) {
    server_id_[i] = x2_cfg_[key][i].asInt();
  }
  return ERROR_HBIPC_OK;
}

int HbipcSession::SConfig(const std::string &path) {
  x2_path_ = path;
  std::ifstream ifs(x2_path_);
  if (!ifs.is_open()) {
    LOGE << "Open config file " << x2_path_ << " failed";
    return ERROR_HBIPC_PATH;
  }
  std::stringstream ss;
  ss << ifs.rdbuf();
  ifs.close();
  std::string content = ss.str();
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  JSONCPP_STRING error;
  std::shared_ptr<Json::CharReader> reader(builder.newCharReader());
  try {
    bool ret = reader->parse(content.c_str(), content.c_str() + content.size(),
                             &x2_cfg_, &error);
    if (ret) {
      // 解析
      domain_name_ = GetStrValue("domain_name");
      GetArray("server_id", 16);
      return ERROR_HBIPC_OK;
    } else {
      return ERROR_HBIPC_PARSE;
    }
  } catch (std::exception &e) {
    return ERROR_HBIPC_PARSE;
  }
}

int HbipcSession::SInitConnection() {
  // 初始化HPIPC系统软件接口
  if ((init_error_code_ = hbipc_cp_init(domain_name_.c_str(), server_id_,
                                        TF_BLOCK, 0, &domain_config_)) < 0) {
    LOGE << "[HbipcPlugin] hbipc_cp_init error: " << init_error_code_;
    return ERROR_HBIPC_CONNECT;
  }

  // 如果成功，返回domain_id；如果失败，返回错误码
  domain_id_ = init_error_code_;
  LOGI << "[HbipcPlugin] domain_id = " << domain_id_;

  if ((init_error_code_ =
           hbipc_cp_accept(domain_id_, server_id_, &connect_, TF_BLOCK)) < 0) {
    LOGE << "[HbipcPlugin] hbipc_cp_accept error: " << init_error_code_;
    hbipc_cp_deinit(domain_id_, server_id_);
    return ERROR_HBIPC_CONNECT;
  }
  LOGI << "[HbipcPlugin] session info: domain_id = " << connect_.domain_id
       << ", "
       << "provider_id = " << connect_.provider_id << ", "
       << "client_id = " << connect_.client_id;

  is_init_ = true;

  return ERROR_HBIPC_OK;
}

int HbipcSession::SDeinitConnection() {
  if (is_init_) {
    if ((init_error_code_ = hbipc_cp_deinit(domain_id_, server_id_)) < 0) {
      LOGE << "[HbipcPlugin] in destructor fun: hbipc_cp_deinit: "
           << init_error_code_;
      return ERROR_HBIPC_UNCONNECT;
    }
  }
  is_init_ = false;
  return ERROR_HBIPC_OK;
}

int HbipcSession::SGetInitErrorCode() const { return init_error_code_; }

int HbipcSession::SGetSendErrorCode() const { return send_error_code_; }

int HbipcSession::SGetRecvErrorCode() const { return recv_error_code_; }

int HbipcSession::SSend(const std::string &proto) {
  const auto proto_size = proto.size();
  /* default 256K */
  if (proto_size > buf_len_max_) {
    LOGE << "[HbipcPlugin] size > BUF_LEN, "
         << "size = " << proto_size / 1024 << "KB";
    return ERROR_HBIPC_SEND;
  }

  std::memcpy(send_proto_buf_, proto.c_str(), proto_size);
  LOGD << "[HbipcPlugin] hbipc_cp_send begin";

  time_point_ = hobot::Timer::tic();

  if ((send_error_code_ =
           hbipc_cp_sendframe(&connect_, send_proto_buf_, proto_size)) < 0) {
    if (send_error_code_ != HBIPC_ERROR_HW_TRANS_ERROR) {
      LOGE << "[HbipcPlugin] hbipc_cp_sendframe error = " << send_error_code_;
    } else {
      LOGE << "[HbipcPlugin] hbipc_cp_sendframe HardWare Trans error";
    }
    return ERROR_HBIPC_SEND;
  } else {
    LOGD << "[HbipcPlugin] hbipc_cp_send end";
    LOGD << "protobuf delay time = " << hobot::Timer::toc(time_point_) << " ms";
    time_point_ = hobot::Timer::tic();
    LOGI << "[HbipcPlugin] send protobuf success, protobuf size = "
         << proto_size / 1024.0 << " KB";
    return ERROR_HBIPC_OK;
  }
}

int HbipcSession::SRecv(std::string *proto) {
  LOGD << "[HbipcPlugin] hbipc_cp recv begin";
  time_point_ = hobot::Timer::tic();
  if ((recv_error_code_ =
           hbipc_cp_recvframe(&connect_, recv_proto_buf_, buf_len_max_)) < 0) {
    if (send_error_code_ != HBIPC_ERROR_HW_TRANS_ERROR) {
      LOGE << "[HbipcPlugin] hbipc_cp_recvframe error = " << recv_error_code_;
    } else {
      LOGE << "[HbipcPlugin] hbipc_cp_recvframe HardWare Trans error";
    }
    return ERROR_HBIPC_SEND;
  } else {
    LOGD << "[HbipcPlugin] hbipc_cp_recv end";
    LOGD << "protobuf delay time = " << hobot::Timer::toc(time_point_) << " ms";
    time_point_ = hobot::Timer::tic();
    LOGI << "[HbipcPlugin] recv protobuf success, protobuf size = "
         << recv_error_code_ / 1024.0 << " KB";
    std::string temp(recv_proto_buf_, recv_error_code_);
    *proto = temp;
    return ERROR_HBIPC_OK;
  }
}

}  // namespace hbipcplugin
}  // namespace xpluginflow
}  // namespace vision
}  // namespace horizon
