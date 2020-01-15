/*
 * @Description: implement of hbipcservice.h
 * @Author: yingmin.li@horizon.ai
 * @Date: 2019-08-29 19:24:47
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-10-14 16:16:11
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */

#ifndef INCLUDE_HBIPCPLUGIN_HBIPCSESSION_H_
#define INCLUDE_HBIPCPLUGIN_HBIPCSESSION_H_
#include <chrono>
#include <mutex>
#include <string>

#include "json/json.h"

#include "hbipc_cp/hbipc_cp.h"
#include "hbipc_cp/hbipc_errno.h"

#include "xpluginflow/utils/singleton.h"

namespace horizon {
namespace vision {
namespace xpluginflow {
namespace hbipcplugin {

enum HbipcErrorCode {
  ERROR_HBIPC_OK = 0,
  ERROR_HBIPC_INIT = 1000,
  ERROR_HBIPC_UNINIT = 1001,
  ERROR_HBIPC_CONNECT = 1002,
  ERROR_HBIPC_UNCONNECT = 1003,
  ERROR_HBIPC_SEND = 1004,
  ERROR_HBIPC_RECV = 1005,
  ERROR_HBIPC_PATH = 1006,
  ERROR_HBIPC_PARSE = 1007,
};

class HbipcSession : public hobot::CSingleton<HbipcSession> {
 public:
  HbipcSession() = default;
  ~HbipcSession();

 public:
  // 配置接口，可以分别配置plugin层与system层
  int SConfig(const std::string &path);

 public:
  // 调用系统软件接口的封装函数
  int SInitConnection(void);
  int SDeinitConnection(void);
  int SGetInitErrorCode() const;
  int SGetSendErrorCode() const;
  int SGetRecvErrorCode() const;
  int SSend(const std::string &proto);
  int SRecv(std::string *proto);

 private:
  std::string GetStrValue(const std::string &key) const;
  int GetArray(const std::string &key, int num);

 private:
  std::string x2_path_;
  Json::Value x2_cfg_;
  mutable std::mutex mutex_;
  int init_error_code_ = 0;
  int send_error_code_ = 0;
  int recv_error_code_ = 0;
  int domain_id_ = -1;
  bool is_init_ = false;
  session connect_ = {0, 0, 0};
  // 256KB
  static const int buf_len_max_ = 256 * 1024;
  char send_proto_buf_[buf_len_max_] = {'0'};
  char recv_proto_buf_[buf_len_max_] = {'0'};
  // X2BIF001 or X2SD001
  std::string domain_name_ = "X2SD001";
  uuid server_id_ = {0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7,
                     0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf};
  struct domain_configuration domain_config_ = {
      2,
      {{const_cast<char *>("X2BIF001"), 0, const_cast<char *>("/dev/x2_bif")},
       {const_cast<char *>("X2SD001"), 1, const_cast<char *>("/dev/x2_sd")}}};
  std::chrono::time_point<std::chrono::system_clock> time_point_;
};

}  // namespace hbipcplugin
}  // namespace xpluginflow
}  // namespace vision
}  // namespace horizon
#endif  // INCLUDE_HBIPCPLUGIN_HBIPCSESSION_H_
