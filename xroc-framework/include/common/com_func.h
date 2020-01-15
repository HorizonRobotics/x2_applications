/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     common functions
 * @author    jianbo.qin
 * @email     jianbo.qin@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.22
 */

#ifndef COMMON_COM_FUNC_H_
#define COMMON_COM_FUNC_H_

#include <string>

#include "json/json.h"

namespace hobotcommon {
bool parse_json(const std::string &in_str, Json::Value &out_jv);

std::string generate_token(const std::string &info);

std::string calc_md5(const std::string &info1, const std::string &info2,
                     const std::string &info3);

time_t conver_time(const char *szTime);

std::string base64_encode(unsigned char const *bytes_to_encode,
                          unsigned int in_len);

std::string base64_decode(std::string const &encoded_string);

unsigned int bkdr_hash(const char *str, unsigned int m = 1999);

unsigned int easy_hash(const char *str);

int64_t getMilliSecond();

void getmac(std::string *outmac);

std::string generate_process_name();

std::string generate_id(const std::string &prefix, int64_t time_stamp,
                        const std::string &device_id, int64_t track_id);

std::string get_parent_path(const std::string& path);
}  // namespace hobotcommon
#endif  // COMMON_COM_FUNC_H_
