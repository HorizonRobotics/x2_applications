/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     common functions
 * @author    jianbo.qin
 * @email     jianbo.qin@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.22
 */
#include "com_func.h"
#include <sys/syscall.h>
#include <string>
//#include <openssl/md5.h>
#include "net/if.h"
#include "sys/ioctl.h"
#include <string.h>
#include <memory>
#include <unistd.h>
#include <sstream>

namespace hobotcommon {
static const int MAXINTERFACES = 32;
static const char *kInterLoop = "000000000000";
static const int kMacLen = strlen(kInterLoop);

bool parse_json(const std::string &in_str, Json::Value &out_jv) {
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  JSONCPP_STRING error;
  std::shared_ptr<Json::CharReader> json_reader(builder.newCharReader());
  try {
    bool ok = json_reader->parse(in_str.c_str(), in_str.c_str() + in_str.size(),
                                 &out_jv, &error);
    if (ok && out_jv.isObject()) {
      return true;
    }
  } catch (std::exception &e) {
    //LOG_WARN("invaid json string");
  }
  return false;
}

pid_t get_tid() { return syscall(SYS_gettid); }

static std::string k_ip_pid;  // NOLINT

std::string generate_token(const std::string &info) {
  if (k_ip_pid.empty()) {
    k_ip_pid = std::to_string(get_tid());
  }
  std::string tok = k_ip_pid;
  if (!info.empty()) {
    tok += info;
  }
  return tok;
}

//    std::string calc_md5(const std::string &info1, const std::string &info2,
//    const std::string &info3){
//        std::string md5;
//        MD5_CTX ctx;
//        unsigned char md[16] = {0};
//        MD5_Init(&ctx);
//        if (!info1.empty()) {
//            MD5_Update(&ctx, info1.c_str(), info1.length());
//        }
//        if (!info2.empty()) {
//            MD5_Update(&ctx, info2.c_str(), info2.length());
//        }
//        if (!info3.empty()) {
//            MD5_Update(&ctx, info3.c_str(), info3.length());
//        }
//        MD5_Final (md, &ctx);
//        char buf[33] = {0};
//        for(int i = 0; i < sizeof(md)/sizeof(unsigned char); i++) {
//            sprintf(buf+2*i ,"%02X", md[i]);
//        }
//        return std::string(buf, 32);
//    }

time_t conver_time(const char *szTime) {
  tm tm_;
  time_t t_;
  strptime(szTime, "%Y%m%d%H%M%S", &tm_);  //将字符串转换为tm时间
  // tm_.tm_isdst = -1;
  t_ = mktime(&tm_);  //将tm时间转换为秒时间
  return t_;
}

static const std::string base64_chars =  // NOLINT
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

static inline bool is_base64(unsigned char c) {
  return (isalnum(c) || (c == '+') || (c == '/'));
}

std::string base64_encode(unsigned char const *bytes_to_encode,
                          unsigned int in_len) {
  std::string ret;
  int i = 0;
  int j = 0;
  unsigned char char_array_3[3];
  unsigned char char_array_4[4];

  while (in_len--) {
    char_array_3[i++] = *(bytes_to_encode++);
    if (i == 3) {
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] =
          ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] =
          ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;

      for (i = 0; (i < 4); i++) ret += base64_chars[char_array_4[i]];
      i = 0;
    }
  }

  if (i) {
    for (j = i; j < 3; j++) char_array_3[j] = '\0';

    char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] =
        ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] =
        ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
    char_array_4[3] = char_array_3[2] & 0x3f;

    for (j = 0; (j < i + 1); j++) ret += base64_chars[char_array_4[j]];

    while ((i++ < 3)) ret += '=';
  }

  return ret;
}

std::string base64_decode(std::string const &encoded_string) {
  int in_len = encoded_string.size();
  int i = 0;
  int j = 0;
  int in_ = 0;
  unsigned char char_array_4[4], char_array_3[3];
  std::string ret;
  ret.reserve(encoded_string.length());

  while (in_len-- && (encoded_string[in_] != '=') &&
         is_base64(encoded_string[in_])) {
    char_array_4[i++] = encoded_string[in_];
    in_++;
    if (i == 4) {
      for (i = 0; i < 4; i++)
        char_array_4[i] = base64_chars.find(char_array_4[i]);

      char_array_3[0] =
          (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
      char_array_3[1] =
          ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
      char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

      for (i = 0; (i < 3); i++) ret += char_array_3[i];
      i = 0;
    }
  }

  if (i) {
    for (j = i; j < 4; j++) char_array_4[j] = 0;

    for (j = 0; j < 4; j++)
      char_array_4[j] = base64_chars.find(char_array_4[j]);

    char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
    char_array_3[1] =
        ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
    char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

    for (j = 0; (j < i - 1); j++) ret += char_array_3[j];
  }

  return ret;
}

unsigned int bkdr_hash(const char *str, unsigned int m) {
  unsigned int seed = 131;
  unsigned int hash = 0;
  while (*str) {
    hash = hash * seed + (*str++);
  }
  return hash % m;
}

unsigned int easy_hash(const char *str) {
  unsigned int hash = 0;
  while (*str) {
    hash = hash + (*str++);
  }
  return hash;
}

int64_t getMilliSecond() {
  struct timespec ts = {0, 0};
  clock_gettime(CLOCK_REALTIME, &ts);
  return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

void getmac(std::string *outmac) {
  int fd = 0;
  int interface = 0;
  struct ifreq buf[MAXINTERFACES];
  struct ifconf ifc;
  char mac[32] = {0};
  if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) >= 0) {
    int i = 0;
    ifc.ifc_len = sizeof(buf);
    ifc.ifc_buf = (caddr_t)buf;
    if (!ioctl(fd, SIOCGIFCONF, reinterpret_cast<char *>(&ifc))) {
      interface = ifc.ifc_len / sizeof(struct ifreq);
      while (i < interface) {
        if (!(ioctl(fd, SIOCGIFHWADDR, reinterpret_cast<char *>(&buf[i])))) {
          snprintf(mac, sizeof(mac), "%02X%02X%02X%02X%02X%02X",
                  (unsigned char)buf[i].ifr_hwaddr.sa_data[0],
                  (unsigned char)buf[i].ifr_hwaddr.sa_data[1],
                  (unsigned char)buf[i].ifr_hwaddr.sa_data[2],
                  (unsigned char)buf[i].ifr_hwaddr.sa_data[3],
                  (unsigned char)buf[i].ifr_hwaddr.sa_data[4],
                  (unsigned char)buf[i].ifr_hwaddr.sa_data[5]);
          if (strncmp(mac, kInterLoop, kMacLen) != 0) {
            *outmac += std::string(mac);
          }
        }
        i++;
      }
    }
  } else {
    //LOG_INFO("[get mac] wrong");
  }
}

std::string generate_process_name() {
  std::string mac;
  getmac(&mac);
  if (mac.size() > 12) {
    mac = mac.substr(0, 12);
  }
  char pid[10];
  snprintf(pid, sizeof(pid), "%06d", getpid());
  std::string pname = mac + pid;
  return pname;
}

std::string generate_id(const std::string &prefix, int64_t time_stamp,
                        const std::string &device_id, int64_t track_id) {
  std::stringstream ss;
  ss << prefix << device_id << "_" << track_id << "_" << time_stamp;
  return ss.str();
}

std::string get_parent_path(const std::string& path) {
  auto pos = path.rfind('/');
  if (std::string::npos != pos) {
    auto parent = path.substr(0, pos);
    return parent + "/";
  } else {
    return std::string("./");
  }
}

}  // namespace hobotcommon
