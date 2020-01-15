/*!
 * Copyright (c) 2017 by Contributors
 * \file buffer_file.hpp
 * \brief
 * \author liangzhujin
 */

#ifndef MXNET_PREDICTOR_BUFFER_FILE_HPP
#define MXNET_PREDICTOR_BUFFER_FILE_HPP

#include <fstream>
#include <string>

#include "hobotlog/hobotlog.hpp"

namespace hobot {
namespace vision {

class BufferFile {
 public:
  std::string file_path_;
  int length_;
  char *buffer_;

  explicit BufferFile(const std::string &file_path) : file_path_(file_path) {
    std::ifstream ifs(file_path.c_str(), std::ios::in | std::ios::binary);
    HOBOT_CHECK(ifs) << "Can't open the file. Please check " << file_path;

    ifs.seekg(0, std::ios::end);
    length_ = ifs.tellg();
    ifs.seekg(0, std::ios::beg);

    buffer_ = new char[sizeof(char) * length_ + 1];
    buffer_[length_] = '\0';
    ifs.read(buffer_, length_);
    ifs.close();
  }

  int GetLength() { return length_; }
  char *GetBuffer() { return buffer_; }

  ~BufferFile() {
    delete[] buffer_;
    buffer_ = nullptr;
  }
};

}  // namespace vision
}  // namespace hobot

#endif  // MXNET_PREDICTOR_BUFFER_FILE_HPP
