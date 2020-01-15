/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     helpers.cpp
 * \Author   Yingmin Li
 * \Mail     yingmin.li-horizon.ai
 * \Version  1.0.0.0
 * \Date     2019/1/10
 * \Brief    implement of helpers.cpp
 */
#include <sstream>
#include "utils/time_helper.h"

namespace hobot {

std::chrono::time_point<std::chrono::system_clock> Timer::tic() {
  return std::chrono::system_clock::now();
}

double Timer::toc_s(
    const std::chrono::time_point<std::chrono::system_clock> &tic) {
  auto time_d =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now() - tic);
  return static_cast<double>(time_d.count()) *
      std::chrono::microseconds::period::num /
      std::chrono::microseconds::period::den;
}

double Timer::toc(
    const std::chrono::time_point<std::chrono::system_clock> &tic) {
  return toc_s(tic) * 1000.0;
}

time_t Timer::current_time_stamp() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

}  // namespace hobot
