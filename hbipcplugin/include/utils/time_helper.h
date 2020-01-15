/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     helpers.h
 * \Author   Yingmin Li
 * \Mail     yingmin.li-horizon.ai
 * \Version  1.0.0.0
 * \Date     2019/1/10
 * \Brief    implement of helpers header
 */
#ifndef TIME_HELPERS_H_
#define TIME_HELPERS_H_

#include <string>
#include <sys/time.h>
#include <chrono>

namespace hobot {
/**
 * @brief The time related helpers.
 */
class Timer {
 public:
  /**
   * @brief Get a clock tic for current time.
   * @return The current time point.
   */
  static std::chrono::time_point<std::chrono::system_clock> tic();

  /**
   * @brief Get the current time stamp.
   * @return The current time stamp.
   */
  static time_t current_time_stamp();

  /**
   * @brief Get the time difference from the tic time.
   * @param tic The tic time.
   * @return The time difference in second.
   */
  static double toc_s(
      const std::chrono::time_point<std::chrono::system_clock> &tic);

  /**
   * @brief Get the time difference from the tic time.
   * @param tic The tic time.
   * @return The time difference in millisecond.
   */
  static double toc(
      const std::chrono::time_point<std::chrono::system_clock> &tic);
};

}  // namespace hobot

#endif  // TIME_HELPERS_H_
