//
// Copyright 2016 Horizon Robotics.
// Created by witwolf on 8/31/16.
//

#ifndef HOBOT_MODULE_TEST_H_
#define HOBOT_MODULE_TEST_H_

#include<functional>
#include<memory>

#include "hobot/hobot.h"


typedef std::function<bool(const hobot::MessageLists &)> fnOutputChecker;

namespace hobot {

class ModuleTest {
 public:
  /**
   * @param module
   * @param forward_index
   * @param input messages
   * @param condition
   * @param output_checker function to check whether output meets expectations
   * @return
   */
  ModuleTest(Module *module,
             int forward_index,
             const MessageLists &input,
             hobot::Expression *condition,
             const fnOutputChecker &output_checker) :
      module(module),
      forward_index(forward_index),
      input(input),
      condition(condition),
      output_checker(output_checker) { }

  /**
   * run test
   * @return true if moudle output meets expectations otherwise false
   */
  bool Run();

 private:
  Module *module;
  int forward_index;
  const MessageLists &input;
  hobot::Expression *condition;
  const fnOutputChecker &output_checker;
};
}  // namespace hobot
#endif  //  HOBOT_MODULE_TEST_H_
