//
// Created by chuanyi.yang@hobot.cc on 06/15/2018.
// Copyright (c) 2018 horizon robotics. All rights reserved.
//
#ifndef COMMON_SINGLETON_H_
#define COMMON_SINGLETON_H_

#include <assert.h>

namespace HobotXRoc {

template <typename T>
class XSingleton {
  struct ObjectCreator {
   public:
    ObjectCreator() { XSingleton<T>::Instance(); }

    inline void DoNothing() const {}
  };

  static ObjectCreator create_object_;

 public:
  typedef T ObjectType;

  static ObjectType &Instance() {
    static ObjectType obj;
    // 据说这个do_nothing是确保create_object构造函数被调用
    // 这跟模板的编译有关
    create_object_.DoNothing();
    return obj;
  }
};

template <typename T>
typename XSingleton<T>::ObjectCreator XSingleton<T>::create_object_;

}  // namespace HobotXRoc
#endif  // COMMON_SINGLETON_H_
