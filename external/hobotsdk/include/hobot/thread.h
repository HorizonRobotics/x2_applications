//
// Created by Lisen Mu on 8/26/16.
// Copyright (c) 2016 Horizon Robotics.
//

#ifndef HOBOT_THREAD_H_
#define HOBOT_THREAD_H_

/*
 *  Copyright 2004 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */


#include <string>

#ifdef HR_WIN
#ifdef HOBOT_DLL_EXPORTS
#ifdef HOBOT_EXPORTS
#define HOBOT_EXPORT __declspec(dllexport)
#else
#define HOBOT_EXPORT __declspec(dllimport)
#endif
#else
#define HOBOT_EXPORT
#endif
#elif HR_POSIX
#define HOBOT_EXPORT
#endif

#ifndef __FUNCTION__
#define __FUNCTION__ ""
#endif
#ifndef __LINE__
#define __LINE__ ""
#endif

#define STRINGIZE_NO_EXPANSION(x) #x
#define STRINGIZE(x) STRINGIZE_NO_EXPANSION(x)
#define LINE_WITH_FUNC(function) function, STRINGIZE(__LINE__)
#define HOBOTLOC LINE_WITH_FUNC(__FUNCTION__)

namespace hobot
{

/**
 * from hobot::MessageHandler to hobotrtc::MessageHandler (saved as void *)
 */
class HOBOT_EXPORT HandlerRef {
 public:
  HandlerRef():handler_r(NULL) {}
  // actual type is hobotrtc::MessageHandler
  void *handler_r;
};

class HOBOT_EXPORT MessageHandler {
 public:
  virtual ~MessageHandler();

  MessageHandler();

  virtual void OnMessage(uint32_t message_id, void* data) = 0;

  // do not leak type info in this .h file
  HandlerRef handler_ref_;

 private:
  void operator=(const MessageHandler&) = delete;
  MessageHandler(const MessageHandler&) = delete;
};

const uint32_t MQID_ANY = static_cast<uint32_t>(-1);

// WARNING! SUBCLASSES MUST CALL Stop() IN THEIR DESTRUCTORS!  See ~Thread().

class HOBOT_EXPORT Thread
{
 public:
  Thread() {}
// NOTE: ALL SUBCLASSES OF Thread MUST CALL Stop() IN THEIR DESTRUCTORS (or
// guarantee Stop() is explicitly called before the subclass is destroyed).
// This is required to avoid a data race between the destructor modifying the
// vtable, and the Thread::PreRun calling the virtual method Run().
  virtual ~Thread() {}

  static Thread* NewThread();

  virtual bool IsCurrent() = 0;

// Sleeps the calling thread for the specified number of milliseconds, during
// which time no processing is performed. Returns false if sleeping was
// interrupted by a signal (POSIX only).
  static bool SleepMs(int millis);

// Sets the thread's name, for debugging. Must be called before Start().
// If |obj| is non-NULL, its value is appended to |name|.
  virtual const std::string &name() const = 0;

  virtual bool SetName(const std::string &name, const void *obj) = 0;

// Starts the execution of the thread.
  virtual bool Start() = 0;

  virtual void Post(const char* func_name,
                    const char* line_num,
                    MessageHandler* phandler,
                    uint32_t id = 0,
                    void* pdata = NULL,
                    bool time_sensitive = false) = 0;

  virtual void PostDelayed(const char* func_name,
                           const char* line_num,
                           int cmsDelay,
                           MessageHandler* phandler,
                           uint32_t id = 0,
                           void* pdata = NULL) = 0;

// Tells the thread to stop and waits until it is joined.
// Never call Stop on the current thread.  Instead use the inherited Quit
// function which will exit the base MessageQueue without terminating the
// underlying OS thread.
  virtual void Stop() = 0;

// By default, Thread::Run() calls ProcessMessages(kForever).  To do other
// work, override Run().  To receive and dispatch messages, call
// ProcessMessages occasionally.
  virtual void Run() = 0;

// From MessageQueue
  virtual void Clear(MessageHandler *phandler,
             uint32_t id = MQID_ANY) = 0;

// Sets the cpu affinity of thread.
// must be called AFTER Start()
  virtual bool SetAffinity(int core_id) = 0;

  virtual bool SetPriority(int priority) = 0;

 private:
  void operator=(const Thread&) = delete;
  Thread(const Thread&) = delete;
};

}  // namespace hobot


#endif  // HOBOT_THREAD_H_
