//
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#ifndef FASTERRCNNMETHOD_STOPWATCH_H
#define FASTERRCNNMETHOD_STOPWATCH_H

#include <chrono>
#include <string>
#include <fstream>

typedef std::chrono::steady_clock::time_point Time;
typedef std::chrono::duration<int, std::milli> Ms;
typedef std::chrono::duration<int, std::micro> Us;

class Stopwatch {
 public:
  Stopwatch();
  static uint64_t CurrentTs();
  void Start();
  void Stop();
  void Reset();
  float Fps();
  int Average();
  int Duration();
  int Min();
  int Max();
  int LastDuration();
  std::string Str();
  int TimingCount();
 private:
  Time start_;
  Time stop_;
  Us total_duration_;
  Us last_duration_;
  Us min_duration_;
  Us max_duration_;
  int timing_count_ = 0;
};

#endif //FASTERRCNNMETHOD_STOPWATCH_H
