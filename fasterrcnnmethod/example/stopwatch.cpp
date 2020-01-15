//
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#include "./stopwatch.h"

#include <stdint.h>
#include <sstream>
#include <string>
#include <algorithm>


uint64_t Stopwatch::CurrentTs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
}

Stopwatch::Stopwatch() :
    start_(std::chrono::steady_clock::now()),
    stop_(start_),
    total_duration_(0),
    last_duration_(0),
    min_duration_(INT32_MAX),
    max_duration_(0) {
}

void Stopwatch::Start() {
  start_ = std::chrono::steady_clock::now();
}

void Stopwatch::Stop() {
  stop_ = std::chrono::steady_clock::now();
  last_duration_ = std::chrono::duration_cast<Us>(stop_ - start_);
  total_duration_ += last_duration_;
  min_duration_ = std::min(min_duration_, last_duration_);
  max_duration_ = std::max(max_duration_, last_duration_);
  timing_count_++;
}

void Stopwatch::Reset() {
  start_ = std::chrono::steady_clock::now();
  stop_ = start_;
  last_duration_ = Us(0);
  total_duration_ = Us(0);
  min_duration_ = Us(INT32_MAX);
  max_duration_ = Us(0);
  timing_count_ = 0;
}

int Stopwatch::Duration() {
  return total_duration_.count();
}

int Stopwatch::LastDuration() {
  return last_duration_.count();
}

int Stopwatch::Min() {
  return min_duration_.count();
}

int Stopwatch::Max() {
  return max_duration_.count();
}

int Stopwatch::TimingCount() {
  return timing_count_;
}

float Stopwatch::Fps() {
  return static_cast<float>(1000000.0 / Average());
}

int Stopwatch::Average() {
  if (timing_count_ < 3) {
    return Duration() / timing_count_;
  } else {
    return (Duration() - Min() - Max()) / (timing_count_ - 2);
  }
}

std::string Stopwatch::Str() {
  std::stringstream ss;
  ss << "count:" << timing_count_
     << ", duration:" << Duration() << "us"
     << ", min:" << Min() << "us"
     << ", max:" << Max() << "us"
     << ", average:" << Average() << "us"
     << ", fps:" << Fps() << "/s"
     << std::endl;
  return ss.str();
}


