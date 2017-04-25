//==================================================
// utilities.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 3, 2016
//==================================================

#pragma once

#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <utility>
#include <cstdint>
#include <chrono>
#include <thread>
#include <type_traits>
#include "common.h"

namespace bh {

/// Normalize value between 0 and 1 based on min and max
template <typename T>
T normalize(const T& value, const T& min, const T& max) {
  static_assert(std::is_floating_point<T>::value, "Normalize only works with floating point types.");
  return (value - min) / (max - min);
}

/// Class to track min and max values of data
template <typename T>
class MinMaxTracker {
public:
  MinMaxTracker()
  : min_(std::numeric_limits<T>::max()),
    max_(std::numeric_limits<T>::lowest()) {}

  void update(const T& value) {
    min_ = std::min(value, min_);
    max_ = std::max(value, max_);
  }

  const T& minimum() const {
    return min_;
  }

  const T& maximum() const {
    return max_;
  }

  T range() const {
    return max_ - min_;
  }

  T normalize(const T& value) const {
    return (value - minimum()) / range();
  }

private:
  T min_;
  T max_;
};

/// Clamp a value between min and max
template <typename T>
T clamp(const T& value, const T& min = 0, const T& max = 1) {
    return std::max(std::min(value, max), min);
}

// -------------------------
// Timing utilities
// -------------------------

using Clock = std::chrono::high_resolution_clock;

class RateCounter {
  std::uint64_t counter_;
  Clock::time_point start_time_;

public:
  RateCounter(bool do_start=true);

  void start();
  void reset();
  void increment();
  void increment(int n);

  std::uint64_t getCounts() const;
  double getElapsedTime() const;
  std::pair<std::uint64_t, double> getCountsAndElapsedTime() const;
  double getRate() const;
  double getRateAndReset();

  double printRate(const std::string& name) const;
  double printRateAndReset(const std::string& name);
};

class Timer {
  Clock::time_point time_;

public:
  Timer();

  void reset();
  double getElapsedTime() const;
  uint64_t getElapsedTimeMs() const;

  double printTiming(const std::string &name) const;
  uint64_t printTimingMs(const std::string &name) const;

};

class PaceMaker {
public:
    using clock = std::chrono::high_resolution_clock;
    PaceMaker(double desired_rate)
        : desired_period_(1 / desired_rate), time_ahead_(0) {
        last_time_ = clock::now();
    }

    void sleep() {
        std::chrono::time_point<clock> now = clock::now();
        std::chrono::duration<double> duration = now - last_time_;
        time_ahead_ = time_ahead_ + desired_period_ - duration;
        if (time_ahead_.count() < 0) {
            time_ahead_ = std::chrono::duration<double>(0);
        }
        std::chrono::milliseconds sleep_time_ms(static_cast<int64_t>(1000 * time_ahead_.count()));
        std::this_thread::sleep_for(sleep_time_ms);
        last_time_ = now;
    }

private:
    std::chrono::time_point<clock> last_time_;
    std::chrono::duration<double> desired_period_;
    std::chrono::duration<double> time_ahead_;
};

#if WITH_PROFILING
  using ProfilingTimer = Timer;
#else
  class ProfilingTimer
  {
  public:
    void start();
    double getElapsedTime() const;
    double stop();

    double printTiming(const std::string &name) const;
    double stopAndPrintTiming(const std::string &name);
  };
#endif

}
