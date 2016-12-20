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
#include <vector>
#include <utility>
#include <cstdint>
#include <chrono>
#include <thread>
#include <ait/common.h>

namespace ait
{

/// Clamp a value between min and max
template <typename T>
T clamp(const T& value, const T& min, const T& max)
{
    return std::max(std::min(value, max), min);
}

// -------------------------
// Timing utilities
// -------------------------

using Clock = std::chrono::high_resolution_clock;

class RateCounter
{
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

class Timer
{
  Clock::time_point time_;

public:
  Timer();

  void reset();
  double getElapsedTime() const;
  uint64_t getElapsedTimeMs() const;

  double printTiming(const std::string &name) const;
  uint64_t printTimingMs(const std::string &name) const;

};

class PaceMaker
{
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


// -------------------------
// STL container utilities
// -------------------------

template <typename Set1, typename Set2>
size_t computeSetIntersectionSize(const Set1& set1, const Set2& set2);

template <typename Set1, typename Set2>
Set1 computeSetIntersection(const Set1& set1, const Set2& set2);

} /* namespace ait */


// -------------------------
// STL container utilities implementations
// -------------------------

template <typename Set1, typename Set2>
size_t ait::computeSetIntersectionSize(const Set1& set1, const Set2& set2) {
    static_assert(std::is_same<typename Set1::key_type, typename Set2::key_type>::value, "Key must be same type");
    if (set1.size() > set2.size()) {
        return computeSetIntersectionSize(set2, set1);
    }
    size_t size = 0;
    for (const auto& key : set1) {
        if (set2.find(key) != set2.cend()) {
            ++size;
        }
    }
    return size;
}

template <typename Set1, typename Set2>
Set1 ait::computeSetIntersection(const Set1& set1, const Set2& set2) {
    static_assert(std::is_same<typename Set1::key_type, typename Set2::key_type>::value, "Key must be same type");
    if (set1.size() > set2.size()) {
        return computeSetIntersection(set2, set1);
    }
    Set1 intersection;
    for (const auto& key : set1) {
        if (set2.find(key) != set2.cend()) {
            intersection.insert(key);
        }
    }
    return intersection;
}


// -------------------------
// File input output utilities
// -------------------------

template <typename T>
void writeToStream(std::ostream& out, const T& value) {
  out.write(reinterpret_cast<char*>(&value), sizeof(value));
  if (!static_cast<bool>(out)) {
    throw AIT_EXCEPTION("Unable to write to stream");
  }
}

template <typename T>
void readFromStream(std::istream& in, T* value) {
  in.read(reinterpret_cast<char*>(value), sizeof(T));
  if (!static_cast<bool>(in)) {
    throw AIT_EXCEPTION("Unable to read from stream");
  }
}

template <typename T>
T readFromStream(std::istream& in) {
  T value;
  in.read(reinterpret_cast<char*>(&value), sizeof(T));
  if (!static_cast<bool>(in)) {
    throw AIT_EXCEPTION("Unable to read from stream");
  }
  return std::move(value);
}
