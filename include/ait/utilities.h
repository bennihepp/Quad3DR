//==================================================
// utilities.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 3, 2016
//==================================================

#pragma once

#include <vector>
#include <utility>
#include <cstdint>

namespace ait
{

class Clock
{
  Clock() = delete;

public:
  static std::int64_t getTickCount();
  static double getTickFrequency();
};

class RateCounter
{
  int counter_;
  std::int64_t start_ticks_;

public:
  RateCounter(bool do_start=true);

  void start();
  void reset();
  void increment();
  void increment(int n);

  std::int64_t getCounts() const;
  double getElapsedTime() const;
  std::pair<std::int64_t, double> getCountsAndElapsedTime() const;
  double getRate() const;
  double getRateAndReset();

  double printRate(const std::string &name) const;
  double printRateAndReset(const std::string &name);
};

class Timer
{
  double timing_;
  bool running;

public:
  Timer(bool startTimer=true);

  void start();
  double getElapsedTime() const;
  double stop();

  double printTiming(const std::string &name) const;
  double stopAndPrintTiming(const std::string &name);

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

////////////////////////
// File system utilities
////////////////////////

template <typename... T>
std::string joinPaths(T const&... paths) {
  boost::filesystem::path result;
  bool _[]{false, (result /= boost::filesystem::path(paths), false)...};
  static_cast<void>(_);
  return result.string();
}

} /* namespace ait */
