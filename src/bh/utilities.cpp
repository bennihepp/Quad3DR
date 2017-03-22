//==================================================
// utilities.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 3, 2016
//==================================================

#include <bh/utilities.h>
#include <iostream>

namespace bh
{

RateCounter::RateCounter(bool do_start)
: counter_(-1)
{
  if (do_start)
  {
    start();
  }
}

void RateCounter::start()
{
  counter_ = 0;
  start_time_ = Clock::now();
}

void RateCounter::reset()
{
  start();
}

void RateCounter::increment()
{
  ++counter_;
}

void RateCounter::increment(int n)
{
  counter_ += n;
}

std::uint64_t RateCounter::getCounts() const
{
  return counter_;
}

double RateCounter::getElapsedTime() const
{
  Clock::time_point time = Clock::now();
  uint64_t elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time - start_time_).count();
  return elapsed_ms / 1000.0;
}

std::pair<std::uint64_t, double> RateCounter::getCountsAndElapsedTime() const
{
  return std::make_pair<std::uint64_t, double>(getCounts(), getElapsedTime());
}

double RateCounter::getRate() const
{
  double elapsed_time = getElapsedTime();
  double rate = counter_ / elapsed_time;
  return rate;
}

double RateCounter::getRateAndReset()
{
  double rate = getRate();
  reset();
  return rate;
}

double RateCounter::printRate(const std::string &name) const
{
  double rate = getRate();
  std::cout << "Rate for " << name << ": " << rate << " Hz" << std::endl;
  return rate;
}

double RateCounter::printRateAndReset(const std::string &name)
{
  double rate = printRate(name);
  reset();
  return rate;
}


Timer::Timer() {
  reset();
}

void Timer::reset()
{
  time_ = Clock::now();
}

double Timer::getElapsedTime() const
{
  uint64_t elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - time_).count();
  double elapsed_time = elapsed_ms / 1000.0;
  return elapsed_time;
}

uint64_t Timer::getElapsedTimeMs() const
{
  uint64_t elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - time_).count();
  return elapsed_ms;
}

double Timer::printTiming(const std::string &name) const
{
  double elapsed_time = getElapsedTime();
  std::cout << "Timing for " << name << ": " << elapsed_time << " s" << std::endl;
  return elapsed_time;
}

uint64_t Timer::printTimingMs(const std::string &name) const
{
  uint64_t  elapsed_ms = getElapsedTimeMs();
  std::cout << "Timing for " << name << ": " << elapsed_ms << " ms" << std::endl;
  return elapsed_ms;
}


#if not WITH_PROFILING
  void ProfilingTimer::start()
  {
  }

  double ProfilingTimer::getElapsedTime() const
  {
    return -1.0;
  }

  double ProfilingTimer::stop()
  {
    return -1.0;
  }

  double ProfilingTimer::printTiming(const std::string &name) const
  {
    return -1.0;
  }

  double ProfilingTimer::stopAndPrintTiming(const std::string &name)
  {
    return -1.0;
  }
#endif

}  // namespace bh
