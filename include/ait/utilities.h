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
#include <ait/common.h>

namespace ait {

/// Normalize value between 0 and 1 based on min and max
template <typename T>
T normalize(const T& value, const T& min, const T& max) {
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


// -------------------------
// Pointer utilities
// -------------------------

template<typename T, typename U>
std::unique_ptr<T> static_cast_ptr(std::unique_ptr<U>&& ptr);

template<typename T, typename U>
std::unique_ptr<T> dynamic_cast_ptr(std::unique_ptr<U>&& ptr);

template<typename T, typename U>
std::unique_ptr<T> reinterpret_cast_ptr(std::unique_ptr<U>&& ptr);

// -------------------------
// STL container utilities
// -------------------------

template <typename Set1, typename Set2>
std::size_t computeSetIntersectionSize(const Set1& set1, const Set2& set2);

template <typename Set1, typename Set2>
Set1 computeSetIntersection(const Set1& set1, const Set2& set2);

template <typename Set1, typename Set2>
std::size_t computeSetDifferenceSize(const Set1& set_a, const Set2& set_b);

template <typename Set1, typename Set2>
Set1 computeSetDifference(const Set1& set_a, const Set2& set_b);

template <typename T>
std::vector<std::size_t> computeHistogram(const std::vector<T>& values, const std::vector<T>& bins);

template <typename T>
std::pair<std::vector<std::size_t>, std::vector<T>> computeHistogram(const std::vector<T>& values, const std::size_t num_bins);

} /* namespace ait */

// -------------------------
// Pointer utilities
// -------------------------

template<typename T, typename U>
std::unique_ptr<T> ait::static_cast_ptr(std::unique_ptr<U>&& ptr) {
  T* raw_ptr = static_cast<T*>(ptr.release());
  return std::move(std::unique_ptr<T>(raw_ptr));
}

template<typename T, typename U>
std::unique_ptr<T> ait::dynamic_cast_ptr(std::unique_ptr<U>&& ptr) {
  T* raw_ptr = dynamic_cast<T*>(ptr.release());
  return std::move(std::unique_ptr<T>(raw_ptr));
}

template<typename T, typename U>
std::unique_ptr<T> ait::reinterpret_cast_ptr(std::unique_ptr<U>&& ptr) {
  T* raw_ptr = reinterpret_cast<T*>(ptr.release());
  return std::move(std::unique_ptr<T>(raw_ptr));
}

// -------------------------
// STL container utilities implementations
// -------------------------

template <typename Set1, typename Set2>
std::size_t ait::computeSetIntersectionSize(const Set1& set1, const Set2& set2) {
    static_assert(std::is_same<typename Set1::key_type, typename Set2::key_type>::value, "Key must be same type");
    if (set1.size() > set2.size()) {
        return computeSetIntersectionSize(set2, set1);
    }
    std::size_t size = 0;
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

template <typename Set1, typename Set2>
std::size_t ait::computeSetDifferenceSize(const Set1& set_a, const Set2& set_b) {
  static_assert(std::is_same<typename Set1::key_type, typename Set2::key_type>::value, "Key must be same type");
  std::size_t size = 0;
  for (const auto& key : set_a) {
      if (set_b.find(key) == set_b.cend()) {
        ++size;
      }
  }
  return size;
}

template <typename Set1, typename Set2>
Set1 ait::computeSetDifference(const Set1& set_a, const Set2& set_b) {
  static_assert(std::is_same<typename Set1::key_type, typename Set2::key_type>::value, "Key must be same type");
  Set1 difference;
  for (const auto& key : set_a) {
      if (set_b.find(key) == set_b.cend()) {
        difference.insert(key);
      }
  }
  return difference;
}

template <typename T>
std::vector<std::size_t> ait::computeHistogram(const std::vector<T>& values, const std::vector<T>& bin_edges) {
  if (values.empty()) {
    throw AIT_EXCEPTION("Cannot compute histogram empty container");
  }
  std::vector<T> sorted_values(values);
  std::sort(sorted_values.begin(), sorted_values.end());
  std::vector<std::size_t> counts;
  counts.resize(bin_edges.size(), 0);
  auto it = sorted_values.cbegin();
  auto edge_it = bin_edges.cbegin();
  auto count_it = counts.begin();
  while (edge_it != bin_edges.cend() && it != sorted_values.cend()) {
    if (*it <= *edge_it) {
      ++(*count_it);
      ++it;
    }
    else {
      ++count_it;
      ++edge_it;
    }
  }
  return counts;
}

template <typename T>
std::pair<std::vector<std::size_t>, std::vector<T>> ait::computeHistogram(const std::vector<T>& values, const std::size_t num_bins) {
  if (values.empty()) {
    throw AIT_EXCEPTION("Cannot compute histogram empty container");
  }
  auto minmax = std::minmax_element(values.cbegin(), values.cend());
  T min = *minmax.first;
  T max = *minmax.second;
  std::vector<T> bin_edges;
  T range = max - min;
  for (std::size_t i = 1; i < num_bins; ++i) {
    T edge = min + i * range / num_bins;
    bin_edges.push_back(edge);
  }
  bin_edges.push_back(max);
  std::cout << "num_bins: " << num_bins << std::endl;
  std::cout << "bin_edges: " << bin_edges.size() << std::endl;
  return std::make_pair(computeHistogram(values, bin_edges), bin_edges);
}
