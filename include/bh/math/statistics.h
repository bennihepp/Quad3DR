//==================================================
// statistics.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Nov 8, 2016
//==================================================
#pragma once

#include <cmath>
#include <type_traits>
#include <Eigen/Dense>
#include "eigen.h"

namespace bh {

template <typename T>
std::vector<std::size_t> computeHistogram(const std::vector<T>& values, const std::vector<T>& bins);

template <typename T>
std::pair<std::vector<std::size_t>, std::vector<T>> computeHistogram(const std::vector<T>& values, const std::size_t num_bins);

template <typename ValueT, typename CountT = size_t>
class Histogram {
public:
  using ValueType = ValueT;
  using CountType = CountT;

  // bin_edges_ contains the left and right edges of all bins.
  // i.e. if there are 2 bins, [0, 1) and [1, 2) bins_edges_ = { 0, 1, 2 }.

  Histogram(const size_t num_bins, const ValueT min, const ValueT max) {
    for (size_t i = 0; i <= num_bins; ++i) {
      const ValueT edge = min + i * (max - min) / num_bins;
      bin_edges_.push_back(edge);
    }
    counts_.resize(bin_edges_.size() - 1, 0);
  }

  template <typename Iterator>
  Histogram(const Iterator first, const Iterator last) {
    for (Iterator it = first; it != last; ++it) {
      bin_edges_.push_back(*it);
    }
    counts_.resize(bin_edges_.size() - 1, 0);
  }

  size_t numBins() const {
    return counts_.size();
  }

  CountType operator()(const size_t bin) const {
    return counts_[bin];
  }

  CountType totalCount() const {
    const CountT total_count = std::accumulate(counts_.begin(), counts_.end());
    return total_count;
  }

  ValueT leftBinEdge(const size_t bin) const {
    return bin_edges_[bin];
  }

  ValueT leftBinEdge(const size_t bin) const {
    return bin_edges_[bin + 1];
  }

  template <typename ValueIterator>
  void accumulate(const ValueIterator first, const ValueIterator last) {
    for (ValueIterator it = first; it != last; ++it) {
      // find bin
      const size_t bin = 0;
      ++counts_[bin];
    }
  }

  template <typename = std::enable_if<std::is_floating_point<CountT>::value>>
  void normalize() {
    const CountT total_count = totalCount();
    std::for_each(counts_.begin(), counts_.end(), [] (CountT& count) {
      count /= total_count;
    });
  }

private:
  std::vector<ValueT> bin_edges_;
  std::vector<CountT> counts_;
};

}

template <typename T>
std::vector<std::size_t> bh::computeHistogram(const std::vector<T>& values, const std::vector<T>& bin_edges) {
  if (values.empty()) {
    throw BH_EXCEPTION("Cannot compute histogram empty container");
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
std::pair<std::vector<std::size_t>, std::vector<T>> bh::computeHistogram(const std::vector<T>& values, const std::size_t num_bins) {
  if (values.empty()) {
    throw BH_EXCEPTION("Cannot compute histogram empty container");
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
