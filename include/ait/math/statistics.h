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

namespace ait {

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
