//==================================================
// histogram.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 08.05.17
//==================================================
#pragma once

#include <bh/common.h>

namespace bh {

template <typename Iterator, typename BinIterator, typename N = size_t>
std::vector<N> computeHistogram(const Iterator first, const Iterator last,
                                const BinIterator first_bin, const BinIterator last_bin);

/// Computes a histogram of values in range [first, last).
/// The bins are given by the range [first_bin, last_bin), i.e. the first bin is [*first_bin, *(first_bin + 1)).
/// Condition: last_bin - first_bin >= 2.
/// Values below *first_bin are ignored.
template <typename Iterator, typename BinIterator, typename N = size_t>
std::vector<N> computeHistogram(const Iterator first, const Iterator last,
                                const BinIterator first_bin, const BinIterator last_bin) {
  BH_ASSERT(last_bin - first_bin >= 2);
  std::vector<N> histogram(last_bin - first_bin, 0);
  for (Iterator it = first; it < last; ++it) {
    BinIterator it_bin = std::upper_bound(first_bin, last_bin, *it);
    if (it_bin != first_bin) {
      --it_bin;
      const N bin_index = it_bin - first_bin;
      ++histogram[bin_index];
    }
  }
  return histogram;
};

}