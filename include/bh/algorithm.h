//==================================================
// algorithm.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 15, 2017
//==================================================

#include <cstddef>
#include <algorithm>
#include "common.h"

namespace bh {
// -------------------------
// STL container utilities
// -------------------------

/// Generate an index sequence. To be used with std::back_inserter
template <typename InsertIt>
void generateIndexSequence(const std::size_t num_elements, InsertIt it);

/// Generate an index sequence.
template <typename IndexT = std::size_t>
std::vector<IndexT> generateIndexSequence(const std::size_t num_elements);

/// Sort index sequence [first_index, last_index) by using the order of sequence [first_order, last_order)
template <typename IndexIt, typename OrderIt>
void argsort(IndexIt first_index, IndexIt last_index, OrderIt first_order, OrderIt last_order);

/// Sort index sequence [first_index, last_index) by using the order of sequence [first_order, last_order).
/// @param comp: Compares two objects of type OrderIt::value_type.
template <typename IndexIt, typename OrderIt, typename Compare>
void argsort(IndexIt first_index, IndexIt last_index, OrderIt first_order, OrderIt last_order, Compare comp);

/// Sort index sequence [first_index, last_index) by using the order of sequence [first_order, last_order)
template <typename OrderIt, typename IndexT = std::size_t>
std::vector<IndexT> argsort(OrderIt first_order, OrderIt last_order);

/// Sort index sequence [first_index, last_index) by using the order of sequence [first_order, last_order).
/// @param comp: Compares two objects of type OrderIt::value_type.
template <typename OrderIt, typename Compare, typename IndexT = std::size_t>
std::vector<IndexT> argsort(OrderIt first_order, OrderIt last_order, Compare comp);

template <typename Set1, typename Set2>
std::size_t computeSetIntersectionSize(const Set1& set1, const Set2& set2);

template <typename Set1, typename Set2>
Set1 computeSetIntersection(const Set1& set1, const Set2& set2);

template <typename Set1, typename Set2>
std::size_t computeSetDifferenceSize(const Set1& set_a, const Set2& set_b);

template <typename Set1, typename Set2>
Set1 computeSetDifference(const Set1& set_a, const Set2& set_b);

// -------------------------
// STL container utilities implementations
// -------------------------

template <typename InsertIt>
void generateIndexSequence(const std::size_t num_elements, InsertIt it) {
  for (std::size_t i = 0; i < num_elements; ++i) {
    *it = i;
    ++it;
  }
}

template <typename IndexT>
std::vector<IndexT> generateIndexSequence(const std::size_t num_elements) {
  std::vector<IndexT> indices(num_elements);
  for (std::size_t i = 0; i < num_elements; ++i) {
    indices[i] = i;
  }
  return indices;
}

template <typename IndexIt, typename OrderIt>
void argsort(IndexIt first_index, IndexIt last_index, OrderIt first_order, OrderIt last_order) {
  using IndexType = typename IndexIt::value_type;
  BH_ASSERT(last_index - first_index == last_order - first_order);
  std::sort(first_index, last_index, [&](const IndexType index1, const IndexType index2) {
    return *(first_order + index1) < *(first_order + index2);
  });
};

template <typename IndexIt, typename OrderIt, typename Compare>
void argsort(IndexIt first_index, IndexIt last_index, OrderIt first_order, OrderIt last_order, Compare comp) {
  using IndexType = typename IndexIt::value_type;
  BH_ASSERT(last_index - first_index == last_order - first_order);
  std::sort(first_index, last_index, [&](const IndexType index1, const IndexType index2) {
    return comp(*(first_order + index1), (first_order + index2));
  });
};

/// Sort index sequence [first_index, last_index) by using the order of sequence [first_order, last_order)
template <typename OrderIt, typename IndexT>
std::vector<IndexT> argsort(OrderIt first_order, OrderIt last_order) {
  std::vector<IndexT> indices = generateIndexSequence<IndexT>(last_order - first_order);
  argsort(indices.begin(), indices.end(), first_order, last_order);
  return indices;
};

/// Sort index sequence [first_index, last_index) by using the order of sequence [first_order, last_order).
/// @param comp: Compares two objects of type OrderIt::value_type.
template <typename OrderIt, typename Compare, typename IndexT>
std::vector<IndexT> argsort(OrderIt first_order, OrderIt last_order, Compare comp) {
  std::vector<IndexT> indices = generateIndexSequence<IndexT>(last_order - first_order);
  argsort(indices.begin(), indices.end(), first_order, last_order, comp);
  return indices;};

template <typename Set1, typename Set2>
std::size_t computeSetIntersectionSize(const Set1& set1, const Set2& set2) {
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
Set1 computeSetIntersection(const Set1& set1, const Set2& set2) {
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
std::size_t computeSetDifferenceSize(const Set1& set_a, const Set2& set_b) {
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
Set1 computeSetDifference(const Set1& set_a, const Set2& set_b) {
  static_assert(std::is_same<typename Set1::key_type, typename Set2::key_type>::value, "Key must be same type");
  Set1 difference;
  for (const auto& key : set_a) {
      if (set_b.find(key) == set_b.cend()) {
        difference.insert(key);
      }
  }
  return difference;
}

} /* namespace bh */
