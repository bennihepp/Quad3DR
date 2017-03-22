//==================================================
// algorithm.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 15, 2017
//==================================================



namespace bh {
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

} /* namespace bh */

// -------------------------
// STL container utilities implementations
// -------------------------

template <typename Set1, typename Set2>
std::size_t bh::computeSetIntersectionSize(const Set1& set1, const Set2& set2) {
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
Set1 bh::computeSetIntersection(const Set1& set1, const Set2& set2) {
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
std::size_t bh::computeSetDifferenceSize(const Set1& set_a, const Set2& set_b) {
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
Set1 bh::computeSetDifference(const Set1& set_a, const Set2& set_b) {
  static_assert(std::is_same<typename Set1::key_type, typename Set2::key_type>::value, "Key must be same type");
  Set1 difference;
  for (const auto& key : set_a) {
      if (set_b.find(key) == set_b.cend()) {
        difference.insert(key);
      }
  }
  return difference;
}
