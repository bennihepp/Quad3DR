//==================================================
// distance_field.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 9, 2017
//==================================================
#pragma once

#include <vector>
#include <boost/serialization/access.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/vector.hpp>
#include "../eigen.h"

namespace bh {

#if __GNUC__ && !__CUDACC__
  #pragma GCC push_options
  #pragma GCC optimize ("fast-math")
#endif

template <typename ValueT>
class Grid3D {
public:
  using size_t = std::size_t;
  using Vector3s = Eigen::Vector3s;
  using ValueType = ValueT;

  Grid3D()
  : dim_x_(0), dim_y_(0), dim_z_(0) {}

  Grid3D(const size_t dim_x, const size_t dim_y, const size_t dim_z)
  : dim_x_(dim_x), dim_y_(dim_y), dim_z_(dim_z) {
    values_.resize(dim_x_ * dim_y_ * dim_z_);
  }

  Vector3s getDimensions() const {
    return Vector3s(dim_x_, dim_y_, dim_z_);
  }

  size_t getDimX() const {
    return dim_x_;
  }

  size_t getDimY() const {
    return dim_x_;
  }

  size_t getDimZ() const {
    return dim_x_;
  }

  size_t getNumElements() const {
    return dim_x_ * dim_y_ * dim_z_;
  }

  bool isValidCoordinate(const Vector3s& xyz) const {
    return 0 <= xyz(0)
           && xyz(0) < dim_x_
           && 0 <= xyz(1)
           && xyz(1) < dim_y_
           && 0 <= xyz(2)
           && xyz(2) < dim_z_;
  }

  bool isValidCoordinate(const size_t x, const size_t y, const size_t z) const {
    return 0 <= x
           && x < dim_x_
           && 0 <= y
           && y < dim_y_
           && 0 <= z
           && z < dim_z_;
  }

  const std::vector<ValueType>& getValues() const {
    return values_;
  }

  std::vector<ValueType>& getValues() {
    return values_;
  }

  size_t getIndex(const Vector3s& xyz) const {
#if !BH_RELEASE
    BH_ASSERT(isValidCoordinate(xyz));
#endif
    return xyz(0) + dim_x_ * (xyz(1) + dim_y_ * xyz(2));
  }

  size_t getIndex(const size_t x, const size_t y, const size_t z) const {
#if !BH_RELEASE
    BH_ASSERT(isValidCoordinate(x, y, z));
#endif
    return x + dim_x_ * (y + dim_y_ * z);
  }

  const ValueType& operator()(const Vector3s& xyz) const {
    const size_t index = getIndex(xyz);
    return (*this)(index);
  }

  ValueType& operator()(const Vector3s& xyz) {
    const size_t index = getIndex(xyz);
    return (*this)(index);
  }

  const ValueType& operator()(const size_t x, const size_t y, const size_t z) const {
    const size_t index = getIndex(x, y, z);
    return (*this)(index);
  }

  ValueType& operator()(const size_t x, const size_t y, const size_t z) {
    const size_t index = getIndex(x, y, z);
    return (*this)(index);
  }

  const ValueType& operator()(const size_t index) const {
#if !BH_RELEASE
    BH_ASSERT(0 <= index);
    BH_ASSERT(index < getNumElements());
#endif
    return values_[index];
  }

  ValueType& operator()(const size_t index) {
#if !BH_RELEASE
    BH_ASSERT(0 <= index);
    BH_ASSERT(index < getNumElements());
#endif
    return values_[index];
  }

  void setAllValues(const ValueType v) {
    std::fill(values_.begin(), values_.end(), v);
  }

private:
  friend class boost::serialization::access;

  template <typename Archive>
  void serialize(Archive& ar, unsigned int version) {
    ar & const_cast<size_t&>(dim_x_);
    ar & const_cast<size_t&>(dim_y_);
    ar & const_cast<size_t&>(dim_z_);
    ar & values_;
  }

  size_t dim_x_;
  size_t dim_y_;
  size_t dim_z_;
  std::vector<ValueType> values_;
};

#if __GNUC__ && !__CUDACC__
  #pragma GCC pop_options
#endif

}
