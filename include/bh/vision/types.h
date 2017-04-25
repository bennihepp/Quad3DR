//==================================================
// types.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 05.04.17
//==================================================
#pragma once

#include <limits>

namespace bh {
namespace vision {

template <typename FloatT>
class Keypoint {
public:
  Keypoint();

  Keypoint(const FloatT x, const FloatT y, const FloatT scale, const FloatT orientation);

  Keypoint(const Keypoint& other) = default;

  FloatT x() const { return x_; }

  FloatT y() const { return y_; }

  FloatT scale() const { return scale_; }

  FloatT orientation() const { return orientation_; }

private:
  FloatT x_;
  FloatT y_;
  FloatT scale_;
  FloatT orientation_;
};

template <typename FloatT>
class KeypointMatch {
public:
  KeypointMatch(const Keypoint<FloatT>& keypoint1, const Keypoint<FloatT>& keypoint2);

  KeypointMatch(const KeypointMatch& keypoint_match) = default;

  const Keypoint<FloatT>& keypoint1() const { return keypoint1_; }

  const Keypoint<FloatT>& keypoint2() const { return keypoint2_; }

private:
  Keypoint<FloatT> keypoint1_;
  Keypoint<FloatT> keypoint2_;
};


using Keypointf = Keypoint<float>;
using Keypointd = Keypoint<double>;

template <typename FloatT>
Keypoint<FloatT>::Keypoint()
    : x_(std::numeric_limits<FloatT>::quiet_NaN()),
      y_(std::numeric_limits<FloatT>::quiet_NaN()),
      scale_(std::numeric_limits<FloatT>::quiet_NaN()),
      orientation_(std::numeric_limits<FloatT>::quiet_NaN()) {}

template <typename FloatT>
Keypoint<FloatT>::Keypoint(const FloatT x, const FloatT y, const FloatT scale, const FloatT orientation)
    : x_(x), y_(y), scale_(scale), orientation_(orientation) {}

template <typename FloatT>
KeypointMatch<FloatT>::KeypointMatch(const Keypoint<FloatT>& keypoint1, const Keypoint<FloatT>& keypoint2)
    : keypoint1_(keypoint1), keypoint2_(keypoint2) {}

}
}