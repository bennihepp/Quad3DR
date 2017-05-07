//==================================================
// utilities.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Nov 8, 2016
//==================================================
#pragma once

#include <cmath>
#include <Eigen/Dense>

namespace bh {

  template <typename FloatT>
  FloatT degreeToRadians(const FloatT degrees) {
    const FloatT pi = (FloatT)M_PI;
    return degrees * pi / 180;
  }

  template <typename FloatT>
  FloatT radiansToDegrees(const FloatT radians) {
    const FloatT pi = (FloatT)M_PI;
    return radians * 180 / pi;
  }

  template <typename FloatT>
  FloatT wrapRadiansTo0And2Pi(const FloatT angle) {
    const FloatT two_pi = 2 * FloatT(M_PI);
    const FloatT wrapped_angle = angle - two_pi * std::floor(angle / two_pi);
    return wrapped_angle;
  }

  template <typename FloatT>
  FloatT wrapDegreesTo0And360(const FloatT angle) {
    const FloatT wrapped_angle = angle - 360 * std::floor(angle / 360);
    return wrapped_angle;
  }

  template <typename FloatT>
  FloatT wrapRadiansToMinusPiAndPi(const FloatT angle) {
    const FloatT angle_0_two_pi = wrapRadiansTo0And2Pi(angle);
    if (angle_0_two_pi >= FloatT(M_PI)) {
      return angle_0_two_pi - 2 * FloatT(M_PI);
    }
    return angle_0_two_pi;
  }

  template <typename FloatT>
  FloatT wrapDegreesToMinus180And180(const FloatT angle) {
    const FloatT angle_0_360 = wrapDegreesTo0And360(angle);
    if (angle_0_360 >= 180) {
      return angle_0_360 - 360;
    }
    return angle_0_360;
  }

  template <typename FloatT1, typename FloatT2>
  bool isApproxEqual(const FloatT1 a, const FloatT2 b,
                     const FloatT1 tolerance = std::numeric_limits<FloatT1>::epsilon()) {
    return std::abs(a - b) <= tolerance;
  }

  template <typename FloatT1, typename FloatT2>
  bool isApproxGreater(const FloatT1 a, const FloatT2 b,
                       const FloatT1 tolerance = std::numeric_limits<FloatT1>::epsilon()) {
    return a > b + tolerance;
  }

  template <typename FloatT1, typename FloatT2>
  bool isApproxGreaterEqual(const FloatT1 a, const FloatT2 b,
                            const FloatT1 tolerance = std::numeric_limits<FloatT1>::epsilon()) {
    return a >= b + tolerance;
  }

  template <typename FloatT1, typename FloatT2>
  bool isApproxSmaller(const FloatT1 a, const FloatT2 b,
                       const FloatT1 tolerance = std::numeric_limits<FloatT1>::epsilon()) {
    return a < b - tolerance;
  }

  template <typename FloatT1, typename FloatT2>
  bool isApproxSmallerEqual(const FloatT1 a, const FloatT2 b,
                            const FloatT1 tolerance = std::numeric_limits<FloatT1>::epsilon()) {
    return a <= b - tolerance;
  }

}
