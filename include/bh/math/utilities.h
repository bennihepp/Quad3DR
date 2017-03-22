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

    template <typename FloatType>
    FloatType degreeToRadians(const FloatType degrees) {
      const FloatType pi = (FloatType)M_PI;
      return degrees * pi / 180;
    }

    template <typename FloatType>
    FloatType radiansToDegrees(const FloatType radians) {
      const FloatType pi = (FloatType)M_PI;
      return radians * 180 / pi;
    }

    template <typename FloatType1, typename FloatType2>
    bool isApproxEqual(const FloatType1 a, const FloatType2 b,
                       const FloatType1 tolerance = std::numeric_limits<FloatType1>::epsilon()) {
      return std::abs(a - b) <= tolerance;
    }

    template <typename FloatType1, typename FloatType2>
    bool isApproxGreater(const FloatType1 a, const FloatType2 b,
                         const FloatType1 tolerance = std::numeric_limits<FloatType1>::epsilon()) {
      return a > b + tolerance;
    }

    template <typename FloatType1, typename FloatType2>
    bool isApproxGreaterEqual(const FloatType1 a, const FloatType2 b,
                              const FloatType1 tolerance = std::numeric_limits<FloatType1>::epsilon()) {
      return a >= b + tolerance;
    }

    template <typename FloatType1, typename FloatType2>
    bool isApproxSmaller(const FloatType1 a, const FloatType2 b,
                         const FloatType1 tolerance = std::numeric_limits<FloatType1>::epsilon()) {
      return a < b - tolerance;
    }

    template <typename FloatType1, typename FloatType2>
    bool isApproxSmallerEqual(const FloatType1 a, const FloatType2 b,
                              const FloatType1 tolerance = std::numeric_limits<FloatType1>::epsilon()) {
      return a <= b - tolerance;
    }

}
