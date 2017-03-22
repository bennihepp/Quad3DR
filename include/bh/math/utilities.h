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

    template <typename FloatType1, typename FloatType2, typename FloatType3>
    bool isApproxEqual(FloatType1 a, FloatType2 b, FloatType3 tolerance) {
      return std::abs(a - b) <= tolerance;
    }

    template <typename FloatType1, typename FloatType2, typename FloatType3>
    bool isApproxGreater(FloatType1 a, FloatType2 b, FloatType3 tolerance) {
      return a > b + tolerance;
    }

    template <typename FloatType1, typename FloatType2, typename FloatType3>
    bool isApproxGreaterEqual(FloatType1 a, FloatType2 b, FloatType3 tolerance) {
      return a >= b + tolerance;
    }

    template <typename FloatType1, typename FloatType2, typename FloatType3>
    bool isApproxSmaller(FloatType1 a, FloatType2 b, FloatType3 tolerance) {
      return a < b - tolerance;
    }

    template <typename FloatType1, typename FloatType2, typename FloatType3>
    bool isApproxSmallerEqual(FloatType1 a, FloatType2 b, FloatType3 tolerance) {
      return a <= b - tolerance;
    }

}
