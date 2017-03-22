//==================================================
// math.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Nov 8, 2016
//==================================================
#pragma once

#include <cmath>
#include <Eigen/Dense>
#include "eigen.h"

namespace ait {

    using Mat4f = Eigen::Matrix<float, 4, 4>;
    using Mat4d = Eigen::Matrix<double, 4, 4>;
    using Mat3f = Eigen::Matrix<float, 3, 3>;
    using Mat3d = Eigen::Matrix<double, 3, 3>;

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
