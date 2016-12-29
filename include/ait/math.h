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
    bool isApproxEqual(FloatType a, FloatType b, FloatType tolerance) {
      return std::abs(a - b) <= tolerance;
    }
}
