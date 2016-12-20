//==================================================
// vision_utilities.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 12, 2016
//==================================================
#pragma once

#include "eigen.h"

namespace ait {

// -------------------
// Camera utilities
//--------------------

/// Scale 3x3 intrinsics matrix
template <typename Derived, typename std::enable_if<Derived::RowsAtCompileTime == 3, bool>::type = false>
typename Derived::PlainObject getScaledIntrinsics(const Eigen::MatrixBase<Derived>& intrinsics, typename Derived::Scalar scale_factor) {
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == Derived::ColsAtCompileTime, INTRINSICS_MATRIX_MUST_BE_SQUARE);
  typename Derived::PlainObject image_transform = Derived::PlainObject::Identity();
  image_transform(0, 0) = scale_factor;
  image_transform(1, 1) = scale_factor;
  image_transform(0, 2) = scale_factor * 0.5 - 0.5;
  image_transform(1, 2) = scale_factor * 0.5 - 0.5;
  typename Derived::PlainObject scaled_intrinsics = image_transform * intrinsics;
  return scaled_intrinsics;
}

/// Scale 4x4 intrinsics matrix
template <typename Derived, typename std::enable_if<Derived::RowsAtCompileTime == 4, bool>::type = false>
typename Derived::PlainObject getScaledIntrinsics(const Eigen::DenseBase<Derived>& intrinsics, typename Derived::Scalar scale_factor) {
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == Derived::ColsAtCompileTime, INTRINSICS_MATRIX_MUST_BE_SQUARE);
  typename Derived::PlainObject scaled_intrinsics = Derived::PlainObject::Identity();
  scaled_intrinsics.template topLeftCorner<3, 3>() = getScaledIntrinsics(intrinsics.template topLeftCorner<3, 3>(), scale_factor);
  return scaled_intrinsics;
}

//template <typename Derived1, typename Derived2>
//Eigen::Vector2d projectPointIntoImage(const Eigen::MatrixBase<Derived1>& hom_point_camera, const Eigen::MatrixBase<Derived2>& intrinsics) const {
//  EIGEN_STATIC_ASSERT(Derived1::RowsAtCompileTime == 3, CAMERA_POINT_MUST_HAVE_3_ROWS);
//  Eigen::Matrix<typename Derived1::Scalar, Derived1::RowsAtCompileTime-1, 1> point_camera(hom_point_camera.hnormalized());
//  Eigen::Matrix<typename Derived1::Scalar, Derived1::RowsAtCompileTime-1, 1> point_image;
//  point_image(0) = intrinsics(0, 0) * point_camera(0) + intrinsics(0, 2);
//  point_image(1) = intrinsics(1, 1) * point_camera(1) + intrinsics(1, 2);
//  return point_image;
//}

}
