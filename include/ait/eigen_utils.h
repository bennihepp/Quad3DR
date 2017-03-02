//==================================================
// eigen_utils.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 14, 2016
//==================================================
#pragma once

#include <Eigen/Dense>
#include "eigen_alignment.h"

namespace ait
{

template <typename Derived>
typename Derived::PlainObject clamp(
    const Eigen::MatrixBase<Derived>& matrix,
const typename Derived::Scalar min = 0,
const typename Derived::Scalar max = 1) {
  typename Derived::PlainObject min_matrix = typename Derived::PlainObject::Constant(min);
  typename Derived::PlainObject max_matrix = typename Derived::PlainObject::Constant(max);
  typename Derived::PlainObject clamped_matrix = matrix.array().min(min_matrix.array());
  clamped_matrix = clamped_matrix.array().max(max_matrix.array());
  return clamped_matrix;
}

template <typename Derived1, typename Derived2>
std::tuple<typename Derived1::Scalar, typename Derived1::PlainObject> computeDistanceAndDirection(
    const Eigen::MatrixBase<Derived1>& from, const Eigen::MatrixBase<Derived2>& to) {
  typename Derived1::PlainObject direction = to - from;
  double distance = direction.norm();
  direction.normalize();
  return std::make_tuple(distance, direction);
}

template <typename Derived1, typename Derived2>
Eigen::Matrix<typename Derived1::Scalar, 3, 3> getZLookAtMatrix(const Derived1& look_at_direction, const Derived2& up_direction) {
  const typename Derived1::PlainObject norm_look_at_direction = look_at_direction.normalized();
  const typename Derived1::PlainObject norm_up_direction = up_direction.normalized();
  typename Derived1::PlainObject right_direction = Derived1::PlainObject::Zero();
  if (norm_look_at_direction.dot(norm_up_direction) == 0) {
    right_direction(0) = 1;
  }
  else {
    right_direction = norm_look_at_direction.cross(norm_up_direction).normalized();
  }
  typename Derived1::PlainObject orientation_up_direction = -right_direction.cross(norm_look_at_direction);
  Eigen::Matrix<typename Derived1::Scalar, 3, 3> orientation = Eigen::Matrix<typename Derived1::Scalar, 3, 3>::Identity();
  orientation.col(2) = norm_look_at_direction.normalized();
  orientation.col(0) = right_direction.normalized();
  orientation.col(1) = orientation_up_direction.normalized();
  return orientation;
}

template <typename Derived1, typename Derived2>
Eigen::Quaternion<typename Derived1::Scalar> getZLookAtQuaternion(
    const Derived1& look_at_direction, const Derived2& up_direction = typename Derived2::PlainObject::UnitZ()) {
  return Eigen::Quaternion<typename Derived1::Scalar>(getZLookAtMatrix(look_at_direction, up_direction));
}

// TODO: Remove? Functionality already in Eigen
//inline Eigen::Vector3d fromHomogeneousVector(const Eigen::Vector4d& vector) {
//    Eigen::Vector3d inhom_vector(vector.topRows<3>());
//    inhom_vector /= vector(3, 0);
//    return inhom_vector;
//}
//
//inline Eigen::Vector2d fromHomogeneousVector(const Eigen::Vector3d& vector) {
//    Eigen::Vector2d inhom_vector(vector.topRows<2>());
//    inhom_vector /= vector(2, 0);
//    return inhom_vector;
//}
//
//inline Eigen::Vector3d toHomogeneousVector(const Eigen::Vector2d& vector) {
//    Eigen::Vector3d hom_vector;
//    hom_vector.topRows<2>() = vector;
//    hom_vector(2) = 1;
//    return hom_vector;
//}
//
//inline Eigen::Vector4d toHomogeneousVector(const Eigen::Vector3d& vector) {
//    Eigen::Vector4d hom_vector;
//    hom_vector.topRows<3>() = vector;
//    hom_vector(3) = 1;
//    return hom_vector;
//}

// TODO: Fix some compilation issues
//template <typename Derived>
//Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime-1, 1>
//        fromHomogeneousCoordinates(const Eigen::DenseBase<Derived>& vector) {
//    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
//    Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime-1, 1> inhom_vector;
////    inhom_vector = vector.topRows<Derived::RowsAtCompileTime-1>();
//    inhom_vector /= vector(Derived::RowsAtCompileTime - 1, 0);
//    return inhom_vector;
//}

//template <typename _Scalar, int _Rows, int _Options, int _MaxRows, int _MaxCols>
//Eigen::Matrix<_Scalar, _Rows-1, 1> fromHomogeneousCoordinates(const Eigen::Matrix<T, Rows, 1>& vector) {
//    typename Eigen::Matrix<T, Rows-1, 1> inhom_vector;
//    Eigen::Vector4d::Scalar
////    inhom_vector.topRows = vector.topRows<Rows-1>();
////    inhom_vector /= vector(Rows - 1, 0);
//    return inhom_vector;
//}

//template <typename Derived>
//Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime+1, 1>
//        toHomogeneousCoordinates(const Eigen::DenseBase<Derived>& vector) {
//    Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime+1, 1> hom_vector;
//    hom_vector.topRows<Derived::RowsAtCompileTime>() = vector;
//    hom_vector(Derived::RowsAtCompileTime, 0) = 1;
//    return hom_vector;
//}

//template <typename T, int Rows>
//Eigen::Matrix<T, Rows + 1, 1> toHomogeneousCoordinates(const Eigen::Matrix<T, Rows, 1>& vector) {
//    Eigen::Matrix<T, Rows+1, 1> hom_vector;
////    hom_vector.topRows<Rows>() = vector;
////    hom_vector(Rows, 0) = 1;
//    return hom_vector;
//}

}

namespace Eigen {
  template <typename _CharT, typename Scalar>
  std::basic_ostream<_CharT>& operator<<(std::basic_ostream<_CharT>& out, const Eigen::Quaternion<Scalar>& quat) {
    out << quat.w() << ", " << quat.x() << ", " << quat.y() << ", " << quat.z();
    return out;
  }
}
