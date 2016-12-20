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

inline std::tuple<double, Eigen::Vector3d> computeDistanceAndDirection(const Eigen::Vector3d& from, const Eigen::Vector3d& to) {
    Eigen::Vector3d direction = to - from;
    double distance = direction.norm();
    direction.normalize();
    return std::make_tuple(distance, direction);
}

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

// TODO
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
