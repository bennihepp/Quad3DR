//==================================================
// eigen_alignment.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 8, 2016
//==================================================
#pragma once

#include<Eigen/StdVector>
#include <unordered_map>

namespace Eigen
{
    using Matrix3x4d = Matrix<double, 3, 4>;
    using Matrix3x4f = Matrix<float, 3, 4>;
}

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3x4f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3x4d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Quaternionf)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Quaterniond)

#define EIGEN_ALIGNED_UNORDERED_MAP(Key, T) std::unordered_map<Key, T, std::hash<Key>, std::equal_to<Key>, \
    Eigen::aligned_allocator<std::pair<Key, T>>>;
#define EIGEN_ALIGNED_UNORDERED_MAP2(Key, T, Hash) std::unordered_map<Key, T, Hash, std::equal_to<Key>, \
    Eigen::aligned_allocator<std::pair<Key, T>>>;
#define EIGEN_ALIGNED_UNORDERED_MAP3(Key, T, Hash, EqualTo) std::unordered_map<Key, T, Hash, EqualTo, \
    Eigen::aligned_allocator<std::pair<Key, T>>>;
