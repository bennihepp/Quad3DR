//==================================================
// eigen.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 12, 2016
//==================================================
#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>
#include <type_traits>
#include <boost/functional/hash.hpp>
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace bh {

  template <typename Derived>
  struct is_eigen_type : public std::integral_constant<bool,
                          std::is_base_of<Eigen::EigenBase<Derived>, Derived>::value> {};

  template <typename Derived>
  struct is_eigen_quaternion_type : public std::integral_constant<bool,
                          std::is_base_of<Eigen::QuaternionBase<Derived>, Derived>::value> {};

  template <typename T, typename std::enable_if<is_eigen_type<T>::value>::type* = nullptr>
  std::size_t eigen_hash(const T& v) {
    size_t val { 0 };
    for (int i = 0; i < v.size(); ++i) {
      boost::hash_combine(val, boost::hash<float>{}(v(i)));
    }
    return val;
  }

  template <typename T, typename std::enable_if<is_eigen_quaternion_type<T>::value>::type* = nullptr>
  std::size_t eigen_hash(const T& v) {
    size_t val { 0 };
    for (int i = 0; i < v.coeffs().size(); ++i) {
      boost::hash_combine(val, boost::hash<float>{}(v.coeffs()(i)));
    }
    return val;
  }

  template <typename FloatT>
  using Vector2 = Eigen::Matrix<FloatT, 2, 1>;

  template <typename FloatT>
  using Vector3 = Eigen::Matrix<FloatT, 3, 1>;

  template <typename FloatT>
  using Vector4 = Eigen::Matrix<FloatT, 4, 1>;

  template <typename FloatT>
  using Matrix2 = Eigen::Matrix<FloatT, 2, 2>;

  template <typename FloatT>
  using Matrix3 = Eigen::Matrix<FloatT, 3, 3>;

  template <typename FloatT>
  using Matrix4 = Eigen::Matrix<FloatT, 4, 4>;
}

namespace Eigen {

using Matrix3x4d = Matrix<double, 3, 4>;
using Matrix3x4f = Matrix<float, 3, 4>;
using Vector2s = Matrix<std::size_t, 3, 1>;
using Vector3s = Matrix<std::size_t, 3, 1>;
using Vector4s = Matrix<std::size_t, 3, 1>;

}

#define BH_EIGEN_DEFINE_STD_HASH_SPECIALIZATION(T)    \
namespace std {                                       \
template<>                                            \
struct hash<T> {                                      \
  std::size_t operator()(const T& v) const {          \
    return bh::eigen_hash(v);                             \
  }                                                   \
};                                                    \
}

#define BH_EIGEN_SPECIALIZE(T)                        \
    BH_EIGEN_DEFINE_STD_HASH_SPECIALIZATION(T)        \
    EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(T)

BH_EIGEN_SPECIALIZE(Eigen::Vector2f)
BH_EIGEN_SPECIALIZE(Eigen::Vector2d)
BH_EIGEN_SPECIALIZE(Eigen::Vector3f)
BH_EIGEN_SPECIALIZE(Eigen::Vector3d)
BH_EIGEN_SPECIALIZE(Eigen::Vector4f)
BH_EIGEN_SPECIALIZE(Eigen::Vector4d)
BH_EIGEN_SPECIALIZE(Eigen::Matrix2f)
BH_EIGEN_SPECIALIZE(Eigen::Matrix2d)
BH_EIGEN_SPECIALIZE(Eigen::Matrix3f)
BH_EIGEN_SPECIALIZE(Eigen::Matrix3d)
BH_EIGEN_SPECIALIZE(Eigen::Matrix4f)
BH_EIGEN_SPECIALIZE(Eigen::Matrix4d)
BH_EIGEN_SPECIALIZE(Eigen::Matrix3x4f)
BH_EIGEN_SPECIALIZE(Eigen::Matrix3x4d)
BH_EIGEN_SPECIALIZE(Eigen::Quaternionf)
BH_EIGEN_SPECIALIZE(Eigen::Quaterniond)

#define EIGEN_ALIGNED_VECTOR(T) std::vector<T, Eigen::aligned_allocator<T>>;
#define EIGEN_ALIGNED_UNORDERED_MAP(Key, T) std::unordered_map<Key, T, std::hash<Key>, std::equal_to<Key>, \
    Eigen::aligned_allocator<std::pair<Key, T>>>;
#define EIGEN_ALIGNED_UNORDERED_MAP2(Key, T, Hash) std::unordered_map<Key, T, Hash, std::equal_to<Key>, \
    Eigen::aligned_allocator<std::pair<Key, T>>>;
#define EIGEN_ALIGNED_UNORDERED_MAP3(Key, T, Hash, EqualTo) std::unordered_map<Key, T, Hash, EqualTo, \
    Eigen::aligned_allocator<std::pair<Key, T>>>;

#define BH_USE_FIXED_EIGEN_TYPES(FloatType) \
    using Vector2 = Eigen::Matrix<FloatType, 2, 1>; \
    using Vector3 = Eigen::Matrix<FloatType, 3, 1>; \
    using Vector4 = Eigen::Matrix<FloatType, 4, 1>; \
    using ColumnVector2 = Eigen::Matrix<FloatType, 1, 2>; \
    using ColumnVector3 = Eigen::Matrix<FloatType, 1, 3>; \
    using ColumnVector4 = Eigen::Matrix<FloatType, 1, 4>; \
    using Matrix4x4 = Eigen::Matrix<FloatType, 4, 4>; \
    using Matrix3x3 = Eigen::Matrix<FloatType, 3, 3>; \
    using Matrix2x2 = Eigen::Matrix<FloatType, 2, 2>; \
    using Matrix3x4 = Eigen::Matrix<FloatType, 3, 4>; \
    using MatrixDynamic = Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic>; \
    using VectorDynamic = Eigen::Matrix<FloatType, Eigen::Dynamic, 1>; \
    using ColumnVectorDynamic = Eigen::Matrix<FloatType, 1, Eigen::Dynamic>; \
    using Quaternion = Eigen::Quaternion<FloatType>; \
    using AngleAxis = Eigen::AngleAxis<FloatType>;

#define USE_FIXED_EIGEN_TYPES(FloatType) BH_USE_FIXED_EIGEN_TYPES(FloatType);

namespace bh {

template<typename FloatT>
class EigenTypes {
public:
  EigenTypes() = delete;

  using FloatType = FloatT;
  USE_FIXED_EIGEN_TYPES(FloatT);
};

}
