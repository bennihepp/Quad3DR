//==================================================
// se3_transform.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 07.04.17
//==================================================
#pragma once

#include <iostream>
#include <bh/eigen.h>
#include <bh/eigen_utils.h>
#include <bh/eigen_serialization.h>
#include <boost/serialization/access.hpp>

namespace bh {

/// A SE3 transform from one coordinate system into another base system.
///
/// The transform is defined by its translation and rotation in the base coordinate system.
/// The transform method transforms a point from the coordinate system into the base system.
template <typename FloatT>
struct SE3Transform {
  using FloatType = FloatT;
  USE_FIXED_EIGEN_TYPES(FloatType)

  SE3Transform()
      : translation_(Vector3::Zero()), quaternion_(Quaternion::Identity()) {}

  SE3Transform(const Vector3& translation, const Quaternion& quaternion)
      : translation_(translation), quaternion_(quaternion) {}

  SE3Transform(const Vector3& translation, const Matrix3x3& rotation)
      : translation_(translation), quaternion_(rotation) {}

  SE3Transform(const Matrix4x4& matrix)
      : translation_(matrix.col(3).template topRows<3>()),
        quaternion_(matrix.template topLeftCorner<3, 3>()) {}

  bool isValid() const {
    return translation_.allFinite()
           && quaternion_.coeffs().allFinite();
  }

  bool operator==(const SE3Transform& other) const {
    return translation_ == other.translation_
           && quaternion_.coeffs() == other.quaternion_.coeffs();
  }

  bool operator!=(const SE3Transform& other) const {
    return !(*this == other);
  }

  template <typename OtherFloatT>
  SE3Transform<OtherFloatT> cast() const {
    return SE3Transform<OtherFloatT>(translation().cast<OtherFloatT>(), quaternion().cast<OtherFloatT>());
  }

  bool isApprox(
          const SE3Transform& other,
          const FloatType precision = Eigen::NumTraits<FloatType>::dummy_precision()) const {
    return translation_.isApprox(other.translation_, precision)
           && quaternion_.isApprox(other.quaternion_, precision);
  }

  Vector3& translation() {
    return translation_;
  }

  const Vector3& translation() const {
    return translation_;
  }

  Quaternion& quaternion() {
    return quaternion_;
  }

  const Quaternion& quaternion() const {
    return quaternion_;
  }

  Matrix3x3 rotation() const {
    return quaternion_.toRotationMatrix();
  }

  Vector3 transform(const Vector3& point) const {
    return quaternion() * point + translation();
  }

  SE3Transform operator*(const SE3Transform& rhs) const {
    return SE3Transform(
            this->quaternion() * rhs.translation() + this->translation(),
            this->quaternion() * rhs.quaternion()
    );
  }
  FloatType getPositionDistanceTo(const SE3Transform& other) const {
    const FloatType pos_distance = (translation_ - other.translation_).norm();
    return pos_distance;
  }

  FloatType getAngularDistanceTo(const SE3Transform& other) const {
    const FloatType angular_distance = quaternion_.angularDistance(other.quaternion_);
    return angular_distance;
  }

  FloatType getDistanceTo(const SE3Transform& other, FloatType angular_factor = 1) const {
    return getPositionDistanceTo(other) + angular_factor * getAngularDistanceTo(other);
  }

  SE3Transform inverse() const {
    SE3Transform inv_transform;
    inv_transform.quaternion() = quaternion_.inverse();
    inv_transform.translation() = - (inv_transform.quaternion() * translation_);
    return inv_transform;
  }

  void invert() {
    quaternion_ = quaternion_.inverse();
    translation_ = - (quaternion() * translation_);
  }

  Matrix3x4 getTransformationMatrix() const {
    Matrix3x4 transformation;
    transformation.template leftCols<3>() = quaternion_.toRotationMatrix();
    transformation.template rightCols<1>() = translation_;
    return transformation;
  }

  Matrix4x4 getTransformationMatrix4x4() const {
    Matrix4x4 transformation;
    transformation.template topRows<3>() = getTransformationMatrix();
    transformation.row(3) = Vector4(0, 0, 0, 1);
    return transformation;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // Boost serialization
  friend class boost::serialization::access;

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & translation_;
    ar & quaternion_;
  }

  Vector3 translation_;
  Quaternion quaternion_;
};

template <typename _CharT, typename FloatT>
std::basic_ostream<_CharT>& operator<<(std::basic_ostream<_CharT>& out, const SE3Transform<FloatT>& se3_transform) {
  out << "t: (" << se3_transform.translation().transpose() << "), q: (" << se3_transform.quaternion() << ")";
  return out;
}

}
