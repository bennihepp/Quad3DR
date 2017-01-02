//==================================================
// pose.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 9, 2016
//==================================================
#pragma once

#include <iostream>
#include <ait/eigen.h>
#include <ait/eigen_utils.h>
#include <ait/eigen_serialization.h>
#include <boost/serialization/access.hpp>

namespace ait {

// A pose gives the transformation from world to image coordinate system
template <typename T>
struct Pose {
  using FloatType = T;
  USE_FIXED_EIGEN_TYPES(FloatType)

  Pose()
  : translation_(0, 0, 0), quaternion_(1, 0, 0, 0) {}

  Pose(const Vector3& translation, const Quaternion& quaternion)
  : translation_(translation), quaternion_(quaternion) {}

  Pose(const Vector3& translation, const Matrix3x3& rotation)
  : translation_(translation), quaternion_(rotation) {}

  Pose(const Matrix4x4& matrix)
  : translation_(matrix.col(3).template topRows<3>()),
    quaternion_(matrix.template topLeftCorner<3, 3>()) {}

  static Pose createFromImageToWorldTransformation(
      const Vector3& translation, const Matrix3x3& rotation) {
    return Pose(translation, rotation);
  }

  static Pose createFromImageToWorldTransformation(
      const Vector3& translation, const Quaternion& quaternion) {
    return Pose(translation, quaternion);
  }

  static Pose createFromWorldToImageTransformation(
      const Vector3& translation, const Matrix3x3& rotation) {
    return Pose(translation, rotation).inverse();
  }

  static Pose createFromWorldToImageTransformation(
      const Vector3& translation, const Quaternion& quaternion) {
    return Pose(translation, quaternion).inverse();
  }

  bool isValid() const {
    return translation_.allFinite() && quaternion_.coeffs().allFinite();
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

  FloatType getDistanceTo(const Pose& other, FloatType rotation_factor = 1) const {
    FloatType pos_distance = (translation_ - other.translation_).norm();
    FloatType angular_distance = quaternion_.angularDistance(other.quaternion_);
    return pos_distance + rotation_factor * angular_distance;
  }

  const Vector3& getWorldPosition() const {
    return translation_;
//    return - (quaternion_.inverse() * translation_);
  }

  Pose inverse() const {
    Pose inv_pose;
    inv_pose.quaternion() = quaternion_.inverse();
    inv_pose.translation() = - (inv_pose.quaternion() * translation_);
    return inv_pose;
  }

  Matrix3x3 rotation() const {
    return quaternion_.toRotationMatrix();
  }

  Matrix3x4 getTransformationImageToWorld() const {
    Matrix3x4 transformation;
    transformation.template leftCols<3>() = quaternion_.toRotationMatrix();
    transformation.template rightCols<1>() = translation_;
    return transformation;
  }

  Matrix3x4 getTransformationWorldToImage() const {
    Matrix3x4 transformation;
    transformation.template leftCols<3>() = quaternion_.inverse().toRotationMatrix();
    transformation.template rightCols<1>() = - transformation.template leftCols<3>() * translation_;
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

template <typename _CharT, typename FloatType>
std::basic_ostream<_CharT>& operator<<(std::basic_ostream<_CharT>& out, const Pose<FloatType>& pose) {
  out << "t: (" << pose.translation().transpose() << "), q: (" << pose.quaternion() << ")" << std::endl;
  return out;
}

}
