//==================================================
// pose.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 9, 2016
//==================================================
#pragma once

#include <ait/eigen_utils.h>

namespace ait {

// A pose gives the transformation from world to image coordinate system
struct Pose
{
  using FloatType = double;
  using Quaternion = Eigen::Quaternion<FloatType>;
  using Vector3 = Eigen::Matrix<FloatType, 3, 1>;
  using Matrix4x4 = Eigen::Matrix<FloatType, 4, 4>;
  using Matrix3x3 = Eigen::Matrix<FloatType, 3, 3>;
  using Matrix3x4 = Eigen::Matrix<FloatType, 3, 4>;

  Pose()
  : translation_(0, 0, 0), quaternion_(1, 0, 0, 0) {}

  Pose(const Vector3& translation, const Quaternion& quaternion)
  : translation_(translation), quaternion_(quaternion) {}

  Pose(const Vector3& translation, const Matrix3x3& rotation)
  : translation_(translation), quaternion_(rotation) {}

  Pose(const Matrix4x4& matrix)
  : translation_(matrix.col(3).topRows<3>()), quaternion_(matrix.topLeftCorner<3, 3>()) {}

  static Pose createFromWorldToImageTransformation(
      const Vector3& translation, const Matrix3x3& rotation) {
    return Pose(translation, rotation).inverse();
  }

  static Pose createFromWorldToImageTransformation(
      const Vector3& translation, const Quaternion& quaternion) {
    return Pose(translation, quaternion).inverse();
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

  Vector3 getWorldPosition() const {
    return - (quaternion_.inverse() * translation_);
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

  Matrix3x4 getTransformationWorldToImage() const {
    Matrix3x4 transformation;
    transformation.leftCols<3>() = quaternion_.toRotationMatrix();
    transformation.rightCols<1>() = translation_;
    return transformation;
  }

  Matrix3x4 getTransformationImageToWorld() const {
    Matrix3x4 transformation_world_to_camera = getTransformationWorldToImage();
    Matrix3x4 transformation;
    transformation.leftCols<3>() = transformation_world_to_camera.leftCols<3>().transpose();
    transformation.rightCols<1>() = - transformation.leftCols<3>() * transformation_world_to_camera.rightCols<1>();
    return transformation;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Vector3 translation_;
  Quaternion quaternion_;
};

}
