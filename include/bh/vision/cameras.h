//==================================================
// cameras.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 05.04.17
//==================================================
#pragma once

#include <bh/eigen.h>

namespace bh {
namespace vision {

// -------------------
// Camera utilities
//--------------------

/// Scale 3x3 intrinsics matrix
template <typename Derived, typename std::enable_if<Derived::RowsAtCompileTime == 3, bool>::type = false>
typename Derived::PlainObject getScaledIntrinsics(
        const Eigen::MatrixBase<Derived>& intrinsics, typename Derived::Scalar scale_factor);

/// Scale 4x4 intrinsics matrix
template <typename Derived, typename std::enable_if<Derived::RowsAtCompileTime == 4, bool>::type = false>
typename Derived::PlainObject getScaledIntrinsics(
        const Eigen::DenseBase<Derived>& intrinsics, typename Derived::Scalar scale_factor);

//template <typename Derived1, typename Derived2>
//Eigen::Vector2d projectPointIntoImage(
//        const Eigen::MatrixBase<Derived1>& hom_point_camera, const Eigen::MatrixBase<Derived2>& intrinsics) const;

// -------------------
// Camera classes
//--------------------

template <typename FloatT>
struct PinholeCamera {
  using FloatType = FloatT;
  USE_FIXED_EIGEN_TYPES(FloatType);

  using IntrinsicsMatrix = Matrix4x4;

  using IntrinsicsMatrix3x3 = Matrix3x3;

  static PinholeCamera createSimple(const size_t width, const size_t height, const FloatType focal_length);

  static PinholeCamera createSimple(const size_t width, const size_t height,
                                   const FloatType focal_length_x, const FloatType focal_length_y);

  static PinholeCamera createSimple(const size_t width, const size_t height,
                                   const FloatType focal_length_x, const FloatType focal_length_y,
                                   const FloatType principal_point_x, const FloatType principal_point_y);

  PinholeCamera();

  PinholeCamera(const size_t width, const size_t height, const IntrinsicsMatrix& intrinsics);

  PinholeCamera(const PinholeCamera& camera) = default;

  PinholeCamera(PinholeCamera&& camera) = default;

  PinholeCamera& operator=(const PinholeCamera& other) = default;

  PinholeCamera& operator=(PinholeCamera&& other) = default;

  PinholeCamera getScaledCamera(const FloatType scale_factor) const;

  bool operator==(const PinholeCamera& other) const;

  bool operator!=(const PinholeCamera& other) const;

  size_t width() const { return width_; }

  size_t& width() { return width_; }

  size_t height() const { return height_; }

  size_t& height() { return height_; }

  const IntrinsicsMatrix& intrinsics() const { return intrinsics_; }

  IntrinsicsMatrix& intrinsics() { return intrinsics_; }

  IntrinsicsMatrix3x3 intrinsics3x3() const;

  FloatType focalLengthX() const { return intrinsics()(0, 0); }

  FloatType focalLengthY() const { return intrinsics()(1, 1); }

  FloatType principalPointX() const { return intrinsics()(0, 2); }

  FloatType principalPointY() const { return intrinsics()(1, 2); }

  bool isInsideImage(const Vector2& image_point) const;

  Vector2 imageToWorld(const Vector2& image_point) const;

  Vector2 worldToImage(const Vector2& world_point) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
  size_t width_;
  size_t height_;
  IntrinsicsMatrix intrinsics_;
};

template <typename FloatT>
class OpenCVCamera {
public:
  using FloatType = FloatT;
  USE_FIXED_EIGEN_TYPES(FloatType);

  using IntrinsicsMatrix = Matrix4x4;

  using IntrinsicsMatrix3x3 = Matrix3x3;

  static OpenCVCamera createSimple(const size_t width, const size_t height, const FloatType focal_length);

  static OpenCVCamera createSimple(const size_t width, const size_t height,
                                   const FloatType focal_length_x, const FloatType focal_length_y);

  static OpenCVCamera createSimple(const size_t width, const size_t height,
                                   const FloatType focal_length_x, const FloatType focal_length_y,
                                   const FloatType principal_point_x, const FloatType principal_point_y);

  OpenCVCamera();

  explicit OpenCVCamera(const PinholeCamera<FloatT>& pinhole_camera);

  OpenCVCamera(const size_t width, const size_t height, const IntrinsicsMatrix& intrinsics);

  OpenCVCamera(const OpenCVCamera& camera) = default;

  OpenCVCamera(OpenCVCamera&& camera) = default;

  OpenCVCamera& operator=(const OpenCVCamera& other) = default;

  OpenCVCamera& operator=(OpenCVCamera&& other) = default;

  OpenCVCamera getScaledCamera(const FloatType scale_factor) const;

  bool operator==(const OpenCVCamera& other) const;

  bool operator!=(const OpenCVCamera& other) const;

  size_t width() const { return pinhole_camera_.width(); }

  size_t& width() { return pinhole_camera_.width(); }

  size_t height() const { return pinhole_camera_.height(); }

  size_t& height() { return pinhole_camera_.height(); }

  const IntrinsicsMatrix& intrinsics() const { return pinhole_camera_.intrinsics(); }

  IntrinsicsMatrix & intrinsics() { return pinhole_camera_.intrinsics(); }

  IntrinsicsMatrix3x3 intrinsics3x3() const { return pinhole_camera_.intrinsics3x3(); }

  FloatType focalLengthX() const { return intrinsics()(0, 0); }

  FloatType focalLengthY() const { return intrinsics()(1, 1); }

  FloatType principalPointX() const { return intrinsics()(0, 2); }

  FloatType principalPointY() const { return intrinsics()(1, 2); }

  FloatType k1() const { return k1_; }

  FloatType& k1() { return k1_; }

  FloatType k2() const { return k2_; }

  FloatType& k2() { return k2_; }

  FloatType k3() const { return k3_; }

  FloatType& k3() { return k3_; }

  FloatType p1() const { return p1_; }

  FloatType& p1() { return p1_; }

  FloatType p2() const { return p2_; }

  FloatType& p2() { return p2_; }

  bool isInsideImage(const Vector2& image_point) const { return pinhole_camera_.isInsideImage(image_point); }

  Vector2 imageToWorld(const Vector2& image_point) const;

  Vector2 imageToWorldWithoutUndistortion(const Vector2& image_point) const;

  Vector2 worldToImage(const Vector2& world_point) const;

  Vector2 worldToImageWithoutDistortion(const Vector2& world_point) const;

  Vector2 distortionOffsetsForWorldPoint(const Vector2& world_point) const;

  Vector2 distortWorldPoint(const Vector2& world_point) const;

  Vector2 undistortWorldPoint(const Vector2& world_point) const;

  const std::array<FloatType, 5>& distCoeffs() const { return dist_coeffs_; };

  std::array<FloatType, 5>& distCoeffs() { return dist_coeffs_; };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
  PinholeCamera<FloatT> pinhole_camera_;
  union {
    struct {
      FloatType k1_; // Radial 1
      FloatType k2_; // Radial 2
      FloatType p1_; // Tangential 1
      FloatType p2_; // Tangential 2
      FloatType k3_; // Radial 3
    };
    std::array<FloatType, 5> dist_coeffs_;
  };
};

// -------------------
// Camera utilities implementation
//--------------------

template <typename Derived, typename std::enable_if<Derived::RowsAtCompileTime == 3, bool>::type>
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

template <typename Derived, typename std::enable_if<Derived::RowsAtCompileTime == 4, bool>::type>
typename Derived::PlainObject getScaledIntrinsics(const Eigen::DenseBase<Derived>& intrinsics, typename Derived::Scalar scale_factor) {
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == Derived::ColsAtCompileTime, INTRINSICS_MATRIX_MUST_BE_SQUARE);
  typename Derived::PlainObject scaled_intrinsics = Derived::PlainObject::Identity();
  scaled_intrinsics.template topLeftCorner<3, 3>() = getScaledIntrinsics(intrinsics.template topLeftCorner<3, 3>(), scale_factor);
  return scaled_intrinsics;
}

//template <typename Derived1, typename Derived2>
//Eigen::Vector2d projectPointIntoImage(
//        const Eigen::MatrixBase<Derived1>& hom_point_camera, const Eigen::MatrixBase<Derived2>& intrinsics) const {
//  EIGEN_STATIC_ASSERT(Derived1::RowsAtCompileTime == 3, CAMERA_POINT_MUST_HAVE_3_ROWS);
//  Eigen::Matrix<typename Derived1::Scalar, Derived1::RowsAtCompileTime-1, 1> point_camera(hom_point_camera.hnormalized());
//  Eigen::Matrix<typename Derived1::Scalar, Derived1::RowsAtCompileTime-1, 1> point_image;
//  point_image(0) = intrinsics(0, 0) * point_camera(0) + intrinsics(0, 2);
//  point_image(1) = intrinsics(1, 1) * point_camera(1) + intrinsics(1, 2);
//  return point_image;
//}

// -------------------
// Camera classes implementation
//--------------------

template <typename FloatT>
auto PinholeCamera<FloatT>::createSimple(
        const size_t width, const size_t height, const FloatType focal_length) -> PinholeCamera {
  return createSimple(width, height, focal_length, focal_length);
}

template <typename FloatT>
auto PinholeCamera<FloatT>::createSimple(
        const size_t width, const size_t height,
        const FloatType focal_length_x, const FloatType focal_length_y) -> PinholeCamera {
  return createSimple(width, height, focal_length_x, focal_length_y, width / 2, height / 2);
}

template <typename FloatT>
auto PinholeCamera<FloatT>::createSimple(
        const size_t width, const size_t height,
        const FloatType focal_length_x, const FloatType focal_length_y,
        const FloatType principal_point_x, const FloatType principal_point_y) -> PinholeCamera {
  IntrinsicsMatrix intrinsics = IntrinsicsMatrix::Identity();
  intrinsics(0, 0) = focal_length_x;
  intrinsics(1, 1) = focal_length_y;
  intrinsics(0, 2) = principal_point_x;
  intrinsics(1, 2) = principal_point_y;
  PinholeCamera camera(width, height, intrinsics);
  return camera;
}

template <typename FloatT>
PinholeCamera<FloatT>::PinholeCamera()
    : width_(0), height_(0) {
  intrinsics_.setIdentity();
}

template <typename FloatT>
PinholeCamera<FloatT>::PinholeCamera(const size_t width, const size_t height, const IntrinsicsMatrix& intrinsics)
    : width_(width), height_(height), intrinsics_(intrinsics) {}

template <typename FloatT>
auto PinholeCamera<FloatT>::getScaledCamera(const FloatType scale_factor) const -> PinholeCamera {
  // Distortion coefficients are in normalized world coordinates (z=1 plane)
  PinholeCamera other = *this;
  other.width() = (size_t)(width() * scale_factor);
  other.height() = (size_t)(height() * scale_factor);
  other.intrinsics() = getScaledIntrinsics(intrinsics(), scale_factor);
  return other;
}

template <typename FloatT>
bool PinholeCamera<FloatT>::operator==(const PinholeCamera& other) const {
  return width() == other.width()
         && height() == other.height()
         && intrinsics() == other.intrinsics();
}

template <typename FloatT>
bool PinholeCamera<FloatT>::operator!=(const PinholeCamera& other) const {
  return !(*this == other);
}


template <typename FloatT>
auto PinholeCamera<FloatT>::intrinsics3x3() const -> IntrinsicsMatrix3x3 {
  return intrinsics().template topLeftCorner<3, 3>();
}

template <typename FloatT>
bool PinholeCamera<FloatT>::isInsideImage(const Vector2& image_point) const {
  return 0 <= image_point(0)
         && image_point(0) < width()
         && 0 <= image_point(1)
         && image_point(1) < height();
}

template <typename FloatT>
auto PinholeCamera<FloatT>::imageToWorld(const Vector2& image_point) const -> Vector2 {
  const FloatType f0 = intrinsics()(0, 0);
  const FloatType f1 = intrinsics()(1, 1);
  const FloatType c0 = intrinsics()(0, 2);
  const FloatType c1 = intrinsics()(1, 2);
  Vector2 world_point(
          (image_point(0) - c0) / f0,
          (image_point(1) - c1) / f1);
  return world_point;
}

template <typename FloatT>
auto PinholeCamera<FloatT>::worldToImage(const Vector2& world_point) const -> Vector2 {
  const FloatType f0 = intrinsics()(0, 0);
  const FloatType f1 = intrinsics()(1, 1);
  const FloatType c0 = intrinsics()(0, 2);
  const FloatType c1 = intrinsics()(1, 2);
  const Vector2 image_point(
          f0 * world_point(0) + c0,
          f1 * world_point(1) + c1);
  return image_point;
}


template <typename FloatT>
auto OpenCVCamera<FloatT>::createSimple(
        const size_t width, const size_t height, const FloatType focal_length) -> OpenCVCamera {
  return createSimple(width, height, focal_length, focal_length);
}

template <typename FloatT>
auto OpenCVCamera<FloatT>::createSimple(
        const size_t width, const size_t height,
        const FloatType focal_length_x, const FloatType focal_length_y) -> OpenCVCamera {
  return createSimple(width, height, focal_length_x, focal_length_y, 0, 0);
}

template <typename FloatT>
auto OpenCVCamera<FloatT>::createSimple(
        const size_t width, const size_t height,
        const FloatType focal_length_x, const FloatType focal_length_y,
        const FloatType principal_point_x, const FloatType principal_point_y) -> OpenCVCamera {
  OpenCVCamera camera(PinholeCamera<FloatT>::createSimple(
          width, height, focal_length_x, focal_length_y, principal_point_x, principal_point_y));
  return camera;
}

template <typename FloatT>
OpenCVCamera<FloatT>::OpenCVCamera()
        : k1_(0), k2_(0), p1_(0), p2_(0), k3_(0) {}

template <typename FloatT>
OpenCVCamera<FloatT>::OpenCVCamera(const size_t width, const size_t height, const IntrinsicsMatrix& intrinsics)
        : pinhole_camera_(width, height, intrinsics), k1_(0), k2_(0), p1_(0), p2_(0), k3_(0) {}

template <typename FloatT>
OpenCVCamera<FloatT>::OpenCVCamera(const PinholeCamera<FloatT>& pinhole_camera)
        : pinhole_camera_(pinhole_camera), k1_(0), k2_(0), p1_(0), p2_(0), k3_(0) {}

template <typename FloatT>
auto OpenCVCamera<FloatT>::getScaledCamera(const FloatType scale_factor) const -> OpenCVCamera {
  // Distortion coefficients are in normalized world coordinates (z=1 plane)
  OpenCVCamera other = *this;
  other.width() = (size_t)(width() * scale_factor);
  other.height() = (size_t)(height() * scale_factor);
  other.intrinsics() = getScaledIntrinsics(intrinsics(), scale_factor);
  return other;
}

template <typename FloatT>
bool OpenCVCamera<FloatT>::operator==(const OpenCVCamera& other) const {
  return pinhole_camera_ == other.pinhole_camera_
         && std::equal(distCoeffs().begin(), distCoeffs().end(),
                       other.distCoeffs().begin());
}

template <typename FloatT>
bool OpenCVCamera<FloatT>::operator!=(const OpenCVCamera& other) const {
  return !(*this == other);
}

template <typename FloatT>
auto OpenCVCamera<FloatT>::imageToWorldWithoutUndistortion(const Vector2& image_point) const -> Vector2 {
  const FloatType f0 = intrinsics()(0, 0);
  const FloatType f1 = intrinsics()(1, 1);
  const FloatType c0 = intrinsics()(0, 2);
  const FloatType c1 = intrinsics()(1, 2);
  Vector2 world_point(
          (image_point(0) - c0) / f0,
          (image_point(1) - c1) / f1);
  return world_point;
}

template <typename FloatT>
auto OpenCVCamera<FloatT>::imageToWorld(const Vector2& image_point) const -> Vector2 {
  return undistortWorldPoint(imageToWorldWithoutUndistortion(image_point));
}

template <typename FloatT>
auto OpenCVCamera<FloatT>::worldToImageWithoutDistortion(const Vector2& world_point) const -> Vector2 {
  const FloatType f0 = intrinsics()(0, 0);
  const FloatType f1 = intrinsics()(1, 1);
  const FloatType c0 = intrinsics()(0, 2);
  const FloatType c1 = intrinsics()(1, 2);
  const Vector2 image_point(
          f0 * world_point(0) + c0,
          f1 * world_point(1) + c1);
  return image_point;
}

template <typename FloatT>
auto OpenCVCamera<FloatT>::worldToImage(const Vector2& world_point) const -> Vector2 {
  return worldToImageWithoutDistortion(distortWorldPoint(world_point));
}

template <typename FloatT>
auto OpenCVCamera<FloatT>::distortionOffsetsForWorldPoint(const Vector2& world_point) const -> Vector2 {
  const FloatType u = world_point(0);
  const FloatType v = world_point(1);
  const FloatType r2 = world_point.squaredNorm();
  const FloatType r4 = r2 * r2;
  const FloatType r6 = r2 * r4;
  const FloatType radial_dist_factor = k1() * r2 + k2() * r4 + k3() * r6;
  const FloatType tangential_dist_x = 2 * p1() * u * v + p2() * (r2 + 2 * u * u);
  const FloatType tangential_dist_y = 2 * p2() * u * v + p1() * (r2 + 2 * v * v);
  const Vector2 distortion_offsets = world_point * radial_dist_factor +
                                     Vector2(tangential_dist_x, tangential_dist_y);
  return distortion_offsets;
}

template <typename FloatT>
auto OpenCVCamera<FloatT>::distortWorldPoint(const Vector2& world_point) const -> Vector2 {
  return world_point + distortionOffsetsForWorldPoint(world_point);
}

template <typename FloatT>
auto OpenCVCamera<FloatT>::undistortWorldPoint(const Vector2& world_point) const -> Vector2 {
  // Solve x_undistorted + x_distortion_offset(x_undistorted) - x_distored == 0 for x_undistorted.
  // We perform Newton method's with numerical derivatives
  const FloatType kMinStepSize = std::numeric_limits<FloatType>::epsilon();
  const size_t kMaxNumIterations = 100;
  const FloatType kSquaredNormTolerance = FloatType(1e-10);
  const FloatType kRelativeStepSize = FloatType(1e-6);

  const Vector2 x0 = world_point;

  Eigen::Matrix<FloatType, 2, 2> jacobian;
  Vector2 x = x0;

  for (size_t i = 0; i < kMaxNumIterations; ++i) {
    const FloatType step_size_0 = std::max(kMinStepSize, std::abs(kRelativeStepSize * x(0)));
    const FloatType step_size_1 = std::max(kMinStepSize, std::abs(kRelativeStepSize * x(1)));
    const Vector2 dist_offsets = distortionOffsetsForWorldPoint(x);
    const Vector2 dist_offsets_0_lower = distortionOffsetsForWorldPoint(x - Vector2(step_size_0, 0));
    const Vector2 dist_offsets_0_upper = distortionOffsetsForWorldPoint(x + Vector2(step_size_0, 0));
    const Vector2 dist_offsets_1_lower = distortionOffsetsForWorldPoint(x - Vector2(0, step_size_1));
    const Vector2 dist_offsets_1_upper = distortionOffsetsForWorldPoint(x + Vector2(0, step_size_1));
    const Vector2 dist_offsets_d0 = (dist_offsets_0_upper - dist_offsets_0_lower) / (2 * step_size_0);
    const Vector2 dist_offsets_d1 = (dist_offsets_1_upper - dist_offsets_1_lower) / (2 * step_size_1);
    // Set jacobian entries
    jacobian(0, 0) = 1 + dist_offsets_d0(0);
    jacobian(0, 1) = dist_offsets_d1(0);
    jacobian(1, 0) = dist_offsets_d0(1);
    jacobian(1, 1) = 1 + dist_offsets_d1(1);
    // Perform step in negative gradient direction
    const Vector2 step_x = - jacobian.inverse() * (x + dist_offsets - x0);
    x += step_x;
    if (step_x.squaredNorm() < kSquaredNormTolerance) {
      break;
    }
  }

  const Vector2 undistorted_point = x;
  return undistorted_point;
}

}
}