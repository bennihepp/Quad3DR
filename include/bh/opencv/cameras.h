//==================================================
// cameras.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 04.04.17
//==================================================
#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <bh/eigen.h>

#if OPENCV_2_4
namespace cv_cuda = cv::gpu;
#else
namespace cv_cuda = cv::cuda;
#endif

namespace bh {
namespace opencv {

template <typename FloatT>
struct PinholeCamera {
  using FloatType = FloatT;
  BH_USE_FIXED_EIGEN_TYPES(FloatType);

  using IntrinsicMatrix = Matrix4x4;

  PinholeCamera();

  PinholeCamera(const size_t width, const size_t height, const IntrinsicMatrix& intrinsics);

  Vector2 imageToWorld(const Vector2& image_point) const;

  Vector2 worldToImage(const Vector2& world_point) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  size_t width() const { return width_; }

  size_t& width() { return width_; }

  size_t height() const { return height_; }

  size_t& height() { return height_; }

  const IntrinsicMatrix& intrinsics() const { return intrinsics_; }

  IntrinsicMatrix& intrinsics() { return intrinsics_; }

private:
  size_t width_;
  size_t height_;
  IntrinsicMatrix intrinsics_;
};

template <typename FloatT>
struct OpenCVCamera {
  using FloatType = FloatT;
  USE_FIXED_EIGEN_TYPES(FloatType);

  using IntrinsicMatrix = Matrix4x4;

  OpenCVCamera();

  OpenCVCamera(const size_t width, const size_t height, const IntrinsicMatrix& intrinsics);

  Vector2 imageToWorld(const Vector2& image_point) const;

  Vector2 imageToWorldWithoutUndistortion(const Vector2& image_point) const;

  Vector2 worldToImage(const Vector2& world_point) const;

  Vector2 worldToImageWithoutDistortion(const Vector2& world_point) const;

  Vector2 distortionOffsetsForWorldPoint(const Vector2& world_point) const;

  Vector2 distortWorldPoint(const Vector2& world_point) const;

  Vector2 undistortWorldPoint(const Vector2& world_point) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  size_t width() const { return pinhole_camera_.width(); }

  size_t& width() { return pinhole_camera_.width(); }

  size_t height() const { return pinhole_camera_.height(); }

  size_t& height() { return pinhole_camera_.height(); }

  const IntrinsicMatrix& intrinsics() const { return pinhole_camera_.intrinsics(); }

  IntrinsicMatrix& intrinsics() { return pinhole_camera_.intrinsics(); }

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

  const std::array<FloatType, 5>& distCoeffs() const { return dist_coeffs_; };

  std::array<FloatType, 5>& distCoeffs() { return dist_coeffs_; };

private:
  PinholeCamera pinhole_camera_;
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

class Cameras {
public:
  Cameras() = delete;

  static cv::Mat undistortPoints(
          cv::InputArray points, cv::InputArray camera_matrix, cv::InputArray dist_coefficients);
};

template <typename FloatT>
PinholeCamera<FloatT>::PinholeCamera()
    : width_(0), height_(0) {
  intrinsics_.setIdentity();
}

template <typename FloatT>
PinholeCamera<FloatT>::PinholeCamera(const size_t width, const size_t height, const IntrinsicMatrix& intrinsics)
    : width_(0), height_(0), intrinsics_(intrinsics) {}

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
OpenCVCamera<FloatT>::OpenCVCamera()
    : k1_(0), k2_(0), p1_(0), p2_(0), k3_(0) {}

template <typename FloatT>
OpenCVCamera<FloatT>::OpenCVCamera(const size_t width, const size_t height, const IntrinsicMatrix& intrinsics)
    : pinhole_camera_(width, height, intrinsics), k1_(0), k2_(0), p1_(0), p2_(0), k3_(0) {}

template <typename FloatT>
auto OpenCVCamera<FloatT>::imageToWorldWithoutUndistortion(const Vector2& image_point) const -> Vector2 {
  return pinhole_camera_.imageToWorld(image_point);
}

template <typename FloatT>
auto OpenCVCamera<FloatT>::imageToWorld(const Vector2& image_point) const -> Vector2 {
  return undistortWorldPoint(imageToWorldWithoutUndistortion(image_point));
}

template <typename FloatT>
auto OpenCVCamera<FloatT>::worldToImageWithoutDistortion(const Vector2& world_point) const -> Vector2 {
  return pinhole_camera_.worldToImage(world_point);
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