//==================================================
// geometry.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 07.04.17
//==================================================
#pragma once

#include "../eigen.h"
#include "../se3_transform.h"
#include "../opencv/matrix.h"
#include "cameras.h"
#include <opencv2/calib3d.hpp>

namespace bh {
namespace vision {

template<typename FloatT>
using FundamentalMatrix = Eigen::Matrix<FloatT, 3, 3>;

template<typename FloatT>
using EssentialMatrix = Eigen::Matrix<FloatT, 3, 3>;

template<typename FloatT>
using ProjectionMatrix = Eigen::Matrix<FloatT, 4, 4>;

}
}

#define BH_USE_VISION_GEOMETRY_TYPES(FloatT) \
  using FundamentalMatrix = bh::vision::FundamentalMatrix<FloatT>; \
  using EssentialMatrix = bh::vision::EssentialMatrix<FloatT>; \
  using ProjectionMatrix = bh::vision::ProjectionMatrix<FloatT>;

namespace bh {
namespace vision {

template <typename FloatT>
typename EigenTypes<FloatT>::Vector2 projectPoint(
        const typename EigenTypes<FloatT>::Vector3& world_point,
        const SE3Transform<FloatT>& to_camera_transform);

template <typename FloatT, typename CameraT>
typename EigenTypes<FloatT>::Vector2 projectPoint(
        const typename EigenTypes<FloatT>::Vector3& world_point,
        const SE3Transform<FloatT>& to_camera_transform,
        const CameraT& camera);

template <typename FloatT>
ProjectionMatrix<FloatT> getProjectionMatrix(const SE3Transform<FloatT>& transform);

template <typename FloatT>
ProjectionMatrix<FloatT> getProjectionMatrix(
        const SE3Transform<FloatT>& transform, const PinholeCamera<FloatT>& camera);

template <typename FloatT>
typename EigenTypes<FloatT>::Vector3 triangulatePoint(
        const ProjectionMatrix<FloatT>& projection_matrix_left,
        const ProjectionMatrix<FloatT>& projection_matrix_right,
        const typename EigenTypes<FloatT>::Vector2& left_point,
        const typename EigenTypes<FloatT>::Vector2& right_point);

template <typename FloatT>
SE3Transform<FloatT> decomposeEssentialMatrix(const EssentialMatrix<FloatT>& essential_matrix);

template <typename FloatT>
SE3Transform<FloatT> decomposeEssentialMatrix(const EssentialMatrix<FloatT>& essential_matrix);

template <typename FloatT>
std::pair<bool, SE3Transform<FloatT>> decomposeEssentialMatrix(
        const EssentialMatrix<FloatT>& essential_matrix,
        const typename EigenTypes<FloatT>::Vector2& world_point1,
        const typename EigenTypes<FloatT>::Vector2& world_point2);

template <typename FloatT, typename Camera1T, typename Camera2T>
std::pair<bool, SE3Transform<FloatT>> decomposeEssentialMatrix(
        const EssentialMatrix<FloatT>& essential_matrix,
        const Camera1T& camera1,
        const typename EigenTypes<FloatT>::Vector2& image_point1,
        const Camera2T& camera2,
        const typename EigenTypes<FloatT>::Vector2& image_point2);

template <typename FloatT, typename Camera1T, typename Camera2T>
EssentialMatrix<FloatT> essentialMatrixFromFundamentalMatrix(
        const FundamentalMatrix<FloatT>& fundamental_matrix,
        const Camera1T& left_camera, const Camera2T& right_camera);

template <typename FloatT, typename Camera1T, typename Camera2T>
FundamentalMatrix<FloatT> fundamentalMatrixFromEssentialMatrix(
        const EssentialMatrix<FloatT>& essential_matrix,
        const Camera1T& left_camera, const Camera2T& right_camera);

//template <typename FloatT>
//computeEssentialMatrix();

template <typename FloatT>
std::pair<bool, FundamentalMatrix<FloatT>> computeFundamentalMatrix8Point(
        const std::vector<typename EigenTypes<FloatT>::Vector2> &points1,
        const std::vector<typename EigenTypes<FloatT>::Vector2> &points2);

//==================================================
// Implementation
//==================================================

template <typename FloatT>
typename EigenTypes<FloatT>::Vector2 projectPoint(
        const typename EigenTypes<FloatT>::Vector3& world_point,
        const SE3Transform<FloatT>& to_camera_transform) {
  const typename EigenTypes<FloatT>::Vector3 camera_point = to_camera_transform.transform(world_point);
  return camera_point.hnormalized();
}

template <typename FloatT, typename CameraT>
typename EigenTypes<FloatT>::Vector2 projectPoint(
        const typename EigenTypes<FloatT>::Vector3& world_point,
        const SE3Transform<FloatT>& to_camera_transform,
        const CameraT& camera) {
  const typename EigenTypes<FloatT>::Vector3 camera_point = to_camera_transform.transform(world_point);
  const typename EigenTypes<FloatT>::Vector2 image_point = camera.worldToImage(camera_point.hnormalized());
  return image_point;
}

template <typename FloatT>
ProjectionMatrix<FloatT> getProjectionMatrix(const SE3Transform<FloatT>& to_camera_transform) {
  return to_camera_transform.getTransformationMatrix4x4();
}

template <typename FloatT>
ProjectionMatrix<FloatT> getProjectionMatrix(
        const SE3Transform<FloatT>& to_camera_transform, const PinholeCamera<FloatT>& camera) {
  return camera.intrinsics() * to_camera_transform.getTransformationMatrix4x4();
}

template <typename FloatT>
typename EigenTypes<FloatT>::Vector3 triangulatePoint(
        const ProjectionMatrix<FloatT>& projection_matrix_left,
        const ProjectionMatrix<FloatT>& projection_matrix_right,
        const typename EigenTypes<FloatT>::Vector2& left_point,
        const typename EigenTypes<FloatT>::Vector2& right_point) {
  using Vector3 = typename EigenTypes<FloatT>::Vector3;
  using Matrix4x4 = typename EigenTypes<FloatT>::Matrix4x4;
  using ProjectionMatrix = bh::vision::ProjectionMatrix<FloatT>;

  const ProjectionMatrix& p_mat_l = projection_matrix_left;
  const ProjectionMatrix& p_mat_r = projection_matrix_right;
  // Triangulation of stereo points according to ...
  // * R. Hartley and P. Sturm, Triangulation, Computer vision and image understanding 68.2, 1997.
  // * HZ, R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision, Cambridge Univ. Press, 2003.
  const FloatT u1 = left_point(0);
  const FloatT v1 = left_point(1);
  const FloatT u2 = right_point(0);
  const FloatT v2 = right_point(1);
  Matrix4x4 A;
  A(0, 0) = u1 * p_mat_l(2, 0) - p_mat_l(0, 0);
  A(0, 1) = u1 * p_mat_l(2, 1) - p_mat_l(0, 1);
  A(0, 2) = u1 * p_mat_l(2, 2) - p_mat_l(0, 2);
  A(0, 3) = u1 * p_mat_l(2, 3) - p_mat_l(0, 3);
  A(1, 0) = v1 * p_mat_l(2, 0) - p_mat_l(1, 0);
  A(1, 1) = v1 * p_mat_l(2, 1) - p_mat_l(1, 1);
  A(1, 2) = v1 * p_mat_l(2, 2) - p_mat_l(1, 2);
  A(1, 3) = v1 * p_mat_l(2, 3) - p_mat_l(1, 3);
  A(2, 0) = u2 * p_mat_r(2, 0) - p_mat_r(0, 0);
  A(2, 1) = u2 * p_mat_r(2, 1) - p_mat_r(0, 1);
  A(2, 2) = u2 * p_mat_r(2, 2) - p_mat_r(0, 2);
  A(2, 3) = u2 * p_mat_r(2, 3) - p_mat_r(0, 3);
  A(3, 0) = v2 * p_mat_r(2, 0) - p_mat_r(1, 0);
  A(3, 1) = v2 * p_mat_r(2, 1) - p_mat_r(1, 1);
  A(3, 2) = v2 * p_mat_r(2, 2) - p_mat_r(1, 2);
  A(3, 3) = v2 * p_mat_r(2, 3) - p_mat_r(1, 3);
  Eigen::JacobiSVD<Matrix4x4> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
//  const EigenTypes<FloatT>::Matrix4x4 U = svd.matrixU();
  const Matrix4x4 V = svd.matrixV();
//  const EigenTypes<FloatT>::Vector4 S = svd.singularValues();
  const FloatT x = V(0, 3);
  const FloatT y = V(1, 3);
  const FloatT z = V(2, 3);
  const FloatT w = V(3, 3);
  const Vector3 point(x / w, y / w, z / w);
  return point;
}

// Decomposes an essential matrix into its 4 possible solutions.
// Returns the transformations to the right/second camera coordinate system.
template <typename FloatT>
std::array<SE3Transform<FloatT>, 4> decomposeEssentialMatrix(const EssentialMatrix<FloatT>& essential_matrix) {
  using Vector3 = typename EigenTypes<FloatT>::Vector3;
  using Matrix3x3 = typename EigenTypes<FloatT>::Matrix3x3;
  using SE3Transform = bh::SE3Transform<FloatT>;

  cv::Mat cv_essential_matrix = bh::opencv::convertEigenToCv32F(essential_matrix);
  cv::Mat cv_translation;
  cv::Mat cv_rotation1;
  cv::Mat cv_rotation2;
  cv::decomposeEssentialMat(cv_essential_matrix, cv_rotation1, cv_rotation2, cv_translation);
  const Matrix3x3 rotation1 = bh::opencv::convertCvToEigen<Matrix3x3>(cv_rotation1);
  const Matrix3x3 rotation2 = bh::opencv::convertCvToEigen<Matrix3x3>(cv_rotation2);
  const Vector3 translation = bh::opencv::convertCvToEigen<Vector3>(cv_translation);
  std::array<SE3Transform, 4> transforms;
  transforms[0] = SE3Transform(translation, rotation1);
  transforms[1] = SE3Transform(-translation, rotation1);
  transforms[2] = SE3Transform(translation, rotation2);
  transforms[3] = SE3Transform(-translation, rotation2);
  return transforms;
};

// Decomposes an essential matrix into its only feasible solution based on an observed correspondence pair.
// Returns the transformation to the right/second camera coordinate system.
template <typename FloatT, typename Camera1T, typename Camera2T>
std::pair<bool, SE3Transform<FloatT>> decomposeEssentialMatrix(
        const EssentialMatrix<FloatT>& essential_matrix,
        const Camera1T& camera1,
        const typename EigenTypes<FloatT>::Vector2& image_point1,
        const Camera2T& camera2,
        const typename EigenTypes<FloatT>::Vector2& image_point2) {
  using Vector2 = typename EigenTypes<FloatT>::Vector2;
  const Vector2 world_point1 = camera1.imageToWorld(image_point1);
  const Vector2 world_point2 = camera1.imageToWorld(image_point2);
  return decomposeEssentialMatrix<FloatT>(essential_matrix, world_point1, world_point2);
};

enum DecomposeEssentialMatrixConfiguration {
  DECOMPOSE_ESSENTIAL_MATRIX_INVALID = 0,
  POSITIVE_TRANSLATION_ROTATION_1 = 1,
  NEGATIVE_TRANSLATION_ROTATION_1 = 2,
  POSITIVE_TRANSLATION_ROTATION_2 = 3,
  NEGATIVE_TRANSLATION_ROTATION_2 = 4,
};

template <typename FloatT>
std::pair<bool, SE3Transform<FloatT>> decomposeEssentialMatrix(
        const EssentialMatrix<FloatT>& essential_matrix,
        const std::vector<typename EigenTypes<FloatT>::Vector2>& world_points1,
        const std::vector<typename EigenTypes<FloatT>::Vector2>& world_points2) {
  using Quaternion = typename EigenTypes<FloatT>::Quaternion;
  using Vector2 = typename EigenTypes<FloatT>::Vector2;
  using Vector3 = typename EigenTypes<FloatT>::Vector3;
  using Matrix3x3 = typename EigenTypes<FloatT>::Matrix3x3;
  using ProjectionMatrix = bh::vision::ProjectionMatrix<FloatT>;
  using SE3Transform = bh::SE3Transform<FloatT>;

  BH_ASSERT(world_points1.size() == world_points2.size());
  cv::Mat cv_essential_matrix = bh::opencv::convertEigenToCv32F(essential_matrix);
  cv::Mat cv_translation;
  cv::Mat cv_rotation1;
  cv::Mat cv_rotation2;
  cv::decomposeEssentialMat(cv_essential_matrix, cv_rotation1, cv_rotation2, cv_translation);
  const Matrix3x3 rotation1 = bh::opencv::convertCvToEigen<Matrix3x3>(cv_rotation1);
  const Matrix3x3 rotation2 = bh::opencv::convertCvToEigen<Matrix3x3>(cv_rotation2);
  const Vector3 translation = bh::opencv::convertCvToEigen<Vector3>(cv_translation);
  const SE3Transform to_left_transform(Vector3::Zero(), Quaternion::Identity());
  const ProjectionMatrix left_projection_matrix = getProjectionMatrix(to_left_transform);
  const SE3Transform to_right_transform1(translation, rotation1);
  const SE3Transform to_right_transform2(-translation, rotation1);
  const SE3Transform to_right_transform3(translation, rotation2);
  const SE3Transform to_right_transform4(-translation, rotation2);
  const ProjectionMatrix right_projection_matrix1 = getProjectionMatrix(to_right_transform1);
  const ProjectionMatrix right_projection_matrix2 = getProjectionMatrix(to_right_transform2);
  const ProjectionMatrix right_projection_matrix3 = getProjectionMatrix(to_right_transform3);
  const ProjectionMatrix right_projection_matrix4 = getProjectionMatrix(to_right_transform4);
  std::array<size_t, 5> config_votes = { 0, 0, 0, 0, 0 };
  for (size_t i = 0; i < world_points1.size(); ++i) {
    const Vector2& world_point1 = world_points1[i];
    const Vector2& world_point2 = world_points2[i];
    const Vector3 point1 = triangulatePoint(left_projection_matrix, right_projection_matrix1, world_point1,
                                            world_point2);
    const Vector3 point2 = triangulatePoint(left_projection_matrix, right_projection_matrix2, world_point1,
                                            world_point2);
    const Vector3 point3 = triangulatePoint(left_projection_matrix, right_projection_matrix3, world_point1,
                                            world_point2);
    const Vector3 point4 = triangulatePoint(left_projection_matrix, right_projection_matrix4, world_point1,
                                            world_point2);
    const Vector3 left_proj_point1 = (left_projection_matrix * point1.homogeneous()).hnormalized();
    const Vector3 left_proj_point2 = (left_projection_matrix * point2.homogeneous()).hnormalized();
    const Vector3 left_proj_point3 = (left_projection_matrix * point3.homogeneous()).hnormalized();
    const Vector3 left_proj_point4 = (left_projection_matrix * point4.homogeneous()).hnormalized();
    const Vector3 right_proj_point1 = (right_projection_matrix1 * point1.homogeneous()).hnormalized();
    const Vector3 right_proj_point2 = (right_projection_matrix2 * point2.homogeneous()).hnormalized();
    const Vector3 right_proj_point3 = (right_projection_matrix3 * point3.homogeneous()).hnormalized();
    const Vector3 right_proj_point4 = (right_projection_matrix4 * point4.homogeneous()).hnormalized();
    const bool left_visible1 = left_proj_point1(2) > 0;
    const bool left_visible2 = left_proj_point2(2) > 0;
    const bool left_visible3 = left_proj_point3(2) > 0;
    const bool left_visible4 = left_proj_point4(2) > 0;
    const bool right_visible1 = right_proj_point1(2) > 0;
    const bool right_visible2 = right_proj_point2(2) > 0;
    const bool right_visible3 = right_proj_point3(2) > 0;
    const bool right_visible4 = right_proj_point4(2) > 0;
    if (left_visible1 && right_visible1) {
      ++config_votes[POSITIVE_TRANSLATION_ROTATION_1];
    }
    else if (left_visible2 && right_visible2) {
      ++config_votes[NEGATIVE_TRANSLATION_ROTATION_1];
    }
    else if (left_visible3 && right_visible3) {
      ++config_votes[POSITIVE_TRANSLATION_ROTATION_2];
    }
    else if (left_visible4 && right_visible4) {
      ++config_votes[NEGATIVE_TRANSLATION_ROTATION_2];
    }
    else {
      ++config_votes[DECOMPOSE_ESSENTIAL_MATRIX_INVALID];
    }
  }
  const auto max_it = std::max_element(config_votes.begin(), config_votes.end());
  const size_t max_vote = *max_it;
  size_t config = max_it - config_votes.begin();
  for (auto it = config_votes.begin(); it != config_votes.end(); ++it) {
    if (it != max_it && *it == max_vote) {
      config = DECOMPOSE_ESSENTIAL_MATRIX_INVALID;
    }
  }
  switch (config) {
    case DECOMPOSE_ESSENTIAL_MATRIX_INVALID:
      return std::make_pair(false, SE3Transform());
    case POSITIVE_TRANSLATION_ROTATION_1:
      return std::make_pair(true, to_right_transform1);
    case NEGATIVE_TRANSLATION_ROTATION_1:
      return std::make_pair(true, to_right_transform2);
    case POSITIVE_TRANSLATION_ROTATION_2:
      return std::make_pair(true, to_right_transform3);
    case NEGATIVE_TRANSLATION_ROTATION_2:
      return std::make_pair(true, to_right_transform4);
    default:
      throw bh::Error("This should never happen");
  };
};

// Decomposes an essential matrix into its only feasible solution based on an observed correspondence pair.
// Returns the transformation to the right/second camera coordinate system.
template <typename FloatT>
std::pair<bool, SE3Transform<FloatT>> decomposeEssentialMatrix(
        const EssentialMatrix<FloatT>& essential_matrix,
        const typename EigenTypes<FloatT>::Vector2& world_point1,
        const typename EigenTypes<FloatT>::Vector2& world_point2) {
  using Quaternion = typename EigenTypes<FloatT>::Quaternion;
  using Vector3 = typename EigenTypes<FloatT>::Vector3;
  using Matrix3x3 = typename EigenTypes<FloatT>::Matrix3x3;
  using ProjectionMatrix = bh::vision::ProjectionMatrix<FloatT>;
  using SE3Transform = bh::SE3Transform<FloatT>;

  cv::Mat cv_essential_matrix = bh::opencv::convertEigenToCv32F(essential_matrix);
  cv::Mat cv_translation;
  cv::Mat cv_rotation1;
  cv::Mat cv_rotation2;
  cv::decomposeEssentialMat(cv_essential_matrix, cv_rotation1, cv_rotation2, cv_translation);
  const Matrix3x3 rotation1 = bh::opencv::convertCvToEigen<Matrix3x3>(cv_rotation1);
  const Matrix3x3 rotation2 = bh::opencv::convertCvToEigen<Matrix3x3>(cv_rotation2);
  const Vector3 translation = bh::opencv::convertCvToEigen<Vector3>(cv_translation);
  const SE3Transform to_left_transform(Vector3::Zero(), Quaternion::Identity());
  const ProjectionMatrix left_projection_matrix = getProjectionMatrix(to_left_transform);
  const SE3Transform to_right_transform1(translation, rotation1);
  const SE3Transform to_right_transform2(-translation, rotation1);
  const SE3Transform to_right_transform3(translation, rotation2);
  const SE3Transform to_right_transform4(-translation, rotation2);
  const ProjectionMatrix right_projection_matrix1 = getProjectionMatrix(to_right_transform1);
  const ProjectionMatrix right_projection_matrix2 = getProjectionMatrix(to_right_transform2);
  const ProjectionMatrix right_projection_matrix3 = getProjectionMatrix(to_right_transform3);
  const ProjectionMatrix right_projection_matrix4 = getProjectionMatrix(to_right_transform4);
//  BH_PRINT_VALUE(right_projection_matrix1);
//  BH_PRINT_VALUE(right_projection_matrix2);
//  BH_PRINT_VALUE(right_projection_matrix3);
//  BH_PRINT_VALUE(right_projection_matrix4);
  const Vector3 point1 = triangulatePoint(left_projection_matrix, right_projection_matrix1, world_point1, world_point2);
  const Vector3 point2 = triangulatePoint(left_projection_matrix, right_projection_matrix2, world_point1, world_point2);
  const Vector3 point3 = triangulatePoint(left_projection_matrix, right_projection_matrix3, world_point1, world_point2);
  const Vector3 point4 = triangulatePoint(left_projection_matrix, right_projection_matrix4, world_point1, world_point2);
  const Vector3 left_proj_point1 = (left_projection_matrix * point1.homogeneous()).hnormalized();
  const Vector3 left_proj_point2 = (left_projection_matrix * point2.homogeneous()).hnormalized();
  const Vector3 left_proj_point3 = (left_projection_matrix * point3.homogeneous()).hnormalized();
  const Vector3 left_proj_point4 = (left_projection_matrix * point4.homogeneous()).hnormalized();
  const Vector3 right_proj_point1 = (right_projection_matrix1 * point1.homogeneous()).hnormalized();
  const Vector3 right_proj_point2 = (right_projection_matrix2 * point2.homogeneous()).hnormalized();
  const Vector3 right_proj_point3 = (right_projection_matrix3 * point3.homogeneous()).hnormalized();
  const Vector3 right_proj_point4 = (right_projection_matrix4 * point4.homogeneous()).hnormalized();
  const bool left_visible1 = left_proj_point1(2) > 0;
  const bool left_visible2 = left_proj_point2(2) > 0;
  const bool left_visible3 = left_proj_point3(2) > 0;
  const bool left_visible4 = left_proj_point4(2) > 0;
  const bool right_visible1 = right_proj_point1(2) > 0;
  const bool right_visible2 = right_proj_point2(2) > 0;
  const bool right_visible3 = right_proj_point3(2) > 0;
  const bool right_visible4 = right_proj_point4(2) > 0;
  if (left_visible1 && right_visible1) {
    return std::make_pair(true, to_right_transform1);
  }
  else if (left_visible2 && right_visible2) {
    return std::make_pair(true, to_right_transform2);
  }
  else if (left_visible3 && right_visible3) {
    return std::make_pair(true, to_right_transform3);
  }
  else if (left_visible4 && right_visible4) {
    return std::make_pair(true, to_right_transform4);
  }
  else {
    return std::make_pair(false, SE3Transform());
  }
};

template <typename FloatT, typename Camera1T, typename Camera2T>
EssentialMatrix<FloatT> essentialMatrixFromFundamentalMatrix(
        const FundamentalMatrix<FloatT>& fundamental_matrix,
        const Camera1T& left_camera, const Camera2T& right_camera) {
  const typename Camera1T::IntrinsicsMatrix3x3 left_intrinsics3x3 = left_camera.intrinsics3x3();
  const typename Camera2T::IntrinsicsMatrix3x3 right_intrinsics3x3 = right_camera.intrinsics3x3();
  const EssentialMatrix<FloatT> essential_matrix
          = right_intrinsics3x3.transpose() * fundamental_matrix * left_intrinsics3x3;
  return essential_matrix;
};

template <typename FloatT, typename Camera1T, typename Camera2T>
FundamentalMatrix<FloatT> fundamentalMatrixFromEssentialMatrix(
        const EssentialMatrix<FloatT>& essential_matrix,
        const Camera1T& left_camera, const Camera2T& right_camera) {
  const typename Camera1T::IntrinsicsMatrix3x3 left_intrinsics3x3_inverse = left_camera.intrinsics3x3().inverse();
  const typename Camera2T::IntrinsicsMatrix3x3 right_intrinsics3x3_inverse = right_camera.intrinsics3x3().transpose().inverse();
  FundamentalMatrix<FloatT> fundamental_matrix
          = right_intrinsics3x3_inverse * essential_matrix * left_intrinsics3x3_inverse;
  return fundamental_matrix;
};

template <typename FloatT>
FundamentalMatrix<FloatT> computeFundamentalMatrix(const SE3Transform<FloatT>& to_right_camera_transform) {
  const PinholeCamera<FloatT> camera;
  return computeFundamentalMatrix(camera, camera, to_right_camera_transform);
}

template <typename FloatT>
typename EigenTypes<FloatT>::Vector3 computeEpipolarLineInImage1(
        const FundamentalMatrix<FloatT>& fundamental_matrix,
        const typename EigenTypes<FloatT>::Vector2& point2) {
  using Vector3 = typename EigenTypes<FloatT>::Vector3;
  const Vector3 line = fundamental_matrix.transpose() * point2.homogeneous();
  return line;
}

template <typename FloatT>
typename EigenTypes<FloatT>::Vector3 computeEpipolarLineInImage2(
        const FundamentalMatrix<FloatT>& fundamental_matrix,
        const typename EigenTypes<FloatT>::Vector2& point1) {
  using Vector3 = typename EigenTypes<FloatT>::Vector3;
  const Vector3 line = fundamental_matrix * point1.homogeneous();
  return line;
}

template <typename FloatT>
FundamentalMatrix<FloatT> computeFundamentalMatrix(
        const PinholeCamera<FloatT>& left_camera,
        const PinholeCamera<FloatT>& right_camera,
        const SE3Transform<FloatT>& left_to_right_camera_transform) {
  using IntrinsicsMatrix3x3 = typename PinholeCamera<FloatT>::IntrinsicsMatrix3x3;
  const IntrinsicsMatrix3x3 left_inverse_intrinsics = left_camera.intrinsics3x3().inverse();
  const IntrinsicsMatrix3x3 right_inverse_intrinsics = right_camera.intrinsics3x3().inverse();
  const typename SE3Transform<FloatT>::Vector3& translation = left_to_right_camera_transform.translation();
  const typename SE3Transform<FloatT>::Matrix3x3& rotation = left_to_right_camera_transform.rotation();
//  const typename EigenTypes<FloatT>::Matrix3x3 skew_sym_transform
//          = bh::skewSymmetricMatrix(rotation.transpose() * translation);
//  const FundamentalMatrix<FloatT> fundamental_matrix
//          = right_inverse_intrinsics.transpose() * rotation * skew_sym_transform * left_inverse_intrinsics;
  const typename EigenTypes<FloatT>::Matrix3x3 skew_sym_transform = bh::skewSymmetricMatrix(translation);
  const FundamentalMatrix<FloatT> fundamental_matrix
          = right_inverse_intrinsics.transpose() * skew_sym_transform * rotation * left_inverse_intrinsics;
  return fundamental_matrix;
}

template <typename FloatT>
std::pair<bool, FundamentalMatrix<FloatT>> computeFundamentalMatrix(
        const std::vector<typename EigenTypes<FloatT>::Vector2>& points1,
        const std::vector<typename EigenTypes<FloatT>::Vector2>& points2) {
  BH_ASSERT(points1.size() == points2.size());
  using Vector2 = typename EigenTypes<FloatT>::Vector2;
  using FundamentalMatrix = bh::vision::FundamentalMatrix<FloatT>;

  std::vector<cv::Point2f> cv_points1;
  std::vector<cv::Point2f> cv_points2;
  for (size_t i = 0; i < points1.size(); ++i) {
    const Vector2& point1 = points1[i];
    const Vector2& point2 = points2[i];
    cv_points1.push_back(cv::Point2f(point1(0), point1(1)));
    cv_points2.push_back(cv::Point2f(point2(0), point2(1)));
  }
  const double cv_param1 = 3.0;
  const double cv_param2 = 0.99;
  cv::Mat cv_fundamental_matrix = cv::findFundamentalMat(
          cv_points1,
          cv_points2,
          CV_FM_LMEDS,
          cv_param1,
          cv_param2
  );
  if (cv_fundamental_matrix.rows == 0) {
    return std::make_pair(false, FundamentalMatrix());
  }
  BH_ASSERT(cv_fundamental_matrix.type() == CV_64F);
  FundamentalMatrix fundamental_matrix = bh::opencv::convertCvToEigen<FundamentalMatrix>(cv_fundamental_matrix);
  return std::make_pair(true, fundamental_matrix);
};

template <typename FloatT>
std::pair<bool, FundamentalMatrix<FloatT>> computeFundamentalMatrix8Point(
        const std::vector<typename EigenTypes<FloatT>::Vector2>& points1,
        const std::vector<typename EigenTypes<FloatT>::Vector2>& points2) {
  BH_ASSERT(points1.size() == points2.size());
  BH_ASSERT(points1.size() == 8);
  using Vector2 = typename EigenTypes<FloatT>::Vector2;
  using FundamentalMatrix = bh::vision::FundamentalMatrix<FloatT>;

  std::vector<cv::Point2f> cv_points1;
  std::vector<cv::Point2f> cv_points2;
  for (size_t i = 0; i < points1.size(); ++i) {
    const Vector2& point1 = points1[i];
    const Vector2& point2 = points2[i];
    cv_points1.push_back(cv::Point2f(point1(0), point1(1)));
    cv_points2.push_back(cv::Point2f(point2(0), point2(1)));
  }
  const double cv_param1 = 3.0;
  const double cv_param2 = 0.99;
  cv::Mat cv_fundamental_matrix = cv::findFundamentalMat(
          cv_points1,
          cv_points2,
          CV_FM_8POINT,
          cv_param1,
          cv_param2
  );
  if (cv_fundamental_matrix.rows == 0) {
    return std::make_pair(false, FundamentalMatrix());
  }
  BH_ASSERT(cv_fundamental_matrix.type() == CV_64F);
  FundamentalMatrix fundamental_matrix = bh::opencv::convertCvToEigen<FundamentalMatrix>(cv_fundamental_matrix);
  return std::make_pair(true, fundamental_matrix);
};

template <typename FloatT>
FloatT computeSampsonDistance(
        const typename EigenTypes<FloatT>::Vector2& point1,
        const typename EigenTypes<FloatT>::Vector2& point2,
        const FundamentalMatrix<FloatT>& fundamental_matrix) {
  using Vector3 = typename EigenTypes<FloatT>::Vector3;
  const FloatT numerator_sqrt = point2.homogeneous().dot(fundamental_matrix * point1.homogeneous());
  const FloatT numerator = numerator_sqrt * numerator_sqrt;
  const Vector3 F_x1 = fundamental_matrix * point1.homogeneous();
  const Vector3 Ft_x2 = fundamental_matrix.transpose() * point2.homogeneous();
  const FloatT denominator = F_x1(0) * F_x1(0) + F_x1(1) * F_x1(1) +
                             Ft_x2(0) * Ft_x2(0) + Ft_x2(1) * Ft_x2(1);
  const FloatT sampson_distance = numerator / denominator;
  return sampson_distance;
}

template <typename FloatT>
FloatT computeSampsonDistanceOpenCV(
        const typename EigenTypes<FloatT>::Vector2& point1,
        const typename EigenTypes<FloatT>::Vector2& point2,
        const FundamentalMatrix<FloatT>& fundamental_matrix) {
  const cv::Mat cv_point1 = bh::opencv::convertEigenToCv64F(point1);
  const cv::Mat cv_point2 = bh::opencv::convertEigenToCv64F(point2);
  const cv::Mat cv_fundamental_matrix = bh::opencv::convertEigenToCv64F(fundamental_matrix);
  const double cv_sampson_distance = cv::sampsonDistance(cv_point1, cv_point2, cv_fundamental_matrix);
  const FloatT sampson_distance = FloatT(cv_sampson_distance);
  return sampson_distance;
}

}
}
