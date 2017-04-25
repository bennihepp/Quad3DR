//==================================================
// test_vision.cpp
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 07.04.17
//==================================================

#include <random>
#include "gtest/gtest.h"
#include <bh/algorithm.h>
#include <bh/math/utilities.h>
#include <bh/se3_transform.h>
#pragma GCC optimize("O0")
#include <bh/vision/geometry.h>

#pragma GCC optimize("O0")

namespace {
using FloatType = float;
using size_t = std::size_t;

BH_USE_FIXED_EIGEN_TYPES(FloatType);
BH_USE_VISION_GEOMETRY_TYPES(FloatType);
using SE3Transform = bh::SE3Transform<FloatType>;
using PinholeCamera = bh::vision::PinholeCamera<FloatType>;

const size_t kNumPoints3D = 1000;
const FloatType kErrorTolerance = 1e-3;
const FloatType kSquaredErrorTolerance = 1e-3;

#if GTEST_HAS_PARAM_TEST

class VisionNormalizedGeometryTest : public ::testing::Test {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//  VisionNormalizedGeometryTest() {}

  virtual ~VisionNormalizedGeometryTest() {}

  virtual void SetUp() {
    uniform_dist_xy = std::uniform_real_distribution<FloatType>(-10, 10),
    uniform_dist_z = std::uniform_real_distribution<FloatType>(1, 10);
    std::random_device rd;
    rnd.seed(rd());

    to_left_transform = SE3Transform(Vector3::Zero(), Quaternion::Identity());
//    from_right_transform = SE3Transform(Vector3(5, 0, 0), Quaternion::Identity());
    from_right_transform = SE3Transform(Vector3(0.1, 0, 0), AngleAxis(-0.2 * M_PI, Vector3::UnitY()).toRotationMatrix());
//    to_right_transform = SE3Transform(Vector3(-5, 0, 0), AngleAxis(+0.2 * M_PI, Vector3::UnitY()).toRotationMatrix());
//    from_right_transform = SE3Transform(Vector3(5, 0, 0), AngleAxis(0.1 * M_PI, Vector3::UnitY()).toRotationMatrix());
//    from_right_transform = SE3Transform(Vector3(5, 0, 0), AngleAxis(-0.25 * M_PI, Vector3::UnitY()).toRotationMatrix());
//    from_right_transform = GetParam();

    to_left_transform = from_left_transform.inverse();
    to_right_transform = from_right_transform.inverse();
//    from_left_transform = to_left_transform.inverse();
//    from_right_transform = to_right_transform.inverse();
    left_projection_matrix = bh::vision::getProjectionMatrix(to_left_transform);
    right_projection_matrix = bh::vision::getProjectionMatrix(to_right_transform);

    {
      const Vector3 point_3d(0, 0, 10);
      const Vector3 left_point_3d = to_left_transform.transform(point_3d);
      const Vector3 right_point_3d = to_right_transform.transform(point_3d);
      if (left_point_3d(2) > 0 && right_point_3d(2) > 0);
      const Vector2 left_image_point = pinhole_camera.worldToImage(left_point_3d.hnormalized());
      const Vector2 right_image_point = pinhole_camera.worldToImage(right_point_3d.hnormalized());
      points_3d.push_back(point_3d);
      left_points_3d.push_back(left_point_3d);
      right_points_3d.push_back(right_point_3d);
      left_image_points.push_back(left_image_point);
      right_image_points.push_back(right_image_point);
//      BH_PRINT_VALUE(point_3d.transpose());
//      BH_PRINT_VALUE(left_point_3d.transpose());
//      BH_PRINT_VALUE(right_point_3d.transpose());
    }

//    createRegularPoints(5, 5, 20, 5, 5);

    createRandomPoints(kNumPoints3D - points_3d.size());
  }

  void createRegularPoints(const FloatType x_span, const FloatType y_span, const FloatType z,
                           const size_t num_points_x, const size_t num_points_y) {
    for (int i = 0; i < (int)num_points_x; ++i) {
      const FloatType x = x_span * (i - (int)num_points_x / 2) / (FloatType)num_points_x;
      for (int j = 0; j < (int)num_points_y; ++j) {
        const FloatType y = y_span * (j - (int)num_points_y / 2) / (FloatType)num_points_y;
        const Vector3 point_3d(x, y, z);
        const Vector3 left_point_3d = to_left_transform.transform(point_3d);
        const Vector3 right_point_3d = to_right_transform.transform(point_3d);
        BH_ASSERT(left_point_3d(2) > kErrorTolerance);
        BH_ASSERT(right_point_3d(2) > kErrorTolerance);
        const Vector2 left_image_point = pinhole_camera.worldToImage(left_point_3d.hnormalized());
        const Vector2 right_image_point = pinhole_camera.worldToImage(right_point_3d.hnormalized());
        points_3d.push_back(point_3d);
        left_points_3d.push_back(left_point_3d);
        right_points_3d.push_back(right_point_3d);
        left_image_points.push_back(left_image_point);
        right_image_points.push_back(right_image_point);

        // Sanity check
        const Vector3 projected_point_3d = left_point_3d(2) * pinhole_camera.imageToWorld(left_image_point).homogeneous();
        const Vector3 projected_right_point_3d = to_right_transform.transform(projected_point_3d);
        const Vector2 projected_right_image_point = pinhole_camera.worldToImage(projected_right_point_3d.hnormalized());
        BH_ASSERT(bh::isApproxEqual<FloatType>((projected_right_image_point - right_image_point).squaredNorm(), 0, kErrorTolerance));
      }
    }
  }

  void createRandomPoints(const size_t num_points_to_generate) {
    size_t i = 0;
    while (i < num_points_to_generate) {
      const FloatType x = getRandomRealXY();
      const FloatType y = getRandomRealXY();
      const FloatType z = getRandomRealZ();
      const Vector3 point_3d(x, y, z);
      const Vector3 left_point_3d = to_left_transform.transform(point_3d);
      const Vector3 right_point_3d = to_right_transform.transform(point_3d);
      if (left_point_3d(2) <= 0 || right_point_3d(2) <= 0) {
        continue;
      }
      const Vector2 left_image_point = pinhole_camera.worldToImage(left_point_3d.hnormalized());
      const Vector2 right_image_point = pinhole_camera.worldToImage(right_point_3d.hnormalized());
      points_3d.push_back(point_3d);
      left_points_3d.push_back(left_point_3d);
      right_points_3d.push_back(right_point_3d);
      left_image_points.push_back(left_image_point);
      right_image_points.push_back(right_image_point);
      ++i;
    }
  }

  virtual void TearDown() {}

protected:
  FloatType getRandomRealXY() {
    return uniform_dist_xy(rnd);
  }

  FloatType getRandomRealZ() {
    return uniform_dist_z(rnd);
  }

  std::mt19937_64 rnd;
  std::uniform_real_distribution<FloatType> uniform_dist_xy;
  std::uniform_real_distribution<FloatType> uniform_dist_z;
  PinholeCamera pinhole_camera;
  SE3Transform from_left_transform;
  SE3Transform from_right_transform;
  SE3Transform to_left_transform;
  SE3Transform to_right_transform;
  ProjectionMatrix left_projection_matrix;
  ProjectionMatrix right_projection_matrix;
  std::vector<Vector3> points_3d;
  std::vector<Vector3> left_points_3d;
  std::vector<Vector3> right_points_3d;
  std::vector<Vector2> left_image_points;
  std::vector<Vector2> right_image_points;
};
}

TEST_F(VisionNormalizedGeometryTest, CheckProjection) {
  for (size_t i = 0; i < points_3d.size(); ++i) {
    const Vector3& point_3d = points_3d[i];
    const Vector2& left_image_point = left_image_points[i];
    const Vector2& right_image_point = right_image_points[i];
    const Vector2 left_projected_point = bh::vision::projectPoint(point_3d, to_left_transform, pinhole_camera);
    EXPECT_TRUE(left_image_point.isApprox(left_projected_point, kErrorTolerance));
    const Vector2 right_projected_point = bh::vision::projectPoint(point_3d, to_right_transform, pinhole_camera);
    EXPECT_TRUE(right_image_point.isApprox(right_projected_point, kErrorTolerance));
  }
}

TEST_F(VisionNormalizedGeometryTest, CheckTriangulation) {
  for (size_t i = 0; i < points_3d.size(); ++i) {
    const Vector2& left_image_point = left_image_points[i];
    const Vector2& right_image_point = right_image_points[i];
    const Vector3 triangulated_point_3d = bh::vision::triangulatePoint(
            left_projection_matrix,
            right_projection_matrix,
            left_image_point,
            right_image_point);
    const Vector3& point_3d = points_3d[i];
    const FloatType squared_err = (point_3d - triangulated_point_3d).squaredNorm();
    EXPECT_LE(squared_err, kSquaredErrorTolerance);
    BH_ASSERT(squared_err <= kSquaredErrorTolerance);
  }
}

TEST_F(VisionNormalizedGeometryTest, CheckComputeFundamentalMatrixAlgebraic) {
  const FundamentalMatrix fundamental_matrix = bh::vision::computeFundamentalMatrix(to_right_transform);
  BH_PRINT_VALUE(to_right_transform.translation().transpose());
  BH_PRINT_VALUE(to_right_transform.rotation());
  for (size_t i = 0; i < left_image_points.size(); ++i) {
    const Vector2& left_image_point = left_image_points[i];
    const Vector2& right_image_point = right_image_points[i];
    const FloatType sampson_error = bh::vision::computeSampsonDistance(left_image_point, right_image_point, fundamental_matrix);
//    BH_PRINT_VALUE(sampson_error);
    EXPECT_LE(sampson_error, kErrorTolerance);

//    const Vector3 algebraic_error_right = fundamental_matrix * right_image_point.homogeneous();
//    const FloatType algebraic_error = left_image_point.homogeneous().dot(algebraic_error_right);
//    const FloatType sampson_error_opencv = bh::vision::computeSampsonDistanceOpenCV(left_image_point, right_image_point, fundamental_matrix);
//    BH_PRINT_VALUE(algebraic_error);
//    BH_PRINT_VALUE(sampson_error_opencv);
//    EXPECT_LE(algebraic_error, kErrorTolerance);
  }
  for (size_t i = 0; i < left_image_points.size(); ++i) {
    const Vector2 &left_image_point = left_image_points[i];
    const Vector2 &right_image_point = right_image_points[i];
    const Vector3 epipolar_line = fundamental_matrix * left_image_point.homogeneous();
    const FloatType epipolar_constraint_error = epipolar_line.dot(right_image_point.homogeneous());
    EXPECT_LT(std::abs(epipolar_constraint_error), kErrorTolerance);
  }
//  const Vector3 epipolar_line2 = Vector3(1, 5, 1).transpose() * fundamental_matrix;
//  BH_PRINT_VALUE(epipolar_line2.transpose());
//  const Vector3 epipolar_line3 = Vector3(3, 5, 1).transpose() * fundamental_matrix;
//  BH_PRINT_VALUE(epipolar_line3.transpose());
//  const Vector3 epipolar_line4 = fundamental_matrix * Vector3(100, 2, 1);
//  BH_PRINT_VALUE(epipolar_line4.transpose());
//  Eigen::JacobiSVD<FundamentalMatrix> svd(fundamental_matrix);
//  BH_PRINT_VALUE(svd.rank());
//  BH_PRINT_VALUE(fundamental_matrix);
}

TEST_F(VisionNormalizedGeometryTest, CheckComputeFundamentalMatrix) {
  std::vector<Vector2> tmp_left_image_points;
  std::vector<Vector2> tmp_right_image_points;
  std::copy(left_image_points.begin(), left_image_points.begin() + 8, std::back_inserter(tmp_left_image_points));
  std::copy(right_image_points.begin(), right_image_points.begin() + 8, std::back_inserter(tmp_right_image_points));
  bool valid_fundamental_matrix;
  FundamentalMatrix fundamental_matrix;
  std::tie(valid_fundamental_matrix, fundamental_matrix)
          = bh::vision::computeFundamentalMatrix8Point<FloatType>(tmp_left_image_points, tmp_right_image_points);
  ASSERT_TRUE(valid_fundamental_matrix);
  const FundamentalMatrix fundamental_matrix_algebraic = bh::vision::computeFundamentalMatrix(to_right_transform);
  for (size_t i = 0; i < tmp_left_image_points.size(); ++i) {
    const Vector2& left_image_point = tmp_left_image_points[i];
    const Vector2& right_image_point = tmp_right_image_points[i];
    const FloatType sampson_error = bh::vision::computeSampsonDistance(left_image_point, right_image_point, fundamental_matrix);
//    const FloatType sampson_error2 = bh::vision::computeSampsonDistance(left_image_point, right_image_point, fundamental_matrix_algebraic);
//    const Vector3 algebraic_error_right = fundamental_matrix * right_point.homogeneous();
//    BH_PRINT_VALUE(algebraic_error_right);
//    const FloatType algebraic_error = left_point.homogeneous().dot(algebraic_error_right);
//    BH_PRINT_VALUE(algebraic_error);
//    BH_PRINT_VALUE(sampson_error);
//    BH_PRINT_VALUE(sampson_error2);
//    EXPECT_LE(algebraic_error, kErrorTolerance);
    EXPECT_LE(sampson_error, kErrorTolerance);
  }
//  for (size_t i = 0; i < left_image_points.size(); ++i) {
//    const Vector2 &left_image_point = left_image_points[i];
//    const Vector2 &right_image_point = right_image_points[i];
//    const Vector3 epipolar_line = fundamental_matrix * left_image_point.homogeneous();
//    const FloatType epipolar_constraint_error = epipolar_line.dot(right_image_point.homogeneous());
//    EXPECT_LT(std::abs(epipolar_constraint_error), kErrorTolerance);
//  }
  const Vector3 epipolar_line = Vector3(1, 1, 1).transpose() * fundamental_matrix;
  BH_PRINT_VALUE(epipolar_line.transpose().normalized());
  const Vector3 epipolar_line2 = Vector3(1, 1, 1).transpose() * fundamental_matrix_algebraic;
  BH_PRINT_VALUE(epipolar_line2.transpose().normalized());
  Eigen::JacobiSVD<FundamentalMatrix> svd(fundamental_matrix);
  BH_PRINT_VALUE(svd.rank());
  BH_PRINT_VALUE(fundamental_matrix);
  BH_PRINT_VALUE(fundamental_matrix_algebraic);
}

TEST_F(VisionNormalizedGeometryTest, CheckDecomposeEssentialMatrix) {
  std::vector<Vector2> tmp_left_image_points;
  std::vector<Vector2> tmp_right_image_points;
  std::copy(left_image_points.begin(), left_image_points.begin() + 8, std::back_inserter(tmp_left_image_points));
  std::copy(right_image_points.begin(), right_image_points.begin() + 8, std::back_inserter(tmp_right_image_points));
  bool valid_fundamental_matrix;
  FundamentalMatrix fundamental_matrix;
  std::tie(valid_fundamental_matrix, fundamental_matrix)
          = bh::vision::computeFundamentalMatrix8Point<FloatType>(tmp_left_image_points, tmp_right_image_points);
  ASSERT_TRUE(valid_fundamental_matrix);
//  BH_PRINT_VALUE(fundamental_matrix);
  for (size_t i = 0; i < tmp_left_image_points.size(); ++i) {
    const Vector2& left_point = tmp_left_image_points[i];
    const Vector2& right_point = tmp_right_image_points[i];
    bool decompose_essential_matrix_success;
    SE3Transform transform;
    std::tie(decompose_essential_matrix_success, transform) = bh::vision::decomposeEssentialMatrix(
            fundamental_matrix, left_point, right_point);
    ASSERT_TRUE(decompose_essential_matrix_success);
    const FloatType translation_squared_err
            = (to_right_transform.translation().normalized() - transform.translation().normalized()).squaredNorm();
    const FloatType angular_error = transform.quaternion().angularDistance(to_right_transform.quaternion());
    EXPECT_LE(translation_squared_err, kErrorTolerance);
    EXPECT_LE(angular_error, kErrorTolerance);
  }
}


//const SE3Transform from_right_transform1(Vector3(5, 0, 0), Quaternion::Identity());
//const SE3Transform from_right_transform2(Vector3(5, 0, 0), AngleAxis(0.25 * M_PI, Vector3::UnitY()).toRotationMatrix());
//
//INSTANTIATE_TEST_CASE_P(CheckProjectionInstantiation,
//                        CheckProjection,
//                        ::testing::Values(SE3Transform(from_right_transform1),
//                                          SE3Transform(from_right_transform2)));
//
//INSTANTIATE_TEST_CASE_P(CheckTriangulationInstantiation,
//                        CheckTriangulation,
//                        ::testing::Values(SE3Transform(from_right_transform1),
//                                          SE3Transform(from_right_transform2)));
//
//INSTANTIATE_TEST_CASE_P(CheckComputeFundamentalMatrixInstantiation,
//                        CheckComputeFundamentalMatrix,
//                        ::testing::Values(SE3Transform(from_right_transform1),
//                                          SE3Transform(from_right_transform2)));
//
//INSTANTIATE_TEST_CASE_P(VisionNormalizedGeometryTestInstantiation,
//                        VisionNormalizedGeometryTest,
//                        ::testing::Values(SE3Transform(from_right_transform1),
//                                          SE3Transform(from_right_transform2)));

#else

// Google Test may not support value-parameterized tests with some
// compilers. If we use conditional compilation to compile out all
// code referring to the gtest_main library, MSVC linker will not link
// that library at all and consequently complain about missing entry
// point defined in that library (fatal error LNK1561: entry point
// must be defined). This dummy test keeps gtest_main linked in.
TEST(DummyTest, ValueParameterizedTestsAreNotSupportedOnThisPlatform) {}

#endif

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}