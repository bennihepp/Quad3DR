//==================================================
// test_se3_transform.cpp
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 12.04.17
//==================================================

#include <random>
#include "gtest/gtest.h"
#include <bh/algorithm.h>
#include <bh/math/utilities.h>
#include <bh/se3_transform.h>

#pragma GCC optimize("O0")

namespace {
using FloatType = double;
using size_t = std::size_t;

BH_USE_FIXED_EIGEN_TYPES(FloatType);
using SE3Transform = bh::SE3Transform<FloatType>;

const size_t kNumPointsToCheck = 1000;
const FloatType kErrorTolerance = 1e-3;
const FloatType kSquaredErrorTolerance = 1e-3;

#if GTEST_HAS_PARAM_TEST

class SE3TransformTest : public ::testing::Test {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//  SE3TransformTest() {}

  virtual ~SE3TransformTest() {}

  virtual void SetUp() {
    std::random_device rd;
    rnd.seed(rd());

    uniform_dist = std::uniform_real_distribution<FloatType>(-10, 10);

    from_a_transform = SE3Transform(Vector3(-5, 0, 0), AngleAxis(0 * M_PI, Vector3::UnitY()).toRotationMatrix());
    from_b_transform = SE3Transform(Vector3(5, 0, 0), AngleAxis(-0.2 * M_PI, Vector3::UnitY()).toRotationMatrix());

    to_a_transform = from_a_transform.inverse();
    to_b_transform = from_b_transform.inverse();

    from_a_to_b_transform = to_b_transform * from_a_transform;
    from_b_to_a_transform = to_a_transform * from_b_transform;
  }

  virtual void TearDown() {}

protected:
  Vector3 getRandomPoint() {
    return Vector3(uniform_dist(rnd), uniform_dist(rnd), uniform_dist(rnd));
  }

  std::mt19937_64 rnd;
  std::uniform_real_distribution<FloatType> uniform_dist;

  SE3Transform from_a_transform;
  SE3Transform to_a_transform;
  SE3Transform from_b_transform;
  SE3Transform to_b_transform;
  SE3Transform from_a_to_b_transform;
  SE3Transform from_b_to_a_transform;
};
}

TEST_F(SE3TransformTest, CheckInverse) {
  for (size_t i = 0; i < kNumPointsToCheck; ++i) {
    const Vector3 point = getRandomPoint();
    const Vector3 point_prime = to_a_transform.transform(point);
    const Vector3 point_back = from_a_transform.transform(point_prime);
    EXPECT_LE((point - point_back).squaredNorm(), kSquaredErrorTolerance);
  }
  for (size_t i = 0; i < kNumPointsToCheck; ++i) {
    const Vector3 point = getRandomPoint();
    const Vector3 point_prime = to_b_transform.transform(point);
    const Vector3 point_back = from_b_transform.transform(point_prime);
    EXPECT_LE((point - point_back).squaredNorm(), kSquaredErrorTolerance);
  }
  for (size_t i = 0; i < kNumPointsToCheck; ++i) {
    const Vector3 point = getRandomPoint();
    const Vector3 point_prime = from_a_to_b_transform.transform(point);
    const Vector3 point_back = from_b_to_a_transform.transform(point_prime);
    EXPECT_LE((point - point_back).squaredNorm(), kSquaredErrorTolerance);
  }
}

TEST_F(SE3TransformTest, CheckTransitivity) {
  for (size_t i = 0; i < kNumPointsToCheck; ++i) {
    const Vector3 point_a = getRandomPoint();
    const Vector3 point = from_a_transform.transform(point_a);
    const Vector3 point_b = to_b_transform.transform(point);
    const Vector3 point_b_tmp = from_a_to_b_transform.transform(point_a);
    EXPECT_LE((point_b - point_b_tmp).squaredNorm(), kSquaredErrorTolerance);
  }
  for (size_t i = 0; i < kNumPointsToCheck; ++i) {
    const Vector3 point_b = getRandomPoint();
    const Vector3 point = from_b_transform.transform(point_b);
    const Vector3 point_a = to_a_transform.transform(point);
    const Vector3 point_a_tmp = from_b_to_a_transform.transform(point_b);
    EXPECT_LE((point_a - point_a_tmp).squaredNorm(), kSquaredErrorTolerance);
  }
  for (size_t i = 0; i < kNumPointsToCheck; ++i) {
    const Vector3 point = getRandomPoint();
    const Vector3 point_prime = from_a_to_b_transform.transform(point);
    const Vector3 point_back = from_b_to_a_transform.transform(point_prime);
    EXPECT_LE((point - point_back).squaredNorm(), kSquaredErrorTolerance);
  }
}

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