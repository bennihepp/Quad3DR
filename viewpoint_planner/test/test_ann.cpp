//==================================================
// test_ann.cpp.cpp
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 22.03.17
//

#include <bh/algorithm.h>
#include <bh/math/utilities.h>
#include <bh/nn/approximate_nearest_neighbor.h>
#include <random>
#include "gtest/gtest.h"

#pragma GCC optimize("O0")

namespace {
using FloatType = float;
using size_t = std::size_t;

const size_t kNumPoints = 1000;
const size_t kPointDimension = 3;
const size_t kKnn = 50;
const FloatType kNumMatchesExp = 0.8;
const FloatType kMaxApproximationFactor = 1.2;

using AnnType = bh::ApproximateNearestNeighbor<FloatType, kPointDimension>;
using AnnPointType = typename AnnType::Point;

class AnnTest : public :: testing::Test {
protected:
  AnnTest()
      : uniform_dist(-10, 10) {
    for (size_t i = 0; i < kNumPoints; ++i) {
      const FloatType x = getRandomReal();
      const FloatType y = getRandomReal();
      const FloatType z = getRandomReal();
      const AnnPointType point(x, y, z);
      points.push_back(point);
    }
    ann.initIndex(points.begin(), points.end());
  }

  void resetAnn() {
    ann = AnnType();
  }

  virtual ~AnnTest() override {}

  FloatType getRandomReal() {
    return uniform_dist(rnd);
  }

  std::mt19937_64 rnd;
  std::uniform_real_distribution<FloatType> uniform_dist;
  AnnType ann;
  std::vector<AnnPointType> points;
};
}

TEST_F(AnnTest, AddPointsShouldNotFail) {
  resetAnn();
  for (size_t i = 0; i < kNumPoints; ++i) {
    const FloatType x = getRandomReal();
    const FloatType y = getRandomReal();
    const FloatType z = getRandomReal();
    ann.addPoint(AnnPointType(x, y, z));
  }
}

TEST_F(AnnTest, AddPointsShouldBeCorrect) {
  resetAnn();
  std::vector<AnnPointType> points;
  for (size_t i = 0; i < kNumPoints; ++i) {
    const FloatType x = getRandomReal();
    const FloatType y = getRandomReal();
    const FloatType z = getRandomReal();
    const AnnPointType point(x, y, z);
    points.push_back(point);
    ann.addPoint(point);
  }
  for (size_t i = 0; i < ann.numPoints(); ++i) {
    EXPECT_EQ(ann.getPoint(i), points[i]);
  }
}

TEST_F(AnnTest, InitIndexPointsShouldBeCorrect) {
  for (size_t i = 0; i < ann.numPoints(); ++i) {
    EXPECT_EQ(ann.getPoint(i), points[i]);
  }
}

TEST_F(AnnTest, AddPointsLaterShouldWork) {
  resetAnn();
  std::vector<AnnPointType> points;
  for (size_t i = 0; i < kNumPoints; ++i) {
    const FloatType x = getRandomReal();
    const FloatType y = getRandomReal();
    const FloatType z = getRandomReal();
    const AnnPointType point(x, y, z);
    points.push_back(point);
  }
  ann.initIndex(points.begin(), points.end());
  for (size_t i = 0; i < kNumPoints; ++i) {
    const FloatType x = getRandomReal();
    const FloatType y = getRandomReal();
    const FloatType z = getRandomReal();
    const AnnPointType point(x, y, z);
    points.push_back(point);
    ann.addPoint(point);
  }
  EXPECT_EQ(ann.numPoints(), 2 * kNumPoints);
  for (size_t i = 0; i < ann.numPoints(); ++i) {
    EXPECT_EQ(ann.getPoint(i), points[i]);
  }
}

TEST_F(AnnTest, KnnSearchShouldWork) {
  std::vector<typename AnnType::IndexType> indices(kKnn);
  std::vector<typename AnnType::DistanceType> distances(kKnn);
  std::vector<typename AnnType::IndexType> indices2(kKnn);
  std::vector<typename AnnType::DistanceType> distances2(kKnn);
  for (size_t i = 0; i < kNumPoints; ++i) {
    const AnnPointType& query_point = points[i];
    ann.knnSearch(query_point, kKnn, &indices, &distances);
    EXPECT_EQ(indices.size(), distances.size());
    EXPECT_LE(indices.size(), kKnn);
    ann.knnSearchExact(query_point, kKnn, &indices2, &distances2);
    for (size_t j = 0; j < std::min(distances.size(), distances2.size()); ++j) {
      EXPECT_LE(distances[j], distances2[j] * kMaxApproximationFactor);
    }
    const size_t num_matches = std::count_if(indices.begin(), indices.end(), [&](const size_t index1) {
      const auto it = std::find_if(indices2.begin(), indices2.end(), [&](const size_t index2) {
        return index1 == index2;
      });
      const bool match = it != indices2.end();
      return match;
    });
    EXPECT_GE(num_matches, kNumMatchesExp * std::min(indices.size(), indices2.size()));
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}