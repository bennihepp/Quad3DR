/*
 * viewpoint.h
 *
 *  Created on: Dec 30, 2016
 *      Author: bhepp
 */
#pragma once

#include <bh/eigen.h>
#include <bh/eigen_serialization.h>
#include <memory>
#include <unordered_set>
#include <boost/serialization/access.hpp>
#include <bh/common.h>
#include <bh/eigen_utils.h>
#include <bh/math/geometry.h>
#include "../reconstruction/dense_reconstruction.h"

using reconstruction::PinholeCamera;
using reconstruction::Point3DId;
using reconstruction::Point3D;
using reconstruction::SparseReconstruction;

class Viewpoint {
public:
  using FloatType = reconstruction::FloatType;
  USE_FIXED_EIGEN_TYPES(FloatType)
  using RayType = bh::Ray<FloatType>;
  using Pose = reconstruction::Pose;

  static constexpr FloatType DEFAULT_PROJECTION_MARGIN = 10;
  static constexpr FloatType MAX_DISTANCE_DEVIATION_BY_STDDEV = 3;
  static constexpr FloatType MAX_NORMAL_DEVIATION_BY_STDDEV = 3;

  Viewpoint(FloatType projection_margin = DEFAULT_PROJECTION_MARGIN);

  Viewpoint(const PinholeCamera* camera, const Pose& pose, FloatType projection_margin = DEFAULT_PROJECTION_MARGIN);

  Viewpoint(const Viewpoint& other);

  Viewpoint(Viewpoint&& other);

  Viewpoint& operator=(const Viewpoint& other);

  Viewpoint& operator=(Viewpoint&& other);

  const Pose& pose() const {
    return pose_;
  }

  const PinholeCamera& camera() const {
    return *camera_;
  }

  FloatType getDistanceTo(const Viewpoint& other) const;

  FloatType getDistanceTo(const Viewpoint& other, FloatType rotation_factor) const;

  std::unordered_set<Point3DId> getProjectedPoint3DIds(const SparseReconstruction::Point3DMapType& points) const;

  std::unordered_set<Point3DId> getProjectedPoint3DIdsFiltered(const SparseReconstruction::Point3DMapType& points) const;

  bool isPointFiltered(const Point3D& point) const;

  Vector3 projectWorldPointIntoCamera(const Vector3& point_world) const;

  bool isWorldPointVisible(const Vector3& point_world) const;

  Vector2 projectWorldPointIntoImage(const Vector3& point_world, bool* behind_camera = nullptr) const;

  RayType getCameraRay(FloatType x, FloatType y) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // Boost serialization
  friend class boost::serialization::access;

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & camera_;
    ar & pose_;
    ar & projection_margin_;
    ar & transformation_world_to_image_;
  }

  const PinholeCamera* camera_;
  Pose pose_;
  FloatType projection_margin_;
  Matrix3x4 transformation_world_to_image_;
};
