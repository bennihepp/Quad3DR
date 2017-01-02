/*
 * viewpoint.cpp
 *
 *  Created on: Dec 30, 2016
 *      Author: bhepp
 */

#include <ait/utilities.h>
#include "viewpoint.h"

using reconstruction::Point3DStatistics;

Viewpoint::Viewpoint(FloatType projection_margin /*= DEFAULT_PROJECTION_MARGIN*/)
: camera_(nullptr), projection_margin_(projection_margin),
  transformation_world_to_image_(pose_.getTransformationWorldToImage()) {}

Viewpoint::Viewpoint(const PinholeCamera* camera, const Pose& pose, FloatType projection_margin /*= DEFAULT_PROJECTION_MARGIN*/)
: camera_(camera), pose_(pose), projection_margin_(projection_margin),
  transformation_world_to_image_(pose_.getTransformationWorldToImage()) {
  if (!pose.isValid()) {
    throw AIT_EXCEPTION("Received invalid pose");
  }
}

Viewpoint::Viewpoint(const Viewpoint& other)
: camera_(other.camera_), pose_(other.pose_),
  projection_margin_(other.projection_margin_),
  transformation_world_to_image_(other.transformation_world_to_image_) {}

Viewpoint::Viewpoint(Viewpoint&& other)
: camera_(other.camera_), pose_(std::move(other.pose_)),
  projection_margin_(other.projection_margin_),
  transformation_world_to_image_(std::move(other.transformation_world_to_image_)) {}

Viewpoint& Viewpoint::operator=(const Viewpoint& other) {
  if (this != &other) {
    camera_ = other.camera_;
    pose_ = other.pose_;
    projection_margin_ = other.projection_margin_;
    transformation_world_to_image_ = other.transformation_world_to_image_;
  }
  return *this;
}

Viewpoint& Viewpoint::operator=(Viewpoint&& other) {
  if (this != &other) {
    camera_ = other.camera_;
    pose_ = std::move(other.pose_);
    projection_margin_ = other.projection_margin_;
    transformation_world_to_image_ = std::move(other.transformation_world_to_image_);
  }
  return *this;
}

Viewpoint::FloatType Viewpoint::getDistanceTo(const Viewpoint& other) const {
  return pose_.getDistanceTo(other.pose_);
}

Viewpoint::FloatType Viewpoint::getDistanceTo(const Viewpoint& other, FloatType rotation_factor) const {
  return pose_.getDistanceTo(other.pose_, rotation_factor);
}

std::unordered_set<Point3DId> Viewpoint::getProjectedPoint3DIds(const SparseReconstruction::Point3DMapType& points) const {
  ait::Timer timer;
  std::unordered_set<Point3DId> proj_point_ids;
  for (const auto& entry : points) {
    const Point3DId point_id = entry.first;
    const Point3D& point_world = entry.second;
    Vector2 point_image = projectWorldPointIntoImage(point_world.pos);
    bool projected = camera_->isPointInViewport(point_image, projection_margin_);
    if (projected) {
      proj_point_ids.insert(point_id);
    }
  }
  timer.printTiming("getProjectedPoint3DIds");
  return proj_point_ids;
}

Viewpoint::RayType Viewpoint::getCameraRay(FloatType x, FloatType y) const {
  Vector3 origin = pose_.getWorldPosition();
  // TODO: Ray origin at projection center or on "image plane"?
  Vector3 direction_camera = camera_->getCameraRay(x, y);
//  Vector3 direction_camera = camera_->unprojectPoint(x, y, 1.0);
  Vector3 direction = pose_.getTransformationImageToWorld().topLeftCorner<3, 3>() * direction_camera;
//  origin = origin + direction;
  return RayType(origin, direction);
}

std::unordered_set<Point3DId> Viewpoint::getProjectedPoint3DIdsFiltered(const SparseReconstruction::Point3DMapType& points) const {
  ait::Timer timer;
  std::unordered_set<Point3DId> proj_point_ids;
  for (const auto& entry : points) {
    const Point3DId point_id = entry.first;
    const Point3D& point_world = entry.second;
    Vector2 point_image = projectWorldPointIntoImage(point_world.getPosition());
    bool projected = camera_->isPointInViewport(point_image, projection_margin_);
    if (projected) {
      if (!isPointFiltered(point_world)) {
        proj_point_ids.insert(point_id);
      }
    }
  }
  timer.printTiming("getProjectedPoint3DIdsFiltered");
  return proj_point_ids;
}

bool Viewpoint::isPointFiltered(const Point3D& point) const {
  const Point3DStatistics& statistics = point.getStatistics();
  std::tuple<FloatType, Vector3> result = ait::computeDistanceAndDirection(point.getPosition(), pose_.getWorldPosition());
  FloatType dist_deviation = std::abs(std::get<0>(result) - statistics.averageDistance());
  if (dist_deviation > MAX_DISTANCE_DEVIATION_BY_STDDEV * statistics.stddevDistance()) {
    return true;
  }
  FloatType one_minus_dot_product = 1 - std::get<1>(result).dot(point.getNormal());
  FloatType one_minus_dot_deviation = std::abs(one_minus_dot_product);
  if (one_minus_dot_deviation > MAX_NORMAL_DEVIATION_BY_STDDEV * statistics.stddevOneMinusDotProduct()) {
    return true;
  }
  return false;
}

Viewpoint::Vector2 Viewpoint::projectWorldPointIntoImage(const Vector3& point_world) const {
  Vector3 point_camera  = transformation_world_to_image_ * point_world.homogeneous();
  Vector2 point_image = camera_->projectPoint(point_camera);
  return point_image;
}
