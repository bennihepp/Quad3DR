//==================================================
// viewpoint_planner_sampling.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 5, 2017
//==================================================

#include "viewpoint_planner.h"
#include <cmath>

std::pair<bool, ViewpointPlanner::Pose> ViewpointPlanner::samplePose(
    const std::size_t max_trials /*= (std::size_t)-1*/,
    const bool biased_orientation /*=true*/) const {
  return samplePose(pose_sample_bbox_, drone_bbox_, max_trials, biased_orientation);
}

std::pair<bool, ViewpointPlanner::Pose> ViewpointPlanner::samplePose(const BoundingBoxType& bbox,
    const BoundingBoxType& object_bbox, std::size_t max_trials /*= (std::size_t)-1*/,
    const bool biased_orientation /*=true*/) const {
  if (max_trials == (std::size_t)-1) {
    max_trials = options_.pose_sample_num_trials;
  }
  std::pair<bool, ViewpointPlanner::Vector3> pos_result = samplePosition(bbox, object_bbox, max_trials);
  if (!pos_result.first) {
    return std::make_pair(false, Pose());
  }
  const Pose::Vector3 pos = pos_result.second.cast<Pose::Vector3::Scalar>();
  Pose::Quaternion orientation;
  if (biased_orientation) {
    orientation = sampleBiasedOrientation(pos, data_->roi_bbox_);
  }
  else {
    orientation = sampleOrientation();
  }
  Pose pose = Pose::createFromImageToWorldTransformation(pos, orientation);
  return std::make_pair(true, pose);
}

std::pair<bool, ViewpointPlanner::Pose> ViewpointPlanner::samplePose(const RegionType& region,
    const BoundingBoxType& object_bbox, std::size_t max_trials /*= (std::size_t)-1*/,
    const bool biased_orientation /*=true*/) const {
  if (max_trials == (std::size_t)-1) {
    max_trials = options_.pose_sample_num_trials;
  }
  std::pair<bool, ViewpointPlanner::Vector3> pos_result = samplePosition(region, object_bbox, max_trials);
  if (!pos_result.first) {
    return std::make_pair(false, Pose());
  }
  const Pose::Vector3 pos = pos_result.second.cast<Pose::Vector3::Scalar>();
  Pose::Quaternion orientation;
  if (biased_orientation) {
    orientation = sampleBiasedOrientation(pos, data_->roi_bbox_);
  }
  else {
    orientation = sampleOrientation();
  }
  Pose pose = Pose::createFromImageToWorldTransformation(pos, orientation);
  return std::make_pair(true, pose);
}

ViewpointPlanner::Pose::Quaternion ViewpointPlanner::sampleOrientation() const {
  // We just sample from the lower sphere
//    using Scalar = Pose::Quaternion::Scalar;
//    Pose::Quaternion orientation = Pose::Quaternion::UnitRandom();
//    Vector3 euler_angles =
//        orientation.toRotationMatrix().eulerAngles(2, 1, 0);
//    Scalar yaw = euler_angles(0);
//    Scalar pitch = euler_angles(1);
//    Scalar roll = euler_angles(2);
  FloatType z = random_.sampleUniform(-1, 0);
  FloatType theta = random_.sampleUniform(-M_PI, +M_PI);
  FloatType x = std::sin(theta) * std::sqrt(1 - z*z);
  FloatType y = std::cos(theta) * std::sqrt(1 - z*z);
  Pose::Vector3 z_axis(0, 0, 1);
  Pose::Vector3 pose_z_axis(x, y, z);
  Pose::Vector3 pose_x_axis;
  if (z == 1) {
    pose_x_axis = Pose::Vector3(1, 0, 0);
  }
  else {
    pose_x_axis = pose_z_axis.cross(z_axis);
  }
  Pose::Vector3 pose_y_axis = pose_z_axis.cross(pose_x_axis);
  Pose::Matrix3x3 rotation;
  rotation.col(0) = pose_x_axis.normalized();
  rotation.col(1) = pose_y_axis.normalized();
  rotation.col(2) = pose_z_axis.normalized();
  return Pose::Quaternion(rotation);
}

ViewpointPlanner::Pose::Quaternion ViewpointPlanner::sampleBiasedOrientation(const Vector3& pos, const BoundingBoxType& bias_bbox) const {
  const FloatType dist = (pos - bias_bbox.getCenter()).norm();
  const FloatType bbox_fov_angle = std::atan(bias_bbox.getMaxExtent() / (2 * dist));
  const FloatType angle_stddev = bh::clamp<FloatType>(bbox_fov_angle, 0, M_PI / 2);
  FloatType angle1 = random_.sampleNormal(0, angle_stddev);
  FloatType angle2 = random_.sampleNormal(0, angle_stddev);
  Vector3 bbox_direction = (bias_bbox.getCenter() - pos).normalized();
  angle1 = bh::wrapRadiansToMinusPiAndPi(angle1);
  angle2 = bh::wrapRadiansToMinusPiAndPi(angle2);
//  angle2 = bh::clamp<FloatType>(angle2, 0, M_PI / 2);
  // Compute viewing direction (i.e. pose z axis)
  Pose::Vector3 viewing_direction = AngleAxis(angle1, Vector3::UnitZ()) * bbox_direction;
  viewing_direction = AngleAxis(angle2, viewing_direction.cross(Vector3::UnitZ())) * viewing_direction;
  // Multicopter camera can only cover pitch in the range of [-90, 0] degrees
  if (viewing_direction(2) > 0) {
    viewing_direction(2) = 0;
  }

  // TODO: Remove old code
//  FloatType angle_deviation = std::abs(random_.sampleNormal(0, angle_stddev));
//  FloatType spherical_angle = random_.sampleUniform(0, 2 * M_PI);
//  Pose::Vector3 viewing_direction;
//  viewing_direction <<
//      viewing_direction(0) * std::cos(angle_deviation),
//      std::sin(angle_deviation) * std::cos(spherical_angle),
//      std::sin(angle_deviation) * std::sin(spherical_angle);

  // Now rotate the viewing direction according to (pose -> bbox) transformation
  // Compute pose x (right) and y (down) axis from viewing direction (pose z axis)
  const Pose::Vector3 z_axis(0, 0, 1);
  Pose::Vector3 pose_z_axis = viewing_direction;
  Pose::Vector3 pose_x_axis;
  // If z axis and viewing direction are parallel, set x (right) direction manually
  if (bh::isApproxEqual(std::abs(z_axis.dot(pose_z_axis)), (FloatType)1, kDotProdEqualTolerance )) {
    pose_x_axis = Pose::Vector3(1, 0, 0);
  }
  else {
    pose_x_axis = pose_z_axis.cross(z_axis);
  }
//  std::cout << "viewing_direction=" << viewing_direction << std::endl;
  const Pose::Vector3 pose_y_axis = pose_z_axis.cross(pose_x_axis);
  Pose::Matrix3x3 rotation;
  rotation.col(0) = pose_x_axis.normalized();
  rotation.col(1) = pose_y_axis.normalized();
  rotation.col(2) = pose_z_axis.normalized();
  return Pose::Quaternion(rotation);
}

std::pair<bool, ViewpointPlanner::Vector3> ViewpointPlanner::samplePosition(
    const std::size_t max_trials /*= (std::size_t)-1*/) const {
  return samplePosition(pose_sample_bbox_, drone_bbox_, max_trials);
}

std::pair<bool, ViewpointPlanner::Vector3> ViewpointPlanner::samplePosition(const BoundingBoxType& bbox,
    const BoundingBoxType& object_bbox, std::size_t max_trials /*= (std::size_t)-1*/) const {
  if (max_trials == (std::size_t)-1) {
    max_trials = options_.pose_sample_num_trials;
  }
  for (size_t i = 0; i < max_trials; ++i) {
    Vector3 pos(
        random_.sampleUniform(bbox.getMinimum(0), bbox.getMaximum(0)),
        random_.sampleUniform(bbox.getMinimum(1), bbox.getMaximum(1)),
        random_.sampleUniform(bbox.getMinimum(2), bbox.getMaximum(2)));
    const bool valid = isValidObjectPosition(pos, object_bbox);
    if (valid) {
      return std::make_pair(true, pos);
    }
  }
  return std::make_pair(false, Vector3());
}

std::pair<bool, ViewpointPlanner::Vector3> ViewpointPlanner::samplePosition(const RegionType& region,
    const BoundingBoxType& object_bbox, std::size_t max_trials /*= (std::size_t)-1*/) const {
  if (max_trials == (std::size_t)-1) {
    max_trials = options_.pose_sample_num_trials;
  }
  const BoundingBoxType bbox = region.getBoundingBox();
  for (size_t i = 0; i < max_trials; ++i) {
    Vector3 pos(
        random_.sampleUniform(bbox.getMinimum(0), bbox.getMaximum(0)),
        random_.sampleUniform(bbox.getMinimum(1), bbox.getMaximum(1)),
        random_.sampleUniform(bbox.getMinimum(2), bbox.getMaximum(2)));
    const bool within_region = region.isPointInside(pos);
    const bool valid = isValidObjectPosition(pos, object_bbox);
    if (within_region && valid) {
      return std::make_pair(true, pos);
    }
  }
  return std::make_pair(false, Vector3());
}

std::pair<bool, ViewpointPlanner::Pose>
ViewpointPlanner::sampleSurroundingPose(const Pose& pose) const {
  // Sample position from sphere around pose
  Vector3 sampled_pos;
  bool found_position = false;
  for (size_t i = 0; i < options_.pose_sample_num_trials; ++i) {
    random_.sampleSphericalShell(options_.pose_sample_min_radius, options_.pose_sample_max_radius, &sampled_pos);
    sampled_pos += pose.getWorldPosition();
    if (pose_sample_bbox_.isInside(sampled_pos)
        && isValidObjectPosition(sampled_pos, drone_bbox_)) {
      found_position = true;
      break;
    }
  }
  if (!found_position) {
    return std::make_pair(false, Pose());
  }
  Pose::Quaternion sampled_orientation = sampleBiasedOrientation(sampled_pos, data_->roi_bbox_);
  Pose sampled_pose = Pose::createFromImageToWorldTransformation(sampled_pos, sampled_orientation);
  return std::make_pair(true, sampled_pose);
}
