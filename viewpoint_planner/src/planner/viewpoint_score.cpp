//==================================================
// viewpoint_score.cpp.cpp
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 28.03.17
//
//==================================================
// viewpoint_score.h.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 28.03.17
//

#include "viewpoint_score.h"

namespace viewpoint_planner {

ViewpointScore::ViewpointScore(
        const Options& options,
        std::function<Vector3(const Viewpoint&, const VoxelType*, const Vector2&)> normal_vector_function)
    : options_(options), normal_vector_function_(normal_vector_function) {
  voxel_sensor_size_ratio_falloff_factor_ = 1 / options_.voxel_sensor_size_ratio_inv_falloff_factor;
  incidence_angle_threshold_ = options_.incidence_angle_threshold_degrees * FloatType(M_PI) / FloatType(180);
  incidence_angle_falloff_factor_ =
          1 / (options_.incidence_angle_inv_falloff_factor_degrees * FloatType(M_PI) / FloatType(180));
}

WeightType ViewpointScore::computeResolutionInformationFactor(const Viewpoint& viewpoint, const VoxelType* node) const {
  const Vector3& camera_position = viewpoint.pose().getWorldPosition();
  const FloatType distance = (camera_position - node->getBoundingBox().getCenter()).norm();
  const FloatType size_x = node->getBoundingBox().getExtent(0);
  const FloatType relative_size_on_sensor = viewpoint.camera().computeRelativeSizeOnSensorHorizontal(size_x, distance);
//  AIT_ASSERT(relative_size_on_sensor >= 0);
  const FloatType voxel_sensor_size_ratio_threshold = options_.voxel_sensor_size_ratio_threshold;
  if (relative_size_on_sensor >= voxel_sensor_size_ratio_threshold) {
    return 1;
  }
  const FloatType factor = std::exp(-voxel_sensor_size_ratio_falloff_factor_ * (voxel_sensor_size_ratio_threshold - relative_size_on_sensor));
//  AIT_ASSERT(factor <= 1);
  return factor;
}

WeightType ViewpointScore::computeIncidenceInformationFactor(
        const Viewpoint& viewpoint, const VoxelType* node, const Vector3& normal_vector) const {
  if (normal_vector == Vector3::Zero()) {
    return 1;
  }
  const Vector3& camera_position = viewpoint.pose().getWorldPosition();
  const Vector3 incidence_vector = (camera_position - node->getBoundingBox().getCenter()).normalized();
  FloatType dot_product = ait::clamp<FloatType>(incidence_vector.dot(normal_vector), -1, +1);
  if (dot_product <= 0) {
    if (options_.incidence_ignore_dot_product_sign) {
      dot_product = std::abs(dot_product);
    }
    else {
      return 0;
    }
  }
  const FloatType angle = std::acos(dot_product);
  if (angle <= incidence_angle_threshold_) {
    return 1;
  }
  const FloatType factor = std::exp(-incidence_angle_falloff_factor_ * (angle - incidence_angle_threshold_));
//  AIT_ASSERT(factor <= 1);
  return factor;
}

WeightType ViewpointScore::computeIncidenceInformationFactor(const Viewpoint& viewpoint, const VoxelType* node) const {
  const Vector2 image_coordinates = viewpoint.projectWorldPointIntoImage(node->getBoundingBox().getCenter());
  return computeIncidenceInformationFactor(viewpoint, node, image_coordinates);
}

WeightType ViewpointScore::computeIncidenceInformationFactor(
        const Viewpoint& viewpoint, const VoxelType* node, const Vector2& image_coordinates) const {
  const Vector3 normal_vector = normal_vector_function_(viewpoint, node, image_coordinates);
  return computeIncidenceInformationFactor(viewpoint, node, normal_vector);
}

FloatType ViewpointScore::computeViewpointObservationFactor(const Viewpoint& viewpoint, const VoxelType* node) const {
  WeightType resolution_factor = computeResolutionInformationFactor(viewpoint, node);
  WeightType incidence_factor = computeIncidenceInformationFactor(viewpoint, node);
  WeightType factor = resolution_factor * incidence_factor;
  return factor;
}

FloatType ViewpointScore::computeViewpointObservationFactor(
        const Viewpoint& viewpoint, const VoxelType* node, const Vector2& image_coordinates) const {
  WeightType resolution_factor = computeResolutionInformationFactor(viewpoint, node);
  WeightType incidence_factor = computeIncidenceInformationFactor(viewpoint, node, image_coordinates);
  WeightType factor = resolution_factor * incidence_factor;
  return factor;
}

FloatType ViewpointScore::computeViewpointObservationScore(
        const Viewpoint& viewpoint,
        const OccupiedTreeType::IntersectionResult& ir) const {
  return computeViewpointObservationScore(viewpoint, ir.node);
//  // TODO: Make sure tree is not pruned for occupied voxels.
//  // Or consider how to handle larger unknown/known voxels
//  CounterType voxel_observation_count = result.node->getObject()->observation_count;
////  if (result.depth < data_->octree_->getTreeDepth()) {
////    voxel_observation_count /= 1 << (data_->octree_->getTreeDepth() - result.depth);
////  }
//  WeightType information_factor = computeObservationInformationFactor(voxel_observation_count);
////  WeightType information_factor = 1;
//  WeightType weight = result.node->getObject()->weight;
//  WeightType information = information_factor * weight;
//  return information;
}

FloatType ViewpointScore::computeViewpointObservationScore(
        const Viewpoint& viewpoint,
        const OccupiedTreeType::IntersectionResultWithScreenCoordinates& ir) const {
  return computeViewpointObservationScore(viewpoint, ir.intersection_result.node, ir.screen_coordinates);
//  // TODO: Make sure tree is not pruned for occupied voxels.
//  // Or consider how to handle larger unknown/known voxels
//  CounterType voxel_observation_count = result.node->getObject()->observation_count;
////  if (result.depth < data_->octree_->getTreeDepth()) {
////    voxel_observation_count /= 1 << (data_->octree_->getTreeDepth() - result.depth);
////  }
//  WeightType information_factor = computeObservationInformationFactor(voxel_observation_count);
////  WeightType information_factor = 1;
//  WeightType weight = result.node->getObject()->weight;
//  WeightType information = information_factor * weight;
//  return information;
}

FloatType ViewpointScore::computeViewpointObservationScore(
        const Viewpoint& viewpoint, const VoxelType* node) const {
  WeightType observation_factor = computeViewpointObservationFactor(viewpoint, node);
  WeightType weight = node->getObject()->weight;
  WeightType information = observation_factor * weight;
  return information;
}

FloatType ViewpointScore::computeViewpointObservationScore(
        const Viewpoint& viewpoint, const VoxelType* node, const Vector2& image_coordinates) const {
  WeightType observation_factor = computeViewpointObservationFactor(viewpoint, node, image_coordinates);
  WeightType weight = node->getObject()->weight;
  WeightType information = observation_factor * weight;
  return information;
}

}
