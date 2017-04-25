//==================================================
// viewpoint_planner_scoring.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 5, 2017
//==================================================

#include "viewpoint_planner.h"

ViewpointPlanner::WeightType ViewpointPlanner::computeResolutionInformationFactor(const Viewpoint& viewpoint, const VoxelType* node) const {
  const Vector3& camera_position = viewpoint.pose().getWorldPosition();
  const FloatType distance = (camera_position - node->getBoundingBox().getCenter()).norm();
  const FloatType size_x = node->getBoundingBox().getExtent(0);
  const FloatType relative_size_on_sensor = viewpoint.camera().computeRelativeSizeOnSensorHorizontal(size_x, distance);
//  BH_ASSERT(relative_size_on_sensor >= 0);
  const FloatType voxel_sensor_size_ratio_threshold = options_.voxel_sensor_size_ratio_threshold;
  if (relative_size_on_sensor >= voxel_sensor_size_ratio_threshold) {
    return 1;
  }
  const FloatType factor = std::exp(-voxel_sensor_size_ratio_falloff_factor_ * (voxel_sensor_size_ratio_threshold - relative_size_on_sensor));
//  BH_ASSERT(factor <= 1);
  return factor;
}

ViewpointPlanner::Vector3 ViewpointPlanner::computeNormalVector(
        const Viewpoint& viewpoint, const VoxelType* node, const Vector2& image_coordinates) const {
  Vector3 normal_vector;
  if (options_.enable_opengl) {
    const std::size_t x = static_cast<std::size_t>(image_coordinates(0));
    const std::size_t y = static_cast<std::size_t>(image_coordinates(1));
    normal_vector = computePoissonMeshNormalVector(viewpoint, x, y);
  }
  else {
    normal_vector = node->getObject()->normal;
  }
  return normal_vector;
}

ViewpointPlanner::WeightType ViewpointPlanner::computeIncidenceInformationFactor(
    const Viewpoint& viewpoint, const VoxelType* node, const Vector3& normal_vector) const {
  return scorer_.computeIncidenceInformationFactor(viewpoint, node, normal_vector);
}

ViewpointPlanner::WeightType ViewpointPlanner::computeIncidenceInformationFactor(
    const Viewpoint& viewpoint, const VoxelType* node) const {
  return scorer_.computeIncidenceInformationFactor(viewpoint, node);
}

ViewpointPlanner::WeightType ViewpointPlanner::computeIncidenceInformationFactor(
    const Viewpoint& viewpoint, const VoxelType* node, const Vector2& screen_coordinates) const {
  return scorer_.computeIncidenceInformationFactor(viewpoint, node, screen_coordinates);
}

Viewpoint ViewpointPlanner::getVirtualViewpoint(const Pose& pose) const {
  Viewpoint viewpoint(&virtual_camera_, pose);
  return viewpoint;
}

ViewpointPlanner::FloatType ViewpointPlanner::computeViewpointObservationFactor(const Viewpoint& viewpoint, const VoxelType* node) const {
  WeightType resolution_factor = computeResolutionInformationFactor(viewpoint, node);
  WeightType incidence_factor = computeIncidenceInformationFactor(viewpoint, node);
  WeightType factor = resolution_factor * incidence_factor;
  return factor;
}

ViewpointPlanner::FloatType ViewpointPlanner::computeViewpointObservationFactor(
    const Viewpoint& viewpoint, const VoxelType* node, const Vector2& screen_coordinates) const {
  WeightType resolution_factor = computeResolutionInformationFactor(viewpoint, node);
  WeightType incidence_factor = computeIncidenceInformationFactor(viewpoint, node, screen_coordinates);
  WeightType factor = resolution_factor * incidence_factor;
  return factor;
}

ViewpointPlanner::FloatType  ViewpointPlanner::computeViewpointObservationScore(
    const Viewpoint& viewpoint,
    const ViewpointPlannerData::OccupiedTreeType::IntersectionResult& result) const {
  return computeViewpointObservationScore(viewpoint, result.node);
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

ViewpointPlanner::FloatType  ViewpointPlanner::computeViewpointObservationScore(
    const Viewpoint& viewpoint,
    const ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates& result) const {
  return computeViewpointObservationScore(viewpoint, result.intersection_result.node, result.screen_coordinates);
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

ViewpointPlanner::FloatType ViewpointPlanner::computeViewpointObservationScore(const Viewpoint& viewpoint, const VoxelType* node) const {
  WeightType observation_factor = computeViewpointObservationFactor(viewpoint, node);
  WeightType weight = node->getObject()->weight;
  WeightType information = observation_factor * weight;
  return information;
}

ViewpointPlanner::FloatType ViewpointPlanner::computeViewpointObservationScore(
    const Viewpoint& viewpoint, const VoxelType* node, const Vector2& screen_coordinates) const {
  WeightType observation_factor = computeViewpointObservationFactor(viewpoint, node, screen_coordinates);
  WeightType weight = node->getObject()->weight;
  WeightType information = observation_factor * weight;
  return information;
}

ViewpointPlanner::FloatType ViewpointPlanner::computeNewInformation(
    const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data,
    const ViewpointEntryIndex new_viewpoint_index) const {
  const ViewpointEntry& new_viewpoint = viewpoint_entries_[new_viewpoint_index];
  FloatType new_information = std::accumulate(new_viewpoint.voxel_set.cbegin(), new_viewpoint.voxel_set.cend(),
    FloatType { 0 }, [&](const FloatType& value, const VoxelWithInformation& vi) {
      const FloatType observation_information = options_.viewpoint_information_factor * vi.information;
      FloatType novel_information = observation_information;
      auto it = viewpoint_path.observed_voxel_map.find(vi.voxel);
      if (it != viewpoint_path.observed_voxel_map.end()) {
        const WeightType voxel_weight = vi.voxel->getObject()->weight;
        BH_ASSERT(it->second <= voxel_weight);
        novel_information = std::min(observation_information, voxel_weight - it->second);
      }
      BH_ASSERT(novel_information >= 0);
      return value + novel_information;
    });
  //    VoxelWithInformationSet difference_set = bh::computeSetDifference(new_viewpoint.voxel_set, viewpoint_path.observed_voxel_set);
  //    FloatType new_information = std::accumulate(difference_set.cbegin(), difference_set.cend(),
  //        FloatType { 0 }, [](const FloatType& value, const VoxelWithInformation& voxel) {
  //          return value + voxel.information;
  //    });
  return new_information;
}

ViewpointPlanner::FloatType ViewpointPlanner::computeNewInformation(
    const std::size_t viewpoint_path_index, const ViewpointEntryIndex new_viewpoint_index) const {
  const ViewpointPath& viewpoint_path = viewpoint_paths_[viewpoint_path_index];
  const ViewpointPathComputationData& comp_data = viewpoint_paths_data_[viewpoint_path_index];
  return computeNewInformation(viewpoint_path, comp_data, new_viewpoint_index);
}

ViewpointPlanner::Vector3 ViewpointPlanner::computeInformationVoxelCenter(const ViewpointEntry& viewpoint_entry) const {
  Vector3 voxel_center = Vector3::Zero();
  FloatType total_weight = 0;
  for (const VoxelWithInformation& vi : viewpoint_entry.voxel_set) {
    const FloatType information = vi.information;
    voxel_center += information * vi.voxel->getBoundingBox().getCenter();
    total_weight += information;
  }
  voxel_center /= total_weight;
  return voxel_center;
}

ViewpointPlanner::FloatType ViewpointPlanner::evaluateNovelViewpointInformation(
    const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data,
    const ViewpointEntryIndex viewpoint_index) {
//  const bool verbose = true;
//
//  const ViewpointEntry& viewpoint_entry = viewpoint_entries_[viewpoint_index];
//
//  if (options_.viewpoint_generate_stereo_pairs) {
//    ViewpointEntryIndex stereo_viewpoint_index = (ViewpointEntryIndex)-1;
//    bool stereo_viewpoint_already_in_path = false;
//
//    // Try to find a matching stereo pair. Otherwise discard.
//    std::tie(stereo_viewpoint_already_in_path, stereo_viewpoint_index) = findMatchingStereoViewpointWithoutLock(
//        viewpoint_path, comp_data, viewpoint_index);
//    if (!stereo_viewpoint_already_in_path && stereo_viewpoint_index == (ViewpointEntryIndex)-1) {
//      if (verbose) {
//        std::cout << "Could not find any usable stereo viewpoint for viewpoint " << viewpoint_index << std::endl;
//      }
//      const FloatType new_information = 0;
//      return new_information;
//    }
//    else {
//      const ViewpointEntry& stereo_viewpoint_entry = viewpoint_entries_[stereo_viewpoint_index];
//      const auto overlap_set = bh::computeSetIntersection(viewpoint_entry.voxel_set, stereo_viewpoint_entry.voxel_set);
//      const FloatType new_information = std::accumulate(overlap_set.cbegin(), overlap_set.cend(),
//        FloatType { 0 }, [&](const FloatType& value, const VoxelWithInformation& vi) {
//          const FloatType observation_information = options_.viewpoint_information_factor * vi.information;
//          FloatType novel_information = observation_information;
//          auto it = viewpoint_path.observed_voxel_map.find(vi.voxel);
//          if (it != viewpoint_path.observed_voxel_map.end()) {
//            const WeightType voxel_weight = vi.voxel->getObject()->weight;
//            BH_ASSERT(it->second <= voxel_weight);
//            novel_information = std::min(observation_information, voxel_weight - it->second);
//          }
//          BH_ASSERT(novel_information >= 0);
//          return value + novel_information;
//        });
//      return new_information;
//    }
//  }
//  else {
    const FloatType new_information = computeNewInformation(viewpoint_path, comp_data, viewpoint_index);
    return new_information;
//  }
}
