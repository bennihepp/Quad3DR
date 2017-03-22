//==================================================
// viewpoint_planner_raycast.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 5, 2017
//==================================================

#include "viewpoint_planner.h"

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult>
ViewpointPlanner::getRaycastHitVoxels(
    const Viewpoint& viewpoint) const {
  const std::size_t y_start = 0;
  const std::size_t y_end = virtual_camera_.height();
  const std::size_t x_start = 0;
  const std::size_t x_end = virtual_camera_.width();
  return getRaycastHitVoxels(viewpoint, x_start, x_end, y_start, y_end);
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult>
ViewpointPlanner::getRaycastHitVoxels(
    const Viewpoint& viewpoint, const std::size_t width, const std::size_t height) const {
  const std::size_t y_start = virtual_camera_.height() / 2 - height / 2;
  const std::size_t y_end = virtual_camera_.height() / 2 + height / 2;
  const std::size_t x_start = virtual_camera_.width() / 2 - width / 2;
  const std::size_t x_end = virtual_camera_.width() / 2 + width / 2;
  return getRaycastHitVoxels(viewpoint, x_start, x_end, y_start, y_end);
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> ViewpointPlanner::getRaycastHitVoxels(
    const Viewpoint& viewpoint,
    const std::size_t x_start, const std::size_t x_end,
    const std::size_t y_start, const std::size_t y_end) const {
  const std::size_t width = x_end - x_start;
  const std::size_t height = y_end - y_start;
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results;
  raycast_results.resize(width * height);
  ait::Timer timer;
  for (size_t y = y_start; y < y_end; ++y) {
//  for (size_t y = virtual_camera_.height()/2-10; y < virtual_camera_.height()/2+10; ++y) {
//  size_t y = virtual_camera_.height() / 2; {
//  size_t y = 85; {
//#if !AIT_DEBUG
//#pragma omp parallel for
//#endif
    for (size_t x = x_start; x < x_end; ++x) {
//    for (size_t x = virtual_camera_.width()/2-10; x < virtual_camera_.width()/2+10; ++x) {
//    size_t x = virtual_camera_.width() / 2; {
//      size_t x = 101; {
      const RayType ray = viewpoint.getCameraRay(x, y);
      const FloatType min_range = options_.raycast_min_range;
      const FloatType max_range = options_.raycast_max_range;
      std::pair<bool, ViewpointPlannerData::OccupiedTreeType::IntersectionResult> result =
          data_->occupied_bvh_.intersects(ray, min_range, max_range);
      if (result.first) {
//        if (result.second.depth < octree_->getTreeDepth()) {
//          continue;
//        }
//#if !AIT_DEBUG
//#pragma omp critical
//#endif
        {
//          if (octree_->isNodeUnknown(result.second.node->getObject()->observation_count)) {
          raycast_results[(y - y_start) * width + (x - x_start)] = result.second;
//          }
        }
      }
    }
  }
  removeDuplicateRaycastHitVoxels(&raycast_results);
//  std::cout << "Voxels: " << raycast_results.size() << std::endl;
  timer.printTiming("getRaycastHitVoxels");
  return raycast_results;
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates>
ViewpointPlanner::getRaycastHitVoxelsWithScreenCoordinates(
    const Viewpoint& viewpoint) const {
  const std::size_t y_start = 0;
  const std::size_t y_end = virtual_camera_.height();
  const std::size_t x_start = 0;
  const std::size_t x_end = virtual_camera_.width();
  return getRaycastHitVoxelsWithScreenCoordinates(viewpoint, x_start, x_end, y_start, y_end);
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates>
ViewpointPlanner::getRaycastHitVoxelsWithScreenCoordinates(
    const Viewpoint& viewpoint, const std::size_t width, const std::size_t height) const {
  const std::size_t y_start = virtual_camera_.height() / 2 - height / 2;
  const std::size_t y_end = virtual_camera_.height() / 2 + height / 2;
  const std::size_t x_start = virtual_camera_.width() / 2 - width / 2;
  const std::size_t x_end = virtual_camera_.width() / 2 + width / 2;
  return getRaycastHitVoxelsWithScreenCoordinates(viewpoint, x_start, x_end, y_start, y_end);
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates>
ViewpointPlanner::getRaycastHitVoxelsWithScreenCoordinates(
    const Viewpoint& viewpoint,
    const std::size_t x_start, const std::size_t x_end,
    const std::size_t y_start, const std::size_t y_end) const {
  const std::size_t width = x_end - x_start;
  const std::size_t height = y_end - y_start;
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates> raycast_results;
  raycast_results.resize(width * height);
  ait::Timer timer;
  for (size_t y = y_start; y < y_end; ++y) {
//  for (size_t y = virtual_camera_.height()/2-10; y < virtual_camera_.height()/2+10; ++y) {
//  size_t y = virtual_camera_.height() / 2; {
//  size_t y = 85; {
//#if !AIT_DEBUG
//#pragma omp parallel for
//#endif
    for (size_t x = x_start; x < x_end; ++x) {
//    for (size_t x = virtual_camera_.width()/2-10; x < virtual_camera_.width()/2+10; ++x) {
//    size_t x = virtual_camera_.width() / 2; {
//      size_t x = 101; {
      const RayType ray = viewpoint.getCameraRay(x, y);
      const FloatType min_range = options_.raycast_min_range;
      const FloatType max_range = options_.raycast_max_range;
      std::pair<bool, ViewpointPlannerData::OccupiedTreeType::IntersectionResult> result =
          data_->occupied_bvh_.intersects(ray, min_range, max_range);
      if (result.first) {
//        if (result.second.depth < octree_->getTreeDepth()) {
//          continue;
//        }
//#if !AIT_DEBUG
//#pragma omp critical
//#endif
        {
//          if (octree_->isNodeUnknown(result.second.node->getObject()->observation_count)) {
          ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates result_with_screen_coordinates;
          result_with_screen_coordinates.intersection_result = result.second;
          result_with_screen_coordinates.screen_coordinates = Vector2(x, y);
          raycast_results[(y - y_start) * width + (x - x_start)] = result_with_screen_coordinates;
//          }
        }
      }
    }
  }
  removeDuplicateRaycastHitVoxels(&raycast_results);
//  std::cout << "Voxels: " << raycast_results.size() << std::endl;
  timer.printTiming("getRaycastHitVoxels");
  return raycast_results;
}

#if WITH_CUDA

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> ViewpointPlanner::getRaycastHitVoxelsCuda(
    const Viewpoint& viewpoint) const {
  const std::size_t y_start = 0;
  const std::size_t y_end = virtual_camera_.height();
  const std::size_t x_start = 0;
  const std::size_t x_end = virtual_camera_.width();
  return getRaycastHitVoxelsCuda(viewpoint, x_start, x_end, y_start, y_end);
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> ViewpointPlanner::getRaycastHitVoxelsCuda(
    const Viewpoint& viewpoint, const std::size_t width, const std::size_t height) const {
  const std::size_t y_start = virtual_camera_.height() / 2 - height / 2;
  const std::size_t y_end = virtual_camera_.height() / 2 + height / 2;
  const std::size_t x_start = virtual_camera_.width() / 2 - width / 2;
  const std::size_t x_end = virtual_camera_.width() / 2 + width / 2;
  return getRaycastHitVoxelsCuda(viewpoint, x_start, x_end, y_start, y_end);
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult>
ViewpointPlanner::getRaycastHitVoxelsCuda(
    const Viewpoint& viewpoint,
    const std::size_t x_start, const std::size_t x_end,
    const std::size_t y_start, const std::size_t y_end) const {
  ait::Timer timer;

  // Perform raycast
  const FloatType min_range = options_.raycast_min_range;
  const FloatType max_range = options_.raycast_max_range;
  using ResultType = ViewpointPlannerData::OccupiedTreeType::IntersectionResult;
  std::vector<ResultType> raycast_results =
      data_->occupied_bvh_.raycastCuda(
          virtual_camera_.intrinsics(),
          viewpoint.pose().getTransformationImageToWorld(),
          x_start, x_end,
          y_start, y_end,
          min_range, max_range);
  removeDuplicateRaycastHitVoxels(&raycast_results);

//  std::cout << "Voxels: " << raycast_results.size() << std::endl;
  timer.printTiming("getRaycastHitVoxelsCuda");
  return raycast_results;
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates>
ViewpointPlanner::getRaycastHitVoxelsWithScreenCoordinatesCuda(
    const Viewpoint& viewpoint) const {
  const std::size_t y_start = 0;
  const std::size_t y_end = virtual_camera_.height();
  const std::size_t x_start = 0;
  const std::size_t x_end = virtual_camera_.width();
  return getRaycastHitVoxelsWithScreenCoordinatesCuda(viewpoint, x_start, x_end, y_start, y_end);
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates>
ViewpointPlanner::getRaycastHitVoxelsWithScreenCoordinatesCuda(
    const Viewpoint& viewpoint, const std::size_t width, const std::size_t height) const {
  const std::size_t y_start = virtual_camera_.height() / 2 - height / 2;
  const std::size_t y_end = virtual_camera_.height() / 2 + height / 2;
  const std::size_t x_start = virtual_camera_.width() / 2 - width / 2;
  const std::size_t x_end = virtual_camera_.width() / 2 + width / 2;
  return getRaycastHitVoxelsWithScreenCoordinatesCuda(viewpoint, x_start, x_end, y_start, y_end);
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates>
ViewpointPlanner::getRaycastHitVoxelsWithScreenCoordinatesCuda(
    const Viewpoint& viewpoint,
    const std::size_t x_start, const std::size_t x_end,
    const std::size_t y_start, const std::size_t y_end) const {
  ait::Timer timer;

  // Perform raycast
  const FloatType min_range = options_.raycast_min_range;
  const FloatType max_range = options_.raycast_max_range;
  using ResultType = ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates;
  std::vector<ResultType> raycast_results =
      data_->occupied_bvh_.raycastWithScreenCoordinatesCuda(
          virtual_camera_.intrinsics(),
          viewpoint.pose().getTransformationImageToWorld(),
          x_start, x_end,
          y_start, y_end,
          min_range, max_range);
  removeDuplicateRaycastHitVoxels(&raycast_results);

//  std::cout << "Voxels: " << raycast_results.size() << std::endl;
  timer.printTiming("getRaycastHitVoxelsCuda");
  return raycast_results;
}

#endif

void ViewpointPlanner::removeDuplicateRaycastHitVoxels(
    std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult>* raycast_results) const {
  using RaycastResult = ViewpointPlannerData::OccupiedTreeType::IntersectionResult;
  std::unordered_map<ViewpointPlannerData::OccupiedTreeType::NodeType*, RaycastResult> raycast_map;
  for (auto it = raycast_results->cbegin(); it != raycast_results->cend(); ++it) {
    if (it->node != nullptr) {
      raycast_map.emplace(it->node, *it);
    }
  }
  raycast_results->clear();
  raycast_results->reserve(raycast_map.size());
  for (auto it = raycast_map.cbegin(); it != raycast_map.cend(); ++it) {
    raycast_results->emplace_back(it->second);
  }
}

void ViewpointPlanner::removeDuplicateRaycastHitVoxels(
    std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates>* raycast_results) const {
  using RaycastResult = ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates;
  std::unordered_map<ViewpointPlannerData::OccupiedTreeType::NodeType*, RaycastResult> raycast_map;
  for (auto it = raycast_results->cbegin(); it != raycast_results->cend(); ++it) {
    if (it->intersection_result.node != nullptr) {
      raycast_map.emplace(it->intersection_result.node, *it);
    }
  }
  raycast_results->clear();
  raycast_results->reserve(raycast_map.size());
  for (auto it = raycast_map.cbegin(); it != raycast_map.cend(); ++it) {
    raycast_results->emplace_back(it->second);
  }
}

std::pair<ViewpointPlanner::VoxelWithInformationSet, ViewpointPlanner::FloatType>
ViewpointPlanner::getRaycastHitVoxelsWithInformationScore(
    const Viewpoint& viewpoint,
    const bool ignore_voxels_with_zero_information) const {
  const std::size_t y_start = 0;
  const std::size_t y_end = virtual_camera_.height();
  const std::size_t x_start = 0;
  const std::size_t x_end = virtual_camera_.width();
  return getRaycastHitVoxelsWithInformationScore(
          viewpoint, x_start, x_end, y_start, y_end, ignore_voxels_with_zero_information);
}

std::pair<ViewpointPlanner::VoxelWithInformationSet, ViewpointPlanner::FloatType>
ViewpointPlanner::getRaycastHitVoxelsWithInformationScore(
    const Viewpoint& viewpoint, const std::size_t width, const std::size_t height,
    const bool ignore_voxels_with_zero_information) const {
  std::size_t y_start = virtual_camera_.height() / 2 - height / 2;
  std::size_t y_end = virtual_camera_.height() / 2 + height / 2;
  std::size_t x_start = virtual_camera_.width() / 2 - width / 2;
  std::size_t x_end = virtual_camera_.width() / 2 + width / 2;
  return getRaycastHitVoxelsWithInformationScore(
          viewpoint, x_start, x_end, y_start, y_end, ignore_voxels_with_zero_information);
}

std::pair<ViewpointPlanner::VoxelWithInformationSet, ViewpointPlanner::FloatType>
ViewpointPlanner::getRaycastHitVoxelsWithInformationScore(
      const Viewpoint& viewpoint,
      const std::size_t x_start, const std::size_t x_end,
      const std::size_t y_start, const std::size_t y_end,
      const bool ignore_voxels_with_zero_information) const {
#if WITH_CUDA
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates> raycast_results;
  if (options_.enable_cuda) {
   raycast_results = getRaycastHitVoxelsWithScreenCoordinatesCuda(
           viewpoint, x_start, x_end, y_start, y_end);
  }
  else {
    raycast_results = getRaycastHitVoxelsWithScreenCoordinates(
            viewpoint, x_start, x_end, y_start, y_end);
  }
#else
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates> raycast_results =
        getRaycastHitVoxels(viewpoint);
#endif
  VoxelWithInformationSet voxel_set;
  FloatType total_information = 0;
//  std::vector<FloatType> informations;
  for (auto it = raycast_results.cbegin(); it != raycast_results.cend(); ++it) {
    const ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates& result = *it;
    WeightType information = computeViewpointObservationScore(
        viewpoint, result.intersection_result.node, result.screen_coordinates);
    if (!ignore_voxels_with_zero_information || information > 0) {
      voxel_set.insert(VoxelWithInformation(result.intersection_result.node, information));
      total_information += information;
//      informations.push_back(information);
    }
  }
//  if (!informations.empty()) {
//    auto result = ait::computeHistogram(informations, 10);
//    std::cout << "Histogram: " << std::endl;
//    for (size_t i = 0; i < result.first.size(); ++i) {
//      std::cout << "  # <= " << result.second[i] << ": " << result.first[i] << std::endl;
//    }
//  }
  return std::make_pair(voxel_set, total_information);
}
