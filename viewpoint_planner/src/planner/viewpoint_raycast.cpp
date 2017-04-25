//==================================================
// viewpoint_raycast.cpp
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 27.03.17
//

#include <unordered_set>
#include "viewpoint_raycast.h"

namespace viewpoint_planner {

ViewpointRaycast::ViewpointRaycast(
        OccupiedTreeType *bvh_tree,
        const FloatType min_range,
        const FloatType max_range)
    : bvh_tree_(bvh_tree), min_range_(min_range), max_range_(max_range) {
#if WITH_CUDA
  enable_cuda_ = false;
#endif
}

void ViewpointRaycast::setEnableCuda(const bool enable_cuda) {
  enable_cuda_ = enable_cuda;
}

std::vector<OccupiedTreeType::IntersectionResult>
ViewpointRaycast::getRaycastHitVoxels(
        const Viewpoint &viewpoint, const bool remove_duplicates) const {
  const std::size_t y_start = 0;
  const std::size_t y_end = viewpoint.camera().height();
  const std::size_t x_start = 0;
  const std::size_t x_end = viewpoint.camera().width();
  return getRaycastHitVoxels(viewpoint, x_start, x_end, y_start, y_end, remove_duplicates);
}

std::vector<OccupiedTreeType::IntersectionResult>
ViewpointRaycast::getRaycastHitVoxels(
        const Viewpoint &viewpoint, const std::size_t width, const std::size_t height,
        const bool remove_duplicates) const {
  const std::size_t y_start = viewpoint.camera().height() / 2 - height / 2;
  const std::size_t y_end = viewpoint.camera().height() / 2 + height / 2;
  const std::size_t x_start = viewpoint.camera().width() / 2 - width / 2;
  const std::size_t x_end = viewpoint.camera().width() / 2 + width / 2;
  return getRaycastHitVoxels(viewpoint, x_start, x_end, y_start, y_end, remove_duplicates);
}

std::vector<OccupiedTreeType::IntersectionResult> ViewpointRaycast::getRaycastHitVoxels(
        const Viewpoint &viewpoint,
        const std::size_t x_start, const std::size_t x_end,
        const std::size_t y_start, const std::size_t y_end,
        const bool remove_duplicates) const {
#if WITH_CUDA
  if (enable_cuda_) {
    return getRaycastHitVoxelsCuda(viewpoint, x_start, x_end, y_start, y_end, remove_duplicates);
  }
  else {
    return getRaycastHitVoxelsCpu(viewpoint, x_start, x_end, y_start, y_end, remove_duplicates);
  }
#else
  return getRaycastHitVoxelsCpu(viewpoint, x_start, x_end, y_start, y_end, remove_duplicates);
#endif
}

std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates>
ViewpointRaycast::getRaycastHitVoxelsWithScreenCoordinates(
        const Viewpoint &viewpoint, const bool remove_duplicates) const {
  const std::size_t y_start = 0;
  const std::size_t y_end = viewpoint.camera().height();
  const std::size_t x_start = 0;
  const std::size_t x_end = viewpoint.camera().width();
  return getRaycastHitVoxelsWithScreenCoordinates(viewpoint, x_start, x_end, y_start, y_end, remove_duplicates);
}

std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates>
ViewpointRaycast::getRaycastHitVoxelsWithScreenCoordinates(
        const Viewpoint &viewpoint, const std::size_t width, const std::size_t height,
        const bool remove_duplicates) const {
  const std::size_t y_start = viewpoint.camera().height() / 2 - height / 2;
  const std::size_t y_end = viewpoint.camera().height() / 2 + height / 2;
  const std::size_t x_start = viewpoint.camera().width() / 2 - width / 2;
  const std::size_t x_end = viewpoint.camera().width() / 2 + width / 2;
  return getRaycastHitVoxelsWithScreenCoordinates(viewpoint, x_start, x_end, y_start, y_end, remove_duplicates);
}

std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates>
ViewpointRaycast::getRaycastHitVoxelsWithScreenCoordinates(
        const Viewpoint &viewpoint,
        const std::size_t x_start, const std::size_t x_end,
        const std::size_t y_start, const std::size_t y_end,
        const bool remove_duplicates) const {
#if WITH_CUDA
  if (enable_cuda_) {
    return getRaycastHitVoxelsWithScreenCoordinatesCuda(viewpoint, x_start, x_end, y_start, y_end, remove_duplicates);
  }
  else {
    return getRaycastHitVoxelsWithScreenCoordinatesCpu(viewpoint, x_start, x_end, y_start, y_end, remove_duplicates);
  }
#else
  return getRaycastHitVoxelsWithScreenCoordinatesCpu(viewpoint, x_start, x_end, y_start, y_end, remove_duplicates);
#endif
}

std::vector<OccupiedTreeType::IntersectionResult> ViewpointRaycast::getRaycastHitVoxelsCpu(
        const Viewpoint &viewpoint,
        const std::size_t x_start, const std::size_t x_end,
        const std::size_t y_start, const std::size_t y_end,
        const bool remove_duplicates) const {
  const std::size_t width = x_end - x_start;
  const std::size_t height = y_end - y_start;
  std::vector<OccupiedTreeType::IntersectionResult> raycast_results;
  raycast_results.resize(width * height);
  ait::Timer timer;
  for (size_t y = y_start; y < y_end; ++y) {
//  for (size_t y = viewpoint.camera().height()/2-10; y < viewpoint.camera().height()/2+10; ++y) {
//  size_t y = viewpoint.camera().height() / 2; {
//  size_t y = 85; {
//#if !AIT_DEBUG
//#pragma omp parallel for
//#endif
    for (size_t x = x_start; x < x_end; ++x) {
//    for (size_t x = viewpoint.camera().width()/2-10; x < viewpoint.camera().width()/2+10; ++x) {
//    size_t x = viewpoint.camera().width() / 2; {
//      size_t x = 101; {
      const RayType ray = viewpoint.getCameraRay(x, y);
      std::pair<bool, OccupiedTreeType::IntersectionResult> result =
              bvh_tree_->intersects(ray, min_range_, max_range_);
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
  if (remove_duplicates) {
    removeDuplicateRaycastHitVoxels(&raycast_results);
  }
  else {
    removeInvalidRaycastHitVoxels(&raycast_results);
  }
//  std::cout << "Voxels: " << raycast_results.size() << std::endl;
  timer.printTiming("getRaycastHitVoxels");
  return raycast_results;
}

std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates>
ViewpointRaycast::getRaycastHitVoxelsWithScreenCoordinatesCpu(
        const Viewpoint &viewpoint,
        const std::size_t x_start, const std::size_t x_end,
        const std::size_t y_start, const std::size_t y_end,
        const bool remove_duplicates) const {
  const std::size_t width = x_end - x_start;
  const std::size_t height = y_end - y_start;
  std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates> raycast_results;
  raycast_results.resize(width * height);
  ait::Timer timer;
  for (size_t y = y_start; y < y_end; ++y) {
//  for (size_t y = viewpoint.camera().height()/2-10; y < viewpoint.camera().height()/2+10; ++y) {
//  size_t y = viewpoint.camera().height() / 2; {
//  size_t y = 85; {
//#if !AIT_DEBUG
//#pragma omp parallel for
//#endif
    for (size_t x = x_start; x < x_end; ++x) {
//    for (size_t x = viewpoint.camera().width()/2-10; x < viewpoint.camera().width()/2+10; ++x) {
//    size_t x = viewpoint.camera().width() / 2; {
//      size_t x = 101; {
      const RayType ray = viewpoint.getCameraRay(x, y);
      std::pair<bool, OccupiedTreeType::IntersectionResult> result =
              bvh_tree_->intersects(ray, min_range_, max_range_);
      if (result.first) {
//        if (result.second.depth < octree_->getTreeDepth()) {
//          continue;
//        }
//#if !AIT_DEBUG
//#pragma omp critical
//#endif
        {
//          if (octree_->isNodeUnknown(result.second.node->getObject()->observation_count)) {
          OccupiedTreeType::IntersectionResultWithScreenCoordinates result_with_screen_coordinates;
          result_with_screen_coordinates.intersection_result = result.second;
          result_with_screen_coordinates.screen_coordinates = Vector2(x, y);
          raycast_results[(y - y_start) * width + (x - x_start)] = result_with_screen_coordinates;
//          }
        }
      }
    }
  }
  if (remove_duplicates) {
    removeDuplicateRaycastHitVoxels(&raycast_results);
  }
  else {
    removeInvalidRaycastHitVoxels(&raycast_results);
  }
//  std::cout << "Voxels: " << raycast_results.size() << std::endl;
  timer.printTiming("getRaycastHitVoxels");
  return raycast_results;
}

#if WITH_CUDA

std::vector<OccupiedTreeType::IntersectionResult>
ViewpointRaycast::getRaycastHitVoxelsCuda(
        const Viewpoint &viewpoint,
        const std::size_t x_start, const std::size_t x_end,
        const std::size_t y_start, const std::size_t y_end,
        const bool remove_duplicates) const {
  ait::Timer timer;

  // Perform raycast
  using ResultType = OccupiedTreeType::IntersectionResult;
  std::vector<ResultType> raycast_results =
          bvh_tree_->raycastCuda(
                  viewpoint.camera().intrinsics(),
                  viewpoint.pose().getTransformationImageToWorld(),
                  x_start, x_end,
                  y_start, y_end,
                  min_range_, max_range_);
  if (remove_duplicates) {
    removeDuplicateRaycastHitVoxels(&raycast_results);
  }
  else {
    removeInvalidRaycastHitVoxels(&raycast_results);
  }

//  std::cout << "Voxels: " << raycast_results.size() << std::endl;
  timer.printTiming("getRaycastHitVoxelsCuda");
  return raycast_results;
}

std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates>
ViewpointRaycast::getRaycastHitVoxelsWithScreenCoordinatesCuda(
        const Viewpoint &viewpoint,
        const std::size_t x_start, const std::size_t x_end,
        const std::size_t y_start, const std::size_t y_end,
        const bool remove_duplicates) const {
  ait::Timer timer;

  // Perform raycast
  const bool fail_on_error = true;
  using ResultType = OccupiedTreeType::IntersectionResultWithScreenCoordinates;
  std::vector<ResultType> raycast_results =
          bvh_tree_->raycastWithScreenCoordinatesCuda(
                  viewpoint.camera().intrinsics(),
                  viewpoint.pose().getTransformationImageToWorld(),
                  x_start, x_end,
                  y_start, y_end,
                  min_range_, max_range_,
                  fail_on_error);
  if (remove_duplicates) {
    removeDuplicateRaycastHitVoxels(&raycast_results);
  }
  else {
    removeInvalidRaycastHitVoxels(&raycast_results);
  }

//  std::cout << "Voxels: " << raycast_results.size() << std::endl;
  timer.printTiming("getRaycastHitVoxelsCuda");
  return raycast_results;
}

#endif

void ViewpointRaycast::removeInvalidRaycastHitVoxels(
        std::vector<OccupiedTreeType::IntersectionResult> *raycast_results) const {
  using RaycastResult = OccupiedTreeType::IntersectionResult;
  raycast_results->erase(std::remove_if(
          raycast_results->begin(),
          raycast_results->end(),
          [](const RaycastResult& ir) { return ir.node == nullptr; }));
}

void ViewpointRaycast::removeInvalidRaycastHitVoxels(
        std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates> *raycast_results) const {
  using RaycastResult = OccupiedTreeType::IntersectionResultWithScreenCoordinates;
  raycast_results->erase(std::remove_if(
          raycast_results->begin(),
          raycast_results->end(),
          [](const RaycastResult& ir) { return ir.intersection_result.node == nullptr; }));
}

void ViewpointRaycast::removeDuplicateRaycastHitVoxels(
        std::vector<OccupiedTreeType::IntersectionResult> *raycast_results) const {
  using RaycastResult = OccupiedTreeType::IntersectionResult;
  std::unordered_map<OccupiedTreeType::NodeType *, RaycastResult> raycast_map;
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

void ViewpointRaycast::removeDuplicateRaycastHitVoxels(
        std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates> *raycast_results) const {
  using RaycastResult = OccupiedTreeType::IntersectionResultWithScreenCoordinates;
  std::unordered_map<OccupiedTreeType::NodeType *, RaycastResult> raycast_map;
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

}
