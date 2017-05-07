//==================================================
// viewpoint_raycast.h.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 27.03.17
//

#pragma once

#include <vector>
#include <utility>
#include "viewpoint_planner_types.h"
#include "occupied_tree.h"
#include "viewpoint.h"

namespace viewpoint_planner {

class ViewpointRaycast {
public:
  ViewpointRaycast(
          OccupiedTreeType *bvh_tree,
          const FloatType min_range = 0,
          const FloatType max_range = std::numeric_limits<FloatType>::max());

#if WITH_CUDA
  void setEnableCuda(const bool enable_cuda);
#endif

  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels with additional info.
  std::vector<OccupiedTreeType::IntersectionResult> getRaycastHitVoxels(
          const Viewpoint &viewpoint, const bool remove_duplicates = true, const bool fail_on_error = true) const;

  /// Perform raycast on the BVH tree on a limited window around the center.
  /// Returns a vector of hit voxels with additional info.
  std::vector<OccupiedTreeType::IntersectionResult> getRaycastHitVoxels(
          const Viewpoint &viewpoint, const size_t width, const size_t height,
          const bool remove_duplicates = true,
          const bool fail_on_error = true) const;

  /// Perform raycast on the BVH tree on a limited window around the center.
  /// Returns a vector of hit voxels with additional info.
  std::vector<OccupiedTreeType::IntersectionResult>
  getRaycastHitVoxels(
          const Viewpoint &viewpoint,
          const size_t x_start, const size_t x_end,
          const size_t y_start, const size_t y_end,
          const bool remove_duplicates = true,
          const bool fail_on_error = true) const;

  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels with additional info.
  std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates>
  getRaycastHitVoxelsWithScreenCoordinates(
          const Viewpoint &viewpoint, const bool remove_duplicates = true, const bool fail_on_error = true) const;

  /// Perform raycast on the BVH tree on a limited window around the center.
  /// Returns a vector of hit voxels with additional info.
  std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates>
  getRaycastHitVoxelsWithScreenCoordinates(
          const Viewpoint &viewpoint, const size_t width, const size_t height,
          const bool remove_duplicates = true,
          const bool fail_on_error = true) const;

  /// Perform raycast on the BVH tree on a limited window around the center.
  /// Returns a vector of hit voxels with additional info.
  std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates>
  getRaycastHitVoxelsWithScreenCoordinates(
          const Viewpoint &viewpoint,
          const size_t x_start, const size_t x_end,
          const size_t y_start, const size_t y_end,
          const bool remove_duplicates = true,
          const bool fail_on_error = true) const;

  /// Remove invalid hit voxels from raycast results
  void removeInvalidRaycastHitVoxels(
          std::vector<OccupiedTreeType::IntersectionResult> *raycast_results) const;

  /// Remove invalid hit voxels from raycast results
  void removeInvalidRaycastHitVoxels(
          std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates> *raycast_results) const;

  /// Remove duplicate hit voxels from raycast results
  void removeDuplicateRaycastHitVoxels(
          std::vector<OccupiedTreeType::IntersectionResult> *raycast_results) const;

  /// Remove duplicate hit voxels from raycast results
  void removeDuplicateRaycastHitVoxels(
          std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates> *raycast_results) const;

#if WITH_CUDA

  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels with additional info.
  std::vector<OccupiedTreeType::IntersectionResult>
  getRaycastHitVoxelsCuda(
          const Viewpoint &viewpoint,
          const size_t x_start, const size_t x_end,
          const size_t y_start, const size_t y_end,
          const bool remove_duplicates = true,
          const bool fail_on_error = true) const;

  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels with additional info.
  std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates>
  getRaycastHitVoxelsWithScreenCoordinatesCuda(
          const Viewpoint &viewpoint,
          const size_t x_start, const size_t x_end,
          const size_t y_start, const size_t y_end,
          const bool remove_duplicates = true,
          const bool fail_on_error = true) const;

#endif

  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels with additional info.
  std::vector<OccupiedTreeType::IntersectionResult>
  getRaycastHitVoxelsCpu(
          const Viewpoint &viewpoint,
          const size_t x_start, const size_t x_end,
          const size_t y_start, const size_t y_end,
          const bool remove_duplicates = true,
          const bool fail_on_error = true) const;

  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels with additional info.
  std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates>
  getRaycastHitVoxelsWithScreenCoordinatesCpu(
          const Viewpoint &viewpoint,
          const size_t x_start, const size_t x_end,
          const size_t y_start, const size_t y_end,
          const bool remove_duplicates = true,
          const bool fail_on_error = true) const;

private:
  OccupiedTreeType *bvh_tree_;
  FloatType min_range_;
  FloatType max_range_;
#if WITH_CUDA
  bool enable_cuda_;
#endif
};

}
