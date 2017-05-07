//==================================================
// viewpoint_planner_raycast.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 5, 2017
//==================================================

#include "viewpoint_planner.h"
#include <bh/opengl/utils.h>

std::unordered_set<size_t> ViewpointPlanner::getVisibleVoxels(const Viewpoint& viewpoint) const {
  ensureOctreeDrawerIsInitialized();
  auto drawing_handle = offscreen_opengl_->beginDrawing();
  const QMatrix4x4 pvm_matrix = offscreen_opengl_->getPvmMatrixFromPose(viewpoint.pose());
  const QMatrix4x4 vm_matrix = offscreen_opengl_->getVmMatrixFromPose(viewpoint.pose());
  octree_drawer_->draw(pvm_matrix, vm_matrix);
  const QImage image_qt = drawing_handle.getImageQt();
  std::unordered_set<size_t> visible_voxels = bh::opengl::getIndicesSetFromImage(image_qt);
  drawing_handle.finish();
  return visible_voxels;
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult>
ViewpointPlanner::getRaycastHitVoxels(
    const Viewpoint& viewpoint, const bool remove_duplicates) const {
  return raycaster_.getRaycastHitVoxels(viewpoint, remove_duplicates);
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult>
ViewpointPlanner::getRaycastHitVoxels(
    const Viewpoint& viewpoint, const std::size_t width, const std::size_t height,
    const bool remove_duplicates) const {
  return raycaster_.getRaycastHitVoxels(viewpoint, width, height, remove_duplicates);
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> ViewpointPlanner::getRaycastHitVoxels(
    const Viewpoint& viewpoint,
    const std::size_t x_start, const std::size_t x_end,
    const std::size_t y_start, const std::size_t y_end,
    const bool remove_duplicates) const {
  return raycaster_.getRaycastHitVoxels(viewpoint, x_start, x_end, y_start, y_end, remove_duplicates);
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates>
ViewpointPlanner::getRaycastHitVoxelsWithScreenCoordinates(
    const Viewpoint& viewpoint, const bool remove_duplicates) const {
  return raycaster_.getRaycastHitVoxelsWithScreenCoordinates(viewpoint, remove_duplicates);
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates>
ViewpointPlanner::getRaycastHitVoxelsWithScreenCoordinates(
    const Viewpoint& viewpoint, const std::size_t width, const std::size_t height,
    const bool remove_duplicates) const {
  return raycaster_.getRaycastHitVoxelsWithScreenCoordinates(
          viewpoint, width, height, remove_duplicates);
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates>
ViewpointPlanner::getRaycastHitVoxelsWithScreenCoordinates(
    const Viewpoint& viewpoint,
    const std::size_t x_start, const std::size_t x_end,
    const std::size_t y_start, const std::size_t y_end,
    const bool remove_duplicates) const {
  return raycaster_.getRaycastHitVoxelsWithScreenCoordinates(
          viewpoint, x_start, x_end, y_start, y_end, remove_duplicates);
}

void ViewpointPlanner::removeInvalidRaycastHitVoxels(
        std::vector<OccupiedTreeType::IntersectionResult>* raycast_results) const {
  raycaster_.removeInvalidRaycastHitVoxels(raycast_results);
}

void ViewpointPlanner::removeInvalidRaycastHitVoxels(
        std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates>* raycast_results) const {
  raycaster_.removeInvalidRaycastHitVoxels(raycast_results);
}

void ViewpointPlanner::removeDuplicateRaycastHitVoxels(
    std::vector<OccupiedTreeType::IntersectionResult>* raycast_results) const {
  raycaster_.removeDuplicateRaycastHitVoxels(raycast_results);
}

void ViewpointPlanner::removeDuplicateRaycastHitVoxels(
    std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates>* raycast_results) const {
  raycaster_.removeDuplicateRaycastHitVoxels(raycast_results);
}

std::pair<ViewpointPlanner::VoxelWithInformationSet, ViewpointPlanner::FloatType>
ViewpointPlanner::getRaycastHitVoxelsWithInformationScore(
    const Viewpoint& viewpoint,
    const bool ignore_voxels_with_zero_information,
    const bool remove_duplicates) const {
  const std::size_t y_start = 0;
  const std::size_t y_end = virtual_camera_.height();
  const std::size_t x_start = 0;
  const std::size_t x_end = virtual_camera_.width();
  return getRaycastHitVoxelsWithInformationScore(
          viewpoint, x_start, x_end, y_start, y_end, ignore_voxels_with_zero_information,
          remove_duplicates);
}

std::pair<ViewpointPlanner::VoxelWithInformationSet, ViewpointPlanner::FloatType>
ViewpointPlanner::getRaycastHitVoxelsWithInformationScore(
    const Viewpoint& viewpoint, const std::size_t width, const std::size_t height,
    const bool ignore_voxels_with_zero_information,
    const bool remove_duplicates) const {
  std::size_t y_start = virtual_camera_.height() / 2 - height / 2;
  std::size_t y_end = virtual_camera_.height() / 2 + height / 2;
  std::size_t x_start = virtual_camera_.width() / 2 - width / 2;
  std::size_t x_end = virtual_camera_.width() / 2 + width / 2;
  return getRaycastHitVoxelsWithInformationScore(
          viewpoint, x_start, x_end, y_start, y_end, ignore_voxels_with_zero_information, remove_duplicates);
}

std::pair<ViewpointPlanner::VoxelWithInformationSet, ViewpointPlanner::FloatType>
ViewpointPlanner::getRaycastHitVoxelsWithInformationScore(
      const Viewpoint& viewpoint,
      const std::size_t x_start, const std::size_t x_end,
      const std::size_t y_start, const std::size_t y_end,
      const bool ignore_voxels_with_zero_information,
      const bool remove_duplicates) const {
  const bool fail_on_error = false;
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates> raycast_results;
  raycast_results = raycaster_.getRaycastHitVoxelsWithScreenCoordinates(
          viewpoint, x_start, x_end, y_start, y_end, remove_duplicates, fail_on_error);
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

std::unordered_set<const ViewpointPlanner::VoxelType*>
ViewpointPlanner::getRaycastHitVoxelsSet(
        const Viewpoint& viewpoint) const {
  const std::size_t y_start = 0;
  const std::size_t y_end = virtual_camera_.height();
  const std::size_t x_start = 0;
  const std::size_t x_end = virtual_camera_.width();
  return getRaycastHitVoxelsSet(
          viewpoint, x_start, x_end, y_start, y_end);
}

std::unordered_set<const ViewpointPlanner::VoxelType*>
ViewpointPlanner::getRaycastHitVoxelsSet(
        const Viewpoint& viewpoint, const std::size_t width, const std::size_t height) const {
  std::size_t y_start = virtual_camera_.height() / 2 - height / 2;
  std::size_t y_end = virtual_camera_.height() / 2 + height / 2;
  std::size_t x_start = virtual_camera_.width() / 2 - width / 2;
  std::size_t x_end = virtual_camera_.width() / 2 + width / 2;
  return getRaycastHitVoxelsSet(
          viewpoint, x_start, x_end, y_start, y_end);
}

std::unordered_set<const ViewpointPlanner::VoxelType*>
ViewpointPlanner::getRaycastHitVoxelsSet(
        const Viewpoint& viewpoint,
        const std::size_t x_start, const std::size_t x_end,
        const std::size_t y_start, const std::size_t y_end) const {
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results;
  raycast_results = raycaster_.getRaycastHitVoxels(
          viewpoint, x_start, x_end, y_start, y_end);
  std::unordered_set<const ViewpointPlanner::VoxelType*> voxel_set;
  for (auto it = raycast_results.cbegin(); it != raycast_results.cend(); ++it) {
    const ViewpointPlannerData::OccupiedTreeType::IntersectionResult& result = *it;
    voxel_set.insert(result.node);
  }
  return voxel_set;
}
