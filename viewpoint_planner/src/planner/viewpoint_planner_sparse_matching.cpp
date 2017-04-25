//==================================================
// viewpoint_planner_sparse.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 9, 2017
//==================================================

#include "viewpoint_planner.h"
#include <bh/algorithm.h>
#include <bh/opengl/utils.h>

bool ViewpointPlanner::isWithinSparseMatchingLimits(const Pose& pose1, const Pose& pose2) const {
  // Consider sparse matching limits (i.e. maximum translation and angular deviation)
  const FloatType translation_distance = pose1.getPositionDistanceTo(pose2);
  if (translation_distance > options_.sparse_matching_max_distance) {
    return false;
  }
  const FloatType angular_distance = pose1.getAngularDistanceTo(pose2);
  if (angular_distance > sparse_matching_max_angular_distance_) {
    return false;
  }
  return true;
}

bool ViewpointPlanner::isWithinSparseMatchingLimits(const Viewpoint& viewpoint1, const Viewpoint& viewpoint2) const {
  return isWithinSparseMatchingLimits(viewpoint1.pose(), viewpoint2.pose());
}

bool ViewpointPlanner::isSparseMatchable2(
        const Viewpoint& viewpoint1,
        const Viewpoint& viewpoint2,
        const std::unordered_set<size_t>& visible_voxels1,
        const std::unordered_set<size_t>& visible_voxels2,
        const FloatType iou_threshold) const {
  const bool verbose = false;

  const std::unordered_set<size_t> intersection_set
          = bh::computeSetIntersection(visible_voxels1, visible_voxels2);
  const std::unordered_set<size_t> union_set
          = bh::computeSetUnion(visible_voxels1, visible_voxels2);
  if (intersection_set.size() == 0) {
    return false;
  }
  const FloatType iou = intersection_set.size() / (FloatType)union_set.size();

  if (verbose) {
    BH_PRINT_VALUE(visible_voxels1.size());
    BH_PRINT_VALUE(visible_voxels2.size());
    BH_PRINT_VALUE(union_set.size());
    BH_PRINT_VALUE(intersection_set.size());
    BH_PRINT_VALUE(iou);
    BH_ASSERT(union_set.size() >= visible_voxels1.size());
    BH_ASSERT(union_set.size() >= visible_voxels2.size());
    BH_ASSERT(intersection_set.size() <= visible_voxels1.size());
    BH_ASSERT(intersection_set.size() <= visible_voxels2.size());
  }


  if (options_.sparse_matching_dump_voxel_images) {
    ensureOctreeDrawerIsInitialized();

    auto drawing_handle = offscreen_opengl_->beginDrawing();
    const QMatrix4x4 pvm_matrix1 = offscreen_opengl_->getPvmMatrixFromPose(viewpoint1.pose());
    const QMatrix4x4 vm_matrix1 = offscreen_opengl_->getVmMatrixFromPose(viewpoint1.pose());
    octree_drawer_->draw(pvm_matrix1, vm_matrix1);
    const QImage image_qt1 = drawing_handle.getImageQt();
    image_qt1.save("dump/octree_dump_1.png");
    drawing_handle->clearWithoutLock();
    const QMatrix4x4 pvm_matrix2 = offscreen_opengl_->getPvmMatrixFromPose(viewpoint2.pose());
    const QMatrix4x4 vm_matrix2 = offscreen_opengl_->getVmMatrixFromPose(viewpoint2.pose());
    octree_drawer_->draw(pvm_matrix2, vm_matrix2);
    const QImage image_qt2 = drawing_handle.getImageQt();
    image_qt2.save("dump/octree_dump_2.png");
    drawing_handle.finish();

    QImage intersection_image1 = image_qt1.copy();
    for (int y = 0; y < intersection_image1.height(); ++y) {
      for (int x = 0; x < intersection_image1.width(); ++x) {
        const QColor color_qt(intersection_image1.pixel(x, y));
        const bh::Color4<float> color = bh::qt::colorFromQt<float>(color_qt);
        const size_t index = bh::opengl::colorToIndex(color);
        if (visible_voxels2.count(index) == 0) {
          intersection_image1.setPixel(x, y, Qt::white);
        }
      }
    }
    intersection_image1.save("dump/octree_intersection_dump_1.png");
    QImage intersection_image2 = image_qt2.copy();
    for (int y = 0; y < intersection_image2.height(); ++y) {
      for (int x = 0; x < intersection_image2.width(); ++x) {
        const QColor color_qt(intersection_image2.pixel(x, y));
        const bh::Color4<float> color = bh::qt::colorFromQt<float>(color_qt);
        const size_t index = bh::opengl::colorToIndex(color);
        if (visible_voxels1.count(index) == 0) {
          intersection_image2.setPixel(x, y, Qt::white);
        }
      }
    }
    intersection_image2.save("dump/octree_intersection_dump_2.png");
  }

//  const QImage mesh_image1 = offscreen_renderer_->drawPoissonMesh(pose1);
//  const QImage mesh_image2 = offscreen_renderer_->drawPoissonMesh(pose2);
//  mesh_image1.save("dump/sparse_matching_dump1.png");
//  mesh_image2.save("dump/sparse_matching_dump2.png");
//  const QImage mesh_indices_image1 = offscreen_renderer_->drawPoissonMeshIndices(pose1);
//  const QImage mesh_indices_image2 = offscreen_renderer_->drawPoissonMeshIndices(pose2);
//  mesh_indices_image1.save("dump/sparse_matching_indices_dump1.png");
//  mesh_indices_image2.save("dump/sparse_matching_indices_dump2.png");
//  const std::unordered_set<size_t> visible_indices1 = offscreen_renderer_->getVisibleTriangles(mesh_indices_image1);
//  const std::unordered_set<size_t> visible_indices2 = offscreen_renderer_->getVisibleTriangles(mesh_indices_image2);
//  const std::unordered_set<size_t> visible_indices_intersection
//          = bh::computeSetIntersection(visible_indices1, visible_indices2);
//  const std::unordered_set<size_t> visible_indices_union
//          = bh::computeSetUnion(visible_indices1, visible_indices2);
//  if (visible_indices_union.size() == 0) {
//    return false;
//  }
//  const FloatType iou = visible_indices_intersection.size() / (FloatType)visible_indices_union.size();
//  BH_PRINT_VALUE(visible_indices1.size());
//  BH_PRINT_VALUE(visible_indices2.size());
//  BH_PRINT_VALUE(visible_indices_intersection.size());
//  BH_PRINT_VALUE(visible_indices_union.size());
//  BH_PRINT_VALUE(iou);
//  BH_ASSERT(visible_indices_union.size() >= visible_indices1.size());
//  BH_ASSERT(visible_indices_union.size() >= visible_indices2.size());
//  BH_ASSERT(visible_indices_intersection.size() <= visible_indices1.size());
//  BH_ASSERT(visible_indices_intersection.size() <= visible_indices2.size());
  const bool iou_within_limits = iou >= iou_threshold;
  return iou_within_limits;
}

bool ViewpointPlanner::isSparseMatchable2(
        const ViewpointEntryIndex& viewpoint_index1,
        const ViewpointEntryIndex& viewpoint_index2,
        const FloatType iou_threshold) const {
  const Viewpoint& viewpoint1 = viewpoint_entries_[viewpoint_index1].viewpoint;
  const Viewpoint& viewpoint2 = viewpoint_entries_[viewpoint_index2].viewpoint;
  const std::unordered_set<size_t>& visible_voxels1 = getCachedVisibleVoxels(viewpoint_index1);
  const std::unordered_set<size_t>& visible_voxels2 = getCachedVisibleVoxels(viewpoint_index2);
  return isSparseMatchable2(viewpoint1, viewpoint2, visible_voxels1, visible_voxels2, iou_threshold);
}

bool ViewpointPlanner::isSparseMatchable2(
        const Viewpoint& viewpoint1, const Viewpoint& viewpoint2,
        const FloatType iou_threshold) const {
//  const std::unordered_set<const VoxelType*> raycast_voxels1 = getRaycastHitVoxelsSet(viewpoint1);
//  const std::unordered_set<const VoxelType*> raycast_voxels2 = getRaycastHitVoxelsSet(viewpoint2);
  const std::unordered_set<size_t> visible_voxels1 = getVisibleVoxels(viewpoint1);
  const std::unordered_set<size_t> visible_voxels2 = getVisibleVoxels(viewpoint2);
  return isSparseMatchable2(viewpoint1, viewpoint2, visible_voxels1, visible_voxels2, iou_threshold);
}

bool ViewpointPlanner::isSparseMatchable2(
        const Viewpoint& viewpoint1,
        const Viewpoint& viewpoint2,
        const std::unordered_set<size_t>& visible_voxels1,
        const std::unordered_set<size_t>& visible_voxels2) const {
  return isSparseMatchable2(viewpoint1, viewpoint2,
                            visible_voxels1, visible_voxels2,
                            options_.sparse_matching_voxels_iou_threshold);
}

bool ViewpointPlanner::isSparseMatchable2(
        const ViewpointEntryIndex& viewpoint_index1,
        const ViewpointEntryIndex& viewpoint_index2) const {
  return isSparseMatchable2(viewpoint_index1, viewpoint_index2, options_.sparse_matching_voxels_iou_threshold);
}

bool ViewpointPlanner::isSparseMatchable2(const Viewpoint& viewpoint1, const Viewpoint& viewpoint2) const {
  return isSparseMatchable2(viewpoint1, viewpoint2, options_.sparse_matching_voxels_iou_threshold);
}

bool ViewpointPlanner::isSparsePointVisible(const Viewpoint& viewpoint, const Point3D& point3d) const {
  const Vector3 point3d_camera = viewpoint.projectWorldPointIntoCamera(point3d.getPosition());
  const Eigen::Vector2i point2d = viewpoint.camera().projectPoint(point3d_camera).cast<int>();
  const bool behind_camera = point3d_camera(2) < 0;
  if (behind_camera) {
    return false;
  }
  const bool projects_into_image = viewpoint.camera().isPointInViewport(point2d);
  if (!projects_into_image) {
    return false;
  }
  const FloatType point_depth = point3d_camera(2);
  BH_ASSERT(point_depth >= 0);
  const FloatType view_depth = computePoissonMeshDepth(viewpoint, point2d(0), point2d(1));
  const bool occluded = point_depth > view_depth + options_.sparse_matching_depth_tolerance;
  if (occluded) {
    return false;
  }
//  const Point3DStatistics& statistics = point3d.getStatistics();
//  FloatType distance;
//  Vector3 direction;
//  std::tie(distance, direction) = bh::computeDistanceAndDirection(point3d.getPosition(), viewpoint.pose().getWorldPosition());
//  const FloatType dist_deviation = std::abs(distance - statistics.averageDistance());
//  if (dist_deviation > options_.sparse_matching_max_distance_deviation_factor * statistics.stddevDistance()) {
//    return false;
//  }
//  const FloatType one_minus_dot_product = 1 - direction.dot(point3d.getNormal());
//  const FloatType one_minus_dot_deviation = std::abs(one_minus_dot_product);
//  if (one_minus_dot_deviation > options_.sparse_matching_max_normal_deviation_factor * statistics.stddevOneMinusDotProduct()) {
//    return false;
//  }
  return true;
}

//bool ViewpointPlanner::isSparsePointMatchable(const Viewpoint& viewpoint1, const Viewpoint& viewpoint2, const Point3D& point3d) const {
//  const Vector3 vec1 = (point3d.getPosition() - viewpoint1.pose().getWorldPosition()).normalized();
//  const Vector3 vec2 = (point3d.getPosition() - viewpoint2.pose().getWorldPosition()).normalized();
//  const FloatType dot_product = vec1.dot(vec2);
//  const bool matchable = dot_product >= options_.sparse_matching_dot_product_threshold;
//  return matchable;
//}

bool ViewpointPlanner::isSparsePointMatchable(
        const Viewpoint& viewpoint1, const Viewpoint& viewpoint2,
        const Point3D& point3d, const Vector3& normal1, const Vector3& normal2) const {
  const Vector3 vec1 = (point3d.getPosition() - viewpoint1.pose().getWorldPosition()).normalized();
  const Vector3 vec2 = (point3d.getPosition() - viewpoint2.pose().getWorldPosition()).normalized();
  const FloatType dot_product = vec1.dot(vec2);
  if (dot_product < options_.sparse_matching_dot_product_threshold) {
    return false;
  }
  const Vector3 ray_direction1 = (viewpoint1.pose().getWorldPosition() - viewpoint1.projectWorldPointIntoCamera(point3d.getPosition())).normalized();
  const FloatType normal_dot_product1 = normal1.dot(ray_direction1);
  if (normal_dot_product1 < options_.sparse_matching_normal_dot_product_threshold) {
    return false;
  }
  const Vector3 ray_direction2 = (viewpoint2.pose().getWorldPosition() - viewpoint2.projectWorldPointIntoCamera(point3d.getPosition())).normalized();
  const FloatType normal_dot_product2= normal2.dot(ray_direction2);
  if (normal_dot_product2 < options_.sparse_matching_normal_dot_product_threshold) {
    return false;
  }
  return true;
}

std::unordered_map<Point3DId, ViewpointPlanner::Vector3> ViewpointPlanner::computeVisibleSparsePoints(
    const Viewpoint& viewpoint,
    const SparseReconstruction::Point3DMapType::const_iterator first,
    const SparseReconstruction::Point3DMapType::const_iterator last) const {
  std::unordered_map<Point3DId, ViewpointPlanner::Vector3> visible_sparse_points;
  for (SparseReconstruction::Point3DMapType::const_iterator it = first; it != last; ++it) {
    const Point3D& point3d = it->second;
    const bool visible = isSparsePointVisible(viewpoint, point3d);
    if (visible) {
      const Vector3 normal = computePoissonMeshNormalVector(viewpoint, point3d.getPosition());
      visible_sparse_points.emplace(point3d.id, normal);
    }
  }
  return visible_sparse_points;
}

std::unordered_map<Point3DId, ViewpointPlanner::Vector3> ViewpointPlanner::computeVisibleSparsePoints(
        const Viewpoint& viewpoint) const {
  const SparseReconstruction::Point3DMapType::const_iterator first = getReconstruction()->getPoints3D().begin();
  const SparseReconstruction::Point3DMapType::const_iterator last = getReconstruction()->getPoints3D().end();
  return computeVisibleSparsePoints(viewpoint, first, last);
}

const std::unordered_map<Point3DId, ViewpointPlanner::Vector3>& ViewpointPlanner::getCachedVisibleSparsePoints(
        const ViewpointEntryIndex viewpoint_index) const {
  std::lock_guard<std::mutex> lock(cached_visible_sparse_points_mutex_);
  auto it = cached_visible_sparse_points_.find(viewpoint_index);
  if (it == cached_visible_sparse_points_.end()) {
    const Viewpoint& viewpoint = viewpoint_entries_[viewpoint_index].viewpoint;
    std::tie(it, std::ignore) = cached_visible_sparse_points_.emplace(viewpoint_index, computeVisibleSparsePoints(viewpoint));
  }
  return it->second;
}

const std::unordered_set<size_t>& ViewpointPlanner::getCachedVisibleVoxels(
        const ViewpointEntryIndex viewpoint_index) const {
  std::lock_guard<std::mutex> lock(cached_visible_voxels_mutex_);
  auto it = cached_visible_voxels_.find(viewpoint_index);
  if (it == cached_visible_voxels_.end()) {
    const Viewpoint& viewpoint = viewpoint_entries_[viewpoint_index].viewpoint;
//    std::tie(it, std::ignore) = cached_visible_voxels_.emplace(viewpoint_index, getRaycastHitVoxelsSet(viewpoint));
    std::tie(it, std::ignore) = cached_visible_voxels_.emplace(viewpoint_index, getVisibleVoxels(viewpoint));
  }
  return it->second;
}

//ViewpointPlanner::FloatType ViewpointPlanner::computeSparseMatchingScore(
//    const Viewpoint& ref_viewpoint, const Viewpoint& other_viewpoint,
//    const std::unordered_set<Point3DId>& ref_visible_points,
//    const std::unordered_set<Point3DId>& other_visible_points) const {
//  const std::size_t num_x_slices = 3;
//  const std::size_t num_y_slices = 3;
//  Eigen::Matrix<std::size_t, num_y_slices, num_x_slices> ref_sector_shared_counts;
//  ref_sector_shared_counts.setZero();
//  Eigen::Matrix<std::size_t, num_y_slices, num_x_slices> other_sector_shared_counts;
//  other_sector_shared_counts.setZero();
//  std::size_t num_shared_points = 0;
//  for (const Point3DId& point3d_id: ref_visible_points) {
//    if (other_visible_points.count(point3d_id) > 0) {
//      const Point3D& point3d = data_->reconstruction_->getPoints3D().at(point3d_id);
//      const bool matchable = isSparsePointMatchable(
//              ref_viewpoint, other_viewpoint, point3d);
//      if (matchable) {
//        ++num_shared_points;
//        const Vector2 ref_point_image = ref_viewpoint.projectWorldPointIntoImage(point3d.getPosition());
//        const FloatType ref_x_slice_float = ref_point_image(0) / ref_viewpoint.camera().width();
//        const FloatType ref_y_slice_float = ref_point_image(1) / ref_viewpoint.camera().height();
//        const std::size_t ref_x_slice = (std::size_t)std::floor(ref_x_slice_float * num_x_slices);
//        const std::size_t ref_y_slice = (std::size_t)std::floor(ref_y_slice_float * num_y_slices);
//        ++ref_sector_shared_counts(ref_y_slice, ref_x_slice);
//        const Vector2 other_point_image = other_viewpoint.projectWorldPointIntoImage(point3d.getPosition());
//        const FloatType other_x_slice_float = other_point_image(0) / other_viewpoint.camera().width();
//        const FloatType other_y_slice_float = other_point_image(1) / other_viewpoint.camera().height();
//        const std::size_t other_x_slice = (std::size_t)std::floor(other_x_slice_float * num_x_slices);
//        const std::size_t other_y_slice = (std::size_t)std::floor(other_y_slice_float * num_y_slices);
//        ++other_sector_shared_counts(other_y_slice, other_x_slice);
//      }
//    }
//  }
//  FloatType matching_score = num_shared_points;
//  const std::size_t min_num_of_filled_sectors = 3;
//  const FloatType min_sector_ratio = FloatType(0.1);
//  std::size_t ref_num_of_filled_sectors = 0;
//  std::size_t other_num_of_filled_sectors = 0;
//  for (size_t y = 0; y < num_y_slices; ++y) {
//    for (size_t x = 0; x < num_x_slices; ++x) {
//      if (ref_sector_shared_counts(y, x) >= num_shared_points * min_sector_ratio) {
//        ++ref_num_of_filled_sectors;
//      }
//      if (other_sector_shared_counts(y, x) >= num_shared_points * min_sector_ratio) {
//        ++other_num_of_filled_sectors;
//      }
//    }
//  }
//  if (ref_num_of_filled_sectors < min_num_of_filled_sectors) {
//    matching_score = 0;
//  }
//  if (other_num_of_filled_sectors < min_num_of_filled_sectors) {
//    matching_score = 0;
//  }
////  BH_PRINT_VALUE(matching_score);
//  return matching_score;
//}

ViewpointPlanner::FloatType ViewpointPlanner::computeSparseMatchingScore(
        const Viewpoint& ref_viewpoint, const Viewpoint& other_viewpoint,
        const std::unordered_map<Point3DId, Vector3>& ref_visible_points,
        const std::unordered_map<Point3DId, Vector3>& other_visible_points) const {
  const std::size_t num_x_slices = 3;
  const std::size_t num_y_slices = 3;
  Eigen::Matrix<std::size_t, num_y_slices, num_x_slices> ref_sector_shared_counts;
  ref_sector_shared_counts.setZero();
  Eigen::Matrix<std::size_t, num_y_slices, num_x_slices> other_sector_shared_counts;
  other_sector_shared_counts.setZero();
  std::size_t num_shared_points = 0;
  for (const auto& entry: ref_visible_points) {
    const Point3DId& point3d_id = entry.first;
    const Vector3& ref_normal = entry.second;
    if (other_visible_points.count(point3d_id) > 0) {
      const Vector3& other_normal = other_visible_points.at(point3d_id);
      const Point3D& point3d = data_->reconstruction_->getPoints3D().at(point3d_id);
      const bool matchable = isSparsePointMatchable(
              ref_viewpoint, other_viewpoint,
              point3d, ref_normal, other_normal);
      if (matchable) {
        ++num_shared_points;
        const Vector2 ref_point_image = ref_viewpoint.projectWorldPointIntoImage(point3d.getPosition());
        const FloatType ref_x_slice_float = ref_point_image(0) / ref_viewpoint.camera().width();
        const FloatType ref_y_slice_float = ref_point_image(1) / ref_viewpoint.camera().height();
        const std::size_t ref_x_slice = (std::size_t)std::floor(ref_x_slice_float * num_x_slices);
        const std::size_t ref_y_slice = (std::size_t)std::floor(ref_y_slice_float * num_y_slices);
        ++ref_sector_shared_counts(ref_y_slice, ref_x_slice);
        const Vector2 other_point_image = other_viewpoint.projectWorldPointIntoImage(point3d.getPosition());
        const FloatType other_x_slice_float = other_point_image(0) / other_viewpoint.camera().width();
        const FloatType other_y_slice_float = other_point_image(1) / other_viewpoint.camera().height();
        const std::size_t other_x_slice = (std::size_t)std::floor(other_x_slice_float * num_x_slices);
        const std::size_t other_y_slice = (std::size_t)std::floor(other_y_slice_float * num_y_slices);
        ++other_sector_shared_counts(other_y_slice, other_x_slice);
      }
    }
  }
  FloatType matching_score = num_shared_points;
  const std::size_t min_num_of_filled_sectors = options_.sparse_matching_num_of_shared_sectors;
  const FloatType min_sector_ratio = options_.sparse_matching_shared_sector_ratio;
  std::size_t ref_num_of_filled_sectors = 0;
  std::size_t other_num_of_filled_sectors = 0;
  for (size_t y = 0; y < num_y_slices; ++y) {
    for (size_t x = 0; x < num_x_slices; ++x) {
      if (ref_sector_shared_counts(y, x) >= num_shared_points * min_sector_ratio) {
        ++ref_num_of_filled_sectors;
      }
      if (other_sector_shared_counts(y, x) >= num_shared_points * min_sector_ratio) {
        ++other_num_of_filled_sectors;
      }
    }
  }
  if (ref_num_of_filled_sectors < min_num_of_filled_sectors) {
    matching_score = 0;
  }
  if (other_num_of_filled_sectors < min_num_of_filled_sectors) {
    matching_score = 0;
  }
//  BH_PRINT_VALUE(matching_score);
  return matching_score;
}

ViewpointPlanner::FloatType ViewpointPlanner::computeSparseMatchingScore(
    const Viewpoint& ref_viewpoint, const Viewpoint& other_viewpoint) const {
  const std::unordered_map<Point3DId, Vector3> ref_visible_points = computeVisibleSparsePoints(ref_viewpoint);
  const std::unordered_map<Point3DId, Vector3> other_visible_points = computeVisibleSparsePoints(other_viewpoint);
  return computeSparseMatchingScore(ref_viewpoint, other_viewpoint,
                                    ref_visible_points, other_visible_points);
}

ViewpointPlanner::FloatType ViewpointPlanner::computeSparseMatchingScore(
    const ViewpointEntryIndex viewpoint_index1, const ViewpointEntryIndex viewpoint_index2) const {
  const Viewpoint& viewpoint1 = viewpoint_entries_[viewpoint_index1].viewpoint;
  const Viewpoint& viewpoint2 = viewpoint_entries_[viewpoint_index2].viewpoint;
  const std::unordered_map<Point3DId, Vector3>& sparse_points_visible1 = getCachedVisibleSparsePoints(viewpoint_index1);
  const std::unordered_map<Point3DId, Vector3>& sparse_points_visible2 = getCachedVisibleSparsePoints(viewpoint_index2);
  return computeSparseMatchingScore(viewpoint1, viewpoint2,
                                    sparse_points_visible1, sparse_points_visible2);
}

//bool ViewpointPlanner::isSparseMatchable(
//    const Viewpoint& ref_viewpoint, const Viewpoint& other_viewpoint,
//    const std::unordered_set<Point3DId>& ref_visible_points,
//    const std::unordered_set<Point3DId>& other_visible_points) const {
//  const FloatType matching_score = computeSparseMatchingScore(ref_viewpoint, other_viewpoint, ref_visible_points, other_visible_points);
//  return matching_score >= options_.sparse_matching_score_threshold;
//}

bool ViewpointPlanner::isSparseMatchable(
        const Viewpoint& ref_viewpoint, const Viewpoint& other_viewpoint,
        const std::unordered_map<Point3DId, Vector3>& ref_visible_points,
        const std::unordered_map<Point3DId, Vector3>& other_visible_points) const {
  const FloatType matching_score = computeSparseMatchingScore(
          ref_viewpoint, other_viewpoint,
          ref_visible_points, other_visible_points);
  return matching_score >= options_.sparse_matching_score_threshold;
}

bool ViewpointPlanner::isSparseMatchable(const Viewpoint& ref_viewpoint, const Viewpoint& other_viewpoint) const {
  // TODO
//  const FloatType increment_distance = (ref_viewpoint.pose().translation() - other_viewpoint.pose().translation()).norm();
//  const FloatType increment_angular_distance = ref_viewpoint.pose().quaternion().angularDistance(other_viewpoint.pose().quaternion());
//  if (increment_distance < options_.sparse_matching_min_distance
//      && increment_angular_distance < options_.sparse_matching_min_angular_distance) {
//    std::cout << "Distance is less than " << increment_distance << " and less than "
//        <<  options_.sparse_matching_min_angular_distance * 180. / M_PI << " -> assume matchable" << std::endl;
//    return true;
//  }
  const FloatType matching_score = computeSparseMatchingScore(ref_viewpoint, other_viewpoint);
  return matching_score >= options_.sparse_matching_score_threshold;
}

bool ViewpointPlanner::isSparseMatchable(
    const ViewpointEntryIndex viewpoint_index1, const ViewpointEntryIndex viewpoint_index2) const {
  // TODO
//  const Viewpoint& viewpoint1 = viewpoint_entries_[viewpoint_index1].viewpoint;
//  const Viewpoint& viewpoint2 = viewpoint_entries_[viewpoint_index2].viewpoint;
//  const FloatType increment_distance = (viewpoint1.pose().translation() - viewpoint2.pose().translation()).norm();
//  const FloatType increment_angular_distance = viewpoint1.pose().quaternion().angularDistance(viewpoint2.pose().quaternion());
//  if (increment_distance < options_.sparse_matching_min_distance
//      && increment_angular_distance < options_.sparse_matching_min_angular_distance) {
//    std::cout << "Distance is less than " << increment_distance << " and less than "
//        <<  options_.sparse_matching_min_angular_distance * 180. / M_PI << " -> assume matchable" << std::endl;
//    return true;
//  }
  const FloatType matching_score = computeSparseMatchingScore(viewpoint_index1, viewpoint_index2);
  return matching_score >= options_.sparse_matching_score_threshold;
}
