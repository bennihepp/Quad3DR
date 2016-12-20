//==================================================
// viewpoint_planner.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 12, 2016
//==================================================

#include <functional>
#include <ait/common.h>
#include <ait/utilities.h>
#include "viewpoint_planner.h"

Viewpoint::Viewpoint(const PinholeCamera* camera, const Pose& pose, double projection_margin)
: camera_(camera), pose_(pose), projection_margin_(projection_margin),
  transformation_world_to_image_(pose_.getTransformationWorldToImage()) {}

std::unordered_set<Point3DId> Viewpoint::getProjectedPoint3DIds(const SparseReconstruction::Point3DMapType& points) const {
  ait::Timer timer;
  std::unordered_set<Point3DId> proj_point_ids;
  for (const auto& entry : points) {
    const Point3DId point_id = entry.first;
    const Point3D& point_world = entry.second;
    Eigen::Vector2d point_image = projectWorldPointIntoImage(point_world.pos);
    bool projected = camera_->isPointInViewport(point_image, projection_margin_);
    if (projected) {
      proj_point_ids.insert(point_id);
    }
  }
  timer.printTiming("getProjectedPoint3DIds");
  return proj_point_ids;
}

Ray Viewpoint::getCameraRay(double x, double y) const {
  Eigen::Vector3d origin = pose_.getWorldPosition();
  // Eigen::Vector3d direction_camera = camera_->getCameraRay(x, y);
  Eigen::Vector3d direction_camera = camera_->unprojectPoint(x, y, 1.0);
  Eigen::Vector3d direction = pose_.getTransformationImageToWorld().topLeftCorner<3, 3>() * direction_camera;
  origin = origin + direction;
  return Ray(origin, direction);
}

void ViewpointPlanner::run() {
  return;
    // Check how many map points fall in an occupied voxel
//        for (size_t max_depth = 1; max_depth <= octree_->getTreeDepth(); ++max_depth) {
//            size_t num_map_point_nodes = 0;
//            double average_occupancy_level = 0.0;
//            for (const auto& entry : sparse_recon_->getPoints3D()) {
//                const Point3D& point_world = entry.second;
//                const OctomapType::NodeType* node = octree_->search(point_world.pos(0), point_world.pos(1), point_world.pos(2), max_depth);
//                if (node == nullptr) {
////                    std::cerr << "ERROR: Map point could not be found in octree" << std::endl;
//                }
//                else {
//                    average_occupancy_level += node->getOccupancy();
//                    ++num_map_point_nodes;
//                }
//            }
//            average_occupancy_level /= num_map_point_nodes;
//            std::cout << "Search depth: " << max_depth << std::endl;
//            std::cout << "  Found " << num_map_point_nodes << " map point nodes out of " << sparse_recon_->getPoints3D().size() << " points" << std::endl;
//            std::cout << "  Average map point occupancy: " << average_occupancy_level << std::endl;
//        }

  // Check how many map points are projected into each image without being occluded
  for (const auto& entry : reconstruction_->getImages()) {
    ait::Timer timer2;
    const Image& image = entry.second;
    const PinholeCameraColmap& camera = reconstruction_->getCameras().at(image.camera_id);
    std::cout << "image_id=" << image.id << std::endl;
    std::cout << "  features: " << image.features.size() << std::endl;
    const double projection_margin = 0;
    std::unordered_set<Point3DId> proj_points = computeProjectedMapPoints(camera.id(), image.pose, projection_margin);
    std::unordered_set<Point3DId> filtered_points = computeFilteredMapPoints(camera.id(), image.pose, projection_margin);
    std::unordered_set<Point3DId> visible_points = computeVisibleMapPoints(camera.id(), image.pose, projection_margin);
    ait::Timer timer;
    std::unordered_set<Point3DId> filtered_visible_points = computeSetIntersection(filtered_points, visible_points);
    timer.printTiming("Intersection computation");
//            std::unordered_set<Point3DId> filtered_points(proj_points);
//            std::unordered_set<Point3DId> visible_points(proj_points);
//            removeOccludedPoints(image.pose, visible_points);
    std::cout << "  projected points: " << proj_points.size() << std::endl;
    std::cout << "  filtered points: " << filtered_points.size() << std::endl;
    std::cout << "  non-occluded points: " << visible_points.size() << std::endl;
    std::cout << "  filtered and non-occluded: " << computeSetIntersectionSize(filtered_points, visible_points) << std::endl;
    std::unordered_set<Point3DId> feature_map_points;
    for (const auto& feature : image.features) {
      if (feature.point3d_id != invalid_point3d_id) {
        feature_map_points.insert(feature.point3d_id);
      }
    }
    timer = ait::Timer();
    std::cout << "  Features with map point: " << feature_map_points.size() << std::endl;
    std::cout << "    and projected: " << computeSetIntersectionSize(proj_points, feature_map_points) << std::endl;
    std::cout << "    and filtered: " << computeSetIntersectionSize(filtered_points, feature_map_points) << std::endl;
    std::cout << "    and non-occluded: " << computeSetIntersectionSize(visible_points, feature_map_points) << std::endl;
    std::cout << "    and filtered and non-occluded: " << computeSetIntersectionSize(filtered_visible_points, feature_map_points) << std::endl;
    timer.printTiming("Intersection size computation");
//            AIT_ASSERT(found_features == features_with_map_point);
    timer2.printTiming("loop");
  }
}

std::unordered_set<Point3DId> Viewpoint::getProjectedPoint3DIdsFiltered(const SparseReconstruction::Point3DMapType& points) const {
  ait::Timer timer;
  std::unordered_set<Point3DId> proj_point_ids;
  for (const auto& entry : points) {
    const Point3DId point_id = entry.first;
    const Point3D& point_world = entry.second;
    Eigen::Vector2d point_image = projectWorldPointIntoImage(point_world.getPosition());
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
  std::tuple<double, Eigen::Vector3d> result = ait::computeDistanceAndDirection(point.getPosition(), pose_.getWorldPosition());
  double dist_deviation = std::abs(std::get<0>(result) - statistics.averageDistance());
  if (dist_deviation > MAX_DISTANCE_DEVIATION_BY_STDDEV * statistics.stddevDistance()) {
    return true;
  }
  double one_minus_dot_product = 1 - std::get<1>(result).dot(point.getNormal());
  double one_minus_dot_deviation = std::abs(one_minus_dot_product);
  if (one_minus_dot_deviation > MAX_NORMAL_DEVIATION_BY_STDDEV * statistics.stddevOneMinusDotProduct()) {
    return true;
  }
  return false;
}

Eigen::Vector2d Viewpoint::projectWorldPointIntoImage(const Eigen::Vector3d& point_world) const {
  Eigen::Vector3d point_camera  = transformation_world_to_image_ * point_world.homogeneous();
  Eigen::Vector2d point_image = camera_->projectPoint(point_camera);
  return point_image;
}


ViewpointPlannerMaps::ViewpointPlannerMaps(const InputOccupancyMapType* octree) {
  std::vector<typename bvh::Tree<TestNode, float>::ObjectWithBoundingBox> objects;
  for (auto it = octree->begin_tree(); it != octree->end_tree(); ++it) {
    if (it.isLeaf()) {
//      if (octree->isNodeFree(&(*it)) || octree->isNodeUnknown(&(*it))) {
      if (octree->isNodeFree(&(*it)) && octree->isNodeKnown(&(*it))) {
        continue;
      }
      typename bvh::Tree<TestNode, float>::ObjectWithBoundingBox object_with_bbox;
      octomap::point3d center_octomap = it.getCoordinate();
      Eigen::Vector3f center;
      center << center_octomap.x(), center_octomap.y(), center_octomap.z();
      const float size = it.getSize();
      object_with_bbox.bounding_box = typename bvh::Tree<TestNode, float>::BoundingBoxType(center, size);
      object_with_bbox.object = nullptr;
      objects.push_back(object_with_bbox);
    }
  }
  std::cout << "Building BVH tree with " << objects.size() << " objects" << std::endl;
  ait::Timer timer;
  occupied_bvh_.build(objects);
  timer.printTimingMs("Building BVH tree");
}


ViewpointPlanner::ViewpointPlanner(std::unique_ptr<DenseReconstruction> dense_recon, std::unique_ptr<OccupancyMapType> octree,
    std::unique_ptr<ViewpointPlannerMaps> maps)
: reconstruction_(std::move(dense_recon)), octree_(std::move(octree)), maps_(std::move(maps)) {
  AIT_ASSERT(reconstruction_->getCameras().size() > 0);
  setScaledVirtualCamera(reconstruction_->getCameras().cbegin()->first, 0.05);
}

template <typename TreeT>
bool ViewpointPlanner::isTreeConsistent(const TreeT& tree) {
  bool consistent = true;
  for (auto it = tree.begin_tree(); it != tree.end_tree(); ++it) {
    if (it->hasChildren()) {
      for (size_t i = 0; i < 8; ++i) {
        if (it->hasChild(i)) {
          if (it->getObservationCount() > it->getChild(i)->getObservationCount()) {
            consistent = false;
            break;
          }
          if (it->getOccupancy() < it->getChild(i)->getOccupancy()) {
            consistent = false;
            break;
          }
        }
      }
      if (!consistent) {
        break;
      }
    }
  }
  return consistent;
}

std::unique_ptr<ViewpointPlanner::OccupancyMapType> ViewpointPlanner::getAugmentedOccupancyMap(
    std::unique_ptr<InputOccupancyMapType> input_tree) {
  ait::Timer timer;
  if (!isTreeConsistent(*input_tree.get())) {
    throw AIT_EXCEPTION("Input tree is inconsistent");
  }

  std::unique_ptr<OccupancyMapType> output_tree;
  output_tree.reset(convertToAugmentedMap(input_tree.get()));

  if (!isTreeConsistent(*output_tree.get())) {
    throw AIT_EXCEPTION("Augmented tree is inconsistent");
  }
  timer.printTimingMs("Copying input tree");

  timer = ait::Timer();
  // Augment tree with weights
  std::vector<TreeNavigatorType> query_nodes;
  AIT_ASSERT(OCCUPANCY_WEIGHT_DEPTH - OCCUPANCY_WEIGHT_REACH > 0);
  AIT_ASSERT(OCCUPANCY_WEIGHT_DEPTH_CUTOFF > OCCUPANCY_WEIGHT_DEPTH);
  for (auto it = output_tree->begin_tree(OCCUPANCY_WEIGHT_DEPTH); it != output_tree->end_tree(); ++it) {
    if (it.getDepth() == OCCUPANCY_WEIGHT_DEPTH) {
      query_nodes.push_back(TreeNavigatorType(output_tree.get(), it.getKey(), &(*it), it.getDepth()));
    }
  }

  float max_total_weight = 0;
  for (const TreeNavigatorType& query_nav : query_nodes) {
    const float dist_cutoff = 0.5f * query_nav.getSize();
    const float dist_cutoff_sq = dist_cutoff * dist_cutoff;
    const Eigen::Vector3f query_pos = query_nav.getPosition();

    ConstTreeNavigatorType parent_nav = query_nav;
    for (size_t i = 0; i < OCCUPANCY_WEIGHT_REACH; ++i) {
      parent_nav.gotoParent();
    }
    std::stack<ConstTreeNavigatorType> node_stack;
    node_stack.push(parent_nav);

    WeightType total_weight = 0;
    size_t total_count = 0;
    while (!node_stack.empty()) {
      ConstTreeNavigatorType nav = node_stack.top();
      node_stack.pop();
      if (nav.hasChildren() && nav.getDepth() < OCCUPANCY_WEIGHT_DEPTH_CUTOFF) {
        for (size_t i = 0; i < 8; ++i) {
          if (nav.hasChild(i)) {
            node_stack.push(nav.child(i));
          }
        }
      }
      else {
        if (nav->getObservationCount() > 0 && output_tree->isNodeOccupied(nav.getNode())) {
//        if (nav->getObservationCount() > 0) {
          WeightType weight = computeWeightContribution(query_pos, dist_cutoff_sq, nav);
//          size_t count;
//          WeightType weight;
//          if (nav->hasChildren()) {
//            size_t count = 1 << (output_tree->getTreeDepth() - nav.getDepth());
//            weight = nav->getMeanChildOccupancy() * count;
//          } else {
//            count = 1;
//            weight = nav->getOccupancy();
//          }
//          total_count += count;
          total_weight += weight;
        }
      }
    }

//    total_weight = total_count > 0 ? total_weight / total_count : 0;
//    std::cout << "total_weight=" << total_weight << ", total_count=" << total_count << std::endl;

//    std::cout << "total_weight: " << total_weight << std::endl;
    if (total_weight > max_total_weight) {
      max_total_weight = total_weight;
    }

    // Pass weight down the tree to the leaf nodes
    std::stack<TreeNavigatorType> node_stack2;
    node_stack2.push(query_nav);
    while (!node_stack2.empty()) {
      TreeNavigatorType nav = node_stack2.top();
      node_stack2.pop();
      AIT_ASSERT(nav->getWeight() == 0);
      nav->setWeight(total_weight);
      if (nav.hasChildren()) {
        for (size_t i = 0; i < 8; ++i) {
          if (nav.hasChild(i)) {
            node_stack2.push(nav.child(i));
          }
        }
      }
    }
  }
  timer.printTimingMs("Augmenting tree");

  std::cout << "Maximum weight: " << max_total_weight << std::endl;
  return std::move(output_tree);
}

ViewpointPlanner::WeightType ViewpointPlanner::computeWeightContribution(
    const Eigen::Vector3f& query_pos, float dist_cutoff_sq, const ConstTreeNavigatorType& nav) {
//  const WeightType observation_factor = computeObservationWeightFactor(nav->getObservationCountSum());
//  WeightType weight = nav->getOccupancy() * observation_factor;
  WeightType weight = nav->getOccupancy() * nav->getObservationCountSum();
  const Eigen::Vector3f node_pos = nav.getPosition();
  float dist_sq = (node_pos - query_pos).squaredNorm();
  dist_sq = std::max(dist_cutoff_sq, dist_sq);
  weight /= std::sqrt(dist_cutoff_sq / dist_sq);
  return weight;
}

ViewpointPlanner::WeightType ViewpointPlanner::computeObservationWeightFactor(CounterType observation_count) {
  return 1 - std::exp(- 0.5f * observation_count);
}

ViewpointPlanner::WeightType ViewpointPlanner::computeObservationInformationFactor(CounterType observation_count) {
  return std::exp(- 1.0f * observation_count);
}

std::unordered_set<Point3DId> ViewpointPlanner::computeProjectedMapPoints(const CameraId camera_id, const Pose& pose,
        double projection_margin) const {
  Viewpoint viewpoint(&reconstruction_->getCameras().at(camera_id), pose, projection_margin);
  std::unordered_set<Point3DId> proj_points = viewpoint.getProjectedPoint3DIds(reconstruction_->getPoints3D());
  return proj_points;
}

std::unordered_set<Point3DId> ViewpointPlanner::computeFilteredMapPoints(const CameraId camera_id, const Pose& pose,
        double projection_margin) const {
  Viewpoint viewpoint(&reconstruction_->getCameras().at(camera_id), pose, projection_margin);
  std::unordered_set<Point3DId> proj_points = viewpoint.getProjectedPoint3DIdsFiltered(reconstruction_->getPoints3D());
  return proj_points;
}

std::unordered_set<Point3DId> ViewpointPlanner::computeVisibleMapPoints(const CameraId camera_id, const Pose& pose,
        double projection_margin) const {
  Viewpoint viewpoint(&reconstruction_->getCameras().at(camera_id), pose, projection_margin);
  std::unordered_set<Point3DId> proj_points = viewpoint.getProjectedPoint3DIds(reconstruction_->getPoints3D());
  removeOccludedPoints(pose, proj_points, OCCLUSION_DIST_MARGIN_FACTOR * octree_->getResolution());
  return proj_points;
}

std::unordered_set<Point3DId> ViewpointPlanner::computeVisibleMapPointsFiltered(const CameraId camera_id, const Pose& pose,
        double projection_margin) const {
  Viewpoint viewpoint(&reconstruction_->getCameras().at(camera_id), pose, projection_margin);
  std::unordered_set<Point3DId> proj_points = viewpoint.getProjectedPoint3DIdsFiltered(reconstruction_->getPoints3D());
  removeOccludedPoints(pose, proj_points, OCCLUSION_DIST_MARGIN_FACTOR * octree_->getResolution());
  return proj_points;
}

//double ViewpointPlanner::computeInformationScore(const CameraId camera_id, const Pose& pose_world_to_image) const {
//  ait::Timer timer;
//  bool ignore_unknown = true;
//  double score = rayCastAccumulate<double>(camera_id, pose_world_to_image, ignore_unknown,
//      [](bool hit, const octomap::point3d& hit_point) -> double {
//    if (hit) {
//      return 1;
//    }
//    else {
//      return 0;
//    }
//  });
//  timer.printTiming("computeInformationScore");
//  return score;
//}

std::vector<std::pair<ViewpointPlannerMaps::OccupiedTreeType::NodeType*, float>> ViewpointPlanner::getRaycastHitVoxelsBVH(
    const Pose& pose_world_to_image) const {
  Viewpoint viewpoint(&virtual_camera_, pose_world_to_image);
  ait::Timer timer;
  Pose pose_image_to_world = viewpoint.pose().inverse();
  Eigen::Vector3d eigen_origin = pose_image_to_world.translation();
  octomap::point3d origin(eigen_origin(0), eigen_origin(1), eigen_origin(2));
  octomap::OcTreeKey origin_key;
  std::vector<std::pair<ViewpointPlannerMaps::OccupiedTreeType::NodeType*, float>> raycast_voxels;
  if (!octree_->coordToKeyChecked(origin, origin_key)) {
    std::cout << "WARNING: Requesting ray cast for out of bounds viewpoint" << std::endl;
    return raycast_voxels;
  }
  for (size_t y = 0; y < virtual_camera_.height(); ++y) {
//  for (size_t y = virtual_camera_.height()/2-10; y < virtual_camera_.height()/2+10; ++y) {
//  size_t y = virtual_camera_.height() / 2; {
//  size_t y = 85; {
#if !AIT_DEBUG
#pragma omp parallel for
#endif
    for (size_t x = 0; x < virtual_camera_.width(); ++x) {
//    for (size_t x = virtual_camera_.width()/2-10; x < virtual_camera_.width()/2+10; ++x) {
//    size_t x = virtual_camera_.width() / 2; {
//      size_t x = 101; {
      const Ray ray = viewpoint.getCameraRay(x, y);
      float min_range = 0.0f;
      float max_range = 60.0f;
      bvh::Ray bvh_ray;
      bvh_ray.origin = ray.origin();
      bvh_ray.direction = ray.direction();
      std::pair<bool, ViewpointPlannerMaps::OccupiedTreeType::IntersectionResult> result =
          maps_->occupied_bvh_.intersects(bvh_ray, min_range, max_range);
      if (result.first) {
#if !AIT_DEBUG
#pragma omp critical
#endif
        raycast_voxels.push_back(std::make_pair(
            result.second.node,
            0));
      }
    }
  }
  timer.printTiming("getRaycastHitVoxelsBHV");
  return raycast_voxels;
}

std::vector<std::pair<ViewpointPlanner::ConstTreeNavigatorType, float>> ViewpointPlanner::getRaycastHitVoxels(
    const Pose& pose_world_to_image) const {
  Viewpoint viewpoint(&virtual_camera_, pose_world_to_image);
  ait::Timer timer;
  Pose pose_image_to_world = viewpoint.pose().inverse();
  Eigen::Vector3d eigen_origin = pose_image_to_world.translation();
  octomap::point3d origin(eigen_origin(0), eigen_origin(1), eigen_origin(2));
  octomap::OcTreeKey origin_key;
  std::vector<std::pair<ConstTreeNavigatorType, float>> raycast_voxels;
  if (!octree_->coordToKeyChecked(origin, origin_key)) {
    std::cout << "WARNING: Requesting ray cast for out of bounds viewpoint" << std::endl;
    return raycast_voxels;
  }
  for (size_t y = 0; y < virtual_camera_.height(); ++y) {
//  for (size_t y = virtual_camera_.height()/2-10; y < virtual_camera_.height()/2+10; ++y) {
//  size_t y = virtual_camera_.height() / 2; {
//  size_t y = 85; {
#if !AIT_DEBUG
#pragma omp parallel for
#endif
    for (size_t x = 0; x < virtual_camera_.width(); ++x) {
//    for (size_t x = virtual_camera_.width()/2-10; x < virtual_camera_.width()/2+10; ++x) {
//    size_t x = virtual_camera_.width() / 2; {
//      size_t x = 101; {
      const Ray ray = viewpoint.getCameraRay(x, y);
      float min_range = 5.0f;
      float max_range = 60.0f;
      OccupancyMapType::RayCastData ray_cast_result;
      bool result = octree_->castRayFast<OccupancyMapType::CollisionUnknownOrOccupied>(
          ray.origin(), ray.direction(), &ray_cast_result, min_range, max_range);
      if (result) {
        CounterType voxel_observation_count = ray_cast_result.end_node->getObservationCount();
        if (ray_cast_result.end_depth < octree_->getTreeDepth()) {
          voxel_observation_count /= 1 << (octree_->getTreeDepth() - ray_cast_result.end_depth);
        }
        WeightType information_factor = computeObservationInformationFactor(voxel_observation_count);
        WeightType weight = ray_cast_result.end_node->getWeight();
        WeightType information = information_factor * weight;
#if !AIT_DEBUG
#pragma omp critical
#endif
        {
        raycast_voxels.push_back(std::make_pair(
            ConstTreeNavigatorType(octree_.get(), ray_cast_result.end_key, ray_cast_result.end_node, ray_cast_result.end_depth),
            information));
        }
      }
    }
  }
  timer.printTiming("getRaycastHitVoxels");
  return raycast_voxels;
}

double ViewpointPlanner::computeInformationScore(const Pose& pose_world_to_image) const {
  Viewpoint viewpoint(&virtual_camera_, pose_world_to_image);
  ait::Timer timer;
  Pose pose_image_to_world = viewpoint.pose().inverse();
  Eigen::Vector3d eigen_origin = pose_image_to_world.translation();
  octomap::point3d origin(eigen_origin(0), eigen_origin(1), eigen_origin(2));
  octomap::OcTreeKey origin_key;
  if (!octree_->coordToKeyChecked(origin, origin_key)) {
    std::cout << "WARNING: Requesting ray cast for out of bounds viewpoint" << std::endl;
    return std::numeric_limits<double>::quiet_NaN();
  }
  double score = 0;
  double unknown_score = 0;
  double unknown_count = 0;
  for (size_t y = 0; y < virtual_camera_.height(); ++y) {
//  size_t y = virtual_camera_.height() / 2; {
#if !AIT_DEBUG
#pragma omp parallel for
#endif
    for (size_t x = 0; x < virtual_camera_.width(); ++x) {
//    size_t x = virtual_camera_.width() / 2; {
      const Ray ray = viewpoint.getCameraRay(x, y);
      double min_range = 5.0;
      double max_range = 60.0;
      OccupancyMapType::RayCastData ray_cast_result;
      bool result = octree_->castRayFast<OccupancyMapType::CollisionUnknownOrOccupied>(
          ray.origin(), ray.direction(), &ray_cast_result, min_range, max_range);
//      if (result && octree_->isNodeUnknown(ray_cast_result.end_node)) {
      if (result) {
//        std::cout << "origin: " << ray.origin().transpose() << ", node_pos: " << end_point.transpose() << std::endl;
//        std::cout << "occupancy: " << end_node->getOccupancy() << ", observations: " << end_node->getObservationCount() << std::endl;
        CounterType voxel_observation_count = ray_cast_result.end_node->getObservationCount();
        if (ray_cast_result.end_depth < octree_->getTreeDepth()) {
          voxel_observation_count /= 1 << (octree_->getTreeDepth() - ray_cast_result.end_depth);
        }
        WeightType information_factor = computeObservationInformationFactor(voxel_observation_count);
        WeightType weight = ray_cast_result.end_node->getWeight();
#if !AIT_DEBUG
//#pragma omp atomic
#endif
//        score += 1;
        score += information_factor * weight;
        if (octree_->isNodeUnknown(ray_cast_result.end_node)) {
          unknown_count += 1;
          unknown_score += weight;
        }
      }
//      std::cout << "x=" << x << ", y=" << y << ", result=" << result << std::endl;
//      std::cout << "x=" << x << ", y=" << y << ", origin=" << origin << " direction=" << ray.direction() << std::endl;
    }
  }
//  score /= virtual_camera_.width() * virtual_camera_.height();
  std::cout << "Unknown voxels visible: " << unknown_count << std::endl;
  std::cout << "Unknown voxels score: " << unknown_score << std::endl;
  timer.printTiming("computeInformationScore");
  return score;
}

void ViewpointPlanner::removeOccludedPoints(const Pose& pose_world_to_image, std::unordered_set<Point3DId>& point3D_ids, double dist_margin) const {
  ait::Timer timer;
  Pose pose_image_to_world = pose_world_to_image.inverse();
  Eigen::Vector3d eigen_origin = pose_image_to_world.translation();
  octomap::point3d origin(eigen_origin(0), eigen_origin(1), eigen_origin(2));
  octomap::KeyRay key_ray;
  octomap::OcTreeKey origin_key;
  if (!octree_->coordToKeyChecked(origin, origin_key)) {
    point3D_ids.clear();
    return;
  }
  for (auto it = point3D_ids.begin(); it != point3D_ids.end(); ) {
    const Point3D& point3D = reconstruction_->getPoints3D().at(*it);
    octomap::point3d end_point(point3D.pos(0), point3D.pos(1), point3D.pos(2));
    octomap::point3d end_point_from_origin = end_point - origin;
    octomap::OcTreeKey end_point_key;
    bool occluded = false;
    if (!octree_->coordToKeyChecked(end_point, end_point_key)) {
      occluded = true;
    } else {
//                bool success = octree_->computeRayKeys(origin, end_point, key_ray);
      double end_point_dist = end_point_from_origin.norm();
      octomap::point3d hit_point;
      const bool ignore_unknown = true;
      octree_->castRay(origin, end_point_from_origin, hit_point, ignore_unknown, end_point_dist);
      double hit_dist = (hit_point - origin).norm();
      if (end_point_key != octree_->coordToKey(hit_point) && hit_dist + dist_margin < end_point_dist) {
        occluded = true;
      }
      else {
        occluded = false;
      }
    }
    if (occluded) {
      it = point3D_ids.erase(it);
    }
    else {
      ++it;
    }
  }
  timer.printTiming("removeOccludedPoints");
}
