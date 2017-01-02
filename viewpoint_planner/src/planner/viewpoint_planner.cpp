//==================================================
// viewpoint_planner.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 12, 2016
//==================================================

#include <functional>
#include <random>
#include <boost/filesystem.hpp>
#include <ait/common.h>
#include <ait/math.h>
#include <ait/utilities.h>
#include <ait/eigen_utils.h>
#include <ait/vision_utilities.h>
#include "viewpoint_planner.h"

using std::swap;

using ait::computeSetIntersection;
using ait::computeSetIntersectionSize;
using ait::computeSetDifference;
using ait::computeSetDifferenceSize;

ViewpointPlanner::ViewpointPlanner(const Options* options, std::unique_ptr<ViewpointPlannerData> data)
: options_(*options), data_(std::move(data)),
  motion_planner_(data_.get()) {
  size_t random_seed = options_.rng_seed;
  if (random_seed == 0) {
    random_seed = std::chrono::system_clock::now().time_since_epoch().count();
  }
  random_.setSeed(random_seed);
  AIT_ASSERT(data_->reconstruction_->getCameras().size() > 0);
  setScaledVirtualCamera(data_->reconstruction_->getCameras().cbegin()->first, options_.virtual_camera_scale);
  drone_extent_ = Vector3(
      options->getValue<FloatType>("drone_extent_x"),
      options->getValue<FloatType>("drone_extent_y"),
      options->getValue<FloatType>("drone_extent_z"));
  FloatType motion_sampling_roi_factor = 2 * options_.sampling_roi_factor;
  motion_planner_.setSpaceBoundingBox(motion_sampling_roi_factor * data_->roi_bbox_);
  motion_planner_.setObjectExtent(drone_extent_);

  reset();

  if (!options_.viewpoint_graph_filename.empty()) {
    loadViewpointGraph(options_.viewpoint_graph_filename);
  }
}

void ViewpointPlanner::reset() {
  std::unique_lock<std::mutex> lock(mutex_);
  viewpoint_entries_.clear();
  viewpoint_ann_.clear();
  viewpoint_graph_.clear();
  viewpoint_paths_.clear();
  viewpoint_paths_.resize(options_.viewpoint_path_branches);
  feature_viewpoint_map_.clear();
  motion_planner_.initialize();
  lock.unlock();

  // Initialize viewpoint graph from existing real images
  std::cout << "Adding previous camera viewpoints to viewpoint graph" << std::endl;
  std::vector<Vector3> ann_points;
  for (const auto& entry : data_->reconstruction_->getImages()) {
    const Pose& pose = entry.second.pose();
//    std::cout << "  image id=" << entry.first << ", pose=" << pose << std::endl;
    FloatType total_information = 0;
    VoxelWithInformationSet voxel_set;
    addViewpointEntry(ViewpointEntry(Viewpoint(&virtual_camera_, pose), total_information, std::move(voxel_set)));
  }
  num_real_viewpoints_ = viewpoint_entries_.size();
  // Initialize viewpoint nearest neighbor index
  std::cout << "initialized viewpoint entries with " << viewpoint_entries_.size() << " viewpoints" << std::endl;

  for (std::size_t i = 0; i < ann_points.size(); ++i) {
    std::cout << "ann_points1[" << i << "]=" << ann_points[i].transpose() << std::endl;
    std::cout << "ann_points2[" << i << "]=" << viewpoint_ann_.getPoint(i).transpose() << std::endl;
  }
//  AIT_ASSERT(false);

  // TODO: Initialize feature_viewpoint_map_ from existing real images
//  for (std::size_t i = 0; i < data_->poisson_mesh_->)
}

void ViewpointPlanner::resetViewpointMotions() {
  std::unique_lock<std::mutex> lock(mutex_);
  for (const ViewpointEntryIndex index : viewpoint_graph_) {
    viewpoint_graph_.getEdges(index).clear();
  }
  lock.unlock();
}

void ViewpointPlanner::resetViewpointPaths() {
  std::unique_lock<std::mutex> lock(mutex_);
  viewpoint_paths_.clear();
  viewpoint_paths_.resize(options_.viewpoint_path_branches);
  lock.unlock();
}

void ViewpointPlanner::saveViewpointGraph(const std::string& filename) const {
  std::cout << "Writing viewpoint graph to " << filename << std::endl;
  std::ofstream ofs(filename);
  boost::archive::binary_oarchive oa(ofs);
  ViewpointEntrySaver ves(viewpoint_entries_, data_->occupied_bvh_);
  oa << num_real_viewpoints_;
  oa << ves;
  oa << viewpoint_graph_;
  std::cout << "Done" << std::endl;
}

void ViewpointPlanner::loadViewpointGraph(const std::string& filename) {
  reset();
  std::cout << "Loading viewpoint graph from " << filename << std::endl;
  std::ifstream ifs(filename);
  boost::archive::binary_iarchive ia(ifs);
  std::size_t new_num_real_viewpoints;
  ia >> new_num_real_viewpoints;
  AIT_ASSERT(new_num_real_viewpoints == num_real_viewpoints_);
  ViewpointEntryLoader vel(&viewpoint_entries_, &data_->occupied_bvh_, data_->reconstruction_.get());
  ia >> vel;
  ia >> viewpoint_graph_;
  std::cout << "Regenerating approximate nearest neighbor index" << std::endl;
  viewpoint_ann_.clear();
  for (const ViewpointEntry& viewpoint_entry : viewpoint_entries_) {
    viewpoint_ann_.addPoint(viewpoint_entry.viewpoint.pose().getWorldPosition());
  }
  std::cout << "Loaded viewpoint graph with " << viewpoint_entries_.size() << " viewpoints." << std::endl;
}

std::pair<bool, ViewpointPlanner::Pose> ViewpointPlanner::samplePose(std::size_t max_trials /*= (std::size_t)-1*/) const {
  return samplePose(data_->roi_bbox_, drone_extent_, max_trials);
}

std::pair<bool, ViewpointPlanner::Pose> ViewpointPlanner::samplePose(const BoundingBoxType& bbox,
    const Vector3& object_extent, std::size_t max_trials /*= (std::size_t)-1*/) const {
  if (max_trials == (std::size_t)-1) {
    max_trials = options_.pose_sample_num_trials;
  }
  std::pair<bool, ViewpointPlanner::Vector3> pos_result = samplePosition(bbox, object_extent, max_trials);
  if (!pos_result.first) {
    return std::make_pair(false, Pose());
  }
  const Pose::Vector3 pos = pos_result.second.cast<Pose::Vector3::Scalar>();
  const Pose::Quaternion orientation = sampleOrientation();
  Pose pose = Pose::createFromImageToWorldTransformation(pos, orientation);
  return std::make_pair(true, pose);
}

const ViewpointPlanner::ViewpointPath& ViewpointPlanner::getBestViewpointPath() const {
  AIT_ASSERT(!viewpoint_paths_.empty());
  const ViewpointPath* best_path = &viewpoint_paths_.front();
  if (viewpoint_paths_.size() > 1) {
    for (auto it = viewpoint_paths_.cbegin() + 1; it != viewpoint_paths_.cend(); ++it) {
      if (it->entries.empty()) {
        continue;
      }
      if (it->acc_objective > best_path->acc_objective) {
        best_path = &(*it);
      }
    }
  }
  return *best_path;
}

/// Acquire lock to acces viewpoint entries (including graph and path) or reset the planning
std::unique_lock<std::mutex> ViewpointPlanner::acquireLock() {
//    return std::move(std::unique_lock<std::mutex>(mutex_));
  return std::unique_lock<std::mutex>(mutex_);
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
  FloatType dist = (pos - bias_bbox.getCenter()).norm();
  FloatType bbox_fov_angle = std::atan(bias_bbox.getMaxExtent() / (2 * dist));
  FloatType angle_stddev = 2 * bbox_fov_angle;
  FloatType angle1 = std::abs(random_.sampleNormal(0, angle_stddev));
  FloatType angle2 = std::abs(random_.sampleNormal(0, angle_stddev));
  Vector3 bbox_direction = (bias_bbox.getCenter() - pos).normalized();
  // Compute viewing direction (i.e. pose z axis)
  Pose::Vector3 viewing_direction = AngleAxis(angle1, Vector3::UnitZ()) * bbox_direction;
  viewing_direction = AngleAxis(angle2, viewing_direction.cross(Vector3::UnitZ())) * viewing_direction;

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
  Pose::Vector3 z_axis(0, 0, 1);
  Pose::Vector3 pose_z_axis = viewing_direction;
  Pose::Vector3 pose_x_axis;
  // If z axis and viewing direction are parallel, set x (right) direction manually
  if (ait::isApproxEqual(std::abs(z_axis.dot(pose_z_axis)), (FloatType)1, kDotProdEqualTolerance )) {
    pose_x_axis = Pose::Vector3(1, 0, 0);
  }
  else {
    pose_x_axis = pose_z_axis.cross(z_axis);
  }
//  std::cout << "viewing_direction=" << viewing_direction << std::endl;
  Pose::Vector3 pose_y_axis = pose_z_axis.cross(pose_x_axis);
  Pose::Matrix3x3 rotation;
  rotation.col(0) = pose_x_axis.normalized();
  rotation.col(1) = pose_y_axis.normalized();
  rotation.col(2) = pose_z_axis.normalized();
  return Pose::Quaternion(rotation);
}

bool ViewpointPlanner::isValidObjectPosition(const Vector3& position, const Vector3& object_extent) {
  BoundingBoxType object_bbox = BoundingBoxType::createFromCenterAndExtent(position, object_extent);
  std::vector<ViewpointPlannerData::OccupiedTreeType::BBoxIntersectionResult> results =
      data_->occupied_bvh_.intersects(object_bbox);
  return results.empty();
}

std::pair<bool, ViewpointPlanner::Vector3> ViewpointPlanner::samplePosition(std::size_t max_trials /*= (std::size_t)-1*/) const {
  return samplePosition(data_->roi_bbox_, drone_extent_, max_trials);
}

std::pair<bool, ViewpointPlanner::Vector3> ViewpointPlanner::samplePosition(const BoundingBoxType& bbox,
    const Vector3& object_extent, std::size_t max_trials /*= (std::size_t)-1*/) const {
  if (max_trials == (std::size_t)-1) {
    max_trials = options_.pose_sample_num_trials;
  }
  for (size_t i = 0; i < max_trials; ++i) {
    Vector3 pos(
        random_.sampleUniform(bbox.getMinimum(0), bbox.getMaximum(0)),
        random_.sampleUniform(bbox.getMinimum(1), bbox.getMaximum(1)),
        random_.sampleUniform(bbox.getMinimum(2), bbox.getMaximum(2)));
    BoundingBoxType object_bbox = BoundingBoxType::createFromCenterAndExtent(pos, object_extent);
    std::vector<ViewpointPlannerData::OccupiedTreeType::BBoxIntersectionResult> results =
        data_->occupied_bvh_.intersects(object_bbox);
    if (results.empty()) {
      return std::make_pair(true, pos);
    }
  }
  return std::make_pair(false, Vector3());
}

void ViewpointPlanner::setVirtualCamera(size_t virtual_width, size_t virtual_height, const Matrix4x4& virtual_intrinsics) {
  virtual_camera_ = PinholeCamera(virtual_width, virtual_height, virtual_intrinsics);
  std::cout << "virtual camera size: " << virtual_width << " x " << virtual_height << std::endl;
}

void ViewpointPlanner::setScaledVirtualCamera(CameraId camera_id, FloatType scale_factor) {
  const PinholeCamera& camera = data_->reconstruction_->getCameras().at(camera_id);
  size_t virtual_width = static_cast<size_t>(scale_factor * camera.width());
  size_t virtual_height = static_cast<size_t>(scale_factor * camera.height());
  reconstruction::CameraMatrix virtual_intrinsics = ait::getScaledIntrinsics(camera.intrinsics(), scale_factor);
  virtual_camera_ = PinholeCamera(virtual_width, virtual_height, virtual_intrinsics);
  std::cout << "Virtual camera size: " << virtual_width << " x " << virtual_height << std::endl;
}

ViewpointPlanner::WeightType ViewpointPlanner::computeObservationInformationFactor(CounterType observation_count) {
//  return std::exp(- 0.1f * observation_count);
  return observation_count == 0 ? 1 : 0;
}

std::unordered_set<Point3DId> ViewpointPlanner::computeProjectedMapPoints(const CameraId camera_id, const Pose& pose,
    FloatType projection_margin) const {
  Viewpoint viewpoint(&data_->reconstruction_->getCameras().at(camera_id), pose, projection_margin);
  std::unordered_set<Point3DId> proj_points = viewpoint.getProjectedPoint3DIds(data_->reconstruction_->getPoints3D());
  return proj_points;
}

std::unordered_set<Point3DId> ViewpointPlanner::computeFilteredMapPoints(const CameraId camera_id, const Pose& pose,
    FloatType projection_margin) const {
  Viewpoint viewpoint(&data_->reconstruction_->getCameras().at(camera_id), pose, projection_margin);
  std::unordered_set<Point3DId> proj_points = viewpoint.getProjectedPoint3DIdsFiltered(data_->reconstruction_->getPoints3D());
  return proj_points;
}

std::unordered_set<Point3DId> ViewpointPlanner::computeVisibleMapPoints(const CameraId camera_id, const Pose& pose,
    FloatType projection_margin) const {
  Viewpoint viewpoint(&data_->reconstruction_->getCameras().at(camera_id), pose, projection_margin);
  std::unordered_set<Point3DId> proj_points = viewpoint.getProjectedPoint3DIds(data_->reconstruction_->getPoints3D());
  removeOccludedPoints(pose, proj_points, kOcclusionDistMarginFactor * data_->octree_->getResolution());
  return proj_points;
}

std::unordered_set<Point3DId> ViewpointPlanner::computeVisibleMapPointsFiltered(const CameraId camera_id, const Pose& pose,
    FloatType projection_margin) const {
  Viewpoint viewpoint(&data_->reconstruction_->getCameras().at(camera_id), pose, projection_margin);
  std::unordered_set<Point3DId> proj_points = viewpoint.getProjectedPoint3DIdsFiltered(data_->reconstruction_->getPoints3D());
  removeOccludedPoints(pose, proj_points, kOcclusionDistMarginFactor * data_->octree_->getResolution());
  return proj_points;
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> ViewpointPlanner::getRaycastHitVoxelsBVH(
    const Pose& pose) const {
  Viewpoint viewpoint(&virtual_camera_, pose);
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results;
  raycast_results.resize(virtual_camera_.width() * virtual_camera_.height());
  ait::Timer timer;
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
      const RayType ray = viewpoint.getCameraRay(x, y);
      float min_range = 0.0f;
      float max_range = 60.0f;
      bvh::Ray bvh_ray;
      bvh_ray.origin = ray.origin();
      bvh_ray.direction = ray.direction();
      std::pair<bool, ViewpointPlannerData::OccupiedTreeType::IntersectionResult> result =
          data_->occupied_bvh_.intersects(bvh_ray, min_range, max_range);
      if (result.first) {
//        if (result.second.depth < octree_->getTreeDepth()) {
//          continue;
//        }
#if !AIT_DEBUG
#pragma omp critical
#endif
        {
//          if (octree_->isNodeUnknown(result.second.node->getObject()->observation_count)) {
          raycast_results[y * virtual_camera_.width() + x] = result.second;
//          }
        }
      }
    }
  }
  std::unordered_map<ViewpointPlannerData::OccupiedTreeType::NodeType*, ViewpointPlannerData::OccupiedTreeType::IntersectionResult>
      raycast_map;
  for (auto it = raycast_results.cbegin(); it != raycast_results.cend(); ++it) {
    if (it->node != nullptr) {
      raycast_map.emplace(it->node, *it);
    }
  }
  raycast_results.clear();
  raycast_results.reserve(raycast_map.size());
  for (auto it = raycast_map.cbegin(); it != raycast_map.cend(); ++it) {
    raycast_results.emplace_back(it->second);
  }
//  std::cout << "Voxels: " << raycast_results.size() << std::endl;
  timer.printTiming("getRaycastHitVoxelsBVH");
  return raycast_results;
}

std::pair<ViewpointPlanner::VoxelWithInformationSet, ViewpointPlanner::FloatType>
ViewpointPlanner::getRaycastHitVoxelsWithInformationScoreBVH(
    const Pose& pose) const {
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results =
      getRaycastHitVoxelsBVH(pose);
  VoxelWithInformationSet voxel_set;
  FloatType total_information = 0;
//  std::vector<FloatType> informations;
  for (auto it = raycast_results.cbegin(); it != raycast_results.cend(); ++it) {
    const ViewpointPlannerData::OccupiedTreeType::IntersectionResult& result = *it;
    WeightType information = computeInformationScore(result);
    voxel_set.insert(VoxelWithInformation(result.node, information));
    total_information += information;
//    informations.push_back(information);
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

float ViewpointPlanner::computeInformationScore(const ViewpointPlannerData::OccupiedTreeType::IntersectionResult& result) const {
  // TODO: Make sure tree is not pruned for occupied voxels.
  // Or consider how to handle larger unknown/known voxels
  CounterType voxel_observation_count = result.node->getObject()->observation_count;
//  if (result.depth < data_->octree_->getTreeDepth()) {
//    voxel_observation_count /= 1 << (data_->octree_->getTreeDepth() - result.depth);
//  }
  WeightType information_factor = computeObservationInformationFactor(voxel_observation_count);
//  WeightType information_factor = 1;
  WeightType weight = result.node->getObject()->weight;
  WeightType information = information_factor * weight;
  return information;
}

void ViewpointPlanner::removeOccludedPoints(const Pose& pose, std::unordered_set<Point3DId>& point3D_ids, FloatType dist_margin) const {
  ait::Timer timer;
  Pose pose_image_to_world = pose.inverse();
  Vector3 eigen_origin = pose_image_to_world.translation();
  octomap::point3d origin(eigen_origin(0), eigen_origin(1), eigen_origin(2));
  octomap::KeyRay key_ray;
  octomap::OcTreeKey origin_key;
  if (!data_->octree_->coordToKeyChecked(origin, origin_key)) {
    point3D_ids.clear();
    return;
  }
  for (auto it = point3D_ids.begin(); it != point3D_ids.end(); ) {
    const Point3D& point3D = data_->reconstruction_->getPoints3D().at(*it);
    octomap::point3d end_point(point3D.pos(0), point3D.pos(1), point3D.pos(2));
    octomap::point3d end_point_from_origin = end_point - origin;
    octomap::OcTreeKey end_point_key;
    bool occluded = false;
    if (!data_->octree_->coordToKeyChecked(end_point, end_point_key)) {
      occluded = true;
    } else {
//                bool success = octree_->computeRayKeys(origin, end_point, key_ray);
      FloatType end_point_dist = end_point_from_origin.norm();
      octomap::point3d hit_point;
      const bool ignore_unknown = true;
      data_->octree_->castRay(origin, end_point_from_origin, hit_point, ignore_unknown, end_point_dist);
      FloatType hit_dist = (hit_point - origin).norm();
      if (end_point_key != data_->octree_->coordToKey(hit_point) && hit_dist + dist_margin < end_point_dist) {
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

std::pair<bool, ViewpointPlanner::Pose>
ViewpointPlanner::sampleSurroundingPose(const Pose& pose) const {
  // Sample position from sphere around pose
  Vector3 sampled_pos;
  bool found_position = false;
  for (size_t i = 0; i < options_.viewpoint_sample_count; ++i) {
    random_.sampleSphericalShell(options_.pose_sample_min_radius, options_.pose_sample_max_radius, &sampled_pos);
    sampled_pos += pose.getWorldPosition();
    BoundingBoxType object_bbox = BoundingBoxType::createFromCenterAndExtent(sampled_pos, drone_extent_);
    std::vector<ViewpointPlannerData::OccupiedTreeType::BBoxIntersectionResult> results =
        data_->occupied_bvh_.intersects(object_bbox);
    if (results.empty()) {
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

bool ViewpointPlanner::generateNextViewpointEntry() {
  const bool verbose = true;

  // Sample viewpoint and add it to the viewpoint graph

//  if (verbose) {
//    std::cout << "Trying to sample viewpoint" << std::endl;
//  }
  std::pair<bool, Pose> sampling_result = sampleSurroundingPose(viewpoint_entries_.cbegin(), viewpoint_entries_.cend());
  if (!sampling_result.first) {
//    if (verbose) {
//      std::cout << "Failed to sample pose." << i << std::endl;
//    }
    return false;
  }

  const Pose& pose = sampling_result.second;

  // Check distance to other viewpoints and discard if too close
  const std::size_t dist_knn = options_.viewpoint_discard_dist_knn;
  const FloatType dist_thres_square = options_.viewpoint_discard_dist_thres_square;
  const std::size_t dist_count_thres = options_.viewpoint_discard_dist_count_thres;
  static std::vector<ViewpointANN::IndexType> knn_indices;
  static std::vector<ViewpointANN::DistanceType> knn_distances;
  knn_indices.resize(dist_knn);
  knn_distances.resize(dist_knn);
  viewpoint_ann_.knnSearch(pose.getWorldPosition(), dist_knn, &knn_indices, &knn_distances);
  std::size_t too_close_count = 0;
//  for (ViewpointANN::IndexType viewpoint_index : knn_indices) {
//    const ViewpointEntry& other_viewpoint = viewpoint_entries_[viewpoint_index];
//    FloatType dist_square = (pose.getWorldPosition() - other_viewpoint.viewpoint.pose().getWorldPosition()).squaredNorm();
  for (ViewpointANN::DistanceType dist_square : knn_distances) {
//    std::cout << "dist_square=" << dist_square << std::endl;
    if (dist_square < dist_thres_square) {
      ++too_close_count;
//      if (verbose) {
//        std::cout << "dist_square=" << dist_square << std::endl;
//      }
      if (too_close_count > dist_count_thres) {
        break;
      }
    }
  }

//  // Test code for approximate nearest neighbor search
//  static std::vector<ViewpointANN::IndexType> knn_indices2;
//  static std::vector<ViewpointANN::DistanceType> knn_distances2;
//  knn_indices2.resize(dist_knn);
//  knn_distances2.resize(dist_knn);
//  viewpoint_ann_.knnSearchExact(pose.getWorldPosition(), dist_knn, &knn_indices2, &knn_distances2);
//  std::size_t too_close_count2 = 0;
//  for (ViewpointANN::IndexType viewpoint_index : knn_indices2) {
//    const ViewpointEntry& other_viewpoint = viewpoint_entries_[viewpoint_index];
//    FloatType dist_square = (pose.getWorldPosition() - other_viewpoint.viewpoint.pose().getWorldPosition()).squaredNorm();
////    std::cout << "dist_square=" << dist_square << std::endl;
//    if (dist_square < dist_thres_square) {
//      ++too_close_count2;
////      if (verbose) {
////        std::cout << "dist_square=" << dist_square << std::endl;
////      }
//      if (too_close_count2 > dist_count_thres) {
//        break;
//      }
//    }
//  }
//  // This is not necessarily true because of approximate nearest neighbor search but for testing we can quickly see if there is a bug.
//  if (too_close_count != too_close_count2) {
//    std::cerr << "TEST ERROR: too_close_count != too_close_count2" << std::endl;
//    std::cerr << "too_close_count=" << too_close_count << ", too_close_count2=" << too_close_count2 << std::endl;
//  }
//  if (too_close_count > too_close_count2) {
//    std::cout << "too_close_count > too_close_count2" << std::endl;
//    for (std::size_t i = 0; i < knn_indices.size(); ++i) {
//      std::cout << "knn_indices[" << i << "]=" << knn_indices[i] << std::endl;
//      std::cout << "knn_distances[" << i << "]=" << knn_distances[i] << std::endl;
//      std::cout << "getPoint(" << i << ")=" << viewpoint_ann_.getPoint(i) << std::endl;
//      std::cout << "viewpoint_entry[" << knn_indices[i] << "]=" << viewpoint_entries_[knn_indices[i]].viewpoint.pose().getWorldPosition().transpose() << std::endl;
//      std::cout << "knn_indices2[" << i << "]=" << knn_indices2[i] << std::endl;
//      std::cout << "knn_distances2[" << i << "]=" << knn_distances2[i] << std::endl;
//      std::cout << "viewpoint_entry[" << knn_indices2[i] << "]=" << viewpoint_entries_[knn_indices2[i]].viewpoint.pose().getWorldPosition().transpose() << std::endl;
//    }
//  }
//  AIT_ASSERT(too_close_count <= too_close_count2);
//  // End of testing code
//  if (verbose) {
//    std::cout << "too_close_count=" << too_close_count << std::endl;
//  }

  if (too_close_count > dist_count_thres) {
//    if (verbose) {
//      std::cout << "too_close_count > dist_count_thres" << std::endl;
//    }
    return false;
  }

  // Compute observed voxels and information and discard if too few voxels or too little information
  std::pair<VoxelWithInformationSet, FloatType> raycast_result =
      getRaycastHitVoxelsWithInformationScoreBVH(pose);
  VoxelWithInformationSet& voxel_set = raycast_result.first;
  FloatType total_information = raycast_result.second;
//    if (verbose) {
//      std::cout << "voxel_set.size()=" << voxel_set.size() << std::endl;
//    }
  if (voxel_set.size() < options_.viewpoint_min_voxel_count) {
    if (verbose) {
      std::cout << "voxel_set.size() < options_.viewpoint_min_voxel_count" << std::endl;
    }
    return false;
  }
//  if (verbose) {
//    std::cout << "total_information=" << total_information << std::endl;
//  }
  if (total_information < options_.viewpoint_min_information) {
    if (verbose) {
      std::cout << "total_information < options_.viewpoint_min_information" << std::endl;
    }
    return false;
  }

//  if (verbose) {
//    std::cout << "pose=" << pose << ", voxel_set=" << voxel_set.size() << ", total_information=" << total_information << std::endl;
//  }

  addViewpointEntry(
      ViewpointEntry(Viewpoint(&virtual_camera_, pose), total_information, std::move(voxel_set)));

  return true;
}

void ViewpointPlanner::computeViewpointMotions() {
  std::cout << "Computing motions on viewpoint graph" << std::endl;
  for (ViewpointEntryIndex from_index : viewpoint_graph_) {
    if (from_index < num_real_viewpoints_) {
      continue;
    }
    std::vector<ViewpointPlanner::ViewpointMotion> motions = findViewpointMotions(from_index);
    std::cout << "Found " << motions.size() << " connections from viewpoint " << from_index << std::endl;

    std::unique_lock<std::mutex> lock(mutex_);
    for (const ViewpointMotion& motion : motions) {
      AIT_ASSERT(motion.from_index == from_index);
      // TODO: Add symmetric edges?
      viewpoint_graph_.addEdge(from_index, motion.to_index, motion.cost);
      viewpoint_graph_.addEdge(motion.to_index, from_index, motion.cost);
    }
    lock.unlock();
  }

  std::cout << "Densifying edges on viewpoint graph" << std::endl;
  // Densify viewpoint motions
  std::stack<std::tuple<ViewpointEntryIndex, FloatType, std::size_t>> node_stack;
  for (const ViewpointEntryIndex index : viewpoint_graph_) {
    node_stack.push(std::make_tuple(index, 0, 0));
  }
  std::vector<std::tuple<ViewpointEntryIndex, ViewpointEntryIndex, FloatType>> new_edges;
  while (!node_stack.empty()) {
    ViewpointEntryIndex index = std::get<0>(node_stack.top());
    FloatType acc_motion_distance = std::get<1>(node_stack.top());
    std::size_t depth = std::get<2>(node_stack.top());
    node_stack.pop();
    if (depth >= options_.viewpoint_motion_densification_max_depth) {
      continue;
    }
    for (const auto& edge : viewpoint_graph_.getEdges(index)) {
      ViewpointEntryIndex other_index = edge.first;
      FloatType motion_distance = edge.second;
      node_stack.push(std::make_tuple(other_index, acc_motion_distance + motion_distance, depth + 1));
      if (depth >= 2) {
        new_edges.push_back(std::make_tuple(index, other_index, acc_motion_distance + motion_distance));
      }
    }
  }
  for (const auto& tuple : new_edges) {
    viewpoint_graph_.addEdge(std::get<0>(tuple), std::get<1>(tuple), std::get<2>(tuple));
  }
  std::cout << "Done" << std::endl;
}

ViewpointPlanner::ViewpointEntryIndex ViewpointPlanner::addViewpointEntryWithoutLock(ViewpointEntry&& viewpoint_entry) {
  ViewpointEntryIndex viewpoint_index = viewpoint_entries_.size();
  viewpoint_ann_.addPoint(viewpoint_entry.viewpoint.pose().getWorldPosition());
  viewpoint_entries_.emplace_back(std::move(viewpoint_entry));
  viewpoint_graph_.addNode(viewpoint_index);
  return viewpoint_index;
}

void ViewpointPlanner::addViewpointEntry(ViewpointEntry&& viewpoint_entry) {
  std::unique_lock<std::mutex> lock(mutex_);
  addViewpointEntryWithoutLock(std::move(viewpoint_entry));
  lock.unlock();
}

std::vector<ViewpointPlanner::ViewpointMotion>
ViewpointPlanner::findViewpointMotions(const ViewpointEntryIndex from_index) {
  ait::Timer timer;
  const ViewpointEntry& from_viewpoint = viewpoint_entries_[from_index];
  // Find motion to other viewpoints in the graph
  const std::size_t dist_knn = options_.viewpoint_motion_max_neighbors;
  const FloatType max_dist_square = options_.viewpoint_motion_max_dist_square;
  static std::vector<ViewpointANN::IndexType> knn_indices;
  static std::vector<ViewpointANN::DistanceType> knn_distances;
  knn_indices.resize(dist_knn);
  knn_distances.resize(dist_knn);
  viewpoint_ann_.knnSearch(from_viewpoint.viewpoint.pose().getWorldPosition(), dist_knn, &knn_indices, &knn_distances);
  std::size_t num_connections = 0;
  std::vector<ViewpointMotion> motions;
#if !AIT_DEBUG
#pragma omp parallel for
#endif
  for (std::size_t i = 0; i < knn_indices.size(); ++i) {
    ViewpointANN::IndexType to_index = knn_indices[i];
    ViewpointANN::DistanceType dist_square = knn_distances[i];
//    std::cout << "dist_square=" << dist_square << std::endl;
    if (dist_square >= max_dist_square) {
      continue;
    }
    // Do not consider real point for finding motion paths
    if (to_index < num_real_viewpoints_) {
      continue;
    }
    const ViewpointEntry& to_viewpoint = viewpoint_entries_[to_index];
    std::pair<bool, FloatType> motion_result = motion_planner_.findMotion(from_viewpoint.viewpoint.pose(), to_viewpoint.viewpoint.pose());
    if (motion_result.first) {
#if !AIT_DEBUG
#pragma omp critical
#endif
      {
        ++num_connections;
        FloatType motion_cost = motion_result.second;
        if (!ait::isApproxGreaterEqual(motion_cost * motion_cost, dist_square, 1e-2f)) {
          FloatType dist_square2 = (from_viewpoint.viewpoint.pose().getWorldPosition() - to_viewpoint.viewpoint.pose().getWorldPosition()).squaredNorm();
          std::cout << "motion_cost=" << motion_cost << std::endl;
          std::cout << "motion_cost*motion_cost=" << motion_cost * motion_cost << std::endl;
          std::cout << "dist_square=" << dist_square << std::endl;
          std::cout << "dist_square2=" << dist_square2 << std::endl;
          std::cout << "motion_cost * motion_cost - dist_square=" << motion_cost * motion_cost - dist_square << std::endl;
        }
        AIT_ASSERT(ait::isApproxGreaterEqual(motion_cost * motion_cost, dist_square, 1e-2f));
        motions.emplace_back(from_index, to_index, motion_cost);
      }
    }
  }
  timer.printTimingMs("ViewpointPlanner::findMotion()");
  return motions;
}

bool ViewpointPlanner::findNextViewpointPathEntry(ViewpointPath* viewpoint_path, const FloatType alpha, const FloatType beta) {
  const bool verbose = false;

  ViewpointPathEntry best_path_entry;

  // TODO: Make proper iterator for graph edges.

  ViewpointEntryIndex from_index = viewpoint_path->entries.back().viewpoint_index;
  // Run through the neighbors and select the best next viewpoint
  for (const auto& edge : viewpoint_graph_.getEdges(from_index)) {
    ViewpointEntryIndex to_index = edge.first;
    const ViewpointEntry& to_viewpoint = viewpoint_entries_[to_index];
    VoxelWithInformationSet difference_set = ait::computeSetDifference(to_viewpoint.voxel_set, viewpoint_path->observed_voxel_set);
//    std::cout << "  j=" << j << ", next_node.voxel_set.size()= " << next_node.voxel_set.size() << std::endl;
//    std::cout << "  j=" << j << ", difference_set.size()= " << difference_set.size() << std::endl;
    FloatType new_information = std::accumulate(difference_set.cbegin(), difference_set.cend(),
        FloatType { 0 }, [](const FloatType& value, const VoxelWithInformation& voxel) {
          return value + voxel.information;
    });
//      if (verbose) {
//        std::cout << "difference_set.size()=" << difference_set.size() <<
//            ", new_information=" << new_information << std::endl;
//      }
    float motion_distance = viewpoint_graph_.getWeight(from_index, to_index);
    FloatType new_objective = new_information - alpha - beta * motion_distance * motion_distance;
//    std::cout << "  j=" << j << ", new_information=" << new_information << std::endl;
    if (verbose) {
      std::cout << "to_index=" << to_index
          << ", voxel_set.size()=" << to_viewpoint.voxel_set.size()
          << ", new_information=" << new_information
          << ", motion_distance=" << motion_distance
          << ", new_objective=" << new_objective << std::endl;
    }
    if (new_objective > best_path_entry.local_objective) {
      best_path_entry.viewpoint_index = to_index;
      best_path_entry.local_information = new_information;
      best_path_entry.local_motion_distance = motion_distance;
      best_path_entry.local_objective = new_objective;
    }
  }

  if (best_path_entry.viewpoint_index == (ViewpointEntryIndex)-1) {
//    std::cout << "viewpoint_path->size()=" << viewpoint_path->size() << std::endl;
//    std::cout << "viewpoint_graph_.size()=" << viewpoint_graph_.size() << std::endl;
    if (!viewpoint_path->entries.empty()) {
//      ViewpointEntryIndex from_index = viewpoint_path->entries.back().viewpoint_index;
//      std::cout << "viewpoint_graph_.getEdges(from_index).size()=" << viewpoint_graph_.getEdges(from_index).size() << std::endl;
    }
    return false;
  }
  AIT_ASSERT(best_path_entry.viewpoint_index != (ViewpointEntryIndex)-1);

  if (best_path_entry.local_objective <= 0) {
    std::cout << "No improvement in objective" << std::endl;
    return false;
  }

  const ViewpointEntry& best_viewpoint_entry = viewpoint_entries_[best_path_entry.viewpoint_index];
  if (verbose) {
    // TODO: remove this set difference (duplicate anyway)
    VoxelWithInformationSet difference_set = ait::computeSetDifference(best_viewpoint_entry.voxel_set, viewpoint_path->observed_voxel_set);
    std::cout << "Found viewpoint with local_information=" << best_path_entry.local_information <<
        ", total_information=" << best_viewpoint_entry.total_information <<
        ", total_voxels=" << best_viewpoint_entry.voxel_set.size() <<
        ", new_voxels=" << difference_set.size() << std::endl;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  viewpoint_path->entries.emplace_back(std::move(best_path_entry));
  viewpoint_path->observed_voxel_set.insert(best_viewpoint_entry.voxel_set.cbegin(), best_viewpoint_entry.voxel_set.cend());
  viewpoint_path->acc_information += best_path_entry.local_information;
  viewpoint_path->acc_motion_distance += best_path_entry.local_motion_distance;
  viewpoint_path->acc_objective += best_path_entry.local_objective;
  viewpoint_path->entries.back().acc_information = viewpoint_path->acc_information;
  viewpoint_path->entries.back().acc_motion_distance = viewpoint_path->acc_motion_distance;
  viewpoint_path->entries.back().acc_objective = viewpoint_path->acc_objective;
  lock.unlock();
//  if (verbose) {
//    std::cout << "new information: " <<  best_path_entry.information <<
//        ", information: " << best_viewpoint_entry.total_information << std::endl;
//  }

  if (verbose) {
    std::cout << "Done." << std::endl;
  }

  if (verbose) {
    std::cout << "Path: " << std::endl;
    for (std::size_t i = 0; i < viewpoint_path->entries.size(); ++i) {
      const ViewpointPathEntry& path_entry = viewpoint_path->entries[i];
      const ViewpointEntry& viewpoint_entry = viewpoint_entries_[path_entry.viewpoint_index];
      std::cout << "  viewpoint_index: " << path_entry.viewpoint_index
          << ", pose[" << i << "]: " << viewpoint_entry.viewpoint.pose().getWorldPosition().transpose()
          << ", local_information: " << path_entry.local_information
          << ", global_information: " << path_entry.acc_information
          << ", local_motion_distance: " << path_entry.local_motion_distance
          << ", global_motion_distance: " << path_entry.acc_motion_distance
          << ", local_objective: " << path_entry.local_objective
          << ", global_objective: " << path_entry.acc_objective << std::endl;
    }
  }

  return true;
}

bool ViewpointPlanner::findNextViewpointPathEntries(const FloatType alpha, const FloatType beta) {
  if (viewpoint_paths_.front().entries.empty()) {
    // Initially find the best viewpoints as starting points for the viewpoint paths

    if (viewpoint_paths_.size() > viewpoint_entries_.size()) {
      // Make sure we don't expect too many viewpoints
      viewpoint_paths_.resize(viewpoint_entries_.size());
    }

    std::vector<ViewpointPathEntry> best_path_entries(viewpoint_paths_.size());
    // Find the overall best viewpoints as starting points
    for (ViewpointEntryIndex viewpoint_index : viewpoint_graph_) {
      const ViewpointEntry& viewpoint_entry = viewpoint_entries_[viewpoint_index];
      FloatType new_information = viewpoint_entry.total_information;
      FloatType new_objective = new_information - alpha;
      if (new_objective > best_path_entries.back().local_objective) {
        std::cout << "new_objective: " << new_objective << std::endl;
        best_path_entries.back().viewpoint_index = viewpoint_index;
        best_path_entries.back().local_information = new_information;
        best_path_entries.back().local_motion_distance = 0;
        best_path_entries.back().local_objective = new_objective;
        best_path_entries.back().acc_information = new_information;
        best_path_entries.back().acc_motion_distance = 0;
        best_path_entries.back().acc_objective = new_objective;
        if (viewpoint_paths_.size() > 1) {
          for (auto it = best_path_entries.rbegin() + 1; it != best_path_entries.rend(); ++it) {
            if (it->acc_objective < (it-1)->acc_objective) {
              swap(*it, *(it-1));
            }
            else {
              break;
            }
          }
        }
      }
    }

    std::unique_lock<std::mutex> lock(mutex_);
    for (auto it = viewpoint_paths_.begin(); it != viewpoint_paths_.end(); ++it) {
      const ViewpointPathEntry& best_path_entry = best_path_entries[it - viewpoint_paths_.begin()];
      const ViewpointEntry& best_viewpoint_entry = viewpoint_entries_[best_path_entry.viewpoint_index];
      it->acc_information = best_path_entry.acc_information;
      it->acc_motion_distance = best_path_entry.acc_motion_distance;
      it->acc_objective = best_path_entry.acc_objective;
      it->entries.emplace_back(std::move(best_path_entry));
      it->observed_voxel_set.insert(best_viewpoint_entry.voxel_set.cbegin(), best_viewpoint_entry.voxel_set.cend());
      std::cout << "acc_objective=" << it->acc_objective << std::endl;
    }
    lock.unlock();

    return true;
  }

  std::size_t changes = 0;
#pragma omp parallel for \
  reduction(+:changes)
  for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
    ViewpointPath& viewpoint_path = viewpoint_paths_[i];
    changes += findNextViewpointPathEntry(&viewpoint_path, alpha, beta);
  }
  std::cout << "changes=" << changes << ", alpha=" << alpha << ", beta=" << beta << std::endl;
  if (changes == 0) {
    return false;
  }
  else {
    return true;
  }
}

bool ViewpointPlanner::findNextViewpointPathEntries() {
  return findNextViewpointPathEntries(options_.objective_parameter_alpha, options_.objective_parameter_beta);
}

void ViewpointPlanner::run() {
  // TODO:
  // Initialize feature_viewpoint_map_ from existing real images
//  for (std::size_t i = 0; i < data_->poisson_mesh_->)

  // TODO:
  // Initialize viewpoint nearest neighbor index

  return;

    // Check how many map points fall in an occupied voxel
//        for (size_t max_depth = 1; max_depth <= octree_->getTreeDepth(); ++max_depth) {
//            size_t num_map_point_nodes = 0;
//            FloatType average_occupancy_level = 0.0;
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
  for (const auto& entry : data_->reconstruction_->getImages()) {
    ait::Timer timer2;
    const ImageColmap& image = entry.second;
    const PinholeCameraColmap& camera = data_->reconstruction_->getCameras().at(image.camera_id());
    std::cout << "image_id=" << image.id() << std::endl;
    std::cout << "  features: " << image.features().size() << std::endl;
    const FloatType projection_margin = 0;
    std::unordered_set<Point3DId> proj_points = computeProjectedMapPoints(camera.id(), image.pose(), projection_margin);
    std::unordered_set<Point3DId> filtered_points = computeFilteredMapPoints(camera.id(), image.pose(), projection_margin);
    std::unordered_set<Point3DId> visible_points = computeVisibleMapPoints(camera.id(), image.pose(), projection_margin);
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
    for (const auto& feature : image.features()) {
      if (feature.point3d_id != reconstruction::invalid_point3d_id) {
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
