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

Viewpoint::Viewpoint(FloatType projection_margin /*= DEFAULT_PROJECTION_MARGIN*/)
: camera_(nullptr), projection_margin_(projection_margin),
  transformation_world_to_image_(pose_.getTransformationWorldToImage()) {}

Viewpoint::Viewpoint(const PinholeCamera* camera, const Pose& pose, FloatType projection_margin /*= DEFAULT_PROJECTION_MARGIN*/)
: camera_(camera), pose_(pose), projection_margin_(projection_margin),
  transformation_world_to_image_(pose_.getTransformationWorldToImage()) {
  if (!pose.isValid()) {
    throw AIT_EXCEPTION("Received invalid pose");
  }
}

Viewpoint::Viewpoint(const Viewpoint& other)
: camera_(other.camera_), pose_(other.pose_),
  projection_margin_(other.projection_margin_),
  transformation_world_to_image_(other.transformation_world_to_image_) {}

Viewpoint::Viewpoint(Viewpoint&& other)
: camera_(other.camera_), pose_(std::move(other.pose_)),
  projection_margin_(other.projection_margin_),
  transformation_world_to_image_(std::move(other.transformation_world_to_image_)) {}

Viewpoint& Viewpoint::operator=(const Viewpoint& other) {
  if (this != &other) {
    camera_ = other.camera_;
    pose_ = other.pose_;
    projection_margin_ = other.projection_margin_;
    transformation_world_to_image_ = other.transformation_world_to_image_;
  }
  return *this;
}

Viewpoint& Viewpoint::operator=(Viewpoint&& other) {
  if (this != &other) {
    camera_ = other.camera_;
    pose_ = std::move(other.pose_);
    projection_margin_ = other.projection_margin_;
    transformation_world_to_image_ = std::move(other.transformation_world_to_image_);
  }
  return *this;
}

Viewpoint::FloatType Viewpoint::getDistanceTo(const Viewpoint& other) const {
  return pose_.getDistanceTo(other.pose_);
}

Viewpoint::FloatType Viewpoint::getDistanceTo(const Viewpoint& other, FloatType rotation_factor) const {
  return pose_.getDistanceTo(other.pose_, rotation_factor);
}

std::unordered_set<Point3DId> Viewpoint::getProjectedPoint3DIds(const SparseReconstruction::Point3DMapType& points) const {
  ait::Timer timer;
  std::unordered_set<Point3DId> proj_point_ids;
  for (const auto& entry : points) {
    const Point3DId point_id = entry.first;
    const Point3D& point_world = entry.second;
    Vector2 point_image = projectWorldPointIntoImage(point_world.pos);
    bool projected = camera_->isPointInViewport(point_image, projection_margin_);
    if (projected) {
      proj_point_ids.insert(point_id);
    }
  }
  timer.printTiming("getProjectedPoint3DIds");
  return proj_point_ids;
}

Viewpoint::RayType Viewpoint::getCameraRay(FloatType x, FloatType y) const {
  Vector3 origin = pose_.getWorldPosition();
  // TODO: Ray origin at projection center or on "image plane"?
  Vector3 direction_camera = camera_->getCameraRay(x, y);
//  Vector3 direction_camera = camera_->unprojectPoint(x, y, 1.0);
  Vector3 direction = pose_.getTransformationImageToWorld().topLeftCorner<3, 3>() * direction_camera;
//  origin = origin + direction;
  return RayType(origin, direction);
}

std::unordered_set<Point3DId> Viewpoint::getProjectedPoint3DIdsFiltered(const SparseReconstruction::Point3DMapType& points) const {
  ait::Timer timer;
  std::unordered_set<Point3DId> proj_point_ids;
  for (const auto& entry : points) {
    const Point3DId point_id = entry.first;
    const Point3D& point_world = entry.second;
    Vector2 point_image = projectWorldPointIntoImage(point_world.getPosition());
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
  std::tuple<FloatType, Vector3> result = ait::computeDistanceAndDirection(point.getPosition(), pose_.getWorldPosition());
  FloatType dist_deviation = std::abs(std::get<0>(result) - statistics.averageDistance());
  if (dist_deviation > MAX_DISTANCE_DEVIATION_BY_STDDEV * statistics.stddevDistance()) {
    return true;
  }
  FloatType one_minus_dot_product = 1 - std::get<1>(result).dot(point.getNormal());
  FloatType one_minus_dot_deviation = std::abs(one_minus_dot_product);
  if (one_minus_dot_deviation > MAX_NORMAL_DEVIATION_BY_STDDEV * statistics.stddevOneMinusDotProduct()) {
    return true;
  }
  return false;
}

Viewpoint::Vector2 Viewpoint::projectWorldPointIntoImage(const Vector3& point_world) const {
  Vector3 point_camera  = transformation_world_to_image_ * point_world.homogeneous();
  Vector2 point_image = camera_->projectPoint(point_camera);
  return point_image;
}


ViewpointPlanner::ViewpointPlanner(const Options* options, std::unique_ptr<ViewpointPlannerData> data)
: options_(*options), data_(std::move(data)) {
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

  reset();
}

void ViewpointPlanner::reset() {
  std::unique_lock<std::mutex> lock(mutex_);
  viewpoint_entries_.clear();
  viewpoint_ann_.clear();
  observed_voxel_set_.clear();
  viewpoint_graph_.clear();
  viewpoint_path_.clear();
  feature_viewpoint_map_.clear();
  lock.unlock();

  // Initialize viewpoint graph from existing real images
  std::cout << "Adding previous camera viewpoints to viewpoint graph" << std::endl;
  std::vector<Vector3> ann_points;
  for (const auto& entry : data_->reconstruction_->getImages()) {
    const Pose& pose = entry.second.pose();
//    std::cout << "  image id=" << entry.first << ", pose=" << pose << std::endl;
//    std::pair<VoxelWithInformationSet, FloatType> raycast_result =
//        getRaycastHitVoxelsWithInformationScoreBVH(pose);
//    VoxelWithInformationSet& voxel_set = raycast_result.first;
//    FloatType total_information = raycast_result.second;
//    std::cout << "voxel_set: " << voxel_set.size() << ", total_information: " << total_information << std::endl;
    FloatType total_information = 0;
    VoxelWithInformationSet voxel_set;
    std::unique_lock<std::mutex> lock(mutex_);
    ViewpointEntryIndex viewpoint_index = viewpoint_entries_.size();
    viewpoint_entries_.emplace_back(Viewpoint(&virtual_camera_, pose), total_information, std::move(voxel_set));
    ann_points.push_back(pose.getWorldPosition());
    // TODO: Initial viewpoints are in the graph for initial sampling.
    // Maybe an explicit start position is better?
    viewpoint_graph_.addNode(viewpoint_index);
    lock.unlock();
  }
  // Initialize viewpoint nearest neighbor index
  viewpoint_ann_.initIndex(ann_points.cbegin(), ann_points.cend());
  std::cout << "initialized viewpoint graph with " << viewpoint_graph_.numOfNodes() << " viewpoints" << std::endl;

  for (std::size_t i = 0; i < ann_points.size(); ++i) {
    std::cout << "ann_points1[" << i << "]=" << ann_points[i].transpose() << std::endl;
    std::cout << "ann_points2[" << i << "]=" << viewpoint_ann_.getPoint(i).transpose() << std::endl;
  }
//  AIT_ASSERT(false);

  // TODO: Initialize feature_viewpoint_map_ from existing real images
//  for (std::size_t i = 0; i < data_->poisson_mesh_->)
}

void ViewpointPlanner::resetViewpointPath() {
  std::unique_lock<std::mutex> lock(mutex_);
  observed_voxel_set_.clear();
  viewpoint_path_.clear();
  lock.unlock();
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

ViewpointPlanner::WeightType ViewpointPlanner::computeObservationWeightFactor(CounterType observation_count) {
  return 1 - std::exp(- 0.5f * observation_count);
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

//FloatType ViewpointPlanner::computeInformationScore(const CameraId camera_id, const Pose& pose_world_to_image) const {
//  ait::Timer timer;
//  bool ignore_unknown = true;
//  FloatType score = rayCastAccumulate<FloatType>(camera_id, pose_world_to_image, ignore_unknown,
//      [](bool hit, const octomap::point3d& hit_point) -> FloatType {
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

std::vector<std::pair<ViewpointPlanner::ConstTreeNavigatorType, float>> ViewpointPlanner::getRaycastHitVoxels(
    const Pose& pose_world_to_image) const {
  Viewpoint viewpoint(&virtual_camera_, pose_world_to_image);
  ait::Timer timer;
  Pose pose_image_to_world = viewpoint.pose().inverse();
  Vector3 eigen_origin = pose_image_to_world.translation();
  octomap::point3d origin(eigen_origin(0), eigen_origin(1), eigen_origin(2));
  octomap::OcTreeKey origin_key;
  std::vector<std::pair<ConstTreeNavigatorType, float>> raycast_voxels;
  if (!data_->octree_->coordToKeyChecked(origin, origin_key)) {
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
      const RayType ray = viewpoint.getCameraRay(x, y);
      float min_range = 0.0f;
      float max_range = 60.0f;
      OccupancyMapType::RayCastData ray_cast_result;
      bool result = data_->octree_->castRayFast<OccupancyMapType::CollisionUnknownOrOccupied>(
          ray.origin(), ray.direction(), &ray_cast_result, min_range, max_range);
      if (result) {
        CounterType voxel_observation_count = ray_cast_result.end_node->getObservationCount();
        if (ray_cast_result.end_depth < data_->octree_->getTreeDepth()) {
          voxel_observation_count /= 1 << (data_->octree_->getTreeDepth() - ray_cast_result.end_depth);
        }
        WeightType information_factor = computeObservationInformationFactor(voxel_observation_count);
        WeightType weight = ray_cast_result.end_node->getWeight();
        WeightType information = information_factor * weight;
#if !AIT_DEBUG
#pragma omp critical
#endif
        {
        raycast_voxels.push_back(std::make_pair(
            ConstTreeNavigatorType(data_->octree_.get(), ray_cast_result.end_key, ray_cast_result.end_node, ray_cast_result.end_depth),
            information));
        }
      }
    }
  }
  timer.printTiming("getRaycastHitVoxels");
  return raycast_voxels;
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

float ViewpointPlanner::computeInformationScoreBVH(
    const Pose& pose) const {
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results =
      getRaycastHitVoxelsBVH(pose);
  FloatType score = 0;
  FloatType unknown_score = 0;
  FloatType unknown_count = 0;
  std::vector<FloatType> informations;
  for (auto it = raycast_results.cbegin(); it != raycast_results.cend(); ++it) {
    const ViewpointPlannerData::OccupiedTreeType::IntersectionResult& result = *it;
    WeightType information = computeInformationScore(result);
    informations.push_back(information);
    score += information;
    if (data_->octree_->isNodeUnknown(result.node->getObject()->observation_count)) {
      unknown_count += 1;
      unknown_score += information;
    }
  }
  score /= virtual_camera_.width() * virtual_camera_.height();
  if (!informations.empty()) {
    auto result = ait::computeHistogram(informations, 10);
    std::cout << "Histogram: " << std::endl;
    for (size_t i = 0; i < result.first.size(); ++i) {
      std::cout << "  # <= " << result.second[i] << ": " << result.first[i] << std::endl;
    }
  }
  return score;
}

float ViewpointPlanner::computeInformationScore(const Pose& pose_world_to_image) const {
  Viewpoint viewpoint(&virtual_camera_, pose_world_to_image);
  ait::Timer timer;
  Pose pose_image_to_world = viewpoint.pose().inverse();
  Vector3 eigen_origin = pose_image_to_world.translation();
  octomap::point3d origin(eigen_origin(0), eigen_origin(1), eigen_origin(2));
  octomap::OcTreeKey origin_key;
  if (!data_->octree_->coordToKeyChecked(origin, origin_key)) {
    std::cout << "WARNING: Requesting ray cast for out of bounds viewpoint" << std::endl;
    return std::numeric_limits<FloatType>::quiet_NaN();
  }
  FloatType score = 0;
  FloatType unknown_score = 0;
  FloatType unknown_count = 0;
  for (size_t y = 0; y < virtual_camera_.height(); ++y) {
//  size_t y = virtual_camera_.height() / 2; {
#if !AIT_DEBUG
#pragma omp parallel for
#endif
    for (size_t x = 0; x < virtual_camera_.width(); ++x) {
//    size_t x = virtual_camera_.width() / 2; {
      const RayType ray = viewpoint.getCameraRay(x, y);
      FloatType min_range = 5.0;
      FloatType max_range = 60.0;
      OccupancyMapType::RayCastData ray_cast_result;
      bool result = data_->octree_->castRayFast<OccupancyMapType::CollisionUnknownOrOccupied>(
          ray.origin(), ray.direction(), &ray_cast_result, min_range, max_range);
//      if (result && octree_->isNodeUnknown(ray_cast_result.end_node)) {
      if (result) {
//        std::cout << "origin: " << ray.origin().transpose() << ", node_pos: " << end_point.transpose() << std::endl;
//        std::cout << "occupancy: " << end_node->getOccupancy() << ", observations: " << end_node->getObservationCount() << std::endl;
        CounterType voxel_observation_count = ray_cast_result.end_node->getObservationCount();
        if (ray_cast_result.end_depth < data_->octree_->getTreeDepth()) {
          voxel_observation_count /= 1 << (data_->octree_->getTreeDepth() - ray_cast_result.end_depth);
        }
        WeightType information_factor = computeObservationInformationFactor(voxel_observation_count);
        WeightType weight = ray_cast_result.end_node->getWeight();
#if !AIT_DEBUG
#pragma omp critical
#endif
        {
//        score += 1;
        score += information_factor * weight;
        if (data_->octree_->isNodeUnknown(ray_cast_result.end_node)) {
          unknown_count += 1;
          unknown_score += weight;
        }
        }
      }
//      std::cout << "x=" << x << ", y=" << y << ", result=" << result << std::endl;
//      std::cout << "x=" << x << ", y=" << y << ", origin=" << origin << " direction=" << ray.direction() << std::endl;
    }
  }
  score /= virtual_camera_.width() * virtual_camera_.height();
  timer.printTiming("computeInformationScore");
  std::cout << "Unknown voxels visible: " << unknown_count << std::endl;
  std::cout << "Unknown voxels score: " << unknown_score << std::endl;
  return score;
}

void ViewpointPlanner::removeOccludedPoints(const Pose& pose_world_to_image, std::unordered_set<Point3DId>& point3D_ids, FloatType dist_margin) const {
  ait::Timer timer;
  Pose pose_image_to_world = pose_world_to_image.inverse();
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

//  if (verbose) {
//    std::cout << "observed_voxel_set_.size()=" << observed_voxel_set_.size() << std::endl;
//  }

  // Sample viewpoint and add it to the viewpoint graph

//  if (verbose) {
//    std::cout << "Trying to sample viewpoint" << std::endl;
//  }
  std::pair<bool, Pose> sampling_result = sampleSurroundingPoseFromEntries(viewpoint_graph_.cbegin(), viewpoint_graph_.cend());
  if (!sampling_result.first) {
//    if (verbose) {
//      std::cout << "Failed to sample pose." << i << std::endl;
//    }
    return false;
  }

  const Pose& pose = sampling_result.second;

  // Check distance to other viewpoints and discard if too close
  std::size_t dist_knn = options_.viewpoint_discard_dist_knn;
  FloatType dist_thres_square = options_.viewpoint_discard_dist_thres_square;
  std::size_t dist_count_thres = options_.viewpoint_discard_dist_count_thres;
  static std::vector<ViewpointANN::IndexType> knn_indices;
  static std::vector<ViewpointANN::DistanceType> knn_distances;
  knn_indices.resize(dist_knn);
  knn_distances.resize(dist_knn);
  viewpoint_ann_.knnSearch(pose.getWorldPosition(), dist_knn, &knn_indices, &knn_distances);
  std::size_t too_close_count = 0;
  for (ViewpointANN::IndexType viewpoint_index : knn_indices) {
    const ViewpointEntry& other_viewpoint = viewpoint_entries_[viewpoint_index];
    FloatType dist_square = (pose.getWorldPosition() - other_viewpoint.viewpoint.pose().getWorldPosition()).squaredNorm();
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

  std::unique_lock<std::mutex> lock(mutex_);
  ViewpointEntryIndex viewpoint_index = viewpoint_entries_.size();
  viewpoint_entries_.emplace_back(Viewpoint(&virtual_camera_, pose), total_information, std::move(voxel_set));
  viewpoint_ann_.addPoint(pose.getWorldPosition());
  viewpoint_graph_.addNode(viewpoint_index);
  lock.unlock();

  return true;
}

bool ViewpointPlanner::findNextViewpointPathEntry() {
  const bool verbose = true;

  // Now run through the viewpoint graph and select the best next viewpoint
  ViewpointPathEntry best_path_entry((ViewpointEntryIndex)-1, std::numeric_limits<FloatType>::lowest());
  for (ViewpointEntryIndex viewpoint_index : viewpoint_graph_) {
    const ViewpointEntry& viewpoint_entry = viewpoint_entries_[viewpoint_index];
    VoxelWithInformationSet difference_set = ait::computeSetDifference(viewpoint_entry.voxel_set, observed_voxel_set_);
//    std::cout << "  j=" << j << ", next_node.voxel_set.size()= " << next_node.voxel_set.size() << std::endl;
//    std::cout << "  j=" << j << ", difference_set.size()= " << difference_set.size() << std::endl;
    FloatType new_information = std::accumulate(difference_set.cbegin(), difference_set.cend(),
        FloatType { 0 }, [](const FloatType& value, const VoxelWithInformation& voxel) {
          return value + voxel.information;
    });
    if (verbose) {
      std::cout << "difference_set.size()=" << difference_set.size() <<
          ", new_information=" << new_information << std::endl;
    }
//    std::cout << "  j=" << j << ", new_information=" << new_information << std::endl;
    if (new_information > best_path_entry.information) {
      best_path_entry = ViewpointPathEntry(viewpoint_index, new_information);
    }
  }

  // TODO: remove this set difference (duplicate anyway)
  const ViewpointEntry& best_viewpoint_entry = viewpoint_entries_[best_path_entry.viewpoint_index];
  VoxelWithInformationSet difference_set = ait::computeSetDifference(best_viewpoint_entry.voxel_set, observed_voxel_set_);
  if (verbose) {
    std::cout << "Found viewpoint with new_information=" << best_path_entry.information <<
        ", total_information=" << best_viewpoint_entry.total_information <<
        ", total_voxels=" << best_viewpoint_entry.voxel_set.size() <<
        ", new_voxels=" << difference_set.size() << std::endl;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  viewpoint_path_.push_back(best_path_entry);
  observed_voxel_set_.insert(best_viewpoint_entry.voxel_set.cbegin(), best_viewpoint_entry.voxel_set.cend());
  lock.unlock();
//  if (verbose) {
//    std::cout << "new information: " <<  best_path_entry.information <<
//        ", information: " << best_viewpoint_entry.total_information << std::endl;
//  }

  if (verbose) {
    std::cout << "Done." << std::endl;
  }

  std::cout << "Path: " << std::endl;
  for (std::size_t i = 0; i < viewpoint_path_.size(); ++i) {
    const ViewpointPathEntry& path_entry = viewpoint_path_[i];
    const ViewpointEntry& viewpoint_entry = viewpoint_entries_[path_entry.viewpoint_index];
    std::cout << "  viewpoint_index: " << path_entry.viewpoint_index << ", pose[" << i << "]: " << viewpoint_entry.viewpoint.pose()
        << ", information: " << path_entry.information << std::endl;
  }

  return true;
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
