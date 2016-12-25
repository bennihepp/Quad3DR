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
#include <ait/utilities.h>
#include <ait/vision_utilities.h>
#include "viewpoint_planner.h"

using ait::Pose;

Viewpoint::Viewpoint(double projection_margin /*= DEFAULT_PROJECTION_MARGIN*/)
: camera_(nullptr), projection_margin_(projection_margin),
  transformation_world_to_image_(pose_.getTransformationWorldToImage()) {}

Viewpoint::Viewpoint(const PinholeCamera* camera, const Pose& pose, double projection_margin /*= DEFAULT_PROJECTION_MARGIN*/)
: camera_(camera), pose_(pose), projection_margin_(projection_margin),
  transformation_world_to_image_(pose_.getTransformationWorldToImage()) {}

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

double Viewpoint::getDistanceTo(const Viewpoint& other) const {
  return pose_.getDistanceTo(other.pose_);
}

double Viewpoint::getDistanceTo(const Viewpoint& other, double rotation_factor) const {
  return pose_.getDistanceTo(other.pose_, rotation_factor);
}

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


ViewpointPlanner::ViewpointPlanner(const Options* options, std::unique_ptr<ViewpointPlannerData> data)
: options_(*options), data_(std::move(data)) {
  size_t random_seed = options->getValue<size_t>("rng_seed");
  if (random_seed == 0) {
    random_seed = std::chrono::system_clock::now().time_since_epoch().count();
  }
  rng_.seed(random_seed);
  AIT_ASSERT(data_->reconstruction_->getCameras().size() > 0);
  FloatType virtual_camera_scale = options->getValue<FloatType>("virtual_camera_scale");
  setScaledVirtualCamera(data_->reconstruction_->getCameras().cbegin()->first, virtual_camera_scale);
}

std::pair<bool, ait::Pose> ViewpointPlanner::samplePose(size_t max_trials /*= 500*/) {
  return samplePose(data_->roi_bbox_, data_->drone_extent_, max_trials);
}

std::pair<bool, ait::Pose> ViewpointPlanner::samplePose(const BoundingBoxType& bbox,
    const Vector3& object_extent, size_t max_trials /*= 500*/) {
  std::pair<bool, ViewpointPlanner::Vector3> pos_result = samplePosition(bbox, object_extent, max_trials);
  if (!pos_result.first) {
    return std::make_pair(false, ait::Pose());
  }
  const ait::Pose::Vector3 pos = pos_result.second.cast<ait::Pose::Vector3::Scalar>();
  // We just sample from the lower spherex
  std::uniform_real_distribution<FloatType> uniform_m_1(-1, 0);
  std::uniform_real_distribution<FloatType> uniform_pm_pi(-M_PI, +M_PI);
//    using Scalar = ait::Pose::Quaternion::Scalar;
//    ait::Pose::Quaternion orientation = ait::Pose::Quaternion::UnitRandom();
//    Eigen::Matrix<3, 1, Scalar> euler_angles =
//        orientation.toRotationMatrix().eulerAngles(2, 1, 0);
//    Scalar yaw = euler_angles(0);
//    Scalar pitch = euler_angles(1);
//    Scalar roll = euler_angles(2);
  FloatType z = uniform_m_1(rng_);
  FloatType theta = uniform_pm_pi(rng_);
  FloatType x = std::sin(theta) * std::sqrt(1 - z*z);
  FloatType y = std::cos(theta) * std::sqrt(1 - z*z);
  ait::Pose::Vector3 z_axis(0, 0, 1);
  ait::Pose::Vector3 pose_z_axis(x, y, z);
  ait::Pose::Vector3 pose_x_axis;
  if (z == 1) {
    pose_x_axis = ait::Pose::Vector3(1, 0, 0);
  }
  else {
    pose_x_axis = pose_z_axis.cross(z_axis);
  }
  ait::Pose::Vector3 pose_y_axis = pose_z_axis.cross(pose_x_axis);
  ait::Pose::Matrix3x3 rotation;
  rotation.col(0) = pose_x_axis.normalized();
  rotation.col(1) = pose_y_axis.normalized();
  rotation.col(2) = pose_z_axis.normalized();
  ait::Pose pose = ait::Pose::createFromWorldToImageTransformation(pos, rotation);
  return std::make_pair(true, pose);
}

std::pair<bool, ViewpointPlanner::Vector3> ViewpointPlanner::samplePosition(size_t max_trials /*= 500*/) {
  return samplePosition(data_->roi_bbox_, data_->drone_extent_, max_trials);
}

std::pair<bool, ViewpointPlanner::Vector3> ViewpointPlanner::samplePosition(const BoundingBoxType& bbox,
    const Vector3& object_extent, size_t max_trials /*= 500*/) {
  std::uniform_real_distribution<FloatType> bbox_x_dist(bbox.getMinimum(0), bbox.getMaximum(0));
  std::uniform_real_distribution<FloatType> bbox_y_dist(bbox.getMinimum(1), bbox.getMaximum(1));
  std::uniform_real_distribution<FloatType> bbox_z_dist(bbox.getMinimum(2), bbox.getMaximum(2));
  for (size_t i = 0; i < max_trials; ++i) {
    Vector3 pos(bbox_x_dist(rng_), bbox_y_dist(rng_), bbox_z_dist(rng_));
    BoundingBoxType object_bbox = BoundingBoxType::createFromCenterAndExtent(pos, object_extent);
    std::vector<ViewpointPlannerData::OccupiedTreeType::BBoxIntersectionResult> results =
        data_->occupied_bvh_.intersects(object_bbox);
    if (results.empty()) {
      return std::make_pair(true, pos);
    }
  }
  return std::make_pair(false, Vector3());
}

void ViewpointPlanner::setVirtualCamera(size_t virtual_width, size_t virtual_height, const Eigen::Matrix4d& virtual_intrinsics) {
  virtual_camera_ = PinholeCamera(virtual_width, virtual_height, virtual_intrinsics);
  std::cout << "virtual camera size: " << virtual_width << " x " << virtual_height << std::endl;
}

void ViewpointPlanner::setScaledVirtualCamera(CameraId camera_id, double scale_factor) {
  const PinholeCameraColmap& camera = data_->reconstruction_->getCameras().at(camera_id);
  size_t virtual_width = static_cast<size_t>(scale_factor * camera.width());
  size_t virtual_height = static_cast<size_t>(scale_factor * camera.height());
  CameraMatrix virtual_intrinsics = ait::getScaledIntrinsics(camera.intrinsics(), scale_factor);
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
        double projection_margin) const {
  Viewpoint viewpoint(&data_->reconstruction_->getCameras().at(camera_id), pose, projection_margin);
  std::unordered_set<Point3DId> proj_points = viewpoint.getProjectedPoint3DIds(data_->reconstruction_->getPoints3D());
  return proj_points;
}

std::unordered_set<Point3DId> ViewpointPlanner::computeFilteredMapPoints(const CameraId camera_id, const Pose& pose,
        double projection_margin) const {
  Viewpoint viewpoint(&data_->reconstruction_->getCameras().at(camera_id), pose, projection_margin);
  std::unordered_set<Point3DId> proj_points = viewpoint.getProjectedPoint3DIdsFiltered(data_->reconstruction_->getPoints3D());
  return proj_points;
}

std::unordered_set<Point3DId> ViewpointPlanner::computeVisibleMapPoints(const CameraId camera_id, const Pose& pose,
        double projection_margin) const {
  Viewpoint viewpoint(&data_->reconstruction_->getCameras().at(camera_id), pose, projection_margin);
  std::unordered_set<Point3DId> proj_points = viewpoint.getProjectedPoint3DIds(data_->reconstruction_->getPoints3D());
  removeOccludedPoints(pose, proj_points, OCCLUSION_DIST_MARGIN_FACTOR * data_->octree_->getResolution());
  return proj_points;
}

std::unordered_set<Point3DId> ViewpointPlanner::computeVisibleMapPointsFiltered(const CameraId camera_id, const Pose& pose,
        double projection_margin) const {
  Viewpoint viewpoint(&data_->reconstruction_->getCameras().at(camera_id), pose, projection_margin);
  std::unordered_set<Point3DId> proj_points = viewpoint.getProjectedPoint3DIdsFiltered(data_->reconstruction_->getPoints3D());
  removeOccludedPoints(pose, proj_points, OCCLUSION_DIST_MARGIN_FACTOR * data_->octree_->getResolution());
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

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> ViewpointPlanner::getRaycastHitVoxelsBVH(
    const Pose& pose_world_to_image) const {
  Viewpoint viewpoint(&virtual_camera_, pose_world_to_image);
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
      const Ray ray = viewpoint.getCameraRay(x, y);
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
  std::cout << "Voxels: " << raycast_results.size() << std::endl;
  timer.printTiming("getRaycastHitVoxelsBVH");
  return raycast_results;
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
      const Ray ray = viewpoint.getCameraRay(x, y);
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
    const Pose& pose_world_to_image) const {
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results =
      getRaycastHitVoxelsBVH(pose_world_to_image);
  VoxelWithInformationSet voxel_set;
  FloatType total_information = 0;
  std::vector<FloatType> informations;
  for (auto it = raycast_results.cbegin(); it != raycast_results.cend(); ++it) {
    const ViewpointPlannerData::OccupiedTreeType::IntersectionResult& result = *it;
    WeightType information = computeInformationScore(result);
    voxel_set.insert(VoxelWithInformation(result.node, information));
    informations.push_back(information);
    total_information += information;
  }
  if (!informations.empty()) {
    auto result = ait::computeHistogram(informations, 10);
    std::cout << "Histogram: " << std::endl;
    for (size_t i = 0; i < result.first.size(); ++i) {
      std::cout << "  # <= " << result.second[i] << ": " << result.first[i] << std::endl;
    }
  }
  return std::make_pair(voxel_set, total_information);
}

float ViewpointPlanner::computeInformationScore(const ViewpointPlannerData::OccupiedTreeType::IntersectionResult& result) const {
  // TODO: Make sure tree is not pruned for occupied voxels
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
    const Pose& pose_world_to_image) const {
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results =
      getRaycastHitVoxelsBVH(pose_world_to_image);
  double score = 0;
  double unknown_score = 0;
  double unknown_count = 0;
  std::vector<double> informations;
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
  Eigen::Vector3d eigen_origin = pose_image_to_world.translation();
  octomap::point3d origin(eigen_origin(0), eigen_origin(1), eigen_origin(2));
  octomap::OcTreeKey origin_key;
  if (!data_->octree_->coordToKeyChecked(origin, origin_key)) {
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

void ViewpointPlanner::removeOccludedPoints(const Pose& pose_world_to_image, std::unordered_set<Point3DId>& point3D_ids, double dist_margin) const {
  ait::Timer timer;
  Pose pose_image_to_world = pose_world_to_image.inverse();
  Eigen::Vector3d eigen_origin = pose_image_to_world.translation();
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
      double end_point_dist = end_point_from_origin.norm();
      octomap::point3d hit_point;
      const bool ignore_unknown = true;
      data_->octree_->castRay(origin, end_point_from_origin, hit_point, ignore_unknown, end_point_dist);
      double hit_dist = (hit_point - origin).norm();
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

void ViewpointPlanner::run() {
  // Compute viewpoint informations for sampled poses
  std::cout << "Sampling viewpoints and computing information score ..." << std::endl;
  const PinholeCamera* camera = &data_->reconstruction_->getCameras().cbegin()->second;
  std::vector<ViewpointNode> viewpoint_nodes;
  std::size_t num_sampled_poses = options_.getValue<std::size_t>("num_sampled_poses");
  for (std::size_t i = 0; i < num_sampled_poses; ++i) {
    std::cout << "Computing pose " << i << " out of " << num_sampled_poses << std::endl;
    std::pair<bool, ait::Pose> result = samplePose();
    if (result.first) {
      const ait::Pose& pose = result.second;
      std::pair<VoxelWithInformationSet, FloatType> result =
          getRaycastHitVoxelsWithInformationScoreBVH(pose);
      VoxelWithInformationSet& voxel_set = result.first;
      FloatType total_information = result.second;
      ViewpointNode viewpoint_node(Viewpoint(camera, pose), total_information, std::move(voxel_set));
      viewpoint_nodes.push_back(std::move(viewpoint_node));
    }
  }
  std::cout << "Successfully computed " << viewpoint_nodes.size() << " poses" << std::endl;
  std::sort(viewpoint_nodes.begin(), viewpoint_nodes.end(),
      [](const ViewpointNode& a, const ViewpointNode& b) {
    return a.total_information > b.total_information;
  });
  for (const ViewpointNode& node : viewpoint_nodes) {
    std::cout << "information: " << node.total_information << std::endl;
  }
  std::cout << "Done." << std::endl;

  std::cout << "Computing viewpoint distances ..." << std::endl;
  viewpoint_graph_ = ViewpointGraph(viewpoint_nodes.begin(), viewpoint_nodes.end());
  for (ViewpointGraph::Index i = 0; i < viewpoint_graph_.numOfNodes(); ++i) {
    const ViewpointGraph::NodeType& node1 = viewpoint_graph_.getNode(i);
    for (ViewpointGraph::Index j = i + 1; j < viewpoint_graph_.numOfNodes(); ++j) {
      const ViewpointGraph::NodeType& node2 = viewpoint_graph_.getNode(j);
      ViewpointGraph::WeightType distance = node1.viewpoint.getDistanceTo(node2.viewpoint);
      viewpoint_graph_.setWeight(i, j, distance);
      viewpoint_graph_.setWeight(j, i, distance);
    }
  }
  std::cout << "Done." << std::endl;

  // Try to find a good path of viewpoints.
  std::cout << "Constructing viewpoint path ..." << std::endl;
  viewpoint_path_.clear();
  std::unordered_set<const ViewpointNode*> visited_nodes;
  VoxelWithInformationSet seen_voxel_set;
  std::size_t num_planned_viewpoints = options_.getValue<std::size_t>("num_planned_viewpoints");
  for (std::size_t i = 0; i < num_planned_viewpoints; ++i) {
    std::cout << "i=" << i << ", seen_voxel_set.size()=" << seen_voxel_set.size() << std::endl;
    ViewpointPathEntry best_entry(nullptr, std::numeric_limits<FloatType>::lowest());
    for (std::size_t j = 0; j < viewpoint_graph_.numOfNodes(); ++j) {
      const ViewpointNode& next_node = viewpoint_graph_.getNode(j);
      VoxelWithInformationSet difference_set = ait::computeSetDifference(next_node.voxel_set, seen_voxel_set);
//      std::cout << "  j=" << j << ", next_node.voxel_set.size()= " << next_node.voxel_set.size() << std::endl;
//      std::cout << "  j=" << j << ", difference_set.size()= " << difference_set.size() << std::endl;
      FloatType new_information = std::accumulate(difference_set.cbegin(), difference_set.cend(),
          FloatType { 0 }, [](const FloatType& value, const VoxelWithInformation& voxel) {
            return value + voxel.information;
      });
//      std::cout << "  j=" << j << ", new_information=" << new_information << std::endl;
      if (new_information > best_entry.information) {
        best_entry = ViewpointPathEntry(&next_node, new_information);
      }
    }
    AIT_ASSERT(best_entry.node != nullptr);
    viewpoint_path_.push_back(best_entry);
    seen_voxel_set.insert(best_entry.node->voxel_set.cbegin(), best_entry.node->voxel_set.cend());
    std::cout << "  " << i << ": new information: " <<  best_entry.information <<
        ", information: " << best_entry.node->total_information << std::endl;
  }
  std::cout << "Done." << std::endl;

  return;

  size_t num_positions = 100;
  for (size_t i = 0; i < num_positions; ++i) {
    std::pair<bool, Vector3> result = samplePosition(data_->roi_bbox_, data_->drone_extent_);
    if (result.first) {
      const Vector3& pos = result.second;
      std::cout << "Sampled position " << i << ": " << pos.transpose() << std::endl;
    }
  }
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
  for (const auto& entry : data_->reconstruction_->getImages()) {
    ait::Timer timer2;
    const ImageColmap& image = entry.second;
    const PinholeCameraColmap& camera = data_->reconstruction_->getCameras().at(image.camera_id());
    std::cout << "image_id=" << image.id() << std::endl;
    std::cout << "  features: " << image.features().size() << std::endl;
    const double projection_margin = 0;
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
