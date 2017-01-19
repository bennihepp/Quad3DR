//==================================================
// viewpoint_planner.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 12, 2016
//==================================================

#include <functional>
#include <random>
#include <ait/boost.h>
#include <boost/filesystem.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/stringbuffer.h>
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

ViewpointPlanner::ViewpointPlanner(
    const Options* options, const MotionPlannerType::Options* motion_options, std::unique_ptr<ViewpointPlannerData> data)
: options_(*options), data_(std::move(data)),
  motion_planner_(motion_options, data_.get(), options->motion_planner_log_filename),
  viewpoint_graph_components_valid_(false) {
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
  // Compute bounding boxes for viewpoint sampling and motion sampling
  Vector3 roi_bbox_extent;
  roi_bbox_extent(0) = 2 * data_->roi_.getPolygon2D().getEnclosingRadius();
  roi_bbox_extent(1) = roi_bbox_extent(0);
  roi_bbox_extent(2) = data_->roi_.getUpperPlaneZ() - data_->roi_.getLowerPlaneZ();
  pose_sample_bbox_ = BoundingBoxType::createFromCenterAndExtent(
      data_->roi_.getCentroid(), roi_bbox_extent * options_.sampling_roi_factor);
  const FloatType motion_sampling_roi_factor = 2 * options_.sampling_roi_factor;
  const BoundingBoxType motion_sampling_bbox = BoundingBoxType::createFromCenterAndExtent(
      data_->roi_.getCentroid(), roi_bbox_extent * motion_sampling_roi_factor);
  motion_planner_.setSpaceBoundingBox(motion_sampling_bbox);
  motion_planner_.setObjectExtent(drone_extent_);

  // TODO: Remove?
  triangulation_max_cos_angle_ = std::cos(options_.triangulation_min_angle_degrees * FloatType(M_PI) / 180);
  const FloatType triangulation_min_sin_angle = std::sin(options_.triangulation_min_angle_degrees * FloatType(M_PI) / 180);
  triangulation_min_sin_angle_square_ = triangulation_min_sin_angle * triangulation_min_sin_angle;
  const FloatType triangulation_max_sin_angle = std::sin(options_.triangulation_max_angle_degrees * FloatType(M_PI) / 180);
  triangulation_max_sin_angle_square_ = triangulation_max_sin_angle * triangulation_max_sin_angle;
  triangulation_max_angular_deviation_ = options_.triangulation_max_angular_deviation_degrees * (FloatType)M_PI / 180;

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
  viewpoint_graph_components_.first.clear();
  viewpoint_graph_components_valid_ = false;
  viewpoint_graph_motions_.clear();
  viewpoint_paths_.clear();
  viewpoint_paths_.resize(options_.viewpoint_path_branches);
  viewpoint_paths_data_.clear();
  viewpoint_paths_data_.resize(options_.viewpoint_path_branches);
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
  if (!options_.ignore_real_observed_voxels) {
    std::cout << "Computing observed voxels for previous camera viewpoints" << std::endl;
#pragma omp parallel for
    for (std::size_t i = 0; i < num_real_viewpoints_; ++i) {
      ViewpointEntry& viewpoint_entry = viewpoint_entries_[i];
      // Compute observed voxels and information
      std::pair<VoxelWithInformationSet, FloatType> raycast_result =
          getRaycastHitVoxelsWithInformationScore(viewpoint_entry.viewpoint.pose());
      const VoxelWithInformationSet& voxel_set = raycast_result.first;
      viewpoint_entry.voxel_set = std::move(voxel_set);
    }
  }
  // Initialize viewpoint nearest neighbor index
  std::cout << "Initialized viewpoint entries with " << viewpoint_entries_.size() << " viewpoints" << std::endl;

  // TODO: Initialize feature_viewpoint_map_ from existing real images
//  for (std::size_t i = 0; i < data_->poisson_mesh_->)
}

void ViewpointPlanner::resetViewpointMotions() {
  std::unique_lock<std::mutex> lock(mutex_);
  for (const ViewpointEntryIndex index : viewpoint_graph_) {
    viewpoint_graph_.getEdgesByNode(index).clear();
  }
  viewpoint_graph_components_valid_ = false;
  viewpoint_graph_motions_.clear();
  lock.unlock();
}

void ViewpointPlanner::resetViewpointPaths() {
  std::unique_lock<std::mutex> lock(mutex_);
  viewpoint_paths_.clear();
  viewpoint_paths_.resize(options_.viewpoint_path_branches);
  viewpoint_paths_data_.clear();
  viewpoint_paths_data_.resize(options_.viewpoint_path_branches);
  lock.unlock();
}

void ViewpointPlanner::saveViewpointGraph(const std::string& filename) const {
  std::cout << "Writing viewpoint graph to " << filename << std::endl;
  std::cout << "Graph has " << viewpoint_graph_.numVertices() << " viewpoints"
      << " and " << viewpoint_graph_.numEdges() << " motions" << std::endl;
  std::ofstream ofs(filename);
  boost::archive::binary_oarchive oa(ofs);
  ViewpointEntrySaver ves(viewpoint_entries_, data_->occupied_bvh_);
  oa << num_real_viewpoints_;
  oa << ves;
  oa << viewpoint_graph_;
  oa << viewpoint_graph_motions_;
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
  viewpoint_graph_.clear();
  ia >> viewpoint_graph_;
  viewpoint_graph_components_valid_ = false;
  std::cout << "Regenerating approximate nearest neighbor index" << std::endl;
  viewpoint_ann_.clear();
  for (const ViewpointEntry& viewpoint_entry : viewpoint_entries_) {
    viewpoint_ann_.addPoint(viewpoint_entry.viewpoint.pose().getWorldPosition());
  }
  viewpoint_graph_motions_.clear();
  ia >> viewpoint_graph_motions_;
  std::cout << "Loaded viewpoint graph with " << viewpoint_entries_.size() << " viewpoints "
      << " and " << viewpoint_graph_.numEdges() << " motions" << std::endl;
  AIT_ASSERT(viewpoint_entries_.size() == viewpoint_graph_.numVertices());
  AIT_ASSERT(viewpoint_graph_.numEdges() == viewpoint_graph_motions_.size());
  // TODO: Would be good to have this as an option??
  if (options_.ignore_real_observed_voxels) {
    std::cout << "Clearing observed voxels for previous camera viewpoints" << std::endl;
    for (std::size_t i = 0; i < num_real_viewpoints_; ++i) {
      ViewpointEntry& viewpoint_entry = viewpoint_entries_[i];
      viewpoint_entry.voxel_set.clear();
    }
  }
  else {
    std::cout << "Computing observed voxels for previous camera viewpoints" << std::endl;
#pragma omp parallel for
    for (std::size_t i = 0; i < num_real_viewpoints_; ++i) {
      ViewpointEntry& viewpoint_entry = viewpoint_entries_[i];
      if (viewpoint_entry.voxel_set.empty()) {
        // Compute observed voxels and information
        std::pair<VoxelWithInformationSet, FloatType> raycast_result =
            getRaycastHitVoxelsWithInformationScore(viewpoint_entry.viewpoint.pose());
        const VoxelWithInformationSet& voxel_set = raycast_result.first;
        viewpoint_entry.voxel_set = std::move(voxel_set);
      }
    }
  }
}

void ViewpointPlanner::saveViewpointPath(const std::string& filename) const {
  std::cout << "Writing viewpoint paths to " << filename << std::endl;
  std::cout << "There are " << viewpoint_paths_.size() << " paths."
      << " The best path has " << getBestViewpointPath().entries.size() << " viewpoints" << std::endl;
  std::ofstream ofs(filename);
  boost::archive::binary_oarchive oa(ofs);
  ViewpointPathSaver vps(viewpoint_paths_, viewpoint_paths_data_, data_->occupied_bvh_);
  oa << vps;
  std::unordered_map<ViewpointIndexPair, Motion, ViewpointIndexPair::Hash> local_viewpoint_path_motions;
  for (const ViewpointPath& viewpoint_path : viewpoint_paths_) {
    for (auto it1 = viewpoint_path.entries.begin(); it1 != viewpoint_path.entries.end(); ++it1) {
      for (auto it2 = viewpoint_path.entries.begin(); it2 != viewpoint_path.entries.end(); ++it2) {
        if (it1 != it2) {
          const ViewpointIndexPair vip(it1->viewpoint_index, it2->viewpoint_index);
          const auto it = viewpoint_graph_motions_.find(vip);
          if (it != viewpoint_graph_motions_.end()) {
            const Motion& motion = it->second;
            local_viewpoint_path_motions.emplace(vip, motion);
          }
        }
      }
    }
  }
  oa << local_viewpoint_path_motions;
  std::cout << "Done" << std::endl;
  reportViewpointPathsStats();
}

void ViewpointPlanner::loadViewpointPath(const std::string& filename) {
  resetViewpointPaths();
  std::cout << "Loading viewpoint paths from " << filename << std::endl;
  std::ifstream ifs(filename);
  boost::archive::binary_iarchive ia(ifs);
  viewpoint_paths_.clear();
  viewpoint_paths_data_.clear();
  ViewpointPathLoader vpl(&viewpoint_paths_, &viewpoint_paths_data_, &data_->occupied_bvh_);
  ia >> vpl;
  //
  std::unordered_map<ViewpointIndexPair, Motion, ViewpointIndexPair::Hash> local_viewpoint_path_motions;
  ia >> local_viewpoint_path_motions;
  for (const auto& motion_entry : local_viewpoint_path_motions) {
    const ViewpointIndexPair& vip = motion_entry.first;
    const Motion& motion = motion_entry.second;
    addViewpointMotion(vip.index1, vip.index2, motion);
  }
//  std::cout << "Recomputing observed voxel sets" << std::endl;
//  std::cout << "Recomputing sorted information lists" << std::endl;
//#pragma omp parallel for
//  for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
//    ViewpointPath& viewpoint_path = viewpoint_paths_[i];
//    ViewpointPathComputationData& comp_data = viewpoint_paths_data_[i];
//    for (const ViewpointPathEntry& path_entry : viewpoint_path.entries) {
//      const ViewpointEntry& viewpoint_entry = viewpoint_entries_[path_entry.viewpoint_index];
//      viewpoint_path.observed_voxel_set.insert(viewpoint_entry.voxel_set.cbegin(), viewpoint_entry.voxel_set.cend());
//    }
//    initializeViewpointPathInformations(&viewpoint_path, &comp_data);
//    updateViewpointPathInformations(&viewpoint_path, &comp_data);
//  }
//  reportViewpointPathsStats();
//  std::cout << "Ensuring connectivity of viewpoints on paths" << std::endl;
#pragma omp parallel for
  for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
    ViewpointPath& viewpoint_path = viewpoint_paths_[i];
    ViewpointPathComputationData& comp_data = viewpoint_paths_data_[i];
//    ensureConnectedViewpointPath(&viewpoint_path, &comp_data);
    for (std::size_t i : viewpoint_path.order) {
      const ViewpointPathEntry& path_entry = viewpoint_path.entries[i];
      bool found_motions = findAndAddShortestMotions(
          path_entry.viewpoint_index, viewpoint_path.entries.begin(), viewpoint_path.entries.begin() + viewpoint_path.order.size());
      if (!found_motions) {
        std::cout << "Could not find motion path for viewpoint on path" << path_entry.viewpoint_index << std::endl;
      }
      AIT_ASSERT(found_motions);
    }
    comp_data.num_connected_entries = viewpoint_path.order.size();
  }
  std::cout << "Loaded " << viewpoint_paths_.size() << " viewpoint paths."
      << " The best path has " << getBestViewpointPath().entries.size() << " viewpoints" << std::endl;
}

rapidjson::Document ViewpointPlanner::getViewpointPathAsJson(const ViewpointPath& viewpoint_path) const {
  std::cout << "Converting viewpoint path to JSON" << std::endl;

  using GpsCoordinateType = reconstruction::SfmToGpsTransformation::GpsCoordinate;
  using GpsFloatType = typename GpsCoordinateType::FloatType;
  using GpsConverter = ait::GpsConverter<GpsFloatType>;
  const GpsCoordinateType gps_reference = data_->reconstruction_->sfmGpsTransformation().gps_reference;
  std::cout << "GPS reference: " << gps_reference << std::endl;
  const GpsConverter gps_converter = GpsConverter::createWGS84(gps_reference);

  using rapidjson::Document;
  using rapidjson::Value;
  using rapidjson::kArrayType;
  using rapidjson::kObjectType;
  Document d;
  d.SetObject();
  auto& allocator = d.GetAllocator();

  Value viewpoints(kArrayType);
  using IteratorType = typename std::vector<std::size_t>::const_iterator;
  const auto generate_json_viewpoint_lambda = [&](IteratorType prev_it, IteratorType it) {
    const ViewpointPathEntry& path_entry = viewpoint_path.entries[*it];
    const ViewpointEntry& viewpoint_entry = viewpoint_entries_[path_entry.viewpoint_index];
    const GpsCoordinateType gps = gps_converter.convertEnuToGps(viewpoint_entry.viewpoint.pose().getWorldPosition().cast<GpsFloatType>());
    Value json_pose(kObjectType);
    const GpsFloatType latitude = gps.latitude();
    const GpsFloatType longitude = gps.longitude();
    const GpsFloatType altitude = gps.altitude();
    // East is yaw = 0, south is yaw = 90, rotation axis is pointing in z-direction
    const Vector3 viewing_direction = viewpoint_entry.viewpoint.pose().rotation().col(2);
    const FloatType yaw_radians = -std::atan2(viewing_direction(1), viewing_direction(0));
    const FloatType camera_pitch_radians = std::atan2(viewing_direction(2), viewing_direction.topRows(2).squaredNorm());
    const FloatType yaw = ait::radiansToDegrees(yaw_radians);
    const FloatType camera_yaw = 0;
    const FloatType camera_pitch = ait::radiansToDegrees(camera_pitch_radians);
    json_pose.AddMember("latitude", latitude, allocator);
    json_pose.AddMember("longitude", longitude, allocator);
    json_pose.AddMember("altitude", altitude, allocator);
    json_pose.AddMember("yaw", yaw, allocator);
    Value json_path_array(kArrayType);
    if (prev_it != it) {
      const ViewpointEntryIndex prev_idx = viewpoint_path.entries[*prev_it].viewpoint_index;
      const ViewpointEntryIndex idx = viewpoint_path.entries[*it].viewpoint_index;
      if (!hasViewpointMotion(prev_idx, idx)) {
        std::cout << "No viewpoint motion between " << prev_idx << " and " << idx << std::endl;
      }
      const Motion motion = getViewpointMotion(prev_idx, idx);
      for (auto path_it = motion.poses.begin(); path_it != motion.poses.end(); ++path_it) {
        const GpsCoordinateType path_gps = gps_converter.convertEnuToGps(path_it->getWorldPosition().cast<GpsFloatType>());
        Value json_pose(kObjectType);
        const GpsFloatType latitude = path_gps.latitude();
        const GpsFloatType longitude = path_gps.longitude();
        const GpsFloatType altitude = path_gps.altitude();
        Value json_path_entry(kObjectType);
        json_path_entry.AddMember("index", path_it - motion.poses.begin(), allocator);
        json_path_entry.AddMember("latitude", latitude, allocator);
        json_path_entry.AddMember("longitude", longitude, allocator);
        json_path_entry.AddMember("altitude", altitude, allocator);
        json_path_entry.AddMember("yaw", yaw, allocator);
        json_path_array.PushBack(json_path_entry, allocator);
      }
    }
    else {
      Value json_path_entry(kObjectType);
      json_path_entry.AddMember("index", 0, allocator);
      json_path_entry.AddMember("latitude", latitude, allocator);
      json_path_entry.AddMember("longitude", longitude, allocator);
      json_path_entry.AddMember("altitude", altitude, allocator);
      json_path_entry.AddMember("yaw", yaw, allocator);
      json_path_array.PushBack(json_path_entry, allocator);
    }
    Value json_viewpoint(kObjectType);
    json_viewpoint.AddMember("index", *it, allocator);
    json_viewpoint.AddMember("pose", json_pose, allocator);
    json_viewpoint.AddMember("path", json_path_array, allocator);
    json_viewpoint.AddMember("camera_yaw", camera_yaw, allocator);
    json_viewpoint.AddMember("camera_pitch", camera_pitch, allocator);
    std::cout << "i=" << *it << ", latitude=" << latitude << ", longitude" << longitude << std::endl;
    return json_viewpoint;
  };

  if (viewpoint_path.order.size() > 1) {
    // TODO: Handle start position to first viewpoint
    for (auto it = viewpoint_path.order.begin(); it != viewpoint_path.order.end(); ++it) {
      auto prev_it = it - 1;
      if (it == viewpoint_path.order.begin()) {
        prev_it = it;
      }
      Value json_viewpoint = generate_json_viewpoint_lambda(prev_it, it);
      const bool take_picture = true;
      json_viewpoint.AddMember("take_picture", take_picture, allocator);
      viewpoints.PushBack(json_viewpoint, allocator);
    }
    Value json_viewpoint = generate_json_viewpoint_lambda(
        viewpoint_path.order.begin() + viewpoint_path.order.size() - 1, viewpoint_path.order.begin());
    const bool take_picture = false;
    json_viewpoint.AddMember("take_picture", take_picture, allocator);
  }

  d.AddMember("viewpoints", viewpoints, allocator);
  return d;
}

std::string ViewpointPlanner::getViewpointPathAsJsonString(const ViewpointPath& viewpoint_path) const {
  rapidjson::Document d = getViewpointPathAsJson(viewpoint_path);
  rapidjson::StringBuffer buffer;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
  d.Accept(writer);
  return buffer.GetString();
}

void ViewpointPlanner::exportViewpointPathAsJson(const std::string& filename, const ViewpointPath& viewpoint_path) const {
  rapidjson::Document d = getViewpointPathAsJson(viewpoint_path);
  std::cout << "Exporting viewpoint path as JSON to" << filename << std::endl;
  std::ofstream ofs(filename);
  rapidjson::OStreamWrapper osw(ofs);
  rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(osw);
  d.Accept(writer);
}

std::pair<bool, ViewpointPlanner::Pose> ViewpointPlanner::samplePose(std::size_t max_trials /*= (std::size_t)-1*/) const {
  return samplePose(pose_sample_bbox_, drone_extent_, max_trials);
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

bool ViewpointPlanner::hasViewpointMotion(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index) const {
  return viewpoint_graph_motions_.count(ViewpointIndexPair(from_index, to_index)) > 0;
}

ViewpointPlanner::Motion ViewpointPlanner::getViewpointMotion(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index) const {
  Motion motion = viewpoint_graph_motions_.at(ViewpointIndexPair(from_index, to_index));
  if (to_index < from_index) {
    std::reverse(motion.poses.begin(), motion.poses.end());
  }
  return motion;
}

void ViewpointPlanner::addViewpointMotion(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index, const Motion& motion) {
  Motion motion_copy = motion;
  addViewpointMotion(from_index, to_index, std::move(motion_copy));
}

void ViewpointPlanner::addViewpointMotion(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index, Motion&& motion) {
  if (to_index < from_index) {
    std::reverse(motion.poses.begin(), motion.poses.end());
  }
  // TODO: Add symmetric edges? Currently graph is unidirectional
  viewpoint_graph_.addEdgeByNode(from_index, to_index, motion.distance);
  viewpoint_graph_components_valid_ = false;
  viewpoint_graph_motions_.emplace(ViewpointIndexPair(from_index, to_index), std::move(motion));
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
  const FloatType dist = (pos - bias_bbox.getCenter()).norm();
  const FloatType bbox_fov_angle = std::atan(bias_bbox.getMaxExtent() / (2 * dist));
  FloatType angle_stddev = ait::clamp<FloatType>(bbox_fov_angle, 0, M_PI / 2);
  const FloatType angle1 = std::abs(random_.sampleNormal(0, angle_stddev));
  FloatType angle2 = random_.sampleNormal(0, angle_stddev);
  Vector3 bbox_direction = (bias_bbox.getCenter() - pos).normalized();
  // TODO: This should be configurable
  // Multicopter camera can only cover pitch in the range of [-90, 0] degrees
  if (bbox_direction(2) > 0) {
    bbox_direction(2) = 0;
  }
  angle2 = ait::clamp<FloatType>(angle2, 0, M_PI / 2);
  // Compute viewing direction (i.e. pose z axis)
  Pose::Vector3 viewing_direction = AngleAxis(angle1, Vector3::UnitZ()) * bbox_direction;
  viewing_direction = AngleAxis(-angle2, viewing_direction.cross(Vector3::UnitZ())) * viewing_direction;

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
  const Pose::Vector3 z_axis(0, 0, 1);
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
  const Pose::Vector3 pose_y_axis = pose_z_axis.cross(pose_x_axis);
  Pose::Matrix3x3 rotation;
  rotation.col(0) = pose_x_axis.normalized();
  rotation.col(1) = pose_y_axis.normalized();
  rotation.col(2) = pose_z_axis.normalized();
  return Pose::Quaternion(rotation);
}

bool ViewpointPlanner::isValidObjectPosition(const Vector3& position, const Vector3& object_extent) const {
  return data_->isValidObjectPosition(position, object_extent);
}

std::pair<bool, ViewpointPlanner::Vector3> ViewpointPlanner::samplePosition(std::size_t max_trials /*= (std::size_t)-1*/) const {
  return samplePosition(pose_sample_bbox_, drone_extent_, max_trials);
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
    const bool valid = isValidObjectPosition(pos, object_extent);
    if (valid) {
      return std::make_pair(true, pos);
    }
  }
  return std::make_pair(false, Vector3());
}

const PinholeCamera& ViewpointPlanner::getVirtualCamera() const {
  return virtual_camera_;
}

void ViewpointPlanner::setVirtualCamera(size_t virtual_width, size_t virtual_height, const Matrix4x4& virtual_intrinsics) {
  virtual_camera_ = PinholeCamera(virtual_width, virtual_height, virtual_intrinsics);
  std::cout << "Virtual camera size: " << virtual_width << " x " << virtual_height << std::endl;
}

void ViewpointPlanner::setScaledVirtualCamera(CameraId camera_id, FloatType scale_factor) {
  const PinholeCamera& camera = data_->reconstruction_->getCameras().at(camera_id);
  size_t virtual_width = static_cast<size_t>(scale_factor * camera.width());
  size_t virtual_height = static_cast<size_t>(scale_factor * camera.height());
  reconstruction::CameraMatrix virtual_intrinsics = ait::getScaledIntrinsics(camera.intrinsics(), scale_factor);
  virtual_camera_ = PinholeCamera(virtual_width, virtual_height, virtual_intrinsics);
  std::cout << "Virtual camera size: " << virtual_width << " x " << virtual_height << std::endl;
}

ViewpointPlanner::WeightType ViewpointPlanner::computeObservationInformationFactor(CounterType observation_count) const {
  const FloatType information_factor = std::exp(- options_.voxel_information_lambda * observation_count);
  return information_factor;
//  return observation_count == 0 ? 1 : 0;
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

ViewpointPlanner::GpsCoordinateType ViewpointPlanner::convertPositionToGps(const Vector3& position) const {
  using GpsFloatType = typename GpsCoordinateType::FloatType;
  using GpsConverter = ait::GpsConverter<GpsFloatType>;
  const GpsCoordinateType gps_reference = data_->reconstruction_->sfmGpsTransformation().gps_reference;
  const GpsConverter gps_converter = GpsConverter::createWGS84(gps_reference);
  const GpsCoordinateType gps = gps_converter.convertEnuToGps(position.cast<GpsFloatType>());
  return gps;
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> ViewpointPlanner::getRaycastHitVoxels(
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
      const FloatType min_range = options_.raycast_min_range;
      const FloatType max_range = options_.raycast_max_range;
      std::pair<bool, ViewpointPlannerData::OccupiedTreeType::IntersectionResult> result =
          data_->occupied_bvh_.intersects(ray, min_range, max_range);
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
  removeDuplicateRaycastHitVoxels(&raycast_results);
//  std::cout << "Voxels: " << raycast_results.size() << std::endl;
  timer.printTiming("getRaycastHitVoxels");
  return raycast_results;
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> ViewpointPlanner::getRaycastHitVoxels(
    const Pose& pose, const std::size_t width, const std::size_t height) const {
  Viewpoint viewpoint(&virtual_camera_, pose);
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results;
  raycast_results.resize(width * height);
  ait::Timer timer;
  const std::size_t y_start = virtual_camera_.height() / 2 - height / 2;
  const std::size_t y_end = virtual_camera_.height() / 2 + height / 2 + 1;
  const std::size_t x_start = virtual_camera_.width() / 2 - width / 2;
  const std::size_t x_end = virtual_camera_.width() / 2 + width / 2 + 1;
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

#if WITH_CUDA
std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> ViewpointPlanner::getRaycastHitVoxelsCuda(
    const Pose& pose) const {
  Viewpoint viewpoint(&virtual_camera_, pose);
  ait::Timer timer;

  const std::size_t y_start = 0;
  const std::size_t y_end = virtual_camera_.height();
  const std::size_t x_start = 0;
  const std::size_t x_end = virtual_camera_.width();

  // Perform raycast
  const FloatType min_range = options_.raycast_min_range;
  const FloatType max_range = options_.raycast_max_range;
  using ResultType = ViewpointPlannerData::OccupiedTreeType::IntersectionResult;
  std::vector<ResultType> raycast_results =
      data_->occupied_bvh_.raycastCuda(
          virtual_camera_.intrinsics(),
          pose.getTransformationImageToWorld(),
          x_start, x_end,
          y_start, y_end,
          min_range, max_range);
  removeDuplicateRaycastHitVoxels(&raycast_results);

//  std::cout << "Voxels: " << raycast_results.size() << std::endl;
  timer.printTiming("getRaycastHitVoxelsCuda");
  return raycast_results;
}

std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> ViewpointPlanner::getRaycastHitVoxelsCuda(
    const Pose& pose, const std::size_t width, const std::size_t height) const {
  Viewpoint viewpoint(&virtual_camera_, pose);
  ait::Timer timer;

  const std::size_t y_start = virtual_camera_.height() / 2 - height / 2;
  const std::size_t y_end = virtual_camera_.height() / 2 + height / 2 + 1;
  const std::size_t x_start = virtual_camera_.width() / 2 - width / 2;
  const std::size_t x_end = virtual_camera_.width() / 2 + width / 2 + 1;

  // Perform raycast
  const FloatType min_range = options_.raycast_min_range;
  const FloatType max_range = options_.raycast_max_range;
  using ResultType = ViewpointPlannerData::OccupiedTreeType::IntersectionResult;
  std::vector<ResultType> raycast_results =
      data_->occupied_bvh_.raycastCuda(
          virtual_camera_.intrinsics(),
          pose.getTransformationImageToWorld(),
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
  std::unordered_map<ViewpointPlannerData::OccupiedTreeType::NodeType*, ViewpointPlannerData::OccupiedTreeType::IntersectionResult>
      raycast_map;
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

std::pair<ViewpointPlanner::VoxelWithInformationSet, ViewpointPlanner::FloatType>
ViewpointPlanner::getRaycastHitVoxelsWithInformationScore(
    const Pose& pose) const {
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> raycast_results =
      getRaycastHitVoxels(pose);
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

ViewpointPlanner::FloatType  ViewpointPlanner::computeInformationScore(const ViewpointPlannerData::OccupiedTreeType::IntersectionResult& result) const {
  return computeInformationScore(result.node);
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

ViewpointPlanner::FloatType ViewpointPlanner::computeInformationScore(const VoxelType* node) const {
  CounterType voxel_observation_count = node->getObject()->observation_count;
  WeightType information_factor = computeObservationInformationFactor(voxel_observation_count);
//  WeightType information_factor = 1;
  WeightType weight = node->getObject()->weight;
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
    const bool valid = isValidObjectPosition(sampled_pos, drone_extent_);
    if (valid) {
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
  const FloatType dist_real_thres_square = options_.viewpoint_discard_dist_real_thres_square;
  static std::vector<ViewpointANN::IndexType> knn_indices;
  static std::vector<ViewpointANN::DistanceType> knn_distances;
  knn_indices.resize(dist_knn);
  knn_distances.resize(dist_knn);
  viewpoint_ann_.knnSearch(pose.getWorldPosition(), dist_knn, &knn_indices, &knn_distances);
  std::size_t too_close_count = 0;
//  for (ViewpointANN::IndexType viewpoint_index : knn_indices) {
//    const ViewpointEntry& other_viewpoint = viewpoint_entries_[viewpoint_index];
//    FloatType dist_square = (pose.getWorldPosition() - other_viewpoint.viewpoint.pose().getWorldPosition()).squaredNorm();
  for (std::size_t i = 0; i < knn_distances.size(); ++i) {
    const ViewpointANN::DistanceType dist_square = knn_distances[i];
    const ViewpointEntryIndex viewpoint_index = knn_indices[i];
    if (viewpoint_index < num_real_viewpoints_ && dist_square < dist_real_thres_square) {
//      if (verbose) {
//        std::cout << "Rejected: Too close to real viewpoint" << std::endl;
//      }
      return false;
    }
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
//      std::cout << "Rejected: Too close to other viewpoints" << std::endl;
//    }
    return false;
  }

  // Compute observed voxels and information and discard if too few voxels or too little information
  std::pair<VoxelWithInformationSet, FloatType> raycast_result =
      getRaycastHitVoxelsWithInformationScore(pose);
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
  for (auto it = viewpoint_graph_.begin(); it != viewpoint_graph_.end(); ++it) {
    const ViewpointEntryIndex from_index = it.node();
    if (from_index < num_real_viewpoints_) {
      continue;
    }
    std::vector<ViewpointPlanner::ViewpointMotion> motions = findViewpointMotions(from_index);
    std::cout << "Found " << motions.size() << " connections from viewpoint " << from_index << std::endl;

    std::unique_lock<std::mutex> lock(mutex_);
    for (ViewpointMotion& motion : motions) {
      AIT_ASSERT(motion.from_index == from_index);
      addViewpointMotion(from_index, motion.to_index, std::move(motion.motion));
      AIT_ASSERT(viewpoint_graph_.numEdges() == viewpoint_graph_motions_.size());
    }
    lock.unlock();
  }

//  // Print info on connection graph
//  for (auto it = viewpoint_graph_.begin(); it != viewpoint_graph_.end(); ++it) {
//    std::cout << "edges for node " << it.node() << std::endl;
//    auto edges = viewpoint_graph_.getEdgesByNode(it.node());
//    for (auto it2 = edges.begin(); it2 != edges.end(); ++it2) {
//      std::cout << "  edge to " << it2.targetNode() << ", from " << it2.sourceNode() << ", weight " << it2.weight() << std::endl;
//    }
//  }

//  std::cout << "Densifying edges on viewpoint graph" << std::endl;
//  // Densify viewpoint motions
//  std::vector<std::tuple<ViewpointEntryIndex, ViewpointEntryIndex, FloatType>> new_edges;
//  for (auto it = viewpoint_graph_.begin(); it != viewpoint_graph_.end(); ++it) {
//    const ViewpointEntryIndex index = it.node();
//    std::stack<std::tuple<ViewpointEntryIndex, FloatType, std::size_t>> node_stack;
//    std::unordered_set<ViewpointEntryIndex> visited_set;
//    node_stack.push(std::make_tuple(index, 0, 0));
//    while (!node_stack.empty()) {
//      ViewpointEntryIndex stack_index = std::get<0>(node_stack.top());
//      FloatType acc_motion_distance = std::get<1>(node_stack.top());
//      std::size_t depth = std::get<2>(node_stack.top());
//      node_stack.pop();
//      if (depth >= options_.viewpoint_motion_densification_max_depth) {
//        continue;
//      }
//      auto edges = viewpoint_graph_.getEdgesByNode(stack_index);
//      for (auto it = edges.begin(); it != edges.end(); ++it) {
//        ViewpointEntryIndex other_index = it.targetNode();
//        if (visited_set.count(other_index) > 0) {
//          continue;
//        }
//        visited_set.emplace(other_index);
//        FloatType motion_distance = it.weight();
//        node_stack.push(std::make_tuple(other_index, acc_motion_distance + motion_distance, depth + 1));
//        if (depth >= 2) {
//          new_edges.push_back(std::make_tuple(index, other_index, acc_motion_distance + motion_distance));
//        }
//      }
//    }
//  }
//  std::cout << "Adding " << new_edges.size() << " edges to the viewpoint graph" << std::endl;
//  for (const auto& tuple : new_edges) {
//    // TODO: Add symmetric edges?
//    viewpoint_graph_.addEdgeByNode(std::get<0>(tuple), std::get<1>(tuple), std::get<2>(tuple));
//    viewpoint_graph_.addEdgeByNode(std::get<1>(tuple), std::get<0>(tuple), std::get<2>(tuple));
//  }
//  for (const auto& index : viewpoint_graph_) {
//    std::cout << "Node " << index << " has " << viewpoint_graph_.getEdgesByNode(index).size() << " edges" << std::endl;
//  }
  std::cout << "Done" << std::endl;
}

ViewpointPlanner::ViewpointEntryIndex ViewpointPlanner::addViewpointEntryWithoutLock(ViewpointEntry&& viewpoint_entry) {
  ViewpointEntryIndex viewpoint_index = viewpoint_entries_.size();
  viewpoint_ann_.addPoint(viewpoint_entry.viewpoint.pose().getWorldPosition());
  viewpoint_entries_.emplace_back(std::move(viewpoint_entry));
  viewpoint_graph_.addNode(viewpoint_index);
  viewpoint_graph_components_valid_ = false;
  return viewpoint_index;
}

void ViewpointPlanner::addViewpointEntry(ViewpointEntry&& viewpoint_entry) {
  std::unique_lock<std::mutex> lock(mutex_);
  addViewpointEntryWithoutLock(std::move(viewpoint_entry));
  lock.unlock();
}

std::vector<ViewpointPlanner::ViewpointMotion>
ViewpointPlanner::findViewpointMotions(const ViewpointEntryIndex from_index) {
//  ait::Timer timer;
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
    if (from_index == to_index) {
      continue;
    }
    const ViewpointEntry& to_viewpoint = viewpoint_entries_[to_index];
    Motion motion;
    bool found_motion;
    std::tie(motion, found_motion) = motion_planner_.findMotion(from_viewpoint.viewpoint.pose(), to_viewpoint.viewpoint.pose());
    if (found_motion) {
#if !AIT_DEBUG
#pragma omp critical
#endif
      {
        ++num_connections;
        // Additional sanity check
//        FloatType motion_cost = motion.cost;
//        if (!ait::isApproxGreaterEqual(motion_cost * motion_cost, dist_square, 1e-2f)) {
//          FloatType dist_square2 = (from_viewpoint.viewpoint.pose().getWorldPosition() - to_viewpoint.viewpoint.pose().getWorldPosition()).squaredNorm();
//          std::cout << "motion_cost=" << motion_cost << std::endl;
//          std::cout << "motion_cost*motion_cost=" << motion_cost * motion_cost << std::endl;
//          std::cout << "dist_square=" << dist_square << std::endl;
//          std::cout << "dist_square2=" << dist_square2 << std::endl;
//          std::cout << "motion_cost * motion_cost - dist_square=" << motion_cost * motion_cost - dist_square << std::endl;
//        }
        AIT_ASSERT(ait::isApproxGreaterEqual(motion.cost * motion.cost, dist_square, 1e-2f));
        motions.emplace_back(from_index, to_index, motion);
      }
    }
  }
//  timer.printTimingMs("ViewpointPlanner::findMotion()");
  return motions;
}

ViewpointPlanner::FloatType ViewpointPlanner::computeNewInformation(
    const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data,
    const ViewpointEntryIndex new_viewpoint_index) const {
  const ViewpointEntry& new_viewpoint = viewpoint_entries_[new_viewpoint_index];
  if (options_.consider_triangulation) {
    FloatType new_information = std::accumulate(new_viewpoint.voxel_set.cbegin(), new_viewpoint.voxel_set.cend(),
          FloatType { 0 }, [&](const FloatType& value, const VoxelWithInformation& voxel) {
            FloatType information = 0;
            if (viewpoint_path.observed_voxel_set.count(voxel) == 0) {
              bool can_be_triangulated;
              std::tie(can_be_triangulated, std::ignore) = canVoxelBeTriangulated(viewpoint_path, comp_data, new_viewpoint, voxel);
              if (can_be_triangulated) {
                information = voxel.information / options_.triangulation_min_count;
              }
            }
            return value + information;
      });
    return new_information;
  }
  else {
    VoxelWithInformationSet difference_set = ait::computeSetDifference(new_viewpoint.voxel_set, viewpoint_path.observed_voxel_set);
    FloatType new_information = std::accumulate(difference_set.cbegin(), difference_set.cend(),
        FloatType { 0 }, [](const FloatType& value, const VoxelWithInformation& voxel) {
          return value + voxel.information;
    });
    return new_information;
  }
}

std::pair<bool, ViewpointPlanner::ViewpointEntryIndex> ViewpointPlanner::canVoxelBeTriangulated(
    const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data,
    const ViewpointEntry& new_viewpoint, const VoxelWithInformation& voxel) const {
  auto found_it = comp_data.voxel_observation_counts.find(voxel.voxel);
  return canVoxelBeTriangulated(viewpoint_path, comp_data, new_viewpoint, found_it);
}

void ViewpointPlanner::initializeViewpointPathInformations(ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data) {
  if (!options_.ignore_real_observed_voxels) {
    for (std::size_t i = 0; i < num_real_viewpoints_; ++i) {
      const ViewpointEntry& viewpoint_entry = viewpoint_entries_[i];
      viewpoint_path->observed_voxel_set.insert(viewpoint_entry.voxel_set.cbegin(), viewpoint_entry.voxel_set.cend());
    }
  }
  comp_data->sorted_new_informations.clear();
  for (auto it = viewpoint_graph_.begin(); it != viewpoint_graph_.end(); ++it) {
    const ViewpointEntryIndex viewpoint_index = it.node();
    const ViewpointEntry& viewpoint_entry = viewpoint_entries_[viewpoint_index];
    comp_data->sorted_new_informations.push_back(std::make_pair(viewpoint_index, viewpoint_entry.total_information));
  }
  std::sort(comp_data->sorted_new_informations.begin(), comp_data->sorted_new_informations.end(),
      [](const std::pair<ViewpointEntryIndex, FloatType>& a, const std::pair<ViewpointEntryIndex, FloatType>& b) {
        return a.second < b.second;
  });
}

void ViewpointPlanner::updateViewpointPathInformations(ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data) const {
  AIT_ASSERT(!comp_data->sorted_new_informations.empty());
//  // Test code for checking the lazy update scheme
//  for (auto it = comp_data->sorted_new_informations.begin(); it != comp_data->sorted_new_informations.end(); ++it) {
//    const FloatType new_information = computeNewInformation(viewpoint_path, it->first);
//    it->second = new_information;
//  }
//  std::sort(comp_data->sorted_new_informations.begin(), comp_data->sorted_new_informations.end(),
//      [](const std::pair<ViewpointEntryIndex, FloatType>& a, const std::pair<ViewpointEntryIndex, FloatType>& b) {
//        return a.second < b.second;
//  });
//  // End of test code

  if (options_.consider_triangulation) {
    // Updating information considering triangulation (array is not sorted)
    for (auto it = comp_data->sorted_new_informations.begin(); it != comp_data->sorted_new_informations.end(); ++it) {
      const ViewpointEntryIndex viewpoint_index = it->first;
      const auto other_it = std::find_if(viewpoint_path->entries.begin(), viewpoint_path->entries.end(),
          [viewpoint_index](const ViewpointPathEntry& path_entry) {
        return viewpoint_index == path_entry.viewpoint_index;
      });
      if (other_it != viewpoint_path->entries.end()) {
        it->second = 0;
      }
      else {
        it->second = computeNewInformation(*viewpoint_path, *comp_data, viewpoint_index);
      }
    }
  }
  else {
    // Updating information without considering triangulation
    // TODO: This would be better to do with a binary heap
    // Our function is sub-modular so we can lazily update the best entries
    std::size_t recompute_count = 0;
    bool change_occured;
    do {
      change_occured = false;
      auto best_it = comp_data->sorted_new_informations.rbegin();
      best_it->second = computeNewInformation(*viewpoint_path, *comp_data, best_it->first);
      ++recompute_count;
      for (auto it = comp_data->sorted_new_informations.rbegin() + 1; it != comp_data->sorted_new_informations.rend(); ++it) {
        if (it->second > (it-1)->second) {
          change_occured = true;
          // Swap with higher entry if update gives better information score
          swap(*it, *(it - 1));
        }
        else {
          break;
        }
      }
    } while (change_occured);
    std::cout << "Recomputed " << recompute_count << " of " << comp_data->sorted_new_informations.size() << " viewpoints" << std::endl;
  }
}
std::pair<ViewpointPlanner::ViewpointEntryIndex, ViewpointPlanner::FloatType> ViewpointPlanner::getBestNextViewpoint(
      const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data) const {
  if (options_.consider_triangulation) {
    auto best_it = comp_data.sorted_new_informations.end();
    FloatType best_information = std::numeric_limits<FloatType>::lowest();
    for (auto it = comp_data.sorted_new_informations.begin(); it != comp_data.sorted_new_informations.end(); ++it) {
      if (it->second > best_information) {
        best_it = it;
        best_information = it->second;
      }
    }
    return std::make_pair(best_it->first, best_it->second);
  }
  else {
    const bool is_primary_path = &viewpoint_path == &viewpoint_paths_.front();
    if (is_primary_path) {
      return std::make_pair(comp_data.sorted_new_informations.back().first, comp_data.sorted_new_informations.back().second);
    }
    else {
      // TODO: Make as parameters
      const std::size_t num_of_good_viewpoints_to_sample_from = 100;
      auto it = random_.sampleDiscreteWeighted(
          comp_data.sorted_new_informations.cend() - num_of_good_viewpoints_to_sample_from,
          comp_data.sorted_new_informations.cend(),
          [](const std::pair<ViewpointEntryIndex, FloatType>& entry) {
        const FloatType information = entry.second;
        return information;
      });
      if (it == comp_data.sorted_new_informations.cend()) {
        it = comp_data.sorted_new_informations.cend() - 1;
      }
      return std::make_pair(it->first, it->second);
    }
  }
}

// TODO: Upper bound is wrong when voxels are triangulated
ViewpointPlanner::FloatType ViewpointPlanner::computeViewpointPathInformationUpperBound(
    const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data, const std::size_t max_num_viewpoints) const {
  FloatType information_upper_bound = viewpoint_path.acc_objective;
  const std::size_t num_viewpoints = std::min(max_num_viewpoints, comp_data.sorted_new_informations.size());
  for (auto it = comp_data.sorted_new_informations.rbegin(); it != comp_data.sorted_new_informations.rbegin() + num_viewpoints; ++it) {
    const FloatType information = it->second;
    information_upper_bound += information;
  }
  return information_upper_bound;
}

bool ViewpointPlanner::addMatchingStereoViewpointWithoutLock(
    ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data, const ViewpointPathEntry& path_entry) {
  const bool verbose = true;

  const ViewpointEntry& viewpoint_entry = viewpoint_entries_[path_entry.viewpoint_index];

  // Compute center position of voxels with information
  Vector3 voxel_center = Vector3::Zero();
  // TODO
//  std::accumulate(viewpoint_entry.voxel_set.begin(), viewpoint_entry.voxel_set.end(), voxel_center,
//      [](Vector3& vector, const VoxelWithInformation& vi) {
//    vector += vi.voxel->getBoundingBox().getCenter();
//    return vector;
//  });
  for (const VoxelWithInformation& vi : viewpoint_entry.voxel_set) {
    voxel_center += vi.voxel->getBoundingBox().getCenter();
  }
  voxel_center /= viewpoint_entry.voxel_set.size();

  // Compute minimum and maximum stereo baseline
  const Pose& pose = viewpoint_entry.viewpoint.pose();
  const FloatType dist_to_voxel_sq = (voxel_center - pose.getWorldPosition()).squaredNorm();
  const FloatType min_baseline_square = 4 * dist_to_voxel_sq * triangulation_min_sin_angle_square_;
  const FloatType max_baseline_square = 4 * dist_to_voxel_sq * triangulation_max_sin_angle_square_;
  std::cout << "min_baseline_square=" << min_baseline_square << std::endl;
  std::cout << "max_baseline_square=" << max_baseline_square << std::endl;

  const auto is_stereo_pair_lambda = [&](const ViewpointEntry& other_viewpoint, const bool ignore_angular_deviation) -> bool {
    const Pose& other_pose = other_viewpoint.viewpoint.pose();
    const Vector3 rel_pose_vector = other_pose.getWorldPosition() - pose.getWorldPosition();
    const Vector3 mid_point = (pose.getWorldPosition() + other_pose.getWorldPosition()) / 2;
    const Vector3 mid_point_to_voxel_center = voxel_center - mid_point;
    const Vector3 mid_point_to_voxel_center_norm = mid_point_to_voxel_center.normalized();
    const FloatType dist_deviation = mid_point_to_voxel_center_norm.dot(rel_pose_vector);
    const FloatType dist_deviation_ratio = std::abs(dist_deviation) / mid_point_to_voxel_center.norm();
    const Vector3 baseline_vector = rel_pose_vector - dist_deviation * mid_point_to_voxel_center_norm;
    const FloatType baseline_square = baseline_vector.squaredNorm();
//      const FloatType other_dist_deviation_ratio = std::abs(other_dist_to_voxel_sq - dist_to_voxel_sq) / dist_to_voxel_sq;
    const FloatType angular_deviation = other_pose.quaternion().angularDistance(pose.quaternion());
    AIT_PRINT_VALUE(rel_pose_vector.transpose());
    AIT_PRINT_VALUE(mid_point_to_voxel_center_norm.transpose());
    AIT_PRINT_VALUE(dist_deviation);
    AIT_PRINT_VALUE(dist_deviation_ratio);
    AIT_PRINT_VALUE(baseline_square);
    AIT_PRINT_VALUE(angular_deviation);
    const bool is_dist_deviation_valid = dist_deviation_ratio <= options_.triangulation_max_dist_deviation_ratio;
    const bool is_baseline_valid = baseline_square >= min_baseline_square && baseline_square <= max_baseline_square;
    bool is_angular_deviation_valid = angular_deviation <= triangulation_max_angular_deviation_;
    AIT_PRINT_VALUE(is_dist_deviation_valid);
    AIT_PRINT_VALUE(is_angular_deviation_valid);
    AIT_PRINT_VALUE(is_baseline_valid);
    if (ignore_angular_deviation) {
      is_angular_deviation_valid = true;
    }
    if (is_dist_deviation_valid && is_baseline_valid && is_angular_deviation_valid) {
      return true;
    }
    else {
      return false;
    }
  };

  // Check if a matching viewpoint already exists
  std::size_t i = 0;
  for (const ViewpointPathEntry& other_path_entry : viewpoint_path->entries) {
    std::cout << "i=" << i << std::endl;
    ++i;
    // TODO: Check shouldn't be necessary
    if (other_path_entry.viewpoint_index != path_entry.viewpoint_index) {
      const ViewpointEntryIndex& other_index = other_path_entry.viewpoint_index;
      std::cout << "other_index=" << other_index << std::endl;
      const ViewpointEntry& other_viewpoint = viewpoint_entries_[other_index];
      const bool ignore_angular_deviation = false;
      if (is_stereo_pair_lambda(other_viewpoint, ignore_angular_deviation)) {
        return true;
      }
    }
  }

  const std::vector<std::size_t>& component = getConnectedComponents().first;

  // Find best viewpoint in baseline range
  // TODO: Should use a radius search??
  ViewpointEntryIndex best_index = (ViewpointEntryIndex)-1;
  FloatType best_information = std::numeric_limits<FloatType>::lowest();
  for (const auto& entry : comp_data->sorted_new_informations) {
    const ViewpointEntryIndex& stereo_index = entry.first;
    const FloatType stereo_information = entry.second;
    const ViewpointEntry& stereo_viewpoint = viewpoint_entries_[stereo_index];
    const bool ignore_angular_deviation = true;
    if (is_stereo_pair_lambda(stereo_viewpoint, ignore_angular_deviation)) {
      if (component[stereo_index] == component[path_entry.viewpoint_index] && stereo_information > best_information) {
        best_index = stereo_index;
        best_information = stereo_information;
      }
    }
  }
  AIT_PRINT_VALUE(best_index);
  AIT_PRINT_VALUE(best_information);
  if (best_index == (ViewpointEntryIndex)-1) {
    if (verbose) {
      std::cout << "Could not find any stereo viewpoint in the right baseline and distance range" << std::endl;
    }
    return false;
  }

  // Add viewpoint candidate with same translation as best found stereo viewpoint and same orientation as reference viewpoint.
  const ViewpointEntry& best_viewpoint_entry = viewpoint_entries_[best_index];
  AIT_PRINT_VALUE(best_viewpoint_entry.total_information);
  const Pose new_pose = Pose::createFromImageToWorldTransformation(
      best_viewpoint_entry.viewpoint.pose().translation(),
      viewpoint_entry.viewpoint.pose().quaternion());
  const Viewpoint new_viewpoint(&viewpoint_entry.viewpoint.camera(), new_pose);
  // TODO: Check if this really makes sense or if a new raycast should be done.
  ViewpointEntry new_viewpoint_entry(new_viewpoint, viewpoint_entry.total_information, viewpoint_entry.voxel_set);
  const std::size_t stereo_viewpoint_index = addViewpointEntryWithoutLock(std::move(new_viewpoint_entry));

  ViewpointPathEntry stereo_path_entry;
  stereo_path_entry.viewpoint_index = stereo_viewpoint_index;
  stereo_path_entry.local_information = 0;
  stereo_path_entry.local_motion_distance = 0;
  stereo_path_entry.local_objective = stereo_path_entry.local_information;

//  if (stereo_path_entry.local_objective <= 0) {
//    if (verbose) {
//      std::cout << "No improvement in objective in stereo viewpoint" << std::endl;
//    }
//    return false;
//  }

  // Make sure stereo viewpoints can be connected
//  if (!findAndAddShortestMotion(path_entry.viewpoint_index, stereo_path_entry.viewpoint_index)) {
//    if (verbose) {
//      std::cout << "Could not find connection between stereo pair" << std::endl;
//    }
//    return false;
//  }

  const ViewpointEntry& stereo_viewpoint_entry = viewpoint_entries_[stereo_path_entry.viewpoint_index];

  // Sanity check. Make sure the best found stereo viewpoint is not already on the path.
  if (std::find_if(viewpoint_path->entries.begin(), viewpoint_path->entries.end(),
      [&](const ViewpointPathEntry& entry) {
    return entry.viewpoint_index == stereo_path_entry.viewpoint_index;
  }) != viewpoint_path->entries.end()) {
    AIT_DEBUG_BREAK;
  }

  if (options_.consider_triangulation) {
    for (const VoxelWithInformation& voxel : stereo_viewpoint_entry.voxel_set) {
      if (viewpoint_path->observed_voxel_set.count(voxel) == 0) {
        auto found_it = comp_data->voxel_observation_counts.find(voxel.voxel);
        bool triangulated;
        ViewpointEntryIndex other_index;
        std::tie(triangulated, other_index) = canVoxelBeTriangulated(*viewpoint_path, *comp_data, stereo_viewpoint_entry, found_it);
        if (triangulated) {
          std::vector<ViewpointEntryIndex>& observing_entries = found_it->second.observing_entries;
          ++found_it->second.num_triangulated;
          bool erased = false;
          if (found_it->second.num_triangulated >= options_.triangulation_min_count) {
            // Voxel can be observed
            comp_data->voxel_observation_counts.erase(found_it);
            viewpoint_path->observed_voxel_set.emplace(voxel);
            erased = true;
          }
          if (!erased) {
            AIT_ASSERT(std::find(observing_entries.begin(), observing_entries.end(), stereo_path_entry.viewpoint_index)
              == observing_entries.end());
            observing_entries.push_back(stereo_path_entry.viewpoint_index);
          }
          // Hack to track which viewpoints triangulated which voxels
          auto it_tmp = comp_data->triangulated_voxel_to_path_entries_map.find(voxel.voxel);
          if (it_tmp == comp_data->triangulated_voxel_to_path_entries_map.end()) {
            comp_data->triangulated_voxel_to_path_entries_map.emplace(
                voxel.voxel, std::vector<std::pair<ViewpointEntryIndex, ViewpointEntryIndex>>());
          }
          std::pair<ViewpointEntryIndex, ViewpointEntryIndex> p = std::make_pair(stereo_path_entry.viewpoint_index, other_index);
          comp_data->triangulated_voxel_to_path_entries_map.at(voxel.voxel).push_back(p);
        }
      }
    }
  }
  else {
      viewpoint_path->observed_voxel_set.insert(stereo_viewpoint_entry.voxel_set.cbegin(), stereo_viewpoint_entry.voxel_set.cend());
  }
  viewpoint_path->entries.emplace_back(std::move(stereo_path_entry));
  viewpoint_path->acc_information += stereo_path_entry.local_information;
  viewpoint_path->acc_motion_distance += stereo_path_entry.local_motion_distance;
  viewpoint_path->acc_objective += stereo_path_entry.local_objective;
  viewpoint_path->entries.back().acc_information = viewpoint_path->acc_information;
  viewpoint_path->entries.back().acc_motion_distance = viewpoint_path->acc_motion_distance;
  viewpoint_path->entries.back().acc_objective = viewpoint_path->acc_objective;

  // Make sure stereo viewpoints can be connected
  auto edges = viewpoint_graph_.getEdgesByNode(best_index);
  for (auto it = edges.begin(); it != edges.end(); ++it) {
    Motion motion = getViewpointMotion(best_index, it.targetNode());
    addViewpointMotion(stereo_viewpoint_index, it.targetNode(), std::move(motion));
  }

  return true;
}

bool ViewpointPlanner::findNextViewpointPathEntry(
    ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data, const FloatType alpha, const FloatType beta) {
  const bool verbose = true;

  // TODO: The mutex should be locked when updating viewpoint information
  updateViewpointPathInformations(viewpoint_path, comp_data);
  ViewpointEntryIndex new_viewpoint_index;
  FloatType new_information;
  std::tie(new_viewpoint_index, new_information) = getBestNextViewpoint(*viewpoint_path, *comp_data);

  ViewpointPathEntry best_path_entry;
  best_path_entry.viewpoint_index = new_viewpoint_index;
  best_path_entry.local_information = new_information;
  best_path_entry.local_motion_distance = 0;
  best_path_entry.local_objective = best_path_entry.local_information;

//  ViewpointEntryIndex from_index = viewpoint_path->entries.back().viewpoint_index;
//  // Run through the neighbors and select the best next viewpoint
////  for (const auto& edge : viewpoint_graph_.getEdgesByNode(from_index)) {
//  //    ViewpointEntryIndex to_index = edge.first;
//  for (const ViewpointEntryIndex to_index : viewpoint_graph_) {
//    const ViewpointEntry& to_viewpoint = viewpoint_entries_[to_index];
//    VoxelWithInformationSet difference_set = ait::computeSetDifference(to_viewpoint.voxel_set, viewpoint_path->observed_voxel_set);
////    std::cout << "  j=" << j << ", next_node.voxel_set.size()= " << next_node.voxel_set.size() << std::endl;
////    std::cout << "  j=" << j << ", difference_set.size()= " << difference_set.size() << std::endl;
//    FloatType new_information = std::accumulate(difference_set.cbegin(), difference_set.cend(),
//        FloatType { 0 }, [](const FloatType& value, const VoxelWithInformation& voxel) {
//          return value + voxel.information;
//    });
////      if (verbose) {
////        std::cout << "difference_set.size()=" << difference_set.size() <<
////            ", new_information=" << new_information << std::endl;
////      }
//    float motion_distance = viewpoint_graph_.getWeight(from_index, to_index);
//    FloatType new_objective = new_information - alpha - beta * motion_distance * motion_distance;
////    std::cout << "  j=" << j << ", new_information=" << new_information << std::endl;
//    if (verbose) {
//      std::cout << "to_index=" << to_index
//          << ", voxel_set.size()=" << to_viewpoint.voxel_set.size()
//          << ", new_information=" << new_information
//          << ", motion_distance=" << motion_distance
//          << ", new_objective=" << new_objective << std::endl;
//    }
//    if (new_objective > best_path_entry.local_objective) {
//      best_path_entry.viewpoint_index = to_index;
//      best_path_entry.local_information = new_information;
//      best_path_entry.local_motion_distance = motion_distance;
//      best_path_entry.local_objective = new_objective;
//    }
//  }

  if (best_path_entry.viewpoint_index == (ViewpointEntryIndex)-1) {
    return false;
  }
  AIT_ASSERT(best_path_entry.viewpoint_index != (ViewpointEntryIndex)-1);

  if (best_path_entry.local_objective <= 0) {
    if (verbose) {
      std::cout << "No improvement in objective" << std::endl;
    }
    return false;
  }

  const ViewpointEntry& best_viewpoint_entry = viewpoint_entries_[best_path_entry.viewpoint_index];
//  if (verbose) {
//    // TODO: remove this set difference (duplicate anyway)
//    VoxelWithInformationSet difference_set = ait::computeSetDifference(best_viewpoint_entry.voxel_set, viewpoint_path->observed_voxel_set);
//    std::cout << "Found viewpoint with local_information=" << best_path_entry.local_information <<
//        ", total_information=" << best_viewpoint_entry.total_information <<
//        ", total_voxels=" << best_viewpoint_entry.voxel_set.size() <<
//        ", new_voxels=" << difference_set.size() << std::endl;
//  }

  std::unique_lock<std::mutex> lock(mutex_);

//  if (verbose) {
//    for (auto entry : viewpoint_path->entries) {
//      std::cout << "entry: " << entry.viewpoint_index << std::endl;
//    }
//    std::cout << "best entry: " << best_path_entry.viewpoint_index << std::endl;
//  }

  // Check that graph is connected
  const std::vector<std::size_t>& component = getConnectedComponents().first;
//  if (verbose) {
//    std::cout << "Number of connected components: " << num_components << std::endl;
//    for (auto entry : viewpoint_path->entries) {
//      std::cout << "  entry " << entry.viewpoint_index << " belongs to component " << component[entry.viewpoint_index] << std::endl;
//    }
//    std::cout << "new viewpoint " << best_path_entry.viewpoint_index << " belongs to component "
//        << component[best_path_entry.viewpoint_index] << std::endl;
//  }
  if (component[best_path_entry.viewpoint_index] != component[viewpoint_path->entries.front().viewpoint_index]) {
    if (verbose) {
      std::cout << "New viewpoint does not belong to the same connected component as the viewpoint path" << std::endl;
    }
    return false;
  }

  if (std::find_if(viewpoint_path->entries.begin(), viewpoint_path->entries.end(),
      [&](const ViewpointPathEntry& entry) {
    return entry.viewpoint_index == best_path_entry.viewpoint_index;
  }) != viewpoint_path->entries.end()) {
    AIT_DEBUG_BREAK;
  }

  if (options_.viewpoint_generate_stereo_pairs) {
    // Try to find a matching stereo pair. Otherwise discard.
    if (!addMatchingStereoViewpointWithoutLock(viewpoint_path, comp_data, best_path_entry)) {
      if (verbose) {
        std::cout << "Could not find any usable stereo viewpoint for viewpoint " << best_path_entry.viewpoint_index << std::endl;
      }
      return false;
    }
  }

  if (options_.consider_triangulation) {
    for (const VoxelWithInformation& voxel : best_viewpoint_entry.voxel_set) {
      if (viewpoint_path->observed_voxel_set.count(voxel) == 0) {
        auto found_it = comp_data->voxel_observation_counts.find(voxel.voxel);
        bool triangulated;
        ViewpointEntryIndex other_index;
        std::tie(triangulated, other_index) = canVoxelBeTriangulated(*viewpoint_path, *comp_data, best_viewpoint_entry, found_it);
        if (triangulated) {
          std::vector<ViewpointEntryIndex>& observing_entries = found_it->second.observing_entries;
          ++found_it->second.num_triangulated;
          bool erased = false;
          if (found_it->second.num_triangulated >= options_.triangulation_min_count) {
            // Voxel can be observed
            comp_data->voxel_observation_counts.erase(found_it);
            viewpoint_path->observed_voxel_set.emplace(voxel);
            erased = true;
          }
          if (!erased) {
            AIT_ASSERT(std::find(observing_entries.begin(), observing_entries.end(), best_path_entry.viewpoint_index)
              == observing_entries.end());
            observing_entries.push_back(best_path_entry.viewpoint_index);
          }
          // Hack to track which viewpoints triangulated which voxels
          auto it_tmp = comp_data->triangulated_voxel_to_path_entries_map.find(voxel.voxel);
          if (it_tmp == comp_data->triangulated_voxel_to_path_entries_map.end()) {
            comp_data->triangulated_voxel_to_path_entries_map.emplace(
                voxel.voxel, std::vector<std::pair<ViewpointEntryIndex, ViewpointEntryIndex>>());
          }
          std::pair<ViewpointEntryIndex, ViewpointEntryIndex> p = std::make_pair(best_path_entry.viewpoint_index, other_index);
          comp_data->triangulated_voxel_to_path_entries_map.at(voxel.voxel).push_back(p);
        }
      }
    }
  }
  else {
      viewpoint_path->observed_voxel_set.insert(best_viewpoint_entry.voxel_set.cbegin(), best_viewpoint_entry.voxel_set.cend());
  }
  viewpoint_path->entries.emplace_back(std::move(best_path_entry));
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

//  if (verbose) {
//    std::cout << "Path: " << std::endl;
//    for (std::size_t i = 0; i < viewpoint_path->entries.size(); ++i) {
//      const ViewpointPathEntry& path_entry = viewpoint_path->entries[i];
//      const ViewpointEntry& viewpoint_entry = viewpoint_entries_[path_entry.viewpoint_index];
//      std::cout << "  viewpoint_index: " << path_entry.viewpoint_index
//          << ", pose[" << i << "]: " << viewpoint_entry.viewpoint.pose().getWorldPosition().transpose()
//          << ", local_information: " << path_entry.local_information
//          << ", global_information: " << path_entry.acc_information
//          << ", local_motion_distance: " << path_entry.local_motion_distance
//          << ", global_motion_distance: " << path_entry.acc_motion_distance
//          << ", local_objective: " << path_entry.local_objective
//          << ", global_objective: " << path_entry.acc_objective << std::endl;
//    }
//  }

//  if (verbose) {
//    std::cout << "Done." << std::endl;
//  }

  return true;
}

void ViewpointPlanner::findInitialViewpointPathEntries(const FloatType alpha, const FloatType beta) {
  if (viewpoint_paths_.size() > viewpoint_graph_.size()) {
    // Make sure we don't expect too many viewpoints
    viewpoint_paths_.resize(viewpoint_graph_.size());
    viewpoint_paths_data_.resize(viewpoint_graph_.size());
  }

  std::vector<ViewpointPathEntry> best_path_entries;

  // Find the overall best viewpoints with some min distance as starting points
  std::vector<ViewpointEntryIndex> entries;
  for (auto it = viewpoint_graph_.begin(); it != viewpoint_graph_.end(); ++it) {
    entries.push_back(it.node());
  }
  for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
    ViewpointPathEntry best_path_entry;
    // Make sure there are viewpoint entries left (otherwise just take one that was already selected)
    if (entries.empty()) {
      best_path_entry = best_path_entries.back();
    }
    else {
      for (ViewpointEntryIndex viewpoint_index : entries) {
        const ViewpointEntry& viewpoint_entry = viewpoint_entries_[viewpoint_index];
        FloatType new_information = viewpoint_entry.total_information;
        FloatType new_objective = new_information - alpha;
        if (new_objective > best_path_entry.local_objective) {
          best_path_entry.viewpoint_index = viewpoint_index;
          best_path_entry.local_information = new_information;
          best_path_entry.local_motion_distance = 0;
          best_path_entry.local_objective = new_objective;
          best_path_entry.acc_information = new_information;
          best_path_entry.acc_motion_distance = 0;
          best_path_entry.acc_objective = new_objective;
        }
      }
    }
    best_path_entries.push_back(std::move(best_path_entry));

    // Remove all viewpoints that are too close to the found best one
    const ViewpointEntry& viewpoint = viewpoint_entries_[best_path_entry.viewpoint_index];
    for (auto it = entries.begin(); it != entries.end();) {
      const ViewpointEntry& other_viewpoint = viewpoint_entries_[*it];
      const FloatType distance = (other_viewpoint.viewpoint.pose().getWorldPosition() - viewpoint.viewpoint.pose().getWorldPosition()).norm();
      if (distance < options_.viewpoint_path_initial_distance) {
        it = entries.erase(it);
      }
      else {
        ++it;
      }
    }
  }

  std::unique_lock<std::mutex> lock(mutex_);
  for (auto it = viewpoint_paths_.begin(); it != viewpoint_paths_.end(); ++it) {
    AIT_ASSERT(it->entries.empty());
    ViewpointPathComputationData& comp_data = viewpoint_paths_data_[it - viewpoint_paths_.begin()];
    const ViewpointPathEntry& best_path_entry = best_path_entries[it - viewpoint_paths_.begin()];
    const ViewpointEntry& best_viewpoint_entry = viewpoint_entries_[best_path_entry.viewpoint_index];
    it->acc_information = best_path_entry.acc_information;
    it->acc_motion_distance = best_path_entry.acc_motion_distance;
    it->acc_objective = best_path_entry.acc_objective;
    it->entries.emplace_back(std::move(best_path_entry));
    if (options_.consider_triangulation) {
      for (const VoxelWithInformation& voxel : best_viewpoint_entry.voxel_set) {
        ViewpointPathComputationData::VoxelTriangulation vt;
        vt.observing_entries.push_back(best_path_entry.viewpoint_index);
        comp_data.voxel_observation_counts.emplace(voxel.voxel, std::move(vt));
      }
    }
    else {
      it->observed_voxel_set.insert(best_viewpoint_entry.voxel_set.cbegin(), best_viewpoint_entry.voxel_set.cend());
    }
    comp_data.num_connected_entries = 1;
    std::cout << "Initial viewpoint [" << (it - viewpoint_paths_.begin())
        << "]: acc_objective=" << it->acc_objective << std::endl;
  }
  lock.unlock();
}

void ViewpointPlanner::reportViewpointPathsStats() const {
  for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
    const ViewpointPath& viewpoint_path = viewpoint_paths_[i];
    const ViewpointPathComputationData& comp_data = viewpoint_paths_data_[i];
    // Compute upper bound assuming we have all viewpoints
    const std::size_t max_num_viewpoints = viewpoint_path.entries.size();
    const FloatType information_upper_bound = computeViewpointPathInformationUpperBound(viewpoint_path, comp_data, max_num_viewpoints);
    std::cout << "Current information for branch " << i << ": " << viewpoint_path.acc_information
        << ", upper bound: " << information_upper_bound
        << ", ratio: " << (viewpoint_path.acc_information / information_upper_bound) << std::endl;
  }
}

bool ViewpointPlanner::findNextViewpointPathEntries(const FloatType alpha, const FloatType beta) {
  if (viewpoint_paths_.front().entries.empty()) {
    findInitialViewpointPathEntries(alpha, beta);
    std::vector<bool> results(viewpoint_paths_.size());
#pragma omp parallel for
    for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
      ViewpointPath& viewpoint_path = viewpoint_paths_[i];
      ViewpointPathComputationData& comp_data = viewpoint_paths_data_[i];
      initializeViewpointPathInformations(&viewpoint_path, &comp_data);
      if (options_.viewpoint_generate_stereo_pairs) {
        results[i] = addMatchingStereoViewpointWithoutLock(&viewpoint_path, &comp_data, viewpoint_path.entries[0]);
      }
    }
    if (options_.viewpoint_generate_stereo_pairs) {
      for (bool result : results) {
        if (!result) {
          std::cout << "Not all initialized viewpoints could be matched with a stereo pair" << std::endl;
          viewpoint_paths_.clear();
          viewpoint_paths_data_.clear();
          return false;
        }
      }
    }
    reportViewpointPathsStats();
    return true;
  }

  std::size_t changes = 0;
#pragma omp parallel for \
  reduction(+:changes)
  for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
    ViewpointPath& viewpoint_path = viewpoint_paths_[i];
    ViewpointPathComputationData& comp_data = viewpoint_paths_data_[i];
    changes += findNextViewpointPathEntry(&viewpoint_path, &comp_data, alpha, beta);
  }
  reportViewpointPathsStats();

  std::cout << "changes=" << changes << ", alpha=" << alpha << ", beta=" << beta << std::endl;
  if (changes == 0) {
    return false;
  }
  else {
    return true;
  }
}

void ViewpointPlanner::computeViewpointTour() {
#pragma omp parallel for
  for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
    ViewpointPath& viewpoint_path = viewpoint_paths_[i];
    ViewpointPathComputationData& comp_data = viewpoint_paths_data_[i];
    ensureConnectedViewpointPath(&viewpoint_path, &comp_data);
  }

  // Solve TSP problem
#pragma omp parallel for
  for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
    ViewpointPath& viewpoint_path = viewpoint_paths_[i];
    ViewpointPathComputationData& comp_data = viewpoint_paths_data_[i];
    solveApproximateTSP(&viewpoint_path, &comp_data);
  }

  // Improve solution with 2 Opt
#pragma omp parallel for
  for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
    ViewpointPath& viewpoint_path = viewpoint_paths_[i];
    ViewpointPathComputationData& comp_data = viewpoint_paths_data_[i];
    improveViewpointTourWith2Opt(&viewpoint_path, &comp_data);
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
