//==================================================
// viewpoint_planner.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 12, 2016
//==================================================

#include <functional>
#include <random>
#include <boost/serialization/unordered_map.hpp>
#include <ait/common.h>
#include <ait/math.h>
#include <ait/utilities.h>
#include <ait/eigen_utils.h>
#include <bh/vision/cameras.h>
#include "viewpoint_planner.h"

using std::swap;

ViewpointPlanner::ViewpointPlanner(
    const Options* options, const MotionPlannerType::Options* motion_options, std::unique_ptr<ViewpointPlannerData> data)
: options_(*options), data_(std::move(data)),
  raycaster_(&data_->occupied_bvh_, options_.raycast_min_range, options_.raycast_max_range),
  scorer_(viewpoint_planner::ViewpointScore::Options(), [&](const Viewpoint& viewpoint, const VoxelType* node, const Vector2& image_coordinates) -> Vector3 {
    return computeNormalVector(viewpoint, node, image_coordinates);
  }),
  motion_planner_(motion_options, data_.get(), options->motion_planner_log_filename),
  viewpoint_sampling_distribution_update_size_(0),
  viewpoint_graph_components_valid_(false), viewpoint_paths_initialized_(false) {
#if WITH_CUDA
  raycaster_.setEnableCuda(options_.enable_cuda);
#endif
  size_t random_seed = options_.rng_seed;
  if (random_seed == 0) {
    random_seed = std::chrono::system_clock::now().time_since_epoch().count();
  }
  random_.setSeed(random_seed);
  if (options_.virtual_camera_width > 0) {
    AIT_ASSERT(options_.virtual_camera_height > 0);
    AIT_ASSERT(options_.virtual_camera_focal_length > 0);
    setVirtualCamera(options_.virtual_camera_width, options_.virtual_camera_height, options_.virtual_camera_focal_length);
  }
  else {
    AIT_ASSERT(static_cast<bool>(hasReconstruction()));
    AIT_ASSERT(data_->reconstruction_->getCameras().size() > 0);
    setScaledVirtualCamera(data_->reconstruction_->getCameras().cbegin()->first, options_.virtual_camera_scale);
  }
  offscreen_renderer_.reset(new viewpoint_planner::ViewpointOffscreenRenderer(
          options_.getOptionsAs<viewpoint_planner::ViewpointOffscreenRenderer::Options>(),
          getVirtualCamera(), data_->poisson_mesh_.get()));
  drone_bbox_ = BoundingBoxType(
          options_.drone_bbox_min,
          options_.drone_bbox_max);
  // Compute bounding boxes for viewpoint sampling and motion sampling
  Vector3 roi_bbox_extent;
  roi_bbox_extent(0) = 2 * data_->roi_.getPolygon2D().getEnclosingRadius();
  roi_bbox_extent(1) = roi_bbox_extent(0);
  roi_bbox_extent(2) = data_->roi_.getUpperPlaneZ() - data_->roi_.getLowerPlaneZ();
  if (options_.isSet("sampling_bbox_min")) {
    AIT_ASSERT(options_.isSet("sampling_bbox_max"));
    pose_sample_bbox_ = BoundingBoxType(
        options_.getValue<Vector3>("sampling_bbox_min"),
        options_.getValue<Vector3>("sampling_bbox_max"));
  }
  else {
    pose_sample_bbox_ = BoundingBoxType::createFromCenterAndExtent(
        data_->roi_.getCentroid(), roi_bbox_extent * options_.sampling_roi_factor);
  }
  BH_PRINT_VALUE(pose_sample_bbox_);

  const std::size_t count_grid_dim = options_.viewpoint_count_grid_dimension;
  viewpoint_count_grid_= ContinuousGridType(
      pose_sample_bbox_, count_grid_dim, count_grid_dim, count_grid_dim);
  viewpoint_count_grid_.setAllValues(0);
  std::cout << "Viewpoint count grid dimension: " << viewpoint_count_grid_.getDimensions().transpose() << std::endl;
  std::cout << "Viewpoint count grid increment: " << viewpoint_count_grid_.getGridIncrement().transpose() << std::endl;

  const FloatType motion_sampling_roi_factor = 2 * options_.sampling_roi_factor;
  const BoundingBoxType motion_sampling_bbox = BoundingBoxType::createFromCenterAndExtent(
      data_->roi_.getCentroid(), roi_bbox_extent * motion_sampling_roi_factor);
  motion_planner_.setSpaceBoundingBox(motion_sampling_bbox);
  motion_planner_.setObjectBoundingBox(drone_bbox_);
  std::cout << "Pose sampling bounding box: " << pose_sample_bbox_ << std::endl;
  std::cout << "Motion sampling bounding box: " << motion_sampling_bbox << std::endl;

  // TODO: Remove?
  triangulation_max_cos_angle_ = std::cos(options_.triangulation_min_angle_degrees * FloatType(M_PI) / FloatType(180));
  const FloatType triangulation_min_sin_angle = std::sin(options_.triangulation_min_angle_degrees * FloatType(M_PI) / FloatType(180));
  triangulation_min_sin_angle_square_ = triangulation_min_sin_angle * triangulation_min_sin_angle;
  const FloatType triangulation_max_sin_angle = std::sin(options_.triangulation_max_angle_degrees * FloatType(M_PI) / FloatType(180));
  triangulation_max_sin_angle_square_ = triangulation_max_sin_angle * triangulation_max_sin_angle;
  triangulation_max_angular_deviation_ = options_.triangulation_max_angular_deviation_degrees * (FloatType)M_PI / FloatType(180);
  incidence_angle_threshold_ = options_.incidence_angle_threshold_degrees * FloatType(M_PI) / FloatType(180);
  incidence_angle_falloff_factor_ = 1 / (options_.incidence_angle_inv_falloff_factor_degrees * FloatType(M_PI) / FloatType(180));
  voxel_sensor_size_ratio_falloff_factor_ = 1 / options_.voxel_sensor_size_ratio_inv_falloff_factor;

  sparse_matching_max_angular_distance_ = options_.sparse_matching_max_angular_distance_degrees * M_PI / FloatType(180);

  if (!options_.viewpoint_graph_filename.empty()) {
    loadViewpointGraph(options_.viewpoint_graph_filename);
  }
  else {
    reset();
  }
}

void ViewpointPlanner::reset() {
  std::unique_lock<std::mutex> lock(mutex_);
  num_of_failed_viewpoint_entry_samples_ = 0;
  viewpoint_entries_.clear();
  viewpoint_ann_.clear();
  viewpoint_graph_.clear();
  viewpoint_graph_components_.first.clear();
  viewpoint_graph_components_valid_ = false;
  viewpoint_graph_motions_.clear();
  viewpoint_count_grid_.setAllValues(0);
  grid_cell_probabilities_ = std::vector<FloatType>(viewpoint_count_grid_.getNumElements());
  std::fill(grid_cell_probabilities_.begin(), grid_cell_probabilities_.end(), 1 / FloatType(grid_cell_probabilities_.size()));
  cached_visible_sparse_points_.clear();
  cached_visible_voxels_.clear();
  viewpoint_paths_initialized_ = false;
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
  if (hasReconstruction()) {
    num_real_viewpoints_ = data_->reconstruction_->getImages().size();
    for (const auto& entry : data_->reconstruction_->getImages()) {
      const Pose& pose = entry.second.pose();
  //    std::cout << "  image id=" << entry.first << ", pose=" << pose << std::endl;
      FloatType total_information = 0;
      VoxelWithInformationSet voxel_set;
      addViewpointEntry(ViewpointEntry(Viewpoint(&virtual_camera_, pose), total_information, std::move(voxel_set)));
    }
    // Initialize viewpoint nearest neighbor index
    std::cout << "Initialized viewpoint entries with " << viewpoint_entries_.size() << " viewpoints" << std::endl;
  }
  else {
    num_real_viewpoints_ = 0;
  }

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
  viewpoint_paths_initialized_ = false;
  viewpoint_paths_.clear();
  viewpoint_paths_.resize(options_.viewpoint_path_branches);
  viewpoint_paths_data_.clear();
  viewpoint_paths_data_.resize(options_.viewpoint_path_branches);
  lock.unlock();
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

ViewpointPlanner::ViewpointPath& ViewpointPlanner::getBestViewpointPath() {
  AIT_ASSERT(!viewpoint_paths_.empty());
  ViewpointPath* best_path = &viewpoint_paths_.front();
  if (viewpoint_paths_.size() > 1) {
    for (auto it = viewpoint_paths_.begin() + 1; it != viewpoint_paths_.end(); ++it) {
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
  return std::unique_lock<std::mutex>(mutex_);
}

bool ViewpointPlanner::isValidObjectPosition(const Vector3& position, const BoundingBoxType& object_bbox) const {
  return data_->isValidObjectPosition(position, object_bbox);
}

auto ViewpointPlanner::findViewpointEntryWithPose(const Pose& pose) const -> std::pair<bool, ViewpointEntryIndex> {
  ViewpointEntryIndex matching_viewpoint_index = (ViewpointEntryIndex)-1;
  const std::size_t knn = options_.viewpoint_motion_max_neighbors;
  static std::vector<ViewpointANN::IndexType> knn_indices;
  static std::vector<ViewpointANN::DistanceType> knn_distances;
  knn_indices.resize(knn);
  knn_distances.resize(knn);
  viewpoint_ann_.knnSearch(pose.getWorldPosition(), knn, &knn_indices, &knn_distances);
  bool found = false;
  for (std::size_t i = 0; i < knn_distances.size(); ++i) {
    const ViewpointANN::DistanceType dist_square = knn_distances[i];
    const ViewpointEntryIndex viewpoint_index = knn_indices[i];
    const ViewpointEntry& viewpoint_entry = viewpoint_entries_[viewpoint_index];
    if (pose == viewpoint_entry.viewpoint.pose()) {
      matching_viewpoint_index = viewpoint_index;
      found = true;
      break;
    }
  }
  return std::make_pair(found, matching_viewpoint_index);
}

const ViewpointPlanner::Options& ViewpointPlanner::getOptions() const {
  return options_;
}

const ViewpointPlannerData& ViewpointPlanner::getPlannerData() const {
  return *data_;
}

ViewpointPlannerData& ViewpointPlanner::getPlannerData() {
  return *data_;
}

void ViewpointPlanner::ensureOctreeDrawerIsInitialized() const {
  if (offscreen_opengl_ && offscreen_opengl_->getContextThread() != QThread::currentThread()) {
    std::cout << "Octree drawer is used in different thread. Reallocating opengl context." << std::endl;
//    auto drawing_handle = offscreen_opengl_->beginDrawing();
//    octree_drawer_.reset();
//    drawing_handle.finish();
    offscreen_opengl_.reset();
  }
  if (!octree_drawer_) {
    octree_drawer_.reset(new rendering::OcTreeDrawer());
  }
  if (!offscreen_opengl_) {
    const PinholeCamera sparse_matching_camera
            = getVirtualCamera().getScaledCamera(options_.sparse_matching_virtual_camera_factor);
    offscreen_opengl_.reset(new bh::opengl::OffscreenOpenGL<FloatType>(sparse_matching_camera));
    auto drawing_handle = offscreen_opengl_->beginDrawing(false);
    octree_drawer_->setOctree(
            getOctree(),
            options_.sparse_matching_occupancy_threshold,
            options_.sparse_matching_observation_count_threshold,
            options_.sparse_matching_render_tree_depth);
    octree_drawer_->setColorFlags(rendering::VoxelDrawer::ColorFlags::Index);
    drawing_handle.finish();
  }
}

const viewpoint_planner::ViewpointOffscreenRenderer& ViewpointPlanner::getOffscreenRenderer() const {
 return *offscreen_renderer_.get();
}

viewpoint_planner::ViewpointOffscreenRenderer& ViewpointPlanner::getOffscreenRenderer() {
  return *offscreen_renderer_.get();
}

const bh::opengl::OffscreenOpenGL<ViewpointPlanner::FloatType>& ViewpointPlanner::getOffscreenOpenGL() const {
  return *offscreen_opengl_.get();
}

bh::opengl::OffscreenOpenGL<ViewpointPlanner::FloatType>& ViewpointPlanner::getOffscreenOpenGL() {
  return *offscreen_opengl_.get();
}

const rendering::OcTreeDrawer& ViewpointPlanner::getOctreeDrawer() const {
  return *octree_drawer_.get();
}

rendering::OcTreeDrawer& ViewpointPlanner::getOctreeDrawer() {
  return *octree_drawer_.get();
}

const PinholeCamera& ViewpointPlanner::getVirtualCamera() const {
  return virtual_camera_;
}

const ViewpointPlanner::BoundingBoxType& ViewpointPlanner::getDroneBoundingBox() const {
  return drone_bbox_;
}

ViewpointPlanner::Vector3 ViewpointPlanner::getDroneExtent() const {
  return drone_bbox_.getExtent();
}

void ViewpointPlanner::setScaledVirtualCamera(CameraId camera_id, FloatType scale_factor) {
  const PinholeCamera& camera = data_->reconstruction_->getCameras().at(camera_id);
  size_t virtual_width = static_cast<size_t>(scale_factor * camera.width());
  size_t virtual_height = static_cast<size_t>(scale_factor * camera.height());
  reconstruction::CameraMatrix virtual_intrinsics = bh::vision::getScaledIntrinsics(camera.intrinsics(), scale_factor);
  setVirtualCamera(virtual_width, virtual_height, virtual_intrinsics);
}

void ViewpointPlanner::setVirtualCamera(size_t virtual_width, size_t virtual_height, const FloatType focal_length) {
  reconstruction::CameraMatrix virtual_intrinsics = reconstruction::CameraMatrix::Zero();
  virtual_intrinsics(0, 0) = focal_length;
  virtual_intrinsics(1, 1) = focal_length;
  virtual_intrinsics(2, 2) = 1;
  virtual_intrinsics(3, 3) = 1;
  virtual_intrinsics(0, 2) = FloatType(virtual_width) / 2;
  virtual_intrinsics(1, 2) = FloatType(virtual_height) / 2;
  setVirtualCamera(virtual_width, virtual_height, virtual_intrinsics);
}

void ViewpointPlanner::setVirtualCamera(size_t virtual_width, size_t virtual_height, const Matrix4x4& virtual_intrinsics) {
  virtual_camera_ = PinholeCamera(virtual_width, virtual_height, virtual_intrinsics);
  std::cout << "Virtual camera size: " << virtual_width << " x " << virtual_height << std::endl;
  std::cout << "Virtual intrinsics: " << virtual_intrinsics << std::endl;
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

ViewpointPlanner::GpsCoordinateType ViewpointPlanner::convertPositionToGps(const Vector3& position) const {
  using GpsFloatType = typename GpsCoordinateType::FloatType;
  using GpsConverter = bh::GpsConverter<GpsFloatType>;
  const GpsCoordinateType gps_reference = data_->reconstruction_->sfmGpsTransformation().gps_reference;
  const GpsConverter gps_converter = GpsConverter::createWGS84(gps_reference);
  const GpsCoordinateType gps = gps_converter.convertEnuToGps(position.cast<GpsFloatType>());
  return gps;
}

std::pair<bool, ViewpointPlanner::ViewpointEntryIndex> ViewpointPlanner::canVoxelBeTriangulated(
    const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data,
    const ViewpointEntry& new_viewpoint, const VoxelWithInformation& voxel) const {
  auto found_it = comp_data.voxel_observation_counts.find(voxel.voxel);
  return canVoxelBeTriangulated(viewpoint_path, comp_data, new_viewpoint, found_it);
}
