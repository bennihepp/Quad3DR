//==================================================
// viewpoint_planner.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 6, 2016
//==================================================
#pragma once

#include <ait/eigen.h>
#include <memory>
#include <unordered_set>
#include <boost/functional/hash.hpp>
#include <ait/common.h>
#include <ait/options.h>
#include <ait/random.h>
#include <ait/eigen_utils.h>
#include <ait/serialization.h>
#include "../mLib/mLib.h"
#include "../octree/occupancy_map.h"
#include "../reconstruction/dense_reconstruction.h"
#include "../graph/graph.h"
#include "viewpoint_planner_data.h"

using reconstruction::CameraId;
using reconstruction::PinholeCamera;
using reconstruction::PinholeCameraColmap;
using reconstruction::ImageId;
using reconstruction::ImageColmap;
using reconstruction::Point3DId;
using reconstruction::Point3D;
using reconstruction::Point3DStatistics;
using reconstruction::SparseReconstruction;
using reconstruction::DenseReconstruction;

template <typename T>
struct Ray {
  using FloatType = T;
  USE_FIXED_EIGEN_TYPES(FloatType)

  Ray(const Vector3& origin, const Vector3& direction)
  : origin_(origin), direction_(direction.normalized()) {}

  const Vector3& origin() const {
    return origin_;
  }

  const Vector3& direction() const {
    return direction_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Vector3 origin_;
  Vector3 direction_;
};

class Viewpoint {
public:
  using FloatType = reconstruction::FloatType;
  USE_FIXED_EIGEN_TYPES(FloatType)
  using RayType = Ray<FloatType>;
  using Pose = reconstruction::Pose;

  static constexpr FloatType DEFAULT_PROJECTION_MARGIN = 10;
  static constexpr FloatType MAX_DISTANCE_DEVIATION_BY_STDDEV = 3;
  static constexpr FloatType MAX_NORMAL_DEVIATION_BY_STDDEV = 3;

  Viewpoint(FloatType projection_margin = DEFAULT_PROJECTION_MARGIN);

  Viewpoint(const PinholeCamera* camera, const Pose& pose, FloatType projection_margin = DEFAULT_PROJECTION_MARGIN);

  Viewpoint(const Viewpoint& other);

  Viewpoint(Viewpoint&& other);

  Viewpoint& operator=(const Viewpoint& other);

  Viewpoint& operator=(Viewpoint&& other);

  const Pose& pose() const {
    return pose_;
  }

  const PinholeCamera& camera() const {
    return *camera_;
  }

  FloatType getDistanceTo(const Viewpoint& other) const;

  FloatType getDistanceTo(const Viewpoint& other, FloatType rotation_factor) const;

  std::unordered_set<Point3DId> getProjectedPoint3DIds(const SparseReconstruction::Point3DMapType& points) const;

  std::unordered_set<Point3DId> getProjectedPoint3DIdsFiltered(const SparseReconstruction::Point3DMapType& points) const;

  bool isPointFiltered(const Point3D& point) const;

  Vector2 projectWorldPointIntoImage(const Vector3& point_world) const;

  Ray<FloatType> getCameraRay(FloatType x, FloatType y) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  const PinholeCamera* camera_;
  Pose pose_;
  FloatType projection_margin_;
  Matrix3x4 transformation_world_to_image_;
};

class ViewpointPlanner {
public:
  using FloatType = ViewpointPlannerData::FloatType;
  USE_FIXED_EIGEN_TYPES(FloatType)
  using BoundingBoxType = ViewpointPlannerData::BoundingBoxType;
  using RayType = Ray<FloatType>;
  using Pose = ait::Pose<FloatType>;

  static constexpr FloatType kDotProdEqualTolerance = FloatType { 1e-5 };
  static constexpr FloatType kOcclusionDistMarginFactor = FloatType { 4 };

  struct Options : ait::ConfigOptions {
    Options()
    : ait::ConfigOptions("viewpoint_planner", "ViewpointPlanner options") {
      addOption<std::size_t>("rng_seed", &rng_seed);
      addOption<FloatType>("virtual_camera_scale", &virtual_camera_scale);
      addOption<FloatType>("drone_extent_x", 3);
      addOption<FloatType>("drone_extent_y", 3);
      addOption<FloatType>("drone_extent_z", 3);
      addOption<std::size_t>("pose_sample_num_trials", &pose_sample_num_trials);
      addOption<FloatType>("pose_sample_min_radius", &pose_sample_min_radius);
      addOption<FloatType>("pose_sample_max_radius", &pose_sample_max_radius);
      addOption<FloatType>("sampling_roi_factor", &sampling_roi_factor);
      addOption<std::size_t>("num_sampled_poses", &num_sampled_poses);
      addOption<std::size_t>("num_planned_viewpoints", &num_planned_viewpoints);
    }

    ~Options() override {}

    std::size_t rng_seed = -0;
    FloatType virtual_camera_scale = 0.05f;
    std::size_t pose_sample_num_trials = 100;
    FloatType pose_sample_min_radius = 2;
    FloatType pose_sample_max_radius = 5;
    FloatType sampling_roi_factor = 2;
    std::size_t num_sampled_poses = 100;
    std::size_t num_planned_viewpoints = 10;
  };

  using MeshType = ViewpointPlannerData::MeshType;
//    using OctomapType = octomap::OcTree;
  using InputOccupancyMapType = OccupancyMap<OccupancyNode>;
  using OccupancyMapType = OccupancyMap<AugmentedOccupancyNode>;
  using TreeNavigatorType = TreeNavigator<OccupancyMapType, OccupancyMapType::NodeType>;
  using ConstTreeNavigatorType = TreeNavigator<const OccupancyMapType, const OccupancyMapType::NodeType>;
  using OccupancyType = OccupancyMapType::NodeType::OccupancyType;
  using CounterType = OccupancyMapType::NodeType::CounterType;
  using WeightType = OccupancyMapType::NodeType::WeightType;
  using VoxelType = ViewpointPlannerData::OccupiedTreeType::NodeType;

  /// Voxel node and it's corresponding amount of information
  struct VoxelWithInformation {
    VoxelWithInformation(const VoxelType* voxel, const FloatType information)
    : voxel(voxel), information(information) {}

    const VoxelType* voxel;
    FloatType information;

    bool operator==(const VoxelWithInformation& other) const {
      return voxel == other.voxel && information == other.information;
    }

    struct Hash {
      std::size_t operator()(const VoxelWithInformation& voxel_with_information) const {
        std::size_t val { 0 };
              boost::hash_combine(val, voxel_with_information.voxel);
              boost::hash_combine(val, voxel_with_information.information);
              return val;
      }
    };
  };

  using VoxelWithInformationSet = std::unordered_set<VoxelWithInformation, VoxelWithInformation::Hash>;

  /// Describes a viewpoint, the set of voxels observed by it and the corresponding information
  struct ViewpointEntry {
    ViewpointEntry()
    : total_information(0) {}

    ViewpointEntry(const Viewpoint& viewpoint, const FloatType total_information,
        const VoxelWithInformationSet& voxel_set)
    : viewpoint(viewpoint), total_information(total_information), voxel_set(voxel_set) {}

    ViewpointEntry(const Viewpoint& viewpoint, const FloatType total_information,
        VoxelWithInformationSet&& voxel_set)
    : viewpoint(viewpoint), total_information(total_information), voxel_set(std::move(voxel_set)) {}

    ViewpointEntry(const ViewpointEntry& other)
    : viewpoint(other.viewpoint), total_information(other.total_information),
      voxel_set(other.voxel_set) {}

    ViewpointEntry(ViewpointEntry&& other)
    : viewpoint(std::move(other.viewpoint)), total_information(other.total_information),
      voxel_set(std::move(other.voxel_set)) {}

    ViewpointEntry& operator=(const ViewpointEntry& other) {
      if (this != &other) {
        viewpoint = other.viewpoint;
        total_information = other.total_information;
        voxel_set = other.voxel_set;
      }
      return *this;
    }

    ViewpointEntry& operator=(ViewpointEntry&& other) {
      if (this != &other) {
        viewpoint = std::move(other.viewpoint);
        total_information = other.total_information;
        voxel_set = std::move(other.voxel_set);
      }
      return *this;
    }

    // Members cannot be constant because std::sort requires move-assignment
    Viewpoint viewpoint;
    FloatType total_information;
    VoxelWithInformationSet voxel_set;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  using ViewpointEntryVector = std::vector<ViewpointEntry>;
  using ViewpointEntryIndex = ViewpointEntryVector::size_type;

  /// Describes a viewpoint and the information it adds to the previous viewpoints
  struct ViewpointPathEntry {
    ViewpointPathEntry(ViewpointEntryIndex viewpoint_index, FloatType information)
    : viewpoint_index(viewpoint_index), information(information) {}

    ViewpointEntryIndex viewpoint_index;
    FloatType information;
  };

  using ViewpointGraph = Graph<ViewpointEntryIndex, FloatType>;
  using ViewpointPath = std::vector<ViewpointPathEntry>;
  using FeatureViewpointMap = std::unordered_map<std::size_t, std::vector<const Viewpoint*>>;

  ViewpointPlanner(const Options* options, std::unique_ptr<ViewpointPlannerData> data);

  void reset();

  const OccupancyMapType* getOctree() const {
      return data_->octree_.get();
  }

  const DenseReconstruction* getReconstruction() const {
      return data_->reconstruction_.get();
  }

  const MeshType* getMesh() const {
    return data_->poisson_mesh_.get();
  }

  const ViewpointEntryVector& getViewpointEntries() const {
    return viewpoint_entries_;
  }

  const ViewpointGraph& getViewpointGraph() const {
    return viewpoint_graph_;
  }

  const ViewpointPath& getViewpointPath() const {
    return viewpoint_path_;
  }

  std::mutex& mutex() {
    return mutex_;
  }

  template <typename IteratorT>
  std::pair<bool, Pose> sampleSurroundingPose(IteratorT first, IteratorT last) const;

  template <typename IteratorT>
  std::pair<bool, Pose> sampleSurroundingPoseFromEntries(IteratorT first, IteratorT last) const;

  std::pair<bool, Pose> sampleSurroundingPose(const Pose& pose) const;

  std::pair<bool, Pose> samplePose(std::size_t max_trials = (std::size_t)-1) const;

  std::pair<bool, Pose> samplePose(const BoundingBoxType& bbox,
      const Vector3& object_extent, std::size_t max_trials = (std::size_t)-1) const;

  std::pair<bool, ViewpointPlanner::Vector3> samplePosition(std::size_t max_trials = (std::size_t)-1) const;

  std::pair<bool, ViewpointPlanner::Vector3> samplePosition(const BoundingBoxType& bbox,
      const Vector3& object_extent, std::size_t max_trials = (std::size_t)-1) const;

  Pose::Quaternion sampleOrientation() const;

  /// Sample an orientation biased towards a bounding box.
  ///
  /// A single angle is sampled from a normal distribution (stddev depends on bounding box size) and clamped to [0, 2 pi].
  /// A spherical angle is then computed by sampling from [0, 2 pi] to determine the direction of the angle.
  /// Finally, the spherical angle is transformed to a 3D vector so that the sampling distribution is around the bounding box center
  /// seen from pos.
  Pose::Quaternion sampleBiasedOrientation(const Vector3& pos, const BoundingBoxType& bias_bbox) const;

  void setVirtualCamera(std::size_t virtual_width, std::size_t virtual_height, const Matrix4x4& virtual_intrinsics);

  void setScaledVirtualCamera(CameraId camera_id, FloatType scale_factor);

  void run();

  bool generateNextViewpoint();

  template <typename T>
  T rayCastAccumulate(const CameraId camera_id, const Pose& pose_world_to_image, bool ignore_unknown,
      std::function<T(bool, const octomap::point3d&)> func, T init = 0) const;

  template <typename T>
  T rayCastAccumulate(const Viewpoint& viewpoint, bool ignore_unknown,
      std::function<T(bool, const octomap::point3d&)> func, T init = 0) const;

  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> getRaycastHitVoxelsBVH(
      const Pose& pose_world_to_image) const;

  std::vector<std::pair<ConstTreeNavigatorType, FloatType>> getRaycastHitVoxels(const Pose& pose_world_to_image) const;

  std::pair<VoxelWithInformationSet, FloatType>
    getRaycastHitVoxelsWithInformationScoreBVH(const Pose& pose_world_to_image) const;

  FloatType computeInformationScore(const ViewpointPlannerData::OccupiedTreeType::IntersectionResult& result) const;

  FloatType computeInformationScoreBVH(const Pose& pose_world_to_image) const;

  FloatType computeInformationScore(const Pose& pose_world_to_image) const;

  std::unordered_set<Point3DId> computeProjectedMapPoints(const CameraId camera_id, const Pose& pose,
      FloatType projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

  std::unordered_set<Point3DId> computeFilteredMapPoints(const CameraId camera_id, const Pose& pose,
      FloatType projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

  std::unordered_set<Point3DId> computeVisibleMapPoints(const CameraId camera_id, const Pose& pose,
      FloatType projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

  std::unordered_set<Point3DId> computeVisibleMapPointsFiltered(const CameraId camera_id, const Pose& pose,
      FloatType projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

  template <typename Set1, typename Set2>
  std::size_t computeSetIntersectionSize(const Set1& set1, const Set2& set2) const;

  template <typename Set1, typename Set2>
  Set1 computeSetIntersection(const Set1& set1, const Set2& set2) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  void removeOccludedPoints(const Pose& pose_world_to_image, std::unordered_set<Point3DId>& point3D_ids, FloatType dist_margin) const;

  static WeightType computeObservationWeightFactor(CounterType observation_count);
  static WeightType computeObservationInformationFactor(CounterType observation_count);

  ait::Random<FloatType, std::int64_t> random_;

  Options options_;

  Vector3 drone_extent_;

  std::unique_ptr<ViewpointPlannerData> data_;
  PinholeCamera virtual_camera_;

  std::mutex mutex_;

  ViewpointEntryVector viewpoint_entries_;
  VoxelWithInformationSet observed_voxel_set_;
  ViewpointGraph viewpoint_graph_;
  ViewpointPath viewpoint_path_;
  // Map from triangle index to viewpoints that project features into it
  FeatureViewpointMap feature_viewpoint_map_;
};

template <typename IteratorT>
std::pair<bool, ViewpointPlanner::Pose> ViewpointPlanner::sampleSurroundingPose(IteratorT first, IteratorT last) const {
  AIT_ASSERT_STR(last - first > 0, "Unable to sample surrounding pose from empty pose set");
  std::size_t index = random_.sampleUniformIntExclusive(0, last - first);
//  std::uniform_int_distribution<std::size_t> dist(0, last - first);
//  std::size_t index = dist(rng_);
  IteratorT it = first;
  for (std::size_t i = 0; i < index; ++i) {
    ++it;
  }
  std::cout << "index=" << index << ", last-first=" << (last-first) << std::endl;
  const Pose& pose = it->viewpoint.pose();
  return sampleSurroundingPose(pose);
}

template <typename IteratorT>
std::pair<bool, ViewpointPlanner::Pose> ViewpointPlanner::sampleSurroundingPoseFromEntries(IteratorT first, IteratorT last) const {
  AIT_ASSERT_STR(last - first > 0, "Unable to sample surrounding pose from empty pose set");
  std::size_t index = random_.sampleUniformIntExclusive(0, last - first);
//  std::uniform_int_distribution<std::size_t> dist(0, last - first);
//  std::size_t index = dist(rng_);
  IteratorT it = first;
  for (std::size_t i = 0; i < index; ++i) {
    ++it;
  }
//  std::cout << "index=" << index << ", last-first=" << (last-first) << std::endl;
  const ViewpointEntryIndex viewpoint_index = *it;
  const Pose& pose = viewpoint_entries_[viewpoint_index].viewpoint.pose();
  return sampleSurroundingPose(pose);
}
