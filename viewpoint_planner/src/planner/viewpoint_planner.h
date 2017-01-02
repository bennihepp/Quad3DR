//==================================================
// viewpoint_planner.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 6, 2016
//==================================================
#pragma once

#include <ait/eigen.h>
#include <ait/eigen_serialization.h>
#include <memory>
#include <unordered_set>
#include <boost/functional/hash.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <ait/common.h>
#include <ait/options.h>
#include <ait/random.h>
#include <ait/eigen_utils.h>
#include <ait/serialization.h>
#include "../mLib/mLib.h"
#include "../octree/occupancy_map.h"
#include "../reconstruction/dense_reconstruction.h"
#include "../graph/graph.h"
#include "../ann/approximate_nearest_neighbor.h"
#include "viewpoint.h"
#include "viewpoint_planner_data.h"
#include "motion_planner.h"

using reconstruction::CameraId;
using reconstruction::PinholeCameraColmap;
using reconstruction::ImageId;
using reconstruction::ImageColmap;
using reconstruction::Point3DId;
using reconstruction::Point3D;
using reconstruction::Point3DStatistics;
using reconstruction::SparseReconstruction;
using reconstruction::DenseReconstruction;

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
      addOption<FloatType>("sampling_roi_factor", &sampling_roi_factor);
      addOption<FloatType>("pose_sample_min_radius", &pose_sample_min_radius);
      addOption<FloatType>("pose_sample_max_radius", &pose_sample_max_radius);
      addOption<std::size_t>("pose_sample_num_trials", &pose_sample_num_trials);
      addOption<std::size_t>("viewpoint_sample_count", &viewpoint_sample_count);
      addOption<std::size_t>("viewpoint_min_voxel_count", &viewpoint_min_voxel_count);
      addOption<FloatType>("viewpoint_min_information", &viewpoint_min_information);
      addOption<std::size_t>("viewpoint_discard_dist_knn", &viewpoint_discard_dist_knn);
      addOption<FloatType>("viewpoint_discard_dist_thres_square", &viewpoint_discard_dist_thres_square);
      addOption<std::size_t>("viewpoint_discard_dist_count_thres", &viewpoint_discard_dist_count_thres);
      addOption<std::size_t>("viewpoint_motion_max_neighbors", &viewpoint_motion_max_neighbors);
      addOption<FloatType>("viewpoint_motion_max_dist_square", &viewpoint_motion_max_dist_square);
      addOption<std::size_t>("viewpoint_motion_min_connections", &viewpoint_motion_min_connections);
      addOption<std::size_t>("viewpoint_motion_densification_max_depth", &viewpoint_motion_densification_max_depth);
      addOption<std::size_t>("viewpoint_path_branches", &viewpoint_path_branches);
      addOption<FloatType>("objective_parameter_alpha", &objective_parameter_alpha);
      addOption<FloatType>("objective_parameter_beta", &objective_parameter_beta);
      addOption<std::string>("viewpoint_graph_filename", &viewpoint_graph_filename);
      // TODO:
      addOption<std::size_t>("num_sampled_poses", &num_sampled_poses);
      addOption<std::size_t>("num_planned_viewpoints", &num_planned_viewpoints);
    }

    ~Options() override {}

    // Random number generator seed (0 will seed from current time)
    std::size_t rng_seed = 0;

    // Scale factor from real camera to virtual camera
    FloatType virtual_camera_scale = 0.05f;

    // Viewpoint sampling parameters

    // Region of interest dilation for viewpoint sampling
    FloatType sampling_roi_factor = 2;
    // Spherical shell for viewpoint sampling
    FloatType pose_sample_min_radius = 2;
    FloatType pose_sample_max_radius = 5;

    // Number of trials for viewpoint position sampling
    std::size_t pose_sample_num_trials = 100;
    // Number of viewpoints to sample per iteration
    std::size_t viewpoint_sample_count = 10;
    // Minimum voxel count for viewpoints
    std::size_t viewpoint_min_voxel_count = 100;
    // Minimum information for viewpoints
    FloatType viewpoint_min_information = 10;

    // For checking whether a sampled viewpoint is too close to existing ones
    //
    // How many nearest neighbors to consider
    std::size_t viewpoint_discard_dist_knn = 10;
    // Squared distance considered to be too close
    FloatType viewpoint_discard_dist_thres_square = 4;
    // How many other viewpoints need to be too close for the sampled viewpoint to be discarded
    std::size_t viewpoint_discard_dist_count_thres = 3;

    // Maximum number of neighbors to consider for finding a motion path
    std::size_t viewpoint_motion_max_neighbors = 10;
    // Maximum distance between viewpoints for finding a motion path
    FloatType viewpoint_motion_max_dist_square = 10;
    // Minimum number of connections that need to be found for a viewpoint to be accepted
    std::size_t viewpoint_motion_min_connections = 3;
    // Number of connections to traverse in the graph for densifying edges
    std::size_t viewpoint_motion_densification_max_depth = 5;

    // Number of viewpoint path branches to explore in parallel
    std::size_t viewpoint_path_branches = 10;
    // Objective factor for reconstruction image
    FloatType objective_parameter_alpha = 0;
    // Objective factor for motion distance
    FloatType objective_parameter_beta = 0;

    // Filename of serialized viewpoint graph
    std::string viewpoint_graph_filename = "";

    // TODO: Needed?
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

  class ViewpointEntrySerializer;

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

    // TODO: Only store pose and not viewpoint.
    Viewpoint viewpoint;
    FloatType total_information;
    VoxelWithInformationSet voxel_set;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    friend class ViewpointEntrySerializer;
  };

  using ViewpointEntryVector = std::vector<ViewpointEntry>;
  using ViewpointEntryIndex = ViewpointEntryVector::size_type;

  class ViewpointEntrySaver {
  public:
    ViewpointEntrySaver(const ViewpointEntryVector& entries, const ViewpointPlannerData::OccupiedTreeType& bvh_tree)
    : entries_(entries), bvh_tree_(bvh_tree) {}

  private:
    // Boost serialization
    friend class boost::serialization::access;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version) const {
      std::unordered_map<const VoxelType*, std::size_t> voxel_index_map;
      // Compute consistent ordering of BVH nodes
      std::size_t voxel_index = 0;
      for (const ViewpointPlannerData::OccupiedTreeType::NodeType& node : bvh_tree_) {
        voxel_index_map.emplace(&node, voxel_index);
        ++voxel_index;
      }
      ar & entries_.size();
      for (const ViewpointEntry& entry : entries_) {
        // Save viewpoint
        ar & entry.viewpoint.pose();
        ar & entry.total_information;
        ar & entry.voxel_set.size();
        for (const VoxelWithInformation& voxel_with_information : entry.voxel_set) {
          std::size_t voxel_index = voxel_index_map.at(voxel_with_information.voxel);
          ar & voxel_index;
          ar & voxel_with_information.information;
        }
      }
    }

    const ViewpointEntryVector& entries_;
    const ViewpointPlannerData::OccupiedTreeType& bvh_tree_;
  };

  class ViewpointEntryLoader {
  public:
    ViewpointEntryLoader(ViewpointEntryVector* entries, ViewpointPlannerData::OccupiedTreeType* bvh_tree, SparseReconstruction* reconstruction)
    : entries_(entries), bvh_tree_(bvh_tree), reconstruction_(reconstruction) {}

  private:
    // Boost serialization
    friend class boost::serialization::access;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version) {
      std::unordered_map<std::size_t, VoxelType*> voxel_index_map;
      // Compute consistent ordering of BVH nodes
      std::size_t voxel_index = 0;
      for (ViewpointPlannerData::OccupiedTreeType::NodeType& node : *bvh_tree_) {
        voxel_index_map.emplace(voxel_index, &node);
        ++voxel_index;
      }
      const PinholeCamera* camera = static_cast<PinholeCamera*>(&(reconstruction_->getCameras().begin()->second));
      std::size_t num_entries;
      ar & num_entries;
      entries_->resize(num_entries);
      for (ViewpointEntry& entry : *entries_) {
        Pose pose;
        ar & pose;
        entry.viewpoint = Viewpoint(camera, pose);
        ar & entry.total_information;
        std::size_t num_voxels;
        ar & num_voxels;
        for (std::size_t i = 0; i < num_voxels; ++i) {
          std::size_t voxel_index;
          ar & voxel_index;
          VoxelType* voxel = voxel_index_map.at(voxel_index);
          FloatType information;
          ar & information;
          VoxelWithInformation voxel_with_information(voxel, information);
          entry.voxel_set.emplace(std::move(voxel_with_information));
        }
      }
    }

    ViewpointEntryVector* entries_;
    ViewpointPlannerData::OccupiedTreeType* bvh_tree_;
    SparseReconstruction* reconstruction_;
  };

  /// Describes a viewpoint and the information it adds to the previous viewpoints
  struct ViewpointPathEntry {
    ViewpointPathEntry()
    : viewpoint_index((ViewpointEntryIndex)-1),
      local_information(std::numeric_limits<FloatType>::lowest()),
      acc_information(std::numeric_limits<FloatType>::lowest()),
      local_motion_distance(std::numeric_limits<FloatType>::max()),
      acc_motion_distance(std::numeric_limits<FloatType>::max()),
      local_objective(std::numeric_limits<FloatType>::lowest()),
      acc_objective(std::numeric_limits<FloatType>::lowest()) {}

    ViewpointEntryIndex viewpoint_index;
    // TODO: Is it necessary to save all the accumulated information with a path entry?
    FloatType local_information;
    FloatType acc_information;
    FloatType local_motion_distance;
    FloatType acc_motion_distance;
    FloatType local_objective;
    FloatType acc_objective;
  };

  struct ViewpointMotion {
    ViewpointMotion(ViewpointEntryIndex from_index, ViewpointEntryIndex to_index, FloatType cost)
    : from_index(from_index), to_index(to_index), cost(cost) {}

    ViewpointEntryIndex from_index;
    ViewpointEntryIndex to_index;
    FloatType cost;
  };

  struct ViewpointPath {
    // Ordered viewpoint entries on the path
    std::vector<ViewpointPathEntry> entries;
    // Set of voxels that are observed on the whole path
    VoxelWithInformationSet observed_voxel_set;
    // Accumulated information over the whole path
    FloatType acc_information;
    // Accumulated motion distance over the whole path
    FloatType acc_motion_distance;
    // Accumulated motion distance over the whole path
    FloatType acc_objective;
  };

  using ViewpointGraph = Graph<ViewpointEntryIndex, FloatType>;
  using ViewpointANN = ApproximateNearestNeighbor<FloatType, 3>;
  using FeatureViewpointMap = std::unordered_map<std::size_t, std::vector<const Viewpoint*>>;


  ViewpointPlanner(const Options* options, std::unique_ptr<ViewpointPlannerData> data);

  /// Set size of virtual camera used for raycasting.
  void setVirtualCamera(std::size_t virtual_width, std::size_t virtual_height, const Matrix4x4& virtual_intrinsics);

  /// Set size of virtual camera used for raycasting by scaling a real camera.
  void setScaledVirtualCamera(CameraId camera_id, FloatType scale_factor);

  /// Reset planning (i.e. viewpoint entries, graph and path)
  void reset();

  /// Reset viewpoint motion paths
  void resetViewpointMotions();

  /// Reset viewpoint paths
  void resetViewpointPaths();

  void saveViewpointGraph(const std::string& filename) const;

  void loadViewpointGraph(const std::string& filename);

  const OccupancyMapType* getOctree() const {
      return data_->octree_.get();
  }

  const DenseReconstruction* getReconstruction() const {
      return data_->reconstruction_.get();
  }

  const MeshType* getMesh() const {
    return data_->poisson_mesh_.get();
  }

  /// Return viewpoint entries for reading. Mutex needs to be locked.
  const ViewpointEntryVector& getViewpointEntries() const {
    return viewpoint_entries_;
  }

  /// Return viewpoint graph for reading. Mutex needs to be locked.
  const ViewpointGraph& getViewpointGraph() const {
    return viewpoint_graph_;
  }

  /// Return viewpoint paths for reading. Mutex needs to be locked.
  const std::vector<ViewpointPath>& getViewpointPaths() const {
    return viewpoint_paths_;
  }

  /// Return best viewpoint path for reading. Mutex needs to be locked.
  const ViewpointPath& getBestViewpointPath() const;

  /// Acquire lock to acces viewpoint entries (including graph and path) or reset the planning
  std::unique_lock<std::mutex> acquireLock();

  // Pose and viewpoint sampling

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

  /// Check if an object can be placed at a position (i.e. is it free space)
  bool isValidObjectPosition(const Vector3& position, const Vector3& object_extent);

  // TODO: remove
  void run();

  /// Generate a new viewpoint entry and add it to the graph.
  bool generateNextViewpointEntry();

  // Compute motions between viewpoints and update the graph with their cost.
  void computeViewpointMotions();

  /// Find the next best viewpoint to add to the viewpoint paths.
  bool findNextViewpointPathEntries(const FloatType alpha, const FloatType beta);

  /// Find the next best viewpoint to add to the viewpoint paths and use parameters from options.
  bool findNextViewpointPathEntries();

  // Raycasting and information computation

  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels with additional info.
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> getRaycastHitVoxelsBVH(
      const Pose& pose) const;

  /// Perform raycast on the BVH tree.
  /// Returns the set of hit voxels with corresponding information + the total information of all voxels.
  std::pair<VoxelWithInformationSet, FloatType>
    getRaycastHitVoxelsWithInformationScoreBVH(const Pose& pose) const;

  /// Returns the information score for a single hit voxel.
  FloatType computeInformationScore(const ViewpointPlannerData::OccupiedTreeType::IntersectionResult& result) const;

  // Matching score computation

  std::unordered_set<Point3DId> computeProjectedMapPoints(const CameraId camera_id, const Pose& pose,
      FloatType projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

  std::unordered_set<Point3DId> computeFilteredMapPoints(const CameraId camera_id, const Pose& pose,
      FloatType projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

  std::unordered_set<Point3DId> computeVisibleMapPoints(const CameraId camera_id, const Pose& pose,
      FloatType projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

  std::unordered_set<Point3DId> computeVisibleMapPointsFiltered(const CameraId camera_id, const Pose& pose,
      FloatType projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /// Remove points that are occluded from the given pose.
  /// A point needs to be dist_margin behind a voxel surface to be removed.
  void removeOccludedPoints(const Pose& pose, std::unordered_set<Point3DId>& point3D_ids, FloatType dist_margin) const;

  /// Compute information factor based on observation count (i.e. exp(-theta * observation_count)
  static WeightType computeObservationInformationFactor(CounterType observation_count);

  /// Add a viewpoint entry to the graph.
  void addViewpointEntry(ViewpointEntry&& viewpoint_entry);

  /// Add a viewpoint entry. This also computes motion distances to other viewpoints and updates edges in the graph.
  void addViewpointEntryAndMotions(ViewpointEntry&& viewpoint_entry, const std::vector<ViewpointMotion>& motions);

  /// Add a viewpoint entry without acquiring a lock. Returns the index of the new viewpoint entry.
  ViewpointEntryIndex addViewpointEntryWithoutLock(ViewpointEntry&& viewpoint_entry);

  /// Find motion paths from provided viewpoint to neighbors in the viewpoint graph.
  std::vector<ViewpointMotion> findViewpointMotions(const Pose& from_pose, std::size_t tentative_viewpoint_index);

  /// Find motion paths from provided viewpoint to neighbors in the viewpoint graph.
  std::vector<ViewpointMotion> findViewpointMotions(const ViewpointEntryIndex from_index);

  /// Find the next best viewpoint to add to a single viewpoint path.
  bool findNextViewpointPathEntry(ViewpointPath* viewpoint_path, const FloatType alpha, const FloatType beta);

  ait::Random<FloatType, std::int64_t> random_;

  Options options_;

  Vector3 drone_extent_;

  std::unique_ptr<ViewpointPlannerData> data_;
  PinholeCamera virtual_camera_;

  std::mutex mutex_;

  MotionPlanner<FloatType> motion_planner_;

  // All tentative viewpoints
  ViewpointEntryVector viewpoint_entries_;
  // Number of real viewpoints at the beginning of the viewpoint_entries_ vector
  // These need to be distinguished because they could be in non-free space of the map
  std::size_t num_real_viewpoints_;
  // Approximate nearest neighbor index for viewpoints
  ViewpointANN viewpoint_ann_;
  // Graph of tentative viewpoints with motion distances
  ViewpointGraph viewpoint_graph_;
  // Current viewpoint paths
  std::vector<ViewpointPath> viewpoint_paths_;
  // Map from triangle index to viewpoints that project features into it
  FeatureViewpointMap feature_viewpoint_map_;
};

template <typename IteratorT>
std::pair<bool, ViewpointPlanner::Pose> ViewpointPlanner::sampleSurroundingPose(IteratorT first, IteratorT last) const {
  AIT_ASSERT_STR(last - first > 0, "Unable to sample surrounding pose from empty pose set");
  std::size_t index = random_.sampleUniformIntExclusive(0, last - first);
  IteratorT it = first + index;
//  std::cout << "index=" << index << ", last-first=" << (last-first) << std::endl;
  const Pose& pose = it->viewpoint.pose();
  return sampleSurroundingPose(pose);
}

template <typename IteratorT>
std::pair<bool, ViewpointPlanner::Pose> ViewpointPlanner::sampleSurroundingPoseFromEntries(IteratorT first, IteratorT last) const {
  AIT_ASSERT_STR(last - first > 0, "Unable to sample surrounding pose from empty pose set");
  std::size_t index = random_.sampleUniformIntExclusive(0, last - first);
  IteratorT it = first + index;
//  std::cout << "index=" << index << ", last-first=" << (last-first) << std::endl;
  const ViewpointEntryIndex viewpoint_index = *it;
  const Pose& pose = viewpoint_entries_[viewpoint_index].viewpoint.pose();
  return sampleSurroundingPose(pose);
}
