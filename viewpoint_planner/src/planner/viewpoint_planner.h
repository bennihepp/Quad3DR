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
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <rapidjson/document.h>
#include <ait/common.h>
#include <ait/options.h>
#include <ait/random.h>
#include <ait/eigen_utils.h>
#include <ait/serialization.h>
#include <ait/graph_boost.h>
#include "../mLib/mLib.h"
#include "../octree/occupancy_map.h"
#include "../reconstruction/dense_reconstruction.h"
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
  using RegionType = ViewpointPlannerData::RegionType;
  using RayType = ait::Ray<FloatType>;
  using Pose = ait::Pose<FloatType>;

  static constexpr FloatType kDotProdEqualTolerance = FloatType { 1e-5 };
  static constexpr FloatType kOcclusionDistMarginFactor = FloatType { 4 };

  struct Options : ait::ConfigOptions {
    Options()
    : ait::ConfigOptions("viewpoint_planner", "ViewpointPlanner options") {
      addOption<std::size_t>("rng_seed", &rng_seed);
      addOption<FloatType>("virtual_camera_scale", &virtual_camera_scale);
      addOption<FloatType>("raycast_min_range", &raycast_min_range);
      addOption<FloatType>("raycast_max_range", &raycast_max_range);
      addOption<bool>("ignore_real_observed_voxels", &ignore_real_observed_voxels);
      addOption<FloatType>("drone_extent_x", 3);
      addOption<FloatType>("drone_extent_y", 3);
      addOption<FloatType>("drone_extent_z", 3);
      addOption<FloatType>("sampling_roi_factor", &sampling_roi_factor);
      addOption<FloatType>("pose_sample_min_radius", &pose_sample_min_radius);
      addOption<FloatType>("pose_sample_max_radius", &pose_sample_max_radius);
      addOption<std::size_t>("pose_sample_num_trials", &pose_sample_num_trials);
      addOption<std::size_t>("viewpoint_sample_count", &viewpoint_sample_count);
      addOption<std::size_t>("viewpoint_min_voxel_count", &viewpoint_min_voxel_count);
      addOption<bool>("viewpoint_generate_stereo_pairs", &viewpoint_generate_stereo_pairs);
      addOption<FloatType>("viewpoint_min_information", &viewpoint_min_information);
      addOption<bool>("consider_triangulation", &consider_triangulation);
      addOption<FloatType>("triangulation_min_angle_degrees", &triangulation_min_angle_degrees);
      addOption<FloatType>("triangulation_max_angle_degrees", &triangulation_max_angle_degrees);
      addOption<FloatType>("triangulation_max_dist_deviation_ratio", &triangulation_max_dist_deviation_ratio);
      addOption<FloatType>("triangulation_max_angular_deviation_degrees", &triangulation_max_angular_deviation_degrees);
      addOption<FloatType>("triangulation_min_overlap_ratio", &triangulation_min_overlap_ratio);
      addOption<std::size_t>("triangulation_knn", &triangulation_knn);
      addOption<std::size_t>("triangulation_min_count", &triangulation_min_count);
      addOption<std::size_t>("viewpoint_discard_dist_knn", &viewpoint_discard_dist_knn);
      addOption<FloatType>("viewpoint_discard_dist_thres_square", &viewpoint_discard_dist_thres_square);
      addOption<std::size_t>("viewpoint_discard_dist_count_thres", &viewpoint_discard_dist_count_thres);
      addOption<FloatType>("viewpoint_discard_dist_real_thres_square", &viewpoint_discard_dist_real_thres_square);
      addOption<std::size_t>("viewpoint_motion_max_neighbors", &viewpoint_motion_max_neighbors);
      addOption<FloatType>("viewpoint_motion_max_dist_square", &viewpoint_motion_max_dist_square);
      addOption<std::size_t>("viewpoint_motion_min_connections", &viewpoint_motion_min_connections);
      addOption<std::size_t>("viewpoint_motion_densification_max_depth", &viewpoint_motion_densification_max_depth);
      addOption<std::size_t>("viewpoint_path_branches", &viewpoint_path_branches);
      addOption<FloatType>("viewpoint_path_initial_distance", &viewpoint_path_initial_distance);
      addOption<FloatType>("objective_parameter_alpha", &objective_parameter_alpha);
      addOption<FloatType>("objective_parameter_beta", &objective_parameter_beta);
      addOption<FloatType>("voxel_information_lambda", &voxel_information_lambda);
      addOption<std::size_t>("viewpoint_path_2opt_max_k_length", &viewpoint_path_2opt_max_k_length);
      addOption<std::string>("viewpoint_graph_filename", &viewpoint_graph_filename);
      // TODO:
      addOption<std::size_t>("num_sampled_poses", &num_sampled_poses);
      addOption<std::size_t>("num_planned_viewpoints", &num_planned_viewpoints);
      addOption<std::string>("motion_planner_log_filename", &motion_planner_log_filename);
    }

    ~Options() override {}

    // Random number generator seed (0 will seed from current time)
    std::size_t rng_seed = 0;

    // Scale factor from real camera to virtual camera
    FloatType virtual_camera_scale = 0.05f;

    // Min and max range for raycast
    FloatType raycast_min_range = 0;
    FloatType raycast_max_range = 60;

    // Whether to ignore the voxels already observed by a previous reconstruction
    bool ignore_real_observed_voxels = false;

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
    // Ensure that every viewpoint is part of a stereo pair
    bool viewpoint_generate_stereo_pairs;
    // Minimum voxel count for viewpoints
    std::size_t viewpoint_min_voxel_count = 100;
    // Minimum information for viewpoints
    FloatType viewpoint_min_information = 10;

    // Whether to check for triangulation constraint
    bool consider_triangulation = true;
    // Minimum angle between two views so that a voxel can be triangulated
    FloatType triangulation_min_angle_degrees = 5;
    // Maximum angle between two views so that a voxel can be triangulated
    FloatType triangulation_max_angle_degrees = 15;
    // Maximum deviation ratio of distances between stereo pair
    FloatType triangulation_max_dist_deviation_ratio = (FloatType)0.1;
    // Maximum angular deviation between stereo viewpoints
    FloatType triangulation_max_angular_deviation_degrees = 20;
    // Minimum ratio of information overlap for a viewpoint to be used for a stereo pair
    FloatType triangulation_min_overlap_ratio = 0.8f;
    // Number of nearest neighbors to consider for stereo pair search
    std::size_t triangulation_knn = 50;
    // Minimum number of viewpairs that can triangulate a voxel to be observed
    std::size_t triangulation_min_count = 2;

    // For checking whether a sampled viewpoint is too close to existing ones
    //
    // How many nearest neighbors to consider
    std::size_t viewpoint_discard_dist_knn = 10;
    // Squared distance considered to be too close
    FloatType viewpoint_discard_dist_thres_square = 4;
    // How many other viewpoints need to be too close for the sampled viewpoint to be discarded
    std::size_t viewpoint_discard_dist_count_thres = 3;
    // Squared distance to a real viewpoint considered to be too close
    FloatType viewpoint_discard_dist_real_thres_square = 9;

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
    // Minimum distance between initial viewpoints on viewpoint path branches
    FloatType viewpoint_path_initial_distance = 3;

    // Objective factor for reconstruction image
    FloatType objective_parameter_alpha = 0;
    // Objective factor for motion distance
    FloatType objective_parameter_beta = 0;
    // Factor in the exponential of the voxel information
    FloatType voxel_information_lambda = FloatType(0.1);

    // Maximum segment length that is reversed by 2 Opt
    std::size_t viewpoint_path_2opt_max_k_length = 15;

    // Filename of serialized viewpoint graph
    std::string viewpoint_graph_filename = "";

    // TODO: Needed?
    std::size_t num_sampled_poses = 100;
    std::size_t num_planned_viewpoints = 10;

    // Log file for OMPL motion planner
    std::string motion_planner_log_filename = "motion_planner.log";
  };

  using PointCloudType = ViewpointPlannerData::PointCloudType;
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

  using MotionPlannerType = MotionPlanner<FloatType>;
  using Motion = typename MotionPlannerType::Motion;

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

  // Pair of indices to index viewpoint motions
  struct ViewpointIndexPair {
    ViewpointIndexPair()
    : index1((ViewpointEntryIndex)-1), index2((ViewpointEntryIndex)-1) {}

    ViewpointIndexPair(ViewpointEntryIndex index1, ViewpointEntryIndex index2)
    : index1(index1), index2(index2) {
      if (this->index2 < this->index1) {
        using std::swap;
        swap(this->index1, this->index2);
      }
      AIT_ASSERT_DBG(this->index1 <= this->index2);
    }

    ViewpointEntryIndex index1;
    ViewpointEntryIndex index2;

    bool operator==(const ViewpointIndexPair& other) const {
      return index1 == other.index1 && index2 == other.index2;
    }

    struct Hash {
      std::size_t operator()(const ViewpointIndexPair& index_pair) const {
        std::size_t val { 0 };
        boost::hash_combine(val, index_pair.index1);
        boost::hash_combine(val, index_pair.index2);
        return val;
      }
    };

  private:
    // Boost serialization
    friend class boost::serialization::access;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version) {
      ar & index1;
      ar & index2;
    }
  };

  class VoxelWithInformationSetSaver {
  public:
    VoxelWithInformationSetSaver(const ViewpointPlannerData::OccupiedTreeType& bvh_tree) {
      // Compute consistent ordering of BVH nodes
      std::size_t voxel_index = 0;
      for (const ViewpointPlannerData::OccupiedTreeType::NodeType& node : bvh_tree) {
        voxel_index_map_.emplace(&node, voxel_index);
        ++voxel_index;
      }
    }

    template <typename Archive>
    void save(const VoxelWithInformationSet& voxel_set, Archive& ar, const unsigned int version) const {
      ar & voxel_set.size();
      for (const VoxelWithInformation& voxel_with_information : voxel_set) {
        std::size_t voxel_index = voxel_index_map_.at(voxel_with_information.voxel);
        ar & voxel_index;
        ar & voxel_with_information.information;
      }
    }

  private:
    std::unordered_map<const VoxelType*, std::size_t> voxel_index_map_;
  };

  class VoxelWithInformationSetLoader {
  public:
    VoxelWithInformationSetLoader(ViewpointPlannerData::OccupiedTreeType* bvh_tree) {
      // Compute consistent ordering of BVH nodes
      std::size_t voxel_index = 0;
      for (ViewpointPlannerData::OccupiedTreeType::NodeType& node : *bvh_tree) {
        index_voxel_map_.emplace(voxel_index, &node);
        ++voxel_index;
      }
    }

    template <typename Archive>
    void load(VoxelWithInformationSet* voxel_set, Archive& ar, const unsigned int version) const {
      std::size_t num_voxels;
      ar & num_voxels;
      for (std::size_t i = 0; i < num_voxels; ++i) {
        std::size_t voxel_index;
        ar & voxel_index;
        VoxelType* voxel = index_voxel_map_.at(voxel_index);
        FloatType information;
        ar & information;
        VoxelWithInformation voxel_with_information(voxel, information);
        voxel_set->emplace(std::move(voxel_with_information));
      }
    }

  private:
    std::unordered_map<std::size_t, VoxelType*> index_voxel_map_;
  };

  class ViewpointEntrySaver {
  public:
    ViewpointEntrySaver(const ViewpointEntryVector& entries, const ViewpointPlannerData::OccupiedTreeType& bvh_tree)
    : entries_(entries), voxel_set_saver_(bvh_tree) {}

  private:
    // Boost serialization
    friend class boost::serialization::access;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version) const {
      ar & entries_.size();
      for (const ViewpointEntry& entry : entries_) {
        // Save viewpoint
        ar & entry.viewpoint.pose();
        ar & entry.total_information;
        voxel_set_saver_.save(entry.voxel_set, ar, version);
      }
    }

    const ViewpointEntryVector& entries_;
    const VoxelWithInformationSetSaver voxel_set_saver_;
  };

  class ViewpointEntryLoader {
  public:
    ViewpointEntryLoader(ViewpointEntryVector* entries, ViewpointPlannerData::OccupiedTreeType* bvh_tree, SparseReconstruction* reconstruction)
    : entries_(entries), voxel_set_loader_(bvh_tree), reconstruction_(reconstruction) {}

  private:
    // Boost serialization
    friend class boost::serialization::access;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version) {
      const PinholeCamera* camera = static_cast<PinholeCamera*>(&(reconstruction_->getCameras().begin()->second));
      std::size_t num_entries;
      ar & num_entries;
      entries_->resize(num_entries);
      for (ViewpointEntry& entry : *entries_) {
        Pose pose;
        ar & pose;
        entry.viewpoint = Viewpoint(camera, pose);
        ar & entry.total_information;
        voxel_set_loader_.load(&entry.voxel_set, ar, version);
      }
    }

    ViewpointEntryVector* entries_;
    const VoxelWithInformationSetLoader voxel_set_loader_;
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

  private:
    // Boost serialization
    friend class boost::serialization::access;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version) {
      ar & viewpoint_index;
      ar & local_information;
      ar & acc_information;
      ar & local_motion_distance;
      ar & acc_motion_distance;
      ar & local_objective;
      ar & acc_objective;
    }
  };

  struct ViewpointMotion {
    ViewpointMotion(ViewpointEntryIndex from_index, ViewpointEntryIndex to_index, Motion motion)
    : from_index(from_index), to_index(to_index), motion(motion) {}

    ViewpointEntryIndex from_index;
    ViewpointEntryIndex to_index;
    Motion motion;
  };

  struct ViewpointPath {
    // Viewpoint entries on the path
    std::vector<ViewpointPathEntry> entries;
    // Indices into entries array ordered according to the flight path
    std::vector<std::size_t> order;
    // Set of voxels that are observed on the whole path
    VoxelWithInformationSet observed_voxel_set;
    // Accumulated information over the whole path
    FloatType acc_information;
    // Accumulated motion distance over the whole path
    FloatType acc_motion_distance;
    // Accumulated motion distance over the whole path
    FloatType acc_objective;

  private:
    // Boost serialization
    friend class boost::serialization::access;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version) {
      ar & entries;
      ar & order;
      ar & acc_information;
      ar & acc_motion_distance;
      ar & acc_objective;
    }
  };

  struct ViewpointPathComputationData {
    // Viewpoint entries sorted by ascending order of their novel information for the corresponding viewpoint
    std::vector<std::pair<ViewpointEntryIndex, FloatType>> sorted_new_informations;
    // Number of viewpoints in the entries array that have been connected to each other
    std::size_t num_connected_entries = 0;
    struct VoxelTriangulation {
      std::size_t num_triangulated = 0;
      std::vector<ViewpointEntryIndex> observing_entries;
    };
    std::unordered_map<const VoxelType*, VoxelTriangulation> voxel_observation_counts;
    std::unordered_map<const VoxelType*, std::vector<std::pair<ViewpointEntryIndex, ViewpointEntryIndex>>> triangulated_voxel_to_path_entries_map;
  };

  class ViewpointPathSaver {
  public:
    ViewpointPathSaver(const std::vector<ViewpointPath>& paths,
        const std::vector<ViewpointPathComputationData>& comp_datas,
        const ViewpointPlannerData::OccupiedTreeType& bvh_tree)
    : paths_(paths), comp_datas_(comp_datas), voxel_set_saver_(bvh_tree) {}

  private:
    // Boost serialization
    friend class boost::serialization::access;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version) const {
      ar & paths_.size();
      for (std::size_t i = 0; i < paths_.size(); ++i) {
        const ViewpointPath& path = paths_[i];
        const ViewpointPathComputationData& comp_data = comp_datas_[i];
        ar & path.entries;
        ar & path.order;
        ar & path.acc_information;
        ar & path.acc_motion_distance;
        ar & path.acc_objective;
        voxel_set_saver_.save(path.observed_voxel_set, ar, version);
        ar & comp_data.sorted_new_informations;
      }
    }

    const std::vector<ViewpointPath>& paths_;
    const std::vector<ViewpointPathComputationData>& comp_datas_;
    const VoxelWithInformationSetSaver voxel_set_saver_;
  };

  class ViewpointPathLoader {
  public:
    ViewpointPathLoader(std::vector<ViewpointPath>* paths,
        std::vector<ViewpointPathComputationData>* comp_datas,
        ViewpointPlannerData::OccupiedTreeType* bvh_tree)
    : paths_(paths), comp_datas_(comp_datas), voxel_set_loader_(bvh_tree) {}

  private:
    // Boost serialization
    friend class boost::serialization::access;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version) {
      std::size_t num_paths;
      ar & num_paths;
      paths_->resize(num_paths);
      comp_datas_->resize(num_paths);
      for (std::size_t i = 0; i < paths_->size(); ++i) {
        ViewpointPath& path = (*paths_)[i];
        ViewpointPathComputationData& comp_data = (*comp_datas_)[i];
        ar & path.entries;
        ar & path.order;
        ar & path.acc_information;
        ar & path.acc_motion_distance;
        ar & path.acc_objective;
        voxel_set_loader_.load(&path.observed_voxel_set, ar, version);
        ar & comp_data.sorted_new_informations;
      }
    }

    std::vector<ViewpointPath>* paths_;
    std::vector<ViewpointPathComputationData>* comp_datas_;
    VoxelWithInformationSetLoader voxel_set_loader_;
  };

  // Wrapper for computations on a viewpoint path
  struct ViewpointPathGraphWrapper {
    using WeightProperty = boost::property<boost::edge_weight_t, ViewpointPlanner::FloatType>;
    using BoostGraph = boost::adjacency_list<
        boost::vecS, boost::vecS, boost::bidirectionalS,
        boost::no_property,
        WeightProperty>;
    using Vertex = typename BoostGraph::vertex_descriptor;
    using IndexMap = boost::property_map<BoostGraph, boost::vertex_index_t>::type;
    using VertexIterator = BoostGraph::vertex_iterator;

    BoostGraph graph;
    IndexMap index_map;
    std::unordered_map<Vertex, ViewpointPlanner::ViewpointEntryIndex> viewpoint_indices;
    std::unordered_map<ViewpointPlanner::ViewpointEntryIndex, Vertex> viewpoint_index_to_vertex_map;
  };

  using ViewpointGraph = Graph<ViewpointEntryIndex, FloatType>;
  using ViewpointANN = ApproximateNearestNeighbor<FloatType, 3>;
  using FeatureViewpointMap = std::unordered_map<std::size_t, std::vector<const Viewpoint*>>;


  ViewpointPlanner(const Options* options,
      const MotionPlannerType::Options* motion_options, std::unique_ptr<ViewpointPlannerData> data);

  /// Get virtual camera
  const PinholeCamera& getVirtualCamera() const;

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

  void saveViewpointPath(const std::string& filename) const;

  void loadViewpointPath(const std::string& filename);

  rapidjson::Document getViewpointPathAsJson(const ViewpointPath& viewpoint_path) const;

  std::string getViewpointPathAsJsonString(const ViewpointPath& viewpoint_path) const;

  void exportViewpointPathAsJson(const std::string& filename, const ViewpointPath& viewpoint_path) const;

  RegionType getRoi() const {
    return data_->roi_;
  }

  BoundingBoxType getRoiBbox() const {
    return data_->roi_.getBoundingBox();
  }

  BoundingBoxType getBvhBbox() const {
    return data_->bvh_bbox_;
  }

  const OccupancyMapType* getOctree() const {
      return data_->octree_.get();
  }

  const ViewpointPlannerData::OccupiedTreeType& getBvhTree() const {
    return data_->occupied_bvh_;
  }

  const DenseReconstruction* getReconstruction() const {
      return data_->reconstruction_.get();
  }

  const PointCloudType* getDensePoints() const {
    return data_->dense_points_.get();
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

  /// Return viewpoint graph for reading. Mutex needs to be locked.
  ViewpointGraph& getViewpointGraph() {
    return viewpoint_graph_;
  }

  /// Return viewpoint paths for reading. Mutex needs to be locked.
  const std::vector<ViewpointPath>& getViewpointPaths() const {
    return viewpoint_paths_;
  }

  /// Return additional computation data for viewpoint paths. Mutex needs to be locked.
  const std::vector<ViewpointPathComputationData>& getViewpointPathsComputationData() const {
    return viewpoint_paths_data_;
  }

  /// Returns whether a motion between two viewpoints exists.
  bool hasViewpointMotion(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index) const;

  /// Return the motion between two viewpoints.
  Motion getViewpointMotion(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index) const;

  /// Adds a viewpoint motion to the graph. Reverses the order depending on the index ordering (see getViewpointMotion).
  void addViewpointMotion(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index, const Motion& motion);
  void addViewpointMotion(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index, Motion&& motion);

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
  bool isValidObjectPosition(const Vector3& position, const Vector3& object_extent) const;

  // TODO: remove
  void run();

  /// Generate a new viewpoint entry and add it to the graph.
  bool generateNextViewpointEntry();

  // Compute motions between viewpoints and update the graph with their cost.
  void computeViewpointMotions();

  /// Find the next best viewpoint to add to the viewpoint paths.
  bool findNextViewpointPathEntries(const FloatType alpha, const FloatType beta);

  /// Find initial viewpoints on viewpoint paths
  void findInitialViewpointPathEntries(const FloatType alpha, const FloatType beta);

  /// Find the next best viewpoint to add to the viewpoint paths and use parameters from options.
  bool findNextViewpointPathEntries();

  /// Compute new information score of a new viewpoint for a given viewpoint path
  FloatType computeNewInformation(const std::size_t viewpoint_path_index, const ViewpointEntryIndex new_viewpoint_index) const;

  /// Compute the connected components of the graph and return a pair of the component labels and the number of components.
  const std::pair<std::vector<std::size_t>, std::size_t>& getConnectedComponents() const;

  /// Compute a viewpoint tour for all paths
  void computeViewpointTour();

  // Raycasting and information computation

  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels with additional info.
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> getRaycastHitVoxels(
      const Pose& pose) const;

  /// Perform raycast on the BVH tree on a limited window around the center.
  /// Returns a vector of hit voxels with additional info.
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> getRaycastHitVoxels(
      const Pose& pose, const std::size_t width, const std::size_t height) const;

#if WITH_CUDA
  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels with additional info.
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> getRaycastHitVoxelsCuda(
      const Pose& pose) const;

  /// Perform raycast on the BVH tree on a limited window around the center.
  /// Returns a vector of hit voxels with additional info.
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> getRaycastHitVoxelsCuda(
      const Pose& pose, const std::size_t width, const std::size_t height) const;
#endif

  /// Perform raycast on the BVH tree.
  /// Returns the set of hit voxels with corresponding information + the total information of all voxels.
  std::pair<VoxelWithInformationSet, FloatType>
    getRaycastHitVoxelsWithInformationScore(const Pose& pose) const;

  /// Returns the information score for a single hit voxel.
  FloatType computeInformationScore(const VoxelType* node) const;

  /// Returns the information score for a single hit voxel result.
  FloatType computeInformationScore(const ViewpointPlannerData::OccupiedTreeType::IntersectionResult& result) const;

  /// Returns the information score for a container of VoxelWithInformation objects.
  template <typename Iterator>
  FloatType computeInformationScore(Iterator first, Iterator last) const;

  // Matching score computation

  std::unordered_set<Point3DId> computeProjectedMapPoints(const CameraId camera_id, const Pose& pose,
      FloatType projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

  std::unordered_set<Point3DId> computeFilteredMapPoints(const CameraId camera_id, const Pose& pose,
      FloatType projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

  std::unordered_set<Point3DId> computeVisibleMapPoints(const CameraId camera_id, const Pose& pose,
      FloatType projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

  std::unordered_set<Point3DId> computeVisibleMapPointsFiltered(const CameraId camera_id, const Pose& pose,
      FloatType projection_margin=Viewpoint::DEFAULT_PROJECTION_MARGIN) const;

  /// Convert local ENU coordinates to GPS coordinates
  using GpsCoordinateType = reconstruction::SfmToGpsTransformation::GpsCoordinate;
  GpsCoordinateType convertPositionToGps(const Vector3& position) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /// Remove duplicate hit voxels from raycast results
  void removeDuplicateRaycastHitVoxels(
      std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult>* raycast_results) const;

  /// Remove points that are occluded from the given pose.
  /// A point needs to be dist_margin behind a voxel surface to be removed.
  void removeOccludedPoints(const Pose& pose, std::unordered_set<Point3DId>& point3D_ids, FloatType dist_margin) const;

  /// Compute information factor based on observation count (i.e. exp(-theta * observation_count)
  WeightType computeObservationInformationFactor(CounterType observation_count) const;

  /// Add a viewpoint entry to the graph.
  void addViewpointEntry(ViewpointEntry&& viewpoint_entry);

  /// Add a viewpoint entry. This also computes motion distances to other viewpoints and updates edges in the graph.
  void addViewpointEntryAndMotions(ViewpointEntry&& viewpoint_entry, const std::vector<ViewpointMotion>& motions);

  /// Add a viewpoint entry without acquiring a lock. Returns the index of the new viewpoint entry.
  ViewpointEntryIndex addViewpointEntryWithoutLock(ViewpointEntry&& viewpoint_entry);

  /// Find a matching viewpoint for stereo matching and add it. If a suitable viewpoint already is on the path no viewpoint is
  /// added. Returns whether a suitable viewpoint was found. Must be called with lock.
  bool addMatchingStereoViewpointWithoutLock(
      ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data, const ViewpointPathEntry& path_entry);

  /// Find motion paths from provided viewpoint to neighbors in the viewpoint graph.
  std::vector<ViewpointMotion> findViewpointMotions(const Pose& from_pose, std::size_t tentative_viewpoint_index);

  /// Find motion paths from provided viewpoint to neighbors in the viewpoint graph.
  std::vector<ViewpointMotion> findViewpointMotions(const ViewpointEntryIndex from_index);

  /// Checks whether a voxel can be triangulated on the viewpoint path
  std::pair<bool, ViewpointEntryIndex> canVoxelBeTriangulated(const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data,
      const ViewpointEntry& new_viewpoint, const VoxelWithInformation& voxel) const;
  template <typename Iterator>
  std::pair<bool, ViewpointEntryIndex> canVoxelBeTriangulated(
      const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data,
      const ViewpointEntry& new_viewpoint, Iterator it) const;

  /// Compute and initialize information scores of other viewpoints given a path
  void initializeViewpointPathInformations(ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data);

  /// Compute and update information scores of other viewpoints given a path.
  void updateViewpointPathInformations(ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data) const;

  /// Returns the best next viewpoint index.
  std::pair<ViewpointEntryIndex, FloatType> getBestNextViewpoint(
      const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data, const bool randomize) const;

  /// Find the next best viewpoint to add to a single viewpoint path.
  bool findNextViewpointPathEntry(
      ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data,
      const bool randomize, const FloatType alpha, const FloatType beta);

  /// Compute new information score of a new viewpoint given a path
  FloatType computeNewInformation(const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data,
      const ViewpointEntryIndex new_viewpoint_index) const;

  /// Compute an upper bound on the information value of a given path
  FloatType computeViewpointPathInformationUpperBound(
      const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data, const std::size_t max_num_viewpoints) const;

  /// Report info of current viewpoint paths
  void reportViewpointPathsStats() const;

  /// Compute approx. shortest cycle to cover all viewpoints in the path (i.e. solve TSP problem)
  std::vector<std::size_t> computeApproximateShortestCycle(const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data);

  /// Reorders the viewpoint path to an approx. shortest cycle covering all viewpoints (i.e. TSP solution)
  void solveApproximateTSP(ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data);

  /// Ensures that all viewpoints on a path are connected by searching for shortest motion paths if necessary
  void ensureConnectedViewpointPath(ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data);

  /// Uses 2 Opt to improve the viewpoint tour
  void improveViewpointTourWith2Opt(ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data);

  /// Find shortest motion between two viewpoints using A-Star on the viewpoint graph. Returns true if successful.
  bool findAndAddShortestMotion(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index);

  /// Find shortest motion between a new viewpoint and existing viewpoints using A-Star on the viewpoint graph. Returns true if successful.
  template <typename Iterator>
  bool findAndAddShortestMotions(const ViewpointEntryIndex from_index, Iterator to_index_first, Iterator to_index_last);

  /// Computes the length of a viewpoint tour (cycle)
  FloatType computeTourLength(const ViewpointPath& viewpoint_path, const std::vector<std::size_t>& order) const;

  /// Returns a new tour order where the path from k_start to k_end is reversed
  std::vector<std::size_t> swap2Opt(
      ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data,
      const std::size_t k_start, const std::size_t k_end) const;

private:
  /// Create a subgraph with the nodes from a viewpoint path
  ViewpointPathGraphWrapper createViewpointPathGraph(const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data);

  mutable ait::Random<FloatType, std::int64_t> random_;

  Options options_;

  BoundingBoxType pose_sample_bbox_;
  Vector3 drone_extent_;
  FloatType triangulation_max_cos_angle_;
  FloatType triangulation_min_sin_angle_square_;
  FloatType triangulation_max_sin_angle_square_;
  FloatType triangulation_max_angular_deviation_;

  std::unique_ptr<ViewpointPlannerData> data_;
  PinholeCamera virtual_camera_;

  std::mutex mutex_;

  MotionPlannerType motion_planner_;

  // All tentative viewpoints
  ViewpointEntryVector viewpoint_entries_;
  // Number of real viewpoints at the beginning of the viewpoint_entries_ vector
  // These need to be distinguished because they could be in non-free space of the map
  std::size_t num_real_viewpoints_;
  // Approximate nearest neighbor index for viewpoints
  ViewpointANN viewpoint_ann_;
  // Graph of tentative viewpoints with motion distances
  ViewpointGraph viewpoint_graph_;
  // Connected components of viewpoint graph
  mutable std::pair<std::vector<std::size_t>, std::size_t> viewpoint_graph_components_;
  // Flag indicating whether connected components are valid
  mutable bool viewpoint_graph_components_valid_;
  // Motion description of the connections in the viewpoint graph (indexed by viewpoint id pair)
  std::unordered_map<ViewpointIndexPair, Motion, ViewpointIndexPair::Hash> viewpoint_graph_motions_;
  // Current viewpoint paths
  std::vector<ViewpointPath> viewpoint_paths_;
  // Data used for computation of viewpoint paths
  std::vector<ViewpointPathComputationData> viewpoint_paths_data_;
  // Map from triangle index to viewpoints that project features into it
  FeatureViewpointMap feature_viewpoint_map_;
};

#include "viewpoint_planner.hxx"

#include "viewpoint_planner_graph.hxx"
