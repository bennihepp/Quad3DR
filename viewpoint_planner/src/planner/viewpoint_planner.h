// viewpoint_planner.h
//==================================================
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
#include <ait/boost_serialization_utils.h>
#if WITH_OPENGL_OFFSCREEN
#include <QOpenGLContext>
#include <QOpenGLFramebufferObject>
#include <QOffscreenSurface>
#include <QOpenGLFunctions>
#include <QSurfaceFormat>
#include <ait/qt_utils.h>
#include <ait/color.h>
#include "../rendering/triangle_drawer.h"
#endif
#include <rapidjson/document.h>
#include <ait/common.h>
#include <ait/options.h>
#include <ait/eigen_options.h>
#include <ait/random.h>
#include <ait/eigen_utils.h>
#include <ait/serialization.h>
#include <ait/graph_boost.h>
#include <bh/math/continuous_grid3d.h>
#include <bh/nn/approximate_nearest_neighbor.h>
#include "../mLib/mLib.h"
#include "../octree/occupancy_map.h"
#include "../reconstruction/dense_reconstruction.h"
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
  using size_t = std::size_t;
  using FloatType = ViewpointPlannerData::FloatType;
  USE_FIXED_EIGEN_TYPES(FloatType)
  using BoundingBoxType = ViewpointPlannerData::BoundingBoxType;
  using ContinuousGridType = bh::ContinuousGrid3D<FloatType, size_t>;
  using RegionType = ViewpointPlannerData::RegionType;
  using RayType = bh::Ray<FloatType>;
  using Pose = ait::Pose<FloatType>;

  static constexpr FloatType kDotProdEqualTolerance = FloatType { 1e-5 };
  static constexpr FloatType kOcclusionDistMarginFactor = FloatType { 4 };

  struct Options : ait::ConfigOptions {
    Options()
    : ait::ConfigOptions("viewpoint_planner", "ViewpointPlanner options") {
      addOption<bool>("enable_cuda", &enable_cuda);
      addOption<bool>("enable_opengl", &enable_opengl);
      addOption<bool>("dump_poisson_mesh_normals_image", &dump_poisson_mesh_normals_image);
      addOption<bool>("dump_poisson_mesh_depth_image", &dump_poisson_mesh_depth_image);
      addOption<bool>("dump_stereo_matching_images", &dump_stereo_matching_images);
      addOption<size_t>("rng_seed", &rng_seed);
      addOption<FloatType>("virtual_camera_scale", &virtual_camera_scale);
      addOption<FloatType>("virtual_camera_width", &virtual_camera_width);
      addOption<FloatType>("virtual_camera_height", &virtual_camera_height);
      addOption<FloatType>("virtual_camera_focal_length", &virtual_camera_focal_length);
      addOption<FloatType>("raycast_min_range", &raycast_min_range);
      addOption<FloatType>("raycast_max_range", &raycast_max_range);
      addOption<bool>("ignore_real_observed_voxels", &ignore_real_observed_voxels);
      addOption<FloatType>("drone_extent_x", 3);
      addOption<FloatType>("drone_extent_y", 3);
      addOption<FloatType>("drone_extent_z", 3);
      addOption<Vector3>("drone_start_position", &drone_start_position);
      addOptionalOption<Vector3>("sampling_bbox_min");
      addOptionalOption<Vector3>("sampling_bbox_max");
      addOption<FloatType>("sampling_roi_factor", &sampling_roi_factor);
      addOption<FloatType>("pose_sample_min_radius", &pose_sample_min_radius);
      addOption<FloatType>("pose_sample_max_radius", &pose_sample_max_radius);
      addOption<size_t>("pose_sample_num_trials", &pose_sample_num_trials);
      addOption<FloatType>("viewpoint_sample_without_reference_probability", &viewpoint_sample_without_reference_probability);
      addOption<size_t>("viewpoint_min_voxel_count", &viewpoint_min_voxel_count);
      addOption<FloatType>("viewpoint_min_information", &viewpoint_min_information);
      addOption<size_t>("viewpoint_sample_motion_and_matching_knn", &viewpoint_sample_motion_and_matching_knn);
      addOption<FloatType>("viewpoint_information_factor", &viewpoint_information_factor);
      addOption<bool>("viewpoint_generate_stereo_pairs", &viewpoint_generate_stereo_pairs);
      addOption<FloatType>("triangulation_min_angle_degrees", &triangulation_min_angle_degrees);
      addOption<FloatType>("triangulation_max_angle_degrees", &triangulation_max_angle_degrees);
      addOption<FloatType>("triangulation_max_dist_deviation_ratio", &triangulation_max_dist_deviation_ratio);
      addOption<FloatType>("triangulation_max_angular_deviation_degrees", &triangulation_max_angular_deviation_degrees);
      addOption<FloatType>("triangulation_min_voxel_overlap_ratio", &triangulation_min_voxel_overlap_ratio);
      addOption<FloatType>("triangulation_min_information_overlap_ratio", &triangulation_min_information_overlap_ratio);
      addOption<size_t>("triangulation_knn", &triangulation_knn);
      addOption<size_t>("triangulation_min_count", &triangulation_min_count);
      addOption<size_t>("viewpoint_count_grid_dimension", &viewpoint_count_grid_dimension);
      addOption<size_t>("viewpoint_discard_dist_knn", &viewpoint_discard_dist_knn);
      addOption<FloatType>("viewpoint_discard_dist_thres_square", &viewpoint_discard_dist_thres_square);
      addOption<size_t>("viewpoint_discard_dist_count_thres", &viewpoint_discard_dist_count_thres);
      addOption<FloatType>("viewpoint_discard_dist_real_thres_square", &viewpoint_discard_dist_real_thres_square);
      addOption<FloatType>("viewpoint_path_discard_dist_thres_square", &viewpoint_path_discard_dist_thres_square);
      addOption<size_t>("viewpoint_motion_max_neighbors", &viewpoint_motion_max_neighbors);
      addOption<FloatType>("viewpoint_motion_max_dist_square", &viewpoint_motion_max_dist_square);
      addOption<size_t>("viewpoint_motion_densification_max_depth", &viewpoint_motion_densification_max_depth);
      addOption<FloatType>("viewpoint_motion_penalty_per_graph_vertex", &viewpoint_motion_penalty_per_graph_vertex);
      addOption<size_t>("viewpoint_path_branches", &viewpoint_path_branches);
      addOption<FloatType>("viewpoint_path_initial_distance", &viewpoint_path_initial_distance);
      addOption<FloatType>("objective_parameter_alpha", &objective_parameter_alpha);
      addOption<FloatType>("objective_parameter_beta", &objective_parameter_beta);
      addOption<FloatType>("voxel_sensor_size_ratio_threshold", &voxel_sensor_size_ratio_threshold);
      addOption<FloatType>("voxel_sensor_size_ratio_inv_falloff_factor", &voxel_sensor_size_ratio_inv_falloff_factor);
      addOption<bool>("incidence_ignore_dot_product_sign", &incidence_ignore_dot_product_sign);
      addOption<FloatType>("incidence_angle_threshold_degrees", &incidence_angle_threshold_degrees);
      addOption<FloatType>("incidence_angle_inv_falloff_factor_degrees", &incidence_angle_inv_falloff_factor_degrees);
      addOption<FloatType>("sparse_matching_depth_tolerance", &sparse_matching_depth_tolerance);
      addOption<FloatType>("sparse_matching_dot_product_threshold", &sparse_matching_dot_product_threshold);
      addOption<FloatType>("sparse_matching_score_threshold", &sparse_matching_score_threshold);
      addOption<FloatType>("sparse_matching_max_distance_deviation_factor", &sparse_matching_max_distance_deviation_factor);
      addOption<FloatType>("sparse_matching_max_normal_deviation_factor", &sparse_matching_max_normal_deviation_factor);
      addOption<FloatType>("sparse_matching_min_distance", &sparse_matching_min_distance);
      addOption<FloatType>("sparse_matching_min_angular_distance", &sparse_matching_min_angular_distance);
      addOption<size_t>("viewpoint_path_2opt_max_k_length", &viewpoint_path_2opt_max_k_length);
      addOption<std::string>("viewpoint_graph_filename", &viewpoint_graph_filename);
      // TODO:
      addOption<size_t>("num_sampled_poses", &num_sampled_poses);
      addOption<size_t>("num_planned_viewpoints", &num_planned_viewpoints);
      addOption<std::string>("motion_planner_log_filename", &motion_planner_log_filename);
    }

    ~Options() override {}

    // Whether to enable CUDA
    bool enable_cuda = true;
    // Whether to enable OpenGL
    bool enable_opengl = true;
    // Whether to dump the poisson mesh normals image after rendering
    bool dump_poisson_mesh_normals_image = false;
    // Whether to dump the poisson mesh depth image after rendering
    bool dump_poisson_mesh_depth_image = false;
    // Whether to write out stereo matching image pairs when generating viewpoint path
    bool dump_stereo_matching_images = false;

    // Random number generator seed (0 will seed from current time)
    size_t rng_seed = 0;

    // Scale factor from real camera to virtual camera
    FloatType virtual_camera_scale = 0.05f;
    // Parameters for virtual camera (overriding scale factor)
    FloatType virtual_camera_width = -1;
    FloatType virtual_camera_height = -1;
    FloatType virtual_camera_focal_length = -1;

    // Min and max range for raycast
    FloatType raycast_min_range = 0;
    FloatType raycast_max_range = 60;

    // Whether to ignore the voxels already observed by a previous reconstruction
    bool ignore_real_observed_voxels = false;

    // Viewpoint sampling parameters

    // Start position of drone. Viewpoint sampling starts from here to ensure connectivity.
    Vector3 drone_start_position = Vector3(0, 0, 0);

    // Region of interest dilation for viewpoint sampling
    FloatType sampling_roi_factor = 2;
    // Spherical shell for viewpoint sampling
    FloatType pose_sample_min_radius = 2;
    FloatType pose_sample_max_radius = 5;

    // Number of trials for viewpoint position sampling
    size_t pose_sample_num_trials = 100;
    // Probability of sampling a new viewpoint independent of a reference viewpoint
    FloatType viewpoint_sample_without_reference_probability = 0.1f;
    // Ensure that every viewpoint is part of a stereo pair
    bool viewpoint_generate_stereo_pairs;
    // Minimum voxel count for viewpoints
    size_t viewpoint_min_voxel_count = 100;
    // Minimum information for viewpoints
    FloatType viewpoint_min_information = 10;
    // Number of neighbours to check for reachability and matchability when generating new viewpoint entry
    size_t viewpoint_sample_motion_and_matching_knn = 20;

    // Factor by which information of a viewpoint is multiplied (i.e. to encourage redundancy)
    FloatType viewpoint_information_factor = FloatType(1 / 3.0);

    // Minimum angle between two views so that a voxel can be triangulated
    FloatType triangulation_min_angle_degrees = 5;
    // Maximum angle between two views so that a voxel can be triangulated
    FloatType triangulation_max_angle_degrees = 15;
    // Maximum deviation ratio of distances between stereo pair
    FloatType triangulation_max_dist_deviation_ratio = FloatType(0.1);
    // Maximum angular deviation between stereo viewpoints
    FloatType triangulation_max_angular_deviation_degrees = 20;
    // Minimum ratio of voxel overlap for a viewpoint to be used for a stereo pair
    FloatType triangulation_min_voxel_overlap_ratio = 0.4f;
    // Minimum ratio of information overlap for a viewpoint to be used for a stereo pair
    FloatType triangulation_min_information_overlap_ratio = 0.2f;
    // Number of nearest neighbors to consider for stereo pair search
    size_t triangulation_knn = 50;
    // Minimum number of viewpairs that can triangulate a voxel to be observed
    size_t triangulation_min_count = 2;

    // Dimension of the viewpoint count grid (x, y and z dimension)
    size_t viewpoint_count_grid_dimension = 100;

    // For checking whether a sampled viewpoint is too close to existing ones
    //
    // How many nearest neighbors to consider
    size_t viewpoint_discard_dist_knn = 10;
    // Squared distance considered to be too close
    FloatType viewpoint_discard_dist_thres_square = 4;
    // How many other viewpoints need to be too close for the sampled viewpoint to be discarded
    size_t viewpoint_discard_dist_count_thres = 3;
    // Squared distance to a real viewpoint considered to be too close
    FloatType viewpoint_discard_dist_real_thres_square = 9;

    // Squared minimum distance to other viewpoints on a viewpoint path
    FloatType viewpoint_path_discard_dist_thres_square = 4;

    // Maximum number of neighbors to consider for finding a motion path
    size_t viewpoint_motion_max_neighbors = 10;
    // Maximum distance between viewpoints for finding a motion path
    FloatType viewpoint_motion_max_dist_square = 10;
    // Number of connections to traverse in the graph for densifying edges
    size_t viewpoint_motion_densification_max_depth = 5;
    // Motion penalty for each viewpoint graph vertex on a motion path
    FloatType viewpoint_motion_penalty_per_graph_vertex = 0;

    // Number of viewpoint path branches to explore in parallel
    size_t viewpoint_path_branches = 10;
    // Minimum distance between initial viewpoints on viewpoint path branches
    FloatType viewpoint_path_initial_distance = 3;

    // Objective factor for reconstruction image
    FloatType objective_parameter_alpha = 0;
    // Objective factor for motion distance
    FloatType objective_parameter_beta = 0;
    // Whether to ignore the dot-product sign when computing incidence information factor
    bool incidence_ignore_dot_product_sign = false;
    // Incidence angle below which the full information is achieved
    FloatType incidence_angle_threshold_degrees = 30;
    // Inverse information falloff factor above the incidence angle threshold
    FloatType incidence_angle_inv_falloff_factor_degrees = 30;
    // Relative voxel size on sensor above which full information is achieved
    FloatType voxel_sensor_size_ratio_threshold = FloatType(0.002);
    // Inverse information falloff factor above the sensor size threshold
    FloatType voxel_sensor_size_ratio_inv_falloff_factor = FloatType(0.001);

    // Depth tolerance when deciding whether a 3d point is visible from a viewpoint
    FloatType sparse_matching_depth_tolerance = FloatType(0.2);
    // Minimum dot product between viewpoint orientations for a point to be matchable
    FloatType sparse_matching_dot_product_threshold = FloatType(0.8);
    // Threshold for sparse matching score
    FloatType sparse_matching_score_threshold = FloatType(50);
    // Within how many standard deviations of the sparse point distance a point is still visible
    FloatType sparse_matching_max_distance_deviation_factor = 3;
    // Within how many standard deviations of the sparse point normal dot product a point is still visible
    FloatType sparse_matching_max_normal_deviation_factor = 3;
    // Distance threshold below which viewpoints are assumed to be matchable
    FloatType sparse_matching_min_distance = FloatType(1);
    // Angular distance threshold below which viewpoints are assumed to be matchable
    FloatType sparse_matching_min_angular_distance = 10 * FloatType(M_PI / 180.0);

    // Maximum segment length that is reversed by 2 Opt
    size_t viewpoint_path_2opt_max_k_length = 15;

    // Filename of serialized viewpoint graph
    std::string viewpoint_graph_filename = "";

    // TODO: Needed?
    size_t num_sampled_poses = 100;
    size_t num_planned_viewpoints = 10;

    // Log file for OMPL motion planner
    std::string motion_planner_log_filename = "motion_planner.log";
  };

  friend class VoxelMapSaver;
  friend class VoxelMapLoader;
  friend class VoxelWithInformationSetSaver;
  friend class VoxelWithInformationSetLoader;
  friend class ViewpointEntrySaver;
  friend class ViewpointEntryLoader;
  friend class ViewpointPathSaver;
  friend class ViewpointPathEntriesLoader;
  friend class ViewpointPathLoader;

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
  using OccupiedTreeType = ViewpointPlannerData::OccupiedTreeType;
  using VoxelType = ViewpointPlannerData::OccupiedTreeType::NodeType;

  using MotionPlannerType = MotionPlanner<FloatType>;
  using SE3Motion = typename MotionPlannerType::Motion;

  /// Voxel node and it's corresponding amount of information
  struct VoxelWrapper {
    VoxelWrapper(const VoxelType* voxel)
    : voxel(voxel) {}

    const VoxelType* voxel;

    bool operator==(const VoxelWrapper& other) const {
      return voxel == other.voxel;
    }

    struct Hash {
      size_t operator()(const VoxelWrapper& voxel_with_information) const {
        size_t val { 0 };
              boost::hash_combine(val, voxel_with_information.voxel);
              return val;
      }
    };
  };

  /// Voxel node and it's corresponding amount of information
  struct VoxelWithInformation {
    VoxelWithInformation(const VoxelType* voxel, const FloatType information)
    : voxel(voxel), information(information) {}

    const VoxelType* voxel;
    FloatType information;

    bool operator==(const VoxelWithInformation& other) const {
      return voxel == other.voxel && information == other.information;
    }

    struct VoxelHash {
      size_t operator()(const VoxelWithInformation& voxel_with_information) const {
        size_t val { 0 };
              boost::hash_combine(val, voxel_with_information.voxel);
              return val;
      }
    };

    struct Hash {
      size_t operator()(const VoxelWithInformation& voxel_with_information) const {
        size_t val { 0 };
              boost::hash_combine(val, voxel_with_information.voxel);
              boost::hash_combine(val, voxel_with_information.information);
              return val;
      }
    };
  };

  using VoxelWithInformationSet = std::unordered_set<VoxelWithInformation, VoxelWithInformation::VoxelHash>;
  using VoxelMap = std::unordered_map<VoxelWrapper, FloatType, VoxelWrapper::Hash>;

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
      AIT_ASSERT(this->index1 <= this->index2);
    }

    ViewpointEntryIndex index1;
    ViewpointEntryIndex index2;

    bool operator==(const ViewpointIndexPair& other) const {
      return index1 == other.index1 && index2 == other.index2;
    }

    struct Hash {
      size_t operator()(const ViewpointIndexPair& index_pair) const {
        size_t val { 0 };
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

  /// Describes a viewpoint and the information it adds to the previous viewpoints
  struct ViewpointPathEntry {
    // TODO: Make non-default constructor to ensure proper index and viewpoint
    ViewpointPathEntry()
    : viewpoint_index((ViewpointEntryIndex)-1),
      local_information(std::numeric_limits<FloatType>::lowest()),
      acc_information(std::numeric_limits<FloatType>::lowest()),
      local_motion_distance(std::numeric_limits<FloatType>::max()),
      acc_motion_distance(std::numeric_limits<FloatType>::max()),
      local_objective(std::numeric_limits<FloatType>::lowest()),
      acc_objective(std::numeric_limits<FloatType>::lowest()),
      mvs_viewpoint(true) {}

    ViewpointEntryIndex viewpoint_index;
    // TODO: Is it necessary to save all the accumulated information with a path entry?
    FloatType local_information;
    FloatType acc_information;
    FloatType local_motion_distance;
    FloatType acc_motion_distance;
    FloatType local_objective;
    FloatType acc_objective;
    bool mvs_viewpoint;
    Viewpoint viewpoint;

  private:
    // Boost serialization
    friend class boost::serialization::access;
    friend class ViewpointPathEntriesLoader;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version) {
      ar & viewpoint_index;
      ar & local_information;
      ar & acc_information;
      ar & local_motion_distance;
      ar & acc_motion_distance;
      ar & local_objective;
      ar & acc_objective;
      if (version > 0) {
        ar & mvs_viewpoint;
      }
      else {
        mvs_viewpoint = true;
      }
    }
  };

  struct ViewpointMotion {
   public:
    using Iterator = std::vector<ViewpointEntryIndex>::iterator;
    using ConstIterator = std::vector<ViewpointEntryIndex>::const_iterator;

    explicit ViewpointMotion() {}

    explicit ViewpointMotion(const std::vector<ViewpointEntryIndex>& viewpoint_indices, const std::vector<SE3Motion>& se3_motions)
    : viewpoint_indices_(viewpoint_indices), se3_motions_(se3_motions) {
#if !AIT_RELEASE
      AIT_ASSERT(viewpoint_indices_.size() >= 2);
      AIT_ASSERT(se3_motions_.size() == viewpoint_indices_.size() - 1);
#endif
    }

    explicit ViewpointMotion(std::vector<ViewpointEntryIndex>&& viewpoint_indices, std::vector<SE3Motion>&& se3_motions)
    : viewpoint_indices_(std::move(viewpoint_indices)), se3_motions_(std::move(se3_motions)) {
#if !AIT_RELEASE
      AIT_ASSERT(viewpoint_indices_.size() >= 2);
      AIT_ASSERT(se3_motions_.size() == viewpoint_indices_.size() - 1);
#endif
    }

    bool isValid() const {
      return !viewpoint_indices_.empty();
    }
    ViewpointEntryIndex fromIndex() const {
      return viewpoint_indices_.front();
    }

    ViewpointEntryIndex toIndex() const {
      return viewpoint_indices_.back();
    }

    Iterator begin() {
      return viewpoint_indices_.begin();
    }

    Iterator end() {
      return viewpoint_indices_.end();
    }

    ConstIterator begin() const {
      return viewpoint_indices_.begin();
    }

    ConstIterator end() const {
      return viewpoint_indices_.end();
    }

    const std::vector<ViewpointEntryIndex>& viewpointIndices() const {
      return viewpoint_indices_;
    }

    const std::vector<SE3Motion>& se3Motions() const {
      return se3_motions_;
    }

    FloatType distance() const {
      const FloatType dist = std::accumulate(
          se3_motions_.begin(), se3_motions_.end(), FloatType(0), [] (const FloatType value, const SE3Motion& se3_motion) -> FloatType {
        // Sanity check
//#if !AIT_RELEASE
//        AIT_ASSERT(se3_motion.poses.size() >= 2);
//        FloatType local_dist = 0;
//        for (auto pose_it = se3_motion.poses.begin() + 1; pose_it != se3_motion.poses.end(); ++pose_it) {
//          const auto prev_pose_it = pose_it - 1;
//          local_dist += (pose_it->getWorldPosition() - prev_pose_it->getWorldPosition()).norm();
//        }
//        AIT_ASSERT(ait::isApproxEqual(se3_motion.distance, local_dist, FloatType(1e-2)));
//#endif
        return value + se3_motion.distance;
      });
      return dist;
    }

    void append(const ViewpointMotion& other) {
      if (isValid()) {
        AIT_ASSERT(toIndex() == other.fromIndex());
        for (auto it = other.viewpointIndices().begin() + 1; it != other.viewpointIndices().end(); ++it) {
          viewpoint_indices_.push_back(*it);
          const SE3Motion& se3_motion = other.se3Motions()[it - 1 - other.viewpointIndices().begin()];
          se3_motions_.push_back(se3_motion);
        }
      }
      else {
        *this = other;
      }
    }

    void reverse() {
#if !AIT_RELEASE
      const FloatType dist_before = distance();
#endif
      std::reverse(viewpoint_indices_.begin(), viewpoint_indices_.end());
      std::reverse(se3_motions_.begin(), se3_motions_.end());
      for (SE3Motion& se3_motion : se3_motions_) {
        std::reverse(se3_motion.poses.begin(), se3_motion.poses.end());
      }
#if !AIT_RELEASE
      const FloatType dist_after = distance();
      AIT_ASSERT(ait::isApproxEqual(dist_before, dist_after, FloatType(1e-2)));
#endif
    }

    ViewpointMotion getReverse() const {
      ViewpointMotion copy(viewpoint_indices_, se3_motions_);
      copy.reverse();
      return copy;
    }

  private:
    // Boost serialization
    friend class boost::serialization::access;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version) {
     ar & viewpoint_indices_;
     ar & se3_motions_;
    }

    std::vector<ViewpointEntryIndex> viewpoint_indices_;
    std::vector<SE3Motion> se3_motions_;
  };

  struct ViewpointPath {
    // Viewpoint entries on the path
    std::vector<ViewpointPathEntry> entries;
    // Indices into entries array ordered according to the flight path
    std::vector<size_t> order;
//    // Set of voxels that are observed on the whole path
//    VoxelWithInformationSet observed_voxel_set;
    // Map with voxels and partial information on the whole path
    VoxelMap observed_voxel_map;
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
    std::vector<std::tuple<ViewpointEntryIndex, FloatType, bool>> sorted_new_informations;
    // Number of viewpoints in the entries array that have been connected to each other
    size_t num_connected_entries = 0;
    struct VoxelTriangulation {
      size_t num_triangulated = 0;
      std::vector<ViewpointEntryIndex> observing_entries;
    };
    std::unordered_map<const VoxelType*, VoxelTriangulation> voxel_observation_counts;
    std::unordered_map<const VoxelType*, std::vector<std::pair<ViewpointEntryIndex, ViewpointEntryIndex>>> triangulated_voxel_to_path_entries_map;
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
  using ViewpointANN = bh::ApproximateNearestNeighbor<FloatType, 3>;
  using FeatureViewpointMap = std::unordered_map<size_t, std::vector<const Viewpoint*>>;


  ViewpointPlanner(const Options* options,
      const MotionPlannerType::Options* motion_options, std::unique_ptr<ViewpointPlannerData> data);

  const Options getOptions() const;

  const ViewpointPlannerData& getPlannerData() const;

  ViewpointPlannerData& getPlannerData();

  /// Get virtual camera
  const PinholeCamera& getVirtualCamera() const;

  const Vector3& getDroneExtent() const;

  /// Set size of virtual camera used for raycasting by scaling a real camera.
  void setScaledVirtualCamera(CameraId camera_id, FloatType scale_factor);

  /// Set size of virtual camera used for raycasting.
  void setVirtualCamera(size_t virtual_width, size_t virtual_height, const FloatType focal_length);

  /// Set size of virtual camera used for raycasting.
  void setVirtualCamera(size_t virtual_width, size_t virtual_height, const Matrix4x4& virtual_intrinsics);

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

  void exportViewpointPathAsText(const std::string& filename, const ViewpointPath& viewpoint_path) const;

  void exportViewpointPathAsSparseReconstruction(const std::string& path, const ViewpointPath& viewpoint_path) const;

  bool isSparsePointVisible(const Viewpoint& viewpoint, const Point3D& point3d) const;

  bool isSparsePointMatchable(const Viewpoint& viewpoint1, const Viewpoint& viewpoint2, const Point3D& point3d) const;

  QImage drawSparsePoints(
      const Viewpoint& viewpoint, const FloatType sparse_point_size = FloatType(1.1)) const;

  QImage drawSparsePoints(
      const ViewpointEntryIndex viewpoint_index, const FloatType sparse_point_size = FloatType(1.1)) const;

  void dumpSparsePoints(const Viewpoint& viewpoint,
                             const std::string& filename,
                             const FloatType sparse_point_size = FloatType(1.1)) const;

  void dumpSparsePoints(const ViewpointEntryIndex viewpoint_index,
                             const std::string& filename,
                             const FloatType sparse_point_size = FloatType(1.1)) const;

  void dumpSparsePoints(const ViewpointEntryIndex viewpoint_index,
                             const FloatType sparse_point_size = FloatType(1.1)) const;

  QImage drawSparseMatching(
      const Viewpoint& viewpoint1, const Viewpoint& viewpoint2,
      const bool draw_lines = true,
      const FloatType sparse_point_size = FloatType(1.1), const FloatType match_line_width = FloatType(0.05)) const;

  QImage drawSparseMatching(
      const ViewpointEntryIndex viewpoint_index1, const ViewpointEntryIndex viewpoint_index2,
      const bool draw_lines = true,
      const FloatType sparse_point_size = FloatType(1.1), const FloatType match_line_width = FloatType(0.05)) const;

  void dumpSparseMatching(
      const ViewpointEntryIndex viewpoint_index1, const ViewpointEntryIndex viewpoint_index2,
      const std::string& filename,
      const bool draw_lines = true,
      const FloatType sparse_point_size = FloatType(1.1), const FloatType match_line_width = FloatType(0.05)) const;

  void dumpSparseMatching(
      const ViewpointEntryIndex viewpoint_index1, const ViewpointEntryIndex viewpoint_index2,
      const bool draw_lines = true,
      const FloatType sparse_point_size = FloatType(1.1), const FloatType match_line_width = FloatType(0.05)) const;

  void dumpSparseMatching(
      const Viewpoint& viewpoint1, const Viewpoint& viewpoint2,
      const std::string& filename,
      const bool draw_lines = true,
      const FloatType sparse_point_size = FloatType(1.1), const FloatType match_line_width = FloatType(0.05)) const;

  std::unordered_set<Point3DId> computeVisibleSparsePoints(
      const Viewpoint& viewpoint,
      const SparseReconstruction::Point3DMapType::const_iterator first,
      const SparseReconstruction::Point3DMapType::const_iterator last) const;

  std::unordered_set<Point3DId> computeVisibleSparsePoints(const Viewpoint& viewpoint) const;

  // Return visible sparse points for a specific viewpoint entry (computes them if not already cached)
  const std::unordered_set<Point3DId>& getCachedVisibleSparsePoints(const ViewpointEntryIndex viewpoint_index) const;

  FloatType computeSparseMatchingScore(
      const Viewpoint& ref_viewpoint, const Viewpoint& other_viewpoint,
      const std::unordered_set<Point3DId>& ref_visible_points,
      const std::unordered_set<Point3DId>& other_visible_points) const;

  FloatType computeSparseMatchingScore(
      const Viewpoint& ref_viewpoint, const Viewpoint& other_viewpoint) const;

  FloatType computeSparseMatchingScore(
      const ViewpointEntryIndex viewpoint_index1, const ViewpointEntryIndex viewpoint_index2) const;

  bool isSparseMatchable(
      const Viewpoint& ref_viewpoint, const Viewpoint& other_viewpoint,
      const std::unordered_set<Point3DId>& ref_visible_points,
      const std::unordered_set<Point3DId>& other_visible_points) const;

  bool isSparseMatchable(
      const ViewpointEntryIndex viewpoint_index1, const ViewpointEntryIndex viewpoint_index2) const;

  bool isSparseMatchable(const Viewpoint& ref_viewpoint, const Viewpoint& other_viewpoint) const;

  void augmentedViewpointPathForSparseMatching(ViewpointPath* viewpoint_path);

  RegionType getRoi() const {
    return data_->roi_;
  }

  BoundingBoxType getRoiBbox() const {
    return data_->roi_bbox_;
  }

  BoundingBoxType getPositiveWeightBbox() const {
    BoundingBoxType positive_weight_bbox(
        data_->roi_bbox_.getMinimum().array() - data_->options_.roi_falloff_distance,
        data_->roi_bbox_.getMaximum().array() + data_->options_.roi_falloff_distance);
    return positive_weight_bbox;
  }

  BoundingBoxType getBvhBbox() const {
    return data_->bvh_bbox_;
  }

  BoundingBoxType getPoseSampleBbox() const {
    return pose_sample_bbox_;
  }

  const OccupancyMapType* getOctree() const {
      return data_->octree_.get();
  }

  const ViewpointPlannerData::OccupiedTreeType& getBvhTree() const {
    return data_->occupied_bvh_;
  }

  bool hasReconstruction() const {
      return static_cast<bool>(data_->reconstruction_);
  }

  const DenseReconstruction* getReconstruction() const {
      return data_->reconstruction_.get();
  }

  bool hasGpsTransformation() const {
    return hasReconstruction() && data_->reconstruction_->hasSfmGpsTransformation();
  }

  bool hasDensePoints() const {
      return static_cast<bool>(data_->dense_points_);
  }

  const PointCloudType* getDensePoints() const {
    return data_->dense_points_.get();
  }

  bool hasMesh() const {
      return static_cast<bool>(data_->poisson_mesh_);
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

  /// Return viewpoint paths for reading. Mutex needs to be locked.
  std::vector<ViewpointPath>& getViewpointPaths() {
    return viewpoint_paths_;
  }

  /// Return additional computation data for viewpoint paths. Mutex needs to be locked.
  const std::vector<ViewpointPathComputationData>& getViewpointPathsComputationData() const {
    return viewpoint_paths_data_;
  }

  /// Returns whether a motion between two viewpoints exists.
  bool hasViewpointMotion(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index) const;

  /// Return the motion between two viewpoints.
  ViewpointMotion getViewpointMotion(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index) const;

  /// Adds a viewpoint motion to the graph. Reverses the order depending on the index ordering (see getViewpointMotion).
//  void addViewpointMotion(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index, const Motion& motion);
//  void addViewpointMotion(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index, Motion&& motion);
  void addViewpointMotion(const ViewpointMotion& motion);
  void addViewpointMotion(ViewpointMotion&& motion);

  /// Return best viewpoint path for reading. Mutex needs to be locked.
  const ViewpointPath& getBestViewpointPath() const;

  /// Return best viewpoint path for reading. Mutex needs to be locked.
  ViewpointPath& getBestViewpointPath();

  /// Acquire lock to acces viewpoint entries (including graph and path) or reset the planning
  std::unique_lock<std::mutex> acquireLock();

#if WITH_OPENGL_OFFSCREEN

  /// Acquire lock for using any OpenGL context
  std::unique_lock<std::mutex> acquireOpenGLLock() const;

#endif

  // Pose and viewpoint sampling

  template <typename IteratorT>
  std::tuple<bool, ViewpointPlanner::Pose, size_t>
  sampleSurroundingPose(IteratorT first, IteratorT last) const;

  template <typename IteratorT>
  IteratorT sampleViewpointByGridCounts(IteratorT first, IteratorT last) const;

  void updateViewpointSamplingDistribution();

//  template <typename IteratorT>
//  std::pair<bool, Pose> sampleSurroundingPoseFromEntries(IteratorT first, IteratorT last) const;

  std::pair<bool, Pose> sampleSurroundingPose(const Pose& pose) const;

  std::pair<bool, Pose> samplePose(const size_t max_trials = (size_t)-1,
       const bool biased_orientation = true) const;

  std::pair<bool, Pose> samplePose(const BoundingBoxType& bbox,
      const Vector3& object_extent, size_t max_trials = (size_t)-1,
      const bool biased_orientation = true) const;

  std::pair<bool, Pose> samplePose(const RegionType& region,
      const Vector3& object_extent, size_t max_trials = (size_t)-1,
      const bool biased_orientation = true) const;

  std::pair<bool, ViewpointPlanner::Vector3> samplePosition(const size_t max_trials = (size_t)-1) const;

  std::pair<bool, ViewpointPlanner::Vector3> samplePosition(const BoundingBoxType& bbox,
      const Vector3& object_extent, size_t max_trials = (size_t)-1) const;

  std::pair<bool, ViewpointPlanner::Vector3> samplePosition(const RegionType& region,
      const Vector3& object_extent, size_t max_trials = (size_t)-1) const;

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

  /// Generate a new viewpoint entry and add it to the graph.
  bool generateNextViewpointEntry();

  /// Compute motions between viewpoints and update the graph with their cost.
  void computeViewpointMotions();

  /// Compute the novel observation information that a viewpoint will generate
  FloatType evaluateNovelViewpointInformation(
      const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data,
      const ViewpointEntryIndex viewpoint_index);

  /// Find the next best viewpoint to add to the viewpoint paths.
  bool findNextViewpointPathEntries(const FloatType alpha, const FloatType beta);

  /// Initialize data structures for finding viewpoint paths
  void initializeViewpointPathEntries(const FloatType alpha, const FloatType beta);

  /// Find initial viewpoints on viewpoint paths
  void findInitialViewpointPathEntries(const FloatType alpha, const FloatType beta);

  /// Find the next best viewpoint to add to the viewpoint paths and use parameters from options.
  bool findNextViewpointPathEntries();

  bool isValidViewpointPathEntry(
      const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data,
      const ViewpointEntryIndex viewpoint_index,
      const bool ignore_if_already_on_path = false) const;

  /// Compute new information score of a new viewpoint for a given viewpoint path
  FloatType computeNewInformation(const size_t viewpoint_path_index, const ViewpointEntryIndex new_viewpoint_index) const;

  /// Compute the connected components of the graph and return a pair of the component labels and the number of components.
  const std::pair<std::vector<size_t>, size_t>& getConnectedComponents() const;

  /// Compute a viewpoint tour for all paths
  void computeViewpointTour();

  /// Add a path entry to a viewpoint path
  void addViewpointPathEntry(const size_t viewpoint_path_index,
                             const ViewpointEntryIndex viewpoint_index,
                             const bool ignore_observed_voxels = false);

  /// Add a path entry to a viewpoint path
  void addViewpointPathEntry(const size_t viewpoint_path_index,
                             const Pose& pose,
                             const bool ignore_observed_voxels = false);

  /// Add a path entry to a viewpoint path
  bool addViewpointPathEntryWithStereoPair(const size_t viewpoint_path_index,
                                           const ViewpointEntryIndex viewpoint_index,
                                           const bool ignore_observed_voxels = false);

  /// Add a path entry together with a corresponding stereo viewpoint.
  /// If a stereo viewpoint can be found returns true. Otherwise returns false.
  bool addViewpointPathEntryWithStereoPair(const size_t viewpoint_path_index,
                                           const Pose& pose,
                                           const bool ignore_observed_voxels = false);

  // Raycasting and information computation

  /// Return viewpoint with virtual camera
  Viewpoint getVirtualViewpoint(const Pose& pose) const;

  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels with additional info.
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> getRaycastHitVoxels(
      const Viewpoint& viewpoint) const;

  /// Perform raycast on the BVH tree on a limited window around the center.
  /// Returns a vector of hit voxels with additional info.
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> getRaycastHitVoxels(
      const Viewpoint& viewpoint, const size_t width, const size_t height) const;

  /// Perform raycast on the BVH tree on a limited window around the center.
  /// Returns a vector of hit voxels with additional info.
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult>
  getRaycastHitVoxels(
      const Viewpoint& viewpoint,
      const size_t x_start, const size_t x_end,
      const size_t y_start, const size_t y_end) const;

  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels with additional info.
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates>
  getRaycastHitVoxelsWithScreenCoordinates(
      const Viewpoint& viewpoint) const;

  /// Perform raycast on the BVH tree on a limited window around the center.
  /// Returns a vector of hit voxels with additional info.
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates>
  getRaycastHitVoxelsWithScreenCoordinates(
      const Viewpoint& viewpoint, const size_t width, const size_t height) const;

  /// Perform raycast on the BVH tree on a limited window around the center.
  /// Returns a vector of hit voxels with additional info.
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates>
  getRaycastHitVoxelsWithScreenCoordinates(
      const Viewpoint& viewpoint,
      const size_t x_start, const size_t x_end,
      const size_t y_start, const size_t y_end) const;

#if WITH_CUDA

  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels with additional info.
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> getRaycastHitVoxelsCuda(
      const Viewpoint& viewpoint) const;

  /// Perform raycast on the BVH tree on a limited window around the center.
  /// Returns a vector of hit voxels with additional info.
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult> getRaycastHitVoxelsCuda(
      const Viewpoint& viewpoint, const size_t width, const size_t height) const;

  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels with additional info.
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult>
  getRaycastHitVoxelsCuda(
      const Viewpoint& viewpoint,
      const size_t x_start, const size_t x_end,
      const size_t y_start, const size_t y_end) const;

  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels with additional info.
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates>
  getRaycastHitVoxelsWithScreenCoordinatesCuda(
      const Viewpoint& viewpoint) const;

  /// Perform raycast on the BVH tree on a limited window around the center.
  /// Returns a vector of hit voxels with additional info.
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates>
  getRaycastHitVoxelsWithScreenCoordinatesCuda(
      const Viewpoint& viewpoint, const size_t width, const size_t height) const;

  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels with additional info.
  std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates>
  getRaycastHitVoxelsWithScreenCoordinatesCuda(
      const Viewpoint& viewpoint,
      const size_t x_start, const size_t x_end,
      const size_t y_start, const size_t y_end) const;

#endif

  /// Perform raycast on the BVH tree.
  /// Returns the set of hit voxels with corresponding information + the total information of all voxels.
  std::pair<VoxelWithInformationSet, FloatType>
    getRaycastHitVoxelsWithInformationScore(
            const Viewpoint& viewpoint,
            const bool ignore_voxels_with_zero_information = false) const;

  /// Perform raycast on the BVH tree on a centered window with specific size.
  /// Returns the set of hit voxels with corresponding information + the total information of all voxels.
  std::pair<VoxelWithInformationSet, FloatType>
    getRaycastHitVoxelsWithInformationScore(
            const Viewpoint& viewpoint, const size_t width, const size_t height,
            const bool ignore_voxels_with_zero_information = false) const;

  /// Perform raycast on the BVH tree.
  /// Returns the set of hit voxels with corresponding information + the total information of all voxels.
  std::pair<VoxelWithInformationSet, FloatType>
    getRaycastHitVoxelsWithInformationScore(
        const Viewpoint& viewpoint,
        const size_t x_start, const size_t x_end,
        const size_t y_start, const size_t y_end,
        const bool ignore_voxels_with_zero_information = false) const;

  /// Returns the observation factor in [0, 1] for a single voxel.
  FloatType computeViewpointObservationFactor(const Viewpoint& viewpoint, const VoxelType* node) const;

  /// Returns the observation factor in [0, 1] for a single voxel.
  FloatType computeViewpointObservationFactor(
      const Viewpoint& viewpoint, const VoxelType* node, const Vector2& screen_coordinates) const;

  /// Returns the observation score for a voxel.
  FloatType computeViewpointObservationScore(const Viewpoint& viewpoint, const VoxelType* node) const;

  /// Returns the observation score for a voxel.
  FloatType computeViewpointObservationScore(
      const Viewpoint& viewpoint, const VoxelType* node, const Vector2& screen_coordinates) const;

  /// Returns the observation score for a single hit voxel result.
  FloatType computeViewpointObservationScore(const Viewpoint& viewpoint,
      const ViewpointPlannerData::OccupiedTreeType::IntersectionResult& result) const;

  /// Returns the observation score for a single hit voxel result.
  FloatType computeViewpointObservationScore(const Viewpoint& viewpoint,
      const ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates& result) const;

  /// Returns the observation score for a container of VoxelWithInformation objects.
  template <typename Iterator>
  FloatType computeInformationScore(const Viewpoint& viewpoint, Iterator first, Iterator last) const;

  /// Compute information weighted center position of observed voxels
  Vector3 computeInformationVoxelCenter(const ViewpointEntry& viewpoint_entry) const;

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

#if WITH_OPENGL_OFFSCREEN

  QImage drawPoissonMesh(const ViewpointEntryIndex viewpoint_index) const;
  QImage drawPoissonMesh(const Viewpoint& viewpoint) const;
  QImage drawPoissonMesh(const Pose& pose) const;
  QImage drawPoissonMesh(const QMatrix4x4& pvm_matrix) const;
  QImage drawPoissonMeshNormals(const ViewpointEntryIndex viewpoint_index) const;
  QImage drawPoissonMeshNormals(const Viewpoint& viewpoint) const;
  QImage drawPoissonMeshNormals(const Pose& pose) const;
  QImage drawPoissonMeshNormals(const QMatrix4x4& pvm_matrix) const;
  QImage drawPoissonMeshDepth(const ViewpointEntryIndex viewpoint_index) const;
  QImage drawPoissonMeshDepth(const Viewpoint& viewpoint) const;
  QImage drawPoissonMeshDepth(const Pose& pose) const;
  QImage drawPoissonMeshDepth(const QMatrix4x4& pvm_matrix, const QMatrix4x4& vm_matrix) const;

  Vector3 computePoissonMeshNormalVector(const Viewpoint& viewpoint, const Vector3& position) const;

  Vector3 computePoissonMeshNormalVector(const Viewpoint& viewpoint,
                                         const size_t x, const size_t y) const;

  FloatType computePoissonMeshDepth(const Viewpoint& viewpoint, const Vector3& position) const;

  FloatType computePoissonMeshDepth(const Viewpoint& viewpoint,
                                         const size_t x, const size_t y) const;

  ait::Color4<uint8_t> encodeDepthValue(const FloatType depth) const;

  FloatType decodeDepthValue(const ait::Color4<uint8_t>& color) const;

  FloatType decodeDepthValue(const QColor& color) const;

  QImage convertEncodedDepthImageToRGB(const QImage& encoded_image, const FloatType min_depth, const FloatType max_depth) const;

#endif

private:
  /// Remove duplicate hit voxels from raycast results
  void removeDuplicateRaycastHitVoxels(
      std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResult>* raycast_results) const;

  /// Remove duplicate hit voxels from raycast results
  void removeDuplicateRaycastHitVoxels(
      std::vector<ViewpointPlannerData::OccupiedTreeType::IntersectionResultWithScreenCoordinates>* raycast_results) const;

  /// Remove points that are occluded from the given pose.
  /// A point needs to be dist_margin behind a voxel surface to be removed.
  void removeOccludedPoints(const Pose& pose, std::unordered_set<Point3DId>& point3D_ids, FloatType dist_margin) const;

  /// Compute information factor based on observation count (i.e. exp(-theta * observation_count)
  WeightType computeObservationInformationFactor(CounterType observation_count) const;

  /// Compute resolution information factor based on distance to voxel
  WeightType computeResolutionInformationFactor(const Viewpoint& viewpoint, const VoxelType* node) const;

  /// Compute incidence information factor based on incidence angle
  WeightType computeIncidenceInformationFactor(
      const Viewpoint& viewpoint, const VoxelType* node, const Vector3& normal_vector) const;

  /// Compute incidence information factor based on incidence angle
  WeightType computeIncidenceInformationFactor(const Viewpoint& viewpoint, const VoxelType* node) const;

  /// Compute incidence information factor based on incidence angle.
  /// Screen coordinates are provided to retrieve normal vector.
  WeightType computeIncidenceInformationFactor(
      const Viewpoint& viewpoint, const VoxelType* node, const Vector2& screen_coordinates) const;

  /// Add a viewpoint entry to the graph.
  ViewpointEntryIndex addViewpointEntry(ViewpointEntry&& viewpoint_entry);

  /// Add a viewpoint entry without acquiring a lock. Returns the index of the new viewpoint entry.
  ViewpointEntryIndex addViewpointEntryWithoutLock(ViewpointEntry&& viewpoint_entry);

  /// Add a path entry to a viewpoint path
  void addViewpointPathEntryWithoutLock(ViewpointPath *viewpoint_path, ViewpointPathComputationData *comp_data,
                                        const ViewpointPathEntry &new_path_entry,
                                        const bool ignore_observed_voxels = false);

  /// Add stereo viewpoint to a viewpoint path
  void addStereoViewpointPathEntryWithoutLock(ViewpointPath *viewpoint_path, ViewpointPathComputationData *comp_data,
                                              const ViewpointPathEntry &first_path_entry,
                                              const ViewpointPathEntry &second_path_entry,
                                              const bool add_second_entry = true);

  /// Find a matching viewpoint for stereo matching. If no suitable viewpoint with enough overlap is already in the graph
  /// a new viewpoint is added. Returns a pair indicating whether the stereo viewpoint is already on the path (true) or not (false)
  /// and the index of the corresponding viewpoint entry. Must be called with lock.
  std::pair<bool, ViewpointEntryIndex> findMatchingStereoViewpointWithoutLock(
      const ViewpointPath& viewpoint_path,
      const ViewpointPathComputationData& comp_data,
      const ViewpointEntryIndex& viewpoint_index,
      const bool ignore_sparse_matching = false,
      const bool ignore_graph_component = false);

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
  void updateViewpointPathInformations(ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data);

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
      const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data, const size_t max_num_viewpoints) const;

  /// Report info of current viewpoint paths
  void reportViewpointPathsStats() const;

  /// Compute approx. shortest cycle to cover all viewpoints in the path (i.e. solve TSP problem)
  std::vector<size_t> computeApproximateShortestCycle(const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data);

  /// Reorders the viewpoint path to an approx. shortest cycle covering all viewpoints (i.e. TSP solution)
  void solveApproximateTSP(ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data);

  /// Ensures that all viewpoints on a path are connected by searching for shortest motion paths if necessary
  void ensureConnectedViewpointPath(
      ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data, const bool recompute_all = false);

  /// Uses 2 Opt to improve the viewpoint tour
  void improveViewpointTourWith2Opt(ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data);

  /// Find shortest motion between two viewpoints using A-Star on the viewpoint graph.
  ViewpointMotion findShortestMotionAStar(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index);

  /// Optimize viewpoint motion by reducing redundant in-between viewpoints
  ViewpointMotion optimizeViewpointMotion(const ViewpointMotion& motion) const;

  /// Find shortest motion between two viewpoints and add resulting motions to the graph. Returns true if a motion was found.
  bool findAndAddShortestMotion(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index);

  /// Find shortest motion between a new viewpoint and existing viewpoints using A-Star on the viewpoint graph. Returns true if successful.
  template <typename Iterator>
  bool findAndAddShortestMotions(const ViewpointEntryIndex from_index, Iterator to_index_first, Iterator to_index_last);

  /// Computes the length of a viewpoint tour (cycle)
  FloatType computeTourLength(const ViewpointPath& viewpoint_path, const std::vector<size_t>& order) const;

  /// Returns a new tour order where the path from k_start to k_end is reversed
  std::vector<size_t> swap2Opt(
      ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data,
      const size_t k_start, const size_t k_end) const;

  /// Create a subgraph with the nodes from a viewpoint path
  ViewpointPathGraphWrapper createViewpointPathGraph(const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data);

#if WITH_OPENGL_OFFSCREEN

  /// Initialize OpenGL offscreen surface for rendering mesh normals
  void initializeOpenGL() const;

  void clearOpenGL() const;

  /// Upload poisson mesh to GPU for rendering
  void uploadPoissonMesh() const;

  void bindOpenGLFbo() const;
  void releaseOpenGLFbo() const;

  void beginOpenGLDrawing() const;
  void finishOpenGLDrawing() const;

  QMatrix4x4 getPvmMatrixFromViewpoint(const Viewpoint& viewpoint) const;
  QMatrix4x4 getPvmMatrixFromPose(const Pose& pose) const;
  QMatrix4x4 getVmMatrixFromViewpoint(const Viewpoint& viewpoint) const;
  QMatrix4x4 getVmMatrixFromPose(const Pose& pose) const;

  mutable Pose cached_poisson_mesh_normals_pose_;
  mutable QImage cached_poisson_mesh_normals_image_;

  mutable Pose cached_poisson_mesh_depth_pose_;
  mutable QImage cached_poisson_mesh_depth_image_;

#endif

  mutable ait::Random<FloatType, std::int64_t> random_;

  Options options_;

  BoundingBoxType pose_sample_bbox_;
  Vector3 drone_extent_;
  FloatType triangulation_max_cos_angle_;
  FloatType triangulation_min_sin_angle_square_;
  FloatType triangulation_max_sin_angle_square_;
  FloatType triangulation_max_angular_deviation_;
  FloatType incidence_angle_threshold_;
  FloatType incidence_angle_falloff_factor_;
  FloatType voxel_sensor_size_ratio_falloff_factor_;

  std::unique_ptr<ViewpointPlannerData> data_;
  PinholeCamera virtual_camera_;

  std::mutex mutex_;

  MotionPlannerType motion_planner_;

  // Counter of failed viewpoint entry samples. Reset to 0 after each successfull sample.
  size_t num_of_failed_viewpoint_entry_samples_;
  // All tentative viewpoints
  ViewpointEntryVector viewpoint_entries_;
  // Cached visible sparse points
  mutable std::unordered_map<ViewpointEntryIndex, std::unordered_set<reconstruction::Point3DId>> cached_visible_sparse_points_;
  // Mutex for cached visible sparse points
  mutable std::mutex cached_visible_sparse_points_mutex_;
  // Number of real viewpoints at the beginning of the viewpoint_entries_ vector
  // These need to be distinguished because they could be in non-free space of the map
  size_t num_real_viewpoints_;
  // Approximate nearest neighbor index for viewpoints
  ViewpointANN viewpoint_ann_;
  // Graph of tentative viewpoints with motion distances
  ViewpointGraph viewpoint_graph_;
  // Grid of viewpoint counts in the sampling space
  ContinuousGridType viewpoint_count_grid_;
  // Mapping from viewpoint to grid indices
  std::vector<Eigen::Vector3s> viewpoint_grid_indices_;
  // Probability for sampling a new viewpoint in a grid cell
  std::vector<FloatType> grid_cell_probabilities_;
  // Discrete distribution to sample from viewpoints
  mutable std::discrete_distribution<size_t> viewpoint_sampling_distribution_;
  // Size of viewpoint graph when distribution was last updated
  size_t viewpoint_sampling_distribution_update_size_;
  // Connected components of viewpoint graph
  mutable std::pair<std::vector<size_t>, size_t> viewpoint_graph_components_;
  // Flag indicating whether connected components are valid
  mutable bool viewpoint_graph_components_valid_;
  // Motion description of the connections in the viewpoint graph (indexed by viewpoint id pair)
  std::unordered_map<ViewpointIndexPair, ViewpointMotion, ViewpointIndexPair::Hash> viewpoint_graph_motions_;
  /// Flag indicating whether viewpoint paths have been initialized
  bool viewpoint_paths_initialized_;
  // Current viewpoint paths
  std::vector<ViewpointPath> viewpoint_paths_;
  // Data used for computation of viewpoint paths
  std::vector<ViewpointPathComputationData> viewpoint_paths_data_;
  // Map from triangle index to viewpoints that project features into it
  FeatureViewpointMap feature_viewpoint_map_;

#if WITH_OPENGL_OFFSCREEN
  mutable std::mutex opengl_mutex_;
  mutable std::mutex poisson_mesh_cache_mutex_;
  mutable QOffscreenSurface* opengl_surface_;
  mutable QOpenGLContext* opengl_context_;
  mutable QOpenGLFramebufferObject* opengl_fbo_;
  mutable TriangleDrawer* poisson_mesh_drawer_;
#endif
};

BOOST_CLASS_VERSION(ViewpointPlanner::ViewpointPathEntry, 2)

#include "viewpoint_planner.hxx"

#include "viewpoint_planner_graph.hxx"
