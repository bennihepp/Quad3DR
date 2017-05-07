// viewpoint_planner.h
//==================================================
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 6, 2016
//==================================================
#pragma once

#include <bh/eigen.h>
#include <bh/eigen_serialization.h>
#include <memory>
#include <unordered_set>
#include <boost/functional/hash.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <bh/boost_serialization_utils.h>
#include <rapidjson/document.h>
#include <bh/common.h>
#include <bh/config_options.h>
#include <bh/eigen_options.h>
#include <bh/random.h>
#include <bh/eigen_utils.h>
#include <bh/graph_boost.h>
#include <bh/math/continuous_grid3d.h>
#include <bh/nn/approximate_nearest_neighbor.h>
#include <bh/opengl/offscreen_opengl.h>
#include "../rendering/octree_drawer.h"
#include "../mLib/mLib.h"
#include "../octree/occupancy_map.h"
#include "../reconstruction/dense_reconstruction.h"
#include "viewpoint.h"
#include "viewpoint_planner_data.h"
#include "viewpoint_planner_types.h"
#include "viewpoint_raycast.h"
#include "viewpoint_score.h"
#include "viewpoint_offscreen_renderer.h"
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
  using Pose = bh::Pose<FloatType>;

  static constexpr FloatType kDotProdEqualTolerance = FloatType { 1e-5 };
  static constexpr FloatType kOcclusionDistMarginFactor = FloatType { 4 };

  struct Options : bh::ConfigOptions {
    Options()
    : bh::ConfigOptions("viewpoint_planner", "ViewpointPlanner options") {
#if WITH_CUDA
      addOption<bool>("enable_cuda", &enable_cuda);
#endif
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
      addOption<FloatType>("drone_velocity", &drone_velocity);
      addOption<FloatType>("viewpoint_recording_time", &viewpoint_recording_time);
      addOption<Vector3>("drone_bbox_min", &drone_bbox_min);
      addOption<Vector3>("drone_bbox_max", &drone_bbox_max);
      addOption<Vector3>("drone_start_position", &drone_start_position);
      addOptionalOption<Vector3>("sampling_bbox_min");
      addOptionalOption<Vector3>("sampling_bbox_max");
      addOption<FloatType>("sampling_roi_factor", &sampling_roi_factor);
      addOption<FloatType>("pose_sample_min_radius", &pose_sample_min_radius);
      addOption<FloatType>("pose_sample_max_radius", &pose_sample_max_radius);
      addOption<size_t>("pose_sample_num_trials", &pose_sample_num_trials);
      addOption<bool>("viewpoint_no_raycast", &viewpoint_no_raycast);
      addOptionalOption<Vector3>("exploration_bbox_min");
      addOptionalOption<Vector3>("exploration_bbox_max");
      addOption<FloatType>("viewpoint_exploration_step", &viewpoint_exploration_step);
      addOption<FloatType>("viewpoint_exploration_step_max", &viewpoint_exploration_step_max);
      addOption<bool>("viewpoint_exploration_dilation_squared", &viewpoint_exploration_dilation_squared);
      addOption<FloatType>("viewpoint_exploration_dilation_start_bbox_distance", &viewpoint_exploration_dilation_start_bbox_distance);
      addOption<FloatType>("viewpoint_exploration_dilation_speed", &viewpoint_exploration_dilation_speed);
      addOption<FloatType>("viewpoint_exploration_angular_dist_threshold_degrees", &viewpoint_exploration_angular_dist_threshold_degrees);
      addOption<size_t>("viewpoint_exploration_num_orientations", &viewpoint_exploration_num_orientations);
      addOption<FloatType>("viewpoint_sample_without_reference_probability", &viewpoint_sample_without_reference_probability);
      addOption<size_t>("viewpoint_min_voxel_count", &viewpoint_min_voxel_count);
      addOption<FloatType>("viewpoint_voxel_distance_threshold", &viewpoint_voxel_distance_threshold);
      addOption<FloatType>("viewpoint_max_too_close_voxel_ratio", &viewpoint_max_too_close_voxel_ratio);
      addOption<FloatType>("viewpoint_min_information", &viewpoint_min_information);
      addOption<size_t>("viewpoint_sample_motion_and_matching_knn", &viewpoint_sample_motion_and_matching_knn);
      addOption<FloatType>("viewpoint_information_factor", &viewpoint_information_factor);
      addOption<bool>("viewpoint_generate_stereo_pairs", &viewpoint_generate_stereo_pairs);
      addOption<size_t>("viewpoint_stereo_num_samples", &viewpoint_stereo_num_samples);
      addOption<size_t>("viewpoint_stereo_max_num_raycast", &viewpoint_stereo_max_num_raycast);
      addOption<FloatType>("triangulation_min_angle_degrees", &triangulation_min_angle_degrees);
      addOption<FloatType>("triangulation_max_angle_degrees", &triangulation_max_angle_degrees);
      addOption<FloatType>("triangulation_max_dist_deviation_ratio", &triangulation_max_dist_deviation_ratio);
      addOption<FloatType>("triangulation_max_angular_deviation_degrees", &triangulation_max_angular_deviation_degrees);
      addOption<FloatType>("triangulation_min_voxel_overlap_ratio", &triangulation_min_voxel_overlap_ratio);
      addOption<FloatType>("triangulation_min_information_overlap_ratio", &triangulation_min_information_overlap_ratio);
      addOption<size_t>("triangulation_knn", &triangulation_knn);
      addOption<size_t>("triangulation_min_count", &triangulation_min_count);
      addOption<bool>("viewpoint_count_grid_enable", &viewpoint_count_grid_enable);
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
      addOption<bool>("viewpoint_path_compute_connections_incremental", &viewpoint_path_compute_connections_incremental);
      addOption<bool>("viewpoint_path_compute_tour_incremental", &viewpoint_path_compute_tour_incremental);
      addOption<bool>("viewpoint_path_conservative_sparse_matching_incremental", &viewpoint_path_conservative_sparse_matching_incremental);
      addOption<FloatType>("viewpoint_path_time_constraint", &viewpoint_path_time_constraint);
      addOption<FloatType>("objective_parameter_alpha", &objective_parameter_alpha);
      addOption<FloatType>("objective_parameter_beta", &objective_parameter_beta);
      addOption<FloatType>("voxel_sensor_size_ratio_threshold", &voxel_sensor_size_ratio_threshold);
      addOption<FloatType>("voxel_sensor_size_ratio_inv_falloff_factor", &voxel_sensor_size_ratio_inv_falloff_factor);
      addOption<bool>("incidence_ignore_dot_product_sign", &incidence_ignore_dot_product_sign);
      addOption<FloatType>("incidence_angle_threshold_degrees", &incidence_angle_threshold_degrees);
      addOption<FloatType>("incidence_angle_inv_falloff_factor_degrees", &incidence_angle_inv_falloff_factor_degrees);
      addOption<FloatType>("sparse_matching_depth_tolerance", &sparse_matching_depth_tolerance);
      addOption<FloatType>("sparse_matching_dot_product_threshold", &sparse_matching_dot_product_threshold);
      addOption<FloatType>("sparse_matching_normal_dot_product_threshold", &sparse_matching_normal_dot_product_threshold);
      addOption<FloatType>("sparse_matching_score_threshold", &sparse_matching_score_threshold);
      addOption<FloatType>("sparse_matching_max_distance_deviation_factor", &sparse_matching_max_distance_deviation_factor);
      addOption<FloatType>("sparse_matching_max_normal_deviation_factor", &sparse_matching_max_normal_deviation_factor);
      addOption<FloatType>("sparse_matching_min_distance", &sparse_matching_min_distance);
      addOption<FloatType>("sparse_matching_min_angular_distance", &sparse_matching_min_angular_distance);
      addOption<FloatType>("sparse_matching_max_distance", &sparse_matching_max_distance);
      addOption<FloatType>("sparse_matching_max_angular_distance_degrees", &sparse_matching_max_angular_distance_degrees);
      addOption<FloatType>("sparse_matching_shared_sector_ratio", &sparse_matching_shared_sector_ratio);
      addOption<size_t>("sparse_matching_num_of_shared_sectors", &sparse_matching_num_of_shared_sectors);
      addOption<FloatType>("sparse_matching_voxels_iou_threshold", &sparse_matching_voxels_iou_threshold);
      addOption<FloatType>("sparse_matching_voxels_conservative_iou_threshold", &sparse_matching_voxels_conservative_iou_threshold);
      addOption<FloatType>("sparse_matching_virtual_camera_factor", &sparse_matching_virtual_camera_factor);
      addOption<FloatType>("sparse_matching_occupancy_threshold", &sparse_matching_occupancy_threshold);
      addOption<size_t>("sparse_matching_observation_count_threshold", &sparse_matching_observation_count_threshold);
      addOption<size_t>("sparse_matching_render_tree_depth", &sparse_matching_render_tree_depth);
      addOption<bool>("sparse_matching_dump_voxel_images", &sparse_matching_dump_voxel_images);
      addOption<bool>("viewpoint_path_2opt_enable", &viewpoint_path_2opt_enable);
      addOption<size_t>("viewpoint_path_2opt_max_k_length", &viewpoint_path_2opt_max_k_length);
      addOption<bool>("viewpoint_path_2opt_check_sparse_matching", &viewpoint_path_2opt_check_sparse_matching);
      addOption<std::string>("viewpoint_graph_filename", &viewpoint_graph_filename);
      // TODO:
      addOption<size_t>("num_sampled_poses", &num_sampled_poses);
      addOption<size_t>("num_planned_viewpoints", &num_planned_viewpoints);
      addOption<std::string>("motion_planner_log_filename", &motion_planner_log_filename);
    }

    ~Options() override {}

#if WITH_CUDA
    // Whether to enable CUDA
    bool enable_cuda = true;
#endif
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

    // Average drone velocity
    FloatType drone_velocity = FloatType(2.0);
    // Time to record a single viewpoint
    FloatType viewpoint_recording_time = FloatType(10.0);

    // Drone bounding box
    Vector3 drone_bbox_min = Vector3(-10, -10, -10);
    Vector3 drone_bbox_max = Vector3(10, 10, 10);

    // Viewpoint sampling parameters

    // Start position of drone. Viewpoint sampling starts from here to ensure connectivity.
    Vector3 drone_start_position = Vector3(0, 0, 0);

    // Region of interest dilation for viewpoint sampling
    FloatType sampling_roi_factor = 2;
    // Spherical shell for viewpoint sampling
    FloatType pose_sample_min_radius = 2;
    FloatType pose_sample_max_radius = 5;

    // TODO
    bool viewpoint_no_raycast = false;
    FloatType viewpoint_exploration_step = 3;
    FloatType viewpoint_exploration_step_max = std::numeric_limits<FloatType>::max();
    bool viewpoint_exploration_dilation_squared = true;
    FloatType viewpoint_exploration_dilation_start_bbox_distance = 0;
    FloatType viewpoint_exploration_dilation_speed = 2;
    FloatType viewpoint_exploration_angular_dist_threshold_degrees = 30;
    size_t viewpoint_exploration_num_orientations = 3;

    // Number of trials for viewpoint position sampling
    size_t pose_sample_num_trials = 100;
    // Probability of sampling a new viewpoint independent of a reference viewpoint
    FloatType viewpoint_sample_without_reference_probability = 0.1f;
    // Ensure that every viewpoint is part of a stereo pair
    bool viewpoint_generate_stereo_pairs;
    // Maximum number of pose samples when searching for stereo pair
    size_t viewpoint_stereo_num_samples = 20;
    // Maximum number of raycast operations when searching for stereo pair
    size_t viewpoint_stereo_max_num_raycast = 5;
    // Minimum voxel count for viewpoints
    size_t viewpoint_min_voxel_count = 100;
    // Distance threshold to voxels (also see 'viewpoint_max_too_close_voxel_ratio')
    FloatType viewpoint_voxel_distance_threshold = FloatType(15.0);
    // Maximum ratio of voxels closer than 'viewpoint_voxel_distance_threshold'
    FloatType viewpoint_max_too_close_voxel_ratio = FloatType(0.5);
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

    // Whether to enable viewpoint count grid for sampling
    bool viewpoint_count_grid_enable = false;
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
    // Whether to compute new connections whenever adding a viewpoint path entry
    bool viewpoint_path_compute_connections_incremental = false;
    // Whether to compute a viewpoint path tour whenever adding a viewpoint path entry
    bool viewpoint_path_compute_tour_incremental = false;
    // Whether to augment a viewpoint path tour for sparse matching whenever adding a viewpoint path entry
    bool viewpoint_path_conservative_sparse_matching_incremental = false;
    // Maximum time constraint for viewpoint path
    FloatType viewpoint_path_time_constraint = std::numeric_limits<FloatType>::max();

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
    // Minimum dot product between viewing ray and surface normal for a point to be matchable
    FloatType sparse_matching_normal_dot_product_threshold = FloatType(0.6);
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
    // Distance threshold above which viewpoints are assumed to not be matchable
    FloatType sparse_matching_max_distance = 4;
    // Angular distance threshold above which viewpoints are assumed to not be matchable
    FloatType sparse_matching_max_angular_distance_degrees = 20;

    // Minimum number of common points in a sector to be considered shared
    FloatType sparse_matching_shared_sector_ratio = FloatType(0.1);
    // Minimum number of sectors with shared visible sparse points
    size_t sparse_matching_num_of_shared_sectors = 3;

    // Minimum IOU of visible triangles in two views to be matchable
    FloatType sparse_matching_voxels_iou_threshold = 0.2;
    FloatType sparse_matching_voxels_conservative_iou_threshold = 0.4;
    FloatType sparse_matching_virtual_camera_factor = 2;
    FloatType sparse_matching_occupancy_threshold = 0.6;
    size_t sparse_matching_observation_count_threshold = 0;
    size_t sparse_matching_render_tree_depth = 14;
    bool sparse_matching_dump_voxel_images = false;

    // Whether to enable 2 Opt
    bool viewpoint_path_2opt_enable = true;
    // Maximum segment length that is reversed by 2 Opt
    size_t viewpoint_path_2opt_max_k_length = 15;
    // Whether sparse matchability is checked for 2 Opt
    bool viewpoint_path_2opt_check_sparse_matching = true;

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
  using OccupiedTreeType = viewpoint_planner::OccupiedTreeType;
  using VoxelType = viewpoint_planner::OccupiedTreeType::NodeType;

  using MotionPlannerType = MotionPlanner<FloatType>;
  using SE3Motion = typename MotionPlannerType::Motion;

  using VoxelWrapper = viewpoint_planner::VoxelWrapper;
  using VoxelWithInformation = viewpoint_planner::VoxelWithInformation;
  using VoxelWithInformationSet = viewpoint_planner::VoxelWithInformationSet;
  using VoxelMap = viewpoint_planner::VoxelMap;

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
      BH_ASSERT(this->index1 <= this->index2);
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
    using SE3MotionVector = EIGEN_ALIGNED_VECTOR(SE3Motion);

    explicit ViewpointMotion() {}

    explicit ViewpointMotion(const std::vector<ViewpointEntryIndex>& viewpoint_indices, const SE3MotionVector& se3_motions)
    : viewpoint_indices_(viewpoint_indices), se3_motions_(se3_motions) {
#if !BH_RELEASE
      BH_ASSERT(viewpoint_indices_.size() >= 2);
      BH_ASSERT(se3_motions_.size() == viewpoint_indices_.size() - 1);
#endif
    }

    explicit ViewpointMotion(std::vector<ViewpointEntryIndex>&& viewpoint_indices, SE3MotionVector&& se3_motions)
    : viewpoint_indices_(std::move(viewpoint_indices)), se3_motions_(std::move(se3_motions)) {
#if !BH_RELEASE
      BH_ASSERT(viewpoint_indices_.size() >= 2);
      BH_ASSERT(se3_motions_.size() == viewpoint_indices_.size() - 1);
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

    const SE3MotionVector& se3Motions() const {
      return se3_motions_;
    }

    FloatType distance() const {
      const FloatType distance = std::accumulate(
          se3_motions_.begin(), se3_motions_.end(), FloatType(0),
          [] (const FloatType value, const SE3Motion& se3_motion) -> FloatType {
        // Sanity check
//#if !BH_RELEASE
//        BH_ASSERT(se3_motion.poses.size() >= 2);
//        FloatType local_dist = 0;
//        for (auto pose_it = se3_motion.poses.begin() + 1; pose_it != se3_motion.poses.end(); ++pose_it) {
//          const auto prev_pose_it = pose_it - 1;
//          local_dist += (pose_it->getWorldPosition() - prev_pose_it->getWorldPosition()).norm();
//        }
//        BH_ASSERT(bh::isApproxEqual(se3_motion.distance, local_dist, FloatType(1e-2)));
//#endif
        return value + se3_motion.distance();
      });
      return distance;
    }

    FloatType se3Distance() const {
      const FloatType se3_distance = std::accumulate(
              se3_motions_.begin(), se3_motions_.end(), FloatType(0),
              [] (const FloatType value, const SE3Motion& se3_motion) -> FloatType {
                return value + se3_motion.se3Distance();
              });
      return se3_distance;
    }

    FloatType cost() const {
      const FloatType cost = std::accumulate(
              se3_motions_.begin(), se3_motions_.end(), FloatType(0),
              [] (const FloatType value, const SE3Motion& se3_motion) -> FloatType {
                return value + se3_motion.cost();
              });
      return cost;
    }

    void append(const ViewpointMotion& other) {
      if (isValid()) {
        BH_ASSERT(toIndex() == other.fromIndex());
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
#if !BH_RELEASE
      const FloatType dist_before = distance();
#endif
      std::reverse(viewpoint_indices_.begin(), viewpoint_indices_.end());
      std::reverse(se3_motions_.begin(), se3_motions_.end());
      for (SE3Motion& se3_motion : se3_motions_) {
        se3_motion.reverse();
      }
#if !BH_RELEASE
      const FloatType dist_after = distance();
      BH_ASSERT(bh::isApproxEqual(dist_before, dist_after, FloatType(1e-2)));
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
    SE3MotionVector se3_motions_;
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

  using ViewpointGraph = bh::Graph<ViewpointEntryIndex, FloatType>;
  using ViewpointANN = bh::ApproximateNearestNeighbor<FloatType, 3>;
  using FeatureViewpointMap = std::unordered_map<size_t, std::vector<const Viewpoint*>>;


  ViewpointPlanner(const Options* options,
      const MotionPlannerType::Options* motion_options, std::unique_ptr<ViewpointPlannerData> data);

  const Options& getOptions() const;

  const ViewpointPlannerData& getPlannerData() const;

  ViewpointPlannerData& getPlannerData();

  void ensureOctreeDrawerIsInitialized() const;

  const viewpoint_planner::ViewpointOffscreenRenderer& getOffscreenRenderer() const;

  viewpoint_planner::ViewpointOffscreenRenderer& getOffscreenRenderer();

  const bh::opengl::OffscreenOpenGL<FloatType>& getOffscreenOpenGL() const;

  bh::opengl::OffscreenOpenGL<FloatType>& getOffscreenOpenGL();

  const rendering::OcTreeDrawer& getOctreeDrawer() const;

  rendering::OcTreeDrawer& getOctreeDrawer();

  /// Get virtual camera
  const PinholeCamera& getVirtualCamera() const;

  const BoundingBoxType& getDroneBoundingBox() const;

  Vector3 getDroneExtent() const;

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

  FloatType getViewpointPathTimeConstraint() const;

  void setViewpointPathTimeConstraint(const FloatType time_constraint);

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

//  bool isSparsePointMatchable(const Viewpoint& viewpoint1, const Viewpoint& viewpoint2, const Point3D& point3d) const;

  bool isSparsePointMatchable(const Viewpoint& viewpoint1, const Viewpoint& viewpoint2,
                              const Point3D& point3d, const Vector3& normal1, const Vector3& normal2) const;

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

  std::unordered_map<Point3DId, ViewpointPlanner::Vector3> computeVisibleSparsePoints(
      const Viewpoint& viewpoint,
      const SparseReconstruction::Point3DMapType::const_iterator first,
      const SparseReconstruction::Point3DMapType::const_iterator last) const;

  std::unordered_map<Point3DId, ViewpointPlanner::Vector3> computeVisibleSparsePoints(const Viewpoint& viewpoint) const;

  // Return visible sparse points for a specific viewpoint entry (computes them if not already cached)
  const std::unordered_map<Point3DId, ViewpointPlanner::Vector3>& getCachedVisibleSparsePoints(const ViewpointEntryIndex viewpoint_index) const;

  const std::unordered_set<size_t>& getCachedVisibleVoxels(const ViewpointEntryIndex viewpoint_index) const;

//  FloatType computeSparseMatchingScore(
//          const Viewpoint& ref_viewpoint, const Viewpoint& other_viewpoint,
//          const std::unordered_set<Point3DId>& ref_visible_points,
//          const std::unordered_set<Point3DId>& other_visible_points) const;

  FloatType computeSparseMatchingScore(
      const Viewpoint& ref_viewpoint, const Viewpoint& other_viewpoint,
      const std::unordered_map<Point3DId, Vector3>& ref_visible_points,
      const std::unordered_map<Point3DId, Vector3>& other_visible_points) const;

  FloatType computeSparseMatchingScore(
      const Viewpoint& ref_viewpoint, const Viewpoint& other_viewpoint) const;

  FloatType computeSparseMatchingScore(
      const ViewpointEntryIndex viewpoint_index1, const ViewpointEntryIndex viewpoint_index2) const;

  bool isWithinSparseMatchingLimits(const Pose& pose1, const Pose& pose2) const;

  bool isWithinSparseMatchingLimits(const Viewpoint& viewpoint1, const Viewpoint& viewpoint2) const;

  bool isSparseMatchable(
      const Viewpoint& ref_viewpoint, const Viewpoint& other_viewpoint,
      const std::unordered_map<Point3DId, Vector3>& ref_visible_points,
      const std::unordered_map<Point3DId, Vector3>& other_visible_points) const;

//  bool isSparseMatchable(
//      const Viewpoint& ref_viewpoint, const Viewpoint& other_viewpoint,
//      const std::unordered_set<Point3DId>& ref_visible_points,
//      const std::unordered_set<Point3DId>& other_visible_points) const;

  bool isSparseMatchable(
      const ViewpointEntryIndex viewpoint_index1, const ViewpointEntryIndex viewpoint_index2) const;

  bool isSparseMatchable(const Viewpoint& ref_viewpoint, const Viewpoint& other_viewpoint) const;

  bool isSparseMatchable2(const Viewpoint& viewpoint1, const Viewpoint& viewpoint2) const;

  bool isSparseMatchable2(
          const ViewpointEntryIndex viewpoint_index1,
          const ViewpointEntryIndex viewpoint_index2) const;

  bool isSparseMatchable2(
          const ViewpointEntryIndex viewpoint_index1,
          const Viewpoint& viewpoint2) const;

  bool isSparseMatchable2(
          const Viewpoint& viewpoint1,
          const Viewpoint& viewpoint2,
          const std::unordered_set<size_t>& visible_voxels1,
          const std::unordered_set<size_t>& visible_voxels2) const;

  bool isSparseMatchable2(
          const Viewpoint& viewpoint1, const Viewpoint& viewpoint2,
          const FloatType iou_threshold) const;

  bool isSparseMatchable2(
          const ViewpointEntryIndex viewpoint_index1,
          const ViewpointEntryIndex viewpoint_index2,
          const FloatType iou_threshold) const;

  bool isSparseMatchable2(
          const ViewpointEntryIndex viewpoint_index1,
          const Viewpoint& viewpoint2,
          const FloatType iou_threshold) const;

  bool isSparseMatchable2(
          const Viewpoint& viewpoint1,
          const Viewpoint& viewpoint2,
          const std::unordered_set<size_t>& visible_voxels1,
          const std::unordered_set<size_t>& visible_voxels2,
          const FloatType iou_threshold) const;

  bool isSparseMatchable2(
          const ViewpointEntryIndex viewpoint_index1,
          const Viewpoint& viewpoint2,
          const std::unordered_set<size_t>& visible_voxels2) const;

  bool isSparseMatchable2(
          const ViewpointEntryIndex viewpoint_index1,
          const Viewpoint& viewpoint2,
          const std::unordered_set<size_t>& visible_voxels2,
          const FloatType iou_threshold) const;

  void augmentedViewpointPathWithSparseMatchingViewpoints(ViewpointPath* viewpoint_path);

  void makeViewpointMotionsSparseMatchable(ViewpointPath* viewpoint_path);

  ViewpointMotion makeViewpointMotionSparseMatchable(const ViewpointMotion& motion);

  const MotionPlannerType& getMotionPlanner() const {
    return motion_planner_;
  }

  RegionType getRoi() const {
    return data_->roi_;
  }

  BoundingBoxType getRoiBbox() const {
    return data_->roi_bbox_;
  }

  const std::vector<RegionType>& getNoFlyZones() const {
    return data_->no_fly_zones_;
  }

  BoundingBoxType getExplorationBbox() const {
    return exploration_bbox_;
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

  const std::vector<ViewpointEntryIndex>& getViewpointExplorationFront() const {
    return viewpoint_exploration_front_;
  }

  const size_t getNumOfRealViewpoints() const {
    return num_real_viewpoints_;
  }

  /// Return viewpoint graph for reading. Mutex needs to be locked.
  const ViewpointGraph& getViewpointGraph() const {
    return viewpoint_graph_;
  }

  /// Return viewpoint graph for reading. Mutex needs to be locked.
  ViewpointGraph& getViewpointGraph() {
    return viewpoint_graph_;
  }

  /// Return whether viewpoint paths have been initialized
  bool areViewpointPathsInitialized() const {
    return viewpoint_paths_initialized_;
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

  bool removeViewpointMotion(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index);

  void removeViewpointMotions(const ViewpointEntryIndex index);

  /// Return best viewpoint path for reading. Mutex needs to be locked.
  const ViewpointPath& getBestViewpointPath() const;

  /// Return best viewpoint path for reading. Mutex needs to be locked.
  ViewpointPath& getBestViewpointPath();

  /// Acquire lock to acces viewpoint entries (including graph and path) or reset the planning
  std::unique_lock<std::mutex> acquireLock();

  /// Acquire lock for using any OpenGL context
  std::unique_lock<std::mutex> acquireOpenGLLock() const;

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
      const BoundingBoxType& object_bbox, size_t max_trials = (size_t)-1,
      const bool biased_orientation = true) const;

  std::pair<bool, Pose> samplePose(const RegionType& region,
      const BoundingBoxType& object_bbox, size_t max_trials = (size_t)-1,
      const bool biased_orientation = true) const;

  std::pair<bool, ViewpointPlanner::Vector3> samplePosition(const size_t max_trials = (size_t)-1) const;

  std::pair<bool, ViewpointPlanner::Vector3> samplePosition(const BoundingBoxType& bbox,
      const BoundingBoxType& object_bbox, size_t max_trials = (size_t)-1) const;

  std::pair<bool, ViewpointPlanner::Vector3> samplePosition(const RegionType& region,
      const BoundingBoxType& object_bbox, size_t max_trials = (size_t)-1) const;

  Pose::Quaternion sampleOrientation() const;

  /// Sample an orientation biased towards a bounding box.
  ///
  /// A single angle is sampled from a normal distribution (stddev depends on bounding box size) and clamped to [0, 2 pi].
  /// A spherical angle is then computed by sampling from [0, 2 pi] to determine the direction of the angle.
  /// Finally, the spherical angle is transformed to a 3D vector so that the sampling distribution is around the bounding box center
  /// seen from pos.
  Pose::Quaternion sampleBiasedOrientation(const Vector3& pos, const BoundingBoxType& bias_bbox) const;

  /// Check if an object can be placed at a position (i.e. is it free space)
  bool isValidObjectPosition(
          const Vector3& position, const BoundingBoxType& object_bbox, const bool ignore_no_fly_zones = false) const;

  /// Find a viewpoint entry with a pose.
  /// Returns whether a matching viewpoint was found and the corresponding index
  std::pair<bool, ViewpointEntryIndex> findViewpointEntryWithPose(const Pose& pose) const;

  /// Generate a new viewpoint entry and add it to the graph.
  bool generateNextViewpointEntry();

  /// Add a viewpoint entry to the graph.
  ViewpointEntryIndex addViewpointEntry(
          ViewpointEntry&& viewpoint_entry, const bool ignore_viewpoint_count_grid = false);

  /// Add a viewpoint entry to the graph.
  ViewpointEntryIndex addViewpointEntry(
          const ViewpointEntry& viewpoint_entry, const bool ignore_viewpoint_count_grid = false);

  ViewpointEntryIndex addViewpointEntry(const Pose& pose, const bool no_raycast = false);

  // TODO
  /// Generate a new viewpoint entry and add it to the graph.
  bool generateNextViewpointEntry2();
  bool tryToAddViewpointEntry(const Pose& pose, const bool no_raycast = false);
  bool tryToAddViewpointEntries(const Vector3& position, const bool no_raycast = false);

  FloatType computeExplorationStep(const Vector3& position) const;

  // TODO
  std::vector<ViewpointEntryIndex> viewpoint_exploration_front_;

  // TODO
  void computeMatchingStereoViewpoints(
          const bool ignore_sparse_matching = false,
          const bool ignore_graph_component = false);

  ViewpointEntryIndex getMatchingStereoViewpointWithoutLock(
          const ViewpointEntryIndex viewpoint_index,
          const bool ignore_sparse_matching = false,
          const bool ignore_graph_component = false);

  /// Compute motions between viewpoints and update the graph with their cost.
  void computeViewpointMotions();

  /// Compute motions from a viewpoint to other viewpoint and update the graph with their cost.
  size_t computeViewpointMotions(const ViewpointEntryIndex from_index, const bool verbose = false);

  bool connectViewpoints(
          const ViewpointEntryIndex from_viewpoint_index,
          const ViewpointEntryIndex to_viewpoint_index,
          const bool ignore_existing_connection = false);

  template <typename Iterator>
  size_t connectViewpointToOtherViewpoints(
          const ViewpointEntryIndex from_viewpoint_index,
          const Iterator to_first,
          const Iterator to_last,
          const bool ignore_existing_connections = false);

  template <typename Iterator>
  size_t connectPathEntryToOtherPathEntries(
          const ViewpointPathEntry& from_path_entry,
          const Iterator to_first,
          const Iterator to_last,
          const bool ignore_existing_connections = false);

  /// Ensures that a viewpoint on a path is connected by searching for shortest motion paths if necessary
  size_t connectViewpointPathEntry(const size_t path_entry_index,
                                   ViewpointPath* viewpoint_path,
                                   const size_t num_connections_to_try,
                                   const bool ignore_existing_connections = false);

  /// Ensures that a viewpoint on a path is connected to path entries with lower index
  size_t connectViewpointPathEntryToLowerEntries(const size_t path_entry_index,
                                                 ViewpointPath* viewpoint_path,
                                                 const bool ignore_existing_connections = false);

  /// Ensures that a viewpoint on a path is fully connected by searching for shortest motion paths if necessary
  bool fullyConnectViewpointPathEntry(const size_t path_entry_index,
                                      ViewpointPath* viewpoint_path,
                                      const bool ignore_existing_connections = false);

  /// Ensures that all viewpoints on a path are connected by searching for shortest motion paths if necessary
  bool ensureFullyConnectedViewpointPath(
          ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data, const bool recompute_all = false);

  /// Compute the novel observation information that a viewpoint will generate
  FloatType evaluateNovelViewpointInformation(
      const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data,
      const ViewpointEntryIndex viewpoint_index);

  enum NextViewpointPathEntryStatus {
    SUCCESS,
    NO_VALID_PATH_ENTRY,
    NO_VIEWPOINTS_LEFT,
    NO_IMPROVEMENT_IN_OBJECTIVE,
    NO_STEREO_VIEWPOINT,
    TIME_CONSTRAINT_EXCEEDED,
  };

  struct NextViewpointPathEntryResult {
    NextViewpointPathEntryResult()
        : has_stereo_entry(false) {}

    NextViewpointPathEntryStatus status;
    ViewpointPathEntry next_path_entry;
    bool has_stereo_entry;
    ViewpointPathEntry stereo_path_entry;
  };

  /// Find the next best viewpoint to add to the viewpoint paths.
  NextViewpointPathEntryStatus findNextViewpointPathEntries(const FloatType alpha, const FloatType beta);

  /// Initialize data structures for finding viewpoint paths
  void initializeViewpointPathEntries(const FloatType alpha, const FloatType beta);

  /// Find initial viewpoints on viewpoint paths
  void findInitialViewpointPathEntries(const FloatType alpha, const FloatType beta);

  /// Find the next best viewpoint to add to the viewpoint paths and use parameters from options.
  NextViewpointPathEntryStatus findNextViewpointPathEntries();

  bool isValidViewpointPathEntry(
      const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data,
      const ViewpointEntryIndex viewpoint_index,
      const bool ignore_if_already_on_path = false) const;

  /// Compute new information score of a new viewpoint for a given viewpoint path
  FloatType computeNewInformation(const size_t viewpoint_path_index, const ViewpointEntryIndex new_viewpoint_index) const;

  /// Compute the connected components of the graph and return a pair of the component labels and the number of components.
  const std::pair<std::vector<size_t>, size_t>& getConnectedComponents() const;

  /// Compute a viewpoint tour for all paths
  void computeViewpointTour(ViewpointPath* viewpoint_path,
                            ViewpointPathComputationData* comp_data,
                            const bool use_manual_start_position = true);

  /// Compute a viewpoint tour for all paths
  void computeViewpointTour(const bool use_manual_start_position = true);

  /// Add a path entry to a viewpoint path
  void addViewpointPathEntry(const size_t viewpoint_path_index,
                             const ViewpointEntryIndex viewpoint_index,
                             const bool ignore_observed_voxels = false);

  /// Add a path entry to a viewpoint path
  void addViewpointPathEntry(const size_t viewpoint_path_index,
                             const Pose& pose,
                             const bool ignore_observed_voxels = false);

  /// Add a path entry to a viewpoint path
  void addViewpointPathEntry(const size_t viewpoint_path_index,
                             const ViewpointPathEntry& new_path_entry,
                             const bool ignore_observed_voxels = false);

  /// Add a path entry to a viewpoint path
  void addViewpointPathEntry(ViewpointPath* viewpoint_path,
                             ViewpointPathComputationData* comp_data,
                             const ViewpointPathEntry& new_path_entry,
                             const bool ignore_observed_voxels = false);

  /// Add a path entry together with a corresponding stereo viewpoint.
  /// If a stereo viewpoint can be found returns true. Otherwise returns false.
  bool addViewpointPathEntryWithStereoPair(const size_t viewpoint_path_index,
                                           const ViewpointEntryIndex viewpoint_index,
                                           const bool ignore_observed_voxels = false);

  /// Add a path entry together with a corresponding stereo viewpoint.
  /// If a stereo viewpoint can be found returns true. Otherwise returns false.
  bool addViewpointPathEntryWithStereoPair(const size_t viewpoint_path_index,
                                           const Pose& pose,
                                           const bool ignore_observed_voxels = false);

  /// Get number of multiview-stereo viewpoints in a viewpoint path
  size_t getNumMVSViewpoints(const ViewpointPath& viewpoint_path) const;

  /// Get number of total (MVS and sparse) viewpoints in a viewpoint path
  size_t getNumTotalViewpoints(const ViewpointPath& viewpoint_path) const;

  /// Compute time required for viewpoint path
  FloatType computeViewpointPathTime(const ViewpointPath& viewpoint_path) const;

  /// Report info of current viewpoint paths
  void reportViewpointPathsStats() const;

  /// Report info on viewpoint path
  void reportViewpointPathsStats(const ViewpointPath& viewpoint_path) const;

  // Visible voxel computation

  std::unordered_set<size_t> getVisibleVoxels(const Viewpoint& viewpoint) const;

  // Raycasting and information computation

  /// Return viewpoint with virtual camera
  Viewpoint getVirtualViewpoint(const Pose& pose) const;

  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels with additional info.
  std::vector<OccupiedTreeType::IntersectionResult> getRaycastHitVoxels(
      const Viewpoint& viewpoint, const bool remove_duplicates = true) const;

  /// Perform raycast on the BVH tree on a limited window around the center.
  /// Returns a vector of hit voxels with additional info.
  std::vector<OccupiedTreeType::IntersectionResult> getRaycastHitVoxels(
      const Viewpoint& viewpoint, const size_t width, const size_t height,
      const bool remove_duplicates = true) const;

  /// Perform raycast on the BVH tree on a limited window around the center.
  /// Returns a vector of hit voxels with additional info.
  std::vector<OccupiedTreeType::IntersectionResult>
  getRaycastHitVoxels(
      const Viewpoint& viewpoint,
      const size_t x_start, const size_t x_end,
      const size_t y_start, const size_t y_end,
      const bool remove_duplicates = true) const;

  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels with additional info.
  std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates>
  getRaycastHitVoxelsWithScreenCoordinates(
      const Viewpoint& viewpoint, const bool remove_duplicates = true) const;

  /// Perform raycast on the BVH tree on a limited window around the center.
  /// Returns a vector of hit voxels with additional info.
  std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates>
  getRaycastHitVoxelsWithScreenCoordinates(
      const Viewpoint& viewpoint, const size_t width, const size_t height,
      const bool remove_duplicates = true) const;

  /// Perform raycast on the BVH tree on a limited window around the center.
  /// Returns a vector of hit voxels with additional info.
  std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates>
  getRaycastHitVoxelsWithScreenCoordinates(
      const Viewpoint& viewpoint,
      const size_t x_start, const size_t x_end,
      const size_t y_start, const size_t y_end,
      const bool remove_duplicates = true) const;

  /// Perform raycast on the BVH tree.
  /// Returns the set of hit voxels with corresponding information + the total information of all voxels.
  std::pair<VoxelWithInformationSet, FloatType>
    getRaycastHitVoxelsWithInformationScore(
            const Viewpoint& viewpoint,
            const bool ignore_voxels_with_zero_information = false,
            const bool remove_duplicates = true) const;

  /// Perform raycast on the BVH tree on a centered window with specific size.
  /// Returns the set of hit voxels with corresponding information + the total information of all voxels.
  std::pair<VoxelWithInformationSet, FloatType>
    getRaycastHitVoxelsWithInformationScore(
            const Viewpoint& viewpoint, const size_t width, const size_t height,
            const bool ignore_voxels_with_zero_information = false,
            const bool remove_duplicates = true) const;

  /// Perform raycast on the BVH tree.
  /// Returns the set of hit voxels with corresponding information + the total information of all voxels.
  std::pair<VoxelWithInformationSet, FloatType>
    getRaycastHitVoxelsWithInformationScore(
        const Viewpoint& viewpoint,
        const size_t x_start, const size_t x_end,
        const size_t y_start, const size_t y_end,
        const bool ignore_voxels_with_zero_information = false,
        const bool remove_duplicates = true) const;

  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels as a set.
  std::unordered_set<const VoxelType*> getRaycastHitVoxelsSet(const Viewpoint& viewpoint) const;

  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels as a set.
  std::unordered_set<const VoxelType*> getRaycastHitVoxelsSet(
          const Viewpoint& viewpoint, const size_t width, const size_t height) const;

  /// Perform raycast on the BVH tree.
  /// Returns a vector of hit voxels as a set.
  std::unordered_set<const VoxelType*> getRaycastHitVoxelsSet(
          const Viewpoint& viewpoint,
          const size_t x_start, const size_t x_end,
          const size_t y_start, const size_t y_end) const;

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
      const OccupiedTreeType::IntersectionResult& result) const;

  /// Returns the observation score for a single hit voxel result.
  FloatType computeViewpointObservationScore(const Viewpoint& viewpoint,
      const OccupiedTreeType::IntersectionResultWithScreenCoordinates& result) const;

  /// Returns the observation score for a container of VoxelWithInformation objects.
  template <typename Iterator>
  FloatType computeInformationScore(const Viewpoint& viewpoint, Iterator first, Iterator last) const;

  /// Compute information weighted center position of observed voxels
  Vector3 computeInformationVoxelCenter(const ViewpointEntry& viewpoint_entry) const;

  /// Convert local ENU coordinates to GPS coordinates
  using GpsCoordinateType = reconstruction::SfmToGpsTransformation::GpsCoordinate;
  GpsCoordinateType convertPositionToGps(const Vector3& position) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  QImage drawPoissonMesh(const ViewpointEntryIndex viewpoint_index) const;
  QImage drawPoissonMeshNormals(const ViewpointEntryIndex viewpoint_index) const;
  QImage drawPoissonMeshDepth(const ViewpointEntryIndex viewpoint_index) const;

  Vector3 computePoissonMeshNormalVector(const Viewpoint& viewpoint, const Vector3& position) const;

  Vector3 computePoissonMeshNormalVector(const Viewpoint& viewpoint,
                                         const size_t x, const size_t y) const;

  FloatType computePoissonMeshDepth(const Viewpoint& viewpoint, const Vector3& position) const;

  FloatType computePoissonMeshDepth(const Viewpoint& viewpoint,
                                         const size_t x, const size_t y) const;

private:
  /// Remove invalid hit voxels from raycast results
  void removeInvalidRaycastHitVoxels(
          std::vector<OccupiedTreeType::IntersectionResult>* raycast_results) const;

  /// Remove invalid hit voxels from raycast results
  void removeInvalidRaycastHitVoxels(
          std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates>* raycast_results) const;

  /// Remove duplicate hit voxels from raycast results
  void removeDuplicateRaycastHitVoxels(
      std::vector<OccupiedTreeType::IntersectionResult>* raycast_results) const;

  /// Remove duplicate hit voxels from raycast results
  void removeDuplicateRaycastHitVoxels(
      std::vector<OccupiedTreeType::IntersectionResultWithScreenCoordinates>* raycast_results) const;

  /// Remove points that are occluded from the given pose.
  /// A point needs to be dist_margin behind a voxel surface to be removed.
  void removeOccludedPoints(const Pose& pose, std::unordered_set<Point3DId>& point3D_ids, FloatType dist_margin) const;

  /// Compute information factor based on observation count (i.e. exp(-theta * observation_count)
  WeightType computeObservationInformationFactor(CounterType observation_count) const;

  /// Compute resolution information factor based on distance to voxel
  WeightType computeResolutionInformationFactor(const Viewpoint& viewpoint, const VoxelType* node) const;

  /// Retrieve normal vector for specific viewpoint
  Vector3 computeNormalVector(
          const Viewpoint& viewpoint, const VoxelType* node, const Vector2& image_coordinates) const;

  /// Compute incidence information factor based on incidence angle
  WeightType computeIncidenceInformationFactor(
      const Viewpoint& viewpoint, const VoxelType* node, const Vector3& normal_vector) const;

  /// Compute incidence information factor based on incidence angle
  WeightType computeIncidenceInformationFactor(const Viewpoint& viewpoint, const VoxelType* node) const;

  /// Compute incidence information factor based on incidence angle.
  /// Screen coordinates are provided to retrieve normal vector.
  WeightType computeIncidenceInformationFactor(
      const Viewpoint& viewpoint, const VoxelType* node, const Vector2& screen_coordinates) const;

  /// Add a viewpoint entry without acquiring a lock. Returns the index of the new viewpoint entry.
  ViewpointEntryIndex addViewpointEntryWithoutLock(
          ViewpointEntry&& viewpoint_entry, const bool ignore_viewpoint_count_grid = false);

  /// Add a viewpoint entry without acquiring a lock. Returns the index of the new viewpoint entry.
  ViewpointEntryIndex addViewpointEntryWithoutLock(
          const ViewpointEntry& viewpoint_entry, const bool ignore_viewpoint_count_grid = false);

  /// Convenience method to add next viewpoint path entry
  void addNextViewpointPathEntryResult(ViewpointPath* viewpoint_path,
                                       ViewpointPathComputationData* comp_data,
                                       const NextViewpointPathEntryResult& result,
                                       const bool ignore_observed_voxels = false);

  /// Add a path entry to a viewpoint path
  size_t addViewpointPathEntryWithoutLock(ViewpointPath *viewpoint_path, ViewpointPathComputationData *comp_data,
                                          const ViewpointPathEntry &new_path_entry,
                                          const bool ignore_observed_voxels = false);

  /// Updates observed voxel set of a viewpoint path. Returns the novel information.
  FloatType addObservedVoxelsToViewpointPath(ViewpointPath* viewpoint_path,
                                                               ViewpointPathComputationData* comp_data,
                                                               const VoxelWithInformationSet& voxel_set);

  /// Add stereo viewpoint to a viewpoint path
  std::pair<size_t, size_t> addStereoViewpointPathEntryWithoutLock(ViewpointPath *viewpoint_path, ViewpointPathComputationData *comp_data,
                                                                   const ViewpointPathEntry &first_path_entry,
                                                                   const ViewpointPathEntry &second_path_entry,
                                                                   const bool ignore_observed_voxels = false,
                                                                   const bool add_second_entry = true);

  /// Updates the latest viewpoint path entry information.
  /// Note that modified observed voxel sets can not be undone.
  void updateLastViewpointPathEntryWithoutLock(ViewpointPath* viewpoint_path,
                                               ViewpointPathComputationData* comp_data,
                                               const bool ignore_observed_voxels = false);

  /// Removes the latest viewpoint path entry. Also updates information and clears the tour order.
  /// Note that modified observed voxel sets can not be undone.
  void removeLastViewpointPathEntryWithoutLock(ViewpointPath* viewpoint_path,
                                               ViewpointPathComputationData* comp_data);

  std::pair<bool, ViewpointEntryIndex> findMatchingStereoViewpointWithoutLock(
          const ViewpointEntryIndex& viewpoint_index,
          const bool ignore_sparse_matching = false,
          const bool ignore_graph_component = false);

  /// Find a matching viewpoint for stereo matching. If no suitable viewpoint with enough overlap is already in the graph
  /// a new viewpoint is added. Returns a pair indicating whether the stereo viewpoint is already on the path (true) or not (false)
  /// and the index of the corresponding viewpoint entry. Must be called with lock.
  std::pair<bool, ViewpointEntryIndex> findMatchingStereoViewpointWithoutLock(
      const ViewpointPath& viewpoint_path,
      const ViewpointPathComputationData& comp_data,
      const ViewpointEntryIndex& viewpoint_index,
      const bool ignore_sparse_matching = false,
      const bool ignore_graph_component = false);

  std::vector<std::pair<ViewpointEntryIndex, SE3Motion>> findSE3Motions(
          const Pose& from_pose);

  /// Find motion paths from provided viewpoint to neighbors in the viewpoint graph.
  std::vector<ViewpointMotion> findViewpointMotions(
          const ViewpointEntryIndex from_index, const bool verbose = false);

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

  /// Updates information scores of other viewpoints given a path and returns best next viewpoint.
  std::pair<ViewpointEntryIndex, FloatType> updateAndGetBestNextViewpoint(
          ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data,
          const bool randomize);

  /// Find the next best viewpoint to add to a single viewpoint path.
  NextViewpointPathEntryResult findNextViewpointPathEntry(
      ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data,
      const bool randomize, const FloatType alpha, const FloatType beta);

  /// Compute new information score of a new viewpoint given a path
  FloatType computeNewInformation(const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data,
      const ViewpointEntryIndex new_viewpoint_index) const;

  /// Compute an upper bound on the information value of a given path
  FloatType computeViewpointPathInformationUpperBound(
      const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data, const size_t max_num_viewpoints) const;

  /// Report info on viewpoint path
  void reportViewpointPathsStats(
          const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data) const;

  /// Compute approx. shortest cycle to cover all viewpoints in the path (i.e. solve TSP problem)
  std::vector<size_t> computeApproximateShortestCycle(const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data);

  /// Reorders the viewpoint path to an approx. shortest cycle covering all viewpoints (i.e. TSP solution)
  bool solveApproximateTSP(ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data);

  /// Uses 2 Opt to improve the viewpoint tour
  void improveViewpointTourWith2Opt(ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data);

  /// Find shortest motion between two viewpoints using A-Star on the viewpoint graph.
  ViewpointMotion findShortestMotionAStar(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index) const;

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

  mutable bh::Random<FloatType, std::int64_t> random_;

  Options options_;

  BoundingBoxType pose_sample_bbox_;
  BoundingBoxType drone_bbox_;
  BoundingBoxType exploration_bbox_;
  FloatType triangulation_max_cos_angle_;
  FloatType triangulation_min_sin_angle_square_;
  FloatType triangulation_max_sin_angle_square_;
  FloatType triangulation_max_angular_deviation_;
  FloatType incidence_angle_threshold_;
  FloatType incidence_angle_falloff_factor_;
  FloatType voxel_sensor_size_ratio_falloff_factor_;
  FloatType sparse_matching_max_angular_distance_;

  std::unique_ptr<ViewpointPlannerData> data_;
  PinholeCamera virtual_camera_;
  viewpoint_planner::ViewpointRaycast raycaster_;
  viewpoint_planner::ViewpointScore scorer_;
  std::unique_ptr<viewpoint_planner::ViewpointOffscreenRenderer> offscreen_renderer_;

  mutable std::unique_ptr<bh::opengl::OffscreenOpenGL<FloatType>> offscreen_opengl_;
  mutable std::unique_ptr<rendering::OcTreeDrawer> octree_drawer_;

  std::mutex mutex_;

  MotionPlannerType motion_planner_;

  // Counter of failed viewpoint entry samples. Reset to 0 after each successfull sample.
  size_t num_of_failed_viewpoint_entry_samples_;
  // All tentative viewpoints
  ViewpointEntryVector viewpoint_entries_;
  // Matching stereo viewpoint index for each viewpoint (or -1 if not matching stereo viewpoint)
  std::vector<ViewpointEntryIndex> stereo_viewpoint_indices_;
  // Flags indicating whether stereo viewpoint has been computed
  std::vector<bool> stereo_viewpoint_computed_flags_;
  // Cached visible sparse points and normals
  mutable std::unordered_map<ViewpointEntryIndex, std::unordered_map<reconstruction::Point3DId, Vector3>> cached_visible_sparse_points_;
  // Mutex for cached visible sparse points
  mutable std::mutex cached_visible_sparse_points_mutex_;
  // Cached visible voxels
  mutable std::unordered_map<ViewpointEntryIndex, std::unordered_set<size_t>> cached_visible_voxels_;
  // Mutex for cached visible sparse points
  mutable std::mutex cached_visible_voxels_mutex_;
  // Number of real viewpoints at the beginning of the viewpoint_entries_ vector
  // These need to be distinguished because they could be in non-free space of the map
  size_t num_real_viewpoints_;
  // Approximate nearest neighbor index for viewpoints
  ViewpointANN viewpoint_ann_;
  // Graph of tentative viewpoints with motion distances
  ViewpointGraph viewpoint_graph_;
  // Grid of viewpoint counts in the sampling space
  ContinuousGridType viewpoint_count_grid_;
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

  // Maximum time constraint for viewpoint paths
  FloatType viewpoint_path_time_constraint_;
};

BOOST_CLASS_VERSION(ViewpointPlanner::ViewpointPathEntry, 2)

#include "viewpoint_planner.hxx"

#include "viewpoint_planner_graph.hxx"
