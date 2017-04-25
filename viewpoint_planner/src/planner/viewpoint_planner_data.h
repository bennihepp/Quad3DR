/*
 * viewpoint_planner_data.h
 *
 *  Created on: Dec 24, 2016
 *      Author: bhepp
 */

#pragma once

#include <bh/eigen.h>
#include <bh/mLib//mLib.h>
#include <memory>
#include <bh/boost.h>
#include <boost/serialization/access.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/filesystem.hpp>
#include <bh/common.h>
#include <bh/config_options.h>
#include <bh/eigen_options.h>
#include <bh/math/geometry.h>
#include "occupied_tree.h"
#include "../octree/occupancy_map.h"
#include "../reconstruction/dense_reconstruction.h"
#include "../bvh/bvh.h"

class ViewpointPlanner;

template <typename FloatT>
class MotionPlanner;

class ViewpointPlannerData {
public:
  using FloatType = float;
  using size_t = std::size_t;
  USE_FIXED_EIGEN_TYPES(FloatType);

  class Options : public bh::ConfigOptions {
  public:

    Options()
    : bh::ConfigOptions("viewpoint_planner.data", "ViewpointPlannerData options") {
      addOption<std::string>("dense_reconstruction_path", "");
      addOption<bool>("dense_reconstruction_has_gps", false);
      addOption<std::string>("dense_points_filename", "");
      addOption<std::string>("poisson_mesh_filename");
      addOption<std::string>("raw_octree_filename");
      addOption<std::string>("octree_filename", "");
      addOption<std::string>("bvh_filename", "");
      addOption<std::string>("distance_field_filename", "");
      addOption<bool>("use_distance_field", &use_distance_field);
      addOption<bool>("force_weights_update", &force_weights_update);
      addOption<bool>("regenerate_augmented_octree", &regenerate_augmented_octree);
      addOption<bool>("regenerate_bvh_tree", &regenerate_bvh_tree);
      addOption<bool>("regenerate_distance_field", &regenerate_distance_field);
      addOption<std::string>("regions_json_filename", &regions_json_filename);
      addOption<FloatType>("obstacle_free_height", &obstacle_free_height);
      addOption<FloatType>("bvh_bbox_min_x", -1000);
      addOption<FloatType>("bvh_bbox_min_y", -1000);
      addOption<FloatType>("bvh_bbox_min_z", -1000);
      addOption<FloatType>("bvh_bbox_max_x", +1000);
      addOption<FloatType>("bvh_bbox_max_y", +1000);
      addOption<FloatType>("bvh_bbox_max_z", +1000);
      addOption<FloatType>("roi_bbox_min_x", -50);
      addOption<FloatType>("roi_bbox_min_y", -50);
      addOption<FloatType>("roi_bbox_min_z", -50);
      addOption<FloatType>("roi_bbox_max_x", +50);
      addOption<FloatType>("roi_bbox_max_y", +50);
      addOption<FloatType>("roi_bbox_max_z", +50);
      addOptionalOption<FloatType>("roi_z_low");
      addOptionalOption<FloatType>("roi_z_high");
      addOptionalOption<Vector2>("roi_vertex1");
      addOptionalOption<Vector2>("roi_vertex2");
      addOptionalOption<Vector2>("roi_vertex3");
      addOptionalOption<Vector2>("roi_vertex4");
      addOption<size_t>("bvh_normal_mesh_knn", &bvh_normal_mesh_knn);
      addOption<FloatType>("bvh_normal_mesh_max_dist", &bvh_normal_mesh_max_dist);
      addOption<size_t>("grid_dimension", &grid_dimension);
      addOption<FloatType>("distance_field_cutoff", &distance_field_cutoff);
      addOption<FloatType>("roi_falloff_distance", &roi_falloff_distance);
      addOption<bool>("weight_falloff_quadratic", &weight_falloff_quadratic);
      addOption<FloatType>("weight_falloff_distance_start", &weight_falloff_distance_start);
      addOption<FloatType>("voxel_information_lambda", &voxel_information_lambda);
      addOption<bool>("ignore_real_observed_voxels", &ignore_real_observed_voxels);
      addOption<FloatType>("invalid_pixel_observation_factor", &invalid_pixel_observation_factor);
      addOption<FloatType>("real_observed_voxels_raycast_min_range", &real_observed_voxels_raycast_min_range);
      addOption<FloatType>("real_observed_voxels_raycast_max_range", &real_observed_voxels_raycast_max_range);
      addOption<bool>("enable_opengl", &enable_opengl);
#if WITH_CUDA
      addOption<bool>("enable_cuda", &enable_cuda);
      addOption<size_t>("cuda_stack_size", &cuda_stack_size);
      addOption<int>("cuda_gpu_id", &cuda_gpu_id);
#endif
    }

    ~Options() override {}

    bool use_distance_field = true;
    bool force_weights_update = false;
    bool regenerate_augmented_octree = false;
    bool regenerate_bvh_tree = false;
    bool regenerate_distance_field = false;
    std::string regions_json_filename = "";
    FloatType obstacle_free_height = std::numeric_limits<FloatType>::max();
    size_t bvh_normal_mesh_knn = 10;
    FloatType bvh_normal_mesh_max_dist = 2;
    size_t grid_dimension = 128;
    FloatType roi_falloff_distance = 10;
    FloatType distance_field_cutoff = 5;
    bool weight_falloff_quadratic = true;
    FloatType weight_falloff_distance_start = 0;
    // Factor in the exponential of the voxel information
    FloatType voxel_information_lambda = FloatType(0.1);
    bool ignore_real_observed_voxels = false;
    FloatType invalid_pixel_observation_factor = FloatType(0.5);
    FloatType real_observed_voxels_raycast_min_range = FloatType(5);
    FloatType real_observed_voxels_raycast_max_range = std::numeric_limits<FloatType>::max();
    bool enable_opengl = true;
#if WITH_CUDA
    bool enable_cuda = true;
    size_t cuda_stack_size = 32 * 1024;
    int cuda_gpu_id = 0;
#endif
  };

  static constexpr double OCCUPANCY_WEIGHT_DEPTH = 12;
  static constexpr double OCCUPANCY_WEIGHT_REACH = 2;
  static constexpr double OCCUPANCY_WEIGHT_DEPTH_CUTOFF = 16;

  using PointCloudIOType = ml::PointCloudIO<FloatType>;
  using PointCloudType = ml::PointCloud<FloatType>;
  using MeshIOType = ml::MeshIO<FloatType>;
  using MeshType = ml::MeshData<FloatType>;
  using BoundingBoxType = bh::BoundingBox3D<FloatType>;
  using Vector3i = Eigen::Vector3i;
  using DistanceFieldType = ml::DistanceField3f;
  using RegionType = bh::PolygonWithLowerAndUpperPlane<FloatType>;

  using RawOccupancyMapType = OccupancyMap<OccupancyNode>;
  using OccupancyMapType = OccupancyMap<AugmentedOccupancyNode>;

  using NodeObjectType = viewpoint_planner::NodeObjectType;
  using OccupiedTreeType = viewpoint_planner::OccupiedTreeType;

  using TreeNavigatorType = TreeNavigator<OccupancyMapType, OccupancyMapType::NodeType>;
  using ConstTreeNavigatorType = TreeNavigator<const OccupancyMapType, const OccupancyMapType::NodeType>;
  using CounterType = OccupancyMapType::NodeType::CounterType;
  using WeightType = OccupancyMapType::NodeType::WeightType;

  ViewpointPlannerData(const Options* options);

  const Options& getOptions() const {
    return options_;
  }

  const OccupancyMapType& getOctree() const {
    return *octree_;
  }

  OccupancyMapType& getOctree() {
    return *octree_;
  }

  const OccupiedTreeType& getOccupancyBVHTree() const {
    return occupied_bvh_;
  }

  // TODO: Intersection and raycasting should be const in BVH tree
  OccupiedTreeType& getOccupancyBVHTree() {
    return occupied_bvh_;
  }

  /// Check if an object can be placed at a position (i.e. is it free space)
  bool isValidObjectPosition(const Vector3& position, const BoundingBoxType& object_bbox) const;

  const reconstruction::DenseReconstruction& getReconstruction() const;

  const DistanceFieldType& getDistanceField() const;

  bool isInsideGrid(const Vector3& xyz) const;
  Vector3i getGridIndices(const Vector3& xyz) const;
  Vector3 getGridPosition(const Vector3i& indices) const;
  Vector3 getGridPosition(int ix, int iy, int iz) const;

private:
  friend class ViewpointPlanner;

  template <typename FloatT>
  friend class MotionPlanner;

  std::time_t getLastWriteTime(const std::string& filename) const {
    return boost::filesystem::last_write_time(filename);
  }

  RegionType convertGpsRegionToEnuRegion(const boost::property_tree::ptree& pt) const;

  void readDenseReconstruction(const std::string& path);
  bool readAndAugmentOctree(
      std::string octree_filename, const std::string& raw_octree_filename, bool binary=false);
  void readDensePoints(const std::string& dense_points_filename);
  void readPoissonMesh(const std::string& mesh_filename);
  bool readBVHTree(std::string bvh_filename, const std::string& octree_filename);
  /// Distance field to poisson mesh based on overall bounding box volume
  bool readMeshDistanceField(std::string df_filename, const std::string& mesh_filename);

  void _readMeshDistanceField(const std::string& df_filename, DistanceFieldType* distance_field);
  void _writeMeshDistanceField(const std::string& df_filename, const DistanceFieldType& distance_field);

  void updateWeights();
  void updateWeightsWithRealViewpoints();

  WeightType computeObservationCountFactor(CounterType observation_count) const;

  std::unique_ptr<RawOccupancyMapType>
  readRawOctree(const std::string& filename, bool binary=false) const;

  std::unique_ptr<OccupancyMapType>
  generateAugmentedOctree(std::unique_ptr<RawOccupancyMapType> raw_octree) const;

  void generateBVHTree(const OccupancyMapType* octree);

  void readCachedBVHTree(const std::string& filename);

  void writeBVHTree(const std::string& filename) const;

  void generateWeightGrid();

  void generateDistanceField();

  template <typename TreeT>
  static bool isTreeConsistent(const TreeT& tree);

  // TODO: Can be removed?
//  static WeightType computeWeightContribution(
//      const Eigen::Vector3f& query_pos, FloatType dist_cutoff_sq, const ConstTreeNavigatorType& nav);

  Options options_;

  BoundingBoxType bvh_bbox_;
  RegionType roi_;
  BoundingBoxType roi_bbox_;
  std::vector<RegionType> no_fly_zones_;

  std::unique_ptr<reconstruction::DenseReconstruction> reconstruction_;
  std::unique_ptr<OccupancyMapType> octree_;

  std::unique_ptr<PointCloudType> dense_points_;
  std::unique_ptr<MeshType> poisson_mesh_;

  BoundingBoxType grid_bbox_;
  Vector3i grid_dim_;
  Vector3 grid_origin_;
  FloatType grid_increment_;

  DistanceFieldType distance_field_;
  OccupiedTreeType occupied_bvh_;
};
