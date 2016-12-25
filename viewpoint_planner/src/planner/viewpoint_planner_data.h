/*
 * viewpoint_planner_data.h
 *
 *  Created on: Dec 24, 2016
 *      Author: bhepp
 */

#pragma once

#include <ait/eigen.h>
#include <ait/mLib.h>
#include <memory>
#include <ait/common.h>
#include <ait/options.h>
#include <ait/serialization.h>
#include "../occupancy_map.h"
#include "../reconstruction/dense_reconstruction.h"
#include "../bvh/bvh.h"

struct NodeObject : ait::Serializable {
  const static std::string kFileTag;

  void write(std::ostream& out) const override {
    ait::writeToStream(out, occupancy);
    ait::writeToStream(out, observation_count);
    ait::writeToStream(out, weight);
  }

  void read(std::istream& in) override {
    ait::readFromStream(in, &occupancy);
    ait::readFromStream(in, &observation_count);
    ait::readFromStream(in, &weight);
  }

  float occupancy;
  uint16_t observation_count;
  float weight;
};

class ViewpointPlanner;

class ViewpointPlannerData {
public:
  using FloatType = float;

  class Options : public ait::ConfigOptions {
  public:

    Options()
    : ait::ConfigOptions("viewpoint_planner.data", "ViewpointPlannerData options") {
      addOption<std::string>("dense_reconstruction_path");
      addOption<std::string>("poisson_mesh_filename");
      addOption<std::string>("raw_octree_filename");
      addOption<std::string>("octree_filename", "");
      addOption<std::string>("bvh_filename", "");
      addOption<std::string>("distance_field_filename", "");
      addOption<size_t>("grid_dimension", 128);
      addOption<FloatType>("roi_bbox_min_x", -50);
      addOption<FloatType>("roi_bbox_min_y", -50);
      addOption<FloatType>("roi_bbox_min_z", -50);
      addOption<FloatType>("roi_bbox_max_x", +50);
      addOption<FloatType>("roi_bbox_max_y", +50);
      addOption<FloatType>("roi_bbox_max_z", +50);
      addOption<FloatType>("drone_extent_x", 3);
      addOption<FloatType>("drone_extent_y", 3);
      addOption<FloatType>("drone_extent_z", 3);
      addOption<FloatType>("distance_field_cutoff", 5);
    }

    ~Options() override {}
  };

  static constexpr double OCCUPANCY_WEIGHT_DEPTH = 12;
  static constexpr double OCCUPANCY_WEIGHT_REACH = 2;
  static constexpr double OCCUPANCY_WEIGHT_DEPTH_CUTOFF = 16;

  using MeshIOType = ml::MeshIO<FloatType>;
  using MeshType = ml::MeshData<FloatType>;
  using BoundingBoxType = bvh::BoundingBox3D<FloatType>;
  using Vector3 = Eigen::Vector3f;
  using Vector3i = Eigen::Vector3i;
  using DistanceFieldType = ml::DistanceField3f;

  using RawOccupancyMapType = OccupancyMap<OccupancyNode>;
  using OccupancyMapType = OccupancyMap<AugmentedOccupancyNode>;
  using OccupiedTreeType = bvh::Tree<NodeObject, FloatType>;

  using TreeNavigatorType = TreeNavigator<OccupancyMapType, OccupancyMapType::NodeType>;
  using ConstTreeNavigatorType = TreeNavigator<const OccupancyMapType, const OccupancyMapType::NodeType>;
  using WeightType = OccupancyMapType::NodeType::WeightType;

  ViewpointPlannerData(const Options* options);

private:
  friend class ViewpointPlanner;

  void readDenseReconstruction(const std::string& path);
  bool readAndAugmentOctree(
      std::string octree_filename, const std::string& raw_octree_filename, bool binary=false);
  void readPoissonMesh(const std::string& mesh_filename);
  bool readBVHTree(std::string bvh_filename, const std::string& octree_filename);
  /// Distance field to poisson mesh based on overall bounding box volume
  bool readMeshDistanceField(std::string df_filename, const std::string& mesh_filename);

  void updateWeights();

  std::unique_ptr<RawOccupancyMapType>
  readRawOctree(const std::string& filename, bool binary=false) const;

  std::unique_ptr<OccupancyMapType>
  generateAugmentedOctree(std::unique_ptr<RawOccupancyMapType> raw_octree) const;

  void generateBVHTree(const OccupancyMapType* octree);

  void readCachedBVHTree(const std::string& filename);

  void writeBVHTree(const std::string& filename) const;

  void generateDistanceField();

  bool isInsideGrid(const Vector3& xyz) const;
  Vector3i getGridIndices(const Vector3& xyz) const;
  Vector3 getGridPosition(const Vector3i& indices) const;
  Vector3 getGridPosition(int ix, int iy, int iz) const;

  template <typename TreeT>
  static bool isTreeConsistent(const TreeT& tree);

  static WeightType computeWeightContribution(
      const Eigen::Vector3f& query_pos, FloatType dist_cutoff_sq, const ConstTreeNavigatorType& nav);

  BoundingBoxType roi_bbox_;
  Vector3 drone_extent_;

  std::unique_ptr<DenseReconstruction> reconstruction_;
  std::unique_ptr<OccupancyMapType> octree_;

  std::unique_ptr<MeshType> poisson_mesh_;
  BoundingBoxType grid_bbox_;
  size_t grid_dimension_;
  Vector3i grid_dim_;
  Vector3 grid_origin_;
  FloatType grid_increment_;
  DistanceFieldType distance_field_;
  FloatType df_cutoff_;
  OccupiedTreeType occupied_bvh_;
};
