//==================================================
// octree_drawer.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 19.04.17
//==================================================
#pragma once

#include <unordered_map>
#include <vector>
#include "../planner/viewpoint_planner_types.h"
#include "../planner/occupied_tree.h"
#include "triangle_drawer.h"
#include "voxel_drawer.h"

namespace rendering {

class OcTreeDrawer {
public:
  using FloatType = float;;

  OcTreeDrawer();

  virtual ~OcTreeDrawer();

  const VoxelDrawer& getVoxelDrawer() const;

  VoxelDrawer& getVoxelDrawer();

  void draw(const QMatrix4x4 &pvm_matrix, const QMatrix4x4 &vm_matrix);

  void clear();

  void setOctree(const viewpoint_planner::OccupancyMapType* octree,
                 const FloatType min_occupancy = FloatType(0.5),
                 const size_t min_observation_count = 1,
                 const size_t render_tree_depth = 0);

  void updateVoxelsFromOctree();

  void updateVoxelData();

  void updateVoxelColorHeightmap();

  void configVoxelDrawer();

  void setDrawOctree(bool draw_octree);

  void setColorFlags(uint32_t color_flags_uint);

  void setAlphaOccupied(FloatType alpha);

  void setMinOccupancy(FloatType min_occupancy);

  void setMaxOccupancy(FloatType max_occupancy);

  void setMinObservations(uint32_t min_observations);

  void setMaxObservations(uint32_t max_observations);

  void setMinVoxelSize(FloatType min_voxel_size);

  void setMaxVoxelSize(FloatType max_voxel_size);

  void setMinWeight(FloatType min_weight);

  void setMaxWeight(FloatType max_weight);

  void setWeightRange(const FloatType low_weight, const FloatType high_weight);

  void setMinInformation(FloatType min_information);

  void setMaxInformation(FloatType max_information);

  void setInformationRange(const FloatType low_information, const FloatType high_information);

private:
  void drawVoxels(const QMatrix4x4 &pvm_matrix, const QMatrix4x4 &vm_matrix);

  void setVertexDataFromOctomathVector(OGLVertexDataRGBA &vertex, const octomath::Vector3 &vec);

  OGLColorData getVoxelColorData(const OGLVoxelData &voxel_data, FloatType min_z, FloatType max_z) const;

  const viewpoint_planner::OccupancyMapType *octree_;

  VoxelDrawer voxel_drawer_;
  bool draw_octree_;

  VoxelDrawer::ColorFlags color_flags_;
  bool draw_free_voxels_;
  FloatType alpha_override_;

  FloatType occupancy_threshold_;
  size_t observation_count_threshold_;
  size_t render_tree_depth_;

  FloatType min_occupancy_;
  FloatType max_occupancy_;
  uint32_t min_observations_;
  uint32_t max_observations_;
  uint32_t low_observation_count_;
  uint32_t high_observation_count_;
  FloatType min_voxel_size_;
  FloatType max_voxel_size_;
  FloatType min_weight_;
  FloatType max_weight_;
  FloatType low_weight_;
  FloatType high_weight_;
  FloatType min_information_;
  FloatType max_information_;
};

}
