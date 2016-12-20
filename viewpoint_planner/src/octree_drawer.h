//==================================================
// octree_drawer.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 7, 2016
//==================================================
#pragma once

#include <unordered_map>
#include <vector>
#include "viewpoint_planner.h"
#include "triangle_drawer.h"
#include "voxel_drawer.h"

class OcTreeDrawer {
public:
    OcTreeDrawer();
    virtual ~OcTreeDrawer();

    void draw(const QMatrix4x4& pvm_matrix, const QMatrix4x4& view_matrix, const QMatrix4x4& model_matrix);

    // initialization of drawer  -------------------------

    /// sets a new OcTree that should be drawn by this drawer
    void setOctree(const ViewpointPlanner::OccupancyMapType* octree) {
      octomap::pose6d o; // initialized to (0,0,0) , (0,0,0,1) by default
      setOctree(octree, o);
    }

    const std::vector<float>& getOccupancyBins() const;

    /// sets a new OcTree that should be drawn by this drawer
    /// origin specifies a global transformation that should be applied
    virtual void setOctree(const ViewpointPlanner::OccupancyMapType* octree, const octomap::pose6d& origin);

    float findOccupancyBin(float occupancy) const;

    void updateVoxelsFromOctree();
    void updateVoxelData();
    void updateVoxelColorHeightmap();
    void updateRaycastVoxels(const std::vector<std::pair<ViewpointPlanner::ConstTreeNavigatorType, float>>& raycast_voxels);
    void updateRaycastVoxels(std::vector<std::pair<ViewpointPlannerMaps::OccupiedTreeType::NodeType*, float>>& raycats_voxels);
    void configVoxelDrawer(VoxelDrawer& voxel_drawer) const;

    float getOccupancyBinThreshold() const;
    void setOccupancyBinThreshold(float occupancy_bin_threshold);
    void setDrawOctree(bool draw_octree);
    void setDrawRaycast(bool draw_raycast);
    void setColorFlags(uint32_t color_flags_uint);
    void setDrawFreeVoxels(bool draw_free_voxels);
    void setDisplayAxes(bool display_axes);
    void setAlphaOccupied(float alpha);
    void setDrawSingleBin(bool draw_single_bin);
    void setMinOccupancy(float min_occupancy);
    void setMaxOccupancy(float max_occupancy);
    void setMinObservations(uint32_t min_observations);
    void setMaxObservations(uint32_t max_observations);
    void setMinVoxelSize(float min_voxel_size);
    void setMaxVoxelSize(float max_voxel_size);
    void setMinWeight(float min_weight);
    void setMaxWeight(float max_weight);

    size_t getRenderTreeDepth() const;
    void setRenderTreeDepth(size_t render_tree_depth);
    size_t getRenderObservationThreshold() const;
    void setRenderObservationThreshold(size_t min_observations);

    // set new origin (move object)
    void setOrigin(octomap::pose6d t);

private:
    void drawVoxelsAboveThreshold(const QMatrix4x4& pvm_matrix, const QMatrix4x4& view_matrix, const QMatrix4x4& model_matrix,
        float occupancy_threshold, bool draw_below_threshold=false);

    void setVertexDataFromOctomathVector(OGLVertexDataRGBA& vertex, const octomath::Vector3& vec);

    OGLColorData getVoxelColorData(const OGLVoxelData& voxel_data, float min_z, float max_z) const;

    void forEachVoxelDrawer(const std::function<void(VoxelDrawer&)> func);

    const ViewpointPlanner::OccupancyMapType* octree_;
    octomap::pose6d origin_;
    octomap::pose6d initial_origin_;

    std::vector<float> occupancy_bins_;
    std::unordered_map<float, VoxelDrawer> voxel_drawer_map_;
    bool draw_octree_;

    std::unique_ptr<VoxelDrawer> raycast_drawer_;
    bool draw_raycast_;

    float occupancy_threshold_;
    VoxelDrawer::ColorFlags color_flags_;
    bool draw_free_voxels_;
    float alpha_override_;
    bool draw_single_bin_;
    size_t render_tree_depth_;
    size_t render_observation_threshold_;

    float min_occupancy_;
    float max_occupancy_;
    uint32_t min_observations_;
    uint32_t max_observations_;
    float min_voxel_size_;
    float max_voxel_size_;
    float min_weight_;
    float max_weight_;
    float low_weight_;
    float high_weight_;
    uint32_t low_observation_count_;
    uint32_t high_observation_count_;
};
