//==================================================
// octree_drawer.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 7, 2016
//==================================================
#pragma once

// This file is adapted from OctoMap.
// Original Copyright notice.
/*
 * This file is part of OctoMap - An Efficient Probabilistic 3D Mapping
 * Framework Based on Octrees
 * http://octomap.github.io
 *
 * Copyright (c) 2009-2014, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved. License for the viewer octovis: GNU GPL v2
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <octovis/SceneObject.h>
#include <map>
#include "triangle_drawer.h"

class OcTreeDrawer: public octomap::SceneObject {
protected:
    struct VoxelData {
        std::vector<OGLTriangleData> triangle_data;
    };

    struct VoxelArrays {
        std::vector<std::vector<GLfloat>> cubeArrays;
        std::vector<GLfloat> colorArray;
    };

public:
    OcTreeDrawer();
    virtual ~OcTreeDrawer();
    void clear();

    void draw() const;

    // initialization of drawer  -------------------------

    /// sets a new OcTree that should be drawn by this drawer
    void setOctree(const octomap::OcTree* octree) {
      octomap::pose6d o; // initialized to (0,0,0) , (0,0,0,1) by default
      setOctree(octree, o);
    }

    const std::vector<double>& getOccupancyBins() const;

    /// sets a new OcTree that should be drawn by this drawer
    /// origin specifies a global transformation that should be applied
    virtual void setOctree(const octomap::OcTree* octree, const octomap::pose6d& origin);

    double findOccupancyBin(double occupancy) const;

    void updateVoxelsFromOctree();
    void updateVoxelArrays();
    void updateVoxelColorHeightmap();

    // modification of existing drawer  ------------------

    /// sets a new selection of the current OcTree to be drawn
    void setOcTreeSelection(const std::list<octomap::OcTreeVolume>& selectedPoints);

    /// sets alpha level for occupied cells
    double getOccupancyThreshold() const;
    void setOccupancyThreshold(double occupancy_threshold);
    void setDrawFreeVoxels(bool draw_free_voxels);
    void setDisplayAxes(bool display_axes);
    void setAlphaOccupied(double alpha);
    void setDrawSingleBin(bool draw_single_bin);
    void setRenderTreeDepth(size_t render_tree_depth) { m_update = true; render_tree_depth_ = render_tree_depth;};

    // set new origin (move object)
    void setOrigin(octomap::pose6d t);
    void enableAxes(bool enabled = true) { m_update = true; m_displayAxes = enabled; };

protected:
    void drawVoxelsAboveThreshold(double occupancy_threshold, bool draw_below_threshold=false) const;
    void drawCubes(const VoxelArrays& voxelArrays) const;

    void drawAxes() const;

    //! setup cube template
    void initCubeTemplate(const octomath::Pose6D& origin,
                          std::vector<octomath::Vector3>& cube_template);
    void generateCube(const octomap::OcTreeVolume& v,
                      const std::vector<octomath::Vector3>& cube_template,
                      VoxelData& voxel_data);
    void generateCube(const octomap::OcTreeVolume& v,
                      const std::vector<octomath::Vector3>& cube_template,
                      VoxelArrays& voxelArrays);
    unsigned int generateCube(const octomap::OcTreeVolume& v,
                                          const std::vector<octomath::Vector3>& cube_template,
                                          const unsigned int& current_array_idx,
                                          GLfloat*** glArray);
    void setCubeColorHeightmap(const octomap::OcTreeVolume& v, std::vector<GLfloat>& colors);
    unsigned int setCubeColorHeightmap(const octomap::OcTreeVolume& v,
                                          const unsigned int current_array_idx,
                                          GLfloat** glColorArray);
    void setCubeColorHeightmap(const octomap::OcTreeVolume& v, VoxelData& voxel_data);
    void setVertexDataFromOctomathVector(OGLVertexData& vertex, const octomath::Vector3& vec);

    const octomap::OcTree* octree_;
    octomap::pose6d origin_;
    octomap::pose6d initial_origin_;

    std::map<double, VoxelArrays> voxelArraysByOccupancy_;
    std::vector<double> occupancy_bins_;
//    std::map<double, TriangleDrawer> voxel_drawer_map_;

    mutable bool m_update;

    double m_occupancyThreshold;
    bool m_drawFreeVoxels;
    double m_alphaOccupied;
    bool m_drawSingleBin;
    size_t render_tree_depth_;
    bool m_displayAxes;
};
