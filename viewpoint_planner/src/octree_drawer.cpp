//==================================================
// octree_drawer.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 7, 2016
//==================================================

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

#include "octree_drawer.h"
#include <qglviewer.h>
#include <ait/common.h>

#define OTD_RAD2DEG 57.2957795

using AbstractOcTree = octomap::AbstractOcTree;
using OcTree = octomap::OcTree;
using pose6d = octomap::pose6d;
using OcTreeVolume = octomap::OcTreeVolume;

OcTreeDrawer::OcTreeDrawer() : SceneObject(),
                             octree_(nullptr), m_occupancyThreshold(0.5), m_drawFreeVoxels(false), m_alphaOccupied(0.8),
                             m_drawSingleBin(false), render_tree_depth_(20), m_displayAxes(true)
{
    m_update = true;
    // origin and movement
    initial_origin_ = octomap::pose6d(0,0,0,0,0,0);
    origin_ = initial_origin_;
    for (size_t i = 0; i < 10; ++i) {
        double occupancy_bin = i / 10.0;
        occupancy_bins_.push_back(occupancy_bin);
    }
    m_occupancyThreshold = occupancy_bins_[occupancy_bins_.size() / 2];
}

OcTreeDrawer::~OcTreeDrawer()
{
    clear();
}

void OcTreeDrawer::draw() const
{
//    if (m_update) {
    // push current status
    glPushMatrix();
    // octomap::pose6d relative_transform = origin * initial_origin_.inv();

    octomap::pose6d relative_transform = origin_;// * initial_origin;

    // apply relative transform
    const octomath::Quaternion& q = relative_transform.rot();
    glTranslatef(relative_transform.x(), relative_transform.y(), relative_transform.z());

    // convert quaternion to angle/axis notation
    float scale = sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
    if (scale) {
        float axis_x = q.x() / scale;
        float axis_y = q.y() / scale;
        float axis_z = q.z() / scale;
        float angle = acos(q.u()) * 2.0f * OTD_RAD2DEG;  //  opengl expects DEG
        glRotatef(angle, axis_x, axis_y, axis_z);
    }

    glEnableClientState(GL_VERTEX_ARRAY);

    drawVoxelsAboveThreshold(m_occupancyThreshold, m_drawFreeVoxels);

    glDisableClientState(GL_VERTEX_ARRAY);

    // reset previous status
    glPopMatrix();
    m_update = false;
//    }
}

double OcTreeDrawer::getOccupancyThreshold() const {
    return m_occupancyThreshold;
}

void OcTreeDrawer::setOccupancyThreshold(double occupancy_threshold) {
    m_update = true;
    m_occupancyThreshold = occupancy_threshold;
}

void OcTreeDrawer::setDrawFreeVoxels(bool draw_free_voxels) {
    m_update = true;
    m_drawFreeVoxels = draw_free_voxels;
}

void OcTreeDrawer::setDisplayAxes(bool display_axes) {
    m_update = true;
    m_displayAxes = display_axes;
}

void OcTreeDrawer::setAlphaOccupied(double alpha) {
    m_update = true;
    m_alphaOccupied = alpha;
    updateVoxelColorHeightmap();

}

void OcTreeDrawer::setDrawSingleBin(bool draw_single_bin)
{
    m_update = true;
    m_drawSingleBin = draw_single_bin;
}

const std::vector<double>& OcTreeDrawer::getOccupancyBins() const {
    return occupancy_bins_;
}

void OcTreeDrawer::setOctree(const OcTree* octree, const pose6d& origin)
{
    octree_ = octree;
    // save origin used during cube generation
    initial_origin_ = octomap::pose6d(octomap::point3d(0,0,0), origin.rot());
    // origin is in global coords
    origin_ = origin;
    updateVoxelsFromOctree();
}

void OcTreeDrawer::updateVoxelsFromOctree()
{
    updateVoxelArrays();
}

double OcTreeDrawer::findOccupancyBin(double occupancy) const {
    auto it = std::upper_bound(occupancy_bins_.cbegin(), occupancy_bins_.cend(), occupancy);
    if (it == occupancy_bins_.cend()) {
        return occupancy_bins_.back();
    }
    if (it != occupancy_bins_.cbegin()) {
        --it;
    }
    return *it;
}

void OcTreeDrawer::updateVoxelArrays()
{
    AIT_ASSERT_STR(octree_ != nullptr, "Octree was not initialized");
    // maximum size to prevent crashes on large maps: (should be checked in a better way than a constant)
    bool uses_origin = ( (origin_.rot().x() != 0.) && (origin_.rot().y() != 0.)
        && (origin_.rot().z() != 0.) && (origin_.rot().u() != 1.) );

    double minX, minY, minZ, maxX, maxY, maxZ;
    octree_->getMetricMin(minX, minY, minZ);
    octree_->getMetricMax(maxX, maxY, maxZ);

    // set min/max Z for color height map
    m_zMin = minZ;
    m_zMax = maxZ;

    std::vector<octomath::Vector3> cube_template;
    initCubeTemplate(origin_, cube_template);

    OcTreeVolume voxel;
    voxelArraysByOccupancy_.clear();
    for (double occupancy_bin : occupancy_bins_) {
        voxelArraysByOccupancy_.emplace(occupancy_bin, VoxelArrays());
        voxelArraysByOccupancy_.at(occupancy_bin).cubeArrays.resize(6);
    }
    for(OcTree::tree_iterator it = octree_->begin_tree(render_tree_depth_), end=octree_->end_tree(); it!= end; ++it) {
        if (it.isLeaf()) { // voxels for leaf nodes
            if (uses_origin)
                voxel = OcTreeVolume(origin_.rot().rotate(it.getCoordinate()), it.getSize());
            else
                voxel = OcTreeVolume(it.getCoordinate(), it.getSize());

            double occupancy = it->getOccupancy();
            double occupancy_bin = findOccupancyBin(occupancy);
            generateCube(voxel, cube_template, voxelArraysByOccupancy_.at(occupancy_bin));
            setCubeColorHeightmap(voxel, voxelArraysByOccupancy_.at(occupancy_bin).colorArray);
        }
    }
    for (auto& entry : voxelArraysByOccupancy_) {
        for (auto& voxel_array : entry.second.cubeArrays) {
            voxel_array.shrink_to_fit();
        }
        entry.second.colorArray.shrink_to_fit();
    }
//    // TODO
//    std::map<double, VoxelData> voxel_data_map;
//    for (size_t i = 1; i < 10; ++i) {
//        double occupancy_bin = i / 10.0;
//        occupancy_bins_.push_back(occupancy_bin);
//        voxel_data_map.emplace(occupancy_bin);
//    }
//    occupancy_bins_.clear();
//    voxelArraysByOccupancy_.clear();
//    for(OcTree::tree_iterator it = octree_->begin_tree(render_tree_depth_), end=octree_->end_tree(); it!= end; ++it) {
//        if (it.isLeaf()) { // voxels for leaf nodes
//            if (uses_origin)
//                voxel = OcTreeVolume(origin_.rot().rotate(it.getCoordinate()), it.getSize());
//            else
//                voxel = OcTreeVolume(it.getCoordinate(), it.getSize());
//
//            double occupancy = it->getOccupancy();
//            double occupancy_bin = std::floor(occupancy * 10) / 10.0;
//            generateCube(voxel, cube_template, voxel_data_map.at(occupancy_bin));
//            setCubeColorHeightmap(voxel, voxelArraysByOccupancy_.at(occupancy_bin).colorArray);
//        }
//    }
//    voxel_drawer_map_.clear();
//    for (auto& entry : voxel_data_map) {
//        voxel_drawer_map_.emplace(entry.first);
//        voxel_drawer_map_.at(entry.first).init();
//        voxel_drawer_map_.at(entry.first).upload(entry.second.triangle_data);
//    }

    for (const auto& entry : voxelArraysByOccupancy_) {
        std::cout << "Voxels in bin " << entry.first << ": " << entry.second.cubeArrays[0].size() / 3 << std::endl;
    }
}

void OcTreeDrawer::updateVoxelColorHeightmap()
{
    AIT_ASSERT_STR(octree_ != nullptr, "Octree was not initialized");
    double minX, minY, minZ, maxX, maxY, maxZ;
    octree_->getMetricMin(minX, minY, minZ);
    octree_->getMetricMax(maxX, maxY, maxZ);

    // set min/max Z for color height map
    m_zMin = minZ;
    m_zMax = maxZ;

    for (auto& entry : voxelArraysByOccupancy_) {
        for (size_t i = 3; i < entry.second.colorArray.size(); i += 4) {
            entry.second.colorArray[i] = m_alphaOccupied;
        }
    }
}

void OcTreeDrawer::initCubeTemplate(const octomath::Pose6D& origin,
                                  std::vector<octomath::Vector3>& cube_template)
{
    cube_template.clear();
    cube_template.reserve(24);

    cube_template.push_back(octomath::Vector3( 1, 1,-1));
    cube_template.push_back(octomath::Vector3( 1,-1,-1));
    cube_template.push_back(octomath::Vector3( 1, 1,-1));
    cube_template.push_back(octomath::Vector3(-1, 1,-1));
    cube_template.push_back(octomath::Vector3( 1, 1,-1));
    cube_template.push_back(octomath::Vector3( 1, 1, 1));

    cube_template.push_back(octomath::Vector3(-1, 1,-1));
    cube_template.push_back(octomath::Vector3(-1,-1,-1));
    cube_template.push_back(octomath::Vector3( 1, 1, 1));
    cube_template.push_back(octomath::Vector3(-1, 1, 1));
    cube_template.push_back(octomath::Vector3( 1,-1,-1));
    cube_template.push_back(octomath::Vector3( 1,-1, 1));

    cube_template.push_back(octomath::Vector3(-1, 1, 1));
    cube_template.push_back(octomath::Vector3(-1,-1, 1));
    cube_template.push_back(octomath::Vector3( 1,-1, 1));
    cube_template.push_back(octomath::Vector3(-1,-1, 1));
    cube_template.push_back(octomath::Vector3(-1,-1,-1));
    cube_template.push_back(octomath::Vector3(-1,-1, 1));

    cube_template.push_back(octomath::Vector3( 1, 1, 1));
    cube_template.push_back(octomath::Vector3( 1,-1, 1));
    cube_template.push_back(octomath::Vector3( 1,-1,-1));
    cube_template.push_back(octomath::Vector3(-1,-1,-1));
    cube_template.push_back(octomath::Vector3(-1, 1,-1));
    cube_template.push_back(octomath::Vector3(-1, 1, 1));
}

void OcTreeDrawer::setVertexDataFromOctomathVector(OGLVertexData& vertex, const octomath::Vector3& vec) {
    vertex.x = vec.x();
    vertex.y = vec.y();
    vertex.z = vec.z();
}

void OcTreeDrawer::generateCube(const octomap::OcTreeVolume& v,
                              const std::vector<octomath::Vector3>& cube_template,
                              VoxelData& voxel_data)
{
    std::vector<OGLTriangleData>& triangle_data = voxel_data.triangle_data;

    // epsilon to be substracted from cube size so that neighboring planes don't overlap
    // seems to introduce strange artifacts nevertheless...
    double eps = 1e-5;

    octomath::Vector3 p;

    double half_cube_size = GLfloat(v.second / 2.0 -eps);

    // Cube surfaces are in gl_array in order: back, front, top, down, left, right.
    // Arrays are filled in parallel (increasing i for all at once)
    // One color array for all surfaces is filled when requested

    OGLVertexData vertex1;
    OGLVertexData vertex2;
    OGLVertexData vertex3;

    for (size_t i = 0; i < 24; i += 3) {
        setVertexDataFromOctomathVector(vertex1, v.first + cube_template[i + 0] * half_cube_size);
        setVertexDataFromOctomathVector(vertex1, v.first + cube_template[i + 1] * half_cube_size);
        setVertexDataFromOctomathVector(vertex1, v.first + cube_template[i + 2] * half_cube_size);
        triangle_data.emplace_back(vertex1, vertex2, vertex3);
//        setCubeColorHeightmap(v, triangle_data.back());
    }
}

void OcTreeDrawer::generateCube(const octomap::OcTreeVolume& v,
                              const std::vector<octomath::Vector3>& cube_template,
                              VoxelArrays& voxelArrays)
{
    std::vector<std::vector<GLfloat>>& cubeArrays = voxelArrays.cubeArrays;

    // epsilon to be substracted from cube size so that neighboring planes don't overlap
    // seems to introduce strange artifacts nevertheless...
    double eps = 1e-5;

    octomath::Vector3 p;

    double half_cube_size = GLfloat(v.second /2.0 -eps);
    // Cube surfaces are in gl_array in order: back, front, top, down, left, right.
    // Arrays are filled in parallel (increasing i for all at once)
    // One color array for all surfaces is filled when requested

    p = v.first + cube_template[0] * half_cube_size;
    cubeArrays[0].push_back(p.x());
    cubeArrays[0].push_back(p.y());
    cubeArrays[0].push_back(p.z());

    p = v.first + cube_template[1] * half_cube_size;
    cubeArrays[1].push_back(p.x());
    cubeArrays[1].push_back(p.y());
    cubeArrays[1].push_back(p.z());

    p = v.first + cube_template[2] * half_cube_size;
    cubeArrays[2].push_back(p.x());
    cubeArrays[2].push_back(p.y());
    cubeArrays[2].push_back(p.z());

    p = v.first + cube_template[3] * half_cube_size;
    cubeArrays[3].push_back(p.x());
    cubeArrays[3].push_back(p.y());
    cubeArrays[3].push_back(p.z());

    p = v.first + cube_template[4] * half_cube_size;
    cubeArrays[4].push_back(p.x());
    cubeArrays[4].push_back(p.y());
    cubeArrays[4].push_back(p.z());

    p = v.first + cube_template[5] * half_cube_size;
    cubeArrays[5].push_back(p.x());
    cubeArrays[5].push_back(p.y());
    cubeArrays[5].push_back(p.z());

    p = v.first + cube_template[6] * half_cube_size;
    cubeArrays[0].push_back(p.x());
    cubeArrays[0].push_back(p.y());
    cubeArrays[0].push_back(p.z());

    p = v.first + cube_template[7] * half_cube_size;
    cubeArrays[1].push_back(p.x());
    cubeArrays[1].push_back(p.y());
    cubeArrays[1].push_back(p.z());

    p = v.first + cube_template[8] * half_cube_size;
    cubeArrays[2].push_back(p.x());
    cubeArrays[2].push_back(p.y());
    cubeArrays[2].push_back(p.z());

    p = v.first + cube_template[9] * half_cube_size;
    cubeArrays[3].push_back(p.x());
    cubeArrays[3].push_back(p.y());
    cubeArrays[3].push_back(p.z());

    p = v.first + cube_template[10] * half_cube_size;
    cubeArrays[4].push_back(p.x());
    cubeArrays[4].push_back(p.y());
    cubeArrays[4].push_back(p.z());

    p = v.first + cube_template[11] * half_cube_size;
    cubeArrays[5].push_back(p.x());
    cubeArrays[5].push_back(p.y());
    cubeArrays[5].push_back(p.z());

    p = v.first + cube_template[12] * half_cube_size;
    cubeArrays[0].push_back(p.x());
    cubeArrays[0].push_back(p.y());
    cubeArrays[0].push_back(p.z());

    p = v.first + cube_template[13] * half_cube_size;
    cubeArrays[1].push_back(p.x());
    cubeArrays[1].push_back(p.y());
    cubeArrays[1].push_back(p.z());

    p = v.first + cube_template[14] * half_cube_size;
    cubeArrays[2].push_back(p.x());
    cubeArrays[2].push_back(p.y());
    cubeArrays[2].push_back(p.z());

    p = v.first + cube_template[15] * half_cube_size;
    cubeArrays[3].push_back(p.x());
    cubeArrays[3].push_back(p.y());
    cubeArrays[3].push_back(p.z());

    p = v.first + cube_template[16] * half_cube_size;
    cubeArrays[4].push_back(p.x());
    cubeArrays[4].push_back(p.y());
    cubeArrays[4].push_back(p.z());

    p = v.first + cube_template[17] * half_cube_size;
    cubeArrays[5].push_back(p.x());
    cubeArrays[5].push_back(p.y());
    cubeArrays[5].push_back(p.z());

    p = v.first + cube_template[18] * half_cube_size;
    cubeArrays[0].push_back(p.x());
    cubeArrays[0].push_back(p.y());
    cubeArrays[0].push_back(p.z());

    p = v.first + cube_template[19] * half_cube_size;
    cubeArrays[1].push_back(p.x());
    cubeArrays[1].push_back(p.y());
    cubeArrays[1].push_back(p.z());

    p = v.first + cube_template[20] * half_cube_size;
    cubeArrays[2].push_back(p.x());
    cubeArrays[2].push_back(p.y());
    cubeArrays[2].push_back(p.z());

    p = v.first + cube_template[21] * half_cube_size;
    cubeArrays[3].push_back(p.x());
    cubeArrays[3].push_back(p.y());
    cubeArrays[3].push_back(p.z());

    p = v.first + cube_template[22] * half_cube_size;
    cubeArrays[4].push_back(p.x());
    cubeArrays[4].push_back(p.y());
    cubeArrays[4].push_back(p.z());

    p = v.first + cube_template[23] * half_cube_size;
    cubeArrays[5].push_back(p.x());
    cubeArrays[5].push_back(p.y());
    cubeArrays[5].push_back(p.z());
}

unsigned int OcTreeDrawer::generateCube(const octomap::OcTreeVolume& v,
                                      const std::vector<octomath::Vector3>& cube_template,
                                      const unsigned int& current_array_idx,
                                      GLfloat*** glArray)
{
    // epsilon to be substracted from cube size so that neighboring planes don't overlap
    // seems to introduce strange artifacts nevertheless...
    double eps = 1e-5;

    octomath::Vector3 p;

    double half_cube_size = GLfloat(v.second /2.0 -eps);
    unsigned int i = current_array_idx;
    // Cube surfaces are in gl_array in order: back, front, top, down, left, right.
    // Arrays are filled in parallel (increasing i for all at once)
    // One color array for all surfaces is filled when requested

    p = v.first + cube_template[0] * half_cube_size;
    (*glArray)[0][i]   = p.x();
    (*glArray)[0][i+1] = p.y();
    (*glArray)[0][i+2] = p.z();

    p = v.first + cube_template[1] * half_cube_size;
    (*glArray)[1][i]   = p.x();
    (*glArray)[1][i+1] = p.y();
    (*glArray)[1][i+2] = p.z();

    p = v.first + cube_template[2] * half_cube_size;
    (*glArray)[2][i]   = p.x();
    (*glArray)[2][i+1] = p.y();
    (*glArray)[2][i+2] = p.z();

    p = v.first + cube_template[3] * half_cube_size;
    (*glArray)[3][i]   = p.x();
    (*glArray)[3][i+1] = p.y();
    (*glArray)[3][i+2] = p.z();

    p = v.first + cube_template[4] * half_cube_size;
    (*glArray)[4][i]   = p.x();
    (*glArray)[4][i+1] = p.y();
    (*glArray)[4][i+2] = p.z();

    p = v.first + cube_template[5] * half_cube_size;
    (*glArray)[5][i]   = p.x();
    (*glArray)[5][i+1] = p.y();
    (*glArray)[5][i+2] = p.z();
    i+= 3;  //-------------------

    p = v.first + cube_template[6] * half_cube_size;
    (*glArray)[0][i]   = p.x();
    (*glArray)[0][i+1] = p.y();
    (*glArray)[0][i+2] = p.z();

    p = v.first + cube_template[7] * half_cube_size;
    (*glArray)[1][i]   = p.x();
    (*glArray)[1][i+1] = p.y();
    (*glArray)[1][i+2] = p.z();

    p = v.first + cube_template[8] * half_cube_size;
    (*glArray)[2][i]   = p.x();
    (*glArray)[2][i+1] = p.y();
    (*glArray)[2][i+2] = p.z();

    p = v.first + cube_template[9] * half_cube_size;
    (*glArray)[3][i]   = p.x();
    (*glArray)[3][i+1] = p.y();
    (*glArray)[3][i+2] = p.z();

    p = v.first + cube_template[10] * half_cube_size;
    (*glArray)[4][i]   = p.x();
    (*glArray)[4][i+1] = p.y();
    (*glArray)[4][i+2] = p.z();

    p = v.first + cube_template[11] * half_cube_size;
    (*glArray)[5][i]   = p.x();
    (*glArray)[5][i+1] = p.y();
    (*glArray)[5][i+2] = p.z();
    i+= 3;  //-------------------

    p = v.first + cube_template[12] * half_cube_size;
    (*glArray)[0][i]   = p.x();
    (*glArray)[0][i+1] = p.y();
    (*glArray)[0][i+2] = p.z();

    p = v.first + cube_template[13] * half_cube_size;
    (*glArray)[1][i]   = p.x();
    (*glArray)[1][i+1] = p.y();
    (*glArray)[1][i+2] = p.z();

    p = v.first + cube_template[14] * half_cube_size;
    (*glArray)[2][i]   = p.x();
    (*glArray)[2][i+1] = p.y();
    (*glArray)[2][i+2] = p.z();

    p = v.first + cube_template[15] * half_cube_size;
    (*glArray)[3][i]   = p.x();
    (*glArray)[3][i+1] = p.y();
    (*glArray)[3][i+2] = p.z();

    p = v.first + cube_template[16] * half_cube_size;
    (*glArray)[4][i]   = p.x();
    (*glArray)[4][i+1] = p.y();
    (*glArray)[4][i+2] = p.z();

    p = v.first + cube_template[17] * half_cube_size;
    (*glArray)[5][i]   = p.x();
    (*glArray)[5][i+1] = p.y();
    (*glArray)[5][i+2] = p.z();
    i+= 3;  //-------------------

    p = v.first + cube_template[18] * half_cube_size;
    (*glArray)[0][i]   = p.x();
    (*glArray)[0][i+1] = p.y();
    (*glArray)[0][i+2] = p.z();

    p = v.first + cube_template[19] * half_cube_size;
    (*glArray)[1][i]   = p.x();
    (*glArray)[1][i+1] = p.y();
    (*glArray)[1][i+2] = p.z();

    p = v.first + cube_template[20] * half_cube_size;
    (*glArray)[2][i]   = p.x();
    (*glArray)[2][i+1] = p.y();
    (*glArray)[2][i+2] = p.z();

    p = v.first + cube_template[21] * half_cube_size;
    (*glArray)[3][i]   = p.x();
    (*glArray)[3][i+1] = p.y();
    (*glArray)[3][i+2] = p.z();

    p = v.first + cube_template[22] * half_cube_size;
    (*glArray)[4][i]   = p.x();
    (*glArray)[4][i+1] = p.y();
    (*glArray)[4][i+2] = p.z();

    p = v.first + cube_template[23] * half_cube_size;
    (*glArray)[5][i]   = p.x();
    (*glArray)[5][i+1] = p.y();
    (*glArray)[5][i+2] = p.z();
    i += 3;  //-------------------

    return i; // updated array idx
}


void OcTreeDrawer::setCubeColorHeightmap(const octomap::OcTreeVolume& v, VoxelData& voxel_data) {
    // TODO
}

unsigned int OcTreeDrawer::setCubeColorHeightmap(const octomap::OcTreeVolume& v,
                                      const unsigned int current_array_idx,
                                      GLfloat** glColorArray)
{
    if (glColorArray == NULL) return current_array_idx;

    unsigned int colorIdx = current_array_idx;
    // color for all 4 vertices (same height)
    for (int k = 0; k < 4; ++k) {
      if (m_colorMode == CM_GRAY_HEIGHT)
        SceneObject::heightMapGray(v.first.z(), *glColorArray + colorIdx);  // sets r,g,b
      else
        SceneObject::heightMapColor(v.first.z(), *glColorArray + colorIdx);   // sets r,g,b
      // set Alpha value:
      (*glColorArray)[colorIdx + 3] = m_alphaOccupied;
      colorIdx += 4;
    }
    return colorIdx;
}

void OcTreeDrawer::setCubeColorHeightmap(const octomap::OcTreeVolume& v, std::vector<GLfloat>& colors)
{
    // color for all 4 vertices (same height)
    for (int k = 0; k < 4; ++k) {
        colors.resize(colors.size() + 4);
        if (m_colorMode == CM_GRAY_HEIGHT) {
            SceneObject::heightMapGray(v.first.z(), &colors[colors.size() - 4]);  // sets r,g,b
        }
        else {
            SceneObject::heightMapColor(v.first.z(), &colors[colors.size() - 4]);   // sets r,g,b
        }
        // set Alpha value:
        colors[colors.size() - 1] = m_alphaOccupied;
    }
}

void OcTreeDrawer::clear()
{
}

void OcTreeDrawer::drawVoxelsAboveThreshold(double occupancy_threshold, bool draw_below_threshold) const
{
    if (m_drawSingleBin) {
        // Find closest lower bin
        double closest_key = 0;
        double closest_dist = std::numeric_limits<double>::infinity();
        for (const auto& entry : voxelArraysByOccupancy_) {
            double dist = std::abs(m_occupancyThreshold - entry.first);
            if (dist < closest_dist) {
                closest_key = entry.first;
                closest_dist = dist;
            }
        }
//        std::cout << "Only drawing bin " << closest_key << std::endl;
        drawCubes(voxelArraysByOccupancy_.at(closest_key));
    }
    else {
        for (const auto& entry : voxelArraysByOccupancy_) {
            if (!draw_below_threshold && entry.first >= occupancy_threshold) {
                drawCubes(entry.second);
            }
            else if (draw_below_threshold && entry.first < occupancy_threshold) {
                drawCubes(entry.second);
            }
        }
    }
}

void OcTreeDrawer::drawCubes(const VoxelArrays& voxelArrays) const
{
    if (voxelArrays.cubeArrays.empty()) {
        std::cerr << "Warning: GLfloat array to draw cubes appears to be empty, nothing drawn.\n";
        return;
    }

    // save current color
    GLfloat* curcol = new GLfloat[4];
    glGetFloatv(GL_CURRENT_COLOR, curcol);

    // enable color pointer when heightColorMode is enabled:
    if (!voxelArrays.colorArray.empty()) {
        glEnableClientState(GL_COLOR_ARRAY);
        glColorPointer(4, GL_FLOAT, 0, voxelArrays.colorArray.data());
    }

    // top surfaces:
    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertexPointer(3, GL_FLOAT, 0, voxelArrays.cubeArrays[0].data());
    glDrawArrays(GL_QUADS, 0, voxelArrays.cubeArrays[0].size() / 3);
    // bottom surfaces:
    glNormal3f(0.0f, -1.0f, 0.0f);
    glVertexPointer(3, GL_FLOAT, 0, voxelArrays.cubeArrays[1].data());
    glDrawArrays(GL_QUADS, 0, voxelArrays.cubeArrays[1].size() / 3);
    // right surfaces:
    glNormal3f(1.0f, 0.0f, 0.0f);
    glVertexPointer(3, GL_FLOAT, 0, voxelArrays.cubeArrays[2].data());
    glDrawArrays(GL_QUADS, 0, voxelArrays.cubeArrays[2].size() / 3);
    // left surfaces:
    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertexPointer(3, GL_FLOAT, 0, voxelArrays.cubeArrays[3].data());
    glDrawArrays(GL_QUADS, 0, voxelArrays.cubeArrays[3].size() / 3);
    // back surfaces:
    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertexPointer(3, GL_FLOAT, 0, voxelArrays.cubeArrays[4].data());
    glDrawArrays(GL_QUADS, 0, voxelArrays.cubeArrays[4].size() / 3);
    // front surfaces:
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertexPointer(3, GL_FLOAT, 0, voxelArrays.cubeArrays[5].data());
    glDrawArrays(GL_QUADS, 0, voxelArrays.cubeArrays[5].size() / 3);

    if (!voxelArrays.colorArray.empty()) {
        glDisableClientState(GL_COLOR_ARRAY);
    }

    // reset color
    glColor4fv(curcol);
    delete[] curcol;
}

void OcTreeDrawer::setOrigin(octomap::pose6d t)
{
    origin_ = t;
    std::cout << "OcTreeDrawer: setting new global origin: " << t << std::endl;

    octomap::pose6d relative_transform = origin_ * initial_origin_.inv();

    std::cout << "origin        : " << origin_ << std::endl;
    std::cout << "inv init orig : " << initial_origin_.inv() << std::endl;
    std::cout << "relative trans: " << relative_transform << std::endl;
}

void OcTreeDrawer::drawAxes() const
{
    octomap::pose6d relative_transform = origin_ * initial_origin_.inv();

    glPushMatrix();

    float length = 0.15f;

    GLboolean lighting, colorMaterial;
    glGetBooleanv(GL_LIGHTING, &lighting);
    glGetBooleanv(GL_COLOR_MATERIAL, &colorMaterial);

    glDisable(GL_COLOR_MATERIAL);

    double angle= 2 * acos(initial_origin_.rot().u());
    double scale = sqrt (initial_origin_.rot().x()*initial_origin_.rot().x()
                         + initial_origin_.rot().y()*initial_origin_.rot().y()
                         + initial_origin_.rot().z()*initial_origin_.rot().z());
    double ax= initial_origin_.rot().x() / scale;
    double ay= initial_origin_.rot().y() / scale;
    double az= initial_origin_.rot().z() / scale;

    if (angle > 0) glRotatef(OTD_RAD2DEG*angle, ax, ay, az);

    float color[4];
    color[0] = 0.7f;  color[1] = 0.7f;  color[2] = 1.0f;  color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    QGLViewer::drawArrow(length, 0.01*length);

    color[0] = 1.0f;  color[1] = 0.7f;  color[2] = 0.7f;  color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    glPushMatrix();
    glRotatef(90.0, 0.0, 1.0, 0.0);
    QGLViewer::drawArrow(length, 0.01*length);
    glPopMatrix();

    color[0] = 0.7f;  color[1] = 1.0f;  color[2] = 0.7f;  color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    glPushMatrix();
    glRotatef(-90.0, 1.0, 0.0, 0.0);
    QGLViewer::drawArrow(length, 0.01*length);
    glPopMatrix();

    glTranslatef(relative_transform.trans().x(), relative_transform.trans().y(), relative_transform.trans().z());

    if (colorMaterial)
      glEnable(GL_COLOR_MATERIAL);
    if (!lighting)
      glDisable(GL_LIGHTING);

    glPopMatrix();
}
