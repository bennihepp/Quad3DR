//==================================================
// octree_drawer.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 7, 2016
//==================================================

#include "octree_drawer.h"
#include <qglviewer.h>
#include <ait/common.h>
#include <ait/utilities.h>

#define OTD_RAD2DEG 57.2957795

using AbstractOcTree = octomap::AbstractOcTree;
using OcTree = octomap::OcTree;
using pose6d = octomap::pose6d;
using OcTreeVolume = octomap::OcTreeVolume;

OcTreeDrawer::OcTreeDrawer() : SceneObjectAdapted(),
                             octree_(nullptr), m_occupancyThreshold(0.5), m_drawFreeVoxels(false), m_alphaOccupied(1.0),
                             m_drawSingleBin(false), render_tree_depth_(20)
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

void OcTreeDrawer::draw(const QMatrix4x4& pvm_matrix, const QMatrix4x4& view_matrix, const QMatrix4x4& model_matrix)
{
    drawVoxelsAboveThreshold(pvm_matrix, view_matrix, model_matrix, m_occupancyThreshold, m_drawFreeVoxels);
    m_update = false;
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

void OcTreeDrawer::setAlphaOccupied(double alpha) {
    m_update = true;
    m_alphaOccupied = alpha;
    for (auto& entry : voxel_drawer_map_) {
      entry.second.overrideAlpha(m_alphaOccupied);
    }
//    updateVoxelColorHeightmap();

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
  ait::Timer timer;
  updateVoxelArrays();
  timer.printTiming("Updating voxel arrays");
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

  double minX, minY, minZ, maxX, maxY, maxZ;
  octree_->getMetricMin(minX, minY, minZ);
  octree_->getMetricMax(maxX, maxY, maxZ);

  // set min/max Z for color height map
  m_zMin = minZ;
  m_zMax = maxZ;

  std::unordered_map<double, std::vector<OGLVoxelData>> voxel_data;
  std::unordered_map<double, std::vector<OGLColorData>> color_data;
  for (double occupancy_bin : occupancy_bins_) {
    voxel_data.emplace(occupancy_bin, std::vector<OGLVoxelData>());
    color_data.emplace(occupancy_bin, std::vector<OGLColorData>());
  }

  for(OcTree::tree_iterator it = octree_->begin_tree(render_tree_depth_), end=octree_->end_tree(); it!= end; ++it) {
    if (it.isLeaf()) {
      const octomap::point3d& voxel_position = it.getCoordinate();
      float voxel_size = it.getSize();

      double occupancy = it->getOccupancy();
      double occupancy_bin = findOccupancyBin(occupancy);

      OGLVertexData vertex(voxel_position.x(), voxel_position.y(), voxel_position.z());
      voxel_data.at(occupancy_bin).emplace_back(vertex, voxel_size);
      OGLColorData color;
      setCubeColorHeightmap(voxel_position, color);
      color_data.at(occupancy_bin).emplace_back(color);
    }
  }

  voxel_drawer_map_.clear();
  for (double occupancy_bin : occupancy_bins_) {
    const auto result = voxel_drawer_map_.emplace(std::piecewise_construct, std::make_tuple(occupancy_bin), std::make_tuple());
    VoxelDrawer& voxel_drawer = result.first->second;
    voxel_drawer.init();
//    std::cout << "Uploading " << voxel_data.at(occupancy_bin).size() << " voxels" << std::endl;
    voxel_drawer.upload(voxel_data.at(occupancy_bin), color_data.at(occupancy_bin));
  }

  for (const auto& entry : voxel_drawer_map_) {
    std::cout << "Voxels in bin " << entry.first << ": " << entry.second.numOfVoxels() << std::endl;
  }
}

void OcTreeDrawer::setVertexDataFromOctomathVector(OGLVertexDataRGBA& vertex, const octomath::Vector3& vec) {
    vertex.x = vec.x();
    vertex.y = vec.y();
    vertex.z = vec.z();
}

void OcTreeDrawer::setCubeColorHeightmap(const octomap::point3d& position, OGLColorData& color) {
  if (m_colorMode == CM_GRAY_HEIGHT) {
    SceneObjectAdapted::heightMapGray(position.z(), reinterpret_cast<float*>(&color));
  }
  else {
    SceneObjectAdapted::heightMapColor(position.z(), reinterpret_cast<float*>(&color));
  }
  // set Alpha value:
  color.a = m_alphaOccupied;
}

void OcTreeDrawer::clear()
{
}

void OcTreeDrawer::drawVoxelsAboveThreshold(const QMatrix4x4& pvm_matrix, const QMatrix4x4& view_matrix, const QMatrix4x4& model_matrix,
    double occupancy_threshold, bool draw_below_threshold) {
  if (m_drawSingleBin) {
    // Find closest lower bin
    double closest_key = 0;
    double closest_dist = std::numeric_limits<double>::infinity();
    for (const auto& entry : voxel_drawer_map_) {
      double dist = std::abs(m_occupancyThreshold - entry.first);
      if (dist < closest_dist) {
        closest_key = entry.first;
        closest_dist = dist;
      }
    }
//        std::cout << "Only drawing bin " << closest_key << std::endl;
    voxel_drawer_map_.at(closest_key).draw(pvm_matrix, view_matrix, model_matrix);
  }
  else {
    for (auto& entry : voxel_drawer_map_) {
      if (!draw_below_threshold && entry.first >= occupancy_threshold) {
        entry.second.draw(pvm_matrix, view_matrix, model_matrix);
      }
      else if (draw_below_threshold && entry.first < occupancy_threshold) {
        entry.second.draw(pvm_matrix, view_matrix, model_matrix);
      }
    }
  }
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
