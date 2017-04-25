//==================================================
// octree_drawer.cpp
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 19.04.17
//==================================================

#include "octree_drawer.h"
#include <bh/common.h>
#include <bh/utilities.h>

using pose6d = octomap::pose6d;
using OcTreeVolume = octomap::OcTreeVolume;

namespace rendering {

OcTreeDrawer::OcTreeDrawer()
        : octree_(nullptr),
          draw_octree_(true),
          color_flags_(VoxelDrawer::ColorFlags::Fixed),
          draw_free_voxels_(false), alpha_override_(1.0),
          occupancy_threshold_(0.5),
          observation_count_threshold_(1),
          render_tree_depth_(size_t(-1)),
          min_occupancy_(0), max_occupancy_(1),
          min_observations_(0), max_observations_(std::numeric_limits<uint32_t>::max()),
          low_observation_count_(0), high_observation_count_(std::numeric_limits<uint32_t>::max()),
          min_voxel_size_(0), max_voxel_size_(std::numeric_limits<FloatType>::max()),
          min_weight_(0), max_weight_(std::numeric_limits<FloatType>::max()),
          low_weight_(0), high_weight_(std::numeric_limits<FloatType>::max()),
          min_information_(0), max_information_(std::numeric_limits<FloatType>::max()) {}

OcTreeDrawer::~OcTreeDrawer() {}

const VoxelDrawer& OcTreeDrawer::getVoxelDrawer() const {
  return voxel_drawer_;
}

VoxelDrawer& OcTreeDrawer::getVoxelDrawer() {
  return voxel_drawer_;
}

void OcTreeDrawer::draw(const QMatrix4x4& pvm_matrix, const QMatrix4x4& vm_matrix) {
  if (draw_octree_) {
    drawVoxels(pvm_matrix, vm_matrix);
  }
}

void OcTreeDrawer::setDrawOctree(bool draw_octree) {
  draw_octree_ = draw_octree;
}

void OcTreeDrawer::setColorFlags(uint32_t color_flags_uint) {
  color_flags_ = static_cast<VoxelDrawer::ColorFlags>(color_flags_uint);
  voxel_drawer_.setColorFlags(color_flags_);
}

void OcTreeDrawer::setAlphaOccupied(FloatType alpha) {
  alpha_override_ = alpha;
  voxel_drawer_.overrideAlpha(alpha_override_);

}

void OcTreeDrawer::setMinOccupancy(FloatType min_occupancy) {
  min_occupancy_ = min_occupancy;
  voxel_drawer_.setMinOccupancy(min_occupancy_);
}

void OcTreeDrawer::setMaxOccupancy(FloatType max_occupancy) {
  max_occupancy_ = max_occupancy;
  voxel_drawer_.setMaxOccupancy(max_occupancy_);
}

void OcTreeDrawer::setMinObservations(uint32_t min_observations) {
  min_observations_ = min_observations;
  voxel_drawer_.setMinObservations(min_observations_);
}

void OcTreeDrawer::setMaxObservations(uint32_t max_observations) {
  max_observations_ = max_observations;
  voxel_drawer_.setMaxObservations(max_observations_);
}

void OcTreeDrawer::setMinVoxelSize(FloatType min_voxel_size) {
  min_voxel_size_ = min_voxel_size;
  voxel_drawer_.setMinVoxelSize(min_voxel_size_);
}

void OcTreeDrawer::setMaxVoxelSize(FloatType max_voxel_size) {
  max_voxel_size_ = max_voxel_size;
  voxel_drawer_.setMaxVoxelSize(max_voxel_size_);
}

void OcTreeDrawer::setMinWeight(FloatType min_weight) {
  min_weight_ = min_weight;
  voxel_drawer_.setMinWeight(min_weight_);
}

void OcTreeDrawer::setMaxWeight(FloatType max_weight) {
  max_weight_ = max_weight;
  voxel_drawer_.setMaxWeight(max_weight_);
}

void OcTreeDrawer::setWeightRange(const FloatType low_weight, const FloatType high_weight) {
  voxel_drawer_.setWeightRange(low_weight, high_weight);
}

void OcTreeDrawer::setMinInformation(FloatType min_information) {
  min_information_ = min_information;
  voxel_drawer_.setMinInformation(min_information_);
}

void OcTreeDrawer::setMaxInformation(FloatType max_information) {
  max_information_ = max_information;
  voxel_drawer_.setMaxInformation(max_information_);
}

void OcTreeDrawer::setInformationRange(const FloatType low_information, const FloatType high_information) {
  voxel_drawer_.setInformationRange(low_information, high_information);
}

void OcTreeDrawer::clear() {
  voxel_drawer_.clear();
}

void OcTreeDrawer::setOctree(
        const viewpoint_planner::OccupancyMapType* octree,
        const FloatType occupancy_threshold,
        const size_t observation_count_threshold,
        const size_t render_tree_depth) {
  octree_ = octree;
  occupancy_threshold_ = occupancy_threshold;
  observation_count_threshold_ = observation_count_threshold;
  render_tree_depth_ = render_tree_depth;
  updateVoxelsFromOctree();
}

void OcTreeDrawer::updateVoxelsFromOctree() {
  bh::Timer timer;
  updateVoxelData();
  timer.printTiming("Updating voxel arrays");
}

void OcTreeDrawer::updateVoxelData() {
  BH_ASSERT_STR(octree_ != nullptr, "Octree was not initialized");

  std::vector<OGLVoxelData> voxel_data;
  std::vector<OGLColorData> color_data;
  std::vector<OGLVoxelInfoData> info_data;

  // Ranges of weight and observation counts
  low_weight_ = std::numeric_limits<FloatType>::max();
  high_weight_ = std::numeric_limits<FloatType>::lowest();
  low_observation_count_ = std::numeric_limits<uint32_t>::max();
  high_observation_count_ = std::numeric_limits<uint32_t>::lowest();
  // Range of z coordinate height color map
  FloatType min_z = std::numeric_limits<FloatType>::infinity();
  FloatType max_z = -std::numeric_limits<FloatType>::infinity();

  for(viewpoint_planner::OccupancyMapType::tree_iterator it = octree_->begin_tree(render_tree_depth_), end=octree_->end_tree(); it!= end; ++it) {
    if (it.isLeaf()) {
      const FloatType occupancy = it->getOccupancy();
      if (occupancy < occupancy_threshold_) {
        continue;
      }

      const size_t observation_count = it->getObservationCount();
      if (observation_count < observation_count_threshold_) {
        continue;
      }

      const octomap::point3d& voxel_position = it.getCoordinate();
      const FloatType voxel_size = it.getSize();
      const FloatType weight = it->getWeight();

      OGLVertexData vertex(voxel_position.x(), voxel_position.y(), voxel_position.z());
      voxel_data.emplace_back(vertex, voxel_size);

      info_data.emplace_back(occupancy, it->getObservationCount(), weight);

      if (voxel_position.z() < min_z) {
        min_z = voxel_position.z();
      }
      if (voxel_position.z() > max_z) {
        max_z = voxel_position.z();
      }
      low_weight_ = std::min(weight, low_weight_);
      high_weight_ = std::max(weight, high_weight_);
      low_observation_count_ = std::min(it->getObservationCount(), low_observation_count_);
      high_observation_count_ = std::max(it->getObservationCount(), high_observation_count_);
    }
  }
  // Compute and output observation and weight ranges for each bin
  for (const OGLVoxelData& voxel : voxel_data) {
    OGLColorData color = getVoxelColorData(voxel, min_z, max_z);
    color_data.emplace_back(color);
  }
  FloatType weight_low = std::numeric_limits<FloatType>::max();
  FloatType weight_high = std::numeric_limits<FloatType>::lowest();
  FloatType observation_low = std::numeric_limits<FloatType>::max();
  FloatType observation_high = std::numeric_limits<FloatType>::lowest();
  for (const OGLVoxelInfoData& info : info_data) {
    weight_low = std::min(info.weight, weight_low);
    weight_high = std::max(info.weight, weight_high);
    observation_low = std::min(info.observation_count, observation_low);
    observation_high = std::max(info.observation_count, observation_high);
  }
//    std::cout << "Weight range for bin " << occupancy_bin << ": [" << weight_low << ", " << weight_high << "]" << std::endl;
//    std::cout << "Observation range for bin " << occupancy_bin << ": [" << observation_low << ", " << observation_high << "]" << std::endl;

  std::cout << "Occupancy map z range: " << "[" << min_z << ", " << max_z << "]" << std::endl;
  std::cout << "Weight range: [" << low_weight_ << ", " << high_weight_ << "]" << std::endl;
  std::cout << "Observation count range: [" << low_observation_count_ << ", " << high_observation_count_ << "]" << std::endl;

  voxel_drawer_.clear();
  voxel_drawer_.init();
  configVoxelDrawer();
//    std::cout << "Uploading " << voxel_data.at(occupancy_bin).size() << " voxels" << std::endl;
  voxel_drawer_.upload(voxel_data, color_data, info_data);
  voxel_drawer_.setWeightRange(0, 1);
  voxel_drawer_.setInformationRange(0, 1);

  std::cout << "Uploaded " << voxel_drawer_.numOfVoxels() << " voxels" << std::endl;
}

void OcTreeDrawer::configVoxelDrawer() {
  voxel_drawer_.setColorFlags(color_flags_);
  voxel_drawer_.setMinObservations(min_observations_);
  voxel_drawer_.setMaxObservations(max_observations_);
  voxel_drawer_.setMinOccupancy(min_occupancy_);
  voxel_drawer_.setMaxOccupancy(max_occupancy_);
  voxel_drawer_.setMinVoxelSize(min_voxel_size_);
  voxel_drawer_.setMaxVoxelSize(max_voxel_size_);
  voxel_drawer_.setMinWeight(min_weight_);
  voxel_drawer_.setMaxWeight(max_weight_);
  voxel_drawer_.setWeightRange(low_weight_, high_weight_);
  voxel_drawer_.setObservationsRange(low_observation_count_, high_observation_count_);
}

void OcTreeDrawer::setVertexDataFromOctomathVector(OGLVertexDataRGBA& vertex, const octomath::Vector3& vec) {
  vertex.x = vec.x();
  vertex.y = vec.y();
  vertex.z = vec.z();
}

OGLColorData OcTreeDrawer::getVoxelColorData(const OGLVoxelData& voxel_data, FloatType min_z, FloatType max_z) const {
  FloatType h = voxel_data.vertex.z;

  if (min_z >= max_z)
    h = 0.5f;
  else{
    h = (1.0f - std::min(std::max((h-min_z)/ (max_z - min_z), 0.0f), 1.0f)) * 0.8f;
  }

  // blend over HSV-values (more colors)
  FloatType r, g, b;
  FloatType s = 1.0f;
  FloatType v = 1.0f;

  h -= floor(h);
  h *= 6;
  int i;
  FloatType m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      r = v; g = n; b = m;
      break;
    case 1:
      r = n; g = v; b = m;
      break;
    case 2:
      r = m; g = v; b = n;
      break;
    case 3:
      r = m; g = n; b = v;
      break;
    case 4:
      r = n; g = m; b = v;
      break;
    case 5:
      r = v; g = m; b = n;
      break;
    default:
      r = 1; g = 0.5f; b = 0.5f;
      break;
  }

  return OGLColorData(r, g, b, alpha_override_);
}

void OcTreeDrawer::drawVoxels(const QMatrix4x4& pvm_matrix, const QMatrix4x4& vm_matrix) {
  voxel_drawer_.draw(pvm_matrix, vm_matrix);
}

}
