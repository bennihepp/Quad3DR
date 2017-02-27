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

using pose6d = octomap::pose6d;
using OcTreeVolume = octomap::OcTreeVolume;

OcTreeDrawer::OcTreeDrawer()
  : octree_(nullptr),
    draw_octree_(true),
    draw_raycast_(false),
    occupancy_threshold_(0.5),
    color_flags_(VoxelDrawer::ColorFlags::Fixed),
    draw_free_voxels_(false), alpha_override_(1.0),
    draw_single_bin_(false), render_tree_depth_(20),
    render_observation_threshold_(1),
    min_occupancy_(0), max_occupancy_(1),
    min_observations_(0), max_observations_(std::numeric_limits<uint32_t>::max()),
    low_observation_count_(0), high_observation_count_(std::numeric_limits<uint32_t>::max()),
    min_voxel_size_(0), max_voxel_size_(std::numeric_limits<FloatType>::max()),
    min_weight_(0), max_weight_(std::numeric_limits<FloatType>::max()),
    low_weight_(0), high_weight_(std::numeric_limits<FloatType>::max()),
    min_information_(0), max_information_(std::numeric_limits<FloatType>::max()) {
  // origin and movement
  initial_origin_ = octomap::pose6d(0, 0, 0, 0, 0, 0);
  origin_ = initial_origin_;
  for (size_t i = 0; i < 10; ++i) {
    FloatType occupancy_bin = i / 10.0;
    occupancy_bins_.push_back(occupancy_bin);
  }
  occupancy_threshold_ = occupancy_bins_[occupancy_bins_.size() / 2];
}

OcTreeDrawer::~OcTreeDrawer() {}

void OcTreeDrawer::draw(const QMatrix4x4& pvm_matrix, const QMatrix4x4& view_matrix, const QMatrix4x4& model_matrix) {
  if (draw_octree_) {
    drawVoxelsAboveThreshold(pvm_matrix, view_matrix, model_matrix, occupancy_threshold_, draw_free_voxels_);
  }
  if (draw_raycast_ && raycast_drawer_) {
    raycast_drawer_->draw(pvm_matrix, view_matrix, model_matrix);
  }
}

OcTreeDrawer::FloatType OcTreeDrawer::getOccupancyBinThreshold() const {
    return occupancy_threshold_;
}

void OcTreeDrawer::setOccupancyBinThreshold(FloatType occupancy_bin_threshold) {
    occupancy_threshold_ = occupancy_bin_threshold;
}

void OcTreeDrawer::setDrawOctree(bool draw_octree) {
  draw_octree_ = draw_octree;
}

void OcTreeDrawer::setDrawRaycast(bool draw_raycast) {
  draw_raycast_ = draw_raycast;
}

void OcTreeDrawer::setColorFlags(uint32_t color_flags_uint) {
    color_flags_ = static_cast<VoxelDrawer::ColorFlags>(color_flags_uint);
    forEachVoxelDrawer([this](VoxelDrawer& voxel_drawer) {
      voxel_drawer.setColorFlags(color_flags_);
    });
}

void OcTreeDrawer::setDrawFreeVoxels(bool draw_free_voxels) {
    draw_free_voxels_ = draw_free_voxels;
}

void OcTreeDrawer::setAlphaOccupied(FloatType alpha) {
    alpha_override_ = alpha;
    for (auto& entry : voxel_drawer_map_) {
      entry.second.overrideAlpha(alpha_override_);
    }
//    updateVoxelColorHeightmap();

}

void OcTreeDrawer::setDrawSingleBin(bool draw_single_bin) {
    draw_single_bin_ = draw_single_bin;
}

const std::vector<OcTreeDrawer::FloatType>& OcTreeDrawer::getOccupancyBins() const {
    return occupancy_bins_;
}

void OcTreeDrawer::setMinOccupancy(FloatType min_occupancy) {
  min_occupancy_ = min_occupancy;
  forEachVoxelDrawer([this](VoxelDrawer& voxel_drawer) {
    voxel_drawer.setMinOccupancy(min_occupancy_);
  });
}

void OcTreeDrawer::setMaxOccupancy(FloatType max_occupancy) {
  max_occupancy_ = max_occupancy;
  forEachVoxelDrawer([this](VoxelDrawer& voxel_drawer) {
    voxel_drawer.setMaxOccupancy(max_occupancy_);
  });
}

void OcTreeDrawer::setMinObservations(uint32_t min_observations) {
  min_observations_ = min_observations;
  forEachVoxelDrawer([this](VoxelDrawer& voxel_drawer) {
    voxel_drawer.setMinObservations(min_observations_);
  });
}

void OcTreeDrawer::setMaxObservations(uint32_t max_observations) {
  max_observations_ = max_observations;
  forEachVoxelDrawer([this](VoxelDrawer& voxel_drawer) {
    voxel_drawer.setMaxObservations(max_observations_);
  });
}

void OcTreeDrawer::setMinVoxelSize(FloatType min_voxel_size) {
  min_voxel_size_ = min_voxel_size;
  forEachVoxelDrawer([this](VoxelDrawer& voxel_drawer) {
    voxel_drawer.setMinVoxelSize(min_voxel_size_);
  });
}

void OcTreeDrawer::setMaxVoxelSize(FloatType max_voxel_size) {
  max_voxel_size_ = max_voxel_size;
  forEachVoxelDrawer([this](VoxelDrawer& voxel_drawer) {
    voxel_drawer.setMaxVoxelSize(max_voxel_size_);
  });
}

void OcTreeDrawer::setMinWeight(FloatType min_weight) {
  min_weight_ = min_weight;
  forEachVoxelDrawer([this](VoxelDrawer& voxel_drawer) {
    voxel_drawer.setMinWeight(min_weight_);
  });
}

void OcTreeDrawer::setMaxWeight(FloatType max_weight) {
  max_weight_ = max_weight;
  forEachVoxelDrawer([this](VoxelDrawer& voxel_drawer) {
    voxel_drawer.setMaxWeight(max_weight_);
  });
}

void OcTreeDrawer::setMinInformation(FloatType min_information) {
  min_information_ = min_information;
  forEachVoxelDrawer([this](VoxelDrawer& voxel_drawer) {
    voxel_drawer.setMinInformation(min_information_);
  });
}

void OcTreeDrawer::setMaxInformation(FloatType max_information) {
  max_information_ = max_information;
  forEachVoxelDrawer([this](VoxelDrawer& voxel_drawer) {
    voxel_drawer.setMaxInformation(max_information_);
  });
}

size_t OcTreeDrawer::getRenderTreeDepth() const {
  return render_tree_depth_;
}

void OcTreeDrawer::setRenderTreeDepth(size_t render_tree_depth) {
  render_tree_depth_ = render_tree_depth;
}

size_t OcTreeDrawer::getRenderObservationThreshold() const {
  return render_observation_threshold_;
}

void OcTreeDrawer::setRenderObservationThreshold(size_t render_observation_threshold) {
  render_observation_threshold_ = render_observation_threshold;
}

void OcTreeDrawer::setOctree(const ViewpointPlanner::OccupancyMapType* octree, const pose6d& origin)
{
    octree_ = octree;
    // save origin used during cube generation
    initial_origin_ = octomap::pose6d(octomap::point3d(0, 0, 0), origin.rot());
    // origin is in global coords
    origin_ = origin;
    updateVoxelsFromOctree();
}

void OcTreeDrawer::updateVoxelsFromOctree()
{
  ait::Timer timer;
  updateVoxelData();
  timer.printTiming("Updating voxel arrays");
}

OcTreeDrawer::FloatType OcTreeDrawer::findOccupancyBin(FloatType occupancy) const {
    auto it = std::upper_bound(occupancy_bins_.cbegin(), occupancy_bins_.cend(), occupancy);
    if (it == occupancy_bins_.cend()) {
        return occupancy_bins_.back();
    }
    if (it != occupancy_bins_.cbegin()) {
        --it;
    }
    return *it;
}

void OcTreeDrawer::updateRaycastVoxels(
    const std::vector<std::pair<ViewpointPlanner::ConstTreeNavigatorType, FloatType>>& raycast_voxels) {
  std::vector<OGLVoxelData> voxel_data;
  std::vector<OGLColorData> color_data;
  std::vector<OGLVoxelInfoData> info_data;
  FloatType low_information = std::numeric_limits<FloatType>::max();
  FloatType high_information = std::numeric_limits<FloatType>::lowest();
  for (const auto& entry : raycast_voxels) {
    const auto& nav = entry.first;
    // TODO
//    std::cout << "raycast voxels: " << raycast_voxels.size() << std::endl;
//    std::cout << "position: " << nav.getPosition().transpose() << std::endl;
//    std::cout << "size: " << nav.getSize() << std::endl;
//    std::cout << "key: " << nav.getKey()[0] << ", " << nav.getKey()[1] << ", " << nav.getKey()[2] << std::endl;
//    std::cout << "depth: " << nav.getDepth() << std::endl;
//    std::cout << "occupancy: " << nav->getOccupancy() << std::endl;
//    std::cout << "observations: " << nav->getObservationCount() << std::endl;
//    std::cout << "observation sum: " << nav->getObservationCountSum() << std::endl;
//    std::cout << "weight: " << nav->getWeight() << std::endl;
    const Eigen::Vector3f voxel_position = nav.getPosition();
    FloatType voxel_size = nav.getSize();

    OGLVertexData vertex(voxel_position(0), voxel_position(1), voxel_position(2));
    voxel_data.emplace_back(vertex, voxel_size);
    if (octree_->isNodeKnown(*nav)) {
      color_data.emplace_back(1, 1, 0, 1);
    }
    else {
      color_data.emplace_back(0, 1, 1, 1);
    }

    FloatType occupancy = nav->getOccupancy();
    FloatType weight = nav->getWeight();
    const FloatType information = entry.second;
    info_data.emplace_back(occupancy, nav->getObservationCount(), weight,information);

    low_information = std::min(information, low_information);
    high_information = std::max(information, high_information);
  }

  if (!raycast_drawer_) {
    raycast_drawer_.reset(new VoxelDrawer());
  }
  raycast_drawer_->init();
  configVoxelDrawer(*raycast_drawer_);
  raycast_drawer_->setInformationRange(low_information, high_information);
  raycast_drawer_->upload(voxel_data, color_data, info_data);
  std::cout << "Information range: [" << low_information << ", " << high_information << "]" << std::endl;
}

void OcTreeDrawer::updateRaycastVoxels(
    const std::vector<std::pair<ViewpointPlannerData::OccupiedTreeType::IntersectionResult, FloatType>>& raycast_voxels) {
  std::vector<OGLVoxelData> voxel_data;
  std::vector<OGLColorData> color_data;
  std::vector<OGLVoxelInfoData> info_data;
  FloatType low_weight = std::numeric_limits<FloatType>::max();
  FloatType high_weight = std::numeric_limits<FloatType>::lowest();
  FloatType low_information = std::numeric_limits<FloatType>::max();
  FloatType high_information = std::numeric_limits<FloatType>::lowest();
  for (const auto& entry : raycast_voxels) {
    const auto& node = entry.first.node;
    // TODO
//    std::cout << "raycast voxels: " << raycast_voxels.size() << std::endl;
//    std::cout << "position: " << nav.getPosition().transpose() << std::endl;
//    std::cout << "size: " << nav.getSize() << std::endl;
//    std::cout << "key: " << nav.getKey()[0] << ", " << nav.getKey()[1] << ", " << nav.getKey()[2] << std::endl;
//    std::cout << "depth: " << nav.getDepth() << std::endl;
//    std::cout << "occupancy: " << nav->getOccupancy() << std::endl;
//    std::cout << "observations: " << nav->getObservationCount() << std::endl;
//    std::cout << "observation sum: " << nav->getObservationCountSum() << std::endl;
//    std::cout << "weight: " << nav->getWeight() << std::endl;
    const Eigen::Vector3f voxel_position = node->getBoundingBox().getCenter();
    FloatType voxel_size = node->getBoundingBox().getMaxExtent();

    OGLVertexData vertex(voxel_position(0), voxel_position(1), voxel_position(2));
    voxel_data.emplace_back(vertex, voxel_size);
    if (octree_->isNodeKnown(node->getObject()->observation_count)) {
      color_data.emplace_back(1, 1, 0, 1);
    }
    else {
      color_data.emplace_back(0, 1, 1, 1);
    }
//    if (octree_->isNodeKnown(*nav)) {
//      color_data.emplace_back(1, 1, 0, 1);
//    }
//    else {
//      color_data.emplace_back(0, 1, 1, 1);
//    }

//    FloatType occupancy = nav->getOccupancy();
//    FloatType weight = nav->getWeight();
    FloatType weight = node->getObject()->weight;
//    const FloatType information = entry.second;
    const FloatType information = entry.second;
    info_data.emplace_back(node->getObject()->occupancy, node->getObject()->observation_count, weight, information);

    low_information = std::min(information, low_information);
    high_information = std::max(information, high_information);
    low_weight = std::min(weight, low_weight);
    high_weight = std::max(weight, high_weight);
  }

  if (!raycast_drawer_) {
    raycast_drawer_.reset(new VoxelDrawer());
  }
  raycast_drawer_->init();
  configVoxelDrawer(*raycast_drawer_);
  raycast_drawer_->setVoxelSizeFactor(kRaycastVoxelSizeFactor);
  raycast_drawer_->setInformationRange(low_information, high_information);
  raycast_drawer_->upload(voxel_data, color_data, info_data);
  std::cout << "Information range: [" << low_information << ", " << high_information << "]" << std::endl;
  std::cout << "Weight range: [" << low_weight << ", " << high_weight << "]" << std::endl;
}

void OcTreeDrawer::updateRaycastVoxels(
    const std::vector<std::pair<const ViewpointPlanner::VoxelType*, FloatType>>& raycast_voxels) {
  std::vector<OGLVoxelData> voxel_data;
  std::vector<OGLColorData> color_data;
  std::vector<OGLVoxelInfoData> info_data;
  FloatType low_weight = std::numeric_limits<FloatType>::max();
  FloatType high_weight = std::numeric_limits<FloatType>::lowest();
  FloatType low_information = std::numeric_limits<FloatType>::max();
  FloatType high_information = std::numeric_limits<FloatType>::lowest();
  for (const auto& entry : raycast_voxels) {
    const auto& node = entry.first;
    // TODO
//    std::cout << "raycast voxels: " << raycast_voxels.size() << std::endl;
//    std::cout << "position: " << nav.getPosition().transpose() << std::endl;
//    std::cout << "size: " << nav.getSize() << std::endl;
//    std::cout << "key: " << nav.getKey()[0] << ", " << nav.getKey()[1] << ", " << nav.getKey()[2] << std::endl;
//    std::cout << "depth: " << nav.getDepth() << std::endl;
//    std::cout << "occupancy: " << nav->getOccupancy() << std::endl;
//    std::cout << "observations: " << nav->getObservationCount() << std::endl;
//    std::cout << "observation sum: " << nav->getObservationCountSum() << std::endl;
//    std::cout << "weight: " << nav->getWeight() << std::endl;
    const Eigen::Vector3f voxel_position = node->getBoundingBox().getCenter();
    FloatType voxel_size = node->getBoundingBox().getMaxExtent();

    OGLVertexData vertex(voxel_position(0), voxel_position(1), voxel_position(2));
    voxel_data.emplace_back(vertex, voxel_size);
    if (octree_->isNodeKnown(node->getObject()->observation_count)) {
      color_data.emplace_back(1, 1, 0, 1);
    }
    else {
      color_data.emplace_back(0, 1, 1, 1);
    }
//    if (octree_->isNodeKnown(*nav)) {
//      color_data.emplace_back(1, 1, 0, 1);
//    }
//    else {
//      color_data.emplace_back(0, 1, 1, 1);
//    }

//    FloatType occupancy = nav->getOccupancy();
//    FloatType weight = nav->getWeight();
    FloatType weight = node->getObject()->weight;
//    const FloatType information = entry.second;
    const FloatType information = entry.second;
    info_data.emplace_back(node->getObject()->occupancy, node->getObject()->observation_count, weight, information);

    low_information = std::min(information, low_information);
    high_information = std::max(information, high_information);
    low_weight = std::min(weight, low_weight);
    high_weight = std::max(weight, high_weight);
  }

  if (!raycast_drawer_) {
    raycast_drawer_.reset(new VoxelDrawer());
  }
  raycast_drawer_->init();
  configVoxelDrawer(*raycast_drawer_);
  raycast_drawer_->setVoxelSizeFactor(kRaycastVoxelSizeFactor);
  raycast_drawer_->setInformationRange(low_information, high_information);
  raycast_drawer_->upload(voxel_data, color_data, info_data);
  std::cout << "Information range: [" << low_information << ", " << high_information << "]" << std::endl;
  std::cout << "Weight range: [" << low_weight << ", " << high_weight << "]" << std::endl;
}

void OcTreeDrawer::updateRaycastVoxels(
    const ViewpointPlanner::VoxelWithInformationSet& raycast_voxels) {
  std::cout << "Updating raycast with " << raycast_voxels.size() << " voxels" << std::endl;
  std::vector<OGLVoxelData> voxel_data;
  std::vector<OGLColorData> color_data;
  std::vector<OGLVoxelInfoData> info_data;
  FloatType low_weight = std::numeric_limits<FloatType>::max();
  FloatType high_weight = std::numeric_limits<FloatType>::lowest();
  FloatType low_information = std::numeric_limits<FloatType>::max();
  FloatType high_information = std::numeric_limits<FloatType>::lowest();
  for (const ViewpointPlanner::VoxelWithInformation& voxel_with_information : raycast_voxels) {
    const ViewpointPlanner::VoxelType* voxel = voxel_with_information.voxel;
    // TODO
//    std::cout << "raycast voxels: " << raycast_voxels.size() << std::endl;
//    std::cout << "position: " << nav.getPosition().transpose() << std::endl;
//    std::cout << "size: " << nav.getSize() << std::endl;
//    std::cout << "key: " << nav.getKey()[0] << ", " << nav.getKey()[1] << ", " << nav.getKey()[2] << std::endl;
//    std::cout << "depth: " << nav.getDepth() << std::endl;
//    std::cout << "occupancy: " << nav->getOccupancy() << std::endl;
//    std::cout << "observations: " << nav->getObservationCount() << std::endl;
//    std::cout << "observation sum: " << nav->getObservationCountSum() << std::endl;
//    std::cout << "weight: " << nav->getWeight() << std::endl;
    const Eigen::Vector3f voxel_position = voxel->getBoundingBox().getCenter();
    FloatType voxel_size = voxel->getBoundingBox().getMaxExtent();

    OGLVertexData vertex(voxel_position(0), voxel_position(1), voxel_position(2));
    voxel_data.emplace_back(vertex, voxel_size);
    if (octree_->isNodeKnown(voxel->getObject()->observation_count)) {
      color_data.emplace_back(1, 1, 0, 1);
    }
    else {
      color_data.emplace_back(0, 1, 1, 1);
    }
//    if (octree_->isNodeKnown(*nav)) {
//      color_data.emplace_back(1, 1, 0, 1);
//    }
//    else {
//      color_data.emplace_back(0, 1, 1, 1);
//    }

//    FloatType occupancy = nav->getOccupancy();
//    FloatType weight = nav->getWeight();
    FloatType weight = voxel->getObject()->weight;
//    const FloatType information = entry.second;
    const FloatType information = voxel_with_information.information;
    info_data.emplace_back(voxel->getObject()->occupancy, voxel->getObject()->observation_count, weight, information);

    low_information = std::min(information, low_information);
    high_information = std::max(information, high_information);
    low_weight = std::min(weight, low_weight);
    high_weight = std::max(weight, high_weight);
  }

  if (!raycast_drawer_) {
    raycast_drawer_.reset(new VoxelDrawer());
  }
  raycast_drawer_->init();
  configVoxelDrawer(*raycast_drawer_);
  raycast_drawer_->setVoxelSizeFactor(kRaycastVoxelSizeFactor);
  raycast_drawer_->setInformationRange(low_information, high_information);
  raycast_drawer_->upload(voxel_data, color_data, info_data);
  std::cout << "Information range: [" << low_information << ", " << high_information << "]" << std::endl;
  std::cout << "Weight range: [" << low_weight << ", " << high_weight << "]" << std::endl;
}

void OcTreeDrawer::updateRaycastVoxels(
    const ViewpointPlanner::VoxelMap& voxel_map) {
  std::cout << "Updating raycast with " << voxel_map.size() << " voxels" << std::endl;
  std::vector<OGLVoxelData> voxel_data;
  std::vector<OGLColorData> color_data;
  std::vector<OGLVoxelInfoData> info_data;
  FloatType low_weight = std::numeric_limits<FloatType>::max();
  FloatType high_weight = std::numeric_limits<FloatType>::lowest();
  FloatType low_information = std::numeric_limits<FloatType>::max();
  FloatType high_information = std::numeric_limits<FloatType>::lowest();
  for (const auto& entry : voxel_map) {
    const ViewpointPlanner::VoxelType* voxel = entry.first.voxel;
    const FloatType information = entry.second;
    // TODO
//    std::cout << "raycast voxels: " << raycast_voxels.size() << std::endl;
//    std::cout << "position: " << nav.getPosition().transpose() << std::endl;
//    std::cout << "size: " << nav.getSize() << std::endl;
//    std::cout << "key: " << nav.getKey()[0] << ", " << nav.getKey()[1] << ", " << nav.getKey()[2] << std::endl;
//    std::cout << "depth: " << nav.getDepth() << std::endl;
//    std::cout << "occupancy: " << nav->getOccupancy() << std::endl;
//    std::cout << "observations: " << nav->getObservationCount() << std::endl;
//    std::cout << "observation sum: " << nav->getObservationCountSum() << std::endl;
//    std::cout << "weight: " << nav->getWeight() << std::endl;
    const Eigen::Vector3f voxel_position = voxel->getBoundingBox().getCenter();
    FloatType voxel_size = voxel->getBoundingBox().getMaxExtent();

    OGLVertexData vertex(voxel_position(0), voxel_position(1), voxel_position(2));
    voxel_data.emplace_back(vertex, voxel_size);
    if (octree_->isNodeKnown(voxel->getObject()->observation_count)) {
      color_data.emplace_back(1, 1, 0, 1);
    }
    else {
      color_data.emplace_back(0, 1, 1, 1);
    }
//    if (octree_->isNodeKnown(*nav)) {
//      color_data.emplace_back(1, 1, 0, 1);
//    }
//    else {
//      color_data.emplace_back(0, 1, 1, 1);
//    }

//    FloatType occupancy = nav->getOccupancy();
//    FloatType weight = nav->getWeight();
    FloatType weight = voxel->getObject()->weight;
//    const FloatType information = entry.second;
    info_data.emplace_back(voxel->getObject()->occupancy, voxel->getObject()->observation_count, weight, information);

    low_information = std::min(information, low_information);
    high_information = std::max(information, high_information);
    low_weight = std::min(weight, low_weight);
    high_weight = std::max(weight, high_weight);
  }

  if (!raycast_drawer_) {
    raycast_drawer_.reset(new VoxelDrawer());
  }
  raycast_drawer_->init();
  configVoxelDrawer(*raycast_drawer_);
  raycast_drawer_->setVoxelSizeFactor(kRaycastVoxelSizeFactor);
  raycast_drawer_->setInformationRange(low_information, high_information);
  raycast_drawer_->upload(voxel_data, color_data, info_data);
  std::cout << "Information range: [" << low_information << ", " << high_information << "]" << std::endl;
  std::cout << "Weight range: [" << low_weight << ", " << high_weight << "]" << std::endl;
}

void OcTreeDrawer::updateVoxelData() {
  AIT_ASSERT_STR(octree_ != nullptr, "Octree was not initialized");

  std::unordered_map<FloatType, std::vector<OGLVoxelData>> voxel_data;
  std::unordered_map<FloatType, std::vector<OGLColorData>> color_data;
  std::unordered_map<FloatType, std::vector<OGLVoxelInfoData>> info_data;
  for (FloatType occupancy_bin : occupancy_bins_) {
    voxel_data.emplace(occupancy_bin, std::vector<OGLVoxelData>());
    color_data.emplace(occupancy_bin, std::vector<OGLColorData>());
    info_data.emplace(occupancy_bin, std::vector<OGLVoxelInfoData>());
  }

  // Ranges of weight and observation counts
  low_weight_ = std::numeric_limits<FloatType>::max();
  high_weight_ = std::numeric_limits<FloatType>::lowest();
  low_observation_count_ = std::numeric_limits<uint32_t>::max();
  high_observation_count_ = std::numeric_limits<uint32_t>::lowest();
  // Range of z coordinate height color map
  FloatType min_z = std::numeric_limits<FloatType>::infinity();
  FloatType max_z = -std::numeric_limits<FloatType>::infinity();

  for(ViewpointPlanner::OccupancyMapType::tree_iterator it = octree_->begin_tree(render_tree_depth_), end=octree_->end_tree(); it!= end; ++it) {
    if (it.isLeaf() && it->getObservationCount() >= render_observation_threshold_) {

      const octomap::point3d& voxel_position = it.getCoordinate();
      FloatType voxel_size = it.getSize();

      FloatType occupancy = it->getOccupancy();
      FloatType occupancy_bin = findOccupancyBin(occupancy);
      FloatType weight = it->getWeight();

      OGLVertexData vertex(voxel_position.x(), voxel_position.y(), voxel_position.z());
      voxel_data.at(occupancy_bin).emplace_back(vertex, voxel_size);

      info_data.at(occupancy_bin).emplace_back(occupancy, it->getObservationCount(), weight);

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
  for (FloatType occupancy_bin : occupancy_bins_) {
    for (const OGLVoxelData& voxel : voxel_data.at(occupancy_bin)) {
      OGLColorData color = getVoxelColorData(voxel, min_z, max_z);
      color_data.at(occupancy_bin).emplace_back(color);
    }
    FloatType weight_low = std::numeric_limits<FloatType>::max();
    FloatType weight_high = std::numeric_limits<FloatType>::lowest();
    FloatType observation_low = std::numeric_limits<FloatType>::max();
    FloatType observation_high = std::numeric_limits<FloatType>::lowest();
    for (const OGLVoxelInfoData& info : info_data.at(occupancy_bin)) {
      weight_low = std::min(info.weight, weight_low);
      weight_high = std::max(info.weight, weight_high);
      observation_low = std::min(info.observation_count, observation_low);
      observation_high = std::max(info.observation_count, observation_high);
    }
//    std::cout << "Weight range for bin " << occupancy_bin << ": [" << weight_low << ", " << weight_high << "]" << std::endl;
//    std::cout << "Observation range for bin " << occupancy_bin << ": [" << observation_low << ", " << observation_high << "]" << std::endl;
  }

  std::cout << "Occupancy map z range: " << "[" << min_z << ", " << max_z << "]" << std::endl;
  std::cout << "Weight range: [" << low_weight_ << ", " << high_weight_ << "]" << std::endl;
  std::cout << "Observation count range: [" << low_observation_count_ << ", " << high_observation_count_ << "]" << std::endl;

  voxel_drawer_map_.clear();
  for (FloatType occupancy_bin : occupancy_bins_) {
    const auto result = voxel_drawer_map_.emplace(std::piecewise_construct, std::make_tuple(occupancy_bin), std::make_tuple());
    VoxelDrawer& voxel_drawer = result.first->second;
    voxel_drawer.init();
    configVoxelDrawer(voxel_drawer);
//    std::cout << "Uploading " << voxel_data.at(occupancy_bin).size() << " voxels" << std::endl;
    voxel_drawer.upload(voxel_data.at(occupancy_bin), color_data.at(occupancy_bin), info_data.at(occupancy_bin));
  }

  for (const auto& entry : voxel_drawer_map_) {
    std::cout << "Voxels in bin " << entry.first << ": " << entry.second.numOfVoxels() << std::endl;
  }
}

void OcTreeDrawer::configVoxelDrawer(VoxelDrawer& voxel_drawer) const {
  voxel_drawer.setColorFlags(color_flags_);
  voxel_drawer.setMinObservations(min_observations_);
  voxel_drawer.setMaxObservations(max_observations_);
  voxel_drawer.setMinOccupancy(min_occupancy_);
  voxel_drawer.setMaxOccupancy(max_occupancy_);
  voxel_drawer.setMinVoxelSize(min_voxel_size_);
  voxel_drawer.setMaxVoxelSize(max_voxel_size_);
  voxel_drawer.setMinWeight(min_weight_);
  voxel_drawer.setMaxWeight(max_weight_);
  voxel_drawer.setWeightRange(low_weight_, high_weight_);
  voxel_drawer.setObservationsRange(low_observation_count_, high_observation_count_);
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

void OcTreeDrawer::drawVoxelsAboveThreshold(const QMatrix4x4& pvm_matrix, const QMatrix4x4& view_matrix, const QMatrix4x4& model_matrix,
    FloatType occupancy_threshold, bool draw_below_threshold) {
  if (draw_single_bin_) {
    // Find closest lower bin
    FloatType closest_key = 0;
    FloatType closest_dist = std::numeric_limits<FloatType>::infinity();
    for (const auto& entry : voxel_drawer_map_) {
      FloatType dist = std::abs(occupancy_threshold_ - entry.first);
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

void OcTreeDrawer::setOrigin(octomap::pose6d t) {
    origin_ = t;
    std::cout << "OcTreeDrawer: setting new global origin: " << t << std::endl;

    octomap::pose6d relative_transform = origin_ * initial_origin_.inv();

    std::cout << "origin        : " << origin_ << std::endl;
    std::cout << "inv init orig : " << initial_origin_.inv() << std::endl;
    std::cout << "relative trans: " << relative_transform << std::endl;
}

void OcTreeDrawer::forEachVoxelDrawer(const std::function<void(VoxelDrawer&)> func) {
  for (auto& entry : voxel_drawer_map_) {
    func(entry.second);
  }
  if (raycast_drawer_) {
    func(*raycast_drawer_);
  }
}
