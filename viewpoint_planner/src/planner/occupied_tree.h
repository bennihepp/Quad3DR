//==================================================
// occupied_tree_node.h.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 27.03.17
//

#pragma once

#include "viewpoint_planner_types.h"
#include <bh/eigen.h>
#include <boost/serialization/access.hpp>

namespace viewpoint_planner {

template<typename FloatT>
struct NodeObject {
  FloatT occupancy;
  uint16_t observation_count;
  FloatT weight;
  Eigen::Matrix<FloatT, 3, 1> normal;

private:
  // Boost serialization
  friend class boost::serialization::access;

  template<typename Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar & occupancy;
    ar & observation_count;
    ar & weight;
    ar & normal;
  }
};

using NodeObjectType = NodeObject<FloatType>;
using OccupiedTreeType = bvh::Tree<NodeObjectType, FloatType>;
using VoxelType = OccupiedTreeType::NodeType;

/// Voxel node and it's corresponding amount of information
struct VoxelWrapper {
  VoxelWrapper(const VoxelType* voxel)
          : voxel(voxel) {}

  const VoxelType* voxel;

  bool operator==(const VoxelWrapper& other) const {
    return voxel == other.voxel;
  }

  struct Hash {
    size_t operator()(const VoxelWrapper& voxel_with_information) const {
      size_t val { 0 };
      boost::hash_combine(val, voxel_with_information.voxel);
      return val;
    }
  };
};

/// Voxel node and it's corresponding amount of information
struct VoxelWithInformation {
  VoxelWithInformation(const VoxelType* voxel, const FloatType information)
          : voxel(voxel), information(information) {}

  const VoxelType* voxel;
  FloatType information;

  bool operator==(const VoxelWithInformation& other) const {
    return voxel == other.voxel && information == other.information;
  }

  struct VoxelHash {
    size_t operator()(const VoxelWithInformation& voxel_with_information) const {
      size_t val { 0 };
      boost::hash_combine(val, voxel_with_information.voxel);
      return val;
    }
  };

  struct Hash {
    size_t operator()(const VoxelWithInformation& voxel_with_information) const {
      size_t val { 0 };
      boost::hash_combine(val, voxel_with_information.voxel);
      boost::hash_combine(val, voxel_with_information.information);
      return val;
    }
  };
};

using VoxelWithInformationSet = std::unordered_set<VoxelWithInformation, VoxelWithInformation::VoxelHash>;
using VoxelMap = std::unordered_map<VoxelWrapper, FloatType, VoxelWrapper::Hash>;

}
