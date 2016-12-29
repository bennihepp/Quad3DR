
//==================================================
// occupancy_node.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 10, 2016
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

#include <tuple>
#include <limits>
#include <cmath>
#include <octomap/octomap_types.h>
#include <octomap/octomap_utils.h>
#include <octomap/OcTreeDataNode.h>

// forward declaration for friend in OcTreeDataNode
namespace octomap {
template<typename NODE,typename I> class OcTreeBaseImpl;
}

template <typename NodeType>
class OccupancyMap;

/**
 * Basic node in the OcTree that can hold arbitrary data of type T in value.
 * This is the base class for nodes used in an OcTree. The used implementation
 * for occupancy mapping is in OcTreeNode.#
 * \tparam T data to be stored in the node (e.g. a float for probabilities)
 *
 * Note: If you derive a class (directly or indirectly) from OcTreeDataNode,
 * you have to implement (at least) the following functions to avoid slicing
 * errors and memory-related bugs:
 * createChild(), getChild(), getChild() const, expandNode()
 * See ColorOcTreeNode in ColorOcTree.h for an example.
 */
class OccupancyNode : public octomap::AbstractOcTreeNode {
  template<typename NODE, typename I>
  friend class octomap::OcTreeBaseImpl;

  template <typename NodeType>
  friend class OccupancyMap;

public:
  using OccupancyType = float;
  using CounterType = uint32_t;

  /// Initialize with zero counts
  OccupancyNode();

  /// Initialize with given counts
  OccupancyNode(OccupancyType occupancy, CounterType observation_count);

  /// Copy constructor, performs a recursive deep-copy of all children
  /// including node data in "value"
  OccupancyNode(const OccupancyNode& rhs);

  /// Delete only own members.
  /// OcTree maintains tree structure and must have deleted children already
  ~OccupancyNode();

  /// Copy the payload (data in "value") from rhs into this node
  /// Opposed to copy ctor, this does not clone the children as well
  void copyData(const OccupancyNode& from);

  /// Equals operator, compares if the stored value is identical
  bool operator==(const OccupancyNode& rhs) const;

  // -- children  ----------------------------------

  /// Test whether the i-th child exists.
  /// @deprecated Replaced by tree->nodeChildExists(...)
  /// \return true if the i-th child exists
//  OCTOMAP_DEPRECATED(bool childExists(unsigned int i) const);

  /// @deprecated Replaced by tree->nodeHasChildren(...)
  /// \return true if the node has at least one child
//  OCTOMAP_DEPRECATED(bool hasChildren() const);

  bool hasChildren() const {
    return children != nullptr;
  }

  bool hasChild(size_t i) const {
    return children[i] != nullptr;
  }

  // -- node occupancy  ----------------------------

  OccupancyType getOccupancy() const {
    return occupancy_;
  }

  OccupancyType getOccupancyLogOdds() const {
    return octomap::logodds(occupancy_);
  }

  /// @return observation count stored in the node
  const CounterType getObservationCount() const {
    return observation_count_;
  }

  void setObservationCount(CounterType count) {
    observation_count_ = count;
  }

  void setOccupancy(OccupancyType occupancy) {
    occupancy_ = occupancy;
  }

  void setOccupancyLogOdds(OccupancyType occupancy_log) {
    occupancy_ = octomap::probability(occupancy_log);
  }

  void addObservationLogOdds(OccupancyType occupancy_log) {
    occupancy_ = octomap::probability(getOccupancyLogOdds() + occupancy_log);
    ++observation_count_;
  }

  /// update this node's occupancy according to its children's maximum occupancy
  void updateFromChildren() {
    std::tuple<OccupancyType, CounterType> acc_tuple = this->getMaxChildOccupancyAndMinObservationCount();
    setOccupancy(std::get<0>(acc_tuple));
    setObservationCount(std::get<1>(acc_tuple));
  }

  OccupancyType getMeanChildOccupancyLogOdds() const;

  OccupancyType getMeanChildOccupancy() const;

  OccupancyType getMaxChildOccupancyLogOdds() const;

  OccupancyType getMaxChildOccupancy() const;

  std::tuple<OccupancyType, CounterType> getMeanChildOccupancyAndSumObservationCount() const;
  std::tuple<OccupancyType, CounterType> getMaxChildOccupancyAndMinObservationCount() const;

  // file IO:

  /// Read node payload (data only) from binary stream
  std::istream& readData(std::istream &s);

  /// Write node payload (data only) to binary stream
  std::ostream& writeData(std::ostream &s) const;

  const OccupancyNode* getChild(size_t pos) const {
    return static_cast<OccupancyNode*>(children[pos]);
  }

  OccupancyNode* getChild(size_t pos) {
    return static_cast<OccupancyNode*>(children[pos]);
  }

protected:
  void allocChildren();

  /// pointer to array of children, may be NULL
  /// @note The tree class manages this pointer, the array, and the memory for it!
  /// The children of a node are always enforced to be the same type as the node
  AbstractOcTreeNode** children;

  OccupancyType occupancy_;
  CounterType observation_count_;

//  CounterType occupied_count_;
//  CounterType free_count_;
};

class AugmentedOccupancyNode : public OccupancyNode {
public:
  using WeightType = float;

  AugmentedOccupancyNode();

  AugmentedOccupancyNode(const AugmentedOccupancyNode& rhs);

  /// Delete only own members.
  /// OcTree maintains tree structure and must have deleted children already
  ~AugmentedOccupancyNode();

  void copyData(const AugmentedOccupancyNode& from);

  const float getWeight() const {
    return weight_;
  }

  void setWeight(float weight) {
    weight_ = weight;
  }

  const AugmentedOccupancyNode* getParent() const {
    return parent_;
  }

  AugmentedOccupancyNode* getParent() {
    return parent_;
  }

  void setParent(AugmentedOccupancyNode* parent) {
    parent_ = parent;
  }

  size_t getObservationCountSum() const {
    return observation_count_sum_;
  }

  void setObservationCountSum(size_t observation_count_sum) {
    observation_count_sum_ = observation_count_sum;
  }

  size_t getSumObservationCount() const;

  /// update this node's occupancy according to its children's maximum occupancy
  void updateFromChildren() {
    OccupancyNode::updateFromChildren();
    size_t observation_count_sum = this->getSumObservationCount();
    observation_count_sum_ = observation_count_sum;
  }

  // file IO:

  /// Read node payload (data only) from binary stream
  std::istream& readData(std::istream &s);

  /// Write node payload (data only) to binary stream
  std::ostream& writeData(std::ostream &s) const;

private:
  AugmentedOccupancyNode* parent_;
  WeightType weight_;
  size_t observation_count_sum_;
};
