//==================================================
// occupancy_node.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 10, 2016
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

#include "occupancy_node.h"

/// Initialize with zero counts
OccupancyNode::OccupancyNode()
: children(nullptr), occupancy_(0.5f), observation_count_(0) {}

/// Initialize with given counts
OccupancyNode::OccupancyNode(OccupancyType occupancy, CounterType observation_count)
: children(nullptr), occupancy_(occupancy), observation_count_(observation_count) {}

/// Copy constructor, performs a recursive deep-copy of all children
/// including node data in "value"
OccupancyNode::OccupancyNode(const OccupancyNode& rhs)
: children(nullptr), occupancy_(rhs.occupancy_), observation_count_(rhs.observation_count_) {
 if (rhs.children != nullptr){
   allocChildren();
   for (unsigned i = 0; i<8; ++i){
     if (rhs.children[i] != nullptr)
       children[i] = new OccupancyNode(*(static_cast<OccupancyNode*>(rhs.children[i])));

   }
 }
}

/// Delete only own members.
/// OcTree maintains tree structure and must have deleted children already
OccupancyNode::~OccupancyNode() {}

/// Copy the payload (data in "value") from rhs into this node
/// Opposed to copy ctor, this does not clone the children as well
void OccupancyNode::copyData(const OccupancyNode& from) {
  occupancy_ = from.occupancy_;
  observation_count_ = from.observation_count_;
}

/// Equals operator, compares if the stored value is identical
bool OccupancyNode::operator==(const OccupancyNode& rhs) const {
  return occupancy_ == rhs.occupancy_ && observation_count_ == rhs.observation_count_;
}

// -- node occupancy  ----------------------------

std::tuple<OccupancyNode::OccupancyType, OccupancyNode::CounterType> OccupancyNode::getMeanChildOccupancyAndSumObservationCount() const {
  OccupancyType mean_occupancy = 0;
  CounterType sum_observation_count = 0;
  CounterType child_count = 0;
  if (children != nullptr){
    for (unsigned int i=0; i<8; i++) {
      if (children[i] != nullptr) {
        mean_occupancy += static_cast<OccupancyNode*>(children[i])->getOccupancy();
        sum_observation_count += static_cast<OccupancyNode*>(children[i])->getObservationCount();
        ++child_count;
      }
    }
  }

  if (child_count > 0) {
    mean_occupancy /= (OccupancyType)child_count;
  }

  return std::make_tuple(mean_occupancy, sum_observation_count);
}

std::tuple<OccupancyNode::OccupancyType, OccupancyNode::CounterType> OccupancyNode::getMaxChildOccupancyAndMinObservationCount() const {
  OccupancyType max_occupancy = -std::numeric_limits<OccupancyType>::max();
  CounterType min_observation_count = std::numeric_limits<CounterType>::max();
  if (children != nullptr){
    for (unsigned int i=0; i<8; i++) {
      if (children[i] != nullptr) {
        OccupancyType occupancy = static_cast<OccupancyNode*>(children[i])->getOccupancy();
        if (occupancy > max_occupancy) {
          max_occupancy = occupancy;
        }
        CounterType observation_count = static_cast<OccupancyNode*>(children[i])->getObservationCount();
        if (observation_count < min_observation_count) {
          min_observation_count = observation_count;
        }
      }
    }
  }

  return std::make_tuple(max_occupancy, min_observation_count);
}

OccupancyNode::OccupancyType OccupancyNode::getMeanChildOccupancyLogOdds() const {
  return octomap::logodds(std::get<0>(getMeanChildOccupancyAndSumObservationCount()));
}

OccupancyNode::OccupancyType OccupancyNode::getMeanChildOccupancy() const {
  return std::get<0>(getMeanChildOccupancyAndSumObservationCount());
}

OccupancyNode::OccupancyType OccupancyNode::getMaxChildOccupancyLogOdds() const {
  return octomap::logodds(std::get<0>(getMaxChildOccupancyAndMinObservationCount()));
}

OccupancyNode::OccupancyType OccupancyNode::getMaxChildOccupancy() const {
  return std::get<0>(getMaxChildOccupancyAndMinObservationCount());
}


// file IO:

/// Read node payload (data only) from binary stream
std::istream& OccupancyNode::readData(std::istream &s) {
  s.read((char*) &occupancy_, sizeof(occupancy_));
  s.read((char*) &observation_count_, sizeof(observation_count_));
  return s;
}

/// Write node payload (data only) to binary stream
std::ostream& OccupancyNode::writeData(std::ostream &s) const {
  s.write((const char*) &occupancy_, sizeof(occupancy_));
  s.write((const char*) &observation_count_, sizeof(observation_count_));
  return s;
}

void OccupancyNode::allocChildren() {
  children = new AbstractOcTreeNode*[8];
  for (unsigned int i=0; i<8; i++) {
    children[i] = nullptr;
  }
}

AugmentedOccupancyNode::AugmentedOccupancyNode()
: parent_(nullptr), weight_(0), observation_count_sum_(0) {}

AugmentedOccupancyNode::AugmentedOccupancyNode(const AugmentedOccupancyNode& rhs) {
  copyData(rhs);
  if (rhs.children != nullptr) {
    allocChildren();
    for (unsigned i = 0; i < 8; ++i){
    if (rhs.children[i] != nullptr)
      children[i] = new AugmentedOccupancyNode(*(static_cast<AugmentedOccupancyNode*>(rhs.children[i])));
    }
  }
  parent_ = nullptr;
  weight_ = rhs.weight_;
  observation_count_sum_ = rhs.observation_count_sum_;
}

AugmentedOccupancyNode::~AugmentedOccupancyNode() {}

/// Copy the payload (data in "value") from rhs into this node
/// Opposed to copy ctor, this does not clone the children as well
void AugmentedOccupancyNode::copyData(const AugmentedOccupancyNode& from) {
  OccupancyNode::copyData(static_cast<const OccupancyNode&>(from));
  parent_ = nullptr;
  weight_ = from.weight_;
  observation_count_sum_ = from.observation_count_sum_;
}


size_t AugmentedOccupancyNode::getSumObservationCount() const {
  size_t observation_count_sum = 0;
  if (children != nullptr){
    for (unsigned int i=0; i<8; i++) {
      if (children[i] != nullptr) {
        observation_count_sum += static_cast<AugmentedOccupancyNode*>(children[i])->getObservationCountSum();
      }
    }
  }

  return observation_count_sum;
}

std::istream& AugmentedOccupancyNode::readData(std::istream &s) {
  OccupancyNode::readData(s);
  s.read((char*)&weight_, sizeof(weight_));
  s.read((char*)&observation_count_sum_, sizeof(observation_count_sum_));
  return s;
}

std::ostream& AugmentedOccupancyNode::writeData(std::ostream &s) const {
  OccupancyNode::writeData(s);
  s.write((const char*)&weight_, sizeof(weight_));
  s.write((const char*)&observation_count_sum_, sizeof(observation_count_sum_));
  return s;
}
