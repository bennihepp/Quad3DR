//==================================================
// occupancy_map.cpp
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

#include <algorithm>
#include <cmath>
#include <octomap/MCTables.h>
#include <ait/common.h>
#include <ait/utilities.h>

template <typename NodeT>
AbstractOccupancyMap<NodeT>::AbstractOccupancyMap() {
  setOccupancyThres(0.7f);
  setProbHit(0.7f);  // = 0.85 in logodds
  setProbMiss(0.4f);  // = -0.4 in logodds
  setObservationThres(1);
}

template <typename NodeT>
OccupancyMap<NodeT>::OccupancyMap(double resolution)
: OcTreeBaseImpl<NodeT, AbstractOccupancyMap<NodeT>>(resolution), use_bbx_limit(false), use_change_detection(false)
{}

template <typename NodeT>
OccupancyMap<NodeT>::OccupancyMap(double resolution, unsigned int tree_depth, unsigned int tree_max_val)
: OcTreeBaseImpl<NodeT, AbstractOccupancyMap<NodeT>>(resolution, tree_depth, tree_max_val), use_bbx_limit(false), use_change_detection(false)
{}

template <typename NodeT>
OccupancyMap<NodeT>::~OccupancyMap(){}

template <typename NodeT>
OccupancyMap<NodeT>::OccupancyMap(const OccupancyMap& rhs)
: OcTreeBaseImpl<NodeT, AbstractOccupancyMap<NodeT>>(rhs), use_bbx_limit(rhs.use_bbx_limit),
  bbx_min(rhs.bbx_min), bbx_max(rhs.bbx_max),
  bbx_min_key(rhs.bbx_min_key), bbx_max_key(rhs.bbx_max_key),
  use_change_detection(rhs.use_change_detection), changed_keys(rhs.changed_keys) {
  this->occ_prob_thres_ = rhs.occ_prob_thres_;
  this->observation_thres_ = rhs.observation_thres_;
}

template <>
typename OccupancyMap<OccupancyNode>::StaticMemberInitializer OccupancyMap<OccupancyNode>::ocTreeMemberInit;

template <>
typename OccupancyMap<AugmentedOccupancyNode>::StaticMemberInitializer OccupancyMap<AugmentedOccupancyNode>::ocTreeMemberInit;

//template <typename NodeT>
//typename OccupancyMap<NodeT>::StaticMemberInitializer OccupancyMap<NodeT>::ocTreeMemberInit;

template <typename NodeT>
void OccupancyMap<NodeT>::insertPointCloud(const Pointcloud& scan, const octomap::point3d& sensor_origin,
                                           double maxrange, bool lazy_eval, bool discretize) {
  KeySet free_cells, occupied_cells;
  if (discretize)
    computeDiscreteUpdate(scan, sensor_origin, free_cells, occupied_cells, maxrange);
  else
    computeUpdate(scan, sensor_origin, free_cells, occupied_cells, maxrange);

  // insert data into tree  -----------------------
  for (KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it) {
    updateNode(*it, false, lazy_eval);
  }
  for (KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it) {
    updateNode(*it, true, lazy_eval);
  }
}

template <typename NodeT>
void OccupancyMap<NodeT>::insertPointCloud(const Pointcloud& pc, const point3d& sensor_origin, const pose6d& frame_origin,
                                           double maxrange, bool lazy_eval, bool discretize) {
  // performs transformation to data and sensor origin first
  Pointcloud transformed_scan (pc);
  transformed_scan.transform(frame_origin);
  point3d transformed_sensor_origin = frame_origin.transform(sensor_origin);
  insertPointCloud(transformed_scan, transformed_sensor_origin, maxrange, lazy_eval, discretize);
}


template <typename NodeT>
void OccupancyMap<NodeT>::insertPointCloudRays(const Pointcloud& pc, const point3d& origin, double maxrange, bool lazy_eval) {
  if (pc.size() < 1)
    return;

#ifdef _OPENMP
  omp_set_num_threads(this->keyrays.size());
  #pragma omp parallel for
#endif
  for (int i = 0; i < (int)pc.size(); ++i) {
    const point3d& p = pc[i];
    unsigned threadIdx = 0;
#ifdef _OPENMP
    threadIdx = omp_get_thread_num();
#endif
    KeyRay* keyray = &(this->keyrays.at(threadIdx));

    if (this->computeRayKeys(origin, p, *keyray)){
#ifdef _OPENMP
      #pragma omp critical
#endif
      {
        for(KeyRay::iterator it=keyray->begin(); it != keyray->end(); it++) {
          updateNode(*it, false, lazy_eval); // insert freespace measurement
        }
        updateNode(p, true, lazy_eval); // update endpoint to be occupied
      }
    }

  }
}

template <typename NodeT>
void OccupancyMap<NodeT>::computeDiscreteUpdate(const Pointcloud& scan, const octomap::point3d& origin,
                                              KeySet& free_cells, KeySet& occupied_cells,
                                              double maxrange)
{
 Pointcloud discretePC;
 discretePC.reserve(scan.size());
 KeySet endpoints;

 for (int i = 0; i < (int)scan.size(); ++i) {
   OcTreeKey k = this->coordToKey(scan[i]);
   std::pair<KeySet::iterator,bool> ret = endpoints.insert(k);
   if (ret.second){ // insertion took place => k was not in set
     discretePC.push_back(this->keyToCoord(k));
   }
 }

 computeUpdate(discretePC, origin, free_cells, occupied_cells, maxrange);
}


template <typename NodeT>
void OccupancyMap<NodeT>::computeUpdate(const Pointcloud& scan, const octomap::point3d& origin,
                                              KeySet& free_cells, KeySet& occupied_cells,
                                              double maxrange) {
#ifdef _OPENMP
  omp_set_num_threads(this->keyrays.size());
  #pragma omp parallel for schedule(guided)
#endif
  for (int i = 0; i < (int)scan.size(); ++i) {
    const point3d& p = scan[i];
    unsigned threadIdx = 0;
#ifdef _OPENMP
    threadIdx = omp_get_thread_num();
#endif
    KeyRay* keyray = &(this->keyrays.at(threadIdx));


    if (!use_bbx_limit) { // no BBX specified
      if ((maxrange < 0.0) || ((p - origin).norm() <= maxrange) ) { // is not maxrange meas.
        // free cells
        if (this->computeRayKeys(origin, p, *keyray)){
#ifdef _OPENMP
          #pragma omp critical (free_insert)
#endif
          {
            free_cells.insert(keyray->begin(), keyray->end());
          }
        }
        // occupied endpoint
        OcTreeKey key;
        if (this->coordToKeyChecked(p, key)){
#ifdef _OPENMP
          #pragma omp critical (occupied_insert)
#endif
          {
            occupied_cells.insert(key);
          }
        }
      } else { // user set a maxrange and length is above
        point3d direction = (p - origin).normalized ();
        point3d new_end = origin + direction * (float) maxrange;
        if (this->computeRayKeys(origin, new_end, *keyray)){
#ifdef _OPENMP
          #pragma omp critical (free_insert)
#endif
          {
            free_cells.insert(keyray->begin(), keyray->end());
          }
        }
      } // end if maxrange
    } else { // BBX was set
      // endpoint in bbx and not maxrange?
      if ( inBBX(p) && ((maxrange < 0.0) || ((p - origin).norm () <= maxrange) ) )  {

        // occupied endpoint
        OcTreeKey key;
        if (this->coordToKeyChecked(p, key)){
#ifdef _OPENMP
          #pragma omp critical (occupied_insert)
#endif
          {
            occupied_cells.insert(key);
          }
        }

        // update freespace, break as soon as bbx limit is reached
        if (this->computeRayKeys(origin, p, *keyray)){
          for(KeyRay::reverse_iterator rit=keyray->rbegin(); rit != keyray->rend(); rit++) {
            if (inBBX(*rit)) {
#ifdef _OPENMP
              #pragma omp critical (free_insert)
#endif
              {
                free_cells.insert(*rit);
              }
            }
            else break;
          }
        } // end if compute ray
      } // end if in BBX and not maxrange
    } // end bbx case

  } // end for all points, end of parallel OMP loop

  // prefer occupied cells over free ones (and make sets disjunct)
  for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ){
    if (occupied_cells.find(*it) != occupied_cells.end()){
      it = free_cells.erase(it);
    } else {
      ++it;
    }
  }
}

template <typename NodeT>
NodeT* OccupancyMap<NodeT>::setNodeOccupancyAndObservationCount(
    const OcTreeKey& key, OccupancyType occupancy, CounterType observation_count, bool lazy_eval) {
  bool createdRoot = false;
  if (this->root == NULL){
    this->root = new NodeT();
    this->tree_size++;
    createdRoot = true;
  }

  return setNodeOccupancyAndObservationCountRecurs(this->root, createdRoot, key, 0, occupancy, observation_count, lazy_eval);
}

template <typename NodeT>
bool OccupancyMap<NodeT>::pruneNode(NodeT* node) {
  if (!isNodeCollapsible(node)) {
    return false;
  }

  // set value to children's values (all assumed equal)
//  node->copyData(*(node->getChild(0)));
  node->updateFromChildren();

  // delete children (known to be leafs at this point!)
  for (size_t i = 0; i < 8; i++) {
    this->deleteNodeChild(node, i);
  }
  delete[] node->children;
  node->children = nullptr;

  return true;
}

template <typename NodeT>
bool OccupancyMap<NodeT>::isNodeCollapsible(const NodeT* node) const {
  // All children must exist, must not have children of
  // their own and have the same occupancy and observation count values.
  // Exception are free space voxels.
  if (!node->hasChild(0)) {
    return false;
  }

  const NodeT* first_child = this->getNodeChild(node, 0);
  if (first_child->hasChildren()) {
    return false;
  }

  for (unsigned int i = 1; i<8; i++) {
    if (!node->hasChild(i) || this->getNodeChild(node, i)->hasChildren()) {
      return false;
    }
    const NodeT* child = this->getNodeChild(node, i);
    if (this->isNodeOccupied(child) != this->isNodeOccupied(first_child)) {
      return false;
    }
    if (this->isNodeKnown(child) != this->isNodeKnown(first_child)) {
      return false;
    }
//    if (this->isNodeFree(first_child) && this->isNodeKnown(first_child)
//        && this->isNodeFree(child) && this->isNodeKnown(child)) {
//      continue;
//    }
//    if (child->getOccupancy() != first_child->getOccupancy()) {
//      return false;
//    }
//    if (child->getObservationCount() != first_child->getObservationCount()) {
//      return false;
//    }
  }

  return true;
}

template <typename NodeT>
NodeT* OccupancyMap<NodeT>::updateNode(const OcTreeKey& key, OccupancyType occupancy_log, bool lazy_eval) {
  bool createdRoot = false;
  if (this->root == NULL){
    this->root = new NodeT();
    this->tree_size++;
    createdRoot = true;
  }

  return updateNodeRecurs(this->root, createdRoot, key, 0, occupancy_log, lazy_eval);
}

template <typename NodeT>
NodeT* OccupancyMap<NodeT>::updateNode(const OcTreeKey& key, bool occupied, bool lazy_eval) {
  bool createdRoot = false;
  if (this->root == NULL){
    this->root = new NodeT();
    this->tree_size++;
    createdRoot = true;
  }

  return updateNodeRecurs(this->root, createdRoot, key, 0, occupied, lazy_eval);
}

template <typename NodeT>
NodeT* OccupancyMap<NodeT>::updateNode(const point3d& value, bool occupied, bool lazy_eval) {
  OcTreeKey key;
  if (!this->coordToKeyChecked(value, key))
    return NULL;

  return updateNode(key, occupied, lazy_eval);
}

template <typename NodeT>
NodeT* OccupancyMap<NodeT>::updateNodeRecurs(NodeT* node, bool node_just_created, const OcTreeKey& key,
                         unsigned int depth, OccupancyType occupancy_log, bool lazy_eval) {
  bool created_node = false;

  assert(node);

  // follow down to last level
  if (depth < this->tree_depth) {
    unsigned int pos = computeChildIdx(key, this->tree_depth -1 - depth);
    if (!this->nodeChildExists(node, pos)) {
      // child does not exist, but maybe it's a pruned node?
      if (!this->nodeHasChildren(node) && !node_just_created ) {
        // current node does not have children AND it is not a new node
        // -> expand pruned node
        this->expandNode(node);
      }
      else {
        // not a pruned node, create requested child
        this->createNodeChild(node, pos);
        created_node = true;
      }
    }

    if (lazy_eval)
      return updateNodeRecurs(this->getNodeChild(node, pos), created_node, key, depth+1, occupancy_log, lazy_eval);
    else {
      NodeT* retval = updateNodeRecurs(this->getNodeChild(node, pos), created_node, key, depth+1, occupancy_log, lazy_eval);
      // prune node if possible, otherwise set own probability
      // note: combining both did not lead to a speedup!
      if (this->pruneNode(node)){
        // return pointer to current parent (pruned), the just updated node no longer exists
        retval = node;
      } else{
        node->updateFromChildren();
      }

      return retval;
    }
  }

  // at last level, update node, end of recursion
  else {
    if (use_change_detection) {
      bool occBefore = this->isNodeOccupied(node);
      updateNode(node, occupancy_log);

      if (node_just_created){  // new node
        changed_keys.insert(std::pair<OcTreeKey,bool>(key, true));
      } else if (occBefore != this->isNodeOccupied(node)) {  // occupancy changed, track it
        KeyBoolMap::iterator it = changed_keys.find(key);
        if (it == changed_keys.end())
          changed_keys.insert(std::pair<OcTreeKey,bool>(key, false));
        else if (it->second == false)
          changed_keys.erase(it);
      }
    } else {
      updateNode(node, occupancy_log);
    }
    return node;
  }
}

template <typename NodeT>
NodeT* OccupancyMap<NodeT>::updateNodeRecurs(
    NodeT* node, bool node_just_created, const OcTreeKey& key,
    unsigned int depth, bool occupied, bool lazy_eval) {
  bool created_node = false;

  assert(node);

  // follow down to last level
  if (depth < this->tree_depth) {
    unsigned int pos = computeChildIdx(key, this->tree_depth -1 - depth);
    if (!this->nodeChildExists(node, pos)) {
      // child does not exist, but maybe it's a pruned node?
      if (!this->nodeHasChildren(node) && !node_just_created ) {
        // current node does not have children AND it is not a new node
        // -> expand pruned node
        this->expandNode(node);
      }
      else {
        // not a pruned node, create requested child
        this->createNodeChild(node, pos);
        created_node = true;
      }
    }

    if (lazy_eval)
      return updateNodeRecurs(this->getNodeChild(node, pos), created_node, key, depth+1, occupied, lazy_eval);
    else {
      NodeT* retval = updateNodeRecurs(this->getNodeChild(node, pos), created_node, key, depth+1, occupied, lazy_eval);
      // prune node if possible, otherwise set own probability
      // note: combining both did not lead to a speedup!
      if (this->pruneNode(node)){
        // return pointer to current parent (pruned), the just updated node no longer exists
        retval = node;
      } else{
        node->updateFromChildren();
      }

      return retval;
    }
  }

  // at last level, update node, end of recursion
  else {
    if (use_change_detection) {
      bool occBefore = this->isNodeOccupied(node);
      updateNode(node, occupied);

      if (node_just_created){  // new node
        changed_keys.insert(std::pair<OcTreeKey,bool>(key, true));
      } else if (occBefore != this->isNodeOccupied(node)) {  // occupancy changed, track it
        KeyBoolMap::iterator it = changed_keys.find(key);
        if (it == changed_keys.end())
          changed_keys.insert(std::pair<OcTreeKey,bool>(key, false));
        else if (it->second == false)
          changed_keys.erase(it);
      }
    } else {
      updateNode(node, occupied);
    }
    return node;
  }
}

template <typename NodeT>
void OccupancyMap<NodeT>::updateNode(NodeT* node, bool occupied, bool lazy_eval) {
  if (occupied) {
    node->addObservationLogOdds(this->prob_hit_log_);
  }
  else {
    node->addObservationLogOdds(this->prob_miss_log_);
  }
}

// TODO: mostly copy of updateNodeRecurs => merge code or general tree modifier / traversal
template <typename NodeT>
NodeT* OccupancyMap<NodeT>::setNodeOccupancyAndObservationCountRecurs(
    NodeT* node, bool node_just_created, const OcTreeKey& key,
    unsigned int depth, OccupancyType occupancy, CounterType observation_count, bool lazy_eval) {
  bool created_node = false;

  assert(node);

  // follow down to last level
  if (depth < this->tree_depth) {
    unsigned int pos = computeChildIdx(key, this->tree_depth -1 - depth);
    if (!this->nodeChildExists(node, pos)) {
      // child does not exist, but maybe it's a pruned node?
      if (!this->nodeHasChildren(node) && !node_just_created ) {
        // current node does not have children AND it is not a new node
        // -> expand pruned node
        this->expandNode(node);
      }
      else {
        // not a pruned node, create requested child
        this->createNodeChild(node, pos);
        created_node = true;
      }
    }

    if (lazy_eval)
      return setNodeOccupancyAndObservationCountRecurs(
          this->getNodeChild(node, pos), created_node, key, depth+1,
          occupancy, observation_count, lazy_eval);
    else {
      NodeT* retval = setNodeOccupancyAndObservationCountRecurs(
          this->getNodeChild(node, pos), created_node, key, depth+1,
          occupancy, observation_count, lazy_eval);
      // prune node if possible, otherwise set own probability
      // note: combining both did not lead to a speedup!
      if (this->pruneNode(node)){
        // return pointer to current parent (pruned), the just updated node no longer exists
        retval = node;
      } else{
        node->updateFromChildren();
      }

      return retval;
    }
  }

  // at last level, update node, end of recursion
  else {
    if (use_change_detection) {
      bool occBefore = this->isNodeOccupied(node);
      node->setOccupancy(occupancy);
      node->setObservationCount(observation_count);

      if (node_just_created){  // new node
        changed_keys.insert(std::pair<OcTreeKey,bool>(key, true));
      } else if (occBefore != this->isNodeOccupied(node)) {  // occupancy changed, track it
        KeyBoolMap::iterator it = changed_keys.find(key);
        if (it == changed_keys.end())
          changed_keys.insert(std::pair<OcTreeKey,bool>(key, false));
        else if (it->second == false)
          changed_keys.erase(it);
      }
    } else {
      node->setOccupancy(occupancy);
      node->setObservationCount(observation_count);
    }
    return node;
  }
}

template <typename NodeT>
void OccupancyMap<NodeT>::updateInnerOccupancy(){
  if (this->root)
    this->updateInnerOccupancyRecurs(this->root, 0);
}

template <typename NodeT>
void OccupancyMap<NodeT>::updateInnerOccupancyRecurs(NodeT* node, unsigned int depth){
  assert(node);

  // only recurse and update for inner nodes:
  if (this->nodeHasChildren(node)){
    // return early for last level:
    if (depth < this->tree_depth){
      for (unsigned int i=0; i<8; i++) {
        if (this->nodeChildExists(node, i)) {
          updateInnerOccupancyRecurs(this->getNodeChild(node, i), depth+1);
        }
      }
    }
    node->updateFromChildren();
  }
}

template <typename NodeT>
bool OccupancyMap<NodeT>::castRay(const point3d& origin, const point3d& directionP, point3d& end,
                                        bool ignoreUnknown, double maxRange) const {

  /// ----------  see OcTreeBase::computeRayKeys  -----------

  // Initialization phase -------------------------------------------------------
  OcTreeKey current_key;
  if ( !OcTreeBaseImpl<NodeT,AbstractOccupancyMap<NodeT>>::coordToKeyChecked(origin, current_key) ) {
    OCTOMAP_WARNING_STR("Coordinates out of bounds during ray casting");
    return false;
  }

  NodeT* startingNode = this->search(current_key);
  if (startingNode){
    if (this->isNodeOccupied(startingNode)){
      // Occupied node found at origin
      // (need to convert from key, since origin does not need to be a voxel center)
      end = this->keyToCoord(current_key);
      return true;
    }
  } else if(!ignoreUnknown){
    end = this->keyToCoord(current_key);
    return false;
  }

  point3d direction = directionP.normalized();
  bool max_range_set = (maxRange > 0.0);

  int step[3];
  double tMax[3];
  double tDelta[3];

  for(unsigned int i=0; i < 3; ++i) {
    // compute step direction
    if (direction(i) > 0.0) step[i] =  1;
    else if (direction(i) < 0.0)   step[i] = -1;
    else step[i] = 0;

    // compute tMax, tDelta
    if (step[i] != 0) {
      // corner point of voxel (in direction of ray)
      double voxelBorder = this->keyToCoord(current_key[i]);
      voxelBorder += double(step[i] * this->resolution * 0.5);

      tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
      tDelta[i] = this->resolution / fabs( direction(i) );
    }
    else {
      tMax[i] =  std::numeric_limits<double>::max();
      tDelta[i] = std::numeric_limits<double>::max();
    }
  }

  if (step[0] == 0 && step[1] == 0 && step[2] == 0){
    OCTOMAP_ERROR("Raycasting in direction (0,0,0) is not possible!");
    return false;
  }

  // for speedup:
  double maxrange_sq = maxRange *maxRange;

  // Incremental phase  ---------------------------------------------------------

  bool done = false;

  while (!done) {
    unsigned int dim;

    // find minimum tMax:
    if (tMax[0] < tMax[1]){
      if (tMax[0] < tMax[2]) dim = 0;
      else                   dim = 2;
    }
    else {
      if (tMax[1] < tMax[2]) dim = 1;
      else                   dim = 2;
    }

    // check for overflow:
    if ((step[dim] < 0 && current_key[dim] == 0)
        || (step[dim] > 0 && current_key[dim] == 2* this->tree_max_val-1))
    {
//      OCTOMAP_WARNING("Coordinate hit bounds in dim %d, aborting raycast\n", dim);
      // return border point nevertheless:
      end = this->keyToCoord(current_key);
      return false;
    }

    // advance in direction "dim"
    current_key[dim] += step[dim];
    tMax[dim] += tDelta[dim];


    // generate world coords from key
    end = this->keyToCoord(current_key);

    // check for maxrange:
    if (max_range_set){
      double dist_from_origin_sq(0.0);
      for (unsigned int j = 0; j < 3; j++) {
        dist_from_origin_sq += ((end(j) - origin(j)) * (end(j) - origin(j)));
      }
      if (dist_from_origin_sq > maxrange_sq)
        return false;

    }

    NodeT* currentNode = this->search(current_key);
    if (currentNode){
      if (this->isNodeOccupied(currentNode)) {
        done = true;
        break;
      }
      // otherwise: node is free and valid, raycasting continues
    } else if (!ignoreUnknown){ // no node found, this usually means we are in "unknown" areas
      return false;
    }
  } // end while

  return true;
}

template <typename NodeT>
void OccupancyMap<NodeT>::allocNodeChildren(NodeT* node) {
  node->allocChildren();
}

template <typename NodeT>
void OccupancyMap<NodeT>::allocNodeChild(NodeT* node, size_t pos) {
  this->createNodeChild(node, pos);
}

template <typename NodeT>
OcTreeKey OccupancyMap<NodeT>::getRootKey() const {
  OcTreeKey key(this->tree_max_val, this->tree_max_val, this->tree_max_val);
  return key;
}

template <typename NodeT>
size_t OccupancyMap<NodeT>::getTreeMaxValue() const {
  return this->tree_max_val;
}


template <typename NodeT>
OcTreeKey OccupancyMap<NodeT>::getParentKey(const OcTreeKey& child_key, size_t child_depth) const {
  // TODO: Make sure this is correct
  OcTreeKey parent_key = child_key;
  for (size_t j = 0; j < 3; ++j) {
    parent_key[j] &= ~(1 << (this->getTreeDepth() - child_depth - 1 - 1));
    parent_key[j] |= 1 << (this->getTreeDepth() - child_depth - 1);
  }
  return parent_key;
}

template <typename NodeT>
void OccupancyMap<NodeT>::createRoot() {
  AIT_ASSERT(this->root == nullptr);
  this->root = new NodeT();
  this->tree_size++;
}

template <typename NodeT>
bool OccupancyMap<NodeT>::getRayIntersection (const point3d& origin, const point3d& direction, const point3d& center,
               point3d& intersection, double delta/*=0.0*/) const {
  // We only need three normals for the six planes
  octomap::point3d normalX(1, 0, 0);
  octomap::point3d normalY(0, 1, 0);
  octomap::point3d normalZ(0, 0, 1);

  // One point on each plane, let them be the center for simplicity
  octomap::point3d pointXNeg(center(0) - float(this->resolution / 2.0), center(1), center(2));
  octomap::point3d pointXPos(center(0) + float(this->resolution / 2.0), center(1), center(2));
  octomap::point3d pointYNeg(center(0), center(1) - float(this->resolution / 2.0), center(2));
  octomap::point3d pointYPos(center(0), center(1) + float(this->resolution / 2.0), center(2));
  octomap::point3d pointZNeg(center(0), center(1), center(2) - float(this->resolution / 2.0));
  octomap::point3d pointZPos(center(0), center(1), center(2) + float(this->resolution / 2.0));

  double lineDotNormal = 0.0;
  double d = 0.0;
  double outD = std::numeric_limits<double>::max();
  octomap::point3d intersect;
  bool found = false;

  // Find the intersection (if any) with each place
  // Line dot normal will be zero if they are parallel, in which case no intersection can be the entry one
  // if there is an intersection does it occur in the bounded plane of the voxel
  // if yes keep only the closest (smallest distance to sensor origin).
  if((lineDotNormal = normalX.dot(direction))){
    d = (pointXNeg - origin).dot(normalX) / lineDotNormal;
    intersect = direction * float(d) + origin;
    if(!(intersect(1) < (pointYNeg(1) - 1e-6) || intersect(1) > (pointYPos(1) + 1e-6) ||
       intersect(2) < (pointZNeg(2) - 1e-6) || intersect(2) > (pointZPos(2) + 1e-6))){
      outD = std::min(outD, d);
      found = true;
    }

    d = (pointXPos - origin).dot(normalX) / lineDotNormal;
    intersect = direction * float(d) + origin;
    if(!(intersect(1) < (pointYNeg(1) - 1e-6) || intersect(1) > (pointYPos(1) + 1e-6) ||
       intersect(2) < (pointZNeg(2) - 1e-6) || intersect(2) > (pointZPos(2) + 1e-6))){
      outD = std::min(outD, d);
      found = true;
    }
  }

  if((lineDotNormal = normalY.dot(direction))){
    d = (pointYNeg - origin).dot(normalY) / lineDotNormal;
    intersect = direction * float(d) + origin;
    if(!(intersect(0) < (pointXNeg(0) - 1e-6) || intersect(0) > (pointXPos(0) + 1e-6) ||
       intersect(2) < (pointZNeg(2) - 1e-6) || intersect(2) > (pointZPos(2) + 1e-6))){
      outD = std::min(outD, d);
      found = true;
    }

    d = (pointYPos - origin).dot(normalY) / lineDotNormal;
    intersect = direction * float(d) + origin;
    if(!(intersect(0) < (pointXNeg(0) - 1e-6) || intersect(0) > (pointXPos(0) + 1e-6) ||
       intersect(2) < (pointZNeg(2) - 1e-6) || intersect(2) > (pointZPos(2) + 1e-6))){
      outD = std::min(outD, d);
      found = true;
    }
  }

  if((lineDotNormal = normalZ.dot(direction))){
    d = (pointZNeg - origin).dot(normalZ) / lineDotNormal;
    intersect = direction * float(d) + origin;
    if(!(intersect(0) < (pointXNeg(0) - 1e-6) || intersect(0) > (pointXPos(0) + 1e-6) ||
       intersect(1) < (pointYNeg(1) - 1e-6) || intersect(1) > (pointYPos(1) + 1e-6))){
      outD = std::min(outD, d);
      found = true;
    }

    d = (pointZPos - origin).dot(normalZ) / lineDotNormal;
    intersect = direction * float(d) + origin;
    if(!(intersect(0) < (pointXNeg(0) - 1e-6) || intersect(0) > (pointXPos(0) + 1e-6) ||
       intersect(1) < (pointYNeg(1) - 1e-6) || intersect(1) > (pointYPos(1) + 1e-6))){
      outD = std::min(outD, d);
      found = true;
    }
  }

  // Substract (add) a fraction to ensure no ambiguity on the starting voxel
  // Don't start on a bondary.
  if(found)
    intersection = direction * float(outD + delta) + origin;

  return found;
}

template <typename NodeT>
bool OccupancyMap<NodeT>::integrateMissOnRay(const point3d& origin, const point3d& end, bool lazy_eval) {

  if (!this->computeRayKeys(origin, end, this->keyrays.at(0))) {
    return false;
  }

  for(KeyRay::iterator it=this->keyrays[0].begin(); it != this->keyrays[0].end(); it++) {
    updateNode(*it, false, lazy_eval); // insert freespace measurement
  }

  return true;
}

template <typename NodeT>
bool OccupancyMap<NodeT>::insertRay(const point3d& origin, const point3d& end, double maxrange, bool lazy_eval)
{
  // cut ray at maxrange
  if ((maxrange > 0) && ((end - origin).norm () > maxrange))
    {
      point3d direction = (end - origin).normalized ();
      point3d new_end = origin + direction * (float) maxrange;
      return integrateMissOnRay(origin, new_end,lazy_eval);
    }
  // insert complete ray
  else
    {
      if (!integrateMissOnRay(origin, end, lazy_eval)) {
        return false;
      }
      updateNode(end, true, lazy_eval); // insert hit cell
      return true;
    }
}

template <typename NodeT>
void OccupancyMap<NodeT>::setBBXMin (point3d& min) {
  bbx_min = min;
  if (!this->coordToKeyChecked(bbx_min, bbx_min_key)) {
    OCTOMAP_ERROR("ERROR while generating bbx min key.\n");
  }
}

template <typename NodeT>
void OccupancyMap<NodeT>::setBBXMax (point3d& max) {
  bbx_max = max;
  if (!this->coordToKeyChecked(bbx_max, bbx_max_key)) {
    OCTOMAP_ERROR("ERROR while generating bbx max key.\n");
  }
}


template <typename NodeT>
bool OccupancyMap<NodeT>::inBBX(const point3d& p) const {
  return ((p.x() >= bbx_min.x()) && (p.y() >= bbx_min.y()) && (p.z() >= bbx_min.z()) &&
          (p.x() <= bbx_max.x()) && (p.y() <= bbx_max.y()) && (p.z() <= bbx_max.z()) );
}


template <typename NodeT>
bool OccupancyMap<NodeT>::inBBX(const OcTreeKey& key) const {
  return ((key[0] >= bbx_min_key[0]) && (key[1] >= bbx_min_key[1]) && (key[2] >= bbx_min_key[2]) &&
          (key[0] <= bbx_max_key[0]) && (key[1] <= bbx_max_key[1]) && (key[2] <= bbx_max_key[2]) );
}

template <typename NodeT>
point3d OccupancyMap<NodeT>::getBBXBounds () const {
  octomap::point3d obj_bounds = (bbx_max - bbx_min);
  obj_bounds /= 2.;
  return obj_bounds;
}

template <typename NodeT>
point3d OccupancyMap<NodeT>::getBBXCenter () const {
  octomap::point3d obj_bounds = (bbx_max - bbx_min);
  obj_bounds /= 2.;
  return bbx_min + obj_bounds;
}


template <typename NodeT>
template <typename CollisionPredicate>
bool OccupancyMap<NodeT>::castRayFast(
    const Eigen::Vector3f& origin, const Eigen::Vector3f& direction,
    Eigen::Vector3f* end_point, NodeT const** end_node, float* end_dist_sq, size_t* end_depth,
    const double min_range, const double max_range) const {
  RayCastData ray_cast_data;
  bool result = castRayFastRecurs<CollisionPredicate>(origin, direction, ray_cast_data, min_range, max_range);
  *end_point = ray_cast_data.end_point;
  *end_node = ray_cast_data.end_node;
  *end_dist_sq = ray_cast_data.end_dist_sq;
  *end_depth = ray_cast_data.end_depth;
  return result;
}

template <typename NodeT>
template <typename CollisionPredicate>
bool OccupancyMap<NodeT>::castRayFast(
    const Eigen::Vector3f& origin, const Eigen::Vector3f& direction, RayCastData* ray_cast_data,
    const double min_range, const double max_range) const {
  RayData ray_data;
  ray_data.origin = origin;
  ray_data.direction = direction.normalized();
  ray_data.inv_direction = direction.cwiseInverse();
  const OcTreeKey cur_key = getRootKey();
  const NodeT* cur_node = this->getRoot();
  size_t cur_depth = 0;
  ray_cast_data->end_node = nullptr;
  ray_cast_data->end_dist_sq = std::numeric_limits<float>::max();
  ray_data.min_range_sq = min_range * min_range;
  if (max_range >= 0) {
    ray_data.max_range_sq = max_range * max_range;
  }
  else {
    ray_data.max_range_sq = std::numeric_limits<float>::max();
  }
  bool result = castRayFastRecurs<CollisionPredicate>(ray_data, cur_depth, cur_key, cur_node, ray_cast_data);
  return result;
}

inline float computeSquaredNorm(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
  return (a - b).squaredNorm();
}

#pragma GCC push_options
#pragma GCC optimize ("fast-math")
template <typename NodeT>
template <typename CollisionPredicate>
bool OccupancyMap<NodeT>::castRayFastRecurs(
    const RayData& ray_data, size_t cur_depth, const OcTreeKey& cur_key, const NodeT* cur_node,
    RayCastData* ray_cast_data) const {
  // Early break because of node semantics (i.e. free nodes)
  if (CollisionPredicate::earlyBreak(this, cur_node)) {
    return false;
  }

  // Compute bounding box of current node
  const Eigen::Vector3f cur_pos = this->keyToCoordEigen(cur_key, cur_depth);
  const float node_size_half = 0.5f * this->getNodeSize(cur_depth);
  const Eigen::Vector3f node_extent_half(node_size_half, node_size_half, node_size_half);
  const Eigen::Vector3f cur_bbox_min = cur_pos - node_extent_half;
  const Eigen::Vector3f cur_bbox_max = cur_pos + node_extent_half;
  bool outside_bounding_box = outsideBoundingBox(ray_data.origin, cur_bbox_min, cur_bbox_max);

  bool below_min_range = false;
  Eigen::Vector3f intersection;
  if (outside_bounding_box) {
    const float dist_to_cur_voxel_sq = computeSquaredNorm(ray_data.origin, cur_pos);
    if (dist_to_cur_voxel_sq < ray_data.min_range_sq && this->isNodeUnknown(cur_node)) {
      below_min_range = true;
    }

    if (!below_min_range) {
      // TODO: Bug in lower bound.
    // Lower bound is given by the enclosing sphere of the voxel (radius is sqrt(3) * node_size_half)
//    const float dist_to_cur_voxel_lower_bound_sq = dist_to_cur_voxel_sq - 3 * node_size_half * node_size_half;
      // Early break if node is already too far away
      // TODO: Do these checks actually help in terms of performance.
//      if (dist_to_cur_voxel_lower_bound_sq > ray_cast_data->end_dist_sq || dist_to_cur_voxel_lower_bound_sq > ray_data.max_range_sq) {
//        return false;
//      }

      // Check if ray intersects current node
    //  bool intersect = intersectRayVoxel(ray_data, cur_pos, cur_depth, &intersection);
      const bool intersect = intersectRayBoundingBox(ray_data, cur_bbox_min, cur_bbox_max, &intersection);
      // Break because node is not intersecting
      if (!intersect || computeSquaredNorm(ray_data.origin, intersection) > ray_data.max_range_sq) {
        return false;
      }
    }
  }
  else {
    below_min_range = true;
  }

//  if (!this->isNodeOccupied(cur_node) && this->isNodeKnown(cur_node)) {
//    return false;
//  }

  if (!cur_node->hasChildren()) {
    // We reached a leaf node
    if (!below_min_range && CollisionPredicate::collision(this, cur_node)) {
      if (!outside_bounding_box) {
        // If already inside the bounding box we want the intersection point to be the start of the ray.
        intersection = ray_data.origin;
      }
      ray_cast_data->end_key = cur_key;
      ray_cast_data->end_point = intersection;
      ray_cast_data->end_node = cur_node;
      ray_cast_data->end_dist_sq = computeSquaredNorm(ray_data.origin, ray_cast_data->end_point);
      ray_cast_data->end_depth = cur_depth;
      return true;
    }
    else {
      return false;
    }
  }

  bool child_hit = false;

  const size_t next_depth = cur_depth + 1;
  const unsigned short int center_offset_key = this->tree_max_val >> next_depth;
  for (size_t i = 0; i < 8; ++i) {
    if (cur_node->hasChild(i)) {
      OcTreeKey next_key;
      octomap::computeChildKey(i, center_offset_key, cur_key, next_key);
      const NodeT* next_node = static_cast<const NodeT*>(cur_node->getChild(i));
      RayCastData ray_cast_data_tmp;
      ray_cast_data_tmp.end_dist_sq = ray_cast_data->end_dist_sq;
      bool result = castRayFastRecurs<CollisionPredicate>(ray_data, next_depth, next_key, next_node,
          &ray_cast_data_tmp);
      if (result) {
        if (ray_cast_data_tmp.end_dist_sq < ray_cast_data->end_dist_sq) {
          child_hit = true;
          *ray_cast_data = ray_cast_data_tmp;
        }
      }
    }
  }

  return child_hit;
}

template <typename NodeT>
bool OccupancyMap<NodeT>::outsideBoundingBox(const Eigen::Vector3f& pos, const Eigen::Vector3f& bbox_min, const Eigen::Vector3f& bbox_max) const {
  return pos(0) < bbox_min(0) || pos(0) > bbox_max(0)
      || pos(1) < bbox_min(1) || pos(1) > bbox_max(1)
      || pos(2) < bbox_min(2) || pos(2) > bbox_max(2);
}

template <typename NodeT>
bool OccupancyMap<NodeT>::intersectRayVoxel(const RayData& ray_data, const Eigen::Vector3f& cur_pos, size_t cur_depth, Eigen::Vector3f* intersection) const {
  const float node_size_half = 0.5f * this->getNodeSize(cur_depth);
  const Eigen::Vector3f node_extent_half(node_size_half, node_size_half, node_size_half);
  const Eigen::Vector3f cur_bbox_min = cur_pos - node_extent_half;
  const Eigen::Vector3f cur_bbox_max = cur_pos + node_extent_half;
  return intersectRayBoundingBox(ray_data, cur_bbox_min, cur_bbox_max, intersection);
}

template <typename NodeT>
bool OccupancyMap<NodeT>::intersectRayBoundingBox(const RayData& ray_data,
    const Eigen::Vector3f& bbox_min, const Eigen::Vector3f& bbox_max, Eigen::Vector3f* intersection) const {
//  std::cout << "  intersecting node at depth " << cur_depth << " with size " << node_size_half * 2 << std::endl;
  float t_min = -std::numeric_limits<float>::max();
  float t_max = std::numeric_limits<float>::max();

//  for (size_t i = 0; i < 3; ++i) {
//    if (ray_data.direction(i) != 0) {
//      float t0 = (bbox_min(i) - ray_data.origin(i)) * ray_data.inv_direction(i);
//      float t1 = (bbox_max(i) - ray_data.origin(i)) * ray_data.inv_direction(i);
//      t_min = std::max(t_min, std::min(t0, t1));
//      t_max = std::min(t_max, std::max(t0, t1));
//    }
//    else {
//      if (ray_data.origin(i) <= bbox_min(i) || ray_data.origin(i) >= bbox_max(i)) {
//        return false;
//      }
//    }
//  }

  // Faster version without explicit check for directions parallel to an axis
  for (size_t i = 0; i < 3; ++i) {
    float t0 = (bbox_min(i) - ray_data.origin(i)) * ray_data.inv_direction(i);
    float t1 = (bbox_max(i) - ray_data.origin(i)) * ray_data.inv_direction(i);
    t_min = std::max(t_min, std::min(t0, t1));
    t_max = std::min(t_max, std::max(t0, t1));
  }

  bool intersect = t_max > std::max(t_min, 0.0f);
  if (intersect) {
    t_min = std::max(t_min, 0.0f);
    *intersection = ray_data.origin + ray_data.direction * t_min;
  }
  return intersect;
}
#pragma GCC pop_options

template <typename NodeT>
std::unique_ptr<OccupancyMap<NodeT>> OccupancyMap<NodeT>::read(const std::string& filename) {
  std::unique_ptr<OccupancyMap<NodeT>> tree(reinterpret_cast<OccupancyMap<NodeT>*>(octomap::AbstractOcTree::read(filename)));
  return std::move(tree);
}
