//==================================================
// occupancy_map.h
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

#include <list>
#include <stdlib.h>
#include <vector>
#include <memory>

#include <boost/iterator/iterator_facade.hpp>

#include <octomap/octomap_types.h>
#include <octomap/octomap_utils.h>
#include <octomap/OcTreeBaseImpl.h>
#include <octomap/AbstractOccupancyOcTree.h>
#include <ait/eigen.h>
#include <src/octree/occupancy_node.h>

using octomap::OcTreeKey;
using octomap::KeyBoolMap;
using octomap::KeySet;
using octomap::KeyRay;
using octomap::Pointcloud;
using octomap::point3d;
using octomap::pose6d;
using octomap::probability;
using octomap::logodds;
using octomap::OcTreeBaseImpl;

template <typename NodeT>
class AbstractOccupancyMap : public octomap::AbstractOcTree {
public:
  using NodeType = NodeT;

  using OccupancyType = typename NodeT::OccupancyType;
  using CounterType = typename NodeT::CounterType;

  AbstractOccupancyMap();
  virtual ~AbstractOccupancyMap() {};

  // -- occupancy queries

  /// queries whether a node is occupied according to the tree's parameter for "occupancy"
  inline bool isNodeOccupied(const NodeT* occupancy_node) const {
    return getNodeOccupancy(occupancy_node) > this->occ_prob_thres_;
  }

  /// queries whether a node is occupied according to the tree's parameter for "occupancy"
  inline bool isNodeOccupied(const OccupancyType occupancy) const {
    return occupancy > this->occ_prob_thres_;
  }

  inline bool isNodeFree(const NodeT* occupancy_node) const {
    return !isNodeOccupied(occupancy_node);
  }

  inline bool isNodeFree(const OccupancyType occupancy) const {
    return !isNodeOccupied(occupancy);
  }

  inline bool isNodeKnown(const NodeT* occupancy_node) const {
    return getNodeObservationCount(occupancy_node) >= this->observation_thres_;
  }

  inline bool isNodeKnown(const CounterType observation_count) const {
    return observation_count >= this->observation_thres_;
  }

  inline bool isNodeUnknown(const NodeT* occupancy_node) const {
    return !isNodeKnown(occupancy_node);
  }

  inline bool isNodeUnknown(const CounterType observation_count) const {
    return !isNodeKnown(observation_count);
  }

  OccupancyType getNodeOccupancy(const NodeT* occupancy_node) const {
    return occupancy_node->getOccupancy();
  }

  CounterType getNodeObservationCount(const NodeT* occupancy_node) const {
    return occupancy_node->getObservationCount();
  }

  // - update functions

  /**
   * Manipulate log_odds value of voxel directly
   *
   * @param key of the NODE that is to be updated
   * @param log_odds_update value to be added (+) to log_odds value of node
   * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
   *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
   * @return pointer to the updated NODE
   */
  virtual NodeT* setNodeOccupancyAndObservationCount(
      const OcTreeKey& key, OccupancyType occupancy, CounterType observation_count, bool lazy_eval = false) = 0;

  /**
   * Integrate occupancy measurement.
   *
   * @param key of the NODE that is to be updated
   * @param occupied true if the node was measured occupied, else false
   * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
   *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
   * @return pointer to the updated NODE
   */
  virtual NodeT* updateNode(const OcTreeKey& key, OccupancyType update_log_odds, bool lazy_eval = false) = 0;

  /**
   * Integrate occupancy measurement.
   *
   * @param key of the NODE that is to be updated
   * @param occupied true if the node was measured occupied, else false
   * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
   *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
   * @return pointer to the updated NODE
   */
  virtual NodeT* updateNode(const OcTreeKey& key, bool occupied, bool lazy_eval = false) = 0;

  virtual NodeT* updateNode(const point3d& value, bool occupied, bool lazy_eval = false) = 0;

  //-- parameters for occupancy and sensor model:

  /// @return threshold (probability) for occupancy - sensor model
  OccupancyType getOccupancyThres() const {
    return occ_prob_thres_;
  }

  /// sets the threshold for occupancy (sensor model)
  void setOccupancyThres(OccupancyType prob) {
    occ_prob_thres_ = prob;
  }

  /// @return threshold for being known
  CounterType getObservationThres() const {
    return observation_thres_;
  }

  /// sets the threshold for being known
  void setObservationThres(CounterType observation_thres) {
    observation_thres_ = observation_thres;
  }

  void setProbHit(OccupancyType prob) {
    prob_hit_log_ = octomap::logodds(prob);
  }

  OccupancyType getProbHit() const {
    return prob_hit_log_;
  }

  void setProbMiss(OccupancyType prob) {
    prob_miss_log_ = octomap::logodds(prob);
  }

  OccupancyType setProbMiss() const {
    return prob_miss_log_;
  }

protected:
  OccupancyType prob_hit_log_;
  OccupancyType prob_miss_log_;
  OccupancyType occ_prob_thres_;
  CounterType observation_thres_;
};


template <typename TreeT, typename NodeT>
class TreeNavigator {
public:
  static TreeNavigator getRootNavigator(TreeT* tree);

  TreeNavigator(TreeT* tree, OcTreeKey key, NodeT* node, size_t depth);

  template <typename OtherTreeT, typename OtherNodeT>
  TreeNavigator(const TreeNavigator<OtherTreeT, OtherNodeT>& other);

  TreeT* getTree();
  const OcTreeKey& getKey() const;
  size_t getDepth() const;

  const NodeT* getNode() const;
  NodeT* getNode();
  const NodeT* operator->() const;
  NodeT* operator->();
  const NodeT* operator*() const;
  NodeT* operator*();

  Eigen::Vector3f getPosition() const;
  float getSize() const;

  bool hasParent() const;
  bool hasChildren() const;
  bool hasChild(size_t i) const;

  void gotoChild(size_t i);
  void gotoParent();

  TreeNavigator child(size_t i) const;
  TreeNavigator parent() const;

private:
  template <typename, typename>
  friend class TreeNavigator;

  TreeT* tree_;
  OcTreeKey key_;
  NodeT* node_;
  size_t depth_;
};


template <typename NodeT>
class OccupancyMap : public OcTreeBaseImpl<NodeT, AbstractOccupancyMap<NodeT>> {
public:
  using TreeNavigatorType = TreeNavigator<OccupancyMap, NodeT>;
  using ConstTreeNavigatorType = TreeNavigator<const OccupancyMap, const NodeT>;
  using OccupancyType = typename NodeT::OccupancyType;
  using CounterType = typename NodeT::CounterType;

  /// Default constructor, sets resolution of leafs
  OccupancyMap(double resolution);
  virtual ~OccupancyMap();

  OccupancyMap* create() const {
    return new OccupancyMap(this->resolution);
  }

  /// Copy constructor
  OccupancyMap(const OccupancyMap& rhs);

  std::string getTreeType() const override {
    return "Ait_OccupancyMap";
  }

  /**
  * Integrate a Pointcloud (in global reference frame), parallelized with OpenMP.
  * Special care is taken that each voxel
  * in the map is updated only once, and occupied nodes have a preference over free ones.
  * This avoids holes in the floor from mutual deletion and is more efficient than the plain
  * ray insertion in insertPointCloudRays().
  *
  * @note replaces insertScan()
  *
  * @param scan Pointcloud (measurement endpoints), in global reference frame
  * @param sensor_origin measurement origin in global reference frame
  * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
  * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
  *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
  * @param discretize whether the scan is discretized first into octree key cells (default: false).
  *   This reduces the number of raycasts using computeDiscreteUpdate(), resulting in a potential speedup.*
  */
  virtual void insertPointCloud(const Pointcloud& scan, const octomap::point3d& sensor_origin,
                 double maxrange=-1., bool lazy_eval = false, bool discretize = false);

  /**
  * Integrate a 3d scan (transform scan before tree update), parallelized with OpenMP.
  * Special care is taken that each voxel
  * in the map is updated only once, and occupied nodes have a preference over free ones.
  * This avoids holes in the floor from mutual deletion and is more efficient than the plain
  * ray insertion in insertPointCloudRays().
  *
  * @note replaces insertScan()
  *
  * @param scan Pointcloud (measurement endpoints) relative to frame origin
  * @param sensor_origin origin of sensor relative to frame origin
  * @param frame_origin origin of reference frame, determines transform to be applied to cloud and sensor origin
  * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
  * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
  *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
  * @param discretize whether the scan is discretized first into octree key cells (default: false).
  *   This reduces the number of raycasts using computeDiscreteUpdate(), resulting in a potential speedup.*
  */
  virtual void insertPointCloud(const Pointcloud& scan, const point3d& sensor_origin, const pose6d& frame_origin,
                 double maxrange=-1., bool lazy_eval = false, bool discretize = false);

  /**
   * Integrate a Pointcloud (in global reference frame), parallelized with OpenMP.
   * This function simply inserts all rays of the point clouds as batch operation.
   * Discretization effects can lead to the deletion of occupied space, it is
   * usually recommended to use insertPointCloud() instead.
   *
   * @param scan Pointcloud (measurement endpoints), in global reference frame
   * @param sensor_origin measurement origin in global reference frame
   * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
   * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
   *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
   */
   virtual void insertPointCloudRays(const Pointcloud& scan, const point3d& sensor_origin, double maxrange = -1., bool lazy_eval = false);

   NodeT* setNodeOccupancyAndObservationCount(
       const OcTreeKey& key, OccupancyType occupancy, CounterType observation_count, bool lazy_eval);

   /**
    * Integrate occupancy measurement.
    *
    * @param key of the NODE that is to be updated
    * @param occupied true if the node was measured occupied, else false
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   virtual NodeT* updateNode(const OcTreeKey& key, OccupancyType update_log_odds, bool lazy_eval = false);

   /**
    * Integrate occupancy measurement.
    *
    * @param key of the NODE that is to be updated
    * @param occupied true if the node was measured occupied, else false
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   virtual NodeT* updateNode(const OcTreeKey& key, bool occupied, bool lazy_eval = false);

   virtual NodeT* updateNode(const point3d& value, bool occupied, bool lazy_eval = false);

   virtual void updateNode(NodeT* node, bool occupied, bool lazy_eval = false);

  virtual bool insertRay(const point3d& origin, const point3d& end, double maxrange=-1.0, bool lazy_eval = false);

  /**
   * Performs raycasting in 3d, similar to computeRay(). Can be called in parallel e.g. with OpenMP
   * for a speedup.
   *
   * A ray is cast from 'origin' with a given direction, the first non-free
   * cell is returned in 'end' (as center coordinate). This could also be the
   * origin node if it is occupied or unknown. castRay() returns true if an occupied node
   * was hit by the raycast. If the raycast returns false you can search() the node at 'end' and
   * see whether it's unknown space.
   *
   *
   * @param[in] origin starting coordinate of ray
   * @param[in] direction A vector pointing in the direction of the raycast (NOT a point in space). Does not need to be normalized.
   * @param[out] end returns the center of the last cell on the ray. If the function returns true, it is occupied.
   * @param[in] ignoreUnknownCells whether unknown cells are ignored (= treated as free). If false (default), the raycast aborts when an unknown cell is hit and returns false.
   * @param[in] maxRange Maximum range after which the raycast is aborted (<= 0: no limit, default)
   * @return true if an occupied cell was hit, false if the maximum range or octree bounds are reached, or if an unknown node was hit.
   */
  virtual bool castRay(const point3d& origin, const point3d& direction, point3d& end,
               bool ignoreUnknownCells=false, double maxRange=-1.0) const;

  OcTreeKey getRootKey() const;

  size_t getTreeMaxValue() const;

  OcTreeKey getParentKey(const OcTreeKey& child_key, size_t child_depth) const;

  void createRoot();

  void allocNodeChildren(NodeT* node);

  void allocNodeChild(NodeT* node, size_t pos);

  /// converts from a discrete key at the lowest tree level into a coordinate
  /// corresponding to the key's center
  inline float keyToCoordFloat(unsigned short int key) const{
    return (float( (int) key - (int) this->tree_max_val ) + 0.5f) * this->resolution;
  }

  inline float keyToCoordFloat(unsigned short int key, unsigned depth) const{
    assert(depth <= this->tree_depth);
    // root is centered on 0 = 0.0
    if (depth == 0) {
      return 0.0f;
    } else if (depth == this->tree_depth) {
      return this->keyToCoord(key);
    } else {
      return (floor( (float(key)-float(this->tree_max_val)) / float(1 << (this->tree_depth - depth)) )  + 0.5f ) * (float)this->getNodeSize(depth);
    }
  }

  /// converts from an addressing key at the lowest tree level into a coordinate
  /// corresponding to the key's center
  inline Eigen::Vector3f keyToCoordEigen(const OcTreeKey& key) const{
    return Eigen::Vector3f(float(this->keyToCoordFloat(key[0])), float(this->keyToCoordFloat(key[1])), float(this->keyToCoordFloat(key[2])));
  }

  /// converts from an addressing key at a given depth into a coordinate
  /// corresponding to the key's center
  inline Eigen::Vector3f keyToCoordEigen(const OcTreeKey& key, unsigned depth) const{
    return Eigen::Vector3f(float(this->keyToCoordFloat(key[0], depth)), float(this->keyToCoordFloat(key[1], depth)), float(this->keyToCoordFloat(key[2], depth)));
  }

  struct RayData {
    Eigen::Vector3f origin;
    Eigen::Vector3f direction;
    Eigen::Vector3f inv_direction;
    float min_range_sq;
    float max_range_sq;
  };

  struct RayCastData {
    OcTreeKey end_key;
    Eigen::Vector3f end_point;
    const NodeT* end_node;
    float end_dist_sq;
    size_t end_depth;
  };

  struct CollisionKnownAndOccupied {
    CollisionKnownAndOccupied() = delete;
    static bool collision(const OccupancyMap* map, const NodeT* node) {
      return map->isNodeOccupied(node) && map->isNodeKnown(node);
    }
    static bool earlyBreak(const OccupancyMap* map, const NodeT* node) {
      return !collision(map, node);
    }
  };

  struct CollisionUnknownOrOccupied {
    CollisionUnknownOrOccupied() = delete;
    static bool collision(const OccupancyMap* map, const NodeT* node) {
      return map->isNodeOccupied(node) || map->isNodeUnknown(node);
    }
    static bool earlyBreak(const OccupancyMap* map, const NodeT* node) {
      return !collision(map, node);
    }
  };

  struct CollisionUnknown {
    CollisionUnknown() = delete;
    static bool collision(const OccupancyMap* map, const NodeT* node) {
      return map->isNodeUnknown(node);
    }
    static bool earlyBreak(const OccupancyMap* map, const NodeT* node) {
//      return map->isNodeFree()
      return false;
    }
  };

  template <typename CollisionPredicate>
  bool castRayFast(const Eigen::Vector3f& origin, const Eigen::Vector3f& direction,
      Eigen::Vector3f* end_point, NodeT const** end_node, float* end_dist_sq, size_t* end_depth,
      const double min_range=0, const double max_range=-1.0) const;

  template <typename CollisionPredicate>
  bool castRayFast(const Eigen::Vector3f& origin, const Eigen::Vector3f& direction, RayCastData* ray_cast_data,
      const double min_range=0, const double max_range=-1.0) const;

  template <typename CollisionPredicate>
  bool castRayFastRecurs(
      const RayData& ray_data, size_t cur_depth, const OcTreeKey& cur_key, const NodeT* cur_node,
          RayCastData* ray_cast_data) const;

  inline bool outsideBoundingBox(const Eigen::Vector3f& pos, const Eigen::Vector3f& bbox_min, const Eigen::Vector3f& bbox_max) const;
  inline bool intersectRayVoxel(const RayData& ray_data, const Eigen::Vector3f& cur_pos, size_t cur_depth, Eigen::Vector3f* intersection) const;
  inline bool intersectRayBoundingBox(const RayData& ray_data, const Eigen::Vector3f& bbox_min, const Eigen::Vector3f& bbox_max,
      Eigen::Vector3f* intersection) const;

  virtual bool getRayIntersection(const point3d& origin, const point3d& direction, const point3d& center,
               point3d& intersection, double delta=0.0) const;

  bool pruneNode(NodeT* node);
  bool isNodeCollapsible(const NodeT* node) const;

  //-- set BBX limit (limits tree updates to this bounding box)

  ///  use or ignore BBX limit (default: ignore)
  void useBBXLimit(bool enable) { use_bbx_limit = enable; }
  bool bbxSet() const { return use_bbx_limit; }
  /// sets the minimum for a query bounding box to use
  void setBBXMin (point3d& min);
  /// sets the maximum for a query bounding box to use
  void setBBXMax (point3d& max);
  /// @return the currently set minimum for bounding box queries, if set
  point3d getBBXMin () const { return bbx_min; }
  /// @return the currently set maximum for bounding box queries, if set
  point3d getBBXMax () const { return bbx_max; }
  point3d getBBXBounds () const;
  point3d getBBXCenter () const;
  /// @return true if point is in the currently set bounding box
  bool inBBX(const point3d& p) const;
  /// @return true if key is in the currently set bounding box
  bool inBBX(const OcTreeKey& key) const;

  //-- change detection on occupancy:
  /// track or ignore changes while inserting scans (default: ignore)
  void enableChangeDetection(bool enable) { use_change_detection = enable; }
  bool isChangeDetectionEnabled() const { return use_change_detection; }
  /// Reset the set of changed keys. Call this after you obtained all changed nodes.
  void resetChangeDetection() { changed_keys.clear(); }

  /**
   * Iterator to traverse all keys of changed nodes.
   * you need to enableChangeDetection() first. Here, an OcTreeKey always
   * refers to a node at the lowest tree level (its size is the minimum tree resolution)
   */
  KeyBoolMap::const_iterator changedKeysBegin() const {return changed_keys.begin();}

  /// Iterator to traverse all keys of changed nodes.
  KeyBoolMap::const_iterator changedKeysEnd() const {return changed_keys.end();}

  /// Number of changes since last reset.
  size_t numChangesDetected() const { return changed_keys.size(); }


  /**
   * Helper for insertPointCloud(). Computes all octree nodes affected by the point cloud
   * integration at once. Here, occupied nodes have a preference over free
   * ones.
   *
   * @param scan point cloud measurement to be integrated
   * @param origin origin of the sensor for ray casting
   * @param free_cells keys of nodes to be cleared
   * @param occupied_cells keys of nodes to be marked occupied
   * @param maxrange maximum range for raycasting (-1: unlimited)
   */
  void computeUpdate(const Pointcloud& scan, const octomap::point3d& origin,
                     KeySet& free_cells,
                     KeySet& occupied_cells,
                     double maxrange);


  /**
   * Helper for insertPointCloud(). Computes all octree nodes affected by the point cloud
   * integration at once. Here, occupied nodes have a preference over free
   * ones. This function first discretizes the scan with the octree grid, which results
   * in fewer raycasts (=speedup) but a slightly different result than computeUpdate().
   *
   * @param scan point cloud measurement to be integrated
   * @param origin origin of the sensor for ray casting
   * @param free_cells keys of nodes to be cleared
   * @param occupied_cells keys of nodes to be marked occupied
   * @param maxrange maximum range for raycasting (-1: unlimited)
   */
  void computeDiscreteUpdate(const Pointcloud& scan, const octomap::point3d& origin,
                     KeySet& free_cells,
                     KeySet& occupied_cells,
                     double maxrange);


  // -- I/O  -----------------------------------------

  /**
   * Updates the occupancy of all inner nodes to reflect their children's occupancy.
   * If you performed batch-updates with lazy evaluation enabled, you must call this
   * before any queries to ensure correct multi-resolution behavior.
   **/
  void updateInnerOccupancy();

  static std::unique_ptr<OccupancyMap> read(const std::string& filename);

protected:
  /**
   * Static member object which ensures that this OcTree's prototype
   * ends up in the classIDMapping only once. You need this as a
   * static member in any derived octree class in order to read .ot
   * files through the AbstractOcTree factory. You should also call
   * ensureLinking() once from the constructor.
   */
  class StaticMemberInitializer{
  public:
    StaticMemberInitializer() {
      OccupancyMap* tree = new OccupancyMap(0.1);
      tree->clearKeyRays();
      octomap::AbstractOcTree::registerTreeType(tree);
    }

    /**
     * Dummy function to ensure that MSVC does not drop the
     * StaticMemberInitializer, causing this tree failing to register.
     * Needs to be called from the constructor of this octree.
     */
    void ensureLinking() {};
  };

  /// to ensure static initialization (only once)
  static StaticMemberInitializer ocTreeMemberInit;

  /// Constructor to enable derived classes to change tree constants.
  /// This usually requires a re-implementation of some core tree-traversal functions as well!
  OccupancyMap(double resolution, unsigned int tree_depth, unsigned int tree_max_val);

  /**
   * Traces a ray from origin to end and updates all voxels on the
   *  way as free.  The volume containing "end" is not updated.
   */
  bool integrateMissOnRay(const point3d& origin, const point3d& end, bool lazy_eval = false);


  // recursive calls ----------------------------

  NodeT* updateNodeRecurs(NodeT* node, bool node_just_created, const OcTreeKey& key,
                         unsigned int depth, bool occupied, bool lazy_eval = false);

  NodeT* updateNodeRecurs(NodeT* node, bool node_just_created, const OcTreeKey& key,
                         unsigned int depth, OccupancyType occupancy_log, bool lazy_eval = false);

  NodeT* setNodeOccupancyAndObservationCountRecurs(NodeT* node, bool node_just_created, const OcTreeKey& key,
                         unsigned int depth, OccupancyType occupancy, CounterType observation_count, bool lazy_eval = false);

  void updateInnerOccupancyRecurs(NodeT* node, unsigned int depth);

protected:
  bool use_bbx_limit;  ///< use bounding box for queries (needs to be set)?
  point3d bbx_min;
  point3d bbx_max;
  OcTreeKey bbx_min_key;
  OcTreeKey bbx_max_key;

  bool use_change_detection;
  /// Set of leaf keys (lowest level) which changed since last resetChangeDetection
  KeyBoolMap changed_keys;
};

OccupancyMap<AugmentedOccupancyNode>* convertToAugmentedMap(const OccupancyMap<OccupancyNode>* input_tree);


#include <src/octree/occupancy_map.hxx>
#include <src/octree/occupancy_map_tree_navigator.hxx>
