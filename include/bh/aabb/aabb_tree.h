//==================================================
// aabb_tree.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 17.03.17
//

#pragma once

#include "../math/geometry.h"

namespace bh {

///
/// Result object for ray intersection with AABB tree.
/// Keep it outside of the AABBTree class so that we do share the same type
/// between multiple collider trees.
///
template <typename FloatT>
class AABBTreeRayIntersection {
public:
  AABBTreeRayIntersection();

  AABBTreeRayIntersection(const size_t node_index, const size_t depth, const FloatT ray_t);

  bool doesIntersect() const;

  size_t nodeIndex() const;

  size_t nodeIndex();

  size_t depth() const;

  FloatT rayT() const;

private:
  size_t node_index_;
  size_t depth_;
  FloatT ray_t_;
};

///
/// AABB tree for collision detection with arbitrary collider objects.
/// The tree is build using the median element along each axis (cyclic) for the split.
/// The collider object has to define two methods:
///     SimpleRayIntersection intersect(const RayDataType& ray, const FloatT t_min, const FloatT t_max) const;
///     BoundingBoxType boundingBox() const;
/// The intersect() method computes the intersection of the collider object with a ray. The ray is considered
/// to intersect only if the ray coefficient t fulfils t_min <= t <= t_max.
/// The boundingBox() method returns an axis-aligned bounding box that contains the collider object.
/// Ideally this is a minimum bounding box.
///
template <typename ColliderT, typename FloatT = float>
class AABBTree {

public:
  using size_t = std::size_t;
  using ColliderType = ColliderT;
  using FloatType = FloatT;

  using BoundingBoxType = BoundingBox3D<FloatT>;
  using RayType = Ray<FloatT>;
  using RayDataType = RayData<FloatT>;
  using SimpleRayIntersection = bh::RayIntersection<FloatT>;
  using RayIntersection = bh::AABBTreeRayIntersection<FloatT>;

  class Node;
  using NodeContainer = std::vector<Node>;
  using NodeIterator = typename NodeContainer::iterator;
  using ConstNodeIterator = typename NodeContainer::const_iterator;

  static const size_t kNoChild = size_t(-1);

  class ColliderPointerWrapper {
  public:
    ColliderPointerWrapper(ColliderT* collider_ptr)
        : collider_ptr_(collider_ptr) {}

    const ColliderT* colliderPtr() const;

    ColliderT* colliderPtr();

    const ColliderT collider() const;

    ColliderT collider();

    RayIntersection intersect(const RayDataType& ray, const FloatT t_min, const FloatT t_max) const;

    BoundingBoxType boundingBox() const;

  private:
    ColliderT* collider_ptr_;
  };

  class Node {
  public:
    Node();

    Node(const size_t left_child_index, const size_t right_child_index,
         const ColliderT& collider, const BoundingBoxType& bounding_box);

    bool isLeaf() const;

    bool hasLeftChild() const;

    size_t leftChildIndex() const;

    bool hasRightChild() const;

    size_t rightChildIndex() const;

    const BoundingBoxType& boundingBox() const;

    const ColliderT& collider() const;

    ColliderT& collider();

  private:
    friend class AABBTree;

    size_t left_child_index_;
    size_t right_child_index_;
    BoundingBoxType bounding_box_;
    ColliderT collider_;
  };

  template <typename ColliderContainer>
  AABBTree(const ColliderContainer& container);

  template <typename ColliderIterator>
  AABBTree(const ColliderIterator first, const ColliderIterator last);

  // TODO: Make info accesible via methods
  void printInfo() const;

  size_t numOfNodes() const;

  const Node& getRoot() const;

  Node& getRoot();

  const Node& getNode(const size_t node_index) const;

  Node& getNode(const size_t node_index);

  NodeIterator begin();

  NodeIterator end();

  ConstNodeIterator begin() const;

  ConstNodeIterator end() const;

  const ColliderT& getCollider(const size_t node_index) const;

  ColliderT& getCollider(const size_t node_index);

  RayIntersection intersect(const RayType& ray,
                            const FloatT t_min = 0,
                            const FloatT t_max = std::numeric_limits<FloatT>::max()) const;

  RayIntersection intersect(const RayDataType& ray,
                            const FloatT t_min = 0,
                            const FloatT t_max = std::numeric_limits<FloatT>::max()) const;

  std::vector<RayIntersection> intersectRange(const RayType& ray,
                                              const FloatT t_min = 0,
                                              const FloatT t_max = std::numeric_limits<FloatT>::max()) const;

  std::vector<RayIntersection> intersectRange(const RayDataType& ray,
                                              const FloatT t_min = 0,
                                              const FloatT t_max = std::numeric_limits<FloatT>::max()) const;

private:
  RayIntersection _intersect(const RayDataType& ray,
                             const FloatT t_min = 0,
                             const FloatT t_max = std::numeric_limits<FloatT>::max()) const;

  bool _intersectRecursive(const size_t node_index,
                           const size_t depth,
                           const RayDataType& ray,
                           const FloatT t_min,
                           const FloatT t_max,
                           RayIntersection* cri_result) const;

  std::vector<RayIntersection> _intersectRange(const RayDataType& ray,
                                               const FloatT t_min = 0,
                                               const FloatT t_max = std::numeric_limits<FloatT>::max()) const;

  bool _intersectRangeRecursive(const size_t node_index,
                                const size_t depth,
                                const RayDataType& ray,
                                const FloatT t_min,
                                const FloatT t_max,
                                std::vector<RayIntersection>* cri_results) const;

  template <typename ColliderIterator>
  void build(const ColliderIterator first, const ColliderIterator last);

  template <typename ColliderIterator>
  void splitMedianRecursive(const size_t node_index,
                            const typename std::vector<ColliderIterator>::iterator first,
                            const typename std::vector<ColliderIterator>::iterator last,
                            const size_t sort_axis);

  size_t allocateNode();

  void updateBoundingBox(Node* node);

  void computeInfoRecursive(const Node* node, const size_t cur_depth,
                            size_t* num_leaf_nodes, size_t* max_depth) const;

  NodeContainer nodes_;
};

}

#include "aabb_tree.hxx"
