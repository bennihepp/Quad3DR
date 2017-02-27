//==================================================
// bvh.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 17, 2016
//==================================================
#pragma once

#include <iostream>
#include <algorithm>
#include <utility>
#include <boost/serialization/access.hpp>
#include <boost/iterator_adaptors.hpp>
#include <ait/common.h>
#include <ait/eigen.h>
#include <ait/geometry.h>
#include <ait/utilities.h>
#if WITH_CUDA
  #include <ait/cuda_utils.h>
  #include "bvh.cuh"
#endif

namespace bvh {

using ait::Ray;
using ait::RayData;
using ait::BoundingBox3D;

#if __GNUC__ && !__CUDACC__
  #pragma GCC push_options
  #pragma GCC optimize ("fast-math")
#endif

template <typename FloatType = float>
class BoundingBox3DInterface {
public:
  virtual ~BoundingBox3DInterface() {}

  virtual BoundingBox3D<FloatType> getBoundingBox() const = 0;
};

template <typename ObjectType, typename FloatType = float>
class Tree;

template <typename ObjectType, typename FloatType = float>
class Node {
public:
  using BoundingBoxType = BoundingBox3D<FloatType>;

  Node()
  : left_child_(nullptr), right_child_(nullptr), object_(nullptr) {}

  ~Node() {}

  const BoundingBoxType& getBoundingBox() const {
    return bounding_box_;
  }

  bool hasLeftChild() const {
    return left_child_ != nullptr;
  }

  const Node* getLeftChild() const {
    return left_child_;
  }

  Node* getLeftChild() {
    return left_child_;
  }

  bool hasRightChild() const {
    return right_child_ != nullptr;
  }

  const Node* getRightChild() const {
    return right_child_;
  }

  Node* getRightChild() {
    return right_child_;
  }

  const ObjectType* getObject() const {
    return object_;
  }

  ObjectType* getObject() {
    return object_;
  }

  bool isLeaf() const {
    return left_child_ == nullptr && right_child_ == nullptr;
  }

  void computeBoundingBox() {
    if (left_child_ != nullptr && right_child_ != nullptr) {
      bounding_box_ = BoundingBox3D<FloatType>::getUnion(left_child_->bounding_box_, right_child_->bounding_box_);
    }
    else if (left_child_ != nullptr) {
      bounding_box_ = left_child_->bounding_box_;
    }
    else if (right_child_ != nullptr) {
      bounding_box_ = right_child_->bounding_box_;
    }
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  friend class Tree<ObjectType, FloatType>;

  BoundingBox3D<FloatType> bounding_box_;

  Node* left_child_;
  Node* right_child_;

  ObjectType* object_;
};

/// A BVH from a list of objects that inherit the BoundingBox3DInterface
template <typename ObjectType, typename FloatType>
class Tree
{
public:
  USE_FIXED_EIGEN_TYPES(FloatType)
  using NodeType = Node<ObjectType, FloatType>;
  using BoundingBoxType = BoundingBox3D<FloatType>;
  using RayType = ait::Ray<FloatType>;
  using RayDataType = ait::RayData<FloatType>;
#if WITH_CUDA
  using CudaNodeType = CudaNode<FloatType>;
  using CudaTreeType = CudaTree<FloatType>;
  using CudaRayType = CudaRay<FloatType>;
#endif

  struct ObjectWithBoundingBox {
    BoundingBoxType bounding_box;
    ObjectType* object;
  };

  struct IntersectionResult {
    IntersectionResult()
    : intersection(Vector3::Zero()), node(nullptr), depth(0), dist_sq(0) {}

    Vector3 intersection;
    NodeType* node;
    std::size_t depth;
    FloatType dist_sq;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct BBoxIntersectionResult {
    using NodePtrType = NodeType*;
    NodePtrType node;
    std::size_t depth;
  };

  struct ConstBBoxIntersectionResult {
    using NodePtrType = const NodeType*;
    NodePtrType node;
    std::size_t depth;
  };

  struct IntersectionData {
    RayDataType ray;
    FloatType min_range_sq;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  template <typename ValueT>
  class _Iterator : public boost::iterator_facade<
    _Iterator<ValueT>,
    ValueT,
    boost::forward_traversal_tag> {
  public:
    _Iterator() {}

    _Iterator(ValueT* node) {
      node_stack_.push(node);
    }

    // Construct from convertible iterator (i.e. const to non-const)
    template <typename OtherIteratorT,
        typename std::enable_if<std::is_convertible<_Iterator, OtherIteratorT>::value, int>::type = 0>
    _Iterator(OtherIteratorT other_it)
    : node_stack_(other_it.node_stack_) {}

  private:
    friend class boost::iterator_core_access;

    void increment() {
      ValueT* node = node_stack_.top();
      node_stack_.pop();
      if (node->hasLeftChild()) {
        node_stack_.push(node->getLeftChild());
      }
      if (node->hasRightChild()) {
        node_stack_.push(node->getRightChild());
      }
    }

    bool equal(const _Iterator& other_it) const {
      return node_stack_ == other_it.node_stack_;
    }

    ValueT& dereference() const {
      return *node_stack_.top();
    }

    std::stack<ValueT*> node_stack_;
  };

  using Iterator = _Iterator<NodeType>;
  using ConstIterator = _Iterator<const NodeType>;

  Tree()
  : root_(nullptr), stored_as_vector_(false), owns_objects_(false), depth_(0), num_nodes_(0), num_leaf_nodes_(0) {
#if WITH_CUDA
    cuda_tree_ = nullptr;
    cuda_stack_size_ = 32 * 1024;
#endif
  }

#if WITH_CUDA
  void setCudaStackSize(const std::size_t cuda_stack_size) {
    cuda_stack_size_ = cuda_stack_size;
  }
#endif
//  Tree(const Node*, bool copy = true) {
//    root_ = nullptr;
//    build(map, copy);
//  }

  Tree(const Tree& other) = delete;

  ~Tree() {
    clear();
#if WITH_CUDA
    if (cuda_tree_ != nullptr) {
      delete cuda_tree_;
    }
#endif
  }

  Iterator begin() {
    return Iterator(getRoot());
  }

  ConstIterator begin() const {
    return ConstIterator(getRoot());
  }

  ConstIterator cbegin() {
    return ConstIterator(getRoot());
  }

  Iterator end() {
    return Iterator();
  }

  ConstIterator end() const {
    return ConstIterator();
  }

  ConstIterator cend() {
    return ConstIterator();
  }

  void clear() {
    if (owns_objects_) {
      clearObjectsRecursive(root_);
    }
    if (root_ != nullptr && !stored_as_vector_) {
      clearRecursive(root_);
    }
    root_ = nullptr;
    nodes_.clear();
    depth_ = 0;
    num_leaf_nodes_ = 0;
    stored_as_vector_ = false;
    owns_objects_ = false;
    voxel_index_map_.clear();
    index_voxel_map_.clear();
  }

  const std::unordered_map<std::size_t, const NodeType*>& getIndexVoxelMap() const {
    if (index_voxel_map_.empty()) {
      computeIndexVoxelMap();
    }
    return index_voxel_map_;
  }

  const std::unordered_map<const NodeType*, std::size_t>& getVoxelIndexMap() const {
    if (voxel_index_map_.empty()) {
      computeVoxelIndexMap();
    }
    return voxel_index_map_;
  }

  NodeType* getRoot() {
//    return &nodes_.front();
    return root_;
  }

  const NodeType* getRoot() const {
//    return &nodes_.front();
    return root_;
  }

  std::size_t getDepth() const {
    return depth_;
  }

  std::size_t getNumOfNodes() const {
//    return nodes_.size();
    return num_nodes_;
  }

  std::size_t getNumOfLeafNodes() const {
    return num_leaf_nodes_;
  }

  void printInfo() const {
    std::cout << "Info: Tree depth " << getDepth() << std::endl;
    std::cout << "Info: NumNodes " << getNumOfNodes() << std::endl;
    std::cout << "Info: NumLeaves " << getNumOfLeafNodes() << std::endl;
    std::cout << "Info: Boundingbox " << getRoot()->getBoundingBox() << std::endl;
    std::cout << "Info: Root pointer " << getRoot() << std::endl;
  }

  void build(std::vector<ObjectWithBoundingBox> objects, bool take_ownership=true) {
    clear();
//    nodes_.emplace_back();
    root_ = allocateNode();
    buildRecursive(root_, objects);
    owns_objects_ = take_ownership;
//    nodes_.shrink_to_fit();
    computeInfo();
    printInfo();
//    for (auto it = begin(); it != end(); ++it) {
//      AIT_ASSERT(!it->isLeaf || it->getObject() == nullptr);
//    }
  }

  // Cannot be const because BBoxIntersectionResult contains a non-const pointer to a node
  std::pair<bool, IntersectionResult> intersects(const RayType& ray, FloatType min_range = 0, FloatType max_range = -1) {
    IntersectionData data;
    data.ray.origin = ray.origin;
    data.ray.direction = ray.direction;
    data.ray.inv_direction = ray.direction.cwiseInverse();
    data.min_range_sq = min_range * min_range;
    IntersectionResult result;
    if (max_range > 0) {
      result.dist_sq = max_range * max_range;
    }
    else {
      result.dist_sq = std::numeric_limits<FloatType>::max();
    }
    bool does_intersect = intersectsRecursive(data, getRoot(), 0, &result);
    return std::make_pair(does_intersect, result);
  }

#if WITH_CUDA
  // Cannot be const because BBoxIntersectionResult contains a non-const pointer to a node
  std::vector<IntersectionResult> raycastCuda(
      const Matrix4x4& intrinsics,
      const Matrix3x4& extrinsics,
      const std::size_t x_start, const std::size_t x_end,
      const std::size_t y_start, const std::size_t y_end,
      FloatType min_range = 0, FloatType max_range = -1,
      const bool fail_on_error = false) {
    ensureCudaTreeIsInitialized();
    CudaMatrix4x4<FloatType> cuda_intrinsics;
    cuda_intrinsics.copyFrom(intrinsics.data(), CudaMatrix4x4<FloatType>::ColumnMajor);
    CudaMatrix3x4<FloatType> cuda_extrinsics;
    cuda_extrinsics.copyFrom(extrinsics.data(), CudaMatrix3x4<FloatType>::ColumnMajor);
//    std::cout << "intrinsics: " << intrinsics << std::endl;
//    cuda_intrinsics.print();
//    std::cout << "extrinsics: " << extrinsics << std::endl;
//    cuda_extrinsics.print();
    try {
      using CudaResultType = typename CudaTreeType::CudaIntersectionResult;
      const std::vector<CudaResultType> cuda_results =
          cuda_tree_->raycastRecursive(
              cuda_intrinsics,
              cuda_extrinsics,
              x_start, x_end,
              y_start, y_end,
              min_range, max_range,
              fail_on_error);
  //    const std::vector<CudaResultType> cuda_results =
  //        cuda_tree_->raycastIterative(
  //            cuda_intrinsics,
  //            cuda_extrinsics,
  //            x_start, x_end,
  //            y_start, y_end,
  //            min_range, max_range);
      std::vector<IntersectionResult> results(cuda_results.size());
  #pragma omp parallel for
      for (std::size_t i = 0; i < cuda_results.size(); ++i) {
        const CudaResultType& cuda_result = cuda_results[i];
        IntersectionResult& result = results[i];
        result.depth = cuda_result.depth;
        result.dist_sq = cuda_result.dist_sq;
        result.intersection(0) = cuda_result.intersection(0);
        result.intersection(1) = cuda_result.intersection(1);
        result.intersection(2) = cuda_result.intersection(2);
        result.node = reinterpret_cast<NodeType*>(cuda_result.node);
      }
      return results;
    }
    catch (const ait::CudaError& err) {
      std::cerr << "CUDA raycast failed: " << err.what() << std::endl;
      std::cerr << "Regenerating CUDA tree" << std::endl;
      cuda_tree_->clear();
      ensureCudaTreeIsInitialized();
      throw ait::Error("Raycast failed");
    }
  }

  // Cannot be const because BBoxIntersectionResult contains a non-const pointer to a node
  std::vector<IntersectionResult> intersectsCuda(
      const std::vector<RayType>& rays, FloatType min_range = 0, FloatType max_range = -1) {
    ensureCudaTreeIsInitialized();
    std::vector<CudaRayType> cuda_rays(rays.size());
#pragma omp parallel for
    for (std::size_t i = 0; i < rays.size(); ++i) {
      const RayType& ray = rays[i];
      CudaRayType& cuda_ray = cuda_rays[i];
      cuda_ray.origin.copyFrom(ray.origin.data());
      cuda_ray.direction.copyFrom(ray.direction.data());
    }
    using CudaResultType = typename CudaTreeType::CudaIntersectionResult;
    const std::vector<CudaResultType> cuda_results =
        cuda_tree_->intersectsRecursive(cuda_rays, min_range, max_range);
//    const std::vector<CudaResultType> cuda_results =
//        cuda_tree_->intersectsIterative(cuda_rays, min_range, max_range);
    std::vector<IntersectionResult> results(cuda_results.size());
#pragma omp parallel for
    for (std::size_t i = 0; i < cuda_results.size(); ++i) {
      const CudaResultType& cuda_result = cuda_results[i];
      IntersectionResult& result = results[i];
      result.depth = cuda_result.depth;
      result.dist_sq = cuda_result.dist_sq;
      result.intersection(0) = cuda_result.intersection(0);
      result.intersection(1) = cuda_result.intersection(1);
      result.intersection(2) = cuda_result.intersection(2);
      result.node = reinterpret_cast<NodeType*>(cuda_result.node);
    }
    return results;
  }
#endif

  std::vector<BBoxIntersectionResult> intersects(const BoundingBoxType& bbox) {
    std::vector<BBoxIntersectionResult> results;
    intersectsRecursive<BBoxIntersectionResult>(bbox, getRoot(), 0, &results);
    return results;
  }

  std::vector<ConstBBoxIntersectionResult> intersects(const BoundingBoxType& bbox) const {
    std::vector<ConstBBoxIntersectionResult> results;
    intersectsRecursive<ConstBBoxIntersectionResult>(bbox, getRoot(), 0, &results);
    return results;
  }

private:
  // Boost serialization
  friend class boost::serialization::access;

  template <typename Archive>
  void save(Archive& ar, const unsigned int version) const {
    // Write some metadata
    ar & getDepth();
    ar & getNumOfNodes();
    ar & getNumOfLeafNodes();
    // Write out all nodes
    std::deque<const NodeType*> node_queue;
    node_queue.push_front(getRoot());
    std::size_t node_counter = 0;
    while (!node_queue.empty()) {
      const NodeType* node = node_queue.back();
      node_queue.pop_back();
      // Write child indices to disk and push childs into writing list
      if (node->hasLeftChild()) {
        node_queue.push_front(node->getLeftChild());
        const std::size_t left_child_index = node_counter + node_queue.size();
        ar & left_child_index;
      }
      else {
        const std::size_t left_child_index = 0;
        ar & left_child_index;
      }
      if (node->hasRightChild()) {
        node_queue.push_front(node->getRightChild());
        std::size_t right_child_index = node_counter + node_queue.size();
        ar & right_child_index;
      }
      else {
        const std::size_t right_child_index = 0;
        ar & right_child_index;
      }
      ar & node->getBoundingBox();
      if (node->getObject() != nullptr) {
        ar & true;
        ar & (*node->getObject());
      }
      else {
        ar & false;
      }
      ++node_counter;
    }
  }

  template <typename Archive>
  void load(Archive& ar, const unsigned int version) {
    std::size_t depth;
    std::size_t num_of_nodes;
    std::size_t num_of_leaf_nodes;
    ar & depth;
    ar & num_of_nodes;
    ar & num_of_leaf_nodes;
    std::cout << "Tree has depth " << depth << ", " << num_of_nodes << " nodes " << " and " << num_of_leaf_nodes << " leaf nodes" << std::endl;
    // Read nodes from disk
    std::vector<NodeType> nodes;
    std::size_t leaf_counter = 0;
    nodes.resize(num_of_nodes);
    for (auto it = nodes.begin(); it != nodes.end(); ++it) {
//        std::cout << "Reading node " << (it - nodes.begin()) << " of " << nodes.size() << std::endl;
      std::size_t left_child_index;
      std::size_t right_child_index;
      ar & left_child_index;
      ar & right_child_index;
      AIT_ASSERT(left_child_index < num_of_nodes);
      AIT_ASSERT(right_child_index < num_of_nodes);
      NodeType& node = *it;
      if (left_child_index == 0) {
        node.left_child_ = nullptr;
      }
      else {
        node.left_child_ = &nodes[left_child_index];
      }
      if (right_child_index == 0) {
        node.right_child_ = nullptr;
      }
      else {
        node.right_child_ = &nodes[right_child_index];
      }
      ar & node.bounding_box_;
      bool has_object;
      ar & has_object;
      if (has_object) {
        node.object_ = new ObjectType();
        ar & (*node.object_);
      }
      else {
        node.object_ = nullptr;
      }
      if (node.isLeaf()) {
        ++leaf_counter;
      }
    }
    std::cout << "leaf nodes: " << leaf_counter << std::endl;

    // Update tree
//      depth_ = depth;
//      num_nodes_ = num_of_nodes;
//      num_leaf_nodes_ = num_of_leaf_nodes;
    nodes_ = std::move(nodes);
    root_ = &nodes_.front();
    stored_as_vector_ = true;
    owns_objects_ = true;
    computeInfo();
    printInfo();
    AIT_ASSERT_STR(getDepth() == depth
        && getNumOfNodes() == num_of_nodes
        && getNumOfLeafNodes() == num_of_leaf_nodes,
        "The tree properties are not as expected");
  }

  BOOST_SERIALIZATION_SPLIT_MEMBER()

  void computeInfo() {
    num_nodes_ = 0;
    num_leaf_nodes_ = 0;
    depth_ = 0;
    computeInfoRecursive(getRoot(), 0);
  }

  void computeInfoRecursive(const NodeType* node, std::size_t cur_depth) {
    depth_ = std::max(cur_depth, depth_);
    ++num_nodes_;
    if (node->isLeaf()) {
      ++num_leaf_nodes_;
    }
    else {
      if (node->left_child_ != nullptr) {
        computeInfoRecursive(node->left_child_, cur_depth + 1);
      }
      if (node->right_child_ != nullptr) {
        computeInfoRecursive(node->right_child_, cur_depth + 1);
      }
    }
  }

  void computeVoxelIndexMap() const {
    std::cout << "BVH: Computing voxel index map" << std::endl;
    // Compute consistent ordering of BVH nodes
    voxel_index_map_.clear();
    std::size_t voxel_index = 0;
    for (const NodeType& node : *this) {
      voxel_index_map_.emplace(&node, voxel_index);
      ++voxel_index;
    }
  }

  void computeIndexVoxelMap() const {
    std::cout << "BVH: Computing index voxel map" << std::endl;
    // Compute consistent ordering of BVH nodes
    index_voxel_map_.clear();
    std::size_t voxel_index = 0;
    for (const NodeType& node : *this) {
      index_voxel_map_.emplace(voxel_index, &node);
      ++voxel_index;
    }
  }

  void clearObjectsRecursive(NodeType* node) {
    if (node->left_child_ != nullptr) {
      clearObjectsRecursive(node->left_child_);
      node->left_child_ = nullptr;
    }
    if (node->right_child_ != nullptr) {
      clearObjectsRecursive(node->right_child_);
      node->right_child_ = nullptr;
    }
    SAFE_DELETE(node->object_);
  }

  void clearRecursive(NodeType* node) {
    if (node->left_child_ != nullptr) {
      clearRecursive(node->left_child_);
      node->left_child_ = nullptr;
    }
    if (node->right_child_ != nullptr) {
      clearRecursive(node->right_child_);
      node->right_child_ = nullptr;
    }
    deallocateNode(node);
  }

  void buildRecursive(NodeType* root, std::vector<ObjectWithBoundingBox>& objects) {
    assert(objects.size() > 2);
//    splitMidPoint(root, objects.begin(), objects.end(), 0);
    splitMedian(root, objects.begin(), objects.end(), 0);
  }

  void splitMedian(NodeType* node,
      typename std::vector<ObjectWithBoundingBox>::iterator begin, typename std::vector<ObjectWithBoundingBox>::iterator end,
      std::size_t sort_axis) {
    if (end - begin > 1) {
      std::stable_sort(begin, end, [&](const ObjectWithBoundingBox& a, const ObjectWithBoundingBox& b) -> bool {
        return a.bounding_box.getCenter(sort_axis) < b.bounding_box.getCenter(sort_axis);
      });

      std::size_t next_sort_axis = (sort_axis + 1) % 3;

      node->left_child_ = allocateNode();
      node->right_child_ = allocateNode();

      splitMedian(node->left_child_, begin, begin + (end - begin) / 2, next_sort_axis);
      splitMedian(node->right_child_, begin + (end - begin) / 2, end, next_sort_axis);

      node->computeBoundingBox();
    }
    else {
      assert(end - begin == 1);
      node->bounding_box_ = begin->bounding_box;
      node->object_ = begin->object;
      AIT_ASSERT(begin->object != nullptr);
    }
  }

  BoundingBoxType computeBoundingBox(typename std::vector<ObjectWithBoundingBox>::iterator begin, typename std::vector<ObjectWithBoundingBox>::iterator end) {
    BoundingBoxType bbox = begin->bounding_box;
    for (auto it = begin; it != end; ++it) {
      bbox.include(it->bounding_box);
    }
    return bbox;
  }

  void splitMidPoint(NodeType* node,
      typename std::vector<ObjectWithBoundingBox>::iterator begin, typename std::vector<ObjectWithBoundingBox>::iterator end, std::size_t cur_depth) {
    if (end - begin > 1) {
      BoundingBoxType bbox = computeBoundingBox(begin, end);

      std::size_t max_extent_index;
      FloatType max_extent = bbox.getMaxExtent(&max_extent_index);
      typename std::vector<ObjectWithBoundingBox>::iterator mid_it = begin + 1;
      std::stable_sort(begin, end, [&](const ObjectWithBoundingBox& a, const ObjectWithBoundingBox& b) -> bool {
        return a.bounding_box.getMinimum(max_extent_index) < b.bounding_box.getMinimum(max_extent_index);
      });
      FloatType middle = bbox.getMinimum(max_extent_index) + max_extent / 2;
      for (; mid_it != end - 1; ++mid_it) {
        if (mid_it->bounding_box.getMinimum(max_extent_index) >= middle) {
          break;
        }
      }
      BoundingBoxType bbox_left = computeBoundingBox(begin, mid_it);
      BoundingBoxType bbox_right = computeBoundingBox(mid_it, end);
      assert(!(bbox == bbox_left));
      assert(!(bbox == bbox_right));
      assert(!(bbox_left == bbox_right));

      node->left_child_ = allocateNode();
      node->right_child_ = allocateNode();

//      assert(node->left_child_ >= nodes_.data());
//      assert(node->right_child_ >= node->left_child_);

      splitMidPoint(node->left_child_, begin, mid_it, cur_depth + 1);
      splitMidPoint(node->right_child_, mid_it, end, cur_depth + 1);

//      assert(node->left_child_ >= nodes_.data());
//      assert(node->right_child_ >= node->left_child_);

      node->computeBoundingBox();
    }
    else {
      assert(end - begin == 1);
      node->bounding_box_ = begin->bounding_box;
      node->object_ = begin->object;
    }
  }


  bool intersectsRecursive(const IntersectionData& data, NodeType* cur_node, std::size_t cur_depth, IntersectionResult* result) const {
    // Early break because of node semantics (i.e. free nodes)
//    if (CollisionPredicate::earlyBreak(this, cur_node)) {
//      return false;
//    }

    bool outside_bounding_box = cur_node->getBoundingBox().isOutside(data.ray.origin);
//    std::cout << "outside_bounding_box: " << outside_bounding_box << std::endl;

//    bool below_min_range = false;
    Vector3 intersection;
    FloatType intersection_dist_sq;
    if (outside_bounding_box) {
      // Check if ray intersects current node
      const bool intersects = cur_node->getBoundingBox().intersects(data.ray, &intersection);
//      std::cout << "intersects: " << intersects << std::endl;
      if (intersects) {
        intersection_dist_sq = (data.ray.origin - intersection).squaredNorm();
//        if (intersection_dist_sq < data.min_range_sq) {
//          below_min_range = true;
//        }
//        std::cout << "intersection_dist_sq: " << intersection_dist_sq << std::endl;
//        std::cout << "data.ray.origin: " << data.ray.origin.transpose() << std::endl;
//        std::cout << "cur_node->bounding_box: " << cur_node->bounding_box_ << std::endl;
//        std::cout << "result->intersection: " << intersection.transpose() << std::endl;
        if (intersection_dist_sq > result->dist_sq) {
//          std::cout << "intersection_dist_sq > dist_sq" <<std::endl;
          return false;
        }
      }
      else {
        return false;
      }
    }
//    else {
//      below_min_range = true;
//    }

//    std::cout << "cur_node->isLeaf(): " << cur_node->isLeaf() << std::endl;
    if (cur_node->isLeaf()) {
//      AIT_ASSERT(cur_node->object_ != nullptr);
//      if (below_min_range) {
//        return false;
//      }
      if (!outside_bounding_box) {
        // If already inside the bounding box we want the intersection point to be the start of the ray.
        intersection = data.ray.origin;
        intersection_dist_sq = 0;
      }
      result->intersection = intersection;
      result->node = cur_node;
      result->depth = cur_depth;
      result->dist_sq = intersection_dist_sq;
      return true;
    }

    bool intersects_left = false;
    bool intersects_right = false;
    if (cur_node->left_child_ != nullptr) {
      intersects_left = intersectsRecursive(data, cur_node->left_child_, cur_depth + 1, result);
    }
    if (cur_node->right_child_ != nullptr) {
      intersects_right = intersectsRecursive(data, cur_node->right_child_, cur_depth + 1, result);
    }
//    std::cout << "intersects_left: " << intersects_left << std::endl;
//    std::cout << "intersects_right: " << intersects_right << std::endl;
    return intersects_left || intersects_right;
  }

  template <typename IntersectionResultT>
  void intersectsRecursive(const BoundingBoxType& bbox,
      typename IntersectionResultT::NodePtrType cur_node,
      std::size_t cur_depth,
      std::vector<IntersectionResultT>* results) const {
    const bool intersects = cur_node->getBoundingBox().intersects(bbox);
//    std::cout << "cur_depth: " << cur_depth << std::endl;
//    std::cout << "intersects: " << intersects << std::endl;
//    std::cout << "cur_node->getBoundingBox(): " << cur_node->getBoundingBox() << std::endl;
//    std::cout << "bbox: " << bbox << std::endl;
    if (!intersects) {
      return;
    }

    if (cur_node->isLeaf()) {
      IntersectionResultT result;
      result.node = cur_node;
      result.depth = cur_depth;
      results->push_back(result);
      return;
    }

    if (cur_node->left_child_ != nullptr) {
      intersectsRecursive(bbox, cur_node->left_child_, cur_depth + 1, results);
    }
    if (cur_node->right_child_ != nullptr) {
      intersectsRecursive(bbox, cur_node->right_child_, cur_depth + 1, results);
    }
  }

  NodeType* allocateNode() {
    return new NodeType;
  }

  void deallocateNode(NodeType* node) {
    delete node;
  }

#if WITH_CUDA
  void ensureCudaTreeIsInitialized() {
    if (cuda_tree_ == nullptr) {
      std::cout << "Previous CUDA stack size was " << ait::CudaManager::getStackSize() << std::endl;
      std::cout << "Setting CUDA stack size to " << cuda_stack_size_ << std::endl;
      ait::CudaManager::setStackSize(cuda_stack_size_);
      CudaNodeType* cuda_root = reinterpret_cast<CudaNodeType*>(getRoot());
      cuda_tree_ = CudaTreeType::createCopyFromHostTree(cuda_root, getNumOfNodes(), getDepth());
    }
  }
#endif

//  std::vector<NodeType> nodes_;
  NodeType* root_;
  std::vector<NodeType> nodes_;
  bool stored_as_vector_;
  bool owns_objects_;
  std::size_t depth_;
  std::size_t num_nodes_;
  std::size_t num_leaf_nodes_;

  mutable std::unordered_map<std::size_t, const NodeType*> index_voxel_map_;
  mutable std::unordered_map<const NodeType*, std::size_t> voxel_index_map_;

#if WITH_CUDA
  CudaTreeType* cuda_tree_;
  std::size_t cuda_stack_size_;
#endif
};

#if __GNUC__ && !__CUDACC__
  #pragma GCC pop_options
#endif

}  // namespace bvh
