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
#include <boost/iterator_adaptors.hpp>
#include <ait/common.h>
#include <ait/eigen.h>
#include <ait/geometry.h>
#include <ait/utilities.h>
#include <ait/serialization.h>

namespace bvh {

using ait::Ray;
using ait::RayData;
using ait::BoundingBox3D;

#pragma GCC push_options
#pragma GCC optimize ("fast-math")

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
    float dist_sq;

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
    RayData ray;
    float min_range_sq;

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
  : root_(nullptr), stored_as_vector_(false), owns_objects_(false), depth_(0), num_nodes_(0), num_leaf_nodes_(0) {}

//  Tree(const Node*, bool copy = true) {
//    root_ = nullptr;
//    build(map, copy);
//  }

  Tree(const Tree& other) = delete;

  ~Tree() {
    clear();
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
  std::pair<bool, IntersectionResult> intersects(const Ray& ray, FloatType min_range = 0, FloatType max_range = -1) {
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

  void read(const std::string& filename) {
    std::ifstream in(filename, std::ios::binary);
    if (in) {
      read(in);
    }
    else {
      throw AIT_EXCEPTION(std::string("Unable to open file for reading: ") + filename);
    }
  }

  void read(std::istream& in) {
    clear();
    Reader reader;
    reader.read(in, this);
  }

  void write(const std::string& filename) const {
    std::ofstream out(filename, std::ios::binary);
    if (out) {
      write(out);
    }
    else {
      throw AIT_EXCEPTION(std::string("Unable to open file for writing: ") + filename);
    }
  }

  void write(std::ostream& out) const {
    Writer writer;
    writer.write(out, this);
  }

private:
  const static std::string kFileTag;

  struct Writer {
    Writer()
    : node_counter_(0) {}

    void write(std::ostream& out, const Tree* tree) {
      // Write some metadata
      out << kFileTag << std::endl;
      out << ObjectType::kFileTag << std::endl;
      ait::writeToStream<std::size_t>(out, tree->getDepth());
      ait::writeToStream<std::size_t>(out, tree->getNumOfNodes());
      ait::writeToStream<std::size_t>(out, tree->getNumOfLeafNodes());
      // Write out all nodes
      std::deque<const NodeType*> node_queue;
      node_queue.push_front(tree->getRoot());
      while (!node_queue.empty()) {
        const NodeType* node = node_queue.back();
        node_queue.pop_back();
        // Write child indices to disk and push childs into writing list
        if (node->hasLeftChild()) {
          node_queue.push_front(node->getLeftChild());
          ait::writeToStream<std::size_t>(out, node_counter_ + node_queue.size());
        }
        else {
          ait::writeToStream<std::size_t>(out, 0);
        }
        if (node->getRightChild()) {
          node_queue.push_front(node->getRightChild());
          ait::writeToStream<std::size_t>(out, node_counter_ + node_queue.size());
        }
        else {
          ait::writeToStream<std::size_t>(out, 0);
        }
        node->getBoundingBox().write(out);
        if (node->getObject() != nullptr) {
          ait::writeToStream<bool>(out, true);
          node->getObject()->write(out);
        }
        else {
          ait::writeToStream<bool>(out, false);
        }
        ++node_counter_;
      }
    }

    std::size_t node_counter_;
  };

  struct Reader {
    Reader() {}

    void read(std::istream& in, Tree* tree) {
      // Read some metadata
      std::string tree_tag;
      std::getline(in, tree_tag);
      if (tree_tag != kFileTag) {
        throw AIT_EXCEPTION(std::string("Found unexpected tree tag: ") + tree_tag);
      }
      std::string object_tag;
      std::getline(in, object_tag);
      if (object_tag != ObjectType::kFileTag) {
        throw AIT_EXCEPTION(std::string("Found unexpected object tag: ") + object_tag);
      }
      std::size_t depth = ait::readFromStream<std::size_t>(in);
      std::size_t num_of_nodes = ait::readFromStream<std::size_t>(in);
      std::size_t num_of_leaf_nodes = ait::readFromStream<std::size_t>(in);
      std::cout << "Tree has depth " << depth << ", " << num_of_nodes << " nodes " << " and " << num_of_leaf_nodes << " leaf nodes" << std::endl;
      // Read nodes from disk
      std::vector<NodeType> nodes;
      std::size_t leaf_counter = 0;
      nodes.resize(num_of_nodes);
      for (auto it = nodes.begin(); it != nodes.end(); ++it) {
//        std::cout << "Reading node " << (it - nodes.begin()) << " of " << nodes.size() << std::endl;
        std::size_t left_child_index = ait::readFromStream<std::size_t>(in);
        std::size_t right_child_index = ait::readFromStream<std::size_t>(in);
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
        node.bounding_box_.read(in);
        bool has_object = ait::readFromStream<bool>(in);
        if (has_object) {
          node.object_ = new ObjectType();
          node.object_->read(in);
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
//      tree->depth_ = depth;
//      tree->num_nodes_ = num_of_nodes;
//      tree->num_leaf_nodes_ = num_of_leaf_nodes;
      tree->nodes_ = std::move(nodes);
      tree->root_ = &tree->nodes_.front();
      tree->stored_as_vector_ = true;
      tree->owns_objects_ = true;
      tree->computeInfo();
      tree->printInfo();
      AIT_ASSERT_STR(tree->getDepth() == depth
          && tree->getNumOfNodes() == num_of_nodes
          && tree->getNumOfLeafNodes() == num_of_leaf_nodes,
          "The tree properties are not as expected");
    }
  };

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
};

template <typename ObjectType, typename FloatType>
const std::string Tree<ObjectType, FloatType>::kFileTag = "BVHTree";

#pragma GCC pop_options

}  // namespace bvh
