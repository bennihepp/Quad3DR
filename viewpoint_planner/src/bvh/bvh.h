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
#include <ait/common.h>
#include <ait/eigen.h>
#include <ait/utilities.h>

namespace bvh {

#pragma GCC push_options
#pragma GCC optimize ("fast-math")

struct Ray {
  Eigen::Vector3f origin;
  Eigen::Vector3f direction;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct RayData : public Ray {
  Eigen::Vector3f inv_direction;
  float min_range_sq;
  float max_range_sq;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename FloatType = float>
class BoundingBox3D {
public:
  using Vector3 = Eigen::Matrix<FloatType, 3, 1>;

  BoundingBox3D() {
    min_ << 0, 0, 0;
    max_ << 0, 0, 0;
  }

  BoundingBox3D(const Vector3& center, FloatType size)
  : min_(center.array() - size / 2), max_(center.array() + size / 2) {}

  BoundingBox3D(const Vector3& min, const Vector3& max)
  : min_(min), max_(max) {}

  bool operator==(const BoundingBox3D& other) const {
    return min_ == other.min_ && max_ == other.max_;
  }
  const Vector3& getMinimum() const {
    return min_;
  }

  const FloatType getMinimum(size_t index) const {
    return min_(index);
  }

  const Vector3& getMaximum() const {
    return min_;
  }

  const FloatType getMaximum(size_t index) const {
    return max_(index);
  }

  const Vector3 getExtent() const {
    return max_ - min_;
  }

  const FloatType getMaxExtent() const {
    return (max_ - min_).maxCoeff();
  }

  const FloatType getMaxExtent(size_t* index) const {
    return (max_ - min_).maxCoeff(index);
  }

  const Vector3 getCenter() const {
    return (min_ + max_) / 2;
  }

  const FloatType getCenter(size_t index) const {
    return (min_(index) + max_(index)) / 2;
  }

  bool isOutside(const Vector3& point) const {
    return (point.array() < min_.array()).any() || (point.array() > max_.array()).any();
  }

  bool isInside(const Vector3& point) const {
    return (point.array() >= min_.array()).all() && (point.array() <= max_.array()).all();
  }

  bool intersects(const Ray& ray, Vector3* intersection = nullptr) const {
    RayData ray_data(ray);
    ray_data.inv_direction = ray.direction.cwiseInverse();
    return intersects(ray_data, intersection);
  }

  bool intersects(const RayData& ray_data, Vector3* intersection = nullptr) const {
  //  std::cout << "  intersecting node at depth " << cur_depth << " with size " << node_size_half * 2 << std::endl;
    float t_min = -std::numeric_limits<FloatType>::max();
    float t_max = std::numeric_limits<FloatType>::max();

  //  for (size_t i = 0; i < 3; ++i) {
  //    if (ray_data.direction(i) != 0) {
  //      float t0 = (min_(i) - ray_data.origin(i)) * ray_data.inv_direction(i);
  //      float t1 = (max_(i) - ray_data.origin(i)) * ray_data.inv_direction(i);
  //      t_min = std::max(t_min, std::min(t0, t1));
  //      t_max = std::min(t_max, std::max(t0, t1));
  //    }
  //    else {
  //      if (ray_data.origin(i) <= min_(i) || ray_data.origin(i) >= max_(i)) {
  //        return false;
  //      }
  //    }
  //  }

    // Faster version without explicit check for directions parallel to an axis
    for (size_t i = 0; i < 3; ++i) {
      float t0 = (min_(i) - ray_data.origin(i)) * ray_data.inv_direction(i);
      float t1 = (max_(i) - ray_data.origin(i)) * ray_data.inv_direction(i);
      t_min = std::max(t_min, std::min(t0, t1));
      t_max = std::min(t_max, std::max(t0, t1));
    }

    bool intersect = t_max > std::max(t_min, FloatType(0));
    if (intersect && intersection != nullptr) {
      t_min = std::max(t_min, FloatType(0));
      *intersection = ray_data.origin + ray_data.direction * t_min;
    }
    return intersect;
  }

  static BoundingBox3D getUnion(const BoundingBox3D& bbox_a, const BoundingBox3D& bbox_b) {
    Vector3 min = bbox_a.min_.array().min(bbox_b.min_.array());
    Vector3 max = bbox_a.max_.array().max(bbox_b.max_.array());
    return BoundingBox3D(min, max);
  }

  void mergeWith(const BoundingBox3D& other) {
    min_ = min_.array().min(other.min_.array());
    max_ = max_.array().max(other.max_.array());
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Vector3 min_;
  Vector3 max_;
};

template <typename FloatType, typename CharType>
std::basic_ostream<CharType>& operator<<(std::basic_ostream<CharType>& out, const BoundingBox3D<FloatType>& bbox) {
  out << "(" << bbox.getMinimum(0) << ", " << bbox.getMinimum(1) << ", " << bbox.getMinimum(2) << ") -> ";
  out << "(" << bbox.getMaximum(0) << ", " << bbox.getMaximum(1) << ", " << bbox.getMaximum(2) << ")";
  return out;
}

template <typename FloatType = float>
class BoundingBox3DInterface {
public:
  virtual ~BoundingBox3DInterface() {}

  virtual BoundingBox3D<FloatType> getBoundingBox() const = 0;
};

template <typename ObjectType, typename FloatType = float>
class Tree;

template <typename ObjectType, typename FloatType = float>
struct Node {
  using BoundingBoxType = BoundingBox3D<FloatType>;

  Node()
  : left_child_(nullptr), right_child_(nullptr), object_(nullptr) {}

  ~Node() {}

  const BoundingBoxType& getBoundingBox() const {
    return bounding_box_;
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
  using Vector3 = Eigen::Matrix<FloatType, 3, 1>;
  using NodeType = Node<ObjectType, FloatType>;
  using BoundingBoxType = BoundingBox3D<FloatType>;

  struct ObjectWithBoundingBox {
    BoundingBoxType bounding_box;
    ObjectType* object;
  };

  struct IntersectionResult {
    Vector3 intersection;
    NodeType* node;
    float dist_sq;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct IntersectionData {
    RayData ray;
    float min_range_sq;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  Tree()
  : root_(nullptr), depth_(0), num_nodes_(0), num_leaf_nodes_(0) {}

//  Tree(const Node*, bool copy = true) {
//    root_ = nullptr;
//    build(map, copy);
//  }

  ~Tree() {
    clear();
  }

  void clear() {
    //    nodes_.clear();
    if (root_ != nullptr) {
      clearRecursive(root_);
      root_ = nullptr;
      depth_ = 0;
      num_leaf_nodes_ = 0;
    }
  }

  NodeType* getRoot() {
//    return &nodes_.front();
    return root_;
  }

  const NodeType* getRoot() const {
//    return &nodes_.front();
    return root_;
  }

  size_t getDepth() const {
    return depth_;
  }

  size_t getNumOfNodes() const {
//    return nodes_.size();
    return num_nodes_;
  }

  size_t getNumOfLeafNodes() const {
    return num_leaf_nodes_;
  }

  void printInfo() const {
    std::cout << "Info: Tree depth " << getDepth() << std::endl;
    std::cout << "Info: NumNodes " << getNumOfNodes() << std::endl;
    std::cout << "Info: NumLeaves " << getNumOfLeafNodes() << std::endl;
  }

  void build(std::vector<ObjectWithBoundingBox>& objects) {
    clear();
//    nodes_.emplace_back();
    root_ = allocateNode();
    buildRecursive(root_, objects);
//    nodes_.shrink_to_fit();
    computeInfo();
    printInfo();
  }

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
    bool does_intersect = intersectsRecursive(data, getRoot(), &result);
    return std::make_pair(does_intersect, result);
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
    out << kFileTag;
    writeToStream<size_t>(out, getDepth());
    writeToStream<size_t>(out, getNumOfNodes());
    writeToStream<size_t>(out, getNumOfLeafNodes());
    writeRecursive(out, getRoot());
  }

private:
  const std::string kFileTag = "BVHTree";

  void writeRecursive(std::ostream& out, const NodeType* node) {
    // TODO
  }

  void computeInfo() {
    num_nodes_ = 0;
    num_leaf_nodes_ = 0;
    depth_ = 0;
    computeInfoRecursive(getRoot(), 0);
  }

  void computeInfoRecursive(const NodeType* node, size_t cur_depth) {
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
      size_t sort_axis) {
    if (end - begin > 1) {
      std::stable_sort(begin, end, [&](const ObjectWithBoundingBox& a, const ObjectWithBoundingBox& b) -> bool {
        return a.bounding_box.getCenter(sort_axis) < b.bounding_box.getCenter(sort_axis);
      });

      size_t next_sort_axis = (sort_axis + 1) % 3;

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
    }
  }

  BoundingBoxType computeBoundingBox(typename std::vector<ObjectWithBoundingBox>::iterator begin, typename std::vector<ObjectWithBoundingBox>::iterator end) {
    BoundingBoxType bbox = begin->bounding_box;
    for (auto it = begin; it != end; ++it) {
      bbox.mergeWith(it->bounding_box);
    }
    return bbox;
  }

  void splitMidPoint(NodeType* node,
      typename std::vector<ObjectWithBoundingBox>::iterator begin, typename std::vector<ObjectWithBoundingBox>::iterator end, size_t cur_depth) {
    if (end - begin > 1) {
      BoundingBoxType bbox = computeBoundingBox(begin, end);

      size_t max_extent_index;
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

  bool intersectsRecursive(const IntersectionData& data, NodeType* cur_node, IntersectionResult* result) const {
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
      result->dist_sq = intersection_dist_sq;
      return true;
    }

    bool intersects_left = false;
    bool intersects_right = false;
    if (cur_node->left_child_ != nullptr) {
      intersects_left = intersectsRecursive(data, cur_node->left_child_, result);
    }
    if (cur_node->right_child_ != nullptr) {
      intersects_right = intersectsRecursive(data, cur_node->right_child_, result);
    }
//    std::cout << "intersects_left: " << intersects_left << std::endl;
//    std::cout << "intersects_right: " << intersects_right << std::endl;
    return intersects_left || intersects_right;
  }

  NodeType* allocateNode() {
    return new NodeType;
  }

  void deallocateNode(NodeType* node) {
    delete node;
  }

//  std::vector<NodeType> nodes_;
  NodeType* root_;
  size_t depth_;
  size_t num_nodes_;
  size_t num_leaf_nodes_;
};

#pragma GCC pop_options

}  // namespace bvh
