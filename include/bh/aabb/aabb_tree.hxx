//==================================================
// aabb_tree.hxx
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 17.03.17
//

namespace bh {

// AABBTreeRayIntersection implementation

template <typename FloatT>
AABBTreeRayIntersection<FloatT>::AABBTreeRayIntersection()
    : node_index_(AABBTree<FloatT>::kNoChild), depth_(0), ray_t_(std::numeric_limits<FloatT>::max()) {}

template <typename FloatT>
AABBTreeRayIntersection<FloatT>::AABBTreeRayIntersection(
        const size_t node_index, const size_t depth, const FloatT ray_t)
    : node_index_(node_index), depth_(depth), ray_t_(ray_t) {}

template <typename FloatT>
auto AABBTreeRayIntersection<FloatT>::doesIntersect() const -> bool {
  return node_index_ != AABBTree<FloatT>::kNoChild;
}

template <typename FloatT>
auto AABBTreeRayIntersection<FloatT>::nodeIndex() const -> size_t {
  return node_index_;
}
template <typename FloatT>
auto AABBTreeRayIntersection<FloatT>::nodeIndex() -> size_t {
  return node_index_;
}

template <typename FloatT>
auto AABBTreeRayIntersection<FloatT>::depth() const -> size_t {
  return depth_;
}

template <typename FloatT>
auto AABBTreeRayIntersection<FloatT>::rayT() const -> FloatT {
  return ray_t_;
}


// _Node implementation

template <typename ColliderT, typename FloatT>
AABBTree<ColliderT, FloatT>::Node::Node()
    : left_child_index_(kNoChild), right_child_index_(kNoChild) {}

template <typename ColliderT, typename FloatT>
AABBTree<ColliderT, FloatT>::Node::Node(const size_t left_child_index, const size_t right_child_index,
                                        const ColliderT& collider, const BoundingBoxType& bounding_box)
    : collider_(collider), bounding_box_(bounding_box),
      left_child_index_(left_child_index), right_child_index_(right_child_index) {}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::Node::isLeaf() const -> bool {
  return left_child_index_ == kNoChild && right_child_index_ == kNoChild;
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::Node::hasLeftChild() const -> bool {
  return left_child_index_ != kNoChild;
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::Node::leftChildIndex() const -> size_t {
  return left_child_index_;
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::Node::hasRightChild() const -> bool {
  return right_child_index_ != kNoChild;
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::Node::rightChildIndex() const -> size_t {
  return right_child_index_;
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::Node::boundingBox() const -> const BoundingBoxType& {
  return bounding_box_;
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::Node::collider() const -> const ColliderT& {
  return collider_;
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::Node::collider() -> ColliderT& {
  return collider_;
}


// AABBTree implementation

template <typename ColliderT, typename FloatT>
template <typename ColliderContainer>
AABBTree<ColliderT, FloatT>::AABBTree::AABBTree(const ColliderContainer& container) {
  build(std::begin(container), std::end(container));
}

template <typename ColliderT, typename FloatT>
template <typename ColliderIterator>
AABBTree<ColliderT, FloatT>::AABBTree::AABBTree(const ColliderIterator first, const ColliderIterator last) {
  build(first, last);
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::numOfNodes() const -> size_t {
  return nodes_.size();
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::getRoot() const -> const Node& {
  return nodes_.front();
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::getRoot() -> Node& {
  return nodes_.front();
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::getNode(const size_t node_index) const -> const Node& {
  return nodes_[node_index];
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::getNode(const size_t node_index) -> Node& {
  return nodes_[node_index];
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::begin() -> NodeIterator {
  return nodes_.begin();
};

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::end() -> NodeIterator {
  return nodes_.end();
};

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::begin() const -> ConstNodeIterator {
  return nodes_.begin();
};

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::end() const -> ConstNodeIterator {
  return nodes_.end();
};

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::getCollider(const size_t node_index) const -> const ColliderT& {
  return nodes_[node_index].collider();
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::getCollider(const size_t node_index) -> ColliderT& {
  return nodes_[index].collider();
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::intersect(
        const RayType& ray,
        const FloatT t_min,
        const FloatT t_max) const -> RayIntersection {
  return intersect(RayDataType(ray), t_min, t_max);
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::intersect(
        const RayDataType& ray,
        const FloatT t_min,
        const FloatT t_max) const -> RayIntersection {
  RayIntersection cri = _intersect(ray, t_min, t_max);
  return cri;
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::_intersect(
        const RayDataType& ray,
        const FloatT t_min,
        const FloatT t_max) const -> RayIntersection {
  const size_t root_index = 0;
  const size_t depth = 0;
  RayIntersection cri_result;
  _intersectRecursive(root_index, depth, ray, t_min, t_max, &cri_result);
  return cri_result;
}

template <typename ColliderT, typename FloatT>
void AABBTree<ColliderT, FloatT>::_intersectRecursive(
        const size_t node_index,
        const size_t depth,
        const RayDataType& ray,
        const FloatT t_min,
        const FloatT t_max,
        RayIntersection* cri_result) const {
  const Node& node = getNode(node_index);
  const SimpleRayIntersection ri = node.boundingBox().intersect(ray, t_min, t_max);
  // Early break
  if (!ri.doesIntersect() || ri.rayT() > cri_result->rayT()) {
    return;
  }

  if (node.isLeaf()) {
    const SimpleRayIntersection ri = node.collider().intersect(ray, t_min, t_max);
    if (ri.doesIntersect() && ri.rayT() <= cri_result->rayT()) {
      *cri_result = RayIntersection(node_index, depth, ri.rayT());
    }
    return;
  }

  if (node.hasLeftChild()) {
    _intersectRecursive(node.leftChildIndex(), depth + 1, ray, t_min, t_max, cri_result);
  }
  if (node.hasRightChild()) {
    _intersectRecursive(node.rightChildIndex(), depth + 1, ray, t_min, t_max, cri_result);
  }
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::intersectRange(
        const RayType& ray,
        const FloatT t_min,
        const FloatT t_max) const -> std::vector<RayIntersection> {
  return intersectRange(RayDataType(ray), t_min, t_max);
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::intersectRange(
        const RayDataType& ray,
        const FloatT t_min,
        const FloatT t_max) const -> std::vector<RayIntersection> {
  RayIntersection cri = _intersectRange(ray, t_min, t_max);
  return cri;
}

template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::_intersectRange(
        const RayDataType& ray,
        const FloatT t_min,
        const FloatT t_max) const -> std::vector<RayIntersection> {
  const size_t root_index = 0;
  const size_t depth = 0;
  std::vector<RayIntersection> cri_results;
  _intersectRangeRecursive(root_index, depth, ray, t_min, t_max, &cri_results);
  return cri_results;
}

template <typename ColliderT, typename FloatT>
void AABBTree<ColliderT, FloatT>::_intersectRangeRecursive(
        const size_t node_index,
        const size_t depth,
        const RayDataType& ray,
        const FloatT t_min,
        const FloatT t_max,
        std::vector<RayIntersection>* cri_results) const {
  const Node& node = getNode(node_index);
  const SimpleRayIntersection ri = node.boundingBox().intersect(ray, t_min, t_max);
  // Early break
  if (!ri.doesIntersect()) {
    return;
  }

  if (node.isLeaf()) {
    const SimpleRayIntersection ri = node.collider().intersect(ray, t_min, t_max);
    if (ri.doesIntersect()) {
      RayIntersection cri(node_index, depth, ri.rayT());
      cri_results->emplace_back(node_index, depth, ri.rayT());
    }
    return;
  }

  if (node.hasLeftChild()) {
    _intersectRangeRecursive(node.leftChildIndex(), depth + 1, ray, t_min, t_max, cri_results);
  }
  if (node.hasRightChild()) {
    _intersectRangeRecursive(node.rightChildIndex(), depth + 1, ray, t_min, t_max, cri_results);
  }
}

template <typename ColliderT, typename FloatT>
template <typename ColliderIterator>
void AABBTree<ColliderT, FloatT>::build(const ColliderIterator first, const ColliderIterator last) {
  std::vector<ColliderIterator> iterators;
  iterators.reserve(last - first);
  for (ColliderIterator it = first; it != last; ++it) {
    iterators.push_back(it);
  }
  nodes_.clear();
  const size_t root_index= allocateNode();
  const size_t initial_sort_axis = 0;
  splitMedianRecursive<ColliderIterator>(root_index, iterators.begin(), iterators.end(), initial_sort_axis);
  nodes_.shrink_to_fit();
  printInfo();
}

template <typename ColliderT, typename FloatT>
template <typename ColliderIterator>
void AABBTree<ColliderT, FloatT>::splitMedianRecursive(
        const size_t node_index,
        const typename std::vector<ColliderIterator>::iterator first,
        const typename std::vector<ColliderIterator>::iterator last,
        const size_t sort_axis) {
  if (last - first > 1) {
    std::stable_sort(first, last, [&](const ColliderIterator& it_a, const ColliderIterator& it_b) -> bool {
      return it_a->boundingBox().getCenter(sort_axis) < it_b->boundingBox().getCenter(sort_axis);
    });

    const std::size_t next_sort_axis = (sort_axis + 1) % 3;

    // Careful: Make sure to always use getNode(node_index), as the std::vector could be resized in the recursive calls

    const size_t left_child_index = allocateNode();
    getNode(node_index).left_child_index_ = left_child_index;
    const size_t right_child_index = allocateNode();
    getNode(node_index).right_child_index_ = right_child_index;

    const typename std::vector<ColliderIterator>::iterator middle = first + (last - first) / 2;
    splitMedianRecursive<ColliderIterator>(getNode(node_index).leftChildIndex(), first, middle, next_sort_axis);
    splitMedianRecursive<ColliderIterator>(getNode(node_index).rightChildIndex(), middle, last, next_sort_axis);

    updateBoundingBox(&getNode(node_index));
  }
  else {
    const ColliderT collider = **first;
    getNode(node_index).bounding_box_ = collider.boundingBox();
    getNode(node_index).collider_ = collider;
  }
}
template <typename ColliderT, typename FloatT>
auto AABBTree<ColliderT, FloatT>::allocateNode() -> size_t {
  const size_t new_index = nodes_.size();
  nodes_.push_back(Node());
  return new_index;
}

template <typename ColliderT, typename FloatT>
void AABBTree<ColliderT, FloatT>::updateBoundingBox(Node* node) {
  if (node->hasLeftChild() && node->hasRightChild()) {
    node->bounding_box_ = BoundingBoxType::getUnion(
            getNode(node->leftChildIndex()).boundingBox(),
            getNode(node->rightChildIndex()).boundingBox());
  }
  else if (node->hasLeftChild()) {
    node->bounding_box_ = getNode(node->leftChildIndex()).boundingBox();
  }
  else if (node->hasRightChild()) {
    node->bounding_box_ = getNode(node->rightChildIndex()).boundingBox();
  }
  else {
    node->bounding_box_ = node->collider().boundingBox();
  }
}

template <typename ColliderT, typename FloatT>
void AABBTree<ColliderT, FloatT>::printInfo() const {
  const size_t num_nodes = nodes_.size();
  const size_t cur_depth = 0;
  size_t num_leaf_nodes = 0;
  size_t max_depth = 0;
  computeInfoRecursive(&getRoot(), cur_depth, &num_leaf_nodes, &max_depth);
  std::cout << "Info: NumNodes " << num_nodes << std::endl;
  std::cout << "Info: NumLeaves " << num_leaf_nodes << std::endl;
  std::cout << "Info: Tree depth " << max_depth << std::endl;
  std::cout << "Info: Boundingbox " << getRoot().boundingBox() << std::endl;
}

template <typename ColliderT, typename FloatT>
void AABBTree<ColliderT, FloatT>::computeInfoRecursive(
        const Node* node, const size_t cur_depth,
        size_t* num_leaf_nodes, size_t* max_depth) const {
  *max_depth = std::max(cur_depth, *max_depth);
  if (node->isLeaf()) {
    ++(*num_leaf_nodes);
  }
  else {
    if (node->hasLeftChild()) {
      computeInfoRecursive(&getNode(node->leftChildIndex()), cur_depth + 1, num_leaf_nodes, max_depth);
    }
    if (node->hasRightChild()) {
      computeInfoRecursive(&getNode(node->rightChildIndex()), cur_depth + 1, num_leaf_nodes, max_depth);
    }
  }
}

}
