//==================================================
// occupancy_map_node_navigator.hxx
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 15, 2016
//==================================================

template <typename TreeT, typename NodeT>
TreeNavigator<TreeT, NodeT> TreeNavigator<TreeT, NodeT>::getRootNavigator(TreeT* tree) {
  return TreeNavigator(tree, tree->getRootKey(), tree->getRoot(), 0);
}

template <typename TreeT, typename NodeT>
TreeNavigator<TreeT, NodeT>::TreeNavigator(TreeT* tree, OcTreeKey key, NodeT* node, size_t depth)
: tree_(tree), key_(key), node_(node), depth_(depth) {}

template <typename TreeT, typename NodeT>
template <typename OtherTreeT, typename OtherNodeT>
TreeNavigator<TreeT, NodeT>::TreeNavigator(const TreeNavigator<OtherTreeT, OtherNodeT>& other)
: tree_(other.tree_), key_(other.key_), node_(other.node_), depth_(other.depth_) {}

template <typename TreeT, typename NodeT>
TreeT* TreeNavigator<TreeT, NodeT>::getTree() {
  return tree_;
}

template <typename TreeT, typename NodeT>
const OcTreeKey& TreeNavigator<TreeT, NodeT>::getKey() const {
  return key_;
}

template <typename TreeT, typename NodeT>
size_t TreeNavigator<TreeT, NodeT>::getDepth() const {
  return depth_;
}

template <typename TreeT, typename NodeT>
const NodeT* TreeNavigator<TreeT, NodeT>::getNode() const {
  return node_;
}

template <typename TreeT, typename NodeT>
NodeT* TreeNavigator<TreeT, NodeT>::getNode(){
  return node_;
}

template <typename TreeT, typename NodeT>
const NodeT* TreeNavigator<TreeT, NodeT>::operator->() const {
  return node_;
}

template <typename TreeT, typename NodeT>
NodeT* TreeNavigator<TreeT, NodeT>::operator->() {
  return node_;
}

template <typename TreeT, typename NodeT>
const NodeT* TreeNavigator<TreeT, NodeT>::operator*() const {
  return node_;
}

template <typename TreeT, typename NodeT>
NodeT* TreeNavigator<TreeT, NodeT>::operator*() {
  return node_;
}

template <typename TreeT, typename NodeT>
float TreeNavigator<TreeT, NodeT>::getSize() const {
  return tree_->getNodeSize(depth_);
}

template <typename TreeT, typename NodeT>
Eigen::Vector3f TreeNavigator<TreeT, NodeT>::getPosition() const {
  const Eigen::Vector3f pos = tree_->keyToCoordEigen(key_, depth_);
  return pos;
}

template <typename TreeT, typename NodeT>
bool TreeNavigator<TreeT, NodeT>::hasParent() const {
  return depth_ > 0;
}

template <typename TreeT, typename NodeT>
bool TreeNavigator<TreeT, NodeT>::hasChildren() const {
  return node_->hasChildren();
}

template <typename TreeT, typename NodeT>
bool TreeNavigator<TreeT, NodeT>::hasChild(size_t i) const {
  return node_->hasChild(i);
}

template <typename TreeT, typename NodeT>
void TreeNavigator<TreeT, NodeT>::gotoChild(size_t i) {
  size_t child_depth = depth_ + 1;
  const unsigned short int center_offset_key = tree_->getTreeMaxValue() >> child_depth;
  OcTreeKey child_key;
  octomap::computeChildKey(i, center_offset_key, key_, child_key);
  key_ = child_key;
  node_ = tree_->getNodeChild(node_, i);
  depth_ = child_depth;
}

template <typename TreeT, typename NodeT>
void TreeNavigator<TreeT, NodeT>::gotoParent() {
  key_ = tree_->getParentKey(key_, depth_);
  node_ = node_->getParent();
  depth_ = depth_ - 1;
}

template <typename TreeT, typename NodeT>
TreeNavigator<TreeT, NodeT> TreeNavigator<TreeT, NodeT>::child(size_t i) const {
  size_t child_depth = depth_ + 1;
  const unsigned short int center_offset_key = tree_->getTreeMaxValue() >> child_depth;
  OcTreeKey child_key;
  octomap::computeChildKey(i, center_offset_key, key_, child_key);
  return TreeNavigator(tree_, child_key, tree_->getNodeChild(node_, i), child_depth);
}

template <typename TreeT, typename NodeT>
TreeNavigator<TreeT, NodeT> TreeNavigator<TreeT, NodeT>::parent() const {
  OcTreeKey parent_key = tree_->getParentKey(key_, depth_);
  NodeT* parent_node = node_->getParent();
  size_t parent_depth = depth_ - 1;
  return TreeNavigator(tree_, parent_key, parent_node, parent_depth);
}
