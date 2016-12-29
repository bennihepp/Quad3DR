//==================================================
// graph.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 24, 2016
//==================================================

#pragma once

#include <vector>
#include <unordered_map>
#include <type_traits>
#include <boost/iterator_adaptors.hpp>
#include <ait/common.h>
#include <ait/eigen.h>

template <typename NodeT, typename WeightT>
class Graph {
public:
  using Node = NodeT;
  using WeightType = WeightT;
  using Index = size_t;

  struct NodeContainer {
    using EdgeMap = std::unordered_map<Index, WeightT>;
    Node node;
    EdgeMap edges;
  };
  using NodeVector = std::vector<NodeContainer>;
//  using IndicesVector = std::vector<Index>;
//  using NodeMap = std::unordered_map<Index, NodeContainer>;

  template <typename ValueT, typename BaseIteratorT>
  class _Iterator : public boost::iterator_adaptor<
  _Iterator<ValueT, BaseIteratorT>,
    BaseIteratorT,
    ValueT,
    boost::random_access_traversal_tag> {
  public:
    _Iterator(BaseIteratorT base_it)
    : _Iterator::iterator_adaptor_(base_it) {}

    // Construct from convertible iterator (i.e. const to non-const)
    template <typename OtherIteratorT,
        typename std::enable_if<std::is_convertible<_Iterator, OtherIteratorT>::value, int>::type = 0>
    _Iterator(OtherIteratorT other_it)
    : _Iterator::iterator_adaptor_(other_it.base()) {}

  private:
    friend class boost::iterator_core_access;
    void increment() {
      ++this->base_reference();
    }

    typename _Iterator::iterator_adaptor_::reference dereference() const {
      return this->base_reference()->node;
    }
  };

  using Iterator = _Iterator<Node, typename NodeVector::iterator>;
  using ConstIterator = _Iterator<const Node, typename NodeVector::const_iterator>;

  Graph() {
    clear();
  }

  template <typename Iterator>
  Graph(Iterator begin, Iterator end) {
    clear();
    for (Iterator it = begin; it != end; ++it) {
      NodeContainer container;
      container.node = *it;
      nodes_.push_back(std::move(container));
//      node_indices_.push_back(running_index_);
//      node_map_.emplace(std::make_pair(running_index_, std::move(container)));
//      ++running_index_;
    }
  }

  Graph(Index num_of_nodes) {
    clear();
    nodes_.resize(num_of_nodes);
  }

  Iterator begin() {
    return Iterator(nodes_.begin());
  }

  ConstIterator begin() const {
    return ConstIterator(nodes_.cbegin());
  }

  ConstIterator cbegin() {
    return ConstIterator(nodes_.cbegin());
  }

  Iterator end() {
    return Iterator(nodes_.end());
  }

  ConstIterator end() const {
    return ConstIterator(nodes_.cend());
  }

  ConstIterator cend() {
    return ConstIterator(nodes_.cend());
  }

  void clear() {
    nodes_.clear();
//    running_index_ = 0;
//    node_indices_.clear();
//    node_map_.clear();
  }

  void addNode(const Node& node) {
    NodeContainer container;
    container.node = node;
    nodes_.push_back(std::move(container));
//    node_indices_.push_back(running_index_);
//      node_map_.emplace(std::make_pair(running_index_, std::move(container)));
//    ++running_index_;
  }

  void addNode(Node&& node) {
    NodeContainer container;
    container.node = std::move(node);
    nodes_.push_back(std::move(container));
  }

  Index numOfNodes() const {
    return nodes_.size();
  }

//  Index numOfEdges() const {
//    return edges_.size();
//  }

  const Node& getNode(Index index) const {
    AIT_ASSERT_DBG(index < numOfNodes());
    return nodes_[index].node;
  }
//  const EdgeType& getEdge(Index node_idx1, Index node_idx_2) const;
//  const EdgeType& getEdge(Index edge_idx) const;
//  Index getEdgeIndex(Index node_idx1, Index node_idx_2) const {
//    return
//  }

  bool isConnected(Index node_idx1, Index node_idx2) const {
    AIT_ASSERT_DBG(node_idx1 < numOfNodes());
    AIT_ASSERT_DBG(node_idx2 < numOfNodes());
    return nodes_[node_idx1].edges.count(node_idx2) > 0;
  }

//  WeightT getWeight(Index edge_idx) const {
//    return weights_(node_idx1, node_idx2);
//  }

  WeightT getWeight(Index node_idx1, Index node_idx2) const {
    AIT_ASSERT_DBG(node_idx1 < numOfNodes());
    AIT_ASSERT_DBG(node_idx2 < numOfNodes());
    typename Node::EdgeMap::const_iterator it = nodes_[node_idx1].edges.find(node_idx2);
    if (it != nodes_[node_idx1].cend()) {
      return it->second;
    }
    else {
      return WeightT { 0 };
    }
  }

  void setWeight(Index node_idx1, Index node_idx2, WeightT weight) {
    AIT_ASSERT_DBG(node_idx1 < numOfNodes());
    AIT_ASSERT_DBG(node_idx2 < numOfNodes());
    nodes_[node_idx2].edges[node_idx1] = weight;
    nodes_[node_idx1].edges[node_idx2] = weight;
  }

private:
  NodeVector nodes_;
//  Index running_index_;
//  IndicesVector node_indices_;
//  NodeMap node_map_;
};
