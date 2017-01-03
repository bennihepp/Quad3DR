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
#include <boost/serialization/access.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <ait/common.h>
#include <ait/eigen.h>

template <typename NodeT, typename WeightT>
class Graph {
public:
  using Node = NodeT;
  using WeightType = WeightT;
  using Index = std::size_t;

  struct NodeContainer {
    using EdgeMap = std::unordered_map<Index, WeightType>;

    Node node;
    EdgeMap edges;

  private:
    // Boost serialization
    friend class boost::serialization::access;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version) {
      ar & node;
      ar & edges;
    }
  };

  using NodeVector = std::vector<NodeContainer>;

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

  bool empty() const {
    return nodes_.empty();
  }

  Index size() const {
    return nodes_.size();
  }

  void clear() {
    nodes_.clear();
  }

  void addNode(const Node& node) {
    NodeContainer container;
    container.node = node;
    nodes_.push_back(std::move(container));
  }

  void addNode(Node&& node) {
    NodeContainer container;
    container.node = std::move(node);
    nodes_.push_back(std::move(container));
  }

  void addEdge(Index idx1, Index idx2, WeightType weight) {
    AIT_ASSERT_DBG(idx1 < size());
    AIT_ASSERT_DBG(idx2 < size());
    nodes_[idx1].edges.insert(std::make_pair(idx2, weight));
  }

  const Node& getNode(Index index) const {
    AIT_ASSERT_DBG(index < size());
    return nodes_[index].node;
  }
  bool isConnected(Index node_idx1, Index node_idx2) const {
    AIT_ASSERT_DBG(node_idx1 < size());
    AIT_ASSERT_DBG(node_idx2 < size());
    return nodes_[node_idx1].edges.count(node_idx2) > 0;
  }

  // TODO: Make extra structure with clean iterator?
  typename NodeContainer::EdgeMap& getEdges(Index node_idx) {
    AIT_ASSERT_DBG(node_idx < size());
    return nodes_[node_idx].edges;
  }

  // TODO: Make extra structure with clean iterator?
  const typename NodeContainer::EdgeMap& getEdges(Index node_idx) const {
    AIT_ASSERT_DBG(node_idx < size());
//    std::cout << "nodes_[node_idx].node=" << nodes_[node_idx].node << std::endl;
//    std::cout << "nodes_[node_idx].edges.size()=" << nodes_[node_idx].edges.size() << std::endl;
    return nodes_[node_idx].edges;
  }

  WeightType getWeight(Index node_idx1, Index node_idx2) const {
    AIT_ASSERT_DBG(node_idx1 < size());
    AIT_ASSERT_DBG(node_idx2 < size());
    const NodeContainer& node_container1 = nodes_[node_idx1];
    typename NodeContainer::EdgeMap::const_iterator it = node_container1.edges.find(node_idx2);
    if (it != node_container1.edges.cend()) {
      return it->second;
    }
    else {
      return WeightType { -1 };
    }
  }

private:
  NodeVector nodes_;

  // Boost serialization
  friend class boost::serialization::access;

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar & nodes_;
  }
};
