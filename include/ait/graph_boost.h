//==================================================
// graph_boost.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Jan 4, 2017
//==================================================
#pragma once

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <ait/common.h>

template <typename NodeT, typename WeightT>
class Graph {
public:
  using Node = NodeT;
  using WeightType = WeightT;
  using Index = std::size_t;

  struct NodePropertyT {
    using kind = boost::vertex_property_tag;
  };

  using BoostGraph = boost::adjacency_list<
      boost::hash_setS, boost::vecS, boost::undirectedS,
      boost::property<NodePropertyT, NodeT>,
      boost::property<boost::edge_weight_t, WeightT>>;

  using Vertex = typename BoostGraph::vertex_descriptor;
  using Edge = typename BoostGraph::edge_descriptor;
  using VertexIterator = typename boost::graph_traits<BoostGraph>::vertex_iterator;
//  using ConstVertexIterator = typename boost::graph_traits<const BoostGraph>::vertex_iterator;
  using EdgeIterator = typename boost::graph_traits<BoostGraph>::edge_iterator;
//  using ConstEdgeIterator = typename boost::graph_traits<const BoostGraph>::edge_iterator;
  using OutEdgeIterator = typename boost::graph_traits<BoostGraph>::out_edge_iterator;
//  using ConstOutEdgeIterator = typename boost::graph_traits<const BoostGraph>::out_edge_iterator;

  using NodePropertyMap = typename boost::property_map<BoostGraph, NodePropertyT>::type;

  Graph() {
    clear();
    node_to_vertex_map_.clear();
  }

  template <typename Iterator>
  Graph(Iterator begin, Iterator end) {
    clear();
    for (Iterator it = begin; it != end; ++it) {
      Vertex v = boost::add_vertex(*it, graph_);
//      NodePropertyMap node_property_map = boost::get(NodePropertyT(), graph_);
//      node_property_map[*v] = *it;
      node_to_vertex_map_.emplace(*it, v);
    }
  }

  const BoostGraph& boostGraph() const {
    return graph_;
  }

  BoostGraph& boostGraph() {
    return graph_;
  }

  std::pair<VertexIterator, VertexIterator> getVertexIterators() {
    return boost::vertices(graph_);
  }

  std::pair<EdgeIterator, EdgeIterator> edges() const {
    return boost::edges(graph_);
  }

  std::pair<OutEdgeIterator, OutEdgeIterator> outEdges(Index node_idx) const {
    return boost::out_edges(node_idx, graph_);
  }

  bool empty() const {
    return size() == 0;
  }

  Index size() const {
    return boost::num_vertices(graph_);
  }

  void clear() {
    graph_.clear();
  }

  Index numVertices() const {
    return boost::num_vertices(graph_);
  }

  Index numEdges() const {
    return boost::num_edges(graph_);
  }

  Index numOutEdges(Vertex v) const {
    return boost::out_degree(v, graph_);
  }

  Index numOutEdgesByNode(Node node) const {
    return numOutEdges(getVertexByNode(node));
  }

  void addNode(const Node& node) {
    Vertex v = boost::add_vertex(node, graph_);
    node_to_vertex_map_.emplace(node, v);
//    boost::put(NodePropertyT(), graph_, v, node);
  }

  void addEdge(Vertex v1, Vertex v2, WeightType weight, bool update = true) {
    bool found;
    Edge edge;
    std::tie(edge, found) = boost::edge(v1, v2, graph_);
    if (!found) {
      boost::add_edge(v1, v2, weight, graph_);
    }
    else if (update) {
      boost::put(boost::edge_weight, graph_, edge, weight);
    }
  }

  void addEdgeByNode(Node node1, Node node2, WeightType weight, bool update = true) {
    addEdge(getVertexByNode(node1), getVertexByNode(node2), weight, update);
  }

  const Node& getNode(Vertex v) const {
    return boost::get(NodePropertyT(), graph_, v);
  }

  Vertex getVertexByNode(NodeT node) const {
    return node_to_vertex_map_.at(node);
  }

  bool isConnected(Vertex v1, Vertex v2) const {
    return boost::edge(v1, v2, graph_).second;
  }

  bool isConnectedByNode(Node node1, Node node2) const {
    return boost::edge(getVertexByNode(node1), getVertexByNode(node2), graph_).second;
  }

  std::pair<OutEdgeIterator, OutEdgeIterator> getEdgeIterators(Index node_idx) {
    return boost::out_edges(node_idx, graph_);
  }

  struct OutEdgeIteratorWrapper : boost::iterator_adaptor<
    OutEdgeIteratorWrapper,
    OutEdgeIterator,
    boost::use_default,
    boost::forward_traversal_tag> {
  public:
    OutEdgeIteratorWrapper()
    : graph_(nullptr) {}

    OutEdgeIteratorWrapper(BoostGraph* graph, OutEdgeIterator base_it)
    : OutEdgeIteratorWrapper::iterator_adaptor_(base_it), graph_(graph) {}

    Vertex source() {
      return boost::source(*(*this), *graph_);
    }

    Node& sourceNode() {
      return boost::get(NodePropertyT(), *graph_, source());
    }

    Vertex target() {
      return boost::target(*(*this), *graph_);
    }

    Node& targetNode() {
      return boost::get(NodePropertyT(), *graph_, target());
    }

    WeightType weight() {
      return boost::get(boost::edge_weight, *graph_, *(*this));
    }

  private:
    friend class boost::iterator_core_access;

    void increment() {
      ++this->base_reference();
    }

    BoostGraph* graph_;
  };

  struct OutEdgesWrapper {
  public:
    OutEdgesWrapper(BoostGraph* graph, Vertex v)
    : graph_(graph), v_(v) {
      std::tie(begin_, end_) = boost::out_edges(v_, *graph_);
    }

    OutEdgeIteratorWrapper begin() const {
      return OutEdgeIteratorWrapper(graph_, begin_);
    }

    OutEdgeIteratorWrapper end() const {
      return OutEdgeIteratorWrapper(graph_, end_);
    }

    bool empty() const {
      return end_ == begin_;
    }

    Index size() {
      return boost::out_degree(v_, *graph_);
    }

    void clear() {
      boost::clear_vertex(v_, *graph_);
      begin_ = end_;
    }

    bool containsTargetNode(Node node) {
      // TODO: Use node-vertex map and boost::edge to check this.
      for (OutEdgeIteratorWrapper it = begin(); it != end(); ++it) {
        if (boost::get(NodePropertyT(), *graph_, it.target()) == node) {
          return true;
        }
      }
      return false;
    }

  private:
    BoostGraph* graph_;
    Vertex v_;
    OutEdgeIterator begin_;
    OutEdgeIterator end_;
  };

  OutEdgesWrapper getEdges(Vertex v) {
    return OutEdgesWrapper(&graph_, v);
  }

  OutEdgesWrapper getEdgesByNode(Node node) {
    return OutEdgesWrapper(&graph_, node_to_vertex_map_[node]);
  }

  struct VertexIteratorWrapper : boost::iterator_adaptor<
    VertexIteratorWrapper,
    VertexIterator,
    boost::use_default,
    boost::forward_traversal_tag> {
  public:
    VertexIteratorWrapper()
    : graph_(nullptr) {}

    VertexIteratorWrapper(BoostGraph* graph, VertexIterator base_it)
    : VertexIteratorWrapper::iterator_adaptor_(base_it), graph_(graph) {}

    Vertex vertex() {
      return *(*this);
    }

    Node node() {
      return boost::get(NodePropertyT(), *graph_, *(*this));
    }

    OutEdgesWrapper edges() {
      return OutEdgesWrapper(graph_, vertex());
    }

  private:
    friend class boost::iterator_core_access;

    void increment() {
      ++this->base_reference();
    }

    BoostGraph* graph_;
  };

  struct VertexWrapper {
  public:
    VertexWrapper(BoostGraph* graph)
    : graph_(graph) {
      std::tie(begin_, end_) = boost::vertices(*graph_);
    }

    VertexIteratorWrapper begin() const {
      return VertexIteratorWrapper(graph_, begin_);
    }

    VertexIteratorWrapper end() const {
      return VertexIteratorWrapper(graph_, end_);
    }

    bool empty() const {
      return end_ == begin_;
    }

    Index size() const {
      return end_ - begin_;
    }

    void clear() {
      for (OutEdgeIterator it = begin_; it != end_; ++it) {
        boost::clear_vertex(it, *graph_);
      }
      begin_ = end_;
    }

  private:
    BoostGraph* graph_;
    VertexIterator begin_;
    VertexIterator end_;
  };

  VertexWrapper vertices() {
    return VertexWrapper(&graph_);
  }

  VertexIteratorWrapper begin() {
    return vertices().begin();
  }

  VertexIteratorWrapper end() {
    return vertices().end();
  }

  WeightType getWeight(Vertex v1, Vertex v2) const {
    std::pair<Edge, bool> result = boost::edge(v1, v2, graph_);
    if (result.second) {
      return boost::get(boost::edge_weight, graph_, result.first);
    }
    else {
      return std::numeric_limits<WeightType>::max();
//      return WeightType { -1 };
    }
  }

  WeightType getWeightByNode(Node node1, Node node2) const {
    return getWeight(getVertexByNode(node1), getVertexByNode(node2));
  }

private:
  BoostGraph graph_;
  std::unordered_map<Node, Vertex> node_to_vertex_map_;

  // Boost serialization
  friend class boost::serialization::access;

  template <typename Archive>
  void save(Archive& ar, const unsigned int version) const {
    ar & graph_;
  }

  template <typename Archive>
  void load(Archive& ar, const unsigned int version) {
    ar & graph_;
    node_to_vertex_map_.clear();
    for (auto it = this->begin(); it != this->end(); ++it) {
      node_to_vertex_map_.emplace(it.node(), *it);
    }
  }

  BOOST_SERIALIZATION_SPLIT_MEMBER()
};
