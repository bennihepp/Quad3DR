//==================================================
// graph.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 24, 2016
//==================================================

#pragma once

#include <ait/eigen.h>

template <typename NodeT, typename WeightT>
class Graph {
public:
  using NodeType = NodeT;
//  using EdgeType = EdgeT;
  using WeightType = WeightT;
  using Index = size_t;
  using AdjacencyMatrix = Eigen::Matrix<WeightT, Eigen::Dynamic, Eigen::Dynamic>;

  Graph() {}

  template <typename Iterator>
  Graph(Iterator begin, Iterator end) {
    for (Iterator it = begin; it != end; ++it) {
      nodes_.push_back(std::move(*it));
    }
    weights_.resize(numOfNodes(), numOfNodes());
    weights_.setZero();
  }

  Graph(Index num_of_nodes) {
    nodes_.resize(num_of_nodes);
    weights_.resize(num_of_nodes, num_of_nodes);
    weights_.setZero();
  }

  void addNode(const NodeT& node) {
    nodes_.push_back(node);
    addNodeWeights();
  }

  void addNode(NodeT&& node) {
    nodes_.push_back(std::move(node));
    addNodeWeights();
  }

  Index numOfNodes() const {
    return nodes_.size();
  }

//  Index numOfEdges() const {
//    return edges_.size();
//  }

  const NodeType& getNode(Index index) const {
    return nodes_[index];
  }
//  const EdgeType& getEdge(Index node_idx1, Index node_idx_2) const;
//  const EdgeType& getEdge(Index edge_idx) const;
//  Index getEdgeIndex(Index node_idx1, Index node_idx_2) const {
//    return
//  }

  bool isConnected(Index node_idx1, Index node_idx2) const {
    return weights_(node_idx1, node_idx2) > 0;
  }

//  WeightT getWeight(Index edge_idx) const {
//    return weights_(node_idx1, node_idx2);
//  }

  WeightT getWeight(Index node_idx1, Index node_idx2) const {
    return weights_(node_idx1, node_idx2);
  }

  void setWeight(Index node_idx1, Index node_idx2, WeightT weight) {
    weights_(node_idx1, node_idx2) = weight;
  }

private:
  void addNodeWeights() {
    AdjacencyMatrix new_weights(numOfNodes(), numOfNodes());
    for (Index i = 0; i < weights_.rows(); ++i) {
      for (Index j = 0; j < weights_.cols(); ++j) {
        new_weights(i, j) = weights_(i, j);
      }
    }
    new_weights.row(numOfNodes()).setZero();
    new_weights.col(numOfNodes()).setZero();
    weights_ = new_weights;
  }

  std::vector<NodeT> nodes_;
//  std::vector<EdgeT> edges_;
  AdjacencyMatrix weights_;
};
