//==================================================
// viewpoint_planner_tsp.cpp
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Jan 4, 2017
//==================================================

#include "viewpoint_planner.h"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/metric_tsp_approx.hpp>

using std::swap;

namespace {

struct astar_found_goal {};

template <typename Graph>
class astar_goal_visitor : public boost::default_astar_visitor {
public:
  using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor;

  astar_goal_visitor(Vertex goal)
  : goal_(goal) {}

//  void examine_vertex()
  void finish_vertex(Vertex v, Graph& g) {
    if (v == goal_) {
      throw astar_found_goal();
    }
  }

private:
  Vertex goal_;
};

template <typename ViewpointPlanner, typename CostType>
class astar_distance_heuristic : public boost::astar_heuristic<typename ViewpointPlanner::ViewpointGraph::BoostGraph, CostType> {
public:
  using Vertex = typename boost::graph_traits<typename ViewpointPlanner::ViewpointGraph::BoostGraph>::vertex_descriptor;

  astar_distance_heuristic(ViewpointPlanner& planner, Vertex goal)
  : planner_(planner) {
    typename ViewpointPlanner::ViewpointEntryIndex goal_index = planner_.getViewpointGraph().getNode(goal);
    goal_viewpoint_ = planner_.getViewpointEntries()[goal_index];
  }

  CostType operator()(Vertex v) {
    typename ViewpointPlanner::ViewpointEntryIndex index = planner_.getViewpointGraph().getNode(v);
    typename ViewpointPlanner::ViewpointEntry viewpoint = planner_.getViewpointEntries()[index];
    return (goal_viewpoint_.viewpoint.pose().getWorldPosition() - viewpoint.viewpoint.pose().getWorldPosition()).norm();
  }

private:
  ViewpointPlanner& planner_;
  typename ViewpointPlanner::ViewpointEntry goal_viewpoint_;
};

}

std::pair<std::vector<std::size_t>, std::size_t> ViewpointPlanner::getConnectedComponents() const {
  // TODO: Cache connected components
  std::vector<std::size_t> component(viewpoint_graph_.numVertices());
  std::size_t num_components = boost::connected_components(viewpoint_graph_.boostGraph(), &component.front());
  return std::make_pair(component, num_components);
}

bool ViewpointPlanner::findAndAddShortestMotion(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index) {
  const bool verbose = true;

  ViewpointGraph::Vertex start = viewpoint_graph_.getVertexByNode(from_index);
  ViewpointGraph::Vertex goal = viewpoint_graph_.getVertexByNode(to_index);
  if (start == goal) {
    if (verbose) {
      std::cout << "viewpoints are the same so no connection is necessary" << std::endl;
    }
    return true;
  }
  if (viewpoint_graph_.isConnected(start, goal)) {
    if (verbose) {
      std::cout << "viewpoints already connected" << std::endl;
    }
    return true;
  }
  std::vector<ViewpointGraph::Vertex> predecessors(viewpoint_graph_.size());
  std::vector<FloatType> distances(viewpoint_graph_.size());
  bool found_goal = false;
  try {
    boost::astar_search(viewpoint_graph_.boostGraph(),
        start,
        astar_distance_heuristic<ViewpointPlanner, FloatType>(*this, goal),
        boost::visitor(astar_goal_visitor<const ViewpointGraph::BoostGraph>(goal)).
        predecessor_map(&predecessors.front()).
        distance_map(&distances.front()));
  } catch (const astar_found_goal& fg) {
    FloatType motion_distance = distances[goal];
    found_goal = true;
    if (verbose) {
      std::cout << "  Found path to " << to_index << " with distance " << motion_distance
          << ", line of sight distance is "
          << (viewpoint_entries_[from_index].viewpoint.pose().getWorldPosition() - viewpoint_entries_[to_index].viewpoint.pose().getWorldPosition()).norm() << std::endl;
    }
    std::vector<ViewpointEntryIndex> reverse_viewpoint_indices_path;
    for (ViewpointGraph::Vertex v = goal;; v = predecessors[v]) {
      if (verbose) {
        std::cout << "   " << viewpoint_graph_.getNode(v) << std::endl;
      }
      reverse_viewpoint_indices_path.push_back(viewpoint_graph_.getNode(v));
      if (predecessors[v] == v) {
        break;
      }
    }
    AIT_ASSERT(reverse_viewpoint_indices_path.back() == from_index);
    AIT_ASSERT(reverse_viewpoint_indices_path.front() == to_index);
    Motion motion;
    motion.distance = 0;
    motion.cost = 0;
    for (auto it = reverse_viewpoint_indices_path.rbegin() + 1; it < reverse_viewpoint_indices_path.rend(); ++it) {
      const ViewpointEntryIndex idx1 = *(it - 1);
      const ViewpointEntryIndex idx2 = *it;
      const Motion& sub_motion = getViewpointMotion(idx1, idx2);
      motion.distance += sub_motion.distance;
      motion.cost += sub_motion.cost;
      for (const Pose& pose : sub_motion.poses) {
        motion.poses.push_back(pose);
      }
      // TODO: Not sure if this helps to speed up later searches
      addViewpointMotion(from_index, idx2, motion);
    }
//    addViewpointMotion(from_index, to_index, std::move(motion));
    AIT_ASSERT(viewpoint_graph_.numEdges() == viewpoint_graph_motions_.size());
  }
  if (verbose && !found_goal) {
    std::cout << "Could not find a motion from " << from_index << " to " << to_index << std::endl;
  }
  //  AIT_ASSERT(found_goal);
  return found_goal;
}

ViewpointPlanner::ViewpointPathGraphWrapper ViewpointPlanner::createViewpointPathGraph(
    const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data) {
  const bool verbose = false;

  using BoostGraph = ViewpointPathGraphWrapper::BoostGraph;
  using Vertex = ViewpointPathGraphWrapper::Vertex;
  using IndexMap = ViewpointPathGraphWrapper::IndexMap;
  using WeightProperty = ViewpointPathGraphWrapper::WeightProperty;

  if (verbose) {
    std::cout << "Building subgraph for viewpoint path" << std::endl;
  }
  ViewpointPathGraphWrapper graph_wrapper;
  graph_wrapper.graph = BoostGraph(comp_data.num_connected_entries);
  graph_wrapper.index_map = boost::get(boost::vertex_index, graph_wrapper.graph);
  std::unordered_map<Vertex, ViewpointEntryIndex>& viewpoint_indices = graph_wrapper.viewpoint_indices;
  std::unordered_map<ViewpointEntryIndex, Vertex>& viewpoint_index_to_vertex_map = graph_wrapper.viewpoint_index_to_vertex_map;
  BoostGraph& graph = graph_wrapper.graph;

  if (verbose) {
    for (std::size_t i = 0; i < comp_data.num_connected_entries; ++i) {
      const ViewpointEntryIndex index1 = viewpoint_path.entries[i].viewpoint_index;
      for (std::size_t j = i+1; j < comp_data.num_connected_entries; ++j) {
        const ViewpointEntryIndex index2 = viewpoint_path.entries[j].viewpoint_index;
        std::cout << "Viewpoint " << index1 << " connected to " << index2 << "? "
            << viewpoint_graph_.isConnectedByNode(index1, index2) << std::endl;
      }
    }
  }

  // Fill maps for viewpoint index to node and vice versa
  for (std::size_t i = 0; i < comp_data.num_connected_entries; ++i) {
    Vertex v1 = boost::vertex(i, graph);
    const ViewpointPathEntry& path_entry = viewpoint_path.entries[i];
    viewpoint_indices.emplace(v1, path_entry.viewpoint_index);
    viewpoint_index_to_vertex_map.emplace(path_entry.viewpoint_index, v1);
  }
  // Add edges to graph
  for (std::size_t i = 0; i < comp_data.num_connected_entries; ++i) {
    Vertex v1 = boost::vertex(i, graph);
    const ViewpointPathEntry& path_entry = viewpoint_path.entries[i];
    auto edges = viewpoint_graph_.getEdgesByNode(path_entry.viewpoint_index);
    for (auto it = edges.begin(); it != edges.end(); ++it) {
      // TODO: There should be no loops in the graph. Could be removed
      if (path_entry.viewpoint_index == it.targetNode()) {
        continue;
      }
      auto it_target = viewpoint_index_to_vertex_map.find(it.targetNode());
      if (it_target != viewpoint_index_to_vertex_map.end()) {
        Vertex v2 = it_target->second;
        boost::add_edge(v1, v2, WeightProperty(it.weight()), graph);
//        if (verbose) {
//          std::cout << "Adding edge from " << v1 << " to " << v2 << std::endl;
//        }
      }
    }
  }

  if (verbose) {
    IndexMap index_map = boost::get(boost::vertex_index, graph);
    std::cout << "Number of edges: " << boost::num_edges(graph) << std::endl;
    boost::graph_traits<BoostGraph>::edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = boost::edges(graph); ei != ei_end; ++ei) {
      std::cout << "  edge from " << index_map[boost::source(*ei, graph)] << " (" << boost::source(*ei, graph) << ")"
          << " to " << index_map[boost::target(*ei, graph)] << " (" << boost::target(*ei, graph) << ")"
          << " with weight " << boost::get(boost::edge_weight, graph, *ei) << std::endl;
    }
  }

  return graph_wrapper;
}

std::vector<std::size_t>
ViewpointPlanner::computeApproximateShortestCycle(const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data) {
  const bool verbose = false;

  if (comp_data.num_connected_entries == 0) {
    if (verbose) {
      std::cout << "Trying to find shortest cycle for empty viewpoint set" << std::endl;
    }
    return std::vector<std::size_t>();
  }

  using BoostGraph = ViewpointPathGraphWrapper::BoostGraph;
  using Vertex = ViewpointPathGraphWrapper::Vertex;

  ViewpointPathGraphWrapper graph_wrapper = createViewpointPathGraph(viewpoint_path, comp_data);
  BoostGraph& graph = graph_wrapper.graph;

  if (verbose) {
    std::cout << "Solving TSP" << std::endl;
  }
  // Solve TSP
  std::vector<Vertex> tsp_tour;
  boost::metric_tsp_approx_tour(graph, std::back_inserter(tsp_tour));
  if (verbose) {
    std::cout << "Boost TSP tour: " << std::endl;
    for (const Vertex& vertex : tsp_tour) {
      std::cout << vertex << std::endl;
    }
  }
  std::vector<std::size_t> viewpoint_tsp_tour;
  viewpoint_tsp_tour.resize(tsp_tour.size() - 1);
  std::transform(tsp_tour.begin(), tsp_tour.end() - 1, viewpoint_tsp_tour.begin(),
      [&](const Vertex& v) {
    return graph_wrapper.index_map[v];
  });
  if (verbose) {
    std::cout << "Viewpoint TSP tour: " << std::endl;
    for (const std::size_t i : viewpoint_tsp_tour) {
      const ViewpointEntryIndex viewpoint_index = viewpoint_path.entries[i].viewpoint_index;
      std::cout << viewpoint_index << std::endl;
    }
  }
  return viewpoint_tsp_tour;
}

void ViewpointPlanner::ensureConnectedViewpointPath(
    ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data) {
  for (std::size_t i = comp_data->num_connected_entries; i < viewpoint_path->entries.size(); ++i) {
    const ViewpointPathEntry& path_entry = viewpoint_path->entries[i];
    // Ensure that the new viewpoints are connected to all other viewpoints in the path
    bool found_motions = findAndAddShortestMotions(
        path_entry.viewpoint_index, viewpoint_path->entries.begin(), viewpoint_path->entries.begin() + i);

    if (!found_motions) {
      std::cout << "Could not find motion path for viewpoint on path" << path_entry.viewpoint_index << std::endl;
    }
    AIT_ASSERT(found_motions);
    ++comp_data->num_connected_entries;
  }
}

void ViewpointPlanner::solveApproximateTSP(
    ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data) {
  const bool verbose = true;

  std::vector<std::size_t> viewpoint_tsp_tour = computeApproximateShortestCycle(*viewpoint_path, *comp_data);
  viewpoint_path->order = std::move(viewpoint_tsp_tour);
  // TODO: Remove
//  std::vector<ViewpointPathEntry> old_path_entries = std::move(viewpoint_path->entries);
//  viewpoint_path->entries.clear();
////  if (comp_data != nullptr) {
////    comp_data->sorted_new_informations.clear();
////  }
//  for (const ViewpointEntryIndex viewpoint_index : viewpoint_tsp_tour) {
//    auto it = std::find_if(old_path_entries.begin(), old_path_entries.end(), [&](const ViewpointPathEntry& entry) -> bool {
//      return entry.viewpoint_index == viewpoint_index;
//    });
//    ViewpointPathEntry path_entry = std::move(*it);
//    viewpoint_path->entries.push_back(std::move(path_entry));
////    if (comp_data != nullptr) {
////      if (comp_data->sorted_new_informations.empty()) {
////        initializeViewpointPathInformations(viewpoint_path, comp_data);
////      }
////      else {
////        updateViewpointPathInformations(viewpoint_path, comp_data);
////      }
////    }
//  }
  if (verbose) {
    std::cout << "After reordering" << std::endl;
    for (const std::size_t i : viewpoint_path->order) {
      std::cout << "  i=" << i << " entry=" << viewpoint_path->entries[i].viewpoint_index << std::endl;
    }
  }
}

std::vector<std::size_t> ViewpointPlanner::swap2Opt(
    ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data,
    const std::size_t k_start, const std::size_t k_end) const {
  std::vector<std::size_t> new_order(viewpoint_path->order.size());
  for (std::size_t i = 0; i < k_start; ++i) {
    new_order[i] = viewpoint_path->order[i];
  }
  for (std::size_t i = 0; i <= k_end - k_start; ++i) {
    new_order[k_start + i] = viewpoint_path->order[k_end - i];
  }
  for (std::size_t i = k_end + 1; i < new_order.size(); ++i) {
    new_order[i] = viewpoint_path->order[i];
  }
  return new_order;
}

ViewpointPlanner::FloatType ViewpointPlanner::computeTourLength(const ViewpointPath& viewpoint_path, const std::vector<std::size_t>& order) const {
  FloatType tour_length = 0;
  for (auto it = order.begin(); it != order.end(); ++it) {
    auto next_it = it + 1;
    if (next_it == order.end()) {
      next_it = order.begin();
    }
    const ViewpointEntryIndex from_index = viewpoint_path.entries[*it].viewpoint_index;
    const ViewpointEntryIndex to_index = viewpoint_path.entries[*next_it].viewpoint_index;
    const FloatType dist = viewpoint_graph_.getWeightByNode(from_index, to_index);
    if (dist != getViewpointMotion(from_index, to_index).distance) {
      std::cout << "dist != getViewpointMotion(from_index, to_index).distance " << __FILE__ << ":" << __LINE__ << std::endl;
      AIT_PRINT_VALUE(dist);
      AIT_PRINT_VALUE(getViewpointMotion(from_index, to_index).distance);
    }
//    AIT_ASSERT(dist == getViewpointMotion(from_index, to_index).distance);
    tour_length += dist;
  }
  return tour_length;
}

void ViewpointPlanner::improveViewpointTourWith2Opt(
    ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data) {
  const bool verbose = true;
  const std::size_t max_k_length = options_.viewpoint_path_2opt_max_k_length;

  if (verbose) {
    std::cout << "Improving tour with 2 Opt" << std::endl;
  }

  const FloatType initial_tour_length = computeTourLength(*viewpoint_path, viewpoint_path->order);
  FloatType shortest_tour_length = initial_tour_length;
  bool improvement_made;
  do {
    improvement_made = false;
    for (std::size_t k_start = 1; k_start < viewpoint_path->order.size(); ++k_start) {
      const std::size_t max_k_end = std::min(k_start + max_k_length, viewpoint_path->order.size());
      for (std::size_t k_end = k_start + 1; k_end < max_k_end; ++k_end) {
        std::vector<std::size_t> new_order = swap2Opt(viewpoint_path, comp_data, k_start, k_end);
        const FloatType new_tour_length = computeTourLength(*viewpoint_path, new_order);
        if (new_tour_length < shortest_tour_length) {
          viewpoint_path->order = std::move(new_order);
          shortest_tour_length = new_tour_length;
          improvement_made = true;
        }
      }
    }
  } while (improvement_made);
  if (verbose) {
    if (shortest_tour_length < initial_tour_length) {
      std::cout << "After 2 Opt" << std::endl;
      for (const std::size_t i : viewpoint_path->order) {
        std::cout << "  i=" << i << " entry=" << viewpoint_path->entries[i].viewpoint_index << std::endl;
      }
      std::cout << "Improved tour length from " << initial_tour_length << " to " << shortest_tour_length << std::endl;
    }
    else {
      std::cout << "Unable to improve tour with 2 Opt" << std::endl;
    }
  }
  if (verbose) {
    const FloatType tour_length_lower_bound = initial_tour_length / 2;
    std::cout << "Current tour: length=" << shortest_tour_length << ", lower bound=" << tour_length_lower_bound
        << ", ratio=" << (shortest_tour_length / tour_length_lower_bound) << std::endl;
  }
}
