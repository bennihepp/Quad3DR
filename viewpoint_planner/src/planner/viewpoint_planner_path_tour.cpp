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
#include <boost/heap/binomial_heap.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <bh/algorithm.h>

using std::swap;

void ViewpointPlanner::computeViewpointTour(const bool use_manual_start_position) {
  const bool verbose = true;
  const bool recompute_all = true;
#pragma omp parallel for
  for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
    ViewpointPath& viewpoint_path = viewpoint_paths_[i];
    ViewpointPathComputationData& comp_data = viewpoint_paths_data_[i];
    ensureConnectedViewpointPath(&viewpoint_path, &comp_data, recompute_all);
  }

  // Solve TSP problem
#pragma omp parallel for
  for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
    ViewpointPath& viewpoint_path = viewpoint_paths_[i];
    ViewpointPathComputationData& comp_data = viewpoint_paths_data_[i];
    solveApproximateTSP(&viewpoint_path, &comp_data);
  }

  if (options_.viewpoint_path_2opt_enable) {
    //  // Improve solution with 2 Opt
    // Cannot use multiple threads due to offscreen rendering for sparse matching
    //#pragma omp parallel for
    for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
      ViewpointPath &viewpoint_path = viewpoint_paths_[i];
      ViewpointPathComputationData &comp_data = viewpoint_paths_data_[i];
      improveViewpointTourWith2Opt(&viewpoint_path, &comp_data);
    }
  }

  if (use_manual_start_position && options_.isSet("drone_start_position")) {
    // Reorder viewpoint tour to start at drone_start_position
    for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
      ViewpointPath& viewpoint_path = viewpoint_paths_[i];
      const auto closest_it = bh::argmin(
              viewpoint_path.order.begin(), viewpoint_path.order.end(),
              [&](const size_t order_idx) {
                const ViewpointPathEntry &path_entry = viewpoint_path.entries[order_idx];
                const Vector3 &world_position = path_entry.viewpoint.pose().getWorldPosition();
                const FloatType dist = (world_position - options_.drone_start_position).squaredNorm();
                return dist;
              }
      );
      if (verbose) {
        std::cout << "Path entry closest to drone start position: "
                  << viewpoint_path.entries[*closest_it].viewpoint_index << std::endl;
      }
      std::vector<size_t> new_order;
      auto it = closest_it;
      do {
        new_order.push_back(*it);
        ++it;
        if (it == std::end(viewpoint_path.order)) {
          it = std::begin(viewpoint_path.order);
        }
      }
      while (it != closest_it);
      viewpoint_path.order = std::move(new_order);
    }
  }

  for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
    ViewpointPath& viewpoint_path = viewpoint_paths_[i];
    std::cout << "Viewpoint path " << i << std::endl;
    for (auto it = viewpoint_path.order.begin(); it != viewpoint_path.order.end(); ++it) {
      auto next_it = it + 1;
      if (next_it == viewpoint_path.order.end()) {
        next_it = viewpoint_path.order.begin();
      }
      const Pose& pose1 = viewpoint_path.entries[*it].viewpoint.pose();
      const Pose& pose2 = viewpoint_path.entries[*next_it].viewpoint.pose();
      const ViewpointEntryIndex viewpoint_index1 = viewpoint_path.entries[*it].viewpoint_index;
      const ViewpointEntryIndex viewpoint_index2 = viewpoint_path.entries[*next_it].viewpoint_index;
//      const bool matchable = isSparseMatchable(viewpoint_index1, viewpoint_index2);
      const bool matchable = isSparseMatchable2(viewpoint_index1, viewpoint_index2);
      const FloatType translation_distance = pose1.getPositionDistanceTo(pose2);
      const FloatType angular_distance = pose1.getAngularDistanceTo(pose2);
      std::cout << "  " << viewpoint_index1 << " -> " << viewpoint_index2 << ": "
                << "matchable=" << matchable
                << " translation_distance=" << translation_distance
                << " angular_distance=" << angular_distance * 180 / M_PI << std::endl;
    }
  }

  if (verbose) {
    reportViewpointPathsStats();
  }
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
    ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data, const bool recompute_all /*= false*/) {
  if (recompute_all) {
    comp_data->num_connected_entries = 0;
  }
  for (std::size_t i = comp_data->num_connected_entries; i < viewpoint_path->entries.size(); ++i) {
    const ViewpointPathEntry& path_entry = viewpoint_path->entries[i];
    // Ensure that the new viewpoints are connected to all other viewpoints in the path
//    bool found_motions = findAndAddShortestMotions(
//        path_entry.viewpoint_index, viewpoint_path->entries.begin(), viewpoint_path->entries.begin() + i);
    // TODO
    bool found_motions = true;
    for (auto it = viewpoint_path->entries.begin(); it != viewpoint_path->entries.begin() + i; ++it) {
      const ViewpointEntryIndex to_index = it->viewpoint_index;
  //    std::cout << "  searching path to viewpoint " << to_index << std::endl;
      bool found = findAndAddShortestMotion(path_entry.viewpoint_index, to_index);
      BH_ASSERT(found);
      if (!found) {
        found_motions = false;
        break;
      }
    }
    if (!found_motions) {
      std::cout << "Could not find motion path for viewpoint on path" << path_entry.viewpoint_index << std::endl;
    }
    BH_ASSERT(found_motions);
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
  if (order.size() <= 1) {
    return 0;
  }
  FloatType tour_length = 0;
  for (auto it = order.begin(); it != order.end(); ++it) {
    auto next_it = it + 1;
    if (next_it == order.end()) {
      next_it = order.begin();
    }
    const ViewpointEntryIndex from_index = viewpoint_path.entries[*it].viewpoint_index;
    const ViewpointEntryIndex to_index = viewpoint_path.entries[*next_it].viewpoint_index;
    if (!hasViewpointMotion(from_index, to_index)) {
      std::cout << "WARNING: No motion from viewpoint " << from_index << " to viewpoint " << to_index << std::endl;
      tour_length += std::numeric_limits<FloatType>::infinity();
    }
    else {
      const FloatType dist = viewpoint_graph_.getWeightByNode(from_index, to_index);
      if (!bh::isApproxEqual(dist, getViewpointMotion(from_index, to_index).distance(), FloatType(1e-2))) {
        std::cout << "WARNING: dist != getViewpointMotion(from_index, to_index).distance " << __FILE__ << ":" << __LINE__ << std::endl;
        BH_PRINT_VALUE(dist);
        BH_PRINT_VALUE(getViewpointMotion(from_index, to_index).distance());
      }
//      BH_ASSERT(dist == getViewpointMotion(from_index, to_index).distance);
      tour_length += dist;
    }
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
          // Check if new tour is matchable
          bool matchable = true;
          if (options_.viewpoint_path_2opt_check_sparse_matching) {
            for (size_t i = k_start - 1; i < k_end + 1; ++i) {
              const size_t from_index = i;
              size_t to_index = i + 1;
              if (to_index == new_order.size()) {
                to_index = 0;
              }
              if (!isSparseMatchable2(from_index, to_index)) {
                matchable = false;
                break;
              }
            }
          }
          if (matchable) {
            viewpoint_path->order = std::move(new_order);
            shortest_tour_length = new_tour_length;
            improvement_made = true;
          }
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
    size_t num_sparse_matching_viewpoints = 0;
    for (auto it = viewpoint_path->order.begin(); it != viewpoint_path->order.end(); ++it) {
      auto next_it = it + 1;
      if (next_it == viewpoint_path->order.end()) {
        next_it = viewpoint_path->order.begin();
      }
      const ViewpointMotion motion = getViewpointMotion(
          viewpoint_path->entries[*it].viewpoint_index, viewpoint_path->entries[*next_it].viewpoint_index);
      num_sparse_matching_viewpoints += motion.viewpointIndices().size() - 2;
    }
    std::cout << "Number of viewpoints=" << viewpoint_path->entries.size() << ", "
        << "number of sparse matching viewpoints=" << num_sparse_matching_viewpoints << std::endl;
  }
}

void ViewpointPlanner::augmentedViewpointPathWithSparseMatchingViewpoints(ViewpointPath* viewpoint_path) {
  ViewpointPath new_viewpoint_path(*viewpoint_path);
  if (viewpoint_path->order.size() > 1) {
    new_viewpoint_path.order.clear();
    for (auto it = viewpoint_path->order.begin(); it != viewpoint_path->order.end(); ++it) {
      auto next_it = it + 1;
      if (next_it == viewpoint_path->order.end()) {
        next_it = viewpoint_path->order.begin();
      }
      new_viewpoint_path.order.push_back(*it);
//      if (new_viewpoint_path.order.size() > 1) {
//        // TODO
//        bool draw_lines = false;
//        const Viewpoint& viewpoint1 = new_viewpoint_path.entries[*(new_viewpoint_path.order.rbegin() + 1)].viewpoint;
//        const Viewpoint& viewpoint2 = new_viewpoint_path.entries[*(new_viewpoint_path.order.rbegin())].viewpoint;
//        if (options_.dump_stereo_matching_images) {
//          dumpSparseMatching(
//              viewpoint1, viewpoint2,
//              "dump/sparse_points_augmented_" + std::to_string(it - viewpoint_path->order.begin()) + "_" +
//              std::to_string(new_viewpoint_path.entries.size()) + ".png",
//              draw_lines);
//          draw_lines = true;
//          dumpSparseMatching(
//              viewpoint1, viewpoint2,
//              "dump/sparse_matching_augmented_" + std::to_string(it - viewpoint_path->order.begin()) + "_" +
//              std::to_string(new_viewpoint_path.entries.size()) + ".png",
//              draw_lines);
//        }
//      }

      const ViewpointEntryIndex idx = viewpoint_path->entries[*it].viewpoint_index;
      const ViewpointEntryIndex next_idx = viewpoint_path->entries[*next_it].viewpoint_index;
      const ViewpointMotion motion = getViewpointMotion(idx, next_idx);
      for (auto it2 = motion.begin() + 1; it2 != motion.end(); ++it2) {
        const ViewpointEntryIndex new_idx = *it2;
        const ViewpointEntryIndex prev_idx = new_viewpoint_path.entries[new_viewpoint_path.order.back()].viewpoint_index;
        const ViewpointEntry& new_viewpoint_entry = viewpoint_entries_[new_idx];
        if (*it2 != next_idx) {
          new_viewpoint_path.order.push_back(new_viewpoint_path.entries.size());
          ViewpointPathEntry new_path_entry;
          new_path_entry.mvs_viewpoint = false;
          new_path_entry.viewpoint_index = new_idx;
          new_path_entry.viewpoint = new_viewpoint_entry.viewpoint;
          new_viewpoint_path.entries.push_back(new_path_entry);
        }
        const SE3Motion& se3_motion = motion.se3Motions()[it2 - 1 - motion.begin()];
        ViewpointMotion sub_motion({ prev_idx, new_idx }, { se3_motion });
        addViewpointMotion(std::move(sub_motion));
        // TODO
//        if (options_.dump_stereo_matching_images) {
//          bool draw_lines = false;
//          const Viewpoint& viewpoint1 = new_viewpoint_path.entries[*(new_viewpoint_path.order.rbegin() + 1)].viewpoint;
//          const Viewpoint& viewpoint2 = new_viewpoint_path.entries[*(new_viewpoint_path.order.rbegin())].viewpoint;
//          dumpSparseMatching(
//              viewpoint1, viewpoint2,
//              "dump/sparse_points_augmented_" + std::to_string(it - viewpoint_path->order.begin()) + "_" +
//              std::to_string(new_viewpoint_path.entries.size()) + ".png",
//              draw_lines);
//          draw_lines = true;
//          dumpSparseMatching(
//              viewpoint1, viewpoint2,
//              "dump/sparse_matching_augmented_" + std::to_string(it - viewpoint_path->order.begin()) + "_" +
//              std::to_string(new_viewpoint_path.entries.size()) + ".png",
//              draw_lines);
//        }
      }
    }
  }
  *viewpoint_path = new_viewpoint_path;
//  std::vector<FloatType> sparse_matching_scores;
//  for (auto it = viewpoint_path->order.begin(); it != viewpoint_path->order.end(); ++it) {
//    auto next_it = it + 1;
//    if (next_it == viewpoint_path->order.end()) {
//      next_it = viewpoint_path->order.begin();
//    }
//    const std::size_t idx = *it;
//    const std::size_t next_idx = *next_it;
//    const Viewpoint& viewpoint = viewpoint_path->entries[idx].viewpoint;
//    const Viewpoint& next_viewpoint = viewpoint_path->entries[next_idx].viewpoint;
//    const FloatType sparse_matching_score = computeSparseMatchingScore(viewpoint, next_viewpoint);
//    sparse_matching_scores.push_back(sparse_matching_score);
//  }
//  std::cout << "New viewpoint path:" << std::endl;
//  for (auto it = viewpoint_path->order.begin(); it != viewpoint_path->order.end(); ++it) {
//    const std::size_t idx = *it;
//    const FloatType sparse_matching_score = sparse_matching_scores[it - viewpoint_path->order.begin()];
//    std::cout << "  i=" << idx << " entry=" << viewpoint_path->entries[idx].viewpoint_index
//        << " sparse matching score " << sparse_matching_score << std::endl;
//  }
  std::cout << "New viewpoint path has " << viewpoint_path->entries.size() << " viewpoints" << std::endl;

//  const auto get_motion_pose_lambda = [](const ViewpointMotion& motion, const FloatType fraction_done) -> Pose {
////    const Pose& first_pose = motion.poses.front();
////    const Pose& last_pose = motion.poses.back();
//    BH_ASSERT(fraction_done <= 1);
//    FloatType accumulated_fraction = 0;
//    for (auto next_it = motion.poses.begin() + 1; next_it != motion.poses.end(); ++next_it) {
//      auto prev_it = next_it - 1;
//      FloatType segment_distance = (next_it->translation() - prev_it->translation()).norm();
//      FloatType segment_fraction = segment_distance / motion.distance;
//      if (accumulated_fraction + segment_fraction >= fraction_done) {
//        const FloatType segment_factor = (fraction_done - accumulated_fraction) / segment_fraction;
//        BH_ASSERT(segment_factor <= 1);
//        const Vector3 translation = prev_it->translation() + segment_factor * (next_it->translation() - prev_it->translation());
//        const Quaternion quaternion = prev_it->quaternion().slerp(segment_factor, next_it->quaternion());
////        const Quaternion quaternion = first_pose.quaternion().slerp(fraction_done, last_pose.quaternion());
//        const Pose pose = Pose::createFromImageToWorldTransformation(translation, quaternion);
//        return pose;
//      }
//      accumulated_fraction += segment_fraction;
//    }
//    return motion.poses.back();
//  };
//
//  const auto create_motion_lambda = [](const ViewpointMotion& motion, const FloatType fraction_start, const FloatType fraction_end,
//      const Pose& from, const Pose& to) -> Motion {
//    Motion new_motion;
//    new_motion.poses.push_back(from);
//    FloatType accumulated_fraction = 0;
//    for (auto next_it = motion.poses.begin() + 1; next_it != motion.poses.end(); ++next_it) {
//      auto prev_it = next_it - 1;
//      const FloatType segment_distance = (next_it->translation() - prev_it->translation()).norm();
//      const FloatType segment_fraction = segment_distance / motion.distance;
//      if (accumulated_fraction + segment_fraction > fraction_start && accumulated_fraction + segment_fraction < fraction_end) {
//        new_motion.poses.push_back(*next_it);
//      }
//      accumulated_fraction += segment_fraction;
//    }
//    new_motion.poses.push_back(to);
//    new_motion.distance = 0;
//    for (auto next_it = motion.poses.begin() + 1; next_it != motion.poses.end(); ++next_it) {
//      auto prev_it = next_it - 1;
//      const FloatType segment_distance = (next_it->translation() - prev_it->translation()).norm();
//      new_motion.distance += segment_distance;
//    }
//    new_motion.cost = new_motion.distance;
//    return new_motion;
//  };
//
//  ViewpointPath new_viewpoint_path(*viewpoint_path);
//  if (viewpoint_path->order.size() > 1) {
//    new_viewpoint_path.order.clear();
//    for (auto it = viewpoint_path->order.begin(); it != viewpoint_path->order.end(); ++it) {
//      auto next_it = it + 1;
//      if (next_it == viewpoint_path->order.end()) {
//        next_it = viewpoint_path->order.begin();
//      }
//      new_viewpoint_path.order.push_back(*it);
//      const ViewpointEntryIndex idx = viewpoint_path->entries[*it].viewpoint_index;
//      const ViewpointEntryIndex next_idx = viewpoint_path->entries[*next_it].viewpoint_index;
//      const Motion motion = getViewpointMotion(idx, next_idx);
//      ViewpointEntry viewpoint_entry_tmp = viewpoint_entries_[idx];
//      const ViewpointEntry viewpoint_entry_end = viewpoint_entries_[next_idx];
////      std::cout << "idx=" << idx
////          << "viewpoint_entry_tmp.voxel_set.size()=" << viewpoint_entry_tmp.voxel_set.size()
////          << ", next_idx=" << next_idx
////          << "viewpoint_entry_end.voxel_set.size()=" << viewpoint_entry_end.voxel_set.size() << std::endl;
//      // Add new viewpoints between it and next_it until the chain is matchable
//      FloatType motion_done = 0;
//      while (!isSparseMatchable(viewpoint_entry_tmp.viewpoint, viewpoint_entry_end.viewpoint)) {
//        const FloatType matching_score = computeSparseMatchingScore(viewpoint_entry_tmp.viewpoint, viewpoint_entry_end.viewpoint);
//        BH_PRINT_VALUE(matching_score);
//        bool tentative_viewpoint_matchable = false;
//        FloatType tentative_motion_done = 1;
//        ViewpointEntry tentative_viewpoint_entry;
//        while (!tentative_viewpoint_matchable) {
//          const FloatType motion_increment = (tentative_motion_done - motion_done) / 2;
//          tentative_motion_done = motion_done + motion_increment;
//          Pose tentative_pose = get_motion_pose_lambda(motion, tentative_motion_done);
//          tentative_viewpoint_entry.viewpoint = getVirtualViewpoint(tentative_pose);
//          const FloatType increment_distance = (viewpoint_entry_tmp.viewpoint.pose().translation() - tentative_pose.translation()).norm();
//          const FloatType increment_angular_distance = viewpoint_entry_tmp.viewpoint.pose().quaternion().angularDistance(tentative_pose.quaternion());
//          std::cout << "motion_done=" << motion_done << ", motion_increment=" << motion_increment
//              << ", increment_distance=" << increment_distance << ", increment_angular_distance=" << increment_angular_distance * 180 / M_PI << std::endl;
//          tentative_viewpoint_matchable = isSparseMatchable(viewpoint_entry_tmp.viewpoint, tentative_viewpoint_entry.viewpoint);
//          const FloatType matching_score = computeSparseMatchingScore(viewpoint_entry_tmp.viewpoint, tentative_viewpoint_entry.viewpoint);
//          BH_PRINT_VALUE(matching_score);
//        }
//        ViewpointEntryIndex prev_idx = new_viewpoint_path.entries[new_viewpoint_path.order.back()].viewpoint_index;
//        ViewpointEntryIndex new_idx = viewpoint_entries_.size();
//        ViewpointPathEntry new_path_entry;
//        new_path_entry.mvs_viewpoint = false;
//        new_path_entry.viewpoint_index = new_idx;
//        new_path_entry.viewpoint = tentative_viewpoint_entry.viewpoint;
//        new_viewpoint_path.order.push_back(new_viewpoint_path.entries.size());
//        new_viewpoint_path.entries.push_back(new_path_entry);
//        auto raycast_result = getRaycastHitVoxelsWithInformationScore(tentative_viewpoint_entry.viewpoint);
//        tentative_viewpoint_entry.voxel_set = std::move(raycast_result.first);
//        addViewpointEntry(std::move(ViewpointEntry(tentative_viewpoint_entry)));
//        Motion new_motion = create_motion_lambda(
//            motion, motion_done, tentative_motion_done, viewpoint_entry_tmp.viewpoint.pose(), tentative_viewpoint_entry.viewpoint.pose());
////        std::cout << "Adding motion with " << new_motion.poses.size() << " poses" << std::endl;
//        addViewpointMotion(prev_idx, new_idx, std::move(new_motion));
//
//        // TODO
//        bool draw_lines = false;
//        dumpSparseMatchingDebug(
//            viewpoint_entry_tmp.viewpoint, tentative_viewpoint_entry.viewpoint,
//            "dump/sparse_points_augmented_" + std::to_string(it - viewpoint_path->order.begin()) + "_" +
//            std::to_string(motion_done) + ".png",
//            draw_lines);
//        draw_lines = true;
//        dumpSparseMatchingDebug(
//            viewpoint_entry_tmp.viewpoint, tentative_viewpoint_entry.viewpoint,
//            "dump/sparse_matching_augmented_" + std::to_string(it - viewpoint_path->order.begin()) + "_" +
//            std::to_string(motion_done) + ".png",
//            draw_lines);
//
//        viewpoint_entry_tmp = tentative_viewpoint_entry;
//        motion_done = tentative_motion_done;
////        std::cout << "Added viewpoint for sparse matching" << std::endl;
//      }
//      Motion new_motion = create_motion_lambda(
//          motion, motion_done, 1, viewpoint_entry_tmp.viewpoint.pose(), viewpoint_entry_end.viewpoint.pose());
//      addViewpointMotion(new_viewpoint_path.entries[new_viewpoint_path.order.back()].viewpoint_index, next_idx, std::move(new_motion));
//
//      // TODO
//      bool draw_lines = false;
//      dumpSparseMatchingDebug(
//          viewpoint_entry_tmp.viewpoint, viewpoint_entry_end.viewpoint,
//          "dump/sparse_points_augmented_" + std::to_string(it - viewpoint_path->order.begin()) + "_" +
//          std::to_string(1) + ".png",
//          draw_lines);
//      draw_lines = true;
//      dumpSparseMatchingDebug(
//          viewpoint_entry_tmp.viewpoint, viewpoint_entry_end.viewpoint,
//          "dump/sparse_matching_augmented_" + std::to_string(it - viewpoint_path->order.begin()) + "_" +
//          std::to_string(1) + ".png",
//          draw_lines);
//    }
//  }
//  *viewpoint_path = new_viewpoint_path;
//  std::vector<FloatType> sparse_matching_scores;
//  for (auto it = viewpoint_path->order.begin(); it != viewpoint_path->order.end(); ++it) {
//    auto next_it = it + 1;
//    if (next_it == viewpoint_path->order.end()) {
//      next_it = viewpoint_path->order.begin();
//    }
//    const std::size_t idx = *it;
//    const std::size_t next_idx = *next_it;
//    const Viewpoint& viewpoint = viewpoint_path->entries[idx].viewpoint;
//    const Viewpoint& next_viewpoint = viewpoint_path->entries[next_idx].viewpoint;
//    const FloatType sparse_matching_score = computeSparseMatchingScore(viewpoint, next_viewpoint);
//    sparse_matching_scores.push_back(sparse_matching_score);
//  }
//  std::cout << "New viewpoint path:" << std::endl;
//  for (auto it = viewpoint_path->order.begin(); it != viewpoint_path->order.end(); ++it) {
//    const std::size_t idx = *it;
//    const FloatType sparse_matching_score = sparse_matching_scores[it - viewpoint_path->order.begin()];
//    std::cout << "  i=" << idx << " entry=" << viewpoint_path->entries[idx].viewpoint_index
//        << " sparse matching score " << sparse_matching_score << std::endl;
//  }
//  std::cout << "New viewpoint path has " << viewpoint_path->entries.size() << " viewpoints" << std::endl;
//  BH_ASSERT(viewpoint_path->order.size() == viewpoint_path->entries.size());
}

void ViewpointPlanner::makeViewpointMotionsSparseMatchable(ViewpointPath* viewpoint_path) {
  std::unique_lock<std::mutex> lock = acquireLock();
  if (viewpoint_path->order.size() > 1) {
    for (auto it = viewpoint_path->order.begin(); it != viewpoint_path->order.end(); ++it) {
      auto next_it = it + 1;
      if (next_it == viewpoint_path->order.end()) {
        next_it = viewpoint_path->order.begin();
      }
      const ViewpointEntryIndex idx = viewpoint_path->entries[*it].viewpoint_index;
      const ViewpointEntryIndex next_idx = viewpoint_path->entries[*next_it].viewpoint_index;
      const ViewpointMotion motion = getViewpointMotion(idx, next_idx);
      std::cout << "Ensuring sparse matchability from viewpoint " << idx << " to " << next_idx << std::endl;
      ViewpointMotion new_motion = makeViewpointMotionSparseMatchable(motion);
      addViewpointMotion(std::move(new_motion));
    }
  }
  reportViewpointPathsStats();
}

ViewpointPlanner::ViewpointMotion ViewpointPlanner::makeViewpointMotionSparseMatchable(
        const ViewpointMotion& motion) {
  // Extend viewpoint motion with sparse matching viewpoints to fulfilll limits
//  const auto interpolate_viewpoint_se3_motion_lambda = [](
//          const Viewpoint &viewpoint1,
//          const SE3Motion& se3_motion) -> Viewpoint {
//    const Pose& pose1 = viewpoint1.pose();
//    const FloatType interpolation_factor = FloatType(0.8);
//    auto it = se3_motion.poses.begin();
//    while (isWithinSparseMatchingLimits(pose1, *it)) {
//      ++it;
//    }
//    const Viewpoint interpolated_viewpoint = Viewpoint(&interpolated_viewpoint.camera(), interpolated_pose);
//    return interpolated_viewpoint;
//  };
//
//  const auto get_sub_se3_motion_lambda = [](const SE3Motion& se3_motion, const Viewpoint& end_viewpoint) {
//    SE3Motion sub_motion;
//    for (auto it = se3_motion.poses.begin(); it != se3_motion.poses.end(); ++it) {
//      sub_motion.poses.push_back(*it);
//      sub_motion.
//    }
//    return sub_motion;
//  };

  const auto get_interpolated_pose_lambda = [&](const Pose& pose1, const Pose& pose2, const FloatType factor) -> Pose {
    BH_ASSERT(factor >= 0);
    BH_ASSERT(factor <= 1);
    const Vector3 translation = pose1.translation() + factor * (pose2.translation() - pose1.translation());
    const Quaternion quaternion = pose1.quaternion().slerp(factor, pose2.quaternion());
    const Pose interpolated_pose = Pose::createFromImageToWorldTransformation(translation, quaternion);
    return interpolated_pose;
  };

  const auto compute_segment_fraction_lambda = [&](const SE3Motion& se3_motion,
                                                   const SE3Motion::PoseVector::const_iterator prev_it,
                                                   const SE3Motion::PoseVector::const_iterator next_it) {
    FloatType segment_fraction;
    if (se3_motion.se3_distance > 0) {
      const FloatType segment_distance = next_it->getDistanceTo(*prev_it);
      BH_ASSERT(segment_distance > 0);
      segment_fraction = segment_distance / se3_motion.se3_distance;
    }
    else {
      segment_fraction = 1 / FloatType(se3_motion.poses.size() - 1);
    }
    return segment_fraction;
  };

  const auto get_max_matchable_fraction_lambda = [&](const SE3Motion& se3_motion) -> FloatType {
    Viewpoint viewpoint1(&getVirtualCamera(), se3_motion.poses.front());
    Viewpoint viewpoint2_tmp(&getVirtualCamera(), se3_motion.poses.back());
    // Early break
    if (isSparseMatchable2(viewpoint1, viewpoint2_tmp, options_.sparse_matching_voxels_conservative_iou_threshold)) {
      return 1;
    }
    FloatType accumulated_fraction = 0;
    for (auto next_it = se3_motion.poses.begin() + 1; next_it != se3_motion.poses.end(); ++next_it) {
      auto prev_it = next_it - 1;
      const FloatType segment_fraction = compute_segment_fraction_lambda(se3_motion, prev_it, next_it);
      Viewpoint viewpoint2(&getVirtualCamera(), *next_it);
      if (!isSparseMatchable2(viewpoint1, viewpoint2, options_.sparse_matching_voxels_conservative_iou_threshold)) {
        // Perform binary search on SE3 motion segment to find matchable viewpoint
        const Pose &pose1 = *prev_it;
        const Pose &pose2 = *next_it;
        FloatType lower_fraction = FloatType(0);
        FloatType upper_fraction = FloatType(1);
        while (upper_fraction - lower_fraction > FloatType(0.01)) {
          const FloatType fraction = (lower_fraction + upper_fraction) / 2;
          const Pose interpolated_pose = get_interpolated_pose_lambda(pose1, pose2, fraction);
          const Viewpoint interpolated_viewpoint(&getVirtualCamera(), interpolated_pose);
          if (isSparseMatchable2(viewpoint1, interpolated_viewpoint,
                                 options_.sparse_matching_voxels_conservative_iou_threshold)) {
            lower_fraction = fraction;
          }
          else {
            upper_fraction = fraction;
          }
        }
        if (lower_fraction == 0) {
          std::cout << "WARNING: Could not find interpolated viewpoint between "
                       << pose1 << " and " << pose2 << "  that is sparse matchable" << std::endl;
        }
        return accumulated_fraction + segment_fraction * lower_fraction;
      }
      accumulated_fraction += segment_fraction;
    }
    return 1;
  };

  const auto split_se3_motion_lambda = [&](const SE3Motion& se3_motion, const FloatType fraction)
          -> std::pair<SE3Motion, SE3Motion> {
    BH_ASSERT(fraction <= 1);
    SE3Motion se3_motion1;
    SE3Motion se3_motion2;
    se3_motion1.poses.push_back(se3_motion.poses.front());
    FloatType accumulated_fraction = 0;
    bool split_happened = false;
    for (auto next_it = se3_motion.poses.begin() + 1; next_it != se3_motion.poses.end(); ++next_it) {
      auto prev_it = next_it - 1;
      const FloatType segment_fraction = compute_segment_fraction_lambda(se3_motion, prev_it, next_it);
      BH_ASSERT(segment_fraction >= -0.01);
      BH_ASSERT(segment_fraction <= 1.01);
      const FloatType segment_fraction_clamped = bh::clamp(segment_fraction);
      const FloatType new_accumulated_fraction = accumulated_fraction + segment_fraction_clamped;
      if (!split_happened && new_accumulated_fraction >= fraction) {
        const FloatType segment_factor = (fraction - accumulated_fraction) / segment_fraction_clamped;
        BH_ASSERT(segment_factor >= 0);
        BH_ASSERT(segment_factor <= 1);
        const Vector3 translation = prev_it->translation() + segment_factor * (next_it->translation() - prev_it->translation());
        const Quaternion quaternion = prev_it->quaternion().slerp(segment_factor, next_it->quaternion());
//        const Quaternion quaternion = first_pose.quaternion().slerp(fraction_done, last_pose.quaternion());
        const Pose pose = Pose::createFromImageToWorldTransformation(translation, quaternion);
        se3_motion1.poses.push_back(pose);
        se3_motion2.poses.push_back(pose);
        split_happened = true;
        BH_ASSERT(se3_motion1.poses.back() == se3_motion2.poses.front());
      }
      if (!split_happened) {
        se3_motion1.poses.push_back(*next_it);
      }
      else {
        se3_motion2.poses.push_back(*next_it);
      }
      accumulated_fraction = new_accumulated_fraction;
    }
    se3_motion1.distance = fraction * se3_motion.distance;
    se3_motion1.cost = fraction * se3_motion.cost;
    se3_motion1.se3_distance = fraction * se3_motion.se3_distance;
    se3_motion2.distance = se3_motion.distance - se3_motion1.distance;
    se3_motion2.cost = se3_motion.cost - se3_motion1.cost;
    se3_motion2.se3_distance = se3_motion.se3_distance * se3_motion1.se3_distance;
    BH_ASSERT(se3_motion1.poses.back() == se3_motion2.poses.front());
    BH_ASSERT(se3_motion.poses.front() == se3_motion1.poses.front());
    BH_ASSERT(se3_motion.poses.back() == se3_motion2.poses.back());
    return std::make_pair(se3_motion1, se3_motion2);
  };

  std::vector<ViewpointEntryIndex> new_motion_viewpoint_indices;
  new_motion_viewpoint_indices.push_back(motion.viewpointIndices()[0]);
  ViewpointMotion::SE3MotionVector new_motion_se3_motions;
  for (size_t i = 1; i < motion.viewpointIndices().size(); ++i) {
    const ViewpointEntryIndex next_viewpoint_index = motion.viewpointIndices()[i];
    SE3Motion se3_motion = motion.se3Motions()[i - 1];
    BH_ASSERT(se3_motion.poses.front() == viewpoint_entries_[motion.viewpointIndices()[i - 1]].viewpoint.pose());
    BH_ASSERT(se3_motion.poses.back() == viewpoint_entries_[motion.viewpointIndices()[i]].viewpoint.pose());

    while (true) {
      const FloatType max_matchable_fraction = get_max_matchable_fraction_lambda(se3_motion);
      BH_ASSERT(max_matchable_fraction >= 0);
      BH_ASSERT(max_matchable_fraction <= 1);
      if (max_matchable_fraction < 1) {
        if (max_matchable_fraction == 0) {
          std::cout << "WARNING: Could not find sparse matchable motion from viewpoint "
                    << motion.fromIndex() << " to " << motion.toIndex()
                    << " (i=" << i << ", in-between viewpoint=" << motion.viewpointIndices()[i] << ")" << std::endl;
          return motion;
        }
        SE3Motion se3_motion1;
        SE3Motion se3_motion2;
        std::tie(se3_motion1, se3_motion2) = split_se3_motion_lambda(se3_motion, max_matchable_fraction);

        // Sanity checks
        const Viewpoint& prev_viewpoint = viewpoint_entries_[new_motion_viewpoint_indices.back()].viewpoint;
        const Viewpoint& next_viewpoint = viewpoint_entries_[next_viewpoint_index].viewpoint;
        BH_ASSERT(se3_motion1.poses.front() == prev_viewpoint.pose());
        BH_ASSERT(se3_motion2.poses.back() == next_viewpoint.pose());
        BH_ASSERT(se3_motion1.poses.back() == se3_motion2.poses.front());

        ViewpointEntry split_motion_viewpoint_entry;
        split_motion_viewpoint_entry.viewpoint = Viewpoint(&prev_viewpoint.camera(), se3_motion1.poses.back());
        const bool ignore_viewpoint_count_grid = true;
        const ViewpointEntryIndex split_motion_viewpoint_index = addViewpointEntryWithoutLock(
                split_motion_viewpoint_entry, ignore_viewpoint_count_grid);
        new_motion_viewpoint_indices.push_back(split_motion_viewpoint_index);
        new_motion_se3_motions.push_back(se3_motion1);
        se3_motion = se3_motion2;
      }
      else {
        new_motion_viewpoint_indices.push_back(next_viewpoint_index);
        new_motion_se3_motions.push_back(se3_motion);
        break;
      }
    }
  }
  const ViewpointMotion new_motion(new_motion_viewpoint_indices, new_motion_se3_motions);
  return new_motion;
}
